package internal

import (
	"context"
	"errors"
	"log"
	"math"
	"sync"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
)

type Drone struct {
	node               *gomavlib.Node
	currentPitch       float32
	currentYaw         float32
	currentRoll        float32
	homePoint          *WayPoint
	currentPosition    *WayPoint
	ackChan            chan *common.MessageCommandAck
	currentPositionNED *common.MessageLocalPositionNed
	nedPositionLock    sync.Mutex
}

func NewDrone(node *gomavlib.Node, ctx context.Context) *Drone {
	drone := &Drone{
		node:    node,
		ackChan: make(chan *common.MessageCommandAck),
	}

	go drone.monitorEventLog(ctx)

	return drone
}

func (d *Drone) sendCommand(cmd message.Message) error {
	checkAck := func(cmd common.MAV_CMD, timeout time.Duration) error {
		exceeded := time.After(timeout)
		for {
			select {
			case ack := <-d.ackChan:
				if ack.Command == cmd {
					return nil
				}
			case <-exceeded:
				log.Printf("Exceeded timer waiting for ack ACCEPT %s\n", cmd.String())
				return context.DeadlineExceeded
			}
		}
	}

	for i := 0; i < 5; i++ {
		if err := d.node.WriteMessageAll(cmd); err != nil {
			return err
		}

		switch msg := cmd.(type) {
		case *common.MessageCommandLong:
			log.Printf("Command %s sent, attempt %d\n", msg.Command.String(), i+1)

			if err := checkAck(msg.Command, time.Second*5); err == nil {
				return nil
			}
		case message.Message:
			log.Printf("Got non MessageCommandLong\n")
			return nil
		}
	}

	return context.DeadlineExceeded
}

func (d *Drone) ChangeMode(mode FlightMode) error {
	msg := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Command:         common.MAV_CMD_DO_SET_MODE,
		Confirmation:    0,
		Param1:          float32(common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
		Param2:          mode.float32(),
	}

	return d.sendCommand(msg)
}

func (d *Drone) ArmAndTakeOff(lat, long float32) error {
	msg := &common.MessageCommandLong{
		TargetSystem:    1, // target system ID (usually 1 for a single drone setup)
		TargetComponent: 1, // target component ID
		Command:         common.MAV_CMD_COMPONENT_ARM_DISARM,
		Confirmation:    0,
		Param1:          1, // 1 to arm, 0 to disarm
	}

	if err := d.sendCommand(msg); err != nil {
		return err
	}

	// Takeoff
	cmdTakeoff := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_TAKEOFF,
		Confirmation:    0,
		Param5:          -35.3612,
		Param6:          149.1654,
		Param7:          10,
	}

	return d.sendCommand(cmdTakeoff)
}

func (d *Drone) ArmAndTakeOffFromHome(ctx context.Context, altitude float32) error {
	armCmd := &common.MessageCommandLong{
		TargetSystem:    1, // target system ID (usually 1 for a single drone setup)
		TargetComponent: 1, // target component ID
		Command:         common.MAV_CMD_COMPONENT_ARM_DISARM,
		Confirmation:    0,
		Param1:          1, // 1 to arm, 0 to disarm
	}

	if err := d.sendCommand(armCmd); err != nil {
		return err
	}

	msg := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_TAKEOFF,
		Param5:          d.currentPosition.Latitude(),
		Param6:          d.currentPosition.Longitude(),
		Param7:          altitude,
	}

	if err := d.sendCommand(msg); err != nil {
		return err
	}

	return d.waitForHeight(ctx, altitude)
}

func (d *Drone) Land(ctx context.Context) error {
	cmdLand := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_LAND,
		Confirmation:    0,
		Param2:          float32(common.PRECISION_LAND_MODE_DISABLED),
	}

	if err := d.sendCommand(cmdLand); err != nil {
		return err
	}

	return d.waitForHeight(ctx, 0)
}

func (d *Drone) CurrentPosition() *WayPoint {
	return d.currentPosition
}

func (d *Drone) waitForHeight(ctx context.Context, desiredAlt float32) error {
	altitudeSubscription, unsubscribe := d.currentPosition.subscribeAlt()
	defer unsubscribe()

	log.Printf("Current altitude: %v Desired Altitude: %v\n", d.currentPosition.alt, desiredAlt)

	for {
		select {
		case <-ctx.Done():
			return errors.New("Exceeded wait time to reach altitude")
		case alt := <-altitudeSubscription:
			if math.Abs(float64(alt-desiredAlt)) < .3 {
				return nil
			}
			log.Printf("Current altitude: %v Desired Altitude: %v\n", d.currentPosition.alt, desiredAlt)
		default:
			continue
		}
	}
}

func (d *Drone) waitForPosition(ctx context.Context, x, y, z float32) error {
	for {
		currentPos := d.GetNEDPosition()
		select {
		case <-ctx.Done():
			return errors.New("deadline exceeded waiting for position")
		default:
			log.Printf("CurrentPos: %v,%v,%v DesiredPos: %v,%v,%v", currentPos.X, currentPos.Y, currentPos.Z, x, y, z)
			if math.Abs(float64(currentPos.X-x)) < .3 &&
				math.Abs(float64(currentPos.Y-y)) < .3 &&
				math.Abs(float64(currentPos.Z-z)) < .3 {
				return nil
			}
		}
	}
}

func (d *Drone) waitForPositionGPS(ctx context.Context, wp *WayPoint) error {
	for {
		select {
		case <-ctx.Done():
			return errors.New("Deadline Exceeded")
		default:
			if d.currentPosition == wp {
				return nil
			}
		}
	}
}

func (d *Drone) StartMission(mission *Mission) error {
	for _, wp := range mission.waypoints {
		if err := d.MoveToWayPoint(context.Background(), wp); err != nil {
			return err
		}
	}

	return nil
}

func (d *Drone) ReturnHome() error {
	return errors.ErrUnsupported
}

func (d *Drone) Move(forward, right, down float32) error {
	// Convert body frame movement to NED frame
	msg := &common.MessageSetPositionTargetLocalNed{
		TargetSystem:    1,
		TargetComponent: 0,
		CoordinateFrame: common.MAV_FRAME_LOCAL_NED,
		X:               forward*float32(math.Cos(float64(d.currentYaw))) - right*float32(math.Sin(float64(d.currentYaw))),
		Y:               forward*float32(math.Sin(float64(d.currentYaw))) + right*float32(math.Cos(float64(d.currentYaw))),
		Z:               -1 * down,
	}

	if err := d.sendCommand(msg); err != nil {
		log.Printf("Error sending Move message: %v", err.Error())
		return err
	}

	return d.waitForPosition(context.Background(), msg.X, msg.Y, msg.Z)
}

func (d *Drone) MoveToWayPoint(ctx context.Context, wp *WayPoint) error {
	mvCMD := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_WAYPOINT,
		Param5:          wp.lat,
		Param6:          wp.long,
		Param7:          wp.alt,
	}

	if err := d.sendCommand(mvCMD); err != nil {
		return err
	}

	return d.waitForPositionGPS(ctx, wp)
}

func (d *Drone) handleFrame(evt *gomavlib.EventFrame) {
	switch msg := evt.Frame.GetMessage().(type) {
	case *common.MessageAttitude:
		d.currentPitch = msg.Pitch
		d.currentRoll = msg.Roll
		d.currentYaw = msg.Yaw
	case *common.MessageGlobalPositionInt:
		if d.currentPosition == nil {
			d.currentPosition = NewWayPoint(
				float32(msg.Lat)/SCALE_FACTOR,
				float32(msg.Lon)/SCALE_FACTOR,
				float32(msg.RelativeAlt/1000),
			)
		} else {
			d.currentPosition.updateWayPoint(
				float32(msg.Lat)/SCALE_FACTOR,
				float32(msg.Lon)/SCALE_FACTOR,
				float32(msg.RelativeAlt/1000),
			)
		}
	case *common.MessageCommandAck:
		d.ackChan <- msg
	case *common.MessageLocalPositionNed:
		d.nedPositionLock.Lock()
		d.currentPositionNED = msg
		d.nedPositionLock.Unlock()
	default:
		break
	}
}

func (d *Drone) monitorEventLog(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
		case ch := <-d.node.Events():
			switch evt := ch.(type) {
			case *gomavlib.EventFrame:
				d.handleFrame(evt)
			default:
				continue
			}
		}
	}
}

func (d *Drone) GetNEDPosition() *common.MessageLocalPositionNed {
	d.nedPositionLock.Lock()
	defer d.nedPositionLock.Unlock()

	return d.currentPositionNED
}
