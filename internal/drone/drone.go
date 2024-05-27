package internal

import (
	"context"
	"errors"
	"log"
	"math"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
)

type Drone struct {
	node            *gomavlib.Node
	currentPitch    float32
	currentYaw      float32
	currentRoll     float32
	homePoint       WayPoint
	currentPosition WayPoint
}

func NewDrone(node *gomavlib.Node, ctx context.Context) *Drone {
	drone := &Drone{node: node}
	go drone.monitorEventLog(ctx)

	return drone
}

func (d *Drone) sendCommand(cmd message.Message) error {
	checkAck := func(cmd common.MAV_CMD, timeout time.Duration) error {
		exceeded := time.After(timeout)
		for {
			select {
			case evt := <-d.node.Events():
				if frm, ok := evt.(*gomavlib.EventFrame); ok {
					if ack, ok := frm.Message().(*common.MessageCommandAck); ok {
						if ack.Command == cmd {
							log.Printf("Received ACK for command %d: result=%d\n", ack.Command, ack.Result)
							if ack.Result == common.MAV_RESULT_ACCEPTED {
								return nil
							}
						}
					}
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
	msg := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Command:         common.MAV_CMD_NAV_TAKEOFF,
		Param5:          d.homePoint.Latitude(),
		Param6:          d.homePoint.Longitude(),
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

func (d *Drone) waitForHeight(ctx context.Context, desiredAlt float32) error {
	altitudeSubscription, unsubscribe := d.currentPosition.subscribeAlt()
	defer unsubscribe()

	for {
		select {
		case <-ctx.Done():
			return errors.New("Exceeded wait time to reach altitude")
		case alt := <-altitudeSubscription:
			if math.Abs(float64(alt-desiredAlt)) < .3 {
				return nil
			}
		default:
			continue
		}
	}

}

func (d *Drone) StartMission(mission *Mission) error {
	return errors.ErrUnsupported
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
		Z:               down,
	}

	return d.sendCommand(msg)
}

func (d *Drone) handleFrame(evt *gomavlib.EventFrame) {
	switch msg := evt.Frame.GetMessage().(type) {
	case *common.MessageAttitude:
		log.Printf("Attitude: Pitch: %v Roll: %v Yaw: %v\n", msg.Pitch, msg.Roll, msg.Yaw)
		d.currentPitch = msg.Pitch
		d.currentRoll = msg.Roll
		d.currentYaw = msg.Yaw
	case *common.MessagePositionTargetGlobalInt:
		d.currentPosition.updateWayPoint(
			float32(msg.LatInt)/SCALE_FACTOR,
			float32(msg.LonInt)/SCALE_FACTOR,
			msg.Alt,
		)
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
