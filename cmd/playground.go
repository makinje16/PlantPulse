package main

import (
	"context"
	"log"
	"math"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
	"github.com/bluenviron/gomavlib/v3/pkg/message"
)

var (
	currentYaw   float32 = 0.0
	currentRoll  float32 = 0.0
	currentPitch float32 = 0.0
)

func main() {
	// create a node which communicates with a UDP endpoint in client mode
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointUDPServer{Address: "127.0.0.1:14550"},
		},
		Dialect:     common.Dialect,
		OutVersion:  gomavlib.V2, // change to V1 if you're unable to communicate with the target
		OutSystemID: 10,
	})
	if err != nil {
		panic(err)
	}
	defer node.Close()

	// Switch to GUIDED mode using MessageCommandLong
	cmdGuided := &common.MessageCommandLong{
		TargetSystem:    1,
		TargetComponent: 1,
		Confirmation:    0,
		Command:         common.MAV_CMD_DO_SET_MODE,
		Param1:          float32(common.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), // Base mode with custom mode enabled
		Param2:          4,                                                 // Custom mode for GUIDED in ArduPilot
	}

	SendCommand(node, cmdGuided)

	// Arm Drone
	cmdArm := &common.MessageCommandLong{
		TargetSystem:    1, // target system ID (usually 1 for a single drone setup)
		TargetComponent: 1, // target component ID
		Command:         common.MAV_CMD_COMPONENT_ARM_DISARM,
		Confirmation:    0,
		Param1:          1, // 1 to arm, 0 to disarm
	}

	SendCommand(node, cmdArm)

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

	SendCommand(node, cmdTakeoff)

	time.Sleep(time.Second * 10)

	MoveDrone(node, 0, -100, -100)

	// cmdLand := &common.MessageCommandLong{
	// 	TargetSystem:    1,
	// 	TargetComponent: 1,
	// 	Command:         common.MAV_CMD_NAV_LAND,
	// 	Confirmation:    0,
	// 	Param2:          float32(common.PRECISION_LAND_MODE_DISABLED),
	// }

	// SendCommand(node, cmdLand)

	for ch := range node.Events() {
		switch ch := ch.(type) {
		case *gomavlib.EventFrame:
			handleFrame(ch)
		default:
			continue
		}
	}
}

func SendCommand(node *gomavlib.Node, cmd message.Message) error {
	checkAck := func(cmd common.MAV_CMD, timeout time.Duration) error {
		exceeded := time.After(timeout)
		for {
			select {
			case evt := <-node.Events():
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
		if err := node.WriteMessageAll(cmd); err != nil {
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

func MoveDrone(node *gomavlib.Node, forward, right, down float32) {
	// Convert body frame movement to NED frame
	msg := &common.MessageSetPositionTargetLocalNed{
		TargetSystem:    1,
		TargetComponent: 0,
		CoordinateFrame: common.MAV_FRAME_LOCAL_NED,
		X:               forward*float32(math.Cos(float64(currentYaw))) - right*float32(math.Sin(float64(currentYaw))),
		Y:               forward*float32(math.Sin(float64(currentYaw))) + right*float32(math.Cos(float64(currentYaw))),
		Z:               down,
	}

	SendCommand(node, msg)
}

func handleFrame(evt *gomavlib.EventFrame) {
	switch msg := evt.Frame.GetMessage().(type) {
	case *common.MessageAttitude:
		log.Printf("Attitude: Pitch: %v Roll: %v Yaw: %v\n", msg.Pitch, msg.Roll, msg.Yaw)
		currentPitch = msg.Pitch
		currentRoll = msg.Roll
		currentYaw = msg.Yaw
	default:
		break
	}
}
