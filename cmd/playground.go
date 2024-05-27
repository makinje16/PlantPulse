package main

import (
	internal "PlantPulse/internal/drone"
	"context"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
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

	ctx, cancelFunc := context.WithCancel(context.Background())
	defer cancelFunc()

	// Create and Change Mode
	drone := internal.NewDrone(node, ctx)
	drone.ChangeMode(internal.FLIGHT_MODE_GUIDED)

	// Arm and TakeOff
	drone.ArmAndTakeOffFromHome(ctx, 100)

	// Land
	drone.Land(ctx)
}
