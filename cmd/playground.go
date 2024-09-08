package main

import (
	idrone "PlantPulse/internal/drone"
	"context"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/common"
)

func main() {
	// create a node which communicates with a UDP endpoint in client mode
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointUDPServer{Address: "127.0.0.12:14550"},
		},
		Dialect:     common.Dialect,
		OutVersion:  gomavlib.V2, // change to V1 if you're unable to communicate with the target
		OutSystemID: 10,
	})
	if err != nil {
		panic(err)
	}

	// for {
	// 	evt := <-node.Events()
	// 	if _, ok := evt.(*gomavlib.EventFrame); ok {
	// 		log.Println("Got event frame")
	// 	}
	// }
	defer node.Close()

	ctx, cancelFunc := context.WithCancel(context.Background())
	defer cancelFunc()

	// Create and Change Mode
	drone := idrone.NewDrone(node, ctx)
	for {
		_ = drone
	}
	// drone.ChangeMode(idrone.FLIGHT_MODE_GUIDED)

	// // Arm and TakeOff
	// drone.ArmAndTakeOffFromHome(ctx, 100)
	// drone.Land(ctx)
	// // Move
	// drone.StartMission(idrone.NewMission([]*idrone.WayPoint{
	// 	idrone.NewWayPoint(-35.36186414, 149.16501711, 100),
	// 	idrone.NewWayPoint(-35.36166203, 149.16766073, 100),
	// 	idrone.NewWayPoint(-35.36294209, 149.16755058, 100),
	// }))

	// // Return Home
	// drone.ReturnHome()
}
