package main

import (
	idrone "PlantPulse/internal/drone"
	"context"
	"time"

	"github.com/bluenviron/gomavlib/v3"
	"github.com/bluenviron/gomavlib/v3/pkg/dialects/ardupilotmega"
)

func main() {
	// create a node which communicates with a UDP endpoint in client mode
	node, err := gomavlib.NewNode(gomavlib.NodeConf{
		Endpoints: []gomavlib.EndpointConf{
			gomavlib.EndpointUDPServer{Address: "127.0.0.12:14550"},
		},
		Dialect:     ardupilotmega.Dialect,
		OutVersion:  gomavlib.V2, // change to V1 if you're unable to communicate with the target
		OutSystemID: 10,
	})
	if err != nil {
		panic(err)
	}

	defer node.Close()

	ctx, cancelFunc := context.WithCancel(context.Background())
	defer cancelFunc()

	// connect and takeoff
	drone := idrone.NewDrone(node, ctx)
	// drone.ArmAndTakeOffFromHome(ctx, 10)
	drone.ChangeGimbalPositionServo(-90, 0)
	// drone.ChangeGimbalPosition(-90, 0)
	time.Sleep(30 * time.Second)
	// Move
	// drone.StartMission(idrone.NewMission([]*idrone.WayPoint{
	// 	idrone.NewWayPoint(-35.36186414, 149.16501711, 100),
	// 	idrone.NewWayPoint(-35.36166203, 149.16766073, 100),
	// 	idrone.NewWayPoint(-35.36294209, 149.16755058, 100),
	// }))

	// Return Home
	drone.ReturnHome()
}
