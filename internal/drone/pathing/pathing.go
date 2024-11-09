package pathing

import (
	"PlantPulse/internal/common"
	"PlantPulse/internal/drone"
	"math"
)

type FlightPath struct {
	Bearing float32
	Start   *FlightPathNode
}

type FlightPathOptions struct {
	CameraFOV float64
	Altitude  float64
	delta     float32
}

type FlightPathNode struct {
	waypoint *drone.WayPoint
	left     *drone.WayPoint
	right    *drone.WayPoint
	up       *drone.WayPoint
	down     *drone.WayPoint
}

type Edge struct {
	point1 *drone.WayPoint
	point2 *drone.WayPoint
}

func NewFlightPath(bounds []*drone.WayPoint, opts FlightPathOptions) *FlightPathNode {
	edges := []Edge{}
	// 1 2 3 4 5
	for i := 0; i < len(bounds); i++ {
		if i == len(bounds)-1 {
			edges = append(edges, Edge{point1: bounds[i], point2: bounds[0]})
		} else {
			edges = append(edges, Edge{point1: bounds[i], point2: bounds[i+1]})
		}
	}

	start, delta := findStartingPoint(bounds[0], edges, opts.CameraFOV, opts.Altitude)

	createFlightPlan(start, delta)

	return start
}

func findStartingPoint(startingBound *drone.WayPoint, edges []Edge, fov float64, alt float64) (start *FlightPathNode, delta float64) {
	halfFovRadians := common.DegreesToRadians(.5 * fov)
	deltaLat = alt * math.Tan(halfFovRadians) * 2

	// hits edges

	return
}

func createFlightPlan(start *FlightPathNode, delta float64) {
	return
}
