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
	CameraWidthFOV  float64
	CameraLengthFOV float64
	Altitude        float64
	delta           float32
}

type FlightPathNode struct {
	waypoint *drone.WayPoint
	left     *drone.WayPoint
	right    *drone.WayPoint
	up       *drone.WayPoint
	down     *drone.WayPoint
}

func NewFlightPath(bounds []*drone.WayPoint, opts FlightPathOptions) *FlightPathNode {
	edges := []Edge{}
	for i := 0; i < len(bounds); i++ {
		if i == len(bounds)-1 {
			edges = append(edges, Edge{point1: bounds[i], point2: bounds[0]})
		} else {
			edges = append(edges, Edge{point1: bounds[i], point2: bounds[i+1]})
		}
	}

	start, delta := findStartingPoint(bounds[0], edges, opts)

	createFlightPlan(start, delta)

	return start
}

func findStartingPoint(startingBound *drone.WayPoint, edges []Edge, opts FlightPathOptions) (start *FlightPathNode, delta float64) {
	halfWidthFOVRadians := common.DegreesToRadians(.5 * opts.CameraWidthFOV)
	halfLengthFOVRadians := common.DegreesToRadians(.5 * opts.CameraLengthFOV)

	deltaLat := alt * math.Tan(halfFovRadians)
	deltaLong := alt * math.Tan(halfLengthFOVRadians)

	// hits edges
	return
}

func validPoint(point *drone.WayPoint, edges []Edge) bool {
	var left, above, right, bottom *Edge

	for _, edge := range edges {
		if BetweenLatEdgePoints(point, edge) {
			left, right = CheckLeftRightOfEdge(point, edge)
		}

		if BetweenLongEdgePoints(point, edge) {
			above, bottom = CheckAboveBelowOfEdge(point, edge)
		}

		if left != nil && above != nil && right != nil && bottom != nil {
			return true
		}
	}

	return false
}

// CheckLeftRightOfEdge takes in a drone.Waypoint and Edge. It
// returns (*Edge, *Edge) where the first paramter is "left"
// and the other "right". In our case, left means  the edge is
// left of the point and right means the edge is to the right
// of the point from a top down perspective.
func CheckLeftRightOfEdge(point *drone.WayPoint, edge Edge) (*Edge, *Edge) {
	pivot, p2 := edge.GetEdgePivot()

	p2Theta := angleUsingPivot(pivot, p2)
	pointTheta := angleUsingPivot(pivot, point)

	if pointTheta < p2Theta {
		return &edge, nil
	}

	return nil, &edge
}

func angleUsingPivot(pivot *drone.WayPoint, other *drone.WayPoint) float64 {

	deltaLat := math.Abs(float64(other.Latitude() - pivot.Latitude()))
	deltaLong := math.Abs(float64(other.Longitude() - pivot.Longitude()))

	theta := math.Atan(deltaLat/deltaLong) * (180 / math.Pi)

	if other.Longitude() < pivot.Longitude() {
		return 180 - theta
	}

	return theta
}

// CheckAboveBelowOfEdge takes in a drone.Waypoint and Edge. It
// returns (*Edge, *Edge) where the first paramter is "above"
// and the other "below". In our case, above means the point would
// intersect eventually when the latitude is increased. Below
// means we would intersect when the latitude is decreased.
func CheckAboveBelowOfEdge(point *drone.WayPoint, edge Edge) (*Edge, *Edge) {
	lat := point.Latitude()

	if lat > edge.point1.Latitude() && lat > edge.point2.Latitude() {
		return nil, &edge
	}

	if lat < edge.point1.Latitude() && lat < edge.point2.Latitude() {
		return &edge, nil
	}

	pivot, p2 := edge.GetEdgePivot()

	p2Theta := angleUsingPivot(pivot, p2)
	pointTheta := angleUsingPivot(pivot, point)

	if p2Theta < pointTheta {
		return &edge, nil
	}

	return nil, &edge
}

func createFlightPlan(start *FlightPathNode, delta float64) {
	return
}

func BetweenLatEdgePoints(point *drone.WayPoint, edge Edge) bool {
	lat := point.Latitude()
	// Make sure we're between the points
	if lat > edge.point1.Latitude() && lat < edge.point2.Latitude() {
		return true
	}

	if lat < edge.point1.Latitude() && lat > edge.point2.Latitude() {
		return true
	}

	return false
}

func BetweenLongEdgePoints(point *drone.WayPoint, edge Edge) bool {
	long := point.Longitude()

	if long > edge.point1.Longitude() && long < edge.point2.Longitude() {
		return true
	}

	if long < edge.point1.Longitude() && long > edge.point2.Longitude() {
		return true
	}

	return false
}
