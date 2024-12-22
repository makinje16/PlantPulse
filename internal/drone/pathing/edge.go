package pathing

import "PlantPulse/internal/drone"

type Edge struct {
	point1 *drone.WayPoint
	point2 *drone.WayPoint
}

func (e *Edge) GetEdgePivot() (*drone.WayPoint, *drone.WayPoint) {
	if e.point1.Latitude() < e.point2.Latitude() {
		return e.point1, e.point2
	}

	return e.point2, e.point1
}
