package drone

type Mission struct {
	waypoints []*WayPoint
}

func NewMission(waypoints []*WayPoint) *Mission {
	return &Mission{waypoints: waypoints}
}

func (m *Mission) Path() []*WayPoint {
	return m.waypoints
}
