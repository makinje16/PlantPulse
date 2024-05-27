package internal

type WayPoint struct {
	x float32
	y float32
	z float32
}

func NewWayPoint(x, y, z float32) *WayPoint {
	return &WayPoint{x, y, z}
}

func (p *WayPoint) updateWayPoint(x, y, z float32) {
	p.x = x
	p.y = y
	p.z = z
}

func (p *WayPoint) X() float32 {
	return p.x
}

func (p *WayPoint) Y() float32 {
	return p.y
}

func (p *WayPoint) Z() float32 {
	return p.z
}
