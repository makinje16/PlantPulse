package drone

import (
	"sync"
)

const SCALE_FACTOR = 1e7

type WayPoint struct {
	lat         float32
	long        float32
	alt         float32
	mapChanLat  map[uint64]chan float32
	mapChanLong map[uint64]chan float32
	mapChanAlt  map[uint64]chan float32

	subCounterLat  uint64
	subCounterLong uint64
	subCounterAlt  uint64

	mu sync.Mutex
}

func NewWayPoint(lat, long, alt float32) *WayPoint {
	return &WayPoint{
		lat:         lat,
		long:        long,
		alt:         alt,
		mapChanLat:  make(map[uint64]chan float32, 10),
		mapChanLong: make(map[uint64]chan float32, 10),
		mapChanAlt:  make(map[uint64]chan float32, 10),
	}
}

func (p *WayPoint) subscribeAlt() (<-chan float32, func()) {
	p.mu.Lock()
	defer p.mu.Unlock()

	ch := make(chan float32)
	futureDelete := p.subCounterAlt
	p.mapChanAlt[p.subCounterAlt] = ch
	p.subCounterAlt++

	unsubscribe := func() {
		close(p.mapChanAlt[futureDelete])
		delete(p.mapChanAlt, futureDelete)
	}

	return ch, unsubscribe
}

func (p *WayPoint) subscribeLat() (<-chan float32, func()) {
	p.mu.Lock()
	defer p.mu.Unlock()

	ch := make(chan float32)
	futureDelete := p.subCounterLat
	p.mapChanLat[p.subCounterLat] = ch
	p.subCounterLat++

	unsubscribe := func() {
		close(p.mapChanLat[futureDelete])
		delete(p.mapChanLat, futureDelete)
	}

	return ch, unsubscribe
}

func (p *WayPoint) subscribeLong() (<-chan float32, func()) {
	p.mu.Lock()
	defer p.mu.Unlock()

	ch := make(chan float32)
	futureDelete := p.subCounterLong
	p.mapChanLong[p.subCounterLong] = ch
	p.subCounterLong++

	unsubscribe := func() {
		close(p.mapChanLong[futureDelete])
		delete(p.mapChanLong, futureDelete)
	}

	return ch, unsubscribe
}

func (p *WayPoint) updateWayPoint(latitude, longitude, altitude float32) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.lat = latitude
	for _, ch := range p.mapChanLat {
		ch <- latitude
	}

	p.long = longitude
	for _, ch := range p.mapChanLong {
		ch <- longitude
	}

	p.alt = altitude
	for _, ch := range p.mapChanAlt {
		ch <- altitude
	}
}

func (p *WayPoint) Equal(other *WayPoint) bool {
	return p.alt == other.alt && p.lat == other.lat && p.long == other.long
}

func (p *WayPoint) Latitude() float32 {
	p.mu.Lock()
	defer p.mu.Unlock()

	return p.lat
}

func (p *WayPoint) Longitude() float32 {
	p.mu.Lock()
	defer p.mu.Unlock()

	return p.long
}

func (p *WayPoint) Altitude() float32 {
	p.mu.Lock()
	defer p.mu.Unlock()

	return p.alt
}
