package internal

type FlightMode uint8

const (
	STABILIZE = 0
	ACRO      = 1
	ALT_HOLD  = 2
	AUTO      = 3
	GUIDED    = 4
	LOITER    = 5
	RTL       = 6
	CIRCLE    = 7
	LAND      = 8
	DRIFT     = 9
	SPORT     = 10
	FLIP      = 11
	AUTOTUNE  = 12
	POSHOLD   = 13
	BRAKE     = 14
)

func (mode FlightMode) float32() float32 {
	return float32(mode)
}
