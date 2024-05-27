package internal

type FlightMode uint8

const (
	FLIGHT_MODE_STABILIZE = 0
	FLIGHT_MODE_ACRO      = 1
	FLIGHT_MODE_ALT_HOLD  = 2
	FLIGHT_MODE_AUTO      = 3
	FLIGHT_MODE_GUIDED    = 4
	FLIGHT_MODE_LOITER    = 5
	FLIGHT_MODE_RTL       = 6
	FLIGHT_MODE_CIRCLE    = 7
	FLIGHT_MODE_LAND      = 8
	FLIGHT_MODE_DRIFT     = 9
	FLIGHT_MODE_SPORT     = 10
	FLIGHT_MODE_FLIP      = 11
	FLIGHT_MODE_AUTOTUNE  = 12
	FLIGHT_MODE_POSHOLD   = 13
	FLIGHT_MODE_BRAKE     = 14
)

func (mode FlightMode) float32() float32 {
	return float32(mode)
}
