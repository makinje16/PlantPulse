package internal

type FlightMode uint8

const (
	STABILIZE = 1
	ACRO      = 2
	ALT_HOLD  = 3
	AUTO      = 4
	GUIDED    = 5
	LOITER    = 6
	RTL       = 7
	CIRCLE    = 8
	LAND      = 9
	DRIFT     = 10
	SPORT     = 11
	FLIP      = 12
	AUTOTUNE  = 13
	POSHOLD   = 14
	BRAKE     = 15
)
