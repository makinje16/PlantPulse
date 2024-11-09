package common

import (
	"math"

	"golang.org/x/exp/constraints"
)

func DegreesToRadians[T constraints.Float](degrees T) T {
	return degrees * T(math.Pi) / 180
}
