package utils

import "math"

// Convert pitch to quaternion for -90 degree pitch (gimbal facing down)
func EulerToQuaternion(pitch, roll, yaw float64) (w, x, y, z float64) {
	// Half angles
	cy := math.Cos(yaw * 0.5)
	sy := math.Sin(yaw * 0.5)
	cr := math.Cos(roll * 0.5)
	sr := math.Sin(roll * 0.5)
	cp := math.Cos(pitch * 0.5)
	sp := math.Sin(pitch * 0.5)

	// Calculate quaternion components
	w = cy*cr*cp + sy*sr*sp
	x = cy*sr*cp - sy*cr*sp
	y = cy*cr*sp + sy*sr*cp
	z = sy*cr*cp - cy*sr*sp
	return
}

// Converts a quaternion (w, x, y, z) into Euler angles (roll, pitch, yaw) in degrees.
func QuaternionToEulerDegrees(w, x, y, z float64) (roll, pitch, yaw float64) {
	// Roll (x-axis rotation)
	sinr_cosp := 2 * (w*x + y*z)
	cosr_cosp := 1 - 2*(x*x+y*y)
	roll = math.Atan2(sinr_cosp, cosr_cosp)

	// Pitch (y-axis rotation)
	sinp := 2 * (w*y - z*x)
	if math.Abs(sinp) >= 1 {
		pitch = math.Copysign(math.Pi/2, sinp) // use 90 degrees if out of range
	} else {
		pitch = math.Asin(sinp)
	}

	// Yaw (z-axis rotation)
	siny_cosp := 2 * (w*z + x*y)
	cosy_cosp := 1 - 2*(y*y+z*z)
	yaw = math.Atan2(siny_cosp, cosy_cosp)

	// Convert radians to degrees
	roll = roll * (180 / math.Pi)
	pitch = pitch * (180 / math.Pi)
	yaw = yaw * (180 / math.Pi)

	return roll, pitch, yaw
}
