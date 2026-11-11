package classic

// profileForQuality maps the public scale to block parameters using an
// empirical curve calibrated on several photographs. The 8.8 fixed-point
// profile is not a guarantee of file sizes or perceptual quality.
func profileForQuality(q int) int {
	if q < 0 {
		q = 0
	}
	if q > 100 {
		q = 100
	}
	points := [...][2]int{
		{0, 0}, {1, 10 * 256}, {5, 22 * 256}, {10, 31 * 256},
		{15, 36 * 256}, {20, 39 * 256}, {21, 40 * 256}, {24, 54 * 256},
		{28, 60 * 256}, {40, 70 * 256}, {50, 73 * 256}, {55, 75 * 256},
		{60, 79 * 256}, {70, 88 * 256}, {75, 92 * 256}, {80, 95 * 256},
		{85, 97 * 256}, {90, 98 * 256}, {99, 99*256 + 192}, {100, 100 * 256},
	}
	for i := 1; i < len(points); i++ {
		a, b := points[i-1], points[i]
		if q <= b[0] {
			return a[1] + (q-a[0])*(b[1]-a[1])/(b[0]-a[0])
		}
	}
	return 100 * 256
}

// interpolateFP linearly interpolates integer quantization steps from a profile.
func interpolateFP(q, q0, q1, v0, v1 int) int {
	return v0 - ((v0-v1)*(q-q0*256)+(q1-q0)*128)/((q1-q0)*256)
}

// Quantization steps are integer levels, so nearby qualities can share a step.
func quantStepsForQuality(quality int) (int, int) {
	q := profileForQuality(quality)
	var y, c int
	switch {
	case q <= 20*256:
		y = interpolateFP(q, 0, 20, 32, 4)
	case q <= 40*256:
		y = interpolateFP(q, 20, 40, 4, 1)
	default:
		y = 1
	}
	switch {
	case q <= 20*256:
		c = interpolateFP(q, 0, 20, 16, 8)
	case q <= 40*256:
		c = interpolateFP(q, 20, 40, 8, 4)
	case q <= 50*256:
		c = interpolateFP(q, 40, 50, 4, 1)
	default:
		c = 1
	}
	return y, c
}

// The lookup table keeps non-power-of-two quantization off the division hot path.
var quantTable = func() [33][256]byte {
	var t [33][256]byte
	for step := 1; step <= 32; step++ {
		for v := 0; v < 256; v++ {
			t[step][v] = byte(min(255, (v+step/2)/step*step))
		}
	}
	return t
}()

func quantizeLevel(v uint8, step int) uint8 {
	if step <= 1 {
		return v
	}
	if step&(step-1) == 0 {
		return uint8(min(255, (int(v)+step/2)&-step))
	}
	if step <= 32 {
		return quantTable[step][v]
	}
	return byte(min(255, (int(v)+step/2)/step*step))
}

var quadReciprocal = [5]uint32{0, 65536, 32768, 21846, 16384}

// Exact floor(sum/count) for 1..4 eight-bit samples.
func quadMean(sum uint64, count uint32) uint8 {
	return uint8((uint32(sum) * quadReciprocal[count]) >> 16)
}

// Classify the four samples without a data-dependent branch for each pixel.
// Differences lie in [-255,255], so the sign bit gives an exact 0/1 mask.
func quadLevels(v0, v1, v2, v3 byte, step int) (fg, bg byte, pattern uint64) {
	sum := uint32(v0) + uint32(v1) + uint32(v2) + uint32(v3)
	avg := sum / 4
	f0 := 1 ^ ((uint32(v0) - avg) >> 31)
	f1 := 1 ^ ((uint32(v1) - avg) >> 31)
	f2 := 1 ^ ((uint32(v2) - avg) >> 31)
	f3 := 1 ^ ((uint32(v3) - avg) >> 31)
	count := f0 + f1 + f2 + f3
	if count == 4 {
		v := quantizeLevel(byte(avg), step)
		return v, v, 0
	}
	fgSum := uint32(v0)*f0 + uint32(v1)*f1 + uint32(v2)*f2 + uint32(v3)*f3
	fg = quantizeLevel(quadMean(uint64(fgSum), count), step)
	bg = quantizeLevel(quadMean(uint64(sum-fgSum), 4-count), step)
	return fg, bg, uint64(f0<<3 | f1<<2 | f2<<1 | f3)
}
