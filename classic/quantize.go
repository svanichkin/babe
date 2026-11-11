package classic

// Quantize the final block levels, not the input pixels: block decisions and
// pattern thresholds retain their original precision. Equal rounded levels
// become solid blocks, so their pattern and background are never written.
func quantStepsForQuality(quality int) (y, chroma int) {
	switch {
	case quality <= 0:
		return 32, 16
	case quality < 20:
		return 4, 8
	case quality < 60:
		return 2, 4
	case quality < 90:
		return 1, 2
	default:
		return 1, 1
	}
}

func quantizeLevel(v uint8, step int) uint8 {
	if step <= 1 {
		return v
	}
	// All codec steps are powers of two. Masking avoids integer division for
	// every block level while producing exactly the same rounded values.
	return uint8(min(255, (int(v)+step/2)&-step))
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
