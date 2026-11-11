package classic

// Quantize the final block levels, not the input pixels: block decisions and
// pattern thresholds retain their original precision. Equal rounded levels
// become solid blocks, so their pattern and background are never written.
func quantStepsForQuality(quality int) (y, chroma int) {
	switch {
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
	return uint8(min(255, (int(v)+step/2)/step*step))
}
