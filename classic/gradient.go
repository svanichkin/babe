package classic

// fitGradient chooses an axis-aligned linear ramp only when it closely fits the
// samples and substantially improves on a solid or two-tone representation.
// Endpoints are stored as bytes, independently of the lossy quantization step.
func fitGradient(plane []byte, stride, x0, y0, size int) (start, end byte, vertical, ok bool) {
	var left, right, top, bottom, sum int
	for y := 0; y < size; y++ {
		for x := 0; x < size; x++ {
			v := int(plane[(y0+y)*stride+x0+x])
			sum += v
			if x == 0 {
				left += v
			}
			if x == size-1 {
				right += v
			}
			if y == 0 {
				top += v
			}
			if y == size-1 {
				bottom += v
			}
		}
	}
	left, right = (left+size/2)/size, (right+size/2)/size
	top, bottom = (top+size/2)/size, (bottom+size/2)/size
	mean := sum / (size * size)
	var highSum, lowSum, highCount, lowCount int
	for y := 0; y < size; y++ {
		for x := 0; x < size; x++ {
			v := int(plane[(y0+y)*stride+x0+x])
			if v >= mean {
				highSum += v
				highCount++
			} else {
				lowSum += v
				lowCount++
			}
		}
	}
	if highCount == 0 || lowCount == 0 {
		return 0, 0, false, false
	}
	hi, lo := highSum/highCount, lowSum/lowCount
	var baseline, horizontalError, verticalError int
	for y := 0; y < size; y++ {
		for x := 0; x < size; x++ {
			v := int(plane[(y0+y)*stride+x0+x])
			b := lo
			if v >= mean {
				b = hi
			}
			d := v - b
			baseline += d * d
			d = v - gradientValue(left, right, x, size)
			horizontalError += d * d
			d = v - gradientValue(top, bottom, y, size)
			verticalError += d * d
		}
	}
	a, b, err := left, right, horizontalError
	if verticalError < err {
		a, b, err, vertical = top, bottom, verticalError, true
	}
	if a == b || err*3 >= baseline || err > size*size*9 {
		return 0, 0, false, false
	}
	return byte(a), byte(b), vertical, true
}

func gradientValue(start, end, pos, size int) int {
	return (start*(size-1-pos) + end*pos + (size-1)/2) / (size - 1)
}

func gradientPattern(pattern uint64, bw, bh int) (vertical, ok bool) {
	if !gradientMode || bw != bh || bw < 4 {
		return false, false
	}
	if pattern == 0 {
		return false, true
	}
	if pattern == ^uint64(0)>>(64-uint(bw*bh)) {
		return true, true
	}
	return false, false
}
