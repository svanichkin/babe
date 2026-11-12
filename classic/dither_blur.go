package classic

// blurLuma is an experimental edge-aware decoder operation. It compares each
// sample with its four neighbours, giving closer values more weight and ignoring
// differences over the threshold. Reads come from an immutable snapshot.
func (d *Decoder) blurLuma(width, height int, get func(int, int) byte, set func(int, int, byte)) {
	const threshold = 12
	n := width * height
	if cap(d.ditherInput) < n {
		d.ditherInput = make([]byte, n)
	} else {
		d.ditherInput = d.ditherInput[:n]
	}
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			d.ditherInput[y*width+x] = get(x, y)
		}
	}
	write := func(y0, y1 int) {
		for y := y0; y < y1; y++ {
			for x := 0; x < width; x++ {
				center := int(d.ditherInput[y*width+x])
				sum, weight := center*(threshold+1), threshold+1
				for _, p := range [4][2]int{{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}} {
					if p[0] < 0 || p[0] >= width || p[1] < 0 || p[1] >= height {
						continue
					}
					v := int(d.ditherInput[p[1]*width+p[0]])
					diff := v - center
					if diff < 0 {
						diff = -diff
					}
					if diff > threshold {
						continue
					}
					w := threshold + 1 - diff
					sum += v * w
					weight += w
				}
				set(x, y, byte((sum+weight/2)/weight))
			}
		}
	}
	if d.Parallel {
		processStreams(4, n, true, func(i int) { write(height*i/4, height*(i+1)/4) })
	} else {
		write(0, height)
	}
}
