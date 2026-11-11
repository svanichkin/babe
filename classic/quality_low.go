package classic

// smoothLowQualityLuma averages low-contrast 2x2 groups before block coding.
// The quality-dependent threshold controls how many groups are smoothed;
// incomplete edge groups remain unchanged.
func smoothLowQualityLuma(plane []byte, w, h, threshold int) {
	for y := 0; y+1 < h; y += 2 {
		for x := 0; x+1 < w; x += 2 {
			i := y*w + x
			if threshold < 255 {
				lo := min(plane[i], plane[i+1], plane[i+w], plane[i+w+1])
				hi := max(plane[i], plane[i+1], plane[i+w], plane[i+w+1])
				if int(hi)-int(lo) > threshold {
					continue
				}
			}
			v := byte((int(plane[i]) + int(plane[i+1]) + int(plane[i+w]) + int(plane[i+w+1])) / 4)
			plane[i], plane[i+1], plane[i+w], plane[i+w+1] = v, v, v, v
		}
	}
}
