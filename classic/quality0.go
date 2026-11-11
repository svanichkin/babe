package classic

// smoothLowQualityLuma averages complete 2x2 groups before block coding. This
// makes q0's two-tone masks coarser and cheaper; incomplete edge groups remain
// unchanged. It is deliberately limited to q0.
func smoothLowQualityLuma(plane []byte, w, h int) {
	for y := 0; y+1 < h; y += 2 {
		for x := 0; x+1 < w; x += 2 {
			i := y*w + x
			v := byte((int(plane[i]) + int(plane[i+1]) + int(plane[i+w]) + int(plane[i+w+1])) / 4)
			plane[i], plane[i+1], plane[i+w], plane[i+w+1] = v, v, v, v
		}
	}
}
