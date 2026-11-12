package classic

// blurLuma affects only the interior of actual 8x8 or larger luma blocks.
// Block-edge pixels remain unchanged for the separate boundary reconstruction.
// Each scan stays within its block and stops at a sharp step.
func (d *Decoder) blurLuma(width, height int, sizes []byte, get func(int, int) byte, set func(int, int, byte)) {
	const threshold = 48
	n := width * height
	if cap(d.ditherInput) < n {
		d.ditherInput = make([]byte, n)
	} else {
		d.ditherInput = d.ditherInput[:n]
	}
	for axis := 0; axis < 2; axis++ {
		for y := 0; y < height; y++ {
			for x := 0; x < width; x++ {
				d.ditherInput[y*width+x] = get(x, y)
			}
		}
		write := func(y0, y1 int) {
			for y := y0; y < y1; y++ {
				for x := 0; x < width; x++ {
					size := int(sizes[y*width+x])
					if size < 8 {
						continue
					}
					blockX, blockY := x/size*size, y/size*size
					if x == blockX || x == blockX+size-1 || y == blockY || y == blockY+size-1 {
						continue
					}
					radius := min(size/2, 8)
					center := int(d.ditherInput[y*width+x])
					sum, weight := center*(threshold+1), threshold+1
					for _, direction := range [2]int{-1, 1} {
						for distance := 1; distance <= radius; distance++ {
							nx, ny := x, y
							if axis == 0 {
								nx += direction * distance
							} else {
								ny += direction * distance
							}
							if nx < blockX || nx >= blockX+size || ny < blockY || ny >= blockY+size || nx >= width || ny >= height {
								break
							}
							v := int(d.ditherInput[ny*width+nx])
							diff := v - center
							if diff < 0 {
								diff = -diff
							}
							if diff > threshold {
								break
							}
							w := threshold + 1 - diff
							sum += v * w
							weight += w
						}
					}
					blurred := (sum + weight/2) / weight
					// Keep the border untouched for -s, but fade the correction in
					// gradually instead of creating a visible one-pixel frame.
					distance := min(x-blockX, blockX+size-1-x, y-blockY, blockY+size-1-y)
					strength := 8
					if distance == 1 {
						strength = 1
					} else if distance == 2 {
						strength = 4
					}
					set(x, y, byte((center*(8-strength)+blurred*strength+4)/8))
				}
			}
		}
		if d.Parallel {
			processStreams(4, n, true, func(i int) { write(height*i/4, height*(i+1)/4) })
		} else {
			write(0, height)
		}
	}
}
