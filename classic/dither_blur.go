package classic

// blurGlyphLuma8 is the block-local form of blurLuma for the validated 8x8
// macroblock partition. Both passes stay inside each glyph, so two 64-byte
// buffers replace the full-image snapshots without changing any sample.
func (d *Decoder) blurGlyphLuma8(width, height int, sizes []byte, get func(int, int) byte, set func(int, int, byte)) {
	rows := height / 8
	write := func(first, last int) {
		for by := first * 8; by < last*8; by += 8 {
			for bx := 0; bx+8 <= width; bx += 8 {
				if sizes[by*width+bx] != 8 {
					continue
				}
				var a [64]byte
				for y := 0; y < 8; y++ {
					for x := 0; x < 8; x++ {
						a[y*8+x] = get(bx+x, by+y)
					}
				}
				if !blurBlock8(&a) {
					continue
				}
				for y := 1; y < 7; y++ {
					for x := 1; x < 7; x++ {
						set(bx+x, by+y, a[y*8+x])
					}
				}
			}
		}
	}
	if d.Parallel {
		processStreams(4, width*height, true, func(i int) { write(rows*i/4, rows*(i+1)/4) })
	} else {
		write(0, rows)
	}
}

// blurGlyphLuma8Pix is blurGlyphLuma8 for the first channel of RGBA pixels.
func (d *Decoder) blurGlyphLuma8Pix(pix []byte, stride, width, height int, sizes []byte) {
	rows := height / 8
	write := func(first, last int) {
		for by := first * 8; by < last*8; by += 8 {
			for bx := 0; bx+8 <= width; bx += 8 {
				if sizes[by*width+bx] != 8 {
					continue
				}
				var a [64]byte
				for y := 0; y < 8; y++ {
					row := pix[(by+y)*stride+bx*4:]
					row = row[:29]
					for x := 0; x < 8; x++ {
						a[y*8+x] = row[x*4]
					}
				}
				if !blurBlock8(&a) {
					continue
				}
				for y := 1; y < 7; y++ {
					row := pix[(by+y)*stride+bx*4:]
					row = row[:29]
					for x := 1; x < 7; x++ {
						row[x*4] = a[y*8+x]
					}
				}
			}
		}
	}
	if d.Parallel {
		processStreams(4, width*height, true, func(i int) { write(rows*i/4, rows*(i+1)/4) })
	} else {
		write(0, rows)
	}
}

// blurBlock8 runs the horizontal and vertical edge-preserving passes on one
// 8x8 glyph and reports whether any interior sample may have changed. A scan
// only accumulates neighbours within 48 levels of its centre and stops at the
// first larger step. When every adjacent pair is equal or differs by more than
// 48, each scan sees only copies of its centre, so both passes are identities.
func blurBlock8(a *[64]byte) bool {
	active := false
	for y := 0; y < 8 && !active; y++ {
		for x := 0; x < 8; x++ {
			v := int(a[y*8+x])
			if x < 7 {
				if d := v - int(a[y*8+x+1]); d != 0 && d >= -48 && d <= 48 {
					active = true
					break
				}
			}
			if y < 7 {
				if d := v - int(a[y*8+x+8]); d != 0 && d >= -48 && d <= 48 {
					active = true
					break
				}
			}
		}
	}
	if !active {
		return false
	}
	var b [64]byte
	src, dst := a, &b
	for axis := 0; axis < 2; axis++ {
		*dst = *src
		step := 1
		if axis == 1 {
			step = 8
		}
		for y := 1; y < 7; y++ {
			for x := 1; x < 7; x++ {
				i, position := y*8+x, x
				if axis == 1 {
					position = y
				}
				center := int(src[i])
				sum, weight := center*49, 49
				for distance := 1; distance <= 4 && position-distance >= 0; distance++ {
					v := int(src[i-distance*step])
					diff := v - center
					if diff < 0 {
						diff = -diff
					}
					if diff > 48 {
						break
					}
					w := 49 - diff
					sum += v * w
					weight += w
				}
				for distance := 1; distance <= 4 && position+distance < 8; distance++ {
					v := int(src[i+distance*step])
					diff := v - center
					if diff < 0 {
						diff = -diff
					}
					if diff > 48 {
						break
					}
					w := 49 - diff
					sum += v * w
					weight += w
				}
				blurred := (sum + weight/2) / weight
				strength := 8
				switch min(x, 7-x, y, 7-y) {
				case 1:
					strength = 1
				case 2:
					strength = 4
				}
				dst[i] = byte((center*(8-strength) + blurred*strength + 4) / 8)
			}
		}
		src, dst = dst, src
	}
	// After two passes the result is back in a.
	return true
}

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
