package classic

// cornerGrid holds shared Y/Cb/Cr values at the corners of a regular block
// grid. Each value is estimated from up to four reconstructed pixels touching
// the corner, so neighbouring cells share exactly the same endpoint.
type cornerGrid struct {
	width, height, step, cols int
	values                    [][3]byte
	flags                     []byte // non-nil for sparse source corners
}

// fillMissing resolves sparse corner channels from the decoded base image.
func (g *cornerGrid) fillMissing(baseline cornerGrid) {
	if g.flags == nil {
		return
	}
	for i := range g.values {
		for c := range 3 {
			if g.flags[i]&(1<<c) == 0 {
				g.values[i][c] = baseline.values[i][c]
			} else {
				g.values[i][c] += baseline.values[i][c]
			}
		}
	}
}

func makeCornerGrid(w, h, step int, sample func(x, y int) [3]byte) cornerGrid {
	cols := (w-1+step-1)/step + 1
	rows := (h-1+step-1)/step + 1
	g := cornerGrid{width: w, height: h, step: step, cols: cols, values: make([][3]byte, cols*rows)}
	for gy := 0; gy < rows; gy++ {
		y := min(gy*step, h-1)
		for gx := 0; gx < cols; gx++ {
			x := min(gx*step, w-1)
			var sum [3]int
			count := 0
			for sy := max(0, y-1); sy <= y; sy++ {
				for sx := max(0, x-1); sx <= x; sx++ {
					v := sample(sx, sy)
					for c := range 3 {
						sum[c] += int(v[c])
					}
					count++
				}
			}
			for c := range 3 {
				g.values[gy*cols+gx][c] = byte((sum[c] + count/2) / count)
			}
		}
	}
	return g
}

func (g cornerGrid) at(x, y int) [3]byte {
	// Most stored corner grids use complete 8×8 cells. Their interpolation
	// denominator is 64, avoiding per-pixel integer divisions.
	if g.step == 8 {
		gx, gy := x>>3, y>>3
		if (gx+1)*8 < g.width && (gy+1)*8 < g.height {
			i := gy*g.cols + gx
			a, b, c, d := g.values[i], g.values[i+1], g.values[i+g.cols], g.values[i+g.cols+1]
			px, py := x&7, y&7
			var out [3]byte
			for k := range 3 {
				out[k] = byte((int(a[k])*(8-px)*(8-py) + int(b[k])*px*(8-py) + int(c[k])*(8-px)*py + int(d[k])*px*py + 32) >> 6)
			}
			return out
		}
	}
	rows := len(g.values) / g.cols
	gx, gy := min(x/g.step, max(g.cols-2, 0)), min(y/g.step, max(rows-2, 0))
	x0, y0 := gx*g.step, gy*g.step
	x1, y1 := min(x0+g.step, g.width-1), min(y0+g.step, g.height-1)
	dx, dy := max(x1-x0, 1), max(y1-y0, 1)
	px, py := x-x0, y-y0
	right, below := min(gx+1, g.cols-1), min(gy+1, rows-1)
	a, b := g.values[gy*g.cols+gx], g.values[gy*g.cols+right]
	c, d := g.values[below*g.cols+gx], g.values[below*g.cols+right]
	var out [3]byte
	den := dx * dy
	for i := range 3 {
		value := int(a[i])*(dx-px)*(dy-py) + int(b[i])*px*(dy-py) + int(c[i])*(dx-px)*py + int(d[i])*px*py
		out[i] = byte((value + den/2) / den)
	}
	return out
}

// lumaAt uses the same interpolation as at without computing unused chroma.
func (g cornerGrid) lumaAt(x, y int) byte {
	if g.step == 8 {
		gx, gy := x>>3, y>>3
		if (gx+1)*8 < g.width && (gy+1)*8 < g.height {
			i := gy*g.cols + gx
			a, b, c, d := int(g.values[i][0]), int(g.values[i+1][0]), int(g.values[i+g.cols][0]), int(g.values[i+g.cols+1][0])
			px, py := x&7, y&7
			return byte((a*(8-px)*(8-py) + b*px*(8-py) + c*(8-px)*py + d*px*py + 32) >> 6)
		}
	}
	rows := len(g.values) / g.cols
	gx, gy := min(x/g.step, max(g.cols-2, 0)), min(y/g.step, max(rows-2, 0))
	x0, y0 := gx*g.step, gy*g.step
	x1, y1 := min(x0+g.step, g.width-1), min(y0+g.step, g.height-1)
	dx, dy := max(x1-x0, 1), max(y1-y0, 1)
	px, py := x-x0, y-y0
	right, below := min(gx+1, g.cols-1), min(gy+1, rows-1)
	a, b := int(g.values[gy*g.cols+gx][0]), int(g.values[gy*g.cols+right][0])
	c, d := int(g.values[below*g.cols+gx][0]), int(g.values[below*g.cols+right][0])
	den := dx * dy
	return byte((a*(dx-px)*(dy-py) + b*px*(dy-py) + c*(dx-px)*py + d*px*py + den/2) / den)
}

// cell8 evaluates at for every pixel of the complete 8x8 cell whose top-left
// pixel is (bx, by), using the same integer formula as at's fast path. It
// reports false when at would use its general path for this cell.
func (g cornerGrid) cell8(bx, by int, out *[64][3]byte) bool {
	gx, gy := bx>>3, by>>3
	if g.step != 8 || bx&7 != 0 || by&7 != 0 || (gx+1)*8 >= g.width || (gy+1)*8 >= g.height {
		return false
	}
	i := gy*g.cols + gx
	a, b, c, d := g.values[i], g.values[i+1], g.values[i+g.cols], g.values[i+g.cols+1]
	// a(8-px)(8-py)+b px(8-py)+c(8-px)py+d px py = 8L + (R-L)px, where L and R
	// are the left and right column values for this row: an exact rewrite.
	for k := range 3 {
		ak, bk, ck, dk := int(a[k]), int(b[k]), int(c[k]), int(d[k])
		for py := 0; py < 8; py++ {
			l, r := ak*(8-py)+ck*py, bk*(8-py)+dk*py
			v, step := 8*l+32, r-l
			row := out[py*8 : py*8+8]
			for px := range row {
				row[px][k] = byte(v >> 6)
				v += step
			}
		}
	}
	return true
}

// lumaCell8 is cell8 for the first channel only.
func (g cornerGrid) lumaCell8(bx, by int, out *[64]byte) bool {
	gx, gy := bx>>3, by>>3
	if g.step != 8 || bx&7 != 0 || by&7 != 0 || (gx+1)*8 >= g.width || (gy+1)*8 >= g.height {
		return false
	}
	i := gy*g.cols + gx
	a, b, c, d := int(g.values[i][0]), int(g.values[i+1][0]), int(g.values[i+g.cols][0]), int(g.values[i+g.cols+1][0])
	for py := 0; py < 8; py++ {
		l, r := a*(8-py)+c*py, b*(8-py)+d*py
		v, step := 8*l+32, r-l
		row := out[py*8 : py*8+8]
		for px := range row {
			row[px] = byte(v >> 6)
			v += step
		}
	}
	return true
}

func (g cornerGrid) writeRGB(pix []byte, stride, y0, y1 int) {
	for y := y0; y < y1; y++ {
		for x := 0; x < g.width; x++ {
			v := g.at(x, y)
			Y, Cb, Cr := int32(v[0]), int32(v[1])-128, int32(v[2])-128
			R := Y + ((91881 * Cr) >> 16)
			G := Y - ((22554*Cb + 46802*Cr) >> 16)
			B := Y + ((116130 * Cb) >> 16)
			o := y*stride + x*4
			pix[o], pix[o+1], pix[o+2], pix[o+3] = byte(max(0, min(255, R))), byte(max(0, min(255, G))), byte(max(0, min(255, B))), 255
		}
	}
}

func (d *Decoder) writeCornerGradient(g cornerGrid) {
	if d.Parallel {
		processStreams(4, g.width*g.height, true, func(i int) {
			y0, y1 := g.height*i/4, g.height*(i+1)/4
			g.writeRGB(d.dst.Pix, d.dst.Stride, y0, y1)
		})
	} else {
		g.writeRGB(d.dst.Pix, d.dst.Stride, 0, g.height)
	}
}

// writeStoredCorners retains the original block's luma residual around its
// estimated ramp while reconstructing the low-frequency color from source
// corners. This preserves more detail than replacing every pixel with a ramp.
// blendGlyphResidual interpolates across a cell boundary when the luma step is
// small enough to be a plausible coding seam. High-contrast edges are kept.
func blendGlyphResidual(x, y, width, height, minSize int, sizes []byte, residual int, luma func(int, int) byte, baseline cornerGrid) int {
	step := int(sizes[y*width+x])
	if step < minSize || step <= 1 {
		return residual
	}
	value := int(luma(x, y))
	delta := 0
	adjust := func(nx, ny int) {
		if nx < 0 || nx >= width || ny < 0 || ny >= height {
			return
		}
		if sizes[ny*width+nx] == 0 {
			return
		}
		n := int(luma(nx, ny))
		diff := value - n
		if diff < -24 || diff > 24 {
			return
		}
		other := n - int(baseline.lumaAt(nx, ny))
		delta += (other - residual) * 3 / 8
	}
	if x%step == 0 {
		adjust(x-1, y)
	} else if x%step == step-1 {
		adjust(x+1, y)
	}
	if y%step == 0 {
		adjust(x, y-1)
	} else if y%step == step-1 {
		adjust(x, y+1)
	}
	return residual + delta
}

// spreadGlyphResidual extends the existing seam correction into the interior,
// without changing the seam itself or any block smaller than minSize.
func spreadGlyphResidual(x, y, width, height, minSize int, sizes []byte, residual int, luma func(int, int) byte, baseline cornerGrid) int {
	step := int(sizes[y*width+x])
	if step < minSize || step <= 1 {
		return residual
	}
	lx, ly := x%step, y%step
	if lx == 0 || ly == 0 || lx == step-1 || ly == step-1 {
		return residual
	}
	apply := func(ex, ey, distance int) {
		if distance > 3 || ex < 0 || ex >= width || ey < 0 || ey >= height {
			return
		}
		edge := int(luma(ex, ey)) - int(baseline.lumaAt(ex, ey))
		corrected := blendGlyphResidual(ex, ey, width, height, minSize, sizes, edge, luma, baseline)
		residual += (corrected - edge) * (4 - distance) / 4
	}
	apply(x-lx, y, lx)
	apply(x+step-1-lx, y, step-1-lx)
	apply(x, y-ly, ly)
	apply(x, y+step-1-ly, step-1-ly)
	return residual
}

// Compute each edge correction once for a complete 8x8 glyph, then reuse it
// for the interior pixels. Keep the original signed integer divisions: changing
// their rounding would change the decoded image.
func glyphResidual8(x0, y0, width, height, minSize int, sizes []byte, luma func(int, int) byte, baseline cornerGrid, out *[64]int16) {
	var edges [64]int16
	var base [64]byte
	if !baseline.lumaCell8(x0, y0, &base) {
		for i := range base {
			base[i] = baseline.lumaAt(x0+i%8, y0+i/8)
		}
	}
	for y := 0; y < 8; y++ {
		for x := 0; x < 8; x++ {
			i := y*8 + x
			r := int(luma(x0+x, y0+y)) - int(base[i])
			out[i] = int16(r)
			if x == 0 || x == 7 || y == 0 || y == 7 {
				edges[i] = int16(blendGlyphResidual(x0+x, y0+y, width, height, minSize, sizes, r, luma, baseline) - r)
				out[i] += edges[i]
			}
		}
	}
	for y := 1; y < 7; y++ {
		for x := 1; x < 7; x++ {
			i := y*8 + x
			if x <= 3 {
				out[i] += edges[y*8] * int16(4-x) / 4
			}
			if x >= 4 {
				out[i] += edges[y*8+7] * int16(x-3) / 4
			}
			if y <= 3 {
				out[i] += edges[x] * int16(4-y) / 4
			}
			if y >= 4 {
				out[i] += edges[56+x] * int16(y-3) / 4
			}
		}
	}
}

func (d *Decoder) writeStoredCorners(g, baseline cornerGrid, luma func(x, y int) byte, sizes []byte, minSize int) {
	var resolved []byte
	// resolveAdjacent reconstructs an ordinary glyph pixel next to a gradient.
	resolveAdjacent := func(x, y int) byte {
		i := y*g.width + x
		if d.Smooth && sizes != nil && sizes[i] <= 4 {
			return luma(x, y)
		}
		v, old := g.at(x, y), baseline.lumaAt(x, y)
		residual := int(luma(x, y)) - int(old)
		if sizes != nil {
			residual = blendGlyphResidual(x, y, g.width, g.height, minSize, sizes, residual, luma, baseline)
			if d.Smooth {
				residual = spreadGlyphResidual(x, y, g.width, g.height, minSize, sizes, residual, luma, baseline)
			}
		}
		return byte(max(0, min(255, residual+int(v[0]))))
	}
	if d.gradientMask != nil && d.gradients != nil {
		// The stitcher reads gradient pixels and ordinary glyphs immediately
		// across their boundaries only. Resolve exactly those pixels.
		n := g.width * g.height
		if cap(d.resolvedBuf) < n {
			d.resolvedBuf = make([]byte, n)
			d.stitchedBuf = make([]byte, n)
		}
		plane, stitched := d.resolvedBuf[:n], d.stitchedBuf[:n]
		mark := func(x, y int) {
			if x >= 0 && y >= 0 && x < g.width && y < g.height && d.gradientMask[y*g.width+x] == 0 {
				plane[y*g.width+x] = resolveAdjacent(x, y)
			}
		}
		// Each band of rows writes only its own rows of plane, then of stitched.
		parallelRows(d.Parallel, g.width, g.height, func(y0, y1 int) {
			for _, b := range d.gradients {
				if b.y+b.size < y0 || b.y-1 >= y1 {
					continue
				}
				for y := max(b.y, y0); y < min(b.y+b.size, y1); y++ {
					for x := b.x; x < b.x+b.size; x++ {
						plane[y*g.width+x] = luma(x, y)
					}
					mark(b.x-1, y)
					mark(b.x+b.size, y)
				}
				for _, y := range [2]int{b.y - 1, b.y + b.size} {
					if y >= y0 && y < y1 {
						for x := b.x; x < b.x+b.size; x++ {
							mark(x, y)
						}
					}
				}
			}
		})
		parallelRows(d.Parallel, g.width, g.height, func(y0, y1 int) {
			stitchGradientRows(stitched, plane, d.gradientMask, g.width, g.height, d.gradients, !d.sharedGradient, y0, y1)
		})
		resolved = stitched
	} else if d.gradientMask != nil {
		resolved = make([]byte, g.width*g.height)
		for y := 0; y < g.height; y++ {
			for x := 0; x < g.width; x++ {
				i := y*g.width + x
				if d.gradientMask[i] != 0 {
					resolved[i] = luma(x, y)
					continue
				}
				adjacent := (x > 0 && d.gradientMask[i-1] != 0) || (x+1 < g.width && d.gradientMask[i+1] != 0) || (y > 0 && d.gradientMask[i-g.width] != 0) || (y+1 < g.height && d.gradientMask[i+g.width] != 0)
				if adjacent {
					resolved[i] = resolveAdjacent(x, y)
				}
			}
		}
		resolved = stitchLargeGradients(resolved, d.gradientMask, g.width, g.height, !d.sharedGradient)
	}
	write := func(y0, y1 int) {
		for by := y0 / 8 * 8; by < y1; by += 8 {
			for bx := 0; bx < g.width; bx += 8 {
				var residuals [64]int16
				cached := d.Smooth && sizes != nil && bx+8 <= g.width && by+8 <= g.height && sizes[by*g.width+bx] == 8 && (d.gradientMask == nil || d.gradientMask[by*g.width+bx] == 0)
				if cached {
					glyphResidual8(bx, by, g.width, g.height, minSize, sizes, luma, baseline, &residuals)
				}
				var cell [64][3]byte
				hasCell, cellReady := false, false
				for y := max(y0, by); y < min(y1, by+8); y++ {
					for x := bx; x < min(g.width, bx+8); x++ {
						o := y*d.dst.Stride + x*4
						if d.Smooth && sizes != nil && sizes[y*g.width+x] <= 4 {
							Y := int32(luma(x, y))
							if resolved != nil && d.gradientMask[y*g.width+x] != 0 {
								Y = int32(resolved[y*g.width+x])
							}
							Cb, Cr := int32(d.dst.Pix[o+1])-128, int32(d.dst.Pix[o+2])-128
							if !d.smoothHasCb {
								Cb = 0
							}
							if !d.smoothHasCr {
								Cr = 0
							}
							R, G, B := Y+((91881*Cr)>>16), Y-((22554*Cb+46802*Cr)>>16), Y+((116130*Cb)>>16)
							d.dst.Pix[o], d.dst.Pix[o+1], d.dst.Pix[o+2], d.dst.Pix[o+3] = byte(max(0, min(255, R))), byte(max(0, min(255, G))), byte(max(0, min(255, B))), 255
							continue
						}
						var v [3]byte
						if !cellReady {
							hasCell, cellReady = g.cell8(bx, by, &cell), true
						}
						if hasCell {
							v = cell[(y-by)*8+x-bx]
						} else {
							v = g.at(x, y)
						}
						var Y int32
						if resolved != nil && d.gradientMask[y*g.width+x] != 0 {
							Y = int32(resolved[y*g.width+x])
						} else if cached {
							Y = int32(max(0, min(255, int(residuals[(y-by)*8+x-bx])+int(v[0]))))
						} else {
							old := baseline.lumaAt(x, y)
							residual := int(luma(x, y)) - int(old)
							if sizes != nil {
								residual = blendGlyphResidual(x, y, g.width, g.height, minSize, sizes, residual, luma, baseline)
								if d.Smooth {
									residual = spreadGlyphResidual(x, y, g.width, g.height, minSize, sizes, residual, luma, baseline)
								}
							}
							Y = int32(max(0, min(255, residual+int(v[0]))))
						}
						Cb, Cr := int32(v[1])-128, int32(v[2])-128
						if d.Smooth {
							mask := d.smoothChannels[y*g.width+x]
							if mask&1 == 0 {
								Cb = 0
								if d.smoothHasCb {
									Cb = int32(d.dst.Pix[o+1]) - 128
								}
							}
							if mask&2 == 0 {
								Cr = 0
								if d.smoothHasCr {
									Cr = int32(d.dst.Pix[o+2]) - 128
								}
							}
						}
						R := Y + ((91881 * Cr) >> 16)
						G := Y - ((22554*Cb + 46802*Cr) >> 16)
						B := Y + ((116130 * Cb) >> 16)
						d.dst.Pix[o], d.dst.Pix[o+1], d.dst.Pix[o+2], d.dst.Pix[o+3] = byte(max(0, min(255, R))), byte(max(0, min(255, G))), byte(max(0, min(255, B))), 255
					}
				}
			}
		}
	}
	parallelRows(d.Parallel, g.width, g.height, write)
}
