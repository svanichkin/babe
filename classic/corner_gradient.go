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
		other := n - int(baseline.at(nx, ny)[0])
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
		edge := int(luma(ex, ey)) - int(baseline.at(ex, ey)[0])
		corrected := blendGlyphResidual(ex, ey, width, height, minSize, sizes, edge, luma, baseline)
		residual += (corrected - edge) * (4 - distance) / 4
	}
	apply(x-lx, y, lx)
	apply(x+step-1-lx, y, step-1-lx)
	apply(x, y-ly, ly)
	apply(x, y+step-1-ly, step-1-ly)
	return residual
}

func (d *Decoder) writeStoredCorners(g, baseline cornerGrid, luma func(x, y int) byte, sizes []byte, minSize int) {
	write := func(y0, y1 int) {
		for y := y0; y < y1; y++ {
			for x := 0; x < g.width; x++ {
				o := y*d.dst.Stride + x*4
				if d.Smooth && sizes != nil && sizes[y*g.width+x] <= 4 {
					Y := int32(luma(x, y))
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
				v, old := g.at(x, y), baseline.at(x, y)
				residual := int(luma(x, y)) - int(old[0])
				if sizes != nil {
					residual = blendGlyphResidual(x, y, g.width, g.height, minSize, sizes, residual, luma, baseline)
					if d.Smooth {
						residual = spreadGlyphResidual(x, y, g.width, g.height, minSize, sizes, residual, luma, baseline)
					}
				}
				Y := int32(max(0, min(255, residual+int(v[0]))))
				if d.gradientMask != nil && d.gradientMask[y*g.width+x] != 0 {
					Y = int32(luma(x, y))
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
	if d.Parallel {
		processStreams(4, g.width*g.height, true, func(i int) { write(g.height*i/4, g.height*(i+1)/4) })
	} else {
		write(0, g.height)
	}
}
