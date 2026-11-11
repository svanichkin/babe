package classic

// cornerGrid holds shared Y/Cb/Cr values at the corners of a regular block
// grid. Each value is estimated from up to four reconstructed pixels touching
// the corner, so neighbouring cells share exactly the same endpoint.
type cornerGrid struct {
	width, height, step, cols int
	values                    [][3]byte
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
