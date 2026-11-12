package classic

import (
	"bytes"
	"image"
	"testing"
)

func TestCornerInterpolationFastPath(t *testing.T) {
	var state uint32 = 123456789
	for _, w := range []int{1, 7, 8, 9, 15, 16, 17, 31, 32, 33, 80} {
		for _, h := range []int{1, 7, 8, 9, 15, 16, 17, 31, 32, 33} {
			cols, rows := (w-1+7)/8+1, (h-1+7)/8+1
			grid := cornerGrid{width: w, height: h, step: 8, cols: cols, values: make([][3]byte, cols*rows)}
			for i := range grid.values {
				for c := range 3 {
					state = state*1664525 + 1013904223
					grid.values[i][c] = byte(state >> 24)
				}
			}
			for y := 0; y < h; y++ {
				for x := 0; x < w; x++ {
					gx, gy := min(x/8, max(cols-2, 0)), min(y/8, max(rows-2, 0))
					x0, y0 := gx*8, gy*8
					dx, dy := max(min(x0+8, w-1)-x0, 1), max(min(y0+8, h-1)-y0, 1)
					px, py := x-x0, y-y0
					right, below := min(gx+1, cols-1), min(gy+1, rows-1)
					a, b := grid.values[gy*cols+gx], grid.values[gy*cols+right]
					c, d := grid.values[below*cols+gx], grid.values[below*cols+right]
					var want [3]byte
					for k := range 3 {
						den := dx * dy
						want[k] = byte((int(a[k])*(dx-px)*(dy-py) + int(b[k])*px*(dy-py) + int(c[k])*(dx-px)*py + int(d[k])*px*py + den/2) / den)
					}
					if got := grid.at(x, y); got != want {
						t.Fatalf("%dx%d (%d,%d): got %v want %v", w, h, x, y, got, want)
					}
					if got := grid.lumaAt(x, y); got != want[0] {
						t.Fatalf("%dx%d (%d,%d): luma %d want %d", w, h, x, y, got, want[0])
					}
				}
			}
		}
	}
}

func TestSharedGradientShiftMatchesDivision(t *testing.T) {
	const w, h = 272, 272
	plane := make([]byte, w*h)
	var state uint32 = 987654321
	for i := range plane {
		state = state*1664525 + 1013904223
		plane[i] = byte(state >> 24)
	}
	mesh := makeSharedGradientMesh(plane, w, h)
	for _, size := range []int{16, 32, 64, 128, 256} {
		block := largeGradient{x: 0, y: 0, size: size}
		for y := 0; y < size; y++ {
			for x := 0; x < size; x++ {
				u, v, den := 2*x+1, 2*y+1, 2*size
				top, bottom := mesh.horizontal(2*x+1, 0), mesh.horizontal(2*x+1, size)
				left, right := mesh.vertical(0, 2*y+1), mesh.vertical(size, 2*y+1)
				a, b, c, d := mesh.node(0, 0), mesh.node(size, 0), mesh.node(0, size), mesh.node(size, size)
				lr := (left*(den-u) + right*u + den/2) / den
				tb := (top*(den-v) + bottom*v + den/2) / den
				corner := (a*(den-u)*(den-v) + b*u*(den-v) + c*(den-u)*v + d*u*v + den*den/2) / (den * den)
				want := byte(max(0, min(255, lr+tb-corner)))
				if got := mesh.at(block, x, y); got != want {
					t.Fatalf("size=%d (%d,%d): got %d want %d", size, x, y, got, want)
				}
			}
		}
	}
}

// Verify sparse neighbour resolution against the original dense approach,
// including partial image edges, mixed glyph sizes and both execution modes.
func TestSparseGradientNeighbourResolutionMatchesDense(t *testing.T) {
	const w, h = 37, 29
	grid := makeCornerGrid(w, h, 8, func(x, y int) [3]byte {
		return [3]byte{byte((x*3 + y*5) % 256), byte(100 + (x+y)%51), byte(110 + (x*2+y)%31)}
	})
	baseline := makeCornerGrid(w, h, 8, func(x, y int) [3]byte { return [3]byte{byte((x*2 + y*4) % 256), 128, 128} })
	luma := make([]byte, w*h)
	sizes := make([]byte, w*h)
	gradient := make([]byte, w*h)
	channels := make([]byte, w*h)
	original := make([]byte, w*h*4)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			i := y*w + x
			luma[i] = byte((x*3 + y*4) % 256)
			sizes[i] = 8
			if (x/8+y/8)%3 == 0 {
				sizes[i] = 4
			}
			if x < 16 && y < 16 {
				gradient[i] = 16
			}
			channels[i] = byte((x + y) % 4)
			original[i*4] = luma[i]
			original[i*4+1] = byte(110 + x%31)
			original[i*4+2] = byte(120 + y%23)
			original[i*4+3] = 255
		}
	}
	read := func(x, y int) byte { return luma[y*w+x] }
	for _, smooth := range []bool{false, true} {
		var activeSizes []byte
		if smooth {
			activeSizes = sizes
		}
		dense := make([]byte, w*h)
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				i := y*w + x
				if gradient[i] != 0 || (smooth && sizes[i] <= 4) {
					dense[i] = read(x, y)
					continue
				}
				v, old := grid.at(x, y), baseline.at(x, y)
				residual := int(read(x, y)) - int(old[0])
				if activeSizes != nil {
					residual = blendGlyphResidual(x, y, w, h, 5, activeSizes, residual, read, baseline)
					residual = spreadGlyphResidual(x, y, w, h, 5, activeSizes, residual, read, baseline)
				}
				dense[i] = byte(max(0, min(255, residual+int(v[0]))))
			}
		}
		stitched := stitchLargeGradients(dense, gradient, w, h, false)
		want := make([]byte, len(original))
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				i := y*w + x
				o := i * 4
				Y := int32(dense[i])
				if gradient[i] != 0 {
					Y = int32(stitched[i])
				}
				var cb, cr int32
				if smooth && sizes[i] <= 4 {
					cb, cr = int32(original[o+1])-128, int32(original[o+2])-128
				} else {
					v := grid.at(x, y)
					cb, cr = int32(v[1])-128, int32(v[2])-128
					if smooth && channels[i]&1 == 0 {
						cb = int32(original[o+1]) - 128
					}
					if smooth && channels[i]&2 == 0 {
						cr = int32(original[o+2]) - 128
					}
				}
				R, G, B := Y+((91881*cr)>>16), Y-((22554*cb+46802*cr)>>16), Y+((116130*cb)>>16)
				want[o], want[o+1], want[o+2], want[o+3] = byte(max(0, min(255, R))), byte(max(0, min(255, G))), byte(max(0, min(255, B))), 255
			}
		}
		for _, parallel := range []bool{false, true} {
			d := NewDecoder()
			d.Parallel, d.Smooth = parallel, smooth
			d.sharedGradient = true
			d.gradientMask = gradient
			d.smoothChannels = channels
			d.smoothHasCb, d.smoothHasCr = true, true
			d.dst = image.NewRGBA(image.Rect(0, 0, w, h))
			copy(d.dst.Pix, original)
			d.writeStoredCorners(grid, baseline, read, activeSizes, 5)
			if !bytes.Equal(d.dst.Pix, want) {
				for i := range want {
					if want[i] != d.dst.Pix[i] {
						t.Fatalf("smooth=%v parallel=%v byte=%d got=%d want=%d", smooth, parallel, i, d.dst.Pix[i], want[i])
					}
				}
			}
		}
	}
}
