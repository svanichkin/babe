package classic

import "encoding/binary"

// Called only after validatePayload has checked every size/type/pattern count.
// Separate planes avoid three channel workers writing the same RGBA cache lines.
func decodeValidatedQuadPlane(data, plane []byte, w, h int) {
	var streams [5][]byte
	pos := 4
	for i := range streams {
		n := int(binary.BigEndian.Uint32(data[pos:]))
		pos += 4
		streams[i] = data[pos : pos+n]
		pos += n
	}
	sizes, types, patterns := newBitReader(streams[0]), newBitReader(streams[1]), newBitReader(streams[2])
	fg, _ := newDeltaStream(streams[3], len(streams[3]))
	bg, _ := newDeltaStream(streams[4], len(streams[4]))
	fw, fh := w/2*2, h/2*2
	for y := 0; y < fh; y += 2 {
		row0, row1 := plane[y*w:y*w+fw], plane[(y+1)*w:(y+1)*w+fw]
		for x := 0; x < fw; x += 2 {
			a, b := row0[x:x+2], row1[x:x+2]
			if sizes.readBitFast() {
				pattern := types.readBitFast()
				v := fg.nextFast()
				if pattern {
					colors := [2]byte{bg.nextFast(), v}
					bits := patterns.readBitsFast(4)
					a[0], a[1] = colors[(bits>>3)&1], colors[(bits>>2)&1]
					b[0], b[1] = colors[(bits>>1)&1], colors[bits&1]
				} else {
					a[0], a[1], b[0], b[1] = v, v, v, v
				}
			} else {
				types.readBitsFast(4)
				a[0], a[1], b[0], b[1] = fg.nextFast(), fg.nextFast(), fg.nextFast(), fg.nextFast()
			}
		}
	}
	for y := 0; y < fh; y++ {
		for x := fw; x < w; x++ {
			plane[y*w+x] = fg.nextFast()
		}
	}
	for y := fh; y < h; y++ {
		for x := 0; x < w; x++ {
			plane[y*w+x] = fg.nextFast()
		}
	}
}

func (d *Decoder) decodeQuadImage(segments [3][]byte, w, h int, lossless bool, storedCorners *cornerGrid) {
	n := w * h
	scratch := [3]*decoderChannelScratch{&d.y, &d.cb, &d.cr}
	var planes [3][]byte
	for i, data := range segments {
		if data == nil {
			continue
		}
		if cap(scratch[i].plane) < n {
			scratch[i].plane = make([]byte, n)
		} else {
			scratch[i].plane = scratch[i].plane[:n]
		}
		planes[i] = scratch[i].plane
	}
	processStreams(3, n, d.Parallel, func(i int) {
		if segments[i] != nil {
			decodeValidatedQuadPlane(segments[i], planes[i], w, h)
		}
	})
	var originalCornerGrid *cornerGrid
	if d.Dither3 && !lossless {
		if storedCorners != nil && !d.GlyphsOnly {
			grid := makeCornerGrid(w, h, storedCorners.step, func(x, y int) [3]byte {
				o := y*w + x
				v := [3]byte{planes[0][o], 128, 128}
				if planes[1] != nil {
					v[1] = planes[1][o]
				}
				if planes[2] != nil {
					v[2] = planes[2][o]
				}
				return v
			})
			originalCornerGrid = &grid
		}
		d.blurLuma(w, h, func(x, y int) byte { return planes[0][y*w+x] }, func(x, y int, v byte) { planes[0][y*w+x] = v })
	}
	if d.GlyphsOnly {
		processStreams(4, n, d.Parallel, func(i int) {
			start, end := n*i/4, n*(i+1)/4
			for j := start; j < end; j++ {
				v, p := planes[0][j], d.dst.Pix[j*4:j*4+4]
				p[0], p[1], p[2], p[3] = v, v, v, 255
			}
		})
		return
	}
	if (d.CornerGradient || storedCorners != nil) && !lossless {
		step := 2
		if storedCorners != nil {
			step = storedCorners.step
		}
		var grid cornerGrid
		if originalCornerGrid != nil {
			grid = *originalCornerGrid
		} else {
			grid = makeCornerGrid(w, h, step, func(x, y int) [3]byte {
				o := y*w + x
				v := [3]byte{planes[0][o], 128, 128}
				if planes[1] != nil {
					v[1] = planes[1][o]
				}
				if planes[2] != nil {
					v[2] = planes[2][o]
				}
				return v
			})
		}
		if storedCorners != nil {
			storedCorners.fillMissing(grid)
			var sizes []byte
			if d.GlyphBlend && 2 >= max(d.GlyphMinSize, 1) {
				sizes = d.glyphBlockSizes(segments[0], w, h)
			}
			d.writeStoredCorners(*storedCorners, grid, func(x, y int) byte { return planes[0][y*w+x] }, sizes, max(d.GlyphMinSize, 1))
		} else {
			d.writeCornerGradient(grid)
		}
		return
	}
	processStreams(4, n, d.Parallel, func(i int) {
		start, end := n*i/4, n*(i+1)/4
		y := planes[0][start:end]
		var cb, cr []byte
		if planes[1] != nil {
			cb = planes[1][start:end]
		}
		if planes[2] != nil {
			cr = planes[2][start:end]
		}
		pix := d.dst.Pix[start*4 : end*4]
		if lossless {
			for j, value := range y {
				r, b := value, value
				if cb != nil {
					b += cb[j] - 128
				}
				if cr != nil {
					r += cr[j] - 128
				}
				p := pix[j*4 : j*4+4]
				p[0], p[1], p[2], p[3] = r, value, b, 255
			}
			return
		}
		for j, value := range y {
			Y, Cb, Cr := int32(value), int32(0), int32(0)
			if cb != nil {
				Cb = int32(cb[j]) - 128
			}
			if cr != nil {
				Cr = int32(cr[j]) - 128
			}
			R := Y + ((91881 * Cr) >> 16)
			G := Y - ((22554*Cb + 46802*Cr) >> 16)
			B := Y + ((116130 * Cb) >> 16)
			p := pix[j*4 : j*4+4]
			p[0], p[1], p[2], p[3] = byte(max(0, min(255, R))), byte(max(0, min(255, G))), byte(max(0, min(255, B))), 255
		}
	})
}
