package classic

import "encoding/binary"

// patternBlockMap reconstructs actual block sizes and marks patterned luma
// blocks. The caller has already validated the channel streams.
func (d *Decoder) patternBlockMap(channel []byte, width, height int) []byte {
	n := width * height
	if cap(d.ditherMap) < n {
		d.ditherMap = make([]byte, n)
	} else {
		d.ditherMap = d.ditherMap[:n]
		clear(d.ditherMap)
	}
	sizeLen := int(binary.BigEndian.Uint32(channel[4:8]))
	sizes := newBitReader(channel[8 : 8+sizeLen])
	off := 8 + sizeLen
	typeLen := int(binary.BigEndian.Uint32(channel[off : off+4]))
	off += 4
	types := newBitReader(channel[off : off+typeLen])
	fill := func(x0, y0, size int) {
		code := byte(size)
		if types.readBitFast() {
			code |= 0x80
		}
		for y := y0; y < y0+size; y++ {
			row := d.ditherMap[y*width+x0 : y*width+x0+size]
			for x := range row {
				row[x] = code
			}
		}
	}
	w, h := width/smallBlock*smallBlock, height/smallBlock*smallBlock
	fullW, fullH := w/macroBlock*macroBlock, h/macroBlock*macroBlock
	for y := 0; y < fullH; y += macroBlock {
		for x := 0; x < fullW; x += macroBlock {
			if sizes.readBitFast() {
				fill(x, y, macroBlock)
			} else {
				for by := 0; by < macroBlock; by += smallBlock {
					for bx := 0; bx < macroBlock; bx += smallBlock {
						fill(x+bx, y+by, smallBlock)
					}
				}
			}
		}
	}
	for y := 0; y < fullH; y += smallBlock {
		for x := fullW; x < w; x += smallBlock {
			fill(x, y, smallBlock)
		}
	}
	for y := fullH; y < h; y += smallBlock {
		for x := 0; x < w; x += smallBlock {
			fill(x, y, smallBlock)
		}
	}
	return d.ditherMap
}

// ditherLuma reads a snapshot of the decoded Y channel and writes only the
// luma output. Within each patterned block it estimates local coverage, then
// selects one of the existing two levels using an ordered 4x4 threshold. Larger
// blocks give more weight to neighbours and therefore a stronger effect.
func ditherCenterWeight(size int) int {
	return max(2, 32/size)
}

func (d *Decoder) ditherLuma(channel []byte, width, height int, get func(int, int) byte, set func(int, int, byte)) {
	if macroBlock < 4 {
		return
	}
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
	blocks := d.patternBlockMap(channel, width, height)
	ordered := [16]int{0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5}
	write := func(y0, y1 int) {
		for y := y0; y < y1; y++ {
			for x := 0; x < width; x++ {
				code := blocks[y*width+x]
				size := int(code & 0x7f)
				if code&0x80 == 0 || size < 4 {
					continue
				}
				x0, y0 := x/size*size, y/size*size
				center := int(d.ditherInput[y*width+x])
				lo, hi := center, center
				centerWeight := ditherCenterWeight(size)
				sum, weight := center*centerWeight, centerWeight
				for _, p := range [4][2]int{{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}} {
					if p[0] < x0 || p[0] >= x0+size || p[1] < y0 || p[1] >= y0+size {
						continue
					}
					v := int(d.ditherInput[p[1]*width+p[0]])
					sum += v
					weight++
					lo = min(lo, v)
					hi = max(hi, v)
				}
				if hi == lo || hi-lo > 48 {
					continue
				}
				level := ordered[(y&3)*4+(x&3)]
				v := lo
				if (sum-lo*weight)*32 >= (hi-lo)*weight*(2*level+1) {
					v = hi
				}
				set(x, y, byte(v))
			}
		}
	}
	if d.Parallel {
		processStreams(4, n, true, func(i int) { write(height*i/4, height*(i+1)/4) })
	} else {
		write(0, height)
	}
}
