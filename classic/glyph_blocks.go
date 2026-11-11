package classic

import "encoding/binary"

// glyphBlockSizes reconstructs the luma block partition from its validated
// size flags. Each pixel contains its actual block size, or zero for an area
// omitted by an old file. The caller validates the channel before this runs.
func (d *Decoder) glyphBlockSizes(channel []byte, width, height int) []byte {
	n := width * height
	if cap(d.glyphBlocks) < n {
		d.glyphBlocks = make([]byte, n)
	} else {
		d.glyphBlocks = d.glyphBlocks[:n]
		clear(d.glyphBlocks)
	}
	length := int(binary.BigEndian.Uint32(channel[4:8]))
	bits := newBitReader(channel[8 : 8+length])
	fill := func(x0, y0, size int) {
		for y := y0; y < y0+size; y++ {
			row := d.glyphBlocks[y*width+x0 : y*width+x0+size]
			for x := range row {
				row[x] = byte(size)
			}
		}
	}
	w, h := width/smallBlock*smallBlock, height/smallBlock*smallBlock
	fullW, fullH := w/macroBlock*macroBlock, h/macroBlock*macroBlock
	for y := 0; y < fullH; y += macroBlock {
		for x := 0; x < fullW; x += macroBlock {
			if bits.readBitFast() {
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
	return d.glyphBlocks
}
