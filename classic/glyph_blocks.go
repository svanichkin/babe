package classic

import (
	"encoding/binary"
	"runtime"
	"sync"
)

// glyphBlockSizes reconstructs the luma block partition from its validated
// size flags. Each pixel contains its actual block size, or zero for an area
// omitted by an old file. The caller validates the channel before this runs.
func (d *Decoder) glyphBlockSizes(channel []byte, width, height int) []byte {
	n := width * height
	if cap(d.glyphBlocks) < n {
		d.glyphBlocks = make([]byte, n)
	} else {
		d.glyphBlocks = d.glyphBlocks[:n]
	}
	fillGlyphBlockSizes(d.glyphBlocks, channel, width, height)
	return d.glyphBlocks
}

// fillGlyphBlockSizes writes the block size of every pixel into dst. Within a
// row of macroblocks every pixel row has the same sizes, so each macro row is
// built once and copied to its remaining rows.
func fillGlyphBlockSizes(dst, channel []byte, width, height int) {
	length := int(binary.BigEndian.Uint32(channel[4:8]))
	bits := newBitReader(channel[8 : 8+length])
	w, h := width/smallBlock*smallBlock, height/smallBlock*smallBlock
	fullW, fullH := w/macroBlock*macroBlock, h/macroBlock*macroBlock
	fillRow := func(row []byte, value byte) {
		for i := range row {
			row[i] = value
		}
	}
	for y := 0; y < fullH; y += macroBlock {
		row := dst[y*width : (y+1)*width]
		for x := 0; x < fullW; x += macroBlock {
			if bits.readBitFast() {
				fillRow(row[x:x+macroBlock], byte(macroBlock))
			} else {
				fillRow(row[x:x+macroBlock], byte(smallBlock))
			}
		}
		fillRow(row[fullW:w], byte(smallBlock))
		clear(row[w:])
		for yy := y + 1; yy < y+macroBlock; yy++ {
			copy(dst[yy*width:(yy+1)*width], row)
		}
	}
	if fullH < h {
		row := dst[fullH*width : (fullH+1)*width]
		fillRow(row[:w], byte(smallBlock))
		clear(row[w:])
		for yy := fullH + 1; yy < h; yy++ {
			copy(dst[yy*width:(yy+1)*width], row)
		}
	}
	clear(dst[h*width:])
}

// markLargeGlyphs sets bit in dst for every pixel of a block larger than 4.
// It is equivalent to testing fillGlyphBlockSizes(...) > 4 for each pixel.
func markLargeGlyphs(dst []byte, bit byte, channel []byte, width, height int) {
	if smallBlock > 4 {
		panic("unsupported small block size")
	}
	if macroBlock <= 4 {
		return
	}
	length := int(binary.BigEndian.Uint32(channel[4:8]))
	bits := newBitReader(channel[8 : 8+length])
	w, h := width/smallBlock*smallBlock, height/smallBlock*smallBlock
	fullW, fullH := w/macroBlock*macroBlock, h/macroBlock*macroBlock
	for y := 0; y < fullH; y += macroBlock {
		for x := 0; x < fullW; x += macroBlock {
			if !bits.readBitFast() {
				continue
			}
			for yy := y; yy < y+macroBlock; yy++ {
				row := dst[yy*width+x : yy*width+x+macroBlock]
				for i := range row {
					row[i] |= bit
				}
			}
		}
	}
}

// parallelRows splits [0,height) into bands for all available CPUs. Each band
// is processed by fn independently. Small images stay on one goroutine.
func parallelRows(parallel bool, width, height int, fn func(y0, y1 int)) {
	workers := 1
	if parallel && width*height >= 64<<10 {
		workers = min(runtime.GOMAXPROCS(0), max(height/16, 1))
	}
	if workers <= 1 {
		fn(0, height)
		return
	}
	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0, y1 := height*i/workers, height*(i+1)/workers
		wg.Add(1)
		go func() {
			defer wg.Done()
			fn(y0, y1)
		}()
	}
	wg.Wait()
}
