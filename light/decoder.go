package light

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"image"
	"io"
	"runtime"
	"sync"
	"sync/atomic"

	"github.com/klauspost/compress/zstd"
)

type Decoder struct {
	// Parallel enables internal goroutines (per-channel decode and RGB conversion).
	// Set to false to reduce goroutine overhead and allocations.
	Parallel bool

	patternCount int
	levels       []int

	chromaGrid   []byte
	chromaTemp   []byte
	chromaXCell  []int
	chromaXFX    []uint16
	chromaXFXInv []uint16
	chromaBands  []decoderChromaBandScratch

	payload []byte
	zdec    *zstd.Decoder

	dst *image.RGBA
}

type decoderChromaBandScratch struct {
	cbTop []uint32
	cbBot []uint32
	crTop []uint32
	crBot []uint32
}

func NewDecoder() *Decoder {
	return &Decoder{Parallel: true, zdec: zstdDecoderPool.Get().(*zstd.Decoder)}
}

func (d *Decoder) Close() {
	if d == nil || d.zdec == nil {
		return
	}
	zstdDecoderPool.Put(d.zdec)
	d.zdec = nil
}

type blockDesc struct {
	x          int
	y          int
	size       int
	fg         uint8
	bg         uint8
	patternIdx uint16
	isPattern  bool
}

func drawBlockIntoPixMask(pix []byte, strideBytes, imgW, imgH int, x0, y0, bw, bh int, pattern patternMask, fg, bg uint8, channelOffset int) error {
	visW, visH := visibleBlockDims(imgW, imgH, x0, y0, bw, bh)
	total := bw * bh
	for yy := 0; yy < visH; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		shift := total - 1 - yy*bw
		word := shift >> 6
		bit := uint(shift & 63)
		wordVal := pattern[word]
		for xx := 0; xx < visW; xx++ {
			val := bg
			if ((wordVal >> bit) & 1) != 0 {
				val = fg
			}
			o := row + xx*4
			if o < 0 || o >= len(pix) {
				return fmt.Errorf("drawBlockIntoPixMask: index out of range")
			}
			pix[o] = val
			if bit == 0 {
				if word > 0 {
					word--
					wordVal = pattern[word]
				}
				bit = 63
			} else {
				bit--
			}
		}
	}
	return nil
}

func drawBlockIntoPixFullMask(pix []byte, strideBytes int, x0, y0, bw, bh int, pattern patternMask, fg, bg uint8, channelOffset int) {
	total := bw * bh
	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		shift := total - 1 - yy*bw
		word := shift >> 6
		bit := uint(shift & 63)
		wordVal := pattern[word]
		for xx := 0; xx < bw; xx++ {
			val := bg
			if ((wordVal >> bit) & 1) != 0 {
				val = fg
			}
			pix[row+xx*4] = val
			if bit == 0 {
				if word > 0 {
					word--
					wordVal = pattern[word]
				}
				bit = 63
			} else {
				bit--
			}
		}
	}
	if activePatternBlur > 0 && channelOffset == 0 {
		imgW := strideBytes / 4
		imgH := len(pix) / strideBytes
		size := bw
		if bh < size {
			size = bh
		}
		radius := (activePatternBlur * size) / 64
		if radius < 1 {
			radius = 1
		}
		blurBlockChannel(pix, strideBytes, imgW, imgH, x0, y0, bw, bh, channelOffset, radius)
	}
}

func decodeChannelToPix(data []byte, imgW, imgH int, levels []int, patternCount int, pix []byte, strideBytes int, channelOffset int, parallel bool) error {
	pos := 0
	if len(levels) == 0 {
		return fmt.Errorf("decodeChannel: missing block levels")
	}

	readU32 := func(label string) (uint32, error) {
		if len(data)-pos < 4 {
			return 0, fmt.Errorf("decodeChannel: truncated while reading %s", label)
		}
		v := binary.BigEndian.Uint32(data[pos : pos+4])
		pos += 4
		return v, nil
	}

	readSlice := func(n uint32, label string) ([]byte, error) {
		if n > uint32(len(data)-pos) {
			return nil, fmt.Errorf("decodeChannel: truncated while reading %s", label)
		}
		end := pos + int(n)
		s := data[pos:end]
		pos = end
		return s, nil
	}

	blockCount, err := readU32("blockCount")
	if err != nil {
		return err
	}
	sizeStreamLen, err := readU32("sizeStreamLen")
	if err != nil {
		return err
	}
	sizeBytes, err := readSlice(sizeStreamLen, "sizeStream")
	if err != nil {
		return err
	}
	sizeBR := newBitReader(sizeBytes)
	patternLen, err := readU32("patternLen")
	if err != nil {
		return err
	}
	patternBytes, err := readSlice(patternLen, "patternStream")
	if err != nil {
		return err
	}
	patternBR := newBitReader(patternBytes)
	fgPackedLen, err := readU32("FG packedLen")
	if err != nil {
		return err
	}
	fgPacked, err := readSlice(fgPackedLen, "FG packed data")
	if err != nil {
		return err
	}
	if blockCount > 0 && fgPackedLen < blockCount {
		return fmt.Errorf("decodeChannel: FG packed data too short")
	}
	fgStream, err := newDeltaStream(fgPacked, int(blockCount))
	if err != nil {
		return err
	}
	bgPackedLen, err := readU32("BG packedLen")
	if err != nil {
		return err
	}
	bgPacked, err := readSlice(bgPackedLen, "BG packed data")
	if err != nil {
		return err
	}
	if bgPackedLen > blockCount {
		return fmt.Errorf("decodeChannel: BG packed data too long")
	}
	bgStream, err := newDeltaStream(bgPacked, int(bgPackedLen))
	if err != nil {
		return err
	}
	bgPerPattern := true

	smallBlock := levels[0]
	topBlock := levels[len(levels)-1]
	w4 := ceilToStep(imgW, smallBlock)
	h4 := ceilToStep(imgH, smallBlock)
	fullW := ceilToStep(imgW, topBlock)
	fullH := ceilToStep(imgH, topBlock)

	blockIndex := 0
	patternBits := patternIndexBitsForCount(patternCount)
	patternLimit := patternCount
	if patternLimit < 1 {
		patternLimit = 1
	}
	useParallel := parallel && blockCount > 2048 && runtime.NumCPU() > 1

	if !useParallel {
		var decodeNode func(x, y, levelIdx int) error
		decodeNode = func(x, y, levelIdx int) error {
			size := levels[levelIdx]
			if levelIdx > 0 {
				useHere, err := sizeBR.readBit()
				if err != nil {
					return err
				}
				if !useHere {
					childSize := levels[levelIdx-1]
					ratio := size / childSize
					for by := 0; by < ratio; by++ {
						for bx := 0; bx < ratio; bx++ {
							cx := x + bx*childSize
							cy := y + by*childSize
							if vw, vh := visibleBlockDims(imgW, imgH, cx, cy, childSize, childSize); vw <= 0 || vh <= 0 {
								continue
							}
							if err := decodeNode(cx, cy, levelIdx-1); err != nil {
								return err
							}
						}
					}
					return nil
				}
			}

			if blockIndex >= int(blockCount) {
				return fmt.Errorf("unexpected end of blocks in main area")
			}
			fg, err := fgStream.next()
			if err != nil {
				return err
			}
			full := x+size <= imgW && y+size <= imgH
			if encodedBlockIsPattern(fg) {
				var bg uint8
				if bgPerPattern {
					bg, err = bgStream.next()
					if err != nil {
						return err
					}
				}
				if full {
					if err := drawBlockIntoPixFull(pix, strideBytes, imgW, x, y, size, size, &patternBR, fg, bg, patternCount, channelOffset); err != nil {
						return err
					}
				} else {
					if err := drawBlockIntoPix(pix, strideBytes, imgW, imgH, x, y, size, size, &patternBR, fg, bg, patternCount, channelOffset); err != nil {
						return err
					}
				}
			} else {
				if full {
					fillBlockIntoPixFull(pix, strideBytes, x, y, size, size, fg, channelOffset)
				} else {
					if err := fillBlockIntoPix(pix, strideBytes, imgW, imgH, x, y, size, size, fg, channelOffset); err != nil {
						return err
					}
				}
			}
			if channelOffset == 0 && activeFilmGrain > 0 {
				applyFilmGrainBlockY(pix, strideBytes, imgW, imgH, x, y, size, activeFilmGrain)
			}
			blockIndex++
			return nil
		}

		for my := 0; my < fullH; my += topBlock {
			for mx := 0; mx < fullW; mx += topBlock {
				if err := decodeNode(mx, my, len(levels)-1); err != nil {
					return err
				}
			}
		}
		for my := 0; my < fullH; my += smallBlock {
			for mx := fullW; mx < w4; mx += smallBlock {
				if blockIndex >= int(blockCount) {
					return fmt.Errorf("unexpected end of blocks in right stripe")
				}
				fg, err := fgStream.next()
				if err != nil {
					return err
				}
				full := mx+smallBlock <= imgW && my+smallBlock <= imgH
				if encodedBlockIsPattern(fg) {
					bg, err := bgStream.next()
					if err != nil {
						return err
					}
					if full {
						if err := drawBlockIntoPixFull(pix, strideBytes, imgW, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, patternCount, channelOffset); err != nil {
							return err
						}
					} else {
						if err := drawBlockIntoPix(pix, strideBytes, imgW, imgH, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, patternCount, channelOffset); err != nil {
							return err
						}
					}
				} else {
					if full {
						fillBlockIntoPixFull(pix, strideBytes, mx, my, smallBlock, smallBlock, fg, channelOffset)
					} else {
						if err := fillBlockIntoPix(pix, strideBytes, imgW, imgH, mx, my, smallBlock, smallBlock, fg, channelOffset); err != nil {
							return err
						}
					}
				}
				if channelOffset == 0 && activeFilmGrain > 0 {
					applyFilmGrainBlockY(pix, strideBytes, imgW, imgH, mx, my, smallBlock, activeFilmGrain)
				}
				blockIndex++
			}
		}
		for my := fullH; my < h4; my += smallBlock {
			for mx := 0; mx < w4; mx += smallBlock {
				if blockIndex >= int(blockCount) {
					return fmt.Errorf("unexpected end of blocks in bottom stripe")
				}
				fg, err := fgStream.next()
				if err != nil {
					return err
				}
				full := mx+smallBlock <= imgW && my+smallBlock <= imgH
				if encodedBlockIsPattern(fg) {
					bg, err := bgStream.next()
					if err != nil {
						return err
					}
					if full {
						if err := drawBlockIntoPixFull(pix, strideBytes, imgW, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, patternCount, channelOffset); err != nil {
							return err
						}
					} else {
						if err := drawBlockIntoPix(pix, strideBytes, imgW, imgH, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, patternCount, channelOffset); err != nil {
							return err
						}
					}
				} else {
					if full {
						fillBlockIntoPixFull(pix, strideBytes, mx, my, smallBlock, smallBlock, fg, channelOffset)
					} else {
						if err := fillBlockIntoPix(pix, strideBytes, imgW, imgH, mx, my, smallBlock, smallBlock, fg, channelOffset); err != nil {
							return err
						}
					}
				}
				if channelOffset == 0 && activeFilmGrain > 0 {
					applyFilmGrainBlockY(pix, strideBytes, imgW, imgH, mx, my, smallBlock, activeFilmGrain)
				}
				blockIndex++
			}
		}

		if blockIndex != int(blockCount) {
			return fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
		}
		return nil
	}

	blocks := make([]blockDesc, 0, blockCount)

	var decodeNodeParallel func(x, y, levelIdx int) error
	decodeNodeParallel = func(x, y, levelIdx int) error {
		size := levels[levelIdx]
		if levelIdx > 0 {
			useHere, err := sizeBR.readBit()
			if err != nil {
				return err
			}
			if !useHere {
				childSize := levels[levelIdx-1]
				ratio := size / childSize
				for by := 0; by < ratio; by++ {
					for bx := 0; bx < ratio; bx++ {
						cx := x + bx*childSize
						cy := y + by*childSize
						if vw, vh := visibleBlockDims(imgW, imgH, cx, cy, childSize, childSize); vw <= 0 || vh <= 0 {
							continue
						}
						if err := decodeNodeParallel(cx, cy, levelIdx-1); err != nil {
							return err
						}
					}
				}
				return nil
			}
		}

		if blockIndex >= int(blockCount) {
			return fmt.Errorf("unexpected end of blocks in main area")
		}
		fg, err := fgStream.next()
		if err != nil {
			return err
		}
		if encodedBlockIsPattern(fg) {
			var bg uint8
			if bgPerPattern {
				bg, err = bgStream.next()
				if err != nil {
					return err
				}
			}
			idx64, err := readPatternIndex(&patternBR, patternBits)
			if err != nil {
				return err
			}
			if idx64 >= uint64(patternLimit) {
				idx64 = uint64(patternLimit - 1)
			}
			blocks = append(blocks, blockDesc{x: x, y: y, size: size, fg: fg, bg: bg, patternIdx: uint16(idx64), isPattern: true})
		} else {
			blocks = append(blocks, blockDesc{x: x, y: y, size: size, fg: fg, isPattern: false})
		}
		blockIndex++
		return nil
	}

	for my := 0; my < fullH; my += topBlock {
		for mx := 0; mx < fullW; mx += topBlock {
			if err := decodeNodeParallel(mx, my, len(levels)-1); err != nil {
				return err
			}
		}
	}
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return fmt.Errorf("unexpected end of blocks in right stripe")
			}
			fg, err := fgStream.next()
			if err != nil {
				return err
			}
			if encodedBlockIsPattern(fg) {
				bg, err := bgStream.next()
				if err != nil {
					return err
				}
				idx64, err := readPatternIndex(&patternBR, patternBits)
				if err != nil {
					return err
				}
				if idx64 >= uint64(patternLimit) {
					idx64 = uint64(patternLimit - 1)
				}
				blocks = append(blocks, blockDesc{x: mx, y: my, size: smallBlock, fg: fg, bg: bg, patternIdx: uint16(idx64), isPattern: true})
			} else {
				blocks = append(blocks, blockDesc{x: mx, y: my, size: smallBlock, fg: fg, isPattern: false})
			}
			blockIndex++
		}
	}
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return fmt.Errorf("unexpected end of blocks in bottom stripe")
			}
			fg, err := fgStream.next()
			if err != nil {
				return err
			}
			if encodedBlockIsPattern(fg) {
				bg, err := bgStream.next()
				if err != nil {
					return err
				}
				idx64, err := readPatternIndex(&patternBR, patternBits)
				if err != nil {
					return err
				}
				if idx64 >= uint64(patternLimit) {
					idx64 = uint64(patternLimit - 1)
				}
				blocks = append(blocks, blockDesc{x: mx, y: my, size: smallBlock, fg: fg, bg: bg, patternIdx: uint16(idx64), isPattern: true})
			} else {
				blocks = append(blocks, blockDesc{x: mx, y: my, size: smallBlock, fg: fg, isPattern: false})
			}
			blockIndex++
		}
	}

	if blockIndex != int(blockCount) {
		return fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
	}
	if len(blocks) == 0 {
		return nil
	}

	booksBySize := make(map[int][]patternMask, len(levels))
	for _, size := range levels {
		booksBySize[size] = fixedPatternCodebook(size, size, patternLimit)
	}

	tileSize := topBlock
	if tileSize <= 0 {
		return nil
	}
	tilesX := ceilToStep(imgW, tileSize) / tileSize
	tilesY := ceilToStep(imgH, tileSize) / tileSize
	totalTiles := tilesX * tilesY
	if totalTiles <= 1 {
		for _, b := range blocks {
			full := b.x+b.size <= imgW && b.y+b.size <= imgH
			if b.isPattern {
				book := booksBySize[b.size]
				idx := int(b.patternIdx)
				if idx < 0 || idx >= len(book) {
					idx = len(book) - 1
				}
				pattern := book[idx]
				if full {
					drawBlockIntoPixFullMask(pix, strideBytes, b.x, b.y, b.size, b.size, pattern, b.fg, b.bg, channelOffset)
				} else {
					if err := drawBlockIntoPixMask(pix, strideBytes, imgW, imgH, b.x, b.y, b.size, b.size, pattern, b.fg, b.bg, channelOffset); err != nil {
						return err
					}
				}
			} else {
				if full {
					fillBlockIntoPixFull(pix, strideBytes, b.x, b.y, b.size, b.size, b.fg, channelOffset)
				} else {
					if err := fillBlockIntoPix(pix, strideBytes, imgW, imgH, b.x, b.y, b.size, b.size, b.fg, channelOffset); err != nil {
						return err
					}
				}
			}
			if channelOffset == 0 && activeFilmGrain > 0 {
				applyFilmGrainBlockY(pix, strideBytes, imgW, imgH, b.x, b.y, b.size, activeFilmGrain)
			}
		}
		return nil
	}

	tileCounts := make([]int, totalTiles)
	for _, b := range blocks {
		tx := b.x / tileSize
		ty := b.y / tileSize
		if tx >= tilesX {
			tx = tilesX - 1
		}
		if ty >= tilesY {
			ty = tilesY - 1
		}
		tileCounts[ty*tilesX+tx]++
	}
	offsets := make([]int, totalTiles+1)
	for i := 0; i < totalTiles; i++ {
		offsets[i+1] = offsets[i] + tileCounts[i]
	}
	ordered := make([]blockDesc, len(blocks))
	cursor := make([]int, totalTiles)
	copy(cursor, offsets[:totalTiles])
	for _, b := range blocks {
		tx := b.x / tileSize
		ty := b.y / tileSize
		if tx >= tilesX {
			tx = tilesX - 1
		}
		if ty >= tilesY {
			ty = tilesY - 1
		}
		idx := ty*tilesX + tx
		pos := cursor[idx]
		ordered[pos] = b
		cursor[idx]++
	}

	workers := min(runtime.NumCPU(), totalTiles)
	var wg sync.WaitGroup
	var drawErr error
	var errMu sync.Mutex
	var nextTile uint32

	for i := 0; i < workers; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for {
				tile := int(atomic.AddUint32(&nextTile, 1)) - 1
				if tile >= totalTiles {
					return
				}
				start := offsets[tile]
				end := offsets[tile+1]
				for j := start; j < end; j++ {
					b := ordered[j]
					full := b.x+b.size <= imgW && b.y+b.size <= imgH
					if b.isPattern {
						book := booksBySize[b.size]
						idx := int(b.patternIdx)
						if idx < 0 || idx >= len(book) {
							idx = len(book) - 1
						}
						pattern := book[idx]
						if full {
							drawBlockIntoPixFullMask(pix, strideBytes, b.x, b.y, b.size, b.size, pattern, b.fg, b.bg, channelOffset)
						} else {
							if err := drawBlockIntoPixMask(pix, strideBytes, imgW, imgH, b.x, b.y, b.size, b.size, pattern, b.fg, b.bg, channelOffset); err != nil {
								errMu.Lock()
								if drawErr == nil {
									drawErr = err
								}
								errMu.Unlock()
								return
							}
						}
					} else {
						if full {
							fillBlockIntoPixFull(pix, strideBytes, b.x, b.y, b.size, b.size, b.fg, channelOffset)
						} else {
							if err := fillBlockIntoPix(pix, strideBytes, imgW, imgH, b.x, b.y, b.size, b.size, b.fg, channelOffset); err != nil {
								errMu.Lock()
								if drawErr == nil {
									drawErr = err
								}
								errMu.Unlock()
								return
							}
						}
					}
					if channelOffset == 0 && activeFilmGrain > 0 {
						applyFilmGrainBlockY(pix, strideBytes, imgW, imgH, b.x, b.y, b.size, activeFilmGrain)
					}
				}
			}
		}()
	}
	wg.Wait()
	if drawErr != nil {
		return drawErr
	}
	return nil
}

func (d *Decoder) decodeRGBA(compData []byte) (*image.RGBA, error) {
	if d.zdec == nil {
		d.zdec = mustNewZstdDecoder()
	}
	payload, err := d.zdec.DecodeAll(compData, d.payload[:0])
	if err != nil {
		return nil, fmt.Errorf("zstd decode: %w", err)
	}
	d.payload = payload

	if len(payload) < len(codec) {
		return nil, fmt.Errorf("read header: short magic")
	}
	pos := 0

	// magic (only BABE-L is supported)
	switch {
	case len(payload)-pos >= len(codec) && string(payload[pos:pos+len(codec)]) == codec:
		pos += len(codec)
	default:
		end := pos + len(codec)
		if end > len(payload) {
			end = len(payload)
		}
		return nil, fmt.Errorf("bad magic: %q", string(payload[pos:end]))
	}

	readU16 := func(label string) (uint16, error) {
		if len(payload)-pos < 2 {
			return 0, fmt.Errorf("decode: truncated while reading %s", label)
		}
		v := binary.BigEndian.Uint16(payload[pos : pos+2])
		pos += 2
		return v, nil
	}
	readU32 := func(label string) (uint32, error) {
		if len(payload)-pos < 4 {
			return 0, fmt.Errorf("decode: truncated while reading %s", label)
		}
		v := binary.BigEndian.Uint32(payload[pos : pos+4])
		pos += 4
		return v, nil
	}

	patternCount, err := readU16("pattern count")
	if err != nil {
		return nil, err
	}
	levelCount, err := readU16("block level count")
	if err != nil {
		return nil, err
	}
	if levelCount == 0 {
		return nil, fmt.Errorf("invalid block level count: %d", levelCount)
	}
	levels := make([]int, 0, int(levelCount))
	for i := 0; i < int(levelCount); i++ {
		level, err := readU16("block level")
		if err != nil {
			return nil, err
		}
		levels = append(levels, int(level))
	}
	levels = normalizeLevels(levels)
	if patternCount == 0 {
		return nil, fmt.Errorf("invalid pattern count in header: %d", patternCount)
	}
	d.patternCount = int(patternCount)
	d.levels = levels

	// channels mask: which Y/Cb/Cr planes are stored.
	if len(payload)-pos < 1 {
		return nil, fmt.Errorf("decode: truncated while reading channels mask")
	}
	channelsMask := payload[pos]
	pos++
	if channelsMask&channelFlagY == 0 {
		return nil, fmt.Errorf("decode: Y channel missing in header")
	}
	hasChromaGrid := (channelsMask & channelFlagChromaGrid) != 0

	imgW32, err := readU32("image width")
	if err != nil {
		return nil, err
	}
	imgH32, err := readU32("image height")
	if err != nil {
		return nil, err
	}
	imgW := int(imgW32)
	imgH := int(imgH32)

	if len(levels) == 0 {
		return nil, fmt.Errorf("missing block levels in header")
	}
	for i := 0; i < len(levels); i++ {
		if levels[i] <= 0 {
			return nil, fmt.Errorf("invalid block level %d at index %d", levels[i], i)
		}
		if i > 0 {
			if levels[i] <= levels[i-1] || levels[i]%levels[i-1] != 0 {
				return nil, fmt.Errorf("invalid block hierarchy %v", levels)
			}
		}
	}

	if d.dst == nil || d.dst.Bounds().Dx() != imgW || d.dst.Bounds().Dy() != imgH {
		d.dst = image.NewRGBA(image.Rect(0, 0, imgW, imgH))
	}
	dst := d.dst
	pix := dst.Pix
	stride := dst.Stride

	ySeg, err := readChannelSegment(payload, &pos)
	if err != nil {
		return nil, err
	}
	hasCb := (channelsMask & channelFlagCb) != 0
	hasCr := (channelsMask & channelFlagCr) != 0

	var cbSeg, crSeg []byte
	if hasChromaGrid {
		hasCb = true
		hasCr = true
	} else if hasCb {
		cbSeg, err = readChannelSegment(payload, &pos)
		if err != nil {
			return nil, err
		}
	}
	if !hasChromaGrid && hasCr {
		crSeg, err = readChannelSegment(payload, &pos)
		if err != nil {
			return nil, err
		}
	}

	smallBlock := levels[0]
	w4 := ceilToStep(imgW, smallBlock)
	h4 := ceilToStep(imgH, smallBlock)
	if w4 != imgW || h4 != imgH {
		for o := 0; o+3 < len(pix); o += 4 {
			pix[o+0] = 0
			pix[o+1] = 128
			pix[o+2] = 128
			pix[o+3] = 255
		}
	}

	var errY, errCb, errCr error
	rgbReady := false
	parallelDraw := d.Parallel && !hasCb && !hasCr
	if hasChromaGrid {
		errY = decodeChannelToPix(ySeg, imgW, imgH, levels, d.patternCount, pix, stride, 0, parallelDraw)
		if errY != nil {
			return nil, errY
		}
		cbPlane, crPlane, nextPos, err := d.decodeChromaGridOverlay(payload, pos, imgW, imgH)
		if err != nil {
			return nil, err
		}
		pos = nextPos
		blitPlaneToPix(cbPlane, pix, stride, imgW, imgH, 1)
		blitPlaneToPix(crPlane, pix, stride, imgW, imgH, 2)
	} else if d.Parallel && imgW*imgH >= 256*256 {
		var wg sync.WaitGroup
		wg.Add(1)
		go decodeChannelToPixWorker(ySeg, imgW, imgH, levels, d.patternCount, pix, stride, 0, parallelDraw, &errY, &wg)
		if hasCb {
			wg.Add(1)
			go decodeChannelToPixWorker(cbSeg, imgW, imgH, levels, d.patternCount, pix, stride, 1, parallelDraw, &errCb, &wg)
		}
		if hasCr {
			wg.Add(1)
			go decodeChannelToPixWorker(crSeg, imgW, imgH, levels, d.patternCount, pix, stride, 2, parallelDraw, &errCr, &wg)
		}
		wg.Wait()
	} else {
		errY = decodeChannelToPix(ySeg, imgW, imgH, levels, d.patternCount, pix, stride, 0, parallelDraw)
		if hasCb {
			errCb = decodeChannelToPix(cbSeg, imgW, imgH, levels, d.patternCount, pix, stride, 1, parallelDraw)
		}
		if hasCr {
			errCr = decodeChannelToPix(crSeg, imgW, imgH, levels, d.patternCount, pix, stride, 2, parallelDraw)
		}
	}

	if errY != nil {
		return nil, errY
	}
	if errCb != nil {
		return nil, errCb
	}
	if errCr != nil {
		return nil, errCr
	}

	if !rgbReady {
		parallelRGB := d.Parallel && imgW*imgH >= 512*512
		if parallelRGB {
			workers := runtime.GOMAXPROCS(0)
			if workers < 1 {
				workers = 1
			}
			maxWorkers := max(imgH/128, 1)
			if workers > maxWorkers {
				workers = maxWorkers
			}
			rowsPerWorker := (imgH + workers - 1) / workers

			var wgRGB sync.WaitGroup
			for i := 0; i < workers; i++ {
				y0 := i * rowsPerWorker
				if y0 >= imgH {
					break
				}
				y1 := min(y0+rowsPerWorker, imgH)

				wgRGB.Add(1)
				go ycbcrToRGBStripe(pix, stride, imgW, y0, y1, hasCb, hasCr, &wgRGB)
			}
			wgRGB.Wait()
		} else {
			ycbcrToRGB(pix, stride, imgW, 0, imgH, hasCb, hasCr)
		}
	}

	return dst, nil
}

func (d *Decoder) Decode(compData []byte, postfilter ...bool) (*image.RGBA, error) {
	enabled := true
	if len(postfilter) > 0 {
		enabled = postfilter[0]
	}
	if enabled {
		return d.decodeRGBA(compData)
	}

	prevFilmGrain := activeFilmGrain
	prevPatternBlur := activePatternBlur
	activeFilmGrain = 0
	activePatternBlur = 0
	defer func() {
		activeFilmGrain = prevFilmGrain
		activePatternBlur = prevPatternBlur
	}()
	return d.decodeRGBA(compData)
}

func Decode(compData []byte, postfilter bool) (image.Image, error) {
	d := NewDecoder()
	defer d.Close()
	return d.Decode(compData, postfilter)
}

func DecodeLayers(compData []byte) (int, int, []uint8, []image.Rectangle, []uint8, []image.Rectangle, []uint8, []image.Rectangle, bool, bool, error) {
	zdec := mustNewZstdDecoder()
	payload, err := zdec.DecodeAll(compData, nil)
	if err != nil {
		return 0, 0, nil, nil, nil, nil, nil, nil, false, false, fmt.Errorf("zstd decode: %w", err)
	}

	pos := 0
	switch {
	case len(payload)-pos >= len(codec) && string(payload[pos:pos+len(codec)]) == codec:
		pos += len(codec)
	default:
		return 0, 0, nil, nil, nil, nil, nil, nil, false, false, fmt.Errorf("bad magic")
	}

	readU16 := func() uint16 {
		v := binary.BigEndian.Uint16(payload[pos : pos+2])
		pos += 2
		return v
	}
	readU32 := func() uint32 {
		v := binary.BigEndian.Uint32(payload[pos : pos+4])
		pos += 4
		return v
	}

	patternCount := int(readU16())
	levelCount := int(readU16())
	levels := make([]int, 0, levelCount)
	for i := 0; i < levelCount; i++ {
		levels = append(levels, int(readU16()))
	}
	levels = normalizeLevels(levels)
	channelsMask := payload[pos]
	pos++
	imgW := int(readU32())
	imgH := int(readU32())
	hasChromaGrid := (channelsMask & channelFlagChromaGrid) != 0

	ySeg, err := readChannelSegment(payload, &pos)
	if err != nil {
		return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
	}
	yPlane, yRects, err := decodeChannelWithRects(ySeg, imgW, imgH, levels, patternCount)
	if err != nil {
		return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
	}

	hasCb := (channelsMask & channelFlagCb) != 0
	hasCr := (channelsMask & channelFlagCr) != 0
	var cbPlane, crPlane []uint8
	var cbRects, crRects []image.Rectangle
	if hasChromaGrid {
		var scratch Decoder
		cbPlane, crPlane, pos, err = scratch.decodeChromaGridOverlay(payload, pos, imgW, imgH)
		if err != nil {
			return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
		}
		hasCb = true
		hasCr = true
	} else if hasCb {
		cbSeg, err := readChannelSegment(payload, &pos)
		if err != nil {
			return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
		}
		cbPlane, cbRects, err = decodeChannelWithRects(cbSeg, imgW, imgH, levels, patternCount)
		if err != nil {
			return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
		}
	}
	if !hasChromaGrid && hasCr {
		crSeg, err := readChannelSegment(payload, &pos)
		if err != nil {
			return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
		}
		crPlane, crRects, err = decodeChannelWithRects(crSeg, imgW, imgH, levels, patternCount)
		if err != nil {
			return 0, 0, nil, nil, nil, nil, nil, nil, false, false, err
		}
	}

	return imgW, imgH, yPlane, yRects, cbPlane, cbRects, crPlane, crRects, hasCb, hasCr, nil
}

// DecodeFrom reads compressed data from r and decodes it.
// This mirrors codecs that accept io.Reader/io.Writer and is useful for
// benchmarking. It allocates to read the full input; for zero-copy decoding,
// prefer Decode([]byte,...).
func decodeChannelToPixWorker(data []byte, imgW, imgH int, levels []int, patternCount int, pix []byte, strideBytes int, channelOffset int, parallel bool, dstErr *error, wg *sync.WaitGroup) {
	defer wg.Done()
	*dstErr = decodeChannelToPix(data, imgW, imgH, levels, patternCount, pix, strideBytes, channelOffset, parallel)
}

func blitPlaneToPix(plane, pix []byte, strideBytes, imgW, imgH, channelOffset int) {
	for y := 0; y < imgH; y++ {
		rowOff := y * strideBytes
		planeOff := y * imgW
		for x := 0; x < imgW; x++ {
			pix[rowOff+x*4+channelOffset] = plane[planeOff+x]
		}
	}
}

func ycbcrToRGB(pix []byte, stride, imgW int, yStart, yEnd int, hasCb, hasCr bool) {
	initYCbCrTables()
	if hasCb && hasCr {
		for y := yStart; y < yEnd; y++ {
			rowOff := y * stride
			for x := 0; x < imgW; x++ {
				o := rowOff + x*4
				Y := int(pix[o+0])
				Cb := pix[o+1]
				Cr := pix[o+2]

				R := Y + int(crToR[Cr])
				G := Y - int(cbToG[Cb]) - int(crToG[Cr])
				B := Y + int(cbToB[Cb])

				if R < 0 {
					R = 0
				} else if R > 255 {
					R = 255
				}
				if G < 0 {
					G = 0
				} else if G > 255 {
					G = 255
				}
				if B < 0 {
					B = 0
				} else if B > 255 {
					B = 255
				}

				pix[o+0] = uint8(R)
				pix[o+1] = uint8(G)
				pix[o+2] = uint8(B)
				pix[o+3] = 255
			}
		}
		return
	}

	if hasCb {
		for y := yStart; y < yEnd; y++ {
			rowOff := y * stride
			for x := 0; x < imgW; x++ {
				o := rowOff + x*4
				Y := int(pix[o+0])
				Cb := pix[o+1]

				R := Y
				G := Y - int(cbToG[Cb])
				B := Y + int(cbToB[Cb])

				if R < 0 {
					R = 0
				} else if R > 255 {
					R = 255
				}
				if G < 0 {
					G = 0
				} else if G > 255 {
					G = 255
				}
				if B < 0 {
					B = 0
				} else if B > 255 {
					B = 255
				}

				pix[o+0] = uint8(R)
				pix[o+1] = uint8(G)
				pix[o+2] = uint8(B)
				pix[o+3] = 255
			}
		}
		return
	}

	if hasCr {
		for y := yStart; y < yEnd; y++ {
			rowOff := y * stride
			for x := 0; x < imgW; x++ {
				o := rowOff + x*4
				Y := int(pix[o+0])
				Cr := pix[o+2]

				R := Y + int(crToR[Cr])
				G := Y - int(crToG[Cr])
				B := Y

				if R < 0 {
					R = 0
				} else if R > 255 {
					R = 255
				}
				if G < 0 {
					G = 0
				} else if G > 255 {
					G = 255
				}
				if B < 0 {
					B = 0
				} else if B > 255 {
					B = 255
				}

				pix[o+0] = uint8(R)
				pix[o+1] = uint8(G)
				pix[o+2] = uint8(B)
				pix[o+3] = 255
			}
		}
		return
	}

	// Y only.
	for y := yStart; y < yEnd; y++ {
		rowOff := y * stride
		for x := 0; x < imgW; x++ {
			o := rowOff + x*4
			Y := pix[o+0]
			pix[o+0] = Y
			pix[o+1] = Y
			pix[o+2] = Y
			pix[o+3] = 255
		}
	}
}

func ycbcrToRGBStripe(pix []byte, stride, imgW int, yStart, yEnd int, hasCb, hasCr bool, wg *sync.WaitGroup) {
	defer wg.Done()
	ycbcrToRGB(pix, stride, imgW, yStart, yEnd, hasCb, hasCr)
}

// Decode reads a BABE-compressed stream (Zstd + three independent Y/Cb/Cr streams)
// and returns an image.Image.
// smoothJunctions performs gradient-based smoothing at intersections of small blocks.
// At each internal junction of the smallBlock grid, it inspects the four corner pixels
// and, if they are luminance-wise similar, fills a small rectangle around the junction
// with a bilinear gradient between the four corner colors.
type yuv struct {
	Y  int16
	Cb int16
	Cr int16
}

var (
	ycbcrOnce sync.Once
	cbToG     [256]int16
	cbToB     [256]int16
	crToR     [256]int16
	crToG     [256]int16
	yFromR    [256]int32
	yFromG    [256]int32
	yFromB    [256]int32
	cbFromR   [256]int32
	cbFromG   [256]int32
	cbFromB   [256]int32
	crFromR   [256]int32
	crFromG   [256]int32
	crFromB   [256]int32
)

func initYCbCrTables() {
	ycbcrOnce.Do(func() {
		for i := 0; i < 256; i++ {
			v := int32(i)
			cb := int32(i) - 128
			cr := int32(i) - 128
			cbToG[i] = int16((22554 * cb) >> 16)
			cbToB[i] = int16((116130 * cb) >> 16)
			crToR[i] = int16((91881 * cr) >> 16)
			crToG[i] = int16((46802 * cr) >> 16)
			yFromR[i] = 77 * v
			yFromG[i] = 150 * v
			yFromB[i] = 29 * v
			cbFromR[i] = -43 * v
			cbFromG[i] = -85 * v
			cbFromB[i] = 128 * v
			crFromR[i] = 128 * v
			crFromG[i] = -107 * v
			crFromB[i] = -21 * v
		}
	})
}

func rgbToYCbCr(r, g, b uint8) yuv {
	initYCbCrTables()
	Y := (yFromR[r] + yFromG[g] + yFromB[b]) >> 8
	Cb := ((cbFromR[r] + cbFromG[g] + cbFromB[b]) >> 8) + 128
	Cr := ((crFromR[r] + crFromG[g] + crFromB[b]) >> 8) + 128
	return yuv{int16(Y), int16(Cb), int16(Cr)}
}

// delta‑coding of an 8‑bit value on a 0..255 ring into int8 [-128..127]
func encodeDelta8(prev, curr uint8) int8 {
	diff := int16(curr) - int16(prev)
	if diff < -128 {
		diff += 256
	} else if diff > 127 {
		diff -= 256
	}
	return int8(diff)
}

func decodeDelta8(prev uint8, d int8) uint8 {
	return uint8(int16(prev) + int16(d))
}

// --- ZSTD helpers ---

func mustNewZstdEncoder() *zstd.Encoder {
	enc, err := zstd.NewWriter(
		nil,
		zstd.WithEncoderConcurrency(1),
		zstd.WithEncoderLevel(zstd.SpeedBetterCompression),
		zstd.WithLowerEncoderMem(true),
	)
	if err != nil {
		panic(err)
	}
	return enc
}

func mustNewZstdDecoder() *zstd.Decoder {
	dec, err := zstd.NewReader(
		nil,
		zstd.WithDecoderConcurrency(1),
		zstd.WithDecoderLowmem(true),
	)
	if err != nil {
		panic(err)
	}
	return dec
}

// bitsNeeded returns how many bits are required to represent v (0..255).
func bitsNeeded(v int) int {
	if v <= 0 {
		return 1
	}
	bits := 0
	for v > 0 {
		bits++
		v >>= 1
	}
	if bits > 8 {
		return 8
	}
	return bits
}

// writeDeltaPackedBytes encodes src using the same layout as deltaPackBytes,
// but streams directly into w to avoid allocating an intermediate slice.
func writeDeltaPackedBytes(w *bufio.Writer, src []uint8, scratch []byte) ([]byte, error) {
	n := len(src)
	if n == 0 {
		return scratch, nil
	}
	if err := w.WriteByte(src[0]); err != nil {
		return scratch, err
	}

	prev := src[0]
	i := 1

	for i < n {
		if len(scratch) == 0 {
			scratch = make([]byte, 4096)
		}
		j := 0
		for j < len(scratch) && i < n {
			d := encodeDelta8(prev, src[i])
			scratch[j] = byte(d)
			prev = src[i]
			i++
			j++
		}
		if _, err := w.Write(scratch[:j]); err != nil {
			return scratch, err
		}
	}
	return scratch, nil
}

func writeU16BE(w *bufio.Writer, v uint16) error {
	var buf [2]byte
	binary.BigEndian.PutUint16(buf[:], v)
	_, err := w.Write(buf[:])
	return err
}

func writeU32BE(w *bufio.Writer, v uint32) error {
	var buf [4]byte
	binary.BigEndian.PutUint32(buf[:], v)
	_, err := w.Write(buf[:])
	return err
}

// Global scale/bits for chroma-related packing (e.g. Cb/Cr).
// packBits must be enough to represent the range [0 .. 255/packScale].
const packScale = 1
const packBits = 8 // bitsNeeded(255 / packScale) for current scale

func init() {
	// Safety check: ensure packBits is consistent with packScale.
	expected := bitsNeeded(255 / packScale)
	if expected != packBits {
		panic(fmt.Sprintf("packBits (%d) mismatch bitsNeeded(255/packScale)=%d", packBits, expected))
	}
}

// deltaStream provides on-the-fly decoding of a delta-packed byte slice
// produced by deltaPackBytes, without allocating a separate output buffer.
type deltaStream struct {
	packed []byte
	n      int // total number of decoded values expected
	i      int // number of values already returned
	prev   byte
}

func newDeltaStream(packed []byte, n int) (deltaStream, error) {
	if n == 0 {
		return deltaStream{}, nil
	}
	if len(packed) < n {
		return deltaStream{}, fmt.Errorf("delta stream truncated: have %d, need %d", len(packed), n)
	}
	return deltaStream{
		packed: packed,
		n:      n,
		i:      0,
		prev:   packed[0],
	}, nil
}

func (ds *deltaStream) next() (byte, error) {
	if ds.n == 0 {
		return 0, io.EOF
	}
	if ds.i >= ds.n {
		return 0, io.EOF
	}
	if ds.i == 0 {
		ds.i = 1
		return ds.prev, nil
	}
	d := int8(ds.packed[ds.i])
	v := decodeDelta8(ds.prev, d)
	ds.prev = v
	ds.i++
	return v, nil
}
