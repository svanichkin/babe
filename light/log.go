package light

import (
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"os"
	"sort"
)

type usedPattern struct {
	size int
	bits patternMask
}

func collectChannelPatterns(data []byte, imgW, imgH int, levels []int, patternCount int) ([]usedPattern, error) {
	pos := 0
	if len(levels) == 0 {
		return nil, fmt.Errorf("decodeChannel: missing block levels")
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
		return nil, err
	}
	sizeStreamLen, err := readU32("sizeStreamLen")
	if err != nil {
		return nil, err
	}
	sizeBytes, err := readSlice(sizeStreamLen, "sizeStream")
	if err != nil {
		return nil, err
	}
	sizeBR := newBitReader(sizeBytes)

	patternLen, err := readU32("patternLen")
	if err != nil {
		return nil, err
	}
	patternBytes, err := readSlice(patternLen, "patternStream")
	if err != nil {
		return nil, err
	}
	patternBR := newBitReader(patternBytes)

	fgPackedLen, err := readU32("FG packedLen")
	if err != nil {
		return nil, err
	}
	fgPacked, err := readSlice(fgPackedLen, "FG packed data")
	if err != nil {
		return nil, err
	}
	if blockCount > 0 && fgPackedLen < blockCount {
		return nil, fmt.Errorf("decodeChannel: FG packed data too short")
	}
	fgStream, err := newDeltaStream(fgPacked, int(blockCount))
	if err != nil {
		return nil, err
	}

	bgPackedLen, err := readU32("BG packedLen")
	if err != nil {
		return nil, err
	}
	bgPacked, err := readSlice(bgPackedLen, "BG packed data")
	if err != nil {
		return nil, err
	}
	if bgPackedLen > blockCount {
		return nil, fmt.Errorf("decodeChannel: BG packed data too long")
	}
	bgStream, err := newDeltaStream(bgPacked, int(bgPackedLen))
	if err != nil {
		return nil, err
	}

	smallBlock := levels[0]
	topBlock := levels[len(levels)-1]
	w4 := ceilToStep(imgW, smallBlock)
	h4 := ceilToStep(imgH, smallBlock)
	fullW := ceilToStep(imgW, topBlock)
	fullH := ceilToStep(imgH, topBlock)

	patternBits := patternIndexBitsForCount(patternCount)
	patternLimit := patternCount
	if patternLimit < 1 {
		patternLimit = 1
	}
	booksBySize := make(map[int][]patternMask, len(levels))
	for _, size := range levels {
		booksBySize[size] = fixedPatternCodebook(size, size, patternLimit)
	}

	blockIndex := 0
	var out []usedPattern
	seen := make(map[string]struct{})

	addPattern := func(size int, idx64 uint64) {
		book := booksBySize[size]
		idx := int(idx64)
		if idx < 0 || idx >= len(book) {
			idx = len(book) - 1
		}
		pat := book[idx]
		key := fmt.Sprintf("%d:%s", size, patternMaskKey(pat))
		if _, ok := seen[key]; ok {
			return
		}
		seen[key] = struct{}{}
		out = append(out, usedPattern{size: size, bits: clonePatternMask(pat)})
	}

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
		if encodedBlockIsPattern(fg) {
			if _, err := bgStream.next(); err != nil {
				return err
			}
			idx64, err := readPatternIndex(&patternBR, patternBits)
			if err != nil {
				return err
			}
			if idx64 >= uint64(patternLimit) {
				idx64 = uint64(patternLimit - 1)
			}
			addPattern(size, idx64)
		}
		blockIndex++
		return nil
	}

	for my := 0; my < fullH; my += topBlock {
		for mx := 0; mx < fullW; mx += topBlock {
			if err := decodeNode(mx, my, len(levels)-1); err != nil {
				return nil, err
			}
		}
	}
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return nil, fmt.Errorf("unexpected end of blocks in right stripe")
			}
			fg, err := fgStream.next()
			if err != nil {
				return nil, err
			}
			if encodedBlockIsPattern(fg) {
				if _, err := bgStream.next(); err != nil {
					return nil, err
				}
				idx64, err := readPatternIndex(&patternBR, patternBits)
				if err != nil {
					return nil, err
				}
				if idx64 >= uint64(patternLimit) {
					idx64 = uint64(patternLimit - 1)
				}
				addPattern(smallBlock, idx64)
			}
			blockIndex++
		}
	}
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return nil, fmt.Errorf("unexpected end of blocks in bottom stripe")
			}
			fg, err := fgStream.next()
			if err != nil {
				return nil, err
			}
			if encodedBlockIsPattern(fg) {
				if _, err := bgStream.next(); err != nil {
					return nil, err
				}
				idx64, err := readPatternIndex(&patternBR, patternBits)
				if err != nil {
					return nil, err
				}
				if idx64 >= uint64(patternLimit) {
					idx64 = uint64(patternLimit - 1)
				}
				addPattern(smallBlock, idx64)
			}
			blockIndex++
		}
	}

	if blockIndex != int(blockCount) {
		return nil, fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
	}
	return out, nil
}

func renderUsedPatternsPNG(patterns []usedPattern, outPath string) error {
	if len(patterns) == 0 {
		img := image.NewRGBA(image.Rect(0, 0, 2, 2))
		green := color.RGBA{0, 255, 0, 255}
		for i := range img.Pix {
			img.Pix[i] = 0
		}
		for y := 0; y < 2; y++ {
			for x := 0; x < 2; x++ {
				img.SetRGBA(x, y, green)
			}
		}
		f, err := os.Create(outPath)
		if err != nil {
			return err
		}
		defer f.Close()
		return png.Encode(f, img)
	}

	grouped := make(map[int][]patternMask)
	var sizes []int
	for _, p := range patterns {
		if _, ok := grouped[p.size]; !ok {
			sizes = append(sizes, p.size)
		}
		grouped[p.size] = append(grouped[p.size], p.bits)
	}
	sort.Ints(sizes)

	width := 1
	height := 1
	for _, size := range sizes {
		rowW := 1 + len(grouped[size])*(size+1)
		if rowW > width {
			width = rowW
		}
		height += size + 1
	}

	img := image.NewRGBA(image.Rect(0, 0, width, height))
	green := color.RGBA{0, 255, 0, 255}
	black := color.RGBA{0, 0, 0, 255}
	white := color.RGBA{255, 255, 255, 255}
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			img.SetRGBA(x, y, green)
		}
	}

	y0 := 1
	for _, size := range sizes {
		x0 := 1
		for _, pat := range grouped[size] {
			for yy := 0; yy < size; yy++ {
				for xx := 0; xx < size; xx++ {
					c := white
					if testPatternBit(pat, size, size, xx, yy) {
						c = black
					}
					img.SetRGBA(x0+xx, y0+yy, c)
				}
			}
			x0 += size + 1
		}
		y0 += size + 1
	}

	f, err := os.Create(outPath)
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, img)
}

func renderPatternGroupsPNG(grouped map[int][]patternMask, outPath string) error {
	if len(grouped) == 0 {
		img := image.NewRGBA(image.Rect(0, 0, 2, 2))
		green := color.RGBA{0, 255, 0, 255}
		for y := 0; y < 2; y++ {
			for x := 0; x < 2; x++ {
				img.SetRGBA(x, y, green)
			}
		}
		f, err := os.Create(outPath)
		if err != nil {
			return err
		}
		defer f.Close()
		return png.Encode(f, img)
	}

	var sizes []int
	for size := range grouped {
		sizes = append(sizes, size)
	}
	sort.Ints(sizes)

	width := 1
	height := 1
	for _, size := range sizes {
		rowW := 1 + len(grouped[size])*(size+1)
		if rowW > width {
			width = rowW
		}
		height += size + 1
	}

	img := image.NewRGBA(image.Rect(0, 0, width, height))
	green := color.RGBA{0, 255, 0, 255}
	black := color.RGBA{0, 0, 0, 255}
	white := color.RGBA{255, 255, 255, 255}
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			img.SetRGBA(x, y, green)
		}
	}

	y0 := 1
	for _, size := range sizes {
		x0 := 1
		for _, pat := range grouped[size] {
			for yy := 0; yy < size; yy++ {
				for xx := 0; xx < size; xx++ {
					c := black
					if testPatternBit(pat, size, size, xx, yy) {
						c = white
					}
					img.SetRGBA(x0+xx, y0+yy, c)
				}
			}
			x0 += size + 1
		}
		y0 += size + 1
	}

	f, err := os.Create(outPath)
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, img)
}

func WritePatternsPNG(compData []byte, outPath string) error {
	zdec := mustNewZstdDecoder()
	defer zstdDecoderPool.Put(zdec)
	payload, err := zdec.DecodeAll(compData, nil)
	if err != nil {
		return fmt.Errorf("zstd decode: %w", err)
	}

	pos := 0
	switch {
	case len(payload)-pos >= len(codec) && string(payload[pos:pos+len(codec)]) == codec:
		pos += len(codec)
	default:
		return fmt.Errorf("bad magic")
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
		return err
	}
	patterns, err := collectChannelPatterns(ySeg, imgW, imgH, levels, patternCount)
	if err != nil {
		return err
	}

	hasCb := (channelsMask & channelFlagCb) != 0
	hasCr := (channelsMask & channelFlagCr) != 0
	if !hasChromaGrid && hasCb {
		cbSeg, err := readChannelSegment(payload, &pos)
		if err != nil {
			return err
		}
		cbPatterns, err := collectChannelPatterns(cbSeg, imgW, imgH, levels, patternCount)
		if err != nil {
			return err
		}
		patterns = append(patterns, cbPatterns...)
	}
	if !hasChromaGrid && hasCr {
		crSeg, err := readChannelSegment(payload, &pos)
		if err != nil {
			return err
		}
		crPatterns, err := collectChannelPatterns(crSeg, imgW, imgH, levels, patternCount)
		if err != nil {
			return err
		}
		patterns = append(patterns, crPatterns...)
	}

	return renderUsedPatternsPNG(patterns, outPath)
}

func WriteCodebookPNG(compData []byte, outPath string) error {
	zdec := mustNewZstdDecoder()
	defer zstdDecoderPool.Put(zdec)
	payload, err := zdec.DecodeAll(compData, nil)
	if err != nil {
		return fmt.Errorf("zstd decode: %w", err)
	}

	pos := 0
	switch {
	case len(payload)-pos >= len(codec) && string(payload[pos:pos+len(codec)]) == codec:
		pos += len(codec)
	default:
		return fmt.Errorf("bad magic")
	}

	readU16 := func() uint16 {
		v := binary.BigEndian.Uint16(payload[pos : pos+2])
		pos += 2
		return v
	}

	patternCount := int(readU16())
	levelCount := int(readU16())
	levels := make([]int, 0, levelCount)
	for i := 0; i < levelCount; i++ {
		levels = append(levels, int(readU16()))
	}
	levels = normalizeLevels(levels)

	grouped := make(map[int][]patternMask, len(levels))
	for _, size := range levels {
		grouped[size] = fixedPatternCodebook(size, size, patternCount)
	}
	return renderPatternGroupsPNG(grouped, outPath)
}
