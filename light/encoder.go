package light

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"io"
	"runtime"
	"sync"

	"github.com/klauspost/compress/zstd"
)

type Encoder struct {
	// Parallel enables internal goroutines (plane extraction and per-channel encode).
	// Set to false to reduce goroutine overhead and allocations.
	Parallel bool

	patternCount   int
	levels         []int
	quality        int
	backgroundTile int
	yQuantShift    int
	bwmode         bool

	yPlane  []uint8
	cbPlane []uint8
	crPlane []uint8

	raw  bytes.Buffer
	bw   *bufio.Writer
	comp []byte

	ch [3]encoderChannelScratch

	zenc *zstd.Encoder
}

func NewEncoder() *Encoder {
	e := &Encoder{}
	e.Parallel = true
	e.patternCount = defaultPatternCount
	e.bw = bufio.NewWriter(&e.raw)
	e.zenc = zstdEncoderPool.Get().(*zstd.Encoder)
	return e
}

func (e *Encoder) Close() {
	if e == nil || e.zenc == nil {
		return
	}
	zstdEncoderPool.Put(e.zenc)
	e.zenc = nil
}

func (e *Encoder) SetPatternCount(n int) {
	e.patternCount = n
}

func (e *Encoder) SetYQuantShift(shift int) {
	e.yQuantShift = shift
}

func (e *Encoder) SetBackgroundTile(tile int) {
	e.backgroundTile = tile
}

func (e *Encoder) SetLevels(levels []int) {
	if len(levels) == 0 {
		e.levels = nil
		return
	}
	e.levels = append(e.levels[:0], levels...)
}

func (e *Encoder) Levels() []int {
	return append([]int(nil), e.levels...)
}

func (e *Encoder) ensurePlanes(w, h int) {
	n := w * h
	if cap(e.yPlane) < n {
		e.yPlane = make([]uint8, n)
		e.cbPlane = make([]uint8, n)
		e.crPlane = make([]uint8, n)
		return
	}
	e.yPlane = e.yPlane[:n]
	e.cbPlane = e.cbPlane[:n]
	e.crPlane = e.crPlane[:n]
}

func (e *Encoder) encodeChannelReuse(channelID int, plane []uint8, stride, w4, h4, fullW, fullH int, useMacro bool, scratch *encoderChannelScratch) (uint32, []byte, []byte, []uint8, []uint8, error) {
	levels := e.levels
	smallBlock := levels[0]
	topBlock := levels[len(levels)-1]
	topLevelCount := countTopLevelBlocks(fullW, fullH, topBlock)

	// Worst-case leaf count is still the full smallest-block grid over the encoded area.
	worstBlockCount := countSmallBlocks(w4, h4, smallBlock)
	if worstBlockCount < 0 {
		worstBlockCount = 0
	}

	scratch.sizeBits = scratch.sizeBits[:0]
	if topLevelCount > 0 {
		// In the worst case every non-leaf smallest block ancestor emits one split bit.
		worstDecisionBits := 0
		if smallBlock > 0 {
			worstDecisionBits = worstBlockCount - topLevelCount
		}
		if worstDecisionBits < topLevelCount {
			worstDecisionBits = topLevelCount
		}
		sizeBytes := (worstDecisionBits + 7) / 8
		if cap(scratch.sizeBits) < sizeBytes {
			scratch.sizeBits = make([]byte, 0, sizeBytes)
		}
	}
	// At most one pattern bit per pixel in the encoded area.
	// For smallBlock==1, patterns are only emitted for macro-blocks, so fullW*fullH is a safe bound.
	patternBits := w4 * h4
	if smallBlock == 1 {
		patternBits = fullW * fullH
	}
	scratch.patternBits = scratch.patternBits[:0]
	if patternBits > 0 {
		patternBytes := (patternBits + 7) / 8
		if cap(scratch.patternBits) < patternBytes {
			scratch.patternBits = make([]byte, 0, patternBytes)
		}
	}

	patternBitCount := patternIndexBitsForCount(e.patternCount)

	sizeStream := newBitStream(scratch.sizeBits)
	patternStream := newBitStream(scratch.patternBits)

	if worstBlockCount > 0 {
		if cap(scratch.fgVals) < worstBlockCount {
			scratch.fgVals = make([]uint8, 0, worstBlockCount)
			scratch.bgVals = make([]uint8, 0, worstBlockCount)
		} else {
			scratch.fgVals = scratch.fgVals[:0]
			scratch.bgVals = scratch.bgVals[:0]
		}
	} else {
		scratch.fgVals = scratch.fgVals[:0]
		scratch.bgVals = scratch.bgVals[:0]
	}

	var blockCount uint32
	height := 0
	if stride > 0 {
		height = len(plane) / stride
	}
	if cap(scratch.levelSpreads) < len(levels) {
		scratch.levelSpreads = make([]int32, len(levels))
	} else {
		scratch.levelSpreads = scratch.levelSpreads[:len(levels)]
	}
	for i := range levels {
		scratch.levelSpreads[i] = spreadForBlockSize(e.quality)
	}
	quantize := func(fg, bg uint8) (uint8, uint8) {
		if channelID == chY {
			return quantizeY(fg, e.yQuantShift), quantizeY(bg, e.yQuantShift)
		}
		return fg, bg
	}

	if len(levels) == 2 && useMacro && smallBlock == 1 && topBlock == 2 && fullW == w4 && fullH == h4 {
		// Specialized hot path for the most common setting (quality >= 80):
		// - macro blocks are 2x2
		// - small blocks are 1x1 (always solid, no pattern bits)
		// This avoids canUseBigBlockChannel/encodeBlockPlane calls for the 1x1 case
		// and encodes 2x2 blocks with direct 4-byte loads.
		for my := 0; my < fullH; my += 2 {
			row0 := my * stride
			row1 := (my + 1) * stride
			for mx := 0; mx < fullW; mx += 2 {
				idx0 := row0 + mx
				idx2 := row1 + mx

				v0 := plane[idx0]
				v1 := plane[idx0+1]
				v2 := plane[idx2]
				v3 := plane[idx2+1]

				minV := v0
				maxV := v0
				if v1 < minV {
					minV = v1
				} else if v1 > maxV {
					maxV = v1
				}
				if v2 < minV {
					minV = v2
				} else if v2 > maxV {
					maxV = v2
				}
				if v3 < minV {
					minV = v3
				} else if v3 > maxV {
					maxV = v3
				}

				useBig := int32(maxV)-int32(minV) < scratch.levelSpreads[len(scratch.levelSpreads)-1]
				sizeStream.writeBit(useBig)

				if useBig {
					sum := uint64(v0) + uint64(v1) + uint64(v2) + uint64(v3)
					avg := uint8(sum / 4)
					thr := avg

					var bits uint64
					var fgSum, bgSum uint64
					var fgCnt, bgCnt uint32

					isFg0 := v0 >= thr
					bits <<= 1
					if isFg0 {
						bits |= 1
						fgSum += uint64(v0)
						fgCnt++
					} else {
						bgSum += uint64(v0)
						bgCnt++
					}

					isFg1 := v1 >= thr
					bits <<= 1
					if isFg1 {
						bits |= 1
						fgSum += uint64(v1)
						fgCnt++
					} else {
						bgSum += uint64(v1)
						bgCnt++
					}

					isFg2 := v2 >= thr
					bits <<= 1
					if isFg2 {
						bits |= 1
						fgSum += uint64(v2)
						fgCnt++
					} else {
						bgSum += uint64(v2)
						bgCnt++
					}

					isFg3 := v3 >= thr
					bits <<= 1
					if isFg3 {
						bits |= 1
						fgSum += uint64(v3)
						fgCnt++
					} else {
						bgSum += uint64(v3)
						bgCnt++
					}

					fg, bg := avg, avg
					isPattern := false
					if fgCnt != 0 && bgCnt != 0 {
						fg = uint8(fgSum / uint64(fgCnt))
						bg = uint8(bgSum / uint64(bgCnt))
						isPattern = fg != bg
					}

					fg, bg = quantize(fg, bg)
					if isPattern {
						mask := patternMaskFromUint64(bits, 4)
						idx, invert := nearestPatternIndex(mask, 2, 2, e.patternCount)
						if invert {
							fg, bg = bg, fg
						}
						fg = encodeBlockFlagIntoFG(fg, true, bg)
						scratch.fgVals = append(scratch.fgVals, fg)
						patternStream.writeBits(idx, patternBitCount)
						scratch.bgVals = append(scratch.bgVals, bg)
					} else {
						fg = encodeBlockFlagIntoFG(fg, false, bg)
						scratch.fgVals = append(scratch.fgVals, fg)
					}
					blockCount++
				} else {
					// 4 solid 1x1 blocks, no pattern bits, type bits are all 0.
					scratch.fgVals = append(scratch.fgVals,
						encodeBlockFlagIntoFG(v0, false, 0),
						encodeBlockFlagIntoFG(v1, false, 0),
						encodeBlockFlagIntoFG(v2, false, 0),
						encodeBlockFlagIntoFG(v3, false, 0),
					)
					blockCount += 4
				}
			}
		}

		// right stripe: 1x1 solid blocks only
		for my := 0; my < fullH; my++ {
			for mx := fullW; mx < w4; mx++ {
				v := plane[my*stride+mx]
				scratch.fgVals = append(scratch.fgVals, encodeBlockFlagIntoFG(v, false, 0))
				blockCount++
			}
		}

		// bottom stripe: 1x1 solid blocks only (including bottom-right corner)
		for my := fullH; my < h4; my++ {
			row := my * stride
			for mx := 0; mx < w4; mx++ {
				v := plane[row+mx]
				scratch.fgVals = append(scratch.fgVals, encodeBlockFlagIntoFG(v, false, 0))
				blockCount++
			}
		}

		scratch.sizeBits = sizeStream.bytesPadded()
		scratch.patternBits = patternStream.bytesPadded()
		return blockCount, scratch.sizeBits, scratch.patternBits, scratch.fgVals, scratch.bgVals, nil
	}

	parallelMain := false
	if e.Parallel && useMacro && fullW > 0 && fullH > 0 && fullW*fullH >= parallelEncodeMinPixels {
		rows := fullH / topBlock
		if rows >= 2 {
			workers := min(runtime.NumCPU(), rows)
			rowsPer := (rows + workers - 1) / workers

			if len(scratch.stripeScratch) < workers {
				scratch.stripeScratch = make([]encoderChannelScratch, workers)
				scratch.stripeResults = make([]stripeResult, workers)
			}
			results := scratch.stripeResults[:workers]
			scratches := scratch.stripeScratch[:workers]

			var wg sync.WaitGroup
			for i := 0; i < workers; i++ {
				y0 := i * rowsPer
				y1 := min(y0+rowsPer, rows)
				if y0 >= y1 {
					continue
				}
				wg.Add(1)
				go func(i, y0, y1 int) {
					defer wg.Done()
					stripeHeight := (y1 - y0) * topBlock
					worstStripeBlocks := countSmallBlocks(fullW, stripeHeight, smallBlock)
					if worstStripeBlocks < 0 {
						worstStripeBlocks = 0
					}
					scratchLocal := &scratches[i]
					scratchLocal.sizeBits = scratchLocal.sizeBits[:0]
					worstStripeDecisionBits := worstStripeBlocks
					sizeBytes := (worstStripeDecisionBits + 7) / 8
					if cap(scratchLocal.sizeBits) < sizeBytes {
						scratchLocal.sizeBits = make([]byte, 0, sizeBytes)
					}
					scratchLocal.patternBits = scratchLocal.patternBits[:0]
					stripePatternBits := fullW * stripeHeight
					patternBytes := (stripePatternBits + 7) / 8
					if cap(scratchLocal.patternBits) < patternBytes {
						scratchLocal.patternBits = make([]byte, 0, patternBytes)
					}
					sizeS := newBitStream(scratchLocal.sizeBits)
					patternS := newBitStream(scratchLocal.patternBits)
					if cap(scratchLocal.fgVals) < worstStripeBlocks {
						scratchLocal.fgVals = make([]uint8, 0, worstStripeBlocks)
						scratchLocal.bgVals = make([]uint8, 0, worstStripeBlocks)
					} else {
						scratchLocal.fgVals = scratchLocal.fgVals[:0]
						scratchLocal.bgVals = scratchLocal.bgVals[:0]
					}

					var blockCountLocal uint32
					var encodeNodeLocal func(x, y, levelIdx int) error
					encodeNodeLocal = func(x, y, levelIdx int) error {
						size := levels[levelIdx]
						if levelIdx == 0 {
							fg, bg, bits, isPattern, err := encodeBlockPlaneReuse(plane, stride, height, x, y, size, size, scratchLocal)
							if err != nil {
								return err
							}
							fg, bg = quantize(fg, bg)
							if isPattern {
								idx, invert := nearestPatternIndex(bits, size, size, e.patternCount)
								if invert {
									invertPatternBitsInPlace(bits, size, size)
									fg, bg = bg, fg
								}
								patternS.writeBits(idx, patternBitCount)
							}
							fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
							scratchLocal.fgVals = append(scratchLocal.fgVals, fg)
							if isPattern {
								scratchLocal.bgVals = append(scratchLocal.bgVals, bg)
							}
							blockCountLocal++
							return nil
						}

						useHere := canUseBlockSizeChannel(plane, stride, height, x, y, size, scratch.levelSpreads[levelIdx])
						sizeS.writeBit(useHere)
						if useHere {
							fg, bg, bits, isPattern, err := encodeBlockPlaneReuse(plane, stride, height, x, y, size, size, scratchLocal)
							if err != nil {
								return err
							}
							fg, bg = quantize(fg, bg)
							if isPattern {
								idx, invert := nearestPatternIndex(bits, size, size, e.patternCount)
								if invert {
									invertPatternBitsInPlace(bits, size, size)
									fg, bg = bg, fg
								}
								patternS.writeBits(idx, patternBitCount)
							}
							fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
							scratchLocal.fgVals = append(scratchLocal.fgVals, fg)
							if isPattern {
								scratchLocal.bgVals = append(scratchLocal.bgVals, bg)
							}
							blockCountLocal++
							return nil
						}

						childSize := levels[levelIdx-1]
						ratio := size / childSize
						for by := 0; by < ratio; by++ {
							for bx := 0; bx < ratio; bx++ {
								cx := x + bx*childSize
								cy := y + by*childSize
								if vw, vh := visibleBlockDims(stride, height, cx, cy, childSize, childSize); vw <= 0 || vh <= 0 {
									continue
								}
								if err := encodeNodeLocal(cx, cy, levelIdx-1); err != nil {
									return err
								}
							}
						}
						return nil
					}

					for my := y0 * topBlock; my < y1*topBlock; my += topBlock {
						for mx := 0; mx < fullW; mx += topBlock {
							if err := encodeNodeLocal(mx, my, len(levels)-1); err != nil {
								results[i] = stripeResult{err: err}
								return
							}
						}
					}

					results[i] = stripeResult{
						sizeBuf:     sizeS.buf,
						sizeByte:    sizeS.byte,
						sizeN:       sizeS.n,
						patternBuf:  patternS.buf,
						patternByte: patternS.byte,
						patternN:    patternS.n,
						fgVals:      scratchLocal.fgVals,
						bgVals:      scratchLocal.bgVals,
						blockCount:  blockCountLocal,
					}
				}(i, y0, y1)
			}
			wg.Wait()

			sizeStream.buf = sizeStream.buf[:0]
			patternStream.buf = patternStream.buf[:0]
			sizeStream.byte, sizeStream.n = 0, 0
			patternStream.byte, patternStream.n = 0, 0

			for i := 0; i < workers; i++ {
				if results[i].err != nil {
					return 0, nil, nil, nil, nil, results[i].err
				}
				sizeStream.appendStream(&bitStream{buf: results[i].sizeBuf, byte: results[i].sizeByte, n: results[i].sizeN})
				patternStream.appendStream(&bitStream{buf: results[i].patternBuf, byte: results[i].patternByte, n: results[i].patternN})
				scratch.fgVals = append(scratch.fgVals, results[i].fgVals...)
				scratch.bgVals = append(scratch.bgVals, results[i].bgVals...)
				blockCount += results[i].blockCount
			}
			parallelMain = true
		}
	}

	var encodeNode func(x, y, levelIdx int) error
	encodeNode = func(x, y, levelIdx int) error {
		size := levels[levelIdx]
		if levelIdx == 0 {
			fg, bg, bits, isPattern, err := encodeBlockPlaneReuse(plane, stride, height, x, y, size, size, scratch)
			if err != nil {
				return err
			}
			fg, bg = quantize(fg, bg)
			if isPattern {
				idx, invert := nearestPatternIndex(bits, size, size, e.patternCount)
				if invert {
					invertPatternBitsInPlace(bits, size, size)
					fg, bg = bg, fg
				}
				patternStream.writeBits(idx, patternBitCount)
			}
			fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				scratch.bgVals = append(scratch.bgVals, bg)
			}
			blockCount++
			return nil
		}

		useHere := canUseBlockSizeChannel(plane, stride, height, x, y, size, scratch.levelSpreads[levelIdx])
		sizeStream.writeBit(useHere)
		if useHere {
			fg, bg, bits, isPattern, err := encodeBlockPlaneReuse(plane, stride, height, x, y, size, size, scratch)
			if err != nil {
				return err
			}
			fg, bg = quantize(fg, bg)
			if isPattern {
				idx, invert := nearestPatternIndex(bits, size, size, e.patternCount)
				if invert {
					invertPatternBitsInPlace(bits, size, size)
					fg, bg = bg, fg
				}
				patternStream.writeBits(idx, patternBitCount)
			}
			fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				scratch.bgVals = append(scratch.bgVals, bg)
			}
			blockCount++
			return nil
		}

		childSize := levels[levelIdx-1]
		ratio := size / childSize
		for by := 0; by < ratio; by++ {
			for bx := 0; bx < ratio; bx++ {
				cx := x + bx*childSize
				cy := y + by*childSize
				if vw, vh := visibleBlockDims(stride, height, cx, cy, childSize, childSize); vw <= 0 || vh <= 0 {
					continue
				}
				if err := encodeNode(cx, cy, levelIdx-1); err != nil {
					return err
				}
			}
		}
		return nil
	}

	// main topBlock x topBlock area
	if !parallelMain {
		for my := 0; my < fullH; my += topBlock {
			for mx := 0; mx < fullW; mx += topBlock {
				if err := encodeNode(mx, my, len(levels)-1); err != nil {
					return 0, nil, nil, nil, nil, err
				}
			}
		}
	}

	// right stripe: small blocks only
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			fg, bg, bits, isPattern, err := encodeBlockPlaneReuse(plane, stride, height, mx, my, smallBlock, smallBlock, scratch)
			if err != nil {
				return 0, nil, nil, nil, nil, err
			}
			fg, bg = quantize(fg, bg)
			fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				idx, invert := nearestPatternIndex(bits, smallBlock, smallBlock, e.patternCount)
				if invert {
					fg, bg = bg, fg
					fg = encodeBlockFlagIntoFG(fg, true, bg)
					scratch.fgVals[len(scratch.fgVals)-1] = fg
				}
				patternStream.writeBits(idx, patternBitCount)
				scratch.bgVals = append(scratch.bgVals, bg)
			}
			blockCount++
		}
	}
	// bottom stripe: small blocks only (including bottom-right corner)
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			fg, bg, bits, isPattern, err := encodeBlockPlaneReuse(plane, stride, height, mx, my, smallBlock, smallBlock, scratch)
			if err != nil {
				return 0, nil, nil, nil, nil, err
			}
			fg, bg = quantize(fg, bg)
			fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				idx, invert := nearestPatternIndex(bits, smallBlock, smallBlock, e.patternCount)
				if invert {
					fg, bg = bg, fg
					fg = encodeBlockFlagIntoFG(fg, true, bg)
					scratch.fgVals[len(scratch.fgVals)-1] = fg
				}
				patternStream.writeBits(idx, patternBitCount)
				scratch.bgVals = append(scratch.bgVals, bg)
			}
			blockCount++
		}
	}

	scratch.sizeBits = sizeStream.bytesPadded()
	scratch.patternBits = patternStream.bytesPadded()
	return blockCount, scratch.sizeBits, scratch.patternBits, scratch.fgVals, scratch.bgVals, nil
}

func (e *Encoder) Encode(img image.Image, quality int, bwmode bool) ([]byte, error) {
	e.quality = clampQuality(quality)
	e.bwmode = bwmode

	if len(e.levels) == 0 {
		e.levels = defaultLevels()
	} else {
		e.levels = normalizeLevels(e.levels)
	}
	if e.patternCount <= 0 {
		e.patternCount = defaultPatternCount
	}
	if e.zenc == nil {
		e.zenc = mustNewZstdEncoder()
	}

	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()
	parallelPixels := w * h

	e.ensurePlanes(w, h)
	if e.Parallel && parallelPixels >= parallelExtractMinPixels {
		extractYCbCrPlanesInto(img, e.yPlane, e.cbPlane, e.crPlane)
	} else {
		extractYCbCrPlanesIntoSerial(img, e.yPlane, e.cbPlane, e.crPlane)
	}

	useChromaGrid := !e.bwmode && e.backgroundTile > 0

	// Decide which channels will be stored. Y is always present; Cb/Cr
	// may be omitted in grayscale mode.
	channelsMask := byte(channelFlagY)
	if !e.bwmode {
		channelsMask |= channelFlagCb | channelFlagCr
		if useChromaGrid {
			channelsMask |= channelFlagChromaGrid
		}
	}

	e.raw.Reset()
	e.bw.Reset(&e.raw)
	// no logging

	levels := e.levels
	smallBlock := levels[0]
	w4 := ceilToStep(w, smallBlock)
	h4 := ceilToStep(h, smallBlock)
	if w == 0 || h == 0 {
		return nil, fmt.Errorf("image too small for %dx%d blocks: %dx%d", smallBlock, smallBlock, w, h)
	}
	topBlock := levels[len(levels)-1]
	fullW := ceilToStep(w, topBlock)
	fullH := ceilToStep(h, topBlock)

	if topBlock < smallBlock || topBlock%smallBlock != 0 {
		return nil, fmt.Errorf("top block (%d) must be >= base block (%d) and a multiple of it",
			topBlock, smallBlock)
	}
	useMacro := topBlock > smallBlock

	estimated := len(codec) + 2 + 2 + len(levels)*2 + 1 + 4 + 4
	estimated += estimateChannelBytes(w4, h4, fullW, fullH, levels, e.patternCount, true)
	if !e.bwmode && !useChromaGrid {
		estimated += estimateChannelBytes(w4, h4, fullW, fullH, levels, e.patternCount, true) * 2
	}
	if useChromaGrid {
		gridW := (w+e.backgroundTile-1)/e.backgroundTile + 1
		gridH := (h+e.backgroundTile-1)/e.backgroundTile + 1
		estimated += 5 + 2*gridW*gridH
	}
	if estimated > 0 {
		e.raw.Grow(estimated)
	}

	// --- Write header ---
	if _, err := e.bw.WriteString(codec); err != nil {
		return nil, err
	}
	if err := writeU16BE(e.bw, uint16(e.patternCount)); err != nil {
		return nil, err
	}
	if err := writeU16BE(e.bw, uint16(len(levels))); err != nil {
		return nil, err
	}
	for _, level := range levels {
		if err := writeU16BE(e.bw, uint16(level)); err != nil {
			return nil, err
		}
	}
	// channels mask: which Y/Cb/Cr planes are stored.
	if err := e.bw.WriteByte(channelsMask); err != nil {
		return nil, err
	}
	if err := writeU32BE(e.bw, uint32(w)); err != nil {
		return nil, err
	}
	if err := writeU32BE(e.bw, uint32(h)); err != nil {
		return nil, err
	}

	var channels [3]encodeChannelSpec
	chCount := 1
	channels[0] = encodeChannelSpec{id: chY, plane: e.yPlane}
	if !e.bwmode && !useChromaGrid {
		channels[1] = encodeChannelSpec{id: chCb, plane: e.cbPlane}
		channels[2] = encodeChannelSpec{id: chCr, plane: e.crPlane}
		chCount = 3
	}

	// no logging

	if e.Parallel && chCount > 1 && parallelPixels >= parallelEncodeMinPixels {
		// Encode channels in parallel; scratch is per-channel so it's safe to reuse.
		var results [3]encodeChannelResult
		var wg sync.WaitGroup
		for i := 0; i < chCount; i++ {
			wg.Add(1)
			ch := channels[i]
			go encodeChannelWorker(e, &results[i], ch.id, ch.plane, w, w4, h4, fullW, fullH, useMacro, &e.ch[ch.id], &wg)
		}
		wg.Wait()

		for i := 0; i < chCount; i++ {
			res := results[i]
			if res.err != nil {
				return nil, res.err
			}
			_ = res

			scratch := &e.ch[channels[i].id]

			// number of blocks for this channel
			if err := writeU32BE(e.bw, res.blockCount); err != nil {
				return nil, err
			}
			// write size stream for this channel
			if err := writeU32BE(e.bw, uint32(len(res.sizeBytes))); err != nil {
				return nil, err
			}
			if _, err := e.bw.Write(res.sizeBytes); err != nil {
				return nil, err
			}
			// write pattern stream for this channel
			if err := writeU32BE(e.bw, uint32(len(res.patternBytes))); err != nil {
				return nil, err
			}
			if _, err := e.bw.Write(res.patternBytes); err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, uint32(len(res.fgVals))); err != nil {
				return nil, err
			}
			var err error
			scratch.deltaBuf, err = writeDeltaPackedBytes(e.bw, res.fgVals, scratch.deltaBuf)
			if err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, uint32(len(res.bgVals))); err != nil {
				return nil, err
			}
			scratch.deltaBuf, err = writeDeltaPackedBytes(e.bw, res.bgVals, scratch.deltaBuf)
			if err != nil {
				return nil, err
			}

			_ = res
		}
	} else {
		// Encode channels sequentially (no additional goroutines).
		for i := 0; i < chCount; i++ {
			ch := channels[i]
			scratch := &e.ch[ch.id]
			blockCount, sizeBytes, patternBytes, fgVals, bgVals, err := e.encodeChannelReuse(ch.id, ch.plane, w, w4, h4, fullW, fullH, useMacro, scratch)
			if err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, blockCount); err != nil {
				return nil, err
			}
			if err := writeU32BE(e.bw, uint32(len(sizeBytes))); err != nil {
				return nil, err
			}
			if _, err := e.bw.Write(sizeBytes); err != nil {
				return nil, err
			}
			if err := writeU32BE(e.bw, uint32(len(patternBytes))); err != nil {
				return nil, err
			}
			if _, err := e.bw.Write(patternBytes); err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, uint32(len(fgVals))); err != nil {
				return nil, err
			}
			scratch.deltaBuf, err = writeDeltaPackedBytes(e.bw, fgVals, scratch.deltaBuf)
			if err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, uint32(len(bgVals))); err != nil {
				return nil, err
			}
			scratch.deltaBuf, err = writeDeltaPackedBytes(e.bw, bgVals, scratch.deltaBuf)
			if err != nil {
				return nil, err
			}

			_ = fgVals
			_ = bgVals
		}
	}

	if useChromaGrid {
		var chroma bytes.Buffer
		encodeChromaGridOverlay(&chroma, e.cbPlane, e.crPlane, w, h, e.backgroundTile)
		if _, err := e.bw.Write(chroma.Bytes()); err != nil {
			return nil, err
		}
	}

	if err := e.bw.Flush(); err != nil {
		return nil, err
	}

	e.comp = e.zenc.EncodeAll(e.raw.Bytes(), e.comp[:0])
	return e.comp, nil
}

// EncodeTo encodes the image and writes the compressed result to w.
// This mirrors the common Go codec style (e.g. image/jpeg), while still reusing
// Encoder internal scratch buffers between calls.
func (e *Encoder) EncodeTo(w io.Writer, img image.Image, quality int, bwmode bool) error {
	comp, err := e.Encode(img, quality, bwmode)
	if err != nil {
		return err
	}
	_, err = w.Write(comp)
	return err
}

// Encode runs the BABE encoder with three fully independent channel streams (Y, Cb, Cr).
// Each channel has its own block list, size stream, pattern stream, and FG/BG levels.
func Encode(img image.Image, quality int, bwmode bool) ([]byte, error) {
	e := NewEncoder()
	defer e.Close()
	return e.Encode(img, quality, bwmode)
}

func readPatternComposite(br *bitReader, bw, bh, patternCount int) (patternMask, error) {
	limit := patternCount
	if limit < 1 {
		limit = 1
	}
	idx, err := readPatternIndex(br, patternIndexBitsForCount(limit))
	if err != nil {
		return nil, err
	}
	book := fixedPatternCodebook(bw, bh, limit)
	if int(idx) >= len(book) {
		return book[len(book)-1], nil
	}
	return book[idx], nil
}

// drawBlockPlane decodes a single block for one channel into a planar buffer.
func drawBlockPlane(plane []uint8, stride int, x0, y0, bw, bh int, br *bitReader, fg, bg uint8, patternCount int) error {
	pattern, err := readPatternComposite(br, bw, bh, patternCount)
	if err != nil {
		return err
	}
	height := 0
	if stride > 0 {
		height = len(plane) / stride
	}
	visW, visH := visibleBlockDims(stride, height, x0, y0, bw, bh)
	for yy := 0; yy < visH; yy++ {
		for xx := 0; xx < visW; xx++ {
			val := bg
			if testPatternBit(pattern, bw, bh, xx, yy) {
				val = fg
			}
			idx := (y0+yy)*stride + (x0 + xx)
			if idx >= len(plane) {
				return fmt.Errorf("drawBlockPlane: index out of range")
			}
			plane[idx] = val
		}
	}
	return nil
}

// drawBlockIntoPix decodes a single block and writes into the packed RGBA buffer.
func drawBlockIntoPix(pix []byte, strideBytes, imgW, imgH int, x0, y0, bw, bh int, br *bitReader, fg, bg uint8, patternCount int, channelOffset int) error {
	pattern, err := readPatternComposite(br, bw, bh, patternCount)
	if err != nil {
		return err
	}
	return drawBlockIntoPixMask(pix, strideBytes, imgW, imgH, x0, y0, bw, bh, pattern, fg, bg, channelOffset)
}

// drawBlockIntoPixFull assumes the block is fully inside the image.
func drawBlockIntoPixFull(pix []byte, strideBytes, imgW int, x0, y0, bw, bh int, br *bitReader, fg, bg uint8, patternCount int, channelOffset int) error {
	pattern, err := readPatternComposite(br, bw, bh, patternCount)
	if err != nil {
		return err
	}
	drawBlockIntoPixFullMask(pix, strideBytes, x0, y0, bw, bh, pattern, fg, bg, channelOffset)
	return nil
}

// fillBlockPlane fills a block with a single value (no pattern bits).
func fillBlockPlane(plane []uint8, stride int, x0, y0, bw, bh int, val uint8) error {
	height := 0
	if stride > 0 {
		height = len(plane) / stride
	}
	visW, visH := visibleBlockDims(stride, height, x0, y0, bw, bh)
	for yy := 0; yy < visH; yy++ {
		for xx := 0; xx < visW; xx++ {
			idx := (y0+yy)*stride + (x0 + xx)
			if idx >= len(plane) {
				return fmt.Errorf("fillBlockPlane: index out of range")
			}
			plane[idx] = val
		}
	}
	return nil
}

// fillBlockIntoPix fills a block with a single value (no pattern bits).
func fillBlockIntoPix(pix []byte, strideBytes, imgW, imgH int, x0, y0, bw, bh int, val uint8, channelOffset int) error {
	visW, visH := visibleBlockDims(imgW, imgH, x0, y0, bw, bh)
	for yy := 0; yy < visH; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < visW; xx++ {
			o := row + xx*4
			if o < 0 || o >= len(pix) {
				return fmt.Errorf("fillBlockIntoPix: index out of range")
			}
			pix[o] = val
		}
	}
	return nil
}

// fillBlockIntoPixFull assumes the block is fully inside the image.
func fillBlockIntoPixFull(pix []byte, strideBytes int, x0, y0, bw, bh int, val uint8, channelOffset int) {
	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < bw; xx++ {
			pix[row+xx*4] = val
		}
	}
}

// readChannelSegment returns a zero-copy slice of the next channel stream
// (as written by Encode) from the decompressed payload. It advances *pos
// to the byte immediately after this segment.
func readChannelSegment(data []byte, pos *int) ([]byte, error) {
	start := *pos

	readU32 := func(label string) (uint32, error) {
		if len(data)-*pos < 4 {
			return 0, fmt.Errorf("readChannelSegment: truncated while reading %s", label)
		}
		v := binary.BigEndian.Uint32(data[*pos : *pos+4])
		*pos += 4
		return v, nil
	}

	// blockCount (4 bytes) – we don't need the value here, just skip it.
	if _, err := readU32("blockCount"); err != nil {
		return nil, err
	}

	// sizeStreamLen (4 bytes) + payload
	sizeStreamLen, err := readU32("sizeStreamLen")
	if err != nil {
		return nil, err
	}
	if uint32(len(data)-*pos) < sizeStreamLen {
		return nil, fmt.Errorf("readChannelSegment: truncated size stream")
	}
	*pos += int(sizeStreamLen)

	// patternLen (4 bytes) + payload
	patternLen, err := readU32("patternLen")
	if err != nil {
		return nil, err
	}
	if uint32(len(data)-*pos) < patternLen {
		return nil, fmt.Errorf("readChannelSegment: truncated pattern stream")
	}
	*pos += int(patternLen)

	// FG packed length (4 bytes) + payload
	fgPackedLen, err := readU32("FG packedLen")
	if err != nil {
		return nil, err
	}
	if uint32(len(data)-*pos) < fgPackedLen {
		return nil, fmt.Errorf("readChannelSegment: truncated FG packed data")
	}
	*pos += int(fgPackedLen)

	// BG packed length (4 bytes) + payload
	bgPackedLen, err := readU32("BG packedLen")
	if err != nil {
		return nil, err
	}
	if uint32(len(data)-*pos) < bgPackedLen {
		return nil, fmt.Errorf("readChannelSegment: truncated BG packed data")
	}
	*pos += int(bgPackedLen)

	end := *pos
	if end < start || end > len(data) {
		return nil, fmt.Errorf("readChannelSegment: invalid segment bounds")
	}

	return data[start:end], nil
}

// decodeChannel decodes one channel stream into a planar buffer of size imgW x imgH.
func decodeChannelWithRects(data []byte, imgW, imgH int, levels []int, patternCount int) ([]uint8, []image.Rectangle, error) {
	pos := 0
	if len(levels) == 0 {
		return nil, nil, fmt.Errorf("decodeChannel: missing block levels")
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
		return nil, nil, err
	}
	sizeStreamLen, err := readU32("sizeStreamLen")
	if err != nil {
		return nil, nil, err
	}
	sizeBytes, err := readSlice(sizeStreamLen, "sizeStream")
	if err != nil {
		return nil, nil, err
	}
	sizeBR := newBitReader(sizeBytes)
	patternLen, err := readU32("patternLen")
	if err != nil {
		return nil, nil, err
	}
	patternBytes, err := readSlice(patternLen, "patternStream")
	if err != nil {
		return nil, nil, err
	}
	patternBR := newBitReader(patternBytes)
	fgPackedLen, err := readU32("FG packedLen")
	if err != nil {
		return nil, nil, err
	}
	fgPacked, err := readSlice(fgPackedLen, "FG packed data")
	if err != nil {
		return nil, nil, err
	}
	if blockCount > 0 && fgPackedLen < blockCount {
		return nil, nil, fmt.Errorf("decodeChannel: FG packed data too short")
	}
	fgStream, err := newDeltaStream(fgPacked, int(blockCount))
	if err != nil {
		return nil, nil, err
	}
	bgPackedLen, err := readU32("BG packedLen")
	if err != nil {
		return nil, nil, err
	}
	bgPacked, err := readSlice(bgPackedLen, "BG packed data")
	if err != nil {
		return nil, nil, err
	}
	if bgPackedLen > blockCount {
		return nil, nil, fmt.Errorf("decodeChannel: BG packed data too long")
	}
	bgStream, err := newDeltaStream(bgPacked, int(bgPackedLen))
	if err != nil {
		return nil, nil, err
	}
	bgPerPattern := true

	plane := make([]uint8, imgW*imgH)
	rects := make([]image.Rectangle, 0, int(blockCount))

	smallBlock := levels[0]
	topBlock := levels[len(levels)-1]
	w4 := ceilToStep(imgW, smallBlock)
	h4 := ceilToStep(imgH, smallBlock)
	fullW := ceilToStep(imgW, topBlock)
	fullH := ceilToStep(imgH, topBlock)

	blockIndex := 0

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
			var bg uint8
			if bgPerPattern {
				bg, err = bgStream.next()
				if err != nil {
					return err
				}
			}
			if err := drawBlockPlane(plane, imgW, x, y, size, size, &patternBR, fg, bg, patternCount); err != nil {
				return err
			}
		} else {
			if err := fillBlockPlane(plane, imgW, x, y, size, size, fg); err != nil {
				return err
			}
		}
		rects = append(rects, image.Rect(x, y, x+size, y+size))
		blockIndex++
		return nil
	}

	for my := 0; my < fullH; my += topBlock {
		for mx := 0; mx < fullW; mx += topBlock {
			if err := decodeNode(mx, my, len(levels)-1); err != nil {
				return nil, nil, err
			}
		}
	}
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return nil, nil, fmt.Errorf("unexpected end of blocks in right stripe")
			}
			fg, err := fgStream.next()
			if err != nil {
				return nil, nil, err
			}
			if encodedBlockIsPattern(fg) {
				bg, err := bgStream.next()
				if err != nil {
					return nil, nil, err
				}
				if err := drawBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, patternCount); err != nil {
					return nil, nil, err
				}
			} else {
				if err := fillBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, fg); err != nil {
					return nil, nil, err
				}
			}
			rects = append(rects, image.Rect(mx, my, mx+smallBlock, my+smallBlock))
			blockIndex++
		}
	}
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return nil, nil, fmt.Errorf("unexpected end of blocks in bottom stripe")
			}
			fg, err := fgStream.next()
			if err != nil {
				return nil, nil, err
			}
			if encodedBlockIsPattern(fg) {
				bg, err := bgStream.next()
				if err != nil {
					return nil, nil, err
				}
				if err := drawBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, patternCount); err != nil {
					return nil, nil, err
				}
			} else {
				if err := fillBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, fg); err != nil {
					return nil, nil, err
				}
			}
			rects = append(rects, image.Rect(mx, my, mx+smallBlock, my+smallBlock))
			blockIndex++
		}
	}

	if blockIndex != int(blockCount) {
		return nil, nil, fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
	}
	return plane, rects, nil
}

// Decoder reuses large scratch buffers across Decode calls to reduce allocations.
// It is not safe for concurrent use. The returned *image.RGBA is reused and will
// be overwritten on the next Decode call.
