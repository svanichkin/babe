package classic

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"io"
	"sync"
)

type Encoder struct {
	// Parallel enables plane extraction, per-channel encoding and entropy workers.
	// Set to false to reduce goroutine overhead and allocations.
	Parallel bool

	// CornerData adds experimental source corner values for lossy decoding.
	// It produces BGC1 files, which require a decoder supporting this feature.
	CornerData bool

	yPlane      []uint8
	cbPlane     []uint8
	crPlane     []uint8
	losslessRGB *image.RGBA

	raw  bytes.Buffer
	bw   *bufio.Writer
	comp []byte

	ch [3]encoderChannelScratch

	packer payloadPacker
}

func NewEncoder() *Encoder {
	e := &Encoder{}
	e.Parallel = true
	e.bw = bufio.NewWriter(&e.raw)
	return e
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

func (e *Encoder) encodeChannelReuse(plane []uint8, stride, w4, h4, fullW, fullH int, useMacro bool, scratch *encoderChannelScratch) (uint32, []byte, []byte, []byte, []uint8, []uint8, error) {
	step := scratch.quantStep
	// macro-block decision bits (only for main fullW x fullH area)
	// Precompute an upper bound on the total block count so we can
	// preallocate FG/BG slices and avoid repeated growth.
	macroGroupsX := 0
	macroGroupsY := 0
	if macroBlock > 0 {
		macroGroupsX = fullW / macroBlock
		macroGroupsY = fullH / macroBlock
	}
	macroGroupCount := macroGroupsX * macroGroupsY

	// worst-case: every macro-block is split into a full grid of small blocks
	mbs := 1
	if smallBlock > 0 {
		mbs = macroBlock / smallBlock
		if mbs < 1 {
			mbs = 1
		}
	}
	worstMainBlocks := macroGroupCount * mbs * mbs

	// right stripe: only small blocks
	rightWidth := w4 - fullW
	rightBlocks := 0
	if rightWidth > 0 && smallBlock > 0 {
		rightBlocks = (fullH / smallBlock) * (rightWidth / smallBlock)
	}

	// bottom stripe (including bottom-right corner): only small blocks
	bottomHeight := h4 - fullH
	bottomBlocks := 0
	if bottomHeight > 0 && smallBlock > 0 {
		bottomBlocks = (bottomHeight / smallBlock) * (w4 / smallBlock)
	}

	worstBlockCount := worstMainBlocks + rightBlocks + bottomBlocks
	if worstBlockCount < 0 {
		worstBlockCount = 0
	}

	scratch.sizeBuf.Reset()
	scratch.typeBuf.Reset()
	scratch.patternBuf.Reset()

	if macroGroupCount > 0 {
		scratch.sizeBuf.Grow((macroGroupCount + 7) / 8)
	}
	if worstBlockCount > 0 {
		scratch.typeBuf.Grow((worstBlockCount + 7) / 8)
	}
	// At most one pattern bit per pixel in the encoded area.
	// For smallBlock==1, patterns are only emitted for macro-blocks, so fullW*fullH is a safe bound.
	patternBits := w4 * h4
	if smallBlock == 1 {
		patternBits = fullW * fullH
	}
	if patternBits > 0 {
		scratch.patternBuf.Grow((patternBits + 7) / 8)
	}

	sizeW := newBitWriter(&scratch.sizeBuf)
	typeW := newBitWriter(&scratch.typeBuf)
	patternW := newBitWriter(&scratch.patternBuf)

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
	height := h4
	spread := allowedMacroSpreadForQuality(encQuality)
	if encQuality != 100 && smallBlock == 1 && macroBlock == 8 && (stride%4 != 0 || height%4 != 0) {
		// The fallback geometry is constant across qualities, so its threshold
		// must vary continuously rather than reset at the usual block bands.
		rem := 100*256 - qualityProfileFP
		spread = 256 + int32(int64(rem)*int64(rem)*255/(100*100*256))
	}
	if encQuality == 0 {
		// Every possible 8-bit spread fits; q0 uses the existing 8x8 blocks.
		spread = 256 * 256
	}

	if useMacro && smallBlock == 1 && macroBlock == 2 {
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

				if encQuality == 100 {
					fg, bg, bits, exact := exactQuad(v0, v1, v2, v3)
					sizeW.writeBit(exact)
					if exact {
						pattern := fg != bg
						scratch.fgVals = append(scratch.fgVals, fg)
						if pattern {
							patternW.writeBits(bits, 4)
							scratch.bgVals = append(scratch.bgVals, bg)
						}
						typeW.writeBit(pattern)
						blockCount++
					} else {
						scratch.fgVals = append(scratch.fgVals, v0, v1, v2, v3)
						typeW.writeBits(0, 4)
						blockCount += 4
					}
					continue
				}

				minV := min(v0, v1, v2, v3)
				maxV := max(v0, v1, v2, v3)

				useBig := int32(maxV)-int32(minV) < blockSpread(spread, mx, my)
				sizeW.writeBit(useBig)

				if useBig {
					fg, bg, bits := quadLevels(v0, v1, v2, v3, step)
					isPattern := fg != bg

					scratch.fgVals = append(scratch.fgVals, fg)
					if isPattern {
						patternW.writeBits(bits, 4)
						scratch.bgVals = append(scratch.bgVals, bg)
					}
					typeW.writeBit(isPattern)
					blockCount++
				} else {
					// 4 solid 1x1 blocks, no pattern bits, type bits are all 0.
					scratch.fgVals = append(scratch.fgVals, quantizeLevel(v0, step), quantizeLevel(v1, step), quantizeLevel(v2, step), quantizeLevel(v3, step))
					typeW.writeBits(0, 4)
					blockCount += 4
				}
			}
		}

		// right stripe: 1x1 solid blocks only
		for my := 0; my < fullH; my++ {
			for mx := fullW; mx < w4; mx++ {
				v := plane[my*stride+mx]
				scratch.fgVals = append(scratch.fgVals, quantizeLevel(v, step))
				typeW.writeBit(false)
				blockCount++
			}
		}

		// bottom stripe: 1x1 solid blocks only (including bottom-right corner)
		for my := fullH; my < h4; my++ {
			row := my * stride
			for mx := 0; mx < w4; mx++ {
				v := plane[row+mx]
				scratch.fgVals = append(scratch.fgVals, quantizeLevel(v, step))
				typeW.writeBit(false)
				blockCount++
			}
		}

		sizeW.flush()
		typeW.flush()
		patternW.flush()

		return blockCount, scratch.sizeBuf.Bytes(), scratch.typeBuf.Bytes(), scratch.patternBuf.Bytes(), scratch.fgVals, scratch.bgVals, nil
	}

	// main macroBlock x macroBlock area
	for my := 0; my < fullH; my += macroBlock {
		for mx := 0; mx < fullW; mx += macroBlock {
			useBig := useMacro && canUseBigBlockChannel(plane, stride, height, mx, my, blockSpread(spread, mx, my))
			sizeW.writeBit(useBig)
			if useBig {
				fg, bg, isPattern, err := encodeChannelBlock(plane, stride, height, mx, my, macroBlock, macroBlock, step, scratch.solidThreshold, &patternW)
				if err != nil {
					return 0, nil, nil, nil, nil, nil, err
				}
				scratch.fgVals = append(scratch.fgVals, fg)
				if isPattern {
					scratch.bgVals = append(scratch.bgVals, bg)
				}
				typeW.writeBit(isPattern)
				blockCount++
			} else {
				// grid of smallBlock x smallBlock blocks covering the macroBlock area
				for by := 0; by < macroBlock; by += smallBlock {
					for bx := 0; bx < macroBlock; bx += smallBlock {
						xx, yy := mx+bx, my+by
						// for smallBlock > 1 we also emit pattern bits; for smallBlock == 1 we skip pattern bits
						var pw *bitWriter
						if smallBlock > 1 {
							pw = &patternW
						}
						fg, bg, isPattern, err := encodeChannelBlock(plane, stride, height, xx, yy, smallBlock, smallBlock, step, scratch.solidThreshold, pw)
						if err != nil {
							return 0, nil, nil, nil, nil, nil, err
						}
						scratch.fgVals = append(scratch.fgVals, fg)
						if isPattern {
							scratch.bgVals = append(scratch.bgVals, bg)
						}
						typeW.writeBit(isPattern)
						blockCount++
					}
				}
			}
		}
	}

	// right stripe: small blocks only
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			var pw *bitWriter
			if smallBlock > 1 {
				pw = &patternW
			}
			fg, bg, isPattern, err := encodeChannelBlock(plane, stride, height, mx, my, smallBlock, smallBlock, step, scratch.solidThreshold, pw)
			if err != nil {
				return 0, nil, nil, nil, nil, nil, err
			}
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				scratch.bgVals = append(scratch.bgVals, bg)
			}
			typeW.writeBit(isPattern)
			blockCount++
		}
	}
	// bottom stripe: small blocks only (including bottom-right corner)
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			var pw *bitWriter
			if smallBlock > 1 {
				pw = &patternW
			}
			fg, bg, isPattern, err := encodeChannelBlock(plane, stride, height, mx, my, smallBlock, smallBlock, step, scratch.solidThreshold, pw)
			if err != nil {
				return 0, nil, nil, nil, nil, nil, err
			}
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				scratch.bgVals = append(scratch.bgVals, bg)
			}
			typeW.writeBit(isPattern)
			blockCount++
		}
	}

	sizeW.flush()
	typeW.flush()
	patternW.flush()

	return blockCount, scratch.sizeBuf.Bytes(), scratch.typeBuf.Bytes(), scratch.patternBuf.Bytes(), scratch.fgVals, scratch.bgVals, nil
}

func (e *Encoder) Encode(img image.Image, quality int, bwmode bool) ([]byte, error) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()

	encodeBW = bwmode

	if err := setBlocksForQuality(quality); err != nil {
		return nil, err
	}
	b := img.Bounds()
	w, h := b.Dx(), b.Dy()
	if err := checkImageSize(w, h); err != nil {
		return nil, err
	}
	if encQuality != 100 && (w%4 != 0 || h%4 != 0) {
		// Whole-pixel edge blocks prevent black strips at image dimensions that
		// are not divisible by the coarser block sizes.
		smallBlock, macroBlock = 1, 8
	}

	yStep, cStep := quantStepsForQuality(encQuality)
	e.ch[chY].quantStep = yStep
	e.ch[chCb].quantStep = cStep
	e.ch[chCr].quantStep = cStep
	e.ch[chY].solidThreshold = 0
	e.ch[chCb].solidThreshold = max(0, (40*256-qualityProfileFP)/40)
	e.ch[chCr].solidThreshold = e.ch[chCb].solidThreshold

	e.ensurePlanes(w, h)
	if encQuality == 100 {
		if err := e.extractLosslessPlanes(img, bwmode); err != nil {
			return nil, err
		}
	} else if e.Parallel {
		extractYCbCrPlanesInto(img, e.yPlane, e.cbPlane, e.crPlane)
	} else {
		extractYCbCrPlanesIntoSerial(img, e.yPlane, e.cbPlane, e.crPlane)
	}
	var sourceCorners cornerGrid
	if e.CornerData && encQuality != 100 {
		const step = 8
		sourceCorners = makeCornerGrid(w, h, step, func(x, y int) [3]byte {
			i := y*w + x
			v := [3]byte{e.yPlane[i], 128, 128}
			if !bwmode {
				v[1], v[2] = e.cbPlane[i], e.crPlane[i]
			}
			return v
		})
	}
	if qualityProfileFP < 35*256 {
		remaining := 35*256 - qualityProfileFP
		threshold := int(int64(remaining) * int64(remaining) * 255 / (35 * 35 * 256 * 256))
		smoothLowQualityLuma(e.yPlane, w, h, threshold)
	}

	// Decide which channels will be stored. Y is always present; Cb/Cr
	// may be omitted in grayscale mode.
	channelsMask := byte(channelFlagY)
	if encQuality == 100 {
		channelsMask |= channelFlagLossless
	}
	if !encodeBW {
		channelsMask |= channelFlagCb | channelFlagCr
	}

	e.raw.Reset()
	e.bw.Reset(&e.raw)

	w4 := (w / smallBlock) * smallBlock
	h4 := (h / smallBlock) * smallBlock
	if w4 == 0 || h4 == 0 {
		return nil, fmt.Errorf("image too small for %dx%d blocks: %dx%d", smallBlock, smallBlock, w, h)
	}
	fullW := (w4 / macroBlock) * macroBlock
	fullH := (h4 / macroBlock) * macroBlock

	if macroBlock < smallBlock || macroBlock%smallBlock != 0 {
		return nil, fmt.Errorf("macroBlock (%d) must be >= smallBlock (%d) and a multiple of it",
			macroBlock, smallBlock)
	}
	useMacro := macroBlock > smallBlock

	// --- Write header ---
	if _, err := e.bw.WriteString(codec); err != nil {
		return nil, err
	}
	if err := writeU16BE(e.bw, uint16(smallBlock)); err != nil {
		return nil, err
	}
	if err := writeU16BE(e.bw, uint16(macroBlock)); err != nil {
		return nil, err
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
	if !encodeBW {
		channels[1] = encodeChannelSpec{id: chCb, plane: e.cbPlane}
		channels[2] = encodeChannelSpec{id: chCr, plane: e.crPlane}
		chCount = 3
	}

	if e.Parallel {
		// Encode channels in parallel; scratch is per-channel so it's safe to reuse.
		var results [3]encodeChannelResult
		var wg sync.WaitGroup
		for i := 0; i < chCount; i++ {
			wg.Add(1)
			ch := channels[i]
			go encodeChannelWorker(e, &results[i], ch.plane, w, w4, h4, fullW, fullH, useMacro, &e.ch[ch.id], &wg)
		}
		wg.Wait()

		for i := 0; i < chCount; i++ {
			res := results[i]
			if res.err != nil {
				return nil, res.err
			}

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
			// write type stream for this channel
			if err := writeU32BE(e.bw, uint32(len(res.typeBytes))); err != nil {
				return nil, err
			}
			if _, err := e.bw.Write(res.typeBytes); err != nil {
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
			if err := writeDeltaPackedBytes(e.bw, res.fgVals); err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, uint32(len(res.bgVals))); err != nil {
				return nil, err
			}
			if err := writeDeltaPackedBytes(e.bw, res.bgVals); err != nil {
				return nil, err
			}
		}
	} else {
		// Encode channels sequentially (no additional goroutines).
		for i := 0; i < chCount; i++ {
			ch := channels[i]
			scratch := &e.ch[ch.id]
			blockCount, sizeBytes, typeBytes, patternBytes, fgVals, bgVals, err := e.encodeChannelReuse(ch.plane, w, w4, h4, fullW, fullH, useMacro, scratch)
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
			if err := writeU32BE(e.bw, uint32(len(typeBytes))); err != nil {
				return nil, err
			}
			if _, err := e.bw.Write(typeBytes); err != nil {
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
			if err := writeDeltaPackedBytes(e.bw, fgVals); err != nil {
				return nil, err
			}

			if err := writeU32BE(e.bw, uint32(len(bgVals))); err != nil {
				return nil, err
			}
			if err := writeDeltaPackedBytes(e.bw, bgVals); err != nil {
				return nil, err
			}
		}
	}

	if err := e.bw.Flush(); err != nil {
		return nil, err
	}

	comp, err := e.packer.compress(e.raw.Bytes(), e.comp[:0], e.Parallel)
	if err != nil {
		return nil, err
	}
	if e.CornerData && encQuality != 100 {
		comp = packCornerImage(comp, sourceCorners)
	}
	e.comp = comp
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
	return e.Encode(img, quality, bwmode)
}

func readPatternComposite(br *bitReader, bw, bh int) (uint64, error) {
	if bw == 6 && bh == 6 && smallBlock == 3 && macroBlock == 6 {
		var out uint64
		for qy := 0; qy < 2; qy++ {
			for qx := 0; qx < 2; qx++ {
				hi, err := br.readBits(8)
				if err != nil {
					return 0, err
				}
				lo, err := br.readBits(1)
				if err != nil {
					return 0, err
				}
				subBits := (uint16(hi) << 1) | uint16(lo)
				for yy := 0; yy < 3; yy++ {
					for xx := 0; xx < 3; xx++ {
						srcShift := 8 - (yy*3 + xx)
						dstY := qy*3 + yy
						dstX := qx*3 + xx
						dstShift := 35 - (dstY*6 + dstX)
						if ((subBits >> srcShift) & 1) != 0 {
							out |= uint64(1) << dstShift
						}
					}
				}
			}
		}
		return out, nil
	}
	if bw == 8 && bh == 8 && smallBlock == 4 && macroBlock == 8 {
		var out uint64
		for qy := 0; qy < 2; qy++ {
			for qx := 0; qx < 2; qx++ {
				sub, err := br.readBits(8)
				if err != nil {
					return 0, err
				}
				sub2, err := br.readBits(8)
				if err != nil {
					return 0, err
				}
				subBits := (uint16(sub) << 8) | uint16(sub2)
				for yy := 0; yy < 4; yy++ {
					for xx := 0; xx < 4; xx++ {
						srcShift := 15 - (yy*4 + xx)
						dstY := qy*4 + yy
						dstX := qx*4 + xx
						dstShift := 63 - (dstY*8 + dstX)
						if ((subBits >> srcShift) & 1) != 0 {
							out |= uint64(1) << dstShift
						}
					}
				}
			}
		}
		return out, nil
	}
	if bw*bh <= 8 {
		v, err := br.readBits(uint8(bw * bh))
		return uint64(v), err
	}
	var out uint64
	remaining := bw * bh
	for remaining > 0 {
		chunk := 8
		if remaining < chunk {
			chunk = remaining
		}
		part, err := br.readBits(uint8(chunk))
		if err != nil {
			return 0, err
		}
		out = (out << chunk) | uint64(part)
		remaining -= chunk
	}
	return out, nil
}

// drawBlockPlane decodes a single block for one channel into a planar buffer.
func drawBlockPlane(plane []uint8, stride int, x0, y0, bw, bh int, br *bitReader, fg, bg uint8) error {
	pattern, err := readPatternComposite(br, bw, bh)
	if err != nil {
		return err
	}
	for yy := 0; yy < bh; yy++ {
		for xx := 0; xx < bw; xx++ {
			val := bg
			shift := bw*bh - 1 - (yy*bw + xx)
			if ((pattern >> shift) & 1) != 0 {
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

// fillBlockPlane fills a block with a single value (no pattern bits).
func fillBlockPlane(plane []uint8, stride int, x0, y0, bw, bh int, val uint8) error {
	for yy := 0; yy < bh; yy++ {
		for xx := 0; xx < bw; xx++ {
			idx := (y0+yy)*stride + (x0 + xx)
			if idx >= len(plane) {
				return fmt.Errorf("fillBlockPlane: index out of range")
			}
			plane[idx] = val
		}
	}
	return nil
}

func drawBlockPix(pix []byte, strideBytes int, x0, y0, bw, bh int, br *bitReader, fg, bg uint8, channelOffset int) error {
	pattern, err := readPatternComposite(br, bw, bh)
	if err != nil {
		return err
	}
	if bw == 1 && bh == 1 {
		o := y0*strideBytes + x0*4 + channelOffset
		if o < 0 || o >= len(pix) {
			return fmt.Errorf("drawBlockPix: index out of range")
		}
		if pattern != 0 {
			pix[o] = fg
		} else {
			pix[o] = bg
		}
		return nil
	}

	if bw == 2 && bh == 2 {
		bits := uint8(pattern)

		row0 := y0*strideBytes + x0*4 + channelOffset
		row1 := row0 + strideBytes
		o00 := row0
		o01 := row0 + 4
		o10 := row1
		o11 := row1 + 4

		if o00 < 0 || o11 >= len(pix) {
			return fmt.Errorf("drawBlockPix: index out of range")
		}

		if (bits & 0b1000) != 0 {
			pix[o00] = fg
		} else {
			pix[o00] = bg
		}
		if (bits & 0b0100) != 0 {
			pix[o01] = fg
		} else {
			pix[o01] = bg
		}
		if (bits & 0b0010) != 0 {
			pix[o10] = fg
		} else {
			pix[o10] = bg
		}
		if (bits & 0b0001) != 0 {
			pix[o11] = fg
		} else {
			pix[o11] = bg
		}
		return nil
	}

	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < bw; xx++ {
			val := bg
			shift := bw*bh - 1 - (yy*bw + xx)
			if ((pattern >> shift) & 1) != 0 {
				val = fg
			}
			o := row + xx*4
			if o < 0 || o >= len(pix) {
				return fmt.Errorf("drawBlockPix: index out of range")
			}
			pix[o] = val
		}
	}
	return nil
}

func fillBlockPix(pix []byte, strideBytes int, x0, y0, bw, bh int, val uint8, channelOffset int) error {
	if bw == 1 && bh == 1 {
		o := y0*strideBytes + x0*4 + channelOffset
		if o < 0 || o >= len(pix) {
			return fmt.Errorf("fillBlockPix: index out of range")
		}
		pix[o] = val
		return nil
	}

	if bw == 2 && bh == 2 {
		row0 := y0*strideBytes + x0*4 + channelOffset
		row1 := row0 + strideBytes
		o00 := row0
		o01 := row0 + 4
		o10 := row1
		o11 := row1 + 4

		if o00 < 0 || o11 >= len(pix) {
			return fmt.Errorf("fillBlockPix: index out of range")
		}
		pix[o00] = val
		pix[o01] = val
		pix[o10] = val
		pix[o11] = val
		return nil
	}

	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < bw; xx++ {
			o := row + xx*4
			if o < 0 || o >= len(pix) {
				return fmt.Errorf("fillBlockPix: index out of range")
			}
			pix[o] = val
		}
	}
	return nil
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

	// typeStreamLen (4 bytes) + payload
	typeStreamLen, err := readU32("typeStreamLen")
	if err != nil {
		return nil, err
	}
	if uint32(len(data)-*pos) < typeStreamLen {
		return nil, fmt.Errorf("readChannelSegment: truncated type stream")
	}
	*pos += int(typeStreamLen)

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
func decodeChannel(data []byte, imgW, imgH int) ([]uint8, error) {
	pos := 0

	// helpers to read from the raw byte slice without extra allocations
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

	// blockCount
	blockCount, err := readU32("blockCount")
	if err != nil {
		return nil, err
	}

	// size stream (bit flags for macro vs small in the main area)
	sizeStreamLen, err := readU32("sizeStreamLen")
	if err != nil {
		return nil, err
	}
	sizeBytes, err := readSlice(sizeStreamLen, "sizeStream")
	if err != nil {
		return nil, err
	}
	sizeBR := newBitReader(sizeBytes)

	// type stream (pattern vs solid blocks)
	typeStreamLen, err := readU32("typeStreamLen")
	if err != nil {
		return nil, err
	}
	typeBytes, err := readSlice(typeStreamLen, "typeStream")
	if err != nil {
		return nil, err
	}
	typeBR := newBitReader(typeBytes)

	// pattern stream (actual bi-level patterns)
	patternLen, err := readU32("patternLen")
	if err != nil {
		return nil, err
	}
	patternBytes, err := readSlice(patternLen, "patternStream")
	if err != nil {
		return nil, err
	}
	patternBR := newBitReader(patternBytes)

	// FG: read mode and decode as a delta stream
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

	// BG: read mode and decode as a delta stream
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
	bgPerPattern := true

	// reconstruct the block geometry and fill the plane
	plane := make([]uint8, imgW*imgH)

	w4 := (imgW / smallBlock) * smallBlock
	h4 := (imgH / smallBlock) * smallBlock
	fullW := (w4 / macroBlock) * macroBlock
	fullH := (h4 / macroBlock) * macroBlock
	macroGroupCount := (fullW / macroBlock) * (fullH / macroBlock)
	useMacro := macroBlock > smallBlock

	// rebuild macro-block size decisions from the size stream
	macroBig := make([]bool, macroGroupCount)
	for i := range macroGroupCount {
		bit, err := sizeBR.readBit()
		if err != nil {
			return nil, err
		}
		macroBig[i] = bit
	}

	blockIndex := 0
	macroIndex := 0

	// main macroBlock x macroBlock area
	for my := 0; my < fullH; my += macroBlock {
		for mx := 0; mx < fullW; mx += macroBlock {
			if blockIndex >= int(blockCount) {
				return nil, fmt.Errorf("unexpected end of blocks in main area")
			}
			if useMacro && macroBig[macroIndex] {
				// macroBlock: read one type bit, FG and BG from the delta streams, use full pattern
				bitType, err := typeBR.readBit()
				if err != nil {
					return nil, err
				}

				fg, err := fgStream.next()
				if err != nil {
					return nil, err
				}
				if bitType {
					bg := fg
					if bgPerPattern {
						bg, err = bgStream.next()
						if err != nil {
							return nil, err
						}
					} else {
						bg, err = bgStream.next()
						if err != nil {
							return nil, err
						}
					}
					if err := drawBlockPlane(plane, imgW, mx, my, macroBlock, macroBlock, &patternBR, fg, bg); err != nil {
						return nil, err
					}
				} else {
					if !bgPerPattern {
						// legacy mode stores BG per block; consume to stay in sync
						if _, err := bgStream.next(); err != nil {
							return nil, err
						}
					}
					if err := fillBlockPlane(plane, imgW, mx, my, macroBlock, macroBlock, fg); err != nil {
						return nil, err
					}
				}
				blockIndex++
			} else {
				// grid of smallBlock x smallBlock blocks covering the macroBlock area
				for by := 0; by < macroBlock; by += smallBlock {
					for bx := 0; bx < macroBlock; bx += smallBlock {
						if blockIndex >= int(blockCount) {
							return nil, fmt.Errorf("unexpected end of blocks in macro grid")
						}
						bitType, err := typeBR.readBit()
						if err != nil {
							return nil, err
						}
						fg, err := fgStream.next()
						if err != nil {
							return nil, err
						}
						if bitType {
							bg := fg
							if bgPerPattern {
								bg, err = bgStream.next()
								if err != nil {
									return nil, err
								}
							} else {
								bg, err = bgStream.next()
								if err != nil {
									return nil, err
								}
							}
							if err := drawBlockPlane(plane, imgW, mx+bx, my+by, smallBlock, smallBlock, &patternBR, fg, bg); err != nil {
								return nil, err
							}
						} else {
							if !bgPerPattern {
								// legacy mode stores BG per block; consume to stay in sync
								if _, err := bgStream.next(); err != nil {
									return nil, err
								}
							}
							if err := fillBlockPlane(plane, imgW, mx+bx, my+by, smallBlock, smallBlock, fg); err != nil {
								return nil, err
							}
						}
						blockIndex++
					}
				}
			}
			macroIndex++
		}
	}

	// right stripe: small blocks only
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return nil, fmt.Errorf("unexpected end of blocks in right stripe")
			}
			bitType, err := typeBR.readBit()
			if err != nil {
				return nil, err
			}
			fg, err := fgStream.next()
			if err != nil {
				return nil, err
			}
			if bitType {
				bg := fg
				if bgPerPattern {
					bg, err = bgStream.next()
					if err != nil {
						return nil, err
					}
				} else {
					bg, err = bgStream.next()
					if err != nil {
						return nil, err
					}
				}
				if err := drawBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, &patternBR, fg, bg); err != nil {
					return nil, err
				}
			} else {
				if !bgPerPattern {
					// legacy mode stores BG per block; consume to stay in sync
					if _, err := bgStream.next(); err != nil {
						return nil, err
					}
				}
				if err := fillBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, fg); err != nil {
					return nil, err
				}
			}
			blockIndex++
		}
	}
	// bottom stripe: small blocks only (including bottom-right corner)
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return nil, fmt.Errorf("unexpected end of blocks in bottom stripe")
			}
			bitType, err := typeBR.readBit()
			if err != nil {
				return nil, err
			}
			fg, err := fgStream.next()
			if err != nil {
				return nil, err
			}
			if bitType {
				bg := fg
				if bgPerPattern {
					bg, err = bgStream.next()
					if err != nil {
						return nil, err
					}
				} else {
					bg, err = bgStream.next()
					if err != nil {
						return nil, err
					}
				}
				if err := drawBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, &patternBR, fg, bg); err != nil {
					return nil, err
				}
			} else {
				if !bgPerPattern {
					// legacy mode stores BG per block; consume to stay in sync
					if _, err := bgStream.next(); err != nil {
						return nil, err
					}
				}
				if err := fillBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, fg); err != nil {
					return nil, err
				}
			}
			blockIndex++
		}
	}

	if blockIndex != int(blockCount) {
		return nil, fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
	}
	if fgStream.i != fgStream.n {
		return nil, fmt.Errorf("color stream mismatch: fg used=%d expected=%d", fgStream.i, fgStream.n)
	}
	if bgStream.i != bgStream.n {
		return nil, fmt.Errorf("color stream mismatch: bg used=%d expected=%d", bgStream.i, bgStream.n)
	}

	return plane, nil
}
