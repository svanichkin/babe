package classic

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"image"
	"io"
	"runtime"
	"sync"

	"github.com/klauspost/compress/zstd"
)

type decoderChannelScratch struct {
	plane    []uint8
	macroBig []bool
}

// Decoder reuses large scratch buffers across Decode calls to reduce allocations.
// It is not safe for concurrent use. The returned *image.RGBA is reused and will
// be overwritten on the next Decode call.
type Decoder struct {
	// Parallel enables internal goroutines (per-channel decode and RGB conversion).
	// Set to false to reduce goroutine overhead and allocations.
	Parallel bool

	payload []byte
	zdec    *zstd.Decoder

	y  decoderChannelScratch
	cb decoderChannelScratch
	cr decoderChannelScratch

	neutral []uint8
	dst     *image.RGBA
}

func NewDecoder() *Decoder {
	return &Decoder{Parallel: true, zdec: mustNewZstdDecoder()}
}

func decodeChannelToPix(data []byte, imgW, imgH int, pix []byte, strideBytes int, channelOffset int) error {
	pos := 0

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

	typeStreamLen, err := readU32("typeStreamLen")
	if err != nil {
		return err
	}
	typeBytes, err := readSlice(typeStreamLen, "typeStream")
	if err != nil {
		return err
	}
	typeBR := newBitReader(typeBytes)

	patternLen, err := readU32("patternLen")
	if err != nil {
		return err
	}
	patternBytes, err := readSlice(patternLen, "patternStream")
	if err != nil {
		return err
	}
	patternBR := newBitReader(patternBytes)

	// FG: delta-coded bytes, one value per block.
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

	// BG: delta-coded bytes, one value per pattern block (type bit = 1).
	bgPerPattern := true
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

	w4 := (imgW / smallBlock) * smallBlock
	h4 := (imgH / smallBlock) * smallBlock
	fullW := (w4 / macroBlock) * macroBlock
	fullH := (h4 / macroBlock) * macroBlock
	useMacro := macroBlock > smallBlock

	blockIndex := 0

	// Hot path for the most common benchmark setting (quality >= 80):
	// - macro blocks are 2x2
	// - small blocks are 1x1 (always solid)
	// BG is stored only for pattern blocks (type bit = 1), otherwise BG == FG.
	if useMacro && smallBlock == 1 && macroBlock == 2 {
		macroGroupsX := fullW / 2
		macroGroupsY := fullH / 2
		macroGroupCount := macroGroupsX * macroGroupsY

		if macroGroupCount > 0 && len(sizeBytes)*8 < macroGroupCount {
			return fmt.Errorf("decodeChannel: size stream too short")
		}
		if int(blockCount) > 0 && len(typeBytes)*8 < int(blockCount) {
			return fmt.Errorf("decodeChannel: type stream too short")
		}

		for my := 0; my < fullH; my += 2 {
			row0 := my*strideBytes + channelOffset
			row1 := row0 + strideBytes
			for mx := 0; mx < fullW; mx += 2 {
				if blockIndex >= int(blockCount) {
					return fmt.Errorf("unexpected end of blocks in main area")
				}

				macroIsBig := sizeBR.readBitFast()
				if macroIsBig {
					bitType := typeBR.readBitFast()
					fg := fgStream.nextFast()

					o00 := row0 + mx*4
					o01 := o00 + 4
					o10 := row1 + mx*4
					o11 := o10 + 4
					if o00 < 0 || o11 >= len(pix) {
						return fmt.Errorf("decodeChannel: index out of range")
					}

					if bitType {
						if bgStream.i >= bgStream.n {
							return fmt.Errorf("decodeChannel: BG stream truncated")
						}
						bg := bgStream.nextFast()

						bits, err := patternBR.readBits(4)
						if err != nil {
							return err
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
					} else {
						pix[o00] = fg
						pix[o01] = fg
						pix[o10] = fg
						pix[o11] = fg
					}

					blockIndex++
					continue
				}

				_ = typeBR.readBitsFast(4) // four 1x1 blocks; type bits are all 0

				v0 := fgStream.nextFast()
				v1 := fgStream.nextFast()
				v2 := fgStream.nextFast()
				v3 := fgStream.nextFast()

				o00 := row0 + mx*4
				o01 := o00 + 4
				o10 := row1 + mx*4
				o11 := o10 + 4
				if o00 < 0 || o11 >= len(pix) {
					return fmt.Errorf("decodeChannel: index out of range")
				}
				pix[o00] = v0
				pix[o01] = v1
				pix[o10] = v2
				pix[o11] = v3

				blockIndex += 4
			}
		}

		// right stripe: 1x1 blocks only
		for my := 0; my < fullH; my++ {
			row := my*strideBytes + channelOffset
			for mx := fullW; mx < w4; mx++ {
				_ = typeBR.readBitFast()
				v := fgStream.nextFast()

				o := row + mx*4
				if o < 0 || o >= len(pix) {
					return fmt.Errorf("decodeChannel: index out of range")
				}
				pix[o] = v
				blockIndex++
			}
		}

		// bottom stripe: 1x1 blocks only (including bottom-right corner)
		for my := fullH; my < h4; my++ {
			row := my*strideBytes + channelOffset
			for mx := 0; mx < w4; mx++ {
				_ = typeBR.readBitFast()
				v := fgStream.nextFast()

				o := row + mx*4
				if o < 0 || o >= len(pix) {
					return fmt.Errorf("decodeChannel: index out of range")
				}
				pix[o] = v
				blockIndex++
			}
		}

		if blockIndex != int(blockCount) {
			return fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
		}
		if fgStream.i != fgStream.n {
			return fmt.Errorf("color stream mismatch: fg used=%d expected=%d", fgStream.i, fgStream.n)
		}
		if bgStream.i != bgStream.n {
			return fmt.Errorf("color stream mismatch: bg used=%d expected=%d", bgStream.i, bgStream.n)
		}
		return nil
	}

	for my := 0; my < fullH; my += macroBlock {
		for mx := 0; mx < fullW; mx += macroBlock {
			if blockIndex >= int(blockCount) {
				return fmt.Errorf("unexpected end of blocks in main area")
			}

			macroIsBig, err := sizeBR.readBit()
			if err != nil {
				return err
			}

			if useMacro && macroIsBig {
				bitType, err := typeBR.readBit()
				if err != nil {
					return err
				}
				fg := fgStream.nextFast()
				if bitType {
					var bg uint8
					if bgPerPattern {
						if bgStream.i >= bgStream.n {
							return fmt.Errorf("decodeChannel: BG stream truncated")
						}
						bg = bgStream.nextFast()
					} else {
						bg = bgStream.nextFast()
					}
					if err := drawBlockPix(pix, strideBytes, mx, my, macroBlock, macroBlock, &patternBR, fg, bg, channelOffset); err != nil {
						return err
					}
				} else {
					if !bgPerPattern {
						_ = bgStream.nextFast()
					}
					if err := fillBlockPix(pix, strideBytes, mx, my, macroBlock, macroBlock, fg, channelOffset); err != nil {
						return err
					}
				}
				blockIndex++
				continue
			}

			for by := 0; by < macroBlock; by += smallBlock {
				for bx := 0; bx < macroBlock; bx += smallBlock {
					if blockIndex >= int(blockCount) {
						return fmt.Errorf("unexpected end of blocks in macro grid")
					}
					bitType, err := typeBR.readBit()
					if err != nil {
						return err
					}
					fg := fgStream.nextFast()
					if bitType {
						var bg uint8
						if bgPerPattern {
							if bgStream.i >= bgStream.n {
								return fmt.Errorf("decodeChannel: BG stream truncated")
							}
							bg = bgStream.nextFast()
						} else {
							bg = bgStream.nextFast()
						}
						if err := drawBlockPix(pix, strideBytes, mx+bx, my+by, smallBlock, smallBlock, &patternBR, fg, bg, channelOffset); err != nil {
							return err
						}
					} else {
						if !bgPerPattern {
							_ = bgStream.nextFast()
						}
						if err := fillBlockPix(pix, strideBytes, mx+bx, my+by, smallBlock, smallBlock, fg, channelOffset); err != nil {
							return err
						}
					}
					blockIndex++
				}
			}
		}
	}

	// right stripe: small blocks only
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return fmt.Errorf("unexpected end of blocks in right stripe")
			}
			bitType, err := typeBR.readBit()
			if err != nil {
				return err
			}
			fg := fgStream.nextFast()
			if bitType {
				var bg uint8
				if bgPerPattern {
					if bgStream.i >= bgStream.n {
						return fmt.Errorf("decodeChannel: BG stream truncated")
					}
					bg = bgStream.nextFast()
				} else {
					bg = bgStream.nextFast()
				}
				if err := drawBlockPix(pix, strideBytes, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, channelOffset); err != nil {
					return err
				}
			} else {
				if !bgPerPattern {
					_ = bgStream.nextFast()
				}
				if err := fillBlockPix(pix, strideBytes, mx, my, smallBlock, smallBlock, fg, channelOffset); err != nil {
					return err
				}
			}
			blockIndex++
		}
	}

	// bottom stripe: small blocks only (including bottom-right corner)
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			if blockIndex >= int(blockCount) {
				return fmt.Errorf("unexpected end of blocks in bottom stripe")
			}
			bitType, err := typeBR.readBit()
			if err != nil {
				return err
			}
			fg := fgStream.nextFast()
			if bitType {
				var bg uint8
				if bgPerPattern {
					if bgStream.i >= bgStream.n {
						return fmt.Errorf("decodeChannel: BG stream truncated")
					}
					bg = bgStream.nextFast()
				} else {
					bg = bgStream.nextFast()
				}
				if err := drawBlockPix(pix, strideBytes, mx, my, smallBlock, smallBlock, &patternBR, fg, bg, channelOffset); err != nil {
					return err
				}
			} else {
				if !bgPerPattern {
					_ = bgStream.nextFast()
				}
				if err := fillBlockPix(pix, strideBytes, mx, my, smallBlock, smallBlock, fg, channelOffset); err != nil {
					return err
				}
			}
			blockIndex++
		}
	}

	if blockIndex != int(blockCount) {
		return fmt.Errorf("block count mismatch: used %d of %d", blockIndex, blockCount)
	}
	if fgStream.i != fgStream.n {
		return fmt.Errorf("color stream mismatch: fg used=%d expected=%d", fgStream.i, fgStream.n)
	}
	if bgStream.i != bgStream.n {
		return fmt.Errorf("color stream mismatch: bg used=%d expected=%d", bgStream.i, bgStream.n)
	}
	return nil
}

func (d *Decoder) decodeChannelInto(data []byte, imgW, imgH int, scratch *decoderChannelScratch) ([]uint8, error) {
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

	// FG: delta-coded bytes, one value per block.
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

	// BG: delta-coded bytes, one value per pattern block (type bit = 1).
	bgPerPattern := true
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

	// reconstruct the block geometry and fill the plane
	needPlane := imgW * imgH
	if cap(scratch.plane) < needPlane {
		scratch.plane = make([]uint8, needPlane)
	} else {
		scratch.plane = scratch.plane[:needPlane]
	}
	plane := scratch.plane

	w4 := (imgW / smallBlock) * smallBlock
	h4 := (imgH / smallBlock) * smallBlock
	fullW := (w4 / macroBlock) * macroBlock
	fullH := (h4 / macroBlock) * macroBlock
	macroGroupCount := (fullW / macroBlock) * (fullH / macroBlock)
	useMacro := macroBlock > smallBlock

	if macroGroupCount > 0 {
		if cap(scratch.macroBig) < macroGroupCount {
			scratch.macroBig = make([]bool, macroGroupCount)
		} else {
			scratch.macroBig = scratch.macroBig[:macroGroupCount]
		}
	} else {
		scratch.macroBig = scratch.macroBig[:0]
	}
	macroBig := scratch.macroBig

	// rebuild macro-block size decisions from the size stream
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

func (d *Decoder) Decode(compData []byte, postfilter bool) (*image.RGBA, error) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()

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

	// magic
	if string(payload[pos:pos+len(codec)]) != codec {
		return nil, fmt.Errorf("bad magic: %q", string(payload[pos:pos+len(codec)]))
	}
	pos += len(codec)

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

	bwSize, err := readU16("block width")
	if err != nil {
		return nil, err
	}
	bhSize, err := readU16("block height")
	if err != nil {
		return nil, err
	}
	if bwSize == 0 || bhSize == 0 {
		return nil, fmt.Errorf("invalid block sizes in header: %dx%d", bwSize, bhSize)
	}
	smallBlock = int(bwSize)
	macroBlock = int(bhSize)

	// channels mask: which Y/Cb/Cr planes are stored.
	if len(payload)-pos < 1 {
		return nil, fmt.Errorf("decode: truncated while reading channels mask")
	}
	channelsMask := payload[pos]
	pos++
	if channelsMask&channelFlagY == 0 {
		return nil, fmt.Errorf("decode: Y channel missing in header")
	}

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

	if macroBlock < smallBlock || macroBlock%smallBlock != 0 {
		return nil, fmt.Errorf("macroBlock (%d) must be >= smallBlock (%d) and a multiple of it",
			macroBlock, smallBlock)
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
	if hasCb {
		cbSeg, err = readChannelSegment(payload, &pos)
		if err != nil {
			return nil, err
		}
	}
	if hasCr {
		crSeg, err = readChannelSegment(payload, &pos)
		if err != nil {
			return nil, err
		}
	}

	w4 := (imgW / smallBlock) * smallBlock
	h4 := (imgH / smallBlock) * smallBlock
	if w4 != imgW || h4 != imgH {
		for o := 0; o+3 < len(pix); o += 4 {
			pix[o+0] = 0
			pix[o+1] = 128
			pix[o+2] = 128
			pix[o+3] = 255
		}
	}

	var errY, errCb, errCr error
	if d.Parallel {
		var wg sync.WaitGroup
		wg.Add(1)
		go decodeChannelToPixWorker(ySeg, imgW, imgH, pix, stride, 0, &errY, &wg)
		if hasCb {
			wg.Add(1)
			go decodeChannelToPixWorker(cbSeg, imgW, imgH, pix, stride, 1, &errCb, &wg)
		}
		if hasCr {
			wg.Add(1)
			go decodeChannelToPixWorker(crSeg, imgW, imgH, pix, stride, 2, &errCr, &wg)
		}
		wg.Wait()
	} else {
		errY = decodeChannelToPix(ySeg, imgW, imgH, pix, stride, 0)
		if hasCb {
			errCb = decodeChannelToPix(cbSeg, imgW, imgH, pix, stride, 1)
		}
		if hasCr {
			errCr = decodeChannelToPix(crSeg, imgW, imgH, pix, stride, 2)
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

	if d.Parallel {
		workers := max(min(runtime.NumCPU(), imgH), 1)
		rowsPerWorker := (imgH + workers - 1) / workers

		var wgRGB sync.WaitGroup
		for i := range workers {
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

	if postfilter {
		return smoothBlocks(dst), nil
	}

	return dst, nil
}

// DecodeFrom reads compressed data from r and decodes it.
// This mirrors codecs that accept io.Reader/io.Writer and is useful for
// benchmarking. It allocates to read the full input; for zero-copy decoding,
// prefer Decode([]byte,...).
func (d *Decoder) DecodeFrom(r io.Reader, postfilter bool) (*image.RGBA, error) {
	compData, err := io.ReadAll(r)
	if err != nil {
		return nil, err
	}
	return d.Decode(compData, postfilter)
}

func decodeChannelToPixWorker(data []byte, imgW, imgH int, pix []byte, strideBytes int, channelOffset int, dstErr *error, wg *sync.WaitGroup) {
	defer wg.Done()
	*dstErr = decodeChannelToPix(data, imgW, imgH, pix, strideBytes, channelOffset)
}

func ycbcrToRGB(pix []byte, stride, imgW int, yStart, yEnd int, hasCb, hasCr bool) {
	if hasCb && hasCr {
		for y := yStart; y < yEnd; y++ {
			rowOff := y * stride
			for x := 0; x < imgW; x++ {
				o := rowOff + x*4
				Y := int32(pix[o+0])
				Cb := int32(pix[o+1]) - 128
				Cr := int32(pix[o+2]) - 128

				R := Y + ((91881 * Cr) >> 16)
				G := Y - ((22554*Cb + 46802*Cr) >> 16)
				B := Y + ((116130 * Cb) >> 16)

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
				Y := int32(pix[o+0])
				Cb := int32(pix[o+1]) - 128

				R := Y
				G := Y - ((22554 * Cb) >> 16)
				B := Y + ((116130 * Cb) >> 16)

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
				Y := int32(pix[o+0])
				Cr := int32(pix[o+2]) - 128

				R := Y + ((91881 * Cr) >> 16)
				G := Y - ((46802 * Cr) >> 16)
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
func Decode(compData []byte, postfilter bool) (image.Image, error) {
	d := NewDecoder()
	return d.Decode(compData, postfilter)
}

// smoothJunctions performs gradient-based smoothing at intersections of small blocks.
// At each internal junction of the smallBlock grid, it inspects the four corner pixels
// and, if they are luminance-wise similar, fills a small rectangle around the junction
// with a bilinear gradient between the four corner colors.
func smoothJunctions(img *image.RGBA) *image.RGBA {
	b := img.Bounds()
	w, h := b.Dx(), b.Dy()

	// need at least 2x2 blocks to have internal junctions
	if w < 2*smallBlock || h < 2*smallBlock {
		return img
	}

	maxRadius := min(smallBlock, 3)
	const junctionLumaSpread int32 = 32
	const lumaSimThreshold int32 = 8

	minX := b.Min.X
	minY := b.Min.Y
	stride := img.Stride
	srcPix := img.Pix

	// Work on a copy so that all reads come from the original image
	// and only writes go to the destination buffer.
	dst := image.NewRGBA(b)
	copy(dst.Pix, img.Pix)
	dstPix := dst.Pix

	// helper to compute luma from 8-bit RGB directly
	luma8 := func(r, g, b uint8) int32 {
		rr := int32(r)
		gg := int32(g)
		bb := int32(b)
		// Integer approximation of ITU-R BT.601:
		// Y ≈ 0.299 R + 0.587 G + 0.114 B
		return (299*rr + 587*gg + 114*bb) / 1000
	}

	similar := func(r1, g1, b1, a1, r2, g2, b2, a2 uint8) bool {
		da := luma8(r1, g1, b1) - luma8(r2, g2, b2)
		if da < 0 {
			da = -da
		}
		return da <= lumaSimThreshold
	}

	lerp8 := func(a, b uint8, t float64) uint8 {
		return uint8(float64(a)*(1-t) + float64(b)*t + 0.5)
	}

	// Parallelise over horizontal stripes. Each worker owns a disjoint band
	// of rows in the destination image. Writes are clamped to the worker's
	// stripe to avoid concurrent writes to the same pixels.
	workers := max(min(runtime.NumCPU(), h), 1)
	rowsPerWorker := (h + workers - 1) / workers

	var wg sync.WaitGroup
	for i := range workers {
		stripeStart := b.Min.Y + i*rowsPerWorker
		if stripeStart >= b.Max.Y {
			break
		}
		stripeEnd := min(stripeStart+rowsPerWorker, b.Max.Y)

		wg.Add(1)
		go func(yStripeStart, yStripeEnd int) {
			defer wg.Done()

			for y := b.Min.Y + smallBlock; y < b.Max.Y; y += smallBlock {
				// Only process junctions whose center row lies inside this stripe.
				if y < yStripeStart || y >= yStripeEnd {
					continue
				}

				for x := b.Min.X + smallBlock; x < b.Max.X; x += smallBlock {
					// four corners around the junction, read from the original image
					idx00 := (y-1-minY)*stride + (x-1-minX)*4 // top-left
					idx10 := (y-1-minY)*stride + (x-minX)*4   // top-right
					idx01 := (y-minY)*stride + (x-1-minX)*4   // bottom-left
					idx11 := (y-minY)*stride + (x-minX)*4     // bottom-right

					r00, g00, b00, a00 := srcPix[idx00+0], srcPix[idx00+1], srcPix[idx00+2], srcPix[idx00+3]
					r10, g10, b10, a10 := srcPix[idx10+0], srcPix[idx10+1], srcPix[idx10+2], srcPix[idx10+3]
					r01, g01, b01, a01 := srcPix[idx01+0], srcPix[idx01+1], srcPix[idx01+2], srcPix[idx01+3]
					r11, g11, b11, a11 := srcPix[idx11+0], srcPix[idx11+1], srcPix[idx11+2], srcPix[idx11+3]

					l00 := luma8(r00, g00, b00)
					l10 := luma8(r10, g10, b10)
					l01 := luma8(r01, g01, b01)
					l11 := luma8(r11, g11, b11)

					minL, maxL := l00, l00
					for _, v := range []int32{l10, l01, l11} {
						if v < minL {
							minL = v
						}
						if v > maxL {
							maxL = v
						}
					}
					// if the corner luminance differs too much, treat it as a hard edge and skip
					if maxL-minL > junctionLumaSpread {
						continue
					}

					// grow small rectangle around the junction while colors remain similar
					L := 0
					for i := 1; i <= maxRadius && x-i >= b.Min.X; i++ {
						idxUp := (y-1-minY)*stride + (x-i-minX)*4
						idxDown := (y-minY)*stride + (x-i-minX)*4
						rUp, gUp, bUp, aUp := srcPix[idxUp+0], srcPix[idxUp+1], srcPix[idxUp+2], srcPix[idxUp+3]
						rDown, gDown, bDown, aDown := srcPix[idxDown+0], srcPix[idxDown+1], srcPix[idxDown+2], srcPix[idxDown+3]
						if !similar(rUp, gUp, bUp, aUp, r00, g00, b00, a00) || !similar(rDown, gDown, bDown, aDown, r01, g01, b01, a01) {
							break
						}
						L = i
					}

					R := 0
					for i := 1; i <= maxRadius && x-1+i < b.Max.X; i++ {
						idxUp := (y-1-minY)*stride + (x-1+i-minX)*4
						idxDown := (y-minY)*stride + (x-1+i-minX)*4
						rUp, gUp, bUp, aUp := srcPix[idxUp+0], srcPix[idxUp+1], srcPix[idxUp+2], srcPix[idxUp+3]
						rDown, gDown, bDown, aDown := srcPix[idxDown+0], srcPix[idxDown+1], srcPix[idxDown+2], srcPix[idxDown+3]
						if !similar(rUp, gUp, bUp, aUp, r10, g10, b10, a10) || !similar(rDown, gDown, bDown, aDown, r11, g11, b11, a11) {
							break
						}
						R = i
					}

					U := 0
					for i := 1; i <= maxRadius && y-i >= b.Min.Y; i++ {
						idxLeft := (y-i-minY)*stride + (x-1-minX)*4
						idxRight := (y-i-minY)*stride + (x-minX)*4
						rLeft, gLeft, bLeft, aLeft := srcPix[idxLeft+0], srcPix[idxLeft+1], srcPix[idxLeft+2], srcPix[idxLeft+3]
						rRight, gRight, bRight, aRight := srcPix[idxRight+0], srcPix[idxRight+1], srcPix[idxRight+2], srcPix[idxRight+3]
						if !similar(rLeft, gLeft, bLeft, aLeft, r00, g00, b00, a00) || !similar(rRight, gRight, bRight, aRight, r10, g10, b10, a10) {
							break
						}
						U = i
					}

					D := 0
					for i := 1; i <= maxRadius && y-1+i < b.Max.Y; i++ {
						idxLeft := (y-1+i-minY)*stride + (x-1-minX)*4
						idxRight := (y-1+i-minY)*stride + (x-minX)*4
						rLeft, gLeft, bLeft, aLeft := srcPix[idxLeft+0], srcPix[idxLeft+1], srcPix[idxLeft+2], srcPix[idxLeft+3]
						rRight, gRight, bRight, aRight := srcPix[idxRight+0], srcPix[idxRight+1], srcPix[idxRight+2], srcPix[idxRight+3]
						if !similar(rLeft, gLeft, bLeft, aLeft, r01, g01, b01, a01) || !similar(rRight, gRight, bRight, aRight, r11, g11, b11, a11) {
							break
						}
						D = i
					}

					if L == 0 || R == 0 || U == 0 || D == 0 {
						continue
					}

					rectMinX := x - L
					rectMaxX := x - 1 + R
					rectMinY := y - U
					rectMaxY := y - 1 + D

					if rectMinX < b.Min.X || rectMaxX >= b.Max.X || rectMinY < b.Min.Y || rectMaxY >= b.Max.Y {
						continue
					}
					if rectMaxX <= rectMinX || rectMaxY <= rectMinY {
						continue
					}

					// Clamp vertical extent to the current worker's stripe so that
					// no two goroutines write to the same rows.
					if rectMaxY < yStripeStart || rectMinY >= yStripeEnd {
						continue
					}
					if rectMinY < yStripeStart {
						rectMinY = yStripeStart
					}
					if rectMaxY >= yStripeEnd {
						rectMaxY = yStripeEnd - 1
					}

					width := rectMaxX - rectMinX
					height := rectMaxY - rectMinY

					for py := rectMinY; py <= rectMaxY; py++ {
						v := float64(py-rectMinY) / float64(height)
						rowOff := (py - minY) * stride
						for px := rectMinX; px <= rectMaxX; px++ {
							u := float64(px-rectMinX) / float64(width)

							rTop := lerp8(r00, r10, u)
							gTop := lerp8(g00, g10, u)
							bTop := lerp8(b00, b10, u)
							aTop := lerp8(a00, a10, u)

							rBot := lerp8(r01, r11, u)
							gBot := lerp8(g01, g11, u)
							bBot := lerp8(b01, b11, u)
							aBot := lerp8(a01, a11, u)

							r := lerp8(rTop, rBot, v)
							g := lerp8(gTop, gBot, v)
							bc := lerp8(bTop, bBot, v)
							a := lerp8(aTop, aBot, v)

							idx := rowOff + (px-minX)*4
							dstPix[idx+0] = r
							dstPix[idx+1] = g
							dstPix[idx+2] = bc
							dstPix[idx+3] = a
						}
					}
				}
			}
		}(stripeStart, stripeEnd)
	}
	wg.Wait()

	return dst
}

// smoothFlatAreas performs a light deblocking pass:
// it smooths only along block boundaries where the luminance difference is small.
func smoothFlatAreas(src *image.RGBA) *image.RGBA {
	b := src.Bounds()
	w, h := b.Dx(), b.Dy()

	// if the image is too small for at least two blocks in each direction, just return it as-is
	if w < 2*smallBlock || h < 2*smallBlock {
		return src
	}

	srcPix := src.Pix
	stride := src.Stride

	// threshold on luminance difference across a block boundary
	// smaller values -> less smoothing
	const boundaryLumaThreshold int32 = 10

	// helper to compute luma from 8-bit RGB directly
	luma8 := func(r, g, b uint8) int32 {
		rr := int32(r)
		gg := int32(g)
		bb := int32(b)
		// Integer approximation of ITU-R BT.601:
		// Y ≈ 0.299 R + 0.587 G + 0.114 B
		return (299*rr + 587*gg + 114*bb) / 1000
	}

	minX := b.Min.X
	minY := b.Min.Y

	// vertical block boundaries: x = k * smallBlock
	workersV := min(max(runtime.NumCPU(), 1), h)
	rowsPerWorker := (h + workersV - 1) / workersV

	var wgV sync.WaitGroup
	for i := range workersV {
		y0 := b.Min.Y + i*rowsPerWorker
		if y0 >= b.Max.Y {
			break
		}
		y1 := y0 + rowsPerWorker
		if y1 > b.Max.Y {
			y1 = b.Max.Y
		}

		wgV.Add(1)
		go func(yStart, yEnd int) {
			defer wgV.Done()
			for y := yStart; y < yEnd; y++ {
				rowOff := (y - minY) * stride
				for x := b.Min.X + smallBlock; x < b.Max.X; x += smallBlock {
					xL := x - 1
					idxL := rowOff + (xL-minX)*4
					idxR := rowOff + (x-minX)*4

					rL := srcPix[idxL+0]
					gL := srcPix[idxL+1]
					bL := srcPix[idxL+2]
					aL := srcPix[idxL+3]

					rR := srcPix[idxR+0]
					gR := srcPix[idxR+1]
					bR := srcPix[idxR+2]
					aR := srcPix[idxR+3]

					lL := luma8(rL, gL, bL)
					lR := luma8(rR, gR, bR)
					d := lL - lR
					if d < 0 {
						d = -d
					}

					// if the two sides are similar in luminance, slightly blur across the boundary
					if d <= boundaryLumaThreshold {
						r := uint8((uint16(rL) + uint16(rR)) / 2)
						g := uint8((uint16(gL) + uint16(gR)) / 2)
						bc := uint8((uint16(bL) + uint16(bR)) / 2)
						a := uint8((uint16(aL) + uint16(aR)) / 2)

						srcPix[idxL+0] = r
						srcPix[idxL+1] = g
						srcPix[idxL+2] = bc
						srcPix[idxL+3] = a

						srcPix[idxR+0] = r
						srcPix[idxR+1] = g
						srcPix[idxR+2] = bc
						srcPix[idxR+3] = a
					}
				}
			}
		}(y0, y1)
	}
	wgV.Wait()

	// horizontal block boundaries: y = k * smallBlock
	workersH := min(max(runtime.NumCPU(), 1), w)
	colsPerWorker := (w + workersH - 1) / workersH

	var wgH sync.WaitGroup
	for i := range workersH {
		x0 := b.Min.X + i*colsPerWorker
		if x0 >= b.Max.X {
			break
		}
		x1 := min(x0+colsPerWorker, b.Max.X)

		wgH.Add(1)
		go func(xStart, xEnd int) {
			defer wgH.Done()
			for x := xStart; x < xEnd; x++ {
				colOff := (x - minX) * 4
				for y := b.Min.Y + smallBlock; y < b.Max.Y; y += smallBlock {
					yT := y - 1
					idxT := (yT-minY)*stride + colOff
					idxB := (y-minY)*stride + colOff

					rT := srcPix[idxT+0]
					gT := srcPix[idxT+1]
					bT := srcPix[idxT+2]
					aT := srcPix[idxT+3]

					rB := srcPix[idxB+0]
					gB := srcPix[idxB+1]
					bB := srcPix[idxB+2]
					aB := srcPix[idxB+3]

					lT := luma8(rT, gT, bT)
					lB := luma8(rB, gB, bB)
					d := lT - lB
					if d < 0 {
						d = -d
					}

					if d <= boundaryLumaThreshold {
						r := uint8((uint16(rT) + uint16(rB)) / 2)
						g := uint8((uint16(gT) + uint16(gB)) / 2)
						bc := uint8((uint16(bT) + uint16(bB)) / 2)
						a := uint8((uint16(aT) + uint16(aB)) / 2)

						srcPix[idxT+0] = r
						srcPix[idxT+1] = g
						srcPix[idxT+2] = bc
						srcPix[idxT+3] = a

						srcPix[idxB+0] = r
						srcPix[idxB+1] = g
						srcPix[idxB+2] = bc
						srcPix[idxB+3] = a
					}
				}
			}
		}(x0, x1)
	}
	wgH.Wait()

	return src
}

// smoothBlocks runs both junction smoothing and light deblocking in a fixed order.
// This keeps the post-process logic in one place for Decode().
func smoothBlocks(src *image.RGBA) *image.RGBA {
	// If we are operating at the finest granularity (1x1 blocks),
	// there are no visible block boundaries to smooth.
	if smallBlock <= 1 {
		return src
	}
	// first apply light deblocking along block boundaries,
	// then apply gradient smoothing at block junctions.
	return smoothJunctions(smoothFlatAreas(src))
}

type yuv struct {
	Y  int16
	Cb int16
	Cr int16
}

func rgbToYCbCr(r, g, b uint8) yuv {
	rr, gg, bb := int32(r), int32(g), int32(b)
	Y := (77*rr + 150*gg + 29*bb) >> 8 // ≈ 0.299 0.587 0.114
	Cb := ((-43*rr - 85*gg + 128*bb) >> 8) + 128
	Cr := ((128*rr - 107*gg - 21*bb) >> 8) + 128
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

var zstdEncPool = sync.Pool{
	New: func() any {
		return mustNewZstdEncoder()
	},
}

var zstdDecPool = sync.Pool{
	New: func() any {
		return mustNewZstdDecoder()
	},
}

func compressZstd(data []byte) ([]byte, error) {
	if len(data) == 0 {
		return data, nil
	}

	enc := zstdEncPool.Get().(*zstd.Encoder)
	out := enc.EncodeAll(data, nil)
	zstdEncPool.Put(enc)
	return out, nil
}

func compressZstdInto(dst []byte, data []byte) ([]byte, error) {
	if len(data) == 0 {
		return data, nil
	}

	enc := zstdEncPool.Get().(*zstd.Encoder)
	out := enc.EncodeAll(data, dst[:0])
	zstdEncPool.Put(enc)
	return out, nil
}

func decompressZstd(data []byte) ([]byte, error) {
	if len(data) == 0 {
		return data, nil
	}

	dec := zstdDecPool.Get().(*zstd.Decoder)
	out, err := dec.DecodeAll(data, nil)
	zstdDecPool.Put(dec)
	return out, err
}

func decompressZstdInto(dst []byte, data []byte) ([]byte, error) {
	if len(data) == 0 {
		return data, nil
	}

	dec := zstdDecPool.Get().(*zstd.Decoder)
	out, err := dec.DecodeAll(data, dst[:0])
	zstdDecPool.Put(dec)
	return out, err
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
func writeDeltaPackedBytes(w *bufio.Writer, src []uint8) error {
	n := len(src)
	if n == 0 {
		return nil
	}
	if err := w.WriteByte(src[0]); err != nil {
		return err
	}

	prev := src[0]
	i := 1

	var buf [4096]byte
	for i < n {
		j := 0
		for j < len(buf) && i < n {
			d := encodeDelta8(prev, src[i])
			buf[j] = byte(d)
			prev = src[i]
			i++
			j++
		}
		if _, err := w.Write(buf[:j]); err != nil {
			return err
		}
	}
	return nil
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

// deltaPackBytes encodes a byte slice using circular delta coding on 0..255.
// The output layout is:
//
//	[0]       = first byte (if len(src) > 0)
//	[1..]     = (len(src)-1) delta bytes (encodeDelta8(prev, curr))
//
// The caller is responsible for storing/transmitting the original length.
func deltaPackBytes(src []byte) ([]byte, error) {
	n := len(src)
	if n == 0 {
		return []byte{}, nil
	}

	out := make([]byte, n)
	out[0] = src[0]

	prev := src[0]

	for i := 1; i < n; i++ {
		d := encodeDelta8(prev, src[i])
		out[i] = byte(d)
		prev = src[i]
	}

	return out, nil
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

// nextFast is a no-error variant of next. The caller must ensure the stream
// has at least one remaining value.
func (ds *deltaStream) nextFast() byte {
	if ds.i == 0 {
		ds.i = 1
		return ds.prev
	}
	d := int8(ds.packed[ds.i])
	ds.prev = decodeDelta8(ds.prev, d)
	ds.i++
	return ds.prev
}
