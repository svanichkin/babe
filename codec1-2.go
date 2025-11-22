// BABE (Bi-Level Adaptive Block Encoding) is a dual-tone block codec for images.
// It uses adaptive block sizes (small and macro blocks), a palette in YCbCr
// color space, delta-coded palette indices, and a light post-process for deblocking
// and gradient smoothing.

package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	_ "image/gif"
	_ "image/jpeg"
	"io"
	"sync"

	"github.com/klauspost/compress/zstd"
)

const (
	codec = "BABE\n"
)

// Default block sizes; these are overridden by compression presets via setBlocksForQuality.
var (
	// base size of the small block (in pixels)
	smallBlock = 1
	// size of the macroblock (in pixels)
	macroBlock = 2
)

// current encode quality in [0..100]; used to drive macro/small decisions more aggressively.
var encQuality int = 100

// Quality mapping:
// - quality is expected in [0..100]
// - smallBlock is fixed to 1
// - macroBlock goes linearly from 100 (at quality=0) down to 2 (at quality=100)

// setBlocksForQuality configures global smallBlock/macroBlock based on a quality in [0..100].
// We use a small set of discrete presets:
//
//	quality ≈100% → 1/2
//	mid-high      → 1/3
//	mid           → 2/4
//	mid-low       → 3/6
//	low (0%)      → 4/8
func setBlocksForQuality(quality int) error {
	// clamp quality to [0..100]
	if quality < 0 {
		quality = 0
	}
	if quality > 100 {
		quality = 100
	}

	// remember quality globally so heuristics (macro vs small) can use it directly
	encQuality = quality

	switch {
	case quality >= 80:
		// highest quality: smallest macro-blocks
		smallBlock = 1
		macroBlock = 2
	case quality >= 60:
		smallBlock = 1
		macroBlock = 3
	case quality >= 40:
		smallBlock = 2
		macroBlock = 4
	case quality >= 20:
		smallBlock = 3
		macroBlock = 6
	default:
		// lowest quality / highest compression
		smallBlock = 4
		macroBlock = 8
	}

	fmt.Printf("QUALITY: %d, smallBlock=%d, macroBlock=%d\n",
		encQuality, smallBlock, macroBlock)

	return nil
}

// BitWriter writes bits to an io.ByteWriter (e.g., *bytes.Buffer).
type BitWriter struct {
	w    io.ByteWriter
	byte byte
	n    uint8 // number of bits written (0..8)
}

func NewBitWriter(w io.ByteWriter) *BitWriter {
	return &BitWriter{w: w}
}

// WriteBit writes a single bit (msb-first in byte).
func (bw *BitWriter) WriteBit(bit bool) error {
	bw.byte <<= 1
	if bit {
		bw.byte |= 1
	}
	bw.n++
	if bw.n == 8 {
		if err := bw.w.WriteByte(bw.byte); err != nil {
			return err
		}
		bw.byte = 0
		bw.n = 0
	}
	return nil
}

// Flush writes any remaining bits, left-padded with zeros.
func (bw *BitWriter) Flush() error {
	if bw.n > 0 {
		bw.byte <<= 8 - bw.n
		if err := bw.w.WriteByte(bw.byte); err != nil {
			return err
		}
		bw.byte = 0
		bw.n = 0
	}
	return nil
}

// BitReader reads bits from a byte slice (msb-first in each byte).
type BitReader struct {
	data []byte
	idx  int
	bit  uint8 // bit position in current byte (0..7), msb-first
}

func NewBitReader(data []byte) *BitReader {
	return &BitReader{data: data, idx: 0, bit: 0}
}

// ReadBit returns the next bit, or error if out of data.
func (br *BitReader) ReadBit() (bool, error) {
	if br.idx >= len(br.data) {
		return false, io.EOF
	}
	b := br.data[br.idx]
	isSet := (b & (1 << (7 - br.bit))) != 0
	br.bit++
	if br.bit == 8 {
		br.bit = 0
		br.idx++
	}
	return isSet, nil
}

// blockMetaCh describes a single block for one channel (Y, Cb, or Cr).
// Geometry is implicit from the traversal order, but we keep a flag to distinguish
// macroBlocks from smallBlocks so we can encode FG/BG differently.
type blockMetaCh struct {
	fg, bg uint8
	isBig  bool // true for macroBlock-sized blocks, false for smallBlock
}

// channel IDs for Y, Cb, Cr.
const (
	chY  = 0
	chCb = 1
	chCr = 2
)

// channelValue extracts the selected channel from a YCbCr triplet.
func channelValue(ycc yuv, ch int) uint8 {
	switch ch {
	case chY:
		return uint8(ycc.Y)
	case chCb:
		return uint8(ycc.Cb)
	case chCr:
		return uint8(ycc.Cr)
	default:
		return uint8(ycc.Y)
	}
}

// canUseBigBlockChannel decides whether a macroBlock region can be encoded as a single block
// for the given channel. It uses a quality-dependent spread threshold: lower quality allows
// larger spread (more macroBlocks), higher quality reduces spread (more small blocks).
func canUseBigBlockChannel(img image.Image, ch, x0, y0 int) bool {
	b := img.Bounds()
	if x0+macroBlock > b.Max.X || y0+macroBlock > b.Max.Y {
		return false
	}

	var minV, maxV int32
	first := true
	for yy := 0; yy < macroBlock; yy++ {
		for xx := 0; xx < macroBlock; xx++ {
			c := img.At(x0+xx, y0+yy)
			r16, g16, b16, _ := c.RGBA()
			r8 := uint8(r16 >> 8)
			g8 := uint8(g16 >> 8)
			b8 := uint8(b16 >> 8)
			ycc := rgbToYCbCr(r8, g8, b8)
			v := int32(channelValue(ycc, ch))
			if first {
				minV, maxV = v, v
				first = false
			} else {
				if v < minV {
					minV = v
				}
				if v > maxV {
					maxV = v
				}
			}
		}
	}

	// quality-dependent spread:
	// в КАЖДОМ диапазоне качества (0–19, 20–39, 40–59, 60–79, 80–100),
	// которые мы используем для подбора smallBlock/macroBlock,
	// spread меняется линейно от 64 (внизу диапазона) до 8 (вверху диапазона).
	q := encQuality
	if q < 0 {
		q = 0
	}
	if q > 100 {
		q = 100
	}

	// подобрать границы диапазона качества в точности как в setBlocksForQuality:
	//   [0–19]  → 4/8
	//   [20–39] → 3/6
	//   [40–59] → 2/4
	//   [60–79] → 1/3
	//   [80–100]→ 1/2
	var groupStart, groupEnd int
	switch {
	case q < 20:
		groupStart, groupEnd = 0, 19
	case q < 40:
		groupStart, groupEnd = 20, 39
	case q < 60:
		groupStart, groupEnd = 40, 59
	case q < 80:
		groupStart, groupEnd = 60, 79
	default:
		groupStart, groupEnd = 80, 100
	}

	// линейная интерполяция от 64 до 8 внутри выбранного диапазона
	const spreadMax int32 = 64
	const spreadMin int32 = 8
	span := groupEnd - groupStart
	var spread int32
	if span <= 0 {
		spread = (spreadMax + spreadMin) / 2
	} else {
		offset := q - groupStart
		spread = spreadMax - int32(offset)*(spreadMax-spreadMin)/int32(span)
	}

	return (maxV - minV) < spread
}

// encodeBlockChannel encodes a single block for one channel:
// it computes a mean-based threshold, writes the FG/BG pattern bits, and returns FG/BG levels.
func encodeBlockChannel(img image.Image, ch, x0, y0, bw, bh int, pw *BitWriter) (uint8, uint8, error) {
	b := img.Bounds()
	if x0 < b.Min.X || y0 < b.Min.Y || x0+bw > b.Max.X || y0+bh > b.Max.Y {
		return 0, 0, fmt.Errorf("block out of bounds")
	}
	total := bw * bh
	if total <= 0 {
		return 0, 0, fmt.Errorf("invalid block size")
	}

	// first pass: compute average for the chosen channel
	var sum uint64
	for yy := 0; yy < bh; yy++ {
		for xx := 0; xx < bw; xx++ {
			c := img.At(x0+xx, y0+yy)
			r16, g16, b16, _ := c.RGBA()
			r8 := uint8(r16 >> 8)
			g8 := uint8(g16 >> 8)
			b8 := uint8(b16 >> 8)
			ycc := rgbToYCbCr(r8, g8, b8)
			val := channelValue(ycc, ch)
			sum += uint64(val)
		}
	}
	thr := int32(sum / uint64(total))

	// second pass: emit bits and accumulate FG/BG means
	var fgSum, bgSum uint64
	var fgCnt, bgCnt uint32

	for yy := 0; yy < bh; yy++ {
		for xx := 0; xx < bw; xx++ {
			c := img.At(x0+xx, y0+yy)
			r16, g16, b16, _ := c.RGBA()
			r8 := uint8(r16 >> 8)
			g8 := uint8(g16 >> 8)
			b8 := uint8(b16 >> 8)
			ycc := rgbToYCbCr(r8, g8, b8)
			val := int32(channelValue(ycc, ch))

			isFg := val >= thr
			if pw != nil {
				if err := pw.WriteBit(isFg); err != nil {
					return 0, 0, err
				}
			}
			if isFg {
				fgSum += uint64(val)
				fgCnt++
			} else {
				bgSum += uint64(val)
				bgCnt++
			}
		}
	}

	// handle degenerate cases: if one of the groups is empty, fall back to the global mean
	avg := uint8(sum / uint64(total))
	var fg, bg uint8
	if fgCnt == 0 || bgCnt == 0 {
		fg, bg = avg, avg
	} else {
		fg = uint8(fgSum / uint64(fgCnt))
		bg = uint8(bgSum / uint64(bgCnt))
	}
	return fg, bg, nil
}

// encodeChannel builds one complete stream for a single channel (Y, Cb, or Cr):
// its own block list, size stream, pattern stream, and per-block FG/BG levels.
func encodeChannel(img image.Image, ch int, w4, h4, fullW, fullH int, useMacro bool) ([]blockMetaCh, []byte, []byte, error) {
	// macro-block decision bits (only for main fullW x fullH area)
	var macroUseBig []bool
	var blocks []blockMetaCh

	var sizeBuf bytes.Buffer
	sizeW := NewBitWriter(&sizeBuf)
	var patternBuf bytes.Buffer
	patternW := NewBitWriter(&patternBuf)

	// main macroBlock x macroBlock area
	for my := 0; my < fullH; my += macroBlock {
		for mx := 0; mx < fullW; mx += macroBlock {
			if useMacro && canUseBigBlockChannel(img, ch, mx, my) {
				fg, bg, err := encodeBlockChannel(img, ch, mx, my, macroBlock, macroBlock, patternW)
				if err != nil {
					return nil, nil, nil, err
				}
				blocks = append(blocks, blockMetaCh{fg: fg, bg: bg, isBig: true})
				macroUseBig = append(macroUseBig, true)
			} else {
				// grid of smallBlock x smallBlock blocks covering the macroBlock area
				for by := 0; by < macroBlock; by += smallBlock {
					for bx := 0; bx < macroBlock; bx += smallBlock {
						xx, yy := mx+bx, my+by
						// for smallBlock > 1 we also emit pattern bits; for smallBlock == 1 we skip pattern bits
						var pw *BitWriter
						if smallBlock > 1 {
							pw = patternW
						}
						fg, bg, err := encodeBlockChannel(img, ch, xx, yy, smallBlock, smallBlock, pw)
						if err != nil {
							return nil, nil, nil, err
						}
						blocks = append(blocks, blockMetaCh{fg: fg, bg: bg, isBig: false})
					}
				}
				macroUseBig = append(macroUseBig, false)
			}
		}
	}

	// right stripe: small blocks only
	for my := 0; my < fullH; my += smallBlock {
		for mx := fullW; mx < w4; mx += smallBlock {
			var pw *BitWriter
			if smallBlock > 1 {
				pw = patternW
			}
			fg, bg, err := encodeBlockChannel(img, ch, mx, my, smallBlock, smallBlock, pw)
			if err != nil {
				return nil, nil, nil, err
			}
			blocks = append(blocks, blockMetaCh{fg: fg, bg: bg, isBig: false})
		}
	}
	// bottom stripe: small blocks only (including bottom-right corner)
	for my := fullH; my < h4; my += smallBlock {
		for mx := 0; mx < w4; mx += smallBlock {
			var pw *BitWriter
			if smallBlock > 1 {
				pw = patternW
			}
			fg, bg, err := encodeBlockChannel(img, ch, mx, my, smallBlock, smallBlock, pw)
			if err != nil {
				return nil, nil, nil, err
			}
			blocks = append(blocks, blockMetaCh{fg: fg, bg: bg, isBig: false})
		}
	}

	// write macroUseBig bits for the main area into the size stream
	for _, useBig := range macroUseBig {
		if err := sizeW.WriteBit(useBig); err != nil {
			return nil, nil, nil, err
		}
	}
	if err := sizeW.Flush(); err != nil {
		return nil, nil, nil, err
	}
	if err := patternW.Flush(); err != nil {
		return nil, nil, nil, err
	}

	return blocks, sizeBuf.Bytes(), patternBuf.Bytes(), nil
}

// Encode runs the BABE encoder with three fully independent channel streams (Y, Cb, Cr).
// Each channel has its own block list, size stream, pattern stream, and FG/BG levels.
func Encode(img image.Image, quality int) ([]byte, error) {
	if err := setBlocksForQuality(quality); err != nil {
		return nil, err
	}

	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()

	var raw bytes.Buffer
	bw := bufio.NewWriter(&raw)

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
	if _, err := bw.WriteString(codec); err != nil {
		return nil, err
	}
	if err := binary.Write(bw, binary.BigEndian, uint16(smallBlock)); err != nil {
		return nil, err
	}
	if err := binary.Write(bw, binary.BigEndian, uint16(macroBlock)); err != nil {
		return nil, err
	}
	if err := binary.Write(bw, binary.BigEndian, uint32(w)); err != nil {
		return nil, err
	}
	if err := binary.Write(bw, binary.BigEndian, uint32(h)); err != nil {
		return nil, err
	}

	// --- Encode three independent channels: Y, Cb, Cr ---
	type channelResult struct {
		blocks       []blockMetaCh
		sizeBytes    []byte
		patternBytes []byte
		err          error
	}

	var (
		results [3]channelResult
		wg      sync.WaitGroup
	)

	channelIDs := []int{chY, chCb, chCr}

	for i, ch := range channelIDs {
		wg.Add(1)
		go func(i, ch int) {
			defer wg.Done()
			blocks, sizeBytes, patternBytes, err := encodeChannel(img, ch, w4, h4, fullW, fullH, useMacro)
			results[i] = channelResult{
				blocks:       blocks,
				sizeBytes:    sizeBytes,
				patternBytes: patternBytes,
				err:          err,
			}
		}(i, ch)
	}

	wg.Wait()

	// Write channels in fixed order: Y, Cb, Cr.
	// For experimentation: smallBlocks store only FG (BG is implicit = 0),
	// macroBlocks store both FG and BG like before.
	for i := 0; i < 3; i++ {
		res := results[i]
		if res.err != nil {
			return nil, res.err
		}
		blockCount := uint32(len(res.blocks))
		// number of blocks for this channel
		if err := binary.Write(bw, binary.BigEndian, blockCount); err != nil {
			return nil, err
		}
		// write size stream for this channel
		if err := binary.Write(bw, binary.BigEndian, uint32(len(res.sizeBytes))); err != nil {
			return nil, err
		}
		if _, err := bw.Write(res.sizeBytes); err != nil {
			return nil, err
		}
		// write pattern stream for this channel
		if err := binary.Write(bw, binary.BigEndian, uint32(len(res.patternBytes))); err != nil {
			return nil, err
		}
		if _, err := bw.Write(res.patternBytes); err != nil {
			return nil, err
		}
		// write FG/BG levels:
		// - if smallBlock == 1: 1 byte per smallBlock (FG only), 2 bytes per macroBlock (FG+BG)
		// - if smallBlock > 1: 2 bytes per block (FG+BG) for both small and macro blocks
		for _, blk := range res.blocks {
			if err := bw.WriteByte(blk.fg); err != nil {
				return nil, err
			}
			if blk.isBig || smallBlock > 1 {
				if err := bw.WriteByte(blk.bg); err != nil {
					return nil, err
				}
			}
		}
	}

	if err := bw.Flush(); err != nil {
		return nil, err
	}
	comp, err := compressZstd(raw.Bytes())
	if err != nil {
		return nil, fmt.Errorf("zstd encode: %w", err)
	}
	return comp, nil
}

// drawBlockPlane decodes a single block for one channel into a planar buffer.
func drawBlockPlane(plane []uint8, stride int, x0, y0, bw, bh int, br *BitReader, fg, bg uint8) error {
	for yy := 0; yy < bh; yy++ {
		for xx := 0; xx < bw; xx++ {
			bit, err := br.ReadBit()
			if err != nil {
				return err
			}
			val := bg
			if bit {
				val = fg
			}
			idx := (y0+yy)*stride + (x0 + xx)
			if idx < 0 || idx >= len(plane) {
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
			if idx < 0 || idx >= len(plane) {
				return fmt.Errorf("fillBlockPlane: index out of range")
			}
			plane[idx] = val
		}
	}
	return nil
}

// decodeChannel decodes one channel stream into a planar buffer of size imgW x imgH.
func decodeChannel(br *bufio.Reader, imgW, imgH int) ([]uint8, error) {
	var blockCount uint32
	if err := binary.Read(br, binary.BigEndian, &blockCount); err != nil {
		return nil, err
	}

	var sizeStreamLen uint32
	if err := binary.Read(br, binary.BigEndian, &sizeStreamLen); err != nil {
		return nil, err
	}
	sizeBytes := make([]byte, sizeStreamLen)
	if _, err := io.ReadFull(br, sizeBytes); err != nil {
		return nil, err
	}
	sizeBR := NewBitReader(sizeBytes)

	var patternLen uint32
	if err := binary.Read(br, binary.BigEndian, &patternLen); err != nil {
		return nil, err
	}
	patternBytes := make([]byte, patternLen)
	if _, err := io.ReadFull(br, patternBytes); err != nil {
		return nil, err
	}
	patternBR := NewBitReader(patternBytes)

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
	for i := 0; i < macroGroupCount; i++ {
		bit, err := sizeBR.ReadBit()
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
				// macroBlock: read FG and BG and use full pattern
				var fg, bg uint8
				if err := binary.Read(br, binary.BigEndian, &fg); err != nil {
					return nil, err
				}
				if err := binary.Read(br, binary.BigEndian, &bg); err != nil {
					return nil, err
				}
				if err := drawBlockPlane(plane, imgW, mx, my, macroBlock, macroBlock, patternBR, fg, bg); err != nil {
					return nil, err
				}
				blockIndex++
			} else {
				// grid of smallBlock blocks:
				// - if smallBlock == 1: each block reads only FG and is filled solid
				// - if smallBlock > 1: each block reads FG+BG and uses pattern bits
				for by := 0; by < macroBlock; by += smallBlock {
					for bx := 0; bx < macroBlock; bx += smallBlock {
						if blockIndex >= int(blockCount) {
							return nil, fmt.Errorf("unexpected end of blocks in macro grid")
						}
						var fg uint8
						if err := binary.Read(br, binary.BigEndian, &fg); err != nil {
							return nil, err
						}
						if smallBlock > 1 {
							var bg uint8
							if err := binary.Read(br, binary.BigEndian, &bg); err != nil {
								return nil, err
							}
							if err := drawBlockPlane(plane, imgW, mx+bx, my+by, smallBlock, smallBlock, patternBR, fg, bg); err != nil {
								return nil, err
							}
						} else {
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
			var fg uint8
			if err := binary.Read(br, binary.BigEndian, &fg); err != nil {
				return nil, err
			}
			if smallBlock > 1 {
				var bg uint8
				if err := binary.Read(br, binary.BigEndian, &bg); err != nil {
					return nil, err
				}
				if err := drawBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, patternBR, fg, bg); err != nil {
					return nil, err
				}
			} else {
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
			var fg uint8
			if err := binary.Read(br, binary.BigEndian, &fg); err != nil {
				return nil, err
			}
			if smallBlock > 1 {
				var bg uint8
				if err := binary.Read(br, binary.BigEndian, &bg); err != nil {
					return nil, err
				}
				if err := drawBlockPlane(plane, imgW, mx, my, smallBlock, smallBlock, patternBR, fg, bg); err != nil {
					return nil, err
				}
			} else {
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

	return plane, nil
}

// Decode reads a BABE-compressed stream (Zstd + three independent Y/Cb/Cr streams)
// and returns an image.Image.
func Decode(compData []byte) (image.Image, error) {
	payload, err := decompressZstd(compData)
	if err != nil {
		return nil, fmt.Errorf("zstd decode: %w", err)
	}

	br := bufio.NewReader(bytes.NewReader(payload))

	// magic
	header := make([]byte, len(codec))
	if _, err := io.ReadFull(br, header); err != nil {
		return nil, fmt.Errorf("read header: %w", err)
	}
	if string(header) != codec {
		return nil, fmt.Errorf("bad magic: %q", string(header))
	}

	var bwSize, bhSize uint16
	var imgW32, imgH32 uint32

	if err := binary.Read(br, binary.BigEndian, &bwSize); err != nil {
		return nil, err
	}
	if err := binary.Read(br, binary.BigEndian, &bhSize); err != nil {
		return nil, err
	}
	if bwSize == 0 || bhSize == 0 {
		return nil, fmt.Errorf("invalid block sizes in header: %dx%d", bwSize, bhSize)
	}
	smallBlock = int(bwSize)
	macroBlock = int(bhSize)

	if err := binary.Read(br, binary.BigEndian, &imgW32); err != nil {
		return nil, err
	}
	if err := binary.Read(br, binary.BigEndian, &imgH32); err != nil {
		return nil, err
	}
	imgW := int(imgW32)
	imgH := int(imgH32)

	if macroBlock < smallBlock || macroBlock%smallBlock != 0 {
		return nil, fmt.Errorf("macroBlock (%d) must be >= smallBlock (%d) and a multiple of it",
			macroBlock, smallBlock)
	}

	// decode three independent channels: Y, Cb, Cr
	yPlane, err := decodeChannel(br, imgW, imgH)
	if err != nil {
		return nil, err
	}
	cbPlane, err := decodeChannel(br, imgW, imgH)
	if err != nil {
		return nil, err
	}
	crPlane, err := decodeChannel(br, imgW, imgH)
	if err != nil {
		return nil, err
	}

	// combine into RGBA
	dst := image.NewRGBA(image.Rect(0, 0, imgW, imgH))
	for y := 0; y < imgH; y++ {
		for x := 0; x < imgW; x++ {
			idx := y*imgW + x
			Y := yPlane[idx]
			Cb := cbPlane[idx]
			Cr := crPlane[idx]
			rgb := yCbCrToRGB(yuv{
				Y:  int16(Y),
				Cb: int16(Cb),
				Cr: int16(Cr),
			})
			dst.Set(x, y, color.RGBA{rgb[0], rgb[1], rgb[2], 255})
		}
	}

	// постпроцесс (градиенты в узлах + лёгкий деблокинг)
	smoothed := smoothBlocks(dst)
	return smoothed, nil
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

	maxRadius := smallBlock
	if maxRadius > 3 {
		maxRadius = 3
	}
	const junctionLumaSpread int32 = 32
	const lumaSimThreshold int32 = 8

	similar := func(a, b color.RGBA) bool {
		da := luma(a) - luma(b)
		if da < 0 {
			da = -da
		}
		return da <= lumaSimThreshold
	}

	lerp8 := func(a, b uint8, t float64) uint8 {
		return uint8(float64(a)*(1-t) + float64(b)*t + 0.5)
	}

	for y := b.Min.Y + smallBlock; y < b.Max.Y; y += smallBlock {
		for x := b.Min.X + smallBlock; x < b.Max.X; x += smallBlock {
			// four corners around the junction
			c00 := img.RGBAAt(x-1, y-1) // top-left
			c10 := img.RGBAAt(x, y-1)   // top-right
			c01 := img.RGBAAt(x-1, y)   // bottom-left
			c11 := img.RGBAAt(x, y)     // bottom-right

			l00 := luma(c00)
			l10 := luma(c10)
			l01 := luma(c01)
			l11 := luma(c11)

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
				up := img.RGBAAt(x-i, y-1)
				down := img.RGBAAt(x-i, y)
				if !similar(up, c00) || !similar(down, c01) {
					break
				}
				L = i
			}

			R := 0
			for i := 1; i <= maxRadius && x-1+i < b.Max.X; i++ {
				up := img.RGBAAt(x-1+i, y-1)
				down := img.RGBAAt(x-1+i, y)
				if !similar(up, c10) || !similar(down, c11) {
					break
				}
				R = i
			}

			U := 0
			for i := 1; i <= maxRadius && y-i >= b.Min.Y; i++ {
				left := img.RGBAAt(x-1, y-i)
				right := img.RGBAAt(x, y-i)
				if !similar(left, c00) || !similar(right, c10) {
					break
				}
				U = i
			}

			D := 0
			for i := 1; i <= maxRadius && y-1+i < b.Max.Y; i++ {
				left := img.RGBAAt(x-1, y-1+i)
				right := img.RGBAAt(x, y-1+i)
				if !similar(left, c01) || !similar(right, c11) {
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

			r00, g00, b00, a00 := c00.R, c00.G, c00.B, c00.A
			r10, g10, b10, a10 := c10.R, c10.G, c10.B, c10.A
			r01, g01, b01, a01 := c01.R, c01.G, c01.B, c01.A
			r11, g11, b11, a11 := c11.R, c11.G, c11.B, c11.A

			width := rectMaxX - rectMinX
			height := rectMaxY - rectMinY

			for py := rectMinY; py <= rectMaxY; py++ {
				v := float64(py-rectMinY) / float64(height)
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

					img.SetRGBA(px, py, color.RGBA{r, g, bc, a})
				}
			}
		}
	}

	return img
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

	// start from a copy of the original
	dst := image.NewRGBA(b)
	copy(dst.Pix, src.Pix)

	// threshold on luminance difference across a block boundary
	// smaller values -> less smoothing
	const boundaryLumaThreshold int32 = 10

	// vertical block boundaries: x = k * smallBlock
	for x := b.Min.X + smallBlock; x < b.Max.X; x += smallBlock {
		for y := b.Min.Y; y < b.Max.Y; y++ {
			// pixels immediately left and right of the boundary
			cl := src.RGBAAt(x-1, y)
			cr := src.RGBAAt(x, y)

			lL := luma(cl)
			lR := luma(cr)
			d := lL - lR
			if d < 0 {
				d = -d
			}

			// if the two sides are similar in luminance, slightly blur across the boundary
			if d <= boundaryLumaThreshold {
				r := uint8((uint16(cl.R) + uint16(cr.R)) / 2)
				g := uint8((uint16(cl.G) + uint16(cr.G)) / 2)
				bc := uint8((uint16(cl.B) + uint16(cr.B)) / 2)
				a := uint8((uint16(cl.A) + uint16(cr.A)) / 2)
				cAvg := color.RGBA{r, g, bc, a}

				dst.SetRGBA(x-1, y, cAvg)
				dst.SetRGBA(x, y, cAvg)
			}
		}
	}

	// horizontal block boundaries: y = k * smallBlock
	for y := b.Min.Y + smallBlock; y < b.Max.Y; y += smallBlock {
		for x := b.Min.X; x < b.Max.X; x++ {
			// pixels immediately above and below the boundary
			ct := src.RGBAAt(x, y-1)
			cb := src.RGBAAt(x, y)

			lT := luma(ct)
			lB := luma(cb)
			d := lT - lB
			if d < 0 {
				d = -d
			}

			if d <= boundaryLumaThreshold {
				r := uint8((uint16(ct.R) + uint16(cb.R)) / 2)
				g := uint8((uint16(ct.G) + uint16(cb.G)) / 2)
				bc := uint8((uint16(ct.B) + uint16(cb.B)) / 2)
				a := uint8((uint16(ct.A) + uint16(cb.A)) / 2)
				cAvg := color.RGBA{r, g, bc, a}

				dst.SetRGBA(x, y-1, cAvg)
				dst.SetRGBA(x, y, cAvg)
			}
		}
	}

	return dst
}

// smoothBlocks runs both junction smoothing and light deblocking in a fixed order.
// This keeps the post-process logic in one place for Decode.
func smoothBlocks(src *image.RGBA) *image.RGBA {
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
	R := float64(r)
	G := float64(g)
	B := float64(b)

	Y := int16(0.299*R + 0.587*G + 0.114*B)
	Cb := int16(-0.169*R - 0.331*G + 0.5*B + 128)
	Cr := int16(0.5*R - 0.418*G - 0.081*B + 128)

	return yuv{Y, Cb, Cr}
}

func yCbCrToRGB(v yuv) [3]uint8 {
	Y := float64(v.Y)
	Cb := float64(v.Cb) - 128
	Cr := float64(v.Cr) - 128

	R := Y + 1.402*Cr
	G := Y - 0.344136*Cb - 0.714136*Cr
	B := Y + 1.772*Cb

	clip := func(x float64) uint8 {
		if x < 0 {
			return 0
		}
		if x > 255 {
			return 255
		}
		return uint8(x)
	}

	return [3]uint8{clip(R), clip(G), clip(B)}
}

// --- ZSTD helpers ---

func compressZstd(data []byte) ([]byte, error) {
	if len(data) == 0 {
		return data, nil
	}
	var buf bytes.Buffer
	enc, err := zstd.NewWriter(&buf)
	if err != nil {
		return nil, err
	}
	if _, err := enc.Write(data); err != nil {
		_ = enc.Close()
		return nil, err
	}
	if err := enc.Close(); err != nil {
		return nil, err
	}
	return buf.Bytes(), nil
}

func decompressZstd(data []byte) ([]byte, error) {
	if len(data) == 0 {
		return data, nil
	}
	dec, err := zstd.NewReader(bytes.NewReader(data))
	if err != nil {
		return nil, err
	}
	defer dec.Close()

	var out bytes.Buffer
	if _, err := out.ReadFrom(dec); err != nil {
		return nil, err
	}
	return out.Bytes(), nil
}

func luma(c color.Color) int32 {
	r, g, b, _ := c.RGBA()
	rr := int32(r >> 8)
	gg := int32(g >> 8)
	bb := int32(b >> 8)

	// Integer approximation of ITU-R BT.601:
	// Y ≈ 0.299 R + 0.587 G + 0.114 B
	return (299*rr + 587*gg + 114*bb) / 1000
}
