// BABE (Bi-Level Adaptive Block Encoding) is a multi-level block codec for images.
// It operates in YCbCr color space, uses an N-level adaptive block hierarchy,
// per-channel bi-level patterns,
// plus a light post-process for deblocking and gradient smoothing.

package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"io"
	"math/bits"
	"runtime"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"

	"github.com/klauspost/compress/zstd"
)

const (
	codec = "BABE-L\n"
)

// current encode quality in [0..100]; used to drive macro/small decisions.

// channel presence flags for the header; at least Y must be set.
const (
	channelFlagY  = 1 << 0
	channelFlagCb = 1 << 1
	channelFlagCr = 1 << 2

	// Cb/Cr are stored as a sparse chroma grid instead of per-channel block streams.
	channelFlagChromaGrid = 1 << 7
)

const (
	defaultPatternCount = 64
)

// Quality mapping:
// - quality is in [0..100]
// - block sizes stay fixed unless they are explicitly overridden.

var defaultBlockLevels = [...]int{1, 2}

func clampQuality(quality int) int {
	if quality < 0 {
		return 0
	}
	if quality > 100 {
		return 100
	}
	return quality
}

// blocksFromSpec parses one or more comma-separated block sizes like
// "4", "2,4,8" or "2,4,8,16". Sizes must be positive, strictly increasing,
// and each next size must be divisible by the previous one.
func blocksFromSpec(spec string) ([]int, error) {
	if strings.Contains(spec, "-") && !strings.Contains(spec, ",") {
		parts := strings.Split(spec, "-")
		if len(parts) != 2 {
			return nil, fmt.Errorf("blocks range must look like 2-64")
		}
		minV, errMin := strconv.Atoi(strings.TrimSpace(parts[0]))
		maxV, errMax := strconv.Atoi(strings.TrimSpace(parts[1]))
		if errMin != nil || errMax != nil || minV < 1 || maxV < minV {
			return nil, fmt.Errorf("blocks range must contain positive sizes like 2-64")
		}
		sizes := make([]int, 0, 8)
		for v := minV; v <= maxV; v *= 2 {
			sizes = append(sizes, v)
			if v == maxV {
				break
			}
			if v > maxV/2 {
				return nil, fmt.Errorf("blocks range %q must double exactly, e.g. 2-64 or 4-32", spec)
			}
		}
		return normalizeLevels(sizes), nil
	}

	parts := strings.Split(spec, ",")
	if len(parts) == 0 {
		return nil, fmt.Errorf("blocks must contain at least one size")
	}
	sizes := make([]int, 0, len(parts))
	for _, part := range parts {
		v, err := strconv.Atoi(strings.TrimSpace(part))
		if err != nil || v < 1 {
			return nil, fmt.Errorf("blocks must contain positive integer sizes like 4, 4,8 or 2,4,8,16")
		}
		sizes = append(sizes, v)
	}
	for i := 1; i < len(sizes); i++ {
		if sizes[i] <= sizes[i-1] {
			return nil, fmt.Errorf("blocks must be strictly increasing, got %v", sizes)
		}
	}
	base := sizes[0]
	for i := 1; i < len(sizes); i++ {
		if sizes[i]%base != 0 {
			return nil, fmt.Errorf("each block size must be divisible by the first one, got %v", sizes)
		}
	}
	return normalizeLevels(sizes), nil
}

func normalizeLevels(levels []int) []int {
	if len(levels) == 2 && levels[0] == levels[1] {
		return levels[:1]
	}
	return levels
}

func defaultLevels() []int {
	return append([]int(nil), defaultBlockLevels[:]...)
}

func spreadForBlockSize(quality int) int32 {
	return allowedMacroSpreadForQuality(quality)
}

func ceilToStep(v, step int) int {
	if step <= 0 || v <= 0 {
		return 0
	}
	return ((v + step - 1) / step) * step
}

func visibleBlockDims(width, height, x0, y0, bw, bh int) (int, int) {
	if x0 < 0 || y0 < 0 || bw <= 0 || bh <= 0 || width <= 0 || height <= 0 {
		return 0, 0
	}
	visW := min(bw, width-x0)
	visH := min(bh, height-y0)
	if visW < 0 {
		visW = 0
	}
	if visH < 0 {
		visH = 0
	}
	return visW, visH
}

func invertPatternBitsInPlace(bits patternMask, bw, bh int) {
	total := bw * bh
	if total <= 0 || len(bits) == 0 {
		return
	}
	for i := range bits {
		bits[i] = ^bits[i]
	}
	usedBits := total % 64
	if usedBits != 0 {
		mask := uint64(1)<<usedBits - 1
		bits[len(bits)-1] &= mask
	}
}

func patternMaskFromUint64Into(dst patternMask, bits uint64, total int) patternMask {
	if total <= 0 {
		return nil
	}
	words := (total + 63) / 64
	if cap(dst) < words {
		dst = make(patternMask, words)
	}
	dst = dst[:words]
	for i := range dst {
		dst[i] = 0
	}
	dst[0] = bits
	return dst
}

func quantizeY(v uint8, shift int) uint8 {
	if shift <= 0 {
		return v
	}
	if shift >= 8 {
		return 128
	}
	step := 1 << shift
	base := int(v>>shift) << shift
	center := base + step/2
	if center > 255 {
		center = 255
	}
	return uint8(center)
}

type bitStream struct {
	buf  []byte
	byte byte
	n    uint8 // number of bits written (0..8) in byte
}

func newBitStream(buf []byte) bitStream {
	return bitStream{buf: buf[:0]}
}

func (bs *bitStream) writeBit(bit bool) {
	bs.byte <<= 1
	if bit {
		bs.byte |= 1
	}
	bs.n++
	if bs.n == 8 {
		bs.buf = append(bs.buf, bs.byte)
		bs.byte = 0
		bs.n = 0
	}
}

func (bs *bitStream) writeBits(bits uint64, n uint8) {
	for i := int(n) - 1; i >= 0; i-- {
		bs.writeBit(((bits >> i) & 1) != 0)
	}
}

func (bs *bitStream) appendStream(other *bitStream) {
	for _, b := range other.buf {
		for i := 7; i >= 0; i-- {
			bs.writeBit(((b >> i) & 1) != 0)
		}
	}
	if other.n > 0 {
		for i := int(other.n) - 1; i >= 0; i-- {
			bs.writeBit(((other.byte >> i) & 1) != 0)
		}
	}
}

func (bs *bitStream) bytesPadded() []byte {
	if bs.n == 0 {
		return bs.buf
	}
	return append(bs.buf, bs.byte<<(8-bs.n))
}

// bitReader reads bits from a byte slice (msb-first in each byte).
type bitReader struct {
	data []byte
	idx  int
	bit  uint8 // bit position in current byte (0..7), msb-first
}

func newBitReader(data []byte) bitReader {
	return bitReader{data: data, idx: 0, bit: 0}
}

// readBit returns the next bit, or error if out of data.
func (br *bitReader) readBit() (bool, error) {
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

// readBits reads n bits (1..8) and returns them in the low n bits of the result,
// msb-first within the n bits. For example, if the next bits are 1,0,1,1 and n=4,
// this returns 0b1011.
func (br *bitReader) readBits(n uint8) (uint8, error) {
	if n == 0 || n > 8 {
		return 0, fmt.Errorf("readBits: invalid bit count %d", n)
	}
	if br.idx >= len(br.data) {
		return 0, io.EOF
	}

	rem := uint8(8 - br.bit)
	if n <= rem {
		b := br.data[br.idx]
		shift := rem - n
		out := uint8((b >> shift) & byte((1<<n)-1))
		br.bit += n
		if br.bit == 8 {
			br.bit = 0
			br.idx++
		}
		return out, nil
	}

	// Need bits from the next byte as well.
	if br.idx+1 >= len(br.data) {
		return 0, io.EOF
	}

	b0 := br.data[br.idx]
	b1 := br.data[br.idx+1]

	first := uint8(b0 & byte((1<<rem)-1)) // lower "rem" bits
	n2 := n - rem
	second := uint8(b1 >> (8 - n2))

	out := (first << n2) | second
	br.idx++
	br.bit = n2
	return out, nil
}

// channel IDs for Y, Cb, Cr.
const (
	chY  = 0
	chCb = 1
	chCr = 2
)

func extractYCbCrPlanesInto(img image.Image, yPlane, cbPlane, crPlane []uint8) {
	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()

	switch src := img.(type) {
	case *image.RGBA:
		extractYCbCrFromRGBA(src, yPlane, cbPlane, crPlane, w, h)
	case *image.NRGBA:
		extractYCbCrFromNRGBA(src, yPlane, cbPlane, crPlane, w, h)
	case *image.YCbCr:
		extractYCbCrFromYCbCr(src, yPlane, cbPlane, crPlane, w, h)
	case *image.Gray:
		extractYCbCrFromGray(src, yPlane, cbPlane, crPlane, w, h)
	default:
		// Fallback: generic path using img.At. Still parallelised by rows.
		workers := min(runtime.NumCPU(), h)
		if workers < 1 {
			workers = 1
		}

		rowsPerWorker := (h + workers - 1) / workers

		var wg sync.WaitGroup
		for i := 0; i < workers; i++ {
			y0 := i * rowsPerWorker
			if y0 >= h {
				break
			}
			y1 := y0 + rowsPerWorker
			if y1 > h {
				y1 = h
			}

			wg.Add(1)
			go extractYCbCrFromImageStripe(img, b, w, yPlane, cbPlane, crPlane, y0, y1, &wg)
		}
		wg.Wait()
	}
}

func extractYCbCrFromImageStripe(img image.Image, b image.Rectangle, w int, yPlane, cbPlane, crPlane []uint8, yStart, yEnd int, wg *sync.WaitGroup) {
	defer wg.Done()
	for y := yStart; y < yEnd; y++ {
		baseIdx := y * w
		for x := 0; x < w; x++ {
			c := img.At(b.Min.X+x, b.Min.Y+y)
			r16, g16, b16, _ := c.RGBA()
			r8 := uint8(r16 >> 8)
			g8 := uint8(g16 >> 8)
			b8 := uint8(b16 >> 8)
			ycc := rgbToYCbCr(r8, g8, b8)
			idx := baseIdx + x
			yPlane[idx] = uint8(ycc.Y)
			cbPlane[idx] = uint8(ycc.Cb)
			crPlane[idx] = uint8(ycc.Cr)
		}
	}
}

func extractYCbCrPlanesIntoSerial(img image.Image, yPlane, cbPlane, crPlane []uint8) {
	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()

	switch src := img.(type) {
	case *image.RGBA:
		extractYCbCrFromRGBASequential(src, yPlane, cbPlane, crPlane, w, h)
	case *image.NRGBA:
		extractYCbCrFromNRGBASequential(src, yPlane, cbPlane, crPlane, w, h)
	case *image.YCbCr:
		extractYCbCrFromYCbCrSequential(src, yPlane, cbPlane, crPlane, w, h)
	case *image.Gray:
		extractYCbCrFromGraySequential(src, yPlane, cbPlane, crPlane, w, h)
	default:
		for y := 0; y < h; y++ {
			baseIdx := y * w
			for x := 0; x < w; x++ {
				c := img.At(b.Min.X+x, b.Min.Y+y)
				r16, g16, b16, _ := c.RGBA()
				r8 := uint8(r16 >> 8)
				g8 := uint8(g16 >> 8)
				b8 := uint8(b16 >> 8)
				ycc := rgbToYCbCr(r8, g8, b8)
				idx := baseIdx + x
				yPlane[idx] = uint8(ycc.Y)
				cbPlane[idx] = uint8(ycc.Cb)
				crPlane[idx] = uint8(ycc.Cr)
			}
		}
	}
}

// extractYCbCrFromRGBA converts an *image.RGBA into planar Y, Cb, Cr slices.
// It assumes dst planes are sized to w*h, where w/h come from src.Bounds().Dx/Dy.
func extractYCbCrFromRGBA(src *image.RGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
	stride := src.Stride
	pix := src.Pix

	workers := runtime.NumCPU()
	if workers > h {
		workers = h
	}
	if workers < 1 {
		workers = 1
	}

	rowsPerWorker := (h + workers - 1) / workers

	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := y0 + rowsPerWorker
		if y1 > h {
			y1 = h
		}

		wg.Add(1)
		go extractYCbCrFromRGBAStripe(pix, stride, w, yPlane, cbPlane, crPlane, y0, y1, &wg)
	}
	wg.Wait()
}

func extractYCbCrFromRGBAStripe(pix []byte, stride, w int, yPlane, cbPlane, crPlane []uint8, yStart, yEnd int, wg *sync.WaitGroup) {
	defer wg.Done()
	for y := yStart; y < yEnd; y++ {
		baseIdx := y * w
		pixRow := y * stride
		for x := 0; x < w; x++ {
			p := pixRow + x*4
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			idx := baseIdx + x
			rr, gg, bb := int32(r8), int32(g8), int32(b8)
			yPlane[idx] = uint8((77*rr + 150*gg + 29*bb) >> 8)
			cbPlane[idx] = uint8(((-43*rr - 85*gg + 128*bb) >> 8) + 128)
			crPlane[idx] = uint8(((128*rr - 107*gg - 21*bb) >> 8) + 128)
		}
	}
}

func extractYCbCrFromRGBASequential(src *image.RGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
	stride := src.Stride
	pix := src.Pix

	for y := 0; y < h; y++ {
		baseIdx := y * w
		pixRow := y * stride
		for x := 0; x < w; x++ {
			p := pixRow + x*4
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			idx := baseIdx + x
			rr, gg, bb := int32(r8), int32(g8), int32(b8)
			yPlane[idx] = uint8((77*rr + 150*gg + 29*bb) >> 8)
			cbPlane[idx] = uint8(((-43*rr - 85*gg + 128*bb) >> 8) + 128)
			crPlane[idx] = uint8(((128*rr - 107*gg - 21*bb) >> 8) + 128)
		}
	}
}

// extractYCbCrFromNRGBA converts an *image.NRGBA into planar Y, Cb, Cr slices.
// It reads RGB directly from Pix; for primarily opaque images this matches the old At/RGBA path well enough.
func extractYCbCrFromNRGBA(src *image.NRGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
	stride := src.Stride
	pix := src.Pix

	workers := runtime.NumCPU()
	if workers > h {
		workers = h
	}
	if workers < 1 {
		workers = 1
	}

	rowsPerWorker := (h + workers - 1) / workers

	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := y0 + rowsPerWorker
		if y1 > h {
			y1 = h
		}

		wg.Add(1)
		go extractYCbCrFromNRGBAStripe(pix, stride, w, yPlane, cbPlane, crPlane, y0, y1, &wg)
	}
	wg.Wait()
}

func extractYCbCrFromNRGBAStripe(pix []byte, stride, w int, yPlane, cbPlane, crPlane []uint8, yStart, yEnd int, wg *sync.WaitGroup) {
	defer wg.Done()
	for y := yStart; y < yEnd; y++ {
		baseIdx := y * w
		pixRow := y * stride
		for x := 0; x < w; x++ {
			p := pixRow + x*4
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			idx := baseIdx + x
			rr, gg, bb := int32(r8), int32(g8), int32(b8)
			yPlane[idx] = uint8((77*rr + 150*gg + 29*bb) >> 8)
			cbPlane[idx] = uint8(((-43*rr - 85*gg + 128*bb) >> 8) + 128)
			crPlane[idx] = uint8(((128*rr - 107*gg - 21*bb) >> 8) + 128)
		}
	}
}

func extractYCbCrFromNRGBASequential(src *image.NRGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
	stride := src.Stride
	pix := src.Pix

	for y := 0; y < h; y++ {
		baseIdx := y * w
		pixRow := y * stride
		for x := 0; x < w; x++ {
			p := pixRow + x*4
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			idx := baseIdx + x
			rr, gg, bb := int32(r8), int32(g8), int32(b8)
			yPlane[idx] = uint8((77*rr + 150*gg + 29*bb) >> 8)
			cbPlane[idx] = uint8(((-43*rr - 85*gg + 128*bb) >> 8) + 128)
			crPlane[idx] = uint8(((128*rr - 107*gg - 21*bb) >> 8) + 128)
		}
	}
}

func extractYCbCrFromYCbCr(src *image.YCbCr, yPlane, cbPlane, crPlane []uint8, w, h int) {
	b := src.Bounds()
	minX, minY := b.Min.X, b.Min.Y

	workers := runtime.NumCPU()
	if workers > h {
		workers = h
	}
	if workers < 1 {
		workers = 1
	}
	rowsPerWorker := (h + workers - 1) / workers

	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := y0 + rowsPerWorker
		if y1 > h {
			y1 = h
		}

		wg.Add(1)
		go extractYCbCrFromYCbCrStripe(src, minX, minY, w, yPlane, cbPlane, crPlane, y0, y1, &wg)
	}
	wg.Wait()
}

func extractYCbCrFromYCbCrStripe(src *image.YCbCr, minX, minY, w int, yPlane, cbPlane, crPlane []uint8, yStart, yEnd int, wg *sync.WaitGroup) {
	defer wg.Done()
	rectMinX, rectMinY := src.Rect.Min.X, src.Rect.Min.Y
	yStride := src.YStride
	yPix := src.Y
	cbPix := src.Cb
	crPix := src.Cr

	for y := yStart; y < yEnd; y++ {
		yAbs := minY + y
		baseIdx := y * w
		yRow := (yAbs - rectMinY) * yStride
		for x := 0; x < w; x++ {
			xAbs := minX + x
			yPlane[baseIdx+x] = yPix[yRow+(xAbs-rectMinX)]
			ci := src.COffset(xAbs, yAbs)
			cbPlane[baseIdx+x] = cbPix[ci]
			crPlane[baseIdx+x] = crPix[ci]
		}
	}
}

func extractYCbCrFromYCbCrSequential(src *image.YCbCr, yPlane, cbPlane, crPlane []uint8, w, h int) {
	b := src.Bounds()
	minX, minY := b.Min.X, b.Min.Y
	rectMinX, rectMinY := src.Rect.Min.X, src.Rect.Min.Y

	yStride := src.YStride
	yPix := src.Y
	cbPix := src.Cb
	crPix := src.Cr

	for y := 0; y < h; y++ {
		yAbs := minY + y
		baseIdx := y * w
		yRow := (yAbs - rectMinY) * yStride
		for x := 0; x < w; x++ {
			xAbs := minX + x
			yPlane[baseIdx+x] = yPix[yRow+(xAbs-rectMinX)]
			ci := src.COffset(xAbs, yAbs)
			cbPlane[baseIdx+x] = cbPix[ci]
			crPlane[baseIdx+x] = crPix[ci]
		}
	}
}

func extractYCbCrFromGray(src *image.Gray, yPlane, cbPlane, crPlane []uint8, w, h int) {
	b := src.Bounds()
	minX, minY := b.Min.X, b.Min.Y
	rectMinX, rectMinY := src.Rect.Min.X, src.Rect.Min.Y
	stride := src.Stride
	pix := src.Pix

	workers := runtime.NumCPU()
	if workers > h {
		workers = h
	}
	if workers < 1 {
		workers = 1
	}
	rowsPerWorker := (h + workers - 1) / workers

	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := y0 + rowsPerWorker
		if y1 > h {
			y1 = h
		}

		wg.Add(1)
		go extractYCbCrFromGrayStripe(pix, stride, minX, minY, rectMinX, rectMinY, w, yPlane, cbPlane, crPlane, y0, y1, &wg)
	}
	wg.Wait()
}

func extractYCbCrFromGrayStripe(pix []byte, stride, minX, minY, rectMinX, rectMinY, w int, yPlane, cbPlane, crPlane []uint8, yStart, yEnd int, wg *sync.WaitGroup) {
	defer wg.Done()
	for y := yStart; y < yEnd; y++ {
		yAbs := minY + y
		baseIdx := y * w
		pixRow := (yAbs - rectMinY) * stride
		for x := 0; x < w; x++ {
			xAbs := minX + x
			v := pix[pixRow+(xAbs-rectMinX)]
			yPlane[baseIdx+x] = v
			cbPlane[baseIdx+x] = 128
			crPlane[baseIdx+x] = 128
		}
	}
}

func extractYCbCrFromGraySequential(src *image.Gray, yPlane, cbPlane, crPlane []uint8, w, h int) {
	b := src.Bounds()
	minX, minY := b.Min.X, b.Min.Y
	rectMinX, rectMinY := src.Rect.Min.X, src.Rect.Min.Y
	stride := src.Stride
	pix := src.Pix

	for y := 0; y < h; y++ {
		yAbs := minY + y
		baseIdx := y * w
		pixRow := (yAbs - rectMinY) * stride
		for x := 0; x < w; x++ {
			xAbs := minX + x
			v := pix[pixRow+(xAbs-rectMinX)]
			yPlane[baseIdx+x] = v
			cbPlane[baseIdx+x] = 128
			crPlane[baseIdx+x] = 128
		}
	}
}

func encodeRingDeltaBytesInPlace(data []byte) {
	if len(data) == 0 {
		return
	}
	prev := data[0]
	for i := 1; i < len(data); i++ {
		curr := data[i]
		data[i] = byte((int(curr) - int(prev) + 256) & 0xFF)
		prev = curr
	}
}

func decodeRingDeltaBytesInPlace(data []byte) {
	if len(data) == 0 {
		return
	}
	prev := data[0]
	for i := 1; i < len(data); i++ {
		prev = byte((int(prev) + int(data[i])) & 0xFF)
		data[i] = prev
	}
}

func powerOfTwoCeil(v int) int {
	if v <= 0 {
		return 1
	}
	n := 1
	for n < v {
		n <<= 1
	}
	return n
}

// hilbertIndices returns raster indices (y*gridW+x) in Hilbert order clipped to gridW x gridH.
func hilbertIndices(gridW, gridH int) []int {
	key := [2]int{gridW, gridH}
	hilbertCacheMu.RLock()
	cached, ok := hilbertCache[key]
	hilbertCacheMu.RUnlock()
	if ok {
		return cached
	}

	n := powerOfTwoCeil(max(gridW, gridH))
	total := gridW * gridH
	order := make([]int, 0, total)
	for i := 0; len(order) < total; i++ {
		x, y := hilbertXY(i, n)
		if x < gridW && y < gridH {
			order = append(order, y*gridW+x)
		}
	}
	hilbertCacheMu.Lock()
	hilbertCache[key] = order
	hilbertCacheMu.Unlock()
	return order
}

// hilbertXY maps distance d to (x,y) for an n x n Hilbert curve, n is power of two.
func hilbertXY(d, n int) (int, int) {
	x, y := 0, 0
	for s := 1; s < n; s <<= 1 {
		rx := (d / 2) & 1
		ry := (d ^ rx) & 1
		if ry == 0 {
			if rx == 1 {
				x = s - 1 - x
				y = s - 1 - y
			}
			x, y = y, x
		}
		x += s * rx
		y += s * ry
		d >>= 2
	}
	return x, y
}

func applyPermutationInto(dst, src []byte, order []int) {
	for i, idx := range order {
		dst[i] = src[idx]
	}
}

func invertPermutationInto(dst, src []byte, order []int) {
	for i, idx := range order {
		dst[idx] = src[i]
	}
}

func encodeChromaGridOverlay(dst *bytes.Buffer, cbPlane, crPlane []uint8, w, h, step int) {
	if step <= 0 {
		step = 16
	}
	dst.WriteByte(byte(step))
	gridW := (w+step-1)/step + 1
	gridH := (h+step-1)/step + 1
	var hdr [4]byte
	binary.BigEndian.PutUint16(hdr[0:2], uint16(gridW))
	binary.BigEndian.PutUint16(hdr[2:4], uint16(gridH))
	dst.Write(hdr[:])
	for gy := 0; gy < gridH; gy++ {
		y := min(gy*step, h-1)
		for gx := 0; gx < gridW; gx++ {
			x := min(gx*step, w-1)
			dst.WriteByte(cbPlane[y*w+x])
		}
	}
	cbGridEnd := dst.Len()
	for gy := 0; gy < gridH; gy++ {
		y := min(gy*step, h-1)
		for gx := 0; gx < gridW; gx++ {
			x := min(gx*step, w-1)
			dst.WriteByte(crPlane[y*w+x])
		}
	}
	gridStart := cbGridEnd - gridW*gridH
	cbGrid := dst.Bytes()[gridStart:cbGridEnd]
	crGrid := dst.Bytes()[cbGridEnd : cbGridEnd+gridW*gridH]

	order := hilbertIndices(gridW, gridH)
	tmp := make([]byte, gridW*gridH)
	applyPermutationInto(tmp, cbGrid, order)
	encodeRingDeltaBytesInPlace(tmp)
	copy(cbGrid, tmp)
	applyPermutationInto(tmp, crGrid, order)
	encodeRingDeltaBytesInPlace(tmp)
	copy(crGrid, tmp)
}

func (d *Decoder) decodeChromaGridOverlay(data []byte, pos, w, h int) ([]uint8, []uint8, int, error) {
	if len(data)-pos < 5 {
		return nil, nil, pos, fmt.Errorf("decode: truncated chroma grid header")
	}
	step := int(data[pos])
	pos++
	if step < 1 {
		return nil, nil, pos, fmt.Errorf("decode: invalid chroma grid step")
	}
	gridW := int(binary.BigEndian.Uint16(data[pos : pos+2]))
	gridH := int(binary.BigEndian.Uint16(data[pos+2 : pos+4]))
	pos += 4
	if gridW < 2 || gridH < 2 {
		return nil, nil, pos, fmt.Errorf("decode: invalid chroma grid size")
	}
	nodeCount := gridW * gridH
	if len(data)-pos < nodeCount*2 {
		return nil, nil, pos, fmt.Errorf("decode: truncated chroma grid payload")
	}
	if cap(d.chromaGrid) < nodeCount*2 {
		d.chromaGrid = make([]byte, nodeCount*2)
	}
	d.chromaGrid = d.chromaGrid[:nodeCount*2]
	cbGrid := d.chromaGrid[:nodeCount]
	crGrid := d.chromaGrid[nodeCount:]
	copy(cbGrid, data[pos:pos+nodeCount])
	pos += nodeCount
	copy(crGrid, data[pos:pos+nodeCount])
	pos += nodeCount
	order := hilbertIndices(gridW, gridH)
	decodeRingDeltaBytesInPlace(cbGrid)
	decodeRingDeltaBytesInPlace(crGrid)
	if cap(d.chromaTemp) < nodeCount {
		d.chromaTemp = make([]byte, nodeCount)
	}
	d.chromaTemp = d.chromaTemp[:nodeCount]
	invertPermutationInto(d.chromaTemp, cbGrid, order)
	copy(cbGrid, d.chromaTemp)
	invertPermutationInto(d.chromaTemp, crGrid, order)
	copy(crGrid, d.chromaTemp)
	cbPlane := d.decodeChromaGridPlaneValues(cbGrid, step, gridW, gridH, w, h)
	crPlane := d.decodeChromaGridPlaneValues(crGrid, step, gridW, gridH, w, h)
	return cbPlane, crPlane, pos, nil
}

func (d *Decoder) decodeChromaGridOverlayIntoPix(data []byte, pos, w, h int, pix []byte, strideBytes int) (int, error) {
	if len(data)-pos < 5 {
		return pos, fmt.Errorf("decode: truncated chroma grid header")
	}
	step := int(data[pos])
	pos++
	if step < 1 {
		return pos, fmt.Errorf("decode: invalid chroma grid step")
	}
	gridW := int(binary.BigEndian.Uint16(data[pos : pos+2]))
	gridH := int(binary.BigEndian.Uint16(data[pos+2 : pos+4]))
	pos += 4
	if gridW < 2 || gridH < 2 {
		return pos, fmt.Errorf("decode: invalid chroma grid size")
	}
	nodeCount := gridW * gridH
	if len(data)-pos < nodeCount*2 {
		return pos, fmt.Errorf("decode: truncated chroma grid payload")
	}
	if cap(d.chromaGrid) < nodeCount*2 {
		d.chromaGrid = make([]byte, nodeCount*2)
	}
	d.chromaGrid = d.chromaGrid[:nodeCount*2]
	cbGrid := d.chromaGrid[:nodeCount]
	crGrid := d.chromaGrid[nodeCount:]
	copy(cbGrid, data[pos:pos+nodeCount])
	pos += nodeCount
	copy(crGrid, data[pos:pos+nodeCount])
	pos += nodeCount
	order := hilbertIndices(gridW, gridH)
	decodeRingDeltaBytesInPlace(cbGrid)
	decodeRingDeltaBytesInPlace(crGrid)
	if cap(d.chromaTemp) < nodeCount {
		d.chromaTemp = make([]byte, nodeCount)
	}
	d.chromaTemp = d.chromaTemp[:nodeCount]
	invertPermutationInto(d.chromaTemp, cbGrid, order)
	copy(cbGrid, d.chromaTemp)
	invertPermutationInto(d.chromaTemp, crGrid, order)
	copy(crGrid, d.chromaTemp)

	d.decodeChromaGridPlaneValuesIntoRGBA(cbGrid, step, gridW, gridH, w, h, pix, strideBytes, 1)
	d.decodeChromaGridPlaneValuesIntoRGBA(crGrid, step, gridW, gridH, w, h, pix, strideBytes, 2)
	return pos, nil
}

func (d *Decoder) ensureChromaX(w int) {
	if cap(d.chromaXCell) < w {
		d.chromaXCell = make([]int, w)
		d.chromaXFX = make([]uint16, w)
		d.chromaXFXInv = make([]uint16, w)
		return
	}
	d.chromaXCell = d.chromaXCell[:w]
	d.chromaXFX = d.chromaXFX[:w]
	d.chromaXFXInv = d.chromaXFXInv[:w]
}

func (d *Decoder) fillChromaX(step, gridW, w int) {
	if w == 0 {
		return
	}
	for x := 0; x < w; x++ {
		cellX := x / step
		if cellX >= gridW-1 {
			cellX = gridW - 2
		}
		x0 := cellX * step
		fx := ((x - x0) << 8) / step
		if fx < 0 {
			fx = 0
		} else if fx > 255 {
			fx = 255
		}
		d.chromaXCell[x] = cellX
		d.chromaXFX[x] = uint16(fx)
		d.chromaXFXInv[x] = uint16(256 - fx)
	}
}

func (d *Decoder) decodeChromaGridPlaneValuesIntoRGBA(grid []byte, step, gridW, gridH, w, h int, pix []byte, strideBytes int, channelOffset int) {
	if w == 0 || h == 0 || step <= 0 {
		return
	}
	d.ensureChromaX(w)
	d.fillChromaX(step, gridW, w)

	for y := 0; y < h; y++ {
		cellY := y / step
		if cellY >= gridH-1 {
			cellY = gridH - 2
		}
		y0 := cellY * step
		fy := ((y - y0) << 8) / step
		if fy < 0 {
			fy = 0
		} else if fy > 255 {
			fy = 255
		}
		fyInv := 256 - fy
		rowBase := cellY * gridW
		row := y*strideBytes + channelOffset
		for x := 0; x < w; x++ {
			cellX := d.chromaXCell[x]
			fx := int(d.chromaXFX[x])
			fxInv := int(d.chromaXFXInv[x])

			i00 := rowBase + cellX
			i10 := i00 + 1
			i01 := i00 + gridW
			i11 := i01 + 1

			v00 := int(grid[i00])
			v10 := int(grid[i10])
			v01 := int(grid[i01])
			v11 := int(grid[i11])

			top := (v00*fxInv + v10*fx)
			bot := (v01*fxInv + v11*fx)
			v := (top*fyInv + bot*fy + 32768) >> 16
			if v < 0 {
				v = 0
			} else if v > 255 {
				v = 255
			}
			pix[row+x*4] = uint8(v)
		}
	}
}

func (d *Decoder) decodeChromaGridPlaneValuesIntoPlane(grid []byte, step, gridW, gridH, w, h int, plane []uint8, stride int) {
	if w == 0 || h == 0 || step <= 0 {
		return
	}
	d.ensureChromaX(w)
	d.fillChromaX(step, gridW, w)

	for y := 0; y < h; y++ {
		cellY := y / step
		if cellY >= gridH-1 {
			cellY = gridH - 2
		}
		y0 := cellY * step
		fy := ((y - y0) << 8) / step
		if fy < 0 {
			fy = 0
		} else if fy > 255 {
			fy = 255
		}
		fyInv := 256 - fy
		rowBase := cellY * gridW
		row := y * stride
		for x := 0; x < w; x++ {
			cellX := d.chromaXCell[x]
			fx := int(d.chromaXFX[x])
			fxInv := int(d.chromaXFXInv[x])

			i00 := rowBase + cellX
			i10 := i00 + 1
			i01 := i00 + gridW
			i11 := i01 + 1

			v00 := int(grid[i00])
			v10 := int(grid[i10])
			v01 := int(grid[i01])
			v11 := int(grid[i11])

			top := (v00*fxInv + v10*fx)
			bot := (v01*fxInv + v11*fx)
			v := (top*fyInv + bot*fy + 32768) >> 16
			if v < 0 {
				v = 0
			} else if v > 255 {
				v = 255
			}
			plane[row+x] = uint8(v)
		}
	}
}

func (d *Decoder) decodeChromaGridPlaneValues(grid []byte, step, gridW, gridH, w, h int) []uint8 {
	out := make([]uint8, w*h)
	if w == 0 || h == 0 {
		return out
	}
	if step <= 0 {
		return out
	}
	d.decodeChromaGridPlaneValuesIntoPlane(grid, step, gridW, gridH, w, h, out, w)
	return out
}

// canUseBigBlockChannel decides whether a macroBlock region can be encoded as a single block
// for the given channel plane. It uses a quality-dependent spread threshold: lower quality
// allows larger spread (more macroBlocks), higher quality reduces spread (more small blocks).
func allowedMacroSpreadForQuality(quality int) int32 {
	// quality-dependent spread changes linearly across the full [0..100] range:
	// lower quality allows larger spread (more macro blocks), higher quality
	// tightens the threshold and forces more subdivision.
	q := quality
	if q < 0 {
		q = 0
	}
	if q > 100 {
		q = 100
	}

	const spreadMax int32 = 64
	const spreadMin int32 = 8
	return spreadMax - int32(q)*(spreadMax-spreadMin)/100
}

func canUseBlockSizeChannel(plane []uint8, stride, height, x0, y0, blockSize int, spread int32) bool {
	if spread <= 0 {
		return false
	}
	if stride <= 0 || height <= 0 || x0 < 0 || y0 < 0 || blockSize <= 0 {
		return false
	}
	visW, visH := visibleBlockDims(stride, height, x0, y0, blockSize, blockSize)
	if visW <= 0 || visH <= 0 {
		return false
	}

	if visW == 2 && visH == 2 {
		idx0 := y0*stride + x0
		idx1 := idx0 + stride

		v0 := plane[idx0]
		v1 := plane[idx0+1]
		v2 := plane[idx1]
		v3 := plane[idx1+1]

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
		return int32(maxV)-int32(minV) < spread
	}

	minV := plane[y0*stride+x0]
	maxV := minV

	for yy := 0; yy < visH; yy++ {
		row := (y0+yy)*stride + x0
		for xx := 0; xx < visW; xx++ {
			v := plane[row+xx]
			if v < minV {
				minV = v
			} else if v > maxV {
				maxV = v
			}
			if int32(maxV)-int32(minV) >= spread {
				return false
			}
		}
	}

	return int32(maxV)-int32(minV) < spread
}

type patternCacheKey struct {
	bw    int
	bh    int
	count int
}

var (
	fixedPatternCacheMu sync.RWMutex
	fixedPatternCache   = map[patternCacheKey][]patternMask{}

	patternIndexCacheMu sync.RWMutex
	patternIndexCache   = map[patternCacheKey]map[uint64]uint16{}

	hilbertCacheMu sync.RWMutex
	hilbertCache   = map[[2]int][]int{}
)

type patternMask []uint64

func newPatternMask(total int) patternMask {
	if total <= 0 {
		return nil
	}
	return make(patternMask, (total+63)/64)
}

func clonePatternMask(src patternMask) patternMask {
	if len(src) == 0 {
		return nil
	}
	dst := make(patternMask, len(src))
	copy(dst, src)
	return dst
}

func patternMaskEmpty(mask patternMask) bool {
	for _, word := range mask {
		if word != 0 {
			return false
		}
	}
	return true
}

func patternMaskKey(mask patternMask) string {
	if len(mask) == 0 {
		return ""
	}
	var b strings.Builder
	for i, word := range mask {
		if i > 0 {
			b.WriteByte(':')
		}
		fmt.Fprintf(&b, "%016x", word)
	}
	return b.String()
}

func formatPatternBits(bits patternMask, bw, bh int) string {
	total := bw * bh
	if total <= 0 {
		return ""
	}
	var b strings.Builder
	b.Grow(total)
	for y := 0; y < bh; y++ {
		for x := 0; x < bw; x++ {
			if testPatternBit(bits, bw, bh, x, y) {
				b.WriteByte('1')
			} else {
				b.WriteByte('0')
			}
		}
	}
	return b.String()
}

func setPatternBit(mask patternMask, shift int) {
	if shift < 0 {
		return
	}
	word := shift / 64
	bit := uint(shift % 64)
	mask[word] |= uint64(1) << bit
}

func testPatternBit(mask patternMask, bw, bh, x, y int) bool {
	shift := bw*bh - 1 - (y*bw + x)
	if shift < 0 {
		return false
	}
	word := shift / 64
	bit := uint(shift % 64)
	if word >= len(mask) {
		return false
	}
	return ((mask[word] >> bit) & 1) != 0
}

func buildPatternMask(bw, bh int, pred func(x, y int) bool) patternMask {
	total := bw * bh
	mask := newPatternMask(total)
	for y := 0; y < bh; y++ {
		for x := 0; x < bw; x++ {
			if pred(x, y) {
				shift := total - 1 - (y*bw + x)
				setPatternBit(mask, shift)
			}
		}
	}
	return mask
}

func appendPatternCandidate(dst *[]patternMask, seen map[string]struct{}, mask patternMask) {
	if len(mask) == 0 {
		return
	}
	if patternMaskEmpty(mask) {
		return
	}
	key := patternMaskKey(mask)
	if _, ok := seen[key]; ok {
		return
	}
	seen[key] = struct{}{}
	*dst = append(*dst, clonePatternMask(mask))
}

func patternMaskFromUint64(bits uint64, total int) patternMask {
	mask := newPatternMask(total)
	if len(mask) > 0 {
		mask[0] = bits
	}
	return mask
}

func basicPatternTemplates() [][]string {
	return [][]string{
		{"11111111", "11111111", "11111111", "11111111", "11111111", "11111111", "11111111", "11111111"},
		{"11110000", "11110000", "11110000", "11110000", "11110000", "11110000", "11110000", "11110000"},
		{"11111111", "11111111", "11111111", "11111111", "00000000", "00000000", "00000000", "00000000"},
		{"11110000", "11110000", "11110000", "11110000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "11110000", "11110000", "11110000", "11110000"},
		{"00001111", "00001111", "00001111", "00001111", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00001111", "00001111", "00001111", "00001111"},
		{"11110000", "11110000", "11110000", "11110000", "00001111", "00001111", "00001111", "00001111"},
		{"11100000", "11100000", "11100000", "11100000", "11100000", "11100000", "11100000", "11100000"},
		{"11000000", "11000000", "11000000", "11000000", "11000000", "11000000", "11000000", "11000000"},
		{"10000000", "10000000", "10000000", "10000000", "10000000", "10000000", "10000000", "10000000"},
		{"11111111", "11111111", "11111111", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"11111111", "11111111", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"11111111", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"01000000", "01000000", "01000000", "01000000", "01000000", "01000000", "01000000", "01000000"},
		{"00100000", "00100000", "00100000", "00100000", "00100000", "00100000", "00100000", "00100000"},
		{"00010000", "00010000", "00010000", "00010000", "00010000", "00010000", "00010000", "00010000"},
		{"00000000", "11111111", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "11111111", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "11111111", "00000000", "00000000", "00000000", "00000000"},
		{"01100000", "01100000", "01100000", "01100000", "01100000", "01100000", "01100000", "01100000"},
		{"00110000", "00110000", "00110000", "00110000", "00110000", "00110000", "00110000", "00110000"},
		{"00011000", "00011000", "00011000", "00011000", "00011000", "00011000", "00011000", "00011000"},
		{"00000000", "11111111", "11111111", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "11111111", "11111111", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "11111111", "11111111", "00000000", "00000000", "00000000"},
		{"01110000", "01110000", "01110000", "01110000", "01110000", "01110000", "01110000", "01110000"},
		{"00111000", "00111000", "00111000", "00111000", "00111000", "00111000", "00111000", "00111000"},
		{"00000000", "11111111", "11111111", "11111111", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "11111111", "11111111", "11111111", "00000000", "00000000", "00000000"},
		{"01111000", "01111000", "01111000", "01111000", "01111000", "01111000", "01111000", "01111000"},
		{"00111100", "00111100", "00111100", "00111100", "00111100", "00111100", "00111100", "00111100"},
		{"00000000", "11111111", "11111111", "11111111", "11111111", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "11111111", "11111111", "11111111", "11111111", "00000000", "00000000"},
		{"01111100", "01111100", "01111100", "01111100", "01111100", "01111100", "01111100", "01111100"},
		{"00000000", "11111111", "11111111", "11111111", "11111111", "11111111", "00000000", "00000000"},
		{"01111110", "01111110", "01111110", "01111110", "01111110", "01111110", "01111110", "01111110"},
		{"00000000", "11111111", "11111111", "11111111", "11111111", "11111111", "11111111", "00000000"},
		{"00000000", "01111110", "01111110", "01111110", "01111110", "01111110", "01111110", "00000000"},
		{"11100000", "11100000", "11100000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00000000", "11100000", "11100000", "11100000"},
		{"00000111", "00000111", "00000111", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00000000", "00000111", "00000111", "00000111"},
		{"11100000", "11100000", "11100000", "00000000", "00000000", "00000111", "00000111", "00000111"},
		{"00000000", "00000000", "00111100", "00111100", "00111100", "00111100", "00000000", "00000000"},
		{"11000000", "11000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "11000000", "11000000"},
		{"00000011", "00000011", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000011", "00000011"},
		{"11000000", "11000000", "00000000", "00000000", "00000000", "00000000", "00000011", "00000011"},
		{"00000000", "00000000", "00000000", "00011000", "00011000", "00000000", "00000000", "00000000"},
		{"10000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "10000000"},
		{"00000001", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000"},
		{"00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000001"},
		{"10000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000000", "00000001"},
	}
}

func scalePatternTemplate(rows []string, bw, bh int) patternMask {
	srcH := len(rows)
	if srcH == 0 || bw <= 0 || bh <= 0 {
		return nil
	}
	srcW := len(rows[0])
	if srcW == 0 {
		return nil
	}
	return buildPatternMask(bw, bh, func(x, y int) bool {
		sx := x * srcW / bw
		sy := y * srcH / bh
		return rows[sy][sx] == '1'
	})
}

func appendBasicPatterns(out *[]patternMask, seen map[string]struct{}, bw, bh int) {
	for _, rows := range basicPatternTemplates() {
		appendPatternCandidate(out, seen, scalePatternTemplate(rows, bw, bh))
	}
}

func synthesizePatternCandidates(bw, bh int) []patternMask {
	out := make([]patternMask, 0, 256)
	seen := make(map[string]struct{}, 256)
	appendBasicPatterns(&out, seen, bw, bh)
	return out
}

func patternIndexBitsForCount(count int) uint8 {
	if count <= 1 {
		return 1
	}
	var bitsN uint8
	for v := count - 1; v > 0; v >>= 1 {
		bitsN++
	}
	return bitsN
}

func readPatternIndex(br *bitReader, bitCount uint8) (uint64, error) {
	var out uint64
	remaining := int(bitCount)
	for remaining > 0 {
		chunk := remaining
		if chunk > 8 {
			chunk = 8
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

func fixedPatternCodebook(bw, bh, count int) []patternMask {
	key := patternCacheKey{bw: bw, bh: bh, count: count}

	fixedPatternCacheMu.RLock()
	book, ok := fixedPatternCache[key]
	fixedPatternCacheMu.RUnlock()
	if ok {
		return book
	}

	if count < 1 {
		count = 1
	}

	candidates := synthesizePatternCandidates(bw, bh)
	if len(candidates) < count {
		count = len(candidates)
	}
	built := append([]patternMask(nil), candidates[:count]...)

	fixedPatternCacheMu.Lock()
	fixedPatternCache[key] = built
	fixedPatternCacheMu.Unlock()
	return built
}

func patternDistance(a, b patternMask) int {
	if len(a) == 1 && len(b) == 1 {
		return bits.OnesCount64(a[0] ^ b[0])
	}
	n := len(a)
	if len(b) > n {
		n = len(b)
	}
	dist := 0
	for i := 0; i < n; i++ {
		var aw, bw uint64
		if i < len(a) {
			aw = a[i]
		}
		if i < len(b) {
			bw = b[i]
		}
		dist += bits.OnesCount64(aw ^ bw)
	}
	return dist
}

func nearestPatternIndex(patternBits patternMask, bw, bh, patternCount int) uint64 {
	limit := patternCount
	if limit < 1 {
		limit = 1
	}
	book := fixedPatternCodebook(bw, bh, limit)
	bestIdx := 0
	bestDist := bw*bh + 1
	if len(patternBits) == 1 {
		p := patternBits[0]
		key := patternCacheKey{bw: bw, bh: bh, count: limit}
		patternIndexCacheMu.RLock()
		if m, ok := patternIndexCache[key]; ok {
			if idx, ok := m[p]; ok {
				patternIndexCacheMu.RUnlock()
				return uint64(idx)
			}
		}
		patternIndexCacheMu.RUnlock()

		for i := 0; i < len(book); i++ {
			dist := bits.OnesCount64(p ^ book[i][0])
			if dist < bestDist {
				bestDist = dist
				bestIdx = i
			}
		}
		patternIndexCacheMu.Lock()
		m, ok := patternIndexCache[key]
		if !ok {
			m = map[uint64]uint16{}
			patternIndexCache[key] = m
		}
		m[p] = uint16(bestIdx)
		patternIndexCacheMu.Unlock()
	} else {
		for i := 0; i < len(book); i++ {
			candidate := book[i]
			dist := patternDistance(patternBits, candidate)
			if dist < bestDist {
				bestDist = dist
				bestIdx = i
			}
		}
	}
	return uint64(bestIdx)
}

func encodeBlockPlaneReuse(plane []uint8, stride, height, x0, y0, bw, bh int, scratch *encoderChannelScratch) (uint8, uint8, patternMask, bool, error) {
	total := bw * bh
	if total <= 0 {
		return 0, 0, nil, false, fmt.Errorf("invalid block size")
	}

	if bw == 1 && bh == 1 {
		if x0 < 0 || y0 < 0 || x0 >= stride {
			return 0, 0, nil, false, fmt.Errorf("encodeBlockPlane: index out of range")
		}
		idx := y0*stride + x0
		if idx < 0 || idx >= len(plane) {
			return 0, 0, nil, false, fmt.Errorf("encodeBlockPlane: index out of range")
		}
		v := plane[idx]
		return v, v, nil, false, nil
	}

	if x0 < 0 || y0 < 0 || bw <= 0 || bh <= 0 || stride <= 0 || height <= 0 {
		return 0, 0, nil, false, fmt.Errorf("encodeBlockPlane: index out of range")
	}
	visW, visH := visibleBlockDims(stride, height, x0, y0, bw, bh)
	if visW <= 0 || visH <= 0 {
		return 0, 0, nil, false, fmt.Errorf("encodeBlockPlane: index out of range")
	}

	if visW == 2 && visH == 2 {
		idx0 := y0*stride + x0
		idx1 := idx0 + stride

		v0 := plane[idx0]
		v1 := plane[idx0+1]
		v2 := plane[idx1]
		v3 := plane[idx1+1]

		sum := uint64(v0) + uint64(v1) + uint64(v2) + uint64(v3)
		avg := uint8(sum / 4)
		thr := avg

		var fgSum, bgSum uint64
		var fgCnt, bgCnt uint32

		var bits uint64
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

		if fgCnt == 0 || bgCnt == 0 {
			return avg, avg, nil, false, nil
		}
		fg := uint8(fgSum / uint64(fgCnt))
		bg := uint8(bgSum / uint64(bgCnt))
		if fg < bg {
			fg, bg = bg, fg
			bits ^= (uint64(1) << total) - 1
		}
		if fg == bg {
			return fg, bg, nil, false, nil
		}
		mask := patternMaskFromUint64Into(scratch.maskBuf, bits, total)
		scratch.maskBuf = mask
		return fg, bg, mask, true, nil
	}

	visibleCount := visW * visH
	if visibleCount <= 0 {
		return 0, 0, nil, false, fmt.Errorf("encodeBlockPlane: empty visible block")
	}

	// Read the visible part of the block once into a small buffer.
	var vals []uint8
	if visibleCount <= 64 {
		var valsBuf [64]uint8
		vals = valsBuf[:visibleCount]
		_ = valsBuf
	} else {
		if cap(scratch.valsBuf) < visibleCount {
			scratch.valsBuf = make([]uint8, visibleCount)
		}
		scratch.valsBuf = scratch.valsBuf[:visibleCount]
		vals = scratch.valsBuf
	}

	var sum uint64
	i := 0
	for yy := 0; yy < visH; yy++ {
		row := (y0+yy)*stride + x0
		for xx := 0; xx < visW; xx++ {
			v := plane[row+xx]
			vals[i] = v
			sum += uint64(v)
			i++
		}
	}
	thr := uint8(sum / uint64(visibleCount))

	var fgSum, bgSum uint64
	var fgCnt, bgCnt uint32

	words := (total + 63) / 64
	if cap(scratch.maskBuf) < words {
		scratch.maskBuf = make(patternMask, words)
	}
	mask := scratch.maskBuf[:words]
	for i := range mask {
		mask[i] = 0
	}
	i = 0
	for yy := 0; yy < visH; yy++ {
		for xx := 0; xx < visW; xx++ {
			v := vals[i]
			if v >= thr {
				setPatternBit(mask, total-1-(yy*bw+xx))
				fgSum += uint64(v)
				fgCnt++
			} else {
				bgSum += uint64(v)
				bgCnt++
			}
			i++
		}
	}

	avg := uint8(sum / uint64(visibleCount))
	if fgCnt == 0 || bgCnt == 0 {
		return avg, avg, nil, false, nil
	}
	fg := uint8(fgSum / uint64(fgCnt))
	bg := uint8(bgSum / uint64(bgCnt))
	if fg < bg {
		fg, bg = bg, fg
		invertPatternBitsInPlace(mask, bw, bh)
	}
	if fg == bg {
		return fg, bg, nil, false, nil
	}
	return fg, bg, mask, true, nil
}

type encoderChannelScratch struct {
	sizeBits      []byte
	patternBits   []byte
	fgVals        []uint8
	bgVals        []uint8
	valsBuf       []uint8
	maskBuf       patternMask
	deltaBuf      []byte
	levelSpreads  []int32
	stripeScratch []encoderChannelScratch
	stripeResults []stripeResult
}

type encodeChannelSpec struct {
	id    int
	plane []uint8
}

type encodeChannelResult struct {
	blockCount   uint32
	sizeBytes    []byte
	patternBytes []byte
	fgVals       []uint8
	bgVals       []uint8
	err          error
}

type stripeResult struct {
	sizeBuf     []byte
	sizeByte    byte
	sizeN       uint8
	patternBuf  []byte
	patternByte byte
	patternN    uint8
	fgVals      []uint8
	bgVals      []uint8
	blockCount  uint32
	err         error
}

// no pattern logging

func encodedBlockIsPattern(fg uint8) bool {
	return (fg & 1) != 0
}

func encodeBlockFlagIntoFG(fg uint8, isPattern bool, bg uint8) uint8 {
	wantOdd := isPattern
	if encodedBlockIsPattern(fg) == wantOdd && (!isPattern || fg != bg) {
		return fg
	}

	best := fg
	for dist := 1; dist < 256; dist++ {
		if down := int(fg) - dist; down >= 0 {
			candidate := uint8(down)
			if encodedBlockIsPattern(candidate) == wantOdd && (!isPattern || candidate != bg) {
				best = candidate
				break
			}
		}
		if up := int(fg) + dist; up <= 255 {
			candidate := uint8(up)
			if encodedBlockIsPattern(candidate) == wantOdd && (!isPattern || candidate != bg) {
				best = candidate
				break
			}
		}
	}
	return best
}

func encodeChannelWorker(e *Encoder, dst *encodeChannelResult, channelID int, plane []uint8, stride, w4, h4, fullW, fullH int, useMacro bool, scratch *encoderChannelScratch, wg *sync.WaitGroup) {
	defer wg.Done()
	blockCount, sizeBytes, patternBytes, fgVals, bgVals, err := e.encodeChannelReuse(channelID, plane, stride, w4, h4, fullW, fullH, useMacro, scratch)
	dst.blockCount = blockCount
	dst.sizeBytes = sizeBytes
	dst.patternBytes = patternBytes
	dst.fgVals = fgVals
	dst.bgVals = bgVals
	dst.err = err
}

func countSmallBlocks(width, height, small int) int {
	if width <= 0 || height <= 0 || small <= 0 {
		return 0
	}
	return (width / small) * (height / small)
}

func countTopLevelBlocks(fullW, fullH, top int) int {
	if fullW <= 0 || fullH <= 0 || top <= 0 {
		return 0
	}
	return (fullW / top) * (fullH / top)
}

// Encoder reuses large scratch buffers across Encode calls to reduce allocations.
// It is not safe for concurrent use. The returned []byte is reused and will be
// overwritten on the next Encode call.
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
	e.zenc = mustNewZstdEncoder()
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
	writePatternIndex := func(bits patternMask, bw, bh int) {
		idx := nearestPatternIndex(bits, bw, bh, e.patternCount)
		patternStream.writeBits(idx, patternBitCount)
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
					fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
					scratch.fgVals = append(scratch.fgVals, fg)
					if isPattern {
						writePatternIndex(patternMaskFromUint64(bits, 4), 2, 2)
						scratch.bgVals = append(scratch.bgVals, bg)
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
	if e.Parallel && useMacro && fullW > 0 && fullH > 0 {
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

					writePatternIndexLocal := func(bits patternMask, bw, bh int) {
						idx := nearestPatternIndex(bits, bw, bh, e.patternCount)
						patternS.writeBits(idx, patternBitCount)
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
							fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
							scratchLocal.fgVals = append(scratchLocal.fgVals, fg)
							if isPattern {
								writePatternIndexLocal(bits, size, size)
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
							fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
							scratchLocal.fgVals = append(scratchLocal.fgVals, fg)
							if isPattern {
								writePatternIndexLocal(bits, size, size)
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
			fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				writePatternIndex(bits, size, size)
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
			fg = encodeBlockFlagIntoFG(fg, isPattern, bg)
			scratch.fgVals = append(scratch.fgVals, fg)
			if isPattern {
				writePatternIndex(bits, size, size)
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
				writePatternIndex(bits, smallBlock, smallBlock)
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
				writePatternIndex(bits, smallBlock, smallBlock)
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

	e.ensurePlanes(w, h)
	if e.Parallel {
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

	if e.Parallel {
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
	visW, visH := visibleBlockDims(imgW, imgH, x0, y0, bw, bh)
	for yy := 0; yy < visH; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < visW; xx++ {
			val := bg
			if testPatternBit(pattern, bw, bh, xx, yy) {
				val = fg
			}
			o := row + xx*4
			if o < 0 || o >= len(pix) {
				return fmt.Errorf("drawBlockIntoPix: index out of range")
			}
			pix[o] = val
		}
	}
	return nil
}

// drawBlockIntoPixFull assumes the block is fully inside the image.
func drawBlockIntoPixFull(pix []byte, strideBytes, imgW int, x0, y0, bw, bh int, br *bitReader, fg, bg uint8, patternCount int, channelOffset int) error {
	pattern, err := readPatternComposite(br, bw, bh, patternCount)
	if err != nil {
		return err
	}
	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < bw; xx++ {
			val := bg
			if testPatternBit(pattern, bw, bh, xx, yy) {
				val = fg
			}
			pix[row+xx*4] = val
		}
	}
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

	payload []byte
	zdec    *zstd.Decoder

	dst *image.RGBA
}

func NewDecoder() *Decoder {
	return &Decoder{Parallel: true, zdec: mustNewZstdDecoder()}
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
	for yy := 0; yy < visH; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < visW; xx++ {
			val := bg
			if testPatternBit(pattern, bw, bh, xx, yy) {
				val = fg
			}
			o := row + xx*4
			if o < 0 || o >= len(pix) {
				return fmt.Errorf("drawBlockIntoPixMask: index out of range")
			}
			pix[o] = val
		}
	}
	return nil
}

func drawBlockIntoPixFullMask(pix []byte, strideBytes int, x0, y0, bw, bh int, pattern patternMask, fg, bg uint8, channelOffset int) {
	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*strideBytes + x0*4 + channelOffset
		for xx := 0; xx < bw; xx++ {
			val := bg
			if testPatternBit(pattern, bw, bh, xx, yy) {
				val = fg
			}
			pix[row+xx*4] = val
		}
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

func (d *Decoder) Decode(compData []byte) (*image.RGBA, error) {
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
	parallelDraw := d.Parallel && !hasCb && !hasCr
	if hasChromaGrid {
		errY = decodeChannelToPix(ySeg, imgW, imgH, levels, d.patternCount, pix, stride, 0, parallelDraw)
		if errY != nil {
			return nil, errY
		}
		nextPos, err := d.decodeChromaGridOverlayIntoPix(payload, pos, imgW, imgH, pix, stride)
		if err != nil {
			return nil, err
		}
		pos = nextPos
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

	return dst, nil
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
)

func initYCbCrTables() {
	ycbcrOnce.Do(func() {
		for i := 0; i < 256; i++ {
			cb := int32(i) - 128
			cr := int32(i) - 128
			cbToG[i] = int16((22554 * cb) >> 16)
			cbToB[i] = int16((116130 * cb) >> 16)
			crToR[i] = int16((91881 * cr) >> 16)
			crToG[i] = int16((46802 * cr) >> 16)
		}
	})
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
