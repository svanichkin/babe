// BABE (Bi-Level Adaptive Block Encoding) is a multi-level block codec for images.
// It operates in YCbCr color space, uses an N-level adaptive block hierarchy,
// per-channel bi-level patterns,
// plus a light post-process for deblocking and gradient smoothing.

package light

import (
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
)

const (
	codec = "BABE-L\n"
)

const (
	parallelExtractMinPixels = 512 * 512
	parallelEncodeMinPixels  = 512 * 512
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

var activeFilmGrain = 0.0
var activePatternBlur = 0

const DefaultPatternCount = defaultPatternCount

func BlocksFromSpec(spec string) ([]int, error) {
	return blocksFromSpec(spec)
}

func SetPostFilterOptions(filmGrain float64, patternBlur int) {
	activeFilmGrain = filmGrain
	activePatternBlur = patternBlur
}

func filmGrainNoiseByte(x, y int) byte {
	v := uint32((x & 63) | ((y & 63) << 6))
	v ^= v >> 16
	v *= 0x7feb352d
	v ^= v >> 15
	v *= 0x846ca68b
	v ^= v >> 16
	return byte(v)
}

func blurBlockChannel(pix []byte, strideBytes, imgW, imgH, x0, y0, bw, bh, channelOffset, radius int) {
	if radius <= 0 || bw <= 0 || bh <= 0 {
		return
	}
	x1 := min(x0+bw, imgW)
	y1 := min(y0+bh, imgH)
	if x0 < 0 || y0 < 0 || x0 >= x1 || y0 >= y1 {
		return
	}
	tmpW := x1 - x0
	tmpH := y1 - y0
	pass := func(src []uint8, dst []uint8) {
		for y := 0; y < tmpH; y++ {
			for x := 0; x < tmpW; x++ {
				sum := 0
				count := 0
				for yy := max(0, y-radius); yy <= min(tmpH-1, y+radius); yy++ {
					for xx := max(0, x-radius); xx <= min(tmpW-1, x+radius); xx++ {
						sum += int(src[yy*tmpW+xx])
						count++
					}
				}
				dst[y*tmpW+x] = uint8(sum / count)
			}
		}
	}

	tmpA := make([]uint8, tmpW*tmpH)
	tmpB := make([]uint8, tmpW*tmpH)
	for y := 0; y < tmpH; y++ {
		row := (y0+y)*strideBytes + x0*4 + channelOffset
		for x := 0; x < tmpW; x++ {
			tmpA[y*tmpW+x] = pix[row+x*4]
		}
	}
	pass(tmpA, tmpB)
	pass(tmpB, tmpA)
	for y := 0; y < tmpH; y++ {
		row := (y0+y)*strideBytes + x0*4 + channelOffset
		for x := 0; x < tmpW; x++ {
			pix[row+x*4] = tmpA[y*tmpW+x]
		}
	}
}

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
		ratio := maxV / minV
		if minV == maxV {
			return []int{minV}, nil
		}
		if ratio < 1 || minV*ratio != maxV {
			return nil, fmt.Errorf("blocks range %q must end on an integer geometric step", spec)
		}
		for factor := 2; factor <= ratio; factor++ {
			sizes := make([]int, 0, 8)
			v := minV
			for {
				sizes = append(sizes, v)
				if v == maxV {
					return normalizeLevels(sizes), nil
				}
				if v > maxV/factor {
					break
				}
				v *= factor
			}
		}
		return nil, fmt.Errorf("blocks range %q must form an integer geometric ladder, e.g. 15-120 or 3-27", spec)
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
	for i := 1; i < len(sizes); i++ {
		if sizes[i]%sizes[i-1] != 0 {
			return nil, fmt.Errorf("each block size must be divisible by the previous one, got %v", sizes)
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

func applyFilmGrainBlockY(pix []byte, stride, w, h, x0, y0, blockSize int, amount float64) {
	if amount <= 0 {
		return
	}
	if blockSize <= 0 {
		return
	}
	size := blockSize / 2
	if size < 1 {
		size = 1
	}
	baseAmp := amount * 6.0
	blockW := min(x0+blockSize, w)
	blockH := min(y0+blockSize, h)
	applyCell := func(bx, by int, n float64) {
		x1 := min(bx+size, blockW)
		y1 := min(by+size, blockH)
		cx := bx + (x1-bx)/2
		cy := by + (y1-by)/2
		o := cy*stride + cx*4
		luma := float64(pix[o+0]) / 255.0
		shadowWeight := 1.0 - luma
		shadowWeight *= shadowWeight
		if shadowWeight <= 0 {
			return
		}
		delta := int(n * 2.0 * baseAmp * shadowWeight)
		if delta == 0 {
			return
		}
		for y := by; y < y1; y++ {
			row := y * stride
			for x := bx; x < x1; x++ {
				p := row + x*4
				v := int(pix[p+0]) + delta
				if v < 0 {
					v = 0
				} else if v > 255 {
					v = 255
				}
				pix[p+0] = uint8(v)
			}
		}
	}

	for by := y0; by < blockH; by += size {
		for bx := x0; bx < blockW; bx += size {
			n := float64(filmGrainNoiseByte(bx/size, by/size))/255.0 - 0.5
			applyCell(bx, by, n)
		}
	}
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
	initYCbCrTables()
	yr := yFromR[:]
	yg := yFromG[:]
	yb := yFromB[:]
	cbr := cbFromR[:]
	cbg := cbFromG[:]
	cbb := cbFromB[:]
	crr := crFromR[:]
	crg := crFromG[:]
	crb := crFromB[:]
	for y := yStart; y < yEnd; y++ {
		idx := y * w
		pixRow := y * stride
		rowEnd := pixRow + w*4
		p := pixRow
		for ; p+16 <= rowEnd; p += 16 {
			r0, g0, b0 := pix[p+0], pix[p+1], pix[p+2]
			r1, g1, b1 := pix[p+4], pix[p+5], pix[p+6]
			r2, g2, b2 := pix[p+8], pix[p+9], pix[p+10]
			r3, g3, b3 := pix[p+12], pix[p+13], pix[p+14]

			yPlane[idx+0] = uint8((yr[r0] + yg[g0] + yb[b0]) >> 8)
			cbPlane[idx+0] = uint8(((cbr[r0] + cbg[g0] + cbb[b0]) >> 8) + 128)
			crPlane[idx+0] = uint8(((crr[r0] + crg[g0] + crb[b0]) >> 8) + 128)

			yPlane[idx+1] = uint8((yr[r1] + yg[g1] + yb[b1]) >> 8)
			cbPlane[idx+1] = uint8(((cbr[r1] + cbg[g1] + cbb[b1]) >> 8) + 128)
			crPlane[idx+1] = uint8(((crr[r1] + crg[g1] + crb[b1]) >> 8) + 128)

			yPlane[idx+2] = uint8((yr[r2] + yg[g2] + yb[b2]) >> 8)
			cbPlane[idx+2] = uint8(((cbr[r2] + cbg[g2] + cbb[b2]) >> 8) + 128)
			crPlane[idx+2] = uint8(((crr[r2] + crg[g2] + crb[b2]) >> 8) + 128)

			yPlane[idx+3] = uint8((yr[r3] + yg[g3] + yb[b3]) >> 8)
			cbPlane[idx+3] = uint8(((cbr[r3] + cbg[g3] + cbb[b3]) >> 8) + 128)
			crPlane[idx+3] = uint8(((crr[r3] + crg[g3] + crb[b3]) >> 8) + 128)

			idx += 4
		}
		for ; p < rowEnd; p += 4 {
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			yPlane[idx] = uint8((yr[r8] + yg[g8] + yb[b8]) >> 8)
			cbPlane[idx] = uint8(((cbr[r8] + cbg[g8] + cbb[b8]) >> 8) + 128)
			crPlane[idx] = uint8(((crr[r8] + crg[g8] + crb[b8]) >> 8) + 128)
			idx++
		}
	}
}

func extractYCbCrFromRGBASequential(src *image.RGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
	stride := src.Stride
	pix := src.Pix
	initYCbCrTables()
	yr := yFromR[:]
	yg := yFromG[:]
	yb := yFromB[:]
	cbr := cbFromR[:]
	cbg := cbFromG[:]
	cbb := cbFromB[:]
	crr := crFromR[:]
	crg := crFromG[:]
	crb := crFromB[:]

	for y := 0; y < h; y++ {
		idx := y * w
		pixRow := y * stride
		rowEnd := pixRow + w*4
		p := pixRow
		for ; p+16 <= rowEnd; p += 16 {
			r0, g0, b0 := pix[p+0], pix[p+1], pix[p+2]
			r1, g1, b1 := pix[p+4], pix[p+5], pix[p+6]
			r2, g2, b2 := pix[p+8], pix[p+9], pix[p+10]
			r3, g3, b3 := pix[p+12], pix[p+13], pix[p+14]

			yPlane[idx+0] = uint8((yr[r0] + yg[g0] + yb[b0]) >> 8)
			cbPlane[idx+0] = uint8(((cbr[r0] + cbg[g0] + cbb[b0]) >> 8) + 128)
			crPlane[idx+0] = uint8(((crr[r0] + crg[g0] + crb[b0]) >> 8) + 128)

			yPlane[idx+1] = uint8((yr[r1] + yg[g1] + yb[b1]) >> 8)
			cbPlane[idx+1] = uint8(((cbr[r1] + cbg[g1] + cbb[b1]) >> 8) + 128)
			crPlane[idx+1] = uint8(((crr[r1] + crg[g1] + crb[b1]) >> 8) + 128)

			yPlane[idx+2] = uint8((yr[r2] + yg[g2] + yb[b2]) >> 8)
			cbPlane[idx+2] = uint8(((cbr[r2] + cbg[g2] + cbb[b2]) >> 8) + 128)
			crPlane[idx+2] = uint8(((crr[r2] + crg[g2] + crb[b2]) >> 8) + 128)

			yPlane[idx+3] = uint8((yr[r3] + yg[g3] + yb[b3]) >> 8)
			cbPlane[idx+3] = uint8(((cbr[r3] + cbg[g3] + cbb[b3]) >> 8) + 128)
			crPlane[idx+3] = uint8(((crr[r3] + crg[g3] + crb[b3]) >> 8) + 128)

			idx += 4
		}
		for ; p < rowEnd; p += 4 {
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			yPlane[idx] = uint8((yr[r8] + yg[g8] + yb[b8]) >> 8)
			cbPlane[idx] = uint8(((cbr[r8] + cbg[g8] + cbb[b8]) >> 8) + 128)
			crPlane[idx] = uint8(((crr[r8] + crg[g8] + crb[b8]) >> 8) + 128)
			idx++
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
	initYCbCrTables()
	for y := yStart; y < yEnd; y++ {
		idx := y * w
		pixRow := y * stride
		rowEnd := pixRow + w*4
		for p := pixRow; p < rowEnd; p += 4 {
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			yPlane[idx] = uint8((yFromR[r8] + yFromG[g8] + yFromB[b8]) >> 8)
			cbPlane[idx] = uint8(((cbFromR[r8] + cbFromG[g8] + cbFromB[b8]) >> 8) + 128)
			crPlane[idx] = uint8(((crFromR[r8] + crFromG[g8] + crFromB[b8]) >> 8) + 128)
			idx++
		}
	}
}

func extractYCbCrFromNRGBASequential(src *image.NRGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
	stride := src.Stride
	pix := src.Pix
	initYCbCrTables()

	for y := 0; y < h; y++ {
		idx := y * w
		pixRow := y * stride
		rowEnd := pixRow + w*4
		for p := pixRow; p < rowEnd; p += 4 {
			r8 := pix[p+0]
			g8 := pix[p+1]
			b8 := pix[p+2]
			yPlane[idx] = uint8((yFromR[r8] + yFromG[g8] + yFromB[b8]) >> 8)
			cbPlane[idx] = uint8(((cbFromR[r8] + cbFromG[g8] + cbFromB[b8]) >> 8) + 128)
			crPlane[idx] = uint8(((crFromR[r8] + crFromG[g8] + crFromB[b8]) >> 8) + 128)
			idx++
		}
	}
}

func extractYCbCrFromYCbCr(src *image.YCbCr, yPlane, cbPlane, crPlane []uint8, w, h int) {
	if extractYCbCrFromYCbCrFastRows(src, yPlane, cbPlane, crPlane, w, 0, h) {
		return
	}

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

func extractYCbCrFromYCbCrFastRows(src *image.YCbCr, yPlane, cbPlane, crPlane []uint8, w, yStart, yEnd int) bool {
	yStride := src.YStride
	cStride := src.CStride
	yPix := src.Y
	cbPix := src.Cb
	crPix := src.Cr

	switch src.SubsampleRatio {
	case image.YCbCrSubsampleRatio444:
		for y := yStart; y < yEnd; y++ {
			base := y * w
			copy(yPlane[base:base+w], yPix[y*yStride:y*yStride+w])
			cRow := y * cStride
			copy(cbPlane[base:base+w], cbPix[cRow:cRow+w])
			copy(crPlane[base:base+w], crPix[cRow:cRow+w])
		}
		return true
	case image.YCbCrSubsampleRatio422:
		for y := yStart; y < yEnd; y++ {
			base := y * w
			copy(yPlane[base:base+w], yPix[y*yStride:y*yStride+w])
			dstCb := cbPlane[base : base+w]
			dstCr := crPlane[base : base+w]
			si := y * cStride
			for di := 0; di < w; {
				cbv := cbPix[si]
				crv := crPix[si]
				si++
				dstCb[di] = cbv
				dstCr[di] = crv
				di++
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
			}
		}
		return true
	case image.YCbCrSubsampleRatio420:
		for y := yStart; y < yEnd; y++ {
			base := y * w
			copy(yPlane[base:base+w], yPix[y*yStride:y*yStride+w])
			dstCb := cbPlane[base : base+w]
			dstCr := crPlane[base : base+w]
			si := (y >> 1) * cStride
			for di := 0; di < w; {
				cbv := cbPix[si]
				crv := crPix[si]
				si++
				dstCb[di] = cbv
				dstCr[di] = crv
				di++
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
			}
		}
		return true
	case image.YCbCrSubsampleRatio440:
		for y := yStart; y < yEnd; y++ {
			base := y * w
			copy(yPlane[base:base+w], yPix[y*yStride:y*yStride+w])
			cRow := (y >> 1) * cStride
			copy(cbPlane[base:base+w], cbPix[cRow:cRow+w])
			copy(crPlane[base:base+w], crPix[cRow:cRow+w])
		}
		return true
	case image.YCbCrSubsampleRatio411:
		for y := yStart; y < yEnd; y++ {
			base := y * w
			copy(yPlane[base:base+w], yPix[y*yStride:y*yStride+w])
			dstCb := cbPlane[base : base+w]
			dstCr := crPlane[base : base+w]
			si := y * cStride
			for di := 0; di < w; {
				cbv := cbPix[si]
				crv := crPix[si]
				si++
				dstCb[di] = cbv
				dstCr[di] = crv
				di++
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
			}
		}
		return true
	case image.YCbCrSubsampleRatio410:
		for y := yStart; y < yEnd; y++ {
			base := y * w
			copy(yPlane[base:base+w], yPix[y*yStride:y*yStride+w])
			dstCb := cbPlane[base : base+w]
			dstCr := crPlane[base : base+w]
			si := (y >> 1) * cStride
			for di := 0; di < w; {
				cbv := cbPix[si]
				crv := crPix[si]
				si++
				dstCb[di] = cbv
				dstCr[di] = crv
				di++
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
				if di < w {
					dstCb[di] = cbv
					dstCr[di] = crv
					di++
				}
			}
		}
		return true
	default:
		return false
	}
}

func extractYCbCrFromYCbCrStripe(src *image.YCbCr, minX, minY, w int, yPlane, cbPlane, crPlane []uint8, yStart, yEnd int, wg *sync.WaitGroup) {
	defer wg.Done()
	if extractYCbCrFromYCbCrFastRows(src, yPlane, cbPlane, crPlane, w, yStart, yEnd) {
		return
	}
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
	if extractYCbCrFromYCbCrFastRows(src, yPlane, cbPlane, crPlane, w, 0, h) {
		return
	}

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

	d.decodeChromaGridIntoRGB(cbGrid, crGrid, step, gridW, gridH, w, h, pix, strideBytes)
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

func (d *Decoder) ensureChromaBandScratch(workers, w int) []decoderChromaBandScratch {
	if workers < 1 {
		workers = 1
	}
	if len(d.chromaBands) < workers {
		d.chromaBands = make([]decoderChromaBandScratch, workers)
	}
	bands := d.chromaBands[:workers]
	for i := range bands {
		if cap(bands[i].cbTop) < w {
			bands[i].cbTop = make([]uint32, w)
			bands[i].cbBot = make([]uint32, w)
			bands[i].crTop = make([]uint32, w)
			bands[i].crBot = make([]uint32, w)
			continue
		}
		bands[i].cbTop = bands[i].cbTop[:w]
		bands[i].cbBot = bands[i].cbBot[:w]
		bands[i].crTop = bands[i].crTop[:w]
		bands[i].crBot = bands[i].crBot[:w]
	}
	return bands
}

func (d *Decoder) decodeChromaGridIntoRGB(cbGrid, crGrid []byte, step, gridW, gridH, w, h int, pix []byte, strideBytes int) {
	if w == 0 || h == 0 || step <= 0 {
		return
	}
	d.ensureChromaX(w)
	d.fillChromaX(step, gridW, w)
	initYCbCrTables()

	workers := 1
	if d.Parallel && w*h >= 512*512 {
		workers = runtime.GOMAXPROCS(0)
		if workers < 1 {
			workers = 1
		}
		maxWorkers := max(h/64, 1)
		if workers > maxWorkers {
			workers = maxWorkers
		}
	}
	bands := d.ensureChromaBandScratch(workers, w)

	render := func(yStart, yEnd int, band *decoderChromaBandScratch) {
		for bandStart := yStart; bandStart < yEnd; {
			cellY := bandStart / step
			if cellY >= gridH-1 {
				cellY = gridH - 2
			}
			rowBase := cellY * gridW
			cbRow0 := cbGrid[rowBase : rowBase+gridW]
			cbRow1 := cbGrid[rowBase+gridW : rowBase+2*gridW]
			crRow0 := crGrid[rowBase : rowBase+gridW]
			crRow1 := crGrid[rowBase+gridW : rowBase+2*gridW]
			for x := 0; x < w; x++ {
				cellX := d.chromaXCell[x]
				fx := uint32(d.chromaXFX[x])
				fxInv := uint32(d.chromaXFXInv[x])

				band.cbTop[x] = uint32(cbRow0[cellX])*fxInv + uint32(cbRow0[cellX+1])*fx
				band.cbBot[x] = uint32(cbRow1[cellX])*fxInv + uint32(cbRow1[cellX+1])*fx
				band.crTop[x] = uint32(crRow0[cellX])*fxInv + uint32(crRow0[cellX+1])*fx
				band.crBot[x] = uint32(crRow1[cellX])*fxInv + uint32(crRow1[cellX+1])*fx
			}

			bandEnd := min((cellY+1)*step, yEnd)
			y0 := cellY * step
			for y := bandStart; y < bandEnd; y++ {
				fy := ((y - y0) << 8) / step
				if fy < 0 {
					fy = 0
				} else if fy > 255 {
					fy = 255
				}
				fyInv := 256 - fy
				row := y * strideBytes
				fyU := uint32(fy)
				fyInvU := uint32(fyInv)
				cbTop := band.cbTop
				cbBot := band.cbBot
				crTop := band.crTop
				crBot := band.crBot
				for x := 0; x < w; x++ {
					cb := uint8((cbTop[x]*fyInvU + cbBot[x]*fyU + 32768) >> 16)
					cr := uint8((crTop[x]*fyInvU + crBot[x]*fyU + 32768) >> 16)

					o := row + x*4
					Y := int(pix[o+0])
					R := Y + int(crToR[cr])
					G := Y - int(cbToG[cb]) - int(crToG[cr])
					B := Y + int(cbToB[cb])

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
			bandStart = bandEnd
		}
	}

	if workers == 1 {
		render(0, h, &bands[0])
		return
	}

	rowsPerWorker := (h + workers - 1) / workers
	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := min(y0+rowsPerWorker, h)
		wg.Add(1)
		go func(worker, y0, y1 int) {
			defer wg.Done()
			render(y0, y1, &bands[worker])
		}(i, y0, y1)
	}
	wg.Wait()
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

func AllowedMacroSpreadForQuality(quality int) int32 {
	return allowedMacroSpreadForQuality(quality)
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

	zstdEncoderPool sync.Pool
	zstdDecoderPool sync.Pool
)

func init() {
	zstdEncoderPool.New = func() any { return mustNewZstdEncoder() }
	zstdDecoderPool.New = func() any { return mustNewZstdDecoder() }

	// Prewarm one codec instance so one-shot CLI runs don't pay cold init inside measured work.
	zstdEncoderPool.Put(mustNewZstdEncoder())
	zstdDecoderPool.Put(mustNewZstdDecoder())
}

type patternMask []uint64

type PatternMask = patternMask

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
	b.Grow(len(mask) * 8)
	var buf [8]byte
	for i, word := range mask {
		if i > 0 {
			b.WriteByte(':')
		}
		binary.LittleEndian.PutUint64(buf[:], word)
		b.Write(buf[:])
	}
	return b.String()
}

func PatternMaskKey(mask patternMask) string {
	return patternMaskKey(mask)
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

func FormatPatternBits(bits patternMask, bw, bh int) string {
	return formatPatternBits(bits, bw, bh)
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

func BuildPatternMask(bw, bh int, pred func(x, y int) bool) patternMask {
	return buildPatternMask(bw, bh, pred)
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
		{"00000000", "00011000", "00011000", "00011000", "00011000", "00011000", "00011000", "00000000"},
		{"00000000", "00110000", "00110000", "00110000", "00110000", "00110000", "00110000", "00000000"},
		{"00000000", "00000000", "00001100", "00001100", "00001100", "00001100", "00000000", "00000000"},
		{"00000000", "00000000", "00000110", "00000110", "00000110", "00000110", "00000000", "00000000"},
	}
}

func BasicPatternTemplates() [][]string {
	return basicPatternTemplates()
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

func estimateChannelBytes(w4, h4, fullW, fullH int, levels []int, patternCount int, includeSizeBits bool) int {
	if len(levels) == 0 {
		return 0
	}
	smallBlock := levels[0]
	topBlock := levels[len(levels)-1]
	worstBlockCount := countSmallBlocks(w4, h4, smallBlock)
	if worstBlockCount < 0 {
		worstBlockCount = 0
	}

	sizeBytes := 0
	if includeSizeBits {
		topLevelCount := countTopLevelBlocks(fullW, fullH, topBlock)
		worstDecisionBits := worstBlockCount - topLevelCount
		if worstDecisionBits < topLevelCount {
			worstDecisionBits = topLevelCount
		}
		if worstDecisionBits > 0 {
			sizeBytes = (worstDecisionBits + 7) / 8
		}
	}

	patternBits := w4 * h4
	if smallBlock == 1 {
		patternBits = fullW * fullH
	}
	patternBytes := 0
	if patternBits > 0 {
		patternBytes = (patternBits*int(patternIndexBitsForCount(patternCount)) + 7) / 8
	}

	// 4 u32 lengths/counts + streams + fg/bg packed bytes worst-case.
	return 16 + sizeBytes + patternBytes + worstBlockCount + worstBlockCount
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

func FixedPatternCodebook(bw, bh, count int) []patternMask {
	return fixedPatternCodebook(bw, bh, count)
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

func nearestPatternIndex(patternBits patternMask, bw, bh, patternCount int) (uint64, bool) {
	limit := patternCount
	if limit < 1 {
		limit = 1
	}
	book := fixedPatternCodebook(bw, bh, limit)
	bestIdx := 0
	bestDist := bw*bh + 1
	worstIdx := 0
	worstDist := -1
	total := bw * bh
	if len(patternBits) == 1 {
		p := patternBits[0]
		key := patternCacheKey{bw: bw, bh: bh, count: limit}
		patternIndexCacheMu.RLock()
		if m, ok := patternIndexCache[key]; ok {
			if idx, ok := m[p]; ok {
				patternIndexCacheMu.RUnlock()
				return uint64(idx), false
			}
		}
		patternIndexCacheMu.RUnlock()

		for i := 0; i < len(book); i++ {
			dist := bits.OnesCount64(p ^ book[i][0])
			if dist < bestDist {
				bestDist = dist
				bestIdx = i
			}
			if dist > worstDist {
				worstDist = dist
				worstIdx = i
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
			if dist > worstDist {
				worstDist = dist
				worstIdx = i
			}
		}
	}
	invert := total-worstDist < bestDist
	if invert {
		return uint64(worstIdx), true
	}
	return uint64(bestIdx), false
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
