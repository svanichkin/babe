// BABE (Bi-Level Adaptive Block Encoding) is a dual-tone block codec for images.
// It operates in YCbCr color space, uses adaptive block sizes (small and macro blocks)
// and per-channel bi-level patterns, plus a light post-process for deblocking
// and gradient smoothing.

package classic

import (
	"bytes"
	"fmt"
	"image"
	"io"
	"runtime"
	"sync"
)

const (
	codec = "BABE\n"
)

var (
	ycbcrOnce sync.Once
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

// Default block sizes; these are overridden by compression presets via setBlocksForQuality.
var (
	// base size of the small block (in pixels)
	smallBlock = 1
	// size of the macroblock (in pixels)
	macroBlock = 2

	// codecStateMu serializes access to global codec configuration/state.
	// The current codec implementation still uses package-level block settings
	// and mode flags during encode/decode.
	codecStateMu sync.Mutex
)

// Current encode quality and its fixed-point profile. Access is serialized by
// codecStateMu, including the per-channel workers started by an encode.
var encQuality int = 70
var qualityProfileFP int = 70 * 256

// channel presence flags for the header; at least Y must be set.
const (
	channelFlagY        = 1 << 0
	channelFlagCb       = 1 << 1
	channelFlagCr       = 1 << 2
	channelFlagLossless = 1 << 3
	channelFlagGradient = 1 << 4
)

// Protected by codecStateMu, including channel workers.
var gradientMode bool

// encodeBW toggles grayscale mode; when true, only the Y channel is stored.
var encodeBW bool

// Quality mapping:
// - quality is in [0..100]
// - smallBlock/macroBlock are chosen from a small set of presets
//   to trade off quality vs. compression.

// setBlocksForQuality configures the block sizes from the calibrated quality profile.
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
	qualityProfileFP = profileForQuality(quality)

	switch {
	case qualityProfileFP >= 75*256:
		smallBlock, macroBlock = 1, 2
	case qualityProfileFP >= 40*256:
		smallBlock, macroBlock = 2, 4
	default:
		smallBlock, macroBlock = 4, 8
	}

	return nil
}

// bitWriter writes bits to a bytes.Buffer (msb-first in each byte).
// It avoids interface-based io.ByteWriter to keep allocations low.
type bitWriter struct {
	buf  *bytes.Buffer
	byte byte
	n    uint8 // number of bits written (0..8)
}

func newBitWriter(buf *bytes.Buffer) bitWriter {
	return bitWriter{buf: buf}
}

// writeBit writes a single bit (msb-first in byte).
func (bw *bitWriter) writeBit(bit bool) {
	bw.byte <<= 1
	if bit {
		bw.byte |= 1
	}
	bw.n++
	if bw.n == 8 {
		_ = bw.buf.WriteByte(bw.byte)
		bw.byte = 0
		bw.n = 0
	}
}

// writeBits writes n bits from bits (msb-first within the provided n bits).
// For example, if n=4 and bits=0b1011, this writes: 1,0,1,1.
func (bw *bitWriter) writeBits(bits uint64, n uint8) {
	for n > 0 {
		free := uint8(8 - bw.n)
		if free == 0 {
			_ = bw.buf.WriteByte(bw.byte)
			bw.byte = 0
			bw.n = 0
			free = 8
		}

		k := free
		if k > n {
			k = n
		}

		shift := n - k
		chunk := uint8((bits >> shift) & ((1 << k) - 1))

		bw.byte = (bw.byte << k) | byte(chunk)
		bw.n += k
		n -= k

		if bw.n == 8 {
			_ = bw.buf.WriteByte(bw.byte)
			bw.byte = 0
			bw.n = 0
		}
	}
}

// flush writes any remaining bits, left-padded with zeros.
func (bw *bitWriter) flush() {
	if bw.n > 0 {
		bw.byte <<= 8 - bw.n
		_ = bw.buf.WriteByte(bw.byte)
		bw.byte = 0
		bw.n = 0
	}
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

// readBitFast is a no-error variant of readBit. The caller must ensure
// there is enough input data remaining.
func (br *bitReader) readBitFast() bool {
	b := br.data[br.idx]
	isSet := (b & (1 << (7 - br.bit))) != 0
	br.bit++
	if br.bit == 8 {
		br.bit = 0
		br.idx++
	}
	return isSet
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

// readBitsFast is a no-error variant of readBits. The caller must ensure
// there is enough input data remaining, and 1 <= n <= 8.
func (br *bitReader) readBitsFast(n uint8) uint8 {
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
		return out
	}

	// Need bits from the next byte as well.
	b0 := br.data[br.idx]
	b1 := br.data[br.idx+1]

	first := uint8(b0 & byte((1<<rem)-1)) // lower "rem" bits
	n2 := n - rem
	second := uint8(b1 >> (8 - n2))

	out := (first << n2) | second
	br.idx++
	br.bit = n2
	return out
}

// channel IDs for Y, Cb, Cr.
const (
	chY  = 0
	chCb = 1
	chCr = 2
)

// extractYCbCrPlanes converts an image.Image into three planar Y, Cb, Cr slices.
// Each plane has size w*h and is indexed as plane[y*w + x] with 0 <= x < w and 0 <= y < h.
//
// For common concrete types (RGBA/NRGBA) we bypass img.At/RGBA() and read pixels
// directly from the backing Pix slice to reduce allocations and overhead.
func extractYCbCrPlanes(img image.Image) ([]uint8, []uint8, []uint8, int, int) {
	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()

	yPlane := make([]uint8, w*h)
	cbPlane := make([]uint8, w*h)
	crPlane := make([]uint8, w*h)

	extractYCbCrPlanesInto(img, yPlane, cbPlane, crPlane)
	return yPlane, cbPlane, crPlane, w, h
}

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

func extractYCbCrFromRGBASequential(src *image.RGBA, yPlane, cbPlane, crPlane []uint8, w, h int) {
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

// allowedMacroSpreadForQuality returns the maximum spread as an 8.8 fixed-point
// value. The threshold is interpolated within each compatible block geometry.
func allowedMacroSpreadForQuality(quality int) int32 {
	fp := profileForQuality(quality)
	if fp >= 100*256 {
		fp = 100*256 - 1
	}
	switch {
	case fp >= 75*256:
		rem := 100*256 - fp
		return 256 + int32(int64(rem)*int64(rem)*63/(25*25*256))
	case fp >= 40*256:
		return 256*256 - int32((fp-40*256)*255/35)
	default:
		return 256*256 - int32(fp*255/40)
	}
}

// blockSpread distributes the fractional part of a threshold across blocks
// using their position. This avoids changing a large number of blocks at once
// when a threshold crosses an integer; it is deterministic across worker counts.
func blockSpread(spread int32, x, y int) int32 {
	whole, frac := spread>>8, uint32(spread&255)
	if frac == 0 {
		return whole
	}
	v := uint32(x)*0x1f123bb5 ^ uint32(y)*0x5f356495
	v ^= v >> 16
	v *= 0x7feb352d
	v ^= v >> 15
	v *= 0x846ca68b
	v ^= v >> 16
	if v&255 < frac {
		whole++
	}
	return whole
}

// canUseBigBlockChannel reports whether a macroblock's spread fits its threshold.
func canUseBigBlockChannel(plane []uint8, stride, height, x0, y0 int, spread int32) bool {
	if spread <= 0 {
		return false
	}
	if stride <= 0 || height <= 0 || x0 < 0 || y0 < 0 || macroBlock <= 0 {
		return false
	}
	if x0+macroBlock > stride || y0+macroBlock > height {
		return false
	}

	if macroBlock == 2 {
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

	for yy := 0; yy < macroBlock; yy++ {
		row := (y0+yy)*stride + x0
		for xx := 0; xx < macroBlock; xx++ {
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

// encodeBlockPlane encodes a single block for one planar channel:
// - computes a mean-based threshold
// - computes FG/BG levels
// - returns whether a bi-level pattern is actually needed (FG != BG)
// - writes pattern bits to pw only when needed
func writePatternComposite(pw *bitWriter, bits uint64, bw, bh int) {
	if pw == nil {
		return
	}
	if bw == 6 && bh == 6 && smallBlock == 3 && macroBlock == 6 {
		const child = 3
		for qy := 0; qy < 2; qy++ {
			for qx := 0; qx < 2; qx++ {
				var sub uint64
				for yy := 0; yy < child; yy++ {
					for xx := 0; xx < child; xx++ {
						srcY := qy*child + yy
						srcX := qx*child + xx
						srcShift := 35 - (srcY*6 + srcX)
						sub <<= 1
						if ((bits >> srcShift) & 1) != 0 {
							sub |= 1
						}
					}
				}
				pw.writeBits(sub, 9)
			}
		}
		return
	}
	if bw == 8 && bh == 8 && smallBlock == 4 && macroBlock == 8 {
		const child = 4
		for qy := 0; qy < 2; qy++ {
			for qx := 0; qx < 2; qx++ {
				var sub uint64
				for yy := 0; yy < child; yy++ {
					for xx := 0; xx < child; xx++ {
						srcY := qy*child + yy
						srcX := qx*child + xx
						srcShift := 63 - (srcY*8 + srcX)
						sub <<= 1
						if ((bits >> srcShift) & 1) != 0 {
							sub |= 1
						}
					}
				}
				pw.writeBits(sub, 16)
			}
		}
		return
	}
	pw.writeBits(bits, uint8(bw*bh))
}

func encodeBlockPlane(plane []uint8, stride, height, x0, y0, bw, bh, quantStep int, pw *bitWriter) (uint8, uint8, bool, error) {
	total := bw * bh
	if total <= 0 {
		return 0, 0, false, fmt.Errorf("invalid block size")
	}

	if bw == 1 && bh == 1 {
		if x0 < 0 || y0 < 0 || x0 >= stride {
			return 0, 0, false, fmt.Errorf("encodeBlockPlane: index out of range")
		}
		idx := y0*stride + x0
		if idx < 0 || idx >= len(plane) {
			return 0, 0, false, fmt.Errorf("encodeBlockPlane: index out of range")
		}
		v := quantizeLevel(plane[idx], quantStep)
		return v, v, false, nil
	}

	if x0 < 0 || y0 < 0 || bw <= 0 || bh <= 0 || stride <= 0 || height <= 0 || x0+bw > stride {
		return 0, 0, false, fmt.Errorf("encodeBlockPlane: index out of range")
	}
	if y0+bh > height {
		return 0, 0, false, fmt.Errorf("encodeBlockPlane: index out of range")
	}

	if bw == 2 && bh == 2 {
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
			v := quantizeLevel(avg, quantStep)
			return v, v, false, nil
		}
		fg := quantizeLevel(uint8(fgSum/uint64(fgCnt)), quantStep)
		bg := quantizeLevel(uint8(bgSum/uint64(bgCnt)), quantStep)
		if fg == bg {
			return fg, bg, false, nil
		}
		writePatternComposite(pw, bits, 2, 2)
		return fg, bg, true, nil
	}

	// Read the block once into a small stack buffer.
	var valsBuf [64]uint8
	if total > len(valsBuf) {
		return 0, 0, false, fmt.Errorf("encodeBlockPlane: block too large")
	}
	vals := valsBuf[:total]

	var sum uint64
	i := 0
	for yy := 0; yy < bh; yy++ {
		row := (y0+yy)*stride + x0
		for xx := 0; xx < bw; xx++ {
			v := plane[row+xx]
			vals[i] = v
			sum += uint64(v)
			i++
		}
	}
	thr := uint8(sum / uint64(total))

	var fgSum, bgSum uint64
	var fgCnt, bgCnt uint32

	var bits uint64
	for _, v := range vals {
		isFg := v >= thr
		bits <<= 1
		if isFg {
			bits |= 1
			fgSum += uint64(v)
			fgCnt++
		} else {
			bgSum += uint64(v)
			bgCnt++
		}
	}

	avg := uint8(sum / uint64(total))
	if fgCnt == 0 || bgCnt == 0 {
		v := quantizeLevel(avg, quantStep)
		return v, v, false, nil
	}
	fg := quantizeLevel(uint8(fgSum/uint64(fgCnt)), quantStep)
	bg := quantizeLevel(uint8(bgSum/uint64(bgCnt)), quantStep)
	if fg == bg {
		return fg, bg, false, nil
	}
	writePatternComposite(pw, bits, bw, bh)
	return fg, bg, true, nil
}

// encodeChannelBlock optionally replaces a low-contrast block with its
// quantized average. This reduces chroma detail at the lower quality levels.
func encodeChannelBlock(plane []uint8, stride, height, x0, y0, bw, bh, step int, solidThreshold int, pw *bitWriter) (uint8, uint8, bool, error) {
	if gradientMode && pw != nil && bw == bh && bw >= 4 && x0 >= 0 && y0 >= 0 && stride > 0 && height > 0 && x0+bw <= stride && y0+bh <= height && (y0+bh-1)*stride+x0+bw <= len(plane) {
		if start, end, vertical, ok := fitGradient(plane, stride, x0, y0, bw); ok {
			var bits uint64
			if vertical {
				bits = ^uint64(0) >> (64 - uint(bw*bh))
			}
			writePatternComposite(pw, bits, bw, bh)
			return start, end, true, nil
		}
	}
	if solidThreshold <= 0 {
		return encodeBlockPlane(plane, stride, height, x0, y0, bw, bh, step, pw)
	}
	if x0 < 0 || y0 < 0 || bw <= 0 || bh <= 0 || stride <= 0 || height <= 0 || x0+bw > stride || y0+bh > height || (y0+bh-1)*stride+x0+bw > len(plane) {
		return 0, 0, false, fmt.Errorf("encodeChannelBlock: index out of range")
	}
	var sum int
	minV, maxV := byte(255), byte(0)
	if solidThreshold >= 256 {
		for y := 0; y < bh; y++ {
			row := (y0+y)*stride + x0
			for x := 0; x < bw; x++ {
				sum += int(plane[row+x])
			}
		}
	} else {
		for y := 0; y < bh; y++ {
			row := (y0+y)*stride + x0
			for x := 0; x < bw; x++ {
				v := plane[row+x]
				sum += int(v)
				minV = min(minV, v)
				maxV = max(maxV, v)
			}
		}
	}
	if solidThreshold < 256 && int(maxV)-int(minV) >= solidThreshold {
		return encodeBlockPlane(plane, stride, height, x0, y0, bw, bh, step, pw)
	}
	v := quantizeLevel(uint8(sum/(bw*bh)), step)
	return v, v, false, nil
}

type encoderChannelScratch struct {
	quantStep      int
	solidThreshold int
	sizeBuf        bytes.Buffer
	typeBuf        bytes.Buffer
	patternBuf     bytes.Buffer
	fgVals         []uint8
	bgVals         []uint8
}

type encodeChannelSpec struct {
	id    int
	plane []uint8
}

type encodeChannelResult struct {
	blockCount   uint32
	sizeBytes    []byte
	typeBytes    []byte
	patternBytes []byte
	fgVals       []uint8
	bgVals       []uint8
	err          error
}

func encodeChannelWorker(e *Encoder, dst *encodeChannelResult, plane []uint8, stride, w4, h4, fullW, fullH int, useMacro bool, scratch *encoderChannelScratch, wg *sync.WaitGroup) {
	defer wg.Done()
	blockCount, sizeBytes, typeBytes, patternBytes, fgVals, bgVals, err := e.encodeChannelReuse(plane, stride, w4, h4, fullW, fullH, useMacro, scratch)
	dst.blockCount = blockCount
	dst.sizeBytes = sizeBytes
	dst.typeBytes = typeBytes
	dst.patternBytes = patternBytes
	dst.fgVals = fgVals
	dst.bgVals = bgVals
	dst.err = err
}

// Encoder reuses large scratch buffers across Encode calls to reduce allocations.
// It is not safe for concurrent use. The returned []byte is reused and will be
// overwritten on the next Encode call.
