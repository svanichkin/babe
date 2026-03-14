package main

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"sync"

	"github.com/klauspost/compress/zstd"
)

const codec = "BABE1B\n"

const (
	channelFlagY  = 1 << 0
	channelFlagCb = 1 << 1
	channelFlagCr = 1 << 2
)

const (
	colorModeYCbCr byte = iota
	colorModeRGB
	colorModePaletteRGB
)

const (
	yStorageRaw byte = iota
	yStoragePatternGrid
	yStoragePatternGridDirect
	yStoragePatternLayers
)

const (
	yLow  = 64
	yHigh = 192

	bwYLow  = 24
	bwYHigh = 236

	cbLow  = 64
	cbHigh = 192

	crLow  = 64
	crHigh = 192
)

const blueNoiseTileSize = 128
const (
	neighborhoodBackSpan    = 2
	neighborhoodForwardSpan = 3
	paletteTileSize         = 16
)

var (
	blueNoiseTileOnce sync.Once
	blueNoiseTile     []uint16
)

var stylizedColorPalette = [][3]uint8{
	{18, 12, 32},
	{42, 28, 78},
	{68, 40, 122},
	{28, 74, 196},
	{42, 140, 238},
	{94, 206, 255},
	{248, 170, 78},
	{255, 128, 110},
	{255, 104, 176},
	{255, 196, 214},
	{245, 231, 198},
	{255, 245, 236},
	{116, 74, 42},
	{188, 58, 52},
	{42, 120, 72},
	{22, 22, 22},
}

var rgbPrimaryPalette = [][3]uint8{
	{0, 0, 0},
	{255, 255, 255},
	{255, 0, 0},
	{0, 255, 0},
	{0, 0, 255},
}

var zxSpectrumPalette = [][3]uint8{
	{0, 0, 0},
	{0, 0, 205},
	{205, 0, 0},
	{205, 0, 205},
	{0, 205, 0},
	{0, 205, 205},
	{205, 205, 0},
	{205, 205, 205},
	{0, 0, 0},
	{0, 0, 255},
	{255, 0, 0},
	{255, 0, 255},
	{0, 255, 0},
	{0, 255, 255},
	{255, 255, 0},
	{255, 255, 255},
}

var sunsetPalette = [][3]uint8{
	{19, 17, 28},
	{63, 40, 86},
	{111, 76, 140},
	{255, 122, 87},
	{255, 176, 97},
	{255, 227, 170},
}

var pastelPalette = [][3]uint8{
	{38, 43, 68},
	{118, 126, 161},
	{176, 203, 255},
	{255, 214, 220},
	{255, 244, 214},
	{227, 255, 214},
}

var oceanPalette = [][3]uint8{
	{10, 22, 38},
	{18, 55, 86},
	{35, 103, 150},
	{67, 170, 208},
	{142, 222, 255},
	{228, 248, 255},
}

var forestPalette = [][3]uint8{
	{16, 24, 15},
	{42, 70, 38},
	{78, 118, 56},
	{132, 168, 88},
	{198, 210, 132},
	{240, 232, 196},
}

var cgaPalette = [][3]uint8{
	{0, 0, 0},
	{85, 255, 255},
	{255, 85, 255},
	{255, 255, 255},
}

var egaPalette = [][3]uint8{
	{0, 0, 0},
	{0, 0, 170},
	{0, 170, 0},
	{0, 170, 170},
	{170, 0, 0},
	{170, 0, 170},
	{170, 85, 0},
	{170, 170, 170},
	{85, 85, 85},
	{85, 85, 255},
	{85, 255, 85},
	{85, 255, 255},
	{255, 85, 85},
	{255, 85, 255},
	{255, 255, 85},
	{255, 255, 255},
}

var vgaPalette = [][3]uint8{
	{0, 0, 0},
	{0, 0, 170},
	{0, 170, 0},
	{0, 170, 170},
	{170, 0, 0},
	{170, 0, 170},
	{170, 85, 0},
	{170, 170, 170},
	{85, 85, 85},
	{85, 85, 255},
	{85, 255, 85},
	{85, 255, 255},
	{255, 85, 85},
	{255, 85, 255},
	{255, 255, 85},
	{255, 255, 255},
}

var c64Palette = [][3]uint8{
	{0, 0, 0},
	{255, 255, 255},
	{136, 57, 50},
	{103, 182, 189},
	{139, 63, 150},
	{85, 160, 73},
	{64, 49, 141},
	{191, 206, 114},
	{139, 84, 41},
	{87, 66, 0},
	{184, 105, 98},
	{80, 80, 80},
	{120, 120, 120},
	{148, 224, 137},
	{120, 105, 196},
	{159, 159, 159},
}

var gameBoyPalette = [][3]uint8{
	{15, 56, 15},
	{48, 98, 48},
	{139, 172, 15},
	{155, 188, 15},
}

var pico8Palette = [][3]uint8{
	{0, 0, 0},
	{29, 43, 83},
	{126, 37, 83},
	{0, 135, 81},
	{171, 82, 54},
	{95, 87, 79},
	{194, 195, 199},
	{255, 241, 232},
	{255, 0, 77},
	{255, 163, 0},
	{255, 236, 39},
	{0, 228, 54},
	{41, 173, 255},
	{131, 118, 156},
	{255, 119, 168},
	{255, 204, 170},
}

var db16Palette = [][3]uint8{
	{20, 12, 28},
	{68, 36, 52},
	{48, 52, 109},
	{78, 74, 78},
	{133, 76, 48},
	{52, 101, 36},
	{208, 70, 72},
	{117, 113, 97},
	{89, 125, 206},
	{210, 125, 44},
	{133, 149, 161},
	{109, 170, 44},
	{210, 170, 153},
	{109, 194, 202},
	{218, 212, 94},
	{222, 238, 214},
}

var nesPalette = [][3]uint8{
	{124, 124, 124},
	{0, 0, 252},
	{0, 0, 188},
	{68, 40, 188},
	{148, 0, 132},
	{168, 0, 32},
	{168, 16, 0},
	{136, 20, 0},
	{80, 48, 0},
	{0, 120, 0},
	{0, 104, 0},
	{0, 88, 0},
	{0, 64, 88},
	{0, 0, 0},
	{188, 188, 188},
	{248, 248, 248},
}

var ymcPalette = [][3]uint8{
	{0, 0, 0},
	{255, 255, 255},
	{255, 255, 0},
	{255, 0, 255},
	{0, 255, 255},
}

var ymcRGBPalette = [][3]uint8{
	{0, 0, 0},
	{255, 255, 255},
	{255, 0, 0},
	{0, 255, 0},
	{0, 0, 255},
	{255, 255, 0},
	{255, 0, 255},
	{0, 255, 255},
}

type bitWriter struct {
	buf  *bytes.Buffer
	byte byte
	n    uint8
}

func newBitWriter(buf *bytes.Buffer) bitWriter {
	return bitWriter{buf: buf}
}

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

func (bw *bitWriter) writeBits(bits uint64, n uint8) {
	for i := int(n) - 1; i >= 0; i-- {
		bw.writeBit(((bits >> i) & 1) != 0)
	}
}

func (bw *bitWriter) flush() {
	if bw.n == 0 {
		return
	}
	bw.byte <<= 8 - bw.n
	_ = bw.buf.WriteByte(bw.byte)
	bw.byte = 0
	bw.n = 0
}

type bitReader struct {
	data []byte
	idx  int
	bit  uint8
}

func newBitReader(data []byte) bitReader {
	return bitReader{data: data}
}

func (br *bitReader) readBit() (bool, error) {
	if br.idx >= len(br.data) {
		return false, io.EOF
	}
	b := br.data[br.idx]
	v := (b & (1 << (7 - br.bit))) != 0
	br.bit++
	if br.bit == 8 {
		br.bit = 0
		br.idx++
	}
	return v, nil
}

func (br *bitReader) readBits(n uint8) (uint8, error) {
	var out uint8
	for i := uint8(0); i < n; i++ {
		bit, err := br.readBit()
		if err != nil {
			return 0, err
		}
		out <<= 1
		if bit {
			out |= 1
		}
	}
	return out, nil
}

func (br *bitReader) readBitsInt(n int) (int, error) {
	out := 0
	for i := 0; i < n; i++ {
		bit, err := br.readBit()
		if err != nil {
			return 0, err
		}
		out <<= 1
		if bit {
			out |= 1
		}
	}
	return out, nil
}

type Encoder struct {
	Parallel bool

	yPlane  []uint8
	cbPlane []uint8
	crPlane []uint8

	raw  bytes.Buffer
	bw   *bufio.Writer
	comp []byte

	yBits  bytes.Buffer
	cbBits bytes.Buffer
	crBits bytes.Buffer

	errCurr []float64
	errNext []float64

	zenc *zstd.Encoder
}

type EncodeOptions struct {
	BW                 bool
	YBits              int
	CbBits             int
	CrBits             int
	Pattern            string
	TileSize           int
	BlockSize          int
	UseZstd            bool
	Shuffle            bool
	RGBMode            bool
	ZXMode             bool
	Palette            string
	RawPalette         bool
	RawTop16           bool
	RawTree            bool
	RawTreeAdapt       bool
	RawShift           int
	ReconstructPalette bool
	BlockSubset        bool

	PatternW int
	PatternH int
}

func NewEncoder() *Encoder {
	e := &Encoder{Parallel: true}
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

func normalizeEncodeOptions(opts EncodeOptions) (EncodeOptions, error) {
	if opts.BW {
		if opts.YBits < 1 || opts.YBits > 4 {
			return opts, fmt.Errorf("bw bit depth must be in [1..4], got %d", opts.YBits)
		}
		if opts.Pattern != "" {
			w, h, err := parsePatternDims(opts.Pattern)
			if err != nil {
				return opts, err
			}
			opts.PatternW = w
			opts.PatternH = h
		}
		opts.CbBits = 0
		opts.CrBits = 0
		return opts, nil
	}
	if opts.Pattern != "" {
		w, h, err := parsePatternDims(opts.Pattern)
		if err != nil {
			return opts, err
		}
		opts.PatternW = w
		opts.PatternH = h
	}
	if opts.YBits < 1 || opts.YBits > 4 {
		return opts, fmt.Errorf("Y bit depth must be in [1..4], got %d", opts.YBits)
	}
	if opts.CbBits < 1 || opts.CbBits > 4 {
		return opts, fmt.Errorf("Cb bit depth must be in [1..4], got %d", opts.CbBits)
	}
	if opts.CrBits < 1 || opts.CrBits > 4 {
		return opts, fmt.Errorf("Cr bit depth must be in [1..4], got %d", opts.CrBits)
	}
	if opts.TileSize != 0 {
		if opts.TileSize < 2 || opts.TileSize > 255 {
			return opts, fmt.Errorf("tile size must be in [2..255], got %d", opts.TileSize)
		}
	}
	if opts.BlockSize != 0 {
		if opts.BlockSize < 2 || opts.BlockSize > 255 {
			return opts, fmt.Errorf("block size must be in [2..255], got %d", opts.BlockSize)
		}
	}
	return opts, nil
}

func (e *Encoder) EncodeWithOptions(img image.Image, quality int, opts EncodeOptions) ([]byte, error) {
	if quality < 0 {
		quality = 0
	}
	if quality > 100 {
		quality = 100
	}
	if e.zenc == nil {
		e.zenc = mustNewZstdEncoder()
	}

	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()
	if w <= 0 || h <= 0 {
		return nil, fmt.Errorf("image has invalid size %dx%d", w, h)
	}
	opts, err := normalizeEncodeOptions(opts)
	if err != nil {
		return nil, err
	}

	e.ensurePlanes(w, h)
	paletteMode := (opts.ZXMode || opts.Palette != "") && !opts.BW
	if (opts.RGBMode || paletteMode) && !opts.BW {
		extractRGBPlanes(img, e.yPlane, e.cbPlane, e.crPlane, e.Parallel)
	} else {
		extractYCbCrPlanes(img, e.yPlane, e.cbPlane, e.crPlane, e.Parallel)
	}

	e.yBits.Reset()
	e.cbBits.Reset()
	e.crBits.Reset()

	channelsMask := byte(channelFlagY)
	yBits := opts.YBits
	cbBits := opts.CbBits
	crBits := opts.CrBits
	yPhaseX, yPhaseY := planeBlueNoisePhase(e.yPlane, w, h, 0x59)
	cbPhaseX, cbPhaseY := planeBlueNoisePhase(e.cbPlane, w, h, 0x83)
	crPhaseX, crPhaseY := planeBlueNoisePhase(e.crPlane, w, h, 0xC7)
	if opts.BW {
		e.ensureErrorBuffers(w)
		quantizePhotoLumaToBits(e.yPlane, w, h, quality, yBits, yPhaseX, yPhaseY, opts.Shuffle, &e.yBits, e.errCurr, e.errNext)
	} else {
		if paletteMode {
			e.cbBits.Reset()
			e.crBits.Reset()
			chosenPalette := paletteFromMode(opts.ZXMode, opts.Palette, e.yPlane, e.cbPlane, e.crPlane, w, h)
			yBits = max(1, bitsNeeded(len(chosenPalette)-1))
			cbBits = 0
			crBits = 0
			quantizePaletteIndices(e.yPlane, e.cbPlane, e.crPlane, w, h, quality, chosenPalette, yPhaseX, yPhaseY, cbPhaseX, cbPhaseY, crPhaseX, crPhaseY, opts.Shuffle, true, opts.TileSize, opts.BlockSize, opts.RawPalette, opts.RawTop16, opts.RawTree, opts.RawTreeAdapt, opts.RawShift, opts.ReconstructPalette, opts.BlockSubset, &e.yBits)
		} else {
			if cbBits > 0 {
				channelsMask |= channelFlagCb
			}
			if crBits > 0 {
				channelsMask |= channelFlagCr
			}
			quantizeColorToPalette(e.yPlane, e.cbPlane, e.crPlane, w, h, quality, yBits, cbBits, crBits, yPhaseX, yPhaseY, cbPhaseX, cbPhaseY, crPhaseX, crPhaseY, opts.Shuffle, opts.RGBMode, opts.ZXMode, opts.Palette, &e.yBits, &e.cbBits, &e.crBits)
		}
	}

	e.raw.Reset()
	e.bw.Reset(&e.raw)

	if _, err := e.bw.WriteString(codec); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(byte(quality)); err != nil {
		return nil, err
	}
	colorMode := colorModeYCbCr
	if paletteMode {
		colorMode = colorModePaletteRGB
	} else if opts.RGBMode && !opts.BW {
		colorMode = colorModeRGB
	}
	if err := e.bw.WriteByte(colorMode); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(byte(yBits)); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(byte(cbBits)); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(byte(crBits)); err != nil {
		return nil, err
	}
	yStorageMode := yStorageRaw
	patternW := 0
	patternH := 0
	if opts.BW && opts.PatternW > 0 && opts.PatternH > 0 {
		rawMono := append([]byte(nil), e.yBits.Bytes()...)
		e.yBits.Reset()
		if yBits == 1 {
			if opts.PatternW == 2 && opts.PatternH == 2 {
				repackMonoBitsToPatternGridDirect(rawMono, w, h, opts.PatternW, opts.PatternH, &e.yBits)
				yStorageMode = yStoragePatternGridDirect
			} else {
				repackMonoBitsToPatternGrid(rawMono, w, h, opts.PatternW, opts.PatternH, &e.yBits)
				yStorageMode = yStoragePatternGrid
			}
		} else {
			repackMultiLevelToPatternLayers(rawMono, w, h, yBits, opts.PatternW, opts.PatternH, &e.yBits)
			yStorageMode = yStoragePatternLayers
		}
		patternW = opts.PatternW
		patternH = opts.PatternH
	}
	if err := e.bw.WriteByte(yStorageMode); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(byte(patternW)); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(byte(patternH)); err != nil {
		return nil, err
	}
	if err := e.bw.WriteByte(channelsMask); err != nil {
		return nil, err
	}
	if err := writeU32BE(e.bw, uint32(w)); err != nil {
		return nil, err
	}
	if err := writeU32BE(e.bw, uint32(h)); err != nil {
		return nil, err
	}
	if err := writeBitPlane(e.bw, e.yBits.Bytes()); err != nil {
		return nil, err
	}
	if (channelsMask & channelFlagCb) != 0 {
		if err := writeBitPlane(e.bw, e.cbBits.Bytes()); err != nil {
			return nil, err
		}
	}
	if (channelsMask & channelFlagCr) != 0 {
		if err := writeBitPlane(e.bw, e.crBits.Bytes()); err != nil {
			return nil, err
		}
	}
	if err := e.bw.Flush(); err != nil {
		return nil, err
	}

	if opts.UseZstd {
		e.comp = e.zenc.EncodeAll(e.raw.Bytes(), e.comp[:0])
		return e.comp, nil
	}
	e.comp = append(e.comp[:0], e.raw.Bytes()...)
	return e.comp, nil
}

func (e *Encoder) Encode(img image.Image, quality int, bwmode bool, bwBits int) ([]byte, error) {
	return e.EncodeWithOptions(img, quality, EncodeOptions{
		BW:     bwmode,
		YBits:  bwBits,
		CbBits: 1,
		CrBits: 1,
	})
}

func (e *Encoder) EncodeTo(w io.Writer, img image.Image, quality int, bwmode bool, bwBits int) error {
	comp, err := e.Encode(img, quality, bwmode, bwBits)
	if err != nil {
		return err
	}
	_, err = w.Write(comp)
	return err
}

func Encode(img image.Image, quality int, bwmode bool, bwBits int) ([]byte, error) {
	return NewEncoder().Encode(img, quality, bwmode, bwBits)
}

func EncodeWithBits(img image.Image, quality, yBits, cbBits, crBits int) ([]byte, error) {
	return NewEncoder().EncodeWithOptions(img, quality, EncodeOptions{
		BW:      cbBits == 0 && crBits == 0,
		YBits:   yBits,
		CbBits:  cbBits,
		CrBits:  crBits,
		Pattern: "",
	})
}

type Decoder struct {
	Parallel bool

	payload []byte
	zdec    *zstd.Decoder

	yBits  []byte
	cbBits []byte
	crBits []byte

	dst *image.RGBA
}

func NewDecoder() *Decoder {
	return &Decoder{Parallel: true, zdec: mustNewZstdDecoder()}
}

func (d *Decoder) Decode(compData []byte, postfilter bool) (*image.RGBA, error) {
	payload := compData
	if len(compData) < len(codec) || string(compData[:len(codec)]) != codec {
		if d.zdec == nil {
			d.zdec = mustNewZstdDecoder()
		}
		var err error
		payload, err = d.zdec.DecodeAll(compData, d.payload[:0])
		if err != nil {
			return nil, fmt.Errorf("zstd decode: %w", err)
		}
		d.payload = payload
	} else {
		d.payload = append(d.payload[:0], compData...)
		payload = d.payload
	}

	pos := 0
	if len(payload) < len(codec) || string(payload[:len(codec)]) != codec {
		return nil, fmt.Errorf("bad magic")
	}
	pos += len(codec)

	if len(payload)-pos < 9 {
		return nil, fmt.Errorf("decode: truncated header")
	}
	quality := int(payload[pos])
	_ = quality
	pos++
	colorMode := payload[pos]
	pos++
	if colorMode != colorModeYCbCr && colorMode != colorModeRGB && colorMode != colorModePaletteRGB {
		return nil, fmt.Errorf("decode: invalid color mode %d", colorMode)
	}
	yBitDepth := int(payload[pos])
	pos++
	cbBitDepth := int(payload[pos])
	pos++
	crBitDepth := int(payload[pos])
	pos++
	yStorageMode := payload[pos]
	pos++
	patternW := int(payload[pos])
	pos++
	patternH := int(payload[pos])
	pos++
	if yBitDepth < 1 || yBitDepth > 8 {
		return nil, fmt.Errorf("decode: invalid Y bit depth %d", yBitDepth)
	}
	if cbBitDepth < 0 || cbBitDepth > 4 {
		return nil, fmt.Errorf("decode: invalid Cb bit depth %d", cbBitDepth)
	}
	if crBitDepth < 0 || crBitDepth > 4 {
		return nil, fmt.Errorf("decode: invalid Cr bit depth %d", crBitDepth)
	}
	if yStorageMode != yStorageRaw && yStorageMode != yStoragePatternGrid && yStorageMode != yStoragePatternGridDirect && yStorageMode != yStoragePatternLayers {
		return nil, fmt.Errorf("decode: invalid Y storage mode %d", yStorageMode)
	}
	if (yStorageMode == yStoragePatternGrid || yStorageMode == yStoragePatternGridDirect || yStorageMode == yStoragePatternLayers) && (patternW < 1 || patternH < 1) {
		return nil, fmt.Errorf("decode: invalid pattern grid %dx%d", patternW, patternH)
	}
	channelsMask := payload[pos]
	pos++
	if channelsMask&channelFlagY == 0 {
		return nil, fmt.Errorf("decode: Y channel missing in header")
	}

	imgW, err := readU32BE(payload, &pos, "image width")
	if err != nil {
		return nil, err
	}
	imgH, err := readU32BE(payload, &pos, "image height")
	if err != nil {
		return nil, err
	}

	yBits, err := readBitPlane(payload, &pos, "Y")
	if err != nil {
		return nil, err
	}
	d.yBits = append(d.yBits[:0], yBits...)

	hasCb := channelsMask&channelFlagCb != 0
	hasCr := channelsMask&channelFlagCr != 0

	d.cbBits = d.cbBits[:0]
	if hasCb {
		cbBits, err := readBitPlane(payload, &pos, "Cb")
		if err != nil {
			return nil, err
		}
		d.cbBits = append(d.cbBits[:0], cbBits...)
	}

	d.crBits = d.crBits[:0]
	if hasCr {
		crBits, err := readBitPlane(payload, &pos, "Cr")
		if err != nil {
			return nil, err
		}
		d.crBits = append(d.crBits[:0], crBits...)
	}

	if d.dst == nil || d.dst.Bounds().Dx() != int(imgW) || d.dst.Bounds().Dy() != int(imgH) {
		d.dst = image.NewRGBA(image.Rect(0, 0, int(imgW), int(imgH)))
	}

	if err := decodeToRGBA(d.dst, d.yBits, d.cbBits, d.crBits, hasCb, hasCr, yBitDepth, cbBitDepth, crBitDepth, yStorageMode, patternW, patternH, colorMode, d.Parallel); err != nil {
		return nil, err
	}

	if postfilter {
		return smoothBlocks(d.dst), nil
	}
	return d.dst, nil
}

func (d *Decoder) DecodeFrom(r io.Reader, postfilter bool) (*image.RGBA, error) {
	compData, err := io.ReadAll(r)
	if err != nil {
		return nil, err
	}
	return d.Decode(compData, postfilter)
}

func Decode(compData []byte, postfilter bool) (image.Image, error) {
	return NewDecoder().Decode(compData, postfilter)
}

func extractYCbCrPlanes(img image.Image, yPlane, cbPlane, crPlane []uint8, parallel bool) {
	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()

	if !parallel || h < 32 {
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				i := y*w + x
				yc := color.YCbCrModel.Convert(img.At(b.Min.X+x, b.Min.Y+y)).(color.YCbCr)
				yPlane[i] = yc.Y
				cbPlane[i] = yc.Cb
				crPlane[i] = yc.Cr
			}
		}
		return
	}

	workers := min(runtime.NumCPU(), h)
	rowsPerWorker := (h + workers - 1) / workers
	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := min(y0+rowsPerWorker, h)
		wg.Add(1)
		go func(y0, y1 int) {
			defer wg.Done()
			for y := y0; y < y1; y++ {
				for x := 0; x < w; x++ {
					i := y*w + x
					yc := color.YCbCrModel.Convert(img.At(b.Min.X+x, b.Min.Y+y)).(color.YCbCr)
					yPlane[i] = yc.Y
					cbPlane[i] = yc.Cb
					crPlane[i] = yc.Cr
				}
			}
		}(y0, y1)
	}
	wg.Wait()
}

func extractRGBPlanes(img image.Image, rPlane, gPlane, bPlane []uint8, parallel bool) {
	b := img.Bounds()
	w := b.Dx()
	h := b.Dy()

	if !parallel || h < 32 {
		for y := 0; y < h; y++ {
			for x := 0; x < w; x++ {
				i := y*w + x
				r, g, b, _ := img.At(b.Min.X+x, b.Min.Y+y).RGBA()
				rPlane[i] = uint8(r >> 8)
				gPlane[i] = uint8(g >> 8)
				bPlane[i] = uint8(b >> 8)
			}
		}
		return
	}

	workers := min(runtime.NumCPU(), h)
	rowsPerWorker := (h + workers - 1) / workers
	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := min(y0+rowsPerWorker, h)
		wg.Add(1)
		go func(y0, y1 int) {
			defer wg.Done()
			for y := y0; y < y1; y++ {
				for x := 0; x < w; x++ {
					i := y*w + x
					r, g, b, _ := img.At(b.Min.X+x, b.Min.Y+y).RGBA()
					rPlane[i] = uint8(r >> 8)
					gPlane[i] = uint8(g >> 8)
					bPlane[i] = uint8(b >> 8)
				}
			}
		}(y0, y1)
	}
	wg.Wait()
}

func (e *Encoder) ensureErrorBuffers(w int) {
	need := w + 2
	if cap(e.errCurr) < need {
		e.errCurr = make([]float64, need)
		e.errNext = make([]float64, need)
		return
	}
	e.errCurr = e.errCurr[:need]
	e.errNext = e.errNext[:need]
}

func quantizePlaneToBits(plane []uint8, w, h, quality, baseAmplitude int, dst *bytes.Buffer) {
	quantizePlaneToBitsN(plane, w, h, quality, baseAmplitude, 0, 255, 1, 0, 0, dst)
}

func quantizePlaneToBitsN(plane []uint8, w, h, quality, baseAmplitude int, low, high uint8, bits, phaseX, phaseY int, dst *bytes.Buffer) {
	bw := newBitWriter(dst)
	baseAmp := ditherAmplitude(baseAmplitude, quality)
	if bits <= 1 {
		center := float64(low+high) * 0.5
		for y := 0; y < h; y++ {
			row := y * w
			for x := 0; x < w; x++ {
				i := row + x
				amp := adaptiveDitherAmplitude(plane, w, h, x, y, baseAmp)
				threshold := center + blueNoiseBias(x, y, phaseX, phaseY, amp)
				value := float64(plane[i])
				bw.writeBit(value >= threshold)
			}
		}
		bw.flush()
		return
	}
	levelCount := (1 << bits) - 1
	lo := float64(low)
	hi := float64(high)
	for y := 0; y < h; y++ {
		row := y * w
		for x := 0; x < w; x++ {
			i := row + x
			norm := clamp01((float64(plane[i]) - lo) / (hi - lo))
			amp := adaptiveDitherAmplitude(plane, w, h, x, y, baseAmp)
			jitter := blueNoiseBias(x, y, phaseX, phaseY, amp) / 255.0
			scaled := clamp01(norm+jitter/float64(levelCount)) * float64(levelCount)
			level := int(math.Round(scaled))
			if level < 0 {
				level = 0
			}
			if level > levelCount {
				level = levelCount
			}
			bw.writeBits(uint64(level), uint8(bits))
		}
	}
	bw.flush()
}

func quantizeColorToPalette(yPlane, cbPlane, crPlane []uint8, w, h, quality, yBits, cbBits, crBits, yPhaseX, yPhaseY, cbPhaseX, cbPhaseY, crPhaseX, crPhaseY int, shuffle, rgbMode, zxMode bool, paletteName string, yDst, cbDst, crDst *bytes.Buffer) {
	yw := newBitWriter(yDst)
	cbw := newBitWriter(cbDst)
	crw := newBitWriter(crDst)
	rowY := make([]int, w)
	rowCb := make([]int, w)
	rowCr := make([]int, w)
	prevOutY := make([]float64, w)
	prevOutCb := make([]float64, w)
	prevOutCr := make([]float64, w)
	currOutY := make([]float64, w)
	currOutCb := make([]float64, w)
	currOutCr := make([]float64, w)
	for i := range prevOutY {
		prevOutY[i], prevOutCb[i], prevOutCr[i] = math.NaN(), math.NaN(), math.NaN()
		currOutY[i], currOutCb[i], currOutCr[i] = math.NaN(), math.NaN(), math.NaN()
	}

	low0, high0 := uint8(yLow), uint8(yHigh)
	low1, high1 := uint8(cbLow), uint8(cbHigh)
	low2, high2 := uint8(crLow), uint8(crHigh)
	if rgbMode || zxMode || paletteName != "" {
		low0, high0 = 0, 255
		low1, high1 = 0, 255
		low2, high2 = 0, 255
	} else {
		low0, high0 = 0, 255
		low1, high1 = 0, 255
		low2, high2 = 0, 255
	}
	yLevels := quantLevels(low0, high0, yBits)
	cbLevels := quantLevels(low1, high1, max(cbBits, 1))
	crLevels := quantLevels(low2, high2, max(crBits, 1))
	palette := stylizedPaletteYCbCr(min(1<<min(yBits+max(cbBits, 1)+max(crBits, 1), 4), len(stylizedColorPalette)))
	if zxMode {
		palette = zxPaletteRGB()
	} else if paletteName != "" {
		if specPalette := paletteFromSpec(paletteName); len(specPalette) > 0 {
			palette = specPalette
		} else if strings.HasPrefix(paletteName, "adaptive:") {
			palette = adaptivePaletteForImage(paletteName, yPlane, cbPlane, crPlane, w, h)
		} else {
			palette = buildCoveredPalette(paletteName, yPlane, cbPlane, crPlane, w, h)
		}
	} else if rgbMode {
		palette = stylizedPaletteRGB(min(1<<min(yBits+max(cbBits, 1)+max(crBits, 1), 4), len(rgbPrimaryPalette)))
	}

	yAmp := ditherAmplitude(44, quality)
	cbAmp := ditherAmplitude(28, quality)
	crAmp := ditherAmplitude(28, quality)
	fsStrength := fsDiffusionStrength(quality)

	errYCurr := make([]float64, w+2)
	errYCurrNext := make([]float64, w+2)
	errCbCurr := make([]float64, w+2)
	errCbNext := make([]float64, w+2)
	errCrCurr := make([]float64, w+2)
	errCrNext := make([]float64, w+2)

	for y := 0; y < h; y++ {
		row := y * w
		if y&1 == 0 {
			for x := 0; x < w; x++ {
				i := row + x
				targetY := clamp255(float64(yPlane[i]) + errYCurr[x+1] + noiseBias(x, y, yPhaseX, yPhaseY, yAmp, shuffle))
				targetCb := 128.0
				targetCr := 128.0
				if rgbMode {
					targetCb = 0
					targetCr = 0
				}
				if cbBits > 0 {
					targetCb = clamp255(float64(cbPlane[i]) + errCbCurr[x+1] + noiseBias(x, y, cbPhaseX, cbPhaseY, cbAmp, shuffle))
				}
				if crBits > 0 {
					targetCr = clamp255(float64(crPlane[i]) + errCrCurr[x+1] + noiseBias(x, y, crPhaseX, crPhaseY, crAmp, shuffle))
				}
				upLeftY, upLeftCb, upLeftCr := sampleNeighbor(prevOutY, prevOutCb, prevOutCr, x-1)
				upRightY, upRightCb, upRightCr := sampleNeighbor(prevOutY, prevOutCb, prevOutCr, x+1)
				bestY, bestCb, bestCr, outY, outCb, outCr := nearestColorMode(targetY, targetCb, targetCr, yLevels, cbLevels, crLevels, palette, cbBits > 0, crBits > 0, rgbMode || zxMode || paletteName != "", upLeftY, upLeftCb, upLeftCr, upRightY, upRightCb, upRightCr)
				if crBits > 0 {
					rowCr[x] = bestCr
				}
				rowY[x] = bestY
				if cbBits > 0 {
					rowCb[x] = bestCb
				}
				currOutY[x], currOutCb[x], currOutCr[x] = outY, outCb, outCr
				diffuseColorError(targetY-outY, targetCb-outCb, targetCr-outCr, x, errYCurr, errYCurrNext, errCbCurr, errCbNext, errCrCurr, errCrNext, cbBits > 0, crBits > 0, true, fsStrength)
			}
		} else {
			for x := w - 1; x >= 0; x-- {
				i := row + x
				targetY := clamp255(float64(yPlane[i]) + errYCurr[x+1] + noiseBias(x, y, yPhaseX, yPhaseY, yAmp, shuffle))
				targetCb := 128.0
				targetCr := 128.0
				if rgbMode {
					targetCb = 0
					targetCr = 0
				}
				if cbBits > 0 {
					targetCb = clamp255(float64(cbPlane[i]) + errCbCurr[x+1] + noiseBias(x, y, cbPhaseX, cbPhaseY, cbAmp, shuffle))
				}
				if crBits > 0 {
					targetCr = clamp255(float64(crPlane[i]) + errCrCurr[x+1] + noiseBias(x, y, crPhaseX, crPhaseY, crAmp, shuffle))
				}
				upLeftY, upLeftCb, upLeftCr := sampleNeighbor(prevOutY, prevOutCb, prevOutCr, x-1)
				upRightY, upRightCb, upRightCr := sampleNeighbor(prevOutY, prevOutCb, prevOutCr, x+1)
				bestY, bestCb, bestCr, outY, outCb, outCr := nearestColorMode(targetY, targetCb, targetCr, yLevels, cbLevels, crLevels, palette, cbBits > 0, crBits > 0, rgbMode || zxMode || paletteName != "", upLeftY, upLeftCb, upLeftCr, upRightY, upRightCb, upRightCr)
				rowY[x] = bestY
				if cbBits > 0 {
					rowCb[x] = bestCb
				}
				if crBits > 0 {
					rowCr[x] = bestCr
				}
				currOutY[x], currOutCb[x], currOutCr[x] = outY, outCb, outCr
				diffuseColorError(targetY-outY, targetCb-outCb, targetCr-outCr, x, errYCurr, errYCurrNext, errCbCurr, errCbNext, errCrCurr, errCrNext, cbBits > 0, crBits > 0, false, fsStrength)
			}
		}
		for x := 0; x < w; x++ {
			yw.writeBits(uint64(rowY[x]), uint8(yBits))
			if cbBits > 0 {
				cbw.writeBits(uint64(rowCb[x]), uint8(cbBits))
			}
			if crBits > 0 {
				crw.writeBits(uint64(rowCr[x]), uint8(crBits))
			}
		}
		for i := range errYCurr {
			errYCurr[i], errYCurrNext[i] = errYCurrNext[i], 0
			errCbCurr[i], errCbNext[i] = errCbNext[i], 0
			errCrCurr[i], errCrNext[i] = errCrNext[i], 0
		}
		copy(prevOutY, currOutY)
		copy(prevOutCb, currOutCb)
		copy(prevOutCr, currOutCr)
	}

	yw.flush()
	if cbBits > 0 {
		cbw.flush()
	}
	if crBits > 0 {
		crw.flush()
	}
}

func paletteFromMode(zxMode bool, paletteName string, rPlane, gPlane, bPlane []uint8, w, h int) [][3]uint8 {
	if zxMode {
		return zxPaletteRGB()
	}
	if specPalette := paletteFromSpec(paletteName); len(specPalette) > 0 {
		return specPalette
	}
	if graySize := grayPaletteSize(paletteName); graySize > 0 {
		return buildGrayPalette(graySize)
	}
	if strings.HasPrefix(paletteName, "adaptive:") {
		return adaptivePaletteForImage(paletteName, rPlane, gPlane, bPlane, w, h)
	}
	if paletteName != "" {
		return buildCoveredPalette(paletteName, rPlane, gPlane, bPlane, w, h)
	}
	return rgbPrimaryPalette
}

func quantizePaletteIndices(rPlane, gPlane, bPlane []uint8, w, h, quality int, palette [][3]uint8, rPhaseX, rPhaseY, gPhaseX, gPhaseY, bPhaseX, bPhaseY int, shuffle, adaptive bool, tileSize, blockSize int, rawMode, rawTop16, rawTree, rawTreeAdapt bool, rawShift int, reconstructMode, blockSubset bool, dst *bytes.Buffer) {
	dst.Reset()
	if len(palette) < 1 {
		palette = rgbPrimaryPalette
	}
	hdr := make([]byte, 2+len(palette)*3)
	binary.BigEndian.PutUint16(hdr[0:2], uint16(len(palette)))
	pos := 2
	for _, c := range palette {
		hdr[pos+0] = c[0]
		hdr[pos+1] = c[1]
		hdr[pos+2] = c[2]
		pos += 3
	}
	dst.Write(hdr)

	rAmp := ditherAmplitude(44, quality)
	gAmp := ditherAmplitude(28, quality)
	bAmp := ditherAmplitude(28, quality)
	fsStrength := fsDiffusionStrength(quality)
	rowIdx := make([]int, w)
	pixelIdx := make([]int, w*h)
	freq := make([]int, len(palette))
	errRCurr := make([]float64, w+2)
	errRNext := make([]float64, w+2)
	errGCurr := make([]float64, w+2)
	errGNext := make([]float64, w+2)
	errBCurr := make([]float64, w+2)
	errBNext := make([]float64, w+2)

	for y := 0; y < h; y++ {
		row := y * w
		if y&1 == 0 {
			for x := 0; x < w; x++ {
				i := row + x
				targetR := clamp255(float64(rPlane[i]) + errRCurr[x+1] + noiseBias(x, y, rPhaseX, rPhaseY, rAmp, shuffle))
				targetG := clamp255(float64(gPlane[i]) + errGCurr[x+1] + noiseBias(x, y, gPhaseX, gPhaseY, gAmp, shuffle))
				targetB := clamp255(float64(bPlane[i]) + errBCurr[x+1] + noiseBias(x, y, bPhaseX, bPhaseY, bAmp, shuffle))
				if adaptive {
					targetR, targetG, targetB = adaptivePaletteTarget(rPlane, gPlane, bPlane, w, h, x, y, targetR, targetG, targetB)
				}
				idx, out := nearestPaletteEntry(targetR, targetG, targetB, palette)
				rowIdx[x] = idx
				diffuseColorError(targetR-float64(out[0]), targetG-float64(out[1]), targetB-float64(out[2]), x, errRCurr, errRNext, errGCurr, errGNext, errBCurr, errBNext, true, true, true, fsStrength)
			}
		} else {
			for x := w - 1; x >= 0; x-- {
				i := row + x
				targetR := clamp255(float64(rPlane[i]) + errRCurr[x+1] + noiseBias(x, y, rPhaseX, rPhaseY, rAmp, shuffle))
				targetG := clamp255(float64(gPlane[i]) + errGCurr[x+1] + noiseBias(x, y, gPhaseX, gPhaseY, gAmp, shuffle))
				targetB := clamp255(float64(bPlane[i]) + errBCurr[x+1] + noiseBias(x, y, bPhaseX, bPhaseY, bAmp, shuffle))
				if adaptive {
					targetR, targetG, targetB = adaptivePaletteTarget(rPlane, gPlane, bPlane, w, h, x, y, targetR, targetG, targetB)
				}
				idx, out := nearestPaletteEntry(targetR, targetG, targetB, palette)
				rowIdx[x] = idx
				diffuseColorError(targetR-float64(out[0]), targetG-float64(out[1]), targetB-float64(out[2]), x, errRCurr, errRNext, errGCurr, errGNext, errBCurr, errBNext, true, true, false, fsStrength)
			}
		}
		for x := 0; x < w; x++ {
			idx := rowIdx[x]
			pixelIdx[row+x] = idx
			freq[idx]++
		}
		for i := range errRCurr {
			errRCurr[i], errRNext[i] = errRNext[i], 0
			errGCurr[i], errGNext[i] = errGNext[i], 0
			errBCurr[i], errBNext[i] = errBNext[i], 0
		}
	}

	canvasIdx := 0
	for i := 1; i < len(freq); i++ {
		if freq[i] > freq[canvasIdx] {
			canvasIdx = i
		}
	}
	if rawMode {
		if blockSize > 0 && blockSubset {
			dst.WriteByte(5) // raw palette indices with per-block subset
			dst.WriteByte(byte(blockSize))
			for by := 0; by < h; by += blockSize {
				y1 := min(by+blockSize, h)
				for bx := 0; bx < w; bx += blockSize {
					x1 := min(bx+blockSize, w)
					localFreq := make(map[int]int, len(palette))
					for y := by; y < y1; y++ {
						row := y * w
						for x := bx; x < x1; x++ {
							localFreq[pixelIdx[row+x]]++
						}
					}
					subset := make([]int, 0, len(localFreq))
					for idx := range localFreq {
						subset = append(subset, idx)
					}
					sort.Slice(subset, func(i, j int) bool {
						if localFreq[subset[i]] != localFreq[subset[j]] {
							return localFreq[subset[i]] > localFreq[subset[j]]
						}
						return subset[i] < subset[j]
					})
					if len(subset) < 1 {
						subset = append(subset, canvasIdx)
					}
					dst.WriteByte(byte(len(subset)))
					for _, idx := range subset {
						dst.WriteByte(byte(idx))
					}
					localBits := bitsNeeded(len(subset) - 1)
					if localBits < 1 {
						localBits = 1
					}
					inv := make([]int, len(palette))
					for i := range inv {
						inv[i] = -1
					}
					for i, idx := range subset {
						inv[idx] = i
					}
					var block bytes.Buffer
					bw := newBitWriter(&block)
					for y := by; y < y1; y++ {
						row := y * w
						for x := bx; x < x1; x++ {
							bw.writeBits(uint64(inv[pixelIdx[row+x]]), uint8(localBits))
						}
					}
					bw.flush()
					dst.Write(block.Bytes())
				}
			}
			_ = canvasIdx
			return
		}
		if blockSize > 0 {
			dst.WriteByte(4) // raw packed palette indices, chunked by NxN blocks
			dst.WriteByte(byte(blockSize))
			indexBits := bitsNeeded(len(palette) - 1)
			if indexBits < 1 {
				indexBits = 1
			}
			for by := 0; by < h; by += blockSize {
				y1 := min(by+blockSize, h)
				for bx := 0; bx < w; bx += blockSize {
					x1 := min(bx+blockSize, w)
					var block bytes.Buffer
					bw := newBitWriter(&block)
					for y := by; y < y1; y++ {
						row := y * w
						for x := bx; x < x1; x++ {
							bw.writeBits(uint64(pixelIdx[row+x]), uint8(indexBits))
						}
					}
					bw.flush()
					dst.Write(block.Bytes())
				}
			}
			_ = canvasIdx
			return
		}
		indexBits := bitsNeeded(len(palette) - 1)
		if indexBits < 1 {
			indexBits = 1
		}
		var rawBuf bytes.Buffer
		bw := newBitWriter(&rawBuf)
		for _, idx := range pixelIdx {
			bw.writeBits(uint64(idx), uint8(indexBits))
		}
		bw.flush()
		if rawTreeAdapt {
			dst.WriteByte(9) // raw palette indices as frequency-adapted tree
			freq := make([]int, len(palette))
			for _, idx := range pixelIdx {
				freq[idx]++
			}
			order := make([]byte, len(palette))
			for i := range order {
				order[i] = byte(i)
			}
			sort.Slice(order, func(i, j int) bool {
				fi := freq[int(order[i])]
				fj := freq[int(order[j])]
				if fi != fj {
					return fi > fj
				}
				return order[i] < order[j]
			})
			dst.Write(order)
			encodeRawIndexTreeFreq(pixelIdx, order, dst)
			_ = canvasIdx
			return
		}
		if rawTop16 && rawTree {
			rawStream := rawBuf.Bytes()
			bestShift := 0
			bestSize := int(^uint(0) >> 1)
			var bestTopA [16]byte
			var bestPrefix byte
			var bestSuffix byte
			var bestMaskA []byte
			var bestIDsA []byte
			var bestTree []byte
			startShift := 0
			endShift := 7
			if rawShift >= 0 {
				startShift = rawShift
				endShift = rawShift
			}
			for shift := startShift; shift <= endShift; shift++ {
				byteStream := packShiftedBytes(rawStream, w*h*indexBits, shift)
				topA, maskA, idsA, otherA := splitTop16Stream(byteStream)
				otherVals := make([]int, len(otherA))
				for i, b := range otherA {
					otherVals[i] = int(b)
				}
				var tree bytes.Buffer
				encodeRawIndexTree(otherVals, 0, 256, &tree)
				size := len(maskA) + len(idsA) + tree.Len()
				if size < bestSize {
					bestSize = size
					bestShift = shift
					bestTopA = topA
					bestPrefix = gatherPackedBits(rawStream, 0, shift)
					suffixCount := (w*h*indexBits - shift) & 7
					bestSuffix = gatherPackedBits(rawStream, w*h*indexBits-suffixCount, suffixCount)
					bestMaskA = append(bestMaskA[:0], maskA...)
					bestIDsA = append(bestIDsA[:0], idsA...)
					bestTree = append(bestTree[:0], tree.Bytes()...)
				}
			}
			dst.WriteByte(8) // raw shifted top16 + tree residual
			dst.WriteByte(byte(bestShift))
			dst.WriteByte(bestPrefix)
			dst.WriteByte(bestSuffix)
			dst.Write(bestTopA[:])
			dst.Write(bestMaskA)
			dst.Write(bestIDsA)
			dst.Write(bestTree)
			_ = canvasIdx
			return
		}
		if rawTree {
			dst.WriteByte(7) // raw palette indices as binary range tree
			encodeRawIndexTree(pixelIdx, 0, len(palette), dst)
			_ = canvasIdx
			return
		}
		if rawTop16 {
			rawStream := rawBuf.Bytes()
			shift, topA, prefixBits, suffixBits, stream := encodeShiftedTop16Stream(rawStream, w*h*indexBits, rawShift)
			dst.WriteByte(6) // raw packed palette indices with shifted top16 byte coding
			dst.WriteByte(byte(shift))
			dst.WriteByte(prefixBits)
			dst.WriteByte(suffixBits)
			dst.Write(topA[:])
			dst.Write(stream)
			_ = canvasIdx
			return
		}
		dst.WriteByte(1) // raw packed palette indices
		dst.Write(rawBuf.Bytes())
		_ = canvasIdx
		return
	}
	if reconstructMode {
		remaining := make([]int, len(pixelIdx))
		copy(remaining, pixelIdx)
		order := make([]int, 0, len(palette))
		planes := make([][]byte, 0, max(0, len(palette)-1))
		for {
			currFreq := make([]int, len(palette))
			for _, idx := range remaining {
				currFreq[idx]++
			}
			chosen := -1
			for i, c := range currFreq {
				if c == 0 {
					continue
				}
				if chosen < 0 || c > currFreq[chosen] {
					chosen = i
				}
			}
			if chosen < 0 {
				break
			}
			order = append(order, chosen)
			if len(remaining) == currFreq[chosen] {
				break
			}
			hasOther := false
			var planeBuf bytes.Buffer
			bw := newBitWriter(&planeBuf)
			nextRemaining := make([]int, 0, len(remaining)-currFreq[chosen])
			for _, idx := range remaining {
				match := idx == chosen
				bw.writeBit(match)
				if !match {
					hasOther = true
					nextRemaining = append(nextRemaining, idx)
				}
			}
			bw.flush()
			if hasOther {
				plane := append([]byte(nil), planeBuf.Bytes()...)
				planes = append(planes, plane)
				remaining = nextRemaining
				continue
			}
			break
		}
		if len(order) < 1 {
			order = append(order, canvasIdx)
		}
		dst.WriteByte(3) // sequential peel planes
		tmp := make([]byte, 2)
		binary.BigEndian.PutUint16(tmp, uint16(len(order)))
		dst.Write(tmp)
		for _, idx := range order {
			dst.WriteByte(byte(idx))
		}
		for _, plane := range planes {
			dst.Write(plane)
		}
		return
	}
	if tileSize <= 0 {
		tileSize = paletteTileSize
	}
	dst.WriteByte(2) // tile-local subset mode
	dst.WriteByte(byte(tileSize))
	dst.WriteByte(byte(tileSize))

	tilesX := ceilDiv(w, tileSize)
	tilesY := ceilDiv(h, tileSize)
	for ty := 0; ty < tilesY; ty++ {
		y0 := ty * tileSize
		y1 := min(y0+tileSize, h)
		for tx := 0; tx < tilesX; tx++ {
			x0 := tx * tileSize
			x1 := min(x0+tileSize, w)

			localFreq := make(map[int]int, len(palette))
			for y := y0; y < y1; y++ {
				row := y * w
				for x := x0; x < x1; x++ {
					localFreq[pixelIdx[row+x]]++
				}
			}
			subset := make([]int, 0, len(localFreq))
			for idx := range localFreq {
				subset = append(subset, idx)
			}
			sort.Slice(subset, func(i, j int) bool {
				fi := localFreq[subset[i]]
				fj := localFreq[subset[j]]
				if fi != fj {
					return fi > fj
				}
				return subset[i] < subset[j]
			})
			if len(subset) < 1 {
				subset = append(subset, canvasIdx)
			}
			dst.WriteByte(byte(len(subset)))
			for _, idx := range subset {
				dst.WriteByte(byte(idx))
			}
			localBits := bitsNeeded(len(subset) - 1)
			if localBits < 1 {
				localBits = 1
			}
			inv := make([]int, len(palette))
			for i := range inv {
				inv[i] = -1
			}
			for i, idx := range subset {
				inv[idx] = i
			}
			var tileBuf bytes.Buffer
			bw := newBitWriter(&tileBuf)
			order := zOrderIndices(x1-x0, y1-y0)
			values := make([]int, 0, len(order))
			for _, localPi := range order {
				lx := localPi % (x1 - x0)
				ly := localPi / (x1 - x0)
				globalPi := (y0+ly)*w + (x0 + lx)
				values = append(values, inv[pixelIdx[globalPi]])
			}
			for bit := localBits - 1; bit >= 0; bit-- {
				for _, v := range values {
					bw.writeBit(((v >> bit) & 1) != 0)
				}
			}
			bw.flush()
			dst.Write(tileBuf.Bytes())
		}
	}
}

func packShiftedBytes(src []byte, totalBits, shift int) []byte {
	if totalBits <= shift {
		return nil
	}
	byteCount := (totalBits - shift) / 8
	out := make([]byte, byteCount)
	for i := 0; i < byteCount; i++ {
		base := shift + i*8
		var v byte
		for b := 0; b < 8; b++ {
			if monoBitAt(src, base+b) {
				v |= 1 << (7 - b)
			}
		}
		out[i] = v
	}
	return out
}

func gatherPackedBits(src []byte, start, count int) byte {
	var out byte
	for i := 0; i < count; i++ {
		if monoBitAt(src, start+i) {
			out |= 1 << (7 - i)
		}
	}
	return out
}

func splitTop16Stream(src []byte) ([16]byte, []byte, []byte, []byte) {
	var top [16]byte
	freq := make(map[byte]int, 256)
	for _, b := range src {
		freq[b]++
	}
	type kv struct {
		b byte
		c int
	}
	pairs := make([]kv, 0, len(freq))
	for b, c := range freq {
		pairs = append(pairs, kv{b: b, c: c})
	}
	sort.Slice(pairs, func(i, j int) bool {
		if pairs[i].c != pairs[j].c {
			return pairs[i].c > pairs[j].c
		}
		return pairs[i].b < pairs[j].b
	})
	index := make(map[byte]int, 16)
	for i := 0; i < min(16, len(pairs)); i++ {
		top[i] = pairs[i].b
		index[pairs[i].b] = i
	}
	mask := make([]byte, (len(src)+7)>>3)
	topIDs := make([]byte, 0, len(src))
	others := make([]byte, 0, len(src))
	for i, b := range src {
		if idx, ok := index[b]; ok {
			monoBitSet(mask, i)
			topIDs = append(topIDs, byte(idx))
		} else {
			others = append(others, b)
		}
	}
	var idsBuf bytes.Buffer
	bw := newBitWriter(&idsBuf)
	for _, id := range topIDs {
		bw.writeBits(uint64(id), 4)
	}
	bw.flush()
	return top, mask, idsBuf.Bytes(), others
}

func encodeShiftedTop16Stream(rawStream []byte, totalBits int, forcedShift int) (int, [16]byte, byte, byte, []byte) {
	bestShift := 0
	bestSize := int(^uint(0) >> 1)
	var bestTopA [16]byte
	var bestPrefix byte
	var bestSuffix byte
	var bestStream []byte
	startShift := 0
	endShift := 7
	if forcedShift >= 0 {
		startShift = forcedShift
		endShift = forcedShift
	}
	for shift := startShift; shift <= endShift; shift++ {
		byteStream := packShiftedBytes(rawStream, totalBits, shift)
		topA, maskA, idsA, otherA := splitTop16Stream(byteStream)
		var payload bytes.Buffer
		payload.Write(maskA)
		payload.Write(idsA)
		payload.Write(otherA)
		size := payload.Len()
		if size < bestSize {
			bestSize = size
			bestShift = shift
			bestTopA = topA
			bestPrefix = gatherPackedBits(rawStream, 0, shift)
			suffixCount := (totalBits - shift) & 7
			bestSuffix = gatherPackedBits(rawStream, totalBits-suffixCount, suffixCount)
			bestStream = append(bestStream[:0], payload.Bytes()...)
		}
	}
	return bestShift, bestTopA, bestPrefix, bestSuffix, bestStream
}

func adaptivePaletteTarget(rPlane, gPlane, bPlane []uint8, w, h, x, y int, targetR, targetG, targetB float64) (float64, float64, float64) {
	localR := neighborhoodMean(rPlane, w, h, x, y)
	localG := neighborhoodMean(gPlane, w, h, x, y)
	localB := neighborhoodMean(bPlane, w, h, x, y)
	rangeNorm := neighborhoodRange(rPlane, w, h, x, y) + neighborhoodRange(gPlane, w, h, x, y) + neighborhoodRange(bPlane, w, h, x, y)
	rangeNorm /= 3.0 * 255.0
	detailWeight := 0.18 + rangeNorm*0.28
	targetR = clamp255(targetR*(1.0-detailWeight) + localR*detailWeight)
	targetG = clamp255(targetG*(1.0-detailWeight) + localG*detailWeight)
	targetB = clamp255(targetB*(1.0-detailWeight) + localB*detailWeight)
	return targetR, targetG, targetB
}

func nearestPaletteEntry(targetR, targetG, targetB float64, palette [][3]uint8) (int, [3]uint8) {
	bestIdx := 0
	best := palette[0]
	bestDist := math.MaxFloat64
	for i, p := range palette {
		dr := targetR - float64(p[0])
		dg := targetG - float64(p[1])
		db := targetB - float64(p[2])
		dist := dr*dr*1.2 + dg*dg + db*db
		if dist < bestDist {
			bestDist = dist
			bestIdx = i
			best = p
		}
	}
	return bestIdx, best
}

func nearestPaletteColor(targetY, targetCb, targetCr float64, yLevels, cbLevels, crLevels []uint8, hasCb, hasCr bool) (int, int, int, float64, float64, float64) {
	bestY, bestCb, bestCr := 0, 0, 0
	bestDist := math.MaxFloat64
	bestOutY := float64(yLevels[0])
	bestOutCb := 128.0
	bestOutCr := 128.0
	for yi, yv := range yLevels {
		for cbi, cbv := range cbLevels {
			for cri, crv := range crLevels {
				dist := paletteDistance(targetY, targetCb, targetCr, float64(yv), float64(cbv), float64(crv), hasCb, hasCr)
				if dist < bestDist {
					bestDist = dist
					bestY, bestCb, bestCr = yi, cbi, cri
					bestOutY = float64(yv)
					if hasCb {
						bestOutCb = float64(cbv)
					}
					if hasCr {
						bestOutCr = float64(crv)
					}
				}
			}
		}
	}
	return bestY, bestCb, bestCr, bestOutY, bestOutCb, bestOutCr
}

func nearestColorMode(targetY, targetCb, targetCr float64, yLevels, cbLevels, crLevels []uint8, palette [][3]uint8, hasCb, hasCr, rgbMode bool, upLeftY, upLeftCb, upLeftCr, upRightY, upRightCb, upRightCr float64) (int, int, int, float64, float64, float64) {
	if rgbMode {
		return nearestStylizedRGBColor(targetY, targetCb, targetCr, yLevels, cbLevels, crLevels, palette, hasCb, hasCr, upLeftY, upLeftCb, upLeftCr, upRightY, upRightCb, upRightCr)
	}
	return nearestPaletteColor(targetY, targetCb, targetCr, yLevels, cbLevels, crLevels, hasCb, hasCr)
}

func nearestStylizedColor(targetY, targetCb, targetCr float64, yLevels, cbLevels, crLevels []uint8, palette [][3]uint8, hasCb, hasCr bool) (int, int, int, float64, float64, float64) {
	bestY, bestCb, bestCr := 0, 0, 0
	bestOutY := float64(yLevels[0])
	bestOutCb := 128.0
	bestOutCr := 128.0
	bestDist := math.MaxFloat64
	for _, p := range palette {
		py, pcb, pcr := rgbToYCbCrFloats(p[0], p[1], p[2])
		yi, qy := nearestLevelIndex(py, yLevels)
		cbi, qcb := 0, 128.0
		cri, qcr := 0, 128.0
		if hasCb {
			cbi, qcb = nearestLevelIndex(pcb, cbLevels)
		}
		if hasCr {
			cri, qcr = nearestLevelIndex(pcr, crLevels)
		}
		dist := paletteDistance(targetY, targetCb, targetCr, qy, qcb, qcr, hasCb, hasCr)
		if dist < bestDist {
			bestDist = dist
			bestY, bestCb, bestCr = yi, cbi, cri
			bestOutY, bestOutCb, bestOutCr = qy, qcb, qcr
		}
	}
	return bestY, bestCb, bestCr, bestOutY, bestOutCb, bestOutCr
}

func nearestLevelIndex(target float64, levels []uint8) (int, float64) {
	bestIdx := 0
	bestOut := float64(levels[0])
	bestDist := math.Abs(target - bestOut)
	for i := 1; i < len(levels); i++ {
		out := float64(levels[i])
		dist := math.Abs(target - out)
		if dist < bestDist {
			bestDist = dist
			bestIdx = i
			bestOut = out
		}
	}
	return bestIdx, bestOut
}

func stylizedPaletteYCbCr(n int) [][3]uint8 {
	if n < 1 {
		n = 1
	}
	if n > len(stylizedColorPalette) {
		n = len(stylizedColorPalette)
	}
	return stylizedColorPalette[:n]
}

func stylizedPaletteRGB(n int) [][3]uint8 {
	if n < 1 {
		n = 1
	}
	if n > len(rgbPrimaryPalette) {
		n = len(rgbPrimaryPalette)
	}
	return rgbPrimaryPalette[:n]
}

func zxPaletteRGB() [][3]uint8 {
	return zxSpectrumPalette
}

func namedPaletteRGB(name string) [][3]uint8 {
	if p := paletteFromSpec(name); len(p) > 0 {
		return p
	}
	if graySize := grayPaletteSize(name); graySize > 0 {
		return buildGrayPalette(graySize)
	}
	switch name {
	case "cga":
		return cgaPalette
	case "ega":
		return egaPalette
	case "vga":
		return vgaPalette
	case "c64":
		return c64Palette
	case "gameboy":
		return gameBoyPalette
	case "pico8":
		return pico8Palette
	case "db16":
		return db16Palette
	case "nes":
		return nesPalette
	case "sunset":
		return sunsetPalette
	case "pastel":
		return pastelPalette
	case "ocean":
		return oceanPalette
	case "forest":
		return forestPalette
	case "ymc":
		return ymcPalette
	case "ymcrgb":
		return ymcRGBPalette
	default:
		return rgbPrimaryPalette
	}
}

func grayPaletteSize(name string) int {
	name = strings.ToLower(strings.TrimSpace(name))
	if !strings.HasPrefix(name, "gray:") {
		return 0
	}
	n, err := strconv.Atoi(strings.TrimPrefix(name, "gray:"))
	if err != nil || n < 1 || n > 256 {
		return 0
	}
	return n
}

func buildGrayPalette(size int) [][3]uint8 {
	if size < 1 {
		size = 1
	}
	palette := make([][3]uint8, size)
	if size == 1 {
		palette[0] = [3]uint8{128, 128, 128}
		return palette
	}
	for i := 0; i < size; i++ {
		v := uint8(math.Round(float64(i) * 255.0 / float64(size-1)))
		palette[i] = [3]uint8{v, v, v}
	}
	return palette
}

func paletteFromSpec(spec string) [][3]uint8 {
	spec = strings.ToLower(strings.TrimSpace(spec))
	if spec == "" {
		return nil
	}
	var palette [][3]uint8
	for _, chunk := range strings.Split(spec, "+") {
		chunk = strings.TrimSpace(chunk)
		if chunk == "" {
			return nil
		}
		for len(chunk) > 0 {
			if strings.HasPrefix(chunk, "bw") || strings.HasPrefix(chunk, "wb") {
				palette = appendUniqueColor(palette, [3]uint8{0, 0, 0}, 0)
				palette = appendUniqueColor(palette, [3]uint8{255, 255, 255}, 0)
				chunk = chunk[2:]
				continue
			}
			switch chunk[0] {
			case 'y':
				palette = appendUniqueColor(palette, [3]uint8{255, 255, 0}, 0)
			case 'c':
				palette = appendUniqueColor(palette, [3]uint8{0, 255, 255}, 0)
			case 'm':
				palette = appendUniqueColor(palette, [3]uint8{255, 0, 255}, 0)
			case 'r':
				palette = appendUniqueColor(palette, [3]uint8{255, 0, 0}, 0)
			case 'g':
				palette = appendUniqueColor(palette, [3]uint8{0, 255, 0}, 0)
			case 'b':
				palette = appendUniqueColor(palette, [3]uint8{0, 0, 255}, 0)
			case 'w':
				palette = appendUniqueColor(palette, [3]uint8{255, 255, 255}, 0)
			case 'k':
				palette = appendUniqueColor(palette, [3]uint8{0, 0, 0}, 0)
			default:
				return nil
			}
			chunk = chunk[1:]
		}
	}
	return palette
}

func buildCoveredPalette(name string, rPlane, gPlane, bPlane []uint8, w, h int) [][3]uint8 {
	base := append([][3]uint8(nil), namedPaletteRGB(name)...)
	if len(rPlane) == 0 || w == 0 || h == 0 {
		return base
	}

	type bucketColor struct {
		r, g, b uint8
		luma    int
		valid   bool
	}
	var buckets [5]bucketColor
	targets := [5]int{20, 64, 128, 192, 236}
	bestDist := [5]int{1 << 30, 1 << 30, 1 << 30, 1 << 30, 1 << 30}

	stepX := max(1, w/96)
	stepY := max(1, h/96)
	for y := 0; y < h; y += stepY {
		row := y * w
		for x := 0; x < w; x += stepX {
			i := row + x
			r := rPlane[i]
			g := gPlane[i]
			b := bPlane[i]
			luma := perceptualLuma(r, g, b)
			for bi, target := range targets {
				d := absInt(luma - target)
				if d < bestDist[bi] {
					bestDist[bi] = d
					buckets[bi] = bucketColor{r: r, g: g, b: b, luma: luma, valid: true}
				}
			}
		}
	}

	for _, c := range buckets {
		if !c.valid {
			continue
		}
		base = appendUniqueColor(base, [3]uint8{c.r, c.g, c.b}, 18)
	}
	return base
}

func adaptivePaletteSize(name string) int {
	if !strings.HasPrefix(name, "adaptive:") {
		return 16
	}
	if strings.EqualFold(strings.TrimPrefix(name, "adaptive:"), "auto") {
		return 16
	}
	n, err := strconv.Atoi(strings.TrimPrefix(name, "adaptive:"))
	if err != nil || n < 2 {
		return 16
	}
	if n > 256 {
		return 256
	}
	return n
}

func adaptivePaletteForImage(name string, rPlane, gPlane, bPlane []uint8, w, h int) [][3]uint8 {
	if strings.HasPrefix(strings.ToLower(strings.TrimPrefix(name, "adaptive:")), "auto") {
		size := chooseAdaptivePaletteSizeForName(name, rPlane, gPlane, bPlane, w, h)
		return buildAdaptivePalette(rPlane, gPlane, bPlane, w, h, size)
	}
	return buildAdaptivePalette(rPlane, gPlane, bPlane, w, h, adaptivePaletteSize(name))
}

func chooseAdaptivePaletteSizeForName(name string, rPlane, gPlane, bPlane []uint8, w, h int) int {
	base := chooseAdaptivePaletteSize(rPlane, gPlane, bPlane, w, h)
	pct := adaptiveAutoPercent(name)
	if pct >= 100 {
		return base
	}
	if pct <= 0 {
		return 2
	}
	size := 2 + int(math.Round(float64(base-2)*float64(pct)/100.0))
	if size < 2 {
		size = 2
	}
	if size > base {
		size = base
	}
	return size
}

func adaptiveAutoPercent(name string) int {
	if !strings.HasPrefix(strings.ToLower(name), "adaptive:auto") {
		return 100
	}
	parts := strings.Split(name, ":")
	if len(parts) < 3 {
		return 100
	}
	if len(parts) == 2 {
		return 100
	}
	pct, err := strconv.Atoi(parts[2])
	if err != nil {
		return 100
	}
	if pct < 0 {
		return 0
	}
	if pct > 100 {
		return 100
	}
	return pct
}

func chooseAdaptivePaletteSize(rPlane, gPlane, bPlane []uint8, w, h int) int {
	candidates := []int{8, 12, 16, 24, 32}
	samples := sampleImageColors(rPlane, gPlane, bPlane, w, h, 512)
	if len(samples) == 0 {
		return 16
	}
	chosen := candidates[0]
	prevErr := math.MaxFloat64
	for i, size := range candidates {
		palette := buildAdaptivePalette(rPlane, gPlane, bPlane, w, h, size)
		err := averagePaletteError(samples, palette)
		if i == 0 {
			chosen = size
			prevErr = err
			continue
		}
		if prevErr <= 0 {
			break
		}
		improvement := (prevErr - err) / prevErr
		if improvement >= 0.10 {
			chosen = size
			prevErr = err
			continue
		}
		break
	}
	return chosen
}

func averagePaletteError(samples [][3]uint8, palette [][3]uint8) float64 {
	if len(samples) == 0 || len(palette) == 0 {
		return 0
	}
	total := 0.0
	for _, c := range samples {
		total += float64(nearestPaletteRGBDist2(c, palette))
	}
	return total / float64(len(samples))
}

func buildAdaptivePalette(rPlane, gPlane, bPlane []uint8, w, h, size int) [][3]uint8 {
	if size < 2 {
		size = 2
	}
	samples := sampleImageColors(rPlane, gPlane, bPlane, w, h, max(size*24, 128))
	if len(samples) == 0 {
		return rgbPrimaryPalette
	}
	palette := make([][3]uint8, 0, min(size, len(samples)))

	// Start with darkest and brightest anchors, then spread by farthest-point selection.
	darkIdx, brightIdx := 0, 0
	darkLuma, brightLuma := 1<<30, -1
	for i, c := range samples {
		l := perceptualLuma(c[0], c[1], c[2])
		if l < darkLuma {
			darkLuma, darkIdx = l, i
		}
		if l > brightLuma {
			brightLuma, brightIdx = l, i
		}
	}
	palette = append(palette, samples[darkIdx])
	if brightIdx != darkIdx {
		palette = append(palette, samples[brightIdx])
	}

	for len(palette) < size && len(palette) < len(samples) {
		bestIdx := -1
		bestDist := -1
		for i, c := range samples {
			dist := nearestPaletteRGBDist2(c, palette)
			if dist > bestDist {
				bestDist = dist
				bestIdx = i
			}
		}
		if bestIdx < 0 {
			break
		}
		palette = appendUniqueColor(palette, samples[bestIdx], 10)
		if len(palette) == len(samples) {
			break
		}
	}
	return palette
}

func sampleImageColors(rPlane, gPlane, bPlane []uint8, w, h, limit int) [][3]uint8 {
	if len(rPlane) == 0 || w == 0 || h == 0 {
		return nil
	}
	stepX := max(1, w/int(math.Sqrt(float64(limit))))
	stepY := max(1, h/int(math.Sqrt(float64(limit))))
	samples := make([][3]uint8, 0, limit)
	for y := 0; y < h; y += stepY {
		row := y * w
		for x := 0; x < w; x += stepX {
			i := row + x
			samples = append(samples, [3]uint8{rPlane[i], gPlane[i], bPlane[i]})
			if len(samples) >= limit {
				return samples
			}
		}
	}
	return samples
}

func nearestPaletteRGBDist2(c [3]uint8, palette [][3]uint8) int {
	best := 1 << 30
	for _, p := range palette {
		if d := rgbDist2(c, p); d < best {
			best = d
		}
	}
	return best
}

func appendUniqueColor(palette [][3]uint8, c [3]uint8, minDist int) [][3]uint8 {
	for _, p := range palette {
		if rgbDist2(p, c) <= minDist*minDist {
			return palette
		}
	}
	return append(palette, c)
}

func rgbDist2(a, b [3]uint8) int {
	dr := int(a[0]) - int(b[0])
	dg := int(a[1]) - int(b[1])
	db := int(a[2]) - int(b[2])
	return dr*dr + dg*dg + db*db
}

func perceptualLuma(r, g, b uint8) int {
	return (54*int(r) + 183*int(g) + 19*int(b)) >> 8
}

func nearestStylizedRGBColor(targetR, targetG, targetB float64, rLevels, gLevels, bLevels []uint8, palette [][3]uint8, hasG, hasB bool, upLeftR, upLeftG, upLeftB, upRightR, upRightG, upRightB float64) (int, int, int, float64, float64, float64) {
	bestR, bestG, bestB := 0, 0, 0
	bestOutR := float64(rLevels[0])
	bestOutG := 0.0
	bestOutB := 0.0
	bestDist := math.MaxFloat64
	for pi, p := range palette {
		ri, qr := nearestLevelIndex(float64(p[0]), rLevels)
		gi, qg := 0, 0.0
		bi, qb := 0, 0.0
		if hasG {
			gi, qg = nearestLevelIndex(float64(p[1]), gLevels)
		}
		if hasB {
			bi, qb = nearestLevelIndex(float64(p[2]), bLevels)
		}
		dr := targetR - qr
		dist := dr * dr * 1.20
		if hasG {
			dg := targetG - qg
			dist += dg * dg
		}
		if hasB {
			db := targetB - qb
			dist += db * db
		}
		dist += paletteAdjacencyPenalty(qr, qg, qb, upLeftR, upLeftG, upLeftB, upRightR, upRightG, upRightB, hasG, hasB)
		dist += paletteOrderPenalty(pi)
		if dist < bestDist {
			bestDist = dist
			bestR, bestG, bestB = ri, gi, bi
			bestOutR, bestOutG, bestOutB = qr, qg, qb
		}
	}
	return bestR, bestG, bestB, bestOutR, bestOutG, bestOutB
}

func paletteOrderPenalty(index int) float64 {
	return float64(index) * 96.0
}

func paletteAdjacencyPenalty(r, g, b, upLeftR, upLeftG, upLeftB, upRightR, upRightG, upRightB float64, hasG, hasB bool) float64 {
	penalty := 0.0
	if sameQuantizedColor(r, g, b, upLeftR, upLeftG, upLeftB, hasG, hasB) {
		penalty += 3000
	}
	if sameQuantizedColor(r, g, b, upRightR, upRightG, upRightB, hasG, hasB) {
		penalty += 3000
	}
	return penalty
}

func sampleNeighbor(y, cb, cr []float64, x int) (float64, float64, float64) {
	if x < 0 || x >= len(y) {
		return math.NaN(), math.NaN(), math.NaN()
	}
	return y[x], cb[x], cr[x]
}

func sameQuantizedColor(r0, g0, b0, r1, g1, b1 float64, hasG, hasB bool) bool {
	if math.IsNaN(r1) {
		return false
	}
	if math.Abs(r0-r1) > 0.5 {
		return false
	}
	if hasG && math.Abs(g0-g1) > 0.5 {
		return false
	}
	if hasB && math.Abs(b0-b1) > 0.5 {
		return false
	}
	return true
}

func rgbToYCbCrFloats(r, g, b uint8) (float64, float64, float64) {
	yc := color.YCbCrModel.Convert(color.RGBA{R: r, G: g, B: b, A: 255}).(color.YCbCr)
	return float64(yc.Y), float64(yc.Cb), float64(yc.Cr)
}

func diffuseColorError(dy, dcb, dcr float64, x int, yCurr, yNext, cbCurr, cbNext, crCurr, crNext []float64, hasCb, hasCr, leftToRight bool, strength float64) {
	dy *= strength
	dcb *= strength
	dcr *= strength
	if leftToRight {
		yCurr[x+2] += dy * (7.0 / 16.0)
		yNext[x+0] += dy * (3.0 / 16.0)
		yNext[x+1] += dy * (5.0 / 16.0)
		yNext[x+2] += dy * (1.0 / 16.0)
		if hasCb {
			cbCurr[x+2] += dcb * (7.0 / 16.0)
			cbNext[x+0] += dcb * (3.0 / 16.0)
			cbNext[x+1] += dcb * (5.0 / 16.0)
			cbNext[x+2] += dcb * (1.0 / 16.0)
		}
		if hasCr {
			crCurr[x+2] += dcr * (7.0 / 16.0)
			crNext[x+0] += dcr * (3.0 / 16.0)
			crNext[x+1] += dcr * (5.0 / 16.0)
			crNext[x+2] += dcr * (1.0 / 16.0)
		}
		return
	}
	yCurr[x+0] += dy * (7.0 / 16.0)
	yNext[x+2] += dy * (3.0 / 16.0)
	yNext[x+1] += dy * (5.0 / 16.0)
	yNext[x+0] += dy * (1.0 / 16.0)
	if hasCb {
		cbCurr[x+0] += dcb * (7.0 / 16.0)
		cbNext[x+2] += dcb * (3.0 / 16.0)
		cbNext[x+1] += dcb * (5.0 / 16.0)
		cbNext[x+0] += dcb * (1.0 / 16.0)
	}
	if hasCr {
		crCurr[x+0] += dcr * (7.0 / 16.0)
		crNext[x+2] += dcr * (3.0 / 16.0)
		crNext[x+1] += dcr * (5.0 / 16.0)
		crNext[x+0] += dcr * (1.0 / 16.0)
	}
}

func quantLevels(low, high uint8, bits int) []uint8 {
	if bits < 1 {
		return []uint8{128}
	}
	levelCount := (1 << bits) - 1
	out := make([]uint8, levelCount+1)
	for i := 0; i <= levelCount; i++ {
		out[i] = low + uint8((int(high-low)*i+levelCount/2)/levelCount)
	}
	return out
}

func paletteDistance(ty, tcb, tcr, py, pcb, pcr float64, hasCb, hasCr bool) float64 {
	dy := ty - py
	dist := dy * dy * 2.4
	if hasCb {
		dcb := tcb - pcb
		dist += dcb * dcb * 0.85
	}
	if hasCr {
		dcr := tcr - pcr
		dist += dcr * dcr * 0.85
	}
	return dist
}

func clamp255(v float64) float64 {
	if v < 0 {
		return 0
	}
	if v > 255 {
		return 255
	}
	return v
}

func quantizePhotoLumaToBits(plane []uint8, w, h, quality, bwBits, phaseX, phaseY int, shuffle bool, dst *bytes.Buffer, errCurr, errNext []float64) {
	for i := range errCurr {
		errCurr[i] = 0
		errNext[i] = 0
	}

	bw := newBitWriter(dst)
	baseAmp := ditherAmplitude(18, quality)
	fsStrength := fsDiffusionStrength(quality)
	detailStrength := 0.42 + float64(quality)/320.0
	gamma := 0.88 + float64(100-quality)/650.0
	levels := quantLevels(bwYLow, bwYHigh, bwBits)
	rowLevels := make([]uint8, w)

	for y := 0; y < h; y++ {
		if y&1 == 0 {
			for x := 0; x < w; x++ {
				target := bwPhotoTarget(plane, w, h, x, y, gamma, detailStrength)*255.0 + errCurr[x+1]
				target = clamp255(target + noiseBias(x, y, phaseX, phaseY, adaptivePhotoAmplitude(plane, w, h, x, y, baseAmp), shuffle))
				level, out := nearestGrayLevel(target, levels)
				rowLevels[x] = level
				diff := target - out
				diff *= fsStrength
				errCurr[x+2] += diff * (7.0 / 16.0)
				errNext[x+0] += diff * (3.0 / 16.0)
				errNext[x+1] += diff * (5.0 / 16.0)
				errNext[x+2] += diff * (1.0 / 16.0)
			}
		} else {
			for x := w - 1; x >= 0; x-- {
				target := bwPhotoTarget(plane, w, h, x, y, gamma, detailStrength)*255.0 + errCurr[x+1]
				target = clamp255(target + noiseBias(x, y, phaseX, phaseY, adaptivePhotoAmplitude(plane, w, h, x, y, baseAmp), shuffle))
				level, out := nearestGrayLevel(target, levels)
				rowLevels[x] = level
				diff := target - out
				diff *= fsStrength
				errCurr[x+0] += diff * (7.0 / 16.0)
				errNext[x+2] += diff * (3.0 / 16.0)
				errNext[x+1] += diff * (5.0 / 16.0)
				errNext[x+0] += diff * (1.0 / 16.0)
			}
		}
		for x := 0; x < w; x++ {
			bw.writeBits(uint64(rowLevels[x]), uint8(bwBits))
		}

		for i := range errCurr {
			errCurr[i], errNext[i] = errNext[i], 0
		}
	}

	bw.flush()
}

func nearestGrayLevel(target float64, levels []uint8) (uint8, float64) {
	bestIdx := 0
	bestOut := float64(levels[0])
	bestDist := math.Abs(target - bestOut)
	for i := 1; i < len(levels); i++ {
		out := float64(levels[i])
		dist := math.Abs(target - out)
		if dist < bestDist {
			bestDist = dist
			bestIdx = i
			bestOut = out
		}
	}
	return uint8(bestIdx), bestOut
}

func bwPhotoTarget(plane []uint8, w, h, x, y int, gamma, detailStrength float64) float64 {
	idx := y*w + x
	base := math.Pow(float64(plane[idx])/255.0, gamma)
	local := neighborhoodMean(plane, w, h, x, y) / 255.0
	detail := base - local
	target := base + detail*detailStrength

	// Softer toe and shoulder to keep highlights and shadows from breaking apart.
	target = clamp01(target)
	target = 0.04 + target*0.92
	target = photographicCurve(target)
	return clamp01(target)
}

func photographicCurve(v float64) float64 {
	// Gentle S-curve with compressed toe/shoulder and more open midtones.
	v = clamp01(v)
	toe := math.Pow(v, 1.08)
	shoulder := 1.0 - math.Pow(1.0-v, 1.16)
	mix := 0.58 - math.Abs(v-0.5)*0.35
	if mix < 0.2 {
		mix = 0.2
	}
	return clamp01(toe*(1.0-mix) + shoulder*mix)
}

func neighborhoodMean(plane []uint8, w, h, x, y int) float64 {
	var sum int
	var count int
	for yy := max(0, y-neighborhoodBackSpan); yy <= min(h-1, y+neighborhoodForwardSpan); yy++ {
		row := yy * w
		for xx := max(0, x-neighborhoodBackSpan); xx <= min(w-1, x+neighborhoodForwardSpan); xx++ {
			sum += int(plane[row+xx])
			count++
		}
	}
	return float64(sum) / float64(count)
}

func neighborhoodRange(plane []uint8, w, h, x, y int) float64 {
	lo := 255
	hi := 0
	for yy := max(0, y-neighborhoodBackSpan); yy <= min(h-1, y+neighborhoodForwardSpan); yy++ {
		row := yy * w
		for xx := max(0, x-neighborhoodBackSpan); xx <= min(w-1, x+neighborhoodForwardSpan); xx++ {
			v := int(plane[row+xx])
			if v < lo {
				lo = v
			}
			if v > hi {
				hi = v
			}
		}
	}
	return float64(hi - lo)
}

func adaptiveDitherAmplitude(plane []uint8, w, h, x, y int, base float64) float64 {
	tone := float64(plane[y*w+x]) / 255.0
	rangeNorm := neighborhoodRange(plane, w, h, x, y) / 255.0
	midtone := 1.0 - math.Abs(tone-0.5)*2.0
	texture := 0.55 + rangeNorm*0.75
	toneWeight := 0.25 + midtone*0.75
	return base * toneWeight * texture
}

func adaptivePhotoAmplitude(plane []uint8, w, h, x, y int, base float64) float64 {
	tone := float64(plane[y*w+x]) / 255.0
	rangeNorm := neighborhoodRange(plane, w, h, x, y) / 255.0
	midtone := 1.0 - math.Abs(tone-0.5)*2.0
	texture := 0.35 + rangeNorm*0.55
	toneWeight := 0.12 + midtone*0.88
	return base * toneWeight * texture
}

func clamp01(v float64) float64 {
	if v < 0 {
		return 0
	}
	if v > 1 {
		return 1
	}
	return v
}

func ditherAmplitude(base, quality int) float64 {
	_ = quality
	return float64(base) * 3.0
}

func fsDiffusionStrength(quality int) float64 {
	return 0.22 + (float64(quality)/100.0)*0.78
}

func blueNoiseBias(x, y, phaseX, phaseY int, amplitude float64) float64 {
	tile := getBlueNoiseTile()
	mask := blueNoiseTileSize - 1
	xi := (x + phaseX) & mask
	yi := (y + phaseY) & mask
	sample := float64(tile[(yi*blueNoiseTileSize)+xi])
	maxRank := float64(len(tile) - 1)
	return ((sample / maxRank) - 0.5) * amplitude
}

func shuffleBias(x, y, phaseX, phaseY int, amplitude float64) float64 {
	v := splitmix64(uint64((x+phaseX)&0xffff) | (uint64((y+phaseY)&0xffff) << 16) | 0x9e3779b97f4a7c15)
	norm := float64(v&0xffff) / 65535.0
	return (norm - 0.5) * amplitude
}

func noiseBias(x, y, phaseX, phaseY int, amplitude float64, shuffle bool) float64 {
	if shuffle {
		return shuffleBias(x, y, phaseX, phaseY, amplitude)
	}
	return blueNoiseBias(x, y, phaseX, phaseY, amplitude)
}

func splitmix64(x uint64) uint64 {
	x += 0x9e3779b97f4a7c15
	x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9
	x = (x ^ (x >> 27)) * 0x94d049bb133111eb
	return x ^ (x >> 31)
}

func planeBlueNoisePhase(plane []uint8, w, h int, salt uint64) (int, int) {
	var hash uint64 = 1469598103934665603 ^ salt
	stepX := max(1, w/17)
	stepY := max(1, h/17)
	for y := 0; y < h; y += stepY {
		row := y * w
		for x := 0; x < w; x += stepX {
			hash ^= uint64(plane[row+x]) + uint64((x&255)<<8) + uint64((y&255)<<16)
			hash *= 1099511628211
		}
	}
	return int(hash & uint64(blueNoiseTileSize-1)), int((hash >> 8) & uint64(blueNoiseTileSize-1))
}

func getBlueNoiseTile() []uint16 {
	blueNoiseTileOnce.Do(func() {
		blueNoiseTile = generateBlueNoiseTileVoidAndCluster(blueNoiseTileSize)
	})
	return blueNoiseTile
}

func generateBlueNoiseTileVoidAndCluster(size int) []uint16 {
	n := size * size
	kernel := buildBlueNoiseKernel(size, 1.9)
	rng := newBlueNoiseRNG(0x9e3779b97f4a7c15)
	on := make([]bool, n)
	energy := make([]float64, n)

	targetOnes := n / 2
	for count := 0; count < targetOnes; {
		idx := int(rng.next() % uint64(n))
		if on[idx] {
			continue
		}
		on[idx] = true
		applyKernelDelta(energy, kernel, size, idx, 1)
		count++
	}

	for iter := 0; iter < n*4; iter++ {
		cluster := findTightestCluster(on, energy)
		applyKernelDelta(energy, kernel, size, cluster, -1)
		on[cluster] = false

		void := findLargestVoid(on, energy)
		if void == cluster {
			on[cluster] = true
			applyKernelDelta(energy, kernel, size, cluster, 1)
			break
		}
		on[void] = true
		applyKernelDelta(energy, kernel, size, void, 1)
	}

	ranks := make([]uint16, n)
	workOn := append([]bool(nil), on...)
	workEnergy := append([]float64(nil), energy...)
	half := 0
	for _, v := range workOn {
		if v {
			half++
		}
	}

	for rank := half - 1; rank >= 0; rank-- {
		cluster := findTightestCluster(workOn, workEnergy)
		ranks[cluster] = uint16(rank)
		workOn[cluster] = false
		applyKernelDelta(workEnergy, kernel, size, cluster, -1)
	}

	workOn = append([]bool(nil), on...)
	workEnergy = append([]float64(nil), energy...)
	for rank := half; rank < n; rank++ {
		void := findLargestVoid(workOn, workEnergy)
		ranks[void] = uint16(rank)
		workOn[void] = true
		applyKernelDelta(workEnergy, kernel, size, void, 1)
	}

	return ranks
}

func buildBlueNoiseKernel(size int, sigma float64) []float64 {
	kernel := make([]float64, size*size)
	twoSigma2 := 2 * sigma * sigma
	for y := 0; y < size; y++ {
		dy := min(y, size-y)
		for x := 0; x < size; x++ {
			dx := min(x, size-x)
			d2 := float64(dx*dx + dy*dy)
			kernel[y*size+x] = math.Exp(-d2 / twoSigma2)
		}
	}
	return kernel
}

func applyKernelDelta(energy, kernel []float64, size, idx, sign int) {
	x0 := idx % size
	y0 := idx / size
	scale := float64(sign)
	for y := 0; y < size; y++ {
		yy := (y + y0) & (size - 1)
		rowOff := yy * size
		krow := y * size
		for x := 0; x < size; x++ {
			xx := (x + x0) & (size - 1)
			energy[rowOff+xx] += kernel[krow+x] * scale
		}
	}
}

func findTightestCluster(on []bool, energy []float64) int {
	bestIdx := -1
	bestEnergy := -math.MaxFloat64
	for i, active := range on {
		if active && energy[i] > bestEnergy {
			bestEnergy = energy[i]
			bestIdx = i
		}
	}
	return bestIdx
}

func findLargestVoid(on []bool, energy []float64) int {
	bestIdx := -1
	bestEnergy := math.MaxFloat64
	for i, active := range on {
		if !active && energy[i] < bestEnergy {
			bestEnergy = energy[i]
			bestIdx = i
		}
	}
	return bestIdx
}

type blueNoiseRNG struct {
	state uint64
}

func newBlueNoiseRNG(seed uint64) blueNoiseRNG {
	if seed == 0 {
		seed = 1
	}
	return blueNoiseRNG{state: seed}
}

func (r *blueNoiseRNG) next() uint64 {
	x := r.state
	x ^= x >> 12
	x ^= x << 25
	x ^= x >> 27
	r.state = x
	return x * 2685821657736338717
}

func writeBitPlane(w *bufio.Writer, bits []byte) error {
	if err := writeU32BE(w, uint32(len(bits))); err != nil {
		return err
	}
	_, err := w.Write(bits)
	return err
}

func readBitPlane(payload []byte, pos *int, label string) ([]byte, error) {
	n, err := readU32BE(payload, pos, label+" plane length")
	if err != nil {
		return nil, err
	}
	if n > len(payload)-*pos {
		return nil, fmt.Errorf("decode: truncated while reading %s plane", label)
	}
	start := *pos
	*pos += n
	return payload[start:*pos], nil
}

func decodeToRGBA(dst *image.RGBA, yBits, cbBits, crBits []byte, hasCb, hasCr bool, yBitDepth, cbBitDepth, crBitDepth int, yStorageMode byte, patternW, patternH int, colorMode byte, parallel bool) error {
	b := dst.Bounds()
	w := b.Dx()
	h := b.Dy()
	if colorMode == colorModePaletteRGB {
		return decodePaletteToRGBA(dst, yBits, yBitDepth, yStorageMode, patternW, patternH)
	}
	if yStorageMode == yStorageRaw && len(yBits) < packedPlaneLenBits(w, h, yBitDepth) {
		return fmt.Errorf("decode: Y plane too short")
	}
	if hasCb && len(cbBits) < packedPlaneLenBits(w, h, cbBitDepth) {
		return fmt.Errorf("decode: Cb plane too short")
	}
	if hasCr && len(crBits) < packedPlaneLenBits(w, h, crBitDepth) {
		return fmt.Errorf("decode: Cr plane too short")
	}

	yLo := uint8(yLow)
	yHi := uint8(yHigh)
	c0Lo, c0Hi := yLo, yHi
	c1Lo, c1Hi := uint8(cbLow), uint8(cbHigh)
	c2Lo, c2Hi := uint8(crLow), uint8(crHigh)
	if (colorMode == colorModeRGB || colorMode == colorModeYCbCr) && (hasCb || hasCr) {
		c0Lo, c0Hi = 0, 255
		c1Lo, c1Hi = 0, 255
		c2Lo, c2Hi = 0, 255
	}
	if !hasCb && !hasCr {
		yLo = bwYLow
		yHi = bwYHigh
		c0Lo, c0Hi = yLo, yHi
	}
	var yPlane []uint8
	var err error
	if !hasCb && !hasCr && yBitDepth == 1 {
		switch yStorageMode {
		case yStoragePatternGridDirect:
			yPlane, err = unpackMonoPatternGridDirect(yBits, w, h, patternW, patternH, yLo, yHi)
		case yStoragePatternGrid:
			yPlane, err = unpackMonoPatternGrid(yBits, w, h, patternW, patternH, yLo, yHi)
		default:
			yPlane, err = unpackPlaneLevels(yBits, w, h, yBitDepth, c0Lo, c0Hi)
		}
		if err != nil {
			return err
		}
	} else if !hasCb && !hasCr && yStorageMode == yStoragePatternLayers {
		yPlane, err = unpackPatternLayers(yBits, w, h, yBitDepth, patternW, patternH, yLo, yHi)
		if err != nil {
			return err
		}
	} else {
		yPlane, err = unpackPlaneLevels(yBits, w, h, yBitDepth, c0Lo, c0Hi)
		if err != nil {
			return err
		}
	}

	var cbPlane []uint8
	if hasCb {
		cbPlane, err = unpackPlaneLevels(cbBits, w, h, cbBitDepth, c1Lo, c1Hi)
		if err != nil {
			return err
		}
	}

	var crPlane []uint8
	if hasCr {
		crPlane, err = unpackPlaneLevels(crBits, w, h, crBitDepth, c2Lo, c2Hi)
		if err != nil {
			return err
		}
	}

	if !parallel || h < 32 {
		fillRGBAStripe(dst.Pix, dst.Stride, w, 0, h, yPlane, cbPlane, crPlane, hasCb, hasCr, colorMode)
		return nil
	}

	workers := min(runtime.NumCPU(), h)
	rowsPerWorker := (h + workers - 1) / workers
	var wg sync.WaitGroup
	for i := 0; i < workers; i++ {
		y0 := i * rowsPerWorker
		if y0 >= h {
			break
		}
		y1 := min(y0+rowsPerWorker, h)
		wg.Add(1)
		go func(y0, y1 int) {
			defer wg.Done()
			fillRGBAStripe(dst.Pix, dst.Stride, w, y0, y1, yPlane, cbPlane, crPlane, hasCb, hasCr, colorMode)
		}(y0, y1)
	}
	wg.Wait()
	return nil
}

func decodePaletteToRGBA(dst *image.RGBA, data []byte, bitDepth int, yStorageMode byte, patternW, patternH int) error {
	if len(data) < 2 {
		return fmt.Errorf("decode: truncated palette payload")
	}
	paletteSize := int(binary.BigEndian.Uint16(data[:2]))
	if paletteSize < 1 {
		return fmt.Errorf("decode: invalid palette size")
	}
	if len(data) < 2+paletteSize*3 {
		return fmt.Errorf("decode: truncated palette colors")
	}
	palette := data[2 : 2+paletteSize*3]
	w := dst.Bounds().Dx()
	h := dst.Bounds().Dy()
	if yStorageMode != yStorageRaw {
		return fmt.Errorf("decode: palette mode only supports raw index storage")
	}
	pos := 2 + paletteSize*3
	if len(data)-pos < 1 {
		return fmt.Errorf("decode: truncated palette submode")
	}
	submode := int(data[pos])
	pos++
	if submode == 2 {
		if len(data)-pos < 2 {
			return fmt.Errorf("decode: truncated tile subset header")
		}
		tileW := int(data[pos])
		tileH := int(data[pos+1])
		pos += 2
		if tileW < 1 || tileH < 1 {
			return fmt.Errorf("decode: invalid tile subset size")
		}
		tilesX := ceilDiv(w, tileW)
		tilesY := ceilDiv(h, tileH)
		for ty := 0; ty < tilesY; ty++ {
			y0 := ty * tileH
			y1 := min(y0+tileH, h)
			for tx := 0; tx < tilesX; tx++ {
				x0 := tx * tileW
				x1 := min(x0+tileW, w)
				if len(data)-pos < 1 {
					return fmt.Errorf("decode: truncated tile subset size")
				}
				subsetSize := int(data[pos])
				pos++
				if subsetSize < 1 || subsetSize > paletteSize {
					return fmt.Errorf("decode: invalid tile subset size")
				}
				if len(data)-pos < subsetSize {
					return fmt.Errorf("decode: truncated tile subset")
				}
				subset := make([]int, subsetSize)
				for i := 0; i < subsetSize; i++ {
					subset[i] = int(data[pos+i])
					if subset[i] < 0 || subset[i] >= paletteSize {
						return fmt.Errorf("decode: tile subset index out of range")
					}
				}
				pos += subsetSize
				localBits := bitsNeeded(subsetSize - 1)
				if localBits < 1 {
					localBits = 1
				}
				pixels := (x1 - x0) * (y1 - y0)
				tileBytes := (pixels*localBits + 7) >> 3
				if len(data)-pos < tileBytes {
					return fmt.Errorf("decode: truncated tile local index stream")
				}
				br := newBitReader(data[pos : pos+tileBytes])
				pos += tileBytes
				order := zOrderIndices(x1-x0, y1-y0)
				values := make([]int, len(order))
				for bit := localBits - 1; bit >= 0; bit-- {
					for i := range values {
						b, err := br.readBit()
						if err != nil {
							return fmt.Errorf("decode: truncated tile local index stream")
						}
						values[i] <<= 1
						if b {
							values[i] |= 1
						}
					}
				}
				for i, localPi := range order {
					v := values[i]
					if v >= len(subset) {
						return fmt.Errorf("decode: tile local index out of range")
					}
					lx := localPi % (x1 - x0)
					ly := localPi / (x1 - x0)
					idx := subset[v]
					p := idx * 3
					off := (y0+ly)*dst.Stride + (x0+lx)*4
					dst.Pix[off+0] = palette[p+0]
					dst.Pix[off+1] = palette[p+1]
					dst.Pix[off+2] = palette[p+2]
					dst.Pix[off+3] = 255
				}
			}
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 5 {
		if len(data)-pos < 1 {
			return fmt.Errorf("decode: truncated block subset header")
		}
		blockSize := int(data[pos])
		pos++
		if blockSize < 1 {
			return fmt.Errorf("decode: invalid block subset size")
		}
		for by := 0; by < h; by += blockSize {
			y1 := min(by+blockSize, h)
			for bx := 0; bx < w; bx += blockSize {
				x1 := min(bx+blockSize, w)
				if len(data)-pos < 1 {
					return fmt.Errorf("decode: truncated block subset size")
				}
				subsetSize := int(data[pos])
				pos++
				if subsetSize < 1 || subsetSize > paletteSize {
					return fmt.Errorf("decode: invalid block subset size")
				}
				if len(data)-pos < subsetSize {
					return fmt.Errorf("decode: truncated block subset")
				}
				subset := make([]int, subsetSize)
				for i := 0; i < subsetSize; i++ {
					subset[i] = int(data[pos+i])
					if subset[i] < 0 || subset[i] >= paletteSize {
						return fmt.Errorf("decode: block subset index out of range")
					}
				}
				pos += subsetSize
				localBits := bitsNeeded(subsetSize - 1)
				if localBits < 1 {
					localBits = 1
				}
				pixels := (x1 - bx) * (y1 - by)
				blockBytes := (pixels*localBits + 7) >> 3
				if len(data)-pos < blockBytes {
					return fmt.Errorf("decode: truncated block subset index stream")
				}
				br := newBitReader(data[pos : pos+blockBytes])
				pos += blockBytes
				for y := by; y < y1; y++ {
					for x := bx; x < x1; x++ {
						v, err := br.readBits(uint8(localBits))
						if err != nil {
							return fmt.Errorf("decode: truncated block subset index stream")
						}
						if int(v) >= len(subset) {
							return fmt.Errorf("decode: block subset local index out of range")
						}
						idx := subset[int(v)]
						p := idx * 3
						off := y*dst.Stride + x*4
						dst.Pix[off+0] = palette[p+0]
						dst.Pix[off+1] = palette[p+1]
						dst.Pix[off+2] = palette[p+2]
						dst.Pix[off+3] = 255
					}
				}
			}
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 4 {
		if len(data)-pos < 1 {
			return fmt.Errorf("decode: truncated block raw header")
		}
		blockSize := int(data[pos])
		pos++
		if blockSize < 1 {
			return fmt.Errorf("decode: invalid block size")
		}
		indexBits := bitsNeeded(paletteSize - 1)
		if indexBits < 1 {
			indexBits = 1
		}
		for by := 0; by < h; by += blockSize {
			y1 := min(by+blockSize, h)
			for bx := 0; bx < w; bx += blockSize {
				x1 := min(bx+blockSize, w)
				pixels := (x1 - bx) * (y1 - by)
				blockBytes := (pixels*indexBits + 7) >> 3
				if len(data)-pos < blockBytes {
					return fmt.Errorf("decode: truncated block raw index stream")
				}
				br := newBitReader(data[pos : pos+blockBytes])
				pos += blockBytes
				for y := by; y < y1; y++ {
					for x := bx; x < x1; x++ {
						idx, err := br.readBits(uint8(indexBits))
						if err != nil {
							return fmt.Errorf("decode: truncated block raw index stream")
						}
						if int(idx) >= paletteSize {
							return fmt.Errorf("decode: block raw palette index out of range")
						}
						p := int(idx) * 3
						off := y*dst.Stride + x*4
						dst.Pix[off+0] = palette[p+0]
						dst.Pix[off+1] = palette[p+1]
						dst.Pix[off+2] = palette[p+2]
						dst.Pix[off+3] = 255
					}
				}
			}
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 1 {
		indexBits := bitsNeeded(paletteSize - 1)
		if indexBits < 1 {
			indexBits = 1
		}
		pixels := w * h
		streamBytes := (pixels*indexBits + 7) >> 3
		if len(data)-pos < streamBytes {
			return fmt.Errorf("decode: truncated raw palette index stream")
		}
		br := newBitReader(data[pos : pos+streamBytes])
		for i := 0; i < pixels; i++ {
			idx, err := br.readBits(uint8(indexBits))
			if err != nil {
				return fmt.Errorf("decode: truncated raw palette index stream")
			}
			if int(idx) >= paletteSize {
				return fmt.Errorf("decode: raw palette index out of range")
			}
			p := int(idx) * 3
			off := (i/w)*dst.Stride + (i%w)*4
			dst.Pix[off+0] = palette[p+0]
			dst.Pix[off+1] = palette[p+1]
			dst.Pix[off+2] = palette[p+2]
			dst.Pix[off+3] = 255
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 6 {
		indexBits := bitsNeeded(paletteSize - 1)
		if indexBits < 1 {
			indexBits = 1
		}
		totalBits := w * h * indexBits
		if len(data)-pos < 19 {
			return fmt.Errorf("decode: truncated raw top16 header")
		}
		shift := int(data[pos])
		pos++
		prefixBits := data[pos]
		pos++
		suffixBits := data[pos]
		pos++
		if shift < 0 || shift > 7 {
			return fmt.Errorf("decode: invalid raw top16 shift")
		}
		if prefixBits>>shift != 0 {
			// kept packed in high bits only
		}
		var topA [16]byte
		copy(topA[:], data[pos:pos+16])
		pos += 16
		byteCount := 0
		if totalBits > shift {
			byteCount = (totalBits - shift) / 8
		}
		maskBytes := (byteCount + 7) >> 3
		if len(data)-pos < maskBytes {
			return fmt.Errorf("decode: truncated raw top16 mask")
		}
		maskA := data[pos : pos+maskBytes]
		pos += maskBytes
		topCountA := 0
		for i := 0; i < byteCount; i++ {
			if monoBitAt(maskA, i) {
				topCountA++
			}
		}
		idBytesA := (topCountA*4 + 7) >> 3
		if len(data)-pos < idBytesA {
			return fmt.Errorf("decode: truncated raw top16 id stream")
		}
		idReaderA := newBitReader(data[pos : pos+idBytesA])
		pos += idBytesA
		otherCountA := byteCount - topCountA
		if len(data)-pos < otherCountA {
			return fmt.Errorf("decode: truncated raw top16 other stream")
		}
		otherA := data[pos : pos+otherCountA]
		pos += otherCountA
		byteStream := make([]byte, byteCount)
		otherPos := 0
		for i := 0; i < byteCount; i++ {
			if monoBitAt(maskA, i) {
				v, err := idReaderA.readBits(4)
				if err != nil {
					return fmt.Errorf("decode: truncated raw top16 symbol")
				}
				byteStream[i] = topA[int(v)]
			} else {
				byteStream[i] = otherA[otherPos]
				otherPos++
			}
		}
		rawBits := make([]byte, (totalBits+7)>>3)
		for i := 0; i < shift; i++ {
			if (prefixBits & (1 << (7 - i))) != 0 {
				monoBitSet(rawBits, i)
			}
		}
		for i, b := range byteStream {
			base := shift + i*8
			for j := 0; j < 8; j++ {
				if (b & (1 << (7 - j))) != 0 {
					monoBitSet(rawBits, base+j)
				}
			}
		}
		suffixCount := (totalBits - shift) & 7
		suffixStart := totalBits - suffixCount
		for i := 0; i < suffixCount; i++ {
			if (suffixBits & (1 << (7 - i))) != 0 {
				monoBitSet(rawBits, suffixStart+i)
			}
		}
		br := newBitReader(rawBits)
		pixels := w * h
		for i := 0; i < pixels; i++ {
			idx, err := br.readBits(uint8(indexBits))
			if err != nil {
				return fmt.Errorf("decode: truncated raw palette index stream")
			}
			if int(idx) >= paletteSize {
				return fmt.Errorf("decode: raw palette index out of range")
			}
			p := int(idx) * 3
			off := (i/w)*dst.Stride + (i%w)*4
			dst.Pix[off+0] = palette[p+0]
			dst.Pix[off+1] = palette[p+1]
			dst.Pix[off+2] = palette[p+2]
			dst.Pix[off+3] = 255
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 7 {
		values, err := decodeRawIndexTree(data, &pos, w*h, 0, paletteSize)
		if err != nil {
			return err
		}
		for i, idx := range values {
			if idx < 0 || idx >= paletteSize {
				return fmt.Errorf("decode: raw tree palette index out of range")
			}
			p := idx * 3
			off := (i/w)*dst.Stride + (i%w)*4
			dst.Pix[off+0] = palette[p+0]
			dst.Pix[off+1] = palette[p+1]
			dst.Pix[off+2] = palette[p+2]
			dst.Pix[off+3] = 255
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 8 {
		indexBits := bitsNeeded(paletteSize - 1)
		if indexBits < 1 {
			indexBits = 1
		}
		totalBits := w * h * indexBits
		if len(data)-pos < 19 {
			return fmt.Errorf("decode: truncated raw top16 tree header")
		}
		shift := int(data[pos])
		pos++
		prefixBits := data[pos]
		pos++
		suffixBits := data[pos]
		pos++
		if shift < 0 || shift > 7 {
			return fmt.Errorf("decode: invalid raw top16 tree shift")
		}
		var topA [16]byte
		copy(topA[:], data[pos:pos+16])
		pos += 16
		byteCount := 0
		if totalBits > shift {
			byteCount = (totalBits - shift) / 8
		}
		maskBytes := (byteCount + 7) >> 3
		if len(data)-pos < maskBytes {
			return fmt.Errorf("decode: truncated raw top16 tree mask")
		}
		maskA := data[pos : pos+maskBytes]
		pos += maskBytes
		topCountA := 0
		for i := 0; i < byteCount; i++ {
			if monoBitAt(maskA, i) {
				topCountA++
			}
		}
		idBytesA := (topCountA*4 + 7) >> 3
		if len(data)-pos < idBytesA {
			return fmt.Errorf("decode: truncated raw top16 tree id stream")
		}
		idReaderA := newBitReader(data[pos : pos+idBytesA])
		pos += idBytesA
		otherCountA := byteCount - topCountA
		otherVals, err := decodeRawIndexTree(data, &pos, otherCountA, 0, 256)
		if err != nil {
			return err
		}
		byteStream := make([]byte, byteCount)
		otherPos := 0
		for i := 0; i < byteCount; i++ {
			if monoBitAt(maskA, i) {
				v, err := idReaderA.readBits(4)
				if err != nil {
					return fmt.Errorf("decode: truncated raw top16 tree symbol")
				}
				byteStream[i] = topA[int(v)]
			} else {
				byteStream[i] = byte(otherVals[otherPos])
				otherPos++
			}
		}
		rawBits := make([]byte, (totalBits+7)>>3)
		for i := 0; i < shift; i++ {
			if (prefixBits & (1 << (7 - i))) != 0 {
				monoBitSet(rawBits, i)
			}
		}
		for i, b := range byteStream {
			base := shift + i*8
			for j := 0; j < 8; j++ {
				if (b & (1 << (7 - j))) != 0 {
					monoBitSet(rawBits, base+j)
				}
			}
		}
		suffixCount := (totalBits - shift) & 7
		suffixStart := totalBits - suffixCount
		for i := 0; i < suffixCount; i++ {
			if (suffixBits & (1 << (7 - i))) != 0 {
				monoBitSet(rawBits, suffixStart+i)
			}
		}
		br := newBitReader(rawBits)
		pixels := w * h
		for i := 0; i < pixels; i++ {
			idx, err := br.readBits(uint8(indexBits))
			if err != nil {
				return fmt.Errorf("decode: truncated raw palette index stream")
			}
			if int(idx) >= paletteSize {
				return fmt.Errorf("decode: raw palette index out of range")
			}
			p := int(idx) * 3
			off := (i/w)*dst.Stride + (i%w)*4
			dst.Pix[off+0] = palette[p+0]
			dst.Pix[off+1] = palette[p+1]
			dst.Pix[off+2] = palette[p+2]
			dst.Pix[off+3] = 255
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 9 {
		if len(data)-pos < paletteSize {
			return fmt.Errorf("decode: truncated raw treeadapt order")
		}
		order := append([]byte(nil), data[pos:pos+paletteSize]...)
		pos += paletteSize
		values, err := decodeRawIndexTreeFreq(data, &pos, w*h, order)
		if err != nil {
			return err
		}
		for i, idx := range values {
			if idx < 0 || idx >= paletteSize {
				return fmt.Errorf("decode: raw treeadapt palette index out of range")
			}
			p := idx * 3
			off := (i/w)*dst.Stride + (i%w)*4
			dst.Pix[off+0] = palette[p+0]
			dst.Pix[off+1] = palette[p+1]
			dst.Pix[off+2] = palette[p+2]
			dst.Pix[off+3] = 255
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	if submode == 3 {
		if len(data)-pos < 2 {
			return fmt.Errorf("decode: truncated reconstruct palette header")
		}
		usedCount := int(binary.BigEndian.Uint16(data[pos : pos+2]))
		pos += 2
		if usedCount < 1 || usedCount > paletteSize {
			return fmt.Errorf("decode: invalid reconstruct palette size")
		}
		if len(data)-pos < usedCount {
			return fmt.Errorf("decode: truncated reconstruct palette order")
		}
		order := make([]int, usedCount)
		for i := 0; i < usedCount; i++ {
			order[i] = int(data[pos+i])
			if order[i] < 0 || order[i] >= paletteSize {
				return fmt.Errorf("decode: reconstruct palette index out of range")
			}
		}
		pos += usedCount
		remainingPos := make([]int, w*h)
		for i := range remainingPos {
			remainingPos[i] = i
		}
		for i := 0; i < usedCount-1; i++ {
			planeLen := (len(remainingPos) + 7) >> 3
			if len(data)-pos < planeLen {
				return fmt.Errorf("decode: truncated reconstruct plane data")
			}
			plane := data[pos : pos+planeLen]
			pos += planeLen
			idx := order[i]
			p := idx * 3
			nextRemaining := make([]int, 0, len(remainingPos))
			for j, pi := range remainingPos {
				if monoBitAt(plane, j) {
					off := (pi/w)*dst.Stride + (pi%w)*4
					dst.Pix[off+0] = palette[p+0]
					dst.Pix[off+1] = palette[p+1]
					dst.Pix[off+2] = palette[p+2]
					dst.Pix[off+3] = 255
				} else {
					nextRemaining = append(nextRemaining, pi)
				}
			}
			remainingPos = nextRemaining
		}
		base := order[usedCount-1]
		baseOff := base * 3
		for _, pi := range remainingPos {
			off := (pi/w)*dst.Stride + (pi%w)*4
			dst.Pix[off+0] = palette[baseOff+0]
			dst.Pix[off+1] = palette[baseOff+1]
			dst.Pix[off+2] = palette[baseOff+2]
			dst.Pix[off+3] = 255
		}
		for y := 0; y < h; y++ {
			rowOff := y * dst.Stride
			for x := 0; x < w; x++ {
				off := rowOff + x*4
				if dst.Pix[off+3] == 0 {
					dst.Pix[off+0] = palette[baseOff+0]
					dst.Pix[off+1] = palette[baseOff+1]
					dst.Pix[off+2] = palette[baseOff+2]
					dst.Pix[off+3] = 255
				}
			}
		}
		_, _, _ = bitDepth, patternW, patternH
		return nil
	}
	canvasIdx := submode
	if canvasIdx < 0 || canvasIdx >= paletteSize {
		return fmt.Errorf("decode: invalid canvas index")
	}
	maskLen := packedPlaneLenBits(w, h, 1)
	if len(data)-pos < maskLen {
		return fmt.Errorf("decode: truncated exception mask")
	}
	mask := data[pos : pos+maskLen]
	pos += maskLen
	br := newBitReader(data[pos:])
	literalCount := paletteSize - 1
	literalBits := bitsNeeded(literalCount - 1)
	invMap := make([]int, 0, literalCount)
	for pi := 0; pi < paletteSize; pi++ {
		if pi != canvasIdx {
			invMap = append(invMap, pi)
		}
	}
	for y := 0; y < h; y++ {
		rowOff := y * dst.Stride
		for x := 0; x < w; x++ {
			idx := canvasIdx
			if monoBitAt(mask, y*w+x) {
				if literalBits > 0 {
					v, err := br.readBits(uint8(literalBits))
					if err != nil {
						return fmt.Errorf("decode: truncated palette literal stream")
					}
					if int(v) >= len(invMap) {
						return fmt.Errorf("decode: palette literal out of range")
					}
					idx = invMap[int(v)]
				} else if len(invMap) == 1 {
					idx = invMap[0]
				} else {
					return fmt.Errorf("decode: invalid palette literal configuration")
				}
			}
			p := idx * 3
			off := rowOff + x*4
			dst.Pix[off+0] = palette[p+0]
			dst.Pix[off+1] = palette[p+1]
			dst.Pix[off+2] = palette[p+2]
			dst.Pix[off+3] = 255
		}
	}
	_, _ = patternW, patternH
	return nil
}

func fillRGBAStripe(pix []byte, stride, w, y0, y1 int, yPlane, cbPlane, crPlane []byte, hasCb, hasCr bool, colorMode byte) {
	for y := y0; y < y1; y++ {
		rowOff := y * stride
		for x := 0; x < w; x++ {
			idx := y*w + x
			var r, g, b uint8
			if colorMode == colorModeRGB && (hasCb || hasCr) {
				r = yPlane[idx]
				if hasCb {
					g = cbPlane[idx]
				}
				if hasCr {
					b = crPlane[idx]
				}
			} else {
				yy := yPlane[idx]
				cb := uint8(128)
				cr := uint8(128)
				if hasCb {
					cb = cbPlane[idx]
				}
				if hasCr {
					cr = crPlane[idx]
				}
				r, g, b = color.YCbCrToRGB(yy, cb, cr)
			}
			o := rowOff + x*4
			pix[o+0] = r
			pix[o+1] = g
			pix[o+2] = b
			pix[o+3] = 255
		}
	}
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func zOrderIndices(w, h int) []int {
	if w <= 0 || h <= 0 {
		return nil
	}
	maxDim := w
	if h > maxDim {
		maxDim = h
	}
	side := 1
	for side < maxDim {
		side <<= 1
	}
	total := w * h
	out := make([]int, 0, total)
	for code := 0; len(out) < total; code++ {
		x, y := decodeMorton2(uint32(code))
		if x >= side || y >= side {
			break
		}
		if x < w && y < h {
			out = append(out, y*w+x)
		}
	}
	return out
}

func decodeMorton2(code uint32) (int, int) {
	var x, y uint32
	for bit := uint(0); bit < 16; bit++ {
		x |= ((code >> (bit * 2)) & 1) << bit
		y |= ((code >> (bit*2 + 1)) & 1) << bit
	}
	return int(x), int(y)
}

func reconstructLevel(bits []byte, idx int, low, high uint8) uint8 {
	byteIdx := idx >> 3
	shift := 7 - uint(idx&7)
	if ((bits[byteIdx] >> shift) & 1) != 0 {
		return high
	}
	return low
}

func unpackPlaneLevels(data []byte, w, h, bits int, low, high uint8) ([]uint8, error) {
	n := w * h
	br := newBitReader(data)
	out := make([]uint8, n)
	levelCount := (1 << bits) - 1
	for i := 0; i < n; i++ {
		v, err := br.readBits(uint8(bits))
		if err != nil {
			return nil, fmt.Errorf("decode: truncated plane")
		}
		if levelCount == 0 {
			out[i] = low
			continue
		}
		out[i] = low + uint8((int(high-low)*int(v)+levelCount/2)/levelCount)
	}
	return out, nil
}

func repackMonoBitsToPatternGrid(src []byte, w, h, patternW, patternH int, dst *bytes.Buffer) {
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	blockBits := patternW * patternH
	blockBytes := (blockBits + 7) >> 3
	type entry struct {
		key   string
		count int
		data  []byte
	}
	order := make([][]byte, 0, blocksX*blocksY)
	freq := make(map[string]*entry)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			x0 := bx * patternW
			block := make([]byte, blockBytes)
			bitPos := 0
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					bit := x < w && y < h && monoBitAt(src, y*w+x)
					if bit {
						byteIdx := bitPos >> 3
						shift := 7 - uint(bitPos&7)
						block[byteIdx] |= 1 << shift
					}
					bitPos++
				}
			}
			order = append(order, block)
			key := string(block)
			if e := freq[key]; e != nil {
				e.count++
			} else {
				freq[key] = &entry{key: key, count: 1, data: append([]byte(nil), block...)}
			}
		}
	}
	palette := make([]entry, 0, len(freq))
	for _, e := range freq {
		palette = append(palette, *e)
	}
	sort.Slice(palette, func(i, j int) bool {
		if palette[i].count != palette[j].count {
			return palette[i].count > palette[j].count
		}
		return palette[i].key < palette[j].key
	})
	indexOf := make(map[string]int, len(palette))
	for i, e := range palette {
		indexOf[e.key] = i
	}
	indexBits := bitsNeeded(len(palette) - 1)
	if indexBits < 1 {
		indexBits = 1
	}
	var hdr [10]byte
	binary.BigEndian.PutUint32(hdr[0:4], uint32(len(palette)))
	hdr[4] = byte(blockBytes)
	hdr[5] = byte(indexBits)
	binary.BigEndian.PutUint32(hdr[6:10], uint32(len(order)))
	dst.Write(hdr[:])
	for _, e := range palette {
		dst.Write(e.data)
	}
	bw := newBitWriter(dst)
	for _, block := range order {
		bw.writeBits(uint64(indexOf[string(block)]), uint8(indexBits))
	}
	bw.flush()
}

func buildGlobalPatternPalette(planes [][]byte, w, h, patternW, patternH int) ([]byte, int, int, map[string]int) {
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	blockBits := patternW * patternH
	blockBytes := (blockBits + 7) >> 3
	type entry struct {
		key   string
		count int
		data  []byte
	}
	freq := make(map[string]*entry)
	for _, src := range planes {
		for by := 0; by < blocksY; by++ {
			y0 := by * patternH
			for bx := 0; bx < blocksX; bx++ {
				x0 := bx * patternW
				block := make([]byte, blockBytes)
				bitPos := 0
				for dy := 0; dy < patternH; dy++ {
					y := y0 + dy
					for dx := 0; dx < patternW; dx++ {
						x := x0 + dx
						bit := x < w && y < h && monoBitAt(src, y*w+x)
						if bit {
							byteIdx := bitPos >> 3
							shift := 7 - uint(bitPos&7)
							block[byteIdx] |= 1 << shift
						}
						bitPos++
					}
				}
				key := string(block)
				if e := freq[key]; e != nil {
					e.count++
				} else {
					freq[key] = &entry{key: key, count: 1, data: append([]byte(nil), block...)}
				}
			}
		}
	}
	entries := make([]entry, 0, len(freq))
	for _, e := range freq {
		entries = append(entries, *e)
	}
	sort.Slice(entries, func(i, j int) bool {
		if entries[i].count != entries[j].count {
			return entries[i].count > entries[j].count
		}
		return entries[i].key < entries[j].key
	})
	indexOf := make(map[string]int, len(entries))
	paletteData := make([]byte, 0, len(entries)*blockBytes)
	for i, e := range entries {
		indexOf[e.key] = i
		paletteData = append(paletteData, e.data...)
	}
	indexBits := bitsNeeded(len(entries) - 1)
	if indexBits < 1 {
		indexBits = 1
	}
	return paletteData, blockBytes, indexBits, indexOf
}

func pairPlaneStates(a, b []byte, pixelCount int) []uint8 {
	out := make([]uint8, pixelCount)
	for i := 0; i < pixelCount; i++ {
		var v uint8
		if monoBitAt(a, i) {
			v |= 0x2
		}
		if monoBitAt(b, i) {
			v |= 0x1
		}
		out[i] = v
	}
	return out
}

func repackStatePatternGrid(states []uint8, w, h, patternW, patternH, stateBits int, dst *bytes.Buffer) {
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	blockBits := patternW * patternH * stateBits
	blockBytes := (blockBits + 7) >> 3
	type entry struct {
		key   string
		count int
		data  []byte
	}
	order := make([][]byte, 0, blocksX*blocksY)
	freq := make(map[string]*entry)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			x0 := bx * patternW
			block := make([]byte, blockBytes)
			var tmp bytes.Buffer
			bw := newBitWriter(&tmp)
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					v := uint8(0)
					if x < w && y < h {
						v = states[y*w+x]
					}
					bw.writeBits(uint64(v), uint8(stateBits))
				}
			}
			bw.flush()
			copy(block, tmp.Bytes())
			order = append(order, block)
			key := string(block)
			if e := freq[key]; e != nil {
				e.count++
			} else {
				freq[key] = &entry{key: key, count: 1, data: append([]byte(nil), block...)}
			}
		}
	}
	palette := make([]entry, 0, len(freq))
	for _, e := range freq {
		palette = append(palette, *e)
	}
	sort.Slice(palette, func(i, j int) bool {
		if palette[i].count != palette[j].count {
			return palette[i].count > palette[j].count
		}
		return palette[i].key < palette[j].key
	})
	indexOf := make(map[string]int, len(palette))
	for i, e := range palette {
		indexOf[e.key] = i
	}
	indexBits := bitsNeeded(len(palette) - 1)
	if indexBits < 1 {
		indexBits = 1
	}
	var hdr [10]byte
	binary.BigEndian.PutUint32(hdr[0:4], uint32(len(palette)))
	hdr[4] = byte(blockBytes)
	hdr[5] = byte(indexBits)
	binary.BigEndian.PutUint32(hdr[6:10], uint32(len(order)))
	dst.Write(hdr[:])
	for _, e := range palette {
		dst.Write(e.data)
	}
	bw := newBitWriter(dst)
	for _, block := range order {
		bw.writeBits(uint64(indexOf[string(block)]), uint8(indexBits))
	}
	bw.flush()
}

func repackStatePatternGridDirect(states []uint8, w, h, patternW, patternH, stateBits int, dst *bytes.Buffer) {
	bw := newBitWriter(dst)
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			x0 := bx * patternW
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					v := uint8(0)
					if x < w && y < h {
						v = states[y*w+x]
					}
					bw.writeBits(uint64(v), uint8(stateBits))
				}
			}
		}
	}
	bw.flush()
}

func repackMonoBitsToPatternGridDirect(src []byte, w, h, patternW, patternH int, dst *bytes.Buffer) {
	bw := newBitWriter(dst)
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			x0 := bx * patternW
			var pattern uint64
			bitPos := 0
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					if x < w && y < h && monoBitAt(src, y*w+x) {
						pattern |= 1 << bitPos
					}
					bitPos++
				}
			}
			bw.writeBits(pattern, uint8(patternW*patternH))
		}
	}
	bw.flush()
}

func repackMultiLevelToPatternLayers(src []byte, w, h, bits, patternW, patternH int, dst *bytes.Buffer) {
	n := w * h
	br := newBitReader(src)
	mono := make([][]byte, bits)
	for i := 0; i < bits; i++ {
		mono[i] = make([]byte, packedPlaneLenBits(w, h, 1))
	}
	for idx := 0; idx < n; idx++ {
		level, err := br.readBitsInt(bits)
		if err != nil {
			panic("repack multilevel pattern layers: truncated source")
		}
		for bit := 0; bit < bits; bit++ {
			shiftBit := bits - 1 - bit
			if ((level >> shiftBit) & 1) != 0 {
				byteIdx := idx >> 3
				shift := 7 - uint(idx&7)
				mono[bit][byteIdx] |= 1 << shift
			}
		}
	}
	dst.WriteByte(byte(bits))
	for i := 0; i < bits; i++ {
		var layer bytes.Buffer
		if patternW == 2 && patternH == 2 {
			repackMonoBitsToPatternGridDirect(mono[i], w, h, patternW, patternH, &layer)
		} else {
			repackMonoBitsToPatternGrid(mono[i], w, h, patternW, patternH, &layer)
		}
		var lenBuf [4]byte
		binary.BigEndian.PutUint32(lenBuf[:], uint32(layer.Len()))
		dst.Write(lenBuf[:])
		dst.Write(layer.Bytes())
	}
}

func monoBitAt(src []byte, idx int) bool {
	byteIdx := idx >> 3
	shift := 7 - uint(idx&7)
	return ((src[byteIdx] >> shift) & 1) != 0
}

func monoBitSet(dst []byte, idx int) {
	byteIdx := idx >> 3
	shift := 7 - uint(idx&7)
	dst[byteIdx] |= 1 << shift
}

func encodeRawIndexTree(values []int, lo, hi int, dst *bytes.Buffer) {
	if len(values) == 0 || hi-lo <= 1 {
		return
	}
	mid := lo + (hi-lo)/2
	plane := make([]byte, (len(values)+7)>>3)
	left := make([]int, 0, len(values))
	right := make([]int, 0, len(values))
	for i, v := range values {
		if v >= mid {
			monoBitSet(plane, i)
			right = append(right, v)
		} else {
			left = append(left, v)
		}
	}
	dst.Write(plane)
	encodeRawIndexTree(left, lo, mid, dst)
	encodeRawIndexTree(right, mid, hi, dst)
}

func decodeRawIndexTree(data []byte, pos *int, count, lo, hi int) ([]int, error) {
	if count == 0 {
		return nil, nil
	}
	if hi-lo <= 1 {
		out := make([]int, count)
		for i := range out {
			out[i] = lo
		}
		return out, nil
	}
	planeLen := (count + 7) >> 3
	if len(data)-*pos < planeLen {
		return nil, fmt.Errorf("decode: truncated raw tree plane")
	}
	plane := data[*pos : *pos+planeLen]
	*pos += planeLen
	mid := lo + (hi-lo)/2
	rightCount := countMonoBitsPrefix(plane, count)
	leftCount := count - rightCount
	left, err := decodeRawIndexTree(data, pos, leftCount, lo, mid)
	if err != nil {
		return nil, err
	}
	right, err := decodeRawIndexTree(data, pos, rightCount, mid, hi)
	if err != nil {
		return nil, err
	}
	out := make([]int, count)
	leftPos, rightPos := 0, 0
	for i := 0; i < count; i++ {
		if monoBitAt(plane, i) {
			out[i] = right[rightPos]
			rightPos++
		} else {
			out[i] = left[leftPos]
			leftPos++
		}
	}
	return out, nil
}

func encodeRawIndexTreeFreq(values []int, symbols []byte, dst *bytes.Buffer) {
	if len(values) == 0 || len(symbols) <= 1 {
		return
	}
	leftSyms, rightSyms := splitSymbolsBalanced(symbols)
	rightSet := make(map[int]struct{}, len(rightSyms))
	for _, s := range rightSyms {
		rightSet[int(s)] = struct{}{}
	}
	plane := make([]byte, (len(values)+7)>>3)
	leftVals := make([]int, 0, len(values))
	rightVals := make([]int, 0, len(values))
	for i, v := range values {
		if _, ok := rightSet[v]; ok {
			monoBitSet(plane, i)
			rightVals = append(rightVals, v)
		} else {
			leftVals = append(leftVals, v)
		}
	}
	dst.Write(plane)
	encodeRawIndexTreeFreq(leftVals, leftSyms, dst)
	encodeRawIndexTreeFreq(rightVals, rightSyms, dst)
}

func decodeRawIndexTreeFreq(data []byte, pos *int, count int, symbols []byte) ([]int, error) {
	if count == 0 {
		return nil, nil
	}
	if len(symbols) <= 1 {
		out := make([]int, count)
		for i := range out {
			out[i] = int(symbols[0])
		}
		return out, nil
	}
	planeLen := (count + 7) >> 3
	if len(data)-*pos < planeLen {
		return nil, fmt.Errorf("decode: truncated raw treeadapt plane")
	}
	plane := data[*pos : *pos+planeLen]
	*pos += planeLen
	rightCount := countMonoBitsPrefix(plane, count)
	leftCount := count - rightCount
	leftSyms, rightSyms := splitSymbolsBalanced(symbols)
	left, err := decodeRawIndexTreeFreq(data, pos, leftCount, leftSyms)
	if err != nil {
		return nil, err
	}
	right, err := decodeRawIndexTreeFreq(data, pos, rightCount, rightSyms)
	if err != nil {
		return nil, err
	}
	out := make([]int, count)
	leftPos, rightPos := 0, 0
	for i := 0; i < count; i++ {
		if monoBitAt(plane, i) {
			out[i] = right[rightPos]
			rightPos++
		} else {
			out[i] = left[leftPos]
			leftPos++
		}
	}
	return out, nil
}

func splitSymbolsBalanced(symbols []byte) ([]byte, []byte) {
	if len(symbols) <= 1 {
		return append([]byte(nil), symbols...), nil
	}
	split := len(symbols) / 2
	if split <= 0 {
		split = 1
	}
	if split >= len(symbols) {
		split = len(symbols) - 1
	}
	left := append([]byte(nil), symbols[:split]...)
	right := append([]byte(nil), symbols[split:]...)
	return left, right
}

func countMonoBitsPrefix(src []byte, n int) int {
	c := 0
	for i := 0; i < n; i++ {
		if monoBitAt(src, i) {
			c++
		}
	}
	return c
}

func unpackMonoPatternGrid(data []byte, w, h, patternW, patternH int, low, high uint8) ([]uint8, error) {
	if len(data) < 10 {
		return nil, fmt.Errorf("decode: Y pattern grid stream too short")
	}
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	paletteSize := int(binary.BigEndian.Uint32(data[0:4]))
	blockBytes := int(data[4])
	indexBits := int(data[5])
	blockCount := int(binary.BigEndian.Uint32(data[6:10]))
	if paletteSize < 1 || blockBytes < 1 || indexBits < 1 {
		return nil, fmt.Errorf("decode: invalid Y pattern palette header")
	}
	paletteBytes := paletteSize * blockBytes
	if len(data) < 10+paletteBytes {
		return nil, fmt.Errorf("decode: truncated Y palette")
	}
	palette := data[10 : 10+paletteBytes]
	br := newBitReader(data[10+paletteBytes:])
	out := make([]uint8, w*h)
	if blockCount != blocksX*blocksY {
		return nil, fmt.Errorf("decode: unexpected pattern block count")
	}
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			idx, err := br.readBitsInt(indexBits)
			if err != nil {
				return nil, fmt.Errorf("decode: truncated Y pattern indices")
			}
			patIdx := idx
			if patIdx >= paletteSize {
				return nil, fmt.Errorf("decode: pattern index out of range")
			}
			block := palette[patIdx*blockBytes : (patIdx+1)*blockBytes]
			x0 := bx * patternW
			bitPos := 0
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				row := 0
				if y < h {
					row = y * w
				}
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					byteIdx := bitPos >> 3
					shift := 7 - uint(bitPos&7)
					bit := ((block[byteIdx] >> shift) & 1) != 0
					if x < w && y < h {
						v := low
						if bit {
							v = high
						}
						out[row+x] = v
					}
					bitPos++
				}
			}
		}
	}
	return out, nil
}

func unpackMonoPatternGridWithPalette(data []byte, w, h, patternW, patternH int, palette []byte, paletteSize, blockBytes, indexBits int, low, high uint8) ([]uint8, error) {
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	br := newBitReader(data)
	out := make([]uint8, w*h)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			idx, err := br.readBitsInt(indexBits)
			if err != nil {
				return nil, fmt.Errorf("decode: truncated global pattern indices")
			}
			if idx < 0 || idx >= paletteSize {
				return nil, fmt.Errorf("decode: global pattern index out of range")
			}
			block := palette[idx*blockBytes : (idx+1)*blockBytes]
			x0 := bx * patternW
			bitPos := 0
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				row := 0
				if y < h {
					row = y * w
				}
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					byteIdx := bitPos >> 3
					shift := 7 - uint(bitPos&7)
					bit := ((block[byteIdx] >> shift) & 1) != 0
					if x < w && y < h {
						v := low
						if bit {
							v = high
						}
						out[row+x] = v
					}
					bitPos++
				}
			}
		}
	}
	return out, nil
}

func unpackStatePatternGrid(data []byte, w, h, patternW, patternH, stateBits int) ([]uint8, error) {
	if len(data) < 10 {
		return nil, fmt.Errorf("decode: state pattern grid stream too short")
	}
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	paletteSize := int(binary.BigEndian.Uint32(data[0:4]))
	blockBytes := int(data[4])
	indexBits := int(data[5])
	blockCount := int(binary.BigEndian.Uint32(data[6:10]))
	if paletteSize < 1 || blockBytes < 1 || indexBits < 1 {
		return nil, fmt.Errorf("decode: invalid state pattern palette header")
	}
	paletteBytes := paletteSize * blockBytes
	if len(data) < 10+paletteBytes {
		return nil, fmt.Errorf("decode: truncated state pattern palette")
	}
	palette := data[10 : 10+paletteBytes]
	br := newBitReader(data[10+paletteBytes:])
	out := make([]uint8, w*h)
	if blockCount != blocksX*blocksY {
		return nil, fmt.Errorf("decode: unexpected state pattern block count")
	}
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			idx, err := br.readBitsInt(indexBits)
			if err != nil {
				return nil, fmt.Errorf("decode: truncated state pattern indices")
			}
			if idx < 0 || idx >= paletteSize {
				return nil, fmt.Errorf("decode: state pattern index out of range")
			}
			block := palette[idx*blockBytes : (idx+1)*blockBytes]
			x0 := bx * patternW
			sr := newBitReader(block)
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				for dx := 0; dx < patternW; dx++ {
					v, err := sr.readBits(uint8(stateBits))
					if err != nil {
						return nil, fmt.Errorf("decode: truncated state pattern block")
					}
					x := x0 + dx
					if x < w && y < h {
						out[y*w+x] = v
					}
				}
			}
		}
	}
	return out, nil
}

func unpackStatePatternGridDirect(data []byte, w, h, patternW, patternH, stateBits int) ([]uint8, error) {
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	br := newBitReader(data)
	out := make([]uint8, w*h)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			x0 := bx * patternW
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				for dx := 0; dx < patternW; dx++ {
					v, err := br.readBits(uint8(stateBits))
					if err != nil {
						return nil, fmt.Errorf("decode: truncated direct state pattern stream")
					}
					x := x0 + dx
					if x < w && y < h {
						out[y*w+x] = v
					}
				}
			}
		}
	}
	return out, nil
}

func unpackMonoPatternGridDirect(data []byte, w, h, patternW, patternH int, low, high uint8) ([]uint8, error) {
	blocksX := ceilDiv(w, patternW)
	blocksY := ceilDiv(h, patternH)
	br := newBitReader(data)
	out := make([]uint8, w*h)
	for by := 0; by < blocksY; by++ {
		y0 := by * patternH
		for bx := 0; bx < blocksX; bx++ {
			pattern, err := br.readBitsInt(patternW * patternH)
			if err != nil {
				return nil, fmt.Errorf("decode: truncated direct pattern stream")
			}
			x0 := bx * patternW
			bitPos := 0
			for dy := 0; dy < patternH; dy++ {
				y := y0 + dy
				row := 0
				if y < h {
					row = y * w
				}
				for dx := 0; dx < patternW; dx++ {
					x := x0 + dx
					if x < w && y < h {
						v := low
						if ((pattern >> bitPos) & 1) != 0 {
							v = high
						}
						out[row+x] = v
					}
					bitPos++
				}
			}
		}
	}
	return out, nil
}

func unpackPatternLayers(data []byte, w, h, bits, patternW, patternH int, low, high uint8) ([]uint8, error) {
	if len(data) < 1 {
		return nil, fmt.Errorf("decode: Y layered pattern stream too short")
	}
	if int(data[0]) != bits {
		return nil, fmt.Errorf("decode: unexpected layered pattern count")
	}
	pos := 1
	layers := make([][]uint8, bits)
	for i := 0; i < bits; i++ {
		n, err := readU32BE(data, &pos, "Y layered pattern length")
		if err != nil {
			return nil, err
		}
		if n > len(data)-pos {
			return nil, fmt.Errorf("decode: truncated Y layered pattern stream")
		}
		layerData := data[pos : pos+n]
		pos += n
		if patternW == 2 && patternH == 2 {
			layers[i], err = unpackMonoPatternGridDirect(layerData, w, h, patternW, patternH, 0, 255)
		} else {
			layers[i], err = unpackMonoPatternGrid(layerData, w, h, patternW, patternH, 0, 255)
		}
		if err != nil {
			return nil, err
		}
	}
	out := make([]uint8, w*h)
	levelCount := (1 << bits) - 1
	for i := range out {
		level := 0
		for bit := 0; bit < bits; bit++ {
			level <<= 1
			if layers[bit][i] != 0 {
				level |= 1
			}
		}
		out[i] = low + uint8((int(high-low)*level+levelCount/2)/levelCount)
	}
	return out, nil
}

func packedPlaneLenBits(w, h, bits int) int {
	n := w * h * bits
	return (n + 7) >> 3
}

func ceilDiv(a, b int) int {
	return (a + b - 1) / b
}

func bitsNeeded(v int) int {
	if v <= 0 {
		return 1
	}
	n := 0
	for v > 0 {
		n++
		v >>= 1
	}
	return n
}

func smoothBlocks(img *image.RGBA) *image.RGBA {
	return img
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func writeU32BE(w *bufio.Writer, v uint32) error {
	var buf [4]byte
	binary.BigEndian.PutUint32(buf[:], v)
	_, err := w.Write(buf[:])
	return err
}

func readU32BE(src []byte, pos *int, label string) (int, error) {
	if len(src)-*pos < 4 {
		return 0, fmt.Errorf("decode: truncated while reading %s", label)
	}
	v := binary.BigEndian.Uint32(src[*pos : *pos+4])
	*pos += 4
	return int(v), nil
}

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
