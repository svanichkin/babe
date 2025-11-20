package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"io"
	"runtime"

	"github.com/klauspost/compress/zstd"
)

func ValidateQuality(q int) int {
	if q < 1 {
		return 1
	}
	if q > 100 {
		return 100
	}
	return q
}

// ImageToRGBA copies any image.Image into an *image.RGBA with bounds starting at (0,0).
func ImageToRGBA(src image.Image) *image.RGBA {
	b := src.Bounds()
	dst := image.NewRGBA(image.Rect(0, 0, b.Dx(), b.Dy()))
	draw.Draw(dst, dst.Bounds(), src, b.Min, draw.Src)
	return dst
}

func WriteHeader(b *bytes.Buffer, quality int, w, h uint16) error {
	// Header: magic(4) + width(uint16) + height(uint16) + quality(uint8)
	if _, err := b.Write([]byte(magicCodec2)); err != nil {
		return err
	}

	if err := binary.Write(b, binary.BigEndian, w); err != nil {
		return err
	}
	if err := binary.Write(b, binary.BigEndian, h); err != nil {
		return err
	}
	if err := b.WriteByte(byte(quality)); err != nil {
		return err
	}
	return nil
}

func ReadHeader(r *bytes.Reader) (quality, w, h int, err error) {
	// Read header.
	magic := make([]byte, len(magicCodec2))
	if _, err = r.Read(magic); err != nil {
		return
	}
	if string(magic) != magicCodec2 {
		return 0, 0, 0, ErrInvalidMagic
	}

	var w16, h16 uint16
	if err = binary.Read(r, binary.BigEndian, &w16); err != nil {
		return
	}
	if err = binary.Read(r, binary.BigEndian, &h16); err != nil {
		return
	}
	qByte, err := r.ReadByte()
	if err != nil {
		return
	}
	quality = int(qByte)
	w, h = int(w16), int(h16)
	return
}

// CloseColor returns true if colors a and b are "close enough" (approximate match),
// но сравнение делается в пространстве YCoCg, а не в исходном RGB.
// tol интерпретируется как допуск по цветовым каналам (Co/Cg), а по яркости
// допускаем немного больший разброс.
func CloseColor(a, b color.RGBA, tol int) bool {
	ya, coa, cga := RgbToYCoCg(a)
	yb, cob, cgb := RgbToYCoCg(b)

	dY := Abs32(ya - yb)
	dCo := Abs32(coa - cob)
	dCg := Abs32(cga - cgb)

	// Яркость можно допускать чуть сильнее, чем цветовой сдвиг.
	yTol := int32(tol * 2)
	cTol := int32(tol)

	return dY <= yTol && dCo <= cTol && dCg <= cTol
}

// WriteBit writes a single bit.
func (bw *BitWriter) WriteBit(v bool) error {
	if v {
		bw.acc |= 1 << (7 - bw.nbit)
	}
	bw.nbit++
	if bw.nbit == 8 {
		if err := bw.buf.WriteByte(bw.acc); err != nil {
			return err
		}
		bw.acc = 0
		bw.nbit = 0
	}
	return nil
}

// WriteByte writes a full byte, respecting current bit alignment.
func (bw *BitWriter) WriteByte(b byte) error {
	// Если по байтовой границе, пишем напрямую.
	if bw.nbit == 0 {
		return bw.buf.WriteByte(b)
	}
	// Иначе — по битам.
	for i := 0; i < 8; i++ {
		bit := (b & (1 << (7 - i))) != 0
		if err := bw.WriteBit(bit); err != nil {
			return err
		}
	}
	return nil
}

// Flush дописывает хвостовой байт, если нужно.
func (bw *BitWriter) Flush() error {
	if bw.nbit == 0 {
		return nil
	}
	if err := bw.buf.WriteByte(bw.acc); err != nil {
		return err
	}
	bw.acc = 0
	bw.nbit = 0
	return nil
}

// ReadBit читает один бит.
func (br *BitReader) ReadBit() (bool, error) {
	if br.nbit == 0 {
		if br.pos >= len(br.data) {
			return false, io.EOF
		}
		br.acc = br.data[br.pos]
		br.pos++
	}
	bit := (br.acc & (1 << (7 - br.nbit))) != 0
	br.nbit++
	if br.nbit == 8 {
		br.nbit = 0
	}
	return bit, nil
}

// ReadByte читает байт, учитывая текущее битовое выравнивание.
func (br *BitReader) ReadByte() (byte, error) {
	if br.nbit == 0 {
		if br.pos >= len(br.data) {
			return 0, io.EOF
		}
		b := br.data[br.pos]
		br.pos++
		return b, nil
	}
	// Не по границе — собираем по битам.
	var b byte
	for i := 0; i < 8; i++ {
		bit, err := br.ReadBit()
		if err != nil {
			return 0, err
		}
		if bit {
			b |= 1 << (7 - i)
		}
	}
	return b, nil
}

func SmoothEdges(src *image.RGBA, tol int) *image.RGBA {
	b := src.Bounds()
	dst := image.NewRGBA(b)

	for y := b.Min.Y; y < b.Max.Y; y++ {
		for x := b.Min.X; x < b.Max.X; x++ {
			c := src.RGBAAt(x, y)
			lc := Luma(c)

			type pt struct{ x, y int }
			neigh := []pt{
				{x - 1, y},
				{x + 1, y},
				{x, y - 1},
				{x, y + 1},
				{x - 1, y - 1},
				{x + 1, y - 1},
				{x - 1, y + 1},
				{x + 1, y + 1},
			}

			// Проверяем, есть ли сильный перепад яркости с соседями
			// и одновременно измеряем максимальный перепад.
			edge := false
			maxDl := 0
			for _, p := range neigh {
				if p.x < b.Min.X || p.x >= b.Max.X || p.y < b.Min.Y || p.y >= b.Max.Y {
					continue
				}
				nc := src.RGBAAt(p.x, p.y)
				ln := Luma(nc)
				dl := int(ln - lc)
				if dl < 0 {
					dl = -dl
				}
				if dl > maxDl {
					maxDl = dl
				}
				if dl > tol*2 {
					edge = true
				}
			}

			if !edge {
				// Нет сильного перепада — оставляем как есть.
				dst.SetRGBA(x, y, c)
				continue
			}

			// На границе: сила сглаживания зависит от того,
			// насколько сильно отличаются цвета.
			// - если maxDl небольшой (цвета близки) — сглаживаем сильнее
			// - если maxDl большой (резкая граница) — сглаживаем слабее

			// Считаем средний цвет соседей.
			var sumNR, sumNG, sumNB, nCount int
			for _, p := range neigh {
				if p.x < b.Min.X || p.x >= b.Max.X || p.y < b.Min.Y || p.y >= b.Max.Y {
					continue
				}
				nc := src.RGBAAt(p.x, p.y)
				sumNR += int(nc.R)
				sumNG += int(nc.G)
				sumNB += int(nc.B)
				nCount++
			}

			if nCount == 0 {
				// На краю изображения — просто оставляем исходный.
				dst.SetRGBA(x, y, c)
				continue
			}

			avgNR := float64(sumNR) / float64(nCount)
			avgNG := float64(sumNG) / float64(nCount)
			avgNB := float64(sumNB) / float64(nCount)

			// Вычисляем коэффициент сглаживания alpha в [0..1].
			// Базируемся на maxDl:
			// - небольшие перепады -> среднее сглаживание
			// - средние/большие перепады -> более сильное сглаживание, чтобы убрать "зубья"
			var alpha float64
			// switch {
			// case maxDl < tol*2:
			// 	// слабая граница — лёгкое сглаживание
			// 	alpha = 0.4
			// case maxDl < tol*4:
			// 	// нормальная граница — заметное сглаживание
			// 	alpha = 0.6
			// default:
			// 	// очень контрастная граница — тоже сглаживаем ощутимо, но не до мыла
			// 	alpha = 0.5
			// }
			switch {
			case maxDl < tol*3:
				// слабая граница — лёгкое сглаживание
				alpha = 0.4
			case maxDl < tol*5:
				// нормальная граница — заметное сглаживание
				alpha = 0.3
			default:
				// очень контрастная граница — тоже сглаживаем ощутимо, но не до мыла
				alpha = 0.1
			}

			// Итоговый цвет: смесь исходного и среднего по соседям.
			outR := float64(c.R)*(1-alpha) + avgNR*alpha
			outG := float64(c.G)*(1-alpha) + avgNG*alpha
			outB := float64(c.B)*(1-alpha) + avgNB*alpha

			avg := color.RGBA{
				R: uint8(outR + 0.5),
				G: uint8(outG + 0.5),
				B: uint8(outB + 0.5),
				A: 255,
			}
			dst.SetRGBA(x, y, avg)
		}
	}

	return dst
}

func NewBitWriter(buf *bytes.Buffer) *BitWriter {
	return &BitWriter{buf: buf}
}

// NewBitReaderFromReader читает остаток r в память и создаёт BitReader.
func NewBitReaderFromReader(r *bytes.Reader) *BitReader {
	rest, _ := io.ReadAll(r)
	return &BitReader{data: rest}
}

// NewBitReaderFromBytes создаёт BitReader из уже прочитанных данных.
func NewBitReaderFromBytes(b []byte) *BitReader {
	return &BitReader{data: b}
}

func EncodeZstd(b io.Writer, raw *bytes.Buffer) error {
	enc, err := zstd.NewWriter(b, zstd.WithEncoderConcurrency(runtime.NumCPU()))
	if err != nil {
		return err
	}
	if _, err := enc.Write(raw.Bytes()); err != nil {
		enc.Close()
		return err
	}
	if err := enc.Close(); err != nil {
		return err
	}
	return nil
}

func DecodeZstd(r io.Reader) ([]byte, error) {
	dec, err := zstd.NewReader(r)
	if err != nil {
		return nil, err
	}
	defer dec.Close()

	plain, err := io.ReadAll(dec)
	if err != nil {
		return nil, err
	}

	if len(plain) < 2 {
		return nil, fmt.Errorf("codec2: truncated payload (no palette)")
	}

	return plain, nil
}

// Luma returns integer Luma (0..255) for an RGBA pixel.
func Luma(c color.RGBA) int32 {
	// Rec. 601-type weights.
	return (299*int32(c.R) + 587*int32(c.G) + 114*int32(c.B) + 500) / 1000
}

// RgbToYCoCg converts an RGBA color into integer YCoCg components.
// This is a reversible transform in theory; здесь нам важны только относительные расстояния.
func RgbToYCoCg(c color.RGBA) (y, co, cg int32) {
	r := int32(c.R)
	g := int32(c.G)
	b := int32(c.B)

	co = r - b
	tmp := b + co/2
	cg = g - tmp
	y = tmp + cg/2
	return y, co, cg
}

func Abs32(v int32) int32 {
	if v < 0 {
		return -v
	}
	return v
}
