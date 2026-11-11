package classic

import (
	"bytes"
	"fmt"
	"image"
	"image/color"
	_ "image/jpeg"
	"math/rand"
	"os"
	"testing"
)

func TestByteRangeRoundTrip(t *testing.T) {
	rng := rand.New(rand.NewSource(123))
	for _, n := range []int{0, 1, 2, 31, 256, 4096, byteRangeBlock - 1, byteRangeBlock, byteRangeBlock + 1, byteRangeBlock*3 + 123} {
		for kind := 0; kind < 5; kind++ {
			src := make([]byte, n)
			for i := range src {
				switch kind {
				case 0:
					src[i] = 0
				case 1:
					src[i] = 255
				case 2:
					src[i] = byte(rng.Intn(256))
				case 3:
					src[i] = byte(rng.Intn(7))
				case 4:
					if i%201 == 0 {
						src[i] = byte(rng.Intn(256))
					}
				}
			}
			packed := encodeByteRangeInto(nil, src)
			out := make([]byte, n)
			if err := decodeByteRangeInto(out, packed); err != nil || !bytes.Equal(src, out) {
				t.Fatalf("n=%d kind=%d: %v", n, kind, err)
			}
			if len(packed) > 0 {
				if err := decodeByteRangeInto(out, packed[:len(packed)-1]); err == nil {
					t.Fatal("accepted truncation")
				}
			}
			if err := decodeByteRangeInto(out, append(bytes.Clone(packed), 0)); err == nil {
				t.Fatal("accepted trailing data")
			}
		}
	}
}

func TestQuadMeanExact(t *testing.T) {
	for count := uint32(1); count <= 4; count++ {
		for sum := uint64(0); sum <= uint64(count)*255; sum++ {
			if got, want := quadMean(sum, count), uint8(sum/uint64(count)); got != want {
				t.Fatalf("mean %d/%d: %d != %d", sum, count, got, want)
			}
		}
	}
	for _, step := range []int{1, 2, 4, 8, 16, 32} {
		for v := 0; v < 256; v++ {
			want := uint8(min(255, (v+step/2)/step*step))
			if quantizeLevel(uint8(v), step) != want {
				t.Fatal("quantization changed")
			}
		}
	}
}

func TestQuadLevelsMatchBlockEncoder(t *testing.T) {
	rng := rand.New(rand.NewSource(437))
	var buf bytes.Buffer
	for i := 0; i < 100000; i++ {
		var samples [4]byte
		for j := range samples {
			samples[j] = byte(rng.Intn(256))
			if i < 256 {
				samples[j] = byte(i)
			}
		}
		for _, step := range []int{1, 2} {
			buf.Reset()
			pw := newBitWriter(&buf)
			wantFG, wantBG, wantPattern, err := encodeBlockPlane(samples[:], 2, 2, 0, 0, 2, 2, step, &pw)
			pw.flush()
			fg, bg, bits := quadLevels(samples[0], samples[1], samples[2], samples[3], step)
			if err != nil || fg != wantFG || bg != wantBG || (fg != bg) != wantPattern {
				t.Fatalf("samples %v step %d: (%d,%d) != (%d,%d): %v", samples, step, fg, bg, wantFG, wantBG, err)
			}
			if wantPattern && byte(bits<<4) != buf.Bytes()[0] {
				t.Fatal("quad pattern changed")
			}
		}
	}
}

func FuzzByteRangeDecode(f *testing.F) {
	for _, n := range []int{1, 512, byteRangeBlock + 1} {
		for _, alphabet := range []int{1, 7, 256} {
			src := make([]byte, n)
			for i := range src {
				src[i] = byte(i % alphabet)
			}
			f.Add(encodeByteRangeInto(nil, src), uint16(n))
		}
	}
	f.Fuzz(func(t *testing.T, data []byte, n uint16) {
		out := make([]byte, int(n))
		if err := decodeByteRangeInto(out, data); err == nil {
			// Successfully decoded inputs must have a stable canonical round trip.
			encoded := encodeByteRangeInto(nil, out)
			got := make([]byte, len(out))
			if err := decodeByteRangeInto(got, encoded); err != nil || !bytes.Equal(out, got) {
				t.Fatalf("unstable round trip: %v", err)
			}
		}
	})
}

func TestQuadPlanesMatchOriginalDecoder(t *testing.T) {
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	for _, dims := range []image.Point{{1, 1}, {2, 3}, {17, 19}, {256, 129}} {
		img := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		for y := 0; y < dims.Y; y++ {
			for x := 0; x < dims.X; x++ {
				img.SetRGBA(x, y, color.RGBA{byte(x*7 + y*3), byte(x*3 + y*5), byte(x ^ y), 255})
			}
		}
		for _, q := range []int{80, 99} {
			for _, bw := range []bool{false, true} {
				encoded, err := e.Encode(img, q, bw)
				if err != nil {
					t.Fatal(err)
				}
				raw := e.raw.Bytes()
				pos := payloadHeaderSize
				want := image.NewRGBA(img.Bounds())
				channels := 3
				if bw {
					channels = 1
				}
				for c := 0; c < channels; c++ {
					segment, err := readChannelSegment(raw, &pos)
					if err != nil {
						t.Fatal(err)
					}
					if err = decodeChannelToPix(segment, dims.X, dims.Y, want.Pix, want.Stride, c); err != nil {
						t.Fatal(err)
					}
				}
				ycbcrToRGB(want.Pix, want.Stride, dims.X, 0, dims.Y, !bw, !bw)
				got, err := d.Decode(encoded, false)
				if err != nil || !bytes.Equal(want.Pix, got.Pix) {
					t.Fatalf("%v q%d bw%v: pixels differ, %v", dims, q, bw, err)
				}
			}
		}
	}
}

// The same real BABE streams, quality and decoded bytes are used for both
// implementations. Input preparation and buffer allocation are outside timing.
func BenchmarkRangeStreams(b *testing.B) {
	for _, q := range []int{10, 80} {
		f, err := os.Open("../3.jpg")
		if err != nil {
			b.Skip("3.jpg is not present")
		}
		img, _, err := image.Decode(f)
		f.Close()
		if err != nil {
			b.Fatal(err)
		}
		e := NewEncoder()
		if _, err = e.Encode(img, q, false); err != nil {
			b.Fatal(err)
		}
		r := payloadReader{data: e.raw.Bytes(), pos: payloadHeaderSize}
		var streams [][]byte
		total := 0
		for c := 0; c < 3; c++ {
			r.u32()
			for s := 0; s < 5; s++ {
				n, _ := r.u32()
				src, _ := r.take(n)
				if n > 0 {
					streams = append(streams, src)
					total += n
				}
			}
		}
		for _, method := range []string{"binary", "byte"} {
			packed := make([][]byte, len(streams))
			out := make([][]byte, len(streams))
			size := 0
			for i, src := range streams {
				out[i] = make([]byte, len(src))
				if method == "binary" {
					packed[i] = encodeRangeInto(nil, src, 0)
					err = decodeRangeInto(out[i], packed[i], 0)
				} else {
					packed[i] = encodeByteRangeInto(nil, src)
					err = decodeByteRangeInto(out[i], packed[i])
				}
				if err != nil || !bytes.Equal(src, out[i]) {
					b.Fatalf("range round trip: %v", err)
				}
				size += min(len(src), len(packed[i]))
			}
			b.Run(fmt.Sprintf("Q%d/%s/Encode", q, method), func(b *testing.B) {
				b.SetBytes(int64(total))
				b.ReportAllocs()
				for j := 0; j < b.N; j++ {
					for i, src := range streams {
						if method == "binary" {
							packed[i] = encodeRangeInto(packed[i], src, 0)
						} else {
							packed[i] = encodeByteRangeInto(packed[i], src)
						}
					}
				}
				b.ReportMetric(float64(size), "stored-bytes")
			})
			b.Run(fmt.Sprintf("Q%d/%s/Decode", q, method), func(b *testing.B) {
				b.SetBytes(int64(total))
				b.ReportAllocs()
				for j := 0; j < b.N; j++ {
					for i, src := range packed {
						var err error
						if method == "binary" {
							err = decodeRangeInto(out[i], src, 0)
						} else {
							err = decodeByteRangeInto(out[i], src)
						}
						if err != nil {
							b.Fatal(err)
						}
					}
				}
			})
			for i, src := range streams {
				if !bytes.Equal(src, out[i]) {
					b.Fatal("range round trip mismatch")
				}
			}
		}
	}
}
