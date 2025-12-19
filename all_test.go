package main

import (
	"bytes"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/jpeg"
	_ "image/jpeg"
	"os"
	"runtime"
	"testing"
	"time"

	"github.com/xfmoulet/qoi"
)

// -----------------------------
// Unit tests
// -----------------------------

func makeTestImage(w, h int) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			img.SetRGBA(x, y, color.RGBA{
				R: uint8((x * 17) ^ (y * 31)),
				G: uint8((x * 43) + (y * 13)),
				B: uint8((x * 7) ^ (y * 11)),
				A: 255,
			})
		}
	}
	return img
}

func TestEncodeDecode_RoundTrip(t *testing.T) {
	src := makeTestImage(64, 48)

	for _, tc := range []struct {
		name       string
		quality    int
		bw         bool
		postfilter bool
	}{
		{name: "color_no_postfilter", quality: 70, bw: false, postfilter: false},
		{name: "color_postfilter", quality: 70, bw: false, postfilter: true},
		{name: "color_no_postfilter_q80", quality: 80, bw: false, postfilter: false},
		{name: "bw_no_postfilter", quality: 70, bw: true, postfilter: false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			comp, err := Encode(src, tc.quality, tc.bw)
			if err != nil {
				t.Fatalf("Encode: %v", err)
			}
			if len(comp) == 0 {
				t.Fatalf("Encode returned empty payload")
			}

			dec, err := Decode(comp, tc.postfilter)
			if err != nil {
				t.Fatalf("Decode: %v", err)
			}
			if dec == nil {
				t.Fatalf("Decode returned nil image")
			}
			if got, want := dec.Bounds(), src.Bounds(); got != want {
				t.Fatalf("bounds mismatch: got %v want %v", got, want)
			}
		})
	}
}

func TestEncode_ImageTooSmall(t *testing.T) {
	// At quality=0, smallBlock becomes 4, so a 1x1 image cannot be encoded.
	img := makeTestImage(1, 1)
	if _, err := Encode(img, 0, false); err == nil {
		t.Fatalf("expected error for too-small image, got nil")
	}
}

// -----------------------------
// Benchmark helpers
// -----------------------------

func loadTestImage(t testing.TB) image.Image {
	t.Helper()
	f, err := os.Open("benchmark.jpg")
	if err != nil {
		t.Skip("benchmark image missing: expected benchmark.jpg")
		return nil
	}
	defer f.Close()

	img, _, err := image.Decode(f)
	if err != nil {
		t.Fatalf("failed to decode benchmark image: %v", err)
	}
	return toRGBA(img)
}

func toRGBA(src image.Image) *image.RGBA {
	b := src.Bounds()
	dst := image.NewRGBA(image.Rect(0, 0, b.Dx(), b.Dy()))
	draw.Draw(dst, dst.Bounds(), src, b.Min, draw.Src)
	return dst
}

func benchmarkEncodeDecode(b *testing.B, encode func() ([]byte, error), decode func([]byte) error) {
	// Warm-up outside timed section.
	enc, err := encode()
	if err != nil {
		b.Fatalf("encode failed: %v", err)
	}
	if err := decode(enc); err != nil {
		b.Fatalf("decode failed: %v", err)
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		enc, err := encode()
		if err != nil {
			b.Fatalf("encode failed: %v", err)
		}
		if err := decode(enc); err != nil {
			b.Fatalf("decode failed: %v", err)
		}
	}
}

// BenchmarkCodecs is the "fair" comparative benchmark:
// - identical loop shape per codec: encode(); decode()
// - warm-up before timing
// - optional sample lines under -v (outside timing)
// - all codecs reuse their state between iterations:
//   - JPEG/QOI reuse `bytes.Buffer`/`bytes.Reader` via Reset()
//   - BABE reuses `*Encoder`/`*Decoder` internal scratch buffers
func BenchmarkCodecs(b *testing.B) {
	img := loadTestImage(b)

	b.Run("JPEG", func(b *testing.B) {
		var buf bytes.Buffer
		var r bytes.Reader

		if testing.Verbose() {
			b.Logf("cpus=%d gomaxprocs=%d goroutines=%d", runtime.NumCPU(), runtime.GOMAXPROCS(0), runtime.NumGoroutine())

			buf.Reset()
			startEnc := time.Now()
			if err := jpeg.Encode(&buf, img, &jpeg.Options{Quality: 80}); err != nil {
				b.Fatalf("jpeg encode failed: %v", err)
			}
			encTime := time.Since(startEnc)
			enc := buf.Bytes()

			r.Reset(enc)
			startDec := time.Now()
			if _, err := jpeg.Decode(&r); err != nil {
				b.Fatalf("jpeg decode failed: %v", err)
			}
			decTime := time.Since(startDec)

			b.Logf("encode=%v decode=%v size=%d bytes", encTime, decTime, len(enc))
		}

		benchmarkEncodeDecode(b,
			func() ([]byte, error) {
				buf.Reset()
				if err := jpeg.Encode(&buf, img, &jpeg.Options{Quality: 80}); err != nil {
					return nil, err
				}
				return buf.Bytes(), nil
			},
			func(enc []byte) error {
				r.Reset(enc)
				_, err := jpeg.Decode(&r)
				return err
			},
		)
	})

	b.Run("BABE", func(b *testing.B) {
		enc := NewEncoder()
		dec := NewDecoder()

		if testing.Verbose() {
			b.Logf("cpus=%d gomaxprocs=%d goroutines=%d", runtime.NumCPU(), runtime.GOMAXPROCS(0), runtime.NumGoroutine())

			startEnc := time.Now()
			encBytes, err := enc.Encode(img, 80, false)
			if err != nil {
				b.Fatalf("encode failed: %v", err)
			}
			encTime := time.Since(startEnc)

			startDec := time.Now()
			if _, err := dec.Decode(encBytes, false); err != nil {
				b.Fatalf("decode failed: %v", err)
			}
			decTime := time.Since(startDec)

			b.Logf("encode=%v decode=%v size=%d bytes", encTime, decTime, len(encBytes))
		}

		benchmarkEncodeDecode(b,
			func() ([]byte, error) { return enc.Encode(img, 80, false) },
			func(encBytes []byte) error {
				_, err := dec.Decode(encBytes, false)
				return err
			},
		)
	})

	b.Run("QOI", func(b *testing.B) {
		var buf bytes.Buffer
		var r bytes.Reader

		if testing.Verbose() {
			b.Logf("cpus=%d gomaxprocs=%d goroutines=%d", runtime.NumCPU(), runtime.GOMAXPROCS(0), runtime.NumGoroutine())

			buf.Reset()
			startEnc := time.Now()
			if err := qoi.Encode(&buf, img); err != nil {
				b.Fatalf("qoi encode failed: %v", err)
			}
			encTime := time.Since(startEnc)
			enc := buf.Bytes()

			r.Reset(enc)
			startDec := time.Now()
			if _, err := qoi.Decode(&r); err != nil {
				b.Fatalf("qoi decode failed: %v", err)
			}
			decTime := time.Since(startDec)

			b.Logf("encode=%v decode=%v size=%d bytes", encTime, decTime, len(enc))
		}

		benchmarkEncodeDecode(b,
			func() ([]byte, error) {
				buf.Reset()
				if err := qoi.Encode(&buf, img); err != nil {
					return nil, err
				}
				return buf.Bytes(), nil
			},
			func(enc []byte) error {
				r.Reset(enc)
				_, err := qoi.Decode(&r)
				return err
			},
		)
	})
}

// -----------------------------
// Summary table (single output)
// -----------------------------

type summaryRow struct {
	name   string
	result testing.BenchmarkResult
	sizeB  int
	encNS  int64
	decNS  int64
}

type summaryBenchFn func(*testing.B) (sizeB int, encTotal, decTotal time.Duration)

// Run with:
//
//	GOCACHE="$PWD/.gocache" go test -run TestBenchmarkSummary -v
func TestBenchmarkSummary(t *testing.T) {
	img := loadTestImage(t)

	rows := []summaryRow{
		runSummaryBench("JPEG", benchJPEG(img)),
		runSummaryBench("BABE", benchBABE(img)),
		runSummaryBench("QOI", benchQOI(img)),
	}

	fmt.Println()
	fmt.Printf("%-6s  %10s  %10s  %12s  %12s  %9s  %10s\n", "codec", "enc_ms", "dec_ms", "ns/op", "B/op", "allocs/op", "size(B)")
	fmt.Printf("%-6s  %10s  %10s  %12s  %12s  %9s  %10s\n", "------", "----------", "----------", "------------", "------------", "---------", "----------")
	for _, r := range rows {
		encMS := float64(r.encNS) / 1e6
		decMS := float64(r.decNS) / 1e6
		fmt.Printf("%-6s  %10.3f  %10.3f  %12d  %12d  %9d  %10d\n",
			r.name,
			encMS,
			decMS,
			r.result.NsPerOp(),
			r.result.AllocedBytesPerOp(),
			r.result.AllocsPerOp(),
			r.sizeB,
		)
	}
}

func runSummaryBench(name string, fn summaryBenchFn) summaryRow {
	sizeB, encTotal, decTotal := 0, time.Duration(0), time.Duration(0)
	res := testing.Benchmark(func(b *testing.B) {
		sizeB, encTotal, decTotal = fn(b)
	})

	encNS := int64(0)
	decNS := int64(0)
	if res.N > 0 {
		encNS = encTotal.Nanoseconds() / int64(res.N)
		decNS = decTotal.Nanoseconds() / int64(res.N)
	}
	return summaryRow{name: name, result: res, sizeB: sizeB, encNS: encNS, decNS: decNS}
}

func benchJPEG(img image.Image) summaryBenchFn {
	return func(b *testing.B) (int, time.Duration, time.Duration) {
		var buf bytes.Buffer
		var r bytes.Reader
		sizeB := 0
		var encTotal, decTotal time.Duration

		// Warm-up and reset so one-time allocations don't dominate the summary.
		buf.Reset()
		if err := jpeg.Encode(&buf, img, &jpeg.Options{Quality: 80}); err != nil {
			b.Fatalf("jpeg encode failed: %v", err)
		}
		enc := buf.Bytes()
		r.Reset(enc)
		if _, err := jpeg.Decode(&r); err != nil {
			b.Fatalf("jpeg decode failed: %v", err)
		}
		b.ResetTimer()

		for i := 0; i < b.N; i++ {
			buf.Reset()
			startEnc := time.Now()
			if err := jpeg.Encode(&buf, img, &jpeg.Options{Quality: 80}); err != nil {
				b.Fatalf("jpeg encode failed: %v", err)
			}
			encTotal += time.Since(startEnc)

			enc := buf.Bytes()
			sizeB = len(enc)

			r.Reset(enc)
			startDec := time.Now()
			if _, err := jpeg.Decode(&r); err != nil {
				b.Fatalf("jpeg decode failed: %v", err)
			}
			decTotal += time.Since(startDec)
		}
		return sizeB, encTotal, decTotal
	}
}

func benchBABE(img image.Image) summaryBenchFn {
	enc := NewEncoder()
	dec := NewDecoder()
	return func(b *testing.B) (int, time.Duration, time.Duration) {
		sizeB := 0
		var encTotal, decTotal time.Duration

		// Warm-up and reset so one-time allocations don't dominate the summary.
		encBytes, err := enc.Encode(img, 80, false)
		if err != nil {
			b.Fatalf("encode failed: %v", err)
		}
		if _, err := dec.Decode(encBytes, false); err != nil {
			b.Fatalf("decode failed: %v", err)
		}
		b.ResetTimer()

		for i := 0; i < b.N; i++ {
			startEnc := time.Now()
			encBytes, err := enc.Encode(img, 80, false)
			if err != nil {
				b.Fatalf("encode failed: %v", err)
			}
			encTotal += time.Since(startEnc)
			sizeB = len(encBytes)

			startDec := time.Now()
			if _, err := dec.Decode(encBytes, false); err != nil {
				b.Fatalf("decode failed: %v", err)
			}
			decTotal += time.Since(startDec)
		}
		return sizeB, encTotal, decTotal
	}
}

func benchQOI(img image.Image) summaryBenchFn {
	return func(b *testing.B) (int, time.Duration, time.Duration) {
		var buf bytes.Buffer
		var r bytes.Reader
		sizeB := 0
		var encTotal, decTotal time.Duration

		// Warm-up and reset so one-time allocations don't dominate the summary.
		buf.Reset()
		if err := qoi.Encode(&buf, img); err != nil {
			b.Fatalf("qoi encode failed: %v", err)
		}
		enc := buf.Bytes()
		r.Reset(enc)
		if _, err := qoi.Decode(&r); err != nil {
			b.Fatalf("qoi decode failed: %v", err)
		}
		b.ResetTimer()

		for i := 0; i < b.N; i++ {
			buf.Reset()
			startEnc := time.Now()
			if err := qoi.Encode(&buf, img); err != nil {
				b.Fatalf("qoi encode failed: %v", err)
			}
			encTotal += time.Since(startEnc)

			enc := buf.Bytes()
			sizeB = len(enc)

			r.Reset(enc)
			startDec := time.Now()
			if _, err := qoi.Decode(&r); err != nil {
				b.Fatalf("qoi decode failed: %v", err)
			}
			decTotal += time.Since(startDec)
		}
		return sizeB, encTotal, decTotal
	}
}
