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

func TestDefaultLevels(t *testing.T) {
	for _, quality := range []int{0, 15, 30, 45, 60, 75, 90, 100} {
		enc := NewEncoder()
		img := makeTestImage(8, 8)
		if _, err := enc.Encode(img, quality, false); err != nil {
			t.Fatalf("Encode(quality=%d): %v", quality, err)
		}
		if len(enc.levels) != 2 || enc.levels[0] != 1 || enc.levels[1] != 2 {
			t.Fatalf("quality %d: got levels %v want [1 2]", quality, enc.levels)
		}
	}
}

// spread factor overrides removed

func TestAllowedMacroSpreadForQuality_Monotonic(t *testing.T) {
	prev := allowedMacroSpreadForQuality(0)
	if prev != 64 {
		t.Fatalf("quality 0 spread = %d, want 64", prev)
	}

	for q := 1; q <= 100; q++ {
		cur := allowedMacroSpreadForQuality(q)
		if cur > prev {
			t.Fatalf("spread increased at quality %d: prev=%d cur=%d", q, prev, cur)
		}
		prev = cur
	}

	if got := allowedMacroSpreadForQuality(100); got != 8 {
		t.Fatalf("quality 100 spread = %d, want 8", got)
	}
}

func TestEncodeDecode_RoundTrip(t *testing.T) {
	src := makeTestImage(64, 48)

	for _, tc := range []struct {
		name    string
		quality int
		bw      bool
	}{
		{name: "color", quality: 70, bw: false},
		{name: "color_q80", quality: 80, bw: false},
		{name: "bw", quality: 70, bw: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			comp, err := Encode(src, tc.quality, tc.bw)
			if err != nil {
				t.Fatalf("Encode: %v", err)
			}
			if len(comp) == 0 {
				t.Fatalf("Encode returned empty payload")
			}

			dec, err := NewDecoder().Decode(comp)
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

func TestEncodeDecode_RoundTrip_BackgroundTile(t *testing.T) {
	src := makeTestImage(64, 48)

	enc := NewEncoder()
	enc.backgroundTile = 10
	comp, err := enc.Encode(src, 70, false)
	if err != nil {
		t.Fatalf("Encode: %v", err)
	}
	if len(comp) == 0 {
		t.Fatalf("Encode returned empty payload")
	}

	dec, err := NewDecoder().Decode(comp)
	if err != nil {
		t.Fatalf("Decode: %v", err)
	}
	if got, want := dec.Bounds(), src.Bounds(); got != want {
		t.Fatalf("bounds mismatch: got %v want %v", got, want)
	}

	imgW, imgH, yPlane, _, cbPlane, _, crPlane, _, hasCb, hasCr, err := DecodeLayers(comp)
	if err != nil {
		t.Fatalf("DecodeLayers: %v", err)
	}
	if imgW != src.Bounds().Dx() || imgH != src.Bounds().Dy() {
		t.Fatalf("layer bounds mismatch: got %dx%d want %dx%d", imgW, imgH, src.Bounds().Dx(), src.Bounds().Dy())
	}
	if len(yPlane) != imgW*imgH {
		t.Fatalf("Y plane len = %d, want %d", len(yPlane), imgW*imgH)
	}
	if !hasCb || !hasCr {
		t.Fatalf("expected chroma planes to be present")
	}
	if len(cbPlane) != imgW*imgH || len(crPlane) != imgW*imgH {
		t.Fatalf("chroma plane lengths = %d/%d, want %d", len(cbPlane), len(crPlane), imgW*imgH)
	}
}

func TestEncode_ImageTooSmall(t *testing.T) {
	// Edge clipping now allows encoding even when the image is smaller than the nominal block size.
	img := makeTestImage(1, 1)
	if _, err := Encode(img, 0, false); err != nil {
		t.Fatalf("unexpected error for clipped edge block: %v", err)
	}
}

func TestEncodeDecode_RoundTrip_BlockRange(t *testing.T) {
	src := makeTestImage(64, 64)

	levels, err := blocksFromSpec("2-64")
	if err != nil {
		t.Fatalf("blocksFromSpec: %v", err)
	}
	enc := NewEncoder()
	enc.levels = levels
	comp, err := enc.Encode(src, 70, false)
	if err != nil {
		t.Fatalf("Encode: %v", err)
	}

	dec, err := NewDecoder().Decode(comp)
	if err != nil {
		t.Fatalf("Decode: %v", err)
	}
	if got, want := dec.Bounds(), src.Bounds(); got != want {
		t.Fatalf("bounds mismatch: got %v want %v", got, want)
	}
}

func TestEncodeDecode_RoundTrip_SingleBlockLevel(t *testing.T) {
	src := makeTestImage(64, 64)

	levels, err := blocksFromSpec("16-16")
	if err != nil {
		t.Fatalf("blocksFromSpec: %v", err)
	}
	enc := NewEncoder()
	enc.levels = levels
	comp, err := enc.Encode(src, 0, true)
	if err != nil {
		t.Fatalf("Encode: %v", err)
	}

	dec, err := NewDecoder().Decode(comp)
	if err != nil {
		t.Fatalf("Decode: %v", err)
	}
	if got, want := dec.Bounds(), src.Bounds(); got != want {
		t.Fatalf("bounds mismatch: got %v want %v", got, want)
	}
}

func TestPatternSetBasic_Order(t *testing.T) {
	rows := basicPatternTemplates()
	book := fixedPatternCodebook(8, 8, len(rows))
	if len(book) != len(rows) {
		t.Fatalf("unexpected codebook size: got %d want %d", len(book), len(rows))
	}

	expect := make([]patternMask, 0, len(rows))
	for _, patternRows := range rows {
		expect = append(expect, buildPatternMask(8, 8, func(x, y int) bool {
			return patternRows[y][x] == '1'
		}))
	}
	for i := range expect {
		if patternMaskKey(book[i]) != patternMaskKey(expect[i]) {
			t.Fatalf("pattern %d mismatch: got %s want %s", i, formatPatternBits(book[i], 8, 8), formatPatternBits(expect[i], 8, 8))
		}
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

func loadTestImagePrefer(t testing.TB, paths ...string) image.Image {
	t.Helper()
	for _, path := range paths {
		f, err := os.Open(path)
		if err != nil {
			continue
		}
		img, _, err := image.Decode(f)
		_ = f.Close()
		if err != nil {
			continue
		}
		return toRGBA(img)
	}
	t.Skipf("benchmark image missing: tried %v", paths)
	return nil
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
		var buf bytes.Buffer

		if testing.Verbose() {
			b.Logf("cpus=%d gomaxprocs=%d goroutines=%d", runtime.NumCPU(), runtime.GOMAXPROCS(0), runtime.NumGoroutine())

			startEnc := time.Now()
			buf.Reset()
			if err := enc.EncodeTo(&buf, img, 80, false); err != nil {
				b.Fatalf("encode failed: %v", err)
			}
			encBytes := buf.Bytes()
			encTime := time.Since(startEnc)

			startDec := time.Now()
			if _, err := dec.Decode(encBytes); err != nil {
				b.Fatalf("decode failed: %v", err)
			}
			decTime := time.Since(startDec)

			b.Logf("encode=%v decode=%v size=%d bytes", encTime, decTime, len(encBytes))
		}

		benchmarkEncodeDecode(b,
			func() ([]byte, error) {
				buf.Reset()
				if err := enc.EncodeTo(&buf, img, 80, false); err != nil {
					return nil, err
				}
				return buf.Bytes(), nil
			},
			func(encBytes []byte) error {
				_, err := dec.Decode(encBytes)
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

func benchmarkBABEQuality(b *testing.B, quality int, bw bool) {
	img := loadTestImage(b)
	enc := NewEncoder()
	dec := NewDecoder()
	var buf bytes.Buffer

	if testing.Verbose() {
		b.Logf("quality=%d bw=%v cpus=%d gomaxprocs=%d goroutines=%d", quality, bw, runtime.NumCPU(), runtime.GOMAXPROCS(0), runtime.NumGoroutine())

		startEnc := time.Now()
		buf.Reset()
		if err := enc.EncodeTo(&buf, img, quality, bw); err != nil {
			b.Fatalf("encode failed: %v", err)
		}
		encBytes := buf.Bytes()
		encTime := time.Since(startEnc)

		startDec := time.Now()
		if _, err := dec.Decode(encBytes); err != nil {
			b.Fatalf("decode failed: %v", err)
		}
		decTime := time.Since(startDec)

		b.Logf("encode=%v decode=%v size=%d bytes", encTime, decTime, len(encBytes))
	}

	benchmarkEncodeDecode(b,
		func() ([]byte, error) {
			buf.Reset()
			if err := enc.EncodeTo(&buf, img, quality, bw); err != nil {
				return nil, err
			}
			return buf.Bytes(), nil
		},
		func(encBytes []byte) error {
			_, err := dec.Decode(encBytes)
			return err
		},
	)
}

func BenchmarkBABEQualitySteps(b *testing.B) {
	for _, quality := range []int{0, 20, 40, 60, 80} {
		b.Run(fmt.Sprintf("Q%d", quality), func(b *testing.B) {
			benchmarkBABEQuality(b, quality, false)
		})
	}
}

func BenchmarkBABEQualityStepsBW(b *testing.B) {
	for _, quality := range []int{0, 20, 40, 60, 80} {
		b.Run(fmt.Sprintf("Q%d_BW", quality), func(b *testing.B) {
			benchmarkBABEQuality(b, quality, true)
		})
	}
}

func BenchmarkBABEDecodeQ0Patterns64Blocks8_128Tile32Q4(b *testing.B) {
	img := loadTestImagePrefer(b, "4.jpg", "benchmark.jpg")
	levels, err := blocksFromSpec("8-128")
	if err != nil {
		b.Fatalf("blocksFromSpec: %v", err)
	}
	enc := NewEncoder()
	enc.patternCount = 64
	enc.backgroundTile = 32
	enc.yQuantShift = 4
	enc.levels = levels
	comp, err := enc.Encode(img, 0, false)
	if err != nil {
		b.Fatalf("Encode: %v", err)
	}
	dec := NewDecoder()
	if _, err := dec.Decode(comp); err != nil {
		b.Fatalf("warmup decode: %v", err)
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if _, err := dec.Decode(comp); err != nil {
			b.Fatalf("Decode: %v", err)
		}
	}
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
		var buf bytes.Buffer
		sizeB := 0
		var encTotal, decTotal time.Duration

		// Warm-up and reset so one-time allocations don't dominate the summary.
		buf.Reset()
		if err := enc.EncodeTo(&buf, img, 80, false); err != nil {
			b.Fatalf("encode failed: %v", err)
		}
		encBytes := buf.Bytes()
		if _, err := dec.Decode(encBytes); err != nil {
			b.Fatalf("decode failed: %v", err)
		}
		b.ResetTimer()

		for i := 0; i < b.N; i++ {
			buf.Reset()
			startEnc := time.Now()
			if err := enc.EncodeTo(&buf, img, 80, false); err != nil {
				b.Fatalf("encode failed: %v", err)
			}
			encTotal += time.Since(startEnc)
			encBytes := buf.Bytes()
			sizeB = len(encBytes)

			startDec := time.Now()
			if _, err := dec.Decode(encBytes); err != nil {
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
