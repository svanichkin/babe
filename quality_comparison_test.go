package main

import (
	"bytes"
	"fmt"
	"image/jpeg"
	"testing"

	"github.com/xfmoulet/qoi"
)

// BenchmarkQualityComparison compares warmed in-memory operations on the same
// RGBA source. The optional benchmark.jpg is loaded before the timers start.
func BenchmarkQualityComparison(b *testing.B) {
	img := loadTestImage(b)
	for _, quality := range []int{100, 80, 50, 10, 0} {
		b.Run(fmt.Sprintf("Q%d", quality), func(b *testing.B) {
			b.Run("JPEG", func(b *testing.B) {
				var buf bytes.Buffer
				var r bytes.Reader
				encode := func() ([]byte, error) {
					buf.Reset()
					err := jpeg.Encode(&buf, img, &jpeg.Options{Quality: quality})
					return buf.Bytes(), err
				}
				decode := func(data []byte) error {
					r.Reset(data)
					_, err := jpeg.Decode(&r)
					return err
				}
				benchmarkCodecOperations(b, encode, decode)
			})
			b.Run("BABE", func(b *testing.B) {
				e, d := NewEncoder(), NewDecoder()
				encode := func() ([]byte, error) { return e.Encode(img, quality, false) }
				decode := func(data []byte) error { _, err := d.Decode(data, false); return err }
				benchmarkCodecOperations(b, encode, decode)
			})
		})
	}
	b.Run("QOI", func(b *testing.B) {
		var buf bytes.Buffer
		var r bytes.Reader
		encode := func() ([]byte, error) {
			buf.Reset()
			err := qoi.Encode(&buf, img)
			return buf.Bytes(), err
		}
		decode := func(data []byte) error {
			r.Reset(data)
			_, err := qoi.Decode(&r)
			return err
		}
		benchmarkCodecOperations(b, encode, decode)
	})
}

func benchmarkCodecOperations(b *testing.B, encode func() ([]byte, error), decode func([]byte) error) {
	encoded, err := encode()
	if err != nil {
		b.Fatal(err)
	}
	if err := decode(encoded); err != nil {
		b.Fatal(err)
	}
	fileBytes := len(encoded)
	b.Run("Encode", func(b *testing.B) {
		b.ReportAllocs()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			if _, err := encode(); err != nil {
				b.Fatal(err)
			}
		}
		b.ReportMetric(float64(fileBytes), "file-bytes")
	})
	// Encode can reuse its output buffer, so capture its final result before
	// measuring the decoder; neither path includes loading or writing files.
	encoded, err = encode()
	if err != nil {
		b.Fatal(err)
	}
	b.Run("Decode", func(b *testing.B) {
		b.ReportAllocs()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			if err := decode(encoded); err != nil {
				b.Fatal(err)
			}
		}
	})
}
