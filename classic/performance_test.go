package classic

import (
	"fmt"
	"image"
	_ "image/jpeg"
	"os"
	"testing"
)

// Local photo benchmarks are optional: sample JPEGs are intentionally not tracked.
func BenchmarkImageCodec(b *testing.B) {
	for _, file := range []string{"1.jpg", "3.jpg"} {
		f, err := os.Open("../" + file)
		if err != nil {
			continue
		}
		img, _, err := image.Decode(f)
		f.Close()
		if err != nil {
			b.Fatal(err)
		}
		for _, q := range []int{10, 70, 80} {
			b.Run(fmt.Sprintf("%s/Q%d", file, q), func(b *testing.B) {
				e, d := NewEncoder(), NewDecoder()
				defer d.zdec.Close()
				encoded, err := e.Encode(img, q, false)
				if err != nil {
					b.Fatal(err)
				}
				b.Run("Encode", func(b *testing.B) {
					b.ReportAllocs()
					for i := 0; i < b.N; i++ {
						encoded, err = e.Encode(img, q, false)
						if err != nil {
							b.Fatal(err)
						}
					}
					b.ReportMetric(float64(len(encoded)), "file-bytes")
				})
				if _, err = d.Decode(encoded, false); err != nil {
					b.Fatal(err)
				}
				b.Run("Decode", func(b *testing.B) {
					b.ReportAllocs()
					for i := 0; i < b.N; i++ {
						if _, err := d.Decode(encoded, false); err != nil {
							b.Fatal(err)
						}
					}
				})
				b.Run("Cycle", func(b *testing.B) {
					b.ReportAllocs()
					for i := 0; i < b.N; i++ {
						encoded, err = e.Encode(img, q, false)
						if err != nil {
							b.Fatal(err)
						}
						if _, err := d.Decode(encoded, false); err != nil {
							b.Fatal(err)
						}
					}
					b.ReportMetric(float64(len(encoded)), "file-bytes")
				})
			})
		}
	}
}
