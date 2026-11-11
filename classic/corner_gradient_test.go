package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestCornerGridBilinear(t *testing.T) {
	g := cornerGrid{width: 9, height: 9, step: 4, cols: 3, values: make([][3]byte, 9)}
	for y := range 3 {
		for x := range 3 {
			g.values[y*3+x] = [3]byte{byte(x*40 + y*20), 128, 128}
		}
	}
	for y := range 9 {
		for x := range 9 {
			got := g.at(x, y)
			if want := byte(x*10 + y*5); got != [3]byte{want, 128, 128} {
				t.Fatalf("(%d,%d): %v, want %d", x, y, got, want)
			}
		}
	}
}

func TestCornerGradientDecoding(t *testing.T) {
	changed := false
	for _, dims := range []image.Point{{1, 1}, {1, 9}, {9, 1}, {5, 7}, {32, 24}, {33, 25}} {
		img := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		for y := 0; y < dims.Y; y++ {
			for x := 0; x < dims.X; x++ {
				img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 7), uint8((x + y) * 3), 255})
			}
		}
		for _, quality := range []int{0, 17, 80, 100} {
			encoded, err := NewEncoder().Encode(img, quality, false)
			if err != nil {
				t.Fatal(err)
			}
			serial, parallel := NewDecoder(), NewDecoder()
			serial.CornerGradient, parallel.CornerGradient = true, true
			serial.Parallel = false
			a, err := serial.Decode(encoded, false)
			if err != nil {
				t.Fatal(err)
			}
			b, err := parallel.Decode(encoded, false)
			if err != nil || !bytes.Equal(a.Pix, b.Pix) {
				t.Fatalf("%v q%d: serial/parallel mismatch: %v", dims, quality, err)
			}
			if quality == 100 && !bytes.Equal(a.Pix, img.Pix) {
				t.Fatalf("%v: lossless pixels changed", dims)
			}
			if dims == (image.Point{32, 24}) && quality == 0 {
				normal, err := NewDecoder().Decode(encoded, false)
				if err != nil {
					t.Fatal(err)
				}
				changed = !bytes.Equal(a.Pix, normal.Pix)
			}
		}
	}
	if !changed {
		t.Fatal("corner reconstruction did not affect lossy output")
	}
}
