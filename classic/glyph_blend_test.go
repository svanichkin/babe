package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestBlendGlyphResidual(t *testing.T) {
	grid := cornerGrid{width: 8, height: 8, step: 4, cols: 3, values: make([][3]byte, 9)}
	luma := func(x, y int) byte {
		if x < 4 {
			return 100
		}
		return 116
	}
	left := blendGlyphResidual(3, 2, 8, 8, 4, 100, luma, grid)
	right := blendGlyphResidual(4, 2, 8, 8, 4, 116, luma, grid)
	if left != 106 || right != 110 {
		t.Fatalf("seam was not blended symmetrically: %d, %d", left, right)
	}
	if got := blendGlyphResidual(2, 2, 8, 8, 4, 100, luma, grid); got != 100 {
		t.Fatalf("interior changed: %d", got)
	}
	strong := func(x, y int) byte {
		if x < 4 {
			return 30
		}
		return 220
	}
	if got := blendGlyphResidual(3, 2, 8, 8, 4, 30, strong, grid); got != 30 {
		t.Fatalf("strong edge changed: %d", got)
	}
}

func TestGlyphsOnly(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 17, 19))
	for y := range 19 {
		for x := range 17 {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 9), uint8(y * 9), uint8(x * y), 255})
		}
	}
	for _, quality := range []int{0, 10, 80, 100} {
		for _, corners := range []bool{false, true} {
			e := NewEncoder()
			e.CornerData = corners
			data, err := e.Encode(img, quality, false)
			if err != nil {
				t.Fatal(err)
			}
			serial, parallel := NewDecoder(), NewDecoder()
			serial.GlyphsOnly, parallel.GlyphsOnly, serial.Parallel = true, true, false
			a, err := serial.Decode(data, false)
			if err != nil {
				t.Fatal(err)
			}
			b, err := parallel.Decode(data, false)
			if err != nil || !bytes.Equal(a.Pix, b.Pix) {
				t.Fatalf("q%d corners=%v: parallel differs: %v", quality, corners, err)
			}
			for i := 0; i < len(a.Pix); i += 4 {
				if a.Pix[i] != a.Pix[i+1] || a.Pix[i] != a.Pix[i+2] || a.Pix[i+3] != 255 {
					t.Fatalf("q%d: non-grayscale pixel", quality)
				}
				if quality == 100 && a.Pix[i] != img.Pix[i+1] {
					t.Fatal("lossless glyph view is not the green channel")
				}
			}
		}
	}
}
