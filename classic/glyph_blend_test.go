package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestBlendGlyphResidual(t *testing.T) {
	grid := cornerGrid{width: 8, height: 8, step: 4, cols: 3, values: make([][3]byte, 9)}
	sizes := make([]byte, 64)
	for i := range sizes {
		sizes[i] = 4
	}
	luma := func(x, y int) byte {
		if x < 4 {
			return 100
		}
		return 116
	}
	left := blendGlyphResidual(3, 2, 8, 8, 4, sizes, 100, luma, grid)
	right := blendGlyphResidual(4, 2, 8, 8, 4, sizes, 116, luma, grid)
	if left != 106 || right != 110 {
		t.Fatalf("seam was not blended symmetrically: %d, %d", left, right)
	}
	if got := blendGlyphResidual(2, 2, 8, 8, 4, sizes, 100, luma, grid); got != 100 {
		t.Fatalf("interior changed: %d", got)
	}
	strong := func(x, y int) byte {
		if x < 4 {
			return 30
		}
		return 220
	}
	if got := blendGlyphResidual(3, 2, 8, 8, 4, sizes, 30, strong, grid); got != 30 {
		t.Fatalf("strong edge changed: %d", got)
	}
	if got := blendGlyphResidual(3, 2, 8, 8, 8, sizes, 100, luma, grid); got != 100 {
		t.Fatalf("4x4 block changed under minimum 8: %d", got)
	}
	for i := range sizes {
		sizes[i] = 8
	}
	if got := blendGlyphResidual(3, 2, 8, 8, 4, sizes, 100, luma, grid); got != 100 {
		t.Fatalf("internal pixel of 8x8 block changed: %d", got)
	}
}

func TestGlyphBlockSizesUsePartition(t *testing.T) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()
	oldSmall, oldMacro := smallBlock, macroBlock
	defer func() { smallBlock, macroBlock = oldSmall, oldMacro }()
	smallBlock, macroBlock = 4, 8
	// The first 8×8 group is a macroblock; the second is split into four 4×4 blocks.
	segment := []byte{0, 0, 0, 0, 0, 0, 0, 1, 0x80}
	sizes := NewDecoder().glyphBlockSizes(segment, 16, 8)
	for y := 0; y < 8; y++ {
		for x := 0; x < 16; x++ {
			want := byte(8)
			if x >= 8 {
				want = 4
			}
			if sizes[y*16+x] != want {
				t.Fatalf("(%d,%d): got %d want %d", x, y, sizes[y*16+x], want)
			}
		}
	}
	grid := cornerGrid{width: 16, height: 8, step: 8, cols: 3, values: make([][3]byte, 6)}
	luma := func(x, y int) byte {
		if x < 8 {
			return 100
		}
		return 116
	}
	if got := blendGlyphResidual(7, 2, 16, 8, 8, sizes, 100, luma, grid); got != 106 {
		t.Fatalf("eligible side not blended: %d", got)
	}
	if got := blendGlyphResidual(8, 2, 16, 8, 8, sizes, 116, luma, grid); got != 116 {
		t.Fatalf("smaller side changed: %d", got)
	}
}

func TestGlyphMinimumAboveLargestBlockIsNoOp(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 32, 32))
	for y := range 32 {
		for x := range 32 {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 7), uint8((x + y) * 3), 255})
		}
	}
	e := NewEncoder()
	e.CornerData = true
	data, err := e.Encode(img, 80, false)
	if err != nil {
		t.Fatal(err)
	}
	standard, limited := NewDecoder(), NewDecoder()
	limited.GlyphBlend, limited.GlyphMinSize = true, 4
	a, err := standard.Decode(data, false)
	if err != nil {
		t.Fatal(err)
	}
	b, err := limited.Decode(data, false)
	if err != nil || !bytes.Equal(a.Pix, b.Pix) {
		t.Fatalf("minimum larger than blocks changed output: %v", err)
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
