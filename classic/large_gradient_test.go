package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestLargeGradientSelectionAndEdges(t *testing.T) {
	const w, h = 128, 64
	plane := make([]byte, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			plane[y*w+x] = byte(30 + x/2 + y/2)
		}
	}
	// A high-contrast line must not be swallowed by a selected gradient.
	for y := 0; y < h; y++ {
		plane[y*w+31] = 255
	}
	blocks := selectLargeGradients(plane, w, h, 10)
	hasLarge, hasUnaffectedArea := false, false
	for _, b := range blocks {
		if b.size > 16 {
			hasLarge = true
		}
		if b.x >= 64 {
			hasUnaffectedArea = true
		}
		if b.x <= 31 && b.x+b.size > 31 {
			t.Fatalf("gradient swallowed line: %+v", b)
		}
		for y := b.y; y < b.y+b.size; y++ {
			for x := b.x; x < b.x+b.size; x++ {
				d := int(plane[y*w+x]) - int(gradientAt(b, x, y))
				if d < -18 || d > 18 {
					t.Fatal("gradient exceeds error limit")
				}
			}
		}
	}
	if !hasLarge || !hasUnaffectedArea {
		t.Fatalf("expected gradients in smooth area: %+v", blocks)
	}
	if err := validateLargeGradients(blocks, w, h); err != nil {
		t.Fatal(err)
	}
}

func TestLargeGradientEncodeDecode(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 128, 64))
	for y := 0; y < 64; y++ {
		for x := 0; x < 128; x++ {
			v := uint8(30 + x/2 + y/2)
			img.SetRGBA(x, y, color.RGBA{v, v, v, 255})
		}
	}
	e := NewEncoder()
	e.LargeGradients = true
	e.SparseCorners = true
	e.CornerMinSize = 5
	data, err := e.Encode(img, 10, false)
	if err != nil {
		t.Fatal(err)
	}
	data = bytes.Clone(data)
	if string(data[:4]) != gradientMagic {
		t.Fatal("expected encoded gradients")
	}
	_, blocks, err := unpackLargeGradients(data)
	if err != nil || len(blocks) == 0 {
		t.Fatalf("invalid gradient stream: %v", err)
	}
	a, b := NewDecoder(), NewDecoder()
	a.Smooth, b.Smooth, a.Parallel = true, true, false
	one, err := a.Decode(data, false)
	if err != nil {
		t.Fatal(err)
	}
	two, err := b.Decode(data, false)
	if err != nil || !bytes.Equal(one.Pix, two.Pix) {
		t.Fatalf("parallel mismatch: %v", err)
	}
	for _, point := range []image.Point{{4, 4}, {32, 32}, {96, 32}} {
		r, _, _, _ := one.At(point.X, point.Y).RGBA()
		want := int(img.RGBAAt(point.X, point.Y).R)
		got := int(r >> 8)
		if got-want > 4 || want-got > 4 {
			t.Fatalf("gradient pixel %v got %d want %d", point, got, want)
		}
	}
	broken := bytes.Clone(data)
	broken[len(broken)-1] ^= 1
	if _, err := NewDecoder().Decode(broken, false); err == nil {
		t.Fatal("corrupt gradient data accepted")
	}
	lossless, err := e.Encode(img, 100, false)
	if err != nil {
		t.Fatal(err)
	}
	base, err := NewEncoder().Encode(img, 100, false)
	if err != nil || !bytes.Equal(lossless, base) {
		t.Fatalf("lossless data changed: %v", err)
	}
}

func TestValidateLargeGradients(t *testing.T) {
	block := largeGradient{x: 0, y: 0, size: 32}
	for _, blocks := range [][]largeGradient{{block, block}, {{x: 16, y: 16, size: 64}}} {
		if validateLargeGradients(blocks, 64, 64) == nil {
			t.Fatal("accepted invalid placement")
		}
	}
}
