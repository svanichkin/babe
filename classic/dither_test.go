package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestDitherPatterns(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 128, 128))
	for y := range 128 {
		for x := range 128 {
			v := uint8(x*3 + y*5)
			img.SetRGBA(x, y, color.RGBA{v, v, v, 255})
		}
	}
	for _, quality := range []int{10, 100} {
		plain, experimental := NewEncoder(), NewEncoder()
		experimental.DitherPatterns = true
		a, err := plain.Encode(img, quality, false)
		if err != nil {
			t.Fatal(err)
		}
		a = bytes.Clone(a)
		b, err := experimental.Encode(img, quality, false)
		if err != nil {
			t.Fatal(err)
		}
		b = bytes.Clone(b)
		if quality == 100 && !bytes.Equal(a, b) {
			t.Fatal("dither changed the lossless stream")
		}
		if quality == 10 && bytes.Equal(a, b) {
			t.Fatal("dither did not change gradient patterns")
		}
		experimental.Parallel = false
		serial, err := experimental.Encode(img, quality, false)
		if err != nil || !bytes.Equal(b, serial) {
			t.Fatalf("serial/parallel dither mismatch: %v", err)
		}
		if _, err := NewDecoder().Decode(b, false); err != nil {
			t.Fatal(err)
		}
	}
}

func TestSimplifyPattern2(t *testing.T) {
	if got := simplifyPattern2(0xfffe, 4); got != 0xffff {
		t.Fatalf("single-bit detail: %04x", got)
	}
	if got := simplifyPattern2(0x5a5a, 4); got != 0x5a5a {
		t.Fatalf("tie changed: %04x", got)
	}
	if got := simplifyPattern2(0xffff, 4); got != 0xffff {
		t.Fatalf("constant changed: %04x", got)
	}
}

func TestSimplifyPatternsLosslessUnchanged(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 32, 32))
	for y := range 32 {
		for x := range 32 {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 7), uint8(x * y), 255})
		}
	}
	plain, experimental := NewEncoder(), NewEncoder()
	experimental.SimplifyPatterns = true
	a, err := plain.Encode(img, 100, false)
	if err != nil {
		t.Fatal(err)
	}
	a = bytes.Clone(a)
	b, err := experimental.Encode(img, 100, false)
	if err != nil || !bytes.Equal(a, b) {
		t.Fatal("simplification changed lossless encoding", err)
	}
}
