package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestGradientBlocks(t *testing.T) {
	for _, vertical := range []bool{false, true} {
		for _, reverse := range []bool{false, true} {
			img := image.NewRGBA(image.Rect(0, 0, 256, 256))
			for y := 0; y < 256; y++ {
				for x := 0; x < 256; x++ {
					v := x
					if vertical {
						v = y
					}
					if reverse {
						v = 255 - v
					}
					img.SetRGBA(x, y, color.RGBA{uint8(v), uint8(v), uint8(v), 255})
				}
			}
			var errors [2]uint64
			var sizes [2]int
			for i := range 2 {
				e := NewEncoder()
				e.Gradient = i == 1
				data, err := e.Encode(img, 20, false)
				if err != nil {
					t.Fatal(err)
				}
				sizes[i] = len(data)
				decoded, err := NewDecoder().Decode(data, false)
				if err != nil {
					t.Fatal(err)
				}
				for j := 0; j < len(img.Pix); j += 4 {
					d := int(img.Pix[j]) - int(decoded.Pix[j])
					errors[i] += uint64(d * d)
				}
			}
			t.Logf("vertical=%v reverse=%v: SSE %d -> %d, bytes %d -> %d", vertical, reverse, errors[0], errors[1], sizes[0], sizes[1])
			if errors[1] >= errors[0] {
				t.Fatalf("gradient did not improve reconstruction")
			}
		}
	}
}

func TestGradientFlagLosslessAndDecoderReuse(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 32, 32))
	for y := 0; y < 32; y++ {
		for x := 0; x < 32; x++ {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 7), uint8((x + y) * 3), 255})
		}
	}
	e := NewEncoder()
	e.Gradient = true
	gradient, err := e.Encode(img, 20, false)
	if err != nil {
		t.Fatal(err)
	}
	gradient = bytes.Clone(gradient)
	lossless, err := e.Encode(img, 100, false)
	if err != nil {
		t.Fatal(err)
	}
	lossless = bytes.Clone(lossless)
	standard := NewEncoder()
	plain, err := standard.Encode(img, 20, false)
	if err != nil {
		t.Fatal(err)
	}
	plain = bytes.Clone(plain)
	wantPlain, err := NewDecoder().Decode(plain, false)
	if err != nil {
		t.Fatal(err)
	}
	wantPlainPixels := bytes.Clone(wantPlain.Pix)
	d := NewDecoder()
	for _, data := range [][]byte{gradient, plain, gradient, lossless, plain, lossless} {
		decoded, err := d.Decode(data, false)
		if err != nil {
			t.Fatal(err)
		}
		if bytes.Equal(data, lossless) && !bytes.Equal(decoded.Pix, img.Pix) {
			t.Fatal("lossless changed pixels")
		}
		if bytes.Equal(data, plain) && !bytes.Equal(decoded.Pix, wantPlainPixels) {
			t.Fatal("gradient state leaked into a regular decode")
		}
	}
	regularLossless, err := standard.Encode(img, 100, false)
	if err != nil || !bytes.Equal(lossless, regularLossless) {
		t.Fatal("gradient option changed lossless encoding", err)
	}
}
