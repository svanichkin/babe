package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestDither3Threshold(t *testing.T) {
	plane := []byte{100, 100, 110, 110, 200, 200}
	d := NewDecoder()
	d.Parallel = false
	d.blurLuma(len(plane), 1, func(x, y int) byte { return plane[x] }, func(x, y int, v byte) { plane[x] = v })
	if plane[1] <= 100 || plane[2] >= 110 {
		t.Fatalf("small transition was not softened: %v", plane)
	}
	if plane[3] != 110 || plane[4] != 200 {
		t.Fatalf("strong edge was modified: %v", plane)
	}
	flat := []byte{73, 73, 73, 73}
	d.blurLuma(2, 2, func(x, y int) byte { return flat[y*2+x] }, func(x, y int, v byte) { flat[y*2+x] = v })
	if !bytes.Equal(flat, []byte{73, 73, 73, 73}) {
		t.Fatalf("flat area changed: %v", flat)
	}
}

func TestDither3Decode(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 128, 128))
	for y := range 128 {
		for x := range 128 {
			v := uint8(x*3 + y*5)
			img.SetRGBA(x, y, color.RGBA{v, v, v, 255})
		}
	}
	changed := false
	for _, q := range []int{0, 10, 80, 100} {
		for _, mode := range []string{"plain", "dense", "sparse"} {
			e := NewEncoder()
			e.CornerData = mode == "dense"
			e.SparseCorners = mode == "sparse"
			data, err := e.Encode(img, q, false)
			if err != nil {
				t.Fatal(err)
			}
			data = bytes.Clone(data)
			plain, serial, parallel := NewDecoder(), NewDecoder(), NewDecoder()
			serial.Dither3, parallel.Dither3, serial.Parallel = true, true, false
			base, err := plain.Decode(data, false)
			if err != nil {
				t.Fatal(err)
			}
			a, err := serial.Decode(data, false)
			if err != nil {
				t.Fatal(err)
			}
			b, err := parallel.Decode(data, false)
			if err != nil || !bytes.Equal(a.Pix, b.Pix) {
				t.Fatalf("q%d %s: serial/parallel differ: %v", q, mode, err)
			}
			if q == 100 && (!bytes.Equal(a.Pix, img.Pix) || !bytes.Equal(base.Pix, img.Pix)) {
				t.Fatal("lossless pixels changed")
			}
			if q == 80 && mode == "plain" && bytes.Equal(a.Pix, base.Pix) {
				t.Fatal("blur did not affect eligible small blocks")
			}
			if q < 100 && !bytes.Equal(a.Pix, base.Pix) {
				changed = true
			}
		}
	}
	if !changed {
		t.Fatal("decoder blur did not change any lossy image")
	}
}

func TestDither2And3Together(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 64, 64))
	for y := range 64 {
		for x := range 64 {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 3), uint8(y * 3), uint8(x + y), 255})
		}
	}
	for _, q := range []int{10, 80, 100} {
		e := NewEncoder()
		e.SparseCorners = true
		data, err := e.Encode(img, q, false)
		if err != nil {
			t.Fatal(err)
		}
		a, b := NewDecoder(), NewDecoder()
		a.Dither2, a.Dither3, a.GlyphBlend, a.GlyphMinSize, a.Parallel = true, true, true, 4, false
		b.Dither2, b.Dither3, b.GlyphBlend, b.GlyphMinSize = true, true, true, 4
		one, err := a.Decode(data, false)
		if err != nil {
			t.Fatal(err)
		}
		two, err := b.Decode(data, false)
		if err != nil || !bytes.Equal(one.Pix, two.Pix) {
			t.Fatalf("q%d combined: serial/parallel mismatch: %v", q, err)
		}
		if q == 100 && !bytes.Equal(one.Pix, img.Pix) {
			t.Fatal("combined modes changed lossless pixels")
		}
	}
}

func TestDither3SmallImages(t *testing.T) {
	for _, dims := range []image.Point{{1, 1}, {1, 9}, {9, 1}, {5, 7}} {
		img := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		for i := 0; i < len(img.Pix); i += 4 {
			img.Pix[i], img.Pix[i+1], img.Pix[i+2], img.Pix[i+3] = 80, 90, 100, 255
		}
		for _, q := range []int{10, 80, 100} {
			e := NewEncoder()
			e.SparseCorners = true
			data, err := e.Encode(img, q, false)
			if err != nil {
				t.Fatal(err)
			}
			d := NewDecoder()
			d.Dither3 = true
			got, err := d.Decode(data, false)
			if err != nil || got.Bounds() != img.Bounds() {
				t.Fatalf("%v q%d: %v", dims, q, err)
			}
			if q == 100 && !bytes.Equal(got.Pix, img.Pix) {
				t.Fatalf("%v: lossless changed", dims)
			}
		}
	}
}
