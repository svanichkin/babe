package classic

import (
	"bytes"
	"image"
	"image/color"
	"image/draw"
	"image/jpeg"
	"math/rand"
	"testing"
)

func TestLosslessRGBRoundTrip(t *testing.T) {
	rng := rand.New(rand.NewSource(8231))
	for _, dims := range []image.Point{{1, 1}, {1, 19}, {21, 1}, {2, 2}, {17, 19}, {257, 129}} {
		// A nonzero origin and extra stride exercise actual subimages.
		base := image.NewRGBA(image.Rect(0, 0, dims.X+10, dims.Y+10))
		img := base.SubImage(image.Rect(3, 4, 3+dims.X, 4+dims.Y)).(*image.RGBA)
		for y := img.Rect.Min.Y; y < img.Rect.Max.Y; y++ {
			for x := img.Rect.Min.X; x < img.Rect.Max.X; x++ {
				img.SetRGBA(x, y, color.RGBA{byte(rng.Intn(256)), byte(rng.Intn(256)), byte(rng.Intn(256)), 255})
			}
		}
		want := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		draw.Draw(want, want.Bounds(), img, img.Rect.Min, draw.Src)
		e, d := NewEncoder(), NewDecoder()
		defer d.zdec.Close()
		packed, err := e.Encode(img, 100, false)
		if err != nil {
			t.Fatal(err)
		}
		packed = bytes.Clone(packed)
		if len(packed) < 14 || packed[13]&channelFlagLossless == 0 {
			t.Fatal("missing lossless flag")
		}
		for _, filter := range []bool{false, true} {
			got, err := d.Decode(packed, filter)
			if err != nil || !bytes.Equal(got.Pix, want.Pix) {
				t.Fatalf("%v filter=%v: pixel mismatch: %v", dims, filter, err)
			}
		}
		e.Parallel, d.Parallel = false, false
		serial, err := e.Encode(img, 100, false)
		if err != nil || !bytes.Equal(packed, serial) {
			t.Fatalf("%v: serial encoder differs: %v", dims, err)
		}
		got, err := d.Decode(serial, false)
		if err != nil || !bytes.Equal(got.Pix, want.Pix) {
			t.Fatalf("%v: serial decode mismatch: %v", dims, err)
		}
	}
}

func TestLosslessJPEGInput(t *testing.T) {
	var jpegData bytes.Buffer
	if err := jpeg.Encode(&jpegData, compressionTestImage(), &jpeg.Options{Quality: 90}); err != nil {
		t.Fatal(err)
	}
	src, err := jpeg.Decode(&jpegData)
	if err != nil {
		t.Fatal(err)
	}
	want := image.NewRGBA(src.Bounds())
	draw.Draw(want, want.Bounds(), src, src.Bounds().Min, draw.Src)
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	encoded, err := e.Encode(src, 100, false)
	if err != nil {
		t.Fatal(err)
	}
	got, err := d.Decode(encoded, false)
	if err != nil || !bytes.Equal(want.Pix, got.Pix) {
		t.Fatalf("decoded JPEG RGB pixels changed: %v", err)
	}
}

func TestLosslessOpaqueNRGBAAndPalette(t *testing.T) {
	nrgba := image.NewNRGBA(image.Rect(4, 7, 13, 16))
	for y := nrgba.Rect.Min.Y; y < nrgba.Rect.Max.Y; y++ {
		for x := nrgba.Rect.Min.X; x < nrgba.Rect.Max.X; x++ {
			nrgba.SetNRGBA(x, y, color.NRGBA{byte(x * 17), byte(y * 11), byte(x * y), 255})
		}
	}
	palette := image.NewPaletted(image.Rect(0, 0, 9, 9), color.Palette{
		color.NRGBA{255, 0, 0, 255}, color.RGBA{0, 255, 255, 255}, color.Gray{Y: 42},
	})
	for i := range palette.Pix {
		palette.Pix[i] = byte(i % 3)
	}
	for _, src := range []image.Image{nrgba, palette} {
		bounds := src.Bounds()
		want := image.NewRGBA(image.Rect(0, 0, bounds.Dx(), bounds.Dy()))
		draw.Draw(want, want.Bounds(), src, bounds.Min, draw.Src)
		e, d := NewEncoder(), NewDecoder()
		encoded, err := e.Encode(src, 100, false)
		if err != nil {
			t.Fatal(err)
		}
		got, err := d.Decode(encoded, false)
		d.zdec.Close()
		if err != nil || !bytes.Equal(got.Pix, want.Pix) {
			t.Fatalf("%T: pixel mismatch: %v", src, err)
		}
	}
}

func TestLosslessTwoToneBlocks(t *testing.T) {
	colors := []color.RGBA{{0, 0, 0, 255}, {255, 0, 0, 255}, {0, 128, 255, 255}}
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	// Every combination forces solid, two-tone and split blocks in different
	// reversible channels, including the all-zero and wrapped-byte cases.
	for a := range colors {
		for b := range colors {
			for c := range colors {
				for z := range colors {
					img := image.NewRGBA(image.Rect(0, 0, 2, 2))
					for i, index := range []int{a, b, c, z} {
						img.SetRGBA(i%2, i/2, colors[index])
					}
					encoded, err := e.Encode(img, 100, false)
					if err != nil {
						t.Fatal(err)
					}
					decoded, err := d.Decode(encoded, false)
					if err != nil || !bytes.Equal(decoded.Pix, img.Pix) {
						t.Fatalf("colors %d/%d/%d/%d: %v", a, b, c, z, err)
					}
				}
			}
		}
	}
}

func TestLosslessInputRestrictions(t *testing.T) {
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	transparent := image.NewNRGBA(image.Rect(0, 0, 2, 2))
	transparent.SetNRGBA(0, 0, color.NRGBA{255, 0, 0, 128})
	if _, err := e.Encode(transparent, 100, false); err == nil {
		t.Fatal("accepted transparency")
	}
	if _, err := e.Encode(image.NewRGBA64(image.Rect(0, 0, 2, 2)), 100, false); err == nil {
		t.Fatal("accepted 16-bit input")
	}
	if _, err := e.Encode(compressionTestImage(), 100, true); err == nil {
		t.Fatal("accepted lossy grayscale conversion")
	}
	gray := image.NewGray(image.Rect(0, 0, 5, 7))
	for i := range gray.Pix {
		gray.Pix[i] = byte(i * 13)
	}
	encoded, err := e.Encode(gray, 100, true)
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := d.Decode(encoded, false)
	if err != nil {
		t.Fatal(err)
	}
	for i, v := range gray.Pix {
		if decoded.Pix[i*4] != v || decoded.Pix[i*4+1] != v || decoded.Pix[i*4+2] != v || decoded.Pix[i*4+3] != 255 {
			t.Fatal("grayscale pixels changed")
		}
	}
}

func TestExactQuad(t *testing.T) {
	var cases [][4]byte
	for _, a := range []byte{0, 1, 255} {
		for _, b := range []byte{0, 1, 255} {
			for _, c := range []byte{0, 1, 255} {
				for _, d := range []byte{0, 1, 255} {
					cases = append(cases, [4]byte{a, b, c, d})
				}
			}
		}
	}
	for _, values := range cases {
		fg, bg, bits, ok := exactQuad(values[0], values[1], values[2], values[3])
		unique := make(map[byte]bool)
		for _, v := range values {
			unique[v] = true
		}
		if ok != (len(unique) <= 2) {
			t.Fatalf("incorrect split decision for %v", values)
		}
		if !ok {
			continue
		}
		for i, v := range values {
			got := bg
			if bits&(1<<uint(3-i)) != 0 {
				got = fg
			}
			if got != v {
				t.Fatalf("block %v mask %04b decoded %d instead of %d", values, bits, got, v)
			}
		}
	}
}

func TestLosslessCorruption(t *testing.T) {
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	encoded, err := e.Encode(compressionTestImage(), 100, false)
	if err != nil {
		t.Fatal(err)
	}
	for _, n := range []int{0, 1, 4, 13, 22, len(encoded) - 1} {
		if _, err := d.Decode(encoded[:n], false); err == nil {
			t.Fatalf("accepted truncation at %d", n)
		}
	}
	for _, offset := range []int{9, 13, 18, 22, len(encoded) - 1} {
		bad := bytes.Clone(encoded)
		bad[offset] ^= 0xff
		if _, err := d.Decode(bad, false); err == nil {
			t.Fatalf("accepted corruption at %d", offset)
		}
	}
	if _, err := d.Decode(encoded, false); err != nil {
		t.Fatalf("failed to recover after invalid data: %v", err)
	}
}
