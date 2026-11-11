package classic

import (
	"bytes"
	"encoding/binary"
	"image"
	"image/color"
	"testing"
)

func TestCornerContainerRoundTrip(t *testing.T) {
	for _, dims := range []image.Point{{1, 1}, {1, 17}, {17, 1}, {5, 7}, {32, 24}, {33, 25}} {
		img := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		for y := 0; y < dims.Y; y++ {
			for x := 0; x < dims.X; x++ {
				img.SetRGBA(x, y, color.RGBA{byte(x * 7), byte(y * 7), byte((x + y) * 3), 255})
			}
		}
		for _, q := range []int{0, 10, 80, 100} {
			for _, bw := range []bool{false, true} {
				if q == 100 && bw {
					continue
				}
				enc := NewEncoder()
				enc.CornerData = true
				data, err := enc.Encode(img, q, bw)
				if err != nil {
					t.Fatalf("%v q%d: %v", dims, q, err)
				}
				data = bytes.Clone(data)
				if q < 100 && string(data[:4]) != cornerMagic {
					t.Fatal("missing corner container")
				}
				if q == 100 && string(data[:4]) != packedMagic {
					t.Fatal("lossless format changed")
				}
				if q == 100 {
					standard, err := NewEncoder().Encode(img, 100, false)
					if err != nil || !bytes.Equal(data, standard) {
						t.Fatal("lossless bitstream changed", err)
					}
				}
				serial, parallel := NewDecoder(), NewDecoder()
				serial.Parallel = false
				serial.GlyphBlend, parallel.GlyphBlend = true, true
				a, err := serial.Decode(data, false)
				if err != nil {
					t.Fatalf("%v q%d: %v", dims, q, err)
				}
				b, err := parallel.Decode(data, false)
				if err != nil || !bytes.Equal(a.Pix, b.Pix) {
					t.Fatalf("%v q%d: serial/parallel mismatch: %v", dims, q, err)
				}
				if q == 100 && !bytes.Equal(a.Pix, img.Pix) {
					t.Fatal("lossless pixels changed")
				}
			}
		}
	}
}

func TestCornerContainerRejectsDamage(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 16, 16))
	for i := 3; i < len(img.Pix); i += 4 {
		img.Pix[i] = 255
	}
	enc := NewEncoder()
	enc.CornerData = true
	data, err := enc.Encode(img, 10, false)
	if err != nil {
		t.Fatal(err)
	}
	data = bytes.Clone(data)
	for _, broken := range [][]byte{data[:24], data[:len(data)-1], append(bytes.Clone(data), 0)} {
		if _, err := NewDecoder().Decode(broken, false); err == nil {
			t.Fatal("accepted truncated or trailing data")
		}
	}
	for _, offset := range []int{4, 5, 13, 17, 21, len(data) - 1} {
		broken := bytes.Clone(data)
		broken[offset] ^= 0x80
		if _, err := NewDecoder().Decode(broken, false); err == nil {
			t.Fatalf("accepted corruption at %d", offset)
		}
	}
	broken := bytes.Clone(data)
	binary.BigEndian.PutUint32(broken[5:], 100000000)
	if _, err := NewDecoder().Decode(broken, false); err == nil {
		t.Fatal("accepted oversized dimensions")
	}
}
