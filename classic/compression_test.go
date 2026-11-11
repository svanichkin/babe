package classic

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	_ "image/png"
	"math/rand"
	"os"
	"testing"
)

func TestRangeRoundTrip(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for _, n := range []int{0, 1, 2, 31, 256, 4096, 65536} {
		for kind := 0; kind < 4; kind++ {
			src := make([]byte, n)
			for i := range src {
				switch kind {
				case 0:
					src[i] = 0
				case 1:
					src[i] = 255
				case 2:
					src[i] = byte(i)
				case 3:
					src[i] = byte(rng.Intn(256))
				}
			}
			for _, order := range []int{0, 4, 8} {
				packed := encodeRange(src, order)
				got, err := decodeRange(packed, n, order)
				if err != nil || !bytes.Equal(src, got) {
					t.Fatalf("n=%d kind=%d order=%d: %v", n, kind, order, err)
				}
				if _, err := decodeRange(packed[:len(packed)-1], n, order); err == nil {
					t.Fatal("accepted truncated range stream")
				}
			}
		}
	}
	if _, err := decodeRange(nil, 1, 255); err == nil {
		t.Fatal("accepted invalid model")
	}
}

func compressionTestImage() *image.RGBA {
	// Odd dimensions and a nonzero origin exercise both edge stripes and subimages.
	img := image.NewRGBA(image.Rect(7, 5, 72, 54))
	for y := img.Rect.Min.Y; y < img.Rect.Max.Y; y++ {
		for x := img.Rect.Min.X; x < img.Rect.Max.X; x++ {
			img.SetRGBA(x, y, color.RGBA{byte(x*7 + y*3), byte(x*3 + y*5), byte(x ^ y), 255})
		}
	}
	return img
}

func TestCompressionRoundTrip(t *testing.T) {
	img := compressionTestImage()
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	sawRange := false
	for _, q := range []int{0, 10, 19, 20, 39, 40, 59, 60, 70, 79, 80, 89, 90, 100} {
		for _, bw := range []bool{false, true} {
			t.Run(fmt.Sprintf("q%d_bw%v", q, bw), func(t *testing.T) {
				e.Parallel = true
				encoded, err := e.Encode(img, q, bw)
				if err != nil {
					t.Fatal(err)
				}
				encoded = bytes.Clone(encoded)
				if string(encoded[:4]) == packedMagic {
					sawRange = true
				}
				payload := bytes.Clone(e.raw.Bytes())
				unpacked, err := decompressPayload(encoded, nil, d.zdec, true)
				if err != nil || !bytes.Equal(unpacked, payload) {
					t.Fatalf("payload round trip: %v", err)
				}
				ze := mustNewZstdEncoder()
				legacy := ze.EncodeAll(payload, nil)
				ze.Close()
				got, err := d.Decode(encoded, false)
				if err != nil {
					t.Fatal(err)
				}
				pixels := bytes.Clone(got.Pix)
				want, err := d.Decode(legacy, false)
				if err != nil || !bytes.Equal(pixels, want.Pix) {
					t.Fatalf("container changed decoded pixels: %v", err)
				}
				if got.Bounds().Dx() != img.Bounds().Dx() || got.Bounds().Dy() != img.Bounds().Dy() {
					t.Fatal("wrong dimensions")
				}
				e.Parallel, d.Parallel = false, false
				serial, err := e.Encode(img, q, bw)
				if err != nil || !bytes.Equal(encoded, serial) {
					t.Fatalf("serial encoder differs: %v", err)
				}
				got, err = d.Decode(serial, true)
				if err != nil {
					t.Fatal(err)
				}
				d.Parallel = true
				filtered := bytes.Clone(got.Pix)
				parallel, err := d.Decode(serial, true)
				if err != nil || !bytes.Equal(filtered, parallel.Pix) {
					t.Fatalf("postfilter decode: %v", err)
				}
			})
		}
	}
	if !sawRange {
		t.Fatal("test did not exercise the new container")
	}
}

func TestLegacyFile(t *testing.T) {
	for _, name := range []string{"legacy", "mixed"} {
		t.Run(name, func(t *testing.T) {
			// Produced by earlier encoders, not by the implementation under test.
			encoded, err := os.ReadFile("testdata/" + name + ".babe")
			if err != nil {
				t.Fatal(err)
			}
			f, err := os.Open("testdata/" + name + ".png")
			if err != nil {
				t.Fatal(err)
			}
			defer f.Close()
			want, _, err := image.Decode(f)
			if err != nil {
				t.Fatal(err)
			}
			expected := image.NewRGBA(want.Bounds())
			draw.Draw(expected, expected.Bounds(), want, want.Bounds().Min, draw.Src)
			d := NewDecoder()
			defer d.zdec.Close()
			got, err := d.Decode(encoded, false)
			if err != nil || !bytes.Equal(expected.Pix, got.Pix) {
				t.Fatalf("legacy pixels differ: %v", err)
			}
		})
	}
}

func TestParallelEntropyReuse(t *testing.T) {
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	// Large inputs cross the entropy-worker threshold. Alternating dimensions
	// and channel counts checks buffer reuse after both growth and shrinkage.
	for _, size := range []image.Point{{257, 129}, {17, 9}, {129, 257}} {
		img := image.NewRGBA(image.Rect(0, 0, size.X, size.Y))
		for y := 0; y < size.Y; y++ {
			for x := 0; x < size.X; x++ {
				img.SetRGBA(x, y, color.RGBA{byte(x*7 + y*3), byte(x*3 + y*5), byte(x ^ y), 255})
			}
		}
		for _, bw := range []bool{false, true} {
			e.Parallel, d.Parallel = true, true
			parallel, err := e.Encode(img, 80, bw)
			if err != nil {
				t.Fatal(err)
			}
			parallel = bytes.Clone(parallel)
			decoded, err := d.Decode(parallel, false)
			if err != nil {
				t.Fatal(err)
			}
			pixels := bytes.Clone(decoded.Pix)
			e.Parallel, d.Parallel = false, false
			serial, err := e.Encode(img, 80, bw)
			if err != nil || !bytes.Equal(serial, parallel) {
				t.Fatalf("serial/parallel encoding differs: %v", err)
			}
			decoded, err = d.Decode(serial, false)
			if err != nil || !bytes.Equal(pixels, decoded.Pix) {
				t.Fatalf("serial/parallel decoding differs: %v", err)
			}
		}
	}
}

func TestQuantizedBlocks(t *testing.T) {
	// Values on opposite sides of the pattern threshold round to the same
	// output color. They must become solid instead of retaining a useless mask.
	for _, size := range []int{1, 2, 3, 4, 6, 8} {
		plane := make([]byte, size*size)
		for i := range plane {
			plane[i] = byte(127 + i%2)
		}
		var buf bytes.Buffer
		writer := newBitWriter(&buf)
		fg, bg, pattern, err := encodeBlockPlane(plane, size, size, 0, 0, size, size, 4, &writer)
		writer.flush()
		if err != nil || pattern || fg != 128 || bg != 128 || buf.Len() != 0 {
			t.Fatalf("size=%d fg=%d bg=%d pattern=%v err=%v", size, fg, bg, pattern, err)
		}
	}
	for v := 0; v < 256; v++ {
		if quantizeLevel(byte(v), 1) != byte(v) {
			t.Fatal("step=1 changed a color")
		}
		for _, step := range []int{2, 4, 8} {
			diff := int(quantizeLevel(byte(v), step)) - v
			if diff < -step/2 || diff > step/2 {
				t.Fatal("rounding exceeded error bound")
			}
		}
	}
}

func TestCorruptContainer(t *testing.T) {
	e := NewEncoder()
	d := NewDecoder()
	defer d.zdec.Close()
	packed, err := e.Encode(compressionTestImage(), 10, false)
	if err != nil {
		t.Fatal(err)
	}
	packed, err = e.packer.compress(e.raw.Bytes(), nil, false)
	if err != nil {
		t.Fatal(err)
	}
	for _, n := range []int{0, 1, 3, 4, 18, 22, 25, 26, 30, 34, 38, len(packed) - 1} {
		if _, err := d.Decode(packed[:n], false); err == nil {
			t.Fatalf("accepted truncation at %d", n)
		}
	}
	for _, offset := range []int{4, 9, 11, 13, 14, 18, 22, 26, 30, 34, 38, len(packed) - 1} {
		bad := bytes.Clone(packed)
		bad[offset] ^= 0xff
		if _, err := d.Decode(bad, false); err == nil {
			t.Fatalf("accepted corruption at %d", offset)
		}
	}
	if _, err := d.Decode(append(bytes.Clone(packed), 0), false); err == nil {
		t.Fatal("accepted trailing bytes")
	}
	// Geometry must be checked before allocating a declared output stream.
	bad := bytes.Clone(packed)
	binary.BigEndian.PutUint32(bad[14:], ^uint32(0))
	if _, err := d.Decode(bad, false); err == nil {
		t.Fatal("accepted oversized image")
	}
	if _, err := d.Decode(packed, false); err != nil {
		t.Fatalf("decoder could not recover after errors: %v", err)
	}
}

func FuzzDecodeContainer(f *testing.F) {
	e := NewEncoder()
	_, err := e.Encode(compressionTestImage(), 10, false)
	if err != nil {
		f.Fatal(err)
	}
	packed, err := e.packer.compress(e.raw.Bytes(), nil, false)
	if err != nil {
		f.Fatal(err)
	}
	f.Add(packed)
	f.Add([]byte(packedMagic))
	f.Fuzz(func(t *testing.T, data []byte) {
		// Keep mutation-driven memory use small while exercising actual decoding.
		if len(data) < 22 || string(data[:4]) != packedMagic {
			return
		}
		w, h := binary.BigEndian.Uint32(data[14:]), binary.BigEndian.Uint32(data[18:])
		if w > 128 || h > 128 {
			return
		}
		d := NewDecoder()
		defer d.zdec.Close()
		_, _ = d.Decode(data, false)
	})
}
