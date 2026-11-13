package classic

import (
	"bytes"
	"encoding/binary"
	"hash/crc32"
	"image"
	"image/color"
	"testing"
)

func smoothTestImage(w, h int) *image.RGBA {
	img := image.NewRGBA(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			// A smooth ramp, a textured half and a sharp diagonal edge.
			v := 30 + x*150/max(w, 1) + y*50/max(h, 1)
			if x > w/2 && (x*7+y*3)%11 < 4 {
				v += 60
			}
			if x+y < w/3 {
				v = 230
			}
			v = min(v, 255)
			img.SetRGBA(x, y, color.RGBA{uint8(v), uint8(v * 3 / 4), uint8(255 - v/2), 255})
		}
	}
	return img
}

// BSM2 must decode to exactly the pixels of the legacy BGG3/BGC wrappers.
func TestSmoothContainerMatchesLegacy(t *testing.T) {
	for _, dims := range []image.Point{{1, 1}, {7, 5}, {33, 17}, {64, 64}, {130, 66}, {320, 200}} {
		img := smoothTestImage(dims.X, dims.Y)
		for _, q := range []int{0, 3, 10, 20, 30, 50, 80} {
			for _, bw := range []bool{false, true} {
				for _, opts := range [][2]bool{{true, true}, {true, false}, {false, true}} {
					encode := func(legacy bool) []byte {
						e := NewEncoder()
						e.SparseCorners, e.LargeGradients, e.CornerMinSize = opts[0], opts[1], 5
						e.LegacySmooth = legacy
						data, err := e.Encode(img, q, bw)
						if err != nil {
							t.Fatalf("%v q%d: %v", dims, q, err)
						}
						return bytes.Clone(data)
					}
					modern, legacy := encode(false), encode(true)
					if string(modern[:4]) != smoothMagic && q < 70 {
						t.Fatalf("%v q%d: got %q, want BSM2", dims, q, modern[:4])
					}
					for _, smooth := range []bool{false, true} {
						if smooth && !opts[0] {
							continue
						}
						a, b := NewDecoder(), NewDecoder()
						a.Smooth, b.Smooth = smooth, smooth
						a.Parallel = false
						got, err := a.Decode(modern, false)
						if err != nil {
							t.Fatalf("%v q%d bw=%v %v: %v", dims, q, bw, opts, err)
						}
						want, err := b.Decode(legacy, false)
						if err != nil {
							t.Fatal(err)
						}
						if !bytes.Equal(got.Pix, want.Pix) {
							t.Fatalf("%v q%d bw=%v %v smooth=%v: pixels differ", dims, q, bw, opts, smooth)
						}
					}
				}
			}
		}
	}
}

func TestSmoothContainerParallelMatchesSerial(t *testing.T) {
	img := smoothTestImage(1100, 700)
	for _, q := range []int{0, 10, 30} {
		serial, parallel := NewEncoder(), NewEncoder()
		serial.Parallel = false
		for _, e := range []*Encoder{serial, parallel} {
			e.SparseCorners, e.LargeGradients, e.CornerMinSize = true, true, 5
		}
		a, err := serial.Encode(img, q, false)
		if err != nil {
			t.Fatal(err)
		}
		a = bytes.Clone(a)
		b, err := parallel.Encode(img, q, false)
		if err != nil || !bytes.Equal(a, b) {
			t.Fatalf("q%d: parallel encoding differs: %v", q, err)
		}
		d1, d2 := NewDecoder(), NewDecoder()
		d1.Parallel, d1.Smooth, d2.Smooth = false, true, true
		p1, err1 := d1.Decode(a, false)
		p2, err2 := d2.Decode(a, false)
		if err1 != nil || err2 != nil || !bytes.Equal(p1.Pix, p2.Pix) {
			t.Fatalf("q%d: parallel decoding differs: %v %v", q, err1, err2)
		}
	}
}

func TestSmoothContainerRejectsDamage(t *testing.T) {
	img := smoothTestImage(96, 80)
	e := NewEncoder()
	e.SparseCorners, e.LargeGradients, e.CornerMinSize = true, true, 5
	data, err := e.Encode(img, 5, false)
	if err != nil {
		t.Fatal(err)
	}
	data = bytes.Clone(data)
	for i := 4; i < len(data); i += max(1, len(data)/97) {
		broken := bytes.Clone(data)
		broken[i] ^= 0x10
		if _, err := NewDecoder().Decode(broken, false); err == nil {
			t.Fatalf("accepted damage at byte %d", i)
		}
	}
	if _, err := NewDecoder().Decode(data[:len(data)-1], false); err == nil {
		t.Fatal("accepted truncated data")
	}
	// With a matching checksum, damaged streams must still fail cleanly.
	seed := uint32(1)
	for it := 0; it < 2000; it++ {
		broken := bytes.Clone(data)
		for k := 0; k < 3; k++ {
			seed = seed*1664525 + 1013904223
			broken[26+int(seed>>8)%(len(broken)-26)] ^= byte(seed>>24 | 1)
		}
		binary.BigEndian.PutUint32(broken[22:], crc32.ChecksumIEEE(broken[26:]))
		d := NewDecoder()
		d.Smooth = it%2 == 0
		_, _ = d.Decode(broken, false)
	}
}
