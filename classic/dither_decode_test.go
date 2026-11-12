package classic

import (
	"bytes"
	"encoding/binary"
	"image"
	"image/color"
	"testing"
)

func TestPatternBlockMap(t *testing.T) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()
	oldSmall, oldMacro := smallBlock, macroBlock
	defer func() { smallBlock, macroBlock = oldSmall, oldMacro }()
	smallBlock, macroBlock = 4, 8
	// First group is an 8x8 pattern; second is split, with only its first
	// 4x4 child patterned.
	segment := make([]byte, 14)
	binary.BigEndian.PutUint32(segment[4:], 1)
	segment[8] = 0x80
	binary.BigEndian.PutUint32(segment[9:], 1)
	segment[13] = 0xc0
	blocks := NewDecoder().patternBlockMap(segment, 16, 8)
	for y := 0; y < 8; y++ {
		for x := 0; x < 16; x++ {
			want := byte(8 | 0x80)
			if x >= 8 {
				want = 4
				if x < 12 && y < 4 {
					want |= 0x80
				}
			}
			if got := blocks[y*16+x]; got != want {
				t.Fatalf("(%d,%d) got %x want %x", x, y, got, want)
			}
		}
	}
}

func TestDitherLumaKeepsLevelsAndStrongEdges(t *testing.T) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()
	oldSmall, oldMacro := smallBlock, macroBlock
	defer func() { smallBlock, macroBlock = oldSmall, oldMacro }()
	smallBlock, macroBlock = 4, 8
	segment := make([]byte, 14)
	binary.BigEndian.PutUint32(segment[4:], 1)
	segment[8] = 0x80
	binary.BigEndian.PutUint32(segment[9:], 1)
	segment[13] = 0x80
	for _, high := range []byte{100, 200} {
		plane := make([]byte, 64)
		for y := range 8 {
			for x := range 8 {
				if x < 4 {
					plane[y*8+x] = 80
				} else {
					plane[y*8+x] = high
				}
			}
		}
		original := bytes.Clone(plane)
		d := NewDecoder()
		d.Parallel = false
		d.ditherLuma(segment, 8, 8, func(x, y int) byte { return plane[y*8+x] }, func(x, y int, v byte) { plane[y*8+x] = v })
		for _, v := range plane {
			if v != 80 && v != high {
				t.Fatalf("introduced new level %d", v)
			}
		}
		if high == 100 && bytes.Equal(plane, original) {
			t.Fatal("moderate edge was not dithered")
		}
		if high == 200 && !bytes.Equal(plane, original) {
			t.Fatal("strong edge was modified")
		}
	}
}

func TestDitherStrengthTracksBlockSize(t *testing.T) {
	if ditherCenterWeight(4) != 8 || ditherCenterWeight(8) != 4 || ditherCenterWeight(16) != 2 {
		t.Fatal("larger blocks do not increase dither strength as intended")
	}
	codecStateMu.Lock()
	defer codecStateMu.Unlock()
	oldSmall, oldMacro := smallBlock, macroBlock
	defer func() { smallBlock, macroBlock = oldSmall, oldMacro }()
	smallBlock, macroBlock = 4, 8
	segment := make([]byte, 14)
	binary.BigEndian.PutUint32(segment[4:], 1)
	segment[8] = 0x80
	binary.BigEndian.PutUint32(segment[9:], 1)
	segment[13] = 0x80
	const pattern = "0101100001000101001010111010100100101000110100001000000100110100"
	plane := make([]byte, 64)
	for i, v := range pattern {
		if v == '1' {
			plane[i] = 100
		} else {
			plane[i] = 80
		}
	}
	original := bytes.Clone(plane)
	d := NewDecoder()
	d.Parallel = false
	d.ditherLuma(segment, 8, 8, func(x, y int) byte { return plane[y*8+x] }, func(x, y int, v byte) { plane[y*8+x] = v })
	changed := 0
	for i := range plane {
		if plane[i] != original[i] {
			changed++
		}
	}
	// The previous fixed-strength decoder changes seven pixels on this fixture.
	if changed != 10 {
		t.Fatalf("8x8 block: changed %d pixels, expected 10", changed)
	}
}

func TestDither2Decode(t *testing.T) {
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
			serial.Dither2, parallel.Dither2, serial.Parallel = true, true, false
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
				t.Fatalf("q%d %s serial/parallel differ: %v", q, mode, err)
			}
			if q == 100 && (!bytes.Equal(a.Pix, img.Pix) || !bytes.Equal(base.Pix, img.Pix)) {
				t.Fatal("lossless pixels changed")
			}
			if q == 80 && !bytes.Equal(a.Pix, base.Pix) {
				t.Fatal("2x2 blocks should not be dithered")
			}
			if q < 80 && !bytes.Equal(a.Pix, base.Pix) {
				changed = true
			}
		}
	}
	if !changed {
		t.Fatal("decoder dither did not change any lossy image")
	}
}
