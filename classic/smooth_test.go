package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestSpreadGlyphResidual(t *testing.T) {
	grid := cornerGrid{width: 16, height: 8, step: 8, cols: 3, values: make([][3]byte, 6)}
	sizes := bytes.Repeat([]byte{8}, 16*8)
	luma := func(x, y int) byte {
		if x < 8 {
			return 100
		}
		return 116
	}
	for x, want := range map[int]int{3: 100, 4: 101, 5: 103, 6: 104, 7: 100} {
		got := spreadGlyphResidual(x, 3, 16, 8, 5, sizes, 100, luma, grid)
		if got != want {
			t.Fatalf("x=%d: got %d want %d", x, got, want)
		}
	}
	strong := func(x, y int) byte {
		if x < 8 {
			return 30
		}
		return 220
	}
	if got := spreadGlyphResidual(6, 3, 16, 8, 5, sizes, 30, strong, grid); got != 30 {
		t.Fatalf("strong edge: %d", got)
	}
	for i := range sizes {
		sizes[i] = 4
	}
	if got := spreadGlyphResidual(2, 3, 16, 8, 5, sizes, 100, luma, grid); got != 100 {
		t.Fatalf("small block changed: %d", got)
	}
}

func TestSmoothDecode(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 32, 24))
	for y := 0; y < 24; y++ {
		for x := 0; x < 32; x++ {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 9), uint8(x * y), 255})
		}
	}
	for _, q := range []int{0, 10, 80, 100} {
		for _, bw := range []bool{false, true} {
			if q == 100 && bw {
				continue
			}
			e := NewEncoder()
			e.SparseCorners, e.CornerMinSize = true, 5
			e.LegacySmooth = true
			data, err := e.Encode(img, q, bw)
			if err != nil {
				t.Fatal(err)
			}
			a, b := NewDecoder(), NewDecoder()
			a.Smooth, b.Smooth, a.Parallel = true, true, false
			imA, err := a.Decode(data, false)
			if err != nil {
				t.Fatalf("q%d bw%v: %v", q, bw, err)
			}
			imB, err := b.Decode(data, false)
			if err != nil || !bytes.Equal(imA.Pix, imB.Pix) {
				t.Fatalf("q%d bw%v: parallel mismatch: %v", q, bw, err)
			}
			if q == 80 {
				base, _, err := unpackSparseCornerImage(data)
				if err != nil {
					t.Fatal(err)
				}
				plain, err := NewDecoder().Decode(base, false)
				if err != nil || !bytes.Equal(imA.Pix, plain.Pix) {
					t.Fatalf("small blocks changed: %v", err)
				}
			}
			if q == 100 && !bytes.Equal(imA.Pix, img.Pix) {
				t.Fatal("lossless output changed")
			}
		}
	}
}

func TestSmoothPreservesSmallGlyphPixels(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 64, 64))
	for y := range 64 {
		for x := range 64 {
			v := uint8(80 + x/2 + y/3)
			if x >= 32 && (x+y)%2 == 0 {
				v = 240
			}
			img.SetRGBA(x, y, color.RGBA{v, uint8(int(v) * 3 / 4), uint8(int(v) / 2), 255})
		}
	}
	e := NewEncoder()
	e.SparseCorners, e.CornerMinSize = true, 5
	e.LegacySmooth = true
	data, err := e.Encode(img, 10, false)
	if err != nil {
		t.Fatal(err)
	}
	base, _, err := unpackSparseCornerImage(data)
	if err != nil {
		t.Fatal(err)
	}
	scratch := NewDecoder()
	payload, err := decompressPayload(base, nil, scratch.zdec, false)
	if err != nil {
		t.Fatal(err)
	}
	header, err := parsePayloadHeader(payload)
	if err != nil {
		t.Fatal(err)
	}
	pos := payloadHeaderSize
	seg, err := readChannelSegment(payload, &pos)
	if err != nil {
		t.Fatal(err)
	}
	codecStateMu.Lock()
	oldSmall, oldMacro := smallBlock, macroBlock
	smallBlock, macroBlock = header.small, header.macro
	sizes := bytes.Clone(scratch.glyphBlockSizes(seg, 64, 64))
	smallBlock, macroBlock = oldSmall, oldMacro
	codecStateMu.Unlock()
	plain, err := NewDecoder().Decode(base, false)
	if err != nil {
		t.Fatal(err)
	}
	d := NewDecoder()
	d.Smooth = true
	smooth, err := d.Decode(data, false)
	if err != nil {
		t.Fatal(err)
	}
	smallCount, largeCount := 0, 0
	for i, size := range sizes {
		if size <= 4 {
			smallCount++
			if !bytes.Equal(plain.Pix[i*4:i*4+4], smooth.Pix[i*4:i*4+4]) {
				t.Fatalf("small glyph pixel %d changed", i)
			}
		} else {
			largeCount++
		}
	}
	if smallCount == 0 || largeCount == 0 {
		t.Fatalf("expected mixed glyph sizes, got %d small and %d large", smallCount, largeCount)
	}
}
