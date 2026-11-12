package classic

import (
	"bytes"
	"image"
	"image/color"
	"testing"
)

func TestDither3Threshold(t *testing.T) {
	const w, h = 8, 8
	plane := make([]byte, w*h)
	for y := range h {
		for x := range w {
			v := byte(100)
			if x >= 3 {
				v = 120
			}
			if x >= 6 {
				v = 200
			}
			plane[y*w+x] = v
		}
	}
	sizes := bytes.Repeat([]byte{8}, len(plane))
	d := NewDecoder()
	d.Parallel = false
	d.blurLuma(w, h, sizes, func(x, y int) byte { return plane[y*w+x] }, func(x, y int, v byte) { plane[y*w+x] = v })
	if plane[3*w+2] <= 100 || plane[3*w+3] >= 120 {
		t.Fatalf("small transition was not softened: %v", plane)
	}
	if plane[3*w+6] != 200 || plane[3*w+7] != 200 {
		t.Fatalf("strong edge was modified: %v", plane)
	}
	flat := []byte{73, 73, 73, 73}
	d.blurLuma(2, 2, bytes.Repeat([]byte{8}, 4), func(x, y int) byte { return flat[y*2+x] }, func(x, y int, v byte) { flat[y*2+x] = v })
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
			if q == 80 && !bytes.Equal(a.Pix, base.Pix) {
				t.Fatal("blur affected blocks smaller than 8x8")
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

func TestDither3OnlyLargeBlocks(t *testing.T) {
	const w, h = 16, 8
	plane, sizes := make([]byte, w*h), make([]byte, w*h)
	for y := range h {
		for x := range w {
			i := y*w + x
			if x < 8 {
				sizes[i] = 8
			} else {
				sizes[i] = 4
			}
			if x%8 < 4 {
				plane[i] = 100
			} else {
				plane[i] = 120
			}
		}
	}
	original := bytes.Clone(plane)
	d := NewDecoder()
	d.Parallel = false
	d.blurLuma(w, h, sizes, func(x, y int) byte { return plane[y*w+x] }, func(x, y int, v byte) { plane[y*w+x] = v })
	changed := false
	for y := range h {
		for x := range w {
			i := y*w + x
			if x < 8 && (x == 0 || x == 7 || y == 0 || y == 7) && plane[i] != original[i] {
				t.Fatalf("8x8 block edge changed at %d,%d", x, y)
			}
			if x < 8 && plane[i] != original[i] {
				changed = true
			}
			if x >= 8 && plane[i] != original[i] {
				t.Fatalf("4x4 side changed at %d,%d", x, y)
			}
		}
	}
	if !changed {
		t.Fatal("8x8 block was not blurred")
	}
	strong := make([]byte, 64)
	for y := range 8 {
		for x := 4; x < 8; x++ {
			strong[y*8+x] = 255
		}
	}
	before := bytes.Clone(strong)
	d.blurLuma(8, 8, bytes.Repeat([]byte{8}, 64), func(x, y int) byte { return strong[y*8+x] }, func(x, y int, v byte) { strong[y*8+x] = v })
	if !bytes.Equal(before, strong) {
		t.Fatalf("blur crossed strong edge: %v", strong)
	}
}

func TestDither3TapersTowardGlyphEdges(t *testing.T) {
	const size = 8
	plane := make([]byte, size*size)
	for y := range size {
		for x := range size {
			if x < 4 {
				plane[y*size+x] = 100
			} else {
				plane[y*size+x] = 132
			}
		}
	}
	d := NewDecoder()
	d.Parallel = false
	d.blurLuma(size, size, bytes.Repeat([]byte{8}, len(plane)), func(x, y int) byte { return plane[y*size+x] }, func(x, y int, v byte) { plane[y*size+x] = v })
	near, middle := int(plane[1*size+3])-100, int(plane[3*size+3])-100
	if near <= 0 || near >= middle {
		t.Fatalf("blur does not taper toward the fixed block edge: near=%d middle=%d", near, middle)
	}
	if plane[3] != 100 {
		t.Fatal("block edge changed")
	}
}

func TestDither3Softens32LevelGlyphStep(t *testing.T) {
	const size = 8
	plane := make([]byte, size*size)
	for y := range size {
		for x := range size {
			if x < 4 {
				plane[y*size+x] = 96
			} else {
				plane[y*size+x] = 128
			}
		}
	}
	d := NewDecoder()
	d.Parallel = false
	d.blurLuma(size, size, bytes.Repeat([]byte{8}, len(plane)), func(x, y int) byte { return plane[y*size+x] }, func(x, y int, v byte) { plane[y*size+x] = v })
	if plane[3*size+3] <= 96 || plane[3*size+4] >= 128 {
		t.Fatalf("32-level step was not blurred: %v", plane[3*size:4*size])
	}
}

func TestDither3PreservesBlockBoundaries(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 64, 64))
	for y := range 64 {
		for x := range 64 {
			v := uint8((x*3 + y*2) % 256)
			img.SetRGBA(x, y, color.RGBA{v, v, v, 255})
		}
	}
	e := NewEncoder()
	e.SparseCorners = true
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
	ySeg, err := readChannelSegment(payload, &pos)
	if err != nil {
		t.Fatal(err)
	}
	codecStateMu.Lock()
	oldSmall, oldMacro := smallBlock, macroBlock
	smallBlock, macroBlock = header.small, header.macro
	sizes := bytes.Clone(scratch.glyphBlockSizes(ySeg, 64, 64))
	smallBlock, macroBlock = oldSmall, oldMacro
	codecStateMu.Unlock()
	plain, blurred := NewDecoder(), NewDecoder()
	plain.GlyphBlend, plain.GlyphMinSize = true, 4
	blurred.GlyphBlend, blurred.GlyphMinSize, blurred.Dither3 = true, 4, true
	a, err := plain.Decode(data, false)
	if err != nil {
		t.Fatal(err)
	}
	b, err := blurred.Decode(data, false)
	if err != nil {
		t.Fatal(err)
	}
	for y := range 64 {
		for x := range 64 {
			size := int(sizes[y*64+x])
			if size == 0 {
				continue
			}
			if x%size != 0 && x%size != size-1 && y%size != 0 && y%size != size-1 {
				continue
			}
			o := y*a.Stride + x*4
			if !bytes.Equal(a.Pix[o:o+4], b.Pix[o:o+4]) {
				t.Fatalf("boundary changed at %d,%d (block %d)", x, y, size)
			}
		}
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
