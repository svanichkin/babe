package classic

import (
	"bytes"
	"encoding/binary"
	"hash/crc32"
	"image"
	"image/color"
	"testing"
)

func TestCompactGradientRejectsInvalidRecords(t *testing.T) {
	base := []byte("BRC1fake")
	wrap := func(raw []byte, count uint32) []byte {
		out := append([]byte(nil), compactSharedGradientMagic...)
		for _, n := range []uint32{64, 64, uint32(len(base)), uint32(len(raw)), count} {
			out = binary.BigEndian.AppendUint32(out, n)
		}
		hash := crc32.NewIEEE()
		hash.Write(base)
		hash.Write(raw)
		out = binary.BigEndian.AppendUint32(out, hash.Sum32())
		out = append(out, base...)
		return append(out, encodeByteRangeInto(nil, raw)...)
	}
	for _, tc := range []struct {
		raw   []byte
		count uint32
	}{
		{[]byte{5, 0, 0}, 1},
		{[]byte{0, 0x80, 0x80}, 1},
		{[]byte{0, 100, 0}, 1},
		{[]byte{0, 0, 0, 0, 0, 0}, 2},
	} {
		if _, _, _, err := unpackSharedGradients(wrap(tc.raw, tc.count)); err == nil {
			t.Fatalf("accepted invalid record %v", tc.raw)
		}
	}
}

func TestSharedGradientMixedSizeBoundary(t *testing.T) {
	const w, h = 48, 32
	plane := make([]byte, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			plane[y*w+x] = byte(70 + x + y/2)
		}
	}
	mesh := makeSharedGradientMesh(plane, w, h)
	blocks := []largeGradient{{x: 0, y: 0, size: 32}, {x: 32, y: 0, size: 16}, {x: 32, y: 16, size: 16}}
	used := sharedNodeMask(blocks, w, h)
	count := 0
	for _, yes := range used {
		if yes {
			count++
		}
	}
	if count != 11 {
		t.Fatalf("got %d shared vertices, want 11", count)
	}
	for y := 0; y < h; y++ {
		left := int(mesh.at(blocks[0], 31, y))
		b := blocks[1]
		if y >= 16 {
			b = blocks[2]
		}
		right := int(mesh.at(b, 32, y))
		if right-left < -2 || right-left > 3 {
			t.Fatalf("discontinuous mixed-size edge at y=%d: %d %d", y, left, right)
		}
	}
	for x := 32; x < 48; x++ {
		a, b := int(mesh.at(blocks[1], x, 15)), int(mesh.at(blocks[2], x, 16))
		if b-a < -2 || b-a > 3 {
			t.Fatalf("discontinuous edge at x=%d: %d %d", x, a, b)
		}
	}
	base := []byte("BRC1fake")
	data := packSharedGradients(base, blocks, mesh)
	decoded, gotBlocks, gotMesh, err := unpackSharedGradients(data)
	if err != nil || !bytes.Equal(decoded, base) || len(gotBlocks) != 3 {
		t.Fatalf("shared round trip failed: %v", err)
	}
	for i, yes := range used {
		if yes && gotMesh.values[i] != mesh.values[i] {
			t.Fatalf("vertex %d changed", i)
		}
	}
}

func TestSharedGradientRealEdge(t *testing.T) {
	const w, h = 64, 32
	plane := make([]byte, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			v := 40 + x/2 + y/2
			if x >= 32 {
				v += 80
			}
			plane[y*w+x] = byte(v)
		}
	}
	mesh := makeSharedGradientMesh(plane, w, h)
	blocks := selectSharedGradients(plane, w, h, 10, mesh)
	if len(blocks) == 0 {
		t.Fatal("no gradients selected away from the strong edge")
	}
	for _, b := range blocks {
		if (b.x < 32 && b.x+b.size == 32) || (b.x == 32) {
			t.Fatalf("gradient touching strong edge selected: %+v", b)
		}
	}
}

func TestSharedGradientCorruption(t *testing.T) {
	const w, h = 32, 32
	p := make([]byte, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			p[y*w+x] = byte(50 + x + y)
		}
	}
	m := makeSharedGradientMesh(p, w, h)
	blocks := []largeGradient{{x: 0, y: 0, size: 16}, {x: 16, y: 0, size: 16}}
	data := packSharedGradients([]byte("BRC1fake"), blocks, m)
	for _, index := range []int{4, 8, 12, 16, 20, 24, len(data) - 1} {
		bad := bytes.Clone(data)
		bad[index] ^= 0x40
		if _, _, _, err := unpackSharedGradients(bad); err == nil {
			t.Fatalf("accepted damaged byte %d", index)
		}
	}
	for n := 0; n < sharedGradientHeaderSize; n++ {
		if _, _, _, err := unpackSharedGradients(data[:n]); err == nil {
			t.Fatalf("accepted truncated header %d", n)
		}
	}
}

func TestVeryLargeGradientBlocks(t *testing.T) {
	for _, side := range []int{128, 256} {
		img := image.NewRGBA(image.Rect(0, 0, side, side))
		for y := 0; y < side; y++ {
			for x := 0; x < side; x++ {
				v := uint8(40 + x/4 + y/4)
				img.SetRGBA(x, y, color.RGBA{v, v, v, 255})
			}
		}
		for _, q := range []int{0, 10, 50} {
			e := NewEncoder()
			e.LargeGradients, e.SparseCorners, e.CornerMinSize = true, true, 5
			e.LegacySmooth = true
			data, err := e.Encode(img, q, false)
			if err != nil {
				t.Fatal(err)
			}
			if string(data[:4]) != compactSharedGradientMagic {
				t.Fatalf("%d q%d: expected shared gradient", side, q)
			}
			_, blocks, _, err := unpackSharedGradients(data)
			if err != nil {
				t.Fatal(err)
			}
			if len(blocks) != 1 || blocks[0].size != side {
				t.Fatalf("%d q%d: blocks=%+v", side, q, blocks)
			}
			serial, parallel := NewDecoder(), NewDecoder()
			serial.Smooth, parallel.Smooth, serial.Parallel = true, true, false
			a, err := serial.Decode(data, false)
			if err != nil {
				t.Fatal(err)
			}
			b, err := parallel.Decode(data, false)
			if err != nil || !bytes.Equal(a.Pix, b.Pix) {
				t.Fatalf("%d q%d parallel mismatch: %v", side, q, err)
			}
			for _, p := range []image.Point{{1, 1}, {side / 2, side / 2}, {side - 2, side - 2}} {
				got, _, _, _ := a.At(p.X, p.Y).RGBA()
				want := int(img.RGBAAt(p.X, p.Y).R)
				diff := int(got>>8) - want
				if diff < -4 || diff > 4 {
					t.Fatalf("%d q%d at %v: got %d want %d", side, q, p, got>>8, want)
				}
			}
		}
	}
}

func TestStitch256GradientUsesCorrectSize(t *testing.T) {
	const w, h = 512, 256
	plane := make([]byte, w*h)
	sizes := make([]byte, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			i := y*w + x
			sizes[i] = 255
			if x < 256 {
				plane[i] = 100
			} else {
				plane[i] = 116
			}
		}
	}
	out := stitchLargeGradients(plane, sizes, w, h, true)
	if out[128*w+255] <= 100 || out[128*w+256] >= 116 {
		t.Fatal("256x256 border was not stitched")
	}
	if out[128*w+254] <= 100 {
		t.Fatal("256x256 interior correction missing")
	}
	if out[128*w+128] != 100 {
		t.Fatal("256x256 center unexpectedly changed")
	}
}

func TestGradientTileMomentsExact(t *testing.T) {
	const w, h = 304, 272
	plane := make([]byte, w*h)
	var state uint32 = 123456789
	for i := range plane {
		state = state*1664525 + 1013904223
		plane[i] = byte(state >> 24)
	}
	sums := makeGradientTileSums(plane, w, h, false)
	for _, b := range []largeGradient{{x: 0, y: 0, size: 256}, {x: 16, y: 16, size: 128}, {x: 240, y: 192, size: 64}, {x: 272, y: 240, size: 32}, {x: 288, y: 256, size: 16}} {
		var want, wx, wy float64
		for y := 0; y < b.size; y++ {
			for x := 0; x < b.size; x++ {
				v := float64(plane[(b.y+y)*w+b.x+x])
				want += v
				wx += v * float64(2*x-(b.size-1))
				wy += v * float64(2*y-(b.size-1))
			}
		}
		got, gx, gy := sums.planeMoments(b.x, b.y, b.size)
		if got != want || gx != wx || gy != wy {
			t.Fatalf("%+v: got %v %v %v, want %v %v %v", b, got, gx, gy, want, wx, wy)
		}
	}
}

func TestLegacySharedGradientRecords(t *testing.T) {
	const w, h = 256, 256
	plane := make([]byte, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			plane[y*w+x] = byte(40 + x/4 + y/4)
		}
	}
	mesh := makeSharedGradientMesh(plane, w, h)
	blocks := []largeGradient{{x: 0, y: 0, size: 256}}
	used := sharedNodeMask(blocks, w, h)
	var raw []byte
	for _, b := range blocks {
		raw = binary.BigEndian.AppendUint32(raw, uint32(b.x))
		raw = binary.BigEndian.AppendUint32(raw, uint32(b.y))
		raw = append(raw, byte(b.size))
	}
	prev := byte(0)
	for i, yes := range used {
		if yes {
			v := mesh.values[i]
			raw = append(raw, v-prev)
			prev = v
		}
	}
	base := []byte("BRC1fake")
	data := append([]byte(nil), sharedGradientMagic...)
	for _, n := range []uint32{w, h, uint32(len(base)), uint32(len(raw)), uint32(len(blocks))} {
		data = binary.BigEndian.AppendUint32(data, n)
	}
	hash := crc32.NewIEEE()
	hash.Write(base)
	hash.Write(raw)
	data = binary.BigEndian.AppendUint32(data, hash.Sum32())
	data = append(data, base...)
	data = append(data, encodeByteRangeInto(nil, raw)...)
	decoded, got, decodedMesh, err := unpackSharedGradients(data)
	if err != nil || !bytes.Equal(decoded, base) || len(got) != 1 || got[0].size != 256 {
		t.Fatalf("legacy BGG2 decode: %v %+v", err, got)
	}
	for i, yes := range used {
		if yes && decodedMesh.values[i] != mesh.values[i] {
			t.Fatalf("legacy node %d changed", i)
		}
	}
}
