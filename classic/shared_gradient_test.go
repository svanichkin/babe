package classic

import (
	"bytes"
	"testing"
)

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
