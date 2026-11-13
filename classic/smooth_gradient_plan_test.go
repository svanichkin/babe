package classic

import (
	"bytes"
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"image"
	"image/color"
	"os"
	"testing"
)

// Captured from the BSM1 encoder before implicit gradient glyphs were added:
// smoothTestImage(96,80), q10, SparseCorners/LargeGradients, CornerMinSize=5.
func TestSmoothReadsBSM1Gradients(t *testing.T) {
	data, err := os.ReadFile("testdata/bsm1-gradient.babe")
	if err != nil {
		t.Fatal(err)
	}
	d := NewDecoder()
	for _, tc := range []struct{ mode, hash string }{
		{"smooth", "1675db7a085ff1d0f04c0deefb7cdd5782fd0b192871982d9e0042505851e946"},
		{"raw", "7bf70268c5cb893bea2b1c075ee9672eaf54ff4cd1c0cea28ddebd7e7436ebe5"},
		{"glyphs", "bc371600ade492b5952123079469f0307f83ab168d49a499e04d53623095f8ff"},
	} {
		d.Smooth, d.Dither2, d.GlyphsOnly = true, tc.mode == "raw", tc.mode == "glyphs"
		img, err := d.Decode(data, false)
		if err != nil {
			t.Fatal(err)
		}
		if got := fmt.Sprintf("%x", sha256.Sum256(img.Pix)); got != tc.hash {
			t.Fatalf("%s: pixels changed: %s", tc.mode, got)
		}
	}
}

func TestSmoothSizeViewMatchesDenseMap(t *testing.T) {
	oldSmall, oldMacro := smallBlock, macroBlock
	t.Cleanup(func() { smallBlock, macroBlock = oldSmall, oldMacro })
	for _, sizes := range [][2]int{{1, 2}, {2, 4}, {4, 8}, {1, 8}} {
		smallBlock, macroBlock = sizes[0], sizes[1]
		for _, dims := range []image.Point{{1, 1}, {7, 5}, {16, 16}, {35, 29}} {
			g := newSmoothGeometry(dims.X, dims.Y, smallBlock, macroBlock)
			flags := make([]byte, g.groupsX*g.groupsY)
			for i := range flags {
				flags[i] = byte((i*13 + i/3) % 2)
			}
			v := smoothSizeView{geo: g, big: flags}
			dense := make([]byte, g.w*g.h)
			fillGlyphBlockSizes(dense, appendSizeSegment(nil, 0, flags), g.w, g.h)
			for y := -1; y <= g.h; y++ {
				for x := -1; x <= g.w; x++ {
					want := 0
					if x >= 0 && x < g.w && y >= 0 && y < g.h {
						want = int(dense[y*g.w+x])
					}
					if got := v.at(x, y); got != want {
						t.Fatalf("%v blocks %v at %d,%d: %d != %d", dims, sizes, x, y, got, want)
					}
				}
			}
		}
	}
}

// A fully covered band must not even read the source plane. This also checks
// 256-pixel leaves (64 macroblocks per run) and runs crossing band boundaries.
func TestSmoothGradientSkipsSourceAnalysis(t *testing.T) {
	for _, macro := range []int{4, 8} {
		g := newSmoothGeometry(768, 768, macro/2, macro)
		plane := bytes.Repeat([]byte{117}, g.w*g.h)
		mesh := makeSharedGradientMesh(plane, g.w, g.h)
		var gradients []largeGradient
		for y := 0; y < g.h; y += 256 {
			for x := 0; x < g.w; x += 256 {
				gradients = append(gradients, largeGradient{x: x, y: y, size: 256})
			}
		}
		plan := makeSmoothGradientPlan(g, gradients, mesh, 32)
		for _, rows := range g.bandRows {
			blocks, err := encodeSmoothBlocks(nil, g.w, g, rows, false, 32, 0, 0, plan, nil)
			if err != nil || len(blocks) != 0 {
				t.Fatalf("macro %d rows %v: %d blocks, %v", macro, rows, len(blocks), err)
			}
		}
	}
}

// Exercise mixed gradient/glyph neighbours, the raw rendering fallback, band
// boundaries and scratch reuse after changing both dimensions and gradients.
func TestSmoothImplicitGlyphsMatchLegacy(t *testing.T) {
	e, legacy := NewEncoder(), NewEncoder()
	legacy.LegacySmooth = true
	d, ref := NewDecoder(), NewDecoder()
	for _, dims := range []image.Point{{256, 256}, {772, 520}, {67, 53}, {128, 128}} {
		img := smoothTestImage(dims.X, dims.Y)
		for y := 0; y < dims.Y; y++ {
			width := dims.X / 2
			if dims.X == 256 {
				width = dims.X
			}
			for x := 0; x < width; x++ {
				v := byte(32 + 96*x/dims.X + 64*y/dims.Y)
				img.SetRGBA(x, y, color.RGBA{v, v, v, 255})
			}
		}
		for _, q := range []int{0, 10, 50} {
			for _, gradients := range []bool{true, false} {
				for _, enc := range []*Encoder{e, legacy} {
					enc.SparseCorners, enc.LargeGradients, enc.CornerMinSize = true, gradients, 5
				}
				modern, err := e.Encode(img, q, false)
				if err != nil {
					t.Fatal(err)
				}
				if string(modern[:4]) != smoothMagic || gradients && binary.BigEndian.Uint32(modern[31:]) == 0 {
					t.Fatalf("%v q%d: expected BSM2 with gradients=%v", dims, q, gradients)
				}
				old, err := legacy.Encode(img, q, false)
				if err != nil {
					t.Fatal(err)
				}
				for _, mode := range []string{"smooth", "raw", "glyphs"} {
					for _, dec := range []*Decoder{d, ref} {
						dec.Smooth, dec.Dither2, dec.GlyphsOnly = true, mode == "raw", mode == "glyphs"
					}
					got, err := d.Decode(modern, false)
					if err != nil {
						t.Fatal(err)
					}
					want, err := ref.Decode(old, false)
					if err != nil || !bytes.Equal(got.Pix, want.Pix) {
						t.Fatalf("%v q%d gradients=%v %s: pixels differ: %v", dims, q, gradients, mode, err)
					}
				}
			}
		}
	}
}

func TestSmoothRejectsInvalidGradientCount(t *testing.T) {
	e := NewEncoder()
	e.SparseCorners, e.LargeGradients, e.CornerMinSize = true, true, 5
	data, err := e.Encode(smoothTestImage(128, 128), 10, false)
	if err != nil {
		t.Fatal(err)
	}
	for _, n := range []uint32{0, 65, 1 << 31, ^uint32(0)} {
		t.Run(fmt.Sprint(n), func(t *testing.T) {
			bad := bytes.Clone(data)
			bad[26] |= 1
			binary.BigEndian.PutUint32(bad[31:], n)
			binary.BigEndian.PutUint32(bad[22:], crc32.ChecksumIEEE(bad[26:]))
			if _, err := NewDecoder().Decode(bad, false); err == nil {
				t.Fatal("accepted invalid count")
			}
		})
	}
}

func TestSmoothImplicitGlyphsRawEncoderFallback(t *testing.T) {
	img := smoothTestImage(128, 128)
	for _, q := range []int{0, 10, 50} {
		for _, simplify := range []bool{false, true} {
			var pixels [2][]byte
			for i := range pixels {
				e := NewEncoder()
				e.LegacySmooth = i != 0
				e.SparseCorners, e.LargeGradients, e.CornerMinSize = true, true, 5
				e.DitherPatterns, e.SimplifyPatterns = !simplify, simplify
				data, err := e.Encode(img, q, false)
				if err != nil {
					t.Fatal(err)
				}
				d := NewDecoder()
				d.Smooth = true
				out, err := d.Decode(data, false)
				if err != nil {
					t.Fatal(err)
				}
				pixels[i] = out.Pix
			}
			if !bytes.Equal(pixels[0], pixels[1]) {
				t.Fatalf("q%d simplify=%v: raw fallback pixels differ", q, simplify)
			}
		}
	}
}
