package classic

import (
	"bytes"
	"image"
	"image/color"
	"math/rand"
	"testing"
)

func TestGlyphRareTonePreserved(t *testing.T) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()
	oldSmall, oldMacro, oldDither, oldPattern := smallBlock, macroBlock, encodeDither, encodePattern2
	defer func() {
		smallBlock, macroBlock, encodeDither, encodePattern2 = oldSmall, oldMacro, oldDither, oldPattern
	}()
	encodeDither, encodePattern2 = false, false
	for _, size := range []int{2, 4, 8} {
		if size == 8 {
			smallBlock, macroBlock = 4, 8
		} else {
			smallBlock, macroBlock = size, size
		}
		for _, high := range []byte{104, 128} {
			for _, rareHigh := range []bool{false, true} {
				for pos := 0; pos < size*size; pos++ {
					plane := bytes.Repeat([]byte{high}, size*size)
					if rareHigh {
						for i := range plane {
							plane[i] = 96
						}
						plane[pos] = high
					} else {
						plane[pos] = 96
					}
					var output bytes.Buffer
					writer := newBitWriter(&output)
					fg, bg, pattern, err := encodeBlockPlane(plane, size, size, 0, 0, size, size, 1, &writer)
					writer.flush()
					if err != nil || !pattern {
						t.Fatalf("%dx%d high=%d rareHigh=%v pos=%d: pattern=%v fg=%d bg=%d err=%v", size, size, high, rareHigh, pos, pattern, fg, bg, err)
					}
					got := make([]byte, len(plane))
					reader := newBitReader(output.Bytes())
					if err := drawBlockPlane(got, size, 0, 0, size, size, &reader, fg, bg); err != nil || !bytes.Equal(got, plane) {
						t.Fatalf("%dx%d rareHigh=%v pos=%d: reconstructed incorrectly: %v", size, size, rareHigh, pos, err)
					}
				}
			}
		}
	}
}

func TestGlyphPatternBitOrder(t *testing.T) {
	codecStateMu.Lock()
	defer codecStateMu.Unlock()
	oldSmall, oldMacro := smallBlock, macroBlock
	defer func() { smallBlock, macroBlock = oldSmall, oldMacro }()
	rng := rand.New(rand.NewSource(2411))
	for _, layout := range [][2]int{{2, 4}, {3, 6}, {4, 8}, {1, 8}} {
		smallBlock, macroBlock = layout[0], layout[1]
		for _, size := range []int{smallBlock, macroBlock} {
			for offset := uint8(0); offset < 8; offset++ {
				for i := 0; i < 100; i++ {
					mask := rng.Uint64() >> uint(64-size*size)
					var out bytes.Buffer
					w := newBitWriter(&out)
					w.writeBits(0, offset)
					writePatternComposite(&w, mask, size, size)
					w.flush()
					r := newBitReader(out.Bytes())
					if offset > 0 {
						if _, err := r.readBits(offset); err != nil {
							t.Fatal(err)
						}
					}
					got, err := readPatternComposite(&r, size, size)
					if err != nil || got != mask {
						t.Fatalf("layout %v size %d offset %d got %x want %x: %v", layout, size, offset, got, mask, err)
					}
				}
			}
		}
	}
}

func TestGlyphRarePixelImage(t *testing.T) {
	img := image.NewGray(image.Rect(0, 0, 8, 8))
	for i := range img.Pix {
		img.Pix[i] = 96
	}
	img.SetGray(3, 3, color.Gray{Y: 128})
	data, err := NewEncoder().Encode(img, 10, true)
	if err != nil {
		t.Fatal(err)
	}
	got, err := NewDecoder().Decode(data, false)
	if err != nil {
		t.Fatal(err)
	}
	if got.RGBAAt(3, 3).R != 128 || got.RGBAAt(0, 0).R != 96 {
		t.Fatalf("q10 lost two-tone detail: feature=%v background=%v", got.RGBAAt(3, 3), got.RGBAAt(0, 0))
	}
}
