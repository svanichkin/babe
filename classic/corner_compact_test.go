package classic

import (
	"bytes"
	"encoding/binary"
	"image"
	"image/color"
	"testing"
)

func TestSplitCornerRecords(t *testing.T) {
	for _, cols := range []int{1, 8, 19} {
		n := cols * 7
		records := make([]byte, n*4)
		for i := 0; i < n; i++ {
			bits := byte((i/3 + i/cols) % 8)
			records[i*4] = bits
			for c := 0; c < 3; c++ {
				if bits&(1<<c) != 0 {
					records[i*4+c+1] = byte(i*37 + c*41)
				}
			}
		}
		encoded := encodeSplitCornerRecords(records, cols)
		decoded := make([]byte, len(records))
		if err := decodeSplitCornerRecords(decoded, encoded, cols); err != nil || !bytes.Equal(records, decoded) {
			t.Fatalf("cols=%d: records changed: %v", cols, err)
		}
		for i := 0; i < 16; i++ {
			if err := decodeSplitCornerRecords(decoded, encoded[:i], cols); err == nil {
				t.Fatalf("accepted truncated header %d", i)
			}
		}
		broken := bytes.Clone(encoded)
		broken[3] ^= 0x7f
		if err := decodeSplitCornerRecords(decoded, broken, cols); err == nil {
			t.Fatal("accepted invalid lengths")
		}
	}
}

func TestRedundantGradientCorners(t *testing.T) {
	grid := cornerGrid{width: 48, height: 32, step: 8, cols: 7, values: make([][3]byte, 7*5)}
	// Only x<32 is a gradient: nodes on the right edge still influence glyphs.
	mask := redundantGradientCorners(48, 32, grid, []largeGradient{{x: 0, y: 0, size: 32}})
	if !mask[2*grid.cols+2] {
		t.Fatal("interior node not removed")
	}
	if mask[2*grid.cols+4] {
		t.Fatal("boundary node that influences glyphs removed")
	}
	if redundantGradientCorners(48, 32, grid, nil) != nil {
		t.Fatal("unexpected omissions without gradients")
	}
}

// Repack the same base and source corners both ways: omitted luma nodes must
// not change RGB pixels, including next to gradients and partial image edges.
func TestGradientCornerElisionPreservesPixels(t *testing.T) {
	for _, dims := range []image.Point{{128, 64}, {129, 65}, {80, 48}} {
		img := image.NewRGBA(image.Rectangle{Max: dims})
		for y := 0; y < dims.Y; y++ {
			for x := 0; x < dims.X; x++ {
				v := uint8(40 + (x+y)/3)
				if x > dims.X/2 && (x+y)%3 == 0 {
					v = 220
				}
				img.SetRGBA(x, y, color.RGBA{v, uint8(int(v) * 3 / 4), uint8(int(v) / 2), 255})
			}
		}
		for _, q := range []int{0, 10, 50} {
			e := NewEncoder()
			e.LargeGradients = true
			e.SparseCorners = true
			e.CornerMinSize = 5
			e.LegacySmooth = true
			data, err := e.Encode(img, q, false)
			if err != nil {
				t.Fatal(err)
			}
			if string(data[:4]) != compactSharedGradientMagic {
				t.Fatalf("%v q%d: expected gradients", dims, q)
			}
			wrapped, blocks, mesh, err := unpackSharedGradients(data)
			if err != nil {
				t.Fatal(err)
			}
			base, _, err := unpackSparseCornerImage(wrapped)
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
			w, h := dims.X, dims.Y
			ys, cb, cr := make([]byte, w*h), make([]byte, w*h), make([]byte, w*h)
			extractYCbCrPlanesIntoSerial(img, ys, cb, cr)
			grid := makeCornerGrid(w, h, 8, func(x, y int) [3]byte { i := y*w + x; return [3]byte{ys[i], cb[i], cr[i]} })
			codecStateMu.Lock()
			oldSmall, oldMacro := smallBlock, macroBlock
			smallBlock, macroBlock = header.small, header.macro
			full, err1 := packSparseCorners(base, payload, grid, 5, nil, false)
			elided, err2 := packSparseCorners(base, payload, grid, 5, blocks, false)
			smallBlock, macroBlock = oldSmall, oldMacro
			codecStateMu.Unlock()
			if err1 != nil || err2 != nil {
				t.Fatalf("repack: %v %v", err1, err2)
			}
			for _, smooth := range []bool{false, true} {
				a, b := NewDecoder(), NewDecoder()
				a.Smooth, b.Smooth = smooth, smooth
				ia, ea := a.Decode(packSharedGradients(full, blocks, mesh), false)
				ib, eb := b.Decode(packSharedGradients(elided, blocks, mesh), false)
				if ea != nil || eb != nil || !bytes.Equal(ia.Pix, ib.Pix) {
					t.Fatalf("%v q%d smooth=%v: omitted corners changed pixels: %v %v", dims, q, smooth, ea, eb)
				}
			}
		}
	}
}

func TestSplitAndLegacyCornersDecodeIdentically(t *testing.T) {
	const w, h = 128, 128
	img := image.NewRGBA(image.Rect(0, 0, w, h))
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 2), uint8(y * 2), uint8(x + y), 255})
		}
	}
	for _, bw := range []bool{false, true} {
		e := NewEncoder()
		e.SparseCorners = true
		e.CornerMinSize = 5
		e.LegacySmooth = true
		data, err := e.Encode(img, 0, bw)
		if err != nil {
			t.Fatal(err)
		}
		if string(data[:4]) != splitCornerMagic {
			t.Fatalf("bw=%v: expected split format, got %s", bw, data[:4])
		}
		base, grid, err := unpackSparseCornerImage(data)
		if err != nil {
			t.Fatal(err)
		}
		records := make([]byte, len(grid.values)*4)
		for i, v := range grid.values {
			records[i*4] = grid.flags[i]
			copy(records[i*4+1:i*4+4], v[:])
		}
		packed := encodeByteRangeInto(nil, records)
		header := bytes.Clone(data[:cornerHeaderSize])
		copy(header[:4], sparseCornerMagic)
		binary.BigEndian.PutUint32(header[17:], uint32(len(packed)))
		old := append(header, base...)
		old = append(old, packed...)
		for _, smooth := range []bool{false, true} {
			a, b := NewDecoder(), NewDecoder()
			a.Smooth, b.Smooth = smooth, smooth
			ia, ea := a.Decode(data, false)
			ib, eb := b.Decode(old, false)
			if ea != nil || eb != nil || !bytes.Equal(ia.Pix, ib.Pix) {
				t.Fatalf("bw=%v smooth=%v: different pixels %v %v", bw, smooth, ea, eb)
			}
		}
	}
}

func TestSparseCornersParallelEncoderMatchesSerial(t *testing.T) {
	for _, dims := range []image.Point{{128, 128}, {161, 129}} {
		img := image.NewRGBA(image.Rectangle{Max: dims})
		for y := 0; y < dims.Y; y++ {
			for x := 0; x < dims.X; x++ {
				img.SetRGBA(x, y, color.RGBA{uint8(x*7 + y), uint8(y*5 + x/3), uint8(x * y), 255})
			}
		}
		for _, q := range []int{0, 10, 50, 80} {
			for _, grad := range []bool{false, true} {
				serial, parallel := NewEncoder(), NewEncoder()
				serial.Parallel = false
				serial.SparseCorners, parallel.SparseCorners = true, true
				serial.CornerMinSize, parallel.CornerMinSize = 5, 5
				serial.LargeGradients, parallel.LargeGradients = grad, grad
				a, err := serial.Encode(img, q, false)
				if err != nil {
					t.Fatal(err)
				}
				b, err := parallel.Encode(img, q, false)
				if err != nil {
					t.Fatal(err)
				}
				if !bytes.Equal(a, b) {
					t.Fatalf("%v q%d gradients=%v: serial and parallel bitstreams differ", dims, q, grad)
				}
			}
		}
	}
}
