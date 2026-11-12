package classic

import (
	"bytes"
	"encoding/binary"
	"hash/crc32"
	"image"
	"image/color"
	"testing"
)

func TestSparseCornersRoundTrip(t *testing.T) {
	for _, dims := range []image.Point{{1, 1}, {1, 17}, {17, 1}, {5, 7}, {32, 24}, {33, 25}} {
		img := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		for y := 0; y < dims.Y; y++ {
			for x := 0; x < dims.X; x++ {
				img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 7), uint8((x + y) * 3), 255})
			}
		}
		for _, q := range []int{0, 10, 80, 100} {
			for _, bw := range []bool{false, true} {
				if q == 100 && bw {
					continue
				}
				e := NewEncoder()
				e.SparseCorners = true
				e.CornerMinSize = 4
				data, err := e.Encode(img, q, bw)
				if err != nil {
					t.Fatalf("%v q%d: %v", dims, q, err)
				}
				data = bytes.Clone(data)
				if q != 100 && string(data[:4]) != sparseCornerMagic {
					t.Fatal("missing sparse corner format")
				}
				if q == 100 {
					standard, err := NewEncoder().Encode(img, 100, false)
					if err != nil || !bytes.Equal(data, standard) {
						t.Fatal("sparse mode changed lossless bitstream", err)
					}
				}
				a, b := NewDecoder(), NewDecoder()
				a.Parallel = false
				imA, err := a.Decode(data, false)
				if err != nil {
					t.Fatal(err)
				}
				imB, err := b.Decode(data, false)
				if err != nil || !bytes.Equal(imA.Pix, imB.Pix) {
					t.Fatalf("%v q%d: parallel mismatch: %v", dims, q, err)
				}
				if q == 100 && !bytes.Equal(imA.Pix, img.Pix) {
					t.Fatal("lossless mismatch")
				}
				if q != 100 {
					broken := bytes.Clone(data)
					broken[len(broken)-1] ^= 0x80
					if _, err := NewDecoder().Decode(broken, false); err == nil {
						t.Fatal("accepted damaged sparse stream")
					}
				}
			}
		}
	}
}

func TestSparseCornersRejectInvalidFlags(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 16, 16))
	for i := 3; i < len(img.Pix); i += 4 {
		img.Pix[i] = 255
	}
	e := NewEncoder()
	e.SparseCorners = true
	data, err := e.Encode(img, 10, false)
	if err != nil {
		t.Fatal(err)
	}
	baseLen := int(binary.BigEndian.Uint32(data[13:]))
	base := bytes.Clone(data[cornerHeaderSize : cornerHeaderSize+baseLen])
	cols, rows := (16-1+7)/8+1, (16-1+7)/8+1
	records := make([]byte, cols*rows*4)
	if err := decodeByteRangeInto(records, data[cornerHeaderSize+baseLen:]); err != nil {
		t.Fatal(err)
	}
	for _, mutate := range []func([]byte){func(b []byte) { b[0] = 8 }, func(b []byte) { b[0], b[1] = 0, 1 }} {
		altered := bytes.Clone(records)
		mutate(altered)
		packed := encodeByteRangeInto(nil, altered)
		broken := append(bytes.Clone(data[:cornerHeaderSize]), base...)
		binary.BigEndian.PutUint32(broken[17:], uint32(len(packed)))
		hash := crc32.NewIEEE()
		_, _ = hash.Write(base)
		_, _ = hash.Write(altered)
		binary.BigEndian.PutUint32(broken[21:], hash.Sum32())
		broken = append(broken, packed...)
		if _, err := NewDecoder().Decode(broken, false); err == nil {
			t.Fatal("accepted invalid sparse corner record")
		}
	}
}

func TestSparseCornersRespectBlockSize(t *testing.T) {
	img := image.NewRGBA(image.Rect(0, 0, 32, 32))
	for y := range 32 {
		for x := range 32 {
			img.SetRGBA(x, y, color.RGBA{uint8(x * 7), uint8(y * 7), uint8((x + y) * 3), 255})
		}
	}
	e := NewEncoder()
	e.SparseCorners, e.CornerMinSize = true, 9
	data, err := e.Encode(img, 10, false)
	if err != nil {
		t.Fatal(err)
	}
	_, grid, err := unpackSparseCornerImage(data)
	if err != nil {
		t.Fatal(err)
	}
	for _, flags := range grid.flags {
		if flags != 0 {
			t.Fatal("corner stored for a block below the minimum")
		}
	}
}
