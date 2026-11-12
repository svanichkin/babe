package classic

import (
	"bytes"
	"encoding/binary"
	"hash/crc32"
	"math/rand"
	"testing"
)

func TestBackgroundContrast(t *testing.T) {
	rng := rand.New(rand.NewSource(283))
	for _, n := range []int{0, 1, 7, 8, 9, 255, 1024} {
		for mode := 0; mode < 3; mode++ {
			fg, types := make([]byte, n), make([]byte, (n+7)/8)
			var bg, want []byte
			var previousFG, previousBG byte
			for i := range fg {
				f, b := byte(rng.Intn(256)), byte(rng.Intn(256))
				fg[i] = f - previousFG
				previousFG = f
				if mode == 0 || (mode == 1 && rng.Intn(3) == 0) {
					types[i/8] |= 128 >> uint(i&7)
					bg = append(bg, b-previousBG)
					previousBG = b
					want = append(want, f-b)
				}
			}
			original := bytes.Clone(bg)
			if err := transformBackground(bg, fg, types, false); err != nil || !bytes.Equal(bg, want) {
				t.Fatalf("n=%d mode=%d forward: %v", n, mode, err)
			}
			if err := transformBackground(bg, fg, types, true); err != nil || !bytes.Equal(bg, original) {
				t.Fatalf("n=%d mode=%d inverse: %v", n, mode, err)
			}
		}
	}
	for _, inverse := range []bool{false, true} {
		for _, tc := range []struct{ bg, fg, types []byte }{
			{nil, []byte{1}, nil},
			{nil, []byte{1}, []byte{128}},
			{[]byte{1}, []byte{1}, []byte{0}},
		} {
			if err := transformBackground(tc.bg, tc.fg, tc.types, inverse); err == nil {
				t.Fatal("accepted inconsistent contrast lengths")
			}
		}
	}
}

// Force method 17 regardless of encoder heuristics, and compare the complete
// reconstructed payload (including its CRC) with the legacy representation.
func TestContrastPayload(t *testing.T) {
	for _, q := range []int{0, 10, 50, 100} {
		enc, dec := NewEncoder(), NewDecoder()
		defer dec.zdec.Close()
		if _, err := enc.Encode(compressionTestImage(), q, false); err != nil {
			t.Fatal(err)
		}
		raw := bytes.Clone(enc.raw.Bytes())
		h, err := parsePayloadHeader(raw)
		if err != nil {
			t.Fatal(err)
		}
		r := payloadReader{data: raw, pos: payloadHeaderSize}
		out := append([]byte(packedMagic), raw[:payloadHeaderSize]...)
		out = binary.BigEndian.AppendUint32(out, crc32.ChecksumIEEE(raw))
		badOffset := 0
		for c := 0; c < h.channels; c++ {
			blocks, _ := r.u32()
			out = binary.BigEndian.AppendUint32(out, uint32(blocks))
			var streams [5][]byte
			for s := range streams {
				n, _ := r.u32()
				streams[s], _ = r.take(n)
			}
			for s, src := range streams {
				packed, method := src, byte(methodRaw)
				if s == 4 {
					contrast := bytes.Clone(src)
					if err := transformBackground(contrast, streams[3], streams[1], false); err != nil {
						t.Fatal(err)
					}
					packed, method = encodeByteRangeInto(nil, contrast), methodContrastRange
				}
				out = binary.BigEndian.AppendUint32(out, uint32(len(src)))
				out = binary.BigEndian.AppendUint32(out, uint32(len(packed)))
				if c == 0 && s == 3 {
					badOffset = len(out)
				}
				out = append(out, method)
				out = append(out, packed...)
			}
		}
		for _, parallel := range []bool{false, true} {
			got, err := decompressPayload(out, nil, dec.zdec, parallel)
			if err != nil || !bytes.Equal(got, raw) {
				t.Fatalf("q=%d parallel=%v: %v", q, parallel, err)
			}
		}
		broken := bytes.Clone(out)
		broken[badOffset] = methodContrastRange
		if _, err := decompressPayload(broken, nil, dec.zdec, true); err == nil {
			t.Fatal("accepted contrast coding on foreground")
		}
		if _, err := decompressPayload(out[:len(out)-1], nil, dec.zdec, true); err == nil {
			t.Fatal("accepted truncated contrast payload")
		}
	}
}

func TestRangeSizeEstimate(t *testing.T) {
	rng := rand.New(rand.NewSource(1245))
	for _, n := range []int{0, 1, 5, 127, 1024, 32768, 65537} {
		for _, alphabet := range []int{1, 2, 15, 31, 32, 128, 256} {
			src := make([]byte, n)
			for i := range src {
				src[i] = byte(rng.Intn(alphabet))
				if i%3 != 0 {
					src[i] /= 4
				}
			}
			actual := float64(len(encodeByteRangeInto(nil, src)))
			estimate := estimateRangeBytes(src)
			// Model overhead dominates short streams. Large streams may differ by
			// a few termination/normalization bits per entropy block.
			tolerance := float64(4 * ((n + byteRangeBlock - 1) / byteRangeBlock))
			if estimate < actual-tolerance || estimate > actual+tolerance {
				t.Fatalf("n=%d alphabet=%d estimate=%.2f actual=%.0f", n, alphabet, estimate, actual)
			}
		}
	}
}
