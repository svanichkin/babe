package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
)

// BGC1 is an experimental wrapper: an ordinary BRC1 image plus a range-coded
// grid of source corner values. It is deliberately distinct from BRC1 so that
// older decoders reject it rather than silently reconstructing the wrong image.
const cornerMagic = "BGC1"
const cornerHeaderSize = 25

func packCornerImage(base []byte, g cornerGrid) []byte {
	delta := make([]byte, 0, len(g.values)*3)
	for i, v := range g.values {
		var prev [3]byte
		if i%g.cols != 0 {
			prev = g.values[i-1]
		}
		for c := range 3 {
			delta = append(delta, v[c]-prev[c])
		}
	}
	compressed := encodeByteRangeInto(nil, delta)
	out := make([]byte, 0, cornerHeaderSize+len(base)+len(compressed))
	out = append(out, cornerMagic...)
	out = append(out, byte(g.step))
	out = binary.BigEndian.AppendUint32(out, uint32(g.width))
	out = binary.BigEndian.AppendUint32(out, uint32(g.height))
	out = binary.BigEndian.AppendUint32(out, uint32(len(base)))
	out = binary.BigEndian.AppendUint32(out, uint32(len(compressed)))
	hash := crc32.NewIEEE()
	_, _ = hash.Write(base)
	_, _ = hash.Write(delta)
	out = binary.BigEndian.AppendUint32(out, hash.Sum32())
	out = append(out, base...)
	return append(out, compressed...)
}

func unpackCornerImage(data []byte) ([]byte, *cornerGrid, error) {
	if len(data) < cornerHeaderSize || string(data[:4]) != cornerMagic {
		return nil, nil, fmt.Errorf("invalid corner header")
	}
	step := int(data[4])
	w, h := int(binary.BigEndian.Uint32(data[5:])), int(binary.BigEndian.Uint32(data[9:]))
	baseLen, cornerLen := uint64(binary.BigEndian.Uint32(data[13:])), uint64(binary.BigEndian.Uint32(data[17:]))
	if err := checkImageSize(w, h); err != nil {
		return nil, nil, err
	}
	if step < 2 || step > 8 || baseLen < 4 || baseLen+cornerLen != uint64(len(data)-cornerHeaderSize) {
		return nil, nil, fmt.Errorf("invalid corner container lengths or step")
	}
	cols, rows := (w-1+step-1)/step+1, (h-1+step-1)/step+1
	if uint64(cols)*uint64(rows)*3 > maxPayloadSize {
		return nil, nil, fmt.Errorf("corner grid exceeds limit")
	}
	base := data[cornerHeaderSize : cornerHeaderSize+int(baseLen)]
	if string(base[:4]) != packedMagic {
		return nil, nil, fmt.Errorf("corner container requires BRC1 base")
	}
	delta := make([]byte, cols*rows*3)
	if err := decodeByteRangeInto(delta, data[cornerHeaderSize+int(baseLen):]); err != nil {
		return nil, nil, fmt.Errorf("invalid corner stream: %w", err)
	}
	hash := crc32.NewIEEE()
	_, _ = hash.Write(base)
	_, _ = hash.Write(delta)
	if hash.Sum32() != binary.BigEndian.Uint32(data[21:]) {
		return nil, nil, fmt.Errorf("corner checksum mismatch")
	}
	g := &cornerGrid{width: w, height: h, step: step, cols: cols, values: make([][3]byte, cols*rows)}
	for i := range g.values {
		var prev [3]byte
		if i%cols != 0 {
			prev = g.values[i-1]
		}
		for c := range 3 {
			g.values[i][c] = prev[c] + delta[i*3+c]
		}
	}
	return base, g, nil
}
