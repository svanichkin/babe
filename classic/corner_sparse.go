package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
)

// BGC2 stores a channel mask and only the selected source corner values.
const sparseCornerMagic = "BGC2"

func packSparseCorners(base, raw []byte, source cornerGrid, minSize int) ([]byte, error) {
	w, h := source.width, source.height
	mask := raw[9]
	pos := payloadHeaderSize
	var segments [3][]byte
	for c := range 3 {
		if mask&(1<<c) == 0 {
			continue
		}
		var err error
		segments[c], err = readChannelSegment(raw, &pos)
		if err != nil {
			return nil, err
		}
	}
	baseline := cornerGrid{width: w, height: h, step: source.step, cols: source.cols, values: make([][3]byte, len(source.values))}
	for i := range baseline.values {
		baseline.values[i][1], baseline.values[i][2] = 128, 128
	}
	flags := make([]byte, len(source.values))
	scratch := &Decoder{}
	var channelScratch decoderChannelScratch
	for c := range 3 {
		if segments[c] == nil {
			continue
		}
		var plane []byte
		if smallBlock == 1 && macroBlock == 2 {
			if cap(channelScratch.plane) < w*h {
				channelScratch.plane = make([]byte, w*h)
			} else {
				channelScratch.plane = channelScratch.plane[:w*h]
			}
			plane = channelScratch.plane
			decodeValidatedQuadPlane(segments[c], plane, w, h)
		} else {
			var err error
			plane, err = scratch.decodeChannelInto(segments[c], w, h, &channelScratch)
			if err != nil {
				return nil, err
			}
		}
		sizes := scratch.glyphBlockSizes(segments[c], w, h)
		for i, v := range source.values {
			x, y := min((i%source.cols)*source.step, w-1), min((i/source.cols)*source.step, h-1)
			eligible := false
			sum, count := 0, 0
			for yy := max(0, y-1); yy <= y; yy++ {
				for xx := max(0, x-1); xx <= x; xx++ {
					sum += int(plane[yy*w+xx])
					count++
					if int(sizes[yy*w+xx]) >= minSize {
						eligible = true
					}
				}
			}
			baseline.values[i][c] = byte((sum + count/2) / count)
			diff := int(v[c]) - int(baseline.values[i][c])
			if eligible && (diff < -3 || diff > 3) {
				flags[i] |= 1 << c
			}
		}
	}
	// One flag and three residual bytes per node. Zeros at omitted channels
	// compress well and avoid trusting an attacker-controlled raw length.
	records := make([]byte, len(flags)*4)
	for i, bits := range flags {
		records[i*4] = bits
		for c := range 3 {
			if bits&(1<<c) != 0 {
				records[i*4+1+c] = source.values[i][c] - baseline.values[i][c]
			}
		}
	}
	compressed := encodeByteRangeInto(nil, records)
	out := make([]byte, 0, cornerHeaderSize+len(base)+len(compressed))
	out = append(out, sparseCornerMagic...)
	out = append(out, byte(source.step))
	out = binary.BigEndian.AppendUint32(out, uint32(w))
	out = binary.BigEndian.AppendUint32(out, uint32(h))
	out = binary.BigEndian.AppendUint32(out, uint32(len(base)))
	out = binary.BigEndian.AppendUint32(out, uint32(len(compressed)))
	hash := crc32.NewIEEE()
	_, _ = hash.Write(base)
	_, _ = hash.Write(records)
	out = binary.BigEndian.AppendUint32(out, hash.Sum32())
	out = append(out, base...)
	return append(out, compressed...), nil
}

func unpackSparseCornerImage(data []byte) ([]byte, *cornerGrid, error) {
	if len(data) < cornerHeaderSize || string(data[:4]) != sparseCornerMagic {
		return nil, nil, fmt.Errorf("invalid sparse corner header")
	}
	step := int(data[4])
	w, h := int(binary.BigEndian.Uint32(data[5:])), int(binary.BigEndian.Uint32(data[9:]))
	baseLen, cornerLen := uint64(binary.BigEndian.Uint32(data[13:])), uint64(binary.BigEndian.Uint32(data[17:]))
	if err := checkImageSize(w, h); err != nil {
		return nil, nil, err
	}
	if step != 8 || baseLen < 4 || baseLen+cornerLen != uint64(len(data)-cornerHeaderSize) {
		return nil, nil, fmt.Errorf("invalid sparse corner lengths or step")
	}
	cols, rows := (w-1+step-1)/step+1, (h-1+step-1)/step+1
	if uint64(cols)*uint64(rows)*4 > maxPayloadSize {
		return nil, nil, fmt.Errorf("corner grid exceeds limit")
	}
	base := data[cornerHeaderSize : cornerHeaderSize+int(baseLen)]
	if string(base[:4]) != packedMagic {
		return nil, nil, fmt.Errorf("sparse corners require BRC1 base")
	}
	records := make([]byte, cols*rows*4)
	if err := decodeByteRangeInto(records, data[cornerHeaderSize+int(baseLen):]); err != nil {
		return nil, nil, fmt.Errorf("invalid corner stream: %w", err)
	}
	hash := crc32.NewIEEE()
	_, _ = hash.Write(base)
	_, _ = hash.Write(records)
	if hash.Sum32() != binary.BigEndian.Uint32(data[21:]) {
		return nil, nil, fmt.Errorf("corner checksum mismatch")
	}
	g := &cornerGrid{width: w, height: h, step: step, cols: cols, values: make([][3]byte, cols*rows), flags: make([]byte, cols*rows)}
	for i := range g.values {
		g.flags[i] = records[i*4]
		if g.flags[i]&^byte(7) != 0 {
			return nil, nil, fmt.Errorf("invalid corner flags")
		}
		for c := range 3 {
			v := records[i*4+1+c]
			if g.flags[i]&(1<<c) == 0 && v != 0 {
				return nil, nil, fmt.Errorf("invalid omitted corner value")
			}
			g.values[i][c] = v
		}
	}
	return base, g, nil
}
