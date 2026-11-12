package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"runtime"
)

// BGC2 stores a channel mask and only the selected source corner values.
const sparseCornerMagic = "BGC2"
const splitCornerMagic = "BGC3"

// Split flags and selected residuals into four independent range-coded
// streams. This is lossless and keeps omitted residuals out of the streams.
func encodeSplitCornerRecords(records []byte, cols int) []byte {
	n := len(records) / 4
	var channels [4][]byte
	channels[0] = make([]byte, n)
	for i := 0; i < n; i++ {
		bits := records[i*4]
		channels[0][i] = bits
		for c := 0; c < 3; c++ {
			if bits&(1<<c) != 0 {
				channels[c+1] = append(channels[c+1], records[i*4+c+1])
			}
		}
	}
	var packed [4][]byte
	out := make([]byte, 16)
	flagsDelta := make([]byte, n)
	for i, bits := range channels[0] {
		if i%cols != 0 {
			bits ^= channels[0][i-1]
		}
		flagsDelta[i] = bits
	}
	for i := range packed {
		packed[i] = encodeByteRangeInto(nil, channels[i])
	}
	predicted := encodeByteRangeInto(nil, flagsDelta)
	usePrediction := len(predicted) < len(packed[0])
	if usePrediction {
		packed[0] = predicted
	}
	for i := range packed {
		length := uint32(len(packed[i]))
		if i == 0 && usePrediction {
			length |= 1 << 31
		}
		binary.BigEndian.PutUint32(out[i*4:], length)
	}
	for _, stream := range packed {
		out = append(out, stream...)
	}
	return out
}

func decodeSplitCornerRecords(records, data []byte, cols int) error {
	if len(data) < 16 {
		return fmt.Errorf("truncated split corner header")
	}
	var streams [4][]byte
	pos := 16
	usePrediction := binary.BigEndian.Uint32(data)&(1<<31) != 0
	for i := range streams {
		rawLength := binary.BigEndian.Uint32(data[i*4:])
		if i == 0 {
			rawLength &= ^uint32(1 << 31)
		}
		length := uint64(rawLength)
		if length > uint64(len(data)-pos) {
			return fmt.Errorf("invalid split corner lengths")
		}
		streams[i] = data[pos : pos+int(length)]
		pos += int(length)
	}
	if pos != len(data) {
		return fmt.Errorf("trailing split corner data")
	}
	n := len(records) / 4
	flags := make([]byte, n)
	if err := decodeByteRangeInto(flags, streams[0]); err != nil {
		return err
	}
	if usePrediction {
		for i := range flags {
			if i%cols != 0 {
				flags[i] ^= flags[i-1]
			}
		}
	}
	var counts [3]int
	for i, bits := range flags {
		if bits & ^byte(7) != 0 {
			return fmt.Errorf("invalid corner flags")
		}
		records[i*4] = bits
		for c := 0; c < 3; c++ {
			if bits&(1<<c) != 0 {
				counts[c]++
			}
		}
	}
	for c, count := range counts {
		values := make([]byte, count)
		if err := decodeByteRangeInto(values, streams[c+1]); err != nil {
			return err
		}
		j := 0
		for i, bits := range flags {
			if bits&(1<<c) != 0 {
				records[i*4+c+1] = values[j]
				j++
			}
		}
	}
	return nil
}

// redundantGradientCorners marks luma nodes whose entire interpolation
// footprint is covered by gradients. Their values cannot affect any glyph.
func redundantGradientCorners(w, h int, source cornerGrid, gradients []largeGradient) []bool {
	if len(gradients) == 0 {
		return nil
	}
	cellsW, cellsH := (w+7)/8, (h+7)/8
	covered := make([]bool, cellsW*cellsH)
	for _, b := range gradients {
		for y := b.y / 8; y < (b.y+b.size)/8; y++ {
			for x := b.x / 8; x < (b.x+b.size)/8; x++ {
				covered[y*cellsW+x] = true
			}
		}
	}
	redundant := make([]bool, len(source.values))
	for i := range redundant {
		gx, gy := i%source.cols, i/source.cols
		all, found := true, false
		for y := gy - 1; y <= gy; y++ {
			for x := gx - 1; x <= gx; x++ {
				if x < 0 || y < 0 || x >= cellsW || y >= cellsH {
					continue
				}
				found = true
				if !covered[y*cellsW+x] {
					all = false
				}
			}
		}
		redundant[i] = found && all
	}
	return redundant
}

func packSparseCorners(base, raw []byte, source cornerGrid, minSize int, gradients []largeGradient, parallel bool) ([]byte, error) {
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
	redundantLuma := redundantGradientCorners(w, h, source, gradients)
	decodeChannel := func(c int, scratch *Decoder, channelScratch *decoderChannelScratch, channelFlags []byte) error {
		if segments[c] == nil {
			return nil
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
			plane, err = scratch.decodeChannelInto(segments[c], w, h, channelScratch)
			if err != nil {
				return err
			}
		}
		sizes := scratch.glyphBlockSizes(segments[c], w, h)
		for i, v := range source.values {
			if c == 0 && redundantLuma != nil && redundantLuma[i] {
				continue
			}
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
				if channelFlags != nil {
					channelFlags[i] = 1
				} else {
					flags[i] |= 1 << c
				}
			}
		}
		return nil
	}
	// Keep the small-image workers independent. On larger images reuse one
	// plane buffer to avoid multiplying peak memory by the channel count.
	if parallel && runtime.GOMAXPROCS(0) > 1 && w*h >= 16<<10 && w*h <= 2<<20 && segments[1] != nil && segments[2] != nil {
		var results [3][]byte
		var errors [3]error
		for c := range 3 {
			results[c] = make([]byte, len(flags))
		}
		processStreams(3, w*h, true, func(c int) {
			var scratch Decoder
			var channelScratch decoderChannelScratch
			errors[c] = decodeChannel(c, &scratch, &channelScratch, results[c])
		})
		for c := range 3 {
			if errors[c] != nil {
				return nil, errors[c]
			}
			for i, bit := range results[c] {
				flags[i] |= bit << c
			}
		}
	} else {
		var scratch Decoder
		var channelScratch decoderChannelScratch
		for c := range 3 {
			if err := decodeChannel(c, &scratch, &channelScratch, nil); err != nil {
				return nil, err
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
	magic := sparseCornerMagic
	if split := encodeSplitCornerRecords(records, source.cols); len(split) < len(compressed) {
		compressed, magic = split, splitCornerMagic
	}
	out := make([]byte, 0, cornerHeaderSize+len(base)+len(compressed))
	out = append(out, magic...)
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
	if len(data) < cornerHeaderSize || (string(data[:4]) != sparseCornerMagic && string(data[:4]) != splitCornerMagic) {
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
	var err error
	if string(data[:4]) == splitCornerMagic {
		err = decodeSplitCornerRecords(records, data[cornerHeaderSize+int(baseLen):], cols)
	} else {
		err = decodeByteRangeInto(records, data[cornerHeaderSize+int(baseLen):])
	}
	if err != nil {
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
