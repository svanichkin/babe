package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"runtime"
	"sync"
)

// BSM1 stores a complete -smooth image: the BRC1 glyph channels, the selected
// corner residuals and the shared-vertex gradients, each context-modelled with
// adaptive binary range coding. It describes exactly the same data as
// BGG3(BGC2/BGC3(BRC1)), so decoded pixels are identical; only storage differs.
//
//	0  "BSM1"
//	4  BRC1 payload header (18 bytes)
//	22 CRC32 of every following byte (4)
//	26 flags: bit 0 gradients, bit 1 corners (1)
//	27 corner minimum block size (1)
//	28 level steps for Y, Cb, Cr (3)
//	31 gradient block count (4)
//	35 stream count (1), then one 4-byte length per stream, then the streams:
//	   gradients (if any), per channel its main bands then stripes (if any),
//	   and per channel its corners (if any).
const smoothMagic = "BSM1"
const smoothHeaderSize = 36

type smoothImage struct {
	header    []byte          // BRC1 payload header
	blocks    [][]smoothBlock // per stored channel, in coding order
	gradients []largeGradient
	mesh      *sharedGradientMesh
	source    *cornerGrid // source corners; nil stores none
	minSize   int
}

func smoothChannelSegments(raw []byte, h payloadHeader) ([][]byte, error) {
	pos := payloadHeaderSize
	var segs [][]byte
	for c := 0; c < h.channels; c++ {
		seg, err := readChannelSegment(raw, &pos)
		if err != nil {
			return nil, err
		}
		segs = append(segs, seg)
	}
	return segs, nil
}

// sizesForCorners returns per-pixel block sizes for each stored channel.
func smoothSizeMaps(scratch *smoothScratch, segs [][]byte, w, h int) [][]byte {
	maps := make([][]byte, len(segs))
	for c := range maps {
		scratch.sizes[c] = grow(scratch.sizes[c], w*h)
		maps[c] = scratch.sizes[c]
	}
	processStreams(len(segs), w*h, true, func(c int) {
		fillGlyphBlockSizes(maps[c], segs[c], w, h)
	})
	return maps
}

func cornerEligible(sizes []byte, w, h, x, y, minSize int) bool {
	for yy := max(0, y-1); yy <= y; yy++ {
		for xx := max(0, x-1); xx <= x; xx++ {
			if int(sizes[yy*w+xx]) >= minSize {
				return true
			}
		}
	}
	return false
}

type cornerModels struct {
	flag [6 * 9]bitModel
	sign [6 * 3]bitModel
	mag  [6]uintModel
}

// cornerBaseline is the decoder's estimate of a corner from glyph pixels.
func cornerBaseline(rec []byte, w, x, y int) int {
	sum, count := 0, 0
	for yy := max(0, y-1); yy <= y; yy++ {
		for xx := max(0, x-1); xx <= x; xx++ {
			sum += int(rec[yy*w+xx])
			count++
		}
	}
	return (sum + count/2) / count
}

func cornerBucket(d int) int {
	switch d = absInt(d); {
	case d <= 1:
		return 0
	case d <= 3:
		return 1
	case d <= 5:
		return 2
	case d <= 9:
		return 3
	case d <= 17:
		return 4
	default:
		return 5
	}
}

// codeCornerChannel codes the flag and value of every eligible node of channel
// c. Flags are read from bit c of each record's first byte when encoding and
// returned in flags; residuals use the record's byte 1+c. The difference
// between a prediction from decoded neighbours and the glyph baseline selects
// the contexts: it estimates whether, and in which direction, a node needs a
// stored value.
func codeCornerChannel(sc *symCoder, c int, records, flags []byte, w, h, minSize int, sizes, rec []byte, redundant []bool) error {
	const step = 8
	cols, rows := (w-1+step-1)/step+1, (h-1+step-1)/step+1
	m := &cornerModels{}
	resetBitModels(m.flag[:])
	resetBitModels(m.sign[:])
	for i := range m.mag {
		resetBitModels(m.mag[i][:])
	}
	// st: 0 not flagged, 1 flagged, 2 not eligible.
	st := make([]uint8, cols*rows)
	val := make([]uint8, cols*rows)
	for gy := 0; gy < rows; gy++ {
		y := min(gy*step, h-1)
		for gx := 0; gx < cols; gx++ {
			x := min(gx*step, w-1)
			i := gy*cols + gx
			var flag uint32
			if sc.e != nil {
				flag = uint32(records[i*4]>>c) & 1
			}
			base := cornerBaseline(rec, w, x, y)
			val[i] = uint8(base)
			eligible := cornerEligible(sizes, w, h, x, y, minSize) && (redundant == nil || !redundant[i])
			if !eligible {
				st[i] = 2
				if flag != 0 {
					return fmt.Errorf("corner flag on ineligible node")
				}
				continue
			}
			pred := base
			switch {
			case gx > 0 && gy > 0:
				l, t, tl := int(val[i-1]), int(val[i-cols]), int(val[i-cols-1])
				switch {
				case tl >= max(l, t):
					pred = min(l, t)
				case tl <= min(l, t):
					pred = max(l, t)
				default:
					pred = l + t - tl
				}
			case gx > 0:
				pred = int(val[i-1])
			case gy > 0:
				pred = int(val[i-cols])
			}
			bucket := cornerBucket(pred - base)
			l, t := uint8(2), uint8(2)
			if gx > 0 {
				l = st[i-1]
			}
			if gy > 0 {
				t = st[i-cols]
			}
			bit := sc.bit(&m.flag[(bucket*3+int(l))*3+int(t)], flag)
			st[i] = uint8(bit)
			if bit == 0 {
				continue
			}
			flags[i] = 1
			// The sign context is the side of the baseline the prediction lies on.
			side := 0
			if pred > base {
				side = 1
			} else if pred < base {
				side = 2
			}
			var r int
			if sc.e != nil {
				r = int(int8(records[i*4+1+c]))
			}
			neg := uint32(0)
			if r < 0 {
				neg = 1
			}
			neg = sc.bit(&m.sign[bucket*3+side], neg)
			mag := int(sc.uint(&m.mag[bucket], uint32(max(absInt(r), 1)-1))) + 1
			if mag > 128 || (mag == 128 && neg == 0) {
				return errCtxStream
			}
			if neg != 0 {
				mag = -mag
			}
			rb := byte(int8(mag))
			if sc.e != nil && rb != records[i*4+1+c] {
				return fmt.Errorf("unsupported corner residual")
			}
			records[i*4+1+c] = rb
			val[i] = uint8(base + mag)
		}
	}
	return nil
}

// codeCorners codes each stored channel's corners as an independent stream.
func codeCorners(coders []*symCoder, records []byte, w, h, minSize int, sizes, recs [][]byte, redundant []bool, parallel bool) error {
	n := len(records) / 4
	flags := make([][]byte, len(coders))
	errs := make([]error, len(coders))
	runParallel(len(coders), parallel, func(c int) {
		flags[c] = make([]byte, n)
		var red []bool
		if c == 0 {
			red = redundant
		}
		errs[c] = codeCornerChannel(coders[c], c, records, flags[c], w, h, minSize, sizes[c], recs[c], red)
	})
	for _, err := range errs {
		if err != nil {
			return err
		}
	}
	for i := 0; i < n; i++ {
		var f byte
		for c := range flags {
			f |= flags[c][i] << c
		}
		if coders[0].e != nil && records[i*4] != f {
			return fmt.Errorf("unsupported corner flags")
		}
		records[i*4] = f
	}
	return nil
}

type gradientModels struct {
	split [5]bitModel
	zero  [2]bitModel
	sign  [2]bitModel
	mag   [2]uintModel
}

// codeGradients codes the gradient quadtree and its used mesh vertices.
func codeGradients(sc *symCoder, blocks []largeGradient, mesh *sharedGradientMesh, w, h int) ([]largeGradient, *sharedGradientMesh, error) {
	m := &gradientModels{}
	resetBitModels(m.split[:])
	resetBitModels(m.zero[:])
	resetBitModels(m.sign[:])
	for i := range m.mag {
		resetBitModels(m.mag[i][:])
	}
	type key struct{ x, y, size int }
	var set map[key]bool
	if sc.e != nil {
		set = make(map[key]bool, len(blocks))
		for _, b := range blocks {
			set[key{b.x, b.y, b.size}] = true
		}
	}
	var out []largeGradient
	var visit func(x, y, n, level int)
	visit = func(x, y, n, level int) {
		if x+n > w || y+n > h {
			if n > 16 {
				for i := 0; i < 4; i++ {
					visit(x+i%2*n/2, y+i/2*n/2, n/2, level+1)
				}
			}
			return
		}
		var leaf uint32
		if set[key{x, y, n}] {
			leaf = 1
		}
		if sc.bit(&m.split[level], leaf) != 0 {
			out = append(out, largeGradient{x: x, y: y, size: n})
			return
		}
		if n > 16 {
			for i := 0; i < 4; i++ {
				visit(x+i%2*n/2, y+i/2*n/2, n/2, level+1)
			}
		}
	}
	for y := 0; y < h; y += 256 {
		for x := 0; x < w; x += 256 {
			visit(x, y, 256, 0)
		}
	}
	if sc.e != nil && len(out) != len(blocks) {
		return nil, nil, fmt.Errorf("gradients are not a quadtree")
	}
	if len(out) == 0 {
		return out, nil, nil
	}
	used := sharedNodeMask(out, w, h)
	cols := w/16 + 1
	values := make([]byte, len(used))
	prev := 128
	for i, yes := range used {
		if !yes {
			continue
		}
		gx, gy := i%cols, i/cols
		l, t := gx > 0 && used[i-1], gy > 0 && used[i-cols]
		pred, ctx := prev, 1
		switch {
		case l && t:
			a, b := int(values[i-1]), int(values[i-cols])
			c := (a + b) / 2
			if gx > 0 && used[i-cols-1] {
				c = int(values[i-cols-1])
			}
			pred = min(255, max(0, a+b-c))
			ctx = 0
		case l:
			pred = int(values[i-1])
		case t:
			pred = int(values[i-cols])
		}
		var v int
		if sc.e != nil {
			v = int(mesh.values[i])
		}
		v = pred + sc.signed(&m.zero[ctx], &m.sign[ctx], &m.mag[ctx], v-pred)
		if v < 0 || v > 255 {
			return nil, nil, errCtxStream
		}
		values[i] = byte(v)
		prev = v
	}
	return out, &sharedGradientMesh{width: w, height: h, cols: cols, values: values}, nil
}

// runParallel runs fn(0..n-1) on up to GOMAXPROCS goroutines.
func runParallel(n int, parallel bool, fn func(int)) {
	workers := 1
	if parallel {
		workers = min(runtime.GOMAXPROCS(0), n)
	}
	if workers <= 1 {
		for i := 0; i < n; i++ {
			fn(i)
		}
		return
	}
	var wg sync.WaitGroup
	next := make(chan int, n)
	for i := 0; i < n; i++ {
		next <- i
	}
	close(next)
	for k := 0; k < workers; k++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := range next {
				fn(i)
			}
		}()
	}
	wg.Wait()
}

func packSmooth(scratch *smoothScratch, img smoothImage, steps [3]int, parallel bool) ([]byte, error) {
	h, err := parsePayloadHeader(img.header)
	if err != nil {
		return nil, err
	}
	if h.small > 4 || h.macro > 8 || len(img.blocks) != h.channels {
		return nil, fmt.Errorf("unsupported block sizes")
	}
	geo := newSmoothGeometry(h.w, h.h, h.small, h.macro)
	var streams [][]byte
	gradients := img.gradients
	if len(gradients) != 0 {
		e := newCtxEncoder(nil)
		gradients, _, err = codeGradients(&symCoder{e: e}, img.gradients, img.mesh, h.w, h.h)
		if err != nil {
			return nil, err
		}
		streams = append(streams, e.finish())
	}

	// Channel bands are independent streams. Encode them in parallel.
	type job struct {
		c, band int
		stripes bool
		blocks  []smoothBlock
		out     []byte
		err     error
	}
	var jobs []*job
	states := make([]*smoothChannelState, h.channels)
	for c := 0; c < h.channels; c++ {
		blocks := img.blocks[c]
		scratch.states[c] = resetSmoothChannelState(scratch.states[c], geo, steps[c])
		states[c] = scratch.states[c]
		k := 0
		for b, rows := range geo.bandRows {
			start := k
			for k < len(blocks) && int(blocks[k].y) < rows[1]*geo.macro && int(blocks[k].x) < geo.fullW && int(blocks[k].y) < geo.fullH {
				k++
			}
			jobs = append(jobs, &job{c: c, band: b, blocks: blocks[start:k]})
		}
		if geo.hasStripes {
			jobs = append(jobs, &job{c: c, stripes: true, blocks: blocks[k:]})
		} else if k != len(blocks) {
			return nil, fmt.Errorf("unexpected trailing blocks")
		}
	}
	run := func(stripes bool) func(int) {
		return func(i int) {
			j := jobs[i]
			if j.stripes != stripes {
				return
			}
			e := newCtxEncoder(nil)
			sc := &symCoder{e: e}
			if stripes {
				band := &smoothBand{y0: 0, stripes: true, lastIdx: states[j.c].levels.kmax / 2}
				_, j.err = states[j.c].codeStripes(sc, band, j.blocks)
			} else {
				rows := geo.bandRows[j.band]
				band := &smoothBand{y0: rows[0] * geo.macro, rows: rows, lastIdx: states[j.c].levels.kmax / 2}
				_, j.err = states[j.c].codeMainBand(sc, band, j.blocks)
			}
			j.out = e.finish()
		}
	}
	runParallel(len(jobs), parallel, run(false))
	runParallel(len(jobs), parallel, run(true))
	for _, j := range jobs {
		if j.err != nil {
			return nil, j.err
		}
		streams = append(streams, j.out)
	}

	// The BRC1 payload is reconstructed for the checksum and block sizes.
	var channelRaw [3][]byte
	runParallel(h.channels, parallel, func(c int) {
		channelRaw[c] = appendSmoothChannel(nil, [][]smoothBlock{img.blocks[c]}, geo)
	})
	raw := append([]byte(nil), img.header[:payloadHeaderSize]...)
	for c := 0; c < h.channels; c++ {
		raw = append(raw, channelRaw[c]...)
	}

	var records []byte
	if img.source != nil {
		segs, err := smoothChannelSegments(raw, h)
		if err != nil {
			return nil, err
		}
		sizes := smoothSizeMaps(scratch, segs, h.w, h.h)
		redundant := redundantCornerMask(h.w, h.h, img.gradients)
		recs := make([][]byte, h.channels)
		coders := make([]*symCoder, h.channels)
		for c := range recs {
			recs[c] = states[c].rec
			coders[c] = &symCoder{e: newCtxEncoder(nil)}
		}
		records = cornerRecordsFromPlanes(img.source, recs, sizes, img.minSize, redundant)
		coded := append([]byte(nil), records...)
		if err := codeCorners(coders, coded, h.w, h.h, img.minSize, sizes, recs, redundant, parallel); err != nil {
			return nil, err
		}
		for _, sc := range coders {
			streams = append(streams, sc.e.finish())
		}
	}
	if len(streams) > 255 {
		return nil, fmt.Errorf("too many streams")
	}

	out := make([]byte, 0, smoothHeaderSize+4*len(streams))
	out = append(out, smoothMagic...)
	out = append(out, img.header[:payloadHeaderSize]...)
	out = append(out, 0, 0, 0, 0) // checksum of everything that follows
	var flags byte
	if len(img.gradients) != 0 {
		flags |= 1
	}
	if img.source != nil {
		flags |= 2
	}
	out = append(out, flags, byte(img.minSize), byte(steps[0]), byte(steps[1]), byte(steps[2]))
	out = binary.BigEndian.AppendUint32(out, uint32(len(img.gradients)))
	out = append(out, byte(len(streams)))
	for _, s := range streams {
		out = binary.BigEndian.AppendUint32(out, uint32(len(s)))
	}
	for _, s := range streams {
		out = append(out, s...)
	}
	binary.BigEndian.PutUint32(out[22:], crc32.ChecksumIEEE(out[26:]))
	return out, nil
}

// cornerRecordsFromPlanes selects stored corners exactly as
// sparseCornerRecords, from already reconstructed glyph planes.
func cornerRecordsFromPlanes(source *cornerGrid, planes, sizes [][]byte, minSize int, redundant []bool) []byte {
	w, h := source.width, source.height
	records := make([]byte, len(source.values)*4)
	for c := range planes {
		for i, v := range source.values {
			if c == 0 && redundant != nil && redundant[i] {
				continue
			}
			x, y := min((i%source.cols)*source.step, w-1), min((i/source.cols)*source.step, h-1)
			base := cornerBaseline(planes[c], w, x, y)
			diff := int(v[c]) - base
			if (diff < -3 || diff > 3) && cornerEligible(sizes[c], w, h, x, y, minSize) {
				records[i*4] |= 1 << c
				records[i*4+1+c] = v[c] - byte(base)
			}
		}
	}
	return records
}

// encodeSmoothBlocks encodes macro-group rows [rows[0], rows[1]), or the
// right and bottom stripes, exactly as encodeChannelReuse serializes them.
func encodeSmoothBlocks(plane []byte, stride int, geo smoothGeometry, rows [2]int, stripes bool, step, solidThreshold int, spread int32, out []smoothBlock) ([]smoothBlock, error) {
	add := func(x, y, size int) error {
		fg, bg, pattern, bits, err := encodeChannelBlockBits(plane, stride, geo.h4, x, y, size, size, step, solidThreshold)
		if err != nil {
			return err
		}
		if !pattern {
			bg, bits = fg, 0
		}
		out = append(out, smoothBlock{x: int32(x), y: int32(y), size: uint8(size), pattern: pattern, fg: fg, bg: bg, bits: bits})
		return nil
	}
	if stripes {
		for y := 0; y < geo.fullH; y += geo.small {
			for x := geo.fullW; x < geo.w4; x += geo.small {
				if err := add(x, y, geo.small); err != nil {
					return nil, err
				}
			}
		}
		for y := geo.fullH; y < geo.h4; y += geo.small {
			for x := 0; x < geo.w4; x += geo.small {
				if err := add(x, y, geo.small); err != nil {
					return nil, err
				}
			}
		}
		return out, nil
	}
	useMacro := geo.macro > geo.small
	for gy := rows[0]; gy < rows[1]; gy++ {
		my := gy * geo.macro
		for mx := 0; mx < geo.fullW; mx += geo.macro {
			if useMacro && canUseBigBlockChannel(plane, stride, geo.h4, mx, my, blockSpread(spread, mx, my)) {
				if err := add(mx, my, geo.macro); err != nil {
					return nil, err
				}
				continue
			}
			for by := 0; by < geo.macro; by += geo.small {
				for bx := 0; bx < geo.macro; bx += geo.small {
					if err := add(mx+bx, my+by, geo.small); err != nil {
						return nil, err
					}
				}
			}
		}
	}
	return out, nil
}

func redundantCornerMask(w, h int, gradients []largeGradient) []bool {
	if len(gradients) == 0 {
		return nil
	}
	const step = 8
	cols, rows := (w-1+step-1)/step+1, (h-1+step-1)/step+1
	return redundantGradientCorners(w, h, cornerGrid{width: w, height: h, step: step, cols: cols, values: make([][3]byte, cols*rows)}, gradients)
}

// smoothDecoded is a decoded BSM1 image. Either raw holds the complete BRC1
// payload, or planes hold the reconstructed channels and segments hold each
// channel's block count and macroblock size stream, as a BRC1 segment prefix.
type smoothDecoded struct {
	header    []byte
	raw       []byte
	planes    [3][]byte
	segments  [3][]byte
	gradients []largeGradient
	mesh      *sharedGradientMesh
	corners   *cornerGrid
}

// unpackSmooth decodes a BSM1 image. The BRC1 payload is rebuilt (into dst)
// only when needRaw is set or the geometry requires it.
func unpackSmooth(scratch *smoothScratch, data, dst []byte, needRaw, parallel bool) (smoothDecoded, error) {
	var res smoothDecoded
	fail := func(format string, args ...any) (smoothDecoded, error) {
		return smoothDecoded{}, fmt.Errorf("smooth container: "+format, args...)
	}
	if len(data) < smoothHeaderSize || string(data[:4]) != smoothMagic {
		return fail("invalid header")
	}
	header := data[4 : 4+payloadHeaderSize]
	res.header = header
	h, err := parsePayloadHeader(header)
	if err != nil {
		return fail("%v", err)
	}
	if h.small > 4 || h.macro > 8 || header[9]&channelFlagLossless != 0 {
		return fail("unsupported block sizes")
	}
	if crc32.ChecksumIEEE(data[26:]) != binary.BigEndian.Uint32(data[22:]) {
		return fail("checksum mismatch")
	}
	// Pattern serialization and block size maps use the current geometry.
	smallBlock, macroBlock = h.small, h.macro
	needRaw = needRaw || h.macro == 2 || h.w4 != h.w || h.h4 != h.h
	flags, minSize := data[26], int(data[27])
	if flags&^3 != 0 {
		return fail("invalid flags")
	}
	var steps [3]int
	for c := range steps {
		steps[c] = int(data[28+c])
		if steps[c] < 1 || steps[c] > 128 {
			return fail("invalid level step")
		}
	}
	gradientCount := int(binary.BigEndian.Uint32(data[31:]))
	streamCount := int(data[35])
	geo := newSmoothGeometry(h.w, h.h, h.small, h.macro)
	perChannel := len(geo.bandRows)
	if geo.hasStripes {
		perChannel++
	}
	want := h.channels * perChannel
	if flags&1 != 0 {
		want++
	}
	if flags&2 != 0 {
		want += h.channels
	}
	if streamCount != want || len(data) < smoothHeaderSize+4*streamCount {
		return fail("invalid stream count")
	}
	streams := make([][]byte, streamCount)
	pos := smoothHeaderSize + 4*streamCount
	for i := range streams {
		n := uint64(binary.BigEndian.Uint32(data[smoothHeaderSize+4*i:]))
		if n > uint64(len(data)-pos) {
			return fail("truncated stream")
		}
		streams[i] = data[pos : pos+int(n)]
		pos += int(n)
	}
	if pos != len(data) {
		return fail("trailing data")
	}
	next := 0
	if flags&1 != 0 {
		d := newCtxDecoder(streams[next])
		next++
		res.gradients, res.mesh, err = codeGradients(&symCoder{d: d}, nil, nil, h.w, h.h)
		if err != nil || d.err() != nil || len(res.gradients) != gradientCount || gradientCount == 0 {
			return fail("invalid gradients")
		}
		if err := validateLargeGradients(res.gradients, h.w, h.h); err != nil {
			return fail("%v", err)
		}
	}

	type job struct {
		c, band int
		stripes bool
		src     []byte
		blocks  []smoothBlock
		count   int
		err     error
	}
	var jobs []*job
	states := make([]*smoothChannelState, h.channels)
	for c := 0; c < h.channels; c++ {
		scratch.states[c] = resetSmoothChannelState(scratch.states[c], geo, steps[c])
		states[c] = scratch.states[c]
		for b := range geo.bandRows {
			jobs = append(jobs, &job{c: c, band: b, src: streams[next]})
			next++
		}
		if geo.hasStripes {
			jobs = append(jobs, &job{c: c, stripes: true, src: streams[next]})
			next++
		}
	}
	run := func(stripes bool) func(int) {
		return func(i int) {
			j := jobs[i]
			if j.stripes != stripes {
				return
			}
			d := newCtxDecoder(j.src)
			sc := &symCoder{d: d}
			band := &smoothBand{discard: !needRaw}
			if stripes {
				band.stripes, band.lastIdx = true, states[j.c].levels.kmax/2
				j.blocks, j.err = states[j.c].codeStripes(sc, band, j.blocks)
			} else {
				rows := geo.bandRows[j.band]
				band.y0, band.rows, band.lastIdx = rows[0]*geo.macro, rows, states[j.c].levels.kmax/2
				j.blocks, j.err = states[j.c].codeMainBand(sc, band, j.blocks)
			}
			j.count = band.count
			if j.err == nil {
				j.err = d.err()
			}
		}
	}
	for len(scratch.blocks) < len(jobs) {
		scratch.blocks = append(scratch.blocks, nil)
	}
	for i, j := range jobs {
		j.blocks = scratch.blocks[i][:0]
	}
	runParallel(len(jobs), parallel, run(false))
	runParallel(len(jobs), parallel, run(true))
	for i, j := range jobs {
		scratch.blocks[i] = j.blocks
	}
	for _, j := range jobs {
		if j.err != nil {
			return fail("channel %d: %v", j.c, j.err)
		}
	}
	var segs [][]byte
	if needRaw {
		var channelRaw [3][]byte
		runParallel(h.channels, parallel && h.w*h.h >= 1<<16, func(c int) {
			var parts [][]smoothBlock
			for _, j := range jobs {
				if j.c == c {
					parts = append(parts, j.blocks)
				}
			}
			channelRaw[c] = appendSmoothChannel(nil, parts, geo)
		})
		raw := append(dst[:0], header...)
		for c := 0; c < h.channels; c++ {
			raw = append(raw, channelRaw[c]...)
		}
		if len(raw) > maxPayloadSize {
			return fail("payload exceeds limit")
		}
		if segs, err = smoothChannelSegments(raw, h); err != nil {
			return fail("%v", err)
		}
		res.raw = raw
	} else {
		for c := 0; c < h.channels; c++ {
			count := 0
			for _, j := range jobs {
				if j.c == c {
					count += j.count
				}
			}
			res.planes[c] = states[c].rec
			res.segments[c] = appendSizeSegment(nil, count, states[c].groupBig)
			segs = append(segs, res.segments[c])
		}
	}

	if flags&2 != 0 {
		const step = 8
		cols, rows := (h.w-1+step-1)/step+1, (h.h-1+step-1)/step+1
		records := make([]byte, cols*rows*4)
		sizes := smoothSizeMaps(scratch, segs, h.w, h.h)
		recs := make([][]byte, h.channels)
		coders := make([]*symCoder, h.channels)
		for c := range recs {
			recs[c] = states[c].rec
			coders[c] = &symCoder{d: newCtxDecoder(streams[next+c])}
		}
		if err := codeCorners(coders, records, h.w, h.h, minSize, sizes, recs, redundantCornerMask(h.w, h.h, res.gradients), parallel); err != nil {
			return fail("invalid corners")
		}
		for _, sc := range coders {
			if sc.d.err() != nil {
				return fail("invalid corners")
			}
		}
		g := &cornerGrid{width: h.w, height: h.h, step: step, cols: cols, values: make([][3]byte, cols*rows), flags: make([]byte, cols*rows)}
		for i := range g.values {
			g.flags[i] = records[i*4]
			for c := range 3 {
				g.values[i][c] = records[i*4+1+c]
			}
		}
		res.corners = g
	}
	return res, nil
}

// appendSizeSegment writes a BRC1 channel segment prefix: the block count and
// the macroblock size stream. Block size maps read nothing else.
func appendSizeSegment(dst []byte, blocks int, groupBig []uint8) []byte {
	dst = binary.BigEndian.AppendUint32(dst, uint32(blocks))
	dst = binary.BigEndian.AppendUint32(dst, uint32((len(groupBig)+7)/8))
	var b byte
	for i, big := range groupBig {
		b = b<<1 | big
		if i&7 == 7 {
			dst = append(dst, b)
			b = 0
		}
	}
	if n := len(groupBig) & 7; n != 0 {
		dst = append(dst, b<<(8-n))
	}
	return dst
}
