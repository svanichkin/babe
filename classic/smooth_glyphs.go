package classic

import (
	"bytes"
	"encoding/binary"
	"fmt"
)

// Context modelling of one BRC1 channel. The coder walks the same block order
// as the BRC1 streams and codes, per block, its type bit, its foreground level
// predicted from neighbouring blocks, its contrast and its pattern bits in
// raster order. Pattern bits use neighbouring pattern bits; across the block
// border they use whether the reconstructed neighbour pixel is nearer the
// foreground or the background. The decoder rebuilds the identical streams.

type smoothBlock struct {
	x, y    int32
	size    uint8
	pattern bool
	fg, bg  uint8
	bits    uint64 // raster order: pixel (px,py) is bit size*size-1-(py*size+px)
}

type smoothGeometry struct {
	w, h, small, macro   int
	w4, h4, fullW, fullH int
	groupsX, groupsY     int
	cellsX, cellsY       int
	shift                uint // log2(small)
	hasStripes           bool
	bandRows             [][2]int // macro-group rows per band
}

func newSmoothGeometry(w, h, small, macro int) smoothGeometry {
	g := smoothGeometry{w: w, h: h, small: small, macro: macro}
	g.w4, g.h4 = w/small*small, h/small*small
	g.fullW, g.fullH = g.w4/macro*macro, g.h4/macro*macro
	g.groupsX, g.groupsY = g.fullW/macro, g.fullH/macro
	g.cellsX, g.cellsY = g.w4/small, g.h4/small
	for 1<<g.shift < small {
		g.shift++
	}
	g.hasStripes = g.w4 > g.fullW || g.h4 > g.fullH
	// Each band has its own models; a band boundary costs some adaptation,
	// so bands are only used for parallelism on larger images.
	bands := min(max(w*h/(1<<18), 1), 16)
	bands = min(bands, g.groupsY)
	for i := 0; i < bands; i++ {
		g.bandRows = append(g.bandRows, [2]int{g.groupsY * i / bands, g.groupsY * (i + 1) / bands})
	}
	return g
}

// levelMap converts quantized levels to indices: level k is min(255, k*step).
type levelMap struct {
	step, kmax int
	idx        [256]uint8
	lvl        [256]uint8
}

func newLevelMap(step int) levelMap {
	m := levelMap{step: step, kmax: (255 + step - 1) / step}
	for k := 0; k <= m.kmax; k++ {
		m.lvl[k] = byte(min(255, k*step))
	}
	for v := 0; v < 256; v++ {
		k := v / step
		if v == 255 {
			k = m.kmax
		}
		m.idx[v] = byte(k)
	}
	return m
}

func (m *levelMap) valid(v byte) bool { return m.lvl[m.idx[v]] == v }

type smoothChannelState struct {
	gradients *smoothGradientPlan
	geo       smoothGeometry
	levels    levelMap
	groupBig  []uint8
	cellType  []uint8
	cellFg    []uint8
	cellCon   []uint8
	cellDone  []bool
	rec       []uint8
}

func grow[T any](s []T, n int) []T {
	if cap(s) < n {
		return make([]T, n)
	}
	return s[:n]
}

// resetSmoothChannelState prepares st (or a new state) for geo, reusing its
// buffers. Only cellDone needs clearing: other cell data and rec are read
// only where cellDone is set, and groupBig only after it was written.
func resetSmoothChannelState(st *smoothChannelState, geo smoothGeometry, step int) *smoothChannelState {
	if st == nil {
		st = &smoothChannelState{}
	}
	n := geo.cellsX * geo.cellsY
	st.geo, st.levels = geo, newLevelMap(step)
	st.gradients = nil
	st.groupBig = grow(st.groupBig, geo.groupsX*geo.groupsY)
	st.cellType, st.cellFg, st.cellCon = grow(st.cellType, n), grow(st.cellFg, n), grow(st.cellCon, n)
	st.cellDone = grow(st.cellDone, n)
	clear(st.cellDone)
	st.rec = grow(st.rec, geo.w*geo.h)
	return st
}

// smoothScratch keeps BSM2 buffers between images.
type smoothScratch struct {
	states [3]*smoothChannelState
	blocks [][]smoothBlock
}

type smoothGlyphModels struct {
	size    [9]bitModel
	typ     [18]bitModel
	zero    [20]bitModel
	sign    [20]bitModel
	mag     [20]uintModel
	con     [2 * 5 * 6]uintModel
	pattern [3 * 4 * 81]bitModel
}

func newSmoothGlyphModels() *smoothGlyphModels {
	m := &smoothGlyphModels{}
	resetBitModels(m.size[:])
	resetBitModels(m.typ[:])
	resetBitModels(m.zero[:])
	resetBitModels(m.sign[:])
	resetBitModels(m.pattern[:])
	for i := range m.mag {
		resetBitModels(m.mag[i][:])
	}
	for i := range m.con {
		resetBitModels(m.con[i][:])
	}
	return m
}

// symCoder runs the same modelling code for encoding and decoding.
type symCoder struct {
	e *ctxEncoder
	d *ctxDecoder
}

func (s *symCoder) bit(m *bitModel, v uint32) uint32 {
	if s.e != nil {
		s.e.encode(m, v)
		return v
	}
	return s.d.decode(m)
}

func (s *symCoder) uint(m *uintModel, v uint32) uint32 {
	if s.e != nil {
		s.e.encodeUint(m, v)
		return v
	}
	return s.d.decodeUint(m)
}

// signed codes v with a zero flag, a sign and magnitude-1.
func (s *symCoder) signed(zero, sign *bitModel, mag *uintModel, v int) int {
	var nz uint32
	if v != 0 {
		nz = 1
	}
	if s.bit(zero, nz) == 0 {
		return 0
	}
	var neg uint32
	if v < 0 {
		neg, v = 1, -v
	}
	neg = s.bit(sign, neg)
	m := int(s.uint(mag, uint32(max(v, 1)-1))) + 1
	if neg != 0 {
		return -m
	}
	return m
}

func absInt(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func bucketOf(v int) int {
	switch {
	case v <= 0:
		return 0
	case v == 1:
		return 1
	case v <= 3:
		return 2
	default:
		return 3
	}
}

// band describes the coding region; neighbours above y0 are unavailable.
type smoothBand struct {
	discard bool // decode without keeping blocks
	count   int  // blocks coded
	tmp     smoothBlock
	y0      int
	rows    [2]int
	stripes bool
	lastIdx int
}

func (st *smoothChannelState) cellAvail(cx, cy int, band *smoothBand) bool {
	g := &st.geo
	if uint(cx) >= uint(g.cellsX) || uint(cy) >= uint(g.cellsY) || cy<<g.shift < band.y0 {
		return false
	}
	return st.cellDone[cy*g.cellsX+cx]
}

// pixelState is 1 when the reconstructed pixel is nearer fg than bg, 0 when
// not, and 2 when it is unavailable. With fg > bg, |v-fg| < |v-bg| is 2v > fg+bg.
func (st *smoothChannelState) pixelState(x, y int, fg, bg uint8, band *smoothBand) uint32 {
	g := &st.geo
	if uint(x) >= uint(g.w4) || uint(y) >= uint(g.h4) || y < band.y0 || !st.cellDone[(y>>g.shift)*g.cellsX+x>>g.shift] {
		return 2
	}
	if 2*int(st.rec[y*g.w+x]) > int(fg)+int(bg) {
		return 1
	}
	return 0
}

// codeBlock codes (or decodes into) b, whose x, y and size are already set.
func (st *smoothChannelState) codeBlock(sc *symCoder, m *smoothGlyphModels, band *smoothBand, b *smoothBlock) error {
	g := &st.geo
	x0, y0, n := int(b.x), int(b.y), int(b.size)
	cx, cy := x0>>g.shift, y0>>g.shift
	kind := 0
	if n != g.macro {
		kind = 1
	}
	lAvail, tAvail := st.cellAvail(cx-1, cy, band), st.cellAvail(cx, cy-1, band)
	state := func(avail bool, i int) int {
		if !avail {
			return 2
		}
		return int(st.cellType[i])
	}
	li, ti := cy*g.cellsX+cx-1, (cy-1)*g.cellsX+cx
	if n > 1 {
		var t uint32
		if b.pattern {
			t = 1
		}
		b.pattern = sc.bit(&m.typ[kind*9+state(lAvail, li)*3+state(tAvail, ti)], t) != 0
	} else if b.pattern {
		return fmt.Errorf("single-pixel pattern")
	}

	// Foreground: median edge detector over neighbouring block levels.
	pred, bucket := band.lastIdx, 4
	if lAvail && tAvail {
		l, t := int(st.cellFg[li]), int(st.cellFg[ti])
		tl := (l + t) / 2
		if st.cellAvail(cx-1, cy-1, band) {
			tl = int(st.cellFg[li-g.cellsX])
		}
		switch {
		case tl >= max(l, t):
			pred = min(l, t)
		case tl <= min(l, t):
			pred = max(l, t)
		default:
			pred = l + t - tl
		}
		bucket = bucketOf(absInt(l-t) + absInt(tl-l)/2)
	} else if lAvail {
		pred, bucket = int(st.cellFg[li]), 3
	} else if tAvail {
		pred, bucket = int(st.cellFg[ti]), 3
	}
	// Prefer the reconstructed pixels along the block's top and left edges.
	var edge [16]int
	ne := 0
	if lAvail {
		for py := 0; py < n; py++ {
			edge[ne] = int(st.rec[(y0+py)*g.w+x0-1])
			ne++
		}
	}
	if tAvail {
		for px := 0; px < n; px++ {
			edge[ne] = int(st.rec[(y0-1)*g.w+x0+px])
			ne++
		}
	}
	conPred := -1
	if ne > 0 {
		sum := 0
		for _, v := range edge[:ne] {
			sum += v
		}
		mean := (sum + ne/2) / ne
		hi, lo, nh, nl := 0, 0, 0, 0
		for _, v := range edge[:ne] {
			if v*ne >= sum {
				hi += v
				nh++
			} else {
				lo += v
				nl++
			}
		}
		hiV, loV := mean, mean
		if nh > 0 {
			hiV = (hi + nh/2) / nh
		}
		if nl > 0 {
			loV = (lo + nl/2) / nl
		}
		near := func(v int) int { return min((v+st.levels.step/2)/st.levels.step, st.levels.kmax) }
		if b.pattern {
			pred = near(hiV)
		} else {
			pred = near(mean)
		}
		conPred = near(hiV) - near(loV)
		bucket = bucketOf(conPred)
		if !lAvail || !tAvail {
			bucket = min(bucket+1, 3)
		}
	}
	pk := 0
	if b.pattern {
		pk = 1
	}
	ctx := (kind*2+pk)*5 + bucket
	lv := &st.levels
	if sc.e != nil && (!lv.valid(b.fg) || b.pattern && (!lv.valid(b.bg) || lv.idx[b.bg] >= lv.idx[b.fg])) {
		return fmt.Errorf("levels do not match step %d", lv.step)
	}
	fgIdx := int(lv.idx[b.fg])
	fgIdx = pred + sc.signed(&m.zero[ctx], &m.sign[ctx], &m.mag[ctx], fgIdx-pred)
	if fgIdx < 0 || fgIdx > lv.kmax {
		return errCtxStream
	}
	band.lastIdx = fgIdx
	b.fg = lv.lvl[fgIdx]

	con := 0
	if b.pattern {
		// Contrast context: neighbouring pattern contrasts.
		sum, count := 0, 0
		if lAvail && st.cellType[li] != 0 {
			sum += int(st.cellCon[li])
			count++
		}
		if tAvail && st.cellType[ti] != 0 {
			sum += int(st.cellCon[ti])
			count++
		}
		cb := 0
		if count > 0 {
			cb = 1 + min(bucketOf((sum+count/2)/count-1), 3)
		}
		con = fgIdx - int(lv.idx[b.bg])
		pb := 5
		if conPred >= 0 {
			switch {
			case conPred == 0:
				pb = 0
			case conPred == 1:
				pb = 1
			case conPred <= 3:
				pb = 2
			case conPred <= 7:
				pb = 3
			default:
				pb = 4
			}
		}
		con = int(sc.uint(&m.con[(kind*5+cb)*6+pb], uint32(max(con, 1)-1))) + 1
		if con > fgIdx {
			return errCtxStream
		}
		b.bg = lv.lvl[fgIdx-con]
		if b.bg == b.fg {
			return errCtxStream
		}

		sizeClass := 0
		switch n {
		case 4:
			sizeClass = 1
		case 8:
			sizeClass = 2
		}
		models := m.pattern[sizeClass*4*81 : (sizeClass+1)*4*81]
		// States outside the block: row above from x0-1 to x0+n, column left.
		var above [10]uint32
		var left [8]uint32
		for i := 0; i < n+2; i++ {
			above[i] = st.pixelState(x0-1+i, y0-1, b.fg, b.bg, band)
		}
		for i := 0; i < n; i++ {
			left[i] = st.pixelState(x0-1, y0+i, b.fg, b.bg, band)
		}
		// grid holds decoded bits with a one-pixel frame of outside states;
		// the right frame column is unavailable (2).
		var grid [9][10]uint32 // only n+1 rows and n+2 columns are used
		copy(grid[0][:n+2], above[:n+2])
		for py := 0; py < n; py++ {
			grid[py+1][0] = left[py]
			grid[py+1][n+1] = 2
		}
		total := n * n
		var out uint64
		if d := sc.d; d != nil {
			for py := 0; py < n; py++ {
				up, row := &grid[py], &grid[py+1]
				parity := (py & 1) * 2
				for px := 0; px < n; px++ {
					bit := d.decode(&models[(parity+px&1)*81+int(row[px]*27+up[px+1]*9+up[px]*3+up[px+2])])
					row[px+1] = bit
					out = out<<1 | uint64(bit)
				}
			}
		} else {
			e := sc.e
			for py := 0; py < n; py++ {
				up, row := &grid[py], &grid[py+1]
				parity := (py & 1) * 2
				for px := 0; px < n; px++ {
					bit := uint32(b.bits>>(total-1-(py*n+px))) & 1
					e.encode(&models[(parity+px&1)*81+int(row[px]*27+up[px+1]*9+up[px]*3+up[px+2])], bit)
					row[px+1] = bit
					out = out<<1 | uint64(bit)
				}
			}
		}
		b.bits = out
		if out == 0 || out == 1<<total-1 && total < 64 || total == 64 && out == ^uint64(0) {
			// A pattern block always has both levels in this format.
			return errCtxStream
		}
	}

	// Record the reconstructed block for later contexts.
	if b.pattern {
		bits, fgv, bgv := b.bits<<(64-n*n), b.fg, b.bg
		for py := 0; py < n; py++ {
			row := st.rec[(y0+py)*g.w+x0 : (y0+py)*g.w+x0+n]
			for px := range row {
				v := bgv
				if int64(bits) < 0 {
					v = fgv
				}
				row[px] = v
				bits <<= 1
			}
		}
	} else {
		for py := 0; py < n; py++ {
			row := st.rec[(y0+py)*g.w+x0 : (y0+py)*g.w+x0+n]
			for px := range row {
				row[px] = b.fg
			}
		}
	}
	var t uint8
	if b.pattern {
		t = 1
	}
	cells := n >> g.shift
	for yy := cy; yy < cy+cells; yy++ {
		for xx := cx; xx < cx+cells; xx++ {
			i := yy*g.cellsX + xx
			st.cellType[i], st.cellFg[i], st.cellCon[i], st.cellDone[i] = t, uint8(fgIdx), uint8(con), true
		}
	}
	return nil
}

// codeMainBand codes macro-group rows [rows[0], rows[1]). For encoding,
// blocks holds the band's blocks in order; for decoding they are appended.
func (st *smoothChannelState) codeMainBand(sc *symCoder, band *smoothBand, blocks []smoothBlock) ([]smoothBlock, error) {
	g := &st.geo
	m := newSmoothGlyphModels()
	k := 0
	next := func(x, y, size int) (*smoothBlock, error) {
		if sc.e != nil {
			if k >= len(blocks) || int(blocks[k].x) != x || int(blocks[k].y) != y || int(blocks[k].size) != size {
				return nil, fmt.Errorf("unexpected block layout")
			}
			k++
			return &blocks[k-1], nil
		}
		band.count++
		if band.discard {
			band.tmp = smoothBlock{x: int32(x), y: int32(y), size: uint8(size)}
			return &band.tmp, nil
		}
		blocks = append(blocks, smoothBlock{x: int32(x), y: int32(y), size: uint8(size)})
		return &blocks[len(blocks)-1], nil
	}
	for gy := band.rows[0]; gy < band.rows[1]; gy++ {
		for gx := 0; gx < g.groupsX; gx++ {
			if st.gradients != nil {
				run := st.gradients.runs[gy*g.groupsX+gx]
				if run.length != 0 {
					count := int(run.length)
					band.lastIdx = int(st.levels.idx[run.level])
					if sc.d != nil {
						band.count += count
						if !band.discard {
							for k := 0; k < count; k++ {
								blocks = append(blocks, smoothBlock{x: int32((gx + k) * g.macro), y: int32(gy * g.macro), size: uint8(g.macro), fg: run.level, bg: run.level})
							}
						}
					}
					gx += count - 1
					continue
				}
			}
			x0, y0 := gx*g.macro, gy*g.macro
			l, t := uint32(2), uint32(2)
			if gx > 0 {
				l = uint32(st.groupBig[gy*g.groupsX+gx-1])
			}
			if gy > 0 && (gy-1)*g.macro >= band.y0 {
				t = uint32(st.groupBig[(gy-1)*g.groupsX+gx])
			}
			var big uint32
			if sc.e != nil && k < len(blocks) && int(blocks[k].size) == g.macro && int(blocks[k].x) == x0 && int(blocks[k].y) == y0 {
				big = 1
			}
			big = sc.bit(&m.size[l*3+t], big)
			st.groupBig[gy*g.groupsX+gx] = uint8(big)
			if big != 0 {
				b, err := next(x0, y0, g.macro)
				if err != nil {
					return nil, err
				}
				if err := st.codeBlock(sc, m, band, b); err != nil {
					return nil, err
				}
				continue
			}
			for by := 0; by < g.macro; by += g.small {
				for bx := 0; bx < g.macro; bx += g.small {
					b, err := next(x0+bx, y0+by, g.small)
					if err != nil {
						return nil, err
					}
					if err := st.codeBlock(sc, m, band, b); err != nil {
						return nil, err
					}
				}
			}
		}
	}
	if sc.e != nil && k != len(blocks) {
		return nil, fmt.Errorf("unexpected block count")
	}
	return blocks, nil
}

func (st *smoothChannelState) codeStripes(sc *symCoder, band *smoothBand, blocks []smoothBlock) ([]smoothBlock, error) {
	g := &st.geo
	m := newSmoothGlyphModels()
	k := 0
	visit := func(x, y int) error {
		var b *smoothBlock
		if sc.e != nil {
			if k >= len(blocks) || int(blocks[k].x) != x || int(blocks[k].y) != y || int(blocks[k].size) != g.small {
				return fmt.Errorf("unexpected stripe layout")
			}
			b = &blocks[k]
			k++
		} else if band.count++; band.discard {
			band.tmp = smoothBlock{x: int32(x), y: int32(y), size: uint8(g.small)}
			b = &band.tmp
		} else {
			blocks = append(blocks, smoothBlock{x: int32(x), y: int32(y), size: uint8(g.small)})
			b = &blocks[len(blocks)-1]
		}
		return st.codeBlock(sc, m, band, b)
	}
	for y := 0; y < g.fullH; y += g.small {
		for x := g.fullW; x < g.w4; x += g.small {
			if err := visit(x, y); err != nil {
				return nil, err
			}
		}
	}
	for y := g.fullH; y < g.h4; y += g.small {
		for x := 0; x < g.w4; x += g.small {
			if err := visit(x, y); err != nil {
				return nil, err
			}
		}
	}
	if sc.e != nil && k != len(blocks) {
		return nil, fmt.Errorf("unexpected stripe count")
	}
	return blocks, nil
}

// parseSmoothChannel reads a validated BRC1 channel segment into blocks in
// coding order. Patterns are converted to raster order.
func parseSmoothChannel(seg []byte, geo smoothGeometry) ([]smoothBlock, error) {
	var streams [5][]byte
	pos := 4
	for s := range streams {
		n := int(binary.BigEndian.Uint32(seg[pos:]))
		streams[s] = seg[pos+4 : pos+4+n]
		pos += 4 + n
	}
	sizes, types, patterns := newBitReader(streams[0]), newBitReader(streams[1]), newBitReader(streams[2])
	fg, bg := streams[3], streams[4]
	var fgv, bgv byte
	fi, bi := 0, 0
	blocks := make([]smoothBlock, 0, int(binary.BigEndian.Uint32(seg)))
	add := func(x, y, size int) error {
		if fi >= len(fg) {
			return fmt.Errorf("short foreground stream")
		}
		fgv += fg[fi]
		fi++
		b := smoothBlock{x: int32(x), y: int32(y), size: uint8(size), fg: fgv, bg: fgv}
		b.pattern = types.readBitFast()
		if b.pattern {
			if bi >= len(bg) {
				return fmt.Errorf("short background stream")
			}
			bgv += bg[bi]
			bi++
			b.bg = bgv
			bits, err := readPatternComposite(&patterns, size, size)
			if err != nil {
				return err
			}
			b.bits = bits
		}
		blocks = append(blocks, b)
		return nil
	}
	for gy := 0; gy < geo.groupsY; gy++ {
		for gx := 0; gx < geo.groupsX; gx++ {
			x0, y0 := gx*geo.macro, gy*geo.macro
			if sizes.readBitFast() {
				if err := add(x0, y0, geo.macro); err != nil {
					return nil, err
				}
				continue
			}
			for by := 0; by < geo.macro; by += geo.small {
				for bx := 0; bx < geo.macro; bx += geo.small {
					if err := add(x0+bx, y0+by, geo.small); err != nil {
						return nil, err
					}
				}
			}
		}
	}
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
	if fi != len(fg) || bi != len(bg) {
		return nil, fmt.Errorf("inconsistent channel streams")
	}
	return blocks, nil
}

// appendSmoothChannel serializes blocks as a BRC1 channel record.
func appendSmoothChannel(dst []byte, parts [][]smoothBlock, geo smoothGeometry) []byte {
	var sizeBuf, typeBuf, patternBuf bytes.Buffer
	sizeW, typeW, patternW := newBitWriter(&sizeBuf), newBitWriter(&typeBuf), newBitWriter(&patternBuf)
	count := 0
	for _, blocks := range parts {
		count += len(blocks)
	}
	typeBuf.Grow(count/8 + 1)
	fg := make([]byte, 0, count)
	bg := make([]byte, 0, count)
	var prevFg, prevBg byte
	for _, blocks := range parts {
		for i := range blocks {
			b := &blocks[i]
			if int(b.size) == geo.macro && int(b.x) < geo.fullW && int(b.y) < geo.fullH {
				sizeW.writeBit(true)
			} else if int(b.x) < geo.fullW && int(b.y) < geo.fullH && int(b.x)%geo.macro == 0 && int(b.y)%geo.macro == 0 {
				sizeW.writeBit(false)
			}
			typeW.writeBit(b.pattern)
			fg = append(fg, b.fg-prevFg)
			prevFg = b.fg
			if b.pattern {
				bg = append(bg, b.bg-prevBg)
				prevBg = b.bg
				writePatternComposite(&patternW, b.bits, int(b.size), int(b.size))
			}
		}
	}
	sizeW.flush()
	typeW.flush()
	patternW.flush()
	dst = binary.BigEndian.AppendUint32(dst, uint32(count))
	for _, s := range [][]byte{sizeBuf.Bytes(), typeBuf.Bytes(), patternBuf.Bytes(), fg, bg} {
		dst = binary.BigEndian.AppendUint32(dst, uint32(len(s)))
		dst = append(dst, s...)
	}
	return dst
}
