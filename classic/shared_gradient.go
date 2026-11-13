package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"math/bits"
)

// BGG2 stores each 16-pixel boundary vertex once, even when blocks of
// different sizes meet there. Block interiors use a Coons patch over its
// piecewise-linear boundary, ensuring that both sides use the same edge.
const sharedGradientMagic = "BGG2"
const compactSharedGradientMagic = "BGG3"
const sharedGradientHeaderSize = 28
const sharedGradientRecordSize = 9

type sharedGradientMesh struct {
	width, height, cols int
	values              []byte
}

func makeSharedGradientMesh(plane []byte, w, h int) *sharedGradientMesh {
	cols, rows := w/16+1, h/16+1
	m := &sharedGradientMesh{width: w, height: h, cols: cols, values: make([]byte, cols*rows)}
	for gy := 0; gy < rows; gy++ {
		for gx := 0; gx < cols; gx++ {
			px, py := gx*16, gy*16
			sum, n := 0, 0
			for y := max(0, py-1); y <= min(h-1, py); y++ {
				for x := max(0, px-1); x <= min(w-1, px); x++ {
					sum += int(plane[y*w+x])
					n++
				}
			}
			m.values[gy*cols+gx] = byte((sum + n/2) / n)
		}
	}
	return m
}
func (m *sharedGradientMesh) node(x, y int) int { return int(m.values[(y/16)*m.cols+x/16]) }
func (m *sharedGradientMesh) horizontal(x2, y int) int {
	x := x2 / 32 * 16
	t := x2 % 32
	a := m.node(x, y)
	if t == 0 {
		return a
	}
	b := m.node(x+16, y)
	return (a*(32-t) + b*t + 16) / 32
}
func (m *sharedGradientMesh) vertical(x, y2 int) int {
	y := y2 / 32 * 16
	t := y2 % 32
	a := m.node(x, y)
	if t == 0 {
		return a
	}
	b := m.node(x, y+16)
	return (a*(32-t) + b*t + 16) / 32
}
func (m *sharedGradientMesh) at(b largeGradient, x, y int) byte {
	u, v := 2*(x-b.x)+1, 2*(y-b.y)+1
	den := 2 * b.size
	shift := uint(bits.TrailingZeros(uint(b.size)) + 1)
	top := m.horizontal(2*x+1, b.y)
	bottom := m.horizontal(2*x+1, b.y+b.size)
	left := m.vertical(b.x, 2*y+1)
	right := m.vertical(b.x+b.size, 2*y+1)
	a, bb := m.node(b.x, b.y), m.node(b.x+b.size, b.y)
	c, d := m.node(b.x, b.y+b.size), m.node(b.x+b.size, b.y+b.size)
	lr := (left*(den-u) + right*u + den/2) >> shift
	tb := (top*(den-v) + bottom*v + den/2) >> shift
	corner := (a*(den-u)*(den-v) + bb*u*(den-v) + c*(den-u)*v + d*u*v + den*den/2) >> (2 * shift)
	return byte(max(0, min(255, lr+tb-corner)))
}

// Reuse the horizontal boundary samples across all rows of a patch. Within
// a row, the left/right and corner terms are linear: advance their numerators
// instead of looking up the same mesh vertices for every pixel. Rounding stays
// at exactly the same three points as in at.
type sharedGradientPatch struct {
	top, bottom [256]int
	a, b, c, d  int
}

func (m *sharedGradientMesh) preparePatch(b largeGradient) sharedGradientPatch {
	p := sharedGradientPatch{
		a: m.node(b.x, b.y), b: m.node(b.x+b.size, b.y),
		c: m.node(b.x, b.y+b.size), d: m.node(b.x+b.size, b.y+b.size),
	}
	for x := 0; x < b.size; x++ {
		p.top[x] = m.horizontal(2*(b.x+x)+1, b.y)
		p.bottom[x] = m.horizontal(2*(b.x+x)+1, b.y+b.size)
	}
	return p
}

func (p *sharedGradientPatch) row(m *sharedGradientMesh, b largeGradient, y int, out []byte) {
	den, v := 2*b.size, 2*(y-b.y)+1
	shift := uint(bits.TrailingZeros(uint(b.size)) + 1)
	left, right := m.vertical(b.x, 2*y+1), m.vertical(b.x+b.size, 2*y+1)
	lr, lrStep := left*(den-1)+right+den/2, 2*(right-left)
	corner := p.a*(den-1)*(den-v) + p.b*(den-v) + p.c*(den-1)*v + p.d*v + den*den/2
	cornerStep := 2 * ((p.b-p.a)*(den-v) + (p.d-p.c)*v)
	for x := 0; x < b.size; x++ {
		tb := (p.top[x]*(den-v) + p.bottom[x]*v + den/2) >> shift
		out[x] = byte(max(0, min(255, (lr>>shift)+tb-(corner>>(2*shift)))))
		lr += lrStep
		corner += cornerStep
	}
}

func selectSharedGradients(plane []byte, w, h, quality int, m *sharedGradientMesh) []largeGradient {
	return selectSharedGradientsParallel(plane, w, h, quality, m, false)
}

// selectSharedGradientsParallel processes 256x256 tiles independently and
// returns their blocks in tile order, the same as the serial selection.
func selectSharedGradientsParallel(plane []byte, w, h, quality int, m *sharedGradientMesh, parallel bool) []largeGradient {
	if w < 16 || h < 16 {
		return nil
	}
	stats := makeGradientTileSums(plane, w, h, parallel)
	var tiles [][2]int
	for y := 0; y < h; y += 256 {
		for x := 0; x < w; x += 256 {
			tiles = append(tiles, [2]int{x, y})
		}
	}
	results := make([][]largeGradient, len(tiles))
	runParallel(len(tiles), parallel && w*h >= 1<<16, func(i int) {
		candidates := selectLargeGradientTile(plane, w, h, quality, &stats, tiles[i][0], tiles[i][1])
		results[i] = refineSharedGradients(plane, w, h, quality, m, candidates)
	})
	var out []largeGradient
	for _, r := range results {
		out = append(out, r...)
	}
	return out
}

func refineSharedGradients(plane []byte, w, h, quality int, m *sharedGradientMesh, candidates []largeGradient) []largeGradient {
	limit, maxError := 5.0, 10
	if quality <= 10 {
		limit, maxError = 16, 18
	}
	if quality == 0 {
		limit, maxError = 36, 26
	}
	var result []largeGradient
	var visit func(largeGradient)
	visit = func(b largeGradient) {
		var patch sharedGradientPatch
		var row [256]byte
		var sum int64
		maxDiff := 0
		lo, hi := 255, 0
	verifyPatch:
		for y := b.y; y < b.y+b.size; y++ {
			// Many candidates fail at their first few pixels. Avoid preparing
			// all edges (or a whole row) until the first row has passed.
			if y == b.y+1 {
				patch = m.preparePatch(b)
			}
			if y > b.y {
				patch.row(m, b, y, row[:b.size])
			}
			for x := b.x; x < b.x+b.size; x++ {
				predicted := int(row[x-b.x])
				if y == b.y {
					predicted = int(m.at(b, x, y))
				}
				lo = min(lo, predicted)
				hi = max(hi, predicted)
				diff := int(plane[y*w+x]) - predicted
				if diff < 0 {
					diff = -diff
				}
				sum += int64(diff * diff)
				maxDiff = max(maxDiff, diff)
				if maxDiff > maxError || sum > int64(limit)*int64(b.size*b.size) {
					break verifyPatch
				}
			}
		}
		// Reject a block that crosses a prominent real boundary at its perimeter.
		// The neighbour can then remain represented by ordinary glyphs.
		safe := hi-lo >= 4 && float64(sum)/float64(b.size*b.size) <= limit && maxDiff <= maxError
		for side := 0; safe && side < 4; side++ {
			total, count := 0, 0
			for k := 0; k < b.size; k++ {
				x, y, nx, ny := b.x+k, b.y, b.x+k, b.y-1
				switch side {
				case 1:
					x, y, nx, ny = b.x+k, b.y+b.size-1, b.x+k, b.y+b.size
				case 2:
					x, y, nx, ny = b.x, b.y+k, b.x-1, b.y+k
				case 3:
					x, y, nx, ny = b.x+b.size-1, b.y+k, b.x+b.size, b.y+k
				}
				if nx < 0 || ny < 0 || nx >= w || ny >= h {
					continue
				}
				d := int(plane[y*w+x]) - int(plane[ny*w+nx])
				if d < 0 {
					d = -d
				}
				total += d
				count++
			}
			if count > 0 && total > count*12 {
				safe = false
				break
			}
		}
		if safe && hi-lo >= 4 && float64(sum)/float64(b.size*b.size) <= limit && maxDiff <= maxError {
			result = append(result, b)
			return
		}
		if b.size > 16 {
			n := b.size / 2
			for y := 0; y < 2; y++ {
				for x := 0; x < 2; x++ {
					visit(largeGradient{x: b.x + x*n, y: b.y + y*n, size: n})
				}
			}
		}
	}
	for _, b := range candidates {
		visit(b)
	}
	return result
}

func sharedNodeMask(blocks []largeGradient, w, h int) []bool {
	cols := w/16 + 1
	used := make([]bool, cols*(h/16+1))
	for _, b := range blocks {
		for k := 0; k <= b.size; k += 16 {
			used[(b.y/16)*cols+(b.x+k)/16] = true
			used[((b.y+b.size)/16)*cols+(b.x+k)/16] = true
			used[((b.y+k)/16)*cols+b.x/16] = true
			used[((b.y+k)/16)*cols+(b.x+b.size)/16] = true
		}
	}
	return used
}
func packSharedGradients(base []byte, blocks []largeGradient, m *sharedGradientMesh) []byte {
	used := sharedNodeMask(blocks, m.width, m.height)
	raw := make([]byte, 0, len(blocks)*3+len(used))
	for _, b := range blocks {
		code := byte(0)
		for size := b.size; size > 16; size /= 2 {
			code++
		}
		raw = append(raw, code)
		raw = binary.AppendUvarint(raw, uint64(b.x/16))
		raw = binary.AppendUvarint(raw, uint64(b.y/16))
	}
	prev := byte(0)
	for i, yes := range used {
		if yes {
			v := m.values[i]
			raw = append(raw, v-prev)
			prev = v
		}
	}
	packed := encodeByteRangeInto(nil, raw)
	out := make([]byte, 0, sharedGradientHeaderSize+len(base)+len(packed))
	out = append(out, compactSharedGradientMagic...)
	for _, v := range []uint32{uint32(m.width), uint32(m.height), uint32(len(base)), uint32(len(raw)), uint32(len(blocks))} {
		out = binary.BigEndian.AppendUint32(out, v)
	}
	hash := crc32.NewIEEE()
	hash.Write(base)
	hash.Write(raw)
	out = binary.BigEndian.AppendUint32(out, hash.Sum32())
	out = append(out, base...)
	return append(out, packed...)
}
func unpackSharedGradients(data []byte) ([]byte, []largeGradient, *sharedGradientMesh, error) {
	if len(data) < sharedGradientHeaderSize || (string(data[:4]) != sharedGradientMagic && string(data[:4]) != compactSharedGradientMagic) {
		return nil, nil, nil, fmt.Errorf("invalid shared gradient header")
	}
	w, h := int(binary.BigEndian.Uint32(data[4:])), int(binary.BigEndian.Uint32(data[8:]))
	if err := checkImageSize(w, h); err != nil {
		return nil, nil, nil, err
	}
	baseLen, rawLen, count := uint64(binary.BigEndian.Uint32(data[12:])), uint64(binary.BigEndian.Uint32(data[16:])), uint64(binary.BigEndian.Uint32(data[20:]))
	minimumRecordSize := uint64(sharedGradientRecordSize)
	if string(data[:4]) == compactSharedGradientMagic {
		minimumRecordSize = 3
	}
	if baseLen < 4 || baseLen > uint64(len(data)-sharedGradientHeaderSize) || count == 0 || count > uint64(w/16)*uint64(h/16) || rawLen < count*minimumRecordSize || rawLen > count*sharedGradientRecordSize+uint64(w/16+1)*uint64(h/16+1) {
		return nil, nil, nil, fmt.Errorf("invalid shared gradient lengths")
	}
	base := data[sharedGradientHeaderSize : sharedGradientHeaderSize+int(baseLen)]
	raw := make([]byte, int(rawLen))
	if err := decodeByteRangeInto(raw, data[sharedGradientHeaderSize+int(baseLen):]); err != nil {
		return nil, nil, nil, fmt.Errorf("invalid shared gradient data: %w", err)
	}
	hash := crc32.NewIEEE()
	hash.Write(base)
	hash.Write(raw)
	if hash.Sum32() != binary.BigEndian.Uint32(data[24:]) {
		return nil, nil, nil, fmt.Errorf("shared gradient checksum mismatch")
	}
	blocks := make([]largeGradient, int(count))
	pos := 0
	for i := range blocks {
		b := &blocks[i]
		if string(data[:4]) == compactSharedGradientMagic {
			if pos >= len(raw) || raw[pos] > 4 {
				return nil, nil, nil, fmt.Errorf("invalid compact gradient size")
			}
			b.size = 16 << raw[pos]
			pos++
			x, n := binary.Uvarint(raw[pos:])
			if n <= 0 || x > uint64(w/16) {
				return nil, nil, nil, fmt.Errorf("invalid compact gradient x")
			}
			pos += n
			y, n := binary.Uvarint(raw[pos:])
			if n <= 0 || y > uint64(h/16) {
				return nil, nil, nil, fmt.Errorf("invalid compact gradient y")
			}
			pos += n
			b.x, b.y = int(x)*16, int(y)*16
		} else {
			p := raw[pos : pos+sharedGradientRecordSize]
			x, y := binary.BigEndian.Uint32(p), binary.BigEndian.Uint32(p[4:])
			if uint64(x) > uint64(w) || uint64(y) > uint64(h) {
				return nil, nil, nil, fmt.Errorf("invalid shared gradient coordinates")
			}
			b.x = int(x)
			b.y = int(y)
			b.size = int(p[8])
			pos += sharedGradientRecordSize
			if b.size == 0 {
				b.size = 256
			}
		}
		if (b.size != 16 && b.size != 32 && b.size != 64 && b.size != 128 && b.size != 256) || b.x%b.size != 0 || b.y%b.size != 0 {
			return nil, nil, nil, fmt.Errorf("invalid shared gradient block")
		}
	}
	if err := validateLargeGradients(blocks, w, h); err != nil {
		return nil, nil, nil, err
	}
	used := sharedNodeMask(blocks, w, h)
	m := &sharedGradientMesh{width: w, height: h, cols: w/16 + 1, values: make([]byte, len(used))}
	prev := byte(0)
	for i, yes := range used {
		if yes {
			if pos >= len(raw) {
				return nil, nil, nil, fmt.Errorf("missing shared gradient vertex")
			}
			prev += raw[pos]
			pos++
			m.values[i] = prev
		}
	}
	if pos != len(raw) {
		return nil, nil, nil, fmt.Errorf("extra shared gradient vertices")
	}
	return base, blocks, m, nil
}
