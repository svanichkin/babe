package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"math"
)

// A large gradient replaces the Y glyphs in a square up to 256 pixels wide.
// Its four corners represent a plane fitted to source luminance.
type largeGradient struct {
	x, y, size int
	corners    [4]byte
}

const gradientMagic = "BGG1"
const gradientRecordSize = 13

func gradientAt(b largeGradient, x, y int) byte {
	n := b.size - 1
	dx, dy := x-b.x, y-b.y
	a, c := int(b.corners[0]), int(b.corners[2])
	bb, dd := int(b.corners[1]), int(b.corners[3])
	v := a*(n-dx)*(n-dy) + bb*dx*(n-dy) + c*(n-dx)*dy + dd*dx*dy
	return byte((v + n*n/2) / (n * n))
}

// stitchLargeGradients anchors each gradient's boundary to its reconstructed
// neighbour and fades that correction into the gradient. Ordinary glyphs are
// read but never changed. Strong steps are left alone as possible real edges.
func stitchLargeGradients(plane, sizes []byte, w, h int, stitchNeighbours bool) []byte {
	out := append([]byte(nil), plane...)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if sizes[y*w+x] != 0 {
				out[y*w+x] = stitchPixel(plane, sizes, w, h, x, y, stitchNeighbours)
			}
		}
	}
	return out
}

// stitchGradientRows writes stitchLargeGradients's result for the pixels of
// blocks in rows [y0, y1) into out. It reads plane only inside the blocks and
// at their direct neighbours, so other entries of plane and out may be stale.
func stitchGradientRows(out, plane, sizes []byte, w, h int, blocks []largeGradient, stitchNeighbours bool, y0, y1 int) {
	for _, b := range blocks {
		for y := max(b.y, y0); y < min(b.y+b.size, y1); y++ {
			for x := b.x; x < b.x+b.size; x++ {
				out[y*w+x] = stitchPixel(plane, sizes, w, h, x, y, stitchNeighbours)
			}
		}
	}
}

func stitchPixel(plane, sizes []byte, w, h, x, y int, stitchNeighbours bool) byte {
	const radius, threshold = 4, 24
	i := y*w + x
	size := int(sizes[i])
	if size == 255 {
		size = 256
	}
	lx, ly := x%size, y%size
	correction := 0
	adjust := func(ex, ey, nx, ny, distance int) {
		if distance >= radius || nx < 0 || nx >= w || ny < 0 || ny >= h {
			return
		}
		edge, other := int(plane[ey*w+ex]), int(plane[ny*w+nx])
		diff := other - edge
		if diff < -threshold || diff > threshold {
			return
		}
		if sizes[ny*w+nx] != 0 && !stitchNeighbours {
			return
		}
		strength := 3
		if sizes[ny*w+nx] == 0 {
			strength = 6
		}
		correction += diff * strength * (radius - distance) / (8 * radius)
	}
	adjust(x-lx, y, x-lx-1, y, lx)
	adjust(x+size-1-lx, y, x+size-lx, y, size-1-lx)
	adjust(x, y-ly, x, y-ly-1, ly)
	adjust(x, y+size-1-ly, x, y+size-ly, size-1-ly)
	return byte(max(0, min(255, int(plane[i])+correction)))
}

// gradientTileSums keeps integral sums at 16-pixel granularity. Every gradient
// candidate is aligned to that grid, so its least-squares plane can be fitted
// without rescanning its pixels at every level of the subdivision tree.
type gradientTileSums struct {
	cols            int
	sum, xsum, ysum []int64
}

func makeGradientTileSums(plane []byte, w, h int, parallel bool) gradientTileSums {
	cols, rows := w/16+1, h/16+1
	s := gradientTileSums{cols: cols, sum: make([]int64, cols*rows), xsum: make([]int64, cols*rows), ysum: make([]int64, cols*rows)}
	// Per-tile moments are independent; the integral is accumulated afterwards.
	parallelRows(parallel, w, h/16, func(t0, t1 int) {
		for ty := t0; ty < t1; ty++ {
			for tx := 0; tx < w/16; tx++ {
				var sum, sx, sy int64
				for y := ty * 16; y < ty*16+16; y++ {
					row := plane[y*w+tx*16 : y*w+tx*16+16]
					for k, b := range row {
						v := int64(b)
						sum += v
						sx += int64(tx*16+k) * v
						sy += int64(y) * v
					}
				}
				i := (ty+1)*cols + tx + 1
				s.sum[i], s.xsum[i], s.ysum[i] = sum, sx, sy
			}
		}
	})
	for ty := 0; ty < h/16; ty++ {
		for tx := 0; tx < w/16; tx++ {
			i := (ty+1)*cols + tx + 1
			s.sum[i] += s.sum[i-1] + s.sum[i-cols] - s.sum[i-cols-1]
			s.xsum[i] += s.xsum[i-1] + s.xsum[i-cols] - s.xsum[i-cols-1]
			s.ysum[i] += s.ysum[i-1] + s.ysum[i-cols] - s.ysum[i-cols-1]
		}
	}
	return s
}

func (s gradientTileSums) planeMoments(x, y, size int) (float64, float64, float64) {
	x0, y0, x1, y1 := x/16, y/16, (x+size)/16, (y+size)/16
	get := func(v []int64) int64 { return v[y1*s.cols+x1] - v[y0*s.cols+x1] - v[y1*s.cols+x0] + v[y0*s.cols+x0] }
	sum, sx, sy := get(s.sum), get(s.xsum), get(s.ysum)
	return float64(sum), float64(2*sx - int64(2*x+size-1)*sum), float64(2*sy - int64(2*y+size-1)*sum)
}

func selectLargeGradients(plane []byte, w, h, quality int) []largeGradient {
	var blocks []largeGradient
	if w < 16 || h < 16 {
		return blocks
	}
	stats := makeGradientTileSums(plane, w, h, false)
	for y := 0; y < h; y += 256 {
		for x := 0; x < w; x += 256 {
			blocks = append(blocks, selectLargeGradientTile(plane, w, h, quality, &stats, x, y)...)
		}
	}
	return blocks
}

// selectLargeGradientTile fits the quadtree of one 256x256 tile.
func selectLargeGradientTile(plane []byte, w, h, quality int, stats *gradientTileSums, tx, ty int) []largeGradient {
	var blocks []largeGradient
	// Conservative error limits preserve small features. Each accepted 16x16
	// block replaces at least four 8x8 glyphs, offsetting its metadata.
	limit, maxError := 5.0, 10
	if quality <= 10 {
		limit, maxError = 16, 18
	}
	if quality == 0 {
		limit, maxError = 36, 26
	}
	var visit func(int, int, int)
	visit = func(x0, y0, n int) {
		if x0+n > w || y0+n > h {
			if n > 16 {
				for y := 0; y < 2; y++ {
					for x := 0; x < 2; x++ {
						visit(x0+x*n/2, y0+y*n/2, n/2)
					}
				}
			}
			return
		}
		sum, sx, sy := stats.planeMoments(x0, y0, n)
		mean := sum / float64(n*n)
		denom := float64(n*n) * float64(n*n-1) / 3
		dx, dy := sx*float64(n-1)/denom, sy*float64(n-1)/denom
		vals := [4]float64{mean - dx - dy, mean + dx - dy, mean - dx + dy, mean + dx + dy}
		b := largeGradient{x: x0, y: y0, size: n}
		lo, hi := 255, 0
		for i, v := range vals {
			z := int(math.Round(v))
			z = max(0, min(255, z))
			b.corners[i] = byte(z)
			lo = min(lo, z)
			hi = max(hi, z)
		}
		var errSum int64
		maxDiff := 0
	verifyPlane:
		for y := 0; y < n; y++ {
			for x := 0; x < n; x++ {
				d := int(plane[(y0+y)*w+x0+x]) - int(gradientAt(b, x0+x, y0+y))
				if d < 0 {
					d = -d
				}
				errSum += int64(d * d)
				maxDiff = max(maxDiff, d)
				if maxDiff > maxError || errSum > int64(limit)*int64(n*n) {
					break verifyPlane
				}
			}
		}
		if hi-lo >= 4 && float64(errSum)/float64(n*n) <= limit && maxDiff <= maxError {
			blocks = append(blocks, b)
			return
		}
		if n > 16 {
			for y := 0; y < 2; y++ {
				for x := 0; x < 2; x++ {
					visit(x0+x*n/2, y0+y*n/2, n/2)
				}
			}
		}
	}
	visit(tx, ty, 256)
	return blocks
}

func packLargeGradients(base []byte, blocks []largeGradient) []byte {
	raw := make([]byte, 0, len(blocks)*gradientRecordSize)
	for _, b := range blocks {
		raw = binary.BigEndian.AppendUint32(raw, uint32(b.x))
		raw = binary.BigEndian.AppendUint32(raw, uint32(b.y))
		raw = append(raw, byte(b.size))
		raw = append(raw, b.corners[:]...)
	}
	packed := encodeByteRangeInto(nil, raw)
	out := make([]byte, 0, 16+len(base)+len(packed))
	out = append(out, gradientMagic...)
	out = binary.BigEndian.AppendUint32(out, uint32(len(base)))
	out = binary.BigEndian.AppendUint32(out, uint32(len(raw)))
	hash := crc32.NewIEEE()
	hash.Write(base)
	hash.Write(raw)
	out = binary.BigEndian.AppendUint32(out, hash.Sum32())
	out = append(out, base...)
	return append(out, packed...)
}

func unpackLargeGradients(data []byte) ([]byte, []largeGradient, error) {
	if len(data) < 16 || string(data[:4]) != gradientMagic {
		return nil, nil, fmt.Errorf("invalid gradient header")
	}
	baseLen, rawLen := uint64(binary.BigEndian.Uint32(data[4:])), uint64(binary.BigEndian.Uint32(data[8:]))
	if baseLen < 4 || baseLen > uint64(len(data)-16) || rawLen == 0 || rawLen%gradientRecordSize != 0 || rawLen > maxImagePixels/256*gradientRecordSize {
		return nil, nil, fmt.Errorf("invalid gradient lengths")
	}
	base := data[16 : 16+int(baseLen)]
	raw := make([]byte, int(rawLen))
	if err := decodeByteRangeInto(raw, data[16+int(baseLen):]); err != nil {
		return nil, nil, fmt.Errorf("invalid gradient data: %w", err)
	}
	hash := crc32.NewIEEE()
	hash.Write(base)
	hash.Write(raw)
	if hash.Sum32() != binary.BigEndian.Uint32(data[12:]) {
		return nil, nil, fmt.Errorf("gradient checksum mismatch")
	}
	blocks := make([]largeGradient, len(raw)/gradientRecordSize)
	for i := range blocks {
		p := raw[i*gradientRecordSize:]
		b := &blocks[i]
		x, y := binary.BigEndian.Uint32(p), binary.BigEndian.Uint32(p[4:])
		if uint64(x) > maxImagePixels || uint64(y) > maxImagePixels {
			return nil, nil, fmt.Errorf("invalid gradient coordinates")
		}
		b.x = int(x)
		b.y = int(y)
		b.size = int(p[8])
		copy(b.corners[:], p[9:13])
		if (b.size != 16 && b.size != 32 && b.size != 64) || b.x%b.size != 0 || b.y%b.size != 0 {
			return nil, nil, fmt.Errorf("invalid gradient block")
		}
	}
	return base, blocks, nil
}

func validateLargeGradients(blocks []largeGradient, w, h int) error {
	cols, rows := (w+15)/16, (h+15)/16
	used := make([]bool, cols*rows)
	for _, b := range blocks {
		if b.x < 0 || b.y < 0 || b.size <= 0 || b.x > w-b.size || b.y > h-b.size {
			return fmt.Errorf("gradient outside image")
		}
		for y := b.y / 16; y < (b.y+b.size)/16; y++ {
			for x := b.x / 16; x < (b.x+b.size)/16; x++ {
				i := y*cols + x
				if used[i] {
					return fmt.Errorf("overlapping gradients")
				}
				used[i] = true
			}
		}
	}
	return nil
}
