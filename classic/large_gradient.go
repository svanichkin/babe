package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"math"
)

// A large gradient replaces the Y glyphs in a 16, 32 or 64 pixel square.
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
func stitchLargeGradients(plane, sizes []byte, w, h int) []byte {
	out := append([]byte(nil), plane...)
	const radius, threshold = 4, 24
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			i := y*w + x
			size := int(sizes[i])
			if size == 0 {
				continue
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
			out[i] = byte(max(0, min(255, int(plane[i])+correction)))
		}
	}
	return out
}

func selectLargeGradients(plane []byte, w, h, quality int) []largeGradient {
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
		sum, sx, sy := 0.0, 0.0, 0.0
		for y := 0; y < n; y++ {
			for x := 0; x < n; x++ {
				v := float64(plane[(y0+y)*w+x0+x])
				sum += v
				sx += v * float64(2*x-(n-1))
				sy += v * float64(2*y-(n-1))
			}
		}
		mean := sum / float64(n*n)
		denom := float64(n*n*(n*n-1)) / 3
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
		errSum, maxDiff := 0, 0
		for y := 0; y < n; y++ {
			for x := 0; x < n; x++ {
				d := int(plane[(y0+y)*w+x0+x]) - int(gradientAt(b, x0+x, y0+y))
				if d < 0 {
					d = -d
				}
				errSum += d * d
				maxDiff = max(maxDiff, d)
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
	for y := 0; y < h; y += 64 {
		for x := 0; x < w; x += 64 {
			visit(x, y, 64)
		}
	}
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
		b.x = int(binary.BigEndian.Uint32(p))
		b.y = int(binary.BigEndian.Uint32(p[4:]))
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
		if b.x > w-b.size || b.y > h-b.size {
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
