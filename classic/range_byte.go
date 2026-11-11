package classic

import (
	"encoding/binary"
	"fmt"
	"io"
)

// Byte symbols replace the eight dependent binary decisions in the legacy
// coder. A local static model is transmitted for each block, keeping adaptation
// outside the hot loop. Normalization/carry use the same range convention.
const (
	byteRangeBlock = 32 << 10
	byteRangeBits  = 12
	byteRangeTotal = 1 << byteRangeBits
)

type byteRangeModel struct {
	freq [256]uint16
	cum  [256]uint16
}

func makeByteRangeModel(src []byte) (byteRangeModel, int, byte) {
	var counts [256]uint32
	for _, v := range src {
		counts[v]++
	}
	used, symbol := 0, byte(0)
	for i, n := range counts {
		if n != 0 {
			used++
			symbol = byte(i)
		}
	}
	var model byteRangeModel
	// Reserve one count for every used symbol, then distribute the remaining
	// probability mass cumulatively. The resulting total is exactly 4096.
	remaining := uint32(byteRangeTotal - used)
	var count, allocated, cum uint32
	for i, n := range counts {
		if n == 0 {
			continue
		}
		count += n
		next := count * remaining / uint32(len(src))
		freq := next - allocated + 1
		model.freq[i], model.cum[i] = uint16(freq), uint16(cum)
		cum += freq
		allocated = next
	}
	return model, used, symbol
}

// Block types: 0 = raw, 1 = repeated byte, 2 = bitmap model, 3 = sparse model.
// Range blocks contain uint32 stored length, model, and the range-coded bytes.
// The decoded block length is min(32768, remaining output length).
func encodeByteRangeInto(dst, src []byte) []byte {
	dst = dst[:0]
	for len(src) != 0 {
		n := min(byteRangeBlock, len(src))
		block := src[:n]
		src = src[n:]
		model, used, symbol := makeByteRangeModel(block)
		if used == 1 {
			dst = append(dst, 1, symbol)
			continue
		}
		start := len(dst)
		kind := byte(2)
		if used < 32 {
			kind = 3
		}
		dst = append(dst, kind, 0, 0, 0, 0)
		if kind == 2 {
			var bitmap [32]byte
			for i, freq := range model.freq {
				if freq != 0 {
					bitmap[i/8] |= 1 << uint(i%8)
				}
			}
			dst = append(dst, bitmap[:]...)
		} else {
			dst = append(dst, byte(used))
			for i, freq := range model.freq {
				if freq != 0 {
					dst = append(dst, byte(i))
				}
			}
		}
		left := used
		for _, freq := range model.freq {
			if freq == 0 {
				continue
			}
			left--
			if left == 0 {
				break
			} // Last frequency is implied by the total.
			dst = binary.AppendUvarint(dst, uint64(freq))
		}
		dst = encodeByteRangeBlock(dst, block, &model)
		if len(dst)-start >= n+1 {
			dst = append(dst[:start], 0)
			dst = append(dst, block...)
		} else {
			binary.BigEndian.PutUint32(dst[start+1:], uint32(len(dst)-start-5))
		}
	}
	return dst
}

func encodeByteRangeBlock(dst, src []byte, model *byteRangeModel) []byte {
	var intervals [256]uint32
	for i := range intervals {
		intervals[i] = uint32(model.cum[i])<<16 | uint32(model.freq[i])
	}
	span, low := ^uint32(0), uint64(0)
	cache, pending := byte(0), 0
	for _, v := range src {
		entry := intervals[v]
		unit := span >> byteRangeBits
		low += uint64((entry >> 16) * unit)
		span = (entry & 65535) * unit
		for span < 1<<24 {
			span <<= 8
			lo, carry := uint32(low), byte(low>>32)
			low = uint64(lo << 8)
			if lo < 0xff000000 || carry != 0 {
				dst = append(dst, cache+carry)
				for pending > 0 {
					dst = append(dst, 0xff+carry)
					pending--
				}
				cache = byte(lo >> 24)
			} else {
				pending++
			}
		}
	}
	e := rangeEncoder{low: low, cache: cache, pending: pending, out: dst}
	for i := 0; i < 5; i++ {
		e.shift()
	}
	// Two guard bytes permit a speculative normalization read without relying
	// on memory beyond the encoded stream. The decoder validates both guards.
	return append(e.out, 0, 0)
}

func readByteRangeModel(src []byte, kind byte, model *byteRangeModel) ([]byte, error) {
	var symbols [256]byte
	used := 0
	if kind == 2 {
		if len(src) < 32 {
			return nil, io.ErrUnexpectedEOF
		}
		for i := 0; i < 256; i++ {
			if src[i/8]&(1<<uint(i%8)) != 0 {
				symbols[used] = byte(i)
				used++
			}
		}
		src = src[32:]
	} else {
		if len(src) < 1 {
			return nil, io.ErrUnexpectedEOF
		}
		used = int(src[0])
		src = src[1:]
		if used < 2 || used >= 32 || len(src) < used {
			return nil, fmt.Errorf("invalid sparse range model")
		}
		copy(symbols[:], src[:used])
		src = src[used:]
		for i := 1; i < used; i++ {
			if symbols[i] <= symbols[i-1] {
				return nil, fmt.Errorf("unordered range symbols")
			}
		}
	}
	if used < 2 {
		return nil, fmt.Errorf("invalid range model alphabet")
	}
	cum := uint64(0)
	for i := 0; i < used; i++ {
		freq := uint64(byteRangeTotal) - cum
		if i < used-1 {
			v, n := binary.Uvarint(src)
			if n <= 0 {
				return nil, fmt.Errorf("truncated range frequency")
			}
			src = src[n:]
			freq = v
		}
		if freq == 0 || freq >= byteRangeTotal || cum+freq > byteRangeTotal {
			return nil, fmt.Errorf("invalid range frequency")
		}
		v := symbols[i]
		model.cum[v] = uint16(cum)
		model.freq[v] = uint16(freq)
		cum += freq
	}
	return src, nil
}

func decodeByteRangeBlock(out, src []byte, model *byteRangeModel) error {
	if len(src) < 7 || src[0] != 0 || src[len(src)-1] != 0 || src[len(src)-2] != 0 {
		return fmt.Errorf("invalid byte range header")
	}
	// One lookup yields symbol, frequency and cumulative frequency. All fields
	// fit in 32 bits because constant blocks have a separate representation.
	var table [byteRangeTotal]uint32
	for symbol, freq := range model.freq {
		cum := uint32(model.cum[symbol])
		f := uint32(freq)
		entry := uint32(symbol) | (f << 8) | (cum << 20)
		for slot := cum; slot < cum+f; slot++ {
			table[slot] = entry
		}
	}
	span := ^uint32(0)
	code := binary.BigEndian.Uint32(src[1:5])
	pos := 5
	for i := range out {
		unit := span >> byteRangeBits
		slot := code / unit
		if slot >= byteRangeTotal {
			return fmt.Errorf("invalid range interval")
		}
		entry := table[slot]
		out[i] = byte(entry)
		code -= (entry >> 20) * unit
		span = ((entry >> 8) & (byteRangeTotal - 1)) * unit
		if pos >= len(src) {
			return io.ErrUnexpectedEOF
		}
		normalize := uint32(0)
		if span < 1<<24 {
			normalize = 1
		}
		shift := normalize * 8
		span <<= shift
		code = code<<shift | (uint32(src[pos]) & (0 - normalize))
		pos += int(normalize)
		// A rare minimum-frequency symbol can require a second byte.
		if span < 1<<24 {
			if pos >= len(src) {
				return io.ErrUnexpectedEOF
			}
			span <<= 8
			code = code<<8 | uint32(src[pos])
			pos++
		}
	}
	if pos != len(src)-2 || code != 0 {
		return fmt.Errorf("invalid byte range termination")
	}
	return nil
}

func decodeByteRangeInto(out, src []byte) error {
	for len(out) != 0 {
		n := min(byteRangeBlock, len(out))
		block := out[:n]
		out = out[n:]
		if len(src) == 0 {
			return io.ErrUnexpectedEOF
		}
		kind := src[0]
		src = src[1:]
		switch kind {
		case 0:
			if len(src) < n {
				return io.ErrUnexpectedEOF
			}
			copy(block, src[:n])
			src = src[n:]
		case 1:
			if len(src) == 0 {
				return io.ErrUnexpectedEOF
			}
			v := src[0]
			src = src[1:]
			for i := range block {
				block[i] = v
			}
		case 2, 3:
			if len(src) < 4 {
				return io.ErrUnexpectedEOF
			}
			stored := uint64(binary.BigEndian.Uint32(src))
			src = src[4:]
			if stored > uint64(len(src)) || stored > byteRangeBlock {
				return fmt.Errorf("invalid byte range block length")
			}
			data := src[:int(stored)]
			src = src[int(stored):]
			var model byteRangeModel
			coded, err := readByteRangeModel(data, kind, &model)
			if err != nil {
				return err
			}
			if err := decodeByteRangeBlock(block, coded, &model); err != nil {
				return err
			}
		default:
			return fmt.Errorf("unknown byte range block type %d", kind)
		}
	}
	if len(src) != 0 {
		return fmt.Errorf("trailing byte range data")
	}
	return nil
}
