package classic

// Binary range coding with adaptive byte trees. Carry propagation follows the
// public-domain LZMA SDK (Igor Pavlov), without LZMA's dictionary or match model:
// https://github.com/ip7z/7zip/blob/main/C/LzmaEnc.c

import (
	"fmt"
	"io"
)

type rangeEncoder struct {
	low     uint64
	span    uint32
	cache   byte
	pending int
	out     []byte
}

func (e *rangeEncoder) shift() {
	low, carry := uint32(e.low), byte(e.low>>32)
	e.low = uint64(low << 8)
	if low < 0xff000000 || carry != 0 {
		e.out = append(e.out, e.cache+carry)
		for e.pending > 0 {
			e.out = append(e.out, 0xff+carry)
			e.pending--
		}
		e.cache = byte(low >> 24)
	} else {
		e.pending++
	}
}

func (e *rangeEncoder) bit(p *uint16, bit uint32) {
	bound := (e.span >> 11) * uint32(*p)
	if bit == 0 {
		e.span = bound
		*p += (2048 - *p) >> 5
	} else {
		e.low += uint64(bound)
		e.span -= bound
		*p -= *p >> 5
	}
	if e.span < 1<<24 {
		e.span <<= 8
		e.shift()
	}
}

// order is the number of high bits of the previous byte used as context.
func encodeRange(src []byte, order int) []byte {
	return encodeRangeInto(nil, src, order)
}

func encodeRangeInto(dst, src []byte, order int) []byte {
	capacity := len(src) + len(src)/16 + 16
	if cap(dst) < capacity {
		dst = make([]byte, 0, capacity)
	}
	e := rangeEncoder{span: ^uint32(0), out: dst[:0]}
	var small [256]uint16
	p := small[:]
	if order != 0 {
		p = make([]uint16, 256<<order)
	}
	for i := range p {
		p[i] = 1024
	}
	prev := 0
	for _, v := range src {
		base, node := (prev>>(8-order))*256, 1
		for k := 7; k >= 0; k-- {
			bit := uint32(v>>k) & 1
			e.bit(&p[base+node], bit)
			node = node*2 + int(bit)
		}
		prev = int(v)
	}
	for i := 0; i < 5; i++ {
		e.shift()
	}
	return e.out
}

func decodeRange(src []byte, n, order int) ([]byte, error) {
	if n < 0 || n > maxPayloadSize {
		return nil, fmt.Errorf("invalid range output length %d", n)
	}
	out := make([]byte, n)
	if err := decodeRangeInto(out, src, order); err != nil {
		return nil, err
	}
	return out, nil
}

func decodeRangeInto(out, src []byte, order int) error {
	if order != 0 && order != 4 && order != 8 {
		return fmt.Errorf("unsupported range model %d", order)
	}
	if len(src) < 5 || src[0] != 0 {
		return fmt.Errorf("invalid range stream header")
	}
	span, code, pos := ^uint32(0), uint32(0), 5
	for _, b := range src[:5] {
		code = code<<8 | uint32(b)
	}
	var small [256]uint16
	p := small[:]
	if order != 0 {
		p = make([]uint16, 256<<order)
	}
	for i := range p {
		p[i] = 1024
	}
	prev := 0
	for i := range out {
		base, node := (prev>>(8-order))*256, 1
		for k := 0; k < 8; k++ {
			prob := &p[base+node]
			bound := (span >> 11) * uint32(*prob)
			bit := 0
			if code < bound {
				span = bound
				*prob += (2048 - *prob) >> 5
			} else {
				bit = 1
				code -= bound
				span -= bound
				*prob -= *prob >> 5
			}
			if span < 1<<24 {
				if pos == len(src) {
					return io.ErrUnexpectedEOF
				}
				span <<= 8
				code = code<<8 | uint32(src[pos])
				pos++
			}
			node = node*2 + bit
		}
		out[i] = byte(node)
		prev = int(out[i])
	}
	if pos != len(src) || code != 0 {
		return fmt.Errorf("invalid range stream termination")
	}
	return nil
}
