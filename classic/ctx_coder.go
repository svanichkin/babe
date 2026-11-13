package classic

import "errors"

// Adaptive binary range coding for context-modelled streams. Each bit is coded
// with the probability of its context's model, which then adapts to the bit.
// Carry propagation follows rangeEncoder.

var errCtxStream = errors.New("invalid context-coded stream")

// bitModel adapts with rate 1/(n+1.5) for its first observations, then with
// a fixed rate, so rarely used contexts learn quickly.
type bitModel struct {
	prob uint16 // probability of a one bit, 1/65536
	n    uint16
}

const bitModelLimit = 80

var bitModelNext = func() [bitModelLimit + 1]uint16 {
	var t [bitModelLimit + 1]uint16
	for n := range t {
		t[n] = uint16(min(n+1, bitModelLimit))
	}
	return t
}()

var bitModelRate = func() [bitModelLimit + 1]uint32 {
	var t [bitModelLimit + 1]uint32
	for n := range t {
		t[n] = uint32(65536 * 2 / (2*n + 3))
	}
	return t
}()

func resetBitModels(m []bitModel) {
	for i := range m {
		m[i] = bitModel{prob: 1 << 15}
	}
}

// p returns the probability of a one bit in [1, 65535] / 65536. The update
// rule never leaves that range: a step moves prob by less than its distance
// to 0 or 65535. With a range of at least 2^24 both coded intervals are then
// non-empty.
func (m *bitModel) p() uint32 { return uint32(m.prob) }

func (m *bitModel) update(bit uint32) {
	r := bitModelRate[m.n]
	m.n = bitModelNext[m.n]
	if bit != 0 {
		m.prob += uint16((uint32(0xffff-m.prob) * r) >> 16)
	} else {
		m.prob -= uint16((uint32(m.prob) * r) >> 16)
	}
}

type ctxEncoder struct {
	low     uint64
	rng     uint32
	cache   byte
	pending int
	started bool
	out     []byte
}

func newCtxEncoder(dst []byte) *ctxEncoder {
	return &ctxEncoder{rng: ^uint32(0), out: dst[:0]}
}

func (e *ctxEncoder) shift() {
	low, carry := uint32(e.low), byte(e.low>>32)
	e.low = uint64(low << 8)
	if low < 0xff000000 || carry != 0 {
		if e.started {
			e.out = append(e.out, e.cache+carry)
		}
		e.started = true
		for ; e.pending > 0; e.pending-- {
			e.out = append(e.out, 0xff+carry)
		}
		e.cache = byte(low >> 24)
	} else {
		e.pending++
	}
}

func (e *ctxEncoder) encode(m *bitModel, bit uint32) {
	bound := (e.rng >> 16) * m.p()
	if bit != 0 {
		e.rng = bound
	} else {
		e.low += uint64(bound)
		e.rng -= bound
	}
	m.update(bit)
	for e.rng < 1<<24 {
		e.rng <<= 8
		e.shift()
	}
}

// encodeDirect writes n equiprobable bits, most significant first.
func (e *ctxEncoder) encodeDirect(v uint32, n int) {
	for i := n - 1; i >= 0; i-- {
		e.rng >>= 1
		if (v>>i)&1 == 0 {
			e.low += uint64(e.rng)
		}
		for e.rng < 1<<24 {
			e.rng <<= 8
			e.shift()
		}
	}
}

func (e *ctxEncoder) finish() []byte {
	for i := 0; i < 5; i++ {
		e.shift()
	}
	return e.out
}

type ctxDecoder struct {
	rng, code uint32
	src       []byte
	pos       int
	overrun   int
}

func newCtxDecoder(src []byte) *ctxDecoder {
	d := &ctxDecoder{rng: ^uint32(0), src: src}
	for i := 0; i < 4; i++ {
		d.code = d.code<<8 | uint32(d.next())
	}
	return d
}

// next returns zero bytes past the end; err reports a materially overrun stream.
func (d *ctxDecoder) next() byte {
	if d.pos < len(d.src) {
		b := d.src[d.pos]
		d.pos++
		return b
	}
	d.overrun++
	return 0
}

func (d *ctxDecoder) err() error {
	// The encoder's final flush emits four bytes that the decoder may never
	// read; reading more than that many past the end means corruption.
	if d.overrun > 4 {
		return errCtxStream
	}
	return nil
}

func (d *ctxDecoder) decode(m *bitModel) uint32 {
	bound := (d.rng >> 16) * uint32(m.prob)
	var bit uint32
	if d.code < bound {
		d.rng = bound
		bit = 1
	} else {
		d.code -= bound
		d.rng -= bound
	}
	m.update(bit)
	if d.rng < 1<<24 {
		d.normalize()
	}
	return bit
}

func (d *ctxDecoder) normalize() {
	for d.rng < 1<<24 {
		d.rng <<= 8
		d.code = d.code<<8 | uint32(d.next())
	}
}

func (d *ctxDecoder) decodeDirect(n int) uint32 {
	var v uint32
	for i := 0; i < n; i++ {
		d.rng >>= 1
		bit := uint32(1)
		if d.code >= d.rng {
			d.code -= d.rng
			bit = 0
		}
		v = v<<1 | bit
		for d.rng < 1<<24 {
			d.rng <<= 8
			d.code = d.code<<8 | uint32(d.next())
		}
	}
	return v
}

// uintModel codes a non-negative integer as a class and a mantissa. Class 0
// is 0, class 1 is 1, and class k >= 2 holds [2^(k-1), 2^k). The class is
// coded in unary with one model per position; the mantissa's top two bits
// below the leading one are modelled per class, the rest are direct.
const uintClasses = 18

type uintModel [uintClasses + 2*uintClasses]bitModel

func uintClass(v uint32) int {
	if v < 2 {
		return int(v)
	}
	k := 1
	for v>>k != 0 {
		k++
	}
	return k
}

func (e *ctxEncoder) encodeUint(m *uintModel, v uint32) {
	k := uintClass(v)
	for i := 0; i < k; i++ {
		e.encode(&m[i], 1)
	}
	if k < uintClasses-1 {
		e.encode(&m[k], 0)
	}
	if k < 2 {
		return
	}
	bits := k - 1 // mantissa bits below the leading one
	for i := bits - 1; i >= 0; i-- {
		bit := (v >> i) & 1
		switch {
		case i == bits-1:
			e.encode(&m[uintClasses+2*k], bit)
		case i == bits-2:
			e.encode(&m[uintClasses+2*k+1], bit)
		default:
			e.encodeDirect(bit, 1)
		}
	}
}

func (d *ctxDecoder) decodeUint(m *uintModel) uint32 {
	k := 0
	for k < uintClasses-1 && d.decode(&m[k]) != 0 {
		k++
	}
	if k < 2 {
		return uint32(k)
	}
	bits := k - 1
	v := uint32(1)
	for i := bits - 1; i >= 0; i-- {
		var bit uint32
		switch {
		case i == bits-1:
			bit = d.decode(&m[uintClasses+2*k])
		case i == bits-2:
			bit = d.decode(&m[uintClasses+2*k+1])
		default:
			bit = d.decodeDirect(1)
		}
		v = v<<1 | bit
	}
	return v
}
