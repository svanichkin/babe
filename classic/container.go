package classic

import (
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"io"
	"runtime"
	"sync"

	"github.com/klauspost/compress/zstd"
)

const (
	packedMagic       = "BRC1"
	payloadHeaderSize = 18
	maxImagePixels    = 1 << 26
	maxPayloadSize    = 1 << 30
	methodZstd        = 254
	methodRaw         = 255
)

type payloadHeader struct {
	small, macro, w, h, channels            int
	w4, h4, fullW, fullH, groups, maxBlocks int
}

func checkImageSize(w, h int) error {
	if w <= 0 || h <= 0 || uint64(w) > maxImagePixels || uint64(h) > maxImagePixels || uint64(w)*uint64(h) > maxImagePixels {
		return fmt.Errorf("invalid image size %dx%d (limit %d pixels)", w, h, maxImagePixels)
	}
	return nil
}

func parsePayloadHeader(data []byte) (payloadHeader, error) {
	var h payloadHeader
	if len(data) < payloadHeaderSize || string(data[:5]) != codec {
		return h, fmt.Errorf("invalid BABE header")
	}
	h.small, h.macro = int(binary.BigEndian.Uint16(data[5:])), int(binary.BigEndian.Uint16(data[7:]))
	if h.small < 1 || h.macro > 8 || h.macro < h.small || h.macro%h.small != 0 {
		return h, fmt.Errorf("invalid block sizes %d/%d", h.small, h.macro)
	}
	mask := data[9]
	if mask&channelFlagY == 0 || mask & ^byte(channelFlagY|channelFlagCb|channelFlagCr) != 0 {
		return h, fmt.Errorf("invalid channel mask %d", mask)
	}
	for _, flag := range []byte{channelFlagY, channelFlagCb, channelFlagCr} {
		if mask&flag != 0 {
			h.channels++
		}
	}
	h.w, h.h = int(binary.BigEndian.Uint32(data[10:])), int(binary.BigEndian.Uint32(data[14:]))
	if err := checkImageSize(h.w, h.h); err != nil {
		return h, err
	}
	h.w4, h.h4 = h.w/h.small*h.small, h.h/h.small*h.small
	if h.w4 == 0 || h.h4 == 0 {
		return h, fmt.Errorf("image smaller than block size")
	}
	h.fullW, h.fullH = h.w4/h.macro*h.macro, h.h4/h.macro*h.macro
	h.groups = (h.fullW / h.macro) * (h.fullH / h.macro)
	h.maxBlocks = (h.w4 / h.small) * (h.h4 / h.small)
	return h, nil
}

type payloadReader struct {
	data []byte
	pos  int
}

func (r *payloadReader) take(n int) ([]byte, error) {
	if n < 0 || n > len(r.data)-r.pos {
		return nil, io.ErrUnexpectedEOF
	}
	s := r.data[r.pos : r.pos+n]
	r.pos += n
	return s, nil
}
func (r *payloadReader) u32() (int, error) {
	b, err := r.take(4)
	if err != nil {
		return 0, err
	}
	v := uint64(binary.BigEndian.Uint32(b))
	if v > maxPayloadSize {
		return 0, fmt.Errorf("stream length exceeds limit")
	}
	return int(v), nil
}

func (h payloadHeader) checkLength(stream, n, blocks int) error {
	valid := false
	switch stream {
	case 0:
		valid = n == (h.groups+7)/8
	case 1:
		valid = n == (blocks+7)/8
	case 2:
		valid = n <= (h.w4*h.h4+7)/8
	case 3:
		valid = n == blocks
	case 4:
		valid = n <= blocks
	}
	if !valid {
		return fmt.Errorf("invalid stream %d length %d", stream, n)
	}
	return nil
}

// BRC1 stores the raw header, checksum, and five streams per channel.
// New files use a single order-0 range pass, or raw bytes if smaller. Methods
// 4/8 and Zstd remain supported when reading files from the exhaustive encoder.
type payloadStream struct {
	src, dst []byte
	size     int
	method   byte
	err      error
}

type payloadPacker struct{ buffers [15][]byte }

// Limit entropy workers to avoid oversubscribing the image conversion workers.
// Small payloads stay serial, and Parallel=false disables all entropy workers.
func processStreams(count, size int, parallel bool, fn func(int)) {
	workers := 1
	if parallel && size >= 16<<10 {
		workers = min(4, runtime.GOMAXPROCS(0), count)
	}
	if workers == 1 {
		for i := 0; i < count; i++ {
			fn(i)
		}
		return
	}
	var wg sync.WaitGroup
	for worker := 0; worker < workers; worker++ {
		wg.Add(1)
		go func(start int) {
			defer wg.Done()
			for i := start; i < count; i += workers {
				fn(i)
			}
		}(worker)
	}
	wg.Wait()
}

func (p *payloadPacker) compress(raw, dst []byte, parallel bool) ([]byte, error) {
	h, err := parsePayloadHeader(raw)
	if err != nil {
		return nil, err
	}
	r := payloadReader{data: raw, pos: payloadHeaderSize}
	var streams [15]payloadStream
	var blocks [3]int
	count := h.channels * 5
	for c := 0; c < h.channels; c++ {
		blocks[c], err = r.u32()
		if err != nil {
			return nil, err
		}
		for s := 0; s < 5; s++ {
			n, err := r.u32()
			if err != nil {
				return nil, err
			}
			src, err := r.take(n)
			if err != nil {
				return nil, err
			}
			streams[c*5+s] = payloadStream{src: src, size: n}
		}
	}
	if r.pos != len(raw) {
		return nil, fmt.Errorf("trailing payload bytes")
	}
	processStreams(count, len(raw), parallel, func(i int) {
		job := &streams[i]
		job.method = methodRaw
		job.dst = job.src
		if len(job.src) > 5 {
			p.buffers[i] = encodeRangeInto(p.buffers[i], job.src, 0)
			if len(p.buffers[i]) < len(job.src) {
				job.dst = p.buffers[i]
				job.method = 0
			}
		}
	})
	size := 26 + h.channels*4 + count*9
	for i := 0; i < count; i++ {
		size += len(streams[i].dst)
	}
	if cap(dst) < size {
		dst = make([]byte, 0, size)
	}
	out := append(dst[:0], packedMagic...)
	out = append(out, raw[:payloadHeaderSize]...)
	out = binary.BigEndian.AppendUint32(out, crc32.ChecksumIEEE(raw))
	for c := 0; c < h.channels; c++ {
		out = binary.BigEndian.AppendUint32(out, uint32(blocks[c]))
		for s := 0; s < 5; s++ {
			job := &streams[c*5+s]
			out = binary.BigEndian.AppendUint32(out, uint32(job.size))
			out = binary.BigEndian.AppendUint32(out, uint32(len(job.dst)))
			out = append(out, job.method)
			out = append(out, job.dst...)
		}
	}
	return out, nil
}

func decompressPayload(data, dst []byte, zd *zstd.Decoder, parallel bool) ([]byte, error) {
	if len(data) < 4 || string(data[:4]) != packedMagic {
		return zd.DecodeAll(data, dst[:0])
	}
	r := payloadReader{data: data, pos: 4}
	header, err := r.take(payloadHeaderSize)
	if err != nil {
		return nil, err
	}
	h, err := parsePayloadHeader(header)
	if err != nil {
		return nil, err
	}
	checksum, err := r.take(4)
	if err != nil {
		return nil, err
	}
	var streams [15]payloadStream
	var blocks [3]int
	count := h.channels * 5
	size := payloadHeaderSize + h.channels*4 + count*4
	hasZstd := false
	// Parse and bound every stream before allocating the output or launching jobs.
	for c := 0; c < h.channels; c++ {
		blocks[c], err = r.u32()
		if err != nil {
			return nil, err
		}
		if blocks[c] < 1 || blocks[c] > h.maxBlocks {
			return nil, fmt.Errorf("invalid block count %d", blocks[c])
		}
		for s := 0; s < 5; s++ {
			n, err := r.u32()
			if err != nil {
				return nil, err
			}
			if err := h.checkLength(s, n, blocks[c]); err != nil {
				return nil, err
			}
			stored, err := r.u32()
			if err != nil {
				return nil, err
			}
			method, err := r.take(1)
			if err != nil {
				return nil, err
			}
			packed, err := r.take(stored)
			if err != nil {
				return nil, err
			}
			switch method[0] {
			case methodRaw:
				if stored != n {
					return nil, fmt.Errorf("incorrect raw stream length")
				}
			case methodZstd:
				hasZstd = true
			case 0, 4, 8:
			default:
				return nil, fmt.Errorf("unsupported compression method %d", method[0])
			}
			streams[c*5+s] = payloadStream{src: packed, size: n, method: method[0]}
			size += n
		}
	}
	if r.pos != len(data) {
		return nil, fmt.Errorf("trailing compressed data")
	}
	if size > maxPayloadSize {
		return nil, fmt.Errorf("payload exceeds limit")
	}
	if cap(dst) < size {
		dst = make([]byte, size)
	} else {
		dst = dst[:size]
	}
	copy(dst, header)
	pos := payloadHeaderSize
	for c := 0; c < h.channels; c++ {
		binary.BigEndian.PutUint32(dst[pos:], uint32(blocks[c]))
		pos += 4
		for s := 0; s < 5; s++ {
			job := &streams[c*5+s]
			binary.BigEndian.PutUint32(dst[pos:], uint32(job.size))
			pos += 4
			// Full slice expressions cap Zstd output at this stream's validated length.
			job.dst = dst[pos : pos+job.size : pos+job.size]
			pos += job.size
		}
	}
	var bounded *zstd.Decoder
	if hasZstd {
		bounded, err = zstd.NewReader(nil, zstd.WithDecoderConcurrency(1), zstd.WithDecoderMaxMemory(maxPayloadSize), zstd.WithDecodeAllCapLimit(true))
		if err != nil {
			return nil, err
		}
		defer bounded.Close()
	}
	processStreams(count, size, parallel, func(i int) {
		job := &streams[i]
		switch job.method {
		case methodRaw:
			copy(job.dst, job.src)
		case methodZstd:
			var decoded []byte
			decoded, job.err = bounded.DecodeAll(job.src, job.dst[:0])
			if job.err == nil && len(decoded) != job.size {
				job.err = fmt.Errorf("incorrect Zstd stream length")
			}
		default:
			job.err = decodeRangeInto(job.dst, job.src, int(job.method))
		}
	})
	for i := 0; i < count; i++ {
		if streams[i].err != nil {
			return nil, streams[i].err
		}
	}
	if crc32.ChecksumIEEE(dst) != binary.BigEndian.Uint32(checksum) {
		return nil, fmt.Errorf("BABE checksum mismatch")
	}
	return dst, nil
}

// Validate the complete structure before starting the decoder's unchecked hot
// paths or worker goroutines. Length checks alone cannot validate the number of
// pattern/background values implied by the block size and type bitstreams.
func validatePayload(data []byte) error {
	h, err := parsePayloadHeader(data)
	if err != nil {
		return err
	}
	r := payloadReader{data: data, pos: payloadHeaderSize}
	for c := 0; c < h.channels; c++ {
		blocks, err := r.u32()
		if err != nil {
			return err
		}
		if blocks < 1 || blocks > h.maxBlocks {
			return fmt.Errorf("invalid block count")
		}
		var streams [5][]byte
		for s := range streams {
			n, err := r.u32()
			if err != nil {
				return err
			}
			if err := h.checkLength(s, n, blocks); err != nil {
				return err
			}
			streams[s], err = r.take(n)
			if err != nil {
				return err
			}
		}
		bit := func(s, i int) bool { return streams[s][i/8]&(1<<(7-i%8)) != 0 }
		count, patterns, patternBits := 0, 0, 0
		visit := func(area int) error {
			if count >= blocks {
				return fmt.Errorf("too few block values")
			}
			if bit(1, count) {
				if area == 1 {
					return fmt.Errorf("single-pixel block cannot have a pattern")
				}
				patterns++
				patternBits += area
			}
			count++
			return nil
		}
		for g := 0; g < h.groups; g++ {
			if bit(0, g) {
				if err := visit(h.macro * h.macro); err != nil {
					return err
				}
			} else {
				for i := 0; i < (h.macro/h.small)*(h.macro/h.small); i++ {
					if err := visit(h.small * h.small); err != nil {
						return err
					}
				}
			}
		}
		stripeBlocks := (h.w4*h.h4 - h.fullW*h.fullH) / (h.small * h.small)
		for i := 0; i < stripeBlocks; i++ {
			if err := visit(h.small * h.small); err != nil {
				return err
			}
		}
		if count != blocks || len(streams[4]) != patterns || len(streams[2]) != (patternBits+7)/8 {
			return fmt.Errorf("inconsistent channel streams")
		}
	}
	if r.pos != len(data) {
		return fmt.Errorf("trailing payload data")
	}
	return nil
}
