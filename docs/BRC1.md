# BRC1 container

The encoder performs a single order-0 range-coding pass for each of the five
streams in each channel. If the result is not smaller than the original stream,
it stores raw bytes. Streams of five bytes or fewer are stored raw immediately,
since range termination alone takes five bytes. New files always use BRC1.

Encoding and decoding process independent streams with up to four workers and
reuse output buffers. Payloads smaller than 16 KiB stay serial. Setting the
encoder/decoder's `Parallel` field to false also disables entropy workers.

The decoder recognizes both BRC1 and previous Zstd-based files, including mixed
range/Zstd containers produced by the earlier exhaustive encoder. Older decoders
cannot read BRC1. The block payload itself retains its previous layout.

All multibyte integers are unsigned, big-endian. All bit streams are MSB-first.

| Field | Bytes | Meaning |
|---|---:|---|
| Magic | 4 | ASCII `BRC1` |
| Original payload header | 18 | `BABE\n` (5), small block size (2), macro block size (2), channel mask (1), width (4), height (4) |
| Checksum | 4 | CRC32/IEEE of the complete reconstructed original payload |
| Channel records | variable | Y, then Cb and Cr if their mask bits are set |

Channel mask bits 0, 1, 2 indicate Y, Cb, Cr respectively. Y is required.
Each channel record begins with a 4-byte block count, followed by five streams:

1. Macro block size flags (one bit per macro group).
2. Block type flags (one bit per block; 1 means a two-tone pattern).
3. Pattern bits (only for pattern blocks).
4. Delta-coded foreground levels (one byte per block).
5. Delta-coded background levels (one byte per pattern block).

Each stream contains a 4-byte raw length, a 4-byte stored length, a one-byte
method, and `stored length` bytes. Methods:

| Method | Representation |
|---:|---|
| 0 | Adaptive range-coded bytes, no previous-byte context; current encoder |
| 4 | Read compatibility: range context is the high nibble of the preceding byte |
| 8 | Read compatibility: range context is the whole preceding byte |
| 254 | Read compatibility: Zstd frame |
| 255 | Raw bytes; stored and raw lengths must be equal |

For each range stream, probabilities reset to 1024 out of 2048. Each context has
a binary tree for the eight bits of a byte. The initial previous byte is zero.
The probability update shifts by 5. The 32-bit interval is normalized below
`1 << 24`, with bytewise carry propagation. Five final shifts terminate the
stream. The implementation follows the carry convention of Igor Pavlov's
public-domain [LZMA SDK](https://github.com/ip7z/7zip/blob/main/C/LzmaEnc.c),
without its dictionary or match model.

The original payload is reconstructed by retaining its 18-byte header, channel
block counts and each stream's raw length/data, discarding the container's method
and stored length fields. CRC validation is followed by structural validation
before block decoding. Trailing bytes, unknown methods, inconsistent block/mask
counts and truncated streams are errors. Image dimensions are limited to
67,108,864 pixels, and decompressed payloads to 1 GiB.

## Quantization

Quantization happens before entropy coding, so the decoder needs no quality or
quantization setting. The encoder rounds each block's final levels to the nearest
multiple of the step, with half steps rounded upward and values clamped to 255:

| Quality | Y step | Cb/Cr step |
|---|---:|---:|
| 0–19 | 4 | 8 |
| 20–59 | 2 | 4 |
| 60–89 | 1 | 2 |
| 90–100 | 1 | 1 |

Thresholds and block subdivision decisions use the original input precision.
If both rounded levels are equal, the block becomes solid; no pattern bits or
background entry are emitted. BW stores only Y. At qualities below 90 the decoded
image can differ from previous builds at the same numeric quality. Entropy coding
and PNG compression add no further loss.
