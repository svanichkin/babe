# BRC1 container

The encoder uses byte-symbol range coding for each of the five streams in each
channel. Each block of up to 32 KiB gets a static frequency table. If the result
is not smaller than the original stream, it stores raw bytes. Streams of five
bytes or fewer are stored raw immediately. Lossy `-smooth` images with 4×4 or
8×8 macroblocks use the BSM2 container described below; other images use BRC1
and its optional corner/gradient wrappers.

Encoding and decoding process independent streams with up to four workers and
reuse output buffers. Payloads smaller than 16 KiB stay serial. Setting the
encoder/decoder's `Parallel` field to false also disables entropy workers.

The decoder recognizes both BRC1 and previous Zstd-based files, including mixed
range/Zstd containers produced by the earlier exhaustive encoder. Earlier BRC1
decoders do not support methods 16/17 or the new lossless color flag; Zstd-only
decoders cannot read BRC1 at all. Files produced by earlier encoders remain
readable, with their original decoded pixel values.

Fixed-width multibyte integers are unsigned, big-endian. Block pattern streams
are MSB-first. Method 16 uses unsigned varints and a bitmap as specified below.

| Field | Bytes | Meaning |
|---|---:|---|
| Magic | 4 | ASCII `BRC1` |
| Original payload header | 18 | `BABE\n` (5), small block size (2), macro block size (2), channel mask (1), width (4), height (4) |
| Checksum | 4 | CRC32/IEEE of the complete reconstructed original payload |
| Channel records | variable | Y, then Cb and Cr if their mask bits are set |

Channel mask bits 0, 1, 2 indicate the three color streams respectively. The
first is required. Bit 3 indicates the reversible lossless RGB transform below;
without it the streams use the original Y/Cb/Cr interpretation. Other bits are
invalid. Lossless mode requires small blocks of 1, macroblocks of 2, and either
all three channels or the first channel alone (for an already grayscale input).
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
| 0 | Read compatibility: adaptive binary range coding, no previous-byte context |
| 4 | Read compatibility: range context is the high nibble of the preceding byte |
| 8 | Read compatibility: range context is the whole preceding byte |
| 16 | Byte-symbol range coding with a static model per block; current encoder |
| 17 | Byte-symbol range coding of within-glyph foreground/background differences |
| 254 | Read compatibility: Zstd frame |
| 255 | Raw bytes; stored and raw lengths must be equal |

## Byte-symbol range coding (method 16)

Split the declared raw length into consecutive blocks of at most 32,768 bytes.
Each block starts with a one-byte kind:

| Kind | Following data |
|---:|---|
| 0 | Raw bytes, exactly the decoded block length |
| 1 | One byte repeated for the entire decoded block |
| 2 | Stored length (4 bytes), 32-byte symbol bitmap, frequencies, range data |
| 3 | Stored length (4 bytes), symbol count (1 byte, 2–31), sorted symbol bytes, frequencies, range data |

For kinds 2 and 3 the stored length includes everything after the length field
in that block. Bitmap bit `i % 8` of byte `i / 8` marks symbol `i`. Enumerate
present symbols in ascending order. Store the frequency of every symbol except
the last as an unsigned base-128 varint (low seven bits first, high bit means
continuation). Frequencies are positive and sum to 4096; the last is implied.
The encoder reserves one count per present symbol, then distributes the
remaining counts proportionally using cumulative integer rounding.

The interval starts at `0xffffffff`, low at zero. For each byte:
`unit = interval >> 12`, `low += cumulative[symbol] * unit`, and
`interval = frequency[symbol] * unit`. While the interval is below `1 << 24`,
shift it by eight and emit a byte with carry propagation. Five final low shifts
terminate the range data, followed by two zero guard bytes. The range data
starts with zero. The decoder initializes its code from the next four bytes,
uses `code / unit` to look up a symbol in a 4096-entry table, subtracts the
cumulative interval and normalizes. At completion the code must be zero and
exactly the two zero guard bytes must remain unread. These guards allow a
speculative normalization read within the stored buffer.

The encoder can store an individual block raw or as a repeated byte. It also
falls back to method 255 if the whole method-16 stream is not smaller than raw.
No Zstd or alternative context-model search is performed.

## Within-glyph contrast (method 17)

Method 17 is valid only for stream 5 (background levels). Its byte-range
representation is identical to method 16, but the symbols are `FG - BG` modulo
256 for each pattern glyph, rather than deltas between successive backgrounds.
After decompressing the channel streams, accumulate the foreground deltas from
zero. For each foreground whose type bit is 1, consume one contrast byte and
recover `BG = FG - contrast` modulo 256. Convert these backgrounds back to the
original delta stream, starting from zero, before checking the payload CRC.
There must be exactly one contrast byte per set type bit within the block count.

The encoder estimates both predictors' range-coded sizes from the normalized
frequency models, including model headers, and chooses method 17 when its
estimate is at least two bytes smaller. Only that candidate is range-encoded;
the original raw bytes are stored if the result is not smaller than raw.
The estimate avoids compressing each background stream twice, but does not
guarantee the absolute smaller of the two packed representations for every
input. It tries this for macroblocks of at least 4 pixels. The 1×1/2×2 path,
including lossless q100, keeps its existing representation to avoid the extra
prediction pass. This changes storage only; every reconstructed pixel is
unchanged. Decoders without method-17 support reject these new streams.

## Legacy binary range coding (methods 0, 4, 8)

For each legacy range stream, probabilities reset to 1024 out of 2048. Each context has
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

## Experimental BGC1 corner wrapper

An opt-in lossy encoder can wrap a normal BRC1 image with corner data. BGC1
is a separate experimental container; an older decoder rejects it. All fields
are big-endian. The wrapper contains magic `BGC1` (4 bytes), grid step (1 byte,
currently 8), width and height (4 bytes each), BRC1 byte length (4), encoded
corner-stream byte length (4), CRC32/IEEE (4), followed by the BRC1 bytes and
the corner stream. The checksum covers the BRC1 bytes followed by the decoded
corner stream. Sizes and dimensions must match exactly, and the base must be a
lossy BRC1 image of the same dimensions.

The corner grid has `ceil((width-1)/8)+1` columns and
`ceil((height-1)/8)+1` rows. The encoder estimates each source Y/Cb/Cr corner
from the source pixels immediately touching it. In row-major order, each byte
stores a channel's modulo-256 difference from the preceding corner in that row,
or from zero at the start of a row. Each corner contributes Y, Cb, Cr in that
order. The resulting bytes are encoded using BRC1 method 16, with their raw
length determined by the grid dimensions. BW uses neutral chroma.

During decoding, these source corner values provide bilinear gradients. The
decoder also retains a luma residual from the base block representation to
help preserve details. The same q100 input always produces ordinary lossless
BRC1, with no corner wrapper.

## Experimental BGC2 selective corners

BGC2 has the same 25-byte header and BRC1 base as BGC1, but a different corner
stream. Each grid node has four decoded bytes: a three-bit channel-presence
mask followed by modulo-256 residuals for Y, Cb and Cr. A residual is source
corner minus the corner estimated from the decoded BRC1 base; absent channels
must have zero residual. The stream is encoded with range method 16 and CRC32
covers the base and decoded four-byte records. The raw stream length is four
times the grid-node count. Invalid flag bits, nonzero omitted values and
checksum mismatches are errors.

The experimental encoder determines each channel's block sizes independently.
It stores a channel at a corner only when a neighbouring block meets the
requested minimum side length and the source and base estimates differ by more
than three levels. A missing channel uses the base estimate. All channels share
the same node coordinates, but use independent presence bits. The decoder
does not need the encoder's minimum to reconstruct the file. BGC2 is separate
from BGC1 so older decoders reject it explicitly. The encoder emits ordinary
lossless BRC1 for q100.

## Experimental BGC3 compact selective corners

`BGC3` stores the same corner records as BGC2 with the same 25-byte outer
header and checksum. Its corner payload starts with four big-endian 32-bit
lengths, followed by four byte-range streams: the flags for all vertices and
the selected Y, Cb and Cr residuals in vertex order. Omitted residuals consume
no bytes. If the top bit of the flags-stream length is set, each flag is
XOR-predicted from the previous flag in its row; the remaining 31 bits are
the actual length. The encoder chooses between BGC2 and BGC3 based on their
encoded sizes. When large gradients cover an entire luma corner's interpolation
footprint, that luma value can be omitted without changing reconstructed pixels.

## Experimental BGG1 large gradients

`BGG1` wraps a lossy BRC1, BGC1 or BGC2 base image with selected 16×16,
32×32 and 64×64 luminance planes. The 16-byte header contains the magic,
big-endian base length, uncompressed record length and CRC32 of the base
followed by the uncompressed records. The base image and one byte-range coded
record stream follow. Each 13-byte record contains big-endian 32-bit x and y,
one byte for the side length, and the four luma corners (top left, top right,
bottom left, bottom right). Blocks are aligned to their size, non-overlapping
and fully inside the base image. The decoder bilinearly interpolates luma;
chroma is still reconstructed from the base image. The current encoder stores
constant placeholder glyphs in the base image under each selected block.
Lossless images cannot contain this wrapper.

## Experimental BGG2 shared gradient vertices

`BGG2` also wraps a lossy BRC1, BGC1, BGC2 or BGC3 base image, but stores each boundary vertex once on
a 16-pixel grid. The 28-byte header contains the magic, then big-endian 32-bit
width, height, base length, raw record length, block count and CRC32 of the
base followed by the uncompressed record data. The base and a byte-range coded
record stream follow. A block record has 32-bit x and y and an 8-bit size.
After all block records, one delta-coded luminance byte is stored for each
vertex used by any block boundary, in row-major grid order; the first delta is
relative to zero. The block geometry determines which vertices are present.

Each block uses a Coons patch from piecewise-linear boundary curves, including
intermediate grid vertices where smaller blocks meet a larger one. Vertices
are sampled from source luminance at the grid junction; the encoder checks
approximation error and prominent image edges before accepting a block. The
decoder rejects overlapping or out-of-bounds blocks and inconsistent lengths.
Gradient boundaries next to ordinary glyphs still use the local correction
described in the README. BGG1 remains readable, and q100 is unaffected.

## Experimental BGG3 compact gradient records

`BGG3` uses the same 28-byte header, checksum, shared vertices and patch
interpolation as BGG2. Blocks may have side lengths 16, 32, 64, 128 or 256.
Instead of BGG2's nine-byte coordinates and size, each BGG3 block record is a
one-byte size code (0 through 4 for the five sizes), followed by unsigned
varints for x/16 and y/16. Delta-coded vertex bytes follow all block records.
BGG1 and BGG2 remain readable. Older decoders cannot read BGG3.

## BSM2 smooth container

`BSM2` stores a lossy `-smooth` image: BRC1 glyph channels, optional selective
corners (the BGC2 records) and optional shared-vertex gradients (as BGG3). It
reconstructs identical pixels. Unlike BSM1, it omits luma glyphs beneath
gradients; their baseline is implicit. BSM1 remains readable with explicit
glyphs and its original context-availability rules.
The current encoder writes it for 4×4 and 8×8 macroblocks; the format also
allows 2×2. Multibyte integers are big-endian.

| Offset | Bytes | Meaning |
|---:|---:|---|
| 0 | 4 | ASCII `BSM2` (or legacy `BSM1`) |
| 4 | 18 | BRC1 payload header; the lossless flag is invalid |
| 22 | 4 | CRC32/IEEE of every following byte |
| 26 | 1 | Flags: bit 0 gradients, bit 1 corners; other bits invalid |
| 27 | 1 | Corner minimum block side |
| 28 | 3 | Level step for Y, Cb, Cr (1–128) |
| 31 | 4 | Gradient block count |
| 35 | 1 | Stream count, followed by a 4-byte length per stream |

The streams follow the lengths without padding: the gradient stream (if
flagged); for each stored channel its band streams and then, if the image has
right or bottom small-block stripes, its stripe stream; and for each stored
channel a corner stream (if flagged). Band boundaries are derived from the
image: `n = clamp(width×height / 2^18, 1, 16)`, limited to the number of
macroblock rows, and band `i` covers macroblock rows
`[rows×i/n, rows×(i+1)/n)`. Every stream is independent and decoded with fresh
models, so streams can be processed in parallel; the stripe streams follow the
main bands and may use their reconstruction.

### Binary range coding

Each stream is coded with an adaptive binary range coder. The range starts at
`0xffffffff`; for a bit with model probability `p` of a one (16 bits, 1–65535),
`bound = (range >> 16) × p`. A one keeps `[0, bound)`, a zero the remainder.
While the range is below `2^24` it shifts by eight, with LZMA carry handling.
The encoder emits no initial cache byte and terminates with five shifts. A
model starts at `p = 32768` and observation count `n = 0`; after each bit it
moves toward 0 or 65535 by `(distance × r) >> 16`, where
`r = floor(131072 / (2n+3))` and `n` saturates at 80. Equiprobable bits halve
the range. A non-negative integer is coded as a class (0, 1, or `k` for
`[2^(k-1), 2^k)`) in unary with one model per position (at most 17), then the
bits below its leading one: the two highest with per-class models, the rest
equiprobable. A signed value is a zero flag, a sign and the magnitude minus one.

### Glyph channels

Levels are coded as indices `k` of `min(255, k×step)`; a file whose levels
are not of that form uses step 1. Blocks are visited in BRC1 order. For each
macroblock group: its size flag (context: the left and upper groups' flags,
or unavailable above the band), then each block: its type bit unless 1×1
(context: block kind and the type of the cells to the left and above), its
foreground index and, for a two-tone block, its contrast `fg - bg - 1` and its
pattern bits in raster order. Foregrounds are predicted from the reconstructed
pixels along the block's top and left edges (the mean, or the mean of the
brighter half for a two-tone block), falling back to a median edge predictor
over neighbouring block levels. Each pattern bit's context is its size class,
position parity and the left, upper, upper-left and upper-right states, where
a state is the decoded bit inside the block, whether the reconstructed pixel
outside it is nearer the foreground than the background, or unavailable.
Pattern blocks must contain both levels. The decoder rebuilds BRC1 streams
only where needed; the reconstructed planes equal BRC1 decoding.

### Implicit luma glyphs in BSM2

The gradient stream is decoded before the glyph channels. Each gradient leaf
covers whole luma macroblocks. These groups have **no size, type, level, contrast
or pattern symbols**, and do not update the corresponding entropy models.
Chroma groups remain explicit. Gradients require a macroblock side of 4 or 8.

For a leaf `(x, y, size)`, interpolate its shared mesh at
`(x + size/2, y + size/2)` and quantize that byte with the stored Y step:
`min(255, ((value + step/2) / step) * step)` using integer division.
Before processing any bands, fill its reconstructed baseline with this level;
mark all its macroblocks as large and all its cells as available solid blocks
with that foreground index and zero contrast. Thus even a gradient cell later
in traversal order is available to neighbouring contexts, subject to the usual
band boundary restrictions. BSM1 does not preseed these cells.

During each band traversal, skip gradient-covered groups in runs, setting the
band's last foreground index to the implicit level. Decoders that need BRC1
streams synthesize solid macroblocks in the original positions; other decoders
need only the seeded plane and partition. Ordinary glyphs and corner baselines
therefore retain the old reconstruction without redundant stored glyphs.

### Corners and gradients

Each corner channel is coded separately in node order. Eligible nodes follow
BGC2 (a neighbouring block side of at least the stored minimum; luma nodes
covered by gradients are excluded) and code a presence flag. The context is
derived from a median prediction from the neighbouring final corner values,
compared with the glyph baseline, and from the neighbours' flags. A stored
value is coded as the sign and the magnitude minus one of its BGC2 residual. Gradients code the 256×256 tile
quadtree in raster order (one leaf flag per node that fits, per level), then
every used 16-pixel vertex in raster order as a signed difference from a
median prediction over its decoded left and upper vertices.

## Quantization

At qualities 0–99 quantization happens before entropy coding, so the decoder needs no quality or
quantization setting. The encoder rounds each block's final levels to the nearest
multiple of the step, with half steps rounded upward and values clamped to 255:

At quality 0 the steps are 32 for luma and 16 for chroma. For qualities 1–99,
the encoder interpolates integer steps and block thresholds through an
empirical quality profile. Low-contrast 2×2 luma groups may be averaged and
low-contrast chroma blocks may be stored as solid colors; these effects diminish
at higher qualities. At quality 0, complete 2×2 luma groups are averaged and
8×8 macroblocks are used where they fit. If either image dimension is not a
multiple of four, 1×1 edge blocks preserve pixels that older encoders omitted.
These are encoder choices within the existing block format. Exact file size,
visual quality and their ordering between adjacent levels are not guaranteed.
If both rounded levels are equal, the block becomes solid; no pattern bits or
background entry are emitted. BW stores only Y. At qualities below 90 the decoded
image can differ from previous builds at the same numeric quality. Entropy coding
and PNG compression add no further loss. For blocks of 3×3 or larger, if the
rounded-down mean equals the minimum and classifying at that mean would make a
nonuniform block solid, the encoder retries the split for level spreads of at
least 8. Smaller spreads may still be represented as solid in lossy modes.

## Lossless quality 100

This mode uses the same BRC1 channel streams, block flags, masks, delta levels
and entropy compression as the lossy modes. Channel mask bit 3 selects an
invertible color transform: the three channels are `G`, `B-G+128` and
`R-G+128`, with arithmetic modulo 256. Decode uses `G`, `B=G+Cb-128` and
`R=G+Cr-128`, also modulo 256. For an already grayscale image, the single G
channel can be stored and replicated to RGB.

There is no level quantization. A 2×2 macroblock is encoded as one solid or
two-tone block only if its channel samples contain at most two distinct byte
values. Otherwise it is subdivided into four exact 1×1 blocks. Thus every
decoded 8-bit RGB sample equals the input RGB sample. The alpha channel is not
encoded: transparent images and 16-bit input images are rejected at q100. BW
mode rejects a color image at q100 because converting it to grayscale is lossy.
JPEG sources are compared after decoding to 8-bit RGB; original JPEG bytes,
metadata and precision beyond those RGB bytes are not preserved.
