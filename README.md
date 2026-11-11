# Babe — Bi-Level Adaptive Block Encoding

**Babe** is an experimental dual‑tone block-based image codec designed for extremely lightweight compression with visually smooth output.
The encoder uses one adaptive range-coding pass per stream, with parallel processing and reusable buffers. Encoding is lossy; decoding restores the encoded image without further loss.

## Features

- Two‑tone block encoding with adaptive subdivision
- Quality-dependent quantization of YCbCr block levels
- Delta-coded block levels
- Adaptive range coding with parallel stream compression and decompression
- Lossy encoder, PNG output on decode
- Minimal API: `Encode(image, quality, bw)` and `Decode(data, postfilter)`

## How It Works

At a high level, Babe works as a lossy, block-based codec with a dual-tone model per block and independent Y, Cb and Cr streams.

1. **Color space and input**
   - The source image is converted to a YUV-like space.
   - Luma and chroma are processed with different sensitivity so that most detail is preserved in brightness while color is simplified more aggressively.

2. **Block partitioning**
   - The image is split into rectangular blocks (macroblocks).
   - Blocks can be further subdivided adaptively depending on local contrast and quality settings.
   - Very flat or low-contrast regions tend to keep larger blocks; detailed regions get smaller blocks.

3. **Dual-tone model**
   - For each block Babe tries to approximate all pixels using only **two representative colors** (a “dual-tone”).
   - A simple pattern (bit mask) inside the block tells, for each pixel, which of the two tones is used.
   - This creates a kind of ordered dither / posterization that looks smooth at a distance but is cheap to store.

4. **Block-level quantization**
   - Each block's foreground and background levels are rounded according to quality.
   - Chroma uses coarser steps than luma. If both levels become equal, the block is stored as solid, without a pattern or background value.
   - Quantization steps for Y / Cb,Cr: quality 0–19: 4 / 8; 20–59: 2 / 4; 60–89: 1 / 2; 90–100: 1 / 1.
   - This adds loss compared with older builds at the same quality below 90. Quality 100 still uses the codec's lossy block approximation.

5. **Delta coding**
   - Foreground and background values are stored as byte differences from the preceding value in their respective streams.

6. **Pattern and metadata encoding**
   - Each channel has block size flags, solid/pattern flags, pattern bits, foreground values and background values.
   - Solid blocks have no pattern or background entry.

7. **Bitstream layout**
   - Channels are independent. The header contains dimensions, block sizes and a channel mask; BW mode stores only luma.

8. **Range compression**
   - Each stream uses one adaptive order-0 range-coding pass. Incompressible streams are stored raw when that is smaller.
   - New files always use the `BRC1` container with a checksum. There is no search across codecs or context models, and Zstd is not used for encoding.
   - Up to four entropy workers process independent streams. Small payloads run serially; `Encoder.Parallel` and `Decoder.Parallel` also control these workers.
   - The decoder still accepts older Zstd-based `.babe` files and mixed range/Zstd `BRC1` files. Older Zstd-only programs cannot decode `BRC1` files.

9. **Decoding**
   - Decode reverses the process:
     - Decompress range-coded or Zstd streams and restore the block parameters.
     - Reconstruct each block’s two tones and pattern.
     - Stitch all blocks into a full‑resolution image in YUV space and convert back to RGB.
   - PNG export uses standard lossless compression for faster writing. Compared with maximum PNG compression, files can be slightly larger; decoded pixels are identical.

The format remains experimental. See [the BRC1 layout](docs/BRC1.md) for the container specification.

## CLI Utility

Build and run from the repository directory:

```sh
go build -o Babe .
./Babe 1.jpg 10 1.png
```

This writes `1.babe` and immediately decodes it into `1.png`. Range coding is enabled by default; no compression flag is needed. Use `./Babe 1.jpg 10 bw 1.png` for grayscale.

### Encode an image → `.babe`

```
./Babe input.jpg
```

Produces:

```
input.babe
```

Default quality is **70**. Lower quality prioritizes smaller files.

Specify a custom quality (0–100):

```
./Babe input.jpg 5
```

### Decode `.babe` → PNG

```
./Babe input.babe
```

Produces:

```
input.png
```

## API Usage

### Encode

```go
comp, err := Encode(img, quality, false)
if err != nil {
    // handle error
}
```

### Decode

```go
img, err := Decode(compData, false)
if err != nil {
    // handle error
}
```

`Decode` returns a standard `image.Image`.


## Status

This codec is currently experimental.
Format details, quality tuning, and performance optimizations are still evolving.

Current CLI timings and size tradeoffs are recorded in [the speed measurements](docs/performance.md).

## Benchmark comparison

Current single-run measurement on an Apple M3 using the repository's
`benchmark.jpg`. JPEG and BABE use quality 80; QOI is lossless. Timings cover
only in-memory encoding and decoding.

| Metric | JPEG (q=80) | BABE (q=80) | QOI |
|---|---:|---:|---:|
| Encode time | 8.638 s | 14.609 s | 16.948 s |
| Decode time | 3.770 s | 13.754 s | 4.193 s |
| Total | 12.409 s | 28.363 s | 21.141 s |
| Output size | 13.98 MB | 22.98 MB | 78.98 MB |

Run `go test -run '^TestBenchmarkSummary$' -v` to refresh the comparison on a
local machine. See [the speed measurements](docs/performance.md) for the
smaller-image CLI measurement and the range-coder optimization comparison.

## License

MIT
