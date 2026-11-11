# Babe — Bi-Level Adaptive Block Encoding

**Babe** is an experimental dual‑tone block-based image codec designed for extremely lightweight compression with visually smooth output.
The encoder uses byte-symbol range coding with local frequency tables, parallel processing and reusable buffers. Encoding is lossy; decoding restores the encoded image without further loss.

## Features

- Two‑tone block encoding with adaptive subdivision
- Quality-dependent quantization of YCbCr block levels
- Delta-coded block levels
- Byte-symbol range coding with parallel stream compression and decompression
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
   - Each stream is split into blocks of up to 32 KiB, with one frequency table per block. The range coder processes a whole byte per step. Constant blocks store a single repeated byte; incompressible blocks or streams are stored raw when that is smaller.
   - New files always use the `BRC1` container with a checksum. There is no search across codecs or context models, and Zstd is not used for encoding.
   - Up to four entropy workers process independent streams. Small payloads run serially; `Encoder.Parallel` and `Decoder.Parallel` also control these workers.
   - The decoder still accepts older binary-range and Zstd-based `.babe` files, including mixed range/Zstd `BRC1` files. Older programs do not support the new byte-range method (16).

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

In-memory BABE encoder/decoder timings and size tradeoffs are recorded in [the speed measurements](docs/performance.md).

## Benchmark comparison

Measured on an Apple M3 with Go 1.27.3 using the local `benchmark.jpg`
(5152×7728). Times are medians of three runs with two operations each, after
warm-up. The same decoded RGBA image is fed to both encoders; image loading and
file I/O are excluded. Encoder buffers and BABE decoder buffers are reused.
JPEG decoding returns Go's native YCbCr image, while BABE returns RGBA. Results
vary with the image and machine load; the source image is itself a JPEG.

JPEG and BABE at quality 80, with lossless QOI for reference:

| Metric | JPEG (q=80) | BABE (q=80) | QOI |
|---|---:|---:|---:|
| Encode time | 0.786 s | 0.278 s | 1.096 s |
| Decode time | 0.420 s | 0.296 s | 0.300 s |
| Total | 1.206 s | 0.574 s | 1.396 s |
| Output size | 13.98 MB | 22.69 MB | 78.98 MB |

Total is the sum of the separately measured encode and decode medians.

JPEG and BABE at each requested quality, on the same image and under the same
conditions:

| Quality | JPEG encode | BABE encode | JPEG decode | BABE decode | JPEG size | BABE size |
|---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.961 s | 0.321 s | 0.799 s | 0.455 s | 37.66 MB | 37.14 MB |
| 80 | 0.786 s | 0.278 s | 0.420 s | 0.296 s | 13.98 MB | 22.69 MB |
| 50 | 0.533 s | 0.354 s | 0.242 s | 0.336 s | 5.30 MB | 17.47 MB |
| 10 | 0.417 s | 0.249 s | 0.155 s | 0.135 s | 0.97 MB | 7.47 MB |
| 0 | 0.398 s | 0.280 s | 0.148 s | 0.217 s | 0.67 MB | 6.48 MB |

Go's JPEG encoder clamps quality 0 to **1**. Equal quality numbers do not mean
equal visual quality or equal file size; BABE quality 100 is still lossy. MB is
1,000,000 bytes. This is one image, not a claim about every image or JPEG codec.

To reproduce these measurements with a local `benchmark.jpg`:

```sh
go test -run '^$' -bench '^BenchmarkQualityComparison$' -benchtime=2x -count=3
```

See [the speed measurements](docs/performance.md) for the BABE encoder, decoder
and complete encode/decode cycle comparison against the earlier version.

## Visual comparison

The same [800×572 source image](docs/comparison/source.jpg) was encoded at
quality 80 by each codec. Click an image to inspect it at full resolution.

| JPEG q80 — 71,042 bytes | BABE q80 — 211,612 bytes |
|:---:|:---:|
| [![JPEG q80](docs/comparison/jpeg-q80.jpg)](docs/comparison/jpeg-q80.jpg) | [![BABE q80](docs/comparison/babe-q80.png)](docs/comparison/babe-q80.png) |

The JPEG image is the encoded JPEG itself. The BABE image is its decoded pixels
saved losslessly for display; the stated BABE size is the `.babe` bitstream, not
the preview. Against the decoded source, RGB PSNR is 35.40 dB for JPEG and
40.18 dB for BABE on this example. BABE uses about 2.98× as many bytes here;
the quality numbers and PSNR do not by themselves establish equal visual quality.
The source was already JPEG-compressed.

Recreate the images and size/PSNR figures with:

```sh
go run ./cmd/compareimages -input docs/comparison/source.jpg -out docs/comparison -quality 80
```

## License

MIT
