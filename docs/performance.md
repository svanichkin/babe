> Historical experimental measurements below refer to CLI flags that have since been removed. The corresponding Go API options and benchmark helpers are retained for reproducibility; use `-smooth` for the current CLI.

# BABE encoder and decoder performance

Measured on Apple M3, darwin/arm64, Go 1.27.3, default GOMAXPROCS=8.
Only the in-memory BABE `Encode` and `Decode` APIs are timed. Images are loaded
before timing, encoder/decoder instances and buffers are reused, and both paths
are warmed up. `Cycle` times an actual encode followed immediately by a decode;
it is not the sum of two independently timed measurements. Postfilter is off.
The local samples are 800x572 (`1.jpg`) and 1920x1080 (`3.jpg`).

These historical before/after results compare commit `6978b1b` and v0.2.1,
before q100 lossless mode was introduced. Before: adaptive binary order-0 range coding. After: byte-symbol
range coding with static 32 KiB models, constant/raw block handling, exact
division-free quantization and 2x2 block classification, specialized structural
validation, and separate decode planes with parallel RGB conversion at quality
80–100. No quality thresholds or reconstructed pixel values were changed.
Decoded RGBA hashes match the baseline on both images at qualities 10, 70, 80
and 100. Unit tests, legacy-file fixtures, race detection, vet and separate fuzz
runs for the byte-range decoder and BRC1 container passed.

## Complete encode/decode cycle

For `3.jpg`, quality 80, five alternating pairs of old/new benchmark binaries,
10 iterations per measurement:

| Metric | Before | After |
|---|---:|---:|
| Cycle median | 94.49 ms | 28.75 ms |
| Cycle range across five measurements | 91.35–107.93 ms | 27.80–29.76 ms |
| BABE file size | 1,053,039 bytes | 1,056,745 bytes |

This is **3.29x faster** for the complete cycle, with **0.35% larger** output.
The 10x whole-cycle target has not been reached.

As a separate check on the larger 5152×7728 `benchmark.jpg` at quality 80,
the old `6978b1b` and current test binaries were run in alternating order.
Both used the same benchmark code, RGBA input, warmed buffers, Go version, and
`GOMAXPROCS=8`; each figure is the median of three single-operation runs:

| Codec / operation | Before | After |
|---|---:|---:|
| JPEG encode | 0.747 s | 0.754 s |
| JPEG decode | 0.428 s | 0.417 s |
| BABE encode | 1.027 s | 0.280 s |
| BABE decode | 1.125 s | 0.298 s |
| QOI encode | 1.035 s | 1.017 s |
| QOI decode | 0.277 s | 0.274 s |

These are separate operation measurements, not total test runtime or a measured
encode/decode cycle. On this image BABE encoding and decoding are about 3.7x and
3.8x faster respectively; the other codecs are effectively controls. Three runs
are a small sample, and the measured BABE decode ranged from 0.277 to 0.440 s
after the changes.

Broader checks below use medians of three runs, 10 iterations each, alternating
old/new binaries. Machine load varied considerably in this batch; the separate
five-pair cycle measurement above gives a tighter estimate for quality 80.

| Image / quality | Encode before → after, ms | Decode before → after, ms | Bytes before → after |
|---|---:|---:|---:|
| 1.jpg / 10 | 4.65 → 2.59 | 3.46 → 1.79 | 61,163 → 61,949 |
| 1.jpg / 70 | 9.63 → 4.12 | 9.27 → 3.69 | 228,502 → 234,523 |
| 1.jpg / 80 | 8.69 → 2.78 | 11.20 → 3.39 | 197,992 → 203,071 |
| 3.jpg / 10 | 22.34 → 11.91 | 17.75 → 8.93 | 306,244 → 305,557 |
| 3.jpg / 70 | 48.10 → 18.73 | 47.23 → 16.60 | 1,062,816 → 1,071,171 |
| 3.jpg / 80 | 54.68 → 13.65 | 58.04 → 13.69 | 1,053,039 → 1,056,745 |

Across these six cases, size changes range from -0.22% to +2.63%. Separate
planes add up to three bytes of reusable decoder storage per pixel for the
2x2 path, avoiding concurrent writes to shared RGBA cache lines.

After adding lossless q100, the q80 path on `3.jpg` was checked separately in
three alternating old/new runs of ten operations each. Median encode was
13.57 → 13.64 ms and decode 13.67 → 13.63 ms; the file size remained 1,056,745
bytes. These small differences are within run-to-run variation.

The new lossless q100 on the same `3.jpg` produced 1,901,324 bytes. Median
encode was 25.65 ms, decode 20.82 ms, and the measured encode/decode cycle
46.00 ms, each across three runs of ten warmed operations. The lossy q99 file
was 1,389,521 bytes; the lossless file is 36.8% larger in this example. Their
decoded pixels differ: q100 matches the 8-bit RGB input exactly. These are
measurements of the image codec only, not file I/O or source-image loading.

Quality 0 was subsequently retuned to prioritize smaller files. The same
decoded RGBA inputs produced these results; PSNR is relative to those inputs:

| Image | BABE q0 before | BABE q0 after | JPEG q1 | BABE PSNR before → after | JPEG PSNR |
|---|---:|---:|---:|---:|---:|
| `1.jpg` (800×572) | 60,738 B | 8,843 B | 9,256 B | 30.20 → 23.91 dB | 21.73 dB |
| `3.jpg` (1920×1080) | 300,102 B | 43,292 B | 41,913 B | 29.17 → 23.27 dB | 20.66 dB |

The lower quality is intentional. JPEG quality 0 is clamped to 1 by Go, and
its scale does not correspond to BABE's quality scale. These two images do not
guarantee a size or visual-quality result on other inputs. The new q0 files use
the existing BRC1 format and were decoded with the previous BABE decoder.
Checked qualities 1, 2, 10, 20, 50, 80, 99 and 100 produced byte-identical
files before and after the q0 change on `1.jpg`.

## Entropy coder alone

The same uncompressed streams extracted from `3.jpg` are processed serially by
both range implementations. Medians of three runs with `-benchtime=300ms`:

| Quality / operation | Binary range | Byte range | Speedup |
|---|---:|---:|---:|
| 10 / encode | 19.71 ms | 2.49 ms | 7.9x |
| 10 / decode | 23.06 ms | 3.84 ms | 6.0x |
| 80 / encode | 89.68 ms | 10.44 ms | 8.6x |
| 80 / decode | 95.93 ms | 15.77 ms | 6.1x |

These are entropy-only measurements, including model construction and parsing;
they do not represent the full image codec. Both paths allocate zero bytes per
iteration after warm-up. Faster entropy coding exposes the remaining cost of
block processing, validation and color conversion.

## Experimental corner reconstruction

Unless otherwise stated, the measurements in this section predate the rare
glyph fix described in [edge-aware decoder blur](#edge-aware-decoder-blur).

For an existing ordinary BRC1 file, `-corners` or `Decoder.CornerGradient`
enables the earlier decoder-only experiment: the lossy decoder estimates each
grid corner by averaging the reconstructed samples touching it, then derives
each pixel's Y/Cb/Cr from the four shared corners using bilinear interpolation
as it writes RGB. The grid uses the header's small-block size, or the macroblock
size when the small block is 1×1; it is a regular grid, not the precise adaptive
block partition. All three channels are interpolated, so small features and
edges may blur or change color. The experiment is disabled by default and does
not run for lossless q100. It does not change encoded bytes or the BRC1 format;
ordinary decoding remains available for existing files.

On Apple M3 with Go 1.27.3 and `GOMAXPROCS=8`, these are medians of three
complete encode/decode cycles after warm-up, using the same 5152×7728 JPEG
decoded to RGB. Each cycle starts fresh processes and includes file I/O and
PPM conversion. All six cases were shuffled in each round. CPU sums the stages;
RSS is the largest single process peak. The encoded bitstream was verified to
be byte-identical between modes. This is an initial speed and size comparison,
not a guarantee of improved perceived quality.

| Quality | Decoding | Decode | Full cycle | CPU / cycle | Peak RSS | Encoded size |
|---:|---|---:|---:|---:|---:|---:|
| 0 | Normal | 0.236 s | 0.501 s | 0.877 s | 297.4 MiB | 780,855 B |
| 0 | Corners | 0.458 s | 0.754 s | 1.486 s | 297.7 MiB | 780,855 B |
| 17 | Normal | 0.259 s | 0.613 s | 1.058 s | 299.7 MiB | 8,309,362 B |
| 17 | Corners | 0.433 s | 0.779 s | 1.566 s | 299.9 MiB | 8,309,362 B |
| 80 | Normal | 0.607 s | 1.105 s | 1.843 s | 744.7 MiB | 37,200,272 B |
| 80 | Corners | 0.950 s | 1.456 s | 2.557 s | 707.2 MiB | 37,200,272 B |

Individual cycles are in [the raw data](benchmarks/corners-apple-m3.json).
Reproduce with:

```sh
go build -o /tmp/babe-cornerbench ./cmd/losslessbench
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-cornerbench --rounds 3 --output /tmp/corners-results.json
```

The newer encoder experiment stores corner values sampled from the source
image in a BGC1 wrapper. It interpolates those values and retains the luma
residual of the BABE blocks, rather than deriving every corner from already
quantized blocks. `-corners` while encoding or `Encoder.CornerData` enables it.
See [the format description](BRC1.md#experimental-bgc1-corner-wrapper). Older
decoders cannot read BGC1; q100 remains ordinary lossless BRC1. It is opt-in:
source corners add bytes, and the approach can still blur or change colors.

On the 800×572 `samples/source.jpg`, with JPEG decoded directly as CLI input,
q10 was 47,940 B and 29.52 dB RGB PSNR normally, versus 60,637 B and 29.63 dB
with stored corners. At a comparable size ordinary q15 was 60,011 B and
30.43 dB. PSNR does not fully describe perceived block artifacts, but these
results do not demonstrate a quality advantage at equal size. The decoded
source JPEG was the reference.

On the 5152×7728 input, using the same complete-cycle CLI methodology as
above, three shuffled rounds gave these medians. Input was converted to RGB
PPM before the measurements, so sizes can differ slightly from direct JPEG
input. [Individual cycles](benchmarks/cornerdata-apple-m3.json) are retained.

| Quality | Encoding | Decode | Full cycle | CPU / cycle | Peak RSS | Encoded size |
|---:|---|---:|---:|---:|---:|---:|
| 10 | Normal | 0.326 s | 0.706 s | 1.133 s | 297.6 MiB | 6,751,534 B |
| 10 | Source corners | 0.624 s | 1.031 s | 2.195 s | 299.5 MiB | 7,745,569 B |
| 15 | Normal | 0.270 s | 0.646 s | 1.126 s | 299.6 MiB | 7,648,513 B |
| 15 | Source corners | 0.576 s | 0.939 s | 2.149 s | 301.8 MiB | 8,642,548 B |

```sh
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-cornerbench --rounds 3 --qualities 10,15 --encoded-corners --output /tmp/cornerdata-results.json
```

### Experimental glyph boundaries

For BGC1 images, `-glyphs` / `Decoder.GlyphBlend` interpolates small differences
between the reconstructed luma residuals on opposite sides of an actual
block boundary. It does so while restoring the RGB pixels from the source
corner gradient, using a snapshot of decoded luma so parallel workers see the
same neighbours. Differences above 24 luma levels are left intact. The luma
partition is reconstructed from its block-size flags, so a large macroblock is
not split artificially into smaller cells. `-s N` enables glyph blending with
minimum block side length N; the eligible side may use its smaller neighbour's
value, but the smaller block itself is not modified. The threshold is only a
heuristic and can smooth a real edge or miss a coding artifact. It adds
no bits and is off by default. The setting is not stored in BGC1 and q100 is
unaffected.

On `samples/source.jpg` at q10, both BGC1 variants were 60,637 bytes. RGB PSNR
against the decoded source was 29.63 dB without glyph blending and 29.61 dB
with it. This does not establish a visual improvement; inspect the images at
the same size.

The following historical measurements used the original regular-grid boundary
selection before the actual block partition and `-s` were added. On Apple M3,
Go 1.27.3 and GOMAXPROCS=8, three complete CLI cycles after
warm-up used the same RGB input and byte-identical BGC1 files. Process startup,
PPM conversion and disk I/O are included. CPU sums both stages, and RSS is the
largest individual process peak. Individual results are in
[the raw data](benchmarks/glyphs-apple-m3.json).

| Quality | Decode mode | Decode | Full cycle | CPU / cycle | Peak RSS | Encoded size |
|---:|---|---:|---:|---:|---:|---:|
| 0 | Corners | 0.575 s | 0.874 s | 2.099 s | 299.5 MiB | 1,774,890 B |
| 0 | Corners + glyphs | 0.782 s | 1.096 s | 2.825 s | 299.2 MiB | 1,774,890 B |
| 10 | Corners | 0.734 s | 1.162 s | 2.258 s | 299.7 MiB | 7,745,569 B |
| 10 | Corners + glyphs | 1.000 s | 1.496 s | 3.097 s | 299.5 MiB | 7,745,569 B |

```sh
go build -o /tmp/babe-glyphbench ./cmd/losslessbench
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-glyphbench --rounds 3 --qualities 0,10 --glyph-blend --output /tmp/glyph-results.json
```

After switching to actual block boundaries, three full shuffled cycles on the
same machine and image gave these medians. Both modes used identical BGC1
bitstreams; times include process startup, PPM I/O and encoding. At q80 the
largest block is 2×2, so a minimum of 4 changes no pixels; the timing variation
in that row reflects independent measurements rather than a processing effect.

| Quality | Decode mode | Decode | Full cycle | CPU / cycle | Peak RSS | Encoded size |
|---:|---|---:|---:|---:|---:|---:|
| 10 | Corners | 0.690 s | 1.127 s | 2.461 s | 299.5 MiB | 7,745,569 B |
| 10 | `-s 4` | 1.093 s | 1.507 s | 3.517 s | 299.6 MiB | 7,745,569 B |
| 80 | Corners | 0.955 s | 1.506 s | 3.113 s | 680.8 MiB | 38,194,307 B |
| 80 | `-s 4` | 0.985 s | 1.592 s | 3.301 s | 746.4 MiB | 38,194,307 B |

Per-cycle data: [glyph minimum 4](benchmarks/glyph-min4-apple-m3.json).

```sh
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-glyphbench --rounds 3 --qualities 10,80 --glyph-blend --glyph-min-size 4 --output /tmp/glyph-min4-results.json
```

## Selective corners and pattern experiments

The measurements in this section were recorded before the rare-glyph fix;
encoded sizes and decoded pixels may differ slightly with the current encoder.

The experimental BGC2 encoder stores a shared grid of corner positions but
selects Y, Cb and Cr independently. For each channel it examines that channel's
actual block partition, storing a corner only if a neighbouring block meets the
minimum size and the source differs from the decoded base by more than three
levels. Missing channels use the decoded base estimate. `-sparse` defaults to a
minimum of 4; when `-s N` is used for encoding, it also selects BGC2 with that
minimum. See [BGC2](BRC1.md#experimental-bgc2-selective-corners) for the
representation. The current implementation reconstructs the base channels to
make that choice, so encoding can be slower. BGC1 remains readable; older
decoders do not support BGC2.

`-dither` mildly modulates the two-tone mask threshold with a deterministic
ordered 4×4 pattern on eligible lossy blocks of size 4×4 or larger.
`-pattern2` replaces certain 4×4 or 8×8 masks by 2×2-majority samples expanded
back to the original size, but only if no group ties and at most 1/16 of bits
change. The full mask is still stored in the existing stream; range coding may
or may not compress the repetition better. Both are lossy encoder experiments,
disabled by default; they do not change the BRC1 mask format or q100.

On the 800×572 `samples/source.jpg`, using direct JPEG input to the CLI, these
are sizes and RGB PSNR against the decoded JPEG. The comparable-size rows make
the tradeoff visible; PSNR does not measure all perceptual artifacts.

| Mode | Quality | Encoded size | RGB PSNR |
|---|---:|---:|---:|
| Dense BGC1 corners | 10 | 60,637 B | 29.63 dB |
| Selective BGC2 corners | 10 | 54,374 B | 29.48 dB |
| Selective + dither | 10 | 58,834 B | 29.03 dB |
| Selective + pattern2 | 10 | 53,748 B | 29.30 dB |
| Selective + both | 10 | 58,401 B | 28.94 dB |
| Selective BGC2 corners | 12 | 60,389 B | 29.57 dB |
| Selective + pattern2 | 12 | 59,422 B | 29.39 dB |
| Ordinary BABE, no corners | 15 | 60,011 B | 30.43 dB |

On the local 5152×7728 image, three shuffled full encode/decode CLI cycles per
case after warm-up produced the following medians. Each process reads/writes
PPM and compressed files; CPU sums both stages, peak RSS is the largest single
process high-water mark. Raw records, including all three runs, are in
[experiments-apple-m3.json](benchmarks/experiments-apple-m3.json). The cycle
spread was substantial (for example, dense ranged 1.022–1.536 s), so small
timing differences should not be interpreted as conclusive speedups. This
benchmark uses RGB PPM input and is not directly comparable to direct JPEG
input sizes in the previous table.

| Mode, quality 10 | Decode | Full cycle | CPU / cycle | Peak RSS | Size |
|---|---:|---:|---:|---:|---:|
| Dense BGC1 | 0.729 s | 1.238 s | 2.353 s | 299.8 MiB | 7,745,569 B |
| Selective BGC2 | 0.655 s | 1.707 s | 2.604 s | 299.3 MiB | 7,155,522 B |
| Selective + dither | 0.656 s | 1.545 s | 2.691 s | 299.7 MiB | 7,098,738 B |
| Selective + pattern2 | 0.703 s | 1.442 s | 2.655 s | 299.6 MiB | 7,155,129 B |
| Selective + both | 0.737 s | 1.653 s | 2.833 s | 299.6 MiB | 7,098,311 B |

```sh
go build -o /tmp/babe-experiments ./cmd/losslessbench
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-experiments --rounds 3 --qualities 10 --experiments --output /tmp/experiments.json
```

## Decoder-only dithering

The numerical measurements in this section were recorded before the
rare-glyph fix; current lossy bitstreams may differ slightly.

`-dither2` / `Decoder.Dither2` leaves the encoded data unchanged. During lossy
decoding it finds patterned luma blocks of at least 4×4, estimates local
coverage from the current pixel and its in-block neighbours, and uses a 4×4
ordered threshold to choose one of the existing local luma levels. It skips
local differences above 48 levels to reduce the effect on high-contrast edges.
The current pixel's weight decreases from 8 to 4 between 4×4 and 8×8 blocks,
increasing the influence of neighbours and hence the dithering in larger blocks.
The ordered threshold itself keeps the same 4×4 period; it does not enlarge
the size of the individual dots.
This does not reconstruct lost source detail; it trades some hard transitions
for a spatial pattern and may introduce noise or worsen numerical fidelity.
For BGC2 the source-corner baseline is recovered before dithering. The option
also works with BRC1 and BGC1, and can be combined with `-glyphs-only`. It has
no effect on lossless q100 or the 1×1/2×2 layout.

On the 800×572 `samples/source.jpg`, CLI q10 `-s 4` produced the same 54,374 B
file with and without `-dither2` (verified by SHA-256). RGB PSNR against the
decoded JPEG was 29.46 dB without decoder dithering, 28.91 dB with the former
fixed-strength version and 28.85 dB with size-dependent strength. This
does not establish a perceptual preference; compare the images at equal size.

The following historical medians use the fixed-strength version, measured in
three shuffled complete CLI cycles after warm-up on
the 5152×7728 RGB input, including process startup and file I/O. All comparisons
use byte-identical sparse files, without `-s` glyph-edge blending. CPU sums both
stages; peak RSS is the highest single-process peak. At q80 there are no blocks
large enough to dither; variations reflect independent timing noise.

| Quality | Decoder | Decode | Full cycle | CPU / cycle | Peak RSS | Encoded size |
|---:|---|---:|---:|---:|---:|---:|
| 10 | Normal | 0.692 s | 1.367 s | 2.427 s | 299.6 MiB | 7,155,522 B |
| 10 | `-dither2` | 0.819 s | 1.482 s | 2.915 s | 299.6 MiB | 7,155,522 B |
| 80 | Normal | 0.841 s | 1.710 s | 3.159 s | 718.3 MiB | 37,200,451 B |
| 80 | `-dither2` | 0.870 s | 1.926 s | 3.207 s | 800.5 MiB | 37,200,451 B |

Individual cycles: [dither2-apple-m3.json](benchmarks/dither2-apple-m3.json).

```sh
go build -o /tmp/babe-dither2bench ./cmd/losslessbench
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-dither2bench --rounds 3 --qualities 10,80 --decode-dither --output /tmp/dither2-results.json
```

For size-dependent strength, the old and new decoder were also tested on the
same sparse bitstreams in two sets of three complete cycles. These measurements
are **not reliable for a speed comparison**: at q10 the first set ranged from
3.348–8.894 s for the old version and 2.375–5.592 s for the new version; the
repeat ranged from 6.606–31.864 s and 5.130–41.897 s respectively. System load
averages sampled just after the sets were 372.56 and 390.49 over one minute.
All individual results are retained in [first set](benchmarks/dither2-adaptive-apple-m3.json)
and [repeat](benchmarks/dither2-adaptive-repeat-apple-m3.json). Rerun on a less
loaded machine before drawing a performance conclusion. The previous helper was
built from commit `50b2904` before this change.

```sh
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-dither2-new-helper --previous-helper /tmp/babe-dither2-prev-helper --rounds 3 --qualities 0,10 --decode-dither --output /tmp/dither2-adaptive.json
```

## Edge-aware decoder blur

`-dither3` / `Decoder.Dither3` is a separate experimental lossy decoder mode.
It uses the decoded luma block partition and affects only the interior of actual
8×8 and larger blocks; block-edge pixels and smaller blocks are untouched.
The correction fades in over the first two interior pixels, preventing a sharp
transition from the fixed edge to the blurred center.
Horizontal and vertical passes each use an immutable snapshot. Each scan stays
inside its block and considers up to half the block size in each direction,
capped at eight pixels. The center gets weight 49; a neighbour with absolute
luma difference `d <= 48` gets weight `49-d`, while a larger difference stops
that scan. The rounded weighted mean is used for reconstruction. Thus strong
steps are not blended across, but a real edge within the threshold can still blur.
The separate `-s` boundary blending is unchanged. Chroma is left intact. Source-corner
baselines are computed before the blur, q100 is untouched, and encoded data
does not change. When combined, `-dither2` precedes `-dither3`.

On the top-left 400×250 region of `samples/source.jpg` at q10 with
`-s 4 -pattern2`, the former `-dither3` introduced a visible frame: mean absolute
RGB-derived luma differences at the two pixel transitions just inside the 8×8
grid were 0.829 and 0.907 levels, versus 0.395 and 0.429 without the blur.
With the fade they were 0.397 and 0.426. These are measurements on one region,
not a general quality metric. The old and new decoders used the same encoded
file; a test also checks that `-s` boundary pixels remain unchanged.

Three complete CLI cycles per version on the same 800×572 RGB PPM and the
same q10 `-pattern2` BGC2 bitstream gave 0.0589, 0.0614 and 0.0573 s before
the fade and 0.0620, 0.0651 and 0.0603 s after it. These timings include process
startup, encoding and file I/O; glyph boundary blending was not enabled for the
timing comparison. The one-minute load average after the run was 43.36, so the
small differences should not be treated as a reliable speed result. Individual
CPU and memory values are in [dither3-frame-fix-apple-m3.json](benchmarks/dither3-frame-fix-apple-m3.json).

During the investigation, a separate encoder defect was found: if the rounded
mean of a two-level block equalled its background, a rare brighter feature
could be lost. Lossy blocks now retain such features when the level difference
is at least 8; smaller differences can still collapse under the lossy policy.
An 8×8 block containing 63 samples of 96 and one of 128 formerly decoded as
all 96 and now preserves both levels. On `samples/source.jpg` at q10 the case
affected two luma blocks. The pattern bit order and lossless q100 were also
checked. The fix changes some lossy bitstreams but does not change the format.

Before the rare-glyph and edge-fade fixes, direct CLI input from 800×572
`samples/source.jpg`, q10 `-s 4` produced
the same 54,374-byte `.babe` file before and after this blur change (SHA-256
checked). RGB PSNR against the decoded JPEG was 29.46 dB for the former weak
blur and 29.30 dB for the larger-block interior blur. This does not guarantee
equal perceived quality, nor preservation of every true edge.

The following historical timings refer to the earlier four-neighbour blur.
Three full cycles on the 5152×7728 image used byte-identical BGC2 files with
and without `-dither3`. Their elapsed times were 6.469, 4.009 and 6.008 s
without blur, and 5.981, 4.370 and 8.027 s with blur. The one-minute system
load average immediately after the run was 330.85, and the large spread makes
these measurements unsuitable for a speed comparison. All individual CPU,
RSS and timing values are retained in [dither3-apple-m3.json](benchmarks/dither3-apple-m3.json).

The old and current blur were also compared in three complete cycles on the
same BGC2 bitstream. Old cycle times were 19.459, 7.555 and 31.274 s; current
times were 7.775, 7.989 and 27.421 s. The one-minute system load average after
the run was 676.32, so this run cannot establish a speed difference. The full
records are in [dither3-interior-apple-m3.json](benchmarks/dither3-interior-apple-m3.json).
The previous helper was built from commit `e69338f`.

```sh
go build -o /tmp/babe-dither3bench ./cmd/losslessbench
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-dither3bench --rounds 3 --qualities 10 --decode-blur --output /tmp/dither3-results.json
python3 scripts/compare-corners.py --input benchmark.jpg --helper /tmp/babe-dither3-finalbench --previous-helper /tmp/babe-dither3bench --rounds 3 --qualities 10 --decode-blur --output /tmp/dither3-interior.json
```

## Lossless CLI comparison

The [README table](../README.md#lossless-complete-encodedecode-cycle) uses the
same 5152×7728 local `benchmark.jpg`, decoded once into an opaque 8-bit RGB PPM.
BABE q100, QOI, JPEG XL at efforts 1 and 5, PNG + OxiPNG and WebP each encode this same PPM and
then decode their output to RGB PPM. A SHA-256 comparison of dimensions and
pixel bytes verifies every warm-up and measured cycle. JPEG source metadata and
the original JPEG bitstream are not part of the comparison.

There is one warm-up per format, followed by three complete measured rounds,
each containing all six cases in a shuffled order. Each cycle starts new
processes and includes reading and writing the files, process startup and RGB
conversion. PNG encoding includes both the initial Go PNG encoding and the
subsequent OxiPNG optimization. Full-cycle wall time is measured directly.
CPU time is the sum of user and system time for all stages and threads; peak
RSS is the highest individual process high-water mark among those sequential
stages. The table gives medians across the three cycles, not a sum of medians.
Unlike the earlier in-memory measurements, buffers are not reused across
processes, and disk I/O is included.

Apple M3, macOS, Go 1.27.3, GOMAXPROCS=8; libjxl 0.12.0 (`-d 0 -e 1` or `-d 0 -e 5`,
8 threads), libwebp 1.6.0 (`-lossless -q 75 -m 4 -mt`), OxiPNG 10.2.1
(`-o2 -t 8 --strip safe`). Encoder settings are not equivalent optimization
levels. These are results for one image and machine; three runs reduce reliance
on one timing but do not eliminate effects of system load.

| Format | Full-cycle wall time, three rounds | CPU time, three rounds |
|---|---|---|
| BABE | 1.189, 1.167, 1.177 s | 1.947, 1.978, 2.001 s |
| QOI | 1.708, 2.184, 1.787 s | 1.652, 1.774, 1.711 s |
| JPEG XL, effort 1 | 0.345, 0.422, 0.344 s | 1.260, 1.348, 1.320 s |
| JPEG XL, effort 5 | 5.051, 4.214, 5.059 s | 27.192, 26.475, 27.539 s |
| PNG + OxiPNG | 13.272, 14.129, 13.526 s | 17.315, 17.980, 17.310 s |
| WebP | 15.478, 16.192, 15.967 s | 15.369, 15.917, 15.473 s |

The [individual results and exact settings](benchmarks/lossless-e1-apple-m3.json)
include wall time, CPU, process peak RSS and encoded size for all 18 cycles.
To reproduce on macOS or Linux with `cjxl`, `djxl`, `cwebp`, `dwebp` and
`oxipng` installed:

```sh
go build -o /tmp/babe-losslessbench ./cmd/losslessbench
python3 scripts/compare-lossless.py --input benchmark.jpg --helper /tmp/babe-losslessbench --rounds 3 --threads 8 --output /tmp/lossless-results.json
```

## JPEG at comparable sizes

The [README comparison](../README.md#jpeg-and-babe-at-comparable-sizes) uses
the decoded opaque RGB pixels of `samples/source.jpg` (800×572). JPEG q1 and
BABE q0, and JPEG q80 and BABE q17, were selected for close encoded sizes on
this image. The lossless PNG previews show the BABE decoder output and do not
represent BABE file sizes. RGB PSNR was measured against the decoded source;
it is not a complete perceptual quality metric.

After one warm-up per case, three shuffled rounds each ran a complete encode
and decode for every setting. The measurements use separate processes and PPM
input/output, including startup and disk I/O. Unlike the lossless test, lossy
pixels cannot be required to match the source; the benchmark checks the output
dimensions and PPM structure, while `compareimages` calculates PSNR separately.
CPU sums user and system times of both stages; RSS is the highest single
process peak. This small image makes startup and file overhead significant.
The [raw JSON](benchmarks/jpeg-glyph-fix-apple-m3.json) contains all 12
cycles, and results use their medians. Apple M3, Go 1.27.3, GOMAXPROCS=8.
The machine was heavily loaded: its one-minute load average immediately after
the run was 289.86, and BABE q0 full-cycle times ranged from 0.0183 to 0.0537 s.
Treat these numbers as a rough snapshot rather than a reliable speed ranking.

```sh
go build -o /tmp/babe-losslessbench ./cmd/losslessbench
python3 scripts/compare-jpeg.py --input samples/source.jpg --helper /tmp/babe-losslessbench --rounds 3 --output /tmp/jpeg-results.json
```

## CPU and memory

The historical in-memory measurements cover CPU time, average core use, Go
heap allocations per operation, and peak process RSS for each quality and
codec. The input is the same local 5152×7728 `benchmark.jpg` used in the
time comparison. Measurements were made on Apple M3, darwin/arm64, Go 1.27.3,
`GOMAXPROCS=8`, using three separate processes per case with two timed, warmed
operations in each. Execution order was shuffled. The tables use medians of
the three process results. The [original individual measurements](benchmarks/resources-apple-m3.json)
are retained alongside the updated BABE
[quality-curve measurements](benchmarks/resources-qualitycurve-apple-m3.json),
collected in separate sessions from JPEG and QOI.

CPU time is the change in process user plus system CPU time, including all Go
threads and any GC work during the interval. Cores is CPU time divided by wall
time for that same process, with 1.0 equal to one fully occupied core. Ratios
over 1.0 indicate parallel CPU work. It is an interval average, not peak
utilization or a percentage of the entire machine's capacity.

Allocated bytes are the change in Go `runtime.MemStats.TotalAlloc` during the
timed operations, divided by their count. They exclude already retained buffers
and are not peak or total memory usage. RSS is `getrusage(RUSAGE_SELF).Maxrss`,
the process high-water mark over startup, input loading, warm-up and timed
operations; it is not an incremental, codec-only memory measurement. Encoding
loads the same RGBA source for each codec. Decoding reads a pre-encoded stream
in a fresh process, so an encoder's buffers cannot inflate decoder RSS. JPEG
returns YCbCr, whereas BABE and QOI return RGBA. Warmed BABE instances retain
their scratch and output buffers, while JPEG and QOI allocate decoded images.
The Go runtime, garbage collector, input type and OS affect both allocation and
RSS results. Peaks are medians of three process high-water marks.

On macOS or Linux, reproduce with a local source image:

```sh
go build -o /tmp/babe-measurecodec ./cmd/measurecodec
GOMAXPROCS=8 python3 scripts/measure-codecs.py --binary /tmp/babe-measurecodec --input benchmark.jpg --runs 3 --iterations 2 --output /tmp/babe-resource-results.json
```

The script runs each case separately, stores encoded decoder inputs in a
temporary directory, shuffles execution order and prints both tables. A JPEG
quality of 0 is clamped to 1 by Go's encoder. Sample images are local and may
be replaced, but results for different images are not directly comparable.

## Quality scale

The quality curve is an empirical encoder profile, not a per-image size target.
It interpolates quantization, chroma simplification, luma smoothing and block
thresholds. Fractional block thresholds are distributed deterministically by
block position to avoid a whole image changing at one integer threshold. It
does not repeatedly encode the image to seek a target size. The block and
range-coded BRC1 format is unchanged, so the previous decoder can read files
from the new scale. Opaque 8-bit RGB at quality 100 remains lossless.

The following historical calibration was measured before the rare-glyph fix.
Four local input images were decoded to RGBA before measuring all 101 quality
levels. Each image's straight-line reference runs from its actual q0 size to its
actual q100 size. The mean and maximum errors below are absolute deviations
from that reference divided by the q100–q0 size range, expressed in percentage
points. They measure file-size progression, not visual quality. The included
`samples/source.jpg` is the same file as `1.jpg`.

| Image | Mean deviation before → after | Max deviation before → after | Largest adjacent size decrease before → after |
|---|---:|---:|---:|
| `1.jpg` | 10.70 → 3.72 pp | 30.44 → 9.66 pp | 34.10% → 0.27% |
| `2.jpg` | 16.74 → 4.66 pp | 34.73 → 15.66 pp | 19.49% → none |
| `3.jpg` | 12.04 → 2.34 pp | 28.98 → 7.38 pp | 26.29% → 0.22% |
| `4.jpg` | 12.88 → 3.38 pp | 32.61 → 13.62 pp | 31.26% → 0.58% |

The q0 size of `2.jpg` changed from 13,019 to 20,419 bytes: its 990×660
dimensions are not multiples of four, and the new encoder retains edge pixels
that the previous block layout omitted. `1.jpg`, `3.jpg` and `4.jpg` have
dimensions divisible by four; their q0 files and all four q100 files matched the
previous size and decoded output. Endpoints were also verified byte-for-byte
for `1.jpg` q0 and both `1.jpg` and `2.jpg` q100.

On these four images the largest decrease in adjacent lossy RGB PSNR was
0.27 dB; the earlier scale had decreases of up to 11.77 dB, including an
edge-cropping artifact. This is not proof of monotonic visual quality: PSNR is
not perception, quantization is discrete, and different input content changes
both compression and distortion. In a synthetic gradient test, lossless q100
was 14,018 bytes while q99 was 35,159 bytes. Thus neither monotonic size nor a
linear size/quality curve is guaranteed, especially at the lossless endpoint.

Raw results: [previous scale](benchmarks/quality-curve-previous.csv) and
[calibrated scale before the glyph fix](benchmarks/quality-curve-current.csv).
The [source-image curve after the fix](benchmarks/quality-curve-glyph-fix-source.csv)
is recorded separately; the other three photos are local examples.

```sh
go run ./cmd/qualitycurve -input samples/source.jpg > /tmp/babe-quality.csv
```

## Large shared gradients: 128 and 256 pixels

The following comparison uses `benchmark.jpg` at 5152×7728, decoded from JPEG
for each run. Each of three shuffled rounds ran a complete BABE `-smooth`
encode and decode for each quality and version, in a fresh process. The prior
version is commit `416eec6`; the new version adds 128×128 and 256×256 blocks,
tile-sum acceleration and compact BGG3 coordinates. Times exclude loading
the JPEG and writing a PNG. Entries are medians; the complete-cycle time is
the median of the three encode-plus-decode sums, not the sum of the medians.
The system load increased during the run, so these are approximate timings.

| Quality | Version | Encode | Decode | Encode + decode | Peak RSS | BABE bytes | RGB PSNR |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0 | Before | 0.995 s | 3.827 s | 4.822 s | 619.4 MiB | 1,502,145 | 24.003 dB |
| 0 | After | 0.528 s | 3.875 s | 4.403 s | 619.7 MiB | 1,501,923 | 24.003 dB |
| 10 | Before | 1.179 s | 2.513 s | 3.697 s | 623.9 MiB | 6,923,656 | 28.993 dB |
| 10 | After | 0.675 s | 2.550 s | 3.226 s | 624.2 MiB | 6,923,569 | 28.993 dB |
| 50 | Before | 1.352 s | 0.426 s | 1.783 s | 746.8 MiB | 20,699,063 | 34.422 dB |
| 50 | After | 0.861 s | 0.428 s | 1.300 s | 749.0 MiB | 20,699,039 | 34.422 dB |

This photograph contains few areas suitable for the larger blocks. On a
synthetic 512×512 smooth grayscale gradient at q10, the file went from 1,011
to 506 bytes and PSNR from 37.15 to 37.43 dB; this is a single example, not a
general compression claim. The original 800×572 `samples/source.jpg` used
three 128×128 blocks at q0 and q10, two at q50, and no 256×256 blocks.

## Smooth encode/decode optimization

Three complete shuffled rounds compared commit `5434321` with the optimized
implementation on `samples/source.jpg` (800×572). Each run encoded and decoded
in a fresh process. Times below are medians in milliseconds, excluding JPEG
input and PNG output; cycle is the median of each run's encode-plus-decode
time. "Plain" is the codec without `-smooth` and is shown for context.

| Q | Mode | Encode | Decode | Complete cycle | BABE bytes |
|---:|---|---:|---:|---:|---:|
| 0 | Plain | 1.989 | 1.450 | 3.440 | 8,837 |
| 0 | Smooth before | 10.622 | 30.205 | 40.826 | 16,763 |
| 0 | Smooth after | 9.500 | 17.296 | 26.795 | 16,763 |
| 10 | Plain | 3.094 | 1.957 | 5.191 | 47,943 |
| 10 | Smooth before | 10.814 | 25.050 | 35.690 | 47,524 |
| 10 | Smooth after | 9.678 | 15.067 | 24.841 | 47,524 |
| 50 | Plain | 3.724 | 2.965 | 6.689 | 183,862 |
| 50 | Smooth before | 12.440 | 6.559 | 18.975 | 177,998 |
| 50 | Smooth after | 9.976 | 6.544 | 16.488 | 177,998 |

The before and after BABE bitstreams and RGB pixels were compared byte for byte
at all three qualities. A separate three-round fresh-process measurement on
the same photo showed q10 median peak RSS rising from 13.8 to 15.5 MiB as the
corner channels are processed in parallel; images over 2,097,152 pixels keep
the serial corner preparation to avoid multiplying large plane buffers.

The same three-round comparison on `benchmark.jpg` (5152×7728) showed q10
median encode 0.688→0.678 s, decode 2.510→1.156 s, complete cycle
3.198→1.824 s, and peak RSS 623.7→623.9 MiB. The `.babe` file was 6,923,569
bytes in both versions and PSNR was 28.993 dB. These are approximate results
from one machine; one q0 baseline decode was a large timing outlier.

## Gradient and glyph reuse with contrast coding

This comparison starts at commit `566eeff`, using Go 1.27.3 on darwin/arm64.
The changes reuse shared-gradient boundary samples, cache each 8×8 glyph's
seam corrections, and run both interior-blur passes inside each glyph using
two 64-byte buffers instead of a full-image snapshot. Background colors can
use reversible FG−BG prediction (BRC1 method 17). A model-size estimate chooses
the predictor before range coding, avoiding two complete compression attempts.
The 1×1/2×2 path, including q100, keeps the previous predictor.

Each of three shuffled rounds starts a fresh helper process per case. For the
800×572 `samples/source.jpg` (identical to the local `1.jpg`), each process runs
10 complete in-memory encode/decode cycles on reused codec instances, including
the first cycle, with no separate warm-up. The 5152×7728 photo runs one complete
cycle per process. All entries are medians of the three process results; cycle
is measured independently as encode plus decode in each process, so its median
need not equal the sum of the two stage medians. JPEG loading, output hashing
and PNG export are outside codec timing. CPU sums all process threads during
the cycles. Peak RSS is process-wide, including input loading and retained
buffers; it is not allocated bytes per operation. Do not compare the 10-cycle
RSS directly with the earlier single-cycle tables.

Small photo, `-smooth`:

| Q | Version | Encode ms | Decode ms | Cycle ms | CPU ms/cycle | Peak RSS MiB | BABE bytes |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0 | Before | 9.674 | 18.274 | 27.948 | 47.174 | 25.2 | 16,763 |
| 0 | After | 7.149 | 11.231 | 18.380 | 35.396 | 23.5 | 16,563 |
| 10 | Before | 9.746 | 15.565 | 25.311 | 40.680 | 25.4 | 47,524 |
| 10 | After | 7.049 | 10.107 | 17.157 | 30.402 | 24.7 | 47,002 |
| 50 | Before | 10.381 | 6.665 | 16.984 | 27.985 | 23.6 | 177,998 |
| 50 | After | 9.237 | 5.539 | 14.775 | 25.835 | 23.5 | 168,673 |
| 100 | Before | 5.717 | 4.150 | 9.867 | 18.971 | 19.7 | 374,884 |
| 100 | After | 5.913 | 4.361 | 10.291 | 19.525 | 20.1 | 374,884 |

40-megapixel photo, `-smooth`:

| Q | Version | Encode ms | Decode ms | Cycle ms | CPU ms/cycle | Peak RSS MiB | BABE bytes |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0 | Before | 498.881 | 1336.723 | 1835.604 | 4262.850 | 693.7 | 1,501,923 |
| 0 | After | 493.433 | 850.715 | 1351.060 | 2849.058 | 693.5 | 1,481,821 |
| 10 | Before | 673.381 | 1190.096 | 1859.168 | 3414.449 | 731.6 | 6,923,569 |
| 10 | After | 675.844 | 760.152 | 1435.995 | 2432.736 | 693.5 | 6,748,671 |
| 50 | Before | 856.595 | 433.926 | 1290.522 | 1960.084 | 745.8 | 20,699,039 |
| 50 | After | 867.733 | 454.789 | 1361.059 | 2017.957 | 783.0 | 19,853,337 |

The decoded RGBA SHA-256 matches between versions in every recorded case.
q100 bitstreams also remain identical. These changes preserve image quality;
file-size reductions come from the reversible predictor. Timings vary with
machine load. In particular, the larger q50 case trades a 4.1% smaller file
for a 5.5% longer cycle and a higher measured peak RSS. This is not a speedup
at every quality or a universal memory reduction.

For context, the plain codec on the small photo (separate three-round run):

| Q | Version | Encode ms | Decode ms | Cycle ms | CPU ms/cycle | Peak RSS MiB | BABE bytes |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0 | Before | 1.922 | 1.234 | 3.155 | 6.382 | 11.9 | 8,837 |
| 0 | After | 1.846 | 1.176 | 3.028 | 6.368 | 12.1 | 8,621 |
| 10 | Before | 2.620 | 1.748 | 4.368 | 8.256 | 12.6 | 47,943 |
| 10 | After | 2.627 | 1.797 | 4.424 | 8.479 | 12.8 | 47,473 |
| 50 | Before | 3.343 | 2.877 | 6.232 | 14.313 | 14.2 | 183,862 |
| 50 | After | 3.557 | 3.061 | 6.605 | 14.836 | 15.1 | 174,307 |
| 100 | Before | 5.696 | 4.179 | 9.875 | 19.134 | 19.6 | 374,884 |
| 100 | After | 5.920 | 4.402 | 10.322 | 19.643 | 19.8 | 374,884 |

The plain q50 cycle increased about 6% while the file shrank 5.2%; most speed
improvements above concern gradient reconstruction and smoothing. q100 does
not execute the new prediction or smoothing paths, so its small timing
variations should not be presented as an algorithmic improvement.

Raw rounds: [small smooth](benchmarks/smooth-contrast-source.json),
[large smooth](benchmarks/smooth-contrast-large.json), and
[small plain](benchmarks/smooth-contrast-plain.json).
To reproduce, build `cmd/smoothbench/main.go` against both revisions (use the
same helper source for the older revision), then run:

```sh
python3 scripts/compare-smooth-speed.py --before /tmp/babe-before --after /tmp/babe-after --input samples/source.jpg --iterations 10 --output /tmp/smooth-small.json
python3 scripts/compare-smooth-speed.py --before /tmp/babe-before --after /tmp/babe-after --input benchmark.jpg --qualities 0,10,50 --iterations 1 --output /tmp/smooth-large.json
```

Add `--plain` for the codec without smoothing. The script enforces at least
three rounds and fails if decoded pixel hashes differ.

## Reproduction

Sample JPEGs are local and are not tracked. Run from the repository root:

```sh
go test ./classic -run '^$' -bench BenchmarkImageCodec -benchtime=10x -count=3
go test ./classic -run '^$' -bench 'BenchmarkImageCodec/3.jpg/Q80/Cycle$' -benchtime=10x -count=5
go test ./classic -run '^$' -bench BenchmarkRangeStreams -benchtime=300ms -count=3
go test -run '^TestBenchmarkSummary$' -v
```

For a before/after comparison, build each revision's test binary with the same
`performance_test.go` (including the new Cycle benchmark), then run the binaries
sequentially from `classic/`, alternating their order. Do not run benchmarks
alongside builds, tests or other benchmarks. Absolute timings depend on load.

The earlier in-memory JPEG/BABE/QOI comparison used `benchmark.jpg` and
`BenchmarkQualityComparison`, with three runs of two operations each. JPEG
quality 0 is clamped to 1. BABE speed and resource numbers were remeasured
after the quality-curve changes; JPEG and QOI rows are from earlier sessions.
A previous single-run README
measurement reported 8.638 s for JPEG encoding and 16.948 s for QOI encoding,
despite neither codec receiving comparable optimization here. That run cannot
be reliably compared with the current one: machine load or the measurement
environment may have affected it, but the precise cause was not recorded. Do
not use the ratio of those historical numbers as an optimization speedup.

New files use BRC1 method 16. This decoder continues to read older binary-range,
mixed range/Zstd and Zstd-only files. Older decoders cannot read method 16; see
[the format specification](BRC1.md).
