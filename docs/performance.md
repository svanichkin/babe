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

## Lossless CLI comparison

The [README table](../README.md#lossless-complete-encodedecode-cycle) uses the
same 5152×7728 local `benchmark.jpg`, decoded once into an opaque 8-bit RGB PPM.
BABE q100, QOI, JPEG XL, PNG + OxiPNG and WebP each encode this same PPM and
then decode their output to RGB PPM. A SHA-256 comparison of dimensions and
pixel bytes verifies every warm-up and measured cycle. JPEG source metadata and
the original JPEG bitstream are not part of the comparison.

There is one warm-up per format, followed by three complete measured rounds,
each containing all five formats in a shuffled order. Each cycle starts new
processes and includes reading and writing the files, process startup and RGB
conversion. PNG encoding includes both the initial Go PNG encoding and the
subsequent OxiPNG optimization. Full-cycle wall time is measured directly.
CPU time is the sum of user and system time for all stages and threads; peak
RSS is the highest individual process high-water mark among those sequential
stages. The table gives medians across the three cycles, not a sum of medians.
Unlike the earlier in-memory measurements, buffers are not reused across
processes, and disk I/O is included.

Apple M3, macOS, Go 1.27.3, GOMAXPROCS=8; libjxl 0.12.0 (`-d 0 -e 5`,
8 threads), libwebp 1.6.0 (`-lossless -q 75 -m 4 -mt`), OxiPNG 10.2.1
(`-o2 -t 8 --strip safe`). Encoder settings are not equivalent optimization
levels. These are results for one image and machine; three runs reduce reliance
on one timing but do not eliminate effects of system load.

| Format | Full-cycle wall time, three rounds | CPU time, three rounds |
|---|---|---|
| BABE | 1.677, 1.291, 1.198 s | 1.978, 2.028, 1.962 s |
| QOI | 1.816, 1.715, 1.777 s | 1.720, 1.663, 1.708 s |
| JPEG XL | 4.584, 4.923, 6.276 s | 27.497, 27.319, 28.404 s |
| PNG + OxiPNG | 13.696, 13.448, 14.118 s | 17.684, 17.605, 18.206 s |
| WebP | 18.153, 16.125, 17.961 s | 17.346, 15.975, 16.987 s |

The [individual results and exact settings](benchmarks/lossless-cli-apple-m3.json)
include wall time, CPU, process peak RSS and encoded size for all 15 cycles.
To reproduce on macOS or Linux with `cjxl`, `djxl`, `cwebp`, `dwebp` and
`oxipng` installed:

```sh
go build -o /tmp/babe-losslessbench ./cmd/losslessbench
python3 scripts/compare-lossless.py --input benchmark.jpg --helper /tmp/babe-losslessbench --rounds 3 --threads 8 --output /tmp/lossless-results.json
```

## CPU and memory

The [README](../README.md#benchmark-comparison) lists CPU time, average core
use, Go heap allocations per operation, and peak process RSS for each quality
and codec. The input is the same local 5152×7728 `benchmark.jpg` used in the
time comparison. Measurements were made on Apple M3, darwin/arm64, Go 1.27.3,
`GOMAXPROCS=8`, using three separate processes per case with two timed, warmed
operations in each. Execution order was shuffled. The tables use medians of
the three process results. The [original individual measurements](benchmarks/resources-apple-m3.json)
are retained; the README's BABE rows use the updated
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
[current scale](benchmarks/quality-curve-current.csv). The other three photos
are local examples; replace them with your own to evaluate other content.

```sh
go run ./cmd/qualitycurve -input samples/source.jpg > /tmp/babe-quality.csv
```

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

The README retains the JPEG/BABE/QOI comparison on `benchmark.jpg`, quality 80
for JPEG and BABE, QOI lossless, and adds JPEG/BABE at qualities 0, 10, 50, 80
and 100. Those numbers are medians of three runs with two operations per run
using `BenchmarkQualityComparison`; JPEG quality 0 is clamped to 1. The BABE
speed and resource numbers were remeasured after the quality-curve changes;
JPEG and QOI rows are from earlier sessions. The table
reports encode and decode medians separately. A previous single-run README
measurement reported 8.638 s for JPEG encoding and 16.948 s for QOI encoding,
despite neither codec receiving comparable optimization here. That run cannot
be reliably compared with the current one: machine load or the measurement
environment may have affected it, but the precise cause was not recorded. Do
not use the ratio of those historical numbers as an optimization speedup.

New files use BRC1 method 16. This decoder continues to read older binary-range,
mixed range/Zstd and Zstd-only files. Older decoders cannot read method 16; see
[the format specification](BRC1.md).
