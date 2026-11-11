# BABE encoder and decoder performance

Measured on Apple M3, darwin/arm64, Go 1.27.3, default GOMAXPROCS=8.
Only the in-memory BABE `Encode` and `Decode` APIs are timed. Images are loaded
before timing, encoder/decoder instances and buffers are reused, and both paths
are warmed up. `Cycle` times an actual encode followed immediately by a decode;
it is not the sum of two independently timed measurements. Postfilter is off.
The local samples are 800x572 (`1.jpg`) and 1920x1080 (`3.jpg`).

Before: commit `6978b1b`, adaptive binary order-0 range coding. After: byte-symbol
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
using `BenchmarkQualityComparison`; JPEG quality 0 is clamped to 1. The table
reports encode and decode medians separately. A previous single-run README
measurement reported 8.638 s for JPEG encoding and 16.948 s for QOI encoding,
despite neither codec receiving comparable optimization here. That run cannot
be reliably compared with the current one: machine load or the measurement
environment may have affected it, but the precise cause was not recorded. Do
not use the ratio of those historical numbers as an optimization speedup.

New files use BRC1 method 16. This decoder continues to read older binary-range,
mixed range/Zstd and Zstd-only files. Older decoders cannot read method 16; see
[the format specification](BRC1.md).
