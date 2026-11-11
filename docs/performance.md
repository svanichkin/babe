# Range-coding speed measurements

Measured locally on Apple M3, quality 10, using `Babe input.jpg 10 output.png`. Each result is the median of five CLI runs after warm-up, alternating the old and new binaries. Times vary with machine load.

Before: exhaustive range/Zstd search and maximum PNG compression. After: one order-0 range pass, up to four entropy workers, reused buffers, direct decoding into the payload, and standard PNG compression. Quantization is unchanged.

| Image | Metric | Before | After |
|---|---|---:|---:|
| 1.jpg | BABE bytes | 60625 | 61163 |
| 1.jpg | PNG bytes | 198157 | 210520 |
| 1.jpg | BABE encode ms | 66.48 | 8.15 |
| 1.jpg | BABE decode ms | 8.92 | 6.94 |
| 1.jpg | Whole command ms | 449.28 | 67.97 |
| 3.jpg | BABE bytes | 303829 | 306244 |
| 3.jpg | PNG bytes | 972867 | 1053958 |
| 3.jpg | BABE encode ms | 320.99 | 51.20 |
| 3.jpg | BABE decode ms | 63.75 | 43.33 |
| 3.jpg | Whole command ms | 2629.53 | 373.48 |

The complete CLI timings include JPEG loading, BABE writing, decoding, PNG encoding, process startup and file I/O. The printed encode/decode timings exclude PNG writing. Both output PNGs were decoded and compared pixel-for-pixel with the earlier binary: they are identical.

The new encoder writes only range-coded or raw streams; Zstd and the older context models remain available for reading existing files. File-size growth at quality 10 was 0.8–3.4% across the four local samples. PNG grows by about 6–8% for the two samples above because export now uses standard rather than maximum compression.

Optional reproducible in-process benchmarks (sample JPEGs are not tracked):

```sh
go test ./classic -run '^$' -bench BenchmarkImageCodec -benchtime=3x -count=3
```

These benchmarks exclude JPEG loading, PNG export and file I/O. They report output size and allocations as well as elapsed time.
