# Babe

**Babe** is an experimental 1-bit-per-channel image codec.
It converts the source into `YCbCr`, quantizes each stored channel to a single bit, applies blue-noise dithering during encode, then compresses the packed bitplanes with Zstandard.

## Format

- `Y` is always stored
- `Cb` and `Cr` are stored in color mode, omitted in `bw` mode
- each stored plane uses exactly `1 bit` per pixel
- decode reconstructs the image from fixed low/high levels per channel
- final payload is `zstd(bitstream)`

In color mode this means the raw image model is effectively `3 bits per pixel` before Zstandard.

## Current Model

1. Input is converted to planar `YCbCr`.
2. Each plane is thresholded against a procedurally generated blue-noise tile.
3. The result is packed into 1-bit planes.
4. The packed planes are written into a compact header + payload container.
5. The container is compressed with Zstandard.

The blue-noise tile is generated with a deterministic best-candidate sampler on a toroidal grid, so the threshold map is actually noise-distributed rather than a small ordered-dither table.

This is intentionally aggressive and stylized. The goal is not fidelity; the goal is an ultra-small, fast, visibly dithered codec.

## CLI

Encode:

```bash
babe input.png
babe input.png 85
babe input.png 70 3
babe input.png 100 4:4:4
babe input.png 100 3 1 1
```

Rules:

- one numeric bit depth after `quality` means monochrome `bw`
- a triplet means color `Y:Cb:Cr`
- color triplet can be passed either as `4:4:4` or as `4 4 4`

Decode:

```bash
babe input.babe
```

## API

```go
comp, err := Encode(img, 80, false)
dec, err := Decode(comp, false)
```

`quality` now controls dithering strength rather than block partitioning.
When only one bit depth is provided, the encoder uses the monochrome luma-first photographic pipeline.

## Notes

- The previous block dual-tone model has been removed.
- `postfilter` is currently a no-op.
- The bitstream format changed and is not compatible with older `.babe` files.
