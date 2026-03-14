# Babe

**Babe** is an experimental image codec with several palette and bitplane storage modes.
It can work as a monochrome/YCbCr bitplane codec, or as a palette-index codec with `raw` and `reconstruct` transforms.

## Format

- `Y` is always stored
- `Cb` and `Cr` are stored in color mode, omitted in `bw` mode
- in plain monochrome/color modes each stored plane uses exactly `1 bit` per pixel
- palette modes store palette indices or palette-derived peel planes
- final payload can be stored raw or compressed with `zstd`

In plain color bitplane mode the raw image model is effectively `3 bits per pixel` before Zstandard.

## Current Model

1. Input is converted to planar `YCbCr`.
2. Each plane is thresholded against a procedurally generated blue-noise tile.
3. Depending on mode, the result is stored as packed bitplanes, palette indices, or reconstruct peel planes.
4. The packed payload is written into a compact header + payload container.
5. The container can optionally be compressed with Zstandard.

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
babe input.png 100 -adaptive 16 out.png -z
babe input.png 100 -adaptive auto out.png -z
babe input.png 100 -adaptive auto 70 out.png -z
babe input.png 100 -adaptive 16 out.png -raw -z
babe input.png 100 -adaptive 16 out.png -raw -top16 -z
babe input.png 100 -adaptive 16 out.png -raw -tree -z
babe input.png 100 -adaptive 16 out.png -reconstruct -z
babe input.png 100 -gray 16 out.png -z
```

Rules:

- one numeric bit depth after `quality` means monochrome `bw`
- a triplet means color `Y:Cb:Cr`
- color triplet can be passed either as `4:4:4` or as `4 4 4`
- `-adaptive N` builds an adaptive palette with `N` colors
- `-adaptive auto` chooses the adaptive palette size automatically
- `-adaptive auto P` keeps the automatic choice but pulls it toward `2` colors, where `P=100` keeps full auto and `P=0` forces `2` colors
- `-gray N` builds a grayscale palette with `N` shades, where `N` is in `[1..256]`

Decode:

```bash
babe input.babe
```

Useful flags:

- `-z` or `-zstd`: compress the final payload with `zstd`
- `shuffle`: enable blue-noise phase shuffling
- `-log`: write an additional `*.log.png` visualization for supported modes
- `-tile N`: tile-local palette subset mode, `N` in `[2..255]`
- `-block`: raw palette indices in `16x16` subset blocks
- `-raw`: store palette indices directly instead of reconstruct peel planes
- `-top16`: raw `top16` transform over the packed raw palette stream
- `-tree`: raw binary range-tree transform over palette indices
- `-shift N`: bit shift for the `-top16` raw transform, `N` in `[0..7]`
- `-reconstruct`: store palette data as sequential peel planes

Current palette names:

- `adaptive`
- `zx`
- `cga`
- `ega`
- `vga`
- `c64`
- `gameboy`
- `pico8`
- `db16`
- `nes`
- `sunset`
- `pastel`
- `ocean`
- `forest`
- custom palette specs like `#000000,#ffffff,#ff0000`

## API

```go
comp, err := Encode(img, 80, false)
dec, err := Decode(comp, false)
```

`quality` controls dithering strength rather than block partitioning.
When only one bit depth is provided, the encoder uses the monochrome luma-first photographic pipeline.

## Notes

- The CLI evolves quickly; keep `README.md` aligned with `main.go`.
- `postfilter` is currently a no-op.
- The bitstream format changed and is not compatible with older `.babe` files.
