# Babe

**Babe** is an experimental image codec with several palette and bitplane storage modes.
It can work as a monochrome/YCbCr bitplane codec, or as a palette-index codec with tile-local palette subsets.

## Format

- `Y` is always stored
- `Cb` and `Cr` are stored in color mode, omitted in `bw` mode
- in plain monochrome/color modes each stored plane uses packed bit depths selected on the CLI
- palette modes store palette indices using tile-local subsets
- final payload is compressed with `zstd`

In plain color bitplane mode the raw image model is effectively `3 bits per pixel` before Zstandard.

## Current Model

1. Input is converted to planar `YCbCr`.
2. Each plane is thresholded against a procedurally generated blue-noise tile.
3. Depending on mode, the result is stored as packed bitplanes or palette indices.
4. The packed payload is written into a compact header + payload container.
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
babe input.png 100 -palette bright out.png
babe input.png 100 -palette luma out.png
babe input.png 100 -palette luma 4:2:2 out.png
babe input.png 100 -palette luma 4 2 2 out.png
babe input.png 100 -palette bw 2 out.png
babe input.png 100 -palette 16 out.png
babe input.png 100 -palette auto out.png
babe input.png 100 -palette auto 70 out.png
babe input.png 100 -palette gray 16 out.png
babe input.png 100 -palette zx out.png
babe input.png 100 -palette sunset out.png
babe input.png 100 -palette rgb out.png
babe input.png 100 -palette ycmrgbbw out.png
```

Rules:

- one numeric bit depth after `quality` means monochrome `bw`
- a triplet means color `Y:Cb:Cr`
- color triplet can be passed either as `4:4:4` or as `4 4 4`
- `-palette bright` uses the default stylized color palette path
- `-palette luma` uses luma/chroma bitplanes with default `1:1:1`
- `-palette luma y cb cr` and `-palette luma y:cb:cr` override luma/chroma bit depths
- `-palette bw` uses monochrome mode with default `1` bit
- `-palette bw N` uses monochrome mode with `N` bits, where `N` is in `[1..4]`
- `-palette N` builds an adaptive palette with `N` colors
- `-palette auto` chooses the adaptive palette size automatically
- `-palette auto P` keeps the automatic choice but pulls it toward `2` colors, where `P=100` keeps full auto and `P=0` forces `2` colors
- `-palette gray N` builds a grayscale palette with `N` shades, where `N` is in `[1..256]`
- `-palette zx|cga|ega|vga|c64|gameboy|pico8|db16|nes|sunset|pastel|ocean|forest` selects a fixed named palette
- `-palette <palette-spec>` selects a custom symbolic palette spec like `rgb`, `ymc`, `bw`, or `ycmrgbbw`
- if `decoded.png` is passed, the encoded `.babe` is immediately decoded into that PNG after encode

Decode:

```bash
babe input.babe
```

Named palettes:

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

Palette specs:

- symbolic specs are parsed from the letters `y`, `c`, `m`, `r`, `g`, `b`, `w`, `k`
- `bw` or `wb` adds black and white together
- examples:
  - `rgb`
  - `ymc`
  - `bw`
  - `ycmrgbbw`

## API

```go
comp, err := Encode(img, 80, false, 1)
dec, err := Decode(comp)
```

`quality` controls dithering strength rather than block partitioning.
When only one bit depth is provided, the encoder uses the monochrome luma-first photographic pipeline.

## Notes

- The CLI evolves quickly; keep `README.md` aligned with `main.go`.
- The bitstream format changed and is not compatible with older `.babe` files.
