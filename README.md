# Babe

`Babe` is an experimental lossy image codec based on adaptive block subdivision, two-tone block approximation, and Zstandard compression.

The current on-disk format written by the encoder is `BABE-L` (BABE Light).

## What It Does

- Converts the image into Y/Cb/Cr planes
- Splits the image into a block hierarchy from small blocks up to large blocks
- Encodes each block either as:
  - solid: one color
  - patterned: two colors plus a binary pattern mask
- Delta-packs block colors
- Compresses the raw stream with Zstandard

The codec is designed around fast sequential decode and compact streams, not exact reconstruction.

## Current Format

The compressed `.babe` file is:

```text
zstd( raw_babe_stream )
```

The raw stream begins with the magic:

```text
BABE-L\n
```

Then the file header:

```text
u16  pattern count
u16  block level count
u16  block levels[levelCount]
u8   channel mask
u32  image width
u32  image height
channel stream for Y
optional channel stream for Cb
optional channel stream for Cr
optional chroma grid overlay
```

Channel mask bits:

- bit `0`: Y present
- bit `1`: Cb present
- bit `2`: Cr present
- bit `7`: chroma grid overlay present

## Channel Stream Layout

Each channel stream is written as:

```text
u32  blockCount
u32  sizeStreamLen
[]byte sizeStream
u32  patternStreamLen
[]byte patternStream
u32  fgPackedLen
[]byte fgPacked
u32  bgPackedLen
[]byte bgPacked
```

Meaning of each stream:

- `sizeStream`: split/use bits for the block hierarchy
- `patternStream`: pattern codebook indices for patterned blocks only
- `fgPacked`: one FG value per emitted block, delta-packed
- `bgPacked`: one BG value per patterned block, delta-packed

## Important Format Detail

Block type is encoded in the parity of the stored `fg` byte:

- odd `fg` => patterned block
- even `fg` => solid block

To make this work, the encoder may shift `fg` by `+1` or `-1` so it has the required parity. This is intentional and lossy. The original `fg` is not restored on decode.

For patterned blocks, the encoder also avoids changing `fg` into a value equal to `bg`, so the pattern does not collapse into a flat block.

## Block Hierarchy

By default, block sizes come from quality presets. They can also be overridden explicitly.

Current default quality mapping:

- `80..100` => `1,2`
- `60..79` => `1,3`
- `40..59` => `2,4`
- `20..39` => `3,6`
- `0..19` => `16,32`

You can override this with `-blocks=...`.

Examples:

- `-blocks=2-64`
- `-blocks=2,4,8,16,32,64`
- `-blocks=4,8,16`

The hierarchy must be strictly increasing, and every level must be a multiple of the previous one.

## Patterns

Patterns are not stored as raw bitmasks in the stream. The encoder uses a fixed codebook and writes pattern indices.

Relevant knobs:

- `-patterns=N`
  default: `64`
  valid range: `1..1024`
 

## Chroma Grid Overlay

Optional chroma simplification can be enabled with:

- `-tile N`
  valid range: `2..255`
  default: disabled (`0`)

When enabled, chroma can be stored as a coarse grid overlay instead of full Cb/Cr block streams.

## CLI

Encode:

```bash
babe input.png
babe input.png 70
babe input.png 70 bw
babe input.png 70 out.png
```

Decode:

```bash
babe input.babe
babe input.babe -layers
```

### Encode Keys

- `quality`
  positional
  default: `70`
  range: `0..100`
- `bw`
  grayscale mode, stores only Y
- `-patterns=N`
  pattern codebook size, `1..1024`
- `-blocks=A,B,...`
  explicit block hierarchy
- `-blocks=A-B`
  power-of-two hierarchy from `A` to `B`
- `-q N`
  Y quantization shift, `0..7`
- `-tile N`
  or `-tile=N`
  chroma grid tile size, `2..255`
- `-layers`
  after encode, also writes layer debug output through the decode path
- `<decoded.png>`
  if a `.png` path is present in encode args, the freshly encoded file is decoded to that PNG

### Decode Keys

- `-layers`
  writes split/layer debug output

## API

Encode:

```go
comp, err := Encode(img, quality, bwmode)
if err != nil {
    // handle error
}
```

Decode:

```go
img, err := Decode(comp, false)
if err != nil {
    // handle error
}
```

Convenience constructors are also available:

- `NewEncoder()`
- `NewDecoder()`
- `(*Encoder).Encode`
- `(*Encoder).EncodeTo`
- `(*Decoder).Decode`
- `(*Decoder).DecodeFrom`

## Notes

- The format is experimental and intentionally unstable.
- `BABE-L` is the only on-disk layout supported in this branch.
- The README describes the current codebase, not a stable public specification.

## License

MIT
