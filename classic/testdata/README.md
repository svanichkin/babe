# Legacy compatibility fixture

`legacy.babe` and its decoded `legacy.png` were produced with the unmodified
Zstd-only executable before the compression-first changes, at quality 70.
The input is a 13×11 RGB PNG generated with:

```text
R = (19*x + 7*y) mod 256
G = (3*x + 23*y) mod 256
B = (29*x + 11*y) mod 256
```

The test compares the new decoder's pixels with the old decoder's PNG. This
fixture must not be regenerated using the new encoder when testing compatibility.

`mixed.babe` and `mixed.png` were produced by the exhaustive range/Zstd encoder
before replacing its search with a single range pass. The source is the nonzero
origin, odd-sized image from `compressionTestImage`, at quality 70. These fixtures
verify that reading previous mixed-method BRC1 files preserves their pixels.
