package classic

import (
	"fmt"
	"image"
	"image/color"
	"image/draw"
)

// extractLosslessPlanes records opaque 8-bit RGB as G, B-G and R-G using
// modulo-256 arithmetic. Unlike YCbCr conversion this transform is reversible.
// JPEG input is compared as the decoded 8-bit RGB image, not as file bytes.
func (e *Encoder) extractLosslessPlanes(img image.Image, bw bool) error {
	switch src := img.(type) {
	case *image.RGBA, *image.NRGBA, *image.YCbCr, *image.Gray, *image.CMYK:
	case *image.Paletted:
		// A palette may contain 16-bit colors even when its indices are bytes.
		for _, entry := range src.Palette {
			switch entry.(type) {
			case color.RGBA, color.NRGBA, color.Gray, color.YCbCr, color.CMYK:
			default:
				return fmt.Errorf("lossless quality 100 requires an 8-bit palette (got %T)", entry)
			}
		}
	default:
		return fmt.Errorf("lossless quality 100 supports 8-bit RGB images only (got %T)", img)
	}
	b := img.Bounds()
	w, h := b.Dx(), b.Dy()
	rgba, ok := img.(*image.RGBA)
	if !ok {
		if e.losslessRGB == nil || e.losslessRGB.Bounds().Dx() != w || e.losslessRGB.Bounds().Dy() != h {
			e.losslessRGB = image.NewRGBA(image.Rect(0, 0, w, h))
		}
		rgba = e.losslessRGB
		draw.Draw(rgba, rgba.Bounds(), img, b.Min, draw.Src)
	}
	for y := 0; y < h; y++ {
		row := rgba.Pix[y*rgba.Stride : y*rgba.Stride+w*4]
		for x := 0; x < w; x++ {
			r, g, b, a := row[x*4], row[x*4+1], row[x*4+2], row[x*4+3]
			if a != 255 {
				return fmt.Errorf("lossless quality 100 does not support transparency")
			}
			if bw && (r != g || g != b) {
				return fmt.Errorf("lossless quality 100 cannot convert color to grayscale")
			}
			i := y*w + x
			e.yPlane[i], e.cbPlane[i], e.crPlane[i] = g, b-g+128, r-g+128
		}
	}
	return nil
}

// exactQuad returns a block's actual levels and mask when it has at most two
// distinct samples. Three or four levels must be stored as 1x1 blocks.
func exactQuad(v0, v1, v2, v3 byte) (fg, bg byte, bits uint64, ok bool) {
	fg, bg = v0, v0
	values := [4]byte{v0, v1, v2, v3}
	for _, v := range values[1:] {
		if v != fg {
			bg = v
			break
		}
	}
	for _, v := range values {
		bits <<= 1
		if v == fg {
			bits |= 1
		} else if v != bg {
			return 0, 0, 0, false
		}
	}
	return fg, bg, bits, true
}
