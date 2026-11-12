package classic

import (
	"fmt"
	"math"
)

// transformBackground maps the original delta-coded background levels to
// within-glyph FG-BG differences, or back. The type bits identify which FG
// levels have a background. All arithmetic is modulo 256, including lossless
// channel residuals, so the transformation never changes reconstructed pixels.
func transformBackground(bg, fg, types []byte, inverse bool) error {
	if len(types) != (len(fg)+7)/8 {
		return fmt.Errorf("invalid contrast type stream length")
	}
	var foreground, background byte
	j := 0
	for i, delta := range fg {
		foreground += delta
		if types[i/8]&(128>>uint(i&7)) == 0 {
			continue
		}
		if j >= len(bg) {
			return fmt.Errorf("truncated contrast background stream")
		}
		if inverse {
			value := foreground - bg[j]
			bg[j] = value - background
			background = value
		} else {
			background += bg[j]
			bg[j] = foreground - background
		}
		j++
	}
	if j != len(bg) {
		return fmt.Errorf("trailing contrast background values")
	}
	return nil
}

// estimateRangeBytes compares predictors without range-encoding both complete
// streams. Include each block's actual model header and use its normalized
// probabilities to estimate the coded bits. This is a size estimate, not an
// exact count; final storage still falls back to the original raw bytes.
func estimateRangeBytes(src []byte) float64 {
	total := 0.0
	for len(src) > 0 {
		n := min(byteRangeBlock, len(src))
		block := src[:n]
		src = src[n:]
		model, used, _ := makeByteRangeModel(block)
		if used == 1 {
			total += 2
			continue
		}
		var counts [256]int
		for _, v := range block {
			counts[v]++
		}
		header := 5 + 32 + 7
		if used < 32 {
			header = 5 + 1 + used + 7
		}
		remaining := used
		cost := 0.0
		for i, f := range model.freq {
			if f == 0 {
				continue
			}
			cost += float64(counts[i]) * (byteRangeBits - math.Log2(float64(f)))
			remaining--
			if remaining > 0 {
				header++
				if f >= 128 {
					header++
				}
			}
		}
		total += min(float64(n+1), float64(header)+cost/8)
	}
	return total
}
