package classic

import (
	"bytes"
	"image"
	"image/color"
	"image/draw"
	_ "image/jpeg"
	_ "image/png"
	"math"
	"os"
	"testing"
)

// This checks the calibrated curve on one fixed photograph; it is not a claim
// that arbitrary images have monotonic sizes or perceptually linear quality.
func TestQualityCurveOnExample(t *testing.T) {
	f, err := os.Open("../samples/source.jpg")
	if err != nil {
		t.Fatal(err)
	}
	src, _, err := image.Decode(f)
	f.Close()
	if err != nil {
		t.Fatal(err)
	}
	img := image.NewRGBA(image.Rect(0, 0, src.Bounds().Dx(), src.Bounds().Dy()))
	draw.Draw(img, img.Bounds(), src, src.Bounds().Min, draw.Src)

	f, err = os.Open("../samples/babe-q0.png")
	if err != nil {
		t.Fatal(err)
	}
	reference, _, err := image.Decode(f)
	f.Close()
	if err != nil {
		t.Fatal(err)
	}
	wantZero := image.NewRGBA(img.Bounds())
	draw.Draw(wantZero, wantZero.Bounds(), reference, reference.Bounds().Min, draw.Src)

	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	var sizes [101]int
	lastPSNR := 0.0
	for q := 0; q <= 100; q++ {
		packed, err := e.Encode(img, q, false)
		if err != nil {
			t.Fatalf("q%d: %v", q, err)
		}
		sizes[q] = len(packed)
		decoded, err := d.Decode(packed, false)
		if err != nil {
			t.Fatalf("q%d: %v", q, err)
		}
		if q == 0 && !bytes.Equal(decoded.Pix, wantZero.Pix) {
			t.Fatal("quality 0 example changed")
		}
		if q == 100 {
			if !bytes.Equal(decoded.Pix, img.Pix) {
				t.Fatal("quality 100 is not lossless")
			}
			continue
		}
		var squared uint64
		for i := 0; i < len(img.Pix); i += 4 {
			for c := 0; c < 3; c++ {
				diff := int(img.Pix[i+c]) - int(decoded.Pix[i+c])
				squared += uint64(diff * diff)
			}
		}
		mse := float64(squared) / float64(img.Bounds().Dx()*img.Bounds().Dy()*3)
		psnr := 10 * math.Log10(255*255/mse)
		if q > 0 && psnr < lastPSNR-0.5 {
			t.Fatalf("q%d: PSNR dropped from %.2f to %.2f dB", q, lastPSNR, psnr)
		}
		lastPSNR = psnr
	}
	var totalError float64
	for q := 0; q <= 100; q++ {
		if q > 0 && sizes[q]*100 < sizes[q-1]*98 {
			t.Fatalf("q%d: example size fell from %d to %d bytes", q, sizes[q-1], sizes[q])
		}
		actual := float64(sizes[q]-sizes[0]) / float64(sizes[100]-sizes[0])
		totalError += math.Abs(actual - float64(q)/100)
	}
	if totalError/101 > 0.05 {
		t.Fatalf("mean normalized size deviation = %.3f", totalError/101)
	}
}

func TestQualityCurvePreservesEdges(t *testing.T) {
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	for _, dims := range []image.Point{{1, 1}, {5, 7}, {9, 8}, {8, 9}} {
		img := image.NewRGBA(image.Rect(0, 0, dims.X, dims.Y))
		draw.Draw(img, img.Bounds(), &image.Uniform{C: color.RGBA{180, 160, 140, 255}}, image.Point{}, draw.Src)
		for _, q := range []int{0, 1, 10, 20, 50, 80, 99, 100} {
			data, err := e.Encode(img, q, false)
			if err != nil {
				t.Fatalf("%v q%d: %v", dims, q, err)
			}
			got, err := d.Decode(data, false)
			if err != nil || got.Bounds() != img.Bounds() {
				t.Fatalf("%v q%d: invalid output: %v", dims, q, err)
			}
			c := got.RGBAAt(dims.X-1, dims.Y-1)
			if c.R < 80 || c.G < 80 || c.B < 80 || c.A != 255 {
				t.Fatalf("%v q%d: edge lost: %v", dims, q, c)
			}
		}
	}
}
