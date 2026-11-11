// Command qualitycurve measures BABE size and RGB PSNR across qualities 0–100.
package main

import (
	"encoding/csv"
	"flag"
	"fmt"
	"image"
	"image/draw"
	_ "image/jpeg"
	_ "image/png"
	"math"
	"os"
	"strconv"

	"github.com/svanichkin/Babe/classic"
)

func main() {
	path := flag.String("input", "", "input image")
	flag.Parse()
	if *path == "" {
		fmt.Fprintln(os.Stderr, "usage: qualitycurve -input image.jpg")
		os.Exit(2)
	}
	if err := run(*path); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(path string) error {
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	src, _, err := image.Decode(f)
	f.Close()
	if err != nil {
		return err
	}
	switch src.(type) {
	case *image.RGBA64, *image.NRGBA64, *image.Gray16, *image.Alpha16:
		return fmt.Errorf("quality curve requires an opaque 8-bit RGB image")
	}
	bounds := src.Bounds()
	img := image.NewRGBA(image.Rect(0, 0, bounds.Dx(), bounds.Dy()))
	draw.Draw(img, img.Bounds(), src, bounds.Min, draw.Src)
	// Do not silently benchmark q100 against a converted transparent image.
	for i := 3; i < len(img.Pix); i += 4 {
		if img.Pix[i] != 255 {
			return fmt.Errorf("quality curve requires an opaque 8-bit RGB image")
		}
	}
	e, d := classic.NewEncoder(), classic.NewDecoder()
	w := csv.NewWriter(os.Stdout)
	if err := w.Write([]string{"quality", "bytes", "rgb_psnr_db"}); err != nil {
		return err
	}
	for q := 0; q <= 100; q++ {
		data, err := e.Encode(img, q, false)
		if err != nil {
			return err
		}
		decoded, err := d.Decode(data, false)
		if err != nil {
			return err
		}
		var squared uint64
		for i := 0; i < len(img.Pix); i += 4 {
			for c := 0; c < 3; c++ {
				delta := int(img.Pix[i+c]) - int(decoded.Pix[i+c])
				squared += uint64(delta * delta)
			}
		}
		psnr := math.Inf(1)
		if squared > 0 {
			mse := float64(squared) / float64(bounds.Dx()*bounds.Dy()*3)
			psnr = 10 * math.Log10(255*255/mse)
		}
		if err := w.Write([]string{strconv.Itoa(q), strconv.Itoa(len(data)), strconv.FormatFloat(psnr, 'f', 5, 64)}); err != nil {
			return err
		}
	}
	w.Flush()
	return w.Error()
}
