// Command compareimages generates decoded JPEG and BABE images from one source.
// It writes a JPEG bitstream and a lossless image of the BABE decoder output.
package main

import (
	"bytes"
	"flag"
	"fmt"
	"image"
	"image/draw"
	"image/jpeg"
	"image/png"
	"math"
	"os"
	"path/filepath"

	"github.com/svanichkin/Babe/classic"
)

func main() {
	input := flag.String("input", "", "source image")
	output := flag.String("out", ".", "output directory")
	quality := flag.Int("quality", 80, "quality (0–100)")
	flag.Parse()
	if *input == "" || *quality < 0 || *quality > 100 {
		fmt.Fprintln(os.Stderr, "specify -input and a -quality between 0 and 100")
		os.Exit(2)
	}
	if err := run(*input, *output, *quality); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(input, output string, quality int) error {
	f, err := os.Open(input)
	if err != nil {
		return err
	}
	src, _, err := image.Decode(f)
	f.Close()
	if err != nil {
		return err
	}
	// Use exactly the same input pixels and image type for both encoders.
	bounds := src.Bounds()
	img := image.NewRGBA(image.Rect(0, 0, bounds.Dx(), bounds.Dy()))
	draw.Draw(img, img.Bounds(), src, bounds.Min, draw.Src)
	var jpegData bytes.Buffer
	if err := jpeg.Encode(&jpegData, img, &jpeg.Options{Quality: quality}); err != nil {
		return err
	}
	jpegDecoded, err := jpeg.Decode(bytes.NewReader(jpegData.Bytes()))
	if err != nil {
		return err
	}
	encoded, err := classic.NewEncoder().Encode(img, quality, false)
	if err != nil {
		return err
	}
	decoded, err := classic.NewDecoder().Decode(encoded, false)
	if err != nil {
		return err
	}
	if err := os.MkdirAll(output, 0755); err != nil {
		return err
	}
	jpegName := filepath.Join(output, fmt.Sprintf("jpeg-q%d.jpg", quality))
	if err := os.WriteFile(jpegName, jpegData.Bytes(), 0644); err != nil {
		return err
	}
	babeName := filepath.Join(output, fmt.Sprintf("babe-q%d.png", quality))
	f, err = os.Create(babeName)
	if err != nil {
		return err
	}
	err = png.Encode(f, decoded)
	closeErr := f.Close()
	if err != nil {
		return err
	}
	if closeErr != nil {
		return closeErr
	}
	fmt.Printf("JPEG q%d: %d bytes, RGB PSNR %.2f dB -> %s\n", quality, jpegData.Len(), psnr(img, jpegDecoded), jpegName)
	fmt.Printf("BABE q%d: %d bytes, RGB PSNR %.2f dB -> lossless preview %s\n", quality, len(encoded), psnr(img, decoded), babeName)
	return nil
}

// PSNR is measured against the already-decoded input, using all RGB samples.
func psnr(src *image.RGBA, decoded image.Image) float64 {
	got := image.NewRGBA(src.Bounds())
	draw.Draw(got, got.Bounds(), decoded, decoded.Bounds().Min, draw.Src)
	var squared uint64
	for i := 0; i < len(src.Pix); i += 4 {
		for c := 0; c < 3; c++ {
			delta := int(src.Pix[i+c]) - int(got.Pix[i+c])
			squared += uint64(delta * delta)
		}
	}
	if squared == 0 {
		return math.Inf(1)
	}
	mse := float64(squared) / float64(src.Bounds().Dx()*src.Bounds().Dy()*3)
	return 10 * math.Log10(255*255/mse)
}
