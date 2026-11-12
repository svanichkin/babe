// Command losslessbench provides comparable PPM input/output for CLI benchmarks.
// It is not part of the BABE bitstream or production encoder.
package main

import (
	"bufio"
	"fmt"
	"image"
	"image/draw"
	"image/jpeg"
	"image/png"
	"io"
	"os"
	"strconv"
	"strings"
	"unicode"

	"github.com/svanichkin/Babe/classic"
	"github.com/xfmoulet/qoi"
)

func main() {
	if len(os.Args) != 5 {
		fmt.Fprintln(os.Stderr, "usage: losslessbench prepare|encode|decode babe[:quality]|jpeg[:quality]|qoi|png|source input output")
		os.Exit(2)
	}
	if err := run(os.Args[1], os.Args[2], os.Args[3], os.Args[4]); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(op, codec, input, output string) error {
	quality := 100
	glyphMinSize := 0
	if name, value, ok := strings.Cut(codec, ":"); ok {
		var err error
		quality, err = strconv.Atoi(value)
		if err != nil || quality < 0 || quality > 100 || (name != "babe" && name != "babe-cornerdata" && name != "babe-glyphs" && name != "babe-sparse" && name != "babe-dither" && name != "babe-pattern2" && name != "babe-combined" && name != "jpeg") {
			return fmt.Errorf("invalid codec quality %q", codec)
		}
		if name == "babe-glyphs" {
			glyphMinSize = quality
		}
		codec = name
	}
	f, err := os.Open(input)
	if err != nil {
		return err
	}
	defer f.Close()
	var img image.Image
	switch op {
	case "prepare":
		if codec != "source" {
			return fmt.Errorf("prepare requires source codec")
		}
		img, _, err = image.Decode(f)
		if err == nil {
			switch img.(type) {
			case *image.RGBA64, *image.NRGBA64, *image.Gray16, *image.Alpha16:
				return fmt.Errorf("benchmark requires opaque 8-bit input")
			}
		}
	case "encode":
		img, err = readPPM(f)
	case "decode":
		switch codec {
		case "babe":
			var data []byte
			data, err = io.ReadAll(f)
			if err == nil {
				img, err = classic.NewDecoder().Decode(data, false)
			}
		case "babe-corners":
			var data []byte
			data, err = io.ReadAll(f)
			if err == nil {
				decoder := classic.NewDecoder()
				decoder.CornerGradient = true
				img, err = decoder.Decode(data, false)
			}
		case "babe-glyphs":
			var data []byte
			data, err = io.ReadAll(f)
			if err == nil {
				decoder := classic.NewDecoder()
				decoder.GlyphBlend = true
				decoder.GlyphMinSize = glyphMinSize
				img, err = decoder.Decode(data, false)
			}
		case "qoi":
			img, err = qoi.Decode(f)
		case "png":
			img, err = png.Decode(f)
		case "jpeg":
			img, err = jpeg.Decode(f)
		default:
			return fmt.Errorf("unsupported decoder %q", codec)
		}
	default:
		return fmt.Errorf("unsupported operation %q", op)
	}
	if err != nil {
		return err
	}
	out, err := os.Create(output)
	if err != nil {
		return err
	}
	defer out.Close()
	if op == "prepare" || op == "decode" {
		err = writePPM(out, img)
	} else {
		switch codec {
		case "babe":
			err = classic.NewEncoder().EncodeTo(out, img, quality, false)
		case "babe-cornerdata":
			encoder := classic.NewEncoder()
			encoder.CornerData = true
			err = encoder.EncodeTo(out, img, quality, false)
		case "babe-sparse", "babe-dither", "babe-pattern2", "babe-combined":
			encoder := classic.NewEncoder()
			encoder.SparseCorners = true
			encoder.DitherPatterns = codec == "babe-dither" || codec == "babe-combined"
			encoder.SimplifyPatterns = codec == "babe-pattern2" || codec == "babe-combined"
			err = encoder.EncodeTo(out, img, quality, false)
		case "jpeg":
			err = jpeg.Encode(out, img, &jpeg.Options{Quality: quality})
		case "qoi":
			err = qoi.Encode(out, img)
		case "png":
			err = png.Encode(out, img)
		default:
			err = fmt.Errorf("unsupported encoder %q", codec)
		}
	}
	if err != nil {
		return err
	}
	return out.Close()
}

// The benchmark uses opaque 8-bit P6 PPM. readPPM intentionally bounds image
// dimensions before allocation and reads rows without a second full image copy.
func readPPM(src io.Reader) (*image.RGBA, error) {
	r := bufio.NewReader(src)
	var token [4]string
	for i := range token {
		value, err := ppmToken(r)
		if err != nil {
			return nil, err
		}
		token[i] = value
	}
	w, werr := strconv.Atoi(token[1])
	h, herr := strconv.Atoi(token[2])
	if token[0] != "P6" || token[3] != "255" || werr != nil || herr != nil || w < 1 || h < 1 || w > 1<<26 || h > 1<<26 || uint64(w)*uint64(h) > 1<<26 {
		return nil, fmt.Errorf("invalid P6 image header")
	}
	img := image.NewRGBA(image.Rect(0, 0, w, h))
	row := make([]byte, w*3)
	for y := 0; y < h; y++ {
		if _, err := io.ReadFull(r, row); err != nil {
			return nil, err
		}
		for x := 0; x < w; x++ {
			i, j := y*img.Stride+x*4, x*3
			img.Pix[i], img.Pix[i+1], img.Pix[i+2], img.Pix[i+3] = row[j], row[j+1], row[j+2], 255
		}
	}
	if _, err := r.ReadByte(); err != io.EOF {
		return nil, fmt.Errorf("trailing PPM data")
	}
	return img, nil
}

func ppmToken(r *bufio.Reader) (string, error) {
	var token []byte
	for {
		b, err := r.ReadByte()
		if err != nil {
			return "", err
		}
		if b == '#' && len(token) == 0 {
			if _, err := r.ReadBytes('\n'); err != nil {
				return "", err
			}
			continue
		}
		if unicode.IsSpace(rune(b)) {
			if len(token) == 0 {
				continue
			}
			return string(token), nil
		}
		token = append(token, b)
		if len(token) > 32 {
			return "", fmt.Errorf("PPM header token too long")
		}
	}
}

func writePPM(dst io.Writer, src image.Image) error {
	b := src.Bounds()
	rgba, ok := src.(*image.RGBA)
	if !ok || b.Min != (image.Point{}) {
		rgba = image.NewRGBA(image.Rect(0, 0, b.Dx(), b.Dy()))
		draw.Draw(rgba, rgba.Bounds(), src, b.Min, draw.Src)
	}
	w := bufio.NewWriter(dst)
	if _, err := fmt.Fprintf(w, "P6\n%d %d\n255\n", b.Dx(), b.Dy()); err != nil {
		return err
	}
	row := make([]byte, b.Dx()*3)
	for y := 0; y < b.Dy(); y++ {
		for x := 0; x < b.Dx(); x++ {
			i, j := y*rgba.Stride+x*4, x*3
			if rgba.Pix[i+3] != 255 {
				return fmt.Errorf("benchmark requires opaque pixels")
			}
			row[j], row[j+1], row[j+2] = rgba.Pix[i], rgba.Pix[i+1], rgba.Pix[i+2]
		}
		if _, err := w.Write(row); err != nil {
			return err
		}
	}
	return w.Flush()
}
