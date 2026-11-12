package main

import (
	"fmt"
	"image"
	"image/color"
	_ "image/gif"
	_ "image/jpeg"
	"image/png"
	"io"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"
)

func main() {
	if len(os.Args) < 2 {
		fmt.Fprint(os.Stderr, "Usage:\n  babe <input-image> [quality] [bw] [-corners|-sparse] [-glyphs] [-glyphs-only] [-dither] [-dither2] [-pattern2] [-s size] [decoded.png]\n  babe <input.babe> [-postfilter] [-corners] [-glyphs] [-glyphs-only] [-dither2] [-s size]\n")
		os.Exit(1)
	}
	options, minSize, err := parseMinBlockSize(os.Args[2:])
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}

	inputPath := os.Args[1]
	ext := strings.ToLower(filepath.Ext(inputPath))
	base := strings.TrimSuffix(inputPath, filepath.Ext(inputPath))

	// If input is .babe → decode to PNG
	if ext == ".babe" {
		postfilter := false
		corners := false
		glyphs := minSize > 0
		glyphsOnly := false
		dither2 := false
		for _, a := range options {
			if a == "-postfilter" {
				postfilter = true
			}
			if a == "-corners" {
				corners = true
			}
			if a == "-glyphs" {
				glyphs = true
			}
			if a == "-glyphs-only" {
				glyphsOnly = true
			}
			if a == "-dither2" {
				dither2 = true
			}
		}
		if err := decodeBabe(inputPath, base+".png", false, postfilter, corners, glyphs, glyphsOnly, dither2, minSize); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
		return
	}

	// Otherwise: encode image → .babe with default or provided quality
	quality := defaultEncodeQuality
	encodeArgs := options
	if len(encodeArgs) > 0 {
		q, err := strconv.Atoi(encodeArgs[0])
		if err != nil && !strings.HasPrefix(encodeArgs[0], "-") && encodeArgs[0] != "bw" && !strings.EqualFold(filepath.Ext(encodeArgs[0]), ".png") {
			fmt.Fprintln(os.Stderr, "quality must be an integer between 0 and 100")
			os.Exit(1)
		}
		if err == nil {
			if q < 0 || q > 100 {
				fmt.Fprintln(os.Stderr, "quality must be between 0 and 100")
				os.Exit(1)
			}
			quality = q
			encodeArgs = encodeArgs[1:]
		}
	}

	bwmode := false
	corners := false
	glyphs := minSize > 0
	glyphsOnly := false
	dither := false
	dither2 := false
	sparse := false
	pattern2 := false
	decodeOutPath := ""
	for _, a := range encodeArgs {
		if a == "-dither2" {
			dither2 = true
			continue
		}
		if a == "-pattern2" {
			pattern2 = true
			continue
		}
		if a == "-sparse" {
			sparse = true
			continue
		}
		if a == "-dither" {
			dither = true
			continue
		}
		if a == "-glyphs-only" {
			glyphsOnly = true
			continue
		}
		if a == "-glyphs" {
			glyphs = true
			continue
		}
		if a == "-corners" {
			corners = true
			continue
		}
		if a == "bw" {
			bwmode = true
			continue
		}
		if strings.EqualFold(filepath.Ext(a), ".png") {
			decodeOutPath = a
		}
	}
	if glyphs {
		corners = true
	}
	if minSize > 0 {
		sparse = true
	}

	outPath := base + ".babe"
	if err := encodeToBabe(inputPath, outPath, quality, bwmode, corners, dither, sparse, pattern2, minSize); err != nil {
		fmt.Fprintln(os.Stderr, "encode error:", err)
		os.Exit(1)
	}
	if decodeOutPath != "" {
		if err := decodeBabe(outPath, decodeOutPath, false, false, corners, glyphs, glyphsOnly, dither2, minSize); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
	}
}

func parseMinBlockSize(args []string) ([]string, int, error) {
	remaining := make([]string, 0, len(args))
	minSize := 0
	for i := 0; i < len(args); i++ {
		if args[i] != "-s" {
			remaining = append(remaining, args[i])
			continue
		}
		if minSize != 0 || i+1 >= len(args) {
			return nil, 0, fmt.Errorf("-s requires one positive integer")
		}
		i++
		value, err := strconv.Atoi(args[i])
		if err != nil || value < 1 || value > 65535 {
			return nil, 0, fmt.Errorf("-s requires a positive integer between 1 and 65535")
		}
		minSize = value
	}
	return remaining, minSize, nil
}

func encodeToBabe(inPath, outPath string, quality int, bwmode, corners, dither, sparse, pattern2 bool, minSize int) error {
	info, err := os.Stat(inPath)
	if err != nil {
		return err
	}
	inSize := info.Size()

	in, err := os.Open(inPath)
	if err != nil {
		return err
	}
	defer in.Close()

	img, _, err := image.Decode(in)
	if err != nil {
		return err
	}

	start := time.Now()
	encoder := NewEncoder()
	encoder.CornerData = corners
	encoder.DitherPatterns = dither
	encoder.SparseCorners = sparse
	encoder.SimplifyPatterns = pattern2
	encoder.CornerMinSize = minSize
	enc, err := encoder.Encode(img, quality, bwmode)
	if err != nil {
		return err
	}
	finish := time.Since(start)

	out, err := os.Create(outPath)
	if err != nil {
		return err
	}
	defer out.Close()

	if _, err := out.Write(enc); err != nil {
		return err
	}

	encSize := int64(len(enc))
	ratio := float64(encSize) / float64(inSize)

	formatSize := func(size int64) string {
		if size < 1024*1024 {
			return fmt.Sprintf("%.2f KB", float64(size)/1024)
		}
		return fmt.Sprintf("%.2f MB", float64(size)/(1024*1024))
	}

	fmt.Printf("%s (%s) → %s (%s)\n",
		inPath,
		formatSize(inSize),
		outPath,
		formatSize(encSize),
	)
	fmt.Printf("quality=%d, ratio=%.3f, time=%s\n",
		quality,
		ratio,
		finish,
	)

	return nil
}

func decodeBabe(inPath, outPath string, splitChannels, postfilter, corners, glyphs, glyphsOnly, dither2 bool, minSize int) error {
	pngEncoder := png.Encoder{CompressionLevel: png.DefaultCompression}

	in, err := os.Open(inPath)
	if err != nil {
		return err
	}
	defer in.Close()

	compData, err := io.ReadAll(in)
	if err != nil {
		return err
	}

	compSize := len(compData)

	start := time.Now()
	decoder := NewDecoder()
	decoder.CornerGradient = corners
	decoder.GlyphBlend = glyphs
	decoder.GlyphsOnly = glyphsOnly
	decoder.Dither2 = dither2
	decoder.GlyphMinSize = minSize
	dec, err := decoder.Decode(compData, postfilter)
	if err != nil {
		return err
	}
	finish := time.Since(start)

	formatSize := func(size int64) string {
		if size < 1024*1024 {
			return fmt.Sprintf("%.2f KB", float64(size)/1024)
		}
		return fmt.Sprintf("%.2f MB", float64(size)/(1024*1024))
	}

	if !splitChannels {
		out, err := os.Create(outPath)
		if err != nil {
			return err
		}
		defer out.Close()

		if err := pngEncoder.Encode(out, dec); err != nil {
			return err
		}

		info, err := out.Stat()
		if err != nil {
			return err
		}
		outSize := info.Size()
		ratio := float64(outSize) / float64(compSize)

		fmt.Printf("%s (%s) → %s (%s)\n",
			inPath,
			formatSize(int64(compSize)),
			outPath,
			formatSize(outSize),
		)
		fmt.Printf("ratio=%.3f, time=%s\n",
			ratio,
			finish,
		)

		return nil
	}

	base := strings.TrimSuffix(outPath, filepath.Ext(outPath))
	bounds := dec.Bounds()
	yImg := image.NewGray(bounds)
	cbImg := image.NewGray(bounds)
	crImg := image.NewGray(bounds)

	for y := bounds.Min.Y; y < bounds.Max.Y; y++ {
		for x := bounds.Min.X; x < bounds.Max.X; x++ {
			c := dec.At(x, y)
			yc := color.YCbCrModel.Convert(c).(color.YCbCr)
			yImg.SetGray(x, y, color.Gray{Y: yc.Y})
			cbImg.SetGray(x, y, color.Gray{Y: yc.Cb})
			crImg.SetGray(x, y, color.Gray{Y: yc.Cr})
		}
	}

	yFile, err := os.Create(base + "_Y.png")
	if err != nil {
		return err
	}
	if err := pngEncoder.Encode(yFile, yImg); err != nil {
		yFile.Close()
		return err
	}
	yFile.Close()

	cbFile, err := os.Create(base + "_Cb.png")
	if err != nil {
		return err
	}
	if err := pngEncoder.Encode(cbFile, cbImg); err != nil {
		cbFile.Close()
		return err
	}
	cbFile.Close()

	crFile, err := os.Create(base + "_Cr.png")
	if err != nil {
		return err
	}
	if err := pngEncoder.Encode(crFile, crImg); err != nil {
		crFile.Close()
		return err
	}
	crFile.Close()

	var totalOutSize int64

	info, err := os.Stat(base + "_Y.png")
	if err != nil {
		return err
	}
	totalOutSize += info.Size()

	info, err = os.Stat(base + "_Cb.png")
	if err != nil {
		return err
	}
	totalOutSize += info.Size()

	info, err = os.Stat(base + "_Cr.png")
	if err != nil {
		return err
	}
	totalOutSize += info.Size()

	ratio := float64(totalOutSize) / float64(compSize)

	fmt.Printf("%s (%s) → %s_Y.png,%s_Cb.png,%s_Cr.png (total %s)\n",
		inPath,
		formatSize(int64(compSize)),
		base,
		base,
		base,
		formatSize(totalOutSize),
	)
	fmt.Printf("ratio=%.3f, time=%s\n",
		ratio,
		finish,
	)

	return nil
}
