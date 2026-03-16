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

func writePatternSheets(basePath string) error {
	if len(lastPatternSizesUsed) == 0 {
		return nil
	}

	const (
		cellScale = 8
		padding   = 4
		cols      = 8
	)

	for size := range lastPatternSizesUsed {
		bw, bh := size[0], size[1]
		book := fixedPatternCodebook(bw, bh, activePatternCount)
		rows := (len(book) + cols - 1) / cols
		cellW := bw*cellScale + padding*2
		cellH := bh*cellScale + padding*2
		img := image.NewRGBA(image.Rect(0, 0, cols*cellW, rows*cellH))

		bg := color.RGBA{245, 245, 245, 255}
		cellBG := color.RGBA{225, 225, 225, 255}
		fg := color.RGBA{20, 20, 20, 255}
		grid := color.RGBA{120, 120, 120, 255}
		for y := 0; y < img.Bounds().Dy(); y++ {
			for x := 0; x < img.Bounds().Dx(); x++ {
				img.Set(x, y, bg)
			}
		}

		for i, bits := range book {
			col := i % cols
			row := i / cols
			ox := col * cellW
			oy := row * cellH

			for y := oy; y < oy+cellH; y++ {
				for x := ox; x < ox+cellW; x++ {
					img.Set(x, y, cellBG)
				}
			}
			for x := ox; x < ox+cellW; x++ {
				img.Set(x, oy, grid)
				img.Set(x, oy+cellH-1, grid)
			}
			for y := oy; y < oy+cellH; y++ {
				img.Set(ox, y, grid)
				img.Set(ox+cellW-1, y, grid)
			}

			for py := 0; py < bh; py++ {
				for px := 0; px < bw; px++ {
					shift := bw*bh - 1 - (py*bw + px)
					c := color.Color(color.RGBA{250, 250, 250, 255})
					if ((bits >> shift) & 1) != 0 {
						c = fg
					}
					for sy := 0; sy < cellScale; sy++ {
						for sx := 0; sx < cellScale; sx++ {
							img.Set(ox+padding+px*cellScale+sx, oy+padding+py*cellScale+sy, c)
						}
					}
				}
			}
		}

		outPath := fmt.Sprintf("%s.patterns.%dx%d.png", basePath, bw, bh)
		f, err := os.Create(outPath)
		if err != nil {
			return err
		}
		if err := png.Encode(f, img); err != nil {
			_ = f.Close()
			return err
		}
		if err := f.Close(); err != nil {
			return err
		}
		fmt.Fprintf(os.Stderr, "[log] wrote %s\n", outPath)
	}

	return nil
}

func main() {
	if len(os.Args) < 2 || len(os.Args) > 7 {
		fmt.Fprint(os.Stderr, "Usage:\n  babe <input-image> [quality] [bw] [decoded.png] [-patterns=N] [-pattern-index=per-channel|shared] [-log]\n  babe <input.babe> [-postfilter]\n  (bw flag, decoded.png, -patterns=N, -pattern-index=... and -log can appear anywhere after quality)\n")
		os.Exit(1)
	}

	inputPath := os.Args[1]
	ext := strings.ToLower(filepath.Ext(inputPath))
	base := strings.TrimSuffix(inputPath, filepath.Ext(inputPath))

	// If input is .babe → decode to PNG
	if ext == ".babe" {
		postfilter := false
		for _, a := range os.Args[2:] {
			if a == "-postfilter" {
				postfilter = true
				break
			}
		}
		if err := decodeBabe(inputPath, base+".png", false, postfilter); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
		return
	}

	// Otherwise: encode image → .babe with default or provided quality
	quality := 70
	encodeArgs := os.Args[2:]
	if len(os.Args) >= 3 {
		q, err := strconv.Atoi(os.Args[2])
		if err != nil {
			fmt.Fprintln(os.Stderr, "quality must be an integer between 0 and 100")
			os.Exit(1)
		}
		if q < 0 || q > 100 {
			fmt.Fprintln(os.Stderr, "quality must be between 0 and 100")
			os.Exit(1)
		}
		quality = q
		encodeArgs = os.Args[3:]
	}

	bwmode := false
	decodeOutPath := ""
	patternCount := defaultPatternCount
	patternIndexMode := "per-channel"
	logPatterns := false
	for _, a := range encodeArgs {
		if a == "bw" {
			bwmode = true
			continue
		}
		if a == "-log" {
			logPatterns = true
			continue
		}
		if strings.HasPrefix(a, "-patterns=") {
			v, err := strconv.Atoi(strings.TrimPrefix(a, "-patterns="))
			if err != nil || v < 1 || v > 1024 {
				fmt.Fprintln(os.Stderr, "patterns must be an integer between 1 and 1024")
				os.Exit(1)
			}
			patternCount = v
			continue
		}
		if strings.HasPrefix(a, "-pattern-index=") {
			v := strings.TrimPrefix(a, "-pattern-index=")
			if v != "per-channel" && v != "shared" {
				fmt.Fprintln(os.Stderr, "pattern-index must be per-channel or shared")
				os.Exit(1)
			}
			patternIndexMode = v
			continue
		}
		if strings.EqualFold(filepath.Ext(a), ".png") {
			decodeOutPath = a
		}
	}

	outPath := base + ".babe"
	if err := encodeToBabe(inputPath, outPath, quality, bwmode, patternCount, patternIndexMode, logPatterns); err != nil {
		fmt.Fprintln(os.Stderr, "encode error:", err)
		os.Exit(1)
	}
	if decodeOutPath != "" {
		if err := decodeBabe(outPath, decodeOutPath, false, false); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
	}
}

func encodeToBabe(inPath, outPath string, quality int, bwmode bool, patternCount int, patternIndexMode string, logPatterns bool) error {
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
	activePatternCount = patternCount
	activeSharedPatternIndexes = patternIndexMode == "shared"
	encodeLog = logPatterns
	defer func() {
		activePatternCount = defaultPatternCount
		activeSharedPatternIndexes = false
		encodeLog = false
	}()
	enc, err := Encode(img, quality, bwmode)
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
	if logPatterns {
		base := strings.TrimSuffix(outPath, filepath.Ext(outPath))
		if err := writePatternSheets(base); err != nil {
			return err
		}
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
	fmt.Printf("quality=%d, patterns=%d, pattern-index=%s, ratio=%.3f, time=%s\n",
		quality,
		patternCount,
		patternIndexMode,
		ratio,
		finish,
	)

	return nil
}

func decodeBabe(inPath, outPath string, splitChannels, postfilter bool) error {

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
	dec, err := Decode(compData, postfilter)
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

		if err := png.Encode(out, dec); err != nil {
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
	if err := png.Encode(yFile, yImg); err != nil {
		yFile.Close()
		return err
	}
	yFile.Close()

	cbFile, err := os.Create(base + "_Cb.png")
	if err != nil {
		return err
	}
	if err := png.Encode(cbFile, cbImg); err != nil {
		cbFile.Close()
		return err
	}
	cbFile.Close()

	crFile, err := os.Create(base + "_Cr.png")
	if err != nil {
		return err
	}
	if err := png.Encode(crFile, crImg); err != nil {
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
