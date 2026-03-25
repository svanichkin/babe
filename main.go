package main

import (
	"fmt"
	"image"
	"image/color"
	"image/draw"
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
	if len(os.Args) < 2 || len(os.Args) > 16 {
		fmt.Fprint(os.Stderr, "Usage:\n  babe <input-image> [quality] [bw] [decoded.png] [-patterns=N] [-blocks=A,B|A-B] [-spreads=S1,S2,...] [-color-quant=N] [-pattern-set=basic] [-pattern-index=per-channel|shared] [-tile N]\n  babe <input.babe> [-postfilter] [-layers]\n  (bw flag, decoded.png, -patterns=N, -blocks=A,B|A-B, -spreads=S1,S2,..., -color-quant=N, -pattern-set=basic, -pattern-index=..., -tile N can appear anywhere after quality)\n")
		os.Exit(1)
	}

	inputPath := os.Args[1]
	ext := strings.ToLower(filepath.Ext(inputPath))
	base := strings.TrimSuffix(inputPath, filepath.Ext(inputPath))

	// If input is .babe → decode to PNG
	if ext == ".babe" {
		postfilter := false
		splitChannels := false
		for _, a := range os.Args[2:] {
			if a == "-postfilter" {
				postfilter = true
				continue
			}
			if a == "-layers" {
				splitChannels = true
			}
		}
		if err := decodeBabe(inputPath, base+".png", splitChannels, postfilter); err != nil {
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
	layersOut := false
	patternCount := defaultPatternCount
	blockSpec := ""
	spreadSpec := ""
	colorQuantShift := 0
	patternIndexMode := "per-channel"
	tile := 0
	for i := 0; i < len(encodeArgs); i++ {
		a := encodeArgs[i]
		if a == "bw" {
			bwmode = true
			continue
		}
		if a == "-log" || a == "-sweep" {
			fmt.Fprintln(os.Stderr, "flag is not supported in this build")
			os.Exit(1)
		}
		if a == "-layers" {
			layersOut = true
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
		if strings.HasPrefix(a, "-blocks=") {
			blockSpec = strings.TrimPrefix(a, "-blocks=")
			continue
		}
		if strings.HasPrefix(a, "-spreads=") {
			spreadSpec = strings.TrimPrefix(a, "-spreads=")
			continue
		}
		if strings.HasPrefix(a, "-quality-range=") || strings.HasPrefix(a, "-spread-range=") || strings.HasPrefix(a, "-csv=") {
			fmt.Fprintln(os.Stderr, "sweep flags are not supported in this build")
			os.Exit(1)
		}
		if strings.HasPrefix(a, "-color-quant=") {
			v, err := strconv.Atoi(strings.TrimPrefix(a, "-color-quant="))
			if err != nil || v < 0 || v > 7 {
				fmt.Fprintln(os.Stderr, "color-quant must be an integer between 0 and 7")
				os.Exit(1)
			}
			colorQuantShift = v
			continue
		}
		if strings.HasPrefix(a, "-pattern-set=") {
			v := strings.TrimPrefix(a, "-pattern-set=")
			if v != patternSetBasic {
				fmt.Fprintf(os.Stderr, "pattern-set must be %s\n", patternSetBasic)
				os.Exit(1)
			}
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
		if a == "-tile" {
			if i+1 >= len(encodeArgs) {
				fmt.Fprintln(os.Stderr, "tile requires a value between 2 and 255")
				os.Exit(1)
			}
			v, err := strconv.Atoi(encodeArgs[i+1])
			if err != nil || v < 2 || v > 255 {
				fmt.Fprintln(os.Stderr, "tile must be an integer between 2 and 255")
				os.Exit(1)
			}
			tile = v
			i++
			continue
		}
		if strings.HasPrefix(a, "-tile=") {
			v, err := strconv.Atoi(strings.TrimPrefix(a, "-tile="))
			if err != nil || v < 2 || v > 255 {
				fmt.Fprintln(os.Stderr, "tile must be an integer between 2 and 255")
				os.Exit(1)
			}
			tile = v
			continue
		}
		if strings.EqualFold(filepath.Ext(a), ".png") {
			decodeOutPath = a
		}
	}

	outPath := base + ".babe"
	if blockSpec != "" {
		if err := setBlocksFromSpec(blockSpec); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
	}
	if spreadSpec != "" {
		if err := setSpreadFactorsFromSpec(spreadSpec); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
	}
	defer func() {
		forcedBlockSizes = nil
		forcedSpreadFactors = nil
	}()
	if err := encodeToBabe(inputPath, outPath, quality, bwmode, patternCount, colorQuantShift, patternIndexMode, tile); err != nil {
		fmt.Fprintln(os.Stderr, "encode error:", err)
		os.Exit(1)
	}
	if decodeOutPath != "" {
		if err := decodeBabe(outPath, decodeOutPath, false, false); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
	}
	if layersOut {
		if err := decodeBabe(outPath, base+".png", true, false); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
	}
}

func encodeToBabe(inPath, outPath string, quality int, bwmode bool, patternCount int, colorQuantShift int, patternIndexMode string, tile int) error {
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
	activeColorQuantShift = colorQuantShift
	activeSharedPatternIndexes = patternIndexMode == "shared"
	activeBackgroundTile = tile
	defer func() {
		activePatternCount = defaultPatternCount
		activeColorQuantShift = 0
		activeSharedPatternIndexes = false
		activeBackgroundTile = 0
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
	fmt.Printf("ratio=%.3f, time=%s\n", ratio, finish)

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
	imgW, imgH, yPlane, yRects, cbPlane, cbRects, crPlane, crRects, hasCb, hasCr, err := DecodeLayers(compData)
	if err != nil {
		return err
	}

	makeLayer := func(plane []uint8, rects []image.Rectangle, fallback uint8) *image.RGBA {
		img := image.NewRGBA(image.Rect(0, 0, imgW, imgH))
		gray := image.NewGray(img.Bounds())
		if len(plane) == 0 {
			for i := range gray.Pix {
				gray.Pix[i] = fallback
			}
		} else {
			copy(gray.Pix, plane)
		}
		draw.Draw(img, img.Bounds(), gray, image.Point{}, draw.Src)
		green := color.RGBA{0, 255, 0, 255}
		for _, r := range rects {
			for x := r.Min.X; x < r.Max.X; x++ {
				if r.Min.Y >= 0 && r.Min.Y < imgH {
					img.Set(x, r.Min.Y, green)
				}
				if r.Max.Y-1 >= 0 && r.Max.Y-1 < imgH {
					img.Set(x, r.Max.Y-1, green)
				}
			}
			for y := r.Min.Y; y < r.Max.Y; y++ {
				if r.Min.X >= 0 && r.Min.X < imgW {
					img.Set(r.Min.X, y, green)
				}
				if r.Max.X-1 >= 0 && r.Max.X-1 < imgW {
					img.Set(r.Max.X-1, y, green)
				}
			}
		}
		return img
	}

	yImg := makeLayer(yPlane, yRects, 0)
	cbImg := makeLayer(cbPlane, cbRects, func() uint8 {
		if hasCb {
			return 0
		}
		return 128
	}())
	crImg := makeLayer(crPlane, crRects, func() uint8 {
		if hasCr {
			return 0
		}
		return 128
	}())

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
