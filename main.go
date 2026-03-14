package main

import (
	"encoding/binary"
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
	if len(os.Args) < 2 || len(os.Args) > 16 {
		fmt.Fprint(os.Stderr, "Usage:\n  babe <input-image> [quality] [pattern] [decoded.png] [-z] [shuffle]\n  babe <input-image> [quality] [bright] [decoded.png] [-z] [shuffle]\n  babe <input-image> [quality] [adaptive N] [decoded.png] [-z] [-tile N] [-block] [-raw] [-top16] [-shift N] [-tree] [-treeadapt] [-reconstruct] [shuffle]\n  babe <input-image> [quality] [zx|cga|ega|vga|c64|gameboy|pico8|db16|nes|sunset|pastel|ocean|forest|<palette-spec> ...] [decoded.png] [-z] [-tile N] [-block] [-raw] [-top16] [-shift N] [-tree] [-treeadapt] [-reconstruct] [shuffle]\n  babe <input.babe> [-postfilter]\n")
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
	}

	useZstd := false
	useShuffle := false
	debugLog := false
	rawPalette := false
	reconstructPalette := false
	blockSubset := false
	zxMode := false
	tileSize := 0
	blockSize := 0
	rawTop16 := false
	rawTree := false
	rawTreeAdapt := false
	rawShift := -1
	paletteName := ""
	paletteSpecs := make([]string, 0, 4)
	explicitYCB := false
	decodeOutPath := ""
	yBits, cbBits, crBits := 1, 0, 0
	pattern := ""
	if len(os.Args) >= 4 {
		args := make([]string, 0, len(os.Args[3:]))
		rawArgs := os.Args[3:]
		for i := 0; i < len(rawArgs); i++ {
			a := rawArgs[i]
			if a == "-z" || a == "-zstd" {
				useZstd = true
				continue
			}
			if a == "-log" {
				debugLog = true
				continue
			}
			if a == "-raw" {
				rawPalette = true
				continue
			}
			if a == "-reconstruct" {
				reconstructPalette = true
				continue
			}
			if a == "-tile" {
				if i+1 >= len(rawArgs) {
					fmt.Fprintln(os.Stderr, "-tile requires a size, for example: -tile 16")
					os.Exit(1)
				}
				n, err := strconv.Atoi(rawArgs[i+1])
				if err != nil || n < 2 || n > 255 {
					fmt.Fprintln(os.Stderr, "tile size must be an integer between 2 and 255")
					os.Exit(1)
				}
				tileSize = n
				i++
				continue
			}
			if a == "-block" {
				blockSize = 16
				blockSubset = true
				rawPalette = true
				continue
			}
			if a == "-top16" {
				rawTop16 = true
				rawPalette = true
				continue
			}
			if a == "-tree" {
				rawTree = true
				rawPalette = true
				continue
			}
			if a == "-treeadapt" {
				rawTreeAdapt = true
				rawPalette = true
				continue
			}
			if a == "-shift" {
				if i+1 >= len(rawArgs) {
					fmt.Fprintln(os.Stderr, "-shift requires a value between 0 and 7")
					os.Exit(1)
				}
				n, err := strconv.Atoi(rawArgs[i+1])
				if err != nil || n < 0 || n > 7 {
					fmt.Fprintln(os.Stderr, "shift must be an integer between 0 and 7")
					os.Exit(1)
				}
				rawShift = n
				i++
				continue
			}
			if strings.EqualFold(a, "shuffle") {
				useShuffle = true
				continue
			}
			if strings.EqualFold(filepath.Ext(a), ".png") {
				decodeOutPath = a
				continue
			}
			if isPatternArg(a) {
				pattern = normalizePatternArg(a)
				continue
			}
			if strings.EqualFold(a, "zx") {
				zxMode = true
				continue
			}
			if strings.EqualFold(a, "adaptive") {
				if i+1 >= len(rawArgs) {
					fmt.Fprintln(os.Stderr, "adaptive mode requires a palette size, for example: adaptive 16 or adaptive auto [0..100]")
					os.Exit(1)
				}
				if strings.EqualFold(rawArgs[i+1], "auto") {
					paletteName = "adaptive:auto"
					if i+2 < len(rawArgs) {
						if pct, err := strconv.Atoi(rawArgs[i+2]); err == nil {
							if pct < 0 || pct > 100 {
								fmt.Fprintln(os.Stderr, "adaptive auto quality must be an integer between 0 and 100")
								os.Exit(1)
							}
							paletteName = fmt.Sprintf("adaptive:auto:%d", pct)
							i += 2
							continue
						}
					}
					i++
					continue
				}
				n, err := strconv.Atoi(rawArgs[i+1])
				if err != nil || n < 2 || n > 256 {
					fmt.Fprintln(os.Stderr, "adaptive palette size must be an integer between 2 and 256, or 'auto'")
					os.Exit(1)
				}
				paletteName = fmt.Sprintf("adaptive:%d", n)
				i++
				continue
			}
			switch strings.ToLower(a) {
			case "sunset", "pastel", "ocean", "forest", "cga", "ega", "vga", "c64", "gameboy", "pico8", "db16", "nes":
				paletteName = strings.ToLower(a)
				continue
			}
			if isPaletteSpec(a) {
				paletteSpecs = append(paletteSpecs, strings.ToLower(a))
				continue
			}
			if strings.EqualFold(a, "bright") || strings.EqualFold(a, "ycb") {
				explicitYCB = true
				continue
			}
			args = append(args, a)
		}
		if len(paletteSpecs) > 0 {
			paletteName = strings.Join(paletteSpecs, "+")
		}
		if zxMode || paletteName != "" || explicitYCB {
			yBits, cbBits, crBits = 1, 1, 1
		} else {
			if len(args) > 0 {
				var err error
				yBits, cbBits, crBits, pattern, err = parseBitDepthArgs(args)
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					os.Exit(1)
				}
			}
		}
	}
	bwmode := cbBits == 0 && crBits == 0
	bwBits := yBits

	outPath := base + ".babe"
	if err := encodeToBabe(inputPath, outPath, quality, bwmode, bwBits, yBits, cbBits, crBits, pattern, tileSize, blockSize, useZstd, useShuffle, false, zxMode, paletteName, rawPalette, rawTop16, rawTree, rawTreeAdapt, rawShift, reconstructPalette, blockSubset); err != nil {
		fmt.Fprintln(os.Stderr, "encode error:", err)
		os.Exit(1)
	}
	if decodeOutPath != "" {
		if err := decodeBabe(outPath, decodeOutPath, false, false); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
	}
	if debugLog {
		if err := writePatternLogPNG(outPath, base+".log.png"); err != nil {
			fmt.Fprintln(os.Stderr, "log error:", err)
			os.Exit(1)
		}
	}
}

func parseBitDepthArgs(args []string) (int, int, int, string, error) {
	pattern := ""
	if len(args) > 0 && isPatternArg(args[len(args)-1]) {
		pattern = normalizePatternArg(args[len(args)-1])
		args = args[:len(args)-1]
	}

	switch len(args) {
	case 1:
		if strings.Contains(args[0], ":") {
			parts := strings.Split(args[0], ":")
			if len(parts) != 3 {
				return 0, 0, 0, "", fmt.Errorf("bit depth triplet must look like y:cb:cr, for example 4:4:4")
			}
			y, err := parseBitDepth(parts[0], "y")
			if err != nil {
				return 0, 0, 0, "", err
			}
			cb, err := parseBitDepth(parts[1], "cb")
			if err != nil {
				return 0, 0, 0, "", err
			}
			cr, err := parseBitDepth(parts[2], "cr")
			if err != nil {
				return 0, 0, 0, "", err
			}
			return y, cb, cr, pattern, nil
		}
		bw, err := parseBitDepth(args[0], "bw")
		if err != nil {
			return 0, 0, 0, "", err
		}
		return bw, 0, 0, pattern, nil
	case 3:
		y, err := parseBitDepth(args[0], "y")
		if err != nil {
			return 0, 0, 0, "", err
		}
		cb, err := parseBitDepth(args[1], "cb")
		if err != nil {
			return 0, 0, 0, "", err
		}
		cr, err := parseBitDepth(args[2], "cr")
		if err != nil {
			return 0, 0, 0, "", err
		}
		return y, cb, cr, pattern, nil
	default:
		return 0, 0, 0, "", fmt.Errorf("expected either one bw bit depth or a color triplet")
	}
}

func isPatternArg(s string) bool {
	return strings.ContainsRune(s, 'x') || strings.ContainsRune(s, 'х') || strings.ContainsRune(s, 'X') || strings.ContainsRune(s, 'Х')
}

func normalizePatternArg(s string) string {
	r := strings.NewReplacer("X", "x", "Х", "x", "х", "x")
	return strings.ToLower(r.Replace(s))
}

func parsePatternDims(s string) (int, int, error) {
	s = normalizePatternArg(s)
	parts := strings.Split(s, "x")
	if len(parts) != 2 {
		return 0, 0, fmt.Errorf("pattern must look like WxH, for example 2x4")
	}
	w, err := strconv.Atoi(parts[0])
	if err != nil || w < 1 {
		return 0, 0, fmt.Errorf("invalid pattern width %q", parts[0])
	}
	h, err := strconv.Atoi(parts[1])
	if err != nil || h < 1 {
		return 0, 0, fmt.Errorf("invalid pattern height %q", parts[1])
	}
	return w, h, nil
}

func isPaletteSpec(s string) bool {
	s = strings.ToLower(strings.TrimSpace(s))
	if s == "" {
		return false
	}
	for len(s) > 0 {
		if strings.HasPrefix(s, "bw") || strings.HasPrefix(s, "wb") {
			s = s[2:]
			continue
		}
		switch s[0] {
		case 'y', 'c', 'm', 'r', 'g', 'b', 'w', 'k':
			s = s[1:]
		default:
			return false
		}
	}
	return true
}

func parseBitDepth(s, label string) (int, error) {
	v, err := strconv.Atoi(s)
	if err != nil || v < 0 || v > 4 {
		if label == "bw" {
			return 0, fmt.Errorf("bw bit depth must be an integer between 1 and 4")
		}
		return 0, fmt.Errorf("%s bit depth must be an integer between 0 and 4", label)
	}
	if label == "bw" && v == 0 {
		return 0, fmt.Errorf("bw bit depth must be an integer between 1 and 4")
	}
	if label == "y" && v == 0 {
		return 0, fmt.Errorf("y bit depth must be an integer between 1 and 4")
	}
	return v, nil
}

func encodeToBabe(inPath, outPath string, quality int, bwmode bool, bwBits, yBits, cbBits, crBits int, pattern string, tileSize, blockSize int, useZstd, useShuffle bool, rgbMode, zxMode bool, paletteName string, rawPalette, rawTop16, rawTree, rawTreeAdapt bool, rawShift int, reconstructPalette, blockSubset bool) error {
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
	paletteLabel := paletteName
	if strings.HasPrefix(strings.ToLower(paletteName), "adaptive:auto") {
		b := img.Bounds()
		w := b.Dx()
		h := b.Dy()
		rPlane := make([]uint8, w*h)
		gPlane := make([]uint8, w*h)
		bPlane := make([]uint8, w*h)
		extractRGBPlanes(img, rPlane, gPlane, bPlane, true)
		size := chooseAdaptivePaletteSizeForName(paletteName, rPlane, gPlane, bPlane, w, h)
		if pct := adaptiveAutoPercent(paletteName); pct >= 0 && pct < 100 {
			paletteLabel = fmt.Sprintf("adaptive:auto:%d(%d)", pct, size)
		} else {
			paletteLabel = fmt.Sprintf("adaptive:auto(%d)", size)
		}
	}

	start := time.Now()
	var enc []byte
	if bwmode {
		enc, err = NewEncoder().EncodeWithOptions(img, quality, EncodeOptions{
			BW:       true,
			YBits:    bwBits,
			Pattern:  pattern,
			TileSize: tileSize,
			UseZstd:  useZstd,
			Shuffle:  useShuffle,
		})
	} else {
		enc, err = NewEncoder().EncodeWithOptions(img, quality, EncodeOptions{
			BW:                 false,
			YBits:              yBits,
			CbBits:             cbBits,
			CrBits:             crBits,
			Pattern:            pattern,
			TileSize:           tileSize,
			BlockSize:          blockSize,
			UseZstd:            useZstd,
			Shuffle:            useShuffle,
			RGBMode:            false,
			ZXMode:             zxMode,
			Palette:            paletteName,
			RawPalette:         rawPalette,
			RawTop16:           rawTop16,
			RawTree:            rawTree,
			RawTreeAdapt:       rawTreeAdapt,
			RawShift:           rawShift,
			ReconstructPalette: reconstructPalette,
			BlockSubset:        blockSubset,
		})
	}
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

	patternCount := 0
	patternLabel := ""
	if bwmode && bwBits == 1 {
		if w, h, count, err := inspectBWPatternPalette(enc); err == nil && count > 0 {
			patternLabel = fmt.Sprintf("%dx%d", w, h)
			patternCount = count
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
	if bwmode {
		if patternCount > 0 {
			fmt.Printf("quality=%d, bw_bits=%d, pattern=%s, palette=%d, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, bwBits, patternLabel, patternCount, useZstd, useShuffle, ratio, finish)
		} else {
			fmt.Printf("quality=%d, bw_bits=%d, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, bwBits, useZstd, useShuffle, ratio, finish)
		}
	} else {
		if paletteName != "" {
			if tileSize > 0 {
				fmt.Printf("quality=%d, mode=%s, tile=%d, block=%d, subset=%t, raw=%t, top16=%t, tree=%t, treeadapt=%t, shift=%d, reconstruct=%t, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, paletteLabel, tileSize, blockSize, blockSubset, rawPalette, rawTop16, rawTree, rawTreeAdapt, rawShift, reconstructPalette, useZstd, useShuffle, ratio, finish)
			} else {
				fmt.Printf("quality=%d, mode=%s, block=%d, subset=%t, raw=%t, top16=%t, tree=%t, treeadapt=%t, shift=%d, reconstruct=%t, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, paletteLabel, blockSize, blockSubset, rawPalette, rawTop16, rawTree, rawTreeAdapt, rawShift, reconstructPalette, useZstd, useShuffle, ratio, finish)
			}
		} else if zxMode {
			if tileSize > 0 {
				fmt.Printf("quality=%d, mode=zx, tile=%d, block=%d, subset=%t, raw=%t, top16=%t, tree=%t, treeadapt=%t, shift=%d, reconstruct=%t, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, tileSize, blockSize, blockSubset, rawPalette, rawTop16, rawTree, rawTreeAdapt, rawShift, reconstructPalette, useZstd, useShuffle, ratio, finish)
			} else {
				fmt.Printf("quality=%d, mode=zx, block=%d, subset=%t, raw=%t, top16=%t, tree=%t, treeadapt=%t, shift=%d, reconstruct=%t, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, blockSize, blockSubset, rawPalette, rawTop16, rawTree, rawTreeAdapt, rawShift, reconstructPalette, useZstd, useShuffle, ratio, finish)
			}
		} else if yBits == 1 && cbBits == 1 && crBits == 1 {
			fmt.Printf("quality=%d, mode=ycb, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, useZstd, useShuffle, ratio, finish)
		} else {
			fmt.Printf("quality=%d, y_bits=%d, cb_bits=%d, cr_bits=%d, zstd=%t, shuffle=%t, ratio=%.3f, time=%s\n", quality, yBits, cbBits, crBits, useZstd, useShuffle, ratio, finish)
		}
	}

	return nil
}

func inspectBWPatternPalette(comp []byte) (int, int, int, error) {
	payload := comp
	if len(comp) < len(codec) || string(comp[:len(codec)]) != codec {
		dec := mustNewZstdDecoder()
		var err error
		payload, err = dec.DecodeAll(comp, nil)
		if err != nil {
			return 0, 0, 0, err
		}
	}
	pos := len(codec)
	if len(payload) < pos+8 {
		return 0, 0, 0, fmt.Errorf("short payload")
	}
	pos++ // quality
	pos++ // color mode
	yBits := int(payload[pos])
	pos++
	pos++ // cb bits
	pos++ // cr bits
	yStorageMode := payload[pos]
	pos++
	patternW := int(payload[pos])
	pos++
	patternH := int(payload[pos])
	pos++
	channelsMask := payload[pos]
	pos++
	if yBits != 1 || (yStorageMode != yStoragePatternGrid && yStorageMode != yStoragePatternGridDirect) || channelsMask != channelFlagY {
		return 0, 0, 0, nil
	}
	if _, err := readU32BE(payload, &pos, "image width"); err != nil {
		return 0, 0, 0, err
	}
	if _, err := readU32BE(payload, &pos, "image height"); err != nil {
		return 0, 0, 0, err
	}
	yData, err := readBitPlane(payload, &pos, "Y")
	if err != nil {
		return 0, 0, 0, err
	}
	if len(yData) < 4 {
		if yStorageMode == yStoragePatternGridDirect {
			return patternW, patternH, 1 << (patternW * patternH), nil
		}
		return 0, 0, 0, fmt.Errorf("short Y pattern payload")
	}
	if yStorageMode == yStoragePatternGridDirect {
		return patternW, patternH, 1 << (patternW * patternH), nil
	}
	paletteSize := int(binary.BigEndian.Uint32(yData[0:4]))
	return patternW, patternH, paletteSize, nil
}

func inspectColorPatternPalette(comp []byte) (int, int, int, error) {
	return 0, 0, 0, nil
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

func writePatternLogPNG(inPath, outPath string) error {
	comp, err := os.ReadFile(inPath)
	if err != nil {
		return err
	}
	payload := comp
	if len(comp) < len(codec) || string(comp[:len(codec)]) != codec {
		dec := mustNewZstdDecoder()
		payload, err = dec.DecodeAll(comp, nil)
		if err != nil {
			return err
		}
	}
	pos := len(codec)
	if len(payload) < pos+8 {
		return fmt.Errorf("short payload")
	}
	pos++ // quality
	colorMode := payload[pos]
	pos++
	yBits := int(payload[pos])
	pos++
	pos++ // cb bits
	pos++ // cr bits
	yStorageMode := payload[pos]
	pos++
	patternW := int(payload[pos])
	pos++
	patternH := int(payload[pos])
	pos++
	channelsMask := payload[pos]
	pos++
	w, err := readU32BE(payload, &pos, "image width")
	if err != nil {
		return err
	}
	h, err := readU32BE(payload, &pos, "image height")
	if err != nil {
		return err
	}
	yData, err := readBitPlane(payload, &pos, "Y")
	if err != nil {
		return err
	}

	var img *image.RGBA
	switch {
	case colorMode == colorModePaletteRGB && yStorageMode == yStorageRaw && channelsMask == channelFlagY:
		img, err = buildReconstructLogImage(yData, w, h)
		if err != nil {
			return err
		}
	case yBits == 1 && (yStorageMode == yStoragePatternGrid || yStorageMode == yStoragePatternGridDirect) && channelsMask == channelFlagY:
		img, err = buildPatternLogImage(yData, w, h, yBits, yStorageMode, patternW, patternH)
		if err != nil {
			return err
		}
	default:
		return fmt.Errorf("log is only available for bw pattern mode or palette reconstruct mode")
	}

	out, err := os.Create(outPath)
	if err != nil {
		return err
	}
	defer out.Close()
	return png.Encode(out, img)
}

func buildReconstructLogImage(yData []byte, imgW, imgH int) (*image.RGBA, error) {
	if len(yData) < 2 {
		return nil, fmt.Errorf("short palette payload")
	}
	paletteSize := int(binary.BigEndian.Uint16(yData[0:2]))
	if paletteSize < 1 || len(yData) < 2+paletteSize*3+1 {
		return nil, fmt.Errorf("short palette header")
	}
	paletteRGB := make([]color.RGBA, paletteSize)
	for i := 0; i < paletteSize; i++ {
		base := 2 + i*3
		paletteRGB[i] = color.RGBA{yData[base], yData[base+1], yData[base+2], 255}
	}
	pos := 2 + paletteSize*3
	submode := int(yData[pos])
	pos++
	if submode != 3 {
		return nil, fmt.Errorf("log is only available for reconstruct palette mode")
	}
	if len(yData)-pos < 2 {
		return nil, fmt.Errorf("truncated reconstruct header")
	}
	usedCount := int(binary.BigEndian.Uint16(yData[pos : pos+2]))
	pos += 2
	if usedCount < 1 || usedCount > paletteSize || len(yData)-pos < usedCount {
		return nil, fmt.Errorf("truncated reconstruct order")
	}
	order := make([]int, usedCount)
	for i := 0; i < usedCount; i++ {
		order[i] = int(yData[pos+i])
	}
	pos += usedCount

	type section struct {
		img *image.RGBA
	}
	sections := make([]section, 0, max(1, usedCount))
	currLen := imgW * imgH
	if currLen < 1 {
		return nil, fmt.Errorf("invalid reconstruct image size")
	}
	for i := 0; i < usedCount-1; i++ {
		planeLen := (currLen + 7) >> 3
		if len(yData)-pos < planeLen {
			return nil, fmt.Errorf("truncated reconstruct plane")
		}
		plane := yData[pos : pos+planeLen]
		pos += planeLen
		sec := renderReconstructMask(plane, currLen, imgW, paletteRGB[order[i]])
		sections = append(sections, section{img: sec})
		ones := countBitsPrefix(plane, currLen)
		currLen -= ones
	}
	if currLen > 0 {
		sections = append(sections, section{img: renderSolidStrip(currLen, imgW, paletteRGB[order[usedCount-1]])})
	}
	if len(sections) == 0 {
		return nil, fmt.Errorf("no reconstruct sections")
	}
	maxW := 0
	totalH := 8
	for _, s := range sections {
		if s.img.Bounds().Dx() > maxW {
			maxW = s.img.Bounds().Dx()
		}
		totalH += s.img.Bounds().Dy() + 8
	}
	dst := image.NewRGBA(image.Rect(0, 0, maxW+16, totalH))
	fillRGBA(dst, color.RGBA{32, 32, 32, 255})
	y := 8
	for _, s := range sections {
		drawAt(dst, s.img, 8, y)
		y += s.img.Bounds().Dy() + 8
	}
	return dst, nil
}

func renderReconstructMask(plane []byte, n, w int, fg color.RGBA) *image.RGBA {
	if w < 1 {
		w = 1
	}
	h := ceilDiv(n, w)
	dst := image.NewRGBA(image.Rect(0, 0, w, h))
	bg := previewContrastBG(fg)
	fillRGBA(dst, bg)
	for i := 0; i < n; i++ {
		if monoBitAt(plane, i) {
			dst.SetRGBA(i%w, i/w, fg)
		}
	}
	return dst
}

func renderSolidStrip(n, w int, fg color.RGBA) *image.RGBA {
	if w < 1 {
		w = 1
	}
	h := ceilDiv(n, w)
	dst := image.NewRGBA(image.Rect(0, 0, w, h))
	bg := previewContrastBG(fg)
	fillRGBA(dst, bg)
	for i := 0; i < n; i++ {
		dst.SetRGBA(i%w, i/w, fg)
	}
	return dst
}

func countBitsPrefix(src []byte, n int) int {
	c := 0
	for i := 0; i < n; i++ {
		if monoBitAt(src, i) {
			c++
		}
	}
	return c
}

func buildPatternLogImage(yData []byte, w, h, bitDepth int, yStorageMode byte, patternW, patternH int) (*image.RGBA, error) {
	if len(yData) < 2 {
		return nil, fmt.Errorf("short palette payload")
	}
	paletteSize := int(binary.BigEndian.Uint16(yData[0:2]))
	if len(yData) < 2+paletteSize*3+1 {
		return nil, fmt.Errorf("short palette header")
	}
	paletteRGB := make([]color.RGBA, paletteSize)
	for i := 0; i < paletteSize; i++ {
		base := 2 + i*3
		paletteRGB[i] = color.RGBA{yData[base], yData[base+1], yData[base+2], 255}
	}
	pos := 2 + paletteSize*3
	canvasIdx := int(yData[pos])
	pos++
	if len(yData)-pos < 2 {
		return nil, fmt.Errorf("short pair header")
	}
	pairMode := yData[pos] != 0
	layerCount := int(yData[pos+1])
	pos += 2
	order := make([]int, 0, layerCount)
	for i := 0; i < paletteSize; i++ {
		if i != canvasIdx {
			order = append(order, i)
		}
	}
	type section struct {
		preview *image.RGBA
		glyphs  *image.RGBA
	}
	sections := make([]section, 0, len(order))
	for oi := 0; oi < len(order); {
		if len(yData)-pos < 4 {
			return nil, fmt.Errorf("truncated plane length")
		}
		n := int(binary.BigEndian.Uint32(yData[pos : pos+4]))
		pos += 4
		if len(yData)-pos < n {
			return nil, fmt.Errorf("truncated plane data")
		}
		planeData := yData[pos : pos+n]
		pos += n
		if pairMode && oi+1 < len(order) {
			var states []uint8
			var err error
			if yStorageMode == yStoragePatternGridDirect {
				states, err = unpackStatePatternGridDirect(planeData, w, h, patternW, patternH, 2)
			} else {
				states, err = unpackStatePatternGrid(planeData, w, h, patternW, patternH, 2)
			}
			if err != nil {
				return nil, err
			}
			cA := paletteRGB[order[oi]]
			cB := paletteRGB[order[oi+1]]
			preview := renderStatePreview(states, w, h, paletteRGB[canvasIdx], cA, cB)
			glyphs, err := renderGlyphSheet(planeData, patternW, patternH, 2, yStorageMode == yStoragePatternGridDirect, preview.Bounds().Dx(), paletteRGB[canvasIdx], cA, cB)
			if err != nil {
				return nil, err
			}
			sections = append(sections, section{preview: preview, glyphs: glyphs})
			oi += 2
		} else {
			var mono []uint8
			var err error
			if yStorageMode == yStoragePatternGridDirect {
				mono, err = unpackMonoPatternGridDirect(planeData, w, h, patternW, patternH, 0, 255)
			} else {
				mono, err = unpackMonoPatternGrid(planeData, w, h, patternW, patternH, 0, 255)
			}
			if err != nil {
				return nil, err
			}
			cA := paletteRGB[order[oi]]
			preview := renderMonoPreview(mono, w, h, paletteRGB[canvasIdx], cA)
			glyphs, err := renderGlyphSheet(planeData, patternW, patternH, 1, yStorageMode == yStoragePatternGridDirect, preview.Bounds().Dx(), paletteRGB[canvasIdx], cA, cA)
			if err != nil {
				return nil, err
			}
			sections = append(sections, section{preview: preview, glyphs: glyphs})
			oi++
		}
	}
	if len(sections) == 0 {
		return nil, fmt.Errorf("no sections")
	}
	totalH := 8
	maxW := 0
	for _, s := range sections {
		rowW := max(s.preview.Bounds().Dx(), s.glyphs.Bounds().Dx())
		if rowW > maxW {
			maxW = rowW
		}
		totalH += s.preview.Bounds().Dy() + 6 + s.glyphs.Bounds().Dy() + 12
	}
	dst := image.NewRGBA(image.Rect(0, 0, maxW+8, totalH))
	fillRGBA(dst, color.RGBA{32, 32, 32, 255})
	y := 8
	for _, s := range sections {
		drawAt(dst, s.preview, 8, y)
		y += s.preview.Bounds().Dy() + 6
		drawAt(dst, s.glyphs, 8, y)
		y += s.glyphs.Bounds().Dy() + 12
	}
	_ = bitDepth
	return dst, nil
}

func renderStatePreview(states []uint8, w, h int, canvas, a, b color.RGBA) *image.RGBA {
	src := image.NewRGBA(image.Rect(0, 0, w, h))
	palette := [4]color.RGBA{canvas, b, a, blendRGBA(a, b)}
	for i, v := range states {
		o := i * 4
		c := palette[int(v)&3]
		src.Pix[o+0], src.Pix[o+1], src.Pix[o+2], src.Pix[o+3] = c.R, c.G, c.B, 255
	}
	return scalePreview(src, 256)
}

func renderMonoPreview(mono []uint8, w, h int, canvas, fg color.RGBA) *image.RGBA {
	src := image.NewRGBA(image.Rect(0, 0, w, h))
	for i, v := range mono {
		o := i * 4
		c := canvas
		if v != 0 {
			c = fg
		}
		src.Pix[o+0], src.Pix[o+1], src.Pix[o+2], src.Pix[o+3] = c.R, c.G, c.B, 255
	}
	return scalePreview(src, 256)
}

func scalePreview(src *image.RGBA, maxW int) *image.RGBA {
	sw := src.Bounds().Dx()
	sh := src.Bounds().Dy()
	if sw <= maxW {
		return src
	}
	dw := maxW
	dh := max(1, sh*dw/sw)
	dst := image.NewRGBA(image.Rect(0, 0, dw, dh))
	for y := 0; y < dh; y++ {
		sy := y * sh / dh
		for x := 0; x < dw; x++ {
			sx := x * sw / dw
			dst.SetRGBA(x, y, src.RGBAAt(sx, sy))
		}
	}
	return dst
}

func renderGlyphSheet(data []byte, patternW, patternH, stateBits int, direct bool, targetWidth int, canvas, a, b color.RGBA) (*image.RGBA, error) {
	var glyphs [][]byte
	blockBytes := (patternW*patternH*stateBits + 7) >> 3
	if direct {
		count := 1 << (patternW * patternH * stateBits)
		if count > 256 {
			count = 256
		}
		glyphs = make([][]byte, 0, count)
		for i := 0; i < count; i++ {
			b := make([]byte, blockBytes)
			for j := range b {
				shift := (len(b) - 1 - j) * 8
				b[j] = byte(i >> shift)
			}
			glyphs = append(glyphs, b)
		}
	} else {
		if len(data) < 10 {
			return nil, fmt.Errorf("short glyph payload")
		}
		count := int(binary.BigEndian.Uint32(data[0:4]))
		blockBytes = int(data[4])
		if count > 256 {
			count = 256
		}
		paletteBytes := count * blockBytes
		if len(data) < 10+paletteBytes {
			return nil, fmt.Errorf("truncated glyph table")
		}
		glyphs = make([][]byte, 0, count)
		pos := 10
		for i := 0; i < count; i++ {
			glyphs = append(glyphs, append([]byte(nil), data[pos:pos+blockBytes]...))
			pos += blockBytes
		}
	}
	cell := 1
	stepX := patternW*cell + 2
	stepY := patternH*cell + 2
	cols := max(1, targetWidth/stepX)
	if cols > len(glyphs) {
		cols = len(glyphs)
	}
	rows := ceilDiv(len(glyphs), cols)
	dst := image.NewRGBA(image.Rect(0, 0, cols*stepX, rows*stepY))
	fillRGBA(dst, previewContrastBG(blendRGBA(a, b)))
	for i, g := range glyphs {
		ox := (i % cols) * stepX
		oy := (i / cols) * stepY
		drawRect(dst, ox, oy, patternW*cell+2, patternH*cell+2, color.RGBA{90, 90, 90, 255})
		br := newBitReader(g)
		for y := 0; y < patternH; y++ {
			for x := 0; x < patternW; x++ {
				v, err := br.readBits(uint8(stateBits))
				if err != nil {
					return nil, err
				}
				c := stateColor(v, stateBits, canvas, a, b)
				dst.SetRGBA(ox+1+x*cell, oy+1+y*cell, c)
			}
		}
	}
	return dst, nil
}

func stateColor(v uint8, stateBits int, canvas, a, b color.RGBA) color.RGBA {
	if stateBits == 1 {
		if v == 0 {
			return canvas
		}
		return a
	}
	switch v & 3 {
	case 1:
		return b
	case 2:
		return a
	case 3:
		return blendRGBA(a, b)
	default:
		return canvas
	}
}

func blendRGBA(a, b color.RGBA) color.RGBA {
	return color.RGBA{
		R: uint8((uint16(a.R) + uint16(b.R)) / 2),
		G: uint8((uint16(a.G) + uint16(b.G)) / 2),
		B: uint8((uint16(a.B) + uint16(b.B)) / 2),
		A: 255,
	}
}

func previewContrastBG(fg color.RGBA) color.RGBA {
	bg := color.RGBA{255 - fg.R, 255 - fg.G, 255 - fg.B, 255}
	if contrastScore(fg, bg) >= 220 {
		return bg
	}
	lum := int(fg.R)*299 + int(fg.G)*587 + int(fg.B)*114
	if lum >= 128000 {
		return color.RGBA{24, 24, 24, 255}
	}
	return color.RGBA{244, 244, 244, 255}
}

func contrastScore(a, b color.RGBA) int {
	dr := int(a.R) - int(b.R)
	if dr < 0 {
		dr = -dr
	}
	dg := int(a.G) - int(b.G)
	if dg < 0 {
		dg = -dg
	}
	db := int(a.B) - int(b.B)
	if db < 0 {
		db = -db
	}
	return dr + dg + db
}

func fillRGBA(img *image.RGBA, c color.RGBA) {
	for y := 0; y < img.Bounds().Dy(); y++ {
		for x := 0; x < img.Bounds().Dx(); x++ {
			img.SetRGBA(x, y, c)
		}
	}
}

func drawAt(dst, src *image.RGBA, ox, oy int) {
	for y := 0; y < src.Bounds().Dy(); y++ {
		for x := 0; x < src.Bounds().Dx(); x++ {
			dst.SetRGBA(ox+x, oy+y, src.RGBAAt(x, y))
		}
	}
}

func drawRect(dst *image.RGBA, x, y, w, h int, c color.RGBA) {
	for xx := 0; xx < w; xx++ {
		dst.SetRGBA(x+xx, y, c)
		dst.SetRGBA(x+xx, y+h-1, c)
	}
	for yy := 0; yy < h; yy++ {
		dst.SetRGBA(x, y+yy, c)
		dst.SetRGBA(x+w-1, y+yy, c)
	}
}
