package main

import (
	"encoding/binary"
	"flag"
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
	fs := flag.NewFlagSet("babe", flag.ContinueOnError)
	fs.SetOutput(io.Discard) // we print our own usage/errors
	var paletteFlag string
	fs.StringVar(&paletteFlag, "palette", "", "palette mode: bright|luma|bw|auto|gray|zx|<name>|<palette-spec> (compatible with legacy syntax)")

	usage := func() {
		fmt.Fprint(os.Stderr, "Usage:\n  babe <input-image> [quality] [decoded.png]\n  babe <input-image> [quality] [-palette bright] [decoded.png]\n  babe <input-image> [quality] [-palette luma [y cb cr|y:cb:cr]] [decoded.png]\n  babe <input-image> [quality] [-palette bw [bits]] [decoded.png]\n  babe <input-image> [quality] [-palette N] [decoded.png]\n  babe <input-image> [quality] [-palette auto [P]] [decoded.png]\n  babe <input-image> [quality] [-palette gray N] [decoded.png]\n  babe <input-image> [quality] [-palette zx|cga|ega|vga|c64|gameboy|pico8|db16|nes|sunset|pastel|ocean|forest|<palette-spec>] [decoded.png]\n  babe <input.babe>\n")
	}

	// Stdlib flag parsing stops at first non-flag. To keep the existing CLI (input/quality
	// can appear before flags), we split argv into "flags" and "positionals" ourselves.
	type parseRes struct {
		flags       []string
		positionals []string
	}
	splitArgs := func(argv []string) parseRes {
		var out parseRes
		isFlagLike := func(s string) bool { return strings.HasPrefix(s, "-") && len(s) > 1 }
		for i := 0; i < len(argv); i++ {
			a := argv[i]
			if a == "--" {
				out.positionals = append(out.positionals, argv[i+1:]...)
				break
			}
			if strings.EqualFold(a, "-palette") || strings.EqualFold(a, "--palette") {
				// Legacy: -palette <mode> [extra...]
				if i+1 >= len(argv) {
					out.flags = append(out.flags, "-palette", "")
					continue
				}
				mode := strings.ToLower(argv[i+1])
				i++
				val := mode
				// Consume extra token(s) for some modes and pack into one string.
				if (mode == "auto" || mode == "gray" || mode == "bw") && i+1 < len(argv) && !isFlagLike(argv[i+1]) {
					val = mode + ":" + argv[i+1]
					i++
				} else if mode == "luma" {
					// luma supports either "y:cb:cr" or "y cb cr"
					if i+1 < len(argv) && !isFlagLike(argv[i+1]) {
						next := argv[i+1]
						if strings.Contains(next, ":") {
							val = mode + ":" + next
							i++
						} else if i+3 <= len(argv)-1 && !isFlagLike(argv[i+2]) && !isFlagLike(argv[i+3]) {
							val = mode + ":" + argv[i+1] + ":" + argv[i+2] + ":" + argv[i+3]
							i += 3
						}
					}
				}
				out.flags = append(out.flags, "-palette", val)
				continue
			}
			if isFlagLike(a) {
				out.flags = append(out.flags, a)
				// Only flag we support here is -palette, so other flags are treated as positionals.
				// This preserves current behavior (unknown flags were effectively ignored).
				continue
			}
			out.positionals = append(out.positionals, a)
		}
		return out
	}

	parts := splitArgs(os.Args[1:])
	if err := fs.Parse(parts.flags); err != nil {
		fmt.Fprintln(os.Stderr, err)
		usage()
		os.Exit(1)
	}
	pos := append([]string{}, parts.positionals...)
	// If any leftover args from fs.Parse (should be none) treat as positional.
	pos = append(pos, fs.Args()...)
	if len(pos) < 1 {
		usage()
		os.Exit(1)
	}

	inputPath := pos[0]
	ext := strings.ToLower(filepath.Ext(inputPath))
	base := strings.TrimSuffix(inputPath, filepath.Ext(inputPath))

	// If input is .babe → decode to PNG
	if ext == ".babe" {
		if err := decodeBabe(inputPath, base+".png", false); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
		return
	}

	// Otherwise: encode image → .babe with default or provided quality
	quality := 70
	if len(pos) >= 2 {
		q, err := strconv.Atoi(pos[1])
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

	useZstd := true
	zxMode := false
	paletteName := ""
	paletteMode := ""
	decodeOutPath := ""
	yBits, cbBits, crBits := 1, 0, 0
	if len(pos) >= 3 {
		for _, a := range pos[2:] {
			if strings.EqualFold(filepath.Ext(a), ".png") {
				decodeOutPath = a
				break
			}
		}
	}
	// Palette selection (legacy compatible). `paletteFlag` can be:
	// - "bright" | "luma" | "bw" | "zx"
	// - "auto[:P]" | "gray:N"
	// - "<name>" | "<palette-spec>" | "N" (adaptive N)
	// - "luma:y:cb:cr"
	if paletteFlag != "" {
		value := strings.ToLower(paletteFlag)
		mode := value
		arg := ""
		if i := strings.IndexByte(value, ':'); i >= 0 {
			mode = value[:i]
			arg = value[i+1:]
		}
		if mode == "bright" || mode == "luma" || mode == "bw" {
			paletteMode = mode
		}
		if mode == "auto" {
			paletteName = "adaptive:auto"
			if arg != "" {
				pct, err := strconv.Atoi(arg)
				if err != nil || pct < 0 || pct > 100 {
					fmt.Fprintln(os.Stderr, "palette auto quality must be an integer between 0 and 100")
					os.Exit(1)
				}
				paletteName = fmt.Sprintf("adaptive:auto:%d", pct)
			}
		} else if mode == "gray" {
			n, err := strconv.Atoi(arg)
			if err != nil || n < 1 || n > 256 {
				fmt.Fprintln(os.Stderr, "gray palette size must be an integer between 1 and 256")
				os.Exit(1)
			}
			paletteName = fmt.Sprintf("gray:%d", n)
		} else if mode == "zx" {
			zxMode = true
		} else if mode == "luma" {
			if arg != "" {
				parts := strings.Split(arg, ":")
				if len(parts) != 3 {
					fmt.Fprintln(os.Stderr, "luma bit depth triplet must look like y:cb:cr, for example 4:4:4")
					os.Exit(1)
				}
				var err error
				yBits, err = parseBitDepth(parts[0], "y")
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					os.Exit(1)
				}
				cbBits, err = parseBitDepth(parts[1], "cb")
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					os.Exit(1)
				}
				crBits, err = parseBitDepth(parts[2], "cr")
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					os.Exit(1)
				}
				paletteMode = "luma"
			} else {
				// Default legacy behavior.
				yBits, cbBits, crBits = 1, 1, 1
				paletteMode = "luma"
			}
		} else if mode == "bw" {
			if arg != "" {
				bw, err := parseBitDepth(arg, "bw")
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					os.Exit(1)
				}
				yBits, cbBits, crBits = bw, 0, 0
			} else {
				yBits, cbBits, crBits = 1, 0, 0
			}
			paletteMode = "bw"
		} else {
			// Named palette/spec/adaptive N.
			switch mode {
			case "sunset", "pastel", "ocean", "forest", "cga", "ega", "vga", "c64", "gameboy", "pico8", "db16", "nes":
				paletteName = mode
			default:
				if isPaletteSpec(mode) {
					paletteName = mode
				} else if n, err := strconv.Atoi(mode); err == nil {
					if n < 2 || n > 256 {
						fmt.Fprintln(os.Stderr, "palette value must be a size 2..256, 'auto', 'gray N', a named palette, or a palette spec")
						os.Exit(1)
					}
					paletteName = fmt.Sprintf("adaptive:%d", n)
				} else {
					fmt.Fprintln(os.Stderr, "unknown palette:", paletteFlag)
					os.Exit(1)
				}
			}
		}
	}
	if paletteMode == "bright" {
		yBits, cbBits, crBits = 1, 1, 1
	} else if zxMode || paletteName != "" {
		yBits, cbBits, crBits = 1, 1, 1
	}
	bwmode := cbBits == 0 && crBits == 0
	bwBits := yBits

	outPath := base + ".babe"
	if err := encodeToBabe(inputPath, outPath, quality, bwmode, bwBits, yBits, cbBits, crBits, useZstd, false, zxMode, paletteName, paletteMode); err != nil {
		fmt.Fprintln(os.Stderr, "encode error:", err)
		os.Exit(1)
	}
	if decodeOutPath != "" {
		if err := decodeBabe(outPath, decodeOutPath, false); err != nil {
			fmt.Fprintln(os.Stderr, "decode error:", err)
			os.Exit(1)
		}
	}
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

func encodeToBabe(inPath, outPath string, quality int, bwmode bool, bwBits, yBits, cbBits, crBits int, useZstd bool, rgbMode, zxMode bool, paletteName, paletteMode string) error {
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
			BW:      true,
			YBits:   bwBits,
			UseZstd: useZstd,
		})
	} else {
		enc, err = NewEncoder().EncodeWithOptions(img, quality, EncodeOptions{
			BW:      false,
			YBits:   yBits,
			CbBits:  cbBits,
			CrBits:  crBits,
			UseZstd: useZstd,
			RGBMode: false,
			ZXMode:  zxMode,
			Palette: paletteName,
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
	colorCount := 0
	if !bwmode && paletteName == "" && !zxMode {
		if count, err := inspectImageColorCount(enc); err == nil {
			colorCount = count
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
			fmt.Printf("quality=%d, bw_bits=%d, pattern=%s, palette=%d, zstd=%t, ratio=%.3f, time=%s\n", quality, bwBits, patternLabel, patternCount, useZstd, ratio, finish)
		} else {
			fmt.Printf("quality=%d, bw_bits=%d, zstd=%t, ratio=%.3f, time=%s\n", quality, bwBits, useZstd, ratio, finish)
		}
	} else {
		if paletteName != "" {
			fmt.Printf("quality=%d, mode=%s, zstd=%t, ratio=%.3f, time=%s\n", quality, paletteLabel, useZstd, ratio, finish)
		} else if zxMode {
			fmt.Printf("quality=%d, mode=zx, zstd=%t, ratio=%.3f, time=%s\n", quality, useZstd, ratio, finish)
		} else if paletteMode == "bright" {
			if colorCount > 0 {
				fmt.Printf("quality=%d, mode=bright, colors=%d, zstd=%t, ratio=%.3f, time=%s\n", quality, colorCount, useZstd, ratio, finish)
			} else {
				fmt.Printf("quality=%d, mode=bright, zstd=%t, ratio=%.3f, time=%s\n", quality, useZstd, ratio, finish)
			}
		} else if paletteMode == "luma" || (yBits == 1 && cbBits == 1 && crBits == 1) {
			if colorCount > 0 {
				fmt.Printf("quality=%d, mode=luma, colors=%d, zstd=%t, ratio=%.3f, time=%s\n", quality, colorCount, useZstd, ratio, finish)
			} else {
				fmt.Printf("quality=%d, mode=luma, zstd=%t, ratio=%.3f, time=%s\n", quality, useZstd, ratio, finish)
			}
		} else {
			if colorCount > 0 {
				fmt.Printf("quality=%d, y_bits=%d, cb_bits=%d, cr_bits=%d, colors=%d, zstd=%t, ratio=%.3f, time=%s\n", quality, yBits, cbBits, crBits, colorCount, useZstd, ratio, finish)
			} else {
				fmt.Printf("quality=%d, y_bits=%d, cb_bits=%d, cr_bits=%d, zstd=%t, ratio=%.3f, time=%s\n", quality, yBits, cbBits, crBits, useZstd, ratio, finish)
			}
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

func inspectImageColorCount(comp []byte) (int, error) {
	img, err := Decode(comp)
	if err != nil {
		return 0, err
	}
	b := img.Bounds()
	colors := make(map[[4]uint8]struct{}, min(b.Dx()*b.Dy(), 1<<16))
	for y := b.Min.Y; y < b.Max.Y; y++ {
		for x := b.Min.X; x < b.Max.X; x++ {
			r, g, b8, a := img.At(x, y).RGBA()
			key := [4]uint8{uint8(r >> 8), uint8(g >> 8), uint8(b8 >> 8), uint8(a >> 8)}
			colors[key] = struct{}{}
		}
	}
	return len(colors), nil
}

func decodeBabe(inPath, outPath string, splitChannels bool) error {

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
	dec, err := Decode(compData)
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
