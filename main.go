package main

import (
	"encoding/csv"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	_ "image/gif"
	_ "image/jpeg"
	"image/png"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"time"
)

func writePatternUsageSheets(basePath string) error {
	if len(lastPatternUsageByChannel) == 0 {
		return nil
	}

	const (
		cellScale = 8
		padding   = 4
		cols      = 8
	)

	channels := make([]string, 0, len(lastPatternUsageByChannel))
	for channel := range lastPatternUsageByChannel {
		channels = append(channels, channel)
	}
	sort.Strings(channels)

	for _, channel := range channels {
		sizeUsage := lastPatternUsageByChannel[channel]
		sizes := make([][2]int, 0, len(sizeUsage))
		for size := range sizeUsage {
			sizes = append(sizes, size)
		}
		sort.Slice(sizes, func(i, j int) bool {
			if sizes[i][0] != sizes[j][0] {
				return sizes[i][0] < sizes[j][0]
			}
			return sizes[i][1] < sizes[j][1]
		})

		for _, size := range sizes {
			entries := sizeUsage[size]
			if len(entries) == 0 {
				continue
			}

			bw, bh := size[0], size[1]
			rows := (len(entries) + cols - 1) / cols
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

			for i, entry := range entries {
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
						c := color.Color(color.RGBA{250, 250, 250, 255})
						if testPatternBit(entry.bits, bw, bh, px, py) {
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

			outPath := fmt.Sprintf("%s.pattern-usage.%s.%dx%d.png", basePath, strings.ToLower(channel), bw, bh)
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
	}

	return nil
}

type sweepResult struct {
	Quality     int
	Blocks      string
	Spreads     string
	SizeBytes   int
	MAE         float64
	RMSE        float64
	MaxAbsDiff  uint8
	Compression float64
	Pareto      bool
}

func parseIntSweepSpec(spec string) ([]int, error) {
	parts := strings.Split(spec, "..")
	if len(parts) < 2 || len(parts) > 3 {
		return nil, fmt.Errorf("range must look like 0..100 or 0..100..1")
	}
	start, err := strconv.Atoi(strings.TrimSpace(parts[0]))
	if err != nil {
		return nil, err
	}
	end, err := strconv.Atoi(strings.TrimSpace(parts[1]))
	if err != nil {
		return nil, err
	}
	step := 1
	if len(parts) == 3 {
		step, err = strconv.Atoi(strings.TrimSpace(parts[2]))
		if err != nil {
			return nil, err
		}
	}
	if step <= 0 {
		return nil, fmt.Errorf("step must be positive")
	}
	if end < start {
		return nil, fmt.Errorf("range end must be >= start")
	}
	values := make([]int, 0, (end-start)/step+1)
	for v := start; v <= end; v += step {
		values = append(values, v)
	}
	return values, nil
}

func parseFloatSweepSpec(spec string) ([]float64, error) {
	parts := strings.Split(spec, "..")
	if len(parts) < 2 || len(parts) > 3 {
		return nil, fmt.Errorf("range must look like 0.1..1 or 0.1..1..0.1")
	}
	start, err := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
	if err != nil {
		return nil, err
	}
	end, err := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
	if err != nil {
		return nil, err
	}
	step := 1.0
	if len(parts) == 3 {
		step, err = strconv.ParseFloat(strings.TrimSpace(parts[2]), 64)
		if err != nil {
			return nil, err
		}
	}
	if step <= 0 {
		return nil, fmt.Errorf("step must be positive")
	}
	if end < start {
		return nil, fmt.Errorf("range end must be >= start")
	}
	values := make([]float64, 0, int(math.Floor((end-start)/step))+1)
	for v := start; v <= end+step*0.5; v += step {
		values = append(values, math.Round(v*1000)/1000)
	}
	return values, nil
}

func spreadFactorsToSpec(factors []float64) string {
	parts := make([]string, len(factors))
	for i, v := range factors {
		parts[i] = strconv.FormatFloat(v, 'f', -1, 64)
	}
	return strings.Join(parts, ",")
}

func toRGBAImage(src image.Image) *image.RGBA {
	if rgba, ok := src.(*image.RGBA); ok {
		return rgba
	}
	b := src.Bounds()
	dst := image.NewRGBA(b)
	draw.Draw(dst, b, src, b.Min, draw.Src)
	return dst
}

func diffMetrics(a, b image.Image) (mae float64, rmse float64, maxAbs uint8, err error) {
	ab := a.Bounds()
	bb := b.Bounds()
	if ab.Dx() != bb.Dx() || ab.Dy() != bb.Dy() {
		return 0, 0, 0, fmt.Errorf("image sizes differ: %v vs %v", ab, bb)
	}

	var absSum float64
	var sqSum float64
	pixels := float64(ab.Dx() * ab.Dy() * 3)
	for y := 0; y < ab.Dy(); y++ {
		for x := 0; x < ab.Dx(); x++ {
			ar, ag, abv, _ := a.At(ab.Min.X+x, ab.Min.Y+y).RGBA()
			br, bg, bbv, _ := b.At(bb.Min.X+x, bb.Min.Y+y).RGBA()
			diffs := [3]int{
				int(ar>>8) - int(br>>8),
				int(ag>>8) - int(bg>>8),
				int(abv>>8) - int(bbv>>8),
			}
			for _, d := range diffs {
				ad := d
				if ad < 0 {
					ad = -ad
				}
				absSum += float64(ad)
				sqSum += float64(d * d)
				if uint8(ad) > maxAbs {
					maxAbs = uint8(ad)
				}
			}
		}
	}

	if pixels == 0 {
		return 0, 0, 0, nil
	}
	return absSum / pixels, math.Sqrt(sqSum / pixels), maxAbs, nil
}

func markPareto(results []sweepResult) []sweepResult {
	sort.Slice(results, func(i, j int) bool {
		if results[i].SizeBytes != results[j].SizeBytes {
			return results[i].SizeBytes < results[j].SizeBytes
		}
		return results[i].RMSE < results[j].RMSE
	})

	bestRMSE := math.Inf(1)
	for i := range results {
		if results[i].RMSE < bestRMSE {
			results[i].Pareto = true
			bestRMSE = results[i].RMSE
		}
	}
	return results
}

func writeSweepCSV(path string, results []sweepResult) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()

	w := csv.NewWriter(f)
	defer w.Flush()

	if err := w.Write([]string{"quality", "blocks", "spreads", "size_bytes", "compression_ratio", "mae", "rmse", "max_abs_diff", "pareto"}); err != nil {
		return err
	}
	for _, r := range results {
		row := []string{
			strconv.Itoa(r.Quality),
			r.Blocks,
			r.Spreads,
			strconv.Itoa(r.SizeBytes),
			strconv.FormatFloat(r.Compression, 'f', 6, 64),
			strconv.FormatFloat(r.MAE, 'f', 6, 64),
			strconv.FormatFloat(r.RMSE, 'f', 6, 64),
			strconv.Itoa(int(r.MaxAbsDiff)),
			strconv.FormatBool(r.Pareto),
		}
		if err := w.Write(row); err != nil {
			return err
		}
	}
	return w.Error()
}

func runSweep(inPath string, qualities []int, spreadValues []float64, blockSpec string, bwmode bool, patternCount int, colorQuantShift int, patternIndexMode string, logPatterns bool, csvPath string) error {
	in, err := os.Open(inPath)
	if err != nil {
		return err
	}
	defer in.Close()

	src, _, err := image.Decode(in)
	if err != nil {
		return err
	}
	srcRGBA := toRGBAImage(src)

	info, err := os.Stat(inPath)
	if err != nil {
		return err
	}
	inputBytes := info.Size()

	if blockSpec != "" {
		if err := setBlocksFromSpec(blockSpec); err != nil {
			return err
		}
	} else {
		blockLevels = append(blockLevels[:0], defaultBlockLevels[:]...)
	}
	levels := append([]int(nil), activeLevels()...)

	blockLabel := make([]string, len(levels))
	for i, v := range levels {
		blockLabel[i] = strconv.Itoa(v)
	}

	fixedSpreadMode := len(spreadValues) == 0
	totalCombos := len(qualities)
	if fixedSpreadMode {
		totalCombos *= 1
	} else {
		for range levels {
			totalCombos *= len(spreadValues)
		}
	}
	fmt.Printf("sweep: qualities=%d block-levels=%v spread-values=%d total-combinations=%d\n", len(qualities), levels, len(spreadValues), totalCombos)

	results := make([]sweepResult, 0, min(totalCombos, 4096))
	current := make([]float64, len(levels))
	startedAt := time.Now()
	completed := 0
	lastProgressAt := time.Time{}
	progressInterval := 2 * time.Second

	printProgress := func(force bool) {
		now := time.Now()
		if !force && !lastProgressAt.IsZero() && now.Sub(lastProgressAt) < progressInterval {
			return
		}
		lastProgressAt = now
		if totalCombos <= 0 || completed <= 0 {
			fmt.Fprintf(os.Stderr, "[sweep] %d/%d\n", completed, totalCombos)
			return
		}

		elapsed := now.Sub(startedAt)
		rate := float64(completed) / elapsed.Seconds()
		remaining := totalCombos - completed
		eta := time.Duration(0)
		if rate > 0 && remaining > 0 {
			eta = time.Duration(float64(time.Second) * float64(remaining) / rate)
		}
		percent := 100 * float64(completed) / float64(totalCombos)
		fmt.Fprintf(os.Stderr, "[sweep] %d/%d %.2f%% elapsed=%s rate=%.2f iter/s eta=%s\n",
			completed, totalCombos, percent, elapsed.Round(time.Second), rate, eta.Round(time.Second))
	}

	var visit func(levelIdx int) error
	visit = func(levelIdx int) error {
		if levelIdx == len(levels) {
			spreadSpec := spreadFactorsToSpec(current)
			if err := setSpreadFactorsFromSpec(spreadSpec); err != nil {
				return err
			}
			for _, quality := range qualities {
				activePatternCount = patternCount
				activeColorQuantShift = colorQuantShift
				activeSharedPatternIndexes = patternIndexMode == "shared"
				encodeLog = logPatterns

				comp, err := Encode(srcRGBA, quality, bwmode)
				if err != nil {
					return err
				}
				dec, err := Decode(comp, false)
				if err != nil {
					return err
				}
				mae, rmse, maxAbs, err := diffMetrics(srcRGBA, dec)
				if err != nil {
					return err
				}

				results = append(results, sweepResult{
					Quality:     quality,
					Blocks:      strings.Join(blockLabel, ","),
					Spreads:     spreadSpec,
					SizeBytes:   len(comp),
					MAE:         mae,
					RMSE:        rmse,
					MaxAbsDiff:  maxAbs,
					Compression: float64(len(comp)) / float64(inputBytes),
				})
				completed++
				printProgress(false)
			}
			return nil
		}

		if fixedSpreadMode {
			copy(current, forcedSpreadFactors)
			return visit(len(levels))
		}

		for _, v := range spreadValues {
			current[levelIdx] = v
			if err := visit(levelIdx + 1); err != nil {
				return err
			}
		}
		return nil
	}

	if fixedSpreadMode {
		if len(forcedSpreadFactors) != len(levels) {
			forcedSpreadFactors = make([]float64, len(levels))
			for i := range forcedSpreadFactors {
				forcedSpreadFactors[i] = 1
			}
		}
	}

	if err := visit(0); err != nil {
		return err
	}
	printProgress(true)

	results = markPareto(results)
	sort.Slice(results, func(i, j int) bool {
		if results[i].Pareto != results[j].Pareto {
			return results[i].Pareto
		}
		if results[i].RMSE != results[j].RMSE {
			return results[i].RMSE < results[j].RMSE
		}
		return results[i].SizeBytes < results[j].SizeBytes
	})

	if csvPath == "" {
		base := strings.TrimSuffix(inPath, filepath.Ext(inPath))
		csvPath = base + ".sweep.csv"
	}
	if err := writeSweepCSV(csvPath, results); err != nil {
		return err
	}

	fmt.Printf("wrote %s\n", csvPath)
	fmt.Println("top pareto results:")
	printed := 0
	for _, r := range results {
		if !r.Pareto {
			continue
		}
		fmt.Printf("q=%d blocks=%s spreads=%s size=%d ratio=%.4f mae=%.4f rmse=%.4f maxdiff=%d\n",
			r.Quality, r.Blocks, r.Spreads, r.SizeBytes, r.Compression, r.MAE, r.RMSE, r.MaxAbsDiff)
		printed++
		if printed == 20 {
			break
		}
	}
	return nil
}

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
					c := color.Color(color.RGBA{250, 250, 250, 255})
					if testPatternBit(bits, bw, bh, px, py) {
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
	if len(os.Args) < 2 || len(os.Args) > 16 {
		fmt.Fprint(os.Stderr, "Usage:\n  babe <input-image> [quality] [bw] [decoded.png] [-patterns=N] [-blocks=A,B|A-B] [-spreads=S1,S2,...] [-sweep] [-quality-range=0..100..1] [-spread-range=0.1..1..0.1] [-csv=results.csv] [-color-quant=N] [-pattern-set=basic] [-pattern-index=per-channel|shared] [-backgroundTile N] [-log]\n  babe <input.babe> [-postfilter] [-layers]\n  (bw flag, decoded.png, -patterns=N, -blocks=A,B|A-B, -spreads=S1,S2,..., -sweep, -quality-range=..., -spread-range=..., -csv=..., -color-quant=N, -pattern-set=basic, -pattern-index=..., -backgroundTile N and -log can appear anywhere after quality)\n")
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
	sweepMode := false
	qualityRangeSpec := ""
	spreadRangeSpec := ""
	csvPath := ""
	colorQuantShift := 0
	patternIndexMode := "per-channel"
	backgroundTile := 0
	logPatterns := false
	for i := 0; i < len(encodeArgs); i++ {
		a := encodeArgs[i]
		if a == "bw" {
			bwmode = true
			continue
		}
		if a == "-log" {
			logPatterns = true
			continue
		}
		if a == "-sweep" {
			sweepMode = true
			continue
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
		if strings.HasPrefix(a, "-quality-range=") {
			qualityRangeSpec = strings.TrimPrefix(a, "-quality-range=")
			continue
		}
		if strings.HasPrefix(a, "-spread-range=") {
			spreadRangeSpec = strings.TrimPrefix(a, "-spread-range=")
			continue
		}
		if strings.HasPrefix(a, "-csv=") {
			csvPath = strings.TrimPrefix(a, "-csv=")
			continue
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
		if a == "-backgroundTile" {
			if i+1 >= len(encodeArgs) {
				fmt.Fprintln(os.Stderr, "backgroundTile requires a value between 2 and 255")
				os.Exit(1)
			}
			v, err := strconv.Atoi(encodeArgs[i+1])
			if err != nil || v < 2 || v > 255 {
				fmt.Fprintln(os.Stderr, "backgroundTile must be an integer between 2 and 255")
				os.Exit(1)
			}
			backgroundTile = v
			i++
			continue
		}
		if strings.HasPrefix(a, "-backgroundTile=") {
			v, err := strconv.Atoi(strings.TrimPrefix(a, "-backgroundTile="))
			if err != nil || v < 2 || v > 255 {
				fmt.Fprintln(os.Stderr, "backgroundTile must be an integer between 2 and 255")
				os.Exit(1)
			}
			backgroundTile = v
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
	if sweepMode {
		qualities := []int{quality}
		if qualityRangeSpec != "" {
			values, err := parseIntSweepSpec(qualityRangeSpec)
			if err != nil {
				fmt.Fprintln(os.Stderr, "quality-range:", err)
				os.Exit(1)
			}
			qualities = values
		}

		spreadValues := []float64{1}
		if spreadRangeSpec != "" {
			values, err := parseFloatSweepSpec(spreadRangeSpec)
			if err != nil {
				fmt.Fprintln(os.Stderr, "spread-range:", err)
				os.Exit(1)
			}
			spreadValues = values
		} else if spreadSpec != "" {
			spreadValues = nil
		}

		if err := runSweep(inputPath, qualities, spreadValues, blockSpec, bwmode, patternCount, colorQuantShift, patternIndexMode, logPatterns, csvPath); err != nil {
			fmt.Fprintln(os.Stderr, "sweep error:", err)
			os.Exit(1)
		}
		return
	}
	if err := encodeToBabe(inputPath, outPath, quality, bwmode, patternCount, colorQuantShift, patternIndexMode, backgroundTile, logPatterns); err != nil {
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

func encodeToBabe(inPath, outPath string, quality int, bwmode bool, patternCount int, colorQuantShift int, patternIndexMode string, backgroundTile int, logPatterns bool) error {
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
	activeBackgroundTile = backgroundTile
	encodeLog = logPatterns
	defer func() {
		activePatternCount = defaultPatternCount
		activeColorQuantShift = 0
		activeSharedPatternIndexes = false
		activeBackgroundTile = 0
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
		if err := writePatternUsageSheets(base); err != nil {
			return err
		}
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
	fmt.Printf("quality=%d, patterns=%d, color-quant=%d, pattern-set=%s, pattern-index=%s, backgroundTile=%d, ratio=%.3f, time=%s\n",
		quality,
		patternCount,
		colorQuantShift,
		patternSetBasic,
		patternIndexMode,
		backgroundTile,
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
