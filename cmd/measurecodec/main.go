//go:build darwin || linux

// Command measurecodec measures one warmed codec operation in its own process.
// Decode measurements take a previously prepared bitstream as input so that
// encoder initialization cannot inflate the decoder's peak memory.
package main

import (
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"image"
	"image/draw"
	"image/jpeg"
	"os"
	"runtime"
	"syscall"
	"time"

	"github.com/svanichkin/Babe/classic"
	"github.com/xfmoulet/qoi"
)

type result struct {
	Codec          string  `json:"codec"`
	Operation      string  `json:"operation"`
	Quality        int     `json:"quality"`
	Iterations     int     `json:"iterations"`
	FileBytes      int     `json:"file_bytes"`
	WallSeconds    float64 `json:"wall_seconds_per_op"`
	CPUSeconds     float64 `json:"cpu_seconds_per_op"`
	AllocatedBytes uint64  `json:"allocated_bytes_per_op"`
	PeakRSSBytes   uint64  `json:"peak_rss_bytes"`
}

func main() {
	codec := flag.String("codec", "", "jpeg, babe, or qoi")
	op := flag.String("op", "", "prepare, encode, or decode")
	input := flag.String("input", "", "source image for prepare/encode; bitstream for decode")
	output := flag.String("out", "", "bitstream output for prepare")
	quality := flag.Int("quality", 80, "JPEG/BABE quality from 0 through 100")
	iterations := flag.Int("iterations", 2, "number of timed operations")
	flag.Parse()
	if err := run(*codec, *op, *input, *output, *quality, *iterations); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(codec, op, input, output string, quality, iterations int) error {
	if (codec != "jpeg" && codec != "babe" && codec != "qoi") ||
		(op != "prepare" && op != "encode" && op != "decode") ||
		input == "" || quality < 0 || quality > 100 || iterations < 1 ||
		(op == "prepare" && output == "") {
		return fmt.Errorf("invalid flags; use -codec jpeg|babe|qoi -op prepare|encode|decode -input FILE")
	}
	var operation func() error
	fileBytes := 0
	if op == "prepare" || op == "encode" {
		img, err := loadRGBA(input)
		if err != nil {
			return err
		}
		encode := newEncoder(codec, img, quality)
		encoded, err := encode()
		if err != nil {
			return err
		}
		fileBytes = len(encoded)
		if op == "prepare" {
			return os.WriteFile(output, encoded, 0644)
		}
		operation = func() error { _, err := encode(); return err }
	} else {
		data, err := os.ReadFile(input)
		if err != nil {
			return err
		}
		fileBytes = len(data)
		operation = newDecoder(codec, data)
		if err := operation(); err != nil {
			return err
		}
	}
	// Clear temporary input/conversion and warm-up garbage before measurement.
	// The process-wide peak RSS still includes initialization and warm-up.
	runtime.GC()
	var beforeMem, afterMem runtime.MemStats
	var beforeCPU, afterCPU syscall.Rusage
	runtime.ReadMemStats(&beforeMem)
	if err := syscall.Getrusage(syscall.RUSAGE_SELF, &beforeCPU); err != nil {
		return err
	}
	start := time.Now()
	for i := 0; i < iterations; i++ {
		if err := operation(); err != nil {
			return err
		}
	}
	wall := time.Since(start).Seconds()
	if err := syscall.Getrusage(syscall.RUSAGE_SELF, &afterCPU); err != nil {
		return err
	}
	runtime.ReadMemStats(&afterMem)
	cpu := cpuSeconds(afterCPU) - cpuSeconds(beforeCPU)
	peak := uint64(afterCPU.Maxrss)
	if runtime.GOOS == "linux" {
		peak *= 1024 // Linux reports KiB; Darwin reports bytes.
	}
	r := result{
		Codec: codec, Operation: op, Quality: quality, Iterations: iterations, FileBytes: fileBytes,
		WallSeconds: wall / float64(iterations), CPUSeconds: cpu / float64(iterations),
		AllocatedBytes: (afterMem.TotalAlloc - beforeMem.TotalAlloc) / uint64(iterations), PeakRSSBytes: peak,
	}
	return json.NewEncoder(os.Stdout).Encode(r)
}

func cpuSeconds(r syscall.Rusage) float64 {
	return float64(r.Utime.Sec+r.Stime.Sec) + float64(r.Utime.Usec+r.Stime.Usec)/1e6
}

func loadRGBA(path string) (*image.RGBA, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	src, _, err := image.Decode(f)
	if err != nil {
		return nil, err
	}
	b := src.Bounds()
	img := image.NewRGBA(image.Rect(0, 0, b.Dx(), b.Dy()))
	draw.Draw(img, img.Bounds(), src, b.Min, draw.Src)
	return img, nil
}

func newEncoder(codec string, img *image.RGBA, quality int) func() ([]byte, error) {
	var buf bytes.Buffer
	switch codec {
	case "jpeg":
		return func() ([]byte, error) {
			buf.Reset()
			err := jpeg.Encode(&buf, img, &jpeg.Options{Quality: quality})
			return buf.Bytes(), err
		}
	case "qoi":
		return func() ([]byte, error) {
			buf.Reset()
			err := qoi.Encode(&buf, img)
			return buf.Bytes(), err
		}
	default:
		enc := classic.NewEncoder()
		return func() ([]byte, error) { return enc.Encode(img, quality, false) }
	}
}

func newDecoder(codec string, data []byte) func() error {
	var reader bytes.Reader
	switch codec {
	case "jpeg":
		return func() error { reader.Reset(data); _, err := jpeg.Decode(&reader); return err }
	case "qoi":
		return func() error { reader.Reset(data); _, err := qoi.Decode(&reader); return err }
	default:
		dec := classic.NewDecoder()
		return func() error { _, err := dec.Decode(data, false); return err }
	}
}
