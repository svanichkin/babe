//go:build darwin || linux

// Command smoothbench measures complete in-memory BABE cycles. JPEG/PNG input
// loading and SHA-256 verification are outside the timed codec operations.
package main

import (
	"crypto/sha256"
	"encoding/json"
	"flag"
	"fmt"
	"image"
	_ "image/jpeg"
	_ "image/png"
	"os"
	"runtime"
	"syscall"
	"time"

	"github.com/svanichkin/Babe/classic"
)

func main() {
	input := flag.String("input", "samples/source.jpg", "source image")
	q := flag.Int("q", 10, "quality (0–100)")
	n := flag.Int("n", 10, "complete encode/decode cycles per process")
	smooth := flag.Bool("smooth", true, "enable source corners and shared gradients")
	flag.Parse()
	if err := run(*input, *q, *n, *smooth); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run(input string, q, n int, smooth bool) error {
	if q < 0 || q > 100 || n < 1 {
		return fmt.Errorf("q must be 0–100 and n must be positive")
	}
	f, err := os.Open(input)
	if err != nil {
		return err
	}
	img, _, err := image.Decode(f)
	f.Close()
	if err != nil {
		return err
	}
	enc, dec := classic.NewEncoder(), classic.NewDecoder()
	enc.SparseCorners, enc.LargeGradients, enc.CornerMinSize = smooth, smooth, 5
	dec.Smooth = smooth
	var a, b time.Duration
	var data []byte
	var out *image.RGBA
	var before, after syscall.Rusage
	if err := syscall.Getrusage(syscall.RUSAGE_SELF, &before); err != nil {
		return err
	}
	for i := 0; i < n; i++ {
		start := time.Now()
		data, err = enc.Encode(img, q, false)
		a += time.Since(start)
		if err != nil {
			return err
		}
		start = time.Now()
		out, err = dec.Decode(data, false)
		b += time.Since(start)
		if err != nil {
			return err
		}
	}
	if err := syscall.Getrusage(syscall.RUSAGE_SELF, &after); err != nil {
		return err
	}
	cpu := float64(after.Utime.Sec-before.Utime.Sec+after.Stime.Sec-before.Stime.Sec) + float64(after.Utime.Usec-before.Utime.Usec+after.Stime.Usec-before.Stime.Usec)/1e6
	rss := after.Maxrss
	if runtime.GOOS == "linux" {
		rss *= 1024
	}
	return json.NewEncoder(os.Stdout).Encode(map[string]any{
		"q": q, "iterations": n, "smooth": smooth, "input": input,
		"enc_ms": float64(a) / 1e6 / float64(n), "dec_ms": float64(b) / 1e6 / float64(n), "cycle_ms": float64(a+b) / 1e6 / float64(n),
		"bytes": len(data), "pixels": fmt.Sprintf("%x", sha256.Sum256(out.Pix)), "bitstream": fmt.Sprintf("%x", sha256.Sum256(data)),
		"cpu_ms": cpu * 1000 / float64(n), "rss": rss,
	})
}
