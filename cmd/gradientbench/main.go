// Command gradientbench compares full in-memory cycles with and without ramps.
package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	_ "image/jpeg"
	_ "image/png"
	"math"
	"os"
	"sort"
	"time"

	"github.com/svanichkin/Babe/classic"
)

func main() {
	input := flag.String("input", "samples/source.jpg", "source image")
	flag.Parse()
	f, e := os.Open(*input)
	if e != nil {
		panic(e)
	}
	raw, _, e := image.Decode(f)
	if e != nil {
		panic(e)
	}
	f.Close()
	b := raw.Bounds()
	img := image.NewRGBA(image.Rect(0, 0, b.Dx(), b.Dy()))
	draw.Draw(img, img.Bounds(), raw, b.Min, draw.Src)
	ramp := image.NewRGBA(image.Rect(0, 0, 800, 572))
	for y := 0; y < 572; y++ {
		for x := 0; x < 800; x++ {
			v := uint8(x * 255 / 799)
			ramp.SetRGBA(x, y, color.RGBA{v, v, v, 255})
		}
	}
	for _, entry := range []struct {
		name string
		img  *image.RGBA
	}{{"photo", img}, {"gradient", ramp}} {
		for _, q := range []int{0, 17, 50, 80, 100} {
			type result struct {
				t    float64
				size int
				psnr float64
			}
			var vals [2][]result
			for run := -1; run < 3; run++ {
				for order := 0; order < 2; order++ {
					mode := order
					if run%2 == 0 {
						mode = 1 - order
					}
					enc := classic.NewEncoder()
					enc.Gradient = mode == 1
					dec := classic.NewDecoder()
					start := time.Now()
					data, err := enc.Encode(entry.img, q, false)
					if err != nil {
						panic(err)
					}
					out, err := dec.Decode(data, false)
					if err != nil {
						panic(err)
					}
					elapsed := time.Since(start).Seconds()
					var sq uint64
					for i := 0; i < len(out.Pix); i += 4 {
						for c := 0; c < 3; c++ {
							d := int(entry.img.Pix[i+c]) - int(out.Pix[i+c])
							sq += uint64(d * d)
						}
					}
					psnr := math.Inf(1)
					if sq > 0 {
						psnr = 10 * math.Log10(255*255*float64(entry.img.Bounds().Dx()*entry.img.Bounds().Dy()*3)/float64(sq))
					}
					if run >= 0 {
						vals[mode] = append(vals[mode], result{elapsed, len(data), psnr})
					}
				}
			}
			for mode := 0; mode < 2; mode++ {
				runs := []float64{vals[mode][0].t, vals[mode][1].t, vals[mode][2].t}
				sort.Slice(vals[mode], func(i, j int) bool { return vals[mode][i].t < vals[mode][j].t })
				a := vals[mode]
				fmt.Printf("%s q%d gradient=%v bytes=%d psnr=%.2f cycle=%.5f runs=%v\n", entry.name, q, mode == 1, a[1].size, a[1].psnr, a[1].t, runs)
			}
		}
	}
}
