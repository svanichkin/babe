package main

import (
	"image"

	"github.com/svanichkin/Babe/light"
)

type Encoder = light.Encoder
type Decoder = light.Decoder
type patternMask = light.PatternMask
type encodePreset = light.EncodePreset

const defaultPatternCount = light.DefaultPatternCount

func NewEncoder() *Encoder {
	return light.NewEncoder()
}

func NewDecoder() *Decoder {
	return light.NewDecoder()
}

func Encode(img image.Image, quality int, bwmode bool) ([]byte, error) {
	return light.Encode(img, quality, bwmode)
}

func Decode(compData []byte, postfilter bool) (image.Image, error) {
	return light.Decode(compData, postfilter)
}

func presetFromQuality(quality int) (encodePreset, error) {
	return light.PresetFromQuality(quality)
}

func DecodeLayers(compData []byte) (int, int, []uint8, []image.Rectangle, []uint8, []image.Rectangle, []uint8, []image.Rectangle, bool, bool, error) {
	return light.DecodeLayers(compData)
}

func blocksFromSpec(spec string) ([]int, error) {
	return light.BlocksFromSpec(spec)
}

func setPostFilterOptions(filmGrain float64, blur int) {
	light.SetPostFilterOptions(filmGrain, blur)
}

func allowedMacroSpreadForQuality(quality int) int32 {
	return light.AllowedMacroSpreadForQuality(quality)
}

func basicPatternTemplates() [][]string {
	return light.BasicPatternTemplates()
}

func fixedPatternCodebook(bw, bh, count int) []patternMask {
	return light.FixedPatternCodebook(bw, bh, count)
}

func patternMaskKey(mask patternMask) string {
	return light.PatternMaskKey(mask)
}

func formatPatternBits(bits patternMask, bw, bh int) string {
	return light.FormatPatternBits(bits, bw, bh)
}

func buildPatternMask(bw, bh int, pred func(x, y int) bool) patternMask {
	return light.BuildPatternMask(bw, bh, pred)
}
