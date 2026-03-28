package main

import (
	"image"

	"github.com/svanichkin/Babe/classic"
)

type Encoder = classic.Encoder
type Decoder = classic.Decoder

func NewEncoder() *Encoder {
	return classic.NewEncoder()
}

func NewDecoder() *Decoder {
	return classic.NewDecoder()
}

func Encode(img image.Image, quality int, bwmode bool) ([]byte, error) {
	return classic.Encode(img, quality, bwmode)
}

func Decode(compData []byte, postfilter bool) (image.Image, error) {
	return classic.Decode(compData, postfilter)
}
