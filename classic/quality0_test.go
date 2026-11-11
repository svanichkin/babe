package classic

import (
	"bytes"
	"image"
	_ "image/jpeg"
	"os"
	"testing"
)

func TestQualityZeroSmallerOnExample(t *testing.T) {
	f, err := os.Open("../samples/source.jpg")
	if err != nil {
		t.Fatal(err)
	}
	img, _, err := image.Decode(f)
	f.Close()
	if err != nil {
		t.Fatal(err)
	}
	e, d := NewEncoder(), NewDecoder()
	defer d.zdec.Close()
	atOne, err := e.Encode(img, 1, false)
	if err != nil {
		t.Fatal(err)
	}
	sizeOne := len(atOne)
	baselineOne := bytes.Clone(atOne)
	atZero, err := e.Encode(img, 0, false)
	if err != nil {
		t.Fatal(err)
	}
	if len(atZero)*2 >= sizeOne {
		t.Fatalf("q0 = %d bytes, q1 = %d bytes: q0 did not achieve the expected reduction on the example", len(atZero), sizeOne)
	}
	decoded, err := d.Decode(atZero, false)
	if err != nil {
		t.Fatal(err)
	}
	if decoded.Bounds().Size() != img.Bounds().Size() {
		t.Fatalf("decoded dimensions = %v, want %v", decoded.Bounds(), img.Bounds())
	}
	afterZero, err := e.Encode(img, 1, false)
	if err != nil || !bytes.Equal(afterZero, baselineOne) {
		t.Fatalf("q0 changed the next q1 encode: %v", err)
	}
}
