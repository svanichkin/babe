package main

import "testing"

func TestCLIFlags(t *testing.T) {
	for _, flag := range []string{"-s", "-sparse", "-corners", "-glyphs", "-dither", "-dither2", "-dither3", "-pattern2", "-unknown"} {
		for _, decoding := range []bool{false, true} {
			if err := validateCLIFlags([]string{flag}, decoding); err == nil {
				t.Fatalf("accepted removed flag %s", flag)
			}
		}
	}
	for _, decoding := range []bool{false, true} {
		if err := validateCLIFlags([]string{"-smooth", "-glyphs-only"}, decoding); err != nil {
			t.Fatal(err)
		}
	}
	if validateCLIFlags([]string{"-postfilter"}, true) != nil || validateCLIFlags([]string{"-postfilter"}, false) == nil {
		t.Fatal("postfilter mode validation failed")
	}
}
