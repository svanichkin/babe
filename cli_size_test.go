package main

import (
	"reflect"
	"testing"
)

func TestParseMinBlockSize(t *testing.T) {
	args, size, err := parseMinBlockSize([]string{"10", "-s", "4", "out.png"})
	if err != nil || size != 4 || !reflect.DeepEqual(args, []string{"10", "out.png"}) {
		t.Fatalf("got %v %d %v", args, size, err)
	}
	for _, bad := range [][]string{{"-s"}, {"-s", "0"}, {"-s", "-1"}, {"-s", "x"}, {"-s", "4", "-s", "8"}} {
		if _, _, err := parseMinBlockSize(bad); err == nil {
			t.Fatalf("accepted %v", bad)
		}
	}
}
