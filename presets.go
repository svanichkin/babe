package main

import "fmt"

type encodePreset struct {
	quality   int
	blockSpec string
	tile      int
	q         int
}

func lerpInt(p, lo, hi, pMax int) int {
	if p <= 0 {
		return lo
	}
	if p >= pMax {
		return hi
	}
	return lo + (hi-lo)*p/pMax
}

func linearEncodePreset(p int) (encodePreset, error) {
	if p < 0 || p > 100 {
		return encodePreset{}, fmt.Errorf("p must be an integer between 0 and 100")
	}

	type stage struct {
		hi        int
		blockSpec string
		tileLo    int
		tileHi    int
		q         int
	}
	stages := [...]stage{
		{hi: 8, blockSpec: "16-128", tileLo: 32, tileHi: 30, q: 4},
		{hi: 16, blockSpec: "15-120", tileLo: 30, tileHi: 27, q: 4},
		{hi: 24, blockSpec: "14-112", tileLo: 27, tileHi: 24, q: 4},
		{hi: 32, blockSpec: "13-104", tileLo: 24, tileHi: 22, q: 3},
		{hi: 40, blockSpec: "12-96", tileLo: 22, tileHi: 20, q: 3},
		{hi: 48, blockSpec: "11-88", tileLo: 20, tileHi: 18, q: 3},
		{hi: 56, blockSpec: "10-80", tileLo: 18, tileHi: 16, q: 3},
		{hi: 64, blockSpec: "9-72", tileLo: 16, tileHi: 14, q: 2},
		{hi: 72, blockSpec: "8-64", tileLo: 14, tileHi: 12, q: 2},
		{hi: 80, blockSpec: "7-56", tileLo: 12, tileHi: 10, q: 2},
		{hi: 88, blockSpec: "6-48", tileLo: 10, tileHi: 8, q: 2},
		{hi: 91, blockSpec: "5-40", tileLo: 8, tileHi: 7, q: 1},
		{hi: 95, blockSpec: "4-32", tileLo: 7, tileHi: 6, q: 1},
		{hi: 98, blockSpec: "3-27", tileLo: 6, tileHi: 4, q: 1},
		{hi: 99, blockSpec: "2-16", tileLo: 3, tileHi: 3, q: 0},
		{hi: 100, blockSpec: "1-8", tileLo: 1, tileHi: 1, q: 0},
	}

	cur := stages[len(stages)-1]
	prevHi := -1
	for _, st := range stages {
		if p <= st.hi {
			cur = st
			break
		}
		prevHi = st.hi
	}
	span := cur.hi - prevHi
	stagePos := 0
	if span > 1 {
		stagePos = p - (prevHi + 1)
	}
	tile := lerpInt(stagePos, cur.tileLo, cur.tileHi, max(1, span-1))

	return encodePreset{
		quality:   p,
		blockSpec: cur.blockSpec,
		tile:      tile,
		q:         cur.q,
	}, nil
}
