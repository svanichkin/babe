package classic

import "fmt"

// Each row run belongs to a gradient leaf. Its luma glyphs are implicit: the
// quantized patch-centre value is their old baseline, needed by neighbouring
// predictors and sparse corners even though the gradient replaces their pixels.
type smoothGradientRun struct {
	length uint8 // remaining macroblocks up to this gradient's right edge
	level  byte
}

type smoothGradientFill struct {
	block largeGradient
	level byte
}

type smoothGradientPlan struct {
	runs  []smoothGradientRun
	fills []smoothGradientFill
}

func makeSmoothGradientPlan(g smoothGeometry, blocks []largeGradient, mesh *sharedGradientMesh, step int) *smoothGradientPlan {
	if len(blocks) == 0 {
		return nil
	}
	p := &smoothGradientPlan{runs: make([]smoothGradientRun, g.groupsX*g.groupsY), fills: make([]smoothGradientFill, 0, len(blocks))}
	for _, b := range blocks {
		level := quantizeLevel(mesh.at(b, b.x+b.size/2, b.y+b.size/2), step)
		p.fills = append(p.fills, smoothGradientFill{b, level})
		x0, x1 := b.x/g.macro, (b.x+b.size)/g.macro
		for y := b.y / g.macro; y < (b.y+b.size)/g.macro; y++ {
			for x := x0; x < x1; x++ {
				p.runs[y*g.groupsX+x] = smoothGradientRun{uint8(x1 - x), level}
			}
		}
	}
	return p
}

// Seed only deterministic gradient baselines. Ordinary glyph cells remain
// unavailable until decoded. The plan is shared read-only across band workers.
func (st *smoothChannelState) seedGradients(p *smoothGradientPlan) {
	st.gradients = p
	if p == nil {
		return
	}
	g := &st.geo
	for _, f := range p.fills {
		b, v := f.block, f.level
		for y := b.y; y < b.y+b.size; y++ {
			row := st.rec[y*g.w+b.x : y*g.w+b.x+b.size]
			for x := range row {
				row[x] = v
			}
		}
		for y := b.y / g.macro; y < (b.y+b.size)/g.macro; y++ {
			row := st.groupBig[y*g.groupsX+b.x/g.macro : y*g.groupsX+(b.x+b.size)/g.macro]
			for x := range row {
				row[x] = 1
			}
		}
		for y := b.y >> g.shift; y < (b.y+b.size)>>g.shift; y++ {
			for x := b.x >> g.shift; x < (b.x+b.size)>>g.shift; x++ {
				i := y*g.cellsX + x
				st.cellType[i], st.cellFg[i], st.cellCon[i], st.cellDone[i] = 0, st.levels.idx[v], 0, true
			}
		}
	}
}

// The experimental raw fallback may already contain placeholder glyphs.
// Omit them only after checking that the implicit reconstruction is exact.
func omitGradientGlyphs(blocks []smoothBlock, g smoothGeometry, p *smoothGradientPlan) ([]smoothBlock, error) {
	if p == nil {
		return blocks, nil
	}
	out := blocks[:0]
	for _, b := range blocks {
		x, y := int(b.x), int(b.y)
		if x < g.fullW && y < g.fullH {
			r := p.runs[(y/g.macro)*g.groupsX+x/g.macro]
			if r.length != 0 {
				if int(b.size) != g.macro || b.pattern || b.fg != r.level {
					return nil, fmt.Errorf("nonconstant glyph under gradient")
				}
				continue
			}
		}
		out = append(out, b)
	}
	return out, nil
}
