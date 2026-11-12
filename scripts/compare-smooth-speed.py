#!/usr/bin/env python3
"""Compare complete BABE cycles in shuffled, fresh-process rounds."""
import argparse
import json
import random
import statistics
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--input", default="samples/source.jpg", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--qualities", default="0,10,50,100")
    parser.add_argument("--rounds", default=3, type=int)
    parser.add_argument("--iterations", default=10, type=int)
    parser.add_argument("--plain", action="store_true")
    args = parser.parse_args()
    qualities = [int(q) for q in args.qualities.split(",")]
    if args.rounds < 3 or args.iterations < 1 or any(q < 0 or q > 100 for q in qualities):
        parser.error("require >=3 rounds, positive iterations, and qualities 0–100")
    versions = {"before": args.before.resolve(), "after": args.after.resolve()}
    rows = []
    for rnd in range(args.rounds):
        jobs = [(v, q) for v in versions for q in qualities]
        random.Random(117 + rnd).shuffle(jobs)
        for version, quality in jobs:
            cmd = [str(versions[version]), "-input", str(args.input.resolve()),
                   "-q", str(quality), "-n", str(args.iterations)]
            if args.plain:
                cmd.append("-smooth=false")
            row = json.loads(subprocess.check_output(cmd, text=True))
            row.update(version=version, round=rnd + 1)
            rows.append(row)
            print(f"round={rnd+1} {version} q={quality}: "
                  f"encode={row['enc_ms']:.3f} decode={row['dec_ms']:.3f} ms "
                  f"bytes={row['bytes']}", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2) + "\n")
    for quality in qualities:
        same_quality = [r for r in rows if r["q"] == quality]
        if len({r["pixels"] for r in same_quality}) != 1:
            raise RuntimeError(f"decoded pixels changed at q={quality}")
        for version in versions:
            cases = [r for r in same_quality if r["version"] == version]
            medians = {k: round(statistics.median(r[k] for r in cases), 3)
                       for k in ("enc_ms", "dec_ms", "cycle_ms", "bytes", "cpu_ms", "rss")}
            print(quality, version, medians)


if __name__ == "__main__":
    main()
