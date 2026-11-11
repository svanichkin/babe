#!/usr/bin/env python3
"""Compare complete BABE cycles with ordinary and corner-gradient decoding."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import runpy
import statistics
import subprocess
import tempfile
import time

shared = runpy.run_path(str(Path(__file__).with_name("compare-lossless.py")))
timed, ppm_hash = shared["timed"], shared["ppm_hash"]
parser = argparse.ArgumentParser()
parser.add_argument("--input", required=True)
parser.add_argument("--helper", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--rounds", type=int, default=3)
parser.add_argument("--qualities", default="0,17,80")
parser.add_argument("--encoded-corners", action="store_true", help="compare regular encoding with encoded source corners")
parser.add_argument("--glyph-blend", action="store_true", help="compare decoding stored corners with and without glyph blending")
args = parser.parse_args()
if args.rounds < 1:
    parser.error("rounds must be positive")
qualities = [int(q) for q in args.qualities.split(",")]
if not qualities or any(q < 0 or q > 100 for q in qualities):
    parser.error("qualities must be between 0 and 100")
helper = str(Path(args.helper).resolve())
env = os.environ.copy()
env["GOMAXPROCS"] = "8"
if args.glyph_blend and args.encoded_corners:
    parser.error("select one comparison")
modes = ("babe-cornerdata", "babe-glyphs") if args.glyph_blend else (("babe", "babe-cornerdata") if args.encoded_corners else ("babe", "babe-corners"))
cases = [(quality, mode) for quality in qualities for mode in modes]
measurements, bitstreams = [], {}
with tempfile.TemporaryDirectory(prefix="babe-corners-bench-") as directory:
    root = Path(directory)
    source, encoded, decoded = root / "source.ppm", root / "encoded", root / "decoded.ppm"
    subprocess.run([helper, "prepare", "source", str(Path(args.input).resolve()), str(source)], check=True, env=env)
    reference = ppm_hash(source)

    def cycle(quality, mode, run):
        print(f"{'Warm-up' if run == 0 else f'Round {run}/{args.rounds}'}: q{quality} {mode}", flush=True)
        start = time.perf_counter()
        encoder = "babe-cornerdata" if mode in ("babe-cornerdata", "babe-glyphs") else "babe"
        decoder = "babe" if mode == "babe-cornerdata" else mode
        enc = timed([helper, "encode", f"{encoder}:{quality}", str(source), str(encoded)], env)
        size = encoded.stat().st_size
        dec = timed([helper, "decode", decoder, str(encoded), str(decoded)], env)
        elapsed = time.perf_counter() - start
        output = ppm_hash(decoded)
        if output[:2] != reference[:2] or (quality == 100 and output != reference):
            raise ValueError("invalid decoded dimensions or lossless pixels")
        digest = hashlib.sha256(encoded.read_bytes()).hexdigest()
        key = (quality, encoder)
        if key in bitstreams and bitstreams[key] != digest:
            raise ValueError("encoded bitstream changed between cycles")
        bitstreams[key] = digest
        if run:
            measurements.append({"round": run, "quality": quality, "mode": mode,
                                 "encoded_bytes": size, "encode_wall_seconds": enc["wall"],
                                 "decode_wall_seconds": dec["wall"], "cycle_wall_seconds": elapsed,
                                 "cycle_cpu_seconds": enc["cpu"] + dec["cpu"],
                                 "peak_process_rss_bytes": max(enc["rss"], dec["rss"])})

    for case in cases:
        cycle(*case, 0)
    for run in range(1, args.rounds + 1):
        order = cases.copy()
        random.Random(4231 + run).shuffle(order)
        for case in order:
            cycle(*case, run)

Path(args.output).write_text(json.dumps({"source_dimensions": reference[:2], "rgb_sha256": reference[2],
                                         "rounds": args.rounds, "threads": 8, "measurements": measurements}, indent=2) + "\n")
for quality, mode in cases:
    rows = [m for m in measurements if m["quality"] == quality and m["mode"] == mode]
    med = lambda key: statistics.median(row[key] for row in rows)
    print(f"q{quality} {mode}: {med('encoded_bytes')} B, decode {med('decode_wall_seconds'):.4f} s, "
          f"cycle {med('cycle_wall_seconds'):.4f} s, CPU {med('cycle_cpu_seconds'):.4f} s, "
          f"RSS {med('peak_process_rss_bytes') / 1048576:.1f} MiB")
