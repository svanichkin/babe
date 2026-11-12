#!/usr/bin/env python3
"""Measure complete JPEG and BABE cycles for the README's matched-size images."""
import argparse
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
args = parser.parse_args()
if args.rounds < 1:
    parser.error("rounds must be positive")
helper = str(Path(args.helper).resolve())
source_path = str(Path(args.input).resolve())
env = os.environ.copy()
env["GOMAXPROCS"] = "8"
cases = [("JPEG", 1), ("BABE", 0), ("JPEG", 80), ("BABE", 17)]
measurements = []
with tempfile.TemporaryDirectory(prefix="babe-jpeg-bench-") as directory:
    root = Path(directory)
    source, encoded, decoded = root / "source.ppm", root / "encoded", root / "decoded.ppm"
    subprocess.run([helper, "prepare", "source", source_path, str(source)], check=True, env=env)
    reference = ppm_hash(source)

    def cycle(codec, quality, run):
        print(f"{'Warm-up' if run == 0 else f'Round {run}/{args.rounds}'}: {codec} q{quality}", flush=True)
        start = time.perf_counter()
        enc = timed([helper, "encode", f"{codec.lower()}:{quality}", str(source), str(encoded)], env)
        size = encoded.stat().st_size
        dec = timed([helper, "decode", codec.lower(), str(encoded), str(decoded)], env)
        elapsed = time.perf_counter() - start
        if ppm_hash(decoded)[:2] != reference[:2]:
            raise ValueError("decoded image dimensions differ from input")
        if run:
            measurements.append({"round": run, "codec": codec, "quality": quality,
                                 "encoded_bytes": size, "encode_wall_seconds": enc["wall"],
                                 "decode_wall_seconds": dec["wall"], "cycle_wall_seconds": elapsed,
                                 "cycle_cpu_seconds": enc["cpu"] + dec["cpu"],
                                 "peak_process_rss_bytes": max(enc["rss"], dec["rss"])})

    for case in cases:
        cycle(*case, 0)
    for run in range(1, args.rounds + 1):
        order = cases.copy()
        random.Random(7341 + run).shuffle(order)
        for case in order:
            cycle(*case, run)

Path(args.output).write_text(json.dumps({"source_dimensions": reference[:2], "rgb_sha256": reference[2],
                                         "rounds": args.rounds, "threads": 8, "load_average_after": os.getloadavg(),
                                         "measurements": measurements}, indent=2) + "\n")
for codec, quality in cases:
    rows = [m for m in measurements if m["codec"] == codec and m["quality"] == quality]
    med = lambda key: statistics.median(row[key] for row in rows)
    print(f"{codec} q{quality}: {med('encoded_bytes')} B, cycle {med('cycle_wall_seconds'):.4f} s, "
          f"CPU {med('cycle_cpu_seconds'):.4f} s, RSS {med('peak_process_rss_bytes') / 1048576:.1f} MiB")
