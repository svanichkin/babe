#!/usr/bin/env python3
"""Run each warmed codec measurement in an isolated process and report medians."""
import argparse
import json
import random
import statistics
import subprocess
import sys
import tempfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True, help="built measurecodec executable")
    parser.add_argument("--input", required=True, help="source image")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--iterations", type=int, default=2)
    parser.add_argument("--output", help="write raw measurements as JSON")
    parser.add_argument("--qualities", default="100,80,50,10,0", help="comma-separated JPEG/BABE qualities")
    parser.add_argument("--codecs", default="jpeg,babe,qoi", help="comma-separated codec names")
    args = parser.parse_args()
    if args.runs < 1 or args.iterations < 1:
        parser.error("runs and iterations must be positive")
    binary = str(Path(args.binary).resolve())
    image = str(Path(args.input).resolve())
    try:
        qualities = [int(q) for q in args.qualities.split(",")]
    except ValueError:
        parser.error("qualities must be comma-separated integers")
    codecs = args.codecs.split(",")
    if not qualities or any(q < 0 or q > 100 for q in qualities) or any(c not in ("jpeg", "babe", "qoi") for c in codecs):
        parser.error("invalid quality or codec")
    cases = [(codec, quality) for quality in qualities for codec in ("jpeg", "babe") if codec in codecs]
    if "qoi" in codecs:
        cases.append(("qoi", 80))
    measurements = []
    with tempfile.TemporaryDirectory(prefix="babe-resource-") as directory:
        bitstreams = {}
        for codec, quality in cases:
            out = str(Path(directory) / f"{codec}-{quality}.bin")
            print(f"Preparing {codec} q{quality}", file=sys.stderr, flush=True)
            subprocess.run([binary, "-codec", codec, "-quality", str(quality), "-op", "prepare",
                            "-input", image, "-out", out], check=True)
            bitstreams[codec, quality] = out
        tasks = [(codec, quality, op) for codec, quality in cases for op in ("encode", "decode")]
        for run in range(args.runs):
            random.Random(379 + run).shuffle(tasks)
            for codec, quality, op in tasks:
                print(f"Run {run+1}/{args.runs}: {codec} q{quality} {op}", file=sys.stderr, flush=True)
                src = image if op == "encode" else bitstreams[codec, quality]
                data = subprocess.check_output([binary, "-codec", codec, "-quality", str(quality),
                                                "-op", op, "-input", src,
                                                "-iterations", str(args.iterations)], text=True)
                result = json.loads(data)
                result["run"] = run + 1
                measurements.append(result)
    if args.output:
        Path(args.output).write_text(json.dumps(measurements, indent=2) + "\n")

    def median(codec, quality, op, key):
        rows = (m for m in measurements if m["codec"] == codec and m["quality"] == quality and m["operation"] == op)
        if key == "cores":
            return statistics.median(m["cpu_seconds_per_op"] / m["wall_seconds_per_op"] for m in rows)
        return statistics.median(m[key] for m in rows)

    print("| Quality | Codec | CPU encode s | CPU decode s | Encode cores | Decode cores |")
    print("|---:|---|---:|---:|---:|---:|")
    for codec, quality in cases:
        q = "lossless" if codec == "qoi" else str(quality)
        e = median(codec, quality, "encode", "cpu_seconds_per_op")
        d = median(codec, quality, "decode", "cpu_seconds_per_op")
        ec = median(codec, quality, "encode", "cores")
        dc = median(codec, quality, "decode", "cores")
        print(f"| {q} | {codec.upper()} | {e:.3f} | {d:.3f} | {ec:.2f} | {dc:.2f} |")
    print("\n| Quality | Codec | Alloc encode MiB/op | Alloc decode MiB/op | Peak RSS encode MiB | Peak RSS decode MiB |")
    print("|---:|---|---:|---:|---:|---:|")
    for codec, quality in cases:
        q = "lossless" if codec == "qoi" else str(quality)
        values = [median(codec, quality, op, key) / (1024 * 1024)
                  for key in ("allocated_bytes_per_op", "peak_rss_bytes") for op in ("encode", "decode")]
        print(f"| {q} | {codec.upper()} | {values[0]:.2f} | {values[1]:.2f} | {values[2]:.1f} | {values[3]:.1f} |")


if __name__ == "__main__":
    main()
