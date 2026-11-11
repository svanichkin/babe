#!/usr/bin/env python3
"""Compare complete lossless encode/decode cycles in separate CLI processes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import statistics
import subprocess
import sys
import tempfile
import time


def timed(command, env):
    start = time.perf_counter()
    proc = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    _, status, usage = os.wait4(proc.pid, 0)
    elapsed = time.perf_counter() - start
    proc.returncode = os.waitstatus_to_exitcode(status)
    if proc.returncode:
        raise RuntimeError(f"command failed ({proc.returncode}): {command!r}")
    rss = usage.ru_maxrss if sys.platform == "darwin" else usage.ru_maxrss * 1024
    return {"wall": elapsed, "cpu": usage.ru_utime + usage.ru_stime, "rss": rss}


def ppm_hash(path):
    # Ignore harmless PPM header formatting; compare dimensions and RGB bytes.
    with open(path, "rb") as f:
        def token():
            result = bytearray()
            while True:
                b = f.read(1)
                if not b:
                    raise ValueError(f"truncated PPM header: {path}")
                if b == b"#" and not result:
                    f.readline()
                    continue
                if b in b" \t\r\n":
                    if result:
                        return bytes(result)
                    continue
                result.extend(b)
                if len(result) > 32:
                    raise ValueError(f"invalid PPM header: {path}")
        magic, width, height, maximum = (token() for _ in range(4))
        if magic != b"P6" or maximum != b"255":
            raise ValueError(f"not 8-bit RGB PPM: {path}")
        width, height = int(width), int(height)
        count, digest = 0, hashlib.sha256()
        while block := f.read(1 << 20):
            count += len(block)
            digest.update(block)
        if count != width * height * 3:
            raise ValueError(f"incorrect PPM data length: {path}")
        return width, height, digest.hexdigest()


def version(command):
    p = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=True)
    return p.stdout.splitlines()[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="source image; decoded once to opaque RGB")
    parser.add_argument("--helper", required=True, help="built cmd/losslessbench executable")
    parser.add_argument("--oxipng", default="oxipng")
    parser.add_argument("--cjxl", default="cjxl")
    parser.add_argument("--djxl", default="djxl")
    parser.add_argument("--cwebp", default="cwebp")
    parser.add_argument("--dwebp", default="dwebp")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--output", help="write individual measurements to JSON")
    args = parser.parse_args()
    if args.threads < 1 or args.rounds < 1:
        parser.error("threads and rounds must be positive")
    helper = str(Path(args.helper).resolve())
    image = str(Path(args.input).resolve())
    env = os.environ.copy()
    env["GOMAXPROCS"] = str(args.threads)
    names = ("BABE", "QOI", "JPEG XL", "PNG + OxiPNG", "WebP")
    versions = {
        "cjxl": version([args.cjxl, "--version"]), "djxl": version([args.djxl, "--version"]),
        "cwebp": version([args.cwebp, "-version"]), "dwebp": version([args.dwebp, "-version"]),
        "oxipng": version([args.oxipng, "--version"]),
    }
    measurements = []
    with tempfile.TemporaryDirectory(prefix="babe-lossless-bench-") as directory:
        root = Path(directory)
        source = root / "source.ppm"
        subprocess.run([helper, "prepare", "source", image, str(source)], check=True, env=env)
        reference = ppm_hash(source)

        def cycle(name, round_number):
            print(f"{'Warm-up' if round_number == 0 else f'Round {round_number}/{args.rounds}'}: {name}", file=sys.stderr, flush=True)
            ext = {"BABE": "babe", "QOI": "qoi", "JPEG XL": "jxl", "PNG + OxiPNG": "png", "WebP": "webp"}[name]
            encoded, decoded, rawpng = root / f"output.{ext}", root / "decoded.ppm", root / "raw.png"
            for p in (encoded, decoded, rawpng):
                p.unlink(missing_ok=True)
            if name == "JPEG XL":
                enc = [[args.cjxl, str(source), str(encoded), "-d", "0", "-e", "5", f"--num_threads={args.threads}", "--quiet"]]
                dec = [args.djxl, str(encoded), str(decoded), f"--num_threads={args.threads}", "--quiet"]
            elif name == "WebP":
                enc = [[args.cwebp, "-lossless", "-q", "75", "-m", "4", "-mt", "-quiet", str(source), "-o", str(encoded)]]
                dec = [args.dwebp, str(encoded), "-ppm", "-mt", "-quiet", "-o", str(decoded)]
            elif name == "PNG + OxiPNG":
                enc = [[helper, "encode", "png", str(source), str(rawpng)],
                       [args.oxipng, "-o", "2", "-q", "-t", str(args.threads), "--strip", "safe", "--out", str(encoded), str(rawpng)]]
                dec = [helper, "decode", "png", str(encoded), str(decoded)]
            else:
                codec = name.lower()
                enc = [[helper, "encode", codec, str(source), str(encoded)]]
                dec = [helper, "decode", codec, str(encoded), str(decoded)]
            cycle_start = time.perf_counter()
            stages = [timed(command, env) for command in enc]
            size = encoded.stat().st_size
            decode = timed(dec, env)
            cycle_wall = time.perf_counter() - cycle_start
            if ppm_hash(decoded) != reference:
                raise ValueError(f"{name} round {round_number}: decoded pixels differ from input")
            result = {
                "round": round_number, "codec": name, "encoded_bytes": size,
                "encode_wall_seconds": sum(s["wall"] for s in stages),
                "decode_wall_seconds": decode["wall"], "cycle_wall_seconds": cycle_wall,
                "cycle_cpu_seconds": sum(s["cpu"] for s in stages) + decode["cpu"],
                "peak_process_rss_bytes": max([s["rss"] for s in stages] + [decode["rss"]]),
            }
            if round_number:
                measurements.append(result)

        # Warm up once, then run every codec once in each independently shuffled round.
        for name in names:
            cycle(name, 0)
        for round_number in range(1, args.rounds + 1):
            order = list(names)
            random.Random(1489 + round_number).shuffle(order)
            for name in order:
                cycle(name, round_number)

    result = {"source_dimensions": reference[:2], "rgb_sha256": reference[2],
              "rounds": args.rounds, "threads": args.threads, "versions": versions,
              "settings": {"BABE": "quality 100, opaque RGB", "QOI": "lossless",
                           "JPEG XL": "distance 0, effort 5", "PNG + OxiPNG": "Go PNG default + oxipng -o2 --strip safe",
                           "WebP": "lossless, quality 75, method 4, multithreaded"},
              "measurements": measurements}
    if args.output:
        Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
    print("| Format | Encode s | Decode s | Full cycle s | CPU s/cycle | Peak process RSS MiB | Size MB |")
    print("|---|---:|---:|---:|---:|---:|---:|")
    for name in names:
        rows = [m for m in measurements if m["codec"] == name]
        median = lambda key: statistics.median(row[key] for row in rows)
        print(f"| {name} | {median('encode_wall_seconds'):.3f} | {median('decode_wall_seconds'):.3f} | "
              f"{median('cycle_wall_seconds'):.3f} | {median('cycle_cpu_seconds'):.3f} | "
              f"{median('peak_process_rss_bytes')/1048576:.1f} | {median('encoded_bytes')/1000000:.2f} |")


if __name__ == "__main__":
    main()
