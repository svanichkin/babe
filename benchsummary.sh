#!/usr/bin/env bash
set -euo pipefail

# Prints only the summary table (no go test noise).
# Usage:
#   ./benchsummary.sh
#   GOCACHE="$PWD/.gocache" ./benchsummary.sh

GOCACHE="${GOCACHE:-$PWD/.gocache}"
export GOCACHE

# Use `-count=1` to disable Go test result caching; this summary measures runtime.
go test -count=1 -run TestBenchmarkSummary -v ./... 2>/dev/null | awk '
  BEGIN { printing=0 }
  /^codec[[:space:]]/ { printing=1 }
  printing && /^--- PASS:/ { printing=0 }
  printing { print }
'
