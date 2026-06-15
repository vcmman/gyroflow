#!/usr/bin/env bash
# Reproducible evaluation of the C++ autosync-time port.
#
# Builds cpp_core, runs the unit tests, then exercises gyroflow_autosync against the DJI
# telemetry in ../data and prints a timestamp-sync precision table. See AUTOSYNC_TIME_TESTING.md
# for what each step checks and AUTOSYNC_TIME_REPORT.md for reference numbers.
#
# Usage:  cpp_core/tools/run_autosync_eval.sh [DATA_DIR]
#   DATA_DIR defaults to <repo>/data (sibling of cpp_core's parent).

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # cpp_core/tools
CORE="$(dirname "$HERE")"                              # cpp_core
BUILD="$CORE/build"

# Resolve the data dir: explicit arg, else cpp_core/../data, else the main git worktree's data/
# (the large DJI files usually live only in the main checkout, not in feature worktrees).
resolve_data() {
    local c
    for c in "${1:-}" "$(cd "$CORE/.." && pwd)/data" \
             "$(git -C "$CORE" worktree list 2>/dev/null | head -1 | awk '{print $1}')/data"; do
        [ -n "$c" ] && [ -f "$c/dji_quaternions_full.csv" ] && { echo "$c"; return 0; }
    done
    echo "${1:-$(cd "$CORE/.." && pwd)/data}"  # report the primary candidate for the error msg
}
DATA="$(resolve_data "${1:-}")"
QUAT="$DATA/dji_quaternions_full.csv"

echo "== cpp_core: $CORE"
echo "== data dir: $DATA"
[ -f "$QUAT" ] || { echo "ERROR: missing $QUAT"; exit 1; }

echo
echo "================ 1) Build ================"
cmake -S "$CORE" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$BUILD" -j"$(nproc 2>/dev/null || echo 4)" >/dev/null
echo "build OK"

echo
echo "================ 2) Unit tests ================"
# `ctest --test-dir` needs CMake >= 3.20; run from build/ for portability.
( cd "$BUILD" && ctest --output-on-failure )

echo
echo "================ 3) ω derivation cross-check ================"
echo "C++ omega (deg/s) first 2 intervals vs analysis CSV (rad/s; deg = rad*57.2958):"
"$BUILD/gyroflow_autosync" omega --quat "$QUAT" 2>/dev/null | sed -n '2,3p'
if [ -f "$DATA/dji_quat_analysis.csv" ]; then
    sed -n '2,3p' "$DATA/dji_quat_analysis.csv" | cut -d, -f1,8,9,10
fi

echo
echo "================ 4) Baseline bias (true offset = 0) ================"
for f in 30 60 120; do
    printf "fps=%-4s " "$f"
    "$BUILD/gyroflow_autosync" compare --quat "$QUAT" --fps "$f" --search 120 2>/dev/null | head -1
done

echo
echo "================ 5) Offset recovery precision (fps x noise) ================"
INJ="-50,-25,-10,-3,0,3,10,25,50,80"
printf "%-6s %-8s %-10s %-10s %-10s %-8s\n" fps noise mean_ms rms_ms max_ms recov
for fps in 24 30 60 120; do
    for noise in 0 1 3; do
        out="$("$BUILD/gyroflow_autosync" selftest --quat "$QUAT" --fps "$fps" --search 120 --inject "$INJ" --noise "$noise" 2>&1)"
        mean=$(echo "$out" | awk -F'= ' '/mean error/{print $2}' | tr -d ' ms')
        rms=$(echo  "$out" | awk -F'= ' '/RMS error/{print $2}'  | tr -d ' ms')
        mx=$(echo   "$out" | awk -F'= ' '/max .error/{print $2}' | tr -d ' ms')
        rec=$(echo  "$out" | awk -F'[ (/]' '/recovered\)/{print $5"/"$6}')
        printf "%-6s %-8s %-10s %-10s %-10s %-8s\n" "$fps" "$noise" "$mean" "$rms" "$mx" "$rec"
    done
done

echo
echo "All evaluation steps completed."
