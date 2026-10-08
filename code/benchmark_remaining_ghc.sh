#!/usr/bin/env bash
# Run the remaining GHC experiments for 15-418 Assignment 3.
#
# Run from this directory:
#   bash benchmark_remaining_ghc.sh
# Optional knobs, e.g. TRIALS=1 RUN_CACHE=0 bash benchmark_remaining_ghc.sh
#
# The script writes one complete, timestamped log under benchmark_results/.
# It also saves the two required medium-input routing outputs and plots.

set -u -o pipefail

TRIALS="${TRIALS:-3}"
RUN_CACHE="${RUN_CACHE:-1}"
SA_PROB="${SA_PROB:-0.1}"
SA_ITERS="${SA_ITERS:-5}"
BATCH_SIZE="${BATCH_SIZE:-1}"

if [[ ! "$TRIALS" =~ ^[1-9][0-9]*$ ]]; then
  echo "TRIALS must be a positive integer" >&2
  exit 2
fi
if [[ "$RUN_CACHE" != "0" && "$RUN_CACHE" != "1" ]]; then
  echo "RUN_CACHE must be 0 or 1" >&2
  exit 2
fi
if ! command -v perf >/dev/null 2>&1 && [[ "$RUN_CACHE" == "1" ]]; then
  echo "perf is unavailable; re-run with RUN_CACHE=0 or use a GHC machine with perf." >&2
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="benchmark_results/ghc_remaining_${STAMP}"
LOG="$OUT_DIR/full.log"
mkdir -p "$OUT_DIR/visuals"

# Give OpenMP a reproducible placement policy without overriding a user's
# explicit affinity mask.
export OMP_PROC_BIND="${OMP_PROC_BIND:-close}"
export OMP_PLACES="${OMP_PLACES:-cores}"

exec > >(tee -a "$LOG") 2>&1

echo "=== Assignment 3 remaining GHC benchmark ==="
echo "started: $(date -Is)"
echo "host: $(hostname)"
echo "cwd: $ROOT"
echo "git commit: $(git -C .. rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "TRIALS=$TRIALS RUN_CACHE=$RUN_CACHE SA_PROB=$SA_PROB SA_ITERS=$SA_ITERS BATCH_SIZE=$BATCH_SIZE"
echo "OMP_PROC_BIND=$OMP_PROC_BIND OMP_PLACES=$OMP_PLACES"
echo

make -B

run_case() {
  local suite="$1" input="$2" mode="$3" threads="$4" probability="$5" trial="$6"
  echo "=== CASE suite=$suite input=$input mode=$mode threads=$threads p=$probability trial=$trial ==="
  ./wireroute -f "$input" -n "$threads" -p "$probability" -i "$SA_ITERS" -m "$mode" -b "$BATCH_SIZE"
  local status=$?
  echo "=== CASE_STATUS=$status ==="
  echo
  return "$status"
}

cache_case() {
  local suite="$1" input="$2" mode="$3" threads="$4"
  echo "=== PERF_CACHE suite=$suite input=$input mode=$mode threads=$threads p=$SA_PROB ==="
  # perf sees all OpenMP workers in this process.  Divide total cache misses by
  # threads later to obtain the assignment's arithmetic mean per worker.
  perf stat -e cache-misses -- ./wireroute -f "$input" -n "$threads" -p "$SA_PROB" -i "$SA_ITERS" -m "$mode" -b "$BATCH_SIZE"
  local status=$?
  echo "=== PERF_CACHE_STATUS=$status ==="
  echo
  return "$status"
}

echo "### 1. Required core timing matrix: W/A x few/medium/abundant x 1/2/4/8"
for input in inputs/timeinput/few_wires.txt inputs/timeinput/medium_wires.txt inputs/timeinput/abundant_wires.txt; do
  for mode in W A; do
    for threads in 1 2 4 8; do
      for trial in $(seq 1 "$TRIALS"); do
        run_case core "$input" "$mode" "$threads" "$SA_PROB" "$trial" || exit $?
      done
    done
  done
done

if [[ "$RUN_CACHE" == "1" ]]; then
  echo "### 2. Required core cache-miss matrix (one representative perf run/configuration)"
  for input in inputs/timeinput/few_wires.txt inputs/timeinput/medium_wires.txt inputs/timeinput/abundant_wires.txt; do
    for mode in W A; do
      for threads in 1 2 4 8; do
        cache_case core "$input" "$mode" "$threads" || exit $?
      done
    done
  done
fi

echo "### 3. Required A-mode probability sweep: medium, p={0.01,0.1,0.5}, 1/8 threads"
for probability in 0.01 0.1 0.5; do
  for threads in 1 8; do
    for trial in $(seq 1 "$TRIALS"); do
      run_case probability inputs/timeinput/medium_wires.txt A "$threads" "$probability" "$trial" || exit $?
    done
  done
done

echo "### 4. Required A-mode problem-size sweeps: grid size and wire count, 1/8 threads"
for input in inputs/problemsize/gridsize/medium_512.txt inputs/problemsize/gridsize/medium_2048.txt inputs/problemsize/gridsize/medium_8192.txt; do
  for threads in 1 8; do
    for trial in $(seq 1 "$TRIALS"); do
      run_case gridsize "$input" A "$threads" "$SA_PROB" "$trial" || exit $?
    done
  done
done
for input in inputs/problemsize/numwires/medium_2048_256.txt inputs/problemsize/numwires/medium_2048_1024.txt inputs/problemsize/numwires/medium_2048_4096.txt; do
  for threads in 1 8; do
    for trial in $(seq 1 "$TRIALS"); do
      run_case numwires "$input" A "$threads" "$SA_PROB" "$trial" || exit $?
    done
  done
done

echo "### 5. Required routing visualizations: medium, 8 threads, W and A"
for mode in W A; do
  run_case visualization inputs/timeinput/medium_wires.txt "$mode" 8 "$SA_PROB" 1 || exit $?
  cp outputs/wire_output.txt "$OUT_DIR/visuals/medium_${mode}_wire_output.txt"
  cp outputs/occ_output.txt "$OUT_DIR/visuals/medium_${mode}_occ_output.txt"
  python3 plot_wires.py \
    --wires_output_file "$OUT_DIR/visuals/medium_${mode}_wire_output.txt" \
    --wires_output_plot "$OUT_DIR/visuals/medium_${mode}_wire_plot.png"
done

echo "=== COMPLETE ==="
echo "finished: $(date -Is)"
echo "log: $ROOT/$LOG"
echo "visuals: $ROOT/$OUT_DIR/visuals"
