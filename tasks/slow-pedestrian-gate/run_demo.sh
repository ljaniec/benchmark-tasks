#!/usr/bin/env bash
set -euo pipefail

steps=${1:?Pass the control limit}
out=${2:?Pass a new output directory}
[[ "$steps" =~ ^[1-9][0-9]*$ ]] && (( steps <= 1000 ))
mkdir -- "$out"
out=$(cd -- "$out" && pwd)
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export OPENBLAS_NUM_THREADS=1 GATE_TRACE="$out/trace.jsonl" GATE_AUDIT_DIR="$out/audit"
sha256sum "$task_dir"/*.py "$task_dir"/driver-pickup-screen.json > "$out/source.sha256"
timeout -k 12 600 python "$task_dir/audit_gate_inspect.py" run \
    --embodiment yam_arena --policy yam_scripted \
    -E "task=$task_dir/inspect_gate_task.py" -E control_hz=20 -E object_jitter=0 \
    -E cam_width=320 -E cam_height=176 --seed 0 --max-steps "$steps" \
    --store-frames --scorer task_score --grader none --log-dir "$out/inspect" \
    --instruction "Slow the gate's main closing speed, release the hold-open hook, and let the gate swing into its latch on its own."
