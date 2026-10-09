#!/usr/bin/env bash
set -euo pipefail
cd /home/nkb/colcon_ws
source install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
python=/home/nkb/.cache/tc2026-graph-slam-venv/bin/python3
"$python" scripts/offline_graph_slam/extract.py
"$python" scripts/offline_graph_slam/build_keyframes.py
"$python" scripts/offline_graph_slam/solve_graph.py
"$python" scripts/offline_graph_slam/refine_loops.py
"$python" scripts/offline_graph_slam/evaluate_map.py
"$python" scripts/offline_graph_slam/render.py
"$python" scripts/offline_graph_slam/validate_loops.py
"$python" scripts/offline_graph_slam/build_report.py
