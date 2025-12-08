#!/usr/bin/env bash

# Automatic OKVIS2 hyperparameter tuning for ROS 2 Humble.
# The script runs an endless tuning loop per dataset, perturbing selected
# parameters, running OKVIS, aligning scale via transform_csv_to_txt.py,
# evaluating against ground truth, and persisting the best YAML found.

set -uo pipefail

# --- User settings (absolute paths) ---
BASE_CONFIG="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/config/hilti_challenge_2022_onlinecalib.yaml"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="/home/carroll/EE585/catkin_ws_okvis2/install/setup.bash"
# Stop when trajectories match the ground truth exactly, or when RMSE drops below this threshold (meters).
MATCH_RMSE_THRESHOLD="0.05"
# Each entry: name|dataset_path|ground_truth_txt
# dataset_path must be a directory (OKVIS writes trajectories there). If your source is a bag, play it separately.
DATASETS=(
  "exp02|/home/carroll/datasets_ros2/exp02|/home/carroll/EE585/gt/exp02_construction_multilevel.txt"
  "exp15|/home/carroll/datasets_ros2/exp15|/home/carroll/EE585/gt/exp15_attic_to_upper_gallery.txt"
  "exp21|/home/carroll/datasets_ros2/exp21|/home/carroll/EE585/gt/exp21_outside_building.txt"
)

# Output locations
LOG_ROOT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/tuning_logs"
RUN_ROOT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/tuning_runs"
BEST_ROOT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/config"

# --- Internal state ---
declare -A BEST_ERROR
declare -A BEST_CONFIG
declare -A JOB_NAME
declare -a JOB_PIDS

timestamp() { date +"%Y%m%d_%H%M%S"; }

# Ensure environments are loaded
if [ -f "${ROS_SETUP}" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${ROS_SETUP}"
  set -u
fi
if [ -f "${WORKSPACE_SETUP}" ]; then
  # shellcheck source=/dev/null
  set +u
  source "${WORKSPACE_SETUP}"
  set -u
fi

mkdir -p "${LOG_ROOT}" "${RUN_ROOT}" "${BEST_ROOT}"

randomize_config() {
  local base_config="$1"
  local out_config="$2"
  python3 - "${base_config}" "${out_config}" <<'PY'
import sys
import random
import re

base_path = sys.argv[1]
out_path = sys.argv[2]

# 튜닝할 파라미터와 허용되는 절댓값 변화량(delta), 최소/최대 안전 범위
params = {
    "detection_threshold": ("float", 3.0, 1.0, None),   # +/- 3 around base, keep >1
    "absolute_threshold": ("float", 2.0, 0.1, None),    # +/- 2, keep >0.1
    "matching_threshold": ("float", 3.0, 1.0, None),    # +/- 3, keep >1
    "max_num_keypoints": ("int", 40, 50, None),         # +/- 40, keep >50
    "keyframe_overlap": ("float", 0.03, 0.1, 0.9),      # +/- 0.03, clamp to [0.1,0.9]
}

fallback_defaults = {
    "detection_threshold": 50.0,
    "absolute_threshold": 20.0,
    "matching_threshold": 60.0,
    "max_num_keypoints": 700,
    "keyframe_overlap": 0.59,
}

def extract_base(lines, key):
    pattern = re.compile(rf"^\s*{re.escape(key)}\s*:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")
    for ln in lines:
        m = pattern.match(ln)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                return None
    return None

with open(base_path, "r") as f:
    lines = f.readlines()

# 한 번에 하나의 파라미터만 작은 폭으로 바꾼다 (불안정한 조합 회피)
rng = random.Random()
rng.seed()
target_key = rng.choice(list(params.keys()))
kind, delta, min_val, max_val = params[target_key]
base_val = extract_base(lines, target_key)
if base_val is None:
    base_val = float(fallback_defaults[target_key])

if kind == "int":
    lo = int(base_val - delta)
    hi = int(base_val + delta)
    lo = max(lo, int(min_val)) if min_val is not None else lo
    hi = min(hi, int(max_val)) if max_val is not None else hi
    new_val = rng.randint(lo, hi)
else:
    lo = base_val - delta
    hi = base_val + delta
    if min_val is not None:
        lo = max(lo, float(min_val))
    if max_val is not None:
        hi = min(hi, float(max_val))
    new_val = rng.uniform(lo, hi)
    new_val = float(f"{new_val:.6f}")

print(f"Randomizing '{target_key}' only:")
print(f"  base={base_val} new={new_val} (range {lo}..{hi})")

with open(out_path, "w") as f_out:
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            f_out.write(line)
            continue
        if stripped.startswith(target_key + ":"):
            indent = line[:line.find(target_key)]
            comment = ""
            if "#" in line:
                comment = " #" + line.split("#", 1)[1].strip()
            f_out.write(f"{indent}{target_key}: {new_val}{comment}\n")
        else:
            f_out.write(line)

PY
}

evaluate_traj() {
  local est_txt="$1"
  local gt_txt="$2"
  python3 - "${est_txt}" "${gt_txt}" <<'PY'
import math, sys

def load_traj(path):
    traj = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            t = float(parts[0])
            x, y, z = map(float, parts[1:4])
            traj.append((t, x, y, z))
    return traj

est = load_traj(sys.argv[1])
gt = load_traj(sys.argv[2])

if not est or not gt:
    print("inf")
    sys.exit(0)

# Nearest-neighbor time alignment with a 50 ms tolerance.
err_sq = 0.0
count = 0
j = 0
gt_len = len(gt)
for t, x, y, z in est:
    while j + 1 < gt_len and gt[j + 1][0] <= t:
        j += 1
    candidates = [gt[j]]
    if j + 1 < gt_len:
        candidates.append(gt[j + 1])
    best = min(candidates, key=lambda p: abs(p[0] - t))
    if abs(best[0] - t) > 0.05:
        continue
    dx = x - best[1]
    dy = y - best[2]
    dz = z - best[3]
    err_sq += dx * dx + dy * dy + dz * dz
    count += 1

if count < 10:
    print("inf")
else:
    rmse = math.sqrt(err_sq / count)
    print(f"{rmse:.6f}")
PY
}

tune_dataset() {
  local entry="$1"
  IFS="|" read -r name dataset_path gt_path <<< "${entry}"

  if [ ! -d "${dataset_path}" ]; then
    echo "[${name}] Dataset path must be a directory (OKVIS writes trajectories there). Current value: ${dataset_path}" >&2
    echo "[${name}] If you have a .bag, play it separately and set this to a writable directory."
    return
  fi
  if [ ! -f "${gt_path}" ]; then
    echo "[${name}] Ground truth missing: ${gt_path}" >&2
    return
  fi

  local log_dir="${LOG_ROOT}/${name}"
  local run_dir_root="${RUN_ROOT}/${name}"
  local best_path="${BEST_ROOT}/${name}_best.yaml"
  local history="${log_dir}/history.log"
  mkdir -p "${log_dir}" "${run_dir_root}"

  echo "[${name}] Starting tuning loop. Logs: ${log_dir}"

  local iter=0
  local use_base_config=1
  while true; do
    iter=$((iter + 1))
    local ts
    ts=$(timestamp)
    local run_dir="${run_dir_root}/${ts}"
    mkdir -p "${run_dir}"

    local run_config
    local config_source=""
    if [ "${use_base_config}" -eq 1 ]; then
      run_config="${run_dir}/${name}_${ts}_base.yaml"
      cp "${BASE_CONFIG}" "${run_config}"
      config_source="BASE_CONFIG"
      use_base_config=0
    else
      run_config="${run_dir}/${name}_${ts}.yaml"
      config_source="RANDOMIZED"
    fi
    local ros_log="${run_dir}/okvis.log"
    local traj_csv="${dataset_path}/okvis2-slam-calib-final_trajectory.csv"
    local traj_txt="${run_dir}/${name}_${ts}.txt"

    rm -f "${traj_csv}"

    echo "[${name}] Iter ${iter} -> generating parameters (${config_source})"
    echo "[${name}] Iter ${iter} run_config=${run_config}"
    echo "[${name}] Iter ${iter} dataset=${dataset_path}"
    if [ "${config_source}" = "RANDOMIZED" ]; then
      if ! randomize_config "${BASE_CONFIG}" "${run_config}" > "${run_dir}/config_randomization.log" 2>&1; then
        echo "[${name}] Failed to randomize config" | tee -a "${history}"
        continue
      fi
    fi

    echo "[${name}] Iter ${iter} -> running OKVIS (rviz disabled; waiting for fresh trajectory)"
    local start_ts
    start_ts=$(date +%s)
    if ! ros2 launch okvis okvis_node_synchronous.launch.xml rviz:=false config_filename:="${run_config}" path:="${dataset_path}" \
      > "${ros_log}" 2>&1; then
      echo "[${name}] Iter ${iter} OKVIS exited with error (see ${ros_log})" | tee -a "${history}"
    fi

    if [ -f "${traj_csv}" ]; then
      mtime=$(stat -c %Y "${traj_csv}")
      if [ "${mtime}" -lt "${start_ts}" ] || [ ! -s "${traj_csv}" ]; then
        echo "[${name}] Iter ${iter} trajectory CSV is stale or empty (see ${ros_log})" | tee -a "${history}"
        continue
      fi
    else
      echo "[${name}] Iter ${iter} missing trajectory CSV: ${traj_csv}" | tee -a "${history}"
      continue
    fi

    if [ ! -f "${traj_csv}" ]; then
      echo "[${name}] Iter ${iter} missing trajectory CSV: ${traj_csv}" | tee -a "${history}"
      continue
    fi

    echo "[${name}] Iter ${iter} -> aligning scale via transform_csv_to_txt.py"
    if ! python3 /home/carroll/EE585/catkin_ws_okvis2/src/okvis2/transform_csv_to_txt.py "${traj_csv}" "${traj_txt}" \
      >> "${ros_log}" 2>&1; then
      echo "[${name}] Iter ${iter} conversion failed" | tee -a "${history}"
      continue
    fi

    local files_identical=0
    if cmp -s "${traj_txt}" "${gt_path}"; then
      files_identical=1
      echo "[${name}] Iter ${iter} converted trajectory is identical to GT file."
    else
      echo "[${name}] Iter ${iter} -> evaluating against GT"
    fi

    local error
    if [ "${files_identical}" -eq 1 ]; then
      error="0.0"
    else
      error=$(evaluate_traj "${traj_txt}" "${gt_path}")
    fi

    if [[ "${error}" == "inf" ]]; then
      echo "[${name}] Iter ${iter} evaluation unusable (insufficient overlap)" | tee -a "${history}"
      continue
    fi

    echo "[${name}] Iter ${iter} error=${error}"
    echo "${ts},${iter},${error},${config_source},${run_config}" >> "${history}"

    if [[ -z "${BEST_ERROR[$name]+x}" ]]; then
      BEST_ERROR["${name}"]="${error}"
      BEST_CONFIG["${name}"]="${run_config}"
      cp "${run_config}" "${best_path}"
      echo "[${name}] New best (first) -> ${error} saved to ${best_path}"
    else
      local update_output
      update_output=$(python3 - "${error}" "${BEST_ERROR[$name]}" "${run_config}" "${best_path}" "${name}" <<'PY'
import math, shutil, sys
cur = float(sys.argv[1])
best = float(sys.argv[2])
run_config = sys.argv[3]
best_path = sys.argv[4]
name = sys.argv[5]
if cur < best:
    shutil.copy(run_config, best_path)
    print(f"[{name}] New best -> {cur} saved to {best_path}")
    print(f"UPDATE_BEST {cur}")
PY
)
      if grep -q "UPDATE_BEST" <<<"${update_output}"; then
        BEST_ERROR["${name}"]="${error}"
        BEST_CONFIG["${name}"]="${run_config}"
      fi
      if [ -n "${update_output}" ]; then
        echo "${update_output}"
      fi
    fi

    local matched=0
    if [ "${files_identical}" -eq 1 ]; then
      matched=1
      echo "[${name}] Iter ${iter} match achieved: converted file matches GT exactly."
    elif python3 - "${error}" "${MATCH_RMSE_THRESHOLD}" <<'PY'
import sys
cur = float(sys.argv[1])
th = float(sys.argv[2])
sys.exit(0 if cur <= th else 1)
PY
    then
      matched=1
      echo "[${name}] Iter ${iter} match achieved: RMSE ${error} <= threshold ${MATCH_RMSE_THRESHOLD}."
    fi

    if [ "${matched}" -eq 1 ]; then
      echo "[${name}] Stopping tuning loop after achieving match."
      echo "[${name}] Final best error=${BEST_ERROR[$name]} config=${BEST_CONFIG[$name]}"
      break
    fi
  done
}

cleanup() {
  echo "Stopping tuning jobs..."
  for pid in "${JOB_PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      echo "  killing ${JOB_NAME[${pid}]:-PID ${pid}} (PID ${pid})"
      kill "${pid}" 2>/dev/null
    fi
  done
  wait
}

trap cleanup SIGINT SIGTERM

for entry in "${DATASETS[@]}"; do
  IFS="|" read -r name _ _ <<< "${entry}"
  echo "[${name}] Launching tuning in background"
  ( tune_dataset "${entry}" ) &
  pid=$!
  JOB_PIDS+=("${pid}")
  JOB_NAME["${pid}"]="${name}"
done

for pid in "${JOB_PIDS[@]}"; do
  wait "${pid}"
  echo "[${JOB_NAME[${pid}]:-PID ${pid}}] Tuning job completed (PID ${pid})"
done
