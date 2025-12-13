#!/usr/bin/env bash

# Automatic OKVIS2 hyperparameter tuning with online Hilti submission.
# WARNING: This is a template. You must supply login/session details and may
# need to adjust the Playwright selectors to match the site.

set -euo pipefail

# --- User settings (absolute paths) ---
BASE_CONFIG="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/config/hilti_challenge_2022.yaml"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="/home/carroll/EE585/catkin_ws_okvis2/install/setup.bash"
REPORT_PDF="/home/carroll/Downloads/EE585_HILTI_vfinal.pdf"
TARGET_SCORE="${TARGET_SCORE:-10}"
CHALLENGE_YEAR="2022"
ROS_LOG_DIR="/home/carroll/EE585/tmp/hilti_ros_logs"
LIBTORCH_LIB="/home/carroll/EE585/catkin_ws_okvis2/libtorch/lib"

# If you can log in via cookie, set HILTI_COOKIE="name=value; another=value"
# Otherwise set HILTI_USER / HILTI_PASS and adjust selectors in submit_and_score().
# Defaulting to provided PHPSESSID; override by exporting HILTI_COOKIE.
HILTI_COOKIE="${HILTI_COOKIE:-PHPSESSID=}"
HILTI_USER="${HILTI_USER:-}"
HILTI_PASS="${HILTI_PASS:-}"

# Each entry: name|dataset_path
DATASETS=(
  "exp02|/home/carroll/datasets_ros2/exp02"
  "exp15|/home/carroll/datasets_ros2/exp15"
  "exp21|/home/carroll/datasets_ros2/exp21"
)

# Output locations
LOG_ROOT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/tuning_logs_online"
RUN_ROOT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/tuning_runs_online"
BEST_ROOT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/config"
SUBMISSION_ROOT="/home/carroll/EE585/tmp/hilti_submissions"

timestamp() { date +"%Y%m%d_%H%M%S"; }

# Ensure environments are loaded. Clear ZSH_VERSION so ROS picks bash scripts.
unset ZSH_VERSION
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

mkdir -p "${LOG_ROOT}" "${RUN_ROOT}" "${BEST_ROOT}" "${SUBMISSION_ROOT}"
mkdir -p "${ROS_LOG_DIR}"

# Ensure dependent libs are visible and ROS logs go to a writable location
export LD_LIBRARY_PATH="${LIBTORCH_LIB}:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="${ROS_LOG_DIR}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

randomize_config() {
  local base_config="$1"
  local out_config="$2"
  python3 - "${base_config}" "${out_config}" <<'PY'
import sys
import random
import re

base_path = sys.argv[1]
out_path = sys.argv[2]

params = {
    "detection_threshold": ("float", 3.0, 1.0, None),   # +/- 3 around base, keep >1
    "absolute_threshold": ("float", 2.0, 0.1, None),    # +/- 2, keep >0.1
    "matching_threshold": ("float", 3.0, 1.0, None),    # +/- 3, keep >1
    "max_num_keypoints": ("int", 40, 50, None),         # +/- 40, keep >50
    "keyframe_overlap": ("float", 0.03, 0.1, 0.9),      # +/- 0.03, clamp to [0.1,0.9]
    "num_keyframes": ("int", 3, 1, None),
    "num_loop_closure_frames": ("int", 3, 1, None),
    "num_imu_frames": ("int", 2, 1, None),
    "full_graph_iterations": ("int", 6, 1, None),
}

fallback_defaults = {
    "detection_threshold": 50.0,
    "absolute_threshold": 20.0,
    "matching_threshold": 60.0,
    "max_num_keypoints": 700,
    "keyframe_overlap": 0.59,
    "num_keyframes": 5,
    "num_loop_closure_frames": 5,
    "num_imu_frames", 3,
    "full_graph_iterations": 15,
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
    if lo > hi:
        lo = hi = int(base_val)
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

run_okvis() {
  local dataset_path="$1"
  local run_dir="$2"
  local run_config="$3"
  local ros_log="${run_dir}/okvis.log"
  local traj_csv="${dataset_path}/okvis2-slam-calib-final_trajectory.csv"
  local traj_base
  case "${dataset_path%/}" in
    */exp02) traj_base="exp02_construction_multilevel" ;;
    */exp15) traj_base="exp15_attic_to_upper_gallery" ;;
    */exp21) traj_base="exp21_outside_building" ;;
    *) traj_base="trajectory_${RUN_TS}" ;;
  esac
  local traj_txt="${run_dir}/${traj_base}.txt"

  rm -f "${traj_csv}"

  echo "[run] Running OKVIS on ${dataset_path}"
  local start_ts
  start_ts=$(date +%s)
  if ! ros2 launch okvis okvis_node_synchronous.launch.xml rviz:=false config_filename:="${run_config}" path:="${dataset_path}" \
    > "${ros_log}" 2>&1; then
    echo "[run] OKVIS exited with error (see ${ros_log})"
  fi

  if [ ! -f "${ros_log}" ]; then
    echo "[run] Missing ros log ${ros_log}; skipping iteration."
    return 1
  fi

  if ! grep -qiE "(converged|convergence|99%)" "${ros_log}"; then
    echo "[run] OKVIS log lacks completion markers ('converged/convergence/99%'); skipping iteration."
    return 1
  fi

  if [ ! -f "${traj_csv}" ] || [ ! -s "${traj_csv}" ]; then
    echo "[run] Missing or empty trajectory CSV: ${traj_csv}"
    return 1
  fi
  local mtime
  mtime=$(stat -c %Y "${traj_csv}")
  if [ "${mtime}" -lt "${start_ts}" ]; then
    echo "[run] Trajectory CSV appears stale: ${traj_csv}"
    return 1
  fi

  echo "[run] Converting CSV -> TXT"
  if ! python3 /home/carroll/EE585/catkin_ws_okvis2/src/okvis2/transform_csv_to_txt.py "${traj_csv}" "${traj_txt}" \
    >> "${ros_log}" 2>&1; then
    echo "[run] Conversion failed (see ${ros_log})"
    return 1
  fi

  echo "${traj_txt}"
}

build_zip() {
  local traj_txt="$1"
  local run_config="$2"
  local zip_out="$3"
  zip -j "${zip_out}" "${traj_txt}" "${run_config}"
}

submit_and_score() {
  local zip_path="$1"
  local description="$2"
  python3 - "${zip_path}" "${description}" "${REPORT_PDF}" "${CHALLENGE_YEAR}" "${HILTI_COOKIE}" "${HILTI_USER}" "${HILTI_PASS}" <<'PY'
import os
import re
import sys
from datetime import datetime

zip_path, description, report_pdf, challenge, cookie, user, passwd = sys.argv[1:8]

def dbg(msg):
    print(msg, file=sys.stderr)

try:
    from playwright.sync_api import sync_playwright
except ImportError:
    dbg("Playwright not installed. Run: pip install playwright && playwright install chromium")
    sys.exit(2)

def parse_score(page):
    dbg("[dbg] Navigating to submission/mine to parse score...")
    page.goto("https://submit.hilti-challenge.com/submission/mine", wait_until="networkidle")
    tabs = page.query_selector_all("a, button")
    for t in tabs:
        txt = t.inner_text().strip()
        if txt == "2022":
            dbg("[dbg] Clicking 2022 tab...")
            t.click()
            page.wait_for_timeout(1000)
            break
    rows = page.query_selector_all("table tbody tr")
    parsed = []
    for row in rows:
        cells = [c.inner_text().strip() for c in row.query_selector_all("td")]
        if cells:
            parsed.append(cells)
    dbg(f"[dbg] Parsed rows: {parsed}")
    for cells in reversed(parsed):
        try:
            return float(cells[-1])
        except Exception:
            continue
    return None

with sync_playwright() as p:
    browser = p.chromium.launch(headless=True)
    page = browser.new_page()
    if cookie:
        dbg("[dbg] Injecting cookies...")
        for chunk in cookie.split(";"):
            if "=" not in chunk:
                continue
            name, value = chunk.split("=", 1)
            page.context.add_cookies([{
                "name": name.strip(),
                "value": value.strip(),
                "domain": "submit.hilti-challenge.com",
                "path": "/",
                "httpOnly": True,
                "secure": True,
            }])
    dbg("[dbg] Opening submission form...")
    page.goto("https://submit.hilti-challenge.com/submission/new", wait_until="networkidle")
    if "login" in page.url or page.query_selector('input[name="email"]'):
        dbg("[dbg] On login page; attempting login flow...")
        if user and passwd:
            page.fill('input[name="email"]', user)
            page.fill('input[name="password"]', passwd)
            page.click('button[type="submit"]')
            page.wait_for_load_state("networkidle")
            page.goto("https://submit.hilti-challenge.com/submission/new", wait_until="networkidle")
        elif not cookie:
            raise RuntimeError("Not logged in; set HILTI_COOKIE or HILTI_USER/HILTI_PASS")

    select = page.query_selector('select[name="challenge"]') or page.query_selector("select")
    if not select:
        raise RuntimeError("Challenge select not found; check if login succeeded.")
    dbg("[dbg] Selecting challenge...")
    select.select_option(challenge)

    dbg("[dbg] Uploading solution zip...")
    sol_input = page.query_selector('input[type="file"][name*="solution"]') or page.query_selector('input[type="file"]')
    if sol_input:
        sol_input.set_input_files(zip_path)
    dbg("[dbg] Uploading report PDF...")
    rep_input = page.query_selector('input[type="file"][name*="report"]')
    if not rep_input:
        inputs = page.query_selector_all('input[type="file"]')
        if len(inputs) > 1:
            rep_input = inputs[-1]
    if rep_input:
        rep_input.set_input_files(report_pdf)

    dbg("[dbg] Checking sensors...")
    for sensor in ("Camera", "IMU"):
        loc = page.get_by_label(sensor) if hasattr(page, "get_by_label") else None
        if loc:
            loc.check()
        else:
            el = page.query_selector(f'text="{sensor}"')
            if el:
                el.click()

    dbg(f"[dbg] Filling description: {description}")
    filled = False
    for selector in ('textarea[name="description"]', "textarea", 'input[name="description"]', 'input[type="text"]'):
        el = page.query_selector(selector)
        if el:
            el.fill(description)
            filled = True
            break
    if not filled:
        raise RuntimeError("Description field not found; update selector.")

    dbg("[dbg] Submitting form...")
    btn = page.query_selector('button[type="submit"]')
    if not btn and hasattr(page, "get_by_role"):
        btn = page.get_by_role("button", name=re.compile("upload|submit", re.I))
    if not btn:
        raise RuntimeError("Submit button not found; adjust selector.")
    btn.click()
    dbg("[dbg] Waiting for server to process (40s)...")
    page.wait_for_timeout(40000)

    score = parse_score(page)
    browser.close()

if score is None:
    print("NO_SCORE")
else:
    print(score)
PY
}

tune_dataset() {
  local entry="$1"
  IFS="|" read -r name dataset_path <<< "${entry}"
  if [ ! -d "${dataset_path}" ]; then
    echo "[${name}] dataset path missing: ${dataset_path}"
    return
  fi

  local log_dir="${LOG_ROOT}/${name}"
  local run_dir_root="${RUN_ROOT}/${name}"
  local best_path="${BEST_ROOT}/${name}_best_online.yaml"
  mkdir -p "${log_dir}" "${run_dir_root}"

  local best_score=-1
  local iter=0
  while true; do
    iter=$((iter + 1))
    RUN_TS=$(timestamp)
    local dataset_base
    dataset_base=$(basename "${dataset_path%/}")
    local run_dir="${run_dir_root}/${RUN_TS}"
    mkdir -p "${run_dir}"

    local run_config="${run_dir}/${name}_${RUN_TS}.yaml"
    if [ "${iter}" -eq 1 ]; then
      cp "${BASE_CONFIG}" "${run_config}"
    else
      randomize_config "${BASE_CONFIG}" "${run_config}" > "${run_dir}/config_randomization.log" 2>&1
    fi

    local traj_txt
    if ! traj_txt=$(RUN_TS="${RUN_TS}" run_okvis "${dataset_path}" "${run_dir}" "${run_config}"); then
      echo "[${name}] Iter ${iter} failed run; skipping"
      continue
    fi

    local zip_out="${SUBMISSION_ROOT}/${dataset_base}_${RUN_TS}.zip"
    build_zip "${traj_txt}" "${run_config}" "${zip_out}"

    local desc="okvis2_${name}_${RUN_TS}"
    echo "[${name}] Iter ${iter} submitting ${zip_out} desc=${desc}"
    local score
    score=$(submit_and_score "${zip_out}" "${desc}")

    if [[ "${score}" == "NO_SCORE" ]]; then
      echo "[${name}] Iter ${iter} submission did not return a score."
      continue
    fi
    echo "[${name}] Iter ${iter} score=${score}"

    if (( $(echo "${score} > ${best_score}" | bc -l) )); then
      best_score="${score}"
      cp "${run_config}" "${best_path}"
      echo "[${name}] New best score ${best_score} saved to ${best_path}"
    fi

    if (( $(echo "${score} >= ${TARGET_SCORE}" | bc -l) )); then
      echo "[${name}] Target score reached (${score} >= ${TARGET_SCORE}); stopping."
      break
    fi
  done
}

declare -a TUNE_PIDS
declare -A TUNE_NAME
for entry in "${DATASETS[@]}"; do
  IFS="|" read -r name _ <<< "${entry}"
  echo "[${name}] launching tuning in background"
  ( tune_dataset "${entry}" ) &
  pid=$!
  TUNE_PIDS+=("${pid}")
  TUNE_NAME["${pid}"]="${name}"
done

for pid in "${TUNE_PIDS[@]}"; do
  wait "${pid}"
  echo "[${TUNE_NAME[${pid}]:-job}] completed (PID ${pid})"
done
