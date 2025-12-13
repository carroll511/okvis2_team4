#!/usr/bin/env bash

# Batch-submit one or more trajectories: convert -> zip -> submit -> score.
# Pass a directory containing CSVs (matching *okvis2-slam-calib-final_trajectory.csv)
# or a single CSV path as $1. Defaults to exp15 directory if omitted.

set -euo pipefail

INPUT_PATH="${1:-/home/carroll/datasets_ros2/exp21}"
BASE_CONFIG="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/config/hilti_challenge_2022.yaml"
TRANSFORM_SCRIPT="/home/carroll/EE585/catkin_ws_okvis2/src/okvis2/transform_csv_to_txt.py"
REPORT_PDF="/home/carroll/Downloads/EE585_HILTI_vfinal.pdf"
CHALLENGE_YEAR="2022"

# If you can log in via cookie, set HILTI_COOKIE="name=value; another=value"
HILTI_COOKIE="${HILTI_COOKIE:-PHPSESSID=1a7b8fu3k4qov3lqkjvncl0s3tnp0cie}"
HILTI_USER="${HILTI_USER:-carroll@kaist.ac.kr}"
HILTI_PASS="${HILTI_PASS:-Tlswldb0830!}"
HILTI_COOKIE_OUT="${HILTI_COOKIE_OUT:-}"

SUBMISSION_ROOT="/home/carroll/EE585/tmp/hilti_submit"
TS=$(date +"%Y%m%d_%H%M%S")
BATCH_DIR="${SUBMISSION_ROOT}/${TS}"
SCORES_LOG="${BATCH_DIR}/scores.csv"

mkdir -p "${BATCH_DIR}"
mkdir -p "${SUBMISSION_ROOT}"

PATTERNS=("*okvis2-slam-calib-final_trajectory.csv")
# Allow falling back to live trajectory outputs when final_trajectory isn't present.
if [ "${ALLOW_LIVE_TRAJECTORY:-1}" -eq 1 ]; then
  PATTERNS+=("*okvis2-slam-calib-live_trajectory.csv")
fi

collect_csvs() {
  local search_path="$1"
  shift
  local patterns=("$@")
  local found=()
  for pat in "${patterns[@]}"; do
    while IFS= read -r f; do
      found+=("$f")
    done < <(find "${search_path}" -type f -name "${pat}")
  done
  # Sort for stable processing order
  printf "%s\n" "${found[@]}" | sort
}

if [ -d "${INPUT_PATH}" ]; then
  mapfile -t TRAJ_CSVS < <(collect_csvs "${INPUT_PATH}" "${PATTERNS[@]}")
elif [ -f "${INPUT_PATH}" ]; then
  TRAJ_CSVS=("${INPUT_PATH}")
else
  echo "[err] Input path not found: ${INPUT_PATH}"
  exit 1
fi

echo "dataset_base,source_csv,zip_path,score" > "${SCORES_LOG}"

if [ "${#TRAJ_CSVS[@]}" -eq 0 ]; then
  echo "[warn] No CSVs found under ${INPUT_PATH}; writing placeholder entry."
  echo "none,none,none,NO_SCORE" >> "${SCORES_LOG}"
  echo "[info] Done. Scores logged at ${SCORES_LOG}"
  exit 0
fi

echo "[info] Found ${#TRAJ_CSVS[@]} trajectories to submit."
echo "[info] Scores will be logged to ${SCORES_LOG}"

submit_and_score() {
  local zip_path="$1"
  local description="$2"
  python3 - "${zip_path}" "${description}" "${REPORT_PDF}" "${CHALLENGE_YEAR}" "${HILTI_COOKIE}" "${HILTI_USER}" "${HILTI_PASS}" "${HILTI_COOKIE_OUT}" <<'PY'
import os
import re
import sys
from datetime import datetime

zip_path, description, report_pdf, challenge, cookie, user, passwd, cookie_out = sys.argv[1:9]

def dbg(msg):
    print(msg, file=sys.stderr)
def no_score(msg):
    dbg(f"[err] {msg}")
    print("NO_SCORE")
    sys.exit(0)

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
    def save_cookie_if_requested(ctx):
        if not cookie_out:
            return
        for c in ctx.cookies():
            if c.get("name") == "PHPSESSID":
                try:
                    with open(cookie_out, "w", encoding="utf-8") as f:
                        f.write(f"PHPSESSID={c.get('value')}")
                    dbg(f"[dbg] Saved refreshed PHPSESSID to {cookie_out}")
                except Exception as ex:
                    dbg(f"[dbg] Failed to save cookie to {cookie_out}: {ex}")
                break

    def ensure_logged_in():
        if "login" not in page.url and not page.query_selector('input[name="email"]'):
            return True
        dbg("[dbg] On login page; attempting login flow...")
        if user and passwd:
            page.fill('input[name=\"email\"]', user)
            page.fill('input[name=\"password\"]', passwd)
            page.click('button[type=\"submit\"]')
            page.wait_for_load_state("networkidle")
            page.goto("https://submit.hilti-challenge.com/submission/new", wait_until="networkidle")
            return "login" not in page.url and not page.query_selector('input[name=\"email\"]')
        no_score("Not logged in; set HILTI_COOKIE or HILTI_USER/HILTI_PASS")

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
    if not ensure_logged_in():
        no_score("Login failed; adjust HILTI_COOKIE or HILTI_USER/HILTI_PASS")

    select = page.query_selector('select[name="challenge"]') or page.query_selector("select")
    if not select:
        # Retry login once in case cookie was stale.
        if ensure_logged_in():
            select = page.query_selector('select[name="challenge"]') or page.query_selector("select")
        if not select:
            no_score("Challenge select not found; check selectors/login.")
    dbg("[dbg] Selecting challenge...")
    select.select_option(challenge)
    save_cookie_if_requested(page.context)

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

best_score=""
best_label=""
best_zip=""

idx=0
for TRAJ_CSV in "${TRAJ_CSVS[@]}"; do
  idx=$((idx + 1))
  DATASET_DIR="$(dirname "${TRAJ_CSV}")"
  DATASET_BASE="$(basename "${DATASET_DIR%/}")"
  case "${DATASET_BASE}" in
    exp15) TRAJ_BASENAME="exp15_attic_to_upper_gallery" ;;
    exp02) TRAJ_BASENAME="exp02_construction_multilevel" ;;
    exp21) TRAJ_BASENAME="exp21_outside_building" ;;
    *) TRAJ_BASENAME="${DATASET_BASE}" ;;
  esac

  RUN_TAG="${TS}_$(printf "%02d" "${idx}")"
  RUN_DIR="${BATCH_DIR}/${DATASET_BASE}_${RUN_TAG}"
  RUN_CONFIG="${RUN_DIR}/${TRAJ_BASENAME}_config_${RUN_TAG}.yaml"
  TRAJ_TXT="${RUN_DIR}/${TRAJ_BASENAME}.txt"
  ZIP_OUT="${RUN_DIR}/${TRAJ_BASENAME}_${RUN_TAG}.zip"
  DESC="okvis2_${DATASET_BASE}_${RUN_TAG}"

  mkdir -p "${RUN_DIR}"

  if [ ! -f "${TRAJ_CSV}" ]; then
    echo "[warn] Missing trajectory CSV: ${TRAJ_CSV}; skipping."
    continue
  fi

  echo "[info] (${idx}/${#TRAJ_CSVS[@]}) Copying base config -> ${RUN_CONFIG}"
  cp "${BASE_CONFIG}" "${RUN_CONFIG}"

  echo "[info] (${idx}/${#TRAJ_CSVS[@]}) Converting CSV -> TXT for ${TRAJ_CSV}"
  if ! python3 "${TRANSFORM_SCRIPT}" "${TRAJ_CSV}" "${TRAJ_TXT}"; then
    echo "[warn] Conversion failed for ${TRAJ_CSV}; skipping."
    continue
  fi

  echo "[info] (${idx}/${#TRAJ_CSVS[@]}) Zipping (txt only) ${TRAJ_TXT} -> ${ZIP_OUT}"
  zip -j "${ZIP_OUT}" "${TRAJ_TXT}"

  echo "[info] (${idx}/${#TRAJ_CSVS[@]}) Submitting zip ${ZIP_OUT} (desc=${DESC})"
  score=$(submit_and_score "${ZIP_OUT}" "${DESC}")
  echo "[info] (${idx}/${#TRAJ_CSVS[@]}) score=${score}"

  echo "${DATASET_BASE},${TRAJ_CSV},${ZIP_OUT},${score}" >> "${SCORES_LOG}"

  if [[ "${score}" != "NO_SCORE" ]]; then
    if [ -z "${best_score}" ]; then
      best_score="${score}"
      best_label="${TRAJ_CSV}"
      best_zip="${ZIP_OUT}"
    elif (( $(echo "${score} > ${best_score}" | bc -l) )); then
      best_score="${score}"
      best_label="${TRAJ_CSV}"
      best_zip="${ZIP_OUT}"
    fi
  fi
done

echo "[info] Done. Scores logged at ${SCORES_LOG}"
if [ -n "${best_score}" ]; then
  echo "[info] Best score=${best_score} (source=${best_label}, zip=${best_zip})"
else
  echo "[info] No valid scores returned."
fi
