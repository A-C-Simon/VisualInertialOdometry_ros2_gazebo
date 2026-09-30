# Shared launcher helpers; HW290_DIR and RUN_LABEL must be set before sourcing.
mkdir -p "${HW290_DIR}/../benchmark/results"
RUN_DIR=$(mktemp -d "${HW290_DIR}/../benchmark/results/${RUN_LABEL}_$(date +%Y%m%d_%H%M%S)_XXXXXX")
date +%s.%N > "$RUN_DIR/started.txt"
PIDS=()
ROLES=()
LOGS=()
start_node() {
  local role=$1 log=$2
  shift 2
  python3 "$HW290_DIR/managed_process.py" "$RUN_DIR/$role.resources.json" "$@" > "$log" 2>&1 &
  PIDS+=("$!"); ROLES+=("$role"); LOGS+=("$log")
}
start_hw290_imu() {
  local log=$1
  local raw_args=()
  [[ "${DIAGNOSTICS:-false}" == true ]] && raw_args=(--ros-args -p raw_log_path:="$RUN_DIR/imu_raw.txt")
  case "${HW290_IMU_BACKEND:-cpp}" in
    cpp) start_node imu "$log" "$HW290_DIR/../install_vio/ov_hw290/lib/ov_hw290/hw290_imu" "${raw_args[@]}" ;;
    python) start_node imu "$log" python3 "$HW290_DIR/hw290_imu.py" "${raw_args[@]}" ;;
    *) echo "HW290_IMU_BACKEND must be cpp or python" >&2; return 2 ;;
  esac
}
cleanup() {
  local status=$?
  trap - INT TERM EXIT
  for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
  for p in "${PIDS[@]}"; do wait "$p" 2>/dev/null || true; done
  for i in "${!LOGS[@]}"; do cp "${LOGS[$i]}" "$RUN_DIR/${ROLES[$i]}.log" 2>/dev/null || true; done
  python3 "$HW290_DIR/run_summary.py" "$RUN_DIR" | tee "$RUN_DIR/summary.txt"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait_for_imu() {
  local log=$1
  echo "Waiting for verified 80-120 Hz IMU measurements..."
  for ((attempt=0; attempt<90; attempt++)); do
    if grep -Eq 'IMU_READY: verified' "$log"; then return 0; fi
    if grep -Eq 'ERROR IMU setup failed|Traceback|IMU startup timed out|\[FATAL\]' "$log"; then
      echo 'IMU startup failed: check Nano/IMU power and SDA/SCL wiring. RViz has no world frame until VIO produces poses.' >&2
      tail -8 "$log" >&2
      return 1
    fi
    sleep 0.2
  done
  echo "No IMU output within 18 seconds. Inspect $log" >&2
  return 1
}

wait_for_run() {
  local finished status i
  local remaining=("${PIDS[@]}")
  while ((${#remaining[@]})); do
    status=0
    wait -n -p finished "${remaining[@]}" 2>/dev/null || status=$?
    [[ -n "${finished:-}" ]] || return 0
    for i in "${!PIDS[@]}"; do
      if [[ "${PIDS[$i]}" == "$finished" ]]; then
        [[ "${ROLES[$i]}" == rviz ]] && return "$status"
        echo "${ROLES[$i]} stopped (exit $status). See ${LOGS[$i]}." >&2
      fi
    done
    [[ "$SHOW_RVIZ" == true ]] || return "$status"
    echo 'RViz remains open for diagnostics. Ctrl-C or close RViz to stop and print the summary.' >&2
    # Sensor failure invalidates VIO; keep camera/TF/RViz diagnostics available.
    for i in "${!PIDS[@]}"; do
      if [[ "${ROLES[$i]}" == estimator ]]; then kill -TERM "${PIDS[$i]}" 2>/dev/null || true; fi
    done
    local next=()
    for i in "${remaining[@]}"; do [[ "$i" == "$finished" ]] || next+=("$i"); done
    remaining=("${next[@]}")
  done
}
