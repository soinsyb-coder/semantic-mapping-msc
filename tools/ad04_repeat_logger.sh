#!/usr/bin/env bash

# Evidence logger for the three formal AD04 repeatability runs.
# ROS 2 generated setup files are not safe under nounset.
set -eo pipefail
set +u

SEMANTIC_WORKSPACE="${SEMANTIC_WORKSPACE:-$HOME/semantic_ws}"
SOURCE_DIR="$SEMANTIC_WORKSPACE/src/semantic_perception"
INSTALL_DIR="$SEMANTIC_WORKSPACE/install/semantic_perception"
EXPERIMENT_ROOT="$SEMANTIC_WORKSPACE/experiments"
STATE_FILE="${TMPDIR:-/tmp}/semantic_ad04_repeat_run_dir"

usage()
{
  cat <<'EOF'
Usage:
  ad04_repeat_logger.sh init R1|R2|R3
  ad04_repeat_logger.sh path
  ad04_repeat_logger.sh preflight
  ad04_repeat_logger.sh start-capture
  ad04_repeat_logger.sh mark TEXT...
  ad04_repeat_logger.sh snapshot LABEL
  ad04_repeat_logger.sh status
  ad04_repeat_logger.sh finish

Run init before launching ROS. Run preflight and start-capture after SLAM,
drift diagnostic and RGB-D localizer are stable, but before route motion.
EOF
}

source_ros()
{
  set +u
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  fi
  if [[ -f "$SEMANTIC_WORKSPACE/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$SEMANTIC_WORKSPACE/install/setup.bash"
  fi
}

load_run_dir()
{
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "No active AD04 repeat run." >&2
    exit 1
  fi
  RUN_DIR="$(sed -n '1p' "$STATE_FILE")"
  case "$RUN_DIR" in
    "$EXPERIMENT_ROOT"/*-AD04-R[123]) ;;
    *)
      echo "Refusing unexpected run directory: $RUN_DIR" >&2
      exit 1
      ;;
  esac
  [[ -d "$RUN_DIR" ]] || {
    echo "Run directory does not exist: $RUN_DIR" >&2
    exit 1
  }
}

append_pid()
{
  printf '%s %s\n' "$1" "$2" >> "$RUN_DIR/background_pids.txt"
}

write_source_metadata()
{
  {
    echo "Git status:"
    git -C "$SOURCE_DIR" status --short 2>&1 || true
    echo "Git commit:"
    git -C "$SOURCE_DIR" rev-parse HEAD 2>&1 || true
  } >> "$RUN_DIR/run_metadata.txt"

  find "$SOURCE_DIR" -maxdepth 4 -type f \
    \( -name '*.py' -o -name '*.yaml' -o -name 'setup.py' -o -name 'package.xml' \) \
    -print0 | sort -z | xargs -0 -r sha256sum \
    > "$RUN_DIR/source_sha256.txt"
}

command_init()
{
  local repeat="${2:-}"
  if [[ ! "$repeat" =~ ^R[123]$ ]]; then
    echo "Repeat must be R1, R2 or R3." >&2
    exit 1
  fi
  if [[ -e "$STATE_FILE" ]]; then
    echo "An AD04 repeat run is already active: $(sed -n '1p' "$STATE_FILE")" >&2
    exit 1
  fi

  source_ros
  mkdir -p "$EXPERIMENT_ROOT"
  local run_id start_iso
  run_id="$(date +%Y%m%d-%H%M%S)-AD04-$repeat"
  RUN_DIR="$EXPERIMENT_ROOT/$run_id"
  [[ ! -e "$RUN_DIR" ]] || {
    echo "Run directory already exists: $RUN_DIR" >&2
    exit 1
  }
  mkdir -p "$RUN_DIR"
  printf '%s\n' "$RUN_DIR" > "$STATE_FILE"
  start_iso="$(date --iso-8601=seconds)"

  {
    echo "Run ID: $run_id"
    echo "Repeat: $repeat"
    echo "Start: $start_iso"
    echo "Purpose: fixed-baseline AD04 semantic-fusion repeatability"
    echo "Scene: AWS RoboMaker Small House"
    echo "Baseline: YOLO11n CPU imgsz=640 detector=0.25"
    echo "Fusion: confidence=0.50 radius=0.60 m confirmation=3/10 max_missed=10"
    echo "Ground-truth region: 4 chairs and 1 dining table; at most 3 chairs visible in one view"
    echo "Policy: no parameter changes; motion input recorded on R1 and replayed on R2/R3"
    echo "Host: $(hostname)"
    echo "Kernel: $(uname -srmo)"
  } > "$RUN_DIR/run_metadata.txt"
  write_source_metadata
  printf '%s | INIT | AD04-%s directory created\n' "$start_iso" "$repeat" \
    > "$RUN_DIR/timeline.txt"
  : > "$RUN_DIR/background_pids.txt"
  echo "$RUN_DIR"
}

read_param()
{
  ros2 param get /rgbd_yolo_localizer_node "$1" 2>/dev/null |
    awk -F': ' 'NF {value=$NF} END {print value}'
}

command_preflight()
{
  load_run_dir
  source_ros
  rm -f "$RUN_DIR/preflight_passed.txt"
  : > "$RUN_DIR/preflight_parameters.txt"
  local failures=0 nodes topics item actual
  nodes="$(ros2 node list 2>/dev/null || true)"
  topics="$(ros2 topic list 2>/dev/null || true)"

  for item in /rtabmap/rtabmap /rgbd_yolo_localizer_node /slam_drift_diagnostic_node; do
    if ! grep -Fxq "$item" <<< "$nodes"; then
      echo "Missing node: $item" >&2
      failures=$((failures + 1))
    fi
  done
  for item in /clock /tf /tf_static /odom /map /camera/image /camera/depth_image /camera/camera_info /semantic_map/markers; do
    if ! grep -Fxq "$item" <<< "$topics"; then
      echo "Missing topic: $item" >&2
      failures=$((failures + 1))
    fi
  done

  while read -r item expected; do
    actual="$(read_param "$item" || true)"
    printf '%s=%s expected=%s\n' "$item" "$actual" "$expected" \
      >> "$RUN_DIR/preflight_parameters.txt"
    if [[ "$actual" != "$expected" ]]; then
      echo "Parameter mismatch: $item=$actual expected=$expected" >&2
      failures=$((failures + 1))
    fi
  done <<'EOF'
use_sim_time True
confidence 0.25
imgsz 640
fusion_min_confidence 0.5
fusion_min_valid_ratio 0.5
fusion_max_detection_distance_m 6.0
fusion_radius_m 0.6
fusion_confirmation_hits 3
fusion_confirmation_window 10
fusion_max_missed_frames 10
EOF

  actual="$(ros2 param get /slam_drift_diagnostic_node use_sim_time 2>/dev/null | awk -F': ' 'NF {value=$NF} END {print value}')"
  if [[ "$actual" != "True" ]]; then
    echo "Drift diagnostic use_sim_time=$actual expected=True" >&2
    failures=$((failures + 1))
  fi

  timeout 8 ros2 run tf2_ros tf2_echo map base_footprint \
    > "$RUN_DIR/preflight_tf_map_base.txt" 2>&1 || true
  if ! grep -q 'Translation:' "$RUN_DIR/preflight_tf_map_base.txt"; then
    echo "TF map->base_footprint unavailable" >&2
    failures=$((failures + 1))
  fi

  local source_file installed_file
  while read -r source_file installed_file; do
    if ! cmp -s "$source_file" "$installed_file"; then
      echo "Installed file mismatch: $source_file" >&2
      failures=$((failures + 1))
    fi
  done <<EOF
$SOURCE_DIR/config/rgbd_yolo_localizer.yaml $INSTALL_DIR/share/semantic_perception/config/rgbd_yolo_localizer.yaml
$SOURCE_DIR/semantic_perception/rgbd_yolo_localizer_node.py $INSTALL_DIR/lib/python3.12/site-packages/semantic_perception/rgbd_yolo_localizer_node.py
$SOURCE_DIR/semantic_perception/semantic_object_fusion.py $INSTALL_DIR/lib/python3.12/site-packages/semantic_perception/semantic_object_fusion.py
EOF

  printf '%s\n' "$nodes" > "$RUN_DIR/preflight_nodes.txt"
  ros2 topic list -t > "$RUN_DIR/preflight_topics.txt" 2>&1 || true

  if (( failures > 0 )); then
    printf '%s | PREFLIGHT_FAIL | %d issue(s)\n' "$(date --iso-8601=seconds)" "$failures" \
      >> "$RUN_DIR/timeline.txt"
    echo "Preflight failed with $failures issue(s)." >&2
    exit 1
  fi
  date --iso-8601=seconds > "$RUN_DIR/preflight_passed.txt"
  printf '%s | PREFLIGHT_PASS | nodes, topics, TF, parameters and installed files ready\n' \
    "$(date --iso-8601=seconds)" >> "$RUN_DIR/timeline.txt"
  echo "Preflight passed: $RUN_DIR"
}

command_start_capture()
{
  load_run_dir
  source_ros
  [[ -s "$RUN_DIR/preflight_passed.txt" ]] || {
    echo "Capture blocked: preflight has not passed." >&2
    exit 1
  }
  [[ ! -e "$RUN_DIR/capture_started.txt" ]] || {
    echo "Capture already started." >&2
    exit 1
  }

  ros2 topic echo /rosout rcl_interfaces/msg/Log > "$RUN_DIR/rosout.log" 2>&1 &
  append_pid rosout "$!"

  (
    while true; do
      echo "===== $(date --iso-8601=seconds) ====="
      free -m | sed -n '1,2p'
      ps -eo pid,pcpu,pmem,etime,stat,comm,args --sort=-pcpu | sed -n '1,25p'
      sleep 5
    done
  ) > "$RUN_DIR/system_health.log" 2>&1 &
  append_pid health "$!"

  if command -v vmstat >/dev/null 2>&1; then
    vmstat -t 1 > "$RUN_DIR/vmstat.log" 2>&1 &
    append_pid vmstat "$!"
  fi

  local topic
  local -a requested=(
    /clock /cmd_vel /map /odom /rosout /rtabmap/info /scan
    /semantic_map/markers /tf /tf_static /camera/camera_info
  )
  local -a available=()
  local topic_list
  topic_list="$(ros2 topic list)"
  for topic in "${requested[@]}"; do
    if grep -Fxq "$topic" <<< "$topic_list"; then
      available+=("$topic")
    fi
  done
  printf '%s\n' "${available[@]}" > "$RUN_DIR/rosbag_topics.txt"
  ros2 bag record -s mcap -o "$RUN_DIR/rosbag_core" "${available[@]}" \
    > "$RUN_DIR/rosbag_console.log" 2>&1 &
  local rosbag_pid="$!"
  append_pid rosbag "$rosbag_pid"
  sleep 2
  if ! kill -0 "$rosbag_pid" 2>/dev/null; then
    echo "rosbag failed to remain running; inspect rosbag_console.log" >&2
    stop_capture
    exit 1
  fi
  date --iso-8601=seconds > "$RUN_DIR/capture_started.txt"
  printf '%s | CAPTURE_START | AD04 core evidence capture\n' "$(date --iso-8601=seconds)" \
    >> "$RUN_DIR/timeline.txt"
  echo "Capture started: $RUN_DIR"
}

command_mark()
{
  load_run_dir
  shift
  [[ -s "$RUN_DIR/preflight_passed.txt" && -s "$RUN_DIR/capture_started.txt" ]] || {
    echo "Mark blocked: preflight and capture are required." >&2
    exit 1
  }
  (( $# > 0 )) || {
    echo "Mark text is required." >&2
    exit 1
  }
  printf '%s | MARK | %s\n' "$(date --iso-8601=seconds)" "$*" |
    tee -a "$RUN_DIR/timeline.txt"
}

command_snapshot()
{
  load_run_dir
  source_ros
  local label="${2:-snapshot}" safe directory
  safe="$(tr -cs '[:alnum:]_-' '_' <<< "$label" | sed 's/_$//')"
  directory="$RUN_DIR/snapshot_$(date +%H%M%S)_$safe"
  mkdir -p "$directory"
  ros2 node list > "$directory/nodes.txt" 2>&1 || true
  ros2 topic list -t > "$directory/topics.txt" 2>&1 || true
  timeout 10 ros2 topic echo /semantic_map/markers --once \
    > "$directory/semantic_markers.yaml" 2>&1 || true
  timeout 8 ros2 run tf2_ros tf2_echo map base_footprint \
    > "$directory/tf_map_base.txt" 2>&1 || true
  timeout 8 ros2 run tf2_ros tf2_echo map odom \
    > "$directory/tf_map_odom.txt" 2>&1 || true
  printf '%s | SNAPSHOT | %s | %s\n' "$(date --iso-8601=seconds)" "$label" "$directory" |
    tee -a "$RUN_DIR/timeline.txt"
}

stop_capture()
{
  [[ -f "$RUN_DIR/background_pids.txt" ]] || return 0
  local name pid deadline running
  while read -r name pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $name (PID $pid)"
      kill -INT "$pid" 2>/dev/null || true
    fi
  done < "$RUN_DIR/background_pids.txt"
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    running=0
    while read -r name pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      if kill -0 "$pid" 2>/dev/null; then
        running=1
        break
      fi
    done < "$RUN_DIR/background_pids.txt"
    (( running == 0 )) && break
    sleep 1
  done
  while read -r name pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done < "$RUN_DIR/background_pids.txt"
}

command_status()
{
  load_run_dir
  echo "Run directory: $RUN_DIR"
  while read -r name pid; do
    [[ -n "${pid:-}" ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name: running (PID $pid)"
    else
      echo "$name: stopped (PID $pid)"
    fi
  done < "$RUN_DIR/background_pids.txt"
}

command_finish()
{
  load_run_dir
  echo "Finish 1/5: stopping capture"
  stop_capture

  echo "Finish 2/5: collecting final RTAB-Map database"
  mkdir -p "$RUN_DIR/rtabmap_databases"
  if [[ -f "$HOME/.ros/rtabmap.db" ]]; then
    cp --reflink=auto "$HOME/.ros/rtabmap.db" \
      "$RUN_DIR/rtabmap_databases/rtabmap.db"
  fi

  echo "Finish 3/5: writing hashes"
  printf '%s | FINISH | capture stopped and archive prepared\n' "$(date --iso-8601=seconds)" \
    >> "$RUN_DIR/timeline.txt"
  find "$RUN_DIR" -maxdepth 2 -type f \
    ! -path '*/rosbag_core/*' ! -name artifact_sha256.txt -print0 |
    sort -z | xargs -0 -r sha256sum > "$RUN_DIR/artifact_sha256.txt"

  echo "Finish 4/5: compressing run"
  local archive partial
  archive="$HOME/Downloads/$(basename "$RUN_DIR").tar.gz"
  partial="$archive.partial"
  tar -C "$(dirname "$RUN_DIR")" -czf "$partial" "$(basename "$RUN_DIR")"

  echo "Finish 5/5: verifying archive"
  gzip -t "$partial"
  tar -tzf "$partial" >/dev/null
  mv -f "$partial" "$archive"
  rm -f "$STATE_FILE"
  echo "$archive"
}

case "${1:-}" in
  init) command_init "$@" ;;
  path) load_run_dir; echo "$RUN_DIR" ;;
  preflight) command_preflight ;;
  start-capture) command_start_capture ;;
  mark) command_mark "$@" ;;
  snapshot) command_snapshot "$@" ;;
  status) command_status ;;
  finish) command_finish ;;
  *) usage; exit 1 ;;
esac
