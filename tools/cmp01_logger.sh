#!/usr/bin/env bash

# Evidence logger for CMP01 condition A: SLAM-only.
# Reference condition B is the validated AD04-R3 SLAM+YOLO run.
# ROS 2 generated setup files are not safe under nounset.
set -eo pipefail
set +u

SEMANTIC_WORKSPACE="${SEMANTIC_WORKSPACE:-$HOME/semantic_ws}"
SOURCE_DIR="$SEMANTIC_WORKSPACE/src/semantic_perception"
INSTALL_DIR="$SEMANTIC_WORKSPACE/install/semantic_perception"
EXPERIMENT_ROOT="$SEMANTIC_WORKSPACE/experiments"
STATE_FILE="${TMPDIR:-/tmp}/semantic_cmp01_slamonly_run_dir"
AD04_STATE_FILE="${TMPDIR:-/tmp}/semantic_ad04_repeat_run_dir"
REFERENCE_RUN="${CMP01_REFERENCE_RUN:-$EXPERIMENT_ROOT/20260816-212856-AD04-R3}"
REPLAY_SOURCE="${CMP01_REPLAY_SOURCE:-$SEMANTIC_WORKSPACE/tools/ad04_replay_r1_cmd_vel.py}"
EXPECTED_REPLAY_SHA256="ee7e89ce9fe03ea28cf2774df4f17d18a04065c44b2f90d2128f735af8d22b61"

usage()
{
  cat <<'EOF'
Usage:
  cmp01_logger.sh init
  cmp01_logger.sh path
  cmp01_logger.sh preflight
  cmp01_logger.sh start-capture
  cmp01_logger.sh mark TEXT...
  cmp01_logger.sh snapshot LABEL
  cmp01_logger.sh status
  cmp01_logger.sh finish

Run init before launching ROS. Run preflight and start-capture after SLAM and
the drift diagnostic are stable. The RGB-D YOLO localizer, Nav2 and
explore_lite must remain stopped for the complete run.
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
    echo "No active CMP01 SLAM-only run." >&2
    exit 1
  fi
  RUN_DIR="$(sed -n '1p' "$STATE_FILE")"
  case "$RUN_DIR" in
    "$EXPERIMENT_ROOT"/*-CMP01-SLAMONLY) ;;
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

write_source_hashes()
{
  find "$SOURCE_DIR" -maxdepth 4 -type f \
    \( -name '*.py' -o -name '*.yaml' -o -name 'setup.py' -o -name 'package.xml' \) \
    -print0 | sort -z | xargs -0 -r sha256sum
}

localizer_processes()
{
  pgrep -af \
    '[r]gbd_yolo_localizer_node|[r]gbd_yolo_localizer.launch.py|[y]olo_detector_node' \
    || true
}

navigation_processes()
{
  pgrep -af '[e]xplore_lite|[e]xplore_node|[n]av2_' || true
}

core_processes()
{
  pgrep -af \
    '[r]tabmap_slam/rtabmap|[r]gbd_yolo_localizer_node|[s]lam_drift_diagnostic_node|[r]os2 launch|[g]z sim' \
    || true
}

semantic_topic_info()
{
  if ros2 topic list 2>/dev/null | grep -Fxq /semantic_map/markers; then
    ros2 topic info /semantic_map/markers --verbose 2>&1 || true
  else
    echo "/semantic_map/markers is absent"
  fi
}

semantic_publisher_count()
{
  semantic_topic_info | awk -F': ' \
    '/Publisher count:/ {print $2; found=1} END {if (!found) print 0}'
}

assert_slam_only_runtime()
{
  local failures=0 count
  if [[ -n "$(localizer_processes)" ]]; then
    echo "SLAM-only violation: localizer process is running." >&2
    localizer_processes >&2
    failures=$((failures + 1))
  fi
  if ros2 node list 2>/dev/null | grep -Fxq /rgbd_yolo_localizer_node; then
    echo "SLAM-only violation: /rgbd_yolo_localizer_node exists." >&2
    failures=$((failures + 1))
  fi
  count="$(semantic_publisher_count)"
  if [[ ! "$count" =~ ^[0-9]+$ ]]; then
    echo "SLAM-only check failed: invalid semantic publisher count: $count" >&2
    failures=$((failures + 1))
  elif (( count > 0 )); then
    echo "SLAM-only violation: /semantic_map/markers has $count publisher(s)." >&2
    failures=$((failures + 1))
  fi
  if [[ -n "$(navigation_processes)" ]]; then
    echo "Controlled-route violation: Nav2 or explore process is running." >&2
    navigation_processes >&2
    failures=$((failures + 1))
  fi
  (( failures == 0 ))
}

command_init()
{
  if (( $# != 1 )); then
    echo "CMP01 init takes no condition argument." >&2
    usage
    exit 1
  fi
  if [[ -e "$STATE_FILE" ]]; then
    echo "A CMP01 run is already active: $(sed -n '1p' "$STATE_FILE")" >&2
    exit 1
  fi
  if [[ -e "$AD04_STATE_FILE" ]]; then
    echo "An AD04 run state is still active: $(sed -n '1p' "$AD04_STATE_FILE")" >&2
    exit 1
  fi
  [[ -f "$REFERENCE_RUN/source_sha256.txt" ]] || {
    echo "Reference source baseline is missing: $REFERENCE_RUN/source_sha256.txt" >&2
    exit 1
  }
  [[ -f "$REPLAY_SOURCE" ]] || {
    echo "Replay script is missing: $REPLAY_SOURCE" >&2
    exit 1
  }
  local replay_hash
  replay_hash="$(sha256sum "$REPLAY_SOURCE" | awk '{print $1}')"
  [[ "$replay_hash" == "$EXPECTED_REPLAY_SHA256" ]] || {
    echo "Replay hash mismatch: $replay_hash" >&2
    exit 1
  }
  if [[ -n "$(core_processes)" ]]; then
    echo "Relevant ROS/Gazebo processes are still running:" >&2
    core_processes >&2
    exit 1
  fi

  source_ros
  mkdir -p "$EXPERIMENT_ROOT"
  local run_id start_iso logger_path
  run_id="$(date +%Y%m%d-%H%M%S)-CMP01-SLAMONLY"
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
    echo "Condition: A - SLAM-only"
    echo "Reference condition: B - AD04-R3 SLAM+YOLO"
    echo "Reference run: $REFERENCE_RUN"
    echo "Start: $start_iso"
    echo "Purpose: controlled comparison of SLAM-only against frozen SLAM+YOLO baseline"
    echo "Scene: AWS RoboMaker Small House"
    echo "Motion: exact AD04-R1 ten-event cmd_vel schedule replayed using simulation time"
    echo "Exclusions: no RGB-D YOLO localizer, semantic fusion, Nav2 or explore_lite"
    echo "Analysis policy: compare the formal route simulation-time window, not whole-run database counts"
    echo "Host: $(hostname)"
    echo "Kernel: $(uname -srmo)"
    echo "Git status:"
    git -C "$SOURCE_DIR" status --short 2>&1 || true
    echo "Git commit:"
    git -C "$SOURCE_DIR" rev-parse HEAD 2>&1 || true
  } > "$RUN_DIR/run_metadata.txt"

  write_source_hashes > "$RUN_DIR/source_sha256.txt"
  if ! cmp -s "$REFERENCE_RUN/source_sha256.txt" "$RUN_DIR/source_sha256.txt"; then
    echo "Source baseline differs from AD04-R3." >&2
    diff -u "$REFERENCE_RUN/source_sha256.txt" "$RUN_DIR/source_sha256.txt" \
      > "$RUN_DIR/source_sha256.diff" || true
    printf '%s | INIT_ABORT | source baseline differs from AD04-R3\n' "$start_iso" \
      > "$RUN_DIR/timeline.txt"
    rm -f "$STATE_FILE"
    exit 1
  fi

  install -m 0755 "$REPLAY_SOURCE" "$RUN_DIR/ad04_replay_r1_cmd_vel.py"
  sha256sum "$RUN_DIR/ad04_replay_r1_cmd_vel.py" \
    > "$RUN_DIR/replay_script_sha256.txt"
  logger_path="$(readlink -f "$0")"
  install -m 0755 "$logger_path" "$RUN_DIR/cmp01_logger.sh"
  sha256sum "$RUN_DIR/cmp01_logger.sh" > "$RUN_DIR/logger_sha256.txt"

  printf '%s | INIT | CMP01 SLAM-only directory created; R3 source and replay frozen\n' \
    "$start_iso" > "$RUN_DIR/timeline.txt"
  : > "$RUN_DIR/background_pids.txt"
  echo "$RUN_DIR"
}

command_preflight()
{
  load_run_dir
  source_ros
  rm -f "$RUN_DIR/preflight_passed.txt"
  : > "$RUN_DIR/preflight_parameters.txt"
  local failures=0 nodes topics item actual source_now
  nodes="$(ros2 node list 2>/dev/null || true)"
  topics="$(ros2 topic list 2>/dev/null || true)"

  for item in /rtabmap/rtabmap /slam_drift_diagnostic_node; do
    if ! grep -Fxq "$item" <<< "$nodes"; then
      echo "Missing node: $item" >&2
      failures=$((failures + 1))
    fi
  done
  for item in /clock /cmd_vel /tf /tf_static /odom /map /scan /rtabmap/info /camera/image /camera/depth_image /camera/camera_info /rosout; do
    if ! grep -Fxq "$item" <<< "$topics"; then
      echo "Missing topic: $item" >&2
      failures=$((failures + 1))
    fi
  done

  if ! assert_slam_only_runtime; then
    failures=$((failures + 1))
  fi

  actual="$(ros2 param get /slam_drift_diagnostic_node use_sim_time 2>/dev/null | awk -F': ' 'NF {value=$NF} END {print value}')"
  printf 'drift_use_sim_time=%s expected=True\n' "$actual" \
    >> "$RUN_DIR/preflight_parameters.txt"
  if [[ "$actual" != "True" ]]; then
    echo "Drift diagnostic use_sim_time=$actual expected=True" >&2
    failures=$((failures + 1))
  fi

  actual="$(ros2 param get /rtabmap/rtabmap use_sim_time 2>/dev/null | awk -F': ' 'NF {value=$NF} END {print value}')"
  printf 'rtabmap_use_sim_time=%s expected=True\n' "$actual" \
    >> "$RUN_DIR/preflight_parameters.txt"
  if [[ "$actual" != "True" ]]; then
    echo "RTAB-Map use_sim_time=$actual expected=True" >&2
    failures=$((failures + 1))
  fi

  printf 'localizer_node_present=%s expected=False\n' \
    "$(grep -Fxq /rgbd_yolo_localizer_node <<< "$nodes" && echo True || echo False)" \
    >> "$RUN_DIR/preflight_parameters.txt"
  printf 'semantic_marker_publishers=%s expected=0\n' \
    "$(semantic_publisher_count)" >> "$RUN_DIR/preflight_parameters.txt"

  if ! pgrep -af '/rtabmap_slam/rtabmap.*--delete_db_on_start' >/dev/null; then
    echo "RTAB-Map process does not show --delete_db_on_start." >&2
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

  source_now="$(mktemp)"
  write_source_hashes > "$source_now"
  if ! cmp -s "$RUN_DIR/source_sha256.txt" "$source_now"; then
    echo "Source files changed after CMP01 init." >&2
    failures=$((failures + 1))
  fi
  rm -f "$source_now"

  printf '%s\n' "$nodes" > "$RUN_DIR/preflight_nodes.txt"
  ros2 topic list -t > "$RUN_DIR/preflight_topics.txt" 2>&1 || true
  semantic_topic_info > "$RUN_DIR/preflight_semantic_absence.txt"

  if (( failures > 0 )); then
    printf '%s | PREFLIGHT_FAIL | %d issue(s)\n' "$(date --iso-8601=seconds)" "$failures" \
      >> "$RUN_DIR/timeline.txt"
    echo "Preflight failed with $failures issue(s)." >&2
    exit 1
  fi
  date --iso-8601=seconds > "$RUN_DIR/preflight_passed.txt"
  printf '%s | PREFLIGHT_PASS | SLAM and drift ready; localizer, semantic publisher and navigation absent\n' \
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
  assert_slam_only_runtime || exit 1

  local topic topic_list
  local -a requested=(
    /clock /cmd_vel /map /odom /rosout /rtabmap/info /scan
    /tf /tf_static /camera/camera_info
  )
  local -a available=()
  topic_list="$(ros2 topic list)"
  for topic in "${requested[@]}"; do
    if ! grep -Fxq "$topic" <<< "$topic_list"; then
      echo "Capture blocked: required topic is missing: $topic" >&2
      exit 1
    fi
    available+=("$topic")
  done

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
  printf '%s | CAPTURE_START | CMP01 SLAM-only core evidence capture\n' \
    "$(date --iso-8601=seconds)" >> "$RUN_DIR/timeline.txt"
  echo "Capture started: $RUN_DIR"
}

command_mark()
{
  load_run_dir
  source_ros
  shift
  [[ -s "$RUN_DIR/preflight_passed.txt" && -s "$RUN_DIR/capture_started.txt" ]] || {
    echo "Mark blocked: preflight and capture are required." >&2
    exit 1
  }
  (( $# > 0 )) || {
    echo "Mark text is required." >&2
    exit 1
  }
  assert_slam_only_runtime || exit 1
  printf '%s | MARK | %s\n' "$(date --iso-8601=seconds)" "$*" |
    tee -a "$RUN_DIR/timeline.txt"
}

command_snapshot()
{
  load_run_dir
  source_ros
  [[ -s "$RUN_DIR/preflight_passed.txt" && -s "$RUN_DIR/capture_started.txt" ]] || {
    echo "Snapshot blocked: preflight and capture are required." >&2
    exit 1
  }
  assert_slam_only_runtime || exit 1
  local label="${2:-snapshot}" safe directory
  safe="$(tr -cs '[:alnum:]_-' '_' <<< "$label" | sed 's/_$//')"
  directory="$RUN_DIR/snapshot_$(date +%H%M%S)_$safe"
  mkdir -p "$directory"
  ros2 node list > "$directory/nodes.txt" 2>&1 || true
  ros2 topic list -t > "$directory/topics.txt" 2>&1 || true
  semantic_topic_info > "$directory/semantic_absence.txt"
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

  deadline=$((SECONDS + 10))
  while (( SECONDS < deadline )); do
    running=0
    while read -r name pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      if kill -0 "$pid" 2>/dev/null; then
        running=1
        break
      fi
    done < "$RUN_DIR/background_pids.txt"
    (( running == 0 )) && return 0
    sleep 1
  done

  echo "Capture processes did not stop cleanly:" >&2
  while read -r name pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "$name (PID $pid)" >&2
    fi
  done < "$RUN_DIR/background_pids.txt"
  return 1
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
  source_ros
  local remaining remaining_nodes
  remaining="$(core_processes)"
  if [[ -n "$remaining" ]]; then
    echo "Finish blocked: core processes must exit before database collection:" >&2
    printf '%s\n' "$remaining" >&2
    exit 1
  fi
  remaining_nodes="$(ros2 node list 2>/dev/null | grep -E \
    '^/(robot_state_publisher|ros_gz_bridge|slam_drift_diagnostic_node|rgbd_yolo_localizer_node)$|^/rtabmap(/|_)' \
    || true)"
  if [[ -n "$remaining_nodes" ]]; then
    echo "Finish blocked: core ROS nodes must exit before database collection:" >&2
    printf '%s\n' "$remaining_nodes" >&2
    exit 1
  fi

  echo "Finish 1/6: stopping capture"
  stop_capture

  echo "Finish 2/6: collecting final RTAB-Map database"
  mkdir -p "$RUN_DIR/rtabmap_databases"
  [[ -f "$HOME/.ros/rtabmap.db" ]] || {
    echo "RTAB-Map database is missing: $HOME/.ros/rtabmap.db" >&2
    exit 1
  }
  cp --reflink=auto "$HOME/.ros/rtabmap.db" \
    "$RUN_DIR/rtabmap_databases/rtabmap.db"

  echo "Finish 3/6: checking collected database"
  python3 - "$RUN_DIR/rtabmap_databases/rtabmap.db" \
    > "$RUN_DIR/database_check.txt" <<'PY'
import sqlite3
import sys

path = sys.argv[1]
connection = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
result = connection.execute("PRAGMA quick_check").fetchone()[0]
print("quick_check:", result)
if result != "ok":
    raise SystemExit("database quick_check failed")
for table in ("Node", "Link", "Statistics"):
    count = connection.execute(f'SELECT COUNT(*) FROM "{table}"').fetchone()[0]
    print(f"{table}: {count}")
connection.close()
PY

  echo "Finish 4/6: writing hashes"
  printf '%s | FINISH | controlled shutdown verified; capture stopped and archive prepared\n' \
    "$(date --iso-8601=seconds)" >> "$RUN_DIR/timeline.txt"
  find "$RUN_DIR" -maxdepth 2 -type f \
    ! -path '*/rosbag_core/*' ! -name artifact_sha256.txt -print0 |
    sort -z | xargs -0 -r sha256sum > "$RUN_DIR/artifact_sha256.txt"

  echo "Finish 5/6: compressing run"
  local archive partial
  archive="$HOME/Downloads/$(basename "$RUN_DIR").tar.gz"
  partial="$archive.partial"
  tar -C "$(dirname "$RUN_DIR")" -czf "$partial" "$(basename "$RUN_DIR")"

  echo "Finish 6/6: verifying archive"
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
