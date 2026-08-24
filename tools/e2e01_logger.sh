#!/usr/bin/env bash

# Keep command/pipeline failure handling strict, but deliberately leave
# `nounset` disabled. ROS 2 Jazzy's generated setup chain reads several
# optional AMENT_* variables directly and is not safe to source under `set -u`.
set -eo pipefail
set +u

SEMANTIC_WORKSPACE="${SEMANTIC_WORKSPACE:-$HOME/semantic_ws}"
SEMANTIC_PACKAGE_DIR="$SEMANTIC_WORKSPACE/src/semantic_perception"
BRINGUP_PACKAGE_DIR="$SEMANTIC_WORKSPACE/src/semantic_mapping_bringup"
EXPERIMENT_ROOT="$SEMANTIC_WORKSPACE/experiments"
STATE_FILE="${TMPDIR:-/tmp}/semantic_e2e01_run_dir"

usage()
{
  cat <<'EOF'
Usage:
  e2e01_logger.sh init
  e2e01_logger.sh path
  e2e01_logger.sh preflight
  e2e01_logger.sh start-capture
  e2e01_logger.sh mark TEXT...
  e2e01_logger.sh snapshot LABEL
  e2e01_logger.sh status
  e2e01_logger.sh finish

Run init before launching ROS. Run preflight and start-capture after SLAM,
the RGB-D localizer and the drift diagnostic are ready, but before starting
autonomous exploration.
EOF
}

source_ros()
{
  # Keep nounset off throughout the generated ROS and workspace setup chain.
  # Do not work around individual AMENT_* variables one by one: different
  # installations can reference different optional variables.
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

require_command()
{
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Required command not found: $1" >&2
    exit 1
  fi
}

load_run_dir()
{
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "No active E2E01 run. Execute: $0 init" >&2
    exit 1
  fi
  RUN_DIR="$(sed -n '1p' "$STATE_FILE")"
  case "$RUN_DIR" in
    "$EXPERIMENT_ROOT"/*) ;;
    *)
      echo "Refusing unexpected run directory: $RUN_DIR" >&2
      exit 1
      ;;
  esac
  if [[ ! -d "$RUN_DIR" ]]; then
    echo "Run directory does not exist: $RUN_DIR" >&2
    exit 1
  fi
}

write_git_metadata()
{
  local package_dir="$1"
  local label="$2"
  {
    echo "[$label]"
    if git -C "$package_dir" rev-parse --is-inside-work-tree \
      >/dev/null 2>&1; then
      echo "Commit: $(git -C "$package_dir" rev-parse HEAD)"
      echo "Branch: $(git -C "$package_dir" branch --show-current)"
      echo "Status:"
      git -C "$package_dir" status --short
      echo "Diff stat:"
      git -C "$package_dir" diff --stat
    else
      echo "Not a Git worktree: $package_dir"
    fi
  } >> "$RUN_DIR/run_metadata.txt"
}

write_source_hashes()
{
  : > "$RUN_DIR/source_sha256.txt"
  local source_dir
  for source_dir in "$SEMANTIC_PACKAGE_DIR" "$BRINGUP_PACKAGE_DIR"; do
    if [[ -d "$source_dir" ]]; then
      find "$source_dir" -maxdepth 4 -type f \
        \( -name '*.py' -o -name '*.yaml' -o \
           -name 'package.xml' -o -name 'setup.py' \) \
        -print0
    fi
  done | sort -z | xargs -0 -r sha256sum \
    >> "$RUN_DIR/source_sha256.txt"
}

command_init()
{
  source_ros
  mkdir -p "$EXPERIMENT_ROOT"
  local run_id
  run_id="$(date +%Y%m%d-%H%M%S)-E2E01"
  RUN_DIR="$EXPERIMENT_ROOT/$run_id"
  if [[ -e "$RUN_DIR" ]]; then
    echo "Run directory already exists: $RUN_DIR" >&2
    exit 1
  fi
  mkdir -p "$RUN_DIR"
  printf '%s\n' "$RUN_DIR" > "$STATE_FILE"

  local start_iso start_epoch
  start_iso="$(date --iso-8601=seconds)"
  start_epoch="$(date +%s)"
  printf '%s\n' "$start_epoch" > "$RUN_DIR/start_epoch.txt"

  {
    echo "Run ID: $run_id"
    echo "Start: $start_iso"
    echo "Purpose: full autonomous exploration and semantic-map generation"
    echo "Scene: AWS RoboMaker Small House"
    echo "ROS distro: ${ROS_DISTRO:-not sourced at init}"
    echo "Semantic baseline: YOLO11n CPU, imgsz=640, detector=0.25"
    echo "Fusion baseline: confidence=0.50, radius=0.60 m, 3/10/10"
    echo "Policy: no tuning and no manual driving unless explicitly marked"
    echo "Host: $(hostname)"
    echo "Kernel: $(uname -srmo)"
    echo "Workspace: $SEMANTIC_WORKSPACE"
  } > "$RUN_DIR/run_metadata.txt"

  write_git_metadata "$SEMANTIC_PACKAGE_DIR" semantic_perception
  write_git_metadata "$BRINGUP_PACKAGE_DIR" semantic_mapping_bringup
  write_source_hashes

  {
    echo "===== CPU ====="
    lscpu 2>&1 || true
    echo "===== MEMORY ====="
    free -h 2>&1 || true
    echo "===== STORAGE ====="
    df -h "$SEMANTIC_WORKSPACE" 2>&1 || true
  } > "$RUN_DIR/host_baseline.txt"

  printf '%s | INIT | E2E01 directory created\n' "$start_iso" \
    > "$RUN_DIR/timeline.txt"
  : > "$RUN_DIR/background_pids.txt"

  echo "$RUN_DIR"
}

read_param_value()
{
  local node="$1"
  local parameter="$2"
  ros2 param get "$node" "$parameter" 2>/dev/null |
    awk -F': ' 'NF {value=$NF} END {print value}'
}

command_preflight()
{
  load_run_dir
  source_ros
  require_command ros2

  # A previous pass must never authorize capture after a later failed check.
  rm -f "$RUN_DIR/preflight_passed.txt"

  local failure_count=0
  local node_list topic_list graph_ready graph_deadline
  local -a required_nodes=(
    /rgbd_yolo_localizer_node
    /slam_drift_diagnostic_node
  )
  local -a required_topics=(
    /clock /tf /tf_static /odom /scan /map
    /camera/image /camera/depth_image /camera/camera_info
    /semantic_map/markers
  )

  # DDS discovery and delayed launch actions can make an immediate graph
  # snapshot incomplete even though the processes are healthy. Retry only the
  # readiness snapshot; parameter and TF failures remain hard failures.
  graph_deadline=$((SECONDS + 20))
  while true; do
    node_list="$(ros2 node list 2>/dev/null || true)"
    topic_list="$(ros2 topic list 2>/dev/null || true)"
    graph_ready=1
    local graph_item
    for graph_item in "${required_nodes[@]}"; do
      grep -Fxq "$graph_item" <<< "$node_list" || graph_ready=0
    done
    for graph_item in "${required_topics[@]}"; do
      grep -Fxq "$graph_item" <<< "$topic_list" || graph_ready=0
    done
    if (( graph_ready == 1 || SECONDS >= graph_deadline )); then
      break
    fi
    echo "Waiting for ROS graph readiness..."
    sleep 2
  done

  printf '%s\n' "$node_list" > "$RUN_DIR/preflight_nodes.txt"
  ros2 topic list -t > "$RUN_DIR/preflight_topics.txt" 2>&1 || true
  ros2 service list -t > "$RUN_DIR/preflight_services.txt" 2>&1 || true
  ros2 action list -t > "$RUN_DIR/preflight_actions.txt" 2>&1 || true

  local required_node
  for required_node in "${required_nodes[@]}"; do
    if ! grep -Fxq "$required_node" <<< "$node_list"; then
      echo "Missing node: $required_node" >&2
      failure_count=$((failure_count + 1))
    fi
  done

  local required_topic
  for required_topic in "${required_topics[@]}"; do
    if ! grep -Fxq "$required_topic" <<< "$topic_list"; then
      echo "Missing topic: $required_topic" >&2
      failure_count=$((failure_count + 1))
    fi
  done

  local parameter expected actual
  while read -r parameter expected; do
    actual="$(read_param_value /rgbd_yolo_localizer_node "$parameter" || true)"
    printf '%s=%s expected=%s\n' "$parameter" "$actual" "$expected" \
      >> "$RUN_DIR/preflight_parameters.txt"
    if [[ "$actual" != "$expected" ]]; then
      echo "Parameter mismatch: /rgbd_yolo_localizer_node $parameter=$actual, expected $expected" >&2
      failure_count=$((failure_count + 1))
    fi
  done <<'EOF'
fusion_confirmation_hits 3
fusion_confirmation_window 10
fusion_max_missed_frames 10
fusion_min_confidence 0.5
fusion_radius_m 0.6
confidence 0.25
EOF

  local sim_time_node
  for sim_time_node in \
    /rgbd_yolo_localizer_node \
    /slam_drift_diagnostic_node; do
    actual="$(read_param_value "$sim_time_node" use_sim_time || true)"
    printf '%s use_sim_time=%s expected=True\n' \
      "$sim_time_node" "$actual" >> "$RUN_DIR/preflight_parameters.txt"
    if [[ "$actual" != "True" ]]; then
      echo "Parameter mismatch: $sim_time_node use_sim_time=$actual, expected True" >&2
      failure_count=$((failure_count + 1))
    fi
  done

  timeout 8 ros2 run tf2_ros tf2_echo map base_footprint \
    > "$RUN_DIR/preflight_tf_map_base.txt" 2>&1 || true
  if ! grep -q 'Translation:' "$RUN_DIR/preflight_tf_map_base.txt"; then
    echo "TF check failed: map -> base_footprint unavailable" >&2
    failure_count=$((failure_count + 1))
  fi

  ros2 param dump /rgbd_yolo_localizer_node \
    > "$RUN_DIR/parameters_rgbd_yolo_localizer.yaml" 2>&1 || true
  ros2 param dump /slam_drift_diagnostic_node \
    > "$RUN_DIR/parameters_slam_drift_diagnostic.yaml" 2>&1 || true

  local hz_topic safe_topic
  local -a hz_pids=()
  for hz_topic in /camera/image /camera/depth_image /scan /odom /map; do
    safe_topic="${hz_topic#/}"
    safe_topic="${safe_topic//\//_}"
    timeout 8 ros2 topic hz "$hz_topic" \
      > "$RUN_DIR/preflight_hz_${safe_topic}.txt" 2>&1 &
    hz_pids+=("$!")
  done
  local hz_pid
  for hz_pid in "${hz_pids[@]}"; do
    wait "$hz_pid" 2>/dev/null || true
  done

  if (( failure_count > 0 )); then
    printf '%s | PREFLIGHT_FAIL | failures=%d\n' \
      "$(date --iso-8601=seconds)" "$failure_count" \
      >> "$RUN_DIR/timeline.txt"
    echo "Preflight failed with $failure_count issue(s). Do not start E2E01." >&2
    exit 1
  fi

  printf '%s | PREFLIGHT_PASS | required nodes/topics/TF/parameters ready\n' \
    "$(date --iso-8601=seconds)" >> "$RUN_DIR/timeline.txt"
  date --iso-8601=seconds > "$RUN_DIR/preflight_passed.txt"
  echo "Preflight passed: $RUN_DIR"
}

append_pid()
{
  printf '%s %s\n' "$1" "$2" >> "$RUN_DIR/background_pids.txt"
}

command_start_capture()
{
  load_run_dir
  source_ros
  require_command ros2

  if [[ ! -s "$RUN_DIR/preflight_passed.txt" ]]; then
    echo "Capture blocked: this run has no successful preflight." >&2
    echo "Fix the reported issues, then run: $0 preflight" >&2
    exit 1
  fi

  if [[ -s "$RUN_DIR/capture_started.txt" ]]; then
    echo "Capture already started for this run." >&2
    exit 1
  fi

  ros2 topic echo /rosout rcl_interfaces/msg/Log \
    > "$RUN_DIR/rosout.log" 2>&1 &
  append_pid rosout "$!"

  (
    while true; do
      echo "===== $(date --iso-8601=seconds) ====="
      free -m | sed -n '1,2p'
      ps -eo pid,pcpu,pmem,etime,stat,comm,args --sort=-pcpu |
        sed -n '1,25p'
      echo "--- ROS nodes ---"
      ros2 node list 2>&1 | sort
      sleep 10
    done
  ) > "$RUN_DIR/system_health.log" 2>&1 &
  append_pid health "$!"

  if command -v vmstat >/dev/null 2>&1; then
    vmstat -t 1 > "$RUN_DIR/vmstat.log" 2>&1 &
    append_pid vmstat "$!"
  fi

  local topic topic_regex
  local -a bag_topics=()
  topic_regex='^(/clock|/tf|/tf_static|/rosout|/odom|/scan|/map|/map_metadata|/semantic_map/markers|/cmd_vel|/goal_pose|/initialpose|/plan|/local_plan|/camera/camera_info|/rtabmap/.*|/.*costmap.*/costmap|/.*_action/status)$'
  while IFS= read -r topic; do
    if [[ "$topic" =~ $topic_regex ]]; then
      bag_topics+=("$topic")
    fi
    if [[ "${E2E_RECORD_SENSORS:-0}" == "1" ]] && \
       [[ "$topic" == "/camera/image" || \
          "$topic" == "/camera/depth_image" || \
          "$topic" == "/yolo/rgbd_annotated_image" ]]; then
      bag_topics+=("$topic")
    fi
  done < <(ros2 topic list)

  printf '%s\n' "${bag_topics[@]}" > "$RUN_DIR/rosbag_topics.txt"
  if (( ${#bag_topics[@]} == 0 )); then
    echo "No core topics available for rosbag." >&2
    exit 1
  fi

  ros2 bag record -o "$RUN_DIR/rosbag_core" "${bag_topics[@]}" \
    > "$RUN_DIR/rosbag_console.log" 2>&1 &
  local rosbag_pid="$!"
  append_pid rosbag "$rosbag_pid"
  sleep 2
  if ! kill -0 "$rosbag_pid" 2>/dev/null; then
    echo "rosbag failed to remain running; inspect rosbag_console.log" >&2
    stop_background_processes
    exit 1
  fi

  date --iso-8601=seconds > "$RUN_DIR/capture_started.txt"
  printf '%s | CAPTURE_START | core rosbag, rosout and health monitoring\n' \
    "$(date --iso-8601=seconds)" >> "$RUN_DIR/timeline.txt"
  echo "Capture started: $RUN_DIR"
}

command_mark()
{
  load_run_dir
  shift
  if [[ ! -s "$RUN_DIR/preflight_passed.txt" || \
        ! -s "$RUN_DIR/capture_started.txt" ]]; then
    echo "Mark blocked: successful preflight and active capture are required." >&2
    exit 1
  fi
  if (( $# == 0 )); then
    echo "A phase or anomaly description is required." >&2
    exit 1
  fi
  printf '%s | MARK | %s\n' "$(date --iso-8601=seconds)" "$*" |
    tee -a "$RUN_DIR/timeline.txt"
}

dump_core_parameters()
{
  local destination="$1"
  mkdir -p "$destination"
  local node safe_name
  while IFS= read -r node; do
    if [[ "$node" =~ (rgbd_yolo|slam_drift|rtabmap|controller_server|planner_server|bt_navigator|behavior_server|explore) ]]; then
      safe_name="${node#/}"
      safe_name="${safe_name//\//_}"
      ros2 param dump "$node" > "$destination/${safe_name}.yaml" \
        2>&1 || true
    fi
  done < <(ros2 node list 2>/dev/null)
}

command_snapshot()
{
  load_run_dir
  source_ros
  require_command ros2
  local label="${2:-snapshot}"
  local safe_label snapshot_dir
  safe_label="$(tr -cs '[:alnum:]_-' '_' <<< "$label" | sed 's/_$//')"
  snapshot_dir="$RUN_DIR/snapshot_$(date +%H%M%S)_$safe_label"
  mkdir -p "$snapshot_dir"

  ros2 node list > "$snapshot_dir/nodes.txt" 2>&1 || true
  ros2 topic list -t > "$snapshot_dir/topics.txt" 2>&1 || true
  ros2 service list -t > "$snapshot_dir/services.txt" 2>&1 || true
  ros2 action list -t > "$snapshot_dir/actions.txt" 2>&1 || true
  dump_core_parameters "$snapshot_dir/parameters"

  timeout 10 ros2 topic echo /semantic_map/markers --once \
    > "$snapshot_dir/semantic_markers.yaml" 2>&1 || true
  timeout 10 ros2 topic echo /map_metadata --once \
    > "$snapshot_dir/map_metadata.yaml" 2>&1 || true
  timeout 8 ros2 run tf2_ros tf2_echo map base_footprint \
    > "$snapshot_dir/tf_map_base.txt" 2>&1 || true
  timeout 8 ros2 run tf2_ros tf2_echo map odom \
    > "$snapshot_dir/tf_map_odom.txt" 2>&1 || true

  if [[ "$label" == "final" ]]; then
    timeout 30 ros2 run nav2_map_server map_saver_cli \
      -f "$RUN_DIR/final_map" \
      > "$RUN_DIR/map_saver_console.log" 2>&1 || true
  fi

  printf '%s | SNAPSHOT | %s | %s\n' \
    "$(date --iso-8601=seconds)" "$label" "$snapshot_dir" |
    tee -a "$RUN_DIR/timeline.txt"
}

command_status()
{
  load_run_dir
  echo "Run directory: $RUN_DIR"
  if [[ -f "$RUN_DIR/background_pids.txt" ]]; then
    while read -r name pid; do
      [[ -n "${pid:-}" ]] || continue
      if kill -0 "$pid" 2>/dev/null; then
        echo "$name: running (PID $pid)"
      else
        echo "$name: stopped (PID $pid)"
      fi
    done < "$RUN_DIR/background_pids.txt"
  fi
}

stop_background_processes()
{
  [[ -f "$RUN_DIR/background_pids.txt" ]] || return 0
  local name pid deadline still_running
  while read -r name pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $name (PID $pid)"
      kill -INT "$pid" 2>/dev/null || true
    fi
  done < "$RUN_DIR/background_pids.txt"

  # rosbag needs time after SIGINT to flush metadata and close its storage.
  # Poll instead of using `wait`: commands from an earlier logger invocation
  # are not children of this shell, so `wait PID` cannot reliably reap them.
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    still_running=0
    while read -r name pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      if kill -0 "$pid" 2>/dev/null; then
        still_running=1
        break
      fi
    done < "$RUN_DIR/background_pids.txt"
    (( still_running == 0 )) && break
    sleep 1
  done

  while read -r name pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $name with SIGTERM after graceful-stop timeout (PID $pid)"
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done < "$RUN_DIR/background_pids.txt"
}

copy_recent_rtabmap_databases()
{
  local start_epoch destination database source_key target
  start_epoch="$(sed -n '1p' "$RUN_DIR/start_epoch.txt")"
  destination="$RUN_DIR/rtabmap_databases"
  mkdir -p "$destination"
  : > "$destination/source_paths.txt"
  while IFS= read -r database; do
    # Use a deterministic source-path key so a repeated finish overwrites the
    # same collected copy instead of creating another duplicate.
    source_key="$(printf '%s' "$database" | sha256sum | cut -c1-12)"
    target="$destination/${source_key}_$(basename "$database")"
    cp --reflink=auto "$database" "$target"
    printf '%s\n' "$database" >> "$destination/source_paths.txt"
  done < <(
    find "$HOME/.ros" "$SEMANTIC_WORKSPACE" -maxdepth 5 \
      -path "$RUN_DIR" -prune -o \
      -type f -name '*.db' -newermt "@$start_epoch" -print \
      2>/dev/null | sort -u
  )
}

extract_diagnostics()
{
  local fusion_tmp fault_tmp
  fusion_tmp="$(mktemp "$RUN_DIR/.fusion_diagnostic.XXXXXX")"
  fault_tmp="$(mktemp "$RUN_DIR/.fault_candidates.XXXXXX")"

  find "$RUN_DIR" -maxdepth 2 -type f -name '*.log' -print0 |
    grep -zvE '/(fusion_diagnostic|fault_candidates)\.log$' |
    xargs -0 -r grep -hE \
      'FUSION_EVIDENCE|FUSION_GATE total|FUSION_ASSOC frame' \
      > "$fusion_tmp" || true

  find "$RUN_DIR" -maxdepth 2 -type f -name '*.log' -print0 |
    grep -zvE '/(fusion_diagnostic|fault_candidates)\.log$' |
    xargs -0 -r grep -hiE \
      'SLAM_EVENT|correction_jump|loop closure|error|failed|failure|fatal|exception|traceback|abort|reject|recover|oscillat|stuck|timed out|transform unavailable|tf_unavailable|no frontier|frontier.*empty|exploration.*complete|planner.*failed|controller.*failed' \
      > "$fault_tmp" || true

  mv -f "$fusion_tmp" "$RUN_DIR/fusion_diagnostic.log"
  mv -f "$fault_tmp" "$RUN_DIR/fault_candidates.log"
}

command_finish()
{
  load_run_dir
  echo "Finish 1/6: stopping capture processes gracefully"
  stop_background_processes
  echo "Finish 2/6: collecting recent RTAB-Map databases"
  copy_recent_rtabmap_databases
  echo "Finish 3/6: extracting diagnostic events"
  extract_diagnostics

  local end_iso end_epoch start_epoch
  end_iso="$(date --iso-8601=seconds)"
  end_epoch="$(date +%s)"
  start_epoch="$(sed -n '1p' "$RUN_DIR/start_epoch.txt")"
  if ! grep -q '^End:' "$RUN_DIR/run_metadata.txt"; then
    {
      echo "End: $end_iso"
      echo "Duration seconds: $((end_epoch - start_epoch))"
    } >> "$RUN_DIR/run_metadata.txt"
  fi
  if ! grep -qE '\| FINISH \|' "$RUN_DIR/timeline.txt"; then
    printf '%s | FINISH | capture stopped and archive prepared\n' "$end_iso" \
      >> "$RUN_DIR/timeline.txt"
  fi

  echo "Finish 4/6: hashing non-rosbag artifacts"
  find "$RUN_DIR" -maxdepth 2 -type f \
    ! -path '*/rosbag_core/*' \
    ! -name 'artifact_sha256.txt' -print0 |
    sort -z | xargs -0 -r sha256sum > "$RUN_DIR/artifact_sha256.txt"

  local run_id archive_path archive_tmp run_size
  run_id="$(basename "$RUN_DIR")"
  mkdir -p "$HOME/Downloads"
  archive_path="$HOME/Downloads/${run_id}.tar.gz"
  archive_tmp="${archive_path}.partial"
  run_size="$(du -sh "$RUN_DIR" 2>/dev/null | awk '{print $1}')"
  echo "Finish 5/6: compressing ${run_size:-unknown-size} run; this may take several minutes"
  rm -f "$archive_tmp"
  trap 'rm -f "$archive_tmp"' INT TERM EXIT
  tar -cf - -C "$EXPERIMENT_ROOT" "$run_id" | gzip -1 > "$archive_tmp"
  gzip -t "$archive_tmp"
  mv -f "$archive_tmp" "$archive_path"
  trap - INT TERM EXIT

  echo "Finish 6/6: verifying archive"
  tar -tzf "$archive_path" >/dev/null
  date --iso-8601=seconds > "$RUN_DIR/finish_completed.txt"
  if [[ -f "$STATE_FILE" ]] && \
     [[ "$(sed -n '1p' "$STATE_FILE")" == "$RUN_DIR" ]]; then
    rm -f "$STATE_FILE"
  fi
  echo "$archive_path"
}

main()
{
  local command="${1:-}"
  case "$command" in
    init) command_init ;;
    path) load_run_dir; echo "$RUN_DIR" ;;
    preflight) command_preflight ;;
    start-capture) command_start_capture ;;
    mark) command_mark "$@" ;;
    snapshot) command_snapshot "$@" ;;
    status) command_status ;;
    finish) command_finish ;;
    *) usage; exit 1 ;;
  esac
}

main "$@"
