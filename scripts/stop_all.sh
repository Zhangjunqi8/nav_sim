#!/usr/bin/env bash
# Stop the simulation/navigation stack inside the selected Docker container.
# Run on the host; the container itself stays running for the next start.
set -euo pipefail

CONTAINER="${CONTAINER:-nav_sim_full}"
TIMEOUT="5"
DRY_RUN="false"

info() { printf '[stop_all] %s\n' "$*"; }
die() { printf '[stop_all] %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'
Usage: ./scripts/stop_all.sh [options]

Stop Gazebo, RViz, ROS 2 commands/daemon, nav_sim, Nav2, localization,
hanmole_navigation and their child processes inside the selected container.
Send SIGTERM, wait up to --timeout seconds, then SIGKILL remaining processes.
The container, host processes and other containers stay running.

Options:
  --container NAME  Docker container (default: $CONTAINER or nav_sim_full)
  --timeout SEC     Graceful shutdown timeout, in whole seconds (default: 5)
  --dry-run         List matching processes without sending signals
  -h, --help        Show this help

Examples:
  ./scripts/stop_all.sh
  ./scripts/stop_all.sh --dry-run
  ./scripts/stop_all.sh --container nav_sim_full --timeout 10
  CONTAINER=nav_sim_full ./scripts/stop_all.sh
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --container)
      [[ $# -ge 2 && -n "$2" && "$2" != -* ]] || die "--container requires a name"
      CONTAINER="$2"; shift 2 ;;
    --timeout)
      [[ $# -ge 2 && "$2" =~ ^(0|[1-9][0-9]*)$ ]] \
        || die "--timeout requires a non-negative whole number"
      TIMEOUT="$2"; shift 2 ;;
    --dry-run) DRY_RUN="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1 (use --help)" ;;
  esac
done

command -v docker > /dev/null || die "docker is not installed"
running="$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" \
  || die "cannot inspect container '$CONTAINER' (check its name and Docker access)"
if [[ "$running" != "true" ]]; then
  info "container '$CONTAINER' is already stopped"
  exit 0
fi

info "container=$CONTAINER timeout=${TIMEOUT}s dry_run=$DRY_RUN"
docker exec -i "$CONTAINER" bash -s -- "$TIMEOUT" "$DRY_RUN" <<'BASH'
set -euo pipefail
grace_seconds="$1"
dry_run="$2"
declare -A targets=()

# Match full command lines: Linux truncates several ROS executable names in
# the comm column. Package paths also catch the navigation helper/detector nodes.
process_pattern='(^|[[:space:]/])(gzserver|gzclient|gazebo|rviz2|robot_state_publisher|joint_state_publisher(_gui)?|spawn_entity(\.py)?|ground_truth_publisher|static_transform_publisher|component_container(_isolated)?|nav2_container|amcl|map_server|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|lifecycle_manager|ekf_node|ukf_node|intent_2d_bridge_node|intent_reference_node|cmd_vel_type_adapter_node|onboard_detector_node|dynamic_predictor_node)([[:space:]]|$)'
ros_pattern='(^|[[:space:]/])ros2([[:space:]]|$)|ros2cli\.daemon'
package_pattern='/lib/(nav_sim|hanmole_navigation|nav2_[^/[:space:]]+|robot_localization|slam_toolbox)/[^[:space:]]+'

collect_targets() {
  local pid ppid state command changed
  local -A parents=() commands=()
  while read -r pid ppid state command; do
    [[ "$pid" == "1" || "$pid" == "$$" || "$pid" == "$PPID" || "$state" == Z* ]] && continue
    parents["$pid"]="$ppid"
    commands["$pid"]="$command"
    if [[ "$command" =~ $process_pattern || "$command" =~ $ros_pattern || "$command" =~ $package_pattern ]]; then
      targets["$pid"]="$command"
    fi
  done < <(ps -eo pid=,ppid=,stat=,args=)

  # Include children such as launch wrappers and goal/monitor subprocesses,
  # even if their own executable names are not in the patterns above.
  changed=true
  while [[ "$changed" == "true" ]]; do
    changed=false
    for pid in "${!parents[@]}"; do
      ppid="${parents[$pid]}"
      if [[ -n "${targets[$ppid]:-}" && -z "${targets[$pid]:-}" ]]; then
        targets["$pid"]="${commands[$pid]}"
        changed=true
      fi
    done
  done
}

is_running() {
  local stat
  kill -0 "$1" 2>/dev/null || return 1
  IFS= read -r stat < "/proc/$1/stat" 2>/dev/null || return 1
  stat="${stat##*) }"
  [[ "${stat%% *}" != "Z" && "${stat%% *}" != "X" ]]
}

live_targets() {
  live=()
  local pid
  for pid in "${!targets[@]}"; do
    if is_running "$pid"; then
      live+=("$pid")
    fi
  done
}

collect_targets
live_targets
if [[ ${#live[@]} -eq 0 ]]; then
  printf '[stop_all] no simulation/navigation processes found\n'
  exit 0
fi

printf '[stop_all] matching processes (%s):\n' "${#live[@]}"
for pid in "${live[@]}"; do
  printf '  %s  %s\n' "$pid" "${targets[$pid]}"
done
if [[ "$dry_run" == "true" ]]; then
  printf '[stop_all] dry run: no signals sent\n'
  exit 0
fi

for pid in "${live[@]}"; do
  kill -TERM "$pid" 2>/dev/null || true
done
deadline=$((SECONDS + grace_seconds))
while (( SECONDS < deadline )); do
  live_targets
  [[ ${#live[@]} -eq 0 ]] && break
  sleep 0.2
done

# Catch respawned nodes as well as surviving descendants from the first scan.
collect_targets
live_targets
if [[ ${#live[@]} -gt 0 ]]; then
  printf '[stop_all] forcing shutdown of %s remaining processes\n' "${#live[@]}"
  for pid in "${live[@]}"; do
    kill -KILL "$pid" 2>/dev/null || true
  done
  sleep 0.2
fi
live_targets
if [[ ${#live[@]} -gt 0 ]]; then
  printf '[stop_all] could not stop PIDs: %s\n' "${live[*]}" >&2
  exit 1
fi
printf '[stop_all] all matching processes stopped; container stays running\n'
BASH
