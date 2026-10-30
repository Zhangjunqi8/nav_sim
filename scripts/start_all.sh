#!/usr/bin/env bash
#
# One-command launcher for the nav_sim + hanmole_navigation simulation stack.
#
# Run this on the HOST (it uses docker exec). The container must already be
# running. It performs the full sequence that used to be typed by hand:
#
#   1. stop leftover sim / Nav2 processes (unless --no-kill)
#   2. refresh the container's X cookie from the host mutter cookie
#   3. start Gazebo (Gazebo client + RViz by default; --headless to disable)
#   4. wait for /camera_depth/depth/image_raw
#   5. start Nav2 with config/v0_3/nav2_params_sim.yaml
#   6. wait for AMCL, disable its map->odom TF, publish a static map->odom
#   7. wait for the target gateway to reach "ready"
#   8. optionally send a NavigateToPose goal
#
# Usage:
#   ./start_all.sh start [options]
#   ./start_all.sh stop        (delegates to stop_all.sh)
#   ./start_all.sh status
#
# Start options:
#   --world NAME         world file under nav_sim/worlds (default:
#                        generated_env_ground_14.world)
#   --spawn X,Y          AGV spawn in world frame (default depends on world)
#   --map NAME|PATH      map yaml (default depends on world)
#   --goal X,Y[,YAW]     optional NavigateToPose goal, sent after ready
#   --robot-version V    v0_1/v0_2/v0_3 (default v0_3: boost + approach limiter
#                        with Humble Twist adapters; v0_2 bypasses this chain)
#   --nav-mode M         ekf_odom/wheel_odom (default wheel_odom)
#   --gui                open the Gazebo client window (default: on)
#   --rviz               open RViz with the Nav2 config (default: on)
#   --desktop            shorthand for --gui --rviz (the default)
#   --headless           do not open the Gazebo client or RViz
#   --timeout SEC        readiness timeout per stage (default 150)
#   --no-kill            do not stop existing processes first
#   --intent             enable the intent bridge and IntentCostCritic/
#                        HeadingCritic. Default is OFF (523823ef baseline)
#   --no-intent          explicitly keep intent off (the default)
#   --build              colcon build hanmole_navigation + nav_sim first
#   -h, --help           this help
#
# Environment overrides:
#   CONTAINER            docker container name (default nav_sim_full)
#   NAV_SIM_DOMAIN       ROS domain id (default 42; deliberately NOT inherited
#                        from the host's ROS_DOMAIN_ID, which may be for another
#                        robot)
#
# Notes:
# - Map->odom is published as identity. That is valid in this sim because the
#   planar drive plugin's odom is the world pose and both maps use world
#   coordinates. Do not reuse this script on a real robot.
# - Logs live inside the container: /tmp/sim.log, /tmp/nav.log,
#   /tmp/static_tf.log, /tmp/goal.log.
#
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${CONTAINER:-nav_sim_full}"
DOMAIN="${NAV_SIM_DOMAIN:-42}"
NAV_MAP_DIR="/workspace/src/hanmole_navigation/maps"
PARAMS_FILE="/workspace/src/hanmole_navigation/config/v0_3/nav2_params_sim.yaml"

WORLD="generated_env_ground_14.world"
SPAWN_SPEC=""
MAP=""
GOAL=""
ROBOT_VERSION="v0_3"
NAV_MODE="wheel_odom"
GUI="true"
RVIZ="true"
DO_KILL="true"
BUILD="false"
INTENT="false"
TIMEOUT="150"

info()  { printf '\033[1;32m[start_all]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[start_all]\033[0m %s\n' "$*"; }
die()   { printf '\033[1;31m[start_all]\033[0m %s\n' "$*" >&2; exit 1; }

dex()    { docker exec "$CONTAINER" bash -lc "$*"; }
dex_bg() { docker exec -d "$CONTAINER" bash -lc "$*"; }

usage() {
  awk 'NR==1 {next} /^set -euo pipefail$/ {exit} {sub(/^# ?/, ""); print}' "$0"
}

require_container() {
  local running
  running="$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)"
  [[ "$running" == "true" ]] || die "container '$CONTAINER' is not running"
}

refresh_xauth() {
  local cookie
  cookie="$(ls -t /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -1 || true)"
  if [[ -z "$cookie" ]]; then
    warn "no host X cookie found; Gazebo camera rendering may be disabled"
    return 0
  fi
  # /tmp/.Xauthority inside the container is a read-only bind mount of a stale
  # host file; write a fresh container-local copy instead.
  cat "$cookie" | docker exec -i "$CONTAINER" bash -lc \
    'cat > /tmp/xauth_navsim && chmod 600 /tmp/xauth_navsim'
}

stop_all() {
  CONTAINER="$CONTAINER" bash "$SCRIPT_DIR/stop_all.sh"
}

wait_camera() {
  local i
  for ((i = 0; i < TIMEOUT; i++)); do
    if dex "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; \
            export ROS_DOMAIN_ID=$DOMAIN; \
            timeout 3 ros2 topic list 2>/dev/null | grep -q '^/camera_depth/depth/image_raw$'"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_amcl() {
  local i
  for ((i = 0; i < TIMEOUT; i++)); do
    if dex "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; \
            export ROS_DOMAIN_ID=$DOMAIN; \
            timeout 3 ros2 lifecycle get /amcl 2>/dev/null | grep -q active"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_ready() {
  local i
  for ((i = 0; i < TIMEOUT; i++)); do
    if dex "grep -q 'waiting_for_navigation_ready -> ready' /tmp/nav.log 2>/dev/null"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_sim() {
  info "sim: world=$WORLD spawn=($SPAWN_X,$SPAWN_Y) gui=$GUI rviz=$RVIZ"
  dex_bg "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN
    export XAUTHORITY=/tmp/xauth_navsim
    export GAZEBO_MASTER_URI=http://127.0.0.1:11355
    ros2 launch nav_sim sim.launch.py \
      world:=$WORLD spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y \
      rviz:=false gui:=$GUI publish_ground_truth:=false \
      > /tmp/sim.log 2>&1"
}

start_nav() {
  local adapters="false"
  case "$ROBOT_VERSION" in
    v0_1|v0_3) adapters="true" ;;
  esac
  info "nav: robot_version=$ROBOT_VERSION nav_mode=$NAV_MODE map=$MAP_CONT intent=$INTENT twist_adapters=$adapters"
  dex_bg "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN
    ros2 launch hanmole_navigation navigation_launch.py \
      robot_version:=$ROBOT_VERSION nav_mode:=$NAV_MODE \
      map:=$MAP_CONT params_file:=$PARAMS_FILE \
      use_sim_time:=true use_composition:=True use_global_relocalization:=false \
      use_intent_mpc:=$INTENT use_twist_adapters:=$adapters \
      > /tmp/nav.log 2>&1"
}

start_static_tf() {
  info "publishing static map->odom (identity)"
  dex_bg "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN
    ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
      --frame-id map --child-frame-id odom \
      > /tmp/static_tf.log 2>&1"
}

start_rviz() {
  info "starting RViz (hanmole_navigation/rviz/navigation.rviz)"
  dex_bg "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN
    export XAUTHORITY=/tmp/xauth_navsim
    export DISPLAY=:0
    ros2 run rviz2 rviz2 \
      -d /workspace/src/hanmole_navigation/rviz/navigation.rviz \
      > /tmp/rviz.log 2>&1"
}

send_goal() {
  local spec="$1" gx gy yaw qz qw
  IFS=',' read -r gx gy yaw <<< "$spec"
  gy="${gy:-5.0}"
  yaw="${yaw:-0.0}"
  read -r qz qw < <(awk -v y="$yaw" 'BEGIN{printf "%.9f %.9f\n", sin(y/2.0), cos(y/2.0)}')
  info "goal: x=$gx y=$gy yaw=$yaw"
  dex_bg "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN
    ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
      '{pose: {header: {frame_id: map}, pose: {position: {x: $gx, y: $gy, z: 0.0}, orientation: {z: $qz, w: $qw}}}}' \
      > /tmp/goal.log 2>&1"
  info "goal feedback is in /tmp/goal.log (container)"
}

print_ready() {
  cat <<EOF

$(info "ready")
  goal (container):
    ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 7.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}}"

  monitor (container):
    ros2 topic hz /hanmole/intent/obstacles_2d
    ros2 topic info -v /hanmole/intent/obstacles_2d
    grep -E "IntentCostCritic|Optimizer fail" /tmp/nav.log

  logs:
    /tmp/sim.log  /tmp/nav.log  /tmp/static_tf.log  /tmp/rviz.log  /tmp/goal.log
EOF
}

status() {
  require_container
  info "container $CONTAINER: $(docker inspect -f '{{.State.Status}}' "$CONTAINER")"
  dex "
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    export ROS_DOMAIN_ID=$DOMAIN
    echo '--- nodes ---'
    timeout 5 ros2 node list 2>/dev/null | sort | head -50
    echo '--- topics ---'
    timeout 5 ros2 topic list 2>/dev/null | grep -E 'scan|odom|camera_depth|intent|cmd_vel|^/tf' | head -30
    echo '--- nav tail ---'
    tail -5 /tmp/nav.log 2>/dev/null || true
    echo '--- sim tail ---'
    tail -3 /tmp/sim.log 2>/dev/null || true"
}

main() {
  local cmd="${1:-start}"
  [[ $# -gt 0 ]] && shift || true

  case "$cmd" in
    start)
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --world)          WORLD="$2"; shift 2 ;;
          --spawn)          SPAWN_SPEC="$2"; shift 2 ;;
          --map)            MAP="$2"; shift 2 ;;
          --goal)           GOAL="$2"; shift 2 ;;
          --robot-version)  ROBOT_VERSION="$2"; shift 2 ;;
          --nav-mode)       NAV_MODE="$2"; shift 2 ;;
          --gui)            GUI="true"; shift ;;
          --rviz)           RVIZ="true"; shift ;;
          --desktop)        GUI="true"; RVIZ="true"; shift ;;
          --headless)       GUI="false"; RVIZ="false"; shift ;;
          --timeout)        TIMEOUT="$2"; shift 2 ;;
          --no-kill)        DO_KILL="false"; shift ;;
          --intent)         INTENT="true"; shift ;;
          --no-intent)      INTENT="false"; shift ;;
          --build)          BUILD="true"; shift ;;
          -h|--help)        usage; exit 0 ;;
          *)                die "unknown option: $1" ;;
        esac
      done

      require_container
      [[ "$WORLD" == *.world ]] || WORLD="$WORLD.world"

      if [[ "$BUILD" == "true" ]]; then
        info "colcon build hanmole_navigation + nav_sim"
        dex "source /opt/ros/humble/setup.bash; cd /workspace && \
             colcon build --symlink-install \
               --packages-select hanmole_navigation nav_sim \
               --cmake-args -DCMAKE_BUILD_TYPE=Release"
      fi

      if [[ -z "$SPAWN_SPEC" ]]; then
        case "$WORLD" in
          generated_env_ground_14.world|generated_env_ground.world) SPAWN_SPEC="-11.0,-10.1" ;;
          square_dynamic.world)                                     SPAWN_SPEC="0.0,5.0" ;;
          *) warn "unknown world '$WORLD': defaulting spawn to 0,0"; SPAWN_SPEC="0.0,0.0" ;;
        esac
      fi
      IFS=',' read -r SPAWN_X SPAWN_Y <<< "$SPAWN_SPEC"
      [[ -n "$SPAWN_X" && -n "$SPAWN_Y" ]] || die "--spawn must be X,Y"

      if [[ -z "$MAP" ]]; then
        case "$WORLD" in
          generated_env_ground_14.world|generated_env_ground.world) MAP="generated_env_ground_static.yaml" ;;
          square_dynamic.world)                                     MAP="square_dynamic.yaml" ;;
          *) die "please pass --map for world '$WORLD'" ;;
        esac
      fi
      if [[ "$MAP" == */* ]]; then MAP_CONT="$MAP"; else MAP_CONT="$NAV_MAP_DIR/$MAP"; fi
      dex "test -f '$MAP_CONT'" || die "map not found in container: $MAP_CONT"
      dex "test -f '$PARAMS_FILE'" || die "params not found in container: $PARAMS_FILE"

      if [[ "$DO_KILL" == "true" ]]; then
        info "stopping leftover processes"
        stop_all
      fi
      refresh_xauth
      start_sim
      info "waiting for camera (up to ${TIMEOUT}s)"
      wait_camera || { dex "tail -30 /tmp/sim.log" || true; die "camera never appeared"; }

      start_nav
      info "waiting for AMCL (up to ${TIMEOUT}s)"
      wait_amcl || { dex "tail -40 /tmp/nav.log" || true; die "AMCL never activated"; }

      dex "source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; \
           export ROS_DOMAIN_ID=$DOMAIN; ros2 param set /amcl tf_broadcast false" || true
      start_static_tf
      if [[ "$RVIZ" == "true" ]]; then start_rviz; fi

      info "waiting for target gateway ready (up to ${TIMEOUT}s)"
      wait_ready || { dex "tail -50 /tmp/nav.log" || true; die "navigation never became ready"; }

      if [[ "$INTENT" == "true" ]]; then
        dex "grep -q 'IntentCostCritic initialized' /tmp/nav.log" \
          || warn "IntentCostCritic not found in /tmp/nav.log"
        dex "grep -q 'prediction size is set to: 33' /tmp/nav.log" \
          || warn "predictor did not load prediction_size=33 (check params)"
      fi

      if [[ -n "$GOAL" ]]; then send_goal "$GOAL"; fi
      print_ready
      ;;

    stop)
      require_container
      info "stopping all sim/Nav2 processes"
      stop_all
      info "done"
      ;;

    status)
      status
      ;;

    -h|--help|help)
      usage
      ;;

    *)
      die "unknown command: $cmd (use start|stop|status)"
      ;;
  esac
}

main "$@"
