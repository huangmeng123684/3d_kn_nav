#!/usr/bin/env bash

# Open a 2-column x 4-row tmux workspace.  The four panes in the left
# column enter the robot Docker container and receive commands without
# pressing Enter.  The top two panes in the right column also enter the
# container without receiving commands; the remaining two stay on the host.

set -euo pipefail

SESSION_NAME="${SESSION_NAME:-go2_nav}"
CONTAINER_ID="${CONTAINER_ID:-f3b82610c6d7}"
STARTUP_WAIT="${STARTUP_WAIT:-2}"

recreate=false
if [[ "${1:-}" == "--recreate" ]]; then
  recreate=true
  shift
fi
if (( $# != 0 )); then
  echo "Usage: $0 [--recreate]" >&2
  exit 2
fi

LEFT_TITLES=(livox pct_scan goal_points enable_go2)
RIGHT_TITLES=(container_1 container_2 host_3 host_4)
LEFT_COMMANDS=(
  "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
  "ros2 launch pct_scan_navigation unitree_go2w_pct_scan_navigation.launch.py"
  "python3 /home/code/work_space/kn_nav/src/tools/goal_points_cli.py"
  "ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'"
)

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it with: sudo apt install tmux" >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command was not found." >&2
  exit 1
fi

container_running="$(
  docker inspect --format '{{.State.Running}}' "${CONTAINER_ID}" 2>/dev/null || true
)"
if [[ "${container_running}" != "true" ]]; then
  echo "Docker container ${CONTAINER_ID} does not exist or is not running." >&2
  echo "Check it with: docker ps --no-trunc" >&2
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  if [[ "${recreate}" == "true" ]]; then
    tmux kill-session -t "${SESSION_NAME}"
  else
    echo "Attaching to existing tmux session: ${SESSION_NAME}"
    echo "Use --recreate to discard it and build a fresh layout."
    exec tmux attach-session -t "${SESSION_NAME}"
  fi
fi

tmux new-session -d -s "${SESSION_NAME}" -n navigation
left_0="$(tmux display-message -p -t "${SESSION_NAME}:navigation.0" '#{pane_id}')"

# First make two equal-width columns.
right_0="$(tmux split-window -h -p 50 -t "${left_0}" -P -F '#{pane_id}')"

# Split one full-height column into four nearly equal-height panes.
split_column() {
  local top="$1"
  local second third fourth
  second="$(tmux split-window -v -p 75 -t "${top}" -P -F '#{pane_id}')"
  third="$(tmux split-window -v -p 67 -t "${second}" -P -F '#{pane_id}')"
  fourth="$(tmux split-window -v -p 50 -t "${third}" -P -F '#{pane_id}')"
  printf '%s %s %s %s\n' "${top}" "${second}" "${third}" "${fourth}"
}

read -r -a left_panes <<< "$(split_column "${left_0}")"
read -r -a right_panes <<< "$(split_column "${right_0}")"

tmux set-option -t "${SESSION_NAME}" mouse on
tmux set-window-option -t "${SESSION_NAME}:navigation" remain-on-exit on
tmux set-window-option -t "${SESSION_NAME}:navigation" pane-border-status top
tmux set-window-option -t "${SESSION_NAME}:navigation" pane-border-format \
  ' #[fg=cyan]#{pane_title} #[default]'

for index in "${!left_panes[@]}"; do
  tmux select-pane -t "${left_panes[index]}" -T "${LEFT_TITLES[index]}"
  tmux select-pane -t "${right_panes[index]}" -T "${RIGHT_TITLES[index]}"

  # Run Docker directly as the pane process instead of typing a command into
  # an intermediate host shell.  This guarantees that each left pane is
  # attached to the container before it receives its prepared ROS command.
  tmux respawn-pane -k -t "${left_panes[index]}" \
    "docker exec -it ${CONTAINER_ID} /bin/bash"

  if (( index < 2 )); then
    tmux respawn-pane -k -t "${right_panes[index]}" \
      "docker exec -it ${CONTAINER_ID} /bin/bash"
  fi
done

# Give Docker time to present the container shell, then insert the four
# commands literally.  No C-m is sent here, so none of them is executed.
sleep "${STARTUP_WAIT}"
for index in "${!left_panes[@]}"; do
  tmux send-keys -l -t "${left_panes[index]}" "${LEFT_COMMANDS[index]}"
done

tmux select-pane -t "${left_panes[0]}"
exec tmux attach-session -t "${SESSION_NAME}"
