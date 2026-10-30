#!/usr/bin/env bash
#
# One command for a simulated bimanual run -- both G1 arms and both
# instruments in MuJoCo, no robot and no adapter boards:
#
#   bash start_sim.sh                  # driven by the two dVRK MTMs
#   COMMAND=servo bash start_sim.sh    # no dVRK: drive it with servo_step.py
#
# It reads ENV_SETUP (and MTM_RIGHT, MTM_LEFT, ROS_DOMAIN_ID if set) from
# machine_config.sh, runs start_both_arms_sim.sh, and attaches:
#
#   tmux attach -t both_arms_sim
#
# start_both_arms_sim.sh STAGES the session and starts nothing; press Enter in
# each pane yourself. The board ports and IFACE in machine_config.sh are not
# used here and may stay blank. If the launcher refuses (a session or a stack
# already running), this stops with its message and does not attach.
set -e

cd "$(dirname "$(readlink -f "$0")")"

if [ ! -f machine_config.sh ]; then
  echo "start_sim.sh: machine_config.sh is missing from $(pwd)." >&2
  echo "It holds the per-pane ROS environment (ENV_SETUP); see README.md." >&2
  exit 1
fi
. ./machine_config.sh

bash start_both_arms_sim.sh

# Inside tmux already, attaching would nest the session in a pane.
if [ -n "${TMUX:-}" ]; then
  tmux switch-client -t both_arms_sim
else
  exec tmux attach -t both_arms_sim
fi
