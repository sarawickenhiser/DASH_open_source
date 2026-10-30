#!/usr/bin/env bash
#
# One command for a bimanual hardware run:
#
#   bash start_hw.sh
#
# It reads this machine's settings from machine_config.sh, runs
# start_both_arms_hw.sh, and attaches:
#
#   tmux attach -t both_arms_hw
#
# and nothing more. start_both_arms_hw.sh is called unchanged: it STAGES the
# session and starts nothing, so the three Enters and `enable` are still yours,
# in your order -- the arms raise on `enable` and the instruments go on after
# (SYSTEM_CONTEXT 9.21). If the launcher refuses (boards missing, a session or
# a stack already running), this stops with its message and does not attach.
#
# Anything the launcher reads can still be set per run, and wins over
# machine_config.sh, e.g.
#
#   VIEWER=1 bash start_hw.sh
#   RIGHT_PORT=/dev/serial/by-id/<other> bash start_hw.sh
#   LOG=/some/dir bash start_hw.sh
set -e

cd "$(dirname "$(readlink -f "$0")")"

# The two OpenRB-150 boards, the G1 interface, the log directory and the
# per-pane ROS environment. Every line in it is a default, so whatever is
# already in the environment wins.
if [ ! -f machine_config.sh ]; then
  echo "start_hw.sh: machine_config.sh is missing from $(pwd)." >&2
  echo "It holds this machine's board paths and network interface; see README.md." >&2
  exit 1
fi
. ./machine_config.sh

# RFID is on: each side's rfid_scan_node reads the tag on the mounted
# instrument and looks it up in tool_configs/rfid_registry.json, and that
# side's wrist stays still until its tag is recognised and auto-zeroed.
#
# RIGHT_TOOL_UID / LEFT_TOOL_UID=<registry uid> turn scanning off on that side
# and fake that tag on its /rfid/uid instead -- for a bench with no reader, or
# a tag that will not read. That wrist then sweeps a few seconds after the
# tool-chain Enter whether or not anything is mounted, so seat the instrument
# first. See RIGHT_TOOL_UID in start_both_arms_hw.sh.

bash start_both_arms_hw.sh

# Inside tmux already, attaching would nest the session in a pane.
if [ -n "${TMUX:-}" ]; then
  tmux switch-client -t both_arms_hw
else
  exec tmux attach -t both_arms_hw
fi
