# This machine's settings for start_hw.sh, which sources this file.
# start_sim.sh sources it too, but uses only ENV_SETUP and the optional
# settings at the bottom; the board ports and IFACE may stay blank for it.
#
# Every line is a DEFAULT: anything already set in the environment wins, so a
# single run can still override one value without editing this file:
#
#   RIGHT_PORT=/dev/serial/by-id/<other> bash start_hw.sh
#
# Fill in RIGHT_PORT, LEFT_PORT and IFACE once. While any of them is empty or
# wrong, start_both_arms_hw.sh refuses to stage the session and lists what it
# DID find, so running start_hw.sh once with them blank tells you what to put
# here.

# ---------------------------------------------------------------------------
# The two instrument adapters' OpenRB-150 boards.
#
# Use the stable /dev/serial/by-id path, never /dev/ttyACM*: the ACM numbers
# can swap between boots. The two boards are NOT interchangeable. Each one is a
# single wrist's DYNAMIXEL chain plus its RFID reader, so swapping them drives
# the wrong wrist, and nothing downstream can notice. List them with
#
#   ls -l /dev/serial/by-id/ | grep OpenRB-150
#
# then unplug one board to see which entry disappears.
# ---------------------------------------------------------------------------
export RIGHT_PORT="${RIGHT_PORT:-}"
export LEFT_PORT="${LEFT_PORT:-}"

# ---------------------------------------------------------------------------
# The network interface that reaches the G1. The robot is on Unitree's default
# 192.168.123.0/24 subnet; this is the interface holding an address there:
#
#   ip -brief addr | grep 192.168.123.
# ---------------------------------------------------------------------------
export IFACE="${IFACE:-}"

# ---------------------------------------------------------------------------
# Typed into every tmux pane before its command, so that rclpy, the ROS
# message packages, dynamixel_sdk, proxsuite and unitree_sdk2py are all
# importable there. A pane is a fresh shell, so whatever you sourced in the
# terminal that ran start_hw.sh does not reach it. Examples:
#
#   source /opt/ros/humble/setup.bash
#   conda activate dash && source /opt/ros/humble/setup.bash
#   source /opt/ros/humble/setup.bash && source ~/venvs/dash/bin/activate
# ---------------------------------------------------------------------------
export ENV_SETUP="${ENV_SETUP-source /opt/ros/humble/setup.bash}"

# ---------------------------------------------------------------------------
# Per-cycle CSVs for each arm, plus the pipeline pane's full console log.
# Set LOG= (empty) to turn the CSVs off; the console log then goes to
# ~/rcm_logs.
# ---------------------------------------------------------------------------
export LOG="${LOG-$HOME/rcm_logs}"

# ---------------------------------------------------------------------------
# Optional. The dVRK console's MTM names, if yours are not the defaults, and
# a ROS domain shared with the dVRK console.
# ---------------------------------------------------------------------------
# export MTM_RIGHT="${MTM_RIGHT:-MTMR}"
# export MTM_LEFT="${MTM_LEFT:-MTML}"
# export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
