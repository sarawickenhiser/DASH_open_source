#!/usr/bin/env bash
#
# Both G1 arms, simulated, driven by both MTMs from one console.
#
#   bash start_both_arms_sim.sh
#
# WHAT COMES UP
#
#   teleop, left          run_full_pipeline.py --config <right> --sim-robot   RIGHT arm, MTMR
#   teleop, top right     run_full_pipeline.py --config <left>  --sim-robot   LEFT  arm, MTML
#   teleop, bottom right  watch_both_arms.py                                  both, one window
#   tool.R    mtm_teleop_node arm:=MTMR                           RIGHT wrist + jaws
#   tool.L    mtm_teleop_node arm:=MTML  joint_prefix:=left_      LEFT  wrist + jaws
#
# SIM ONLY.  Each MTM's POSITION drives its own arm's tool TIP through its own
# RCM, and the same handle's ORIENTATION and gripper drive that side's
# instrument wrist and jaws.  Both at once, one console, one window.
#
# HOW THE TWO TOOLS STAY APART  (this is open item 7, and this is the answer)
#
# /h4hr/joint_command is absolute and unnamespaced and every consumer reads it
# BY JOINT NAME, so two mtm_teleop_nodes with the shipped names drive both
# wrists from both handles (SYSTEM_CONTEXT 8.5).  The separation used here is
# the one this graph already uses on /humanoid_rcm/sim_arm: give each side its
# own joint NAMES on the same topic.  The left side is prefixed left_ end to
# end and the right side keeps the only names that have ever existed:
#
#             publisher                    consumer
#   right     mtm_teleop_node              run_full_pipeline --sim-robot
#             roll_joint ...               roll_joint ...
#   left      mtm_teleop_node              run_full_pipeline --sim-robot
#             -p joint_prefix:=left_       --tool-joint-prefix left_
#             left_roll_joint ...          left_roll_joint ...
#
# Both halves of a side MUST carry the same prefix.  A mismatch is silent in
# the worst way -- mtm_teleop publishes at full rate, the pipeline's name
# filter drops every message, and the wrist simply never moves.  Both nodes
# print the names they are using at startup for exactly that reason.
#
# HOW THE TWO PIPELINES STAY APART
#
#   /MTMR/measured_cp   vs  /MTML/measured_cp      already distinct
#   /RCM_pose_right     vs  /RCM_pose_left         already distinct
#   /humanoid_rcm/sim_arm                          SHARED, separated by real
#                                                  joint names -- see
#                                                  publish_sim_arm
#   /h4hr/joint_command                            SHARED, separated by the
#                                                  left_ prefix above
#   /joint_states                                  SHARED, separated by the
#                                                  same prefix: each pipeline's
#                                                  sim-psm child echoes only
#                                                  its own side's names
#   /humanoid_rcm/enable                           SHARED ON PURPOSE: one gate
#                                                  arms both arms, which is
#                                                  what a bimanual run wants
#   /footpedals/clutch, /footpedals/coag           SHARED ON PURPOSE: one
#                                                  clutch freezes both tools
#                                                  and one coag tap re-homes
#                                                  both.  There is one pedal
#                                                  tray, so this is the only
#                                                  behaviour it can have
#
# No topic is invented anywhere in this script, which is the standing rule.
#
# BEFORE YOU DRIVE BOTH: check the ports.  This used to be a warning that the
# shipped pair COLLIDED -- at +-0.10 m the two shafts crossed at 439 mm of 486,
# fine for either arm alone and a collision the moment both moved.  The configs
# now carry +-0.15 m, where the two close to 99.1 mm and do it at the tips.
# Re-check after any change to a port, an insertion depth or a splay:
#   python place_rcm_both_arms.py --report
#
#   FORCE=1        skip the guard
#   MTM_RIGHT=...  handle driving the right arm    (default MTMR)
#   MTM_LEFT=...   handle driving the left  arm    (default MTML)
#   TOOL_ALPHA=... opacity of both instruments in the viewer (default 1.0,
#                  try 0.35 to see the wrist and the RCM marker through them)
#   VIEWER_ARGS=.. anything else for the viewer pane, e.g.
#                  VIEWER_ARGS="--white-bg --exposure 0.8" for a white room
#                  with the robot's specular highlight brought down
#   COMMAND=servo  no dVRK needed: arm setpoints from servo_step.py instead
#                  of the MTMs (see COMMAND below)
#   ENV_SETUP=...  typed into every pane before its command, to make ROS 2
#                  and the Python dependencies importable (default none)
#
# start_sim.sh runs this with ENV_SETUP and the rest from machine_config.sh.
set -e

SESSION="both_arms_sim"
# The bundle is this script's own directory and the ROS nodes sit one level
# above it, wherever the checkout lives.
BUNDLE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
DASH="$(dirname "$BUNDLE")"
TELEOP="$DASH/mtm_teleop_node.py"

RIGHT_CONFIG="configs/g1_psm_adapter_right.yaml"
LEFT_CONFIG="configs/g1_psm_adapter_left.yaml"

# Which handle drives which side.  A console convention, not a physical fact --
# SYSTEM_CONTEXT 8.1 and open item 9 both flag it as an assumption -- so it is
# one variable per side rather than a literal in four places.
MTM_RIGHT="${MTM_RIGHT:-MTMR}"
MTM_LEFT="${MTM_LEFT:-MTML}"

# The left side's joint-name prefix on /h4hr/joint_command and /joint_states.
# It appears TWICE below, once on each half of the left side, and the two must
# match; that is why it is a variable.  The right side takes no prefix, which
# is what keeps it byte-identical to the single-arm right-hand stack.
LEFT_TOOL_PREFIX="${LEFT_TOOL_PREFIX:-left_}"

# Every pane inherits these.  Pinning the RMW matters: a pane that disagrees
# still discovers its peers and then fails to deserialise what they send.
# ROS_DOMAIN_ID is passed through as it is, the same as start_both_arms_hw.sh:
# unset means domain 0, which is also where a fresh terminal running
# servo_step.py and an unconfigured dVRK console will be.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# ---------------------------------------------------------------------------
# Guard.  Same reasoning and same /proc-reading implementation as
# start_left_arm_rcm_teleop.sh: a single-arm stack already running owns the
# topics this one is about to use, and an orphaned --internal-role child is
# invisible in `tmux ls` while still publishing.
#
# `both_arms_hw` ADDED 2026-09-17, and it is the one that matters most here.
# The guard was asymmetric: start_both_arms_hw.sh already refused to start on
# top of `both_arms_sim`, but this script did not refuse the reverse -- so a
# simulated run could be started BESIDE a live hardware run and nothing
# anywhere would say so.
#
# That is not two runs side by side. /h4hr/joint_command, /humanoid_rcm/enable,
# /h4hr/enable_control, /h4hr/estop and /footpedals/* are all shared and
# unnamespaced, and /humanoid_rcm/sim_arm is the channel the hardware arms feed
# the merger on. A simulated pipeline publishing onto those drives the REAL
# robot: last writer wins on the joint command, and one gate arms both.
#
# The reason to reach for it is real and this is not the way to get it -- to
# WATCH a hardware run in MuJoCo, start the viewer alone, which reads and
# commands nothing:
#     VIEWER=1 bash start_both_arms_hw.sh        (a pane in the run)
#     python watch_both_arms.py                  (beside a run already up)
# ---------------------------------------------------------------------------
if [ "${FORCE:-0}" != "1" ]; then
  BLOCKERS=""
  for other in rcm_teleop left_rcm_teleop left_tool both_arms_hw; do
    if tmux has-session -t "$other" 2>/dev/null; then
      BLOCKERS="$BLOCKERS\n  tmux session '$other'   ->  tmux kill-session -t $other"
    fi
  done

  classify_pipeline_pids() {
    local want="$1" pid argv0 args
    for pid in $(pgrep -f "run_full_pipeline\.py" 2>/dev/null); do
      [ "$pid" = "$$" ] && continue
      [ -r "/proc/$pid/cmdline" ] || continue
      args="$(tr '\0' '\n' < "/proc/$pid/cmdline")"
      argv0="$(printf '%s\n' "$args" | head -1)"
      case "$(basename "$argv0")" in
        python|python3|python3.*) ;;
        *) continue ;;
      esac
      printf '%s\n' "$args" | grep -q '/run_full_pipeline\.py$\|^run_full_pipeline\.py$' || continue
      if printf '%s\n' "$args" | grep -qx -- "--internal-role"; then
        if [ "$want" = "child" ]; then printf '%s ' "$pid"; fi
      else
        if [ "$want" = "parent" ]; then printf '%s ' "$pid"; fi
      fi
    done
    return 0
  }

  PARENTS="$(classify_pipeline_pids parent | xargs)"
  ORPHANS="$(classify_pipeline_pids child | xargs)"

  for pid in $PARENTS; do
    BLOCKERS="$BLOCKERS\n  run_full_pipeline pid $pid   ->  kill $pid"
  done
  if [ -n "$ORPHANS" ]; then
    BLOCKERS="$BLOCKERS\n  orphaned pipeline children $ORPHANS   ->  kill $ORPHANS"
  fi

  if [ -n "$BLOCKERS" ]; then
    echo "refusing to start: something is already on this ROS graph." >&2
    echo >&2
    echo "This script starts BOTH arms, so an existing single-arm stack would" >&2
    echo "duplicate one of them.  Found:" >&2
    printf "%b\n" "$BLOCKERS" >&2
    echo >&2
    echo "Clear those, or set FORCE=1 if you have separated the sides yourself." >&2
    exit 1
  fi
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists. Attach with:" >&2
  echo "    tmux attach -t $SESSION" >&2
  echo "or clear it first with:" >&2
  echo "    tmux kill-session -t $SESSION" >&2
  exit 1
fi

# --sim-robot opens no DDS interface and no Dynamixel bus, so no instrument
# chain runs and nothing ever asserts /h4hr/auto_zero_ready -- the teleop gate
# has to come off or the simulated arms would never move.
# ---------------------------------------------------------------------------
# COMMAND -- where each arm's Cartesian tip setpoint comes from.
#
#   COMMAND=mtm     (default) the dVRK handles, exactly as before.  Unchanged.
#   COMMAND=servo   base-frame poses on each side's servo_cp topics.  No
#                   master, no clutch, no heartbeat: the arm stages, aligns and
#                   HOLDS with nothing commanding it, and a setpoint may arrive
#                   minutes later.  See AbsoluteTipSetpointSource.
#
# Under COMMAND=servo the instrument WRIST still belongs to mtm_teleop_node, so
# leaving the dVRK console up gives arm position from servo_cp and wrist plus
# jaws from the handle.  That is a working arrangement, not a mistake -- there
# is no servo wrist commander yet.
#
# THE CLUTCH DOES NOT STOP COMMAND=servo.  The stop is /humanoid_rcm/enable,
# /h4hr/estop, or typing `disable` on the pipeline console.
# ---------------------------------------------------------------------------
COMMAND="${COMMAND:-mtm}"
case "$COMMAND" in
  mtm|servo) ;;
  *)
    echo "COMMAND must be 'mtm' or 'servo'; got '$COMMAND'" >&2
    exit 2
    ;;
esac

COMMON="--sim-robot --allow-motion --confirm-motion MOVE_G1_PSM_WITH_RCM"
if [ "$COMMAND" != "mtm" ]; then
  COMMON="$COMMON --command-source $COMMAND"
fi
RIGHT_PIPELINE="python run_full_pipeline.py --config $RIGHT_CONFIG $COMMON"
LEFT_PIPELINE="python run_full_pipeline.py --config $LEFT_CONFIG $COMMON"
LEFT_PIPELINE="$LEFT_PIPELINE --tool-joint-prefix $LEFT_TOOL_PREFIX"
# Transparent instruments are a viewer-only setting: it fades the two tools in
# the shared window so the G1 wrist and the amber RCM sphere stay visible
# through the shaft.  Nothing that moves is touched.
TOOL_ALPHA="${TOOL_ALPHA:-1.0}"
VIEWER="python watch_both_arms.py --tool-alpha $TOOL_ALPHA ${VIEWER_ARGS:-}"

# No instrument chain runs in sim -- no board, so no RFID reader and no
# auto-zero sweep -- and nothing ever asserts /h4hr/auto_zero_ready, so the
# tool gate has to come off or neither wrist would ever move.  Distinct node
# names because ROS 2 only WARNS about a duplicate and two indistinguishable
# mtm_teleop_nodes is precisely what this script must not have.
RIGHT_TOOL="python $(basename "$TELEOP") --ros-args"
RIGHT_TOOL="$RIGHT_TOOL -p arm:=$MTM_RIGHT -p require_tool_ready:=false"
RIGHT_TOOL="$RIGHT_TOOL -r __node:=mtm_teleop_node_right"

LEFT_TOOL="python $(basename "$TELEOP") --ros-args"
LEFT_TOOL="$LEFT_TOOL -p arm:=$MTM_LEFT -p require_tool_ready:=false"
LEFT_TOOL="$LEFT_TOOL -p joint_prefix:=$LEFT_TOOL_PREFIX"
LEFT_TOOL="$LEFT_TOOL -r __node:=mtm_teleop_node_left"

# Typed into each pane right after its cd: a tmux pane is a fresh shell with
# the tmux SERVER's environment, not this terminal's, so a ROS 2 sourced only
# here is absent there.
ENV_SETUP="${ENV_SETUP:-}"
pane_setup() {
  tmux send-keys -t "$1" "cd \"$2\"${ENV_SETUP:+ && $ENV_SETUP}" ENTER
}

# Pane IDs, never indices, here too: `teleop.0` means a different pane (or no
# pane) under a tmux.conf with base-index or pane-base-index 1.
P_RIGHT=$(tmux new-session -d -s "$SESSION" -n teleop -P -F '#{pane_id}' \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")

P_LEFT=$(tmux split-window -P -F '#{pane_id}' -h -t "$P_RIGHT")
P_VIEW=$(tmux split-window -P -F '#{pane_id}' -v -t "$P_LEFT")

# STAGED, not run: press Enter in each, in order, once the one before it is up.
# The right pipeline first only so the two operator consoles come up in a
# predictable order; neither depends on the other.
pane_setup "$P_RIGHT" "$BUNDLE"
tmux send-keys -t "$P_RIGHT" "$RIGHT_PIPELINE"

pane_setup "$P_LEFT" "$BUNDLE"
tmux send-keys -t "$P_LEFT" "$LEFT_PIPELINE"

pane_setup "$P_VIEW" "$BUNDLE"
tmux send-keys -t "$P_VIEW" "$VIEWER"

tmux select-pane -t "$P_RIGHT"

# ---------------------------------------------------------------------------
# The two tool nodes, in their own window.  Separate from teleop because five
# panes is unreadable and because these two are what an operator restarts on
# their own -- a coag re-home is a per-tool action.
#
# Pane IDs, never indices: tmux renumbers indices by screen position on every
# split, so an index captured before the next split is not the pane it was
# (SYSTEM_CONTEXT 8.1 records measuring exactly that).  They also run from
# $DASH, not $BUNDLE -- mtm_teleop_node.py lives at the repo root.
# ---------------------------------------------------------------------------
P_TOOL_RIGHT=$(tmux new-window -d -t "$SESSION" -n tool -P -F '#{pane_id}' \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")
P_TOOL_LEFT=$(tmux split-window -P -F '#{pane_id}' -h -t "$P_TOOL_RIGHT")

# With the indices shuffled, a titled border is the only reliable way for an
# operator to tell the panes apart.
tmux set-option -t "$SESSION":tool pane-border-status top >/dev/null 2>&1 || true

tmux select-pane -t "$P_TOOL_RIGHT" -T "RIGHT tool  $MTM_RIGHT  roll_joint ..."
pane_setup "$P_TOOL_RIGHT" "$DASH"
tmux send-keys -t "$P_TOOL_RIGHT" "$RIGHT_TOOL"

tmux select-pane -t "$P_TOOL_LEFT" -T "LEFT tool  $MTM_LEFT  ${LEFT_TOOL_PREFIX}roll_joint ..."
pane_setup "$P_TOOL_LEFT" "$DASH"
tmux send-keys -t "$P_TOOL_LEFT" "$LEFT_TOOL"

cat <<BANNER

  session '$SESSION' staged: BOTH G1 ARMS + BOTH TOOLS, SIMULATED

  window 'teleop'
    left           RIGHT arm  <- $MTM_RIGHT position, through /RCM_pose_right
    top right      LEFT  arm  <- $MTM_LEFT position, through /RCM_pose_left
    bottom right   both arms, both instruments and both RCMs in one MuJoCo window

  window 'tool'   (ctrl-b n)
    RIGHT tool  <- $MTM_RIGHT orientation + gripper, as roll_joint ...
    LEFT  tool  <- $MTM_LEFT orientation + gripper, as ${LEFT_TOOL_PREFIX}roll_joint ...

  Nothing is running yet.  Attach and press Enter in each pane, in order:

      tmux attach -t $SESSION

  Start the two pipelines first, then the two tool nodes.  Order matters only
  for the console output: a tool node that starts first simply publishes into
  a graph with no consumer yet.

  Then in EITHER pipeline pane type 'enable' -- /humanoid_rcm/enable is
  shared, so one gate arms both arms.

  ONE PEDAL TRAY, TWO TOOLS.  The clutch freezes BOTH wrists and a coag tap
  re-homes BOTH.  That is not a limitation to work around; there is one tray.

  If a wrist does not move while its pipeline tracks fine, the two halves of
  that side disagree about the joint names.  Both nodes print theirs at
  startup -- they must match:

      $LEFT_TOOL_PREFIX{roll,pitch,yaw,grip}_joint   left
      {roll,pitch,yaw,grip}_joint     right

  Check the ports first if you have not:  python place_rcm_both_arms.py --report

BANNER
