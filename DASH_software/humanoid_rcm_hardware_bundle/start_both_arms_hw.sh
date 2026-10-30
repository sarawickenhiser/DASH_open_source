#!/usr/bin/env bash
#
# Both G1 arms and both instruments, ON HARDWARE, from one console.
#
#   bash start_both_arms_hw.sh            # real G1, real boards.  No modes.
#
# HARDWARE ONLY.  This script takes no arguments.  Simulation lives in
# start_both_arms_sim.sh and nowhere else, so there is no longer a mode word
# that can silently stage a simulated run in front of live boards -- and no
# `--sim-robot` anywhere below to make the difference invisible once staged.
#
# WHAT COMES UP -- ONE window (two with READOUT=<dir>)
#
#   window 'teleop'   everything that DRIVES the robot, on one page
#     left    run_both_arms_pipeline.py    supervises THREE processes:
#                                             the RIGHT arm pipeline
#                                             the LEFT  arm pipeline
#                                             the merger, owning rt/lowcmd
#     right   mtm_teleop_node  RIGHT       $MTM_RIGHT orientation + gripper
#             mtm_teleop_node  LEFT        $MTM_LEFT  orientation + gripper
#             watch_both_arms.py           ONLY with VIEWER=1.  Off by default:
#                                          it reads and draws, nothing gates on
#                                          it, and it was taking half the right
#                                          column from the two panes that matter
#
#   window 'tool'     the two instrument chains, one column per side:
#                     rfid_scan_node, tool_loader_node and auto_zero_node.
#                     ALWAYS staged as of 2026-09-17 -- the instrument is
#                     SCANNED again, and FIXED_TOOL is gone (see THREE STARTS).
#
#   window 'readout'  only with READOUT=<dir>: the END EFFECTOR readout, one
#                     pane per side -- commanded tip pose against the ENCODERS'
#                     tip pose, and the difference.  Read-only, off by default.
#
# THREE STARTS, ONE STOP
#
# Eleven processes come up in this session and pressing Enter in eleven panes
# is not a procedure anyone follows the same way twice.  They are grouped into
# THREE, each one Enter in ONE pane, and each group is a thing that has to go
# up as a unit:
#
#   1. PIPELINE   Enter on the pipeline pane   -> both arms, the merger, and
#                 (window 'teleop')               both PSM drivers, disarmed
#   2. TOOL       Enter on the RIGHT rfid pane -> all SIX instrument nodes,
#                 (window 'tool')                 both sides
#   3. MTM        Enter on the RIGHT MTM pane  -> both mtm_teleop_nodes
#                 (window 'teleop')
#
# The groups stay SEPARATE on purpose, in that order.  The drivers must exist
# before rfid_scan_node can ask one for its serial port, and both wrists must
# be scanned and zeroed before either MTM gate opens -- so collapsing these
# into one keypress would only mean starting things before what they talk to.
#
# A group leader is an ordinary pane whose command line begins with
# `tmux send-keys -t <pane> ENTER` for each of its followers.  The followers'
# commands are already TYPED and waiting at their prompts, exactly as before;
# the leader just presses Enter for you.  Nothing is backgrounded, every node
# still owns its own pane, its own scrollback and its own Ctrl+C.
#
# ONE STOP: Ctrl+C on the PIPELINE pane tears down the whole session -- see
# FANOUT at the bottom.  The three-way split is for STARTING only.
#
# The two MTM nodes were their own window until 2026-09-06.  They are on the
# front page because arming the arms and arming the two handles is ONE operator
# action in one order, and a window switch in the middle of it is where the
# order gets lost.  The instrument chains stay behind: six more panes here
# would leave the console too small to read, and those are started once and
# then watched, not driven.
#
# Each MTM's POSITION drives its own arm's tool tip through its own RCM, and
# the same handle's ORIENTATION and gripper drive that side's instrument wrist
# and jaws.  That is the same split as either single-arm stack, and the same
# one start_both_arms_sim.sh runs in simulation.
#
# ONE LAUNCHER, THREE PROCESSES
#
# Each arm runs the ORDINARY single-arm pipeline in its OWN OS process -- same
# script, same code path, only --remote-arm differs.  Two control loops in one
# interpreter cost far more than two processes: measured here, two co-located
# rclpy executors run 3.6x the CPU of one and three run 7.3x, and the two arms
# fall from 247 Hz each to 190 Hz while burning most of a core.
#
# But exactly ONE process may own rt/lowcmd, because that topic carries the
# whole body in every message; two publishers would make each arm alternate
# between its own live target and the other's frozen snapshot.  So a third,
# tiny merger process holds no control loop, subscribes to both arms' targets
# and publishes one whole-body message at a fixed rate -- the arrangement
# G1_29_ArmController in robot_arm_both_arms_user_study.py already uses on this
# robot.  SYSTEM_CONTEXT 9.11.
#
# LEGS AND WAIST BEHAVE EXACTLY AS IN A SINGLE-ARM RUN: locked at their pose
# when the merger arms, with the same gains.  Deliberately unchanged.
#
# HOW THE TWO SIDES STAY APART
#
# Two schemes, because two kinds of topic need different answers.
#
# 1. JOINT NAMES, on the shared topics.  /h4hr/joint_command and /joint_states
#    stay exactly where they are and the left side's four joints take the
#    left_ prefix (SYSTEM_CONTEXT 9.9).  Every consumer already matched by
#    name.  No topic is invented.  This now reaches the instrument chain too:
#    dynamixel_controller_node, rfid_scan_node, tool_loader_node and
#    auto_zero_node all take -p joint_prefix:= as of 2026-09-06.  Each arm's
#    seven joint TARGETS reach the merger the same way, on the pre-existing
#    /humanoid_rcm/sim_arm topic under real model joint names.
#
# 2. LAUNCH-TIME REMAPPING, for everything that carries no joint names at all.
#    /h4hr/release_port and /h4hr/reclaim_port are SERVICES: two drivers would
#    both offer them and a client's request routes to one of them
#    nondeterministically, so a LEFT scan could take the RIGHT board's serial
#    port.  The Bool/String/Float64MultiArray topics have the same problem --
#    each tool_loader would react to the other's tag and each MTM gate would
#    open on the other's ready flag.  Those are remapped per side below, in the
#    launcher, so no source file gains a name.
#
# SHARED ON PURPOSE, and deliberately NOT remapped:
#
#   /humanoid_rcm/enable        one gate arms both arms
#   /h4hr/enable_control        one console arms both drivers
#   /h4hr/estop, /h4hr/emergency_stop, /h4hr/clear_limit
#                               an estop that stops one arm is not an estop
#   /footpedals/clutch, /coag   there is ONE pedal tray.  The clutch freezes
#                               BOTH wrists and a coag tap re-homes BOTH
#
# BEFORE YOU RUN THIS
#
#   python place_rcm_both_arms.py --report      the two ports must not cross
#
#   DELAY=<sec>      auto_zero_node's hold-off after a tag is seen
#                    (default 2.0; see INSERTION_DELAY below)
#   TOOL_SETTLE=<s>  head start the tool-chain CONSUMERS get before the two
#                    scanners start publishing (default 3)
#   RIGHT_TOOL_UID=<uid>, LEFT_TOOL_UID=<uid>
#                    no scanning: that side fakes this registry tag
#                    (default empty = scan; see RIGHT_TOOL_UID below)
#   VIEWER=1        add the MuJoCo viewer pane (off by default; it reads
#                    and draws only, nothing in the run gates on it)
#   LOG=<dir>        per-cycle CSV per arm (measured vs command, every cycle)
#   CONSOLE_LOG=<f>  everything the pipeline pane prints (default under LOG)
#   READOUT=<dir>    stage another window with the two END EFFECTOR readouts
#   FORCE=1          skip the running-stack guard
#   JAW=mtm|servo    who owns grip_joint (default follows COMMAND)
#   RCM_SOURCE=detected  both pivots from the ArUco detector, no fallback
#                    (default auto; see RCM_SOURCE below)
#   RIGHT_PORT=...   right OpenRB-150               (no default; required)
#   LEFT_PORT=...    left  OpenRB-150               (no default; required)
#   MTM_RIGHT=...    handle driving the right arm   (default MTMR)
#   MTM_LEFT=...     handle driving the left  arm   (default MTML)
#   IFACE=...        G1 network interface           (default eth0 if unset)
#   ENV_SETUP=...    typed into every pane before its command, to make ROS 2
#                    and the Python dependencies importable (default none)
#
# start_hw.sh sets the machine-specific ones from machine_config.sh.
set -e

# No mode word.  See the header: simulation is start_both_arms_sim.sh's job.
# Refused rather than ignored, because `start_both_arms_hw.sh sim` used to be a
# real thing an operator's fingers know, and silently starting a HARDWARE run
# from it is the worst possible answer.
if [ "$#" -gt 0 ]; then
  echo "start_both_arms_hw.sh takes no arguments -- it is hardware-only." >&2
  echo "Got: $*" >&2
  echo >&2
  echo "For a simulated run:  bash start_sim.sh" >&2
  exit 2
fi

SESSION="both_arms_hw"
# The bundle is this script's own directory and the ROS nodes sit one level
# above it, wherever the checkout lives.
BUNDLE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
DASH="$(dirname "$BUNDLE")"

RIGHT_CONFIG="configs/g1_psm_adapter_right.yaml"
LEFT_CONFIG="configs/g1_psm_adapter_left.yaml"

MTM_RIGHT="${MTM_RIGHT:-MTMR}"
MTM_LEFT="${MTM_LEFT:-MTML}"
# The G1 is on 192.168.123.0/24 (the robot answers at .161), and IFACE is the
# interface on THIS machine that holds an address on that subnet -- set it in
# machine_config.sh.  An interface name that is wrong fails inside the merger's
# DDS setup, after tmux has already staged the whole session, so it is checked
# below before anything is staged.
#
# `-`, not `:-`: unset falls back to eth0, but SET-and-empty -- the blank that
# machine_config.sh ships with -- stays empty and is refused below, rather
# than quietly becoming an interface nobody chose.
IFACE="${IFACE-eth0}"

# The left side's joint-name prefix.  It appears on EVERY left-side node below
# and they must all agree; that is why it is one variable.  The right side
# takes no prefix, which keeps it byte-identical to the single-arm stack.
LEFT_PREFIX="${LEFT_PREFIX:-left_}"

# ---------------------------------------------------------------------------
# The instrument chain is BACK, and it is the only path.
#
# FIXED_TOOL (2026-09-07 to 2026-09-17) named the instrument on both drivers
# with -p tool_config:= and took rfid_scan_node, tool_loader_node and
# auto_zero_node out of the run.  It is GONE.  What it cost was never small:
#
#   nothing read a tag, so nothing checked the name against the tool actually
#   screwed onto either adapter -- a wrong name drove that wrist through
#   another instrument's coupling with no complaint anywhere; and
#
#   nothing auto-zeroed either wrist.  The driver's go_to_zero_on_start moved
#   the wrist to a commanded pose, which is not the same as proving the disks
#   can reach their travel.  That was open item 20.
#
# Both drivers therefore go back to the release/reclaim cycle on the OpenRB.
# That path is the one SYSTEM_CONTEXT 9.11 faults 2 and 4 live on, and it is
# gated rather than hoped about: the port's own `is_open` is the single source
# of truth for every write, control_enabled means only what the operator says,
# and tests/test_instrument_port_gating.py (14 cases) covers the
# enable-burst-during-a-scan sequence itself.  9.11's third contact ran 83 s on
# hardware with that gating in.
#
#   DELAY=<sec>        auto_zero_node's hold-off, see INSERTION_DELAY below
#   TOOL_SETTLE=<sec>  the consumers' head start, see TOOL_SETTLE below
# ---------------------------------------------------------------------------

# INSERTION_DELAY -- how long auto_zero_node waits after a tag is recognised
# before it starts turning that wrist.
#
# 2.0 rather than auto_zero_node's own default of 1.0, matching
# start_tool_rfid_test.sh, and for the reason that launcher measured: the tag
# is read as soon as it enters the reader's range, which is BEFORE the
# instrument is seated and before the hand is clear of it.  Seating takes
# 1.5-2 s by the operator's count, so 1 s starts the sweep with a hand still on
# the tool some of the time.  Overshooting costs half a second of standing
# still; undershooting costs a wrist turning under someone's fingers.
#
# It matters more here than on the bench: there are TWO instruments to fit, and
# the second is fitted while the first side's chain is already live.
#
# The hold is timed from the /h4hr/tool_loaded transition arriving at
# auto_zero_node, and nothing is commanded and no driver is armed during it, so
# pulling a tool back out inside the window cancels that sweep for free.
INSERTION_DELAY="${DELAY:-2.0}"

# TOOL_SETTLE -- the head start the tool chain's CONSUMERS get.
#
# /rfid/uid, /h4hr/tool_loaded, /h4hr/update_coupling and /h4hr/update_limits
# are all plain volatile publishers at depth 10.  No transient-local, no
# latching: a message published before the other end has finished discovery is
# simply gone, and nothing retransmits it.
#
# So the group leader starts the two auto_zero_nodes and the two
# tool_loader_nodes, waits, and starts the two scanners last.  Get it backwards
# and the failure is the silent one: rfid publishes a UID nobody is subscribed
# to yet, tool_loader never sees it, /h4hr/tool_loaded stays NONE, the MTM gate
# never opens and that wrist simply never moves with no error anywhere.
#
# 3 s is discovery plus margin, not a measured requirement -- rfid_scan_node
# also has to see /h4hr/joint_command go idle before it borrows the port, which
# is its own delay on top of this.  Cheap insurance either way.
TOOL_SETTLE="${TOOL_SETTLE:-3}"

# ---------------------------------------------------------------------------
# VIEWER -- watch_both_arms.py, off by default as of 2026-09-07.
#
# It is a pure READER: no publisher, no service, it never touches /h4hr/* and
# nothing in the run gates on it, so leaving it out changes no behaviour at
# all.  It was costing a subscriber on the 250 Hz /humanoid_rcm/sim_arm topic
# plus MuJoCo rendering, and -- the reason that actually matters at the console
# -- half the right-hand column, which is height the two MTM panes are the ones
# reading.
#
# NOT the executor cost SYSTEM_CONTEXT 9.11 measures: that is about rclpy
# executors CO-LOCATED IN ONE PROCESS, and the viewer has always been its own.
# Dropping it is a console and load decision, not a loop-rate fix.
#
#   VIEWER=1     bring it back (any value but 'off' does)
#
# Worth turning on when the question is geometric -- whether a shaft really
# passes through its amber RCM sphere, which is the one thing no number in the
# pipeline pane can show you.
# ---------------------------------------------------------------------------
VIEWER="${VIEWER:-off}"

# No defaults: the two boards' by-id paths are serial numbers, so they are
# different on every rig.  start_hw.sh reads them from machine_config.sh, and
# leaving either one empty lands in the refusal below, which lists the boards
# that ARE plugged in.
RIGHT_PORT="${RIGHT_PORT:-}"
LEFT_PORT="${LEFT_PORT:-}"

# ---------------------------------------------------------------------------
# Both boards must actually BE there, checked before tmux stages anything.
#
# A missing by-id path used to surface as the PSM driver exiting three seconds
# into the run, inside a pane the operator was not looking at, with the other
# side already live.  On 2026-09-06 the shipped RIGHT_PORT was a board that is
# not on this machine at all and the whole run got that far before saying so.
#
# The two sides are NOT interchangeable: each board carries that wrist's
# DYNAMIXEL chain (and its PN532 on the same handle), so a right driver opening
# the left board drives the wrong wrist -- the wrong-arm hazard SYSTEM_CONTEXT
# 9.6 is about, one layer down.  So this refuses; it never picks for you.
# ---------------------------------------------------------------------------
# The G1 interface, same idea: a name that does not exist here is worth saying
# before the session is staged rather than from inside ChannelFactoryInitialize.
if [ -z "$IFACE" ] || ! ip link show "$IFACE" >/dev/null 2>&1; then
  if [ -z "$IFACE" ]; then
    echo "refusing to start: IFACE, the network interface that reaches the G1, is not set." >&2
  else
    echo "refusing to start: network interface '$IFACE' does not exist." >&2
  fi
  echo >&2
  echo "Interfaces with an address on the G1 subnet (192.168.123.0/24):" >&2
  ip -brief addr show 2>/dev/null | awk '$3 ~ /^192\.168\.123\./ {print "  " $1 "   " $3}' >&2
  echo >&2
  echo "Set IFACE in machine_config.sh, or for one run:" >&2
  echo "    IFACE=<name> bash start_hw.sh" >&2
  exit 1
fi

missing=""
[ -e "$RIGHT_PORT" ] || missing="$missing\n  RIGHT_PORT  ${RIGHT_PORT:-(not set)}"
[ -e "$LEFT_PORT"  ] || missing="$missing\n  LEFT_PORT   ${LEFT_PORT:-(not set)}"
if [ -n "$missing" ]; then
  echo "refusing to start: an OpenRB-150 board is not where this script expects it." >&2
  printf "%b\n" "$missing" >&2
  echo >&2
  echo "OpenRB-150 boards actually present:" >&2
  found=""
  for dev in /dev/serial/by-id/*OpenRB-150*; do
    [ -e "$dev" ] || continue
    found=1
    claim="unclaimed"
    [ "$dev" = "$RIGHT_PORT" ] && claim="= RIGHT_PORT"
    [ "$dev" = "$LEFT_PORT"  ] && claim="= LEFT_PORT"
    printf '  %s  ->  %s   %s\n' "$dev" "$(readlink -f "$dev")" "$claim" >&2
  done
  [ -n "$found" ] || echo "  (none -- check power and USB)" >&2
  echo >&2
  echo "Set RIGHT_PORT and LEFT_PORT in machine_config.sh, or for one run:" >&2
  echo "    RIGHT_PORT=/dev/serial/by-id/<id> bash start_hw.sh" >&2
  echo >&2
  echo "Confirm which physical wrist a board drives before you use it: each" >&2
  echo "board is one wrist's DYNAMIXEL chain, and swapping them drives the" >&2
  echo "wrong arm with no downstream check able to notice." >&2
  exit 1
fi

# export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-98}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# ---------------------------------------------------------------------------
# The left side's remaps, stated ONCE.
#
# Split by which nodes need which, because a node that remaps a name it does
# not use is harmless but a node that misses one is a silent cross-drive.
# Nothing here is a new topic in any source file: these names exist only on
# this command line.
# ---------------------------------------------------------------------------
L_PORT_SVC="-r /h4hr/release_port:=/h4hr/${LEFT_PREFIX}release_port"
L_PORT_SVC="$L_PORT_SVC -r /h4hr/reclaim_port:=/h4hr/${LEFT_PREFIX}reclaim_port"

L_TOOL="-r /rfid/uid:=/rfid/${LEFT_PREFIX}uid"
L_TOOL="$L_TOOL -r /h4hr/tool_loaded:=/h4hr/${LEFT_PREFIX}tool_loaded"
L_TOOL="$L_TOOL -r /h4hr/update_coupling:=/h4hr/${LEFT_PREFIX}update_coupling"
L_TOOL="$L_TOOL -r /h4hr/update_limits:=/h4hr/${LEFT_PREFIX}update_limits"

L_ZERO="-r /h4hr/auto_zero_ready:=/h4hr/${LEFT_PREFIX}auto_zero_ready"
L_ZERO="$L_ZERO -r /h4hr/auto_zero_status:=/h4hr/${LEFT_PREFIX}auto_zero_status"
L_ZERO="$L_ZERO -r /h4hr/confirm_tool_insertion:=/h4hr/${LEFT_PREFIX}confirm_tool_insertion"

L_DIAG="-r /h4hr/limit_hit:=/h4hr/${LEFT_PREFIX}limit_hit"
L_DIAG="$L_DIAG -r /h4hr/motor_currents:=/h4hr/${LEFT_PREFIX}motor_currents"

# The driver: its own port services, the coupling/limits it consumes, and its
# diagnostics.  NOT /h4hr/joint_command or /joint_states -- those are shared
# and separated by joint NAME, which is the whole point of scheme 1.
L_DRIVER_REMAP="$L_PORT_SVC $L_DIAG"
L_DRIVER_REMAP="$L_DRIVER_REMAP -r /h4hr/update_coupling:=/h4hr/${LEFT_PREFIX}update_coupling"
L_DRIVER_REMAP="$L_DRIVER_REMAP -r /h4hr/update_limits:=/h4hr/${LEFT_PREFIX}update_limits"

# ---------------------------------------------------------------------------
# Guard.  Same /proc-reading implementation as the other launchers: a stack
# already running owns the topics this one is about to use, and an orphaned
# --internal-role child is invisible in `tmux ls` while still publishing.
# ---------------------------------------------------------------------------
if [ "${FORCE:-0}" != "1" ]; then
  BLOCKERS=""
  for other in rcm_teleop left_rcm_teleop left_tool both_arms_sim; do
    if tmux has-session -t "$other" 2>/dev/null; then
      BLOCKERS="$BLOCKERS\n  tmux session '$other'   ->  tmux kill-session -t $other"
    fi
  done

  classify_pipeline_pids() {
    local want="$1" pid argv0 args
    for pid in $(pgrep -f "run_full_pipeline\.py\|run_both_arms_pipeline\.py" 2>/dev/null); do
      [ "$pid" = "$$" ] && continue
      [ -r "/proc/$pid/cmdline" ] || continue
      args="$(tr '\0' '\n' < "/proc/$pid/cmdline")"
      argv0="$(printf '%s\n' "$args" | head -1)"
      case "$(basename "$argv0")" in
        python|python3|python3.*) ;;
        *) continue ;;
      esac
      printf '%s\n' "$args" \
        | grep -q 'run_full_pipeline\.py$\|run_both_arms_pipeline\.py$' || continue
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
    BLOCKERS="$BLOCKERS\n  pipeline pid $pid   ->  kill $pid"
  done
  if [ -n "$ORPHANS" ]; then
    BLOCKERS="$BLOCKERS\n  orphaned pipeline children $ORPHANS   ->  kill $ORPHANS"
  fi

  if [ -n "$BLOCKERS" ]; then
    echo "refusing to start: something is already on this ROS graph." >&2
    echo >&2
    echo "This script starts BOTH arms, so an existing stack would duplicate" >&2
    echo "one of them -- and on hardware a second rt/lowcmd publisher fights" >&2
    echo "this one for every motor on the robot.  Found:" >&2
    printf "%b\n" "$BLOCKERS" >&2
    echo >&2
    echo "Clear those, or FORCE=1 if you have separated the sides yourself." >&2
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

# ---------------------------------------------------------------------------
# RCM_SOURCE -- where the two pivots come from.
#
#   RCM_SOURCE=auto      (default) unchanged: each side defers to anything
#                        already publishing /RCM_pose_right|left, and publishes
#                        its rcm.point_base_m otherwise.
#   RCM_SOURCE=detected  both pivots from the ArUco detector.  The pipeline
#                        refuses to start a side whose detector is missing, and
#                        never falls back to the configured point.  Start the
#                        detector BEFORE pressing Enter on the pipeline pane.
#
# Under auto, a detector that is already running counts as that existing
# publisher -- and then every arm drops its messages, because it stamps
# robot_base and only detected mode accepts that name.  The run sits on
# "waiting for RCM pose".  So with the detector up, pass detected.  See
# SYSTEM_CONTEXT 9.26.
# ---------------------------------------------------------------------------
RCM_SOURCE="${RCM_SOURCE:-auto}"
case "$RCM_SOURCE" in
  auto|detected) ;;
  *)
    echo "RCM_SOURCE must be 'auto' or 'detected'; got '$RCM_SOURCE'" >&2
    exit 2
    ;;
esac

# ---------------------------------------------------------------------------
# CONSOLE_LOG -- everything the pipeline pane prints, on disk.
#
# LOG=<dir> already captures the per-cycle CSVs, which say what the CONTROLLER
# did. This captures what the PROCESSES said, which is a different question and
# the one that goes unanswered when a run dies: the reason an arm exits is
# printed once, to a pane that both arms share and that the 2 Hz rate lines
# scroll away. On 2026-09-09 a right-arm exit-1 left no reason anywhere -- the
# controller log showed 5 um of RCM error at the final cycle, so the failure was
# outside the loop entirely, and the message was simply gone.
#
# PYTHONUNBUFFERED=1 rather than `python -u`, because it must reach the CHILD
# processes too. The arms and the merger are separate processes that inherit
# this pane's stdout, and piping into tee turns that from a tty into a pipe --
# which switches Python's stdout to BLOCK buffering, so a child that dies would
# lose whatever it had not flushed. The variable is inherited; -u would not be.
# ---------------------------------------------------------------------------
CONSOLE_LOG="${CONSOLE_LOG:-${LOG:-$HOME/rcm_logs}/console_$(date +%Y%m%d_%H%M%S).log}"
mkdir -p "$(dirname "$CONSOLE_LOG")"

# Where THIS run starts in that file. The default name carries a timestamp so
# the file is new every time and this is 0 -- but CONSOLE_LOG=<fixed path> is a
# documented option, tee appends, and the fault summary below would then report
# last week's fault as this run's. One stat call settles it.
CONSOLE_START=$(stat -c %s "$CONSOLE_LOG" 2>/dev/null || echo 0)

PIPELINE="PYTHONUNBUFFERED=1 python run_both_arms_pipeline.py"
PIPELINE="$PIPELINE --right-config $RIGHT_CONFIG --left-config $LEFT_CONFIG"
PIPELINE="$PIPELINE --left-tool-joint-prefix $LEFT_PREFIX"
PIPELINE="$PIPELINE --network-interface $IFACE"
PIPELINE="$PIPELINE --right-psm-device $RIGHT_PORT --left-psm-device $LEFT_PORT"
PIPELINE="$PIPELINE --left-psm-remap \"$L_DRIVER_REMAP\""
PIPELINE="$PIPELINE --allow-motion --confirm-motion MOVE_G1_PSM_WITH_RCM"
if [ "$COMMAND" != "mtm" ]; then
  PIPELINE="$PIPELINE --command-source $COMMAND"
fi
if [ "$RCM_SOURCE" != "auto" ]; then
  PIPELINE="$PIPELINE --rcm-source $RCM_SOURCE"
fi

# No --psm-tool-config.  tool_loader_node puts the mounted instrument's
# coupling matrix and joint envelope into each driver, from the tag that side
# actually scanned, through the same two driver callbacks the flag used to
# feed.  run_both_arms_pipeline.py still HAS the flag for a no-RFID debug run;
# this launcher no longer passes it, and passing it alongside a live
# tool_loader would just be two writers for one matrix.

# ---------------------------------------------------------------------------
# LOG=<dir>   write a per-cycle CSV for each arm.
#
# The pipeline's own per-cycle record goes to stdout, and stdout dies with the
# tmux session -- so the run that answers "why did that arm stop where it did"
# is over before anyone can read it.  ~/.ros/log keeps only what went through a
# ROS logger, which is the phase TRANSITIONS and not the residual.
#
# These CSVs carry measured_j0..6 beside command_j0..6 for every control cycle,
# which is what separates "one joint pinned against something" from "every
# joint sagging a little" when an arm stops short of the staging pose.
# SYSTEM_CONTEXT 9.11, third contact.
#
#     LOG=~/rcm_logs bash start_both_arms_hw.sh
#
# The runtime stamps each filename with the start time, so a directory can be
# reused across runs without anything being overwritten.
# ---------------------------------------------------------------------------
if [ -n "${LOG:-}" ]; then
  mkdir -p "$LOG"
  PIPELINE="$PIPELINE --right-log $LOG/right.csv --left-log $LOG/left.csv"
  echo "per-cycle CSVs will be written under $LOG"
fi

# ---------------------------------------------------------------------------
# The two MTM tool nodes' command lines, built HERE because they now start on
# the front window and that window is created next.  No sim variant: there is
# always a board, so /h4hr/auto_zero_ready is always really asserted and the
# tool gate stays on.  `require_tool_ready:=false` was a simulation-only
# concession and is gone with the rest of the simulation path -- on hardware it
# would let a wrist move before its instrument was zeroed.
# ---------------------------------------------------------------------------
R_MTM="python mtm_teleop_node.py --ros-args -p arm:=$MTM_RIGHT"
R_MTM="$R_MTM -r __node:=mtm_teleop_node_right"
L_MTM="python mtm_teleop_node.py --ros-args -p arm:=$MTM_LEFT"
L_MTM="$L_MTM -p joint_prefix:=$LEFT_PREFIX -r __node:=mtm_teleop_node_left"
L_MTM="$L_MTM $L_TOOL $L_ZERO"

# THE GATE IS ARMED ON BOTH SIDES, and there is no longer any way to turn it
# off from here.  `require_tool_ready:=false` was FIXED_TOOL's concession and
# it went with it.  The gate is two checks and the tool chain answers both:
#
#   "an instrument is mounted"   /h4hr/tool_loaded, from the tag this side
#                                actually scanned -- not a name typed into a
#                                launcher and never compared with anything.
#   "it has been auto-zeroed"    /h4hr/auto_zero_ready, after the sweep that
#                                proves the disks reach their travel.
#
# So a wrist that never moves is now a REAL answer about that side's chain, not
# a gate left armed with nothing to open it.  mtm_teleop_node prints its
# gate_reason; read it in that side's MTM pane and work backwards along the
# column in window 'tool' -- auto_zero, tool_loader, rfid, in that order.

# ---------------------------------------------------------------------------
# JAW -- who owns grip_joint.
#
#   JAW=mtm     the handle's gripper, as always.
#   JAW=servo   mtm_teleop_node publishes roll/pitch/yaw ONLY, and the jaw
#               belongs to servo_step.py --jaw / --open / --close.
#
# Defaults to servo under COMMAND=servo and to mtm otherwise, because that is
# the split each mode implies -- but it is separable, since the two are
# genuinely independent: the arm's setpoint source and the jaw's owner are
# different topics.
#
# This matters rather than being cosmetic. mtm_teleop_node publishes all four
# joints at handle rate, and /h4hr/joint_command has no arbitration -- last
# writer wins -- so with JAW=mtm a servo jaw command is overwritten within
# ~10 ms and the jaw judders at the beat frequency between the two publishers.
# The driver merges by NAME, so dropping grip_joint from one publisher is all
# it takes to separate them cleanly.
# ---------------------------------------------------------------------------
if [ "$COMMAND" = "servo" ]; then
  JAW="${JAW:-servo}"
else
  JAW="${JAW:-mtm}"
fi
case "$JAW" in
  mtm|servo) ;;
  *)
    echo "JAW must be 'mtm' or 'servo'; got '$JAW'" >&2
    exit 2
    ;;
esac
if [ "$JAW" = "servo" ]; then
  R_MTM="$R_MTM -p publish_grip:=false"
  L_MTM="$L_MTM -p publish_grip:=false"
fi

# Every pane staged EXCEPT the pipeline's, collected as it is staged.  A
# Ctrl+C on the pipeline pane fans out to these; see FANOUT below.  Built here
# rather than written out by hand so an option that adds a pane -- VIEWER,
# the tool window, READOUT -- cannot quietly leave that pane out of the stop.
FANOUT=""

# One helper for every pane in the session.  It takes the working directory
# because the pipeline and the viewer run from the bundle while the instrument
# and MTM nodes run from the workspace root.
#
# ENV_SETUP runs in each pane right after the cd, because a tmux pane is a
# fresh shell: it has whatever the tmux SERVER's environment had when it
# started, not this terminal's.  Without it, a ROS 2 sourced only in the
# terminal that ran this script is simply absent from every pane.
ENV_SETUP="${ENV_SETUP:-}"
start_pane() {
  tmux select-pane -t "$1" -T "$2"
  tmux send-keys -t "$1" "cd \"$3\"${ENV_SETUP:+ && $ENV_SETUP}" ENTER
  tmux send-keys -t "$1" "$4"
  [ "$1" = "$P_PIPE" ] || FANOUT="$FANOUT $1"
}

# ---------------------------------------------------------------------------
# FRONT WINDOW.  Everything that drives the robot, on one page:
#
#   +-----------------------------+---------------------------+
#   |                             |  RIGHT tool   MTM         |
#   |  run_both_arms_pipeline.py  |                           |
#   |  <-- type 'enable' here     +---------------------------+
#   |                             |  LEFT  tool   MTM         |
#   |                             |                           |
#   +-----------------------------+---------------------------+
#
#   With VIEWER=1 the right column gains watch_both_arms.py on top and the
#   two MTM panes drop to a quarter of the height each.
#
# The pipeline keeps the larger half because it is the only pane anyone TYPES
# into; the other three are read.
#
# Pane IDs, never indices: tmux renumbers indices by screen position on every
# split, so an index captured before the next split is not the pane it was
# (SYSTEM_CONTEXT 8.1 records measuring exactly that).
# ---------------------------------------------------------------------------
P_PIPE=$(tmux new-session -d -s "$SESSION" -n teleop -P -F '#{pane_id}' \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")
if [ "$VIEWER" != "off" ]; then
  # Viewer on top of the right column, the two MTM panes under it.
  P_WATCH=$(tmux split-window -P -F '#{pane_id}' -h -l 45% -t "$P_PIPE")
  P_MTM_R=$(tmux split-window -P -F '#{pane_id}' -v -t "$P_WATCH")
else
  # No viewer: the right column is just the two handles, half each, so both
  # get twice the height they had under it.
  P_MTM_R=$(tmux split-window -P -F '#{pane_id}' -h -l 45% -t "$P_PIPE")
fi
P_MTM_L=$(tmux split-window -P -F '#{pane_id}' -v -l 50% -t "$P_MTM_R")

tmux set-option -t "$SESSION":teleop pane-border-status top >/dev/null 2>&1 || true
# The default 2000 lines is about four minutes at this pane's print rate, and
# the interesting line is always the one that just scrolled off.
tmux set-option -t "$SESSION" history-limit 200000 >/dev/null 2>&1 || true

# stdin stays the terminal -- tee is downstream of the pipeline, so typing
# 'enable' still reaches it and sys.stdin.isatty() is still true.
#
# `trap '' INT` before tee, and it is not cosmetic. Ctrl+C in this pane goes to
# the whole foreground process group, tee included, so tee used to die at the
# same instant as the launcher -- and the launcher's teardown STARTS with a
# print. That print hit a pipe with no reader, and the BrokenPipeError came out
# of Supervisor.shutdown() before anything had been signalled, leaving the arm
# merger publishing rt/lowcmd at 250 Hz with no parent. With INT ignored, tee
# outlives the Ctrl+C, drains the whole teardown into the log, and exits at EOF
# once every process holding this stdout is gone.
#
# So the pane does not return to a prompt until the run is really over. That is
# the point: a prompt while the robot is still stiff is the state this comment
# is about.
PIPELINE="$PIPELINE 2>&1 | { trap '' INT; tee -a '$CONSOLE_LOG'; }"
# ...and typed into its pane at the very BOTTOM of this script, once every
# other pane exists and $FANOUT is complete.  Nothing runs until the operator
# presses Enter, so the order the panes are TYPED in does not matter.
if [ "$VIEWER" != "off" ]; then
  start_pane "$P_WATCH" "viewer  both arms, both tools, both RCMs" \
    "$BUNDLE" "python watch_both_arms.py"
fi
# GROUP 3, the MTM leader.  Enter here starts BOTH handles.  The two are a
# unit at the console -- arming one handle and not the other is a state the
# operator has no reason to want and every reason to mistake for a dead node,
# since a wrist that does not move looks the same either way.
#
# No settle between them: these two talk to nothing of each other's, and both
# only ever WAIT on the tool chain's topics rather than racing them.
start_pane "$P_MTM_R" "BOTH HANDLES  <-- PRESS ENTER HERE   $MTM_RIGHT  roll_joint ..." \
  "$DASH" "tmux send-keys -t $P_MTM_L ENTER; $R_MTM"
start_pane "$P_MTM_L" "LEFT tool  $MTM_LEFT  ${LEFT_PREFIX}roll_joint ...  (starts with the pane above)" \
  "$DASH" "$L_MTM"

# ---------------------------------------------------------------------------
# The two instrument chains.  Pane IDs, never indices: tmux renumbers indices
# by screen position on every split, so an index captured before the next split
# is not the pane it was (SYSTEM_CONTEXT 8.1 records measuring exactly that).
#
# The DRIVERS are not here -- run_both_arms_pipeline starts both of them, and
# starts them disarmed, so that the pipeline owns their lifetime the way the
# single-arm launcher does.
# ---------------------------------------------------------------------------
#
# The COLUMN split comes first, while the pane is still full height, and each
# side is then stacked inside its own column.  Splitting one side's top pane
# horizontally after it had already been stacked (what this did until
# 2026-09-06) cuts only that pane, so the right side ended up with two 80-wide
# panes below a 40-wide one and the sides no longer read as columns.  The 67%
# then 50% splits divide each column into even thirds.
#
# ALWAYS STAGED as of 2026-09-17.  This window was conditional on
# FIXED_TOOL=off for ten days; the instrument is scanned again and there is no
# other path, so there is no condition left to write.
#
# The DRIVERS are not here -- run_both_arms_pipeline starts both of them, and
# starts them disarmed, so that the pipeline owns their lifetime.

P_R_RFID=$(tmux new-window -d -t "$SESSION" -n tool -P -F '#{pane_id}' \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")
P_L_RFID=$(tmux split-window -P -F '#{pane_id}' -h -l 50% -t "$P_R_RFID")
P_R_LOAD=$(tmux split-window -P -F '#{pane_id}' -v -l 67% -t "$P_R_RFID")
P_R_ZERO=$(tmux split-window -P -F '#{pane_id}' -v -l 50% -t "$P_R_LOAD")
P_L_LOAD=$(tmux split-window -P -F '#{pane_id}' -v -l 67% -t "$P_L_RFID")
P_L_ZERO=$(tmux split-window -P -F '#{pane_id}' -v -l 50% -t "$P_L_LOAD")

tmux set-option -t "$SESSION":tool pane-border-status top >/dev/null 2>&1 || true

# The six commands, built before any of them is typed, because the leader's
# command line has to name the other five panes.
R_RFID="python rfid_scan_node.py --ros-args -p device_name:=$RIGHT_PORT"
R_LOAD="python tool_loader_node.py"
R_ZERO="python auto_zero_node.py --ros-args -p insertion_delay_sec:=$INSERTION_DELAY"

L_RFID="python rfid_scan_node.py --ros-args -p device_name:=$LEFT_PORT -p joint_prefix:=$LEFT_PREFIX -r __node:=rfid_scan_node_left $L_PORT_SVC $L_TOOL"
L_LOAD="python tool_loader_node.py --ros-args -p joint_prefix:=$LEFT_PREFIX -r __node:=tool_loader_node_left $L_TOOL"
L_ZERO_CMD="python auto_zero_node.py --ros-args -p joint_prefix:=$LEFT_PREFIX -p insertion_delay_sec:=$INSERTION_DELAY -r __node:=auto_zero_node_left $L_TOOL $L_ZERO"

# RIGHT_TOOL_UID / LEFT_TOOL_UID=<uid> -- no scanning on that side.  Its rfid
# pane republishes this rfid_registry.json UID on its own /rfid/uid at 1 Hz
# instead of running rfid_scan_node, so that driver never releases its port.
# tool_loader, auto_zero and the MTM gate run unchanged off the fake tag --
# which means that wrist sweeps about TOOL_SETTLE + DELAY s after the
# tool-chain Enter, whether or not anything is mounted.  SEAT THE INSTRUMENT
# FIRST.  Empty (default) scans that side as before.
if [ -n "${RIGHT_TOOL_UID:-}" ]; then
  R_RFID="ros2 topic pub -r 1 /rfid/uid std_msgs/msg/String \"{data: '$RIGHT_TOOL_UID'}\""
fi
if [ -n "${LEFT_TOOL_UID:-}" ]; then
  L_RFID="ros2 topic pub -r 1 /rfid/${LEFT_PREFIX}uid std_msgs/msg/String \"{data: '$LEFT_TOOL_UID'}\""
fi

# ---------------------------------------------------------------------------
# GROUP 2, the tool leader.  ONE Enter on the RIGHT rfid pane starts all six.
#
# CONSUMERS FIRST, SCANNERS LAST, and the order is the whole point -- see
# TOOL_SETTLE above for why a missed message here is unrecoverable and silent.
# Down the chain: auto_zero listens to tool_loader, tool_loader listens to
# rfid.  So both auto_zeros, then both tool_loaders, then $TOOL_SETTLE seconds,
# then both scanners.
#
# The leader's own rfid command is the LAST thing on its line, so the right
# scanner starts in step with the left one rather than a whole group ahead of
# it.  That is why the leader is a scanner and not, say, the right auto_zero:
# the pane that must go last is the only one that can hold the others' Enter.
# ---------------------------------------------------------------------------
TOOL_LEAD=""
for follower in "$P_R_ZERO" "$P_L_ZERO" "$P_R_LOAD" "$P_L_LOAD"; do
  TOOL_LEAD="$TOOL_LEAD tmux send-keys -t $follower ENTER;"
done
TOOL_LEAD="$TOOL_LEAD echo \"tool chain: consumers up, scanners in ${TOOL_SETTLE}s...\";"
TOOL_LEAD="$TOOL_LEAD sleep $TOOL_SETTLE;"
TOOL_LEAD="$TOOL_LEAD tmux send-keys -t $P_L_RFID ENTER;"

start_pane "$P_R_RFID" "TOOL CHAIN  <-- PRESS ENTER HERE   starts all six, both sides" \
  "$DASH" "$TOOL_LEAD $R_RFID"
start_pane "$P_R_LOAD" "RIGHT tool_loader  (starts with the rfid pane above)" \
  "$DASH" "$R_LOAD"
start_pane "$P_R_ZERO" "RIGHT auto_zero  hold-off ${INSERTION_DELAY}s  (starts with the rfid pane)" \
  "$DASH" "$R_ZERO"

start_pane "$P_L_RFID" "LEFT rfid  ${LEFT_PREFIX}*  (starts with the RIGHT rfid pane)" \
  "$DASH" "$L_RFID"
start_pane "$P_L_LOAD" "LEFT tool_loader  ${LEFT_PREFIX}*  (starts with the RIGHT rfid pane)" \
  "$DASH" "$L_LOAD"
start_pane "$P_L_ZERO" "LEFT auto_zero  ${LEFT_PREFIX}*  hold-off ${INSERTION_DELAY}s" \
  "$DASH" "$L_ZERO_CMD"

# Land on the leader, so window 'tool' opens with the one pane to press Enter
# in already selected.  Without this tmux leaves the LAST pane split selected,
# which is the left auto_zero -- a follower, and the one pane whose early start
# would put it ahead of the tool_loader it listens to.
tmux select-pane -t "$P_R_RFID"

# ---------------------------------------------------------------------------
# READOUT=<dir>   stage the two END EFFECTOR readouts, one pane per side.
#
# Off by default, and a WINDOW rather than two more panes, for the reason the
# header gives about the tool chains: the front page is already the smallest it
# can be and still be read. Same shape as LOG=<dir> above -- name a directory,
# get a CSV per side -- because this answers the same question one layer out.
#
# LOG= records what the ARM did, measured against command, from rt/lowstate.
# This records where the INSTRUMENT TIP ended up: forward kinematics run twice
# per sample, once on the interpolator's command and once on the encoders'
# answer, and the difference between the two tip poses in mm and degrees.
# Until the driver started sampling Present Position(132) that difference could
# not be formed at all -- the instrument half of /joint_states was a command
# echo, so FK on it agreed with the command by construction.  SYSTEM_CONTEXT 10.
#
# --source live, never the default 'sim'.  Two things turn on it: these are the
# G1's own encoders rather than the pipeline's targets, and /humanoid_rcm/sim_arm
# is ONE topic that both arms publish onto -- section 9.6's wrong-arm hazard
# reaches a reader too.
#
# Read-only. The transport is built allow_motion=False, so enable() raises and
# every write path refuses; these attach to the running stack and start nothing.
# Start them LAST, after the arms are enabled, and Ctrl-C each to write its CSV
# and print its summary.
# ---------------------------------------------------------------------------
if [ -n "${READOUT:-}" ]; then
  mkdir -p "$READOUT"

  P_R_READ=$(tmux new-window -d -t "$SESSION" -n readout -P -F '#{pane_id}' \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION")
  P_L_READ=$(tmux split-window -P -F '#{pane_id}' -h -l 50% -t "$P_R_READ")

  tmux set-option -t "$SESSION":readout pane-border-status top >/dev/null 2>&1 || true

  start_pane "$P_R_READ" "RIGHT tip readout  ->  $READOUT/right_tip.csv" \
    "$BUNDLE" \
    "python pose_readback.py --config $RIGHT_CONFIG --source live --network-interface $IFACE --log $READOUT/right_tip.csv"
  start_pane "$P_L_READ" "LEFT tip readout  ->  $READOUT/left_tip.csv" \
    "$BUNDLE" \
    "python pose_readback.py --config $LEFT_CONFIG --source live --network-interface $IFACE --tool-joint-prefix $LEFT_PREFIX --log $READOUT/left_tip.csv"

  echo "end effector readout staged in window 'readout'; CSVs under $READOUT"
fi

# ---------------------------------------------------------------------------
# The banner.  One mode now -- the start ORDER is the thing an operator follows
# off this screen, and there is only one order left to follow.
# ---------------------------------------------------------------------------
if [ "$VIEWER" != "off" ]; then
  VIEWER_BANNER="
            watch_both_arms.py          both arms and both RCMs in one MuJoCo
            <-- a FOURTH Enter          window.  NOT in any start group: it is
                                        a reader, started and stopped when you
                                        want it, any time after the pipeline.
                                        ON HARDWARE the ARMS and the RCM
                                        markers are live; the instrument WRIST
                                        is not -- see the note below."
else
  VIEWER_BANNER=""
fi

# NOT `${READOUT:+...}` inline in the heredoc, and this is a BUG FIX rather
# than a style change.  Inside ${VAR:+word} bash still does QUOTE PROCESSING on
# the word, so the apostrophe in "the ENCODERS' tip pose" opened a single-quoted
# string that ran past the closing brace to the next apostrophe in the banner --
# `window 'tool'`, twenty lines further down -- and the expansion swallowed
# everything between them.  The shipped banner therefore printed the windows and
# then jumped straight to the joint names: no start order, no TO STOP THE RUN,
# no orphan check, no matter what READOUT was set to.  Silent, because the
# braces happened to rebalance; caught only by diffing the banner against the
# heredoc that was supposed to produce it.
#
# A plain variable has no such hazard: assigned once, expanded where nothing
# re-parses its quotes.  It is what VIEWER_BANNER and TOOL_BANNER already do.
if [ -n "${READOUT:-}" ]; then
  READOUT_BANNER="
  window 'readout'  END EFFECTOR readout, one pane per side.  Commanded tip
                    pose against the ENCODERS' tip pose, and the difference.
                    Read-only; start these LAST, Ctrl-C to write the CSV."
else
  READOUT_BANNER=""
fi

TOOL_BANNER="  window 'tool'     the two instrument chains, one column per side

                        RIGHT                    LEFT
                        rfid  <-- ENTER HERE     rfid
                        tool_loader              tool_loader
                        auto_zero                auto_zero

                    ONE Enter, on the RIGHT rfid pane, starts all six: both
                    auto_zeros and both tool_loaders first, then both scanners
                    ${TOOL_SETTLE}s later.  The other five say so in their titles.
                    Every node keeps its own pane and its own scrollback."

ORDER_BANNER="  THREE Enters, in this order, then one word.  Each starts a whole group.

    1.  PIPELINE   this pane, window 'teleop'
                   -> both arms, the merger, and both PSM drivers DISARMED.
                      Wait for both drivers to print their joint names.

    2.  TOOL       Ctrl-b n  (window 'tool'; the right rfid pane is already
                   selected for you), then Enter
                   -> all six instrument nodes.  Now FIT BOTH INSTRUMENTS.
                      Each side: tag read -> coupling + envelope into that
                      driver -> ${INSERTION_DELAY}s hold-off -> the disks sweep
                      -> /h4hr/auto_zero_ready.  THE WRISTS MOVE HERE, and
                      auto_zero arms the drivers itself to do it.
                      Both columns must reach 'auto-zero complete'.

    3.  MTM        Ctrl-b p  (back to 'teleop'), Ctrl-b o  (to the right
                   column), then Enter
                   -> both mtm_teleop_nodes.  Neither wrist moves until its
                      side's gate sees tool_loaded AND auto_zero_ready, so a
                      handle that does nothing here is a real answer about
                      that side's chain -- read its gate_reason.

    4.  enable     back on this pane (Ctrl-b o until you are here).
                   /humanoid_rcm/enable is shared, so one gate arms BOTH arms.

  With VIEWER=1 the viewer pane is a fourth Enter, outside the groups, any time
  after step 1.  WHAT IT SHOWS ON HARDWARE, because half of it is not obvious:

    LIVE   both G1 arms.  /humanoid_rcm/sim_arm is published on hardware too --
           by publish_arm_target, the channel the arms feed the merger on --
           under the real model joint names the viewer keys on.  These are the
           COMMANDED targets, not the encoders.
    LIVE   both RCM markers, off /RCM_pose_right and /RCM_pose_left, with the
           amber pivot sphere, the cyan shaft, each side's lateral error, and
           the magenta closest-point and gap line when the error is visible.
           The shaft and the error depend only on the seven ARM joints, so
           this half is right even though the wrist below is not.
    FROZEN the instrument wrist and jaws, at the model rest pose.  Only
           publish_sim_arm emits those and every call to it sits under an
           'if simulate:'; on hardware the wrist is driven by mtm_teleop_node
           over /h4hr/joint_command, which this viewer does not read.  The
           wrist joints are DISTAL to the RCM, so this costs nothing for
           watching the pivot -- only for watching the jaws."

# ---------------------------------------------------------------------------
# FANOUT -- one Ctrl+C stops the SESSION, not just the launcher.
#
# Ctrl+C reaches only the foreground process group of the pane it is typed
# into. The launcher's teardown covers everything IT started -- both arms, the
# merger, both PSM drivers, the RCM publishers -- because those are its
# children. It cannot cover the panes beside it: the two MTM nodes, the six
# instrument nodes in window 'tool', the viewer, the readouts under READOUT.
# Those are separate process groups in separate panes -- in a different WINDOW,
# for the tool chain -- and a Ctrl+C here never touched them.
#
# This is the other half of THREE STARTS, ONE STOP. Starting is deliberately
# three keypresses because the groups depend on each other in an order; STOPPING
# has no such order, so it is one. Pane IDs are session-global, which is why one
# fan-out reaches window 'tool' without ever selecting it.
#
# What that left behind is not harmless. The MTM nodes go on publishing
# /h4hr/joint_command with their drivers gone -- nothing moves, the drivers
# torque off on their own SIGINT handler -- but they are still on the graph,
# and NOTHING in the running-stack guard looks for them. Start the panes again
# and each side has two mtm_teleop_nodes publishing the same joints at handle
# rate, which is the last-writer-wins judder that SYSTEM_CONTEXT 9.9 is about.
#
# So the pipeline pane's command line ends by sending Ctrl+C to every other
# pane, as the next command on the line.
#
# THE PRECONDITION, measured in real tmux on 2026-09-17 because this comment
# used to claim "for ANY reason" and that is not true: bash ABANDONS the rest
# of a command list when the foreground job is killed BY SIGINT. If the
# pipeline ever dies from SIGINT instead of catching it, the fan-out and the
# fault summary after it are both skipped, silently, and every other pane keeps
# running -- the two-mtm_teleop_node judder described above.
#
# It works because run_both_arms_pipeline.py CATCHES KeyboardInterrupt and
# exits normally, so bash sees an ordinary exit status and carries on down the
# line. Probed both ways: with a child that catches SIGINT the siblings got
# their Ctrl+C and the summary printed; with one that does not, neither ran.
#
# That is the real reason tests/test_teardown_survives_the_console.py insists
# on the KeyboardInterrupt handler. It is not just about a tidy shutdown inside
# the supervisor -- it is what keeps every command after the pipe reachable.
#
# `send-keys C-c` and not `kill-pane`: it delivers SIGINT to that pane's
# foreground group, which is what each of those processes handles. The
# readouts write their CSV on SIGINT and would lose it to a kill; every pane
# keeps its shell, its prompt and its scrollback, so the reason a run ended is
# still on the screen to read. A pane sitting at a prompt just gets its line
# cancelled.
#
# Pane IDs, never indices: this list is built while the panes are staged and
# tmux renumbers indices on every split.
# ---------------------------------------------------------------------------
STOP_SIBLINGS=""
for pane in $FANOUT; do
  STOP_SIBLINGS="$STOP_SIBLINGS; tmux send-keys -t $pane C-c"
done
PIPELINE="$PIPELINE$STOP_SIBLINGS"

# ---------------------------------------------------------------------------
# ...and then say what went wrong, as the LAST thing in the pane.
#
# The reason a run ends is printed once. The status and rate lines print twice
# a second, so by the time anyone reads the pane that one line is several
# hundred lines up -- in a tmux pane whose scrollback an operator mid-procedure
# is not going to page through. The line exists; it is simply not where anyone
# will look.
#
# So Ctrl+C now ends with it. This runs after the pipeline's pipe has closed
# (so tee has flushed the whole teardown into the log) and after the fan-out,
# which puts it at the bottom of the screen where the prompt is about to be.
#
# It reads the console log rather than watching the processes: that is what the
# pane already tees, so the summary cannot disagree with what was printed, and
# nothing in the control loop had to change to produce it. Identical fault
# lines fold into one entry with a count -- a fault repeating at 2 Hz is one
# condition persisting, not six hundred problems.
#
# show_run_faults.py returns 0 on every path including its own failure. It is
# the last command in a pane whose job is stopping a robot, and a traceback out
# of the summariser would land exactly where the reason is supposed to be.
# ---------------------------------------------------------------------------
PIPELINE="$PIPELINE; python show_run_faults.py '$CONSOLE_LOG' --from-byte $CONSOLE_START"

start_pane "$P_PIPE" "PIPELINE  both arms + merger   <-- type 'enable' here" \
  "$BUNDLE" "$PIPELINE"

# Window 'teleop' is what an operator lands on.
tmux select-window -t "$SESSION":teleop
tmux select-pane -t "$P_PIPE"

cat <<BANNER

  session '$SESSION' staged: BOTH G1 ARMS + BOTH TOOLS -- HARDWARE
  jaw owner:      $JAW$([ "$JAW" = servo ] && echo "  <-- the MTM GRIPPER DOES NOTHING.
                  Use: python servo_step.py --side right --close | --open")
  console log:    $CONSOLE_LOG
  command source: $COMMAND$([ "$COMMAND" = servo ] && echo "  <-- servo_cp
                  THE CLUTCH DOES NOT STOP THIS MODE. Stop with 'hold' on this
                  console, /humanoid_rcm/enable false, or /h4hr/estop.")
  RCM source:     $RCM_SOURCE$([ "$RCM_SOURCE" = detected ] && echo "  <-- ArUco detector, latched at each
                  enable; no fallback to rcm.point_base_m")

  window 'teleop'   everything that drives the robot, one page
    left    run_both_arms_pipeline.py   BOTH arms, one process each, plus the
            <-- TYPE 'enable' HERE      rt/lowcmd merger
                                          RIGHT <- $MTM_RIGHT position, /RCM_pose_right
                                          LEFT  <- $MTM_LEFT position, /RCM_pose_left
    right   RIGHT tool  $MTM_RIGHT           orientation + gripper, roll_joint ...
            <-- ENTER HERE starts BOTH
            LEFT  tool  $MTM_LEFT           orientation + gripper, ${LEFT_PREFIX}roll_joint ...$VIEWER_BANNER

$TOOL_BANNER
$READOUT_BANNER

  Nothing is running yet.  Attach:

      tmux attach -t $SESSION

$ORDER_BANNER

  TO STOP THE RUN: Ctrl+C on the PIPELINE pane, ONCE.  Three starts, one stop.
  The pane then ends with WHAT WENT WRONG IN THIS RUN -- the faults and any
  traceback, pulled back out of the console log, with repeats folded into a
  count.  You do not have to scroll up: the reason is the last thing printed.
  A clean stop says so in as many words, so silence there means the summary
  itself failed, not that the run was fine.
  That tears down both arms, the merger, both PSM drivers and the RCM
  publishers in that order, and then sends Ctrl+C to every other pane in this
  session -- both MTM nodes and all six instrument nodes in window 'tool',
  which you never have to switch to.  Each pane keeps its scrollback, so the
  reason a run ended is still on the screen; the whole teardown is also in
  $CONSOLE_LOG.

  Wait for it.  The pane does not return to a prompt until every process is
  really gone -- that is deliberate.  A prompt while the robot is still stiff
  is the state worth being afraid of.

  BEFORE THE NEXT RUN, if anything looked wrong at shutdown:

      pgrep -af internal-role

  Nothing should print.  An orphaned merger still owns rt/lowcmd -- it will
  refuse to command a robot whose rt/lowstate it has lost, and exit, but do not
  rely on that: this launcher also refuses to start on top of one.

  The merger is the only rt/lowcmd publisher in the run.  If it exits, the
  pipeline refuses to start the arms rather than leaving them commanding
  nothing.  It gives up on a robot that has gone quiet for more than a second
  and will not resume -- so a POWER CYCLE ends the run rather than being ridden
  out into a jolt when the robot comes back.

  ONE PEDAL TRAY, TWO TOOLS.  The clutch freezes BOTH wrists and a coag tap
  re-homes BOTH.  There is one tray; this is the only behaviour it can have.

  IF A WRIST NEVER MOVES while its arm tracks fine, that side's nodes disagree
  about the joint names.  Every one of them prints its names at startup:

      ${LEFT_PREFIX}{roll,pitch,yaw,grip}_joint   left
      {roll,pitch,yaw,grip}_joint     right

  IF A TAG IS NEVER READ, that side's column in window 'tool' says where it
  stopped.  rfid_scan_node only borrows the board once /h4hr/joint_command has
  gone IDLE, and idle means the commanded POSE stopped changing -- that topic
  publishes at 60 Hz whether anything moves or not, so a scanner waiting on a
  twitching wrist prints "Searching every 1.0s" forever and the chain never
  starts.  A scan takes the serial port from that driver for ~0.4 s at a time;
  that is expected and gated (SYSTEM_CONTEXT 9.11), and it is why an arm must
  be holding still while its instrument is fitted.

  Check the ports first if you have not:  python place_rcm_both_arms.py --report

BANNER
