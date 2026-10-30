#!/usr/bin/env python3
"""
Reading a tool_configs/*.json instrument description, in ONE place.

Two things now need the mounted instrument's coupling matrix and joint
envelope, and they must not disagree:

  * tool_loader_node, which learns the instrument from an RFID tag and
    publishes /h4hr/update_coupling and /h4hr/update_limits, and
  * dynamixel_controller_node's `tool_config` parameter, which names the
    instrument up front for runs with no RFID chain at all.

If those two derived the numbers separately, a fixed-tool run and a scanned
run would drive the same instrument through different arithmetic -- and the
difference would show up as a wrist that tracks slightly wrong, which is
exactly the kind of fault nothing downstream can notice. So both call the
functions here and neither owns a second copy.

Nothing in this module touches ROS. It is a pure config reader, which is what
lets the driver use it before rclpy has a publisher of any kind.
"""

import json
import math
import os
import re

import ament_index_python.packages


def find_tool_config_dir():
    """
    Locate tool_configs/, preferring the installed package but falling back to
    the copy beside this module.

    The ament lookup only works when the package is registered in the ament
    index, which requires the resource marker in setup.py's data_files. These
    nodes are normally run straight from the source tree with `python3
    tool_loader_node.py`, where that marker need not exist -- so an ament-only
    lookup raises PackageNotFoundError and takes the node down at import time.
    """
    try:
        share = ament_index_python.packages.get_package_share_directory(
            'hand_for_humanoid_robot'
        )
        candidate = os.path.join(share, 'tool_configs')
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass

    return os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tool_configs')


TOOL_CONFIG_DIR = find_tool_config_dir()
REGISTRY_FILE = os.path.join(TOOL_CONFIG_DIR, 'rfid_registry.json')

# Sentinel published on /rfid/uid and /h4hr/tool_loaded when nothing is
# mounted.  rfid_scan_node, tool_loader_node, auto_zero_node and
# mtm_teleop_node all compare against this exact string.
NO_TAG = 'NONE'

# Grip opening published for EVERY tool, in degrees, overriding whatever the
# mounted instrument's config says its jaw.qmin is.
#
# The jaw numbers in tool_configs/ are ISI's, and they describe ISI's
# cable-driven instruments -- not this hand's differential-disk mechanism. They
# disagree wildly for that reason: 32 of the 46 configs say -20 deg, six say
# -40, five say 0 (the hook and spatula, which have no jaws at all), and
# FENESTRATED_BIPOLAR_FORCEPS_400205 says -200 because someone ramped it on
# hardware and found no stall -- its own comment calls that "test ceiling, not
# a confirmed hard stop". So whichever instrument happened to be mounted
# decided how far the jaws opened, and the same handle motion that opened the
# fenestrated forceps fully barely cracked the Maryland at its -20.
#
# The mechanism is the same whatever is screwed onto it, so the opening should
# be too. -60 is the number the rest of the stack already assumes: it is
# dynamixel_controller_node's own min_grip_deg default, auto_zero_node's
# DEFAULT_LIMITS grip, and DEFAULT_MIN_GRIP_DEG immediately below -- the
# envelope the bare wrist runs under with no tool mounted. Publishing it for
# loaded tools too is what makes "tool mounted" stop changing the answer.
#
# Deliberately a blanket override rather than a floor: a floor would leave the
# fenestrated forceps at -200 and keep the inconsistency for the one tool that
# has been measured furthest.
GRIP_OPEN_DEG = -60.0

# Fallback envelope used when no tool is loaded. Mirrors the parameter
# defaults in dynamixel_controller_node.py.
DEFAULT_MAX_ROLL_DEG = 259.0
DEFAULT_MAX_PITCH_DEG = 79.0
DEFAULT_MAX_YAW_DEG = 79.0
DEFAULT_MIN_GRIP_DEG = -60.0
DEFAULT_MAX_GRIP_DEG = 38.0

# Jaw qmax used when a config carries no "jaw" block at all (80 degrees).
DEFAULT_JAW_QMAX_RAD = 1.39626


def normalize_uid(uid):
    """
    Canonical form of a tag UID: uppercase hex, no separators.

    The two OpenRB firmwares format the same tag differently.  The older
    right-hand board emits space-separated bytes ("F4 03 0A E3"); the left
    board's sketch prints them run together ("F4030AE3").  rfid_registry.json
    was written against the spaced form, so an exact dict lookup recognises a
    tool on one board and calls it UNREGISTERED on the other -- which mutes the
    MTM, because mtm_teleop_node gates on /h4hr/tool_loaded.

    Comparing canonical forms makes the registry serve both boards and keeps it
    editable in whichever style reads better.  Separators are stripped rather
    than normalised to one style so that ":" and "-" pasted from other tools
    also match.  The sentinel NONE passes through unchanged.
    """
    return ''.join(str(uid).split()).replace(':', '').replace('-', '').upper()


def strip_comments(text):
    """
    Remove the comments the tool_configs/ files carry, which json cannot take.

    Both forms are needed and neither is optional: every config opens with a
    /* -*- Mode: Javascript ... -*- */ block, and the bodies are annotated with
    // lines. The (?<!:) guard keeps a "//" that follows a colon -- a URL in a
    value -- from eating the rest of that line.
    """
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'(?<!:)//[^\n]*', '', text)
    return text


def load_json_with_comments(filepath):
    """Parse one tool config file."""
    with open(filepath, 'r') as f:
        text = f.read()
    return json.loads(strip_comments(text))


def resolve_tool_config_path(name):
    """
    Turn what an operator types into a path under tool_configs/.

    Accepts, in order of preference:

      LARGE_NEEDLE_DRIVER_400006          the bare config name
      LARGE_NEEDLE_DRIVER_400006.json     the file name
      /some/where/else.json               an explicit path, taken as given

    A bare name is the form that appears in rfid_registry.json's "name" field
    and on tool_loader_node's own "Tool loaded:" line, so it is the string an
    operator has already seen; requiring the .json suffix would make the two
    spellings of the same instrument mean different things.

    Raises FileNotFoundError naming the directory searched, because the failure
    an operator actually hits is a typo, and a bare "no such file" would send
    them looking at the launcher instead of at what they typed.
    """
    name = str(name).strip()
    if not name:
        raise ValueError('empty tool config name')

    if os.path.isabs(name) or os.sep in name:
        if os.path.isfile(name):
            return name
        raise FileNotFoundError(f'tool config not found: {name}')

    candidates = [name] if name.endswith('.json') else [name + '.json', name]
    for candidate in candidates:
        path = os.path.join(TOOL_CONFIG_DIR, candidate)
        if os.path.isfile(path):
            return path

    available = sorted(
        entry[:-len('.json')]
        for entry in os.listdir(TOOL_CONFIG_DIR)
        if entry.endswith('.json') and entry not in ('index.json', 'rfid_registry.json')
    )
    raise FileNotFoundError(
        f'no tool config named {name!r} in {TOOL_CONFIG_DIR}. '
        f'Known tools: {", ".join(available)}'
    )


def load_tool_config(name):
    """resolve_tool_config_path() then parse. Returns (config dict, path)."""
    path = resolve_tool_config_path(name)
    return load_json_with_comments(path), path


def coupling_flat(config):
    """
    The instrument's ActuatorToJointPosition matrix, flattened row-major.

    This is the exact payload tool_loader_node puts on /h4hr/update_coupling,
    and dynamixel_controller_node.update_coupling_callback picks elements
    0, 5, 9, 10, 11, 14, 15 out of it. Kept as the flat 16 rather than the
    seven the driver uses so that BOTH paths hand the driver the same shape and
    the index arithmetic lives in exactly one function -- the driver's.
    """
    matrix = config['coupling']['ActuatorToJointPosition']
    return [float(v) for row in matrix for v in row]


def limits_list(config):
    """
    The instrument's joint envelope in DEGREES, in the order
    dynamixel_controller_node.update_limits_callback reads:

        [roll_min, roll_max, pitch_min, pitch_max,
         yaw_min, yaw_max, grip_min, grip_max]

    The three DH joints come from the config; the grip OPENING is
    GRIP_OPEN_DEG for every instrument and only the closing limit is per-tool.
    See GRIP_OPEN_DEG for why.
    """
    joints = config['DH']['joints']
    jaw = config.get('jaw', {})

    limits = []
    for joint in joints:
        limits.extend([
            math.degrees(joint['qmin']),
            math.degrees(joint['qmax']),
        ])

    limits.extend([
        GRIP_OPEN_DEG,
        math.degrees(jaw.get('qmax', DEFAULT_JAW_QMAX_RAD)),
    ])
    return limits


def default_limits_list():
    """The bare-wrist fallback envelope, in the same order as limits_list()."""
    return [
        -DEFAULT_MAX_ROLL_DEG, DEFAULT_MAX_ROLL_DEG,
        -DEFAULT_MAX_PITCH_DEG, DEFAULT_MAX_PITCH_DEG,
        -DEFAULT_MAX_YAW_DEG, DEFAULT_MAX_YAW_DEG,
        DEFAULT_MIN_GRIP_DEG, DEFAULT_MAX_GRIP_DEG,
    ]
