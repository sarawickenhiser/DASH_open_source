"""Serial-arm kinematics read directly from the vendored Unitree MJCF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .geometry import (
    as_vector,
    homogeneous,
    normalize,
    quaternion_wxyz_to_rotation,
    skew,
    transform_point,
)


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]

# Rebuilt by np.eye on every call otherwise, and it is the same matrix each
# time. Never written to; the Rodrigues sum below always allocates its result.
_IDENTITY_3 = np.eye(3)


def _column_cross(left: Matrix, right: Matrix) -> Matrix:
    """Column-wise cross product of two ``(3, n)`` arrays."""
    return np.array(
        [
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        ]
    )


@dataclass(frozen=True)
class ToolKinematics:
    """Tool geometry and geometric Jacobians, all in the G1 base frame."""

    shaft_origin: Vector
    shaft_axis: Vector
    tip_position: Vector
    rotation: Matrix
    origin_jacobian: Matrix
    tip_jacobian: Matrix
    angular_jacobian: Matrix


def _attribute_vector(element: ET.Element, name: str, default: str) -> Vector:
    return np.fromstring(element.get(name, default), sep=" ", dtype=float)


class MjcfArmKinematics:
    """Forward kinematics for one active chain within a MuJoCo XML model.

    The supplied ``base_body`` is the coordinate frame origin. Floating-base
    placement in the world and all non-chain joints are deliberately excluded.
    Upstream chain joints can be fixed through ``locked_joints``.
    """

    def __init__(
        self,
        model_path: str | Path,
        *,
        base_body: str,
        end_body: str,
        active_joints: list[str] | tuple[str, ...],
        locked_joints: dict[str, float] | None = None,
        shaft_origin_local: ArrayLike = (0.0, 0.0, 0.0),
        shaft_axis_local: ArrayLike = (0.0, 0.0, -1.0),
        tip_local: ArrayLike = (0.0, 0.0, -0.535),
        body_transform_overrides: dict[str, dict[str, ArrayLike]] | None = None,
        joint_limit_overrides: dict[str, ArrayLike] | None = None,
    ) -> None:
        self.model_path = Path(model_path).resolve()
        root = ET.parse(self.model_path).getroot()
        bodies = [body for body in root.iter("body") if body.get("name")]
        self._bodies = {body.get("name"): body for body in bodies}
        if base_body not in self._bodies or end_body not in self._bodies:
            raise ValueError(f"unknown base/end body: {base_body!r}, {end_body!r}")

        parents = {child: parent for parent in root.iter() for child in parent}
        chain: list[ET.Element] = []
        cursor = self._bodies[end_body]
        base = self._bodies[base_body]
        while True:
            chain.append(cursor)
            if cursor is base:
                break
            cursor = parents.get(cursor)
            while cursor is not None and cursor.tag != "body":
                cursor = parents.get(cursor)
            if cursor is None:
                raise ValueError(f"{base_body!r} is not an ancestor of {end_body!r}")
        self._chain = list(reversed(chain))

        self.active_joints = tuple(active_joints)
        if len(set(self.active_joints)) != len(self.active_joints):
            raise ValueError("active joint names must be unique")
        self._joint_index = {
            name: index for index, name in enumerate(self.active_joints)
        }
        self.locked_joints = dict(locked_joints or {})
        self.body_transform_overrides = dict(body_transform_overrides or {})
        self.joint_limit_overrides = dict(joint_limit_overrides or {})
        self.shaft_origin_local = as_vector(shaft_origin_local)
        self.shaft_axis_local = normalize(shaft_axis_local)
        self.tip_local = as_vector(tip_local)

        chain_joints = {
            joint.get("name"): joint
            for body in self._chain
            for joint in body.findall("joint")
            if joint.get("name")
        }
        missing = set(self.active_joints) - set(chain_joints)
        if missing:
            raise ValueError(f"active joints not on chain: {sorted(missing)}")
        unknown_bodies = set(self.body_transform_overrides) - set(self._bodies)
        if unknown_bodies:
            raise ValueError(
                f"body transform overrides reference unknown bodies: {sorted(unknown_bodies)}"
            )
        unknown_limits = set(self.joint_limit_overrides) - set(chain_joints)
        if unknown_limits:
            raise ValueError(
                f"joint limit overrides reference unknown joints: {sorted(unknown_limits)}"
            )
        self.lower_limits = np.empty(self.dof)
        self.upper_limits = np.empty(self.dof)
        for name, index in self._joint_index.items():
            if name in self.joint_limit_overrides:
                limits = np.asarray(
                    self.joint_limit_overrides[name], dtype=float
                ).reshape(-1)
                if limits.shape != (2,) or not np.all(np.isfinite(limits)):
                    raise ValueError(
                        f"joint limit override for {name!r} must contain two finite values"
                    )
            else:
                limits = _attribute_vector(chain_joints[name], "range", "-inf inf")
            if limits[0] >= limits[1]:
                raise ValueError(f"joint limits for {name!r} are not increasing")
            self.lower_limits[index], self.upper_limits[index] = limits

        # The chain's geometry is fixed: body offsets, joint origins and joint
        # axes are XML attributes, and the overrides are constants too. Walking
        # the MJCF and re-validating all of it on every FK call put the string
        # parsing and the finiteness checks inside a 250 Hz loop. Resolve the
        # whole layout once here; a control cycle then only rotates about the
        # axes.
        #
        # Everything a cycle cannot avoid recomputing is reduced to the least
        # arithmetic that still produces the same numbers:
        #
        # * The chain is carried as a 3x3 rotation and a translation rather
        #   than a 4x4, so no product spends a quarter of its flops on the
        #   constant [0 0 0 1] row.
        # * A hinge contributes Trans(o) @ Rot(axis, angle) @ Trans(-o). Those
        #   three products collapse analytically to one rotation and one
        #   offset, so a joint costs one 3x3 product instead of three 4x4 ones.
        # * The axis is a constant of the model, so the Rodrigues skew terms
        #   K and K@K are constants too. Building them per cycle meant
        #   re-deriving a fixed matrix 250 times a second; they are stored
        #   here and the cycle only weighs them by sin/cos of the angle.
        self._layout: list[tuple[Any, ...]] = []
        for body_index, body in enumerate(self._chain):
            offset = np.eye(4) if body_index == 0 else self._body_transform(body)
            for joint in body.findall("joint"):
                if joint.get("type", "hinge") in {"free", "ball"}:
                    continue
                name = joint.get("name", "")
                local_origin = _attribute_vector(joint, "pos", "0 0 0")
                local_axis = normalize(_attribute_vector(joint, "axis", "0 0 1"))
                column = self._joint_index.get(name, -1)
                locked_angle = (
                    0.0 if column >= 0 else float(self.locked_joints.get(name, 0.0))
                )
                cross = skew(local_axis)
                self._layout.append(
                    (
                        None if offset is None else offset[:3, :3].copy(),
                        None if offset is None else offset[:3, 3].copy(),
                        column,
                        local_origin,
                        local_axis,
                        locked_angle,
                        cross,
                        cross @ cross,
                    )
                )
                # Only the first joint of a body carries that body's offset;
                # the rest are already in the body's own frame.
                offset = None
            if offset is not None and body_index > 0:
                # A body with no joints of its own still displaces the chain.
                self._layout.append(
                    (
                        offset[:3, :3].copy(),
                        offset[:3, 3].copy(),
                        -2,
                        None,
                        None,
                        0.0,
                        None,
                        None,
                    )
                )

        # One-deep memo on the joint vector. A single control cycle evaluates
        # this chain several times at the SAME q -- the shaft pose, the
        # keep-out barrier's Jacobians and the keep-out's own shaft pose are
        # three separate calls -- and each evaluation walks the MJCF in Python
        # at about 1.7 ms. Only the most recent q is kept, which is all the
        # duplication within a cycle needs; the next cycle's q simply misses.
        # Everything handed out here is treated as read-only by every caller.
        self._transform_memo: tuple[Vector | None, Any] = (None, None)
        self._forward_memo: tuple[Vector | None, Any] = (None, None)

    @property
    def dof(self) -> int:
        return len(self.active_joints)

    def _body_transform(self, body: ET.Element) -> Matrix:
        override = self.body_transform_overrides.get(body.get("name", ""), {})
        translation = np.asarray(
            override.get("position_m", _attribute_vector(body, "pos", "0 0 0")),
            dtype=float,
        ).reshape(-1)
        quaternion = np.asarray(
            override.get(
                "quaternion_wxyz",
                _attribute_vector(body, "quat", "1 0 0 0"),
            ),
            dtype=float,
        ).reshape(-1)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError(
                f"body position override for {body.get('name')!r} must be finite xyz"
            )
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError(
                f"body quaternion override for {body.get('name')!r} must be finite wxyz"
            )
        return homogeneous(quaternion_wxyz_to_rotation(quaternion), translation)

    def _tool_transform_and_joints(self, q: ArrayLike) -> tuple[Matrix, Matrix, Matrix]:
        positions = np.asarray(q, dtype=float).reshape(-1)
        if positions.shape != (self.dof,):
            raise ValueError(f"expected {self.dof} joints, got shape {positions.shape}")
        memo_q, memo_value = self._transform_memo
        if memo_q is not None and np.array_equal(memo_q, positions):
            return memo_value

        rotation = np.eye(3)
        translation = np.zeros(3)
        identity = _IDENTITY_3
        joint_origins = np.zeros((3, self.dof))
        joint_axes = np.zeros((3, self.dof))
        for (
            offset_rotation,
            offset_translation,
            column,
            local_origin,
            local_axis,
            locked,
            cross,
            cross_squared,
        ) in self._layout:
            if offset_rotation is not None:
                translation = rotation @ offset_translation + translation
                rotation = rotation @ offset_rotation
            if column == -2:
                continue
            if column >= 0:
                # The axis stays unit under an orthonormal rotation, so the
                # normalize the 4x4 form carried here is redundant.
                joint_origins[:, column] = rotation @ local_origin + translation
                joint_axes[:, column] = rotation @ local_axis
                angle = float(positions[column])
            else:
                angle = locked
            # Trans(o) @ Rot(axis, angle) @ Trans(-o), in closed form.
            joint_rotation = (
                identity
                + np.sin(angle) * cross
                + (1.0 - np.cos(angle)) * cross_squared
            )
            translation = (
                rotation @ (local_origin - joint_rotation @ local_origin) + translation
            )
            rotation = rotation @ joint_rotation
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        self._transform_memo = (
            positions.copy(),
            (transform, joint_origins, joint_axes),
        )
        return transform, joint_origins, joint_axes

    def forward(self, q: ArrayLike) -> ToolKinematics:
        positions = np.asarray(q, dtype=float).reshape(-1)
        memo_q, memo_value = self._forward_memo
        if memo_q is not None and np.array_equal(memo_q, positions):
            return memo_value
        transform, joint_origins, angular_jacobian = self._tool_transform_and_joints(
            positions
        )
        shaft_origin = transform_point(transform, self.shaft_origin_local)
        tip_position = transform_point(transform, self.tip_local)
        shaft_axis = normalize(transform[:3, :3] @ self.shaft_axis_local)

        # One batched cross product per Jacobian rather than one per column.
        # np.cross on a pair of 3-vectors spends most of its time in the
        # moveaxis/axis-normalisation preamble, so calling it 2*dof times a
        # cycle cost more than the arithmetic; _column_cross does the same
        # products over whole rows.
        origin_jacobian = _column_cross(
            angular_jacobian, shaft_origin[:, None] - joint_origins
        )
        tip_jacobian = _column_cross(
            angular_jacobian, tip_position[:, None] - joint_origins
        )
        result = ToolKinematics(
            shaft_origin=shaft_origin,
            shaft_axis=shaft_axis,
            tip_position=tip_position,
            rotation=transform[:3, :3].copy(),
            origin_jacobian=origin_jacobian,
            tip_jacobian=tip_jacobian,
            angular_jacobian=angular_jacobian,
        )
        self._forward_memo = (positions.copy(), result)
        return result
