"""Small, dependency-light rigid-body geometry helpers."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector = NDArray[np.float64]
Matrix = NDArray[np.float64]


def as_vector(value: ArrayLike, size: int = 3) -> Vector:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.shape != (size,):
        raise ValueError(f"expected a {size}-vector, got shape {vector.shape}")
    return vector


def normalize(value: ArrayLike, *, epsilon: float = 1e-12) -> Vector:
    vector = as_vector(value)
    norm = float(np.linalg.norm(vector))
    if norm < epsilon:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / norm


def skew(value: ArrayLike) -> Matrix:
    x, y, z = as_vector(value)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def axis_angle_rotation(axis: ArrayLike, angle: float) -> Matrix:
    unit_axis = normalize(axis)
    cross = skew(unit_axis)
    return (
        np.eye(3)
        + np.sin(angle) * cross
        + (1.0 - np.cos(angle)) * (cross @ cross)
    )


def quaternion_wxyz_to_rotation(quaternion: ArrayLike) -> Matrix:
    q = np.asarray(quaternion, dtype=float).reshape(-1)
    if q.shape != (4,):
        raise ValueError(f"expected a quaternion, got shape {q.shape}")
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_to_quaternion_wxyz(rotation: ArrayLike) -> Vector:
    """Return a normalized MuJoCo-order quaternion from a rotation matrix."""
    matrix = np.asarray(rotation, dtype=float).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        q = np.array(
            [0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale,
             (matrix[0, 2] - matrix[2, 0]) / scale,
             (matrix[1, 0] - matrix[0, 1]) / scale]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            q = np.array([(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                          (matrix[0, 1] + matrix[1, 0]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale])
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            q = np.array([(matrix[0, 2] - matrix[2, 0]) / scale,
                          (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale])
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            q = np.array([(matrix[1, 0] - matrix[0, 1]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale])
    return q / np.linalg.norm(q)


def rotation_aligning_z(direction: ArrayLike) -> Matrix:
    """Construct a stable frame whose local +Z follows ``direction``."""
    z_axis = normalize(direction)
    seed = np.array([0.0, 1.0, 0.0])
    if abs(float(seed @ z_axis)) > 0.95:
        seed = np.array([1.0, 0.0, 0.0])
    x_axis = normalize(np.cross(seed, z_axis))
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def rotation_error(target: ArrayLike, current: ArrayLike) -> Vector:
    """Return the base-frame rotation vector taking ``current`` to ``target``."""
    target_matrix = np.asarray(target, dtype=float).reshape(3, 3)
    current_matrix = np.asarray(current, dtype=float).reshape(3, 3)
    error = target_matrix @ current_matrix.T
    vector = 0.5 * np.array(
        [
            error[2, 1] - error[1, 2],
            error[0, 2] - error[2, 0],
            error[1, 0] - error[0, 1],
        ]
    )
    sine = float(np.linalg.norm(vector))
    cosine = float(np.clip((np.trace(error) - 1.0) / 2.0, -1.0, 1.0))
    if sine > 1e-9:
        return np.arctan2(sine, cosine) * vector / sine
    if cosine > 0.0:
        return vector

    # The skew part vanishes at pi, so recover an axis from the diagonal.
    axis = np.sqrt(np.maximum((np.diag(error) + 1.0) / 2.0, 0.0))
    largest = int(np.argmax(axis))
    if axis[largest] < 1e-9:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        if largest == 0:
            axis[1] = np.copysign(axis[1], error[0, 1] + error[1, 0])
            axis[2] = np.copysign(axis[2], error[0, 2] + error[2, 0])
        elif largest == 1:
            axis[0] = np.copysign(axis[0], error[0, 1] + error[1, 0])
            axis[2] = np.copysign(axis[2], error[1, 2] + error[2, 1])
        else:
            axis[0] = np.copysign(axis[0], error[0, 2] + error[2, 0])
            axis[1] = np.copysign(axis[1], error[1, 2] + error[2, 1])
        axis /= np.linalg.norm(axis)
    return np.pi * axis


def homogeneous(rotation: ArrayLike | None = None, translation: ArrayLike | None = None) -> Matrix:
    transform = np.eye(4)
    if rotation is not None:
        transform[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    if translation is not None:
        transform[:3, 3] = as_vector(translation)
    return transform


def transform_point(transform: ArrayLike, point: ArrayLike) -> Vector:
    matrix = np.asarray(transform, dtype=float).reshape(4, 4)
    return matrix[:3, :3] @ as_vector(point) + matrix[:3, 3]
