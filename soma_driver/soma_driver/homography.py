"""Pixel-to-deck homography and lane-to-IK bridge for SOMA (v0.3).

PURE module (no ROS, no OpenCV): runs under pytest on any machine.

Fits a normalized 2D projective transform (DLT) from 4+ image points
(u, v) in pixels to deck coordinates (x_mm, y_mm) in the robot frame:
  - x_mm: forward along the lanes (away from the column)
  - y_mm: left (+81 mm = left lane, -81 mm = right lane)

Safety invariants:
  - Rejects fewer than 4 points, non-finite coordinates, collinear/degenerate
    point sets, and projective horizon singularities (w <= 0).
  - Refuses points outside either arm's vertical plane (dead zone between
    the two lanes or outboard beyond max_lateral_error_mm): planar arms
    cannot correct lateral error.
  - Delegates pose solving to kinematics.solve_pose(), which enforces
    SERVO_MAP soft limits, plate clearance, elbow curl floor, and base
    torque limits.
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .kinematics import (
    ELBOW_CURL_FLOOR_RAD,
    MAX_BASE_TORQUE_KGCM,
    MIN_CLEARANCE_Z_M,
    SIDES,
    lane_y,
    load_dimensions,
    solve_pose,
)

DEFAULT_MAX_LATERAL_ERROR_MM = 25.0


@dataclass(frozen=True)
class HomographyCalibration:
    """Serialized pixel-to-deck homography with provenance and error metrics."""

    matrix: tuple[tuple[float, float, float],
                  tuple[float, float, float],
                  tuple[float, float, float]]
    image_size: tuple[int, int]
    deck_z_mm: float
    rms_error_mm: float
    max_error_mm: float
    date: str = ''
    source: str = ''

    def _array(self) -> np.ndarray:
        return np.asarray(self.matrix, dtype=np.float64)

    def pixel_to_deck(self, u: float, v: float) -> tuple[float, float]:
        """Project image pixel (u, v) onto deck coordinates (x_mm, y_mm)."""
        if not (math.isfinite(u) and math.isfinite(v)):
            raise ValueError(f'pixel coordinates must be finite: ({u}, {v})')
        vec = self._array() @ np.array([u, v, 1.0], dtype=np.float64)
        w = float(vec[2])
        if not math.isfinite(w) or w <= 1e-9:
            raise ValueError(f'pixel ({u}, {v}) projects onto or behind the horizon (w={w:.3g})')
        x_mm, y_mm = float(vec[0] / w), float(vec[1] / w)
        if not (math.isfinite(x_mm) and math.isfinite(y_mm)):
            raise ValueError(f'non-finite deck projection for pixel ({u}, {v})')
        return x_mm, y_mm

    def to_dict(self) -> dict:
        return {
            'matrix': [[float(v) for v in row] for row in self.matrix],
            'image_size': [int(self.image_size[0]), int(self.image_size[1])],
            'deck_z_mm': float(self.deck_z_mm),
            'rms_error_mm': float(self.rms_error_mm),
            'max_error_mm': float(self.max_error_mm),
            'date': self.date,
            'source': self.source,
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'HomographyCalibration':
        mat = data['matrix']
        if len(mat) != 3 or any(len(r) != 3 for r in mat):
            raise ValueError('homography matrix must be 3x3')
        matrix = tuple(tuple(float(v) for v in r) for r in mat)
        if any(not math.isfinite(v) for r in matrix for v in r):
            raise ValueError('homography matrix contains non-finite entries')
        w, h = int(data['image_size'][0]), int(data['image_size'][1])
        if w <= 0 or h <= 0:
            raise ValueError(f'image_size must be positive: {(w, h)}')
        return cls(
            matrix=matrix,
            image_size=(w, h),
            deck_z_mm=float(data['deck_z_mm']),
            rms_error_mm=float(data['rms_error_mm']),
            max_error_mm=float(data['max_error_mm']),
            date=str(data.get('date', '')),
            source=str(data.get('source', '')),
        )

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2) + '\n')

    @classmethod
    def load(cls, path: Path) -> 'HomographyCalibration':
        return cls.from_dict(json.loads(Path(path).read_text()))


def _normalize_points(pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hartley isotropic normalization for numerically stable DLT."""
    mean = np.mean(pts, axis=0)
    shifted = pts - mean
    dists = np.hypot(shifted[:, 0], shifted[:, 1])
    mean_dist = float(np.mean(dists))
    if mean_dist <= 1e-9:
        raise ValueError('points are coincident or degenerate')
    scale = math.sqrt(2.0) / mean_dist
    T = np.array([
        [scale, 0.0, -scale * mean[0]],
        [0.0, scale, -scale * mean[1]],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    norm = (T @ np.column_stack([pts, np.ones(len(pts))]).T).T
    return norm[:, :2], T


def _has_collinear_triple(pts: np.ndarray, tol: float = 1e-5) -> bool:
    """True when any 3 points in a 4-point minimal set are nearly collinear."""
    if len(pts) > 4:
        return False
    p = pts
    for i in range(4):
        a, b, c = [p[j] for j in range(4) if j != i]
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        if area2 <= tol:
            return True
    return False


def fit_homography(image_pts: list[tuple[float, float]],
                   deck_pts_mm: list[tuple[float, float]],
                   image_size: tuple[int, int],
                   deck_z_mm: float = 40.0,
                   date: str = '',
                   source: str = '') -> HomographyCalibration:
    """Fit a 3x3 pixel-to-deck homography using normalized DLT."""
    if len(image_pts) != len(deck_pts_mm):
        raise ValueError('image_pts and deck_pts_mm must have the same length')
    if len(image_pts) < 4:
        raise ValueError(f'need at least 4 point correspondences, got {len(image_pts)}')
    if image_size[0] <= 0 or image_size[1] <= 0:
        raise ValueError(f'image_size must be positive: {image_size}')
    if not math.isfinite(deck_z_mm):
        raise ValueError('deck_z_mm must be finite')

    src = np.asarray(image_pts, dtype=np.float64)
    dst = np.asarray(deck_pts_mm, dtype=np.float64)
    if src.shape[1] != 2 or dst.shape[1] != 2:
        raise ValueError('points must be 2D (x, y) pairs')
    if not (np.all(np.isfinite(src)) and np.all(np.isfinite(dst))):
        raise ValueError('correspondence points must be finite')

    src_n, T_src = _normalize_points(src)
    dst_n, T_dst = _normalize_points(dst)
    if _has_collinear_triple(src_n) or _has_collinear_triple(dst_n):
        raise ValueError('degenerate configuration: 3 or more calibration points are collinear')

    rows = []
    for (u, v), (x, y) in zip(src_n, dst_n):
        rows.append([-u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x])
        rows.append([0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y])
    A = np.asarray(rows, dtype=np.float64)
    _, s, vt = np.linalg.svd(A)
    if len(s) < 8 or s[7] <= 1e-12 or (len(s) == 9 and (s[-1] / s[-2]) > 0.2):
        raise ValueError('ill-conditioned homography correspondences')

    H_n = vt[-1].reshape(3, 3)
    H = np.linalg.inv(T_dst) @ H_n @ T_src
    if abs(float(np.linalg.det(H))) <= 1e-12 or abs(H[2, 2]) <= 1e-12:
        raise ValueError('singular homography matrix')
    H = H / H[2, 2]

    # Ensure positive w across all calibration points.
    ws = (H @ np.column_stack([src, np.ones(len(src))]).T)[2, :]
    if np.all(ws < 0):
        H = -H
    elif not np.all(ws > 0):
        raise ValueError('calibration points span across the projective horizon')

    cal = HomographyCalibration(
        matrix=tuple(tuple(float(v) for v in row) for row in H),
        image_size=(int(image_size[0]), int(image_size[1])),
        deck_z_mm=float(deck_z_mm),
        rms_error_mm=0.0,
        max_error_mm=0.0,
        date=date,
        source=source,
    )

    errors = []
    for (u, v), (tx, ty) in zip(image_pts, deck_pts_mm):
        px, py = cal.pixel_to_deck(u, v)
        errors.append(math.hypot(px - tx, py - ty))
    rms = math.sqrt(sum(e * e for e in errors) / len(errors))
    max_err = max(errors)

    return HomographyCalibration(
        matrix=cal.matrix,
        image_size=cal.image_size,
        deck_z_mm=cal.deck_z_mm,
        rms_error_mm=rms,
        max_error_mm=max_err,
        date=cal.date,
        source=cal.source,
    )


def er2_point_to_pixel(point_yx: tuple[float, float] | list[float],
                       width: int, height: int) -> tuple[float, float]:
    """Convert Gemini ER 2 normalized [y, x] in 0..1000 to (u, v) pixels."""
    if len(point_yx) != 2:
        raise ValueError(f'ER 2 point must be [y, x]: {point_yx!r}')
    y_norm, x_norm = float(point_yx[0]), float(point_yx[1])
    if not (math.isfinite(y_norm) and math.isfinite(x_norm)):
        raise ValueError(f'ER 2 point must be finite: {point_yx!r}')
    if not (0.0 <= y_norm <= 1000.0 and 0.0 <= x_norm <= 1000.0):
        raise ValueError(f'ER 2 point outside 0..1000: {point_yx!r}')
    if width <= 0 or height <= 0:
        raise ValueError(f'image dimensions must be positive: {(width, height)}')
    return (x_norm / 1000.0 * width, y_norm / 1000.0 * height)


def assign_lane(y_mm: float,
                dims: dict[str, float] | None = None,
                max_lateral_error_mm: float = DEFAULT_MAX_LATERAL_ERROR_MM
                ) -> tuple[str, float]:
    """Assign a deck y_mm coordinate to 'left' or 'right' lane, or refuse.

    Returns (side, lateral_offset_mm) where lateral_offset_mm = y_mm - lane_y_mm.
    """
    if not math.isfinite(y_mm):
        raise ValueError(f'y_mm must be finite: {y_mm}')
    if max_lateral_error_mm <= 0 or not math.isfinite(max_lateral_error_mm):
        raise ValueError('max_lateral_error_mm must be > 0')
    if dims is None:
        dims = load_dimensions()
    best_side = None
    best_offset = None
    for side in SIDES:
        ly_mm = lane_y(side, dims) * 1000.0
        offset = y_mm - ly_mm
        if best_offset is None or abs(offset) < abs(best_offset):
            best_side = side
            best_offset = offset
    assert best_side is not None and best_offset is not None
    if abs(best_offset) > max_lateral_error_mm:
        raise ValueError(
            f'point y={y_mm:.1f} mm is {abs(best_offset):.1f} mm from nearest lane '
            f'({best_side} at {lane_y(best_side, dims) * 1000.0:+.1f} mm), '
            f'exceeding {max_lateral_error_mm:.1f} mm lateral limit')
    return best_side, best_offset


def point_to_pose(cal: HomographyCalibration,
                  u: float, v: float,
                  dims: dict[str, float] | None = None,
                  grasp_z_mm: float | None = None,
                  phi_tool: float = 0.0,
                  max_lateral_error_mm: float = DEFAULT_MAX_LATERAL_ERROR_MM,
                  min_z_m: float = MIN_CLEARANCE_Z_M,
                  max_torque_kgcm: float = MAX_BASE_TORQUE_KGCM,
                  min_elbow_rad: float = ELBOW_CURL_FLOOR_RAD
                  ) -> tuple[str, dict[str, float]]:
    """Map an image pixel (u, v) to (side, 6-joint target dict) or raise/None."""
    if dims is None:
        dims = load_dimensions()
    x_mm, y_mm = cal.pixel_to_deck(u, v)
    side, _ = assign_lane(y_mm, dims=dims, max_lateral_error_mm=max_lateral_error_mm)
    z_mm = cal.deck_z_mm if grasp_z_mm is None else grasp_z_mm
    pose = solve_pose(
        (x_mm / 1000.0, z_mm / 1000.0),
        side,
        dims=dims,
        phi_tool=phi_tool,
        min_z_m=min_z_m,
        max_torque_kgcm=max_torque_kgcm,
        min_elbow_rad=min_elbow_rad,
    )
    if pose is None:
        raise ValueError(
            f'target ({x_mm:.1f}, {z_mm:.1f}) mm on {side} lane is unreachable or unsafe')
    return side, pose


def er2_point_to_pose(cal: HomographyCalibration,
                      point_yx: tuple[float, float] | list[float],
                      dims: dict[str, float] | None = None,
                      grasp_z_mm: float | None = None,
                      phi_tool: float = 0.0,
                      max_lateral_error_mm: float = DEFAULT_MAX_LATERAL_ERROR_MM
                      ) -> tuple[str, dict[str, float]]:
    """Map a Gemini ER 2 normalized [y, x] point directly to a safe arm pose."""
    u, v = er2_point_to_pixel(point_yx, *cal.image_size)
    return point_to_pose(
        cal, u, v, dims=dims, grasp_z_mm=grasp_z_mm,
        phi_tool=phi_tool, max_lateral_error_mm=max_lateral_error_mm,
    )
