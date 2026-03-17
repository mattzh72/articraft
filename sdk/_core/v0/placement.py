from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, Union

from .assets import AssetContext, resolve_asset_root
from .errors import ValidationError
from .geometry_qc import AABB, link_local_aabbs
from .types import Origin, Part

Vec3 = Tuple[float, float, float]


def _as_pair(values: object, *, name: str) -> tuple[float, float]:
    try:
        if len(values) != 2:  # type: ignore[arg-type]
            raise ValidationError(f"{name} must have 2 elements")
        return (float(values[0]), float(values[1]))  # type: ignore[index]
    except TypeError as exc:
        raise ValidationError(f"{name} must have 2 elements") from exc


def _aabb_union(a: list[AABB]) -> AABB:
    if not a:
        raise ValidationError("Cannot union empty AABB list")
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for (mn, mx) in a:
        for i in range(3):
            mins[i] = min(mins[i], mn[i])
            maxs[i] = max(maxs[i], mx[i])
    return (mins[0], mins[1], mins[2]), (maxs[0], maxs[1], maxs[2])


def _aabb_center(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)


def link_local_aabb(
    link: Part,
    *,
    asset_root: Optional[Union[Path, str, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Optional[AABB]:
    root = resolve_asset_root(asset_root, link)
    aabbs = link_local_aabbs(
        link,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )
    if not aabbs:
        return None
    return _aabb_union(aabbs)


def part_local_aabb(
    part: Part,
    *,
    asset_root: Optional[Union[Path, str, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Optional[AABB]:
    return link_local_aabb(
        part,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
    )


def align_centers_xy(child_aabb: AABB, parent_aabb: AABB) -> Origin:
    cx, cy, _ = _aabb_center(child_aabb)
    px, py, _ = _aabb_center(parent_aabb)
    return Origin(xyz=(px - cx, py - cy, 0.0))


def place_on_top(
    child_aabb: AABB,
    parent_aabb: AABB,
    *,
    clearance: float = 0.0,
    align_xy: bool = True,
) -> Origin:
    (cmin, _cmax) = child_aabb
    (_pmin, pmax) = parent_aabb
    dz = float(pmax[2]) + float(clearance) - float(cmin[2])
    dx = 0.0
    dy = 0.0
    if align_xy:
        cx, cy, _ = _aabb_center(child_aabb)
        px, py, _ = _aabb_center(parent_aabb)
        dx = px - cx
        dy = py - cy
    return Origin(xyz=(dx, dy, dz))


def place_in_front_of(
    child_aabb: AABB,
    parent_aabb: AABB,
    *,
    gap: float = 0.0,
    align_yz: bool = True,
) -> Origin:
    (cmin, _cmax) = child_aabb
    (_pmin, pmax) = parent_aabb
    dx = float(pmax[0]) + float(gap) - float(cmin[0])
    dy = 0.0
    dz = 0.0
    if align_yz:
        cx, cy, cz = _aabb_center(child_aabb)
        px, py, pz = _aabb_center(parent_aabb)
        dy = py - cy
        dz = pz - cz
    return Origin(xyz=(dx, dy, dz))


# Mapping from face string to (axis_index, sign, rpy).
# The rpy rotates the child's local +Z axis to point along the face outward normal.
_FACE_TABLE: dict[str, tuple[int, float, Vec3]] = {
    "+x": (0, +1.0, (0.0, math.pi / 2.0, 0.0)),
    "-x": (0, -1.0, (0.0, -math.pi / 2.0, 0.0)),
    "+y": (1, +1.0, (-math.pi / 2.0, 0.0, 0.0)),
    "-y": (1, -1.0, (math.pi / 2.0, 0.0, 0.0)),
    "+z": (2, +1.0, (0.0, 0.0, 0.0)),
    "-z": (2, -1.0, (math.pi, 0.0, 0.0)),
}

# For each face, which two parent-frame axes span the face.
# Order: (first_tangent_index, second_tangent_index).
_FACE_TANGENT_AXES: dict[str, tuple[int, int]] = {
    "+x": (1, 2),
    "-x": (1, 2),
    "+y": (0, 2),
    "-y": (0, 2),
    "+z": (0, 1),
    "-z": (0, 1),
}


def place_on_face(
    parent_link: Part,
    face: str,
    *,
    face_pos: tuple[float, float] = (0.0, 0.0),
    proud: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Origin:
    """
    Compute an ``Origin`` that places a child part on a face of the parent.

    The returned origin includes both the translation (``xyz``) and rotation
    (``rpy``) so that the child's local +Z axis points outward from the
    chosen face.  This eliminates the need to manually compute rpy rotations
    and per-axis offsets when mounting controls, buttons, or panels on the
    side of a body.

    Parameters
    ----------
    parent_link:
        The parent ``Part`` whose geometry defines the mounting surface.
    face:
        Which face of the parent's AABB to use.
        One of ``"+x"``, ``"-x"``, ``"+y"``, ``"-y"``, ``"+z"``, ``"-z"``.
    face_pos:
        Position on the face surface, given as ``(a, b)`` in the parent's
        coordinate frame along the two tangent axes of the face:

        * ``±x`` faces: ``face_pos = (y, z)``
        * ``±y`` faces: ``face_pos = (x, z)``
        * ``±z`` faces: ``face_pos = (x, y)``
    proud:
        Distance the child origin is pushed outward from the face surface.
        With ``proud=0`` the child origin sits exactly on the surface.
        Set this to half the child's thickness to make the child geometry
        sit flush, or slightly more to make it visually proud.
    asset_root:
        Root directory for resolving relative mesh filenames.
    prefer_collisions:
        Whether to use collision geometry (preferred) or visual geometry
        when computing the parent's AABB.

    Returns
    -------
    Origin
        An origin with the correct ``xyz`` and ``rpy`` for mounting
        on the specified face.

    Example
    -------
    Mount a speed dial on the right side (+x) of a mixer head::

        origin = place_on_face(
            head_link, "+x",
            face_pos=(0.13, 0.02),   # y=0.13, z=0.02 on the face
            proud=0.007,             # 7 mm outward
            asset_root=HERE,
        )
        model.articulation(
            "head_to_dial",
            parent="head",
            child="speed_dial",
            articulation_type="revolute",
            origin=origin,
            ...
        )
    """
    face_key = face.strip().lower()
    if face_key not in _FACE_TABLE:
        raise ValidationError(
            f"Invalid face {face!r}; expected one of: {', '.join(sorted(_FACE_TABLE))}"
        )

    axis_idx, sign, rpy = _FACE_TABLE[face_key]
    tang_a, tang_b = _FACE_TANGENT_AXES[face_key]
    face_pos_pair = _as_pair(face_pos, name="face_pos")

    root = resolve_asset_root(asset_root, parent_link)
    aabbs = link_local_aabbs(
        parent_link,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )
    if not aabbs:
        raise ValidationError(
            f"Parent link {parent_link.name!r} has no geometry to compute AABB"
        )
    aabb = _aabb_union(aabbs)
    (mn, mx) = aabb

    # Surface coordinate along the face normal axis.
    surface = float(mx[axis_idx]) if sign > 0 else float(mn[axis_idx])
    normal_coord = surface + sign * float(proud)

    # Build xyz in parent frame.
    xyz = [0.0, 0.0, 0.0]
    xyz[axis_idx] = normal_coord
    xyz[tang_a] = face_pos_pair[0]
    xyz[tang_b] = face_pos_pair[1]

    return Origin(xyz=(xyz[0], xyz[1], xyz[2]), rpy=rpy)


def place_on_face_uv(
    parent_link: Part,
    face: str,
    *,
    uv: tuple[float, float] = (0.5, 0.5),
    uv_margin: float | tuple[float, float] = 0.0,
    proud: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Origin:
    """
    Like :func:`place_on_face`, but address the face using normalized coordinates.

    ``uv`` is interpreted on the chosen face's tangential axes and mapped into
    the parent's AABB bounds:

    - u=0 is the min bound along the first tangent axis; u=1 is the max bound.
    - v=0 is the min bound along the second tangent axis; v=1 is the max bound.

    This is significantly less error-prone than hand-picking absolute ``face_pos``
    values when the parent dimensions are changing.

    Parameters
    ----------
    uv:
        Normalized face coordinates. Values outside [0,1] are clamped.
    uv_margin:
        Fractional margin applied to each axis before mapping, keeping placements
        away from sharp edges. Either a single float for both axes or ``(mu, mv)``.

        Example: ``uv_margin=0.08`` maps u/v into [0.08, 0.92] on each axis.
    """
    face_key = face.strip().lower()
    if face_key not in _FACE_TABLE:
        raise ValidationError(
            f"Invalid face {face!r}; expected one of: {', '.join(sorted(_FACE_TABLE))}"
        )

    axis_idx, sign, rpy = _FACE_TABLE[face_key]
    tang_a, tang_b = _FACE_TANGENT_AXES[face_key]

    if isinstance(uv_margin, tuple):
        mu, mv = _as_pair(uv_margin, name="uv_margin")
    else:
        mu = mv = float(uv_margin)
    mu = max(0.0, min(0.49, mu))
    mv = max(0.0, min(0.49, mv))

    u_raw, v_raw = _as_pair(uv, name="uv")
    u = max(0.0, min(1.0, u_raw))
    v = max(0.0, min(1.0, v_raw))
    u = mu + (1.0 - 2.0 * mu) * u
    v = mv + (1.0 - 2.0 * mv) * v

    root = resolve_asset_root(asset_root, parent_link)
    aabbs = link_local_aabbs(
        parent_link,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )
    if not aabbs:
        raise ValidationError(
            f"Parent link {parent_link.name!r} has no geometry to compute AABB"
        )
    (mn, mx) = _aabb_union(aabbs)

    # Surface coordinate along the face normal axis.
    surface = float(mx[axis_idx]) if sign > 0 else float(mn[axis_idx])
    normal_coord = surface + sign * float(proud)

    # Map uv into the face tangent bounds.
    ta0, ta1 = float(mn[tang_a]), float(mx[tang_a])
    tb0, tb1 = float(mn[tang_b]), float(mx[tang_b])
    face_pos = (ta0 + (ta1 - ta0) * u, tb0 + (tb1 - tb0) * v)

    xyz = [0.0, 0.0, 0.0]
    xyz[axis_idx] = normal_coord
    xyz[tang_a] = float(face_pos[0])
    xyz[tang_b] = float(face_pos[1])

    return Origin(xyz=(xyz[0], xyz[1], xyz[2]), rpy=rpy)


def proud_for_flush_mount(
    child_link: Part,
    *,
    axis: str = "z",
    clearance: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> float:
    """
    Compute a ``proud`` distance that makes a face-mounted child sit flush.

    When you mount a child using ``place_on_face(..., proud=0)``, the *child link
    origin* sits on the parent face. If the child's geometry is centered about
    its link origin, half the geometry will be embedded in the parent.

    This helper returns half the child's thickness along the selected local axis
    (plus optional clearance), which is typically the correct ``proud`` for a
    flush mount when using ``place_on_face``.

    Notes
    -----
    - For most knobs/buttons modeled "standing out" along their local +Z axis,
      use the default ``axis="z"``.
    - Add a small positive ``clearance`` (e.g. 0.001–0.003m) if you want the
      control to be visually proud instead of perfectly flush.
    """
    axis_key = axis.strip().lower()
    axis_idx = {"x": 0, "y": 1, "z": 2}.get(axis_key)
    if axis_idx is None:
        raise ValidationError(f"Invalid axis {axis!r}; expected 'x', 'y', or 'z'.")

    root = resolve_asset_root(asset_root, child_link)
    aabb = link_local_aabb(child_link, asset_root=root, prefer_collisions=prefer_collisions)
    if aabb is None:
        raise ValidationError(f"Child link {child_link.name!r} has no geometry to compute AABB")

    (mn, mx) = aabb
    thickness = float(mx[axis_idx]) - float(mn[axis_idx])
    return 0.5 * thickness + float(clearance)
