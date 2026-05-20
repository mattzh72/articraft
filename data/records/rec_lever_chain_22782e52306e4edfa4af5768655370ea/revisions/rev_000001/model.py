from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


STEEL = Material("brushed_steel", rgba=(0.63, 0.66, 0.68, 1.0))
DARK_STEEL = Material("dark_blued_steel", rgba=(0.10, 0.12, 0.13, 1.0))
PIN = Material("polished_pin", rgba=(0.84, 0.83, 0.78, 1.0))
TAB = Material("yellow_end_tab", rgba=(0.95, 0.70, 0.16, 1.0))


def _circle_profile(cx: float, cy: float, radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(
    p0: tuple[float, float],
    p1: tuple[float, float],
    radius: float,
    *,
    segments: int = 28,
) -> list[tuple[float, float]]:
    """Rounded link outline in the local X/Z sketch plane."""
    x0, z0 = p0
    x1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    ux, uz = dx / length, dz / length
    nx, nz = -uz, ux
    angle_n = math.atan2(nz, nx)

    points: list[tuple[float, float]] = []
    points.append((x0 + radius * nx, z0 + radius * nz))
    points.append((x1 + radius * nx, z1 + radius * nz))

    for i in range(1, segments + 1):
        angle = angle_n - math.pi * i / segments
        points.append((x1 + radius * math.cos(angle), z1 + radius * math.sin(angle)))

    points.append((x0 - radius * nx, z0 - radius * nz))
    for i in range(1, segments + 1):
        angle = angle_n - math.pi - math.pi * i / segments
        points.append((x0 + radius * math.cos(angle), z0 + radius * math.sin(angle)))

    return points


def _lever_mesh(
    name: str,
    *,
    length: float,
    rise: float,
    radius: float = 0.025,
    hole_radius: float = 0.012,
    thickness: float = 0.016,
    end_hole_radius: float | None = None,
):
    outer = _capsule_profile((0.0, 0.0), (length, rise), radius)
    if _signed_area(outer) < 0.0:
        outer = list(reversed(outer))
    holes = [list(reversed(_circle_profile(0.0, 0.0, hole_radius)))]
    if end_hole_radius is not None:
        holes.append(list(reversed(_circle_profile(length, rise, end_hole_radius))))
    else:
        holes.append(list(reversed(_circle_profile(length, rise, hole_radius))))
    geom = ExtrudeWithHolesGeometry(outer, holes, thickness, center=True)
    # ExtrudeWithHolesGeometry extrudes along local +Z.  Rotate so the flat plate
    # lies in the articulated X/Z plane and its thickness is along the pin axis Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _signed_area(profile: list[tuple[float, float]]) -> float:
    return 0.5 * sum(
        profile[i][0] * profile[(i + 1) % len(profile)][1]
        - profile[(i + 1) % len(profile)][0] * profile[i][1]
        for i in range(len(profile))
    )


def _add_y_cylinder(part, name: str, radius: float, length: float, xyz: tuple[float, float, float], material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_pin_stack(
    part,
    *,
    joint_xyz: tuple[float, float, float],
    parent_layer_y: float,
    visual_prefix: str,
    near_side: int,
) -> None:
    """Add retaining collars at a joint, fixed to the parent link."""
    x, _, z = joint_xyz
    # One collar touches the parent plate and visibly fixes the pin to that link.
    contact_y = parent_layer_y + near_side * (0.016 / 2.0 + 0.003)
    far_y = -near_side * 0.038
    _add_y_cylinder(part, f"{visual_prefix}_collar", 0.016, 0.006, (x, contact_y, z), PIN)
    _add_y_cylinder(part, f"{visual_prefix}_cap", 0.014, 0.006, (x, far_y, z), PIN)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_lever_chain")

    # A small bench lug with a clevis-like pair of cheeks supports the first pin.
    base = model.part("base_lug")
    base.visual(
        Box((0.18, 0.14, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
        material=DARK_STEEL,
        name="base_plate",
    )
    for side, y in (("front", -0.035), ("rear", 0.035)):
        base.visual(
            Box((0.052, 0.014, 0.150)),
            origin=Origin(xyz=(0.0, y, 0.105)),
            material=DARK_STEEL,
            name=f"{side}_cheek",
        )
        _add_y_cylinder(
            base,
            f"{side}_boss",
            0.030,
            0.014,
            (0.0, y, 0.150),
            DARK_STEEL,
        )
    base.visual(
        Cylinder(radius=0.008, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=PIN,
        name="base_pin_shaft",
    )
    _add_y_cylinder(base, "base_pin_head_0", 0.016, 0.006, (0.0, -0.056, 0.150), PIN)
    _add_y_cylinder(base, "base_pin_head_1", 0.016, 0.006, (0.0, 0.056, 0.150), PIN)

    # Three rigid offset links.  Their pin centers are not collinear at rest,
    # giving the extended chain a shallow arc instead of a dead-straight line.
    bar_0 = model.part("bar_0")
    bar_0.visual(
        _lever_mesh("bar_0_plate", length=0.220, rise=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=STEEL,
        name="plate",
    )
    bar_0.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.220, 0.0, 0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=PIN,
        name="joint_1_pin_shaft",
    )
    _add_pin_stack(
        bar_0,
        joint_xyz=(0.220, 0.0, 0.040),
        parent_layer_y=0.0,
        visual_prefix="joint_1_pin",
        near_side=-1,
    )

    bar_1 = model.part("bar_1")
    bar_1.visual(
        _lever_mesh("bar_1_plate", length=0.200, rise=0.020),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=STEEL,
        name="plate",
    )
    bar_1.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.200, 0.0, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=PIN,
        name="joint_2_pin_shaft",
    )
    _add_pin_stack(
        bar_1,
        joint_xyz=(0.200, 0.0, 0.020),
        parent_layer_y=0.024,
        visual_prefix="joint_2_pin",
        near_side=1,
    )

    bar_2 = model.part("bar_2")
    bar_2.visual(
        _lever_mesh("bar_2_plate", length=0.180, rise=-0.015, end_hole_radius=0.007),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=STEEL,
        name="plate",
    )
    # Small colored end tab beyond the last eye, overlapped locally into the
    # final link so it reads as one rigid handle/attachment end.
    bar_2.visual(
        Box((0.060, 0.014, 0.026)),
        origin=Origin(xyz=(0.215, -0.024, -0.018), rpy=(0.0, -0.08, 0.0)),
        material=TAB,
        name="end_tab",
    )

    model.articulation(
        "base_to_bar_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bar_0,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=5.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "bar_0_to_bar_1",
        ArticulationType.REVOLUTE,
        parent=bar_0,
        child=bar_1,
        origin=Origin(xyz=(0.220, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=5.0, lower=-2.55, upper=2.55),
    )
    model.articulation(
        "bar_1_to_bar_2",
        ArticulationType.REVOLUTE,
        parent=bar_1,
        child=bar_2,
        origin=Origin(xyz=(0.200, 0.0, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.55, upper=2.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_lug")
    bar_0 = object_model.get_part("bar_0")
    bar_1 = object_model.get_part("bar_1")
    bar_2 = object_model.get_part("bar_2")
    joints = [
        object_model.get_articulation("base_to_bar_0"),
        object_model.get_articulation("bar_0_to_bar_1"),
        object_model.get_articulation("bar_1_to_bar_2"),
    ]

    ctx.allow_overlap(
        bar_0,
        base,
        elem_a="plate",
        elem_b="base_pin_shaft",
        reason="The base pin shaft is intentionally captured through the first lever eye.",
    )
    ctx.allow_overlap(
        bar_0,
        bar_1,
        elem_a="joint_1_pin_shaft",
        elem_b="plate",
        reason="The first inter-link pin shaft intentionally passes through the second lever eye.",
    )
    ctx.allow_overlap(
        bar_1,
        bar_2,
        elem_a="joint_2_pin_shaft",
        elem_b="plate",
        reason="The second inter-link pin shaft intentionally passes through the third lever eye.",
    )

    ctx.check(
        "three parallel revolute pin joints",
        all(j.articulation_type == ArticulationType.REVOLUTE and tuple(j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=[(j.name, j.articulation_type, j.axis) for j in joints],
    )
    ctx.check(
        "joint limits allow compact folding",
        all(j.motion_limits.lower <= -2.3 and j.motion_limits.upper >= 2.3 for j in joints),
        details=[(j.name, j.motion_limits.lower, j.motion_limits.upper) for j in joints],
    )

    ctx.expect_overlap(bar_0, base, axes="xz", min_overlap=0.012, name="first bar is captured on base pin")
    ctx.expect_overlap(bar_1, bar_0, axes="xz", min_overlap=0.012, name="second bar shares first offset pin")
    ctx.expect_overlap(bar_2, bar_1, axes="xz", min_overlap=0.012, name="third bar shares second offset pin")
    ctx.expect_within(
        base,
        bar_0,
        axes="xz",
        inner_elem="base_pin_shaft",
        outer_elem="plate",
        margin=0.002,
        name="base pin is centered in the first lever eye",
    )
    ctx.expect_within(
        bar_0,
        bar_1,
        axes="xz",
        inner_elem="joint_1_pin_shaft",
        outer_elem="plate",
        margin=0.002,
        name="first link pin is centered in the second lever eye",
    )
    ctx.expect_within(
        bar_1,
        bar_2,
        axes="xz",
        inner_elem="joint_2_pin_shaft",
        outer_elem="plate",
        margin=0.002,
        name="second link pin is centered in the third lever eye",
    )

    tab_aabb = ctx.part_element_world_aabb(bar_2, elem="end_tab")
    ctx.check(
        "rest pose forms a shallow extended arc",
        tab_aabb is not None and tab_aabb[1][0] > 0.58 and 0.12 < tab_aabb[0][2] < 0.24,
        details=f"end tab aabb={tab_aabb}",
    )

    with ctx.pose({"base_to_bar_0": 1.75, "bar_0_to_bar_1": -2.10, "bar_1_to_bar_2": 1.85}):
        folded_aabb = ctx.part_element_world_aabb(bar_2, elem="end_tab")
    ctx.check(
        "sample folded pose brings end tab back near the lug",
        folded_aabb is not None and folded_aabb[0][0] < 0.28,
        details=f"folded end tab aabb={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
