from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CRANK_LENGTH = 0.172
RIGHT_ARM_X = 0.076
LEFT_ARM_X = -0.076
PEDAL_FACE_X = 0.095


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _toothed_profile(root_radius: float, tip_radius: float, tooth_count: int) -> list[tuple[float, float]]:
    """A simple chainring outline with alternating root and pointed tooth flats."""

    points: list[tuple[float, float]] = []
    step = 2.0 * pi / tooth_count
    for tooth in range(tooth_count):
        base = tooth * step
        for frac, radius in (
            (0.00, root_radius),
            (0.22, tip_radius),
            (0.50, tip_radius),
            (0.78, root_radius),
        ):
            angle = base + frac * step
            points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _capsule_profile(x0: float, x1: float, width: float, segments: int = 14) -> list[tuple[float, float]]:
    """2D capsule centered on the X axis between x0 and x1."""

    if x1 < x0:
        x0, x1 = x1, x0
    radius = width * 0.5
    pts: list[tuple[float, float]] = []
    # Round nose at x1, top to bottom.
    for i in range(segments + 1):
        angle = pi / 2.0 - pi * i / segments
        pts.append((x1 + radius * cos(angle), radius * sin(angle)))
    # Round heel at x0, bottom to top.
    for i in range(segments + 1):
        angle = -pi / 2.0 - pi * i / segments
        pts.append((x0 + radius * cos(angle), radius * sin(angle)))
    return pts


def _ring_mesh(
    *,
    name: str,
    root_radius: float,
    tip_radius: float,
    inner_radius: float,
    tooth_count: int,
    thickness: float,
    x: float,
):
    geom = ExtrudeWithHolesGeometry(
        _toothed_profile(root_radius, tip_radius, tooth_count),
        [_circle_profile(inner_radius, 96)],
        thickness,
        center=True,
    )
    geom.rotate_y(pi / 2.0).translate(x, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _arm_mesh(*, x: float, direction: float, name: str):
    # The profile X coordinate becomes world -Z after the Y rotation.  A positive
    # profile direction therefore points down; negative points up.
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(0.014 * direction, CRANK_LENGTH * direction, 0.034, 16),
        [_capsule_profile(0.052 * direction, 0.137 * direction, 0.014, 12)],
        0.012,
        center=True,
    )
    geom.rotate_y(pi / 2.0).translate(x, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _spider_arm_mesh(angle: float, *, name: str):
    # Five arms in the right-arm plane connect the hub to the chainring bolt
    # circle.  Profile coordinates are the crank rotation plane.
    r0 = 0.026
    r1 = 0.070
    half_width = 0.005
    tx, ty = cos(angle), sin(angle)
    nx, ny = -sin(angle), cos(angle)
    outer = [
        (r0 * tx + half_width * nx, r0 * ty + half_width * ny),
        (r1 * tx + half_width * nx, r1 * ty + half_width * ny),
        (r1 * tx - half_width * nx, r1 * ty - half_width * ny),
        (r0 * tx - half_width * nx, r0 * ty - half_width * ny),
    ]
    geom = ExtrudeWithHolesGeometry(outer, [], 0.009, center=True)
    geom.rotate_y(pi / 2.0).translate(RIGHT_ARM_X - 0.006, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _bb_shell_mesh():
    outer = [
        (0.028, -0.042),
        (0.030, -0.037),
        (0.030, 0.037),
        (0.028, 0.042),
    ]
    inner = [
        (0.0118, -0.042),
        (0.0118, 0.042),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, "bottom_bracket_shell")


def _bearing_ring_mesh(x: float, name: str):
    outer = [
        (0.020, -0.003),
        (0.020, 0.003),
    ]
    inner = [
        (0.0120, -0.003),
        (0.0120, 0.003),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=48).rotate_y(pi / 2.0).translate(x, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _pedal_body_mesh(*, side: float, name: str):
    # Low-profile clip-in road pedal: a small rounded triangular plate in the
    # YZ plane, offset outboard from its stub axle.
    outer = [
        (-0.040, -0.012),
        (0.030, -0.017),
        (0.043, 0.000),
        (0.030, 0.017),
        (-0.040, 0.012),
    ]
    geom = ExtrudeWithHolesGeometry(outer, [], 0.012, center=True)
    # Local extrusion thickness is turned into the local X thickness.
    geom.rotate_y(pi / 2.0).translate(side * 0.052, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_chainring_road_crankset")

    model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    model.material("carbon_black", rgba=(0.045, 0.047, 0.050, 1.0))
    model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("polished_edges", rgba=(0.88, 0.89, 0.86, 1.0))
    model.material("bearing_black", rgba=(0.02, 0.022, 0.024, 1.0))

    shell = model.part("bottom_bracket_shell")
    shell.visual(_bb_shell_mesh(), material="dark_steel", name="shell_tube")
    shell.visual(
        _bearing_ring_mesh(0.045, "right_bearing_seal"),
        material="bearing_black",
        name="right_bearing_seal",
    )
    shell.visual(
        _bearing_ring_mesh(-0.045, "left_bearing_seal"),
        material="bearing_black",
        name="left_bearing_seal",
    )

    crank = model.part("crank_spindle")
    crank.visual(
        Cylinder(radius=0.010, length=0.176),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="axle_core",
    )
    crank.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="right_spindle_half",
    )
    crank.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.064, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="left_spindle_half",
    )
    crank.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(RIGHT_ARM_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="carbon_black",
        name="right_hub_boss",
    )
    crank.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(LEFT_ARM_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="carbon_black",
        name="left_hub_boss",
    )
    crank.visual(_arm_mesh(x=RIGHT_ARM_X, direction=1.0, name="right_hollow_arm"), material="carbon_black", name="right_arm")
    crank.visual(_arm_mesh(x=LEFT_ARM_X, direction=-1.0, name="left_hollow_arm"), material="carbon_black", name="left_arm")
    crank.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(PEDAL_FACE_X - 0.014, 0.0, -CRANK_LENGTH), rpy=(0.0, pi / 2.0, 0.0)),
        material="carbon_black",
        name="right_pedal_boss",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(-PEDAL_FACE_X + 0.014, 0.0, CRANK_LENGTH), rpy=(0.0, pi / 2.0, 0.0)),
        material="carbon_black",
        name="left_pedal_boss",
    )

    # Right-arm spider and triple rings.  One spider arm sits behind the crank
    # arm, as on a common road crank.
    for idx in range(5):
        angle = idx * 2.0 * pi / 5.0
        crank.visual(_spider_arm_mesh(angle, name=f"spider_arm_{idx}"), material="carbon_black", name=f"spider_arm_{idx}")
    crank.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(RIGHT_ARM_X - 0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="carbon_black",
        name="spider_hub",
    )
    crank.visual(
        _ring_mesh(
            name="large_chainring",
            root_radius=0.106,
            tip_radius=0.112,
            inner_radius=0.055,
            tooth_count=52,
            thickness=0.0032,
            x=0.054,
        ),
        material="brushed_aluminum",
        name="large_chainring",
    )
    crank.visual(
        _ring_mesh(
            name="middle_chainring",
            root_radius=0.087,
            tip_radius=0.093,
            inner_radius=0.043,
            tooth_count=42,
            thickness=0.0030,
            x=0.047,
        ),
        material="brushed_aluminum",
        name="middle_chainring",
    )
    crank.visual(
        _ring_mesh(
            name="small_chainring",
            root_radius=0.067,
            tip_radius=0.073,
            inner_radius=0.027,
            tooth_count=30,
            thickness=0.0030,
            x=0.040,
        ),
        material="brushed_aluminum",
        name="small_chainring",
    )
    for idx in range(5):
        angle = idx * 2.0 * pi / 5.0
        y = 0.060 * sin(angle)
        z = -0.060 * cos(angle)
        crank.visual(
            Cylinder(radius=0.0043, length=0.044),
            origin=Origin(xyz=(0.058, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material="dark_steel",
            name=f"chainring_bolt_{idx}",
        )
        crank.visual(
            Cylinder(radius=0.0070, length=0.003),
            origin=Origin(xyz=(0.079, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material="polished_edges",
            name=f"bolt_head_{idx}",
        )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.0047, length=0.048),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="axle",
    )
    right_pedal.visual(_pedal_body_mesh(side=1.0, name="right_pedal_body"), material="matte_black", name="body")
    right_pedal.visual(Box((0.010, 0.072, 0.006)), origin=Origin(xyz=(0.052, 0.0, 0.010)), material="dark_steel", name="cleat_plate")
    right_pedal.visual(Box((0.012, 0.010, 0.018)), origin=Origin(xyz=(0.052, 0.034, 0.006)), material="polished_edges", name="front_hook")
    right_pedal.visual(Box((0.012, 0.010, 0.018)), origin=Origin(xyz=(0.052, -0.034, 0.006)), material="polished_edges", name="rear_latch")

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.0047, length=0.048),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="axle",
    )
    left_pedal.visual(_pedal_body_mesh(side=-1.0, name="left_pedal_body"), material="matte_black", name="body")
    left_pedal.visual(Box((0.010, 0.072, 0.006)), origin=Origin(xyz=(-0.052, 0.0, 0.010)), material="dark_steel", name="cleat_plate")
    left_pedal.visual(Box((0.012, 0.010, 0.018)), origin=Origin(xyz=(-0.052, 0.034, 0.006)), material="polished_edges", name="front_hook")
    left_pedal.visual(Box((0.012, 0.010, 0.018)), origin=Origin(xyz=(-0.052, -0.034, 0.006)), material="polished_edges", name="rear_latch")

    model.articulation(
        "crank_axle",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=crank,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=18.0),
    )
    model.articulation(
        "right_pedal_axle",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=right_pedal,
        origin=Origin(xyz=(PEDAL_FACE_X, 0.0, -CRANK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )
    model.articulation(
        "left_pedal_axle",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=left_pedal,
        origin=Origin(xyz=(-PEDAL_FACE_X, 0.0, CRANK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("bottom_bracket_shell")
    crank = object_model.get_part("crank_spindle")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    crank_axle = object_model.get_articulation("crank_axle")

    ctx.expect_within(
        crank,
        shell,
        axes="yz",
        inner_elem="axle_core",
        outer_elem="shell_tube",
        margin=0.002,
        name="spindle is centered through the bottom-bracket shell",
    )
    ctx.expect_overlap(
        crank,
        shell,
        axes="x",
        min_overlap=0.070,
        elem_a="axle_core",
        elem_b="shell_tube",
        name="spindle passes through the shell",
    )
    ctx.expect_gap(
        right_pedal,
        crank,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="axle",
        negative_elem="right_pedal_boss",
        name="right pedal axle shoulders against the crank tip",
    )
    ctx.expect_gap(
        crank,
        left_pedal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="left_pedal_boss",
        negative_elem="axle",
        name="left pedal axle shoulders against the crank tip",
    )

    rest_pos = ctx.part_world_position(right_pedal)
    with ctx.pose({crank_axle: pi / 2.0}):
        quarter_pos = ctx.part_world_position(right_pedal)
    ctx.check(
        "crank axle rotates the pedal orbit",
        rest_pos is not None
        and quarter_pos is not None
        and quarter_pos[1] > rest_pos[1] + 0.12
        and quarter_pos[2] > rest_pos[2] + 0.12,
        details=f"rest={rest_pos}, quarter_turn={quarter_pos}",
    )

    return ctx.report()


object_model = build_object_model()
