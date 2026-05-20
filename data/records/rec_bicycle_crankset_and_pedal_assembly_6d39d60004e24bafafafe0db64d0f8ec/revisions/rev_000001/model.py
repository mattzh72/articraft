from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CRANK_LENGTH = 0.170
DRIVE_Z = 0.055
OPPOSITE_Z = -0.055
CHAINRING_Z = 0.040


def _rotated(points: list[tuple[float, float]], angle: float) -> list[tuple[float, float]]:
    ca, sa = cos(angle), sin(angle)
    return [(x * ca - y * sa, x * sa + y * ca) for x, y in points]


def _capsule_profile(
    x0: float,
    x1: float,
    width: float,
    *,
    segments: int = 14,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    radius = width / 2.0
    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        a = -pi / 2.0 + i * pi / segments
        pts.append((x1 + radius * cos(a), radius * sin(a)))
    for i in range(segments + 1):
        a = pi / 2.0 + i * pi / segments
        pts.append((x0 + radius * cos(a), radius * sin(a)))
    return _rotated(pts, angle)


def _extrude_profile(
    profile: list[tuple[float, float]],
    thickness: float,
    z_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )


def _z_cylinder(radius: float, length: float, z_center: float, x: float = 0.0, y: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((x, y, z_center - length / 2.0))
    )


def _radial_bar(
    r0: float,
    r1: float,
    width: float,
    angle: float,
    thickness: float,
    z_center: float,
) -> cq.Workplane:
    ca, sa = cos(angle), sin(angle)
    px, py = -sa * width / 2.0, ca * width / 2.0
    p0 = (r0 * ca, r0 * sa)
    p1 = (r1 * ca, r1 * sa)
    profile = [
        (p0[0] + px, p0[1] + py),
        (p1[0] + px, p1[1] + py),
        (p1[0] - px, p1[1] - py),
        (p0[0] - px, p0[1] - py),
    ]
    return _extrude_profile(profile, thickness, z_center)


def _crank_arm(angle: float, z_center: float) -> cq.Workplane:
    arm = _extrude_profile(_capsule_profile(0.018, CRANK_LENGTH, 0.030, angle=angle), 0.012, z_center)
    slot = _extrude_profile(_capsule_profile(0.050, 0.132, 0.012, segments=10, angle=angle), 0.018, z_center)
    arm = arm.cut(slot)

    end_x = CRANK_LENGTH * cos(angle)
    end_y = CRANK_LENGTH * sin(angle)
    arm = arm.union(_z_cylinder(0.026, 0.018, z_center))
    arm = arm.union(_z_cylinder(0.017, 0.018, z_center, end_x, end_y))
    return arm


def _chainring_profile(teeth: int = 48) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(teeth * 2):
        angle = 2.0 * pi * i / (teeth * 2)
        radius = 0.112 if i % 2 == 0 else 0.106
        pts.append((radius * cos(angle), radius * sin(angle)))
    return pts


def _make_crank_body() -> cq.Workplane:
    ring_thickness = 0.006
    ring = _extrude_profile(_chainring_profile(), ring_thickness, CHAINRING_Z)
    inner_cut = _z_cylinder(0.086, ring_thickness + 0.006, CHAINRING_Z)
    ring = ring.cut(inner_cut)

    body = ring
    for i in range(5):
        a = pi / 2.0 + i * 2.0 * pi / 5.0
        body = body.union(_radial_bar(0.021, 0.092, 0.014, a, 0.007, CHAINRING_Z))
        bx, by = 0.077 * cos(a), 0.077 * sin(a)
        body = body.union(_z_cylinder(0.0065, 0.008, CHAINRING_Z + 0.001, bx, by))

    body = body.union(_z_cylinder(0.030, 0.020, CHAINRING_Z))
    body = body.union(_z_cylinder(0.017, 0.140, 0.0))
    body = body.union(_crank_arm(-pi / 2.0, DRIVE_Z))
    body = body.union(_crank_arm(pi / 2.0, OPPOSITE_Z))
    return body


def _make_bottom_bracket() -> cq.Workplane:
    shell = _z_cylinder(0.032, 0.082, 0.0).cut(_z_cylinder(0.020, 0.090, 0.0))
    shell = shell.union(_z_cylinder(0.037, 0.008, 0.045).cut(_z_cylinder(0.020, 0.012, 0.045)))
    shell = shell.union(_z_cylinder(0.037, 0.008, -0.045).cut(_z_cylinder(0.020, 0.012, -0.045)))
    return shell


def _make_pedal_body() -> cq.Workplane:
    z_mid = 0.095
    frame = cq.Workplane("XY").box(0.110, 0.066, 0.034).translate((0.0, 0.0, z_mid))
    frame = frame.cut(cq.Workplane("XY").box(0.078, 0.038, 0.050).translate((0.0, 0.0, z_mid)))
    frame = frame.union(cq.Workplane("XY").box(0.104, 0.008, 0.030).translate((0.0, 0.0, z_mid)))
    frame = frame.union(cq.Workplane("XY").box(0.010, 0.058, 0.030).translate((0.0, 0.0, z_mid)))
    frame = frame.union(_z_cylinder(0.012, 0.075, z_mid))

    for z in (0.076, 0.114):
        for x in (-0.040, 0.040):
            for y in (-0.024, 0.024):
                frame = frame.union(_z_cylinder(0.0023, 0.006, z, x, y))
    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_crankset")

    model.material("satin_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("anodized_black", rgba=(0.02, 0.022, 0.025, 1.0))
    model.material("dark_grey", rgba=(0.10, 0.105, 0.11, 1.0))
    model.material("polished_steel", rgba=(0.82, 0.84, 0.86, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_cadquery(_make_bottom_bracket(), "bottom_bracket_shell", tolerance=0.0006),
        material="satin_silver",
        name="bb_shell",
    )

    crank_assembly = model.part("crank_assembly")
    crank_assembly.visual(
        mesh_from_cadquery(_make_crank_body(), "spider_chainring_cranks", tolerance=0.0006),
        material="anodized_black",
        name="crank_body",
    )
    for i in range(5):
        a = pi / 2.0 + i * 2.0 * pi / 5.0
        crank_assembly.visual(
            Cylinder(radius=0.0042, length=0.004),
            origin=Origin(xyz=(0.077 * cos(a), 0.077 * sin(a), CHAINRING_Z + 0.0055)),
            material="polished_steel",
            name=f"chainring_bolt_{i}",
        )

    crank_assembly.visual(
        Cylinder(radius=0.006, length=0.115),
        origin=Origin(xyz=(0.0, -CRANK_LENGTH, DRIVE_Z + 0.0575)),
        material="polished_steel",
        name="drive_stub_axle",
    )
    crank_assembly.visual(
        Cylinder(radius=0.006, length=0.115),
        origin=Origin(xyz=(0.0, CRANK_LENGTH, OPPOSITE_Z - 0.0575)),
        material="polished_steel",
        name="opposite_stub_axle",
    )

    drive_pedal = model.part("drive_pedal")
    drive_pedal.visual(
        mesh_from_cadquery(_make_pedal_body(), "drive_platform_pedal", tolerance=0.0006),
        material="dark_grey",
        name="pedal_body",
    )

    opposite_pedal = model.part("opposite_pedal")
    opposite_pedal.visual(
        mesh_from_cadquery(_make_pedal_body(), "opposite_platform_pedal", tolerance=0.0006),
        material="dark_grey",
        name="pedal_body",
    )

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.REVOLUTE,
        parent=bottom_bracket,
        child=crank_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=8.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "drive_pedal_spin",
        ArticulationType.REVOLUTE,
        parent=crank_assembly,
        child=drive_pedal,
        origin=Origin(xyz=(0.0, -CRANK_LENGTH, DRIVE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "opposite_pedal_spin",
        ArticulationType.REVOLUTE,
        parent=crank_assembly,
        child=opposite_pedal,
        origin=Origin(xyz=(0.0, CRANK_LENGTH, OPPOSITE_Z), rpy=(pi, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottom = object_model.get_part("bottom_bracket")
    crank = object_model.get_part("crank_assembly")
    drive_pedal = object_model.get_part("drive_pedal")
    opposite_pedal = object_model.get_part("opposite_pedal")
    bb_joint = object_model.get_articulation("bottom_bracket_spin")
    drive_joint = object_model.get_articulation("drive_pedal_spin")
    opposite_joint = object_model.get_articulation("opposite_pedal_spin")

    ctx.allow_overlap(
        bottom,
        crank,
        elem_a="bb_shell",
        elem_b="crank_body",
        reason="The one-piece crank body mesh includes the through-spindle captured inside the bottom bracket bore.",
    )
    ctx.expect_origin_distance(
        bottom,
        crank,
        axes="xy",
        max_dist=0.001,
        name="bottom bracket and crank spindle are coaxial",
    )
    ctx.expect_overlap(
        bottom,
        crank,
        axes="z",
        min_overlap=0.070,
        elem_a="bb_shell",
        elem_b="crank_body",
        name="spindle remains retained through bottom bracket shell",
    )

    ctx.allow_overlap(
        crank,
        drive_pedal,
        elem_a="drive_stub_axle",
        elem_b="pedal_body",
        reason="The platform pedal sleeve intentionally surrounds the threaded stub axle.",
    )
    ctx.expect_overlap(
        crank,
        drive_pedal,
        axes="z",
        min_overlap=0.040,
        elem_a="drive_stub_axle",
        elem_b="pedal_body",
        name="drive pedal sleeve captures stub axle length",
    )
    ctx.expect_overlap(
        crank,
        drive_pedal,
        axes="xy",
        min_overlap=0.010,
        elem_a="drive_stub_axle",
        elem_b="pedal_body",
        name="drive pedal sleeve is coaxial with stub axle",
    )

    ctx.allow_overlap(
        crank,
        opposite_pedal,
        elem_a="opposite_stub_axle",
        elem_b="pedal_body",
        reason="The opposite platform pedal sleeve intentionally surrounds the threaded stub axle.",
    )
    ctx.expect_overlap(
        crank,
        opposite_pedal,
        axes="z",
        min_overlap=0.040,
        elem_a="opposite_stub_axle",
        elem_b="pedal_body",
        name="opposite pedal sleeve captures stub axle length",
    )
    ctx.expect_overlap(
        crank,
        opposite_pedal,
        axes="xy",
        min_overlap=0.010,
        elem_a="opposite_stub_axle",
        elem_b="pedal_body",
        name="opposite pedal sleeve is coaxial with stub axle",
    )

    ctx.check(
        "bottom bracket uses a revolute joint",
        bb_joint.articulation_type == ArticulationType.REVOLUTE and tuple(bb_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={bb_joint.articulation_type}, axis={bb_joint.axis}",
    )
    ctx.check(
        "both pedals spin on revolute stub axles",
        drive_joint.articulation_type == ArticulationType.REVOLUTE
        and opposite_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(drive_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(opposite_joint.axis) == (0.0, 0.0, 1.0),
        details=f"drive=({drive_joint.articulation_type}, {drive_joint.axis}), opposite=({opposite_joint.articulation_type}, {opposite_joint.axis})",
    )

    ctx.expect_origin_distance(
        drive_pedal,
        crank,
        axes="xy",
        min_dist=0.165,
        max_dist=0.175,
        name="drive pedal sits at road crank radius",
    )
    ctx.expect_origin_distance(
        opposite_pedal,
        crank,
        axes="xy",
        min_dist=0.165,
        max_dist=0.175,
        name="opposite pedal sits at road crank radius",
    )

    drive_rest = ctx.part_world_position(drive_pedal)
    with ctx.pose({bb_joint: pi / 2.0}):
        drive_quarter = ctx.part_world_position(drive_pedal)
    ctx.check(
        "bottom bracket rotation carries crank arms",
        drive_rest is not None
        and drive_quarter is not None
        and abs(drive_rest[1] + CRANK_LENGTH) < 0.004
        and abs(drive_quarter[0] - CRANK_LENGTH) < 0.004,
        details=f"rest={drive_rest}, quarter_turn={drive_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
