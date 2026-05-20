from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _annulus_yz(outer_radius: float, inner_radius: float, length: float, *, x: float = 0.0):
    """Annular cylinder with its axis on local X, authored from a YZ workplane."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x - length / 2.0, 0.0, 0.0))
    )


def _bar_yz(r_inner: float, r_outer: float, width: float, angle_rad: float, thickness: float):
    """Flat radial bar in the YZ plane, extruded along X."""
    uy = math.sin(angle_rad)
    uz = math.cos(angle_rad)
    py = -uz
    pz = uy
    half = width / 2.0
    pts = [
        (r_inner * uy + half * py, r_inner * uz + half * pz),
        (r_outer * uy + half * py, r_outer * uz + half * pz),
        (r_outer * uy - half * py, r_outer * uz - half * pz),
        (r_inner * uy - half * py, r_inner * uz - half * pz),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(pts)
        .close()
        .extrude(thickness)
        .translate((-thickness / 2.0, 0.0, 0.0))
    )


def _bottom_bracket_shell():
    body = _annulus_yz(0.022, 0.012, 0.070)
    # Narrow inner bearing races lightly capture the spindle; the wider body bore
    # stays clear so the shell does not read as a solid rod.
    cup_pos = _annulus_yz(0.026, 0.0088, 0.008, x=0.039)
    cup_neg = _annulus_yz(0.026, 0.0088, 0.008, x=-0.039)
    return body.union(cup_pos).union(cup_neg)


def _crank_arm_shape(direction: int, *, spider: bool):
    """Solid track crank arm. direction=-1 points down, +1 points up."""
    thickness = 0.014
    arm_pts = [
        (-0.016, direction * 0.030),
        (0.016, direction * 0.030),
        (0.010, direction * 0.154),
        (0.015, direction * 0.170),
        (-0.015, direction * 0.170),
        (-0.010, direction * 0.154),
    ]
    arm = (
        cq.Workplane("YZ")
        .polyline(arm_pts)
        .close()
        .extrude(thickness)
        .translate((-thickness / 2.0, 0.0, 0.0))
    )
    boss = _annulus_yz(0.034, 0.0087, thickness)
    pedal_eye = (
        cq.Workplane("YZ")
        .center(0.0, direction * 0.170)
        .circle(0.016)
        .circle(0.006)
        .extrude(thickness)
        .translate((-thickness / 2.0, 0.0, 0.0))
    )
    shape = boss.union(arm).union(pedal_eye)

    if spider:
        for deg in (18, 90, 162, 234, 306):
            angle = math.radians(deg)
            shape = shape.union(_bar_yz(0.026, 0.073, 0.010, angle, thickness))
            y = 0.071 * math.sin(angle)
            z = 0.071 * math.cos(angle)
            boss_pad = (
                cq.Workplane("YZ")
                .center(y, z)
                .circle(0.0085)
                .circle(0.0035)
                .extrude(thickness)
                .translate((-thickness / 2.0, 0.0, 0.0))
            )
            shape = shape.union(boss_pad)

    return shape


def _chainring_shape():
    teeth = 48
    root_radius = 0.099
    tip_radius = 0.108
    inner_radius = 0.052
    thickness = 0.006
    points = []
    for i in range(teeth * 2):
        angle = 2.0 * math.pi * i / (teeth * 2)
        radius = tip_radius if i % 2 == 0 else root_radius
        points.append((radius * math.sin(angle), radius * math.cos(angle)))

    ring = (
        cq.Workplane("YZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((-thickness / 2.0, 0.0, 0.0))
    )

    center_cut = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(thickness + 0.004)
        .translate((-(thickness + 0.004) / 2.0, 0.0, 0.0))
    )
    ring = ring.cut(center_cut)

    # The keyed slot at twelve o'clock makes this read as a keyed track ring
    # rather than a plain circular sprocket.
    key_cut = (
        cq.Workplane("YZ")
        .center(0.0, inner_radius + 0.006)
        .rect(0.010, 0.014)
        .extrude(thickness + 0.004)
        .translate((-(thickness + 0.004) / 2.0, 0.0, 0.0))
    )
    ring = ring.cut(key_cut)

    for deg in (18, 90, 162, 234, 306):
        angle = math.radians(deg)
        y = 0.071 * math.sin(angle)
        z = 0.071 * math.cos(angle)
        bolt_cut = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(0.0038)
            .extrude(thickness + 0.004)
            .translate((-(thickness + 0.004) / 2.0, 0.0, 0.0))
        )
        ring = ring.cut(bolt_cut)

    return ring


def _pedal_shape(direction: int):
    """Open rectangular track pedal/cage with a continuous axle on local X."""
    body_x = 0.060
    body_y = 0.095
    body_z = 0.026
    center_x = direction * 0.082
    cage = cq.Workplane("XY").box(body_x, body_y, body_z).translate((center_x, 0.0, 0.0))
    window = (
        cq.Workplane("XY")
        .box(0.032, 0.061, body_z * 1.6)
        .translate((center_x, 0.0, 0.0))
    )
    cage = cage.cut(window)

    axle = cq.Workplane("YZ").circle(0.0045).extrude(0.112)
    if direction < 0:
        axle = axle.translate((-0.112, 0.0, 0.0))
    cage = cage.union(axle)

    flange = cq.Workplane("YZ").circle(0.010).extrude(0.003)
    if direction < 0:
        flange = flange.translate((-0.003, 0.0, 0.0))
    cage = cage.union(flange)
    return cage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fixie_track_crankset")

    black = Material("black_anodized_aluminum", rgba=(0.01, 0.01, 0.012, 1.0))
    dark = Material("dark_bearing_cups", rgba=(0.06, 0.06, 0.065, 1.0))
    steel = Material("polished_steel", rgba=(0.72, 0.72, 0.70, 1.0))
    silver = Material("brushed_chainring_silver", rgba=(0.82, 0.80, 0.74, 1.0))

    shell = model.part("bottom_bracket")
    shell.visual(
        mesh_from_cadquery(_bottom_bracket_shell(), "bottom_bracket_shell", tolerance=0.0007),
        material=dark,
        name="bearing_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.009, length=0.168),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle_axle",
    )

    right_arm = model.part("right_arm")
    right_arm.visual(
        mesh_from_cadquery(
            _crank_arm_shape(-1, spider=True), "right_solid_crank_arm", tolerance=0.0007
        ),
        material=black,
        name="right_arm_body",
    )

    left_arm = model.part("left_arm")
    left_arm.visual(
        mesh_from_cadquery(
            _crank_arm_shape(1, spider=False), "left_crank_arm", tolerance=0.0007
        ),
        material=black,
        name="left_arm_body",
    )

    chainring = model.part("chainring")
    chainring.visual(
        mesh_from_cadquery(_chainring_shape(), "keyed_track_chainring", tolerance=0.0006),
        material=silver,
        name="toothed_keyed_ring",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        mesh_from_cadquery(_pedal_shape(1), "right_track_pedal", tolerance=0.0007),
        material=black,
        name="right_pedal_cage",
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        mesh_from_cadquery(_pedal_shape(-1), "left_track_pedal", tolerance=0.0007),
        material=black,
        name="left_pedal_cage",
    )

    spin_limits = MotionLimits(effort=80.0, velocity=12.0)
    pedal_limits = MotionLimits(effort=8.0, velocity=18.0)

    model.articulation(
        "bottom_bracket_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "spindle_to_right_arm",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_arm,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_left_arm",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_arm,
        origin=Origin(xyz=(-0.078, 0.0, 0.0)),
    )
    model.articulation(
        "right_arm_to_chainring",
        ArticulationType.FIXED,
        parent=right_arm,
        child=chainring,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )
    model.articulation(
        "right_arm_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_arm,
        child=right_pedal,
        origin=Origin(xyz=(0.007, 0.0, -0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=pedal_limits,
    )
    model.articulation(
        "left_arm_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child=left_pedal,
        origin=Origin(xyz=(-0.007, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=pedal_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spin = object_model.get_articulation("bottom_bracket_to_spindle")
    right_pedal_joint = object_model.get_articulation("right_arm_to_pedal")
    left_pedal_joint = object_model.get_articulation("left_arm_to_pedal")

    ctx.check(
        "spindle has a continuous bottom-bracket rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "both pedals have continuous axle rotations",
        right_pedal_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_pedal_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"right={right_pedal_joint.articulation_type}, left={left_pedal_joint.articulation_type}",
    )

    right_arm = object_model.get_part("right_arm")
    left_arm = object_model.get_part("left_arm")
    chainring = object_model.get_part("chainring")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")
    bottom_bracket = object_model.get_part("bottom_bracket")
    spindle_part = object_model.get_part("spindle")

    ctx.allow_overlap(
        bottom_bracket,
        spindle_part,
        elem_a="bearing_shell",
        elem_b="spindle_axle",
        reason="The spindle is lightly captured inside the two simplified bearing race rings.",
    )
    ctx.allow_overlap(
        spindle_part,
        right_arm,
        elem_a="spindle_axle",
        elem_b="right_arm_body",
        reason="The crank boss is represented as a press-fit keyed taper on the spindle.",
    )
    ctx.allow_overlap(
        spindle_part,
        left_arm,
        elem_a="spindle_axle",
        elem_b="left_arm_body",
        reason="The crank boss is represented as a press-fit keyed taper on the spindle.",
    )
    ctx.expect_overlap(
        bottom_bracket,
        spindle_part,
        axes="x",
        min_overlap=0.06,
        elem_a="bearing_shell",
        elem_b="spindle_axle",
        name="spindle runs through the bottom bracket shell",
    )
    ctx.expect_overlap(
        spindle_part,
        right_arm,
        axes="x",
        min_overlap=0.010,
        elem_a="spindle_axle",
        elem_b="right_arm_body",
        name="right crank boss remains on the spindle",
    )
    ctx.expect_overlap(
        spindle_part,
        left_arm,
        axes="x",
        min_overlap=0.010,
        elem_a="spindle_axle",
        elem_b="left_arm_body",
        name="left crank boss remains on the spindle",
    )

    ctx.expect_gap(
        chainring,
        right_arm,
        axis="x",
        max_gap=0.001,
        max_penetration=0.00001,
        name="chainring sits just outside the right spider",
    )
    ctx.expect_gap(
        right_pedal,
        right_arm,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="right pedal axle starts outboard of right arm",
    )
    ctx.expect_gap(
        left_arm,
        left_pedal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="left pedal axle starts outboard of left arm",
    )

    rest_right = ctx.part_element_world_aabb(right_arm, elem="right_arm_body")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_right = ctx.part_element_world_aabb(right_arm, elem="right_arm_body")
        turned_left = ctx.part_element_world_aabb(left_arm, elem="left_arm_body")
    ctx.check(
        "crank arms follow spindle rotation",
        rest_right is not None
        and turned_right is not None
        and turned_left is not None
        and turned_right[1][1] > 0.14
        and turned_left[0][1] < -0.14,
        details=f"rest_right={rest_right}, turned_right={turned_right}, turned_left={turned_left}",
    )

    return ctx.report()


object_model = build_object_model()
