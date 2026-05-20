from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_x(
    length: float,
    outer_radius: float,
    inner_radius: float,
    *,
    x_start: float,
) -> cq.Workplane:
    """A hollow tube whose axis is the model X axis."""
    return (
        cq.Workplane("YZ", origin=(x_start, 0.0, 0.0))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _solid_cylinder_x(length: float, radius: float, *, x_start: float) -> cq.Workplane:
    return cq.Workplane("YZ", origin=(x_start, 0.0, 0.0)).circle(radius).extrude(length)


def _prism_body_shape() -> cq.Workplane:
    # Faceted YZ cross-section: a compact roof-prism monocular body rather than
    # a round telescope tube.
    section = [
        (-0.033, -0.025),
        (-0.026, -0.036),
        (0.026, -0.036),
        (0.033, -0.025),
        (0.037, 0.006),
        (0.025, 0.038),
        (-0.025, 0.038),
        (-0.037, 0.006),
    ]
    return (
        cq.Workplane("YZ", origin=(-0.058, 0.0, 0.0))
        .polyline(section)
        .close()
        .extrude(0.088)
        .edges()
        .fillet(0.004)
    )


def _objective_housing_shape() -> cq.Workplane:
    tube = _annular_x(0.074, 0.031, 0.0235, x_start=0.018)
    front_retainer = _annular_x(0.009, 0.033, 0.0228, x_start=0.083)
    rear_shoulder = _annular_x(0.010, 0.029, 0.0215, x_start=0.014)
    return tube.union(front_retainer).union(rear_shoulder)


def _focus_ring_shape() -> cq.Workplane:
    ring = _annular_x(0.028, 0.0215, 0.0168, x_start=-0.014)
    for x_center in (-0.009, 0.0, 0.009):
        ring = ring.union(_annular_x(0.003, 0.0230, 0.0168, x_start=x_center - 0.0015))

    # One raised thumb/index pad breaks axial symmetry so rotation is visible.
    pad = (
        cq.Workplane("XY")
        .box(0.018, 0.009, 0.004)
        .translate((0.0, 0.0, 0.0225))
    )
    return ring.union(pad)


def _eyecup_shape() -> cq.Workplane:
    front_collar = _annular_x(0.008, 0.0205, 0.0140, x_start=-0.008)
    soft_cup = _annular_x(0.022, 0.0255, 0.0160, x_start=-0.030)
    rear_lip = _annular_x(0.005, 0.0280, 0.0175, x_start=-0.034)
    tab = (
        cq.Workplane("XY")
        .box(0.013, 0.010, 0.0045)
        .translate((-0.020, 0.0, 0.0255))
    )
    return front_collar.union(soft_cup).union(rear_lip).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monocular_spotter")

    armor = model.material("matte_rubber_armor", rgba=(0.045, 0.050, 0.048, 1.0))
    dark = model.material("black_anodized_metal", rgba=(0.015, 0.016, 0.018, 1.0))
    grip = model.material("dark_gray_grip", rgba=(0.10, 0.11, 0.11, 1.0))
    glass = model.material("coated_blue_glass", rgba=(0.16, 0.33, 0.45, 0.72))
    metal = model.material("brushed_threaded_metal", rgba=(0.55, 0.56, 0.54, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_prism_body_shape(), "prism_body"),
        material=armor,
        name="prism_body",
    )
    body.visual(
        mesh_from_cadquery(_objective_housing_shape(), "objective_housing"),
        material=dark,
        name="objective_housing",
    )
    body.visual(
        Cylinder(radius=0.0240, length=0.0035),
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_glass",
    )
    body.visual(
        mesh_from_cadquery(_solid_cylinder_x(0.060, 0.0152, x_start=-0.110), "rear_barrel"),
        material=dark,
        name="rear_barrel",
    )
    body.visual(
        Box((0.060, 0.005, 0.040)),
        origin=Origin(xyz=(-0.014, 0.036, 0.000)),
        material=grip,
        name="side_grip_0",
    )
    body.visual(
        Box((0.060, 0.005, 0.040)),
        origin=Origin(xyz=(-0.014, -0.036, 0.000)),
        material=grip,
        name="side_grip_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.015, 0.0, -0.040)),
        material=dark,
        name="tripod_boss",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.015, 0.0, -0.046)),
        material=metal,
        name="threaded_insert",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_focus_ring_shape(), "focus_tube"),
        material=grip,
        name="focus_tube",
    )

    eyecup = model.part("eyecup")
    eyecup.visual(
        mesh_from_cadquery(_eyecup_shape(), "eyecup_shell"),
        material=dark,
        name="eyecup_shell",
    )
    eyecup.visual(
        Cylinder(radius=0.0162, length=0.0025),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="ocular_glass",
    )

    model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(-0.096, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    model.articulation(
        "eyecup_twist",
        ArticulationType.REVOLUTE,
        parent=focus_ring,
        child=eyecup,
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    eyecup = object_model.get_part("eyecup")
    focus = object_model.get_articulation("focus_spin")
    twist = object_model.get_articulation("eyecup_twist")

    ctx.check(
        "focus ring rotates continuously on optical axis",
        focus.articulation_type == ArticulationType.CONTINUOUS and tuple(focus.axis) == (1.0, 0.0, 0.0),
        details=f"type={focus.articulation_type}, axis={focus.axis}",
    )
    ctx.check(
        "eyecup twists on a limited revolute joint",
        twist.articulation_type == ArticulationType.REVOLUTE
        and tuple(twist.axis) == (1.0, 0.0, 0.0)
        and twist.motion_limits is not None
        and twist.motion_limits.lower == 0.0
        and twist.motion_limits.upper is not None
        and twist.motion_limits.upper > 1.0,
        details=f"type={twist.articulation_type}, axis={twist.axis}, limits={twist.motion_limits}",
    )
    ctx.expect_within(
        body,
        focus_ring,
        axes="yz",
        inner_elem="rear_barrel",
        outer_elem="focus_tube",
        margin=0.002,
        name="rear eyepiece barrel is centered inside focus ring",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="x",
        elem_a="focus_tube",
        elem_b="rear_barrel",
        min_overlap=0.024,
        name="focus ring remains sleeved over rear barrel",
    )
    ctx.expect_gap(
        focus_ring,
        eyecup,
        axis="x",
        positive_elem="focus_tube",
        negative_elem="eyecup_shell",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="eyecup seats against focus ring rear lip",
    )

    with ctx.pose({focus: math.pi / 2.0, twist: 1.2}):
        ctx.expect_within(
            body,
            focus_ring,
            axes="yz",
            inner_elem="rear_barrel",
            outer_elem="focus_tube",
            margin=0.002,
            name="rotated focus ring stays coaxial",
        )
        ctx.expect_gap(
            focus_ring,
            eyecup,
            axis="x",
            positive_elem="focus_tube",
            negative_elem="eyecup_shell",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="twisted eyecup stays seated on axis",
        )

    return ctx.report()


object_model = build_object_model()
