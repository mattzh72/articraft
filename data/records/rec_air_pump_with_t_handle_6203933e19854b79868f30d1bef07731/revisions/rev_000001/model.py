from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_cylinder(length: float, outer_radius: float, inner_radius: float):
    """CadQuery tube centered on local Z with open ends."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.020)
        .translate((0.0, 0.0, -0.010))
    )
    return outer.cut(inner).translate((0.0, 0.0, -length / 2.0))


def _rectangular_tube_x(length: float, outer_y: float, outer_z: float, inner_y: float, inner_z: float):
    """Open-ended rectangular slide guide centered on local X."""
    outer = cq.Workplane("XY").box(length, outer_y, outer_z)
    inner = cq.Workplane("XY").box(length + 0.040, inner_y, inner_z)
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_floor_air_pump")

    painted_steel = Material("painted_steel", rgba=(0.10, 0.15, 0.18, 1.0))
    blue_enamel = Material("blue_enamel", rgba=(0.02, 0.16, 0.42, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    polished_steel = Material("polished_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    gauge_white = Material("gauge_face_white", rgba=(0.92, 0.90, 0.82, 1.0))
    red = Material("release_red", rgba=(0.85, 0.04, 0.02, 1.0))
    needle_red = Material("needle_red", rgba=(0.80, 0.02, 0.02, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.24, 0.16, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=painted_steel,
        name="base_plate",
    )
    frame.visual(
        Box((0.52, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=painted_steel,
        name="guide_floor",
    )
    frame.visual(
        Box((0.52, 0.014, 0.056)),
        origin=Origin(xyz=(0.0, -0.034, 0.064)),
        material=painted_steel,
        name="guide_side_0",
    )
    frame.visual(
        Box((0.52, 0.014, 0.056)),
        origin=Origin(xyz=(0.0, 0.034, 0.064)),
        material=painted_steel,
        name="guide_side_1",
    )
    frame.visual(
        Box((0.52, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=painted_steel,
        name="guide_top",
    )
    frame.visual(
        mesh_from_cadquery(_hollow_cylinder(0.82, 0.055, 0.030), "barrel_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=blue_enamel,
        name="barrel_shell",
    )
    frame.visual(
        mesh_from_cadquery(_hollow_cylinder(0.055, 0.070, 0.030), "lower_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=painted_steel,
        name="lower_collar",
    )
    frame.visual(
        mesh_from_cadquery(_hollow_cylinder(0.045, 0.067, 0.026), "upper_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=painted_steel,
        name="upper_collar",
    )
    for i, y in enumerate((-0.050, 0.050)):
        frame.visual(
            Box((0.090, 0.020, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=painted_steel,
            name=f"side_bracket_{i}",
        )
    for x in (-0.052, 0.052):
        frame.visual(
            Cylinder(radius=0.007, length=0.760),
            origin=Origin(xyz=(x, 0.033, 0.540)),
            material=polished_steel,
            name=f"tie_rod_{0 if x < 0 else 1}",
        )

    # Gauge housing and fixed gauge details on the front of the cylinder.
    frame.visual(
        Cylinder(radius=0.017, length=0.058),
        origin=Origin(xyz=(0.0, -0.080, 0.735), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="gauge_neck",
    )
    frame.visual(
        Cylinder(radius=0.078, length=0.060),
        origin=Origin(xyz=(0.0, -0.135, 0.735), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="gauge_body",
    )
    frame.visual(
        Cylinder(radius=0.064, length=0.004),
        origin=Origin(xyz=(0.0, -0.167, 0.735), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_white,
        name="gauge_face",
    )
    frame.visual(
        Box((0.008, 0.002, 0.056)),
        origin=Origin(xyz=(0.018, -0.170, 0.753), rpy=(0.0, 0.42, 0.0)),
        material=needle_red,
        name="gauge_needle",
    )
    for i, (x, z, sx, sz) in enumerate(
        (
            (0.0, 0.792, 0.006, 0.016),
            (-0.046, 0.735, 0.014, 0.006),
            (0.046, 0.735, 0.014, 0.006),
            (-0.030, 0.775, 0.010, 0.006),
            (0.030, 0.775, 0.010, 0.006),
        )
    ):
        frame.visual(
            Box((sx, 0.002, sz)),
            origin=Origin(xyz=(x, -0.170, z)),
            material=painted_steel,
            name=f"gauge_tick_{i}",
        )
    frame.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.050, -0.166, 0.675), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="button_collar",
    )

    piston = model.part("piston")
    piston.visual(
        Cylinder(radius=0.012, length=0.840),
        origin=Origin(xyz=(0.0, 0.0, -0.280)),
        material=polished_steel,
        name="piston_rod",
    )
    piston.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=polished_steel,
        name="handle_stem",
    )
    piston.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=polished_steel,
        name="plunger_stop",
    )
    piston.visual(
        Cylinder(radius=0.034, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="t_handle_pad",
    )
    piston.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, -0.215, 0.165)),
        material=black_rubber,
        name="handle_end_0",
    )
    piston.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.215, 0.165)),
        material=black_rubber,
        name="handle_end_1",
    )

    stabilizer = model.part("stabilizer")
    stabilizer.visual(
        Box((0.820, 0.042, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=black_rubber,
        name="slide_bar",
    )
    stabilizer.visual(
        Box((0.120, 0.070, 0.018)),
        origin=Origin(xyz=(-0.380, 0.0, -0.022)),
        material=black_rubber,
        name="foot_pad_0",
    )
    stabilizer.visual(
        Box((0.120, 0.070, 0.018)),
        origin=Origin(xyz=(0.380, 0.0, -0.022)),
        material=black_rubber,
        name="foot_pad_1",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="button_cap",
    )
    release_button.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=red,
        name="button_dome",
    )

    model.articulation(
        "frame_to_piston",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, 0.9975)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.7, lower=0.0, upper=0.420),
    )
    model.articulation(
        "frame_to_stabilizer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stabilizer,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=-0.180, upper=0.180),
    )
    model.articulation(
        "frame_to_release_button",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=release_button,
        origin=Origin(xyz=(0.050, -0.169, 0.675)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.04, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    piston = object_model.get_part("piston")
    stabilizer = object_model.get_part("stabilizer")
    release_button = object_model.get_part("release_button")
    piston_slide = object_model.get_articulation("frame_to_piston")
    stabilizer_slide = object_model.get_articulation("frame_to_stabilizer")
    button_slide = object_model.get_articulation("frame_to_release_button")

    ctx.allow_overlap(
        frame,
        release_button,
        elem_a="button_collar",
        elem_b="button_cap",
        reason="The release button cap intentionally depresses into its shallow rubber collar when pushed.",
    )

    ctx.expect_gap(
        frame,
        stabilizer,
        axis="y",
        positive_elem="guide_side_1",
        negative_elem="slide_bar",
        min_gap=0.003,
        max_gap=0.008,
        name="stabilizer clears positive guide wall",
    )
    ctx.expect_gap(
        stabilizer,
        frame,
        axis="y",
        positive_elem="slide_bar",
        negative_elem="guide_side_0",
        min_gap=0.003,
        max_gap=0.008,
        name="stabilizer clears negative guide wall",
    )
    ctx.expect_gap(
        frame,
        stabilizer,
        axis="z",
        positive_elem="guide_top",
        negative_elem="slide_bar",
        min_gap=0.006,
        max_gap=0.010,
        name="stabilizer clears guide roof",
    )
    ctx.expect_gap(
        stabilizer,
        frame,
        axis="z",
        positive_elem="slide_bar",
        negative_elem="guide_floor",
        max_gap=0.001,
        max_penetration=0.001,
        name="stabilizer rides on guide floor",
    )
    ctx.expect_overlap(
        stabilizer,
        frame,
        axes="x",
        elem_a="slide_bar",
        elem_b="guide_floor",
        min_overlap=0.45,
        name="stabilizer has long retained insertion at center",
    )
    with ctx.pose({stabilizer_slide: 0.180}):
        ctx.expect_gap(
            frame,
            stabilizer,
            axis="y",
            positive_elem="guide_side_1",
            negative_elem="slide_bar",
            min_gap=0.003,
            max_gap=0.008,
            name="extended stabilizer clears positive guide wall",
        )
        ctx.expect_gap(
            frame,
            stabilizer,
            axis="z",
            positive_elem="guide_top",
            negative_elem="slide_bar",
            min_gap=0.006,
            max_gap=0.010,
            name="extended stabilizer clears guide roof",
        )
        ctx.expect_gap(
            stabilizer,
            frame,
            axis="z",
            positive_elem="slide_bar",
            negative_elem="guide_floor",
            max_gap=0.001,
            max_penetration=0.001,
            name="extended stabilizer rides on guide floor",
        )
        ctx.expect_overlap(
            stabilizer,
            frame,
            axes="x",
            elem_a="slide_bar",
            elem_b="guide_floor",
            min_overlap=0.30,
            name="extended stabilizer retains insertion",
        )

    ctx.expect_within(
        piston,
        frame,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        margin=0.0,
        name="piston rod centered within pump cylinder footprint",
    )
    ctx.expect_overlap(
        piston,
        frame,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.60,
        name="collapsed piston rod remains deeply inside cylinder",
    )
    rest_piston = ctx.part_world_position(piston)
    with ctx.pose({piston_slide: 0.420}):
        ctx.expect_overlap(
            piston,
            frame,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel_shell",
            min_overlap=0.20,
            name="raised piston rod still retains insertion",
        )
        raised_piston = ctx.part_world_position(piston)
    ctx.check(
        "T-handle and rod travel upward together",
        rest_piston is not None and raised_piston is not None and raised_piston[2] > rest_piston[2] + 0.40,
        details=f"rest={rest_piston}, raised={raised_piston}",
    )

    ctx.expect_gap(
        frame,
        release_button,
        axis="y",
        positive_elem="button_collar",
        negative_elem="button_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="release button is separate and seated on gauge housing",
    )
    rest_button = ctx.part_world_position(release_button)
    with ctx.pose({button_slide: 0.010}):
        pressed_button = ctx.part_world_position(release_button)
    ctx.check(
        "release button pushes inward",
        rest_button is not None and pressed_button is not None and pressed_button[1] > rest_button[1] + 0.008,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
