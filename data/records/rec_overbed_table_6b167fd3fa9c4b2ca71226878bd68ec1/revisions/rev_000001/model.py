from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL_GREY = Material("powder_coated_steel", rgba=(0.72, 0.74, 0.73, 1.0))
CAST_ALLOY = Material("cast_alloy", rgba=(0.48, 0.50, 0.50, 1.0))
RUBBER = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
TRAY_LAMINATE = Material("warm_laminate", rgba=(0.86, 0.78, 0.63, 1.0))
TRAY_EDGE = Material("soft_grey_edge", rgba=(0.58, 0.60, 0.58, 1.0))
PADDLE_RED = Material("release_red", rgba=(0.75, 0.05, 0.04, 1.0))


def _tray_geometry() -> cq.Workplane:
    """Broad overbed meal tray with a raised rim, authored in meters."""

    base = (
        cq.Workplane("XY")
        .box(1.24, 0.62, 0.035)
        .edges("|Z")
        .fillet(0.035)
        .translate((0.0, 0.35, 0.0175))
    )
    front_lip = cq.Workplane("XY").box(1.15, 0.036, 0.047).translate((0.0, 0.650, 0.0565))
    rear_lip = cq.Workplane("XY").box(1.15, 0.036, 0.047).translate((0.0, 0.050, 0.0565))
    side_lip_0 = cq.Workplane("XY").box(0.038, 0.565, 0.047).translate((-0.595, 0.350, 0.0565))
    side_lip_1 = cq.Workplane("XY").box(0.038, 0.565, 0.047).translate((0.595, 0.350, 0.0565))
    return base.union(front_lip).union(rear_lip).union(side_lip_0).union(side_lip_1)


def _add_caster_yoke(base, x: float, y: float, index: int) -> None:
    sign = 1.0 if x >= 0.0 else -1.0
    base.visual(
        Box((0.006, 0.040, 0.076)),
        origin=Origin(xyz=(x - sign * 0.026, y, 0.076)),
        material=CAST_ALLOY,
        name=f"caster_yoke_{index}_0",
    )
    base.visual(
        Box((0.006, 0.040, 0.076)),
        origin=Origin(xyz=(x + sign * 0.026, y, 0.076)),
        material=CAST_ALLOY,
        name=f"caster_yoke_{index}_1",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.055),
        origin=Origin(xyz=(x, y, 0.135)),
        material=CAST_ALLOY,
        name=f"caster_stem_{index}",
    )


def _add_outer_column(base, x: float, y: float, index: int) -> None:
    base.visual(
        Box((0.170, 0.115, 0.025)),
        origin=Origin(xyz=(x, y, 0.185)),
        material=CAST_ALLOY,
        name=f"column_base_{index}",
    )
    base.visual(
        Box((0.105, 0.012, 0.620)),
        origin=Origin(xyz=(x, y - 0.0315, 0.480)),
        material=STEEL_GREY,
        name=f"outer_column_{index}_front_wall",
    )
    base.visual(
        Box((0.105, 0.012, 0.620)),
        origin=Origin(xyz=(x, y + 0.0315, 0.480)),
        material=STEEL_GREY,
        name=f"outer_column_{index}_rear_wall",
    )
    base.visual(
        Box((0.012, 0.075, 0.620)),
        origin=Origin(xyz=(x - 0.0465, y, 0.480)),
        material=STEEL_GREY,
        name=f"outer_column_{index}_side_wall_0",
    )
    base.visual(
        Box((0.012, 0.075, 0.620)),
        origin=Origin(xyz=(x + 0.0465, y, 0.480)),
        material=STEEL_GREY,
        name=f"outer_column_{index}_side_wall_1",
    )
    base.visual(
        Box((0.0145, 0.030, 0.065)),
        origin=Origin(xyz=(x - 0.03325, y, 0.755)),
        material=TRAY_EDGE,
        name=f"lift_guide_{index}_0",
    )
    base.visual(
        Box((0.0145, 0.030, 0.065)),
        origin=Origin(xyz=(x + 0.03325, y, 0.755)),
        material=TRAY_EDGE,
        name=f"lift_guide_{index}_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bariatric_overbed_table")
    for material in (STEEL_GREY, CAST_ALLOY, RUBBER, TRAY_LAMINATE, TRAY_EDGE, PADDLE_RED):
        model.material(material.name, rgba=material.rgba)

    base = model.part("base_frame")
    base.visual(Box((0.100, 0.860, 0.060)), origin=Origin(xyz=(-0.520, 0.0, 0.140)), material=STEEL_GREY, name="lower_rail_0")
    base.visual(Box((0.100, 0.860, 0.060)), origin=Origin(xyz=(0.520, 0.0, 0.140)), material=STEEL_GREY, name="lower_rail_1")
    base.visual(Box((1.040, 0.090, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.140)), material=STEEL_GREY, name="crossbar")

    caster_locations = (
        (-0.520, -0.370),
        (0.520, -0.370),
        (-0.520, 0.370),
        (0.520, 0.370),
    )
    for i, (x, y) in enumerate(caster_locations):
        _add_caster_yoke(base, x, y, i)

    for i, x in enumerate((-0.270, 0.270)):
        _add_outer_column(base, x, 0.030, i)

    base.visual(Box((0.025, 0.064, 0.070)), origin=Origin(xyz=(-0.482, -0.340, 0.105)), material=CAST_ALLOY, name="brake_pivot_0")
    base.visual(Box((0.025, 0.064, 0.070)), origin=Origin(xyz=(0.482, -0.340, 0.105)), material=CAST_ALLOY, name="brake_pivot_1")

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.016, length=0.939),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL_GREY,
        name="brake_axle",
    )
    brake_bar.visual(Box((0.260, 0.035, 0.018)), origin=Origin(xyz=(0.0, -0.022, 0.000)), material=STEEL_GREY, name="pedal_web")
    brake_bar.visual(Box((0.350, 0.060, 0.018)), origin=Origin(xyz=(0.0, -0.055, 0.008)), material=RUBBER, name="foot_pad")
    model.articulation(
        "base_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake_bar,
        origin=Origin(xyz=(0.0, -0.340, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=0.45),
    )

    for i, (x, y) in enumerate(caster_locations):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=0.045, length=0.034),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=RUBBER,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=0.019, length=0.050),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=CAST_ALLOY,
            name="hub",
        )
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(x, y, 0.055)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=20.0),
        )

    inner_posts = []
    for i, x in enumerate((-0.270, 0.270)):
        inner = model.part(f"inner_post_{i}")
        inner.visual(
            Box((0.052, 0.032, 0.680)),
            origin=Origin(xyz=(0.0, 0.0, -0.160)),
            material=CAST_ALLOY,
            name="inner_tube",
        )
        inner_posts.append(inner)
        model.articulation(
            f"base_to_inner_post_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=inner,
            origin=Origin(xyz=(x, 0.030, 0.790)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.25),
            mimic=Mimic("base_to_inner_post_0") if i == 1 else None,
        )

    carriage = model.part("top_carriage")
    carriage.visual(Box((0.100, 0.065, 0.012)), origin=Origin(xyz=(0.000, 0.000, 0.006)), material=CAST_ALLOY, name="saddle_0")
    carriage.visual(Box((0.100, 0.065, 0.012)), origin=Origin(xyz=(0.540, 0.000, 0.006)), material=CAST_ALLOY, name="saddle_1")
    carriage.visual(Box((0.660, 0.080, 0.035)), origin=Origin(xyz=(0.270, 0.000, 0.028)), material=STEEL_GREY, name="carriage_beam")
    for i, x in enumerate((0.000, 0.540)):
        carriage.visual(Box((0.055, 0.200, 0.025)), origin=Origin(xyz=(x, -0.085, 0.045)), material=STEEL_GREY, name=f"hinge_arm_{i}")
        carriage.visual(Box((0.055, 0.025, 0.060)), origin=Origin(xyz=(x, -0.170, 0.065)), material=STEEL_GREY, name=f"hinge_lug_{i}")
    carriage.visual(
        Cylinder(radius=0.015, length=1.100),
        origin=Origin(xyz=(0.270, -0.170, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CAST_ALLOY,
        name="tilt_hinge_pin",
    )
    model.articulation(
        "post_to_carriage",
        ArticulationType.FIXED,
        parent=inner_posts[0],
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_tray_geometry(), "rounded_meal_tray"), material=TRAY_LAMINATE, name="tray_shell")
    tray.visual(Box((1.080, 0.028, 0.018)), origin=Origin(xyz=(0.0, 0.029, 0.006)), material=CAST_ALLOY, name="tray_hinge_leaf")
    tray.visual(Box((0.260, 0.018, 0.010)), origin=Origin(xyz=(0.0, 0.480, -0.004)), material=CAST_ALLOY, name="paddle_bracket")
    tray.visual(Box((0.260, 0.004, 0.026)), origin=Origin(xyz=(0.0, 0.471, -0.012)), material=CAST_ALLOY, name="paddle_fork_0")
    tray.visual(Box((0.260, 0.004, 0.026)), origin=Origin(xyz=(0.0, 0.489, -0.012)), material=CAST_ALLOY, name="paddle_fork_1")
    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.270, -0.170, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.7, lower=0.0, upper=0.55),
    )

    release = model.part("release_paddle")
    release.visual(
        Cylinder(radius=0.007, length=0.240),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=CAST_ALLOY,
        name="paddle_hinge_pin",
    )
    release.visual(Box((0.200, 0.030, 0.016)), origin=Origin(xyz=(0.0, 0.016, -0.011)), material=PADDLE_RED, name="paddle_neck")
    release.visual(Box((0.300, 0.105, 0.012)), origin=Origin(xyz=(0.0, 0.070, -0.022)), material=PADDLE_RED, name="paddle_plate")
    model.articulation(
        "tray_to_release_paddle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=release,
        origin=Origin(xyz=(0.0, 0.480, -0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.50, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    post_0 = object_model.get_part("inner_post_0")
    post_1 = object_model.get_part("inner_post_1")
    carriage = object_model.get_part("top_carriage")
    tray = object_model.get_part("tray")
    brake = object_model.get_part("brake_bar")
    release = object_model.get_part("release_paddle")

    lift_0 = object_model.get_articulation("base_to_inner_post_0")
    lift_1 = object_model.get_articulation("base_to_inner_post_1")
    tray_hinge = object_model.get_articulation("carriage_to_tray")
    brake_joint = object_model.get_articulation("base_to_brake_bar")
    release_joint = object_model.get_articulation("tray_to_release_paddle")

    ctx.check(
        "twin lift posts slide prismatically",
        lift_0.articulation_type == ArticulationType.PRISMATIC
        and lift_1.articulation_type == ArticulationType.PRISMATIC
        and lift_1.mimic is not None
        and lift_1.mimic.joint == "base_to_inner_post_0",
        details=f"lift_0={lift_0.articulation_type}, lift_1={lift_1.articulation_type}, mimic={lift_1.mimic}",
    )
    ctx.check(
        "four casters spin continuously on axles",
        all(
            object_model.get_articulation(f"base_to_caster_{i}").articulation_type == ArticulationType.CONTINUOUS
            and object_model.get_articulation(f"base_to_caster_{i}").axis == (1.0, 0.0, 0.0)
            for i in range(4)
        ),
    )
    ctx.check(
        "tray and controls have user-facing hinges",
        tray_hinge.articulation_type == ArticulationType.REVOLUTE
        and brake_joint.articulation_type == ArticulationType.REVOLUTE
        and release_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"tray={tray_hinge.articulation_type}, brake={brake_joint.articulation_type}, release={release_joint.articulation_type}",
    )

    ctx.expect_contact(post_0, carriage, elem_a="inner_tube", elem_b="saddle_0", contact_tol=0.002, name="first lift post supports carriage")
    ctx.expect_contact(post_1, carriage, elem_a="inner_tube", elem_b="saddle_1", contact_tol=0.002, name="second lift post supports same carriage")

    tray_box = ctx.part_element_world_aabb(tray, elem="tray_shell")
    ctx.check(
        "patient-room scale broad tray",
        tray_box is not None and (tray_box[1][0] - tray_box[0][0]) > 1.15 and (tray_box[1][1] - tray_box[0][1]) > 0.58,
        details=f"tray_box={tray_box}",
    )

    axle_box = ctx.part_element_world_aabb(brake, elem="brake_axle")
    ctx.check(
        "brake bar bridges lower frame rails",
        axle_box is not None and (axle_box[1][0] - axle_box[0][0]) > 0.88,
        details=f"brake axle aabb={axle_box}",
    )

    rest_tray = ctx.part_world_aabb(tray)
    with ctx.pose({lift_0: 0.25}):
        raised_tray = ctx.part_world_aabb(tray)
        ctx.expect_contact(post_1, carriage, elem_a="inner_tube", elem_b="saddle_1", contact_tol=0.002, name="mimicked post remains under raised carriage")
    ctx.check(
        "lift raises the tray assembly",
        rest_tray is not None and raised_tray is not None and raised_tray[0][2] > rest_tray[0][2] + 0.20,
        details=f"rest={rest_tray}, raised={raised_tray}",
    )

    rest_tray_box = ctx.part_element_world_aabb(tray, elem="tray_shell")
    with ctx.pose({tray_hinge: 0.45}):
        tilted_tray_box = ctx.part_element_world_aabb(tray, elem="tray_shell")
    ctx.check(
        "tray tilts upward about transverse hinge",
        rest_tray_box is not None and tilted_tray_box is not None and tilted_tray_box[1][2] > rest_tray_box[1][2] + 0.15,
        details=f"rest={rest_tray_box}, tilted={tilted_tray_box}",
    )

    rest_pad = ctx.part_element_world_aabb(brake, elem="foot_pad")
    with ctx.pose({brake_joint: 0.35}):
        pressed_pad = ctx.part_element_world_aabb(brake, elem="foot_pad")
    ctx.check(
        "foot brake bar rotates on its pivots",
        rest_pad is not None and pressed_pad is not None and pressed_pad[0][2] < rest_pad[0][2] - 0.010,
        details=f"rest={rest_pad}, pressed={pressed_pad}",
    )

    rest_paddle = ctx.part_element_world_aabb(release, elem="paddle_plate")
    with ctx.pose({release_joint: -0.40}):
        pulled_paddle = ctx.part_element_world_aabb(release, elem="paddle_plate")
    ctx.check(
        "release paddle pulls down below the work surface",
        rest_paddle is not None and pulled_paddle is not None and pulled_paddle[0][2] < rest_paddle[0][2] - 0.015,
        details=f"rest={rest_paddle}, pulled={pulled_paddle}",
    )

    return ctx.report()


object_model = build_object_model()
