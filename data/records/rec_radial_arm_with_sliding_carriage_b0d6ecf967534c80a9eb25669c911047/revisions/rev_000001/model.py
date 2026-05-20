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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_collar_mesh(name: str):
    """Hollow bearing sleeve for the swinging arm, authored with a real bore."""
    collar = (
        cq.Workplane("XY")
        .circle(0.085)
        .circle(0.052)
        .extrude(0.070, both=True)
    )
    return mesh_from_cadquery(collar, name, tolerance=0.0008, angular_tolerance=0.08)


def _carriage_sleeve_mesh(name: str):
    """Closed sliding saddle with a dropped tool head plate and real bore holes."""
    sleeve_length = 0.160
    outer_y = 0.150
    wall = 0.018
    side_height = 0.126

    top = cq.Workplane("XY").box(sleeve_length, outer_y, wall).translate((0.0, 0.0, 0.069))
    bottom = cq.Workplane("XY").box(sleeve_length, outer_y, wall).translate((0.0, 0.0, -0.054))
    side_a = (
        cq.Workplane("XY")
        .box(sleeve_length, wall, side_height)
        .translate((0.0, outer_y / 2.0 - wall / 2.0, 0.0))
    )
    side_b = (
        cq.Workplane("XY")
        .box(sleeve_length, wall, side_height)
        .translate((0.0, -outer_y / 2.0 + wall / 2.0, 0.0))
    )

    head_plate = cq.Workplane("XY").box(0.018, 0.170, 0.170).translate((0.088, 0.0, -0.145))
    tool_boss = cq.Workplane("YZ").circle(0.044).extrude(0.026, both=True).translate((0.101, 0.0, -0.145))
    web = cq.Workplane("XY").box(0.045, 0.110, 0.030).translate((0.070, 0.0, -0.071))

    body = top.union(bottom).union(side_a).union(side_b).union(web).union(head_plate).union(tool_boss)
    center_bore = cq.Workplane("YZ").circle(0.018).extrude(0.080, both=True).translate((0.101, 0.0, -0.145))
    bolt_holes = None
    for y in (-0.055, 0.055):
        for z in (-0.190, -0.100):
            hole = cq.Workplane("YZ").circle(0.006).extrude(0.080, both=True).translate((0.101, y, z))
            bolt_holes = hole if bolt_holes is None else bolt_holes.union(hole)

    body = body.cut(center_bore)
    if bolt_holes is not None:
        body = body.cut(bolt_holes)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_positioning_arm")

    steel = Material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = Material("dark_cast_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    painted_blue = Material("blue_painted_steel", rgba=(0.05, 0.20, 0.55, 1.0))
    orange = Material("orange_carriage", rgba=(0.92, 0.42, 0.08, 1.0))
    black = Material("blackened_hardware", rgba=(0.01, 0.01, 0.012, 1.0))
    bronze = Material("oilite_bronze", rgba=(0.70, 0.48, 0.19, 1.0))

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.240, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="base_plate",
    )
    column.visual(
        Cylinder(radius=0.092, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=dark,
        name="pedestal",
    )
    column.visual(
        Cylinder(radius=0.050, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.075, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.648)),
        material=dark,
        name="lower_retainer",
    )
    column.visual(
        Cylinder(radius=0.075, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.832)),
        material=dark,
        name="upper_retainer",
    )
    for idx, (x, y) in enumerate(((0.155, 0.155), (-0.155, 0.155), (-0.155, -0.155), (0.155, -0.155))):
        column.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.040)),
            material=black,
            name=f"anchor_bolt_{idx}",
        )

    arm = model.part("arm")
    arm.visual(
        _annular_collar_mesh("swing_collar"),
        origin=Origin(),
        material=painted_blue,
        name="swing_collar",
    )
    arm.visual(
        Box((0.006, 0.026, 0.090)),
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
        material=steel,
        name="bearing_pad_0",
    )
    arm.visual(
        Box((0.006, 0.026, 0.090)),
        origin=Origin(xyz=(-0.053, 0.0, 0.0)),
        material=steel,
        name="bearing_pad_1",
    )
    arm.visual(
        Box((0.026, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, 0.053, 0.0)),
        material=steel,
        name="bearing_pad_2",
    )
    arm.visual(
        Box((0.026, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, -0.053, 0.0)),
        material=steel,
        name="bearing_pad_3",
    )
    arm.visual(
        Box((0.910, 0.080, 0.070)),
        origin=Origin(xyz=(0.510, 0.0, 0.0)),
        material=painted_blue,
        name="arm_beam",
    )
    arm.visual(
        Box((0.790, 0.012, 0.014)),
        origin=Origin(xyz=(0.545, 0.046, 0.039)),
        material=steel,
        name="upper_rail_0",
    )
    arm.visual(
        Box((0.790, 0.012, 0.014)),
        origin=Origin(xyz=(0.545, -0.046, 0.039)),
        material=steel,
        name="upper_rail_1",
    )
    arm.visual(
        Box((0.760, 0.018, 0.006)),
        origin=Origin(xyz=(0.555, 0.0, 0.036)),
        material=black,
        name="center_scale_slot",
    )
    arm.visual(
        Box((0.032, 0.112, 0.105)),
        origin=Origin(xyz=(0.978, 0.0, 0.0)),
        material=dark,
        name="end_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        _carriage_sleeve_mesh("slide_sleeve"),
        origin=Origin(),
        material=orange,
        name="slide_sleeve",
    )
    carriage.visual(
        Box((0.120, 0.018, 0.030)),
        origin=Origin(xyz=(0.000, 0.049, 0.000)),
        material=bronze,
        name="slide_pad_0",
    )
    carriage.visual(
        Box((0.120, 0.018, 0.030)),
        origin=Origin(xyz=(0.000, -0.049, 0.000)),
        material=bronze,
        name="slide_pad_1",
    )
    carriage.visual(
        Cylinder(radius=0.023, length=0.008),
        origin=Origin(xyz=(0.116, 0.0, -0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="tool_bore_shadow",
    )
    for idx, (y, z) in enumerate(((-0.055, -0.190), (0.055, -0.190), (-0.055, -0.100), (0.055, -0.100))):
        carriage.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.101, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"plate_bolt_{idx}",
        )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.600),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    column = object_model.get_part("column")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    swing = object_model.get_articulation("column_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.expect_overlap(
        arm,
        column,
        axes="z",
        elem_a="swing_collar",
        elem_b="column_shaft",
        min_overlap=0.120,
        name="swing collar captures column height",
    )
    ctx.expect_within(
        column,
        arm,
        axes="xy",
        inner_elem="column_shaft",
        outer_elem="swing_collar",
        margin=0.001,
        name="column shaft passes through collar footprint",
    )
    ctx.expect_within(
        arm,
        carriage,
        axes="yz",
        inner_elem="arm_beam",
        outer_elem="slide_sleeve",
        margin=0.0,
        name="carriage sleeve surrounds beam section",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="x",
        elem_a="slide_sleeve",
        elem_b="arm_beam",
        min_overlap=0.140,
        name="rest carriage remains on arm",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.600}):
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="slide_sleeve",
            elem_b="arm_beam",
            min_overlap=0.140,
            name="extended carriage remains on arm",
        )
        extended_position = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates outward along arm",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.55,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    with ctx.pose({swing: math.pi / 2.0}):
        swung_position = ctx.part_world_position(carriage)
    ctx.check(
        "arm swing carries carriage around column",
        rest_position is not None
        and swung_position is not None
        and swung_position[1] > rest_position[0] - 0.02
        and abs(swung_position[0]) < 0.03,
        details=f"rest={rest_position}, swung={swung_position}",
    )

    return ctx.report()


object_model = build_object_model()
