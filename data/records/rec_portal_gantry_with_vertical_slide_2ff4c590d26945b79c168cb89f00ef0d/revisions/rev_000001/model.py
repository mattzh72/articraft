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


def _crosshead_carriage_mesh():
    """A compact bearing block with two through-bores for the fixed X guide rods."""
    carriage = cq.Workplane("XY").box(0.220, 0.050, 0.160).translate((0.0, -0.081, 0.0))
    for z in (-0.040, 0.040):
        cutter = cq.Workplane("YZ").center(-0.081, z).cylinder(0.300, 0.017)
        carriage = carriage.cut(cutter)

    front_plate = cq.Workplane("XY").box(0.180, 0.035, 0.280).translate((0.0, -0.121, -0.120))
    carriage = carriage.union(front_plate)
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_portal_axis")

    model.material("painted_steel", color=(0.18, 0.21, 0.24, 1.0))
    model.material("dark_table", color=(0.10, 0.11, 0.12, 1.0))
    model.material("brushed_metal", color=(0.68, 0.70, 0.70, 1.0))
    model.material("black_hardware", color=(0.02, 0.02, 0.018, 1.0))
    model.material("crosshead_blue", color=(0.05, 0.18, 0.42, 1.0))
    model.material("quill_yellow", color=(0.86, 0.58, 0.12, 1.0))

    frame = model.part("portal_frame")
    frame.visual(
        Box((1.25, 0.55, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="dark_table",
        name="base_plate",
    )
    frame.visual(
        Box((0.95, 0.36, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material="black_hardware",
        name="tabletop",
    )
    for x, name in ((-0.56, "column_0"), (0.56, "column_1")):
        frame.visual(
            Box((0.095, 0.105, 0.700)),
            origin=Origin(xyz=(x, 0.0, 0.400)),
            material="painted_steel",
            name=name,
        )
    frame.visual(
        Box((1.16, 0.105, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        material="painted_steel",
        name="top_beam",
    )
    frame.visual(
        Box((1.04, 0.0035, 0.090)),
        origin=Origin(xyz=(0.0, -0.05425, 0.740)),
        material="brushed_metal",
        name="front_way",
    )
    frame.visual(
        Box((1.08, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.030, 0.740)),
        material="painted_steel",
        name="guide_backer",
    )
    for z, rod_name in ((0.700, "lower_guide_rod"), (0.780, "upper_guide_rod")):
        frame.visual(
            Cylinder(radius=0.012, length=1.04),
            origin=Origin(xyz=(0.0, -0.081, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_metal",
            name=rod_name,
        )
        for x, clamp_name in ((-0.50, f"{rod_name}_clamp_0"), (0.50, f"{rod_name}_clamp_1")):
            frame.visual(
                Box((0.040, 0.042, 0.040)),
                origin=Origin(xyz=(x, -0.067, z)),
                material="painted_steel",
                name=clamp_name,
            )
    for x, name in ((-0.22, "slot_0"), (0.0, "slot_1"), (0.22, "slot_2")):
        frame.visual(
            Box((0.045, 0.340, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.083)),
            material="brushed_metal",
            name=name,
        )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_crosshead_carriage_mesh(), "crosshead_carriage", tolerance=0.0008),
        material="crosshead_blue",
        name="bearing_carriage",
    )
    for x, name in ((-0.055, "vertical_rail_0"), (0.055, "vertical_rail_1")):
        crosshead.visual(
            Box((0.012, 0.012, 0.250)),
            origin=Origin(xyz=(x, -0.142, -0.120)),
            material="brushed_metal",
            name=name,
        )
    crosshead.visual(
        Box((0.145, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.148, 0.030)),
        material="black_hardware",
        name="top_stop",
    )

    quill = model.part("quill_plate")
    quill.visual(
        Box((0.112, 0.025, 0.230)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material="quill_yellow",
        name="quill_slide",
    )
    for x, name in ((-0.055, "slide_shoe_0"), (0.055, "slide_shoe_1")):
        quill.visual(
            Box((0.030, 0.012, 0.165)),
            origin=Origin(xyz=(x, 0.006, -0.095)),
            material="black_hardware",
            name=name,
        )
    quill.visual(
        Cylinder(radius=0.027, length=0.052),
        origin=Origin(xyz=(0.0, -0.026, -0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="spindle_boss",
    )
    quill.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.0, -0.040, -0.270)),
        material="brushed_metal",
        name="tool_stub",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.35, lower=-0.300, upper=0.300),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=quill,
        origin=Origin(xyz=(0.0, -0.1605, -0.070)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.220),
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

    frame = object_model.get_part("portal_frame")
    crosshead = object_model.get_part("crosshead")
    quill = object_model.get_part("quill_plate")
    x_slide = object_model.get_articulation("x_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.check(
        "two orthogonal prismatic slides",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        details=f"x={x_slide.articulation_type} {x_slide.axis}, z={z_slide.articulation_type} {z_slide.axis}",
    )
    ctx.expect_within(
        frame,
        crosshead,
        axes="yz",
        inner_elem="upper_guide_rod",
        outer_elem="bearing_carriage",
        margin=0.010,
        name="upper rod passes through carriage bore envelope",
    )
    ctx.expect_within(
        frame,
        crosshead,
        axes="yz",
        inner_elem="lower_guide_rod",
        outer_elem="bearing_carriage",
        margin=0.010,
        name="lower rod passes through carriage bore envelope",
    )
    ctx.expect_gap(
        crosshead,
        quill,
        axis="y",
        positive_elem="vertical_rail_0",
        negative_elem="quill_slide",
        min_gap=0.0,
        max_gap=0.001,
        name="quill plate bears on the fixed vertical rail",
    )
    ctx.expect_overlap(
        quill,
        crosshead,
        axes="z",
        elem_a="quill_slide",
        elem_b="vertical_rail_0",
        min_overlap=0.140,
        name="quill slide remains engaged with vertical guide",
    )

    rest_x = ctx.part_world_position(crosshead)
    with ctx.pose({x_slide: 0.250}):
        moved_x = ctx.part_world_position(crosshead)
        ctx.expect_within(
            frame,
            crosshead,
            axes="yz",
            inner_elem="upper_guide_rod",
            outer_elem="bearing_carriage",
            margin=0.010,
            name="crosshead stays on the horizontal rods while traversing",
        )
    ctx.check(
        "crosshead traverses along beam",
        rest_x is not None and moved_x is not None and moved_x[0] > rest_x[0] + 0.20,
        details=f"rest={rest_x}, moved={moved_x}",
    )

    rest_z = ctx.part_world_position(quill)
    with ctx.pose({z_slide: 0.180}):
        moved_z = ctx.part_world_position(quill)
        ctx.expect_gap(
            quill,
            frame,
            axis="z",
            positive_elem="tool_stub",
            negative_elem="tabletop",
            min_gap=0.040,
            name="lowered quill remains above tabletop",
        )
    ctx.check(
        "quill feeds downward",
        rest_z is not None and moved_z is not None and moved_z[2] < rest_z[2] - 0.15,
        details=f"rest={rest_z}, moved={moved_z}",
    )

    return ctx.report()


object_model = build_object_model()
