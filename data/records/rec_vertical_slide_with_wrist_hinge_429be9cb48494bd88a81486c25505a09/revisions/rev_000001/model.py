from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_vertical_axis_tilt_head")

    wall_mat = model.material("painted_wall", rgba=(0.78, 0.79, 0.76, 1.0))
    plate_mat = model.material("black_powdercoat", rgba=(0.06, 0.065, 0.07, 1.0))
    rail_mat = model.material("brushed_steel", rgba=(0.48, 0.50, 0.50, 1.0))
    shoe_mat = model.material("dark_slider_blocks", rgba=(0.03, 0.035, 0.04, 1.0))
    carriage_mat = model.material("blue_carriage", rgba=(0.05, 0.20, 0.38, 1.0))
    tab_mat = model.material("orange_output_tab", rgba=(0.95, 0.42, 0.08, 1.0))
    pin_mat = model.material("dark_pin_caps", rgba=(0.015, 0.015, 0.018, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.46, 0.018, 0.92)),
        origin=Origin(xyz=(0.0, 0.034, 0.46)),
        material=wall_mat,
        name="wall_patch",
    )
    backplate.visual(
        Box((0.30, 0.025, 0.78)),
        origin=Origin(xyz=(0.0, 0.0125, 0.40)),
        material=plate_mat,
        name="mounting_plate",
    )
    # Two proud vertical rails form the fixed guide on the front of the plate.
    backplate.visual(
        Box((0.030, 0.030, 0.62)),
        origin=Origin(xyz=(-0.085, -0.015, 0.40)),
        material=rail_mat,
        name="guide_rail_0",
    )
    backplate.visual(
        Box((0.030, 0.030, 0.62)),
        origin=Origin(xyz=(0.085, -0.015, 0.40)),
        material=rail_mat,
        name="guide_rail_1",
    )
    for i, x in enumerate((-0.085, 0.085)):
        backplate.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.033, 0.10), rpy=(math.pi / 2, 0.0, 0.0)),
            material=pin_mat,
            name=f"lower_rail_screw_{i}",
        )
        backplate.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.033, 0.70), rpy=(math.pi / 2, 0.0, 0.0)),
            material=pin_mat,
            name=f"upper_rail_screw_{i}",
        )
    backplate.visual(
        Box((0.235, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, -0.017, 0.085)),
        material=rail_mat,
        name="lower_stop",
    )
    backplate.visual(
        Box((0.235, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, -0.017, 0.715)),
        material=rail_mat,
        name="upper_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.170, 0.045, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_mat,
        name="front_block",
    )
    carriage.visual(
        Box((0.048, 0.028, 0.145)),
        origin=Origin(xyz=(-0.085, 0.022, 0.0)),
        material=shoe_mat,
        name="bearing_0",
    )
    carriage.visual(
        Box((0.048, 0.028, 0.145)),
        origin=Origin(xyz=(0.085, 0.022, 0.0)),
        material=shoe_mat,
        name="bearing_1",
    )
    carriage.visual(
        Box((0.118, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.031, 0.015)),
        material=carriage_mat,
        name="hinge_boss",
    )
    for i, x in enumerate((-0.045, 0.045)):
        carriage.visual(
            Box((0.026, 0.034, 0.034)),
            origin=Origin(xyz=(x, -0.041, 0.008)),
            material=carriage_mat,
            name=f"yoke_cheek_{i}",
        )
        carriage.visual(
            Cylinder(radius=0.012, length=0.026),
            origin=Origin(xyz=(x, -0.050, 0.020), rpy=(0.0, math.pi / 2, 0.0)),
            material=rail_mat,
            name=f"hinge_barrel_{i}",
        )
        carriage.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x + (0.015 if x > 0 else -0.015), -0.050, 0.020), rpy=(0.0, math.pi / 2, 0.0)),
            material=pin_mat,
            name=f"pin_cap_{i}",
        )

    output_tab = model.part("output_tab")
    output_tab.visual(
        Cylinder(radius=0.010, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=rail_mat,
        name="hinge_knuckle",
    )
    output_tab.visual(
        Box((0.050, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, -0.073, -0.008)),
        material=tab_mat,
        name="output_blade",
    )
    output_tab.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, -0.138, -0.008)),
        material=tab_mat,
        name="rounded_tip",
    )
    output_tab.visual(
        Cylinder(radius=0.007, length=0.015),
        origin=Origin(xyz=(0.0, -0.138, -0.015), rpy=(0.0, 0.0, 0.0)),
        material=pin_mat,
        name="output_hole",
    )

    model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.066, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.320),
    )
    model.articulation(
        "tab_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_tab,
        origin=Origin(xyz=(0.0, -0.050, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.65, upper=0.85),
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

    backplate = object_model.get_part("backplate")
    carriage = object_model.get_part("carriage")
    output_tab = object_model.get_part("output_tab")
    slide = object_model.get_articulation("slide")
    tab_hinge = object_model.get_articulation("tab_hinge")

    ctx.expect_within(
        carriage,
        backplate,
        axes="x",
        margin=0.002,
        name="carriage stays centered between guide rails",
    )
    ctx.expect_gap(
        backplate,
        carriage,
        axis="y",
        positive_elem="guide_rail_0",
        negative_elem="bearing_0",
        min_gap=0.0,
        max_gap=0.0005,
        name="left bearing contacts the guide rail",
    )
    ctx.expect_gap(
        backplate,
        carriage,
        axis="y",
        positive_elem="guide_rail_1",
        negative_elem="bearing_1",
        min_gap=0.0,
        max_gap=0.0005,
        name="right bearing contacts the guide rail",
    )
    ctx.expect_overlap(
        carriage,
        backplate,
        axes="z",
        elem_a="bearing_0",
        elem_b="guide_rail_0",
        min_overlap=0.12,
        name="collapsed carriage is engaged on guide",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.320}):
        ctx.expect_overlap(
            carriage,
            backplate,
            axes="z",
            elem_a="bearing_0",
            elem_b="guide_rail_0",
            min_overlap=0.12,
            name="raised carriage remains on guide",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "slide moves carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    blade_rest = ctx.part_element_world_aabb(output_tab, elem="output_blade")
    with ctx.pose({tab_hinge: 0.65}):
        blade_tilted = ctx.part_element_world_aabb(output_tab, elem="output_blade")

    ctx.check(
        "tab hinge lifts the output blade",
        blade_rest is not None
        and blade_tilted is not None
        and blade_tilted[1][2] > blade_rest[1][2] + 0.035,
        details=f"rest_aabb={blade_rest}, tilted_aabb={blade_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
