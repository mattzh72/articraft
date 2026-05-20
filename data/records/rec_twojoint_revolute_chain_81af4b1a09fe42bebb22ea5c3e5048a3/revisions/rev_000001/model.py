from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_two_joint_chain")

    plate_mat = Material("powder_coated_backplate", color=(0.10, 0.11, 0.12, 1.0))
    bracket_mat = Material("blackened_steel_clevis", color=(0.16, 0.16, 0.17, 1.0))
    link_mat = Material("painted_blue_links", color=(0.05, 0.24, 0.62, 1.0))
    pin_mat = Material("brushed_pin_caps", color=(0.72, 0.70, 0.64, 1.0))
    bumper_mat = Material("dark_end_bumper", color=(0.03, 0.03, 0.03, 1.0))

    root_x = 0.085
    root_z = 0.250
    elbow_x = 0.310

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.024, 0.340, 0.480)),
        origin=Origin(xyz=(-0.012, 0.0, 0.240)),
        material=plate_mat,
        name="wall_plate",
    )
    # A short welded stand-off keeps the root pivot proud of the wall while
    # leaving visible air around the moving root knuckle.
    backplate.visual(
        Box((0.052, 0.108, 0.092)),
        origin=Origin(xyz=(0.026, 0.0, root_z)),
        material=bracket_mat,
        name="root_standoff",
    )
    for z_offset, name in ((0.038, "root_upper_ear"), (-0.038, "root_lower_ear")):
        backplate.visual(
            Box((0.084, 0.118, 0.012)),
            origin=Origin(xyz=(root_x, 0.0, root_z + z_offset)),
            material=bracket_mat,
            name=name,
        )
    backplate.visual(
        Cylinder(radius=0.010, length=0.118),
        origin=Origin(xyz=(root_x, 0.0, root_z)),
        material=pin_mat,
        name="root_pin",
    )
    for z_offset, name in ((0.050, "root_upper_pin_cap"), (-0.050, "root_lower_pin_cap")):
        backplate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(root_x, 0.0, root_z + z_offset)),
            material=pin_mat,
            name=name,
        )
    for y in (-0.118, 0.118):
        for z in (0.080, 0.400):
            backplate.visual(
                Cylinder(radius=0.015, length=0.006),
                origin=Origin(xyz=(0.003, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pin_mat,
                name=f"wall_screw_{len(backplate.visuals)}",
            )

    link_0 = model.part("link_0")
    link_0.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(),
        material=link_mat,
        name="root_knuckle",
    )
    link_0.visual(
        Box((0.242, 0.046, 0.030)),
        origin=Origin(xyz=(0.146, 0.0, 0.0)),
        material=link_mat,
        name="main_bar",
    )
    # The distal end is a real clevis: two separated ears and a rear bridge.
    link_0.visual(
        Box((0.032, 0.058, 0.080)),
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        material=link_mat,
        name="elbow_bridge",
    )
    for z_offset, name in ((0.038, "elbow_upper_ear"), (-0.038, "elbow_lower_ear")):
        link_0.visual(
            Box((0.090, 0.092, 0.012)),
            origin=Origin(xyz=(elbow_x, 0.0, z_offset)),
            material=link_mat,
            name=name,
        )
    link_0.visual(
        Cylinder(radius=0.009, length=0.118),
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        material=pin_mat,
        name="elbow_pin",
    )
    for z_offset, name in ((0.050, "elbow_upper_pin_cap"), (-0.050, "elbow_lower_pin_cap")):
        link_0.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(elbow_x, 0.0, z_offset)),
            material=pin_mat,
            name=name,
        )

    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(),
        material=link_mat,
        name="elbow_knuckle",
    )
    link_1.visual(
        Box((0.246, 0.042, 0.028)),
        origin=Origin(xyz=(0.146, 0.0, 0.0)),
        material=link_mat,
        name="outer_bar",
    )
    link_1.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.292, 0.0, 0.0)),
        material=link_mat,
        name="end_eye",
    )
    link_1.visual(
        Box((0.026, 0.062, 0.032)),
        origin=Origin(xyz=(0.326, 0.0, 0.0)),
        material=bumper_mat,
        name="end_bumper",
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=link_0,
        origin=Origin(xyz=(root_x, 0.0, root_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-1.35, upper=1.35),
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
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    root_pivot = object_model.get_articulation("root_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")

    ctx.allow_overlap(
        backplate,
        link_0,
        elem_a="root_pin",
        elem_b="root_knuckle",
        reason="The fixed root pin is intentionally captured through the rotating root knuckle.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="elbow_pin",
        elem_b="elbow_knuckle",
        reason="The elbow pin is intentionally captured through the second link knuckle.",
    )

    ctx.expect_overlap(
        link_0,
        backplate,
        axes="xy",
        elem_a="root_knuckle",
        elem_b="root_upper_ear",
        min_overlap=0.030,
        name="root knuckle is centered in clevis plan",
    )
    ctx.expect_overlap(
        backplate,
        link_0,
        axes="z",
        elem_a="root_pin",
        elem_b="root_knuckle",
        min_overlap=0.035,
        name="root pin passes through knuckle",
    )
    ctx.expect_gap(
        backplate,
        link_0,
        axis="z",
        positive_elem="root_upper_ear",
        negative_elem="root_knuckle",
        min_gap=0.006,
        max_gap=0.020,
        name="root upper ear has visible clearance",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xy",
        elem_a="elbow_knuckle",
        elem_b="elbow_upper_ear",
        min_overlap=0.028,
        name="elbow knuckle is centered in clevis plan",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="z",
        elem_a="elbow_pin",
        elem_b="elbow_knuckle",
        min_overlap=0.035,
        name="elbow pin passes through knuckle",
    )
    ctx.expect_gap(
        link_0,
        link_1,
        axis="z",
        positive_elem="elbow_upper_ear",
        negative_elem="elbow_knuckle",
        min_gap=0.006,
        max_gap=0.020,
        name="elbow upper ear has visible clearance",
    )

    rest_outer = ctx.part_world_position(link_1)
    with ctx.pose({root_pivot: 0.55, elbow_pivot: -0.70}):
        moved_outer = ctx.part_world_position(link_1)
        ctx.expect_origin_distance(link_0, link_1, axes="xy", min_dist=0.25, max_dist=0.36)

    ctx.check(
        "two serial pivots move the outer link in plane",
        rest_outer is not None
        and moved_outer is not None
        and abs(moved_outer[2] - rest_outer[2]) < 0.002
        and abs(moved_outer[1] - rest_outer[1]) > 0.06,
        details=f"rest={rest_outer}, moved={moved_outer}",
    )

    return ctx.report()


object_model = build_object_model()
