from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="vertical_mast_side_arm")

    mast_mat = model.material("anodized_dark_mast", color=(0.18, 0.20, 0.22, 1.0))
    rail_mat = model.material("black_guide_rails", color=(0.035, 0.035, 0.04, 1.0))
    carriage_mat = model.material("satin_carriage", color=(0.72, 0.74, 0.73, 1.0))
    upper_mat = model.material("blue_upper_link", color=(0.08, 0.22, 0.55, 1.0))
    forearm_mat = model.material("orange_forearm_link", color=(0.88, 0.36, 0.08, 1.0))
    pin_mat = model.material("dark_pivot_pins", color=(0.02, 0.02, 0.025, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.30, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=mast_mat,
        name="floor_plate",
    )
    mast.visual(
        Box((0.060, 0.060, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=mast_mat,
        name="upright_box",
    )
    mast.visual(
        Box((0.086, 0.012, 1.02)),
        origin=Origin(xyz=(0.0, -0.036, 0.585)),
        material=rail_mat,
        name="front_rail",
    )
    mast.visual(
        Box((0.086, 0.012, 1.02)),
        origin=Origin(xyz=(0.0, 0.036, 0.585)),
        material=rail_mat,
        name="rear_rail",
    )
    mast.visual(
        Box((0.092, 0.092, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 1.155)),
        material=mast_mat,
        name="top_stop",
    )
    mast.visual(
        Box((0.116, 0.116, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=mast_mat,
        name="lower_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.140, 0.024, 0.120)),
        origin=Origin(xyz=(0.0, 0.054, 0.0)),
        material=carriage_mat,
        name="rear_sleeve_plate",
    )
    carriage.visual(
        Box((0.140, 0.024, 0.120)),
        origin=Origin(xyz=(0.0, -0.054, 0.0)),
        material=carriage_mat,
        name="front_sleeve_plate",
    )
    carriage.visual(
        Box((0.024, 0.130, 0.120)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=carriage_mat,
        name="side_sleeve_plate",
    )
    carriage.visual(
        Box((0.024, 0.130, 0.120)),
        origin=Origin(xyz=(-0.058, 0.0, 0.0)),
        material=carriage_mat,
        name="opposite_sleeve_plate",
    )
    carriage.visual(
        Box((0.060, 0.110, 0.070)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=carriage_mat,
        name="root_boss",
    )
    carriage.visual(
        Box((0.090, 0.018, 0.120)),
        origin=Origin(xyz=(0.165, 0.051, 0.0)),
        material=carriage_mat,
        name="root_fork_plate_0",
    )
    carriage.visual(
        Box((0.090, 0.018, 0.120)),
        origin=Origin(xyz=(0.165, -0.051, 0.0)),
        material=carriage_mat,
        name="root_fork_plate_1",
    )
    carriage.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.170, 0.064, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="root_pin_cap_0",
    )
    carriage.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.170, -0.064, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="root_pin_cap_1",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.034, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="shoulder_barrel",
    )
    upper_link.visual(
        Box((0.310, 0.036, 0.034)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=upper_mat,
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.070, 0.095, 0.035)),
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        material=upper_mat,
        name="elbow_bridge",
    )
    upper_link.visual(
        Box((0.100, 0.018, 0.105)),
        origin=Origin(xyz=(0.380, 0.048, 0.0)),
        material=upper_mat,
        name="elbow_fork_plate_0",
    )
    upper_link.visual(
        Box((0.100, 0.018, 0.105)),
        origin=Origin(xyz=(0.380, -0.048, 0.0)),
        material=upper_mat,
        name="elbow_fork_plate_1",
    )
    upper_link.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.380, 0.061, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="elbow_pin_cap_0",
    )
    upper_link.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.380, -0.061, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="elbow_pin_cap_1",
    )

    forearm = model.part("forearm_link")
    forearm.visual(
        Cylinder(radius=0.028, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.270, 0.034, 0.030)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=forearm_mat,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.060, 0.052, 0.042)),
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        material=forearm_mat,
        name="wrist_block",
    )
    forearm.visual(
        Cylinder(radius=0.026, length=0.045),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=pin_mat,
        name="tool_socket",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.62),
    )
    model.articulation(
        "carriage_to_upper",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-0.75, upper=1.25),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.2, lower=-1.55, upper=1.40),
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

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm_link")
    mast_slide = object_model.get_articulation("mast_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper")
    elbow = object_model.get_articulation("upper_to_forearm")

    ctx.allow_overlap(
        carriage,
        upper,
        elem_a="root_fork_plate_0",
        elem_b="shoulder_barrel",
        reason="The shoulder barrel is intentionally captured inside the solid proxy fork plate.",
    )
    ctx.allow_overlap(
        carriage,
        upper,
        elem_a="root_fork_plate_1",
        elem_b="shoulder_barrel",
        reason="The shoulder barrel is intentionally captured inside the opposite solid proxy fork plate.",
    )
    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_fork_plate_0",
        elem_b="elbow_barrel",
        reason="The elbow barrel is intentionally captured inside the solid proxy fork plate.",
    )
    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_fork_plate_1",
        elem_b="elbow_barrel",
        reason="The elbow barrel is intentionally captured inside the opposite solid proxy fork plate.",
    )

    ctx.expect_within(
        mast,
        carriage,
        axes="xy",
        inner_elem="upright_box",
        outer_elem="side_sleeve_plate",
        margin=0.090,
        name="carriage is centered around mast in x",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="side_sleeve_plate",
        elem_b="upright_box",
        min_overlap=0.10,
        name="carriage sleeve engages upright",
    )
    ctx.expect_overlap(
        upper,
        carriage,
        axes="yz",
        elem_a="shoulder_barrel",
        elem_b="root_fork_plate_0",
        min_overlap=0.010,
        name="shoulder barrel sits in root fork",
    )
    ctx.expect_overlap(
        forearm,
        upper,
        axes="yz",
        elem_a="elbow_barrel",
        elem_b="elbow_fork_plate_0",
        min_overlap=0.010,
        name="elbow barrel sits in elbow fork",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({mast_slide: 0.50}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="side_sleeve_plate",
            elem_b="upright_box",
            min_overlap=0.10,
            name="raised carriage remains on upright",
        )
    ctx.check(
        "mast slide raises carriage",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.45,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    closed_upper_aabb = ctx.part_world_aabb(upper)
    with ctx.pose({shoulder: 0.85}):
        lifted_upper_aabb = ctx.part_world_aabb(upper)
    ctx.check(
        "shoulder joint lifts upper link",
        closed_upper_aabb is not None
        and lifted_upper_aabb is not None
        and lifted_upper_aabb[1][2] > closed_upper_aabb[1][2] + 0.12,
        details=f"rest={closed_upper_aabb}, lifted={lifted_upper_aabb}",
    )

    rest_forearm_aabb = ctx.part_world_aabb(forearm)
    with ctx.pose({elbow: 0.90}):
        bent_forearm_aabb = ctx.part_world_aabb(forearm)
    ctx.check(
        "elbow joint bends forearm upward",
        rest_forearm_aabb is not None
        and bent_forearm_aabb is not None
        and bent_forearm_aabb[1][2] > rest_forearm_aabb[1][2] + 0.10,
        details=f"rest={rest_forearm_aabb}, bent={bent_forearm_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
