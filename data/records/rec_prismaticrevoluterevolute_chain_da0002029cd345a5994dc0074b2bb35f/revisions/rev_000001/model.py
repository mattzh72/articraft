from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="slide_shoulder_elbow_chain")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_rail = model.material("brushed_rail", rgba=(0.58, 0.62, 0.66, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.05, 0.18, 0.38, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    black_bushing = model.material("black_bushing", rgba=(0.015, 0.015, 0.014, 1.0))
    stop_red = model.material("stop_red", rgba=(0.70, 0.04, 0.03, 1.0))

    y_axis_cylinder = Origin(rpy=(-pi / 2.0, 0.0, 0.0))

    base = model.part("base_slide")
    base.visual(
        Box((1.25, 0.26, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((1.10, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, -0.070, 0.079)),
        material=brushed_rail,
        name="guide_rail_0",
    )
    base.visual(
        Box((1.10, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, 0.070, 0.079)),
        material=brushed_rail,
        name="guide_rail_1",
    )
    for index, x in enumerate((-0.585, 0.585)):
        base.visual(
            Box((0.050, 0.240, 0.110)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=stop_red,
            name=f"end_stop_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.260, 0.240, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=carriage_blue,
        name="carriage_plate",
    )
    for index, y in enumerate((-0.070, 0.070)):
        carriage.visual(
            Box((0.200, 0.035, 0.006)),
            origin=Origin(xyz=(0.0, y, -0.003)),
            material=black_bushing,
            name=f"slide_shoe_{index}",
        )
    for index, y in enumerate((-0.110, 0.110)):
        carriage.visual(
            Box((0.220, 0.025, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.020)),
            material=carriage_blue,
            name=f"side_clamp_{index}",
        )
    for index, y in enumerate((-0.085, 0.085)):
        carriage.visual(
            Box((0.060, 0.025, 0.220)),
            origin=Origin(xyz=(0.0, y, 0.160)),
            material=carriage_blue,
            name=f"shoulder_upright_{index}",
        )
    carriage.visual(
        Cylinder(radius=0.055, length=0.025),
        origin=Origin(xyz=(0.0, -0.085, 0.275), rpy=y_axis_cylinder.rpy),
        material=black_bushing,
        name="shoulder_bearing_0",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.025),
        origin=Origin(xyz=(0.0, 0.085, 0.275), rpy=y_axis_cylinder.rpy),
        material=black_bushing,
        name="shoulder_bearing_1",
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=0.052, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis_cylinder.rpy),
        material=black_bushing,
        name="shoulder_hub",
    )
    shoulder.visual(
        Box((0.400, 0.055, 0.050)),
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        material=arm_orange,
        name="upper_beam",
    )
    shoulder.visual(
        Box((0.055, 0.120, 0.050)),
        origin=Origin(xyz=(0.425, 0.0, 0.0)),
        material=arm_orange,
        name="elbow_bridge",
    )
    for index, y in enumerate((-0.052, 0.052)):
        shoulder.visual(
            Box((0.120, 0.018, 0.050)),
            origin=Origin(xyz=(0.490, y, 0.0)),
            material=arm_orange,
            name=f"elbow_cheek_{index}",
        )
    shoulder.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.500, -0.052, 0.0), rpy=y_axis_cylinder.rpy),
        material=black_bushing,
        name="elbow_bearing_0",
    )
    shoulder.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.500, 0.052, 0.0), rpy=y_axis_cylinder.rpy),
        material=black_bushing,
        name="elbow_bearing_1",
    )

    forelink = model.part("forelink")
    forelink.visual(
        Cylinder(radius=0.040, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis_cylinder.rpy),
        material=black_bushing,
        name="elbow_hub",
    )
    forelink.visual(
        Box((0.380, 0.045, 0.045)),
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        material=arm_orange,
        name="fore_beam",
    )
    forelink.visual(
        Box((0.050, 0.075, 0.075)),
        origin=Origin(xyz=(0.445, 0.0, 0.0)),
        material=black_bushing,
        name="wrist_mount",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.320, 0.0, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=0.0, upper=0.520),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-0.70, upper=1.40),
    )
    model.articulation(
        "shoulder_to_forelink",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forelink,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.0, lower=-1.45, upper=1.65),
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

    base = object_model.get_part("base_slide")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    slide = object_model.get_articulation("base_to_carriage")
    shoulder_joint = object_model.get_articulation("carriage_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forelink")

    ctx.check(
        "joint chain is prismatic shoulder elbow",
        slide.articulation_type == ArticulationType.PRISMATIC
        and shoulder_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and slide.parent == "base_slide"
        and slide.child == "carriage"
        and shoulder_joint.parent == "carriage"
        and shoulder_joint.child == "shoulder_link"
        and elbow_joint.parent == "shoulder_link"
        and elbow_joint.child == "forelink",
        details=(
            f"got {slide.parent}->{slide.child}, "
            f"{shoulder_joint.parent}->{shoulder_joint.child}, "
            f"{elbow_joint.parent}->{elbow_joint.child}"
        ),
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        min_gap=0.004,
        max_gap=0.010,
        positive_elem="carriage_plate",
        negative_elem="guide_rail_0",
        name="carriage rides just above the slide rail",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        margin=0.020,
        inner_elem="carriage_plate",
        outer_elem="base_plate",
        name="carriage footprint stays over base width",
    )
    ctx.expect_overlap(
        shoulder,
        carriage,
        axes="xz",
        min_overlap=0.080,
        elem_a="shoulder_hub",
        elem_b="shoulder_bearing_0",
        name="shoulder hub aligns with carriage bearings",
    )
    ctx.expect_contact(
        shoulder,
        carriage,
        contact_tol=0.001,
        elem_a="shoulder_hub",
        elem_b="shoulder_bearing_0",
        name="shoulder hub touches bearing face",
    )
    ctx.expect_overlap(
        forelink,
        shoulder,
        axes="xz",
        min_overlap=0.070,
        elem_a="elbow_hub",
        elem_b="elbow_bearing_0",
        name="elbow hub aligns with shoulder fork",
    )
    ctx.expect_contact(
        forelink,
        shoulder,
        contact_tol=0.001,
        elem_a="elbow_hub",
        elem_b="elbow_bearing_0",
        name="elbow hub touches fork bearing",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.45}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            min_gap=0.004,
            max_gap=0.010,
            positive_elem="carriage_plate",
            negative_elem="guide_rail_0",
            name="extended carriage keeps slide clearance",
        )
    ctx.check(
        "slide extends carriage along base rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    wrist_rest = ctx.part_world_position(forelink)
    with ctx.pose({shoulder_joint: 0.90, elbow_joint: 0.85}):
        wrist_raised = ctx.part_world_position(forelink)
    ctx.check(
        "revolute joints lift the arm chain",
        wrist_rest is not None and wrist_raised is not None and wrist_raised[2] > wrist_rest[2] + 0.05,
        details=f"rest={wrist_rest}, raised={wrist_raised}",
    )

    return ctx.report()


object_model = build_object_model()
