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
    model = ArticulatedObject(name="service_console_monitor_mount")

    satin_black = Material("satin_black", rgba=(0.015, 0.016, 0.017, 1.0))
    charcoal = Material("charcoal_powdercoat", rgba=(0.10, 0.105, 0.11, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    steel = Material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    plate_gray = Material("plate_gray", rgba=(0.22, 0.23, 0.24, 1.0))

    lower_len = 0.42
    upper_len = 0.36
    boss_radius = 0.064
    boss_thick = 0.035
    arm_bar_width = 0.026
    arm_bar_sep = 0.076
    shoulder_z = 0.4675

    base = model.part("base")
    base.visual(
        Box((0.36, 0.26, 0.040)),
        origin=Origin(xyz=(-0.030, 0.0, 0.025)),
        material=charcoal,
        name="weighted_slab",
    )
    for name, x, y in (
        ("rubber_foot_0", -0.155, -0.105),
        ("rubber_foot_1", -0.155, 0.105),
        ("rubber_foot_2", 0.095, -0.105),
        ("rubber_foot_3", 0.095, 0.105),
    ):
        base.visual(
            Box((0.060, 0.034, 0.008)),
            origin=Origin(xyz=(x, y, 0.004)),
            material=dark_rubber,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.050, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=satin_black,
        name="support_column",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.4425)),
        material=steel,
        name="shoulder_cap",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=boss_radius, length=boss_thick),
        origin=Origin(),
        material=steel,
        name="shoulder_boss",
    )
    lower_arm.visual(
        Cylinder(radius=boss_radius, length=boss_thick),
        origin=Origin(xyz=(lower_len, 0.0, 0.0)),
        material=steel,
        name="elbow_boss",
    )
    for name, y in (("side_bar_0", -arm_bar_sep / 2.0), ("side_bar_1", arm_bar_sep / 2.0)):
        lower_arm.visual(
            Box((lower_len, arm_bar_width, 0.030)),
            origin=Origin(xyz=(lower_len / 2.0, y, 0.0)),
            material=satin_black,
            name=name,
        )
    lower_arm.visual(
        Box((0.035, 0.102, 0.020)),
        origin=Origin(xyz=(lower_len * 0.42, 0.0, 0.0)),
        material=charcoal,
        name="cross_tie",
    )

    upper_arm = model.part("upper_arm")
    upper_arm_z = boss_thick
    upper_arm.visual(
        Cylinder(radius=0.060, length=boss_thick),
        origin=Origin(xyz=(0.0, 0.0, upper_arm_z)),
        material=steel,
        name="elbow_boss",
    )
    upper_arm.visual(
        Cylinder(radius=0.056, length=boss_thick),
        origin=Origin(xyz=(upper_len, 0.0, upper_arm_z)),
        material=steel,
        name="wrist_boss",
    )
    for name, y in (("side_bar_0", -0.032), ("side_bar_1", 0.032)):
        upper_arm.visual(
            Box((upper_len, 0.024, 0.028)),
            origin=Origin(xyz=(upper_len / 2.0, y, upper_arm_z)),
            material=satin_black,
            name=name,
        )
    upper_arm.visual(
        Box((0.032, 0.088, 0.018)),
        origin=Origin(xyz=(upper_len * 0.58, 0.0, upper_arm_z)),
        material=charcoal,
        name="cross_tie",
    )

    yoke = model.part("head_yoke")
    yoke_stack_z = 0.035
    yoke.visual(
        Cylinder(radius=0.050, length=boss_thick),
        origin=Origin(xyz=(0.0, 0.0, yoke_stack_z)),
        material=steel,
        name="swivel_stack",
    )
    yoke.visual(
        Box((0.110, 0.040, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, yoke_stack_z)),
        material=satin_black,
        name="short_neck",
    )
    yoke.visual(
        Box((0.026, 0.210, 0.040)),
        origin=Origin(xyz=(0.105, 0.0, yoke_stack_z)),
        material=satin_black,
        name="rear_bridge",
    )
    for name, y in (("cheek_0", -0.095), ("cheek_1", 0.095)):
        yoke.visual(
            Box((0.056, 0.020, 0.130)),
            origin=Origin(xyz=(0.145, y, yoke_stack_z)),
            material=satin_black,
            name=name,
        )
    for name, y in (("pivot_cap_0", -0.108), ("pivot_cap_1", 0.108)):
        yoke.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.145, y, yoke_stack_z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    plate = model.part("mounting_plate")
    plate.visual(
        Cylinder(radius=0.017, length=0.170),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_barrel",
    )
    plate.visual(
        Box((0.054, 0.062, 0.040)),
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        material=steel,
        name="tilt_web",
    )
    plate.visual(
        Box((0.018, 0.150, 0.120)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=plate_gray,
        name="vesa_plate",
    )
    for name, y, z in (
        ("bolt_pad_0", -0.050, -0.035),
        ("bolt_pad_1", -0.050, 0.035),
        ("bolt_pad_2", 0.050, -0.035),
        ("bolt_pad_3", 0.050, 0.035),
    ):
        plate.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.071, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_black,
            name=name,
        )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(lower_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "upper_to_yoke",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=yoke,
        origin=Origin(xyz=(upper_len, 0.0, upper_arm_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "yoke_to_plate",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=plate,
        origin=Origin(xyz=(0.145, 0.0, yoke_stack_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2, lower=-0.45, upper=0.45),
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

    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    yoke = object_model.get_part("head_yoke")
    plate = object_model.get_part("mounting_plate")
    base = object_model.get_part("base")

    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_yoke = object_model.get_articulation("upper_to_yoke")
    yoke_to_plate = object_model.get_articulation("yoke_to_plate")

    ctx.check(
        "four named revolute mechanisms",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (base_to_lower, lower_to_upper, upper_to_yoke, yoke_to_plate)
        ),
        details="The mount should articulate at lower arm, upper arm, head swivel, and head tilt.",
    )
    ctx.expect_gap(
        lower,
        base,
        axis="z",
        positive_elem="shoulder_boss",
        negative_elem="shoulder_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower arm bears on base cap",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="elbow_boss",
        negative_elem="elbow_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper arm is stacked on elbow bearing",
    )
    ctx.expect_gap(
        yoke,
        upper,
        axis="z",
        positive_elem="swivel_stack",
        negative_elem="wrist_boss",
        max_gap=0.001,
        max_penetration=0.0,
        name="head swivel stack sits on wrist bearing",
    )
    ctx.expect_within(
        plate,
        yoke,
        axes="y",
        inner_elem="tilt_barrel",
        outer_elem="rear_bridge",
        margin=0.026,
        name="tilt barrel is retained inside compact yoke width",
    )

    rest_plate_aabb = ctx.part_world_aabb(plate)
    with ctx.pose({lower_to_upper: 0.65, upper_to_yoke: -0.55}):
        folded_plate_aabb = ctx.part_world_aabb(plate)
    with ctx.pose({yoke_to_plate: 0.42}):
        tilted_plate_aabb = ctx.part_world_aabb(plate)

    ctx.check(
        "upper arm and swivel reposition head in plan",
        rest_plate_aabb is not None
        and folded_plate_aabb is not None
        and abs(
            ((folded_plate_aabb[0][1] + folded_plate_aabb[1][1]) / 2.0)
            - ((rest_plate_aabb[0][1] + rest_plate_aabb[1][1]) / 2.0)
        )
        > 0.12,
        details=f"rest={rest_plate_aabb}, folded={folded_plate_aabb}",
    )
    ctx.check(
        "tilt joint pitches the mounting plate",
        rest_plate_aabb is not None
        and tilted_plate_aabb is not None
        and abs(
            ((tilted_plate_aabb[0][2] + tilted_plate_aabb[1][2]) / 2.0)
            - ((rest_plate_aabb[0][2] + rest_plate_aabb[1][2]) / 2.0)
        )
        > 0.018,
        details=f"rest={rest_plate_aabb}, tilted={tilted_plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
