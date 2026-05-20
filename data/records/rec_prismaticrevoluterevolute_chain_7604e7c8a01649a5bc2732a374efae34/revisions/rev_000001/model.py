from __future__ import annotations

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
    model = ArticulatedObject(name="bench_positioning_arm")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.55, 0.56, 0.54, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.21, 0.45, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.03, 0.035, 0.04, 1.0))
    fastener_black = model.material("fastener_black", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber = model.material("rubber_pad", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((1.30, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_aluminum,
        name="base_plate",
    )
    for y, name in ((0.070, "rail_mount_0"), (-0.070, "rail_mount_1")):
        base.visual(
            Box((1.15, 0.046, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.045)),
            material=cast_aluminum,
            name=name,
        )
    for y, name in ((0.070, "guide_rail_0"), (-0.070, "guide_rail_1")):
        base.visual(
            Box((1.10, 0.024, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.0675)),
            material=ground_steel,
            name=name,
        )
    for x, name in ((-0.600, "stop_0"), (0.600, "stop_1")):
        base.visual(
            Box((0.045, 0.200, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.0725)),
            material=cast_aluminum,
            name=name,
        )
        base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, 0.070, 0.114)),
            material=fastener_black,
            name=f"{name}_bumper_0",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, -0.070, 0.114)),
            material=fastener_black,
            name=f"{name}_bumper_1",
        )
    for i, x in enumerate((-0.45, -0.18, 0.18, 0.45)):
        for j, y in enumerate((0.070, -0.070)):
            base.visual(
                Cylinder(radius=0.0075, length=0.006),
                origin=Origin(xyz=(x, y, 0.079)),
                material=fastener_black,
                name=f"rail_screw_{i}_{j}",
            )

    carriage = model.part("carriage")
    for y, name in ((0.070, "bearing_shoe_0"), (-0.070, "bearing_shoe_1")):
        carriage.visual(
            Box((0.380, 0.034, 0.024)),
            origin=Origin(xyz=(0.200, y, 0.012)),
            material=dark_anodized,
            name=name,
        )
    carriage.visual(
        Box((0.480, 0.180, 0.040)),
        origin=Origin(xyz=(0.200, 0.0, 0.044)),
        material=blue_anodized,
        name="stage_body",
    )
    carriage.visual(
        Box((0.420, 0.140, 0.012)),
        origin=Origin(xyz=(0.210, 0.0, 0.070)),
        material=ground_steel,
        name="top_cover",
    )
    carriage.visual(
        Box((0.120, 0.120, 0.075)),
        origin=Origin(xyz=(0.340, 0.0, 0.1135)),
        material=cast_aluminum,
        name="shoulder_block",
    )
    for y, name in ((0.055, "block_rib_0"), (-0.055, "block_rib_1")):
        carriage.visual(
            Box((0.145, 0.014, 0.046)),
            origin=Origin(xyz=(0.275, y, 0.099)),
            material=cast_aluminum,
            name=name,
        )
    carriage.visual(
        Cylinder(radius=0.043, length=0.021),
        origin=Origin(xyz=(0.340, 0.0, 0.1615)),
        material=ground_steel,
        name="shoulder_bearing",
    )
    carriage.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.340, 0.0, 0.168)),
        material=fastener_black,
        name="shoulder_fastener",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=ground_steel,
        name="shoulder_washer",
    )
    upper_arm.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=fastener_black,
        name="shoulder_cap",
    )
    for y, name in ((0.034, "link_plate_0"), (-0.034, "link_plate_1")):
        upper_arm.visual(
            Box((0.340, 0.018, 0.024)),
            origin=Origin(xyz=(0.180, y, 0.024)),
            material=blue_anodized,
            name=name,
        )
    upper_arm.visual(
        Box((0.230, 0.070, 0.014)),
        origin=Origin(xyz=(0.180, 0.0, 0.024)),
        material=dark_anodized,
        name="center_rib",
    )
    upper_arm.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.360, 0.0, 0.013)),
        material=ground_steel,
        name="elbow_washer",
    )
    upper_arm.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.360, 0.0, 0.031)),
        material=fastener_black,
        name="elbow_cap",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=ground_steel,
        name="elbow_hub",
    )
    lower_arm.visual(
        Box((0.070, 0.036, 0.012)),
        origin=Origin(xyz=(0.045, 0.0, -0.030)),
        material=ground_steel,
        name="hub_neck",
    )
    lower_arm.visual(
        Box((0.300, 0.040, 0.022)),
        origin=Origin(xyz=(0.200, 0.0, -0.023)),
        material=dark_anodized,
        name="forearm_bar",
    )
    for y, name in ((0.024, "forearm_rib_0"), (-0.024, "forearm_rib_1")):
        lower_arm.visual(
            Box((0.270, 0.009, 0.010)),
            origin=Origin(xyz=(0.205, y, -0.010)),
            material=blue_anodized,
            name=name,
        )
    lower_arm.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.340, 0.0, -0.035)),
        material=ground_steel,
        name="pad_end_hub",
    )
    lower_arm.visual(
        Box((0.115, 0.055, 0.018)),
        origin=Origin(xyz=(0.400, 0.0, -0.035)),
        material=dark_anodized,
        name="pad_mount",
    )
    lower_arm.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.440, 0.0, -0.050)),
        material=ground_steel,
        name="pad_stem",
    )
    lower_arm.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.440, 0.0, -0.069)),
        material=rubber,
        name="pad",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.480, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=0.540),
    )
    model.articulation(
        "carriage_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.340, 0.0, 0.172)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.80, upper=1.20),
    )
    model.articulation(
        "upper_arm_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lower_arm,
        origin=Origin(xyz=(0.360, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.8, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    lower_arm = object_model.get_part("lower_arm")
    slide = object_model.get_articulation("base_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_lower_arm")

    ctx.expect_contact(
        carriage,
        base,
        elem_a="bearing_shoe_0",
        elem_b="guide_rail_0",
        contact_tol=0.001,
        name="carriage shoe rides on rail",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="bearing_shoe_0",
        outer_elem="guide_rail_0",
        margin=0.006,
        name="bearing shoe centered on guide rail",
    )
    ctx.expect_gap(
        upper_arm,
        carriage,
        axis="z",
        positive_elem="link_plate_0",
        negative_elem="top_cover",
        min_gap=0.095,
        name="upper link is spaced above carriage cover",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a="elbow_washer",
        elem_b="elbow_hub",
        contact_tol=0.001,
        name="elbow hub is captured by upper link washer",
    )
    ctx.expect_gap(
        upper_arm,
        lower_arm,
        axis="z",
        positive_elem="link_plate_0",
        negative_elem="forearm_bar",
        min_gap=0.006,
        name="forearm clears upper link plates",
    )

    with ctx.pose({slide: 0.540}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="bearing_shoe_1",
            elem_b="guide_rail_1",
            contact_tol=0.001,
            name="extended carriage still rides the rail",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="stop_1",
            negative_elem="stage_body",
            min_gap=0.050,
            name="extended carriage clears end stop",
        )

    rest_pos = ctx.part_world_position(lower_arm)
    with ctx.pose({shoulder: 1.0, elbow: -1.0}):
        ctx.expect_gap(
            lower_arm,
            carriage,
            axis="z",
            positive_elem="forearm_bar",
            negative_elem="shoulder_block",
            max_penetration=0.0,
            name="folded forearm stays above shoulder block",
        )
        moved_pos = ctx.part_world_position(lower_arm)

    ctx.check(
        "shoulder and elbow move the pad outward in plan",
        rest_pos is not None
        and moved_pos is not None
        and abs(moved_pos[1] - rest_pos[1]) > 0.10,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
