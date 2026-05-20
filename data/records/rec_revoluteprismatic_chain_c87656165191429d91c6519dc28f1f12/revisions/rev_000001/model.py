from __future__ import annotations

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
    model = ArticulatedObject(name="swing_out_slide")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.14, 0.16, 1.0))
    zinc = model.material("zinc_plated_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.04, 0.045, 0.05, 1.0))
    anodized = model.material("anodized_carriage", rgba=(0.18, 0.22, 0.26, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.16, 0.12, 0.012)),
        origin=Origin(xyz=(-0.010, 0.0, -0.061)),
        material=painted_steel,
        name="mounting_plate",
    )
    base.visual(
        Box((0.036, 0.070, 0.092)),
        origin=Origin(xyz=(-0.048, 0.0, -0.009)),
        material=painted_steel,
        name="upright_web",
    )
    base.visual(
        Box((0.092, 0.070, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, -0.036)),
        material=painted_steel,
        name="lower_fork_plate",
    )
    base.visual(
        Box((0.092, 0.070, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.036)),
        material=painted_steel,
        name="upper_fork_plate",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="base_pin",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=zinc,
        name="lower_pin_washer",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=zinc,
        name="upper_pin_washer",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=zinc,
        name="pin_head",
    )

    arm = model.part("primary_arm")
    arm.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_bearing,
        name="hinge_sleeve",
    )
    arm.visual(
        Box((0.360, 0.032, 0.026)),
        origin=Origin(xyz=(0.198, 0.0, 0.0)),
        material=painted_steel,
        name="arm_beam",
    )
    arm.visual(
        Box((0.220, 0.058, 0.012)),
        origin=Origin(xyz=(0.420, 0.0, 0.017)),
        material=painted_steel,
        name="slide_deck",
    )
    arm.visual(
        Box((0.200, 0.008, 0.012)),
        origin=Origin(xyz=(0.420, 0.020, 0.029)),
        material=zinc,
        name="slide_rail_0",
    )
    arm.visual(
        Box((0.200, 0.008, 0.012)),
        origin=Origin(xyz=(0.420, -0.020, 0.029)),
        material=zinc,
        name="slide_rail_1",
    )
    arm.visual(
        Box((0.010, 0.058, 0.030)),
        origin=Origin(xyz=(0.315, 0.0, 0.035)),
        material=painted_steel,
        name="inner_stop",
    )
    arm.visual(
        Box((0.010, 0.058, 0.030)),
        origin=Origin(xyz=(0.525, 0.0, 0.035)),
        material=painted_steel,
        name="outer_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.075, 0.070, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=anodized,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.060, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, -0.015)),
        material=dark_bearing,
        name="guide_pad_0",
    )
    carriage.visual(
        Box((0.060, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, -0.015)),
        material=dark_bearing,
        name="guide_pad_1",
    )
    carriage.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=zinc,
        name="top_mount_boss",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.365, 0.0, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.110),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    arm = object_model.get_part("primary_arm")
    carriage = object_model.get_part("carriage")
    swing = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.allow_overlap(
        base,
        arm,
        elem_a="base_pin",
        elem_b="hinge_sleeve",
        reason="The fixed pin is intentionally captured inside the hinged arm sleeve.",
    )
    ctx.expect_within(
        base,
        arm,
        axes="xy",
        inner_elem="base_pin",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="pin is centered inside hinge sleeve",
    )
    ctx.expect_overlap(
        base,
        arm,
        axes="z",
        elem_a="base_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.045,
        name="pin passes through hinge sleeve",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            arm,
            elem_a="guide_pad_0",
            elem_b="slide_rail_0",
            contact_tol=0.001,
            name="front guide pad rides on rail",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a="guide_pad_1",
            elem_b="slide_rail_1",
            contact_tol=0.001,
            name="rear guide pad rides on rail",
        )
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="guide_pad_0",
            elem_b="slide_rail_0",
            min_overlap=0.055,
            name="carriage is retained on rail at rest",
        )
        retracted_position = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.110}):
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="guide_pad_0",
            elem_b="slide_rail_0",
            min_overlap=0.035,
            name="carriage remains engaged at full slide",
        )
        extended_position = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic joint extends along slide axis",
        retracted_position is not None
        and extended_position is not None
        and extended_position[0] > retracted_position[0] + 0.100,
        details=f"retracted={retracted_position}, extended={extended_position}",
    )

    with ctx.pose({swing: 1.0, slide: 0.0}):
        swung_position = ctx.part_world_position(carriage)

    ctx.check(
        "revolute joint swings carriage around base pin",
        retracted_position is not None
        and swung_position is not None
        and swung_position[1] > retracted_position[1] + 0.20,
        details=f"retracted={retracted_position}, swung={swung_position}",
    )

    return ctx.report()


object_model = build_object_model()
