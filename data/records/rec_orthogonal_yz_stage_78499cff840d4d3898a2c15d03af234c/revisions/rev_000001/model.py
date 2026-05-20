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
    model = ArticulatedObject(name="side_mounted_yz_stage")

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.025, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.67, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.04, 0.20, 0.55, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.28, 0.78, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=matte_black,
        name="base_foot",
    )
    frame.visual(
        Box((0.055, 0.76, 0.72)),
        origin=Origin(xyz=(-0.08, 0.0, 0.39)),
        material=dark_anodized,
        name="back_plate",
    )
    frame.visual(
        Box((0.040, 0.034, 0.64)),
        origin=Origin(xyz=(-0.0375, -0.12, 0.40)),
        material=brushed_steel,
        name="vertical_rail_0",
    )
    frame.visual(
        Box((0.040, 0.034, 0.64)),
        origin=Origin(xyz=(-0.0375, 0.12, 0.40)),
        material=brushed_steel,
        name="vertical_rail_1",
    )
    frame.visual(
        Box((0.10, 0.34, 0.035)),
        origin=Origin(xyz=(-0.035, 0.0, 0.075)),
        material=matte_black,
        name="lower_stop",
    )
    frame.visual(
        Box((0.10, 0.34, 0.035)),
        origin=Origin(xyz=(-0.035, 0.0, 0.735)),
        material=matte_black,
        name="upper_stop",
    )

    vertical_carriage = model.part("vertical_carriage")
    vertical_carriage.visual(
        Box((0.035, 0.065, 0.14)),
        origin=Origin(xyz=(0.0, -0.12, 0.0)),
        material=satin_aluminum,
        name="linear_bearing_0",
    )
    vertical_carriage.visual(
        Box((0.035, 0.065, 0.14)),
        origin=Origin(xyz=(0.0, 0.12, 0.0)),
        material=satin_aluminum,
        name="linear_bearing_1",
    )
    vertical_carriage.visual(
        Box((0.037, 0.36, 0.18)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=satin_aluminum,
        name="carriage_plate",
    )
    vertical_carriage.visual(
        Box((0.10, 0.48, 0.022)),
        origin=Origin(xyz=(0.095, 0.0, 0.075)),
        material=brushed_steel,
        name="side_upper_way",
    )
    vertical_carriage.visual(
        Box((0.10, 0.48, 0.022)),
        origin=Origin(xyz=(0.095, 0.0, -0.035)),
        material=brushed_steel,
        name="side_lower_way",
    )
    vertical_carriage.visual(
        Box((0.040, 0.10, 0.12)),
        origin=Origin(xyz=(0.065, 0.0, 0.02)),
        material=satin_aluminum,
        name="way_mount_web",
    )

    side_slide = model.part("side_slide")
    side_slide.visual(
        Box((0.060, 0.62, 0.088)),
        origin=Origin(xyz=(0.0, 0.23, 0.0)),
        material=blue_anodized,
        name="side_bar",
    )
    side_slide.visual(
        Box((0.16, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.619, 0.068)),
        material=satin_aluminum,
        name="end_platform",
    )
    side_slide.visual(
        Box((0.10, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, 0.540, 0.018)),
        material=blue_anodized,
        name="platform_neck",
    )
    bolt_positions = ((-0.048, 0.580), (0.048, 0.580), (-0.048, 0.658), (0.048, 0.658))
    for i, (x, y) in enumerate(bolt_positions):
        side_slide.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.095)),
            material=dark_anodized,
            name=f"platform_bolt_{i}",
        )

    model.articulation(
        "frame_to_vertical_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=vertical_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.34),
    )

    model.articulation(
        "vertical_carriage_to_side_slide",
        ArticulationType.PRISMATIC,
        parent=vertical_carriage,
        child=side_slide,
        origin=Origin(xyz=(0.145, -0.18, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.22, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    vertical_carriage = object_model.get_part("vertical_carriage")
    side_slide = object_model.get_part("side_slide")
    vertical_joint = object_model.get_articulation("frame_to_vertical_carriage")
    side_joint = object_model.get_articulation("vertical_carriage_to_side_slide")

    ctx.check(
        "two prismatic stages",
        len(object_model.articulations) == 2
        and vertical_joint.articulation_type == ArticulationType.PRISMATIC
        and side_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "vertical joint uses z axis",
        tuple(round(v, 6) for v in vertical_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={vertical_joint.axis}",
    )
    ctx.check(
        "side joint uses y axis",
        tuple(round(v, 6) for v in side_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={side_joint.axis}",
    )

    with ctx.pose({vertical_joint: 0.0, side_joint: 0.0}):
        ctx.expect_contact(
            frame,
            vertical_carriage,
            elem_a="vertical_rail_0",
            elem_b="linear_bearing_0",
            contact_tol=1e-5,
            name="one vertical bearing rides on its rail",
        )
        ctx.expect_contact(
            frame,
            vertical_carriage,
            elem_a="vertical_rail_1",
            elem_b="linear_bearing_1",
            contact_tol=1e-5,
            name="other vertical bearing rides on its rail",
        )
        ctx.expect_contact(
            vertical_carriage,
            side_slide,
            elem_a="side_upper_way",
            elem_b="side_bar",
            contact_tol=1e-5,
            name="side slide is captured by the upper way",
        )
        ctx.expect_contact(
            vertical_carriage,
            side_slide,
            elem_a="side_lower_way",
            elem_b="side_bar",
            contact_tol=1e-5,
            name="side slide is captured by the lower way",
        )
        ctx.expect_overlap(
            side_slide,
            vertical_carriage,
            axes="y",
            elem_a="side_bar",
            elem_b="side_upper_way",
            min_overlap=0.45,
            name="side bar is substantially engaged at rest",
        )

    rest_vertical = ctx.part_world_position(vertical_carriage)
    with ctx.pose({vertical_joint: 0.34}):
        raised_vertical = ctx.part_world_position(vertical_carriage)
        ctx.expect_overlap(
            frame,
            vertical_carriage,
            axes="z",
            elem_a="vertical_rail_0",
            elem_b="linear_bearing_0",
            min_overlap=0.13,
            name="vertical carriage remains on rail when raised",
        )

    rest_side = ctx.part_world_position(side_slide)
    with ctx.pose({side_joint: 0.28}):
        extended_side = ctx.part_world_position(side_slide)
        ctx.expect_overlap(
            side_slide,
            vertical_carriage,
            axes="y",
            elem_a="side_bar",
            elem_b="side_upper_way",
            min_overlap=0.20,
            name="side bar retains insertion when extended",
        )

    ctx.check(
        "vertical stage moves upward",
        rest_vertical is not None
        and raised_vertical is not None
        and raised_vertical[2] > rest_vertical[2] + 0.30,
        details=f"rest={rest_vertical}, raised={raised_vertical}",
    )
    ctx.check(
        "side stage moves sideways",
        rest_side is not None
        and extended_side is not None
        and extended_side[1] > rest_side[1] + 0.25,
        details=f"rest={rest_side}, extended={extended_side}",
    )

    return ctx.report()


object_model = build_object_model()
