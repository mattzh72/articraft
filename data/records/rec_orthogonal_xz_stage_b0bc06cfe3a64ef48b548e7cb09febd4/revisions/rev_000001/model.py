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
    model = ArticulatedObject(name="wall_backed_xz_positioning_module")

    wall_gray = model.material("wall_gray", rgba=(0.54, 0.57, 0.58, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.73, 0.74, 0.72, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.24, 0.52, 1.0))
    mast_orange = model.material("mast_orange", rgba=(0.92, 0.47, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.02, 0.02, 0.018, 1.0))

    back = model.part("back_frame")
    back.visual(
        Box((1.25, 0.025, 0.90)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wall_gray,
        name="wall_plate",
    )
    back.visual(
        Box((1.23, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.035, 0.425)),
        material=frame_dark,
        name="top_frame_rail",
    )
    back.visual(
        Box((1.23, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.035, -0.425)),
        material=frame_dark,
        name="bottom_frame_rail",
    )
    back.visual(
        Box((0.045, 0.045, 0.90)),
        origin=Origin(xyz=(-0.6025, 0.035, 0.0)),
        material=frame_dark,
        name="side_frame_0",
    )
    back.visual(
        Box((0.045, 0.045, 0.90)),
        origin=Origin(xyz=(0.6025, 0.035, 0.0)),
        material=frame_dark,
        name="side_frame_1",
    )
    back.visual(
        Box((1.10, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.035, 0.22)),
        material=rail_steel,
        name="upper_linear_rail",
    )
    back.visual(
        Box((1.10, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.035, -0.22)),
        material=rail_steel,
        name="lower_linear_rail",
    )
    back.visual(
        Cylinder(radius=0.012, length=1.15),
        origin=Origin(xyz=(0.0, 0.063, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_steel,
        name="lead_screw",
    )
    for x, name in ((-0.56, "screw_bearing_0"), (0.56, "screw_bearing_1")):
        back.visual(
            Box((0.065, 0.055, 0.085)),
            origin=Origin(xyz=(x, 0.040, 0.0)),
            material=frame_dark,
            name=name,
        )

    carriage = model.part("lateral_carriage")
    carriage.visual(
        Box((0.28, 0.014, 0.070)),
        origin=Origin(xyz=(0.0, 0.0645, 0.22)),
        material=rubber_black,
        name="upper_bearing",
    )
    carriage.visual(
        Box((0.28, 0.014, 0.070)),
        origin=Origin(xyz=(0.0, 0.0645, -0.22)),
        material=rubber_black,
        name="lower_bearing",
    )
    carriage.visual(
        Box((0.32, 0.035, 0.56)),
        origin=Origin(xyz=(0.0, 0.089, 0.0)),
        material=carriage_blue,
        name="carriage_plate",
    )
    for z, name in ((0.285, "upper_mast_standoff"), (0.0, "middle_mast_standoff"), (-0.285, "lower_mast_standoff")):
        carriage.visual(
            Box((0.32, 0.025, 0.055)),
            origin=Origin(xyz=(0.0, 0.119, z)),
            material=carriage_blue,
            name=name,
        )
    carriage.visual(
        Box((0.035, 0.035, 0.62)),
        origin=Origin(xyz=(-0.115, 0.149, 0.0)),
        material=rail_steel,
        name="vertical_guide_0",
    )
    carriage.visual(
        Box((0.035, 0.035, 0.62)),
        origin=Origin(xyz=(0.115, 0.149, 0.0)),
        material=rail_steel,
        name="vertical_guide_1",
    )
    for z, name in ((0.325, "mast_top_bridge"), (-0.325, "mast_bottom_bridge")):
        carriage.visual(
            Box((0.31, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, 0.149, z)),
            material=frame_dark,
            name=name,
        )
    carriage.visual(
        Box((0.020, 0.018, 0.50)),
        origin=Origin(xyz=(-0.170, 0.154, 0.0)),
        material=frame_dark,
        name="vertical_scale",
    )
    for z, name in ((0.19, "scale_clamp_0"), (-0.19, "scale_clamp_1")):
        carriage.visual(
            Box((0.060, 0.018, 0.025)),
            origin=Origin(xyz=(-0.145, 0.154, z)),
            material=frame_dark,
            name=name,
        )

    mast = model.part("mast_stage")
    mast.visual(
        Box((0.065, 0.016, 0.20)),
        origin=Origin(xyz=(-0.115, 0.1745, 0.0)),
        material=rubber_black,
        name="mast_bearing_0",
    )
    mast.visual(
        Box((0.065, 0.016, 0.20)),
        origin=Origin(xyz=(0.115, 0.1745, 0.0)),
        material=rubber_black,
        name="mast_bearing_1",
    )
    mast.visual(
        Box((0.34, 0.045, 0.18)),
        origin=Origin(xyz=(0.0, 0.205, 0.0)),
        material=mast_orange,
        name="tool_plate",
    )
    mast.visual(
        Box((0.22, 0.017, 0.12)),
        origin=Origin(xyz=(0.0, 0.236, 0.0)),
        material=frame_dark,
        name="tool_mount",
    )
    for x in (-0.075, 0.075):
        for z in (-0.045, 0.045):
            mast.visual(
                Box((0.025, 0.006, 0.025)),
                origin=Origin(xyz=(x, 0.2475, z)),
                material=rail_steel,
                name=f"mount_bolt_{int(x > 0)}_{int(z > 0)}",
            )

    model.articulation(
        "back_to_carriage",
        ArticulationType.PRISMATIC,
        parent=back,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "carriage_to_mast",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.32),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    back = object_model.get_part("back_frame")
    carriage = object_model.get_part("lateral_carriage")
    mast = object_model.get_part("mast_stage")
    x_slide = object_model.get_articulation("back_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_mast")

    ctx.check(
        "module has exactly two prismatic stages",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and len(object_model.articulations) == 2,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_contact(
        carriage,
        back,
        elem_a="upper_bearing",
        elem_b="upper_linear_rail",
        name="lateral carriage rides the upper horizontal rail",
    )
    ctx.expect_contact(
        carriage,
        back,
        elem_a="lower_bearing",
        elem_b="lower_linear_rail",
        name="lateral carriage rides the lower horizontal rail",
    )
    ctx.expect_contact(
        mast,
        carriage,
        elem_a="mast_bearing_0",
        elem_b="vertical_guide_0",
        name="mast stage rides the first vertical guide",
    )
    ctx.expect_contact(
        mast,
        carriage,
        elem_a="mast_bearing_1",
        elem_b="vertical_guide_1",
        name="mast stage rides the second vertical guide",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            back,
            axes="x",
            min_overlap=0.10,
            elem_a="upper_bearing",
            elem_b="upper_linear_rail",
            name="lateral carriage remains captured at full travel",
        )

    ctx.check(
        "positive lateral travel moves the carriage along X",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.30,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        raised_mast_pos = ctx.part_world_position(mast)
        ctx.expect_overlap(
            mast,
            carriage,
            axes="z",
            min_overlap=0.08,
            elem_a="mast_bearing_0",
            elem_b="vertical_guide_0",
            name="mast stage remains captured at full height",
        )

    ctx.check(
        "positive mast travel raises the carried stage along Z",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.25,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    return ctx.report()


object_model = build_object_model()
