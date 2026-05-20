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


REST_DESK_TILT = 0.22


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductors_music_stand")

    model.material("base_steel", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("mast_steel", rgba=(0.20, 0.20, 0.21, 1.0))
    model.material("desk_panel", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("trim", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.23, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="base_steel",
        name="base_disk",
    )
    base.visual(
        Box((0.058, 0.006, 0.56)),
        origin=Origin(xyz=(0.0, 0.026, 0.315)),
        material="mast_steel",
        name="sleeve_front",
    )
    base.visual(
        Box((0.058, 0.006, 0.56)),
        origin=Origin(xyz=(0.0, -0.026, 0.315)),
        material="mast_steel",
        name="sleeve_rear",
    )
    base.visual(
        Box((0.006, 0.046, 0.56)),
        origin=Origin(xyz=(0.026, 0.0, 0.315)),
        material="mast_steel",
        name="sleeve_right",
    )
    base.visual(
        Box((0.006, 0.046, 0.56)),
        origin=Origin(xyz=(-0.026, 0.0, 0.315)),
        material="mast_steel",
        name="sleeve_left",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.046, 0.046, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material="mast_steel",
        name="inner_mast",
    )
    mast.visual(
        Box((0.060, 0.050, 0.100)),
        origin=Origin(xyz=(0.0, -0.005, 0.445)),
        material="mast_steel",
        name="head_spine",
    )
    mast.visual(
        Box((0.018, 0.035, 0.090)),
        origin=Origin(xyz=(-0.035, 0.014, 0.485)),
        material="mast_steel",
        name="head_cheek_0",
    )
    mast.visual(
        Box((0.018, 0.035, 0.090)),
        origin=Origin(xyz=(0.035, 0.014, 0.485)),
        material="mast_steel",
        name="head_cheek_1",
    )
    mast.visual(
        Box((0.090, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, 0.500)),
        material="trim",
        name="head_bar",
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.740, 0.014, 0.450)),
        origin=Origin(xyz=(0.0, 0.014, -0.235)),
        material="desk_panel",
        name="panel",
    )
    desk.visual(
        Box((0.740, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.010, -0.030)),
        material="trim",
        name="top_rail",
    )
    desk.visual(
        Box((0.022, 0.026, 0.460)),
        origin=Origin(xyz=(-0.359, 0.012, -0.230)),
        material="trim",
        name="side_stile_0",
    )
    desk.visual(
        Box((0.022, 0.026, 0.460)),
        origin=Origin(xyz=(0.359, 0.012, -0.230)),
        material="trim",
        name="side_stile_1",
    )
    desk.visual(
        Box((0.740, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.034, -0.445)),
        material="trim",
        name="tray_shelf",
    )
    desk.visual(
        Box((0.740, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.060, -0.427)),
        material="trim",
        name="tray_lip",
    )
    desk.visual(
        Box((0.120, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.0185, -0.022)),
        material="mast_steel",
        name="hinge_block",
    )
    desk.visual(
        Box((0.018, 0.016, 0.420)),
        origin=Origin(xyz=(-0.379, 0.006, -0.225)),
        material="mast_steel",
        name="wing_mount_0",
    )
    desk.visual(
        Box((0.018, 0.016, 0.420)),
        origin=Origin(xyz=(0.379, 0.006, -0.225)),
        material="mast_steel",
        name="wing_mount_1",
    )

    wing_0 = model.part("wing_0")
    wing_0.visual(
        Box((0.180, 0.012, 0.420)),
        origin=Origin(xyz=(-0.096, 0.014, 0.0)),
        material="desk_panel",
        name="panel",
    )
    wing_0.visual(
        Box((0.007, 0.024, 0.420)),
        origin=Origin(xyz=(-0.0025, 0.010, 0.0)),
        material="mast_steel",
        name="hinge_stile",
    )
    wing_0.visual(
        Box((0.018, 0.024, 0.420)),
        origin=Origin(xyz=(-0.176, 0.010, 0.0)),
        material="trim",
        name="outer_stile",
    )
    wing_0.visual(
        Box((0.180, 0.024, 0.028)),
        origin=Origin(xyz=(-0.096, 0.010, 0.196)),
        material="trim",
        name="top_rail",
    )
    wing_0.visual(
        Box((0.180, 0.050, 0.018)),
        origin=Origin(xyz=(-0.096, 0.030, -0.215)),
        material="trim",
        name="tray_shelf",
    )
    wing_0.visual(
        Box((0.180, 0.012, 0.050)),
        origin=Origin(xyz=(-0.096, 0.055, -0.196)),
        material="trim",
        name="tray_lip",
    )

    wing_1 = model.part("wing_1")
    wing_1.visual(
        Box((0.180, 0.012, 0.420)),
        origin=Origin(xyz=(0.096, 0.014, 0.0)),
        material="desk_panel",
        name="panel",
    )
    wing_1.visual(
        Box((0.007, 0.024, 0.420)),
        origin=Origin(xyz=(0.0025, 0.010, 0.0)),
        material="mast_steel",
        name="hinge_stile",
    )
    wing_1.visual(
        Box((0.018, 0.024, 0.420)),
        origin=Origin(xyz=(0.176, 0.010, 0.0)),
        material="trim",
        name="outer_stile",
    )
    wing_1.visual(
        Box((0.180, 0.024, 0.028)),
        origin=Origin(xyz=(0.096, 0.010, 0.196)),
        material="trim",
        name="top_rail",
    )
    wing_1.visual(
        Box((0.180, 0.050, 0.018)),
        origin=Origin(xyz=(0.096, 0.030, -0.215)),
        material="trim",
        name="tray_shelf",
    )
    wing_1.visual(
        Box((0.180, 0.012, 0.050)),
        origin=Origin(xyz=(0.096, 0.055, -0.196)),
        material="trim",
        name="tray_lip",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.220),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.028, 0.500), rpy=(REST_DESK_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.30, upper=0.35),
    )
    model.articulation(
        "desk_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=wing_0,
        origin=Origin(xyz=(-0.389, 0.006, -0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "desk_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=wing_1,
        origin=Origin(xyz=(0.389, 0.006, -0.225)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")

    mast_slide = object_model.get_articulation("base_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    wing_joint_0 = object_model.get_articulation("desk_to_wing_0")
    wing_joint_1 = object_model.get_articulation("desk_to_wing_1")

    ctx.allow_overlap(
        desk,
        mast,
        elem_a="hinge_block",
        elem_b="head_bar",
        reason="The tilting head is simplified as a captured pivot block around the mast head bar.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.001,
        name="mast stays centered within the stand footprint",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_mast",
        min_overlap=0.080,
        name="mast remains inserted in the sleeve at rest",
    )
    ctx.expect_overlap(
        wing_0,
        desk,
        axes="z",
        min_overlap=0.300,
        name="left wing aligns with the desk plane when deployed",
    )
    ctx.expect_overlap(
        wing_1,
        desk,
        axes="z",
        min_overlap=0.300,
        name="right wing aligns with the desk plane when deployed",
    )
    ctx.expect_gap(
        desk,
        wing_0,
        axis="x",
        max_gap=0.030,
        max_penetration=0.0,
        name="left wing sits close to the desk hinge line",
    )
    ctx.expect_gap(
        wing_1,
        desk,
        axis="x",
        max_gap=0.030,
        max_penetration=0.0,
        name="right wing sits close to the desk hinge line",
    )

    slide_limits = mast_slide.motion_limits
    tilt_limits = desk_tilt.motion_limits
    wing_limits_0 = wing_joint_0.motion_limits
    wing_limits_1 = wing_joint_1.motion_limits

    if (
        slide_limits is not None
        and slide_limits.upper is not None
        and tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and wing_limits_0 is not None
        and wing_limits_0.upper is not None
        and wing_limits_1 is not None
        and wing_limits_1.upper is not None
    ):
        rest_mast = ctx.part_world_position(mast)
        with ctx.pose({mast_slide: slide_limits.upper}):
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_mast",
                min_overlap=0.055,
                name="mast retains insertion at full height",
            )
            extended_mast = ctx.part_world_position(mast)

        lower_tray = None
        upper_tray = None
        with ctx.pose({desk_tilt: tilt_limits.lower}):
            lower_aabb = ctx.part_element_world_aabb(desk, elem="tray_lip")
            if lower_aabb is not None:
                lower_tray = 0.5 * (lower_aabb[0][1] + lower_aabb[1][1])
        with ctx.pose({desk_tilt: tilt_limits.upper}):
            upper_aabb = ctx.part_element_world_aabb(desk, elem="tray_lip")
            if upper_aabb is not None:
                upper_tray = 0.5 * (upper_aabb[0][1] + upper_aabb[1][1])

        ctx.check(
            "desk tilt moves the score ledge forward when tilted back",
            lower_tray is not None and upper_tray is not None and upper_tray > lower_tray + 0.10,
            details=f"lower={lower_tray}, upper={upper_tray}",
        )
        ctx.check(
            "mast extends upward",
            rest_mast is not None and extended_mast is not None and extended_mast[2] > rest_mast[2] + 0.15,
            details=f"rest={rest_mast}, extended={extended_mast}",
        )

        wing_0_rest_y = None
        wing_0_folded_y = None
        wing_1_rest_y = None
        wing_1_folded_y = None
        rest_aabb_0 = ctx.part_element_world_aabb(wing_0, elem="outer_stile")
        if rest_aabb_0 is not None:
            wing_0_rest_y = 0.5 * (rest_aabb_0[0][1] + rest_aabb_0[1][1])
        rest_aabb_1 = ctx.part_element_world_aabb(wing_1, elem="outer_stile")
        if rest_aabb_1 is not None:
            wing_1_rest_y = 0.5 * (rest_aabb_1[0][1] + rest_aabb_1[1][1])

        with ctx.pose({wing_joint_0: wing_limits_0.upper, wing_joint_1: wing_limits_1.upper}):
            folded_aabb_0 = ctx.part_element_world_aabb(wing_0, elem="outer_stile")
            if folded_aabb_0 is not None:
                wing_0_folded_y = 0.5 * (folded_aabb_0[0][1] + folded_aabb_0[1][1])
            folded_aabb_1 = ctx.part_element_world_aabb(wing_1, elem="outer_stile")
            if folded_aabb_1 is not None:
                wing_1_folded_y = 0.5 * (folded_aabb_1[0][1] + folded_aabb_1[1][1])

        ctx.check(
            "left wing folds rearward",
            wing_0_rest_y is not None and wing_0_folded_y is not None and wing_0_folded_y < wing_0_rest_y - 0.10,
            details=f"rest={wing_0_rest_y}, folded={wing_0_folded_y}",
        )
        ctx.check(
            "right wing folds rearward",
            wing_1_rest_y is not None and wing_1_folded_y is not None and wing_1_folded_y < wing_1_rest_y - 0.10,
            details=f"rest={wing_1_rest_y}, folded={wing_1_folded_y}",
        )

    return ctx.report()


object_model = build_object_model()
