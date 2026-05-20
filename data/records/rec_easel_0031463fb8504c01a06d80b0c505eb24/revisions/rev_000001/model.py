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


def _leg_yaw(target_xy: tuple[float, float]) -> float:
    target_x, target_y = target_xy
    return math.atan2(-target_x, target_y)


def _add_leg(
    model: ArticulatedObject,
    *,
    parent,
    name: str,
    joint_name: str,
    wood,
    hardware,
    rubber,
    hinge_xyz: tuple[float, float, float],
    target_xy: tuple[float, float],
    shoe_size: tuple[float, float, float],
    stile_size: tuple[float, float, float],
    foot_size: tuple[float, float, float],
    lower: float,
    upper: float,
) -> None:
    leg = model.part(name)
    shoe_clearance = 0.020
    shoe_center_z = -(shoe_clearance + shoe_size[2] * 0.5)
    shoe_bottom_z = -(shoe_clearance + shoe_size[2])
    leg.visual(
        Box(shoe_size),
        origin=Origin(xyz=(0.0, 0.0, shoe_center_z)),
        material=wood,
        name="hinge_shoe",
    )
    leg.visual(
        Cylinder(radius=0.0055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -shoe_clearance), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware,
        name="hinge_pin",
    )
    leg.visual(
        Box((0.016, 0.042, 0.016)),
        origin=Origin(xyz=(0.0, -0.021, -0.026)),
        material=wood,
        name="hinge_tongue",
    )
    leg.visual(
        Box(stile_size),
        origin=Origin(xyz=(0.0, 0.0, shoe_bottom_z - stile_size[2] * 0.5)),
        material=wood,
        name="stile",
    )
    leg.visual(
        Box(foot_size),
        origin=Origin(
            xyz=(0.0, 0.0, shoe_bottom_z - stile_size[2] - foot_size[2] * 0.5)
        ),
        material=rubber,
        name="foot",
    )

    vertical_drop = shoe_clearance + shoe_size[2] + stile_size[2] + foot_size[2] * 0.5
    open_roll = math.atan2(math.hypot(*target_xy), vertical_drop)
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=parent,
        child=leg,
        origin=Origin(xyz=hinge_xyz, rpy=(open_roll, 0.0, _leg_yaw(target_xy))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.0,
            lower=lower,
            upper=upper,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_easel")

    oak = model.material("oak", rgba=(0.66, 0.52, 0.34, 1.0))
    beech = model.material("beech", rgba=(0.77, 0.67, 0.49, 1.0))
    hardware = model.material("hardware", rgba=(0.20, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    felt = model.material("felt", rgba=(0.12, 0.12, 0.13, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.060, 0.030, 1.760)),
        origin=Origin(xyz=(0.0, 0.0, 0.880)),
        material=oak,
        name="mast_beam",
    )
    mast.visual(
        Box((0.142, 0.100, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 1.500)),
        material=oak,
        name="crown_block",
    )
    mast.visual(
        Box((0.092, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.582)),
        material=beech,
        name="top_cap",
    )
    mast.visual(
        Box((0.082, 0.046, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=beech,
        name="base_block",
    )

    front_hinge_z = 1.500
    shoe_size = (0.040, 0.020, 0.044)
    stile_size = (0.036, 0.022, 1.450)
    foot_size = (0.060, 0.038, 0.024)
    _add_leg(
        model,
        parent=mast,
        name="front_leg_0",
        joint_name="mast_to_front_leg_0",
        wood=beech,
        hardware=hardware,
        rubber=rubber,
        hinge_xyz=(0.088, 0.069, front_hinge_z),
        target_xy=(0.280, 0.230),
        shoe_size=shoe_size,
        stile_size=stile_size,
        foot_size=foot_size,
        lower=-0.95,
        upper=0.18,
    )
    _add_leg(
        model,
        parent=mast,
        name="front_leg_1",
        joint_name="mast_to_front_leg_1",
        wood=beech,
        hardware=hardware,
        rubber=rubber,
        hinge_xyz=(-0.088, 0.069, front_hinge_z),
        target_xy=(-0.280, 0.230),
        shoe_size=shoe_size,
        stile_size=stile_size,
        foot_size=foot_size,
        lower=-0.95,
        upper=0.18,
    )
    _add_leg(
        model,
        parent=mast,
        name="rear_leg",
        joint_name="mast_to_rear_leg",
        wood=beech,
        hardware=hardware,
        rubber=rubber,
        hinge_xyz=(0.0, -0.0545, front_hinge_z),
        target_xy=(0.0, -0.400),
        shoe_size=shoe_size,
        stile_size=(0.038, 0.024, 1.430),
        foot_size=foot_size,
        lower=-1.05,
        upper=0.14,
    )

    top_clamp = model.part("top_clamp")
    top_clamp.visual(
        Box((0.106, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.032, 0.0)),
        material=beech,
        name="rear_bridge",
    )
    top_clamp.visual(
        Box((0.030, 0.010, 0.110)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=felt,
        name="rear_runner",
    )
    top_clamp.visual(
        Box((0.018, 0.052, 0.120)),
        origin=Origin(xyz=(0.042, -0.006, 0.0)),
        material=beech,
        name="guide_0",
    )
    top_clamp.visual(
        Box((0.018, 0.052, 0.120)),
        origin=Origin(xyz=(-0.042, -0.006, 0.0)),
        material=beech,
        name="guide_1",
    )
    top_clamp.visual(
        Box((0.106, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.028, 0.060)),
        material=beech,
        name="rear_top",
    )
    top_clamp.visual(
        Box((0.106, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, 0.060)),
        material=beech,
        name="front_top",
    )
    top_clamp.visual(
        Box((0.074, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.032, -0.008)),
        material=beech,
        name="jaw",
    )
    top_clamp.visual(
        Box((0.058, 0.008, 0.105)),
        origin=Origin(xyz=(0.0, 0.045, -0.020)),
        material=felt,
        name="jaw_pad",
    )
    model.articulation(
        "mast_to_top_clamp",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=top_clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=0.280,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.106, 0.018, 0.140)),
        origin=Origin(xyz=(0.0, -0.032, 0.0)),
        material=beech,
        name="rear_bridge",
    )
    tray.visual(
        Box((0.030, 0.010, 0.120)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=felt,
        name="rear_runner",
    )
    tray.visual(
        Box((0.018, 0.082, 0.140)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material=beech,
        name="side_0",
    )
    tray.visual(
        Box((0.018, 0.082, 0.140)),
        origin=Origin(xyz=(-0.044, 0.0, 0.0)),
        material=beech,
        name="side_1",
    )
    tray.visual(
        Box((0.106, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.028, 0.061)),
        material=beech,
        name="rear_top",
    )
    tray.visual(
        Box((0.106, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, 0.061)),
        material=beech,
        name="front_top",
    )
    tray.visual(
        Box((0.106, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.028, -0.061)),
        material=beech,
        name="rear_bottom",
    )
    tray.visual(
        Box((0.106, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, -0.061)),
        material=beech,
        name="front_bottom",
    )
    tray.visual(
        Box((0.084, 0.022, 0.052)),
        origin=Origin(xyz=(0.0, 0.032, -0.040)),
        material=beech,
        name="front_saddle",
    )
    tray.visual(
        Box((0.084, 0.230, 0.036)),
        origin=Origin(xyz=(0.0, 0.148, -0.030)),
        material=beech,
        name="tray_arm",
    )
    tray.visual(
        Box((0.098, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.242, -0.020)),
        material=beech,
        name="tray_web",
    )
    tray.visual(
        Box((0.600, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, 0.283, -0.008)),
        material=oak,
        name="shelf",
    )
    tray.visual(
        Box((0.600, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, 0.323, 0.010)),
        material=oak,
        name="lip",
    )
    tray.visual(
        Box((0.540, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.246, 0.002)),
        material=beech,
        name="back_rib",
    )
    tray.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.058, 0.000, 0.000), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware,
        name="side_boss",
    )
    model.articulation(
        "mast_to_tray",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.18,
            lower=0.0,
            upper=0.460,
        ),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.0042, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware,
        name="shaft",
    )
    clamp_knob.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware,
        name="knob",
    )
    clamp_knob.visual(
        Box((0.008, 0.044, 0.012)),
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        material=hardware,
        name="wing",
    )
    model.articulation(
        "tray_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=tray,
        child=clamp_knob,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    top_clamp = object_model.get_part("top_clamp")
    tray = object_model.get_part("tray")
    clamp_knob = object_model.get_part("clamp_knob")
    front_leg_0 = object_model.get_part("front_leg_0")
    front_leg_1 = object_model.get_part("front_leg_1")
    rear_leg = object_model.get_part("rear_leg")

    top_clamp_slide = object_model.get_articulation("mast_to_top_clamp")
    tray_slide = object_model.get_articulation("mast_to_tray")
    knob_spin = object_model.get_articulation("tray_to_clamp_knob")
    front_leg_fold = object_model.get_articulation("mast_to_front_leg_0")
    rear_leg_fold = object_model.get_articulation("mast_to_rear_leg")

    ctx.allow_overlap(
        front_leg_0,
        mast,
        elem_a="hinge_pin",
        elem_b="crown_block",
        reason="The exposed front hinge pin is intentionally captured by the crown block.",
    )
    ctx.allow_overlap(
        front_leg_0,
        mast,
        elem_a="hinge_tongue",
        elem_b="crown_block",
        reason="The front leg hinge tongue is intentionally seated inside the crown hinge pocket.",
    )
    ctx.allow_overlap(
        front_leg_1,
        mast,
        elem_a="hinge_pin",
        elem_b="crown_block",
        reason="The exposed front hinge pin is intentionally captured by the crown block.",
    )
    ctx.allow_overlap(
        front_leg_1,
        mast,
        elem_a="hinge_tongue",
        elem_b="crown_block",
        reason="The front leg hinge tongue is intentionally seated inside the crown hinge pocket.",
    )
    ctx.allow_overlap(
        rear_leg,
        mast,
        elem_a="hinge_pin",
        elem_b="crown_block",
        reason="The rear hinge pin is intentionally captured by the crown block.",
    )
    ctx.allow_overlap(
        rear_leg,
        mast,
        elem_a="hinge_tongue",
        elem_b="crown_block",
        reason="The rear leg hinge tongue is intentionally seated inside the crown hinge pocket.",
    )

    ctx.expect_overlap(
        top_clamp,
        mast,
        axes="xy",
        min_overlap=0.030,
        name="top clamp stays centered on mast",
    )
    ctx.expect_overlap(
        tray,
        mast,
        axes="xy",
        min_overlap=0.030,
        name="tray carriage stays centered on mast",
    )
    ctx.expect_gap(
        top_clamp,
        tray,
        axis="z",
        min_gap=0.280,
        name="top clamp starts above tray",
    )
    ctx.expect_contact(
        clamp_knob,
        tray,
        elem_a="shaft",
        elem_b="side_boss",
        name="clamp knob is mounted on tray boss",
    )

    tray_rest = ctx.part_world_position(tray)
    top_clamp_rest = ctx.part_world_position(top_clamp)
    front_foot_rest = ctx.part_element_world_aabb(front_leg_0, elem="foot")
    rear_foot_rest = ctx.part_element_world_aabb(rear_leg, elem="foot")

    with ctx.pose({tray_slide: 0.220, top_clamp_slide: 0.180}):
        ctx.expect_overlap(
            top_clamp,
            mast,
            axes="xy",
            min_overlap=0.030,
            name="raised top clamp stays centered on mast",
        )
        ctx.expect_overlap(
            tray,
            mast,
            axes="xy",
            min_overlap=0.030,
            name="raised tray stays centered on mast",
        )
        tray_raised = ctx.part_world_position(tray)
        top_clamp_raised = ctx.part_world_position(top_clamp)

    ctx.check(
        "tray slides upward",
        tray_rest is not None
        and tray_raised is not None
        and tray_raised[2] > tray_rest[2] + 0.15,
        details=f"rest={tray_rest}, raised={tray_raised}",
    )
    ctx.check(
        "top clamp slides upward",
        top_clamp_rest is not None
        and top_clamp_raised is not None
        and top_clamp_raised[2] > top_clamp_rest[2] + 0.15,
        details=f"rest={top_clamp_rest}, raised={top_clamp_raised}",
    )

    with ctx.pose({front_leg_fold: -0.900, rear_leg_fold: -0.950}):
        front_foot_folded = ctx.part_element_world_aabb(front_leg_0, elem="foot")
        rear_foot_folded = ctx.part_element_world_aabb(rear_leg, elem="foot")

    ctx.check(
        "front leg folds upward toward mast",
        front_foot_rest is not None
        and front_foot_folded is not None
        and front_foot_folded[0][2] > front_foot_rest[0][2] + 0.18,
        details=f"rest={front_foot_rest}, folded={front_foot_folded}",
    )
    ctx.check(
        "rear leg folds upward toward mast",
        rear_foot_rest is not None
        and rear_foot_folded is not None
        and rear_foot_folded[0][2] > rear_foot_rest[0][2] + 0.18,
        details=f"rest={rear_foot_rest}, folded={rear_foot_folded}",
    )

    with ctx.pose({knob_spin: math.pi}):
        ctx.expect_contact(
            clamp_knob,
            tray,
            elem_a="shaft",
            elem_b="side_boss",
            name="rotated clamp knob stays seated in boss",
        )

    return ctx.report()


object_model = build_object_model()
