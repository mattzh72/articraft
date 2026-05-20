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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_automatic_espresso_machine")

    body_metal = model.material("body_metal", rgba=(0.77, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.74, 0.75, 1.0))
    tank_clear = model.material("tank_clear", rgba=(0.72, 0.86, 0.96, 0.45))

    body = model.part("body")
    body.visual(
        Box((0.300, 0.380, 0.028)),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=dark_trim,
        name="base_plinth",
    )
    body.visual(
        Box((0.012, 0.360, 0.300)),
        origin=Origin(xyz=(-0.144, -0.020, 0.178)),
        material=body_metal,
        name="left_shell",
    )
    body.visual(
        Box((0.012, 0.360, 0.300)),
        origin=Origin(xyz=(0.144, -0.020, 0.178)),
        material=body_metal,
        name="right_shell",
    )
    body.visual(
        Box((0.276, 0.280, 0.010)),
        origin=Origin(xyz=(0.0, -0.060, 0.355)),
        material=body_metal,
        name="deck",
    )
    body.visual(
        Box((0.240, 0.220, 0.003)),
        origin=Origin(xyz=(0.0, -0.070, 0.3615)),
        material=dark_trim,
        name="warming_mat",
    )
    body.visual(
        Box((0.276, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, -0.188, 0.340)),
        material=body_metal,
        name="rear_bridge",
    )
    body.visual(
        Box((0.276, 0.094, 0.060)),
        origin=Origin(xyz=(0.0, 0.121, 0.325)),
        material=body_metal,
        name="canopy",
    )
    body.visual(
        Box((0.276, 0.012, 0.220)),
        origin=Origin(xyz=(0.0, 0.174, 0.240)),
        material=body_metal,
        name="front_panel",
    )
    body.visual(
        Box((0.180, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.145, 0.311)),
        material=dark_trim,
        name="control_strip",
    )
    body.visual(
        Box((0.276, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, 0.165, 0.044)),
        material=body_metal,
        name="lower_band",
    )
    body.visual(
        Box((0.092, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.210, 0.224)),
        material=dark_trim,
        name="brew_mount",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.030),
        origin=Origin(xyz=(0.0, 0.215, 0.201), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.215, 0.173)),
        material=stainless,
        name="shower_screen",
    )
    body.visual(
        Box((0.024, 0.040, 0.045)),
        origin=Origin(xyz=(0.156, 0.102, 0.276)),
        material=dark_trim,
        name="steam_mount",
    )
    body.visual(
        Box((0.028, 0.160, 0.016)),
        origin=Origin(xyz=(-0.124, 0.072, 0.082)),
        material=dark_trim,
        name="tray_guide_left",
    )
    body.visual(
        Box((0.028, 0.160, 0.016)),
        origin=Origin(xyz=(0.124, 0.072, 0.082)),
        material=dark_trim,
        name="tray_guide_right",
    )
    body.visual(
        Box((0.220, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, 0.065)),
        material=dark_trim,
        name="tray_stop",
    )
    body.visual(
        Box((0.038, 0.200, 0.018)),
        origin=Origin(xyz=(-0.119, -0.100, 0.244)),
        material=dark_trim,
        name="drawer_guide_left",
    )
    body.visual(
        Box((0.038, 0.200, 0.018)),
        origin=Origin(xyz=(0.119, -0.100, 0.244)),
        material=dark_trim,
        name="drawer_guide_right",
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=stainless,
        name="basket_rim",
    )
    portafilter.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=stainless,
        name="basket_body",
    )
    portafilter.visual(
        Box((0.018, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.028, -0.020)),
        material=stainless,
        name="handle_neck",
    )
    portafilter.visual(
        Cylinder(radius=0.011, length=0.132),
        origin=Origin(xyz=(0.0, 0.090, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="handle_bar",
    )
    portafilter.visual(
        Box((0.026, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.140, -0.020)),
        material=black_plastic,
        name="handle_grip",
    )
    portafilter.visual(
        Box((0.020, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.018, -0.042)),
        material=stainless,
        name="spout_block",
    )
    portafilter.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(-0.007, 0.028, -0.051)),
        material=stainless,
        name="spout_0",
    )
    portafilter.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.007, 0.028, -0.051)),
        material=stainless,
        name="spout_1",
    )

    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.0, 0.215, 0.169)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-math.radians(55.0),
            upper=math.radians(15.0),
        ),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.010, length=0.024),
        material=stainless,
        name="pivot_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.016, -0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="wand_arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.160),
        origin=Origin(xyz=(0.0, 0.030, -0.088)),
        material=stainless,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(-0.006, 0.032, -0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="wand_tip",
    )

    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.178, 0.102, 0.276)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=-1.10,
            upper=0.55,
        ),
    )

    tank_drawer = model.part("tank_drawer")
    tank_drawer.visual(
        Box((0.180, 0.170, 0.006)),
        origin=Origin(xyz=(0.0, -0.085, 0.003)),
        material=black_plastic,
        name="tank_floor",
    )
    tank_drawer.visual(
        Box((0.006, 0.170, 0.110)),
        origin=Origin(xyz=(-0.087, -0.085, 0.055)),
        material=dark_trim,
        name="tank_left",
    )
    tank_drawer.visual(
        Box((0.006, 0.170, 0.110)),
        origin=Origin(xyz=(0.087, -0.085, 0.055)),
        material=dark_trim,
        name="tank_right",
    )
    tank_drawer.visual(
        Box((0.180, 0.008, 0.110)),
        origin=Origin(xyz=(0.0, -0.166, 0.055)),
        material=dark_trim,
        name="tank_back",
    )
    tank_drawer.visual(
        Box((0.180, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -0.005, 0.030)),
        material=dark_trim,
        name="tank_front",
    )
    tank_drawer.visual(
        Box((0.110, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.174, 0.060)),
        material=dark_trim,
        name="tank_handle",
    )
    tank_drawer.visual(
        Box((0.016, 0.150, 0.008)),
        origin=Origin(xyz=(-0.092, -0.090, 0.089)),
        material=dark_trim,
        name="rail_left",
    )
    tank_drawer.visual(
        Box((0.016, 0.150, 0.008)),
        origin=Origin(xyz=(0.092, -0.090, 0.089)),
        material=dark_trim,
        name="rail_right",
    )
    tank_drawer.visual(
        Box((0.164, 0.144, 0.102)),
        origin=Origin(xyz=(0.0, -0.086, 0.057)),
        material=tank_clear,
        name="water_tank",
    )

    model.articulation(
        "body_to_tank_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank_drawer,
        origin=Origin(xyz=(0.0, -0.030, 0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.110,
        ),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.220, 0.140, 0.006)),
        origin=Origin(xyz=(0.0, 0.070, 0.003)),
        material=dark_trim,
        name="tray_floor",
    )
    drip_tray.visual(
        Box((0.008, 0.140, 0.028)),
        origin=Origin(xyz=(-0.106, 0.070, 0.014)),
        material=dark_trim,
        name="tray_left",
    )
    drip_tray.visual(
        Box((0.008, 0.140, 0.028)),
        origin=Origin(xyz=(0.106, 0.070, 0.014)),
        material=dark_trim,
        name="tray_right",
    )
    drip_tray.visual(
        Box((0.220, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.004, 0.014)),
        material=dark_trim,
        name="tray_back",
    )
    drip_tray.visual(
        Box((0.220, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, 0.134, 0.019)),
        material=dark_trim,
        name="tray_front",
    )
    drip_tray.visual(
        Box((0.208, 0.128, 0.004)),
        origin=Origin(xyz=(0.0, 0.070, 0.007)),
        material=stainless,
        name="tray_grate",
    )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.020, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    tank_drawer = object_model.get_part("tank_drawer")
    drip_tray = object_model.get_part("drip_tray")

    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    tank_joint = object_model.get_articulation("body_to_tank_drawer")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    with ctx.pose({portafilter_joint: 0.0}):
        ctx.expect_gap(
            body,
            portafilter,
            axis="z",
            positive_elem="shower_screen",
            negative_elem="basket_rim",
            max_gap=0.001,
            max_penetration=1e-5,
            name="portafilter seats beneath the shower screen",
        )
        ctx.expect_overlap(
            body,
            portafilter,
            axes="xy",
            elem_a="brew_mount",
            elem_b="basket_rim",
            min_overlap=0.055,
            name="portafilter aligns with the group head",
        )

    locked_grip = None
    unlocked_grip = None
    with ctx.pose({portafilter_joint: 0.0}):
        locked_grip = _center_from_aabb(ctx.part_element_world_aabb(portafilter, elem="handle_grip"))
    with ctx.pose({portafilter_joint: portafilter_joint.motion_limits.lower}):
        unlocked_grip = _center_from_aabb(ctx.part_element_world_aabb(portafilter, elem="handle_grip"))
    ctx.check(
        "portafilter handle rotates about the brew axis",
        locked_grip is not None
        and unlocked_grip is not None
        and unlocked_grip[0] > locked_grip[0] + 0.080
        and abs(unlocked_grip[2] - locked_grip[2]) <= 0.003,
        details=f"locked={locked_grip}, unlocked={unlocked_grip}",
    )

    steam_rest = None
    steam_swung = None
    with ctx.pose({steam_joint: 0.0}):
        steam_rest = _center_from_aabb(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    with ctx.pose({steam_joint: steam_joint.motion_limits.upper}):
        steam_swung = _center_from_aabb(ctx.part_element_world_aabb(steam_wand, elem="wand_tip"))
    ctx.check(
        "steam wand pivots around the side support",
        steam_rest is not None
        and steam_swung is not None
        and steam_swung[0] < steam_rest[0] - 0.010
        and abs(steam_swung[2] - steam_rest[2]) <= 0.003,
        details=f"rest={steam_rest}, swung={steam_swung}",
    )

    ctx.expect_within(
        drip_tray,
        body,
        axes="x",
        margin=0.004,
        name="drip tray stays centered between the front guides",
    )
    tray_rest = ctx.part_world_position(drip_tray)
    tray_extended = None
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        ctx.expect_within(
            drip_tray,
            body,
            axes="x",
            margin=0.004,
            name="extended drip tray stays centered between the guides",
        )
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="y",
            min_overlap=0.080,
            name="extended drip tray keeps guide engagement",
        )
        tray_extended = ctx.part_world_position(drip_tray)
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[1] > tray_rest[1] + 0.050,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    ctx.expect_within(
        tank_drawer,
        body,
        axes="xz",
        margin=0.006,
        name="tank drawer nests inside the rear bay",
    )
    tank_rest = ctx.part_world_position(tank_drawer)
    tank_extended = None
    with ctx.pose({tank_joint: tank_joint.motion_limits.upper}):
        ctx.expect_within(
            tank_drawer,
            body,
            axes="xz",
            margin=0.006,
            name="extended tank drawer stays aligned in the rear bay",
        )
        ctx.expect_overlap(
            tank_drawer,
            body,
            axes="y",
            min_overlap=0.060,
            name="extended tank drawer retains insertion",
        )
        tank_extended = ctx.part_world_position(tank_drawer)
    ctx.check(
        "tank drawer slides rearward",
        tank_rest is not None and tank_extended is not None and tank_extended[1] < tank_rest[1] - 0.060,
        details=f"rest={tank_rest}, extended={tank_extended}",
    )

    return ctx.report()


object_model = build_object_model()
