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
    model = ArticulatedObject(name="espresso_machine_with_grinder")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    black = model.material("black", rgba=(0.09, 0.09, 0.10, 1.0))
    smoke = model.material("smoke", rgba=(0.28, 0.31, 0.34, 0.42))

    body = model.part("body")
    body.visual(
        Box((0.31, 0.38, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=stainless,
        name="base_shell",
    )
    body.visual(
        Box((0.28, 0.34, 0.06)),
        origin=Origin(xyz=(-0.005, 0.0, 0.31)),
        material=stainless,
        name="top_shoulder",
    )
    body.visual(
        Box((0.018, 0.33, 0.18)),
        origin=Origin(xyz=(0.146, 0.0, 0.18)),
        material=graphite,
        name="front_panel",
    )
    body.visual(
        Box((0.23, 0.25, 0.035)),
        origin=Origin(xyz=(0.11, 0.0, 0.0375)),
        material=black,
        name="drip_tray",
    )
    body.visual(
        Box((0.16, 0.14, 0.12)),
        origin=Origin(xyz=(-0.01, -0.10, 0.40)),
        material=graphite,
        name="grinder_tower",
    )
    body.visual(
        Box((0.12, 0.10, 0.02)),
        origin=Origin(xyz=(-0.01, -0.10, 0.47)),
        material=black,
        name="hopper_frame",
    )
    body.visual(
        Box((0.008, 0.094, 0.09)),
        origin=Origin(xyz=(-0.062, -0.10, 0.525)),
        material=smoke,
        name="hopper_rear_wall",
    )
    body.visual(
        Box((0.008, 0.094, 0.09)),
        origin=Origin(xyz=(0.042, -0.10, 0.525)),
        material=smoke,
        name="hopper_front_wall",
    )
    body.visual(
        Box((0.104, 0.008, 0.09)),
        origin=Origin(xyz=(-0.01, -0.143, 0.525)),
        material=smoke,
        name="hopper_left_wall",
    )
    body.visual(
        Box((0.104, 0.008, 0.09)),
        origin=Origin(xyz=(-0.01, -0.057, 0.525)),
        material=smoke,
        name="hopper_right_wall",
    )
    body.visual(
        Box((0.050, 0.030, 0.015)),
        origin=Origin(xyz=(0.170, -0.11, 0.235)),
        material=black,
        name="grinder_spout",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.05),
        origin=Origin(xyz=(0.173, 0.035, 0.222), rpy=(0.0, pi / 2, 0.0)),
        material=black,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.150, 0.035, 0.188), rpy=(0.0, pi / 2, 0.0)),
        material=black,
        name="group_flange",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.125, 0.188, 0.255), rpy=(pi / 2, 0.0, 0.0)),
        material=black,
        name="wand_mount",
    )

    hopper_lid = model.part("hopper_lid")
    hopper_lid.visual(
        Box((0.112, 0.102, 0.008)),
        origin=Origin(xyz=(0.056, 0.0, 0.004)),
        material=black,
        name="panel",
    )
    hopper_lid.visual(
        Box((0.015, 0.050, 0.012)),
        origin=Origin(xyz=(0.104, 0.0, 0.010)),
        material=graphite,
        name="lip",
    )
    model.articulation(
        "body_to_hopper_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hopper_lid,
        origin=Origin(xyz=(-0.062, -0.10, 0.57)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=stainless,
        name="basket",
    )
    portafilter.visual(
        Box((0.030, 0.016, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, -0.010)),
        material=black,
        name="lug_block",
    )
    portafilter.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(0.090, 0.0, -0.018), rpy=(0.0, pi / 2, 0.0)),
        material=black,
        name="handle",
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.191, 0.035, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.9, upper=0.0),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.04),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=black,
        name="arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.17),
        origin=Origin(xyz=(0.0, 0.040, -0.085)),
        material=stainless,
        name="tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.003, length=0.025),
        origin=Origin(xyz=(0.010, 0.040, -0.170), rpy=(0.0, pi / 2, 0.0)),
        material=stainless,
        name="tip",
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.125, 0.196, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.15, upper=0.25),
    )

    brew_paddle = model.part("brew_paddle")
    brew_paddle.visual(
        Box((0.020, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=black,
        name="mount",
    )
    brew_paddle.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=black,
        name="hub",
    )
    brew_paddle.visual(
        Box((0.040, 0.016, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, -0.003)),
        material=black,
        name="lever",
    )
    brew_paddle.visual(
        Box((0.014, 0.020, 0.028)),
        origin=Origin(xyz=(0.040, 0.0, -0.010)),
        material=graphite,
        name="tab",
    )
    model.articulation(
        "body_to_brew_paddle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=brew_paddle,
        origin=Origin(xyz=(0.165, -0.045, 0.226)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-0.35, upper=0.55),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hopper_lid = object_model.get_part("hopper_lid")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    brew_paddle = object_model.get_part("brew_paddle")

    lid_joint = object_model.get_articulation("body_to_hopper_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    paddle_joint = object_model.get_articulation("body_to_brew_paddle")

    ctx.expect_overlap(
        hopper_lid,
        body,
        axes="xy",
        elem_a="panel",
        elem_b="hopper_frame",
        min_overlap=0.09,
        name="hopper lid covers the bean hopper opening",
    )
    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_head",
        negative_elem="basket",
        max_gap=0.001,
        max_penetration=0.0,
        name="portafilter seats against the group head",
    )
    ctx.expect_gap(
        steam_wand,
        body,
        axis="y",
        positive_elem="tube",
        negative_elem="base_shell",
        min_gap=0.02,
        name="steam wand parks outside the side panel",
    )

    lid_closed = ctx.part_element_world_aabb(hopper_lid, elem="panel")
    with ctx.pose({lid_joint: 1.0}):
        lid_open = ctx.part_element_world_aabb(hopper_lid, elem="panel")
    ctx.check(
        "hopper lid opens upward from the rear hinge",
        lid_closed is not None and lid_open is not None and lid_open[1][2] > lid_closed[1][2] + 0.05,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    handle_center_rest = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({portafilter_joint: -0.75}):
        handle_center_turned = _aabb_center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter handle rotates about the vertical brew axis",
        handle_center_rest is not None
        and handle_center_turned is not None
        and abs(handle_center_turned[1] - handle_center_rest[1]) > 0.045,
        details=f"rest={handle_center_rest}, turned={handle_center_turned}",
    )

    wand_center_rest = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tube"))
    with ctx.pose({wand_joint: -0.95}):
        wand_center_swung = _aabb_center(ctx.part_element_world_aabb(steam_wand, elem="tube"))
    ctx.check(
        "steam wand swings forward on its vertical pivot",
        wand_center_rest is not None
        and wand_center_swung is not None
        and wand_center_swung[0] > wand_center_rest[0] + 0.02,
        details=f"rest={wand_center_rest}, swung={wand_center_swung}",
    )

    paddle_center_rest = _aabb_center(ctx.part_element_world_aabb(brew_paddle, elem="tab"))
    with ctx.pose({paddle_joint: 0.45}):
        paddle_center_raised = _aabb_center(ctx.part_element_world_aabb(brew_paddle, elem="tab"))
    ctx.check(
        "brew paddle lifts from its short front pivot",
        paddle_center_rest is not None
        and paddle_center_raised is not None
        and paddle_center_raised[2] > paddle_center_rest[2] + 0.01,
        details=f"rest={paddle_center_rest}, raised={paddle_center_raised}",
    )

    return ctx.report()


object_model = build_object_model()
