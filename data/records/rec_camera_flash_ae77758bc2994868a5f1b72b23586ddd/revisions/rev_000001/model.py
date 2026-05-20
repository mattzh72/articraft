from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _build_body_shell():
    shell = (
        cq.Workplane("XY")
        .box(0.044, 0.031, 0.094, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .edges(">Z")
        .fillet(0.002)
    )

    control_pocket = (
        cq.Workplane("XY")
        .box(0.032, 0.004, 0.062, centered=(True, True, False))
        .translate((0.0, -0.0135, 0.018))
    )
    return shell.cut(control_pocket)


def _build_head_shell():
    return (
        cq.Workplane("XY")
        .box(0.068, 0.044, 0.036)
        .edges("|Z")
        .fillet(0.0045)
        .edges(">Z")
        .fillet(0.002)
    )


def _add_button(
    model: ArticulatedObject,
    body,
    button_material,
    name: str,
    *,
    joint_xyz: tuple[float, float, float],
    cap_size: tuple[float, float],
) -> None:
    cap_x, cap_z = cap_size
    button = model.part(name)
    button.visual(
        Box((cap_x, 0.0026, cap_z)),
        origin=Origin(xyz=(0.0, -0.0013, 0.0)),
        material=button_material,
        name="cap",
    )
    button.visual(
        Box((cap_x * 0.60, 0.0040, cap_z * 0.60)),
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
        material=button_material,
        name="stem",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=joint_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.04,
            lower=-0.0015,
            upper=0.0,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_flash")

    body_black = model.material("body_black", rgba=(0.13, 0.13, 0.14, 1.0))
    head_black = model.material("head_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_gray = model.material("button_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.36, 0.37, 0.40, 1.0))
    screen_bezel = model.material("screen_bezel", rgba=(0.09, 0.09, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.17, 0.29, 0.33, 0.55))
    diffuser = model.material("diffuser", rgba=(0.95, 0.95, 0.97, 0.96))
    shoe_metal = model.material("shoe_metal", rgba=(0.63, 0.66, 0.70, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "flash_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=body_black,
        name="body_shell",
    )
    body.visual(
        Box((0.024, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.002, 0.115)),
        material=body_black,
        name="neck_collar",
    )
    body.visual(
        Box((0.014, 0.016, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=body_black,
        name="shoe_stem",
    )
    body.visual(
        Box((0.018, 0.020, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=shoe_metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.022, 0.005, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0075, 0.00225)),
        material=shoe_metal,
        name="shoe_rail_front",
    )
    body.visual(
        Box((0.022, 0.005, 0.0015)),
        origin=Origin(xyz=(0.0, -0.0075, 0.00225)),
        material=shoe_metal,
        name="shoe_rail_rear",
    )
    body.visual(
        Box((0.020, 0.0012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0155, 0.061)),
        material=trim_gray,
        name="front_window",
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.024, 0.0008, 0.018)),
        origin=Origin(xyz=(0.0, 0.0004, 0.0)),
        material=screen_bezel,
        name="bezel",
    )
    screen.visual(
        Box((0.018, 0.0006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0011, 0.0)),
        material=screen_glass,
        name="glass",
    )
    model.articulation(
        "body_to_screen",
        ArticulationType.FIXED,
        parent=body,
        child=screen,
        origin=Origin(xyz=(0.0, -0.0155, 0.079)),
    )

    _add_button(
        model,
        body,
        button_gray,
        "top_button",
        joint_xyz=(0.0, -0.0155, 0.090),
        cap_size=(0.010, 0.006),
    )
    _add_button(
        model,
        body,
        button_gray,
        "left_button",
        joint_xyz=(-0.015, -0.0155, 0.079),
        cap_size=(0.006, 0.010),
    )
    _add_button(
        model,
        body,
        button_gray,
        "right_button",
        joint_xyz=(0.015, -0.0155, 0.079),
        cap_size=(0.006, 0.010),
    )
    _add_button(
        model,
        body,
        button_gray,
        "bottom_button",
        joint_xyz=(0.0, -0.0155, 0.068),
        cap_size=(0.010, 0.006),
    )
    _add_button(
        model,
        body,
        button_gray,
        "mode_button",
        joint_xyz=(0.0, -0.0155, 0.053),
        cap_size=(0.013, 0.006),
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=head_black,
        name="turret_base",
    )
    swivel.visual(
        Box((0.020, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=head_black,
        name="swivel_post",
    )
    swivel.visual(
        Box((0.078, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.003, 0.030)),
        material=head_black,
        name="cradle_bridge",
    )
    swivel.visual(
        Box((0.004, 0.020, 0.022)),
        origin=Origin(xyz=(-0.037, 0.006, 0.040)),
        material=head_black,
        name="arm_0",
    )
    swivel.visual(
        Box((0.004, 0.020, 0.022)),
        origin=Origin(xyz=(0.037, 0.006, 0.040)),
        material=head_black,
        name="arm_1",
    )
    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.002, 0.118)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-1.65,
            upper=1.65,
        ),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shell(), "flash_head_shell"),
        origin=Origin(xyz=(0.0, 0.028, 0.013)),
        material=head_black,
        name="head_shell",
    )
    head.visual(
        Box((0.060, 0.003, 0.028)),
        origin=Origin(xyz=(0.0, 0.0505, 0.013)),
        material=diffuser,
        name="diffuser",
    )
    head.visual(
        Box((0.046, 0.018, 0.0015)),
        origin=Origin(xyz=(0.0, 0.026, 0.03125)),
        material=trim_gray,
        name="top_panel",
    )
    head.visual(
        Box((0.020, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, 0.001)),
        material=trim_gray,
        name="hinge_block",
    )
    head.visual(
        Box((0.002, 0.008, 0.008)),
        origin=Origin(xyz=(-0.034, 0.004, 0.002)),
        material=trim_gray,
        name="boss_0",
    )
    head.visual(
        Box((0.002, 0.008, 0.008)),
        origin=Origin(xyz=(0.034, 0.004, 0.002)),
        material=trim_gray,
        name="boss_1",
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.006, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-math.radians(12.0),
            upper=math.radians(78.0),
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("head")
    swivel = object_model.get_part("swivel")
    top_button = object_model.get_part("top_button")
    mode_button = object_model.get_part("mode_button")

    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")
    top_button_joint = object_model.get_articulation("body_to_top_button")
    mode_button_joint = object_model.get_articulation("body_to_mode_button")

    ctx.expect_gap(
        swivel,
        body,
        axis="z",
        positive_elem="turret_base",
        negative_elem="neck_collar",
        min_gap=-1e-6,
        max_gap=0.001,
        name="swivel turret sits on the body collar",
    )
    ctx.expect_gap(
        head,
        swivel,
        axis="z",
        positive_elem="head_shell",
        negative_elem="cradle_bridge",
        min_gap=0.0005,
        max_gap=0.010,
        name="head shell clears the cradle bridge",
    )

    for button_name in (
        "top_button",
        "left_button",
        "right_button",
        "bottom_button",
        "mode_button",
    ):
        button = object_model.get_part(button_name)
        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="body_shell",
            negative_elem="cap",
            min_gap=0.0,
            max_gap=0.0002,
            name=f"{button_name} sits proud of the rear shell",
        )

    tilt_limits = tilt_joint.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            raised_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
        ctx.check(
            "head tilt raises the flash face",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.018,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    swivel_limits = swivel_joint.motion_limits
    if swivel_limits is not None and swivel_limits.upper is not None:
        rest_center = _aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
        with ctx.pose({swivel_joint: swivel_limits.upper}):
            swung_center = _aabb_center(ctx.part_element_world_aabb(head, elem="diffuser"))
        ctx.check(
            "head swivel turns the flash sideways",
            rest_center is not None
            and swung_center is not None
            and abs(swung_center[0] - rest_center[0]) > 0.025,
            details=f"rest={rest_center}, swung={swung_center}",
        )

    top_rest = ctx.part_world_position(top_button)
    top_pressed = None
    top_limits = top_button_joint.motion_limits
    if top_limits is not None and top_limits.lower is not None:
        with ctx.pose({top_button_joint: top_limits.lower}):
            top_pressed = ctx.part_world_position(top_button)
    ctx.check(
        "top button presses inward",
        top_rest is not None and top_pressed is not None and top_pressed[1] > top_rest[1] + 0.001,
        details=f"rest={top_rest}, pressed={top_pressed}",
    )

    mode_rest = ctx.part_world_position(mode_button)
    mode_pressed = None
    mode_limits = mode_button_joint.motion_limits
    if mode_limits is not None and mode_limits.lower is not None:
        with ctx.pose({mode_button_joint: mode_limits.lower}):
            mode_pressed = ctx.part_world_position(mode_button)
    ctx.check(
        "mode button presses inward",
        mode_rest is not None and mode_pressed is not None and mode_pressed[1] > mode_rest[1] + 0.001,
        details=f"rest={mode_rest}, pressed={mode_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
