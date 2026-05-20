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
    TireGeometry,
    WheelGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.36
BODY_D = 0.31
BODY_H = 0.72
FOOT_H = 0.012

REAR_FACE_Y = -BODY_D * 0.5
WHEEL_RADIUS = 0.060
WHEEL_WIDTH = 0.032
WHEEL_CENTER_Z = FOOT_H + WHEEL_RADIUS
WHEEL_CENTER_Y = REAR_FACE_Y - (WHEEL_RADIUS - 0.001)
WHEEL_CENTER_X = 0.142

GUIDE_CENTER_Y = -0.162
GUIDE_HALF_SPAN = 0.114
GUIDE_BOTTOM_Z = 0.455
GUIDE_TOP_Z = 0.700
GUIDE_HEIGHT = GUIDE_TOP_Z - GUIDE_BOTTOM_Z
GUIDE_CENTER_Z = GUIDE_BOTTOM_Z + GUIDE_HEIGHT * 0.5

HANDLE_TRAVEL = 0.180
HANDLE_RAIL_LENGTH = 0.520
HANDLE_RAIL_W = 0.013
HANDLE_RAIL_D = 0.008
HANDLE_RAIL_CENTER_Z = 0.015

DECK_ROLL = -0.38
DECK_CENTER = (0.0, 0.070, 0.704)
DECK_SIZE = (0.190, 0.100, 0.022)


def _rotate_x(point: tuple[float, float, float], roll: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(roll)
    s = math.sin(roll)
    return (x, c * y - s * z, s * y + c * z)


def _panel_origin(x_local: float, y_local: float, proud: float = 0.0) -> Origin:
    dx, dy, dz = _rotate_x((x_local, y_local, DECK_SIZE[2] * 0.5 + proud), DECK_ROLL)
    return Origin(
        xyz=(DECK_CENTER[0] + dx, DECK_CENTER[1] + dy, DECK_CENTER[2] + dz),
        rpy=(DECK_ROLL, 0.0, 0.0),
    )


def _speaker_shell() -> object:
    return (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .edges(">Z")
        .fillet(0.014)
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((a + b) * 0.5 for a, b in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_trolley_speaker")

    cabinet = model.material("cabinet", rgba=(0.13, 0.13, 0.14, 1.0))
    grille = model.material("grille", rgba=(0.07, 0.07, 0.08, 1.0))
    trim = model.material("trim", rgba=(0.19, 0.20, 0.22, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.38, 0.40, 0.43, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.63, 0.66, 1.0))
    button_black = model.material("button_black", rgba=(0.12, 0.12, 0.13, 1.0))
    button_grey = model.material("button_grey", rgba=(0.25, 0.26, 0.28, 1.0))

    wheel_core_mesh = mesh_from_geometry(
        WheelGeometry(0.047, 0.024),
        "speaker_wheel_core",
    )
    wheel_tire_mesh = mesh_from_geometry(
        TireGeometry(0.060, 0.032, inner_radius=0.046),
        "speaker_wheel_tire",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_speaker_shell(), "speaker_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, FOOT_H)),
        material=cabinet,
        name="cabinet_shell",
    )
    body.visual(
        Box((0.304, 0.014, 0.548)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.006, 0.376)),
        material=grille,
        name="front_grille",
    )
    body.visual(
        Cylinder(radius=0.112, length=0.012),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - 0.009, 0.438),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="upper_driver",
    )
    body.visual(
        Cylinder(radius=0.096, length=0.012),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - 0.009, 0.234),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="lower_driver",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - 0.010, 0.598),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim,
        name="tweeter",
    )
    body.visual(
        Box((0.164, 0.010, 0.042)),
        origin=Origin(xyz=(0.0, BODY_D * 0.5 - 0.010, 0.094)),
        material=grille,
        name="bass_port",
    )
    body.visual(
        Box(DECK_SIZE),
        origin=Origin(xyz=DECK_CENTER, rpy=(DECK_ROLL, 0.0, 0.0)),
        material=trim,
        name="control_deck",
    )
    body.visual(
        Box((0.026, 0.055, 0.016)),
        origin=Origin(xyz=(-0.114, 0.120, 0.008)),
        material=dark_rubber,
        name="front_foot_0",
    )
    body.visual(
        Box((0.026, 0.055, 0.016)),
        origin=Origin(xyz=(0.114, 0.120, 0.008)),
        material=dark_rubber,
        name="front_foot_1",
    )
    body.visual(
        Box((0.028, 0.056, 0.020)),
        origin=Origin(xyz=(-0.103, -0.182, 0.082)),
        material=trim,
        name="wheel_bracket_0",
    )
    body.visual(
        Box((0.028, 0.056, 0.020)),
        origin=Origin(xyz=(0.103, -0.182, 0.082)),
        material=trim,
        name="wheel_bracket_1",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(
            xyz=(-0.116, WHEEL_CENTER_Y, WHEEL_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="wheel_boss_0",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(
            xyz=(0.116, WHEEL_CENTER_Y, WHEEL_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="wheel_boss_1",
    )

    left_guide_x = -GUIDE_HALF_SPAN
    right_guide_x = GUIDE_HALF_SPAN

    body.visual(
        Box((0.024, 0.004, GUIDE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x, -0.171, GUIDE_CENTER_Z)),
        material=gunmetal,
        name="left_sleeve_back",
    )
    body.visual(
        Box((0.004, 0.020, GUIDE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x + 0.010, -0.164, GUIDE_CENTER_Z)),
        material=gunmetal,
        name="left_sleeve_inner",
    )
    body.visual(
        Box((0.004, 0.020, GUIDE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x - 0.010, -0.164, GUIDE_CENTER_Z)),
        material=gunmetal,
        name="left_sleeve_outer",
    )
    body.visual(
        Box((0.030, 0.028, 0.028)),
        origin=Origin(xyz=(left_guide_x, -0.163, GUIDE_BOTTOM_Z - 0.014)),
        material=trim,
        name="left_guide_base",
    )

    body.visual(
        Box((0.024, 0.004, GUIDE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x, -0.171, GUIDE_CENTER_Z)),
        material=gunmetal,
        name="right_sleeve_back",
    )
    body.visual(
        Box((0.004, 0.020, GUIDE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x - 0.010, -0.164, GUIDE_CENTER_Z)),
        material=gunmetal,
        name="right_sleeve_inner",
    )
    body.visual(
        Box((0.004, 0.020, GUIDE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x + 0.010, -0.164, GUIDE_CENTER_Z)),
        material=gunmetal,
        name="right_sleeve_outer",
    )
    body.visual(
        Box((0.030, 0.028, 0.028)),
        origin=Origin(xyz=(right_guide_x, -0.163, GUIDE_BOTTOM_Z - 0.014)),
        material=trim,
        name="right_guide_base",
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Box((HANDLE_RAIL_W, HANDLE_RAIL_D, HANDLE_RAIL_LENGTH)),
        origin=Origin(xyz=(-GUIDE_HALF_SPAN, 0.0, HANDLE_RAIL_CENTER_Z)),
        material=steel,
        name="left_rail",
    )
    pull_handle.visual(
        Box((HANDLE_RAIL_W, HANDLE_RAIL_D, HANDLE_RAIL_LENGTH)),
        origin=Origin(xyz=(GUIDE_HALF_SPAN, 0.0, HANDLE_RAIL_CENTER_Z)),
        material=steel,
        name="right_rail",
    )
    pull_handle.visual(
        Box((0.270, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=steel,
        name="handle_crossbar",
    )
    pull_handle.visual(
        Box((0.210, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.299)),
        material=dark_rubber,
        name="handle_grip",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(wheel_tire_mesh, material=dark_rubber, name="tire")
    left_wheel.visual(wheel_core_mesh, material=gunmetal, name="rim")
    left_wheel.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(wheel_tire_mesh, material=dark_rubber, name="tire")
    right_wheel.visual(wheel_core_mesh, material=gunmetal, name="rim")
    right_wheel.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.0036, length=0.022),
        origin=Origin(xyz=(0.0, -0.003, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_black,
        name="pivot_barrel",
    )
    power_rocker.visual(
        Box((0.030, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, -0.002, 0.001)),
        material=button_black,
        name="rocker_base",
    )
    power_rocker.visual(
        Box((0.028, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, 0.004)),
        material=button_grey,
        name="rocker_cap",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        Box((0.012, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button_grey,
        name="button_cap",
    )
    menu_button_0.visual(
        Box((0.007, 0.007, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=button_black,
        name="button_stem",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        Box((0.012, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button_grey,
        name="button_cap",
    )
    menu_button_1.visual(
        Box((0.007, 0.007, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=button_black,
        name="button_stem",
    )

    model.articulation(
        "body_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull_handle,
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, GUIDE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=24.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=24.0),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=_panel_origin(-0.020, -0.006),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-0.26,
            upper=0.26,
        ),
    )
    model.articulation(
        "body_to_menu_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_0,
        origin=_panel_origin(0.018, -0.006),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0032,
        ),
    )
    model.articulation(
        "body_to_menu_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=menu_button_1,
        origin=_panel_origin(0.045, -0.006),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0032,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pull_handle = object_model.get_part("pull_handle")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")

    handle_slide = object_model.get_articulation("body_to_pull_handle")
    left_wheel_spin = object_model.get_articulation("body_to_left_wheel")
    right_wheel_spin = object_model.get_articulation("body_to_right_wheel")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    button_joint_0 = object_model.get_articulation("body_to_menu_button_0")
    button_joint_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.allow_overlap(
        power_rocker,
        body,
        reason="The rocker's pivot barrel embeds into a simplified solid control-deck housing.",
    )
    ctx.allow_overlap(
        menu_button_0,
        body,
        reason="The menu button's switch stem intentionally enters the simplified control-deck housing.",
    )
    ctx.allow_overlap(
        menu_button_1,
        body,
        reason="The menu button's switch stem intentionally enters the simplified control-deck housing.",
    )

    ctx.check(
        "requested articulation families are present",
        handle_slide.articulation_type == ArticulationType.PRISMATIC
        and left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rocker_joint.articulation_type == ArticulationType.REVOLUTE
        and button_joint_0.articulation_type == ArticulationType.PRISMATIC
        and button_joint_1.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"handle={handle_slide.articulation_type}, "
            f"left_wheel={left_wheel_spin.articulation_type}, "
            f"right_wheel={right_wheel_spin.articulation_type}, "
            f"rocker={rocker_joint.articulation_type}, "
            f"button_0={button_joint_0.articulation_type}, "
            f"button_1={button_joint_1.articulation_type}"
        ),
    )

    ctx.expect_gap(
        menu_button_0,
        power_rocker,
        axis="x",
        min_gap=0.004,
        name="power rocker stays distinct from the first menu button",
    )
    ctx.expect_gap(
        menu_button_1,
        menu_button_0,
        axis="x",
        min_gap=0.010,
        name="the two menu buttons remain visibly separate",
    )

    handle_limits = handle_slide.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        ctx.expect_overlap(
            pull_handle,
            body,
            axes="z",
            elem_a="left_rail",
            elem_b="left_sleeve_back",
            min_overlap=0.220,
            name="collapsed pull handle remains seated in the rear guide",
        )
        rest_handle_pos = ctx.part_world_position(pull_handle)
        with ctx.pose({handle_slide: handle_limits.upper}):
            ctx.expect_overlap(
                pull_handle,
                body,
                axes="z",
                elem_a="left_rail",
                elem_b="left_sleeve_back",
                min_overlap=0.055,
                name="extended pull handle still keeps retained insertion in the rear guide",
            )
            extended_handle_pos = ctx.part_world_position(pull_handle)
        ctx.check(
            "pull handle extends upward",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > rest_handle_pos[2] + 0.10,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    button_limits_0 = button_joint_0.motion_limits
    if button_limits_0 is not None and button_limits_0.upper is not None:
        rest_button_0 = ctx.part_world_position(menu_button_0)
        rest_button_1 = ctx.part_world_position(menu_button_1)
        with ctx.pose({button_joint_0: button_limits_0.upper}):
            pushed_button_0 = ctx.part_world_position(menu_button_0)
            untouched_button_1 = ctx.part_world_position(menu_button_1)
        ctx.check(
            "menu button 0 depresses without dragging menu button 1",
            rest_button_0 is not None
            and pushed_button_0 is not None
            and rest_button_1 is not None
            and untouched_button_1 is not None
            and pushed_button_0[2] < rest_button_0[2] - 0.002
            and abs(untouched_button_1[0] - rest_button_1[0]) < 1e-6
            and abs(untouched_button_1[1] - rest_button_1[1]) < 1e-6
            and abs(untouched_button_1[2] - rest_button_1[2]) < 1e-6,
            details=(
                f"button_0_rest={rest_button_0}, button_0_pushed={pushed_button_0}, "
                f"button_1_rest={rest_button_1}, button_1_during={untouched_button_1}"
            ),
        )

    button_limits_1 = button_joint_1.motion_limits
    if button_limits_1 is not None and button_limits_1.upper is not None:
        rest_button_0 = ctx.part_world_position(menu_button_0)
        rest_button_1 = ctx.part_world_position(menu_button_1)
        with ctx.pose({button_joint_1: button_limits_1.upper}):
            untouched_button_0 = ctx.part_world_position(menu_button_0)
            pushed_button_1 = ctx.part_world_position(menu_button_1)
        ctx.check(
            "menu button 1 depresses without dragging menu button 0",
            rest_button_0 is not None
            and pushed_button_1 is not None
            and rest_button_1 is not None
            and untouched_button_0 is not None
            and pushed_button_1[2] < rest_button_1[2] - 0.002
            and abs(untouched_button_0[0] - rest_button_0[0]) < 1e-6
            and abs(untouched_button_0[1] - rest_button_0[1]) < 1e-6
            and abs(untouched_button_0[2] - rest_button_0[2]) < 1e-6,
            details=(
                f"button_1_rest={rest_button_1}, button_1_pushed={pushed_button_1}, "
                f"button_0_rest={rest_button_0}, button_0_during={untouched_button_0}"
            ),
        )

    rocker_limits = rocker_joint.motion_limits
    if (
        rocker_limits is not None
        and rocker_limits.lower is not None
        and rocker_limits.upper is not None
    ):
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            lower_cap_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            upper_cap_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        lower_cap_center = _aabb_center(lower_cap_aabb)
        upper_cap_center = _aabb_center(upper_cap_aabb)
        ctx.check(
            "power rocker tilts around its short pivot",
            lower_cap_center is not None
            and upper_cap_center is not None
            and (
                abs(lower_cap_center[1] - upper_cap_center[1]) > 0.001
                or abs(lower_cap_center[2] - upper_cap_center[2]) > 0.001
            ),
            details=f"lower={lower_cap_center}, upper={upper_cap_center}",
        )

    return ctx.report()


object_model = build_object_model()
