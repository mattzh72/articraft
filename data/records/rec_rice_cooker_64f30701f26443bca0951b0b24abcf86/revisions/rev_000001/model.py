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


BODY_W = 0.218
BODY_D = 0.192
BODY_H = 0.116
BODY_CORNER = 0.026
BODY_TOP_EDGE = 0.013

CONTROL_POD_W = 0.110
CONTROL_POD_D = 0.016
CONTROL_POD_H = 0.058
CONTROL_POD_Y = BODY_D / 2.0 + CONTROL_POD_D / 2.0
CONTROL_POD_Z = 0.040
CONTROL_FRONT_Y = CONTROL_POD_Y + CONTROL_POD_D / 2.0

LID_W = 0.196
LID_D = 0.176
LID_H = 0.036
LID_OFFSET_Y = 0.078
HINGE_Y = -0.076
HINGE_Z = BODY_H

HANDLE_PIVOT_X = BODY_W / 2.0 + 0.014
HANDLE_PIVOT_Z = BODY_H - 0.008
HANDLE_SPAN = HANDLE_PIVOT_X * 2.0


def _rounded_box(
    width: float,
    depth: float,
    height: float,
    *,
    corner_fillet: float,
    top_fillet: float,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    shape = shape.edges("|Z").fillet(corner_fillet)
    shape = shape.edges(">Z").fillet(top_fillet)
    return shape


def _build_body_shell() -> cq.Workplane:
    outer = _rounded_box(
        BODY_W,
        BODY_D,
        BODY_H,
        corner_fillet=BODY_CORNER,
        top_fillet=BODY_TOP_EDGE,
    )
    cavity = _rounded_box(
        0.166,
        0.144,
        0.110,
        corner_fillet=0.018,
        top_fillet=0.010,
    ).translate((0.0, 0.0, 0.018))
    return outer.cut(cavity)


def _build_lid_shell() -> cq.Workplane:
    return _rounded_box(
        LID_W,
        LID_D,
        LID_H,
        corner_fillet=0.020,
        top_fillet=0.012,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.63, 0.64, 0.66, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.22, 0.23, 0.24, 1.0))
    display_glass = model.material("display_glass", rgba=(0.14, 0.25, 0.30, 0.45))
    button_red = model.material("button_red", rgba=(0.73, 0.20, 0.18, 1.0))
    button_light = model.material("button_light", rgba=(0.86, 0.87, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "rice_cooker_body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((CONTROL_POD_W, CONTROL_POD_D, CONTROL_POD_H)),
        origin=Origin(xyz=(0.0, CONTROL_POD_Y, CONTROL_POD_Z)),
        material=trim_gray,
        name="control_pod",
    )
    body.visual(
        Box((0.060, 0.005, 0.026)),
        origin=Origin(xyz=(0.0, CONTROL_FRONT_Y - 0.0025, 0.044)),
        material=trim_gray,
        name="display_bezel",
    )
    body.visual(
        Box((0.047, 0.0016, 0.017)),
        origin=Origin(xyz=(0.0, CONTROL_FRONT_Y + 0.0002, 0.044)),
        material=display_glass,
        name="display_window",
    )
    for idx, x_pos in enumerate((-0.026, 0.026)):
        body.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(
                xyz=(x_pos, CONTROL_FRONT_Y, 0.020),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=button_light,
            name=f"mode_button_{idx}",
        )
    for side, x_pos in (("left", -0.116), ("right", 0.116)):
        body.visual(
            Box((0.016, 0.012, 0.020)),
            origin=Origin(xyz=(x_pos, -0.012, 0.113)),
            material=trim_gray,
            name=f"handle_support_{side}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "rice_cooker_lid"),
        origin=Origin(xyz=(0.0, LID_OFFSET_Y, 0.0)),
        material=shell_white,
        name="lid_panel",
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        Cylinder(radius=0.0075, length=HANDLE_SPAN - 0.032),
        origin=Origin(
            xyz=(HANDLE_SPAN / 2.0, 0.0, 0.086),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_gray,
        name="grip_bar",
    )
    carry_handle.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(0.0085, 0.0, 0.0435), rpy=(0.0, 0.195, 0.0)),
        material=handle_gray,
        name="left_arm",
    )
    carry_handle.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(HANDLE_SPAN - 0.0085, 0.0, 0.0435), rpy=(0.0, -0.195, 0.0)),
        material=handle_gray,
        name="right_arm",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.0105, length=0.002),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="button_bezel",
    )
    power_button.visual(
        Cylinder(radius=0.0085, length=0.0055),
        origin=Origin(xyz=(0.0, 0.00275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_red,
        name="power_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "body_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_handle,
        origin=Origin(xyz=(-HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.0, CONTROL_FRONT_Y, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    carry_handle = object_model.get_part("carry_handle")
    power_button = object_model.get_part("power_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_hinge = object_model.get_articulation("body_to_carry_handle")
    button_slide = object_model.get_articulation("body_to_power_button")

    body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check("travel_size_body_present", body_aabb is not None, "Body shell should produce a valid AABB.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "travel_size_scale",
            0.20 <= size[0] <= 0.26 and 0.18 <= size[1] <= 0.21 and 0.10 <= size[2] <= 0.14,
            details=f"body_size={size!r}",
        )

    with ctx.pose({lid_hinge: 0.0, handle_hinge: 0.0, button_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed_lid_sits_just_above_body",
        )
        ctx.expect_gap(
            carry_handle,
            lid,
            axis="z",
            positive_elem="grip_bar",
            negative_elem="lid_panel",
            min_gap=0.008,
            name="folded_handle_stays_clear_of_lid",
        )

        display_aabb = ctx.part_element_world_aabb(body, elem="display_window")
        button_aabb = ctx.part_element_world_aabb(power_button, elem="power_cap")
        power_below_display = False
        if display_aabb is not None and button_aabb is not None:
            display_mins, display_maxs = display_aabb
            button_mins, button_maxs = button_aabb
            display_center_x = 0.5 * (display_mins[0] + display_maxs[0])
            button_center_x = 0.5 * (button_mins[0] + button_maxs[0])
            power_below_display = (
                button_maxs[2] < display_mins[2] - 0.003
                and abs(button_center_x - display_center_x) < 0.004
            )
        ctx.check(
            "power_button_sits_below_display",
            power_below_display,
            details=f"display_aabb={display_aabb!r}, button_aabb={button_aabb!r}",
        )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0}):
            closed_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
        lid_opens = False
        if closed_lid is not None and open_lid is not None:
            lid_opens = open_lid[1][2] > closed_lid[1][2] + 0.050
        ctx.check(
            "lid_rotates_upward",
            lid_opens,
            details=f"closed_lid={closed_lid!r}, open_lid={open_lid!r}",
        )

    handle_limits = handle_hinge.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_hinge: 0.0}):
            folded_handle = ctx.part_element_world_aabb(carry_handle, elem="grip_bar")
        with ctx.pose({handle_hinge: handle_limits.upper}):
            raised_handle = ctx.part_element_world_aabb(carry_handle, elem="grip_bar")
        handle_raises = False
        if folded_handle is not None and raised_handle is not None:
            handle_raises = raised_handle[1][2] > folded_handle[1][2] + 0.050
        ctx.check(
            "carry_handle_folds_up",
            handle_raises,
            details=f"folded_handle={folded_handle!r}, raised_handle={raised_handle!r}",
        )

    button_limits = button_slide.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_pos = ctx.part_world_position(power_button)
        with ctx.pose({button_slide: button_limits.upper}):
            pressed_pos = ctx.part_world_position(power_button)
        button_presses = False
        if rest_pos is not None and pressed_pos is not None:
            button_presses = pressed_pos[1] < rest_pos[1] - 0.0015
        ctx.check(
            "power_button_presses_inward",
            button_presses,
            details=f"rest_pos={rest_pos!r}, pressed_pos={pressed_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
