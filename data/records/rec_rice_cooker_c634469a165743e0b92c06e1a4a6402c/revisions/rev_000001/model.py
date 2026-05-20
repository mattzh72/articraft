from __future__ import annotations

from math import pi

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


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .ellipse(0.148, 0.120)
        .workplane(offset=0.082)
        .ellipse(0.161, 0.129)
        .workplane(offset=0.088)
        .ellipse(0.166, 0.133)
        .loft(combine=True)
    )
    front_flat = cq.Workplane("XY").box(0.248, 0.120, 0.118).translate((0.0, -0.200, 0.094))
    inner_cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.030)
        .ellipse(0.118, 0.094)
        .workplane(offset=0.148)
        .ellipse(0.126, 0.100)
        .loft(combine=True)
    )
    return outer.cut(front_flat).cut(inner_cavity)


def _inner_liner_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.030)
        .ellipse(0.121, 0.097)
        .workplane(offset=0.143)
        .ellipse(0.128, 0.103)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.033)
        .ellipse(0.118, 0.094)
        .workplane(offset=0.143)
        .ellipse(0.125, 0.100)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _front_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.228, 0.014, 0.114)
    display_pocket = cq.Workplane("XY").box(0.118, 0.004, 0.036).translate((0.0, -0.005, 0.022))
    button_pocket = cq.Workplane("XY").box(0.036, 0.004, 0.014).translate((0.0, -0.005, -0.026))
    button_slot = cq.Workplane("XY").box(0.030, 0.020, 0.014).translate((0.0, 0.0, -0.026))
    return panel.cut(display_pocket).cut(button_pocket).cut(button_slot)


def _lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .ellipse(0.154, 0.118)
        .workplane(offset=0.036)
        .ellipse(0.145, 0.109)
        .loft(combine=True)
        .translate((0.0, -0.057, 0.0))
    )
    handle_recess = cq.Workplane("XY").box(0.078, 0.030, 0.012).translate((0.0, -0.108, 0.030))
    vent_pocket = cq.Workplane("XY").ellipse(0.020, 0.011).extrude(0.006).translate((0.0, -0.008, 0.030))
    vent_slot = cq.Workplane("XY").box(0.012, 0.004, 0.008).translate((0.0, -0.008, 0.031))
    return lid.cut(handle_recess).cut(vent_pocket).cut(vent_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_rice_cooker")

    white = model.material("white_shell", rgba=(0.93, 0.94, 0.94, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.20, 0.22, 0.25, 1.0))
    screen_black = model.material("screen_black", rgba=(0.06, 0.07, 0.08, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    power_red = model.material("power_red", rgba=(0.78, 0.34, 0.28, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_inner_liner_shape(), "inner_liner"),
        material=stainless,
        name="inner_liner",
    )
    body.visual(
        Box((0.020, 0.018, 0.026)),
        origin=Origin(xyz=(-0.172, 0.075, 0.182)),
        material=warm_gray,
        name="shoulder_pivot_0",
    )
    body.visual(
        Box((0.038, 0.024, 0.022)),
        origin=Origin(xyz=(-0.154, 0.072, 0.160)),
        material=warm_gray,
        name="shoulder_bridge_0",
    )
    body.visual(
        Box((0.020, 0.018, 0.026)),
        origin=Origin(xyz=(0.172, 0.075, 0.182)),
        material=warm_gray,
        name="shoulder_pivot_1",
    )
    body.visual(
        Box((0.038, 0.024, 0.022)),
        origin=Origin(xyz=(0.154, 0.072, 0.160)),
        material=warm_gray,
        name="shoulder_bridge_1",
    )
    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_cadquery(_front_panel_shape(), "front_panel"),
        material=dark_panel,
        name="panel_shell",
    )
    front_panel.visual(
        Box((0.104, 0.0020, 0.028)),
        origin=Origin(xyz=(0.0, -0.0032, 0.022)),
        material=screen_black,
        name="display",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid"),
        material=white,
        name="lid_shell",
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        Box((0.308, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.080, 0.090)),
        material=warm_gray,
        name="carry_bar",
    )
    carry_handle.visual(
        Box((0.018, 0.108, 0.014)),
        origin=Origin(xyz=(-0.154, 0.040, 0.060), rpy=(0.75, 0.0, 0.0)),
        material=warm_gray,
        name="handle_arm_0",
    )
    carry_handle.visual(
        Box((0.018, 0.108, 0.014)),
        origin=Origin(xyz=(0.154, 0.040, 0.060), rpy=(0.75, 0.0, 0.0)),
        material=warm_gray,
        name="handle_arm_1",
    )
    carry_handle.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.162, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_gray,
        name="handle_pivot_0",
    )
    carry_handle.visual(
        Box((0.018, 0.024, 0.014)),
        origin=Origin(xyz=(-0.163, 0.003, 0.014)),
        material=warm_gray,
        name="handle_knuckle_0",
    )
    carry_handle.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.162, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_gray,
        name="handle_pivot_1",
    )
    carry_handle.visual(
        Box((0.018, 0.024, 0.014)),
        origin=Origin(xyz=(0.163, 0.003, 0.014)),
        material=warm_gray,
        name="handle_knuckle_1",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.044, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material=power_red,
        name="button_cap",
    )
    power_button.visual(
        Box((0.028, 0.011, 0.012)),
        origin=Origin(xyz=(0.0, 0.0025, 0.0)),
        material=power_red,
        name="button_stem",
    )

    model.articulation(
        "body_to_front_panel",
        ArticulationType.FIXED,
        parent=body,
        child=front_panel,
        origin=Origin(xyz=(0.0, -0.1395, 0.094)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.102, 0.171)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.42),
    )
    model.articulation(
        "body_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.075, 0.186)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "front_panel_to_power_button",
        ArticulationType.PRISMATIC,
        parent=front_panel,
        child=power_button,
        origin=Origin(xyz=(0.0, -0.007, -0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.04, lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_panel = object_model.get_part("front_panel")
    lid = object_model.get_part("lid")
    carry_handle = object_model.get_part("carry_handle")
    power_button = object_model.get_part("power_button")
    lid_joint = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_carry_handle")
    button_joint = object_model.get_articulation("front_panel_to_power_button")

    ctx.expect_overlap(
        front_panel,
        body,
        axes="xz",
        min_overlap=0.10,
        name="front panel spans the cooker face",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.18,
        name="closed lid covers the top opening",
    )
    ctx.expect_gap(
        carry_handle,
        lid,
        axis="z",
        positive_elem="carry_bar",
        min_gap=0.006,
        name="stowed carry handle clears the lid crown",
    )
    ctx.expect_overlap(
        power_button,
        front_panel,
        axes="xz",
        min_overlap=0.014,
        name="power button sits within the front control panel",
    )
    ctx.allow_overlap(
        body,
        carry_handle,
        elem_a="shoulder_pivot_0",
        elem_b="handle_pivot_0",
        reason="The left carry-handle pivot is simplified as a nested solid pin within the shoulder boss.",
    )
    ctx.allow_overlap(
        body,
        carry_handle,
        elem_a="shoulder_pivot_1",
        elem_b="handle_pivot_1",
        reason="The right carry-handle pivot is simplified as a nested solid pin within the shoulder boss.",
    )
    ctx.allow_overlap(
        body,
        carry_handle,
        elem_a="shoulder_bridge_0",
        elem_b="handle_pivot_0",
        reason="The left pivot pin passes through the shoulder-side support bridge in the simplified hinge representation.",
    )
    ctx.allow_overlap(
        body,
        carry_handle,
        elem_a="shoulder_bridge_1",
        elem_b="handle_pivot_1",
        reason="The right pivot pin passes through the shoulder-side support bridge in the simplified hinge representation.",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({lid_joint: upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward on the rear hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05
            and open_lid_aabb[1][1] < closed_lid_aabb[1][1],
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )
        ctx.check(
            "lid swings rearward when opened",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[0][1] > closed_lid_aabb[0][1] + 0.10,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    handle_rest_aabb = ctx.part_element_world_aabb(carry_handle, elem="carry_bar")
    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None
    if handle_upper is not None:
        with ctx.pose({handle_joint: handle_upper}):
            handle_raised_aabb = ctx.part_element_world_aabb(carry_handle, elem="carry_bar")
        ctx.check(
            "carry handle lifts above the lid",
            handle_rest_aabb is not None
            and handle_raised_aabb is not None
            and handle_raised_aabb[1][2] > handle_rest_aabb[1][2] + 0.010
            and handle_raised_aabb[0][1] < handle_rest_aabb[0][1] - 0.03,
            details=f"rest={handle_rest_aabb}, raised={handle_raised_aabb}",
        )

    button_rest_pos = ctx.part_world_position(power_button)
    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    if button_upper is not None:
        with ctx.pose({button_joint: button_upper}):
            button_pressed_pos = ctx.part_world_position(power_button)
        ctx.check(
            "power button presses inward",
            button_rest_pos is not None
            and button_pressed_pos is not None
            and button_pressed_pos[1] > button_rest_pos[1] + 0.002,
            details=f"rest={button_rest_pos}, pressed={button_pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
