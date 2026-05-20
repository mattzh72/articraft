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


BODY_W = 0.64
BODY_D = 0.36
LOWER_H = 0.34
LOWER_Z = 0.075
LID_H = 0.11
WALL = 0.016
HANDLE_RAIL_X = 0.17
HANDLE_Y = -0.224
HANDLE_JOINT_Z = 0.275
WHEEL_RADIUS = 0.09
WHEEL_WIDTH = 0.05
WHEEL_CENTER_X = 0.345
WHEEL_CENTER_Y = -0.145
WHEEL_CENTER_Z = WHEEL_RADIUS


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, LOWER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .edges(">Z")
        .fillet(0.014)
        .translate((0.0, 0.0, LOWER_Z))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * WALL,
            BODY_D - 2.0 * WALL,
            LOWER_H - WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, LOWER_Z + WALL))
    )
    shell = outer.cut(inner)

    for rib_z in (LOWER_Z + 0.072, LOWER_Z + 0.150, LOWER_Z + 0.228):
        shell = shell.union(
            cq.Workplane("XY")
            .box(BODY_W * 0.72, 0.014, 0.012, centered=(True, True, False))
            .translate((0.0, BODY_D * 0.485, rib_z))
        )

    for side_sign in (-1.0, 1.0):
        for rib_z in (LOWER_Z + 0.085, LOWER_Z + 0.163, LOWER_Z + 0.241):
            shell = shell.union(
                cq.Workplane("XY")
                .box(0.018, BODY_D * 0.52, 0.010, centered=(True, True, False))
                .translate((side_sign * (BODY_W * 0.5 - 0.008), 0.01, rib_z))
            )

    for side_sign in (-1.0, 1.0):
        shell = shell.union(
            cq.Workplane("XY")
            .box(0.060, 0.120, 0.165, centered=(True, True, False))
            .translate((side_sign * (BODY_W * 0.5 - 0.048), -BODY_D * 0.30, LOWER_Z + 0.028))
        )

    guide_depth = 0.040
    guide_width = 0.016
    guide_gap = 0.022
    guide_z0 = LOWER_Z + 0.010
    guide_h = 0.31
    guide_y = -BODY_D * 0.5 - 0.016
    for rail_x in (-HANDLE_RAIL_X, HANDLE_RAIL_X):
        for offset in (-0.5 * (guide_gap + guide_width), 0.5 * (guide_gap + guide_width)):
            shell = shell.union(
                cq.Workplane("XY")
                .box(guide_width, guide_depth, guide_h, centered=(True, True, False))
                .translate((rail_x + offset, guide_y, guide_z0))
            )

    return shell


def _lid_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, LID_H, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5, 0.0))
        .edges("|Z")
        .fillet(0.026)
        .edges(">Z")
        .fillet(0.012)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * WALL,
            BODY_D - 2.0 * WALL,
            LID_H - WALL,
            centered=(True, True, False),
        )
        .translate((0.0, BODY_D * 0.5, WALL))
    )
    shell = outer.cut(inner)

    shell = shell.union(
        cq.Workplane("XY")
        .box(0.13, 0.040, 0.050, centered=(True, True, False))
        .translate((0.0, BODY_D - 0.020, 0.020))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.072, 0.040, 0.035, centered=(True, True, False))
        .translate((0.0, BODY_D - 0.020, 0.012))
    )

    for groove_y in (0.090, 0.185, 0.280):
        shell = shell.cut(
            cq.Workplane("XY")
            .box(BODY_W * 0.74, 0.012, 0.006, centered=(True, True, False))
            .translate((0.0, groove_y, LID_H - 0.006))
        )

    return shell


def _pull_handle_shape() -> cq.Workplane:
    rail_w = 0.016
    rail_d = 0.016
    rail_bottom = -0.22
    rail_h = 0.50

    handle = (
        cq.Workplane("XY")
        .box(rail_w, rail_d, rail_h, centered=(True, True, False))
        .translate((-HANDLE_RAIL_X, 0.0, rail_bottom))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(rail_w, rail_d, rail_h, centered=(True, True, False))
        .translate((HANDLE_RAIL_X, 0.0, rail_bottom))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(0.348, 0.016, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, 0.050))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(0.060, 0.012, 0.018, centered=(True, True, False))
        .translate((-0.155, 0.0, 0.248))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(0.060, 0.012, 0.018, centered=(True, True, False))
        .translate((0.155, 0.0, 0.248))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(0.022, 0.014, 0.070, centered=(True, True, False))
        .translate((-0.140, 0.0, 0.248))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(0.022, 0.014, 0.070, centered=(True, True, False))
        .translate((0.140, 0.0, 0.248))
    )
    handle = handle.union(
        cq.Workplane("XY")
        .box(0.300, 0.016, 0.022, centered=(True, True, False))
        .translate((0.0, 0.0, 0.306))
    )
    return handle


def _add_wheel_visuals(part, *, rubber, hub_finish) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.060, length=WHEEL_WIDTH + 0.006),
        origin=spin_origin,
        material=hub_finish,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.020, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=hub_finish,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_chest")

    shell_plastic = model.material("shell_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_plastic = model.material("trim_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.76, 0.79, 0.82, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    button_red = model.material("button_red", rgba=(0.74, 0.12, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "rolling_tool_chest_body"),
        material=shell_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.082, 0.070, LOWER_Z)),
        origin=Origin(xyz=(-0.19, 0.115, LOWER_Z * 0.5)),
        material=trim_plastic,
        name="front_foot_0",
    )
    body.visual(
        Box((0.082, 0.070, LOWER_Z)),
        origin=Origin(xyz=(0.19, 0.115, LOWER_Z * 0.5)),
        material=trim_plastic,
        name="front_foot_1",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(-0.305, WHEEL_CENTER_Y, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_finish,
        name="axle_stub_0",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.305, WHEEL_CENTER_Y, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_finish,
        name="axle_stub_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "rolling_tool_chest_lid"),
        material=shell_plastic,
        name="lid_shell",
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.056, 0.014, 0.028)),
        material=button_red,
        name="button_cap",
    )
    latch_button.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, -0.002)),
        material=trim_plastic,
        name="button_stem",
    )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        mesh_from_cadquery(_pull_handle_shape(), "rolling_tool_chest_pull_handle"),
        origin=Origin(xyz=(0.0, HANDLE_Y, 0.0)),
        material=handle_metal,
        name="handle_frame",
    )

    rear_wheel_0 = model.part("rear_wheel_0")
    _add_wheel_visuals(rear_wheel_0, rubber=rubber, hub_finish=hub_finish)

    rear_wheel_1 = model.part("rear_wheel_1")
    _add_wheel_visuals(rear_wheel_1, rubber=rubber, hub_finish=hub_finish)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5, LOWER_Z + LOWER_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "lid_to_button",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch_button,
        origin=Origin(xyz=(0.0, BODY_D + 0.007, 0.036)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.014),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pull_handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.18),
    )
    model.articulation(
        "body_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel_0,
        origin=Origin(xyz=(-WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "body_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel_1,
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    pull_handle = object_model.get_part("pull_handle")

    lid_hinge = object_model.get_articulation("body_to_lid")
    button_slide = object_model.get_articulation("lid_to_button")
    handle_slide = object_model.get_articulation("body_to_handle")

    with ctx.pose({lid_hinge: 0.0, button_slide: 0.0, handle_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid seats on the lower body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.22,
            name="lid covers the main compartment opening",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.35}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.16,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    handle_rest = ctx.part_world_position(pull_handle)
    with ctx.pose({handle_slide: 0.18}):
        handle_extended = ctx.part_world_position(pull_handle)
    ctx.check(
        "pull handle telescopes upward",
        handle_rest is not None
        and handle_extended is not None
        and handle_extended[2] > handle_rest[2] + 0.16,
        details=f"rest={handle_rest}, extended={handle_extended}",
    )

    button_rest = ctx.part_world_position(latch_button)
    with ctx.pose({button_slide: 0.014}):
        button_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "latch button presses into the lid nose",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] < button_rest[1] - 0.010,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
