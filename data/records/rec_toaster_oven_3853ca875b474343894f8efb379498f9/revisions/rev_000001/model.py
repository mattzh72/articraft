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


BODY_W = 0.43
BODY_D = 0.32
BODY_H = 0.28
SHELL_T = 0.011

DOOR_X = -0.047
DOOR_W = 0.298
DOOR_H = 0.170
DOOR_T = 0.022
DOOR_HINGE_Z = 0.068
DOOR_FRAME_SIDE = 0.020
DOOR_FRAME_TOP = 0.022
DOOR_FRAME_BOTTOM = 0.030
DOOR_OPEN_Q = 1.35

BODY_DOOR_OPEN_W = 0.252
BODY_DOOR_OPEN_H = 0.128
BODY_DOOR_OPEN_Z = DOOR_HINGE_Z + 0.086

CONTROL_X = 0.146
KNOB_Z = 0.190
BUTTON_0_Z = 0.112
BUTTON_1_Z = 0.080

BUTTON_W = 0.023
BUTTON_H = 0.012
BUTTON_CAP_D = 0.012
BUTTON_STEM_D = 0.015
BUTTON_TRAVEL = 0.007

TRAY_Z = 0.030
TRAY_W = 0.284
TRAY_D = 0.235
TRAY_H = 0.014
TRAY_WALL = 0.0018
TRAY_TRAVEL = 0.078


def make_body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * SHELL_T,
            BODY_D - 2.0 * SHELL_T,
            BODY_H - 2.0 * SHELL_T,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SHELL_T))
    )
    shell = outer.cut(inner)

    oven_floor = (
        cq.Workplane("XY")
        .box(0.324, BODY_D - 2.0 * SHELL_T - 0.024, 0.006, centered=(True, True, False))
        .translate((DOOR_X, 0.004, 0.055))
    )
    shell = shell.union(oven_floor)

    front_cut_depth = SHELL_T + 0.010
    door_opening = (
        cq.Workplane("XY")
        .box(BODY_DOOR_OPEN_W, front_cut_depth, BODY_DOOR_OPEN_H)
        .translate(
            (
                DOOR_X,
                -BODY_D / 2.0 + front_cut_depth / 2.0 - 0.001,
                BODY_DOOR_OPEN_Z,
            )
        )
    )
    tray_slot = (
        cq.Workplane("XY")
        .box(TRAY_W, front_cut_depth, TRAY_H + 0.002)
        .translate(
            (
                DOOR_X,
                -BODY_D / 2.0 + front_cut_depth / 2.0 - 0.001,
                TRAY_Z,
            )
        )
    )
    knob_hole = (
        cq.Workplane("XY")
        .box(0.013, front_cut_depth, 0.013)
        .translate(
            (
                CONTROL_X,
                -BODY_D / 2.0 + front_cut_depth / 2.0 - 0.001,
                KNOB_Z,
            )
        )
    )
    button_hole_0 = (
        cq.Workplane("XY")
        .box(BUTTON_W, front_cut_depth, BUTTON_H)
        .translate(
            (
                CONTROL_X,
                -BODY_D / 2.0 + front_cut_depth / 2.0 - 0.001,
                BUTTON_0_Z,
            )
        )
    )
    button_hole_1 = (
        cq.Workplane("XY")
        .box(BUTTON_W, front_cut_depth, BUTTON_H)
        .translate(
            (
                CONTROL_X,
                -BODY_D / 2.0 + front_cut_depth / 2.0 - 0.001,
                BUTTON_1_Z,
            )
        )
    )

    return shell.cut(door_opening).cut(tray_slot).cut(knob_hole).cut(button_hole_0).cut(button_hole_1)


def make_door_frame() -> cq.Workplane:
    opening_w = DOOR_W - 2.0 * DOOR_FRAME_SIDE
    opening_h = DOOR_H - DOOR_FRAME_TOP - DOOR_FRAME_BOTTOM

    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(True, True, False))
        .translate((0.0, -DOOR_T / 2.0, 0.0))
    )
    window = (
        cq.Workplane("XY")
        .box(opening_w, DOOR_T + 0.010, opening_h, centered=(True, True, False))
        .translate((0.0, -DOOR_T / 2.0, DOOR_FRAME_BOTTOM))
    )
    return panel.cut(window)


def make_tray_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_H, centered=(True, False, True))
    inner = (
        cq.Workplane("XY")
        .box(
            TRAY_W - 2.0 * TRAY_WALL,
            TRAY_D - 2.0 * TRAY_WALL,
            TRAY_H,
            centered=(True, False, True),
        )
        .translate((0.0, TRAY_WALL, TRAY_WALL))
    )
    tray = outer.cut(inner)

    grip = cq.Workplane("XY").box(0.218, 0.012, 0.010).translate((0.0, -0.006, 0.0))
    runner_depth = TRAY_D - 0.040
    runner_z = -TRAY_H / 2.0 + 0.0015
    runner_offset_x = TRAY_W / 2.0 - 0.010
    runner_0 = (
        cq.Workplane("XY")
        .box(0.010, runner_depth, 0.004, centered=(True, False, True))
        .translate((runner_offset_x, 0.020, runner_z))
    )
    runner_1 = (
        cq.Workplane("XY")
        .box(0.010, runner_depth, 0.004, centered=(True, False, True))
        .translate((-runner_offset_x, 0.020, runner_z))
    )

    return tray.union(grip).union(runner_0).union(runner_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    stainless = model.material("stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.46, 0.47, 0.49, 1.0))
    glass = model.material("glass", rgba=(0.28, 0.33, 0.36, 0.45))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shape(), "body_shell"),
        material=stainless,
        name="shell",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(make_door_frame(), "door_frame"),
        material=dark_trim,
        name="door_frame",
    )
    door.visual(
        Box((DOOR_W - 0.018, 0.004, DOOR_H - 0.052)),
        origin=Origin(xyz=(0.0, -0.011, 0.099)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.240, 0.018, 0.013)),
        origin=Origin(xyz=(0.0, -0.036, 0.121)),
        material=stainless,
        name="handle_bar",
    )
    door.visual(
        Box((0.018, 0.022, 0.040)),
        origin=Origin(xyz=(-0.122, -0.026, 0.110)),
        material=dark_trim,
        name="handle_mount_0",
    )
    door.visual(
        Box((0.018, 0.022, 0.040)),
        origin=Origin(xyz=(0.122, -0.026, 0.110)),
        material=dark_trim,
        name="handle_mount_1",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.0025, 0.012)),
        origin=Origin(xyz=(0.0, -0.026, 0.013)),
        material=stainless,
        name="dial_marker",
    )
    dial.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial_shaft",
    )

    for name in ("button_0", "button_1"):
        button = model.part(name)
        button.visual(
            Box((BUTTON_W, BUTTON_CAP_D, BUTTON_H)),
            origin=Origin(xyz=(0.0, -BUTTON_CAP_D / 2.0 + 0.0005, 0.0)),
            material=button_black,
            name="cap",
        )
        button.visual(
            Box((0.014, BUTTON_STEM_D, 0.008)),
            origin=Origin(xyz=(0.0, BUTTON_STEM_D / 2.0 - 0.0005, 0.0)),
            material=button_black,
            name="stem",
        )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(make_tray_shape(), "tray_body"),
        material=tray_gray,
        name="tray_body",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_X, -BODY_D / 2.0, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(CONTROL_X, -BODY_D / 2.0, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "button_0_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child="button_0",
        origin=Origin(xyz=(CONTROL_X, -BODY_D / 2.0, BUTTON_0_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "button_1_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child="button_1",
        origin=Origin(xyz=(CONTROL_X, -BODY_D / 2.0, BUTTON_1_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(DOOR_X, -BODY_D / 2.0, TRAY_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.12, lower=0.0, upper=TRAY_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    tray = object_model.get_part("tray")

    door_hinge = object_model.get_articulation("door_hinge")
    dial_spin = object_model.get_articulation("dial_spin")
    button_0_press = object_model.get_articulation("button_0_press")
    button_1_press = object_model.get_articulation("button_1_press")
    tray_slide = object_model.get_articulation("tray_slide")

    ctx.allow_overlap(
        body,
        tray,
        elem_a="shell",
        elem_b="tray_body",
        reason="The crumb tray is intentionally represented as sliding inside the lower shell compartment.",
    )

    with ctx.pose({door_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.006,
            name="closed door sits just ahead of the body",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            min_overlap=0.26,
            name="door spans the front oven opening",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.26,
            name="tray stays centered under the oven cavity",
        )

    closed_door_aabb = ctx.part_world_aabb(door)
    closed_button_0 = ctx.part_world_position(button_0)
    closed_button_1 = ctx.part_world_position(button_1)
    closed_tray = ctx.part_world_position(tray)

    with ctx.pose({door_hinge: DOOR_OPEN_Q}):
        open_door_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(
            door,
            tray,
            axis="z",
            min_gap=0.002,
            name="open door clears the lower tray",
        )

    ctx.check(
        "door rotates downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.04,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({button_0_press: BUTTON_TRAVEL}):
        pressed_button_0 = ctx.part_world_position(button_0)
        stationary_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 depresses independently",
        closed_button_0 is not None
        and pressed_button_0 is not None
        and pressed_button_0[1] > closed_button_0[1] + 0.004
        and closed_button_1 is not None
        and stationary_button_1 is not None
        and abs(stationary_button_1[1] - closed_button_1[1]) < 1e-6,
        details=f"rest={closed_button_0}, pressed={pressed_button_0}, other_rest={closed_button_1}, other_pose={stationary_button_1}",
    )

    with ctx.pose({button_1_press: BUTTON_TRAVEL}):
        pressed_button_1 = ctx.part_world_position(button_1)
        stationary_button_0 = ctx.part_world_position(button_0)
    ctx.check(
        "button_1 depresses independently",
        closed_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[1] > closed_button_1[1] + 0.004
        and closed_button_0 is not None
        and stationary_button_0 is not None
        and abs(stationary_button_0[1] - closed_button_0[1]) < 1e-6,
        details=f"rest={closed_button_1}, pressed={pressed_button_1}, other_rest={closed_button_0}, other_pose={stationary_button_0}",
    )

    with ctx.pose({tray_slide: TRAY_TRAVEL}):
        extended_tray = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.26,
            name="extended tray remains laterally guided",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.14,
            name="extended tray keeps retained insertion",
        )
    ctx.check(
        "tray slides forward",
        closed_tray is not None
        and extended_tray is not None
        and extended_tray[1] < closed_tray[1] - 0.05,
        details=f"rest={closed_tray}, extended={extended_tray}",
    )

    ctx.check(
        "selector dial uses a continuous spin joint",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin.motion_limits is not None
        and dial_spin.motion_limits.lower is None
        and dial_spin.motion_limits.upper is None
        and abs(dial_spin.axis[1]) > 0.9,
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}, limits={dial_spin.motion_limits}",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_shaft",
        name="dial shaft stays mounted in the control panel",
    )

    return ctx.report()


object_model = build_object_model()
