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


BASE_WIDTH = 0.390
BASE_DEPTH = 0.310
BASE_HEIGHT = 0.255

CHAMBER_WIDTH = 0.214
CHAMBER_DEPTH = 0.150
CHAMBER_CENTER_Y = 0.012
CHAMBER_FLOOR_Z = 0.014

LID_WIDTH = 0.324
LID_DEPTH = 0.273
LID_HEIGHT = 0.044
LID_WALL = 0.004

LID_HINGE_Y = BASE_DEPTH * 0.5 - 0.018
LID_HINGE_Z = BASE_HEIGHT

PANEL_CENTER_Y = -0.215
PANEL_TOP_Z = 0.048


def _base_shell() -> cq.Workplane:
    outer_shell = cq.Workplane("XY").box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT, centered=(True, True, False))
    outer_shell = outer_shell.edges("|Z").fillet(0.030)
    outer_shell = outer_shell.faces(">Z").shell(-0.006)

    chamber = (
        cq.Workplane("XY")
        .box(CHAMBER_WIDTH + 0.020, CHAMBER_DEPTH + 0.020, 0.224, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, CHAMBER_CENTER_Y, 0.006))
    )
    chamber = chamber.faces(">Z").shell(-0.005)

    body = outer_shell.union(chamber)

    foot = cq.Workplane("XY").box(0.060, 0.038, 0.008, centered=(True, True, False)).edges("|Z").fillet(0.008)
    for sx in (-0.130, 0.130):
        for sy in (-0.104, 0.104):
            body = body.union(foot.translate((sx, sy, -0.008)))

    return body


def _lid_shell() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_HEIGHT, centered=(True, True, False))
        .translate((0.0, -LID_DEPTH * 0.5, 0.0))
    )
    lid = lid.edges("|Z").fillet(0.020)
    lid = lid.faces("<Z").shell(-LID_WALL)

    panel = (
        cq.Workplane("XY")
        .box(0.236, 0.078, 0.014, centered=(True, True, False))
        .translate((0.0, PANEL_CENTER_Y, 0.034))
    )
    lid = lid.union(panel)

    window_cutter = (
        cq.Workplane("XY")
        .box(0.156, 0.104, 0.060, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, -0.126, -0.004))
    )
    lid = lid.cut(window_cutter)

    vent_slot = (
        cq.Workplane("XY")
        .box(0.034, 0.010, 0.036, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, -0.030, 0.012))
    )
    lid = lid.cut(vent_slot)

    for button_x in (-0.082, -0.041, 0.0, 0.041, 0.082):
        pocket = (
            cq.Workplane("XY")
            .box(0.019, 0.010, 0.022, centered=(True, True, False))
            .edges("|Z")
            .fillet(0.0025)
            .translate((button_x, -0.241, 0.028))
        )
        lid = lid.cut(pocket)

    return lid


def _window_pane() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .box(0.156, 0.104, 0.0015, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )
    glass = (
        cq.Workplane("XY")
        .box(0.148, 0.096, 0.003, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.015)
        .translate((0.0, 0.0, -0.003))
    )
    return flange.union(glass)


def _dial_ring() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(0.031)
        .circle(0.019)
        .extrude(0.010)
    )
    grip = cq.Workplane("XY").circle(0.031).circle(0.026).extrude(0.003).translate((0.0, 0.0, 0.010))
    return ring.union(grip)


def _button_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(0.026, 0.014, 0.004, centered=(True, True, False)).edges("|Z").fillet(0.003)


def _vent_cap_shape() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(0.0035).extrude(0.032).translate((-0.016, 0.0, 0.0035))
    flap = (
        cq.Workplane("XY")
        .box(0.032, 0.022, 0.004, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, -0.022, 0.004))
    )
    return barrel.union(flap)


def _spindle_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.0065).extrude(0.016)
    collar = cq.Workplane("XY").circle(0.011).extrude(0.004).translate((0.0, 0.0, 0.002))
    paddle = cq.Workplane("XY").box(0.040, 0.010, 0.004, centered=(True, True, False)).translate((0.0, 0.0, 0.010))
    return shaft.union(collar).union(paddle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_bread_maker")

    body_finish = model.material("body_finish", rgba=(0.76, 0.76, 0.78, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.50, 0.52, 0.56, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.40, 0.48, 0.54, 0.35))
    steel_finish = model.material("steel_finish", rgba=(0.78, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=body_finish,
        name="base_floor",
    )
    base.visual(
        Box((BASE_WIDTH, 0.018, 0.229)),
        origin=Origin(xyz=(0.0, -0.146, 0.1285)),
        material=body_finish,
        name="front_shell",
    )
    base.visual(
        Box((BASE_WIDTH, 0.018, 0.229)),
        origin=Origin(xyz=(0.0, 0.146, 0.1285)),
        material=body_finish,
        name="rear_shell",
    )
    for side_index, side_x in enumerate((-0.186, 0.186)):
        base.visual(
            Box((0.018, 0.274, 0.229)),
            origin=Origin(xyz=(side_x, 0.0, 0.1285)),
            material=body_finish,
            name=f"side_shell_{side_index}",
        )

    base.visual(
        Box((0.234, 0.010, 0.229)),
        origin=Origin(xyz=(0.0, -0.068, 0.1285)),
        material=body_finish,
        name="chamber_front",
    )
    base.visual(
        Box((0.234, 0.010, 0.229)),
        origin=Origin(xyz=(0.0, 0.092, 0.1285)),
        material=body_finish,
        name="chamber_back",
    )
    for liner_index, liner_x in enumerate((-0.112, 0.112)):
        base.visual(
            Box((0.010, 0.150, 0.229)),
            origin=Origin(xyz=(liner_x, 0.012, 0.1285)),
            material=body_finish,
            name=f"chamber_side_{liner_index}",
        )

    base.visual(
        Box((0.338, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.137, 0.249)),
        material=body_finish,
        name="seat_front",
    )
    base.visual(
        Box((0.338, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.137, 0.249)),
        material=body_finish,
        name="seat_back",
    )
    for seat_index, seat_x in enumerate((-0.163, 0.163)):
        base.visual(
            Box((0.012, 0.262, 0.012)),
            origin=Origin(xyz=(seat_x, 0.0, 0.249)),
            material=body_finish,
            name=f"seat_side_{seat_index}",
        )

    for foot_index, (foot_x, foot_y) in enumerate(((-0.130, -0.104), (-0.130, 0.104), (0.130, -0.104), (0.130, 0.104))):
        base.visual(
            Box((0.060, 0.038, 0.008)),
            origin=Origin(xyz=(foot_x, foot_y, -0.004)),
            material=trim_finish,
            name=f"foot_{foot_index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "bread_maker_lid"),
        material=lid_finish,
        name="lid_shell",
    )

    window = model.part("window")
    window.visual(
        mesh_from_cadquery(_window_pane(), "bread_maker_window"),
        material=glass_finish,
        name="window_glass",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_ring(), "bread_maker_dial"),
        material=trim_finish,
        name="dial_ring",
    )

    button_mesh = mesh_from_cadquery(_button_shape(), "bread_maker_button")
    button_xs = (-0.082, -0.041, 0.0, 0.041, 0.082)
    buttons = []
    for index, button_x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=trim_finish, name="button_cap")
        buttons.append((button, button_x))

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        mesh_from_cadquery(_vent_cap_shape(), "bread_maker_vent_cap"),
        material=lid_finish,
        name="vent_cap_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_spindle_shape(), "bread_maker_spindle"),
        material=steel_finish,
        name="spindle_shell",
    )

    lid_hinge = model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    model.articulation(
        "lid_to_window",
        ArticulationType.FIXED,
        parent=lid,
        child=window,
        origin=Origin(xyz=(0.0, -0.126, 0.039)),
    )

    model.articulation(
        "lid_to_dial",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=dial,
        origin=Origin(xyz=(0.0, -0.206, PANEL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    for index, (button, button_x) in enumerate(buttons):
        model.articulation(
            f"lid_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=lid,
            child=button,
            origin=Origin(xyz=(button_x, -0.241, PANEL_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0028,
            ),
        )

    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.0, -0.018, 0.044)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )

    model.articulation(
        "base_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, CHAMBER_CENTER_Y, CHAMBER_FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    spindle = object_model.get_part("spindle")
    button_2 = object_model.get_part("button_2")
    vent_cap = object_model.get_part("vent_cap")

    lid_hinge = object_model.get_articulation("base_to_lid")
    button_joint = object_model.get_articulation("lid_to_button_2")
    vent_joint = object_model.get_articulation("lid_to_vent_cap")

    base_aabb = ctx.part_world_aabb(base)
    ctx.check("countertop_scale_present", base_aabb is not None, "Expected the bread maker body to compile.")
    if base_aabb is not None:
        mins, maxs = base_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("bread_maker_width", 0.36 <= size[0] <= 0.43, f"size={size!r}")
        ctx.check("bread_maker_depth", 0.28 <= size[1] <= 0.35, f"size={size!r}")
        ctx.check("bread_maker_height", 0.23 <= size[2] <= 0.29, f"size={size!r}")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        name="closed lid seats close to the body rim",
    )
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.20, name="lid covers the body opening")
    ctx.expect_overlap(dial, lid, axes="xy", min_overlap=0.04, name="dial sits on the lid control panel")
    ctx.expect_origin_distance(
        spindle,
        base,
        axes="xy",
        max_dist=0.02,
        name="spindle stays near the chamber center",
    )
    ctx.expect_origin_gap(
        spindle,
        base,
        axis="z",
        min_gap=0.012,
        max_gap=0.030,
        name="spindle rises slightly above the chamber floor",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    open_lid_aabb = None
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.12,
        details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
    )

    rest_pos = ctx.part_world_position(button_2)
    pressed_pos = None
    if button_joint.motion_limits is not None and button_joint.motion_limits.upper is not None:
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button_2)
    ctx.check(
        "program button presses downward",
        rest_pos is not None and pressed_pos is not None and float(pressed_pos[2]) < float(rest_pos[2]) - 0.0015,
        details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
    )

    closed_cap_aabb = ctx.part_world_aabb(vent_cap)
    opened_cap_aabb = None
    if vent_joint.motion_limits is not None and vent_joint.motion_limits.upper is not None:
        with ctx.pose({vent_joint: vent_joint.motion_limits.upper}):
            opened_cap_aabb = ctx.part_world_aabb(vent_cap)
    ctx.check(
        "vent cap flips upward",
        closed_cap_aabb is not None
        and opened_cap_aabb is not None
        and float(opened_cap_aabb[1][2]) > float(closed_cap_aabb[1][2]) + 0.015,
        details=f"closed={closed_cap_aabb!r}, opened={opened_cap_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
