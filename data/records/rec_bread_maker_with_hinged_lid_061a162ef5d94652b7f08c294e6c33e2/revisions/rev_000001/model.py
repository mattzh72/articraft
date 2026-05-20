from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_D = 0.290
BODY_W = 0.220
BODY_H = 0.320
BODY_RADIUS = 0.018
WALL = 0.004
FLOOR = 0.008

CHAMBER_D = 0.172
CHAMBER_W = 0.112
CHAMBER_X = 0.004
CHAMBER_Y = -0.028
LINER_THICKNESS = 0.0012

LID_D = 0.196
LID_W = 0.154
LID_H = 0.032

PANEL_X = 0.052
PANEL_Z = 0.202
PANEL_D = 0.116
PANEL_H = 0.112
PANEL_POCKET = 0.010
PANEL_FLOOR_Y = BODY_W / 2.0

KNOB_X = PANEL_X - 0.022
KNOB_Z = PANEL_Z + 0.017
BUTTON_X = PANEL_X + 0.030
BUTTON_ZS = (PANEL_Z + 0.024, PANEL_Z - 0.022)

PADDLE_X = 0.010
PADDLE_Y = CHAMBER_Y
SPINDLE_Z = FLOOR + 0.002

PLUNGER_Z = BODY_H - 0.052


def _build_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_D, BODY_W, BODY_H, centered=(True, True, False))
    body = body.edges("|Z").fillet(BODY_RADIUS)
    body = body.edges(">Z").fillet(0.007)

    chamber = (
        cq.Workplane("XY")
        .box(CHAMBER_D, CHAMBER_W, BODY_H, centered=(True, True, False))
        .translate((CHAMBER_X, CHAMBER_Y, FLOOR))
    )
    body = body.cut(chamber)

    knob_hole = (
        cq.Workplane("XY")
        .box(0.016, 0.042, 0.016, centered=(True, True, True))
        .translate((KNOB_X, PANEL_FLOOR_Y - 0.011, KNOB_Z))
    )
    body = body.cut(knob_hole)

    for button_z in BUTTON_ZS:
        button_hole = (
            cq.Workplane("XY")
            .box(0.018, 0.040, 0.018, centered=(True, True, True))
            .translate((BUTTON_X, PANEL_FLOOR_Y - 0.010, button_z))
        )
        body = body.cut(button_hole)

    front_slot = (
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.020, centered=(True, True, True))
        .translate((BODY_D / 2.0 - 0.007, CHAMBER_Y, PLUNGER_Z))
    )
    body = body.cut(front_slot)

    return cq.Workplane(obj=body.val())


def _build_lid_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(LID_D, LID_W, LID_H, centered=(False, True, False))
    shell = shell.edges("|Z").fillet(0.010)
    shell = shell.edges(">Z").fillet(0.005)

    inner = (
        cq.Workplane("XY")
        .box(LID_D - 0.016, LID_W - 0.016, LID_H - 0.003, centered=(False, True, False))
        .translate((0.008, 0.0, 0.0))
    )
    shell = shell.cut(inner)

    grip = (
        cq.Workplane("XY")
        .box(0.034, 0.074, 0.010, centered=(False, True, False))
        .translate((LID_D - 0.052, 0.0, LID_H - 0.007))
    )
    return shell.union(grip)


def _build_paddle_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.009)
        .extrude(0.016)
        .translate((0.0, 0.0, 0.001))
    )
    blade = (
        cq.Workplane("XY")
        .box(0.046, 0.016, 0.006, centered=(True, True, False))
        .translate((0.002, 0.0, 0.004))
    )
    fin = (
        cq.Workplane("XY")
        .box(0.022, 0.010, 0.010, centered=(True, True, False))
        .translate((0.010, 0.0, 0.010))
    )
    paddle = hub.union(blade).union(fin)
    bore = cq.Workplane("XY").circle(0.0065).extrude(0.024)
    return paddle.cut(bore)


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[i] - mins[i]) for i in range(3))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float((mins[i] + maxs[i]) * 0.5) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bread_maker")

    shell_white = model.material("shell_white", rgba=(0.92, 0.92, 0.90, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.70, 0.72, 0.73, 1.0))
    liner_metal = model.material("liner_metal", rgba=(0.83, 0.84, 0.86, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    button_grey = model.material("button_grey", rgba=(0.82, 0.84, 0.85, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "bread_maker_body"),
        material=shell_white,
        name="shell",
    )
    body.visual(
        Box((CHAMBER_D - 0.012, CHAMBER_W - 0.012, 0.002)),
        origin=Origin(xyz=(CHAMBER_X, CHAMBER_Y, FLOOR + 0.001)),
        material=liner_metal,
        name="liner_floor",
    )
    liner_height = BODY_H - FLOOR - 0.015
    liner_z = FLOOR + liner_height / 2.0 - 0.0005
    body.visual(
        Box((CHAMBER_D - 0.008, LINER_THICKNESS, liner_height)),
        origin=Origin(
            xyz=(
                CHAMBER_X,
                CHAMBER_Y - (CHAMBER_W - LINER_THICKNESS) / 2.0,
                liner_z,
            )
        ),
        material=liner_metal,
        name="liner_left",
    )
    body.visual(
        Box((CHAMBER_D - 0.008, LINER_THICKNESS, liner_height)),
        origin=Origin(
            xyz=(
                CHAMBER_X,
                CHAMBER_Y + (CHAMBER_W - LINER_THICKNESS) / 2.0,
                liner_z,
            )
        ),
        material=liner_metal,
        name="liner_right",
    )
    body.visual(
        Box((LINER_THICKNESS, CHAMBER_W - 0.008, liner_height)),
        origin=Origin(
            xyz=(
                CHAMBER_X - (CHAMBER_D - LINER_THICKNESS) / 2.0,
                CHAMBER_Y,
                liner_z,
            )
        ),
        material=liner_metal,
        name="liner_rear",
    )
    body.visual(
        Box((LINER_THICKNESS, CHAMBER_W - 0.008, liner_height)),
        origin=Origin(
            xyz=(
                CHAMBER_X + (CHAMBER_D - LINER_THICKNESS) / 2.0,
                CHAMBER_Y,
                liner_z,
            )
        ),
        material=liner_metal,
        name="liner_front",
    )
    body.visual(
        Cylinder(radius=0.0082, length=0.002),
        origin=Origin(xyz=(PADDLE_X, PADDLE_Y, SPINDLE_Z + 0.001)),
        material=trim_grey,
        name="spindle_seat",
    )
    body.visual(
        Cylinder(radius=0.0058, length=0.018),
        origin=Origin(xyz=(PADDLE_X, PADDLE_Y, SPINDLE_Z + 0.010)),
        material=dark_grey,
        name="spindle",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_D, BODY_W, BODY_H)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "bread_maker_lid"),
        origin=Origin(xyz=(0.0, 0.0, 0.0004)),
        material=shell_white,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_D, LID_W, LID_H)),
        mass=0.65,
        origin=Origin(xyz=(LID_D / 2.0, 0.0, LID_H / 2.0)),
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_build_paddle_shape(), "kneading_paddle"),
        material=dark_grey,
        name="paddle_body",
    )
    paddle.visual(
        Box((0.010, 0.006, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
        material=trim_grey,
        name="paddle_fin",
    )
    paddle.inertial = Inertial.from_geometry(
        Box((0.050, 0.018, 0.020)),
        mass=0.10,
        origin=Origin(xyz=(0.005, 0.0, 0.010)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.018,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.046, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "bread_maker_timer_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="knob_shell",
    )
    timer_knob.visual(
        Cylinder(radius=0.0042, length=0.016),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="shaft",
    )
    timer_knob.visual(
        Box((0.006, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, 0.017, 0.014)),
        material=button_grey,
        name="pointer",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Box((0.046, 0.020, 0.040)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    for index, button_z in enumerate(BUTTON_ZS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.022, 0.006, 0.022)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=button_grey,
            name="cap",
        )
        button.visual(
            Box((0.015, 0.018, 0.015)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=dark_grey,
            name="stem",
        )
        button.inertial = Inertial.from_geometry(Box((0.022, 0.022, 0.022)), mass=0.02)
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BUTTON_X, PANEL_FLOOR_Y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.005,
            ),
        )

    release_plunger = model.part("release_plunger")
    release_plunger.visual(
        Box((0.006, 0.022, 0.022)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=button_grey,
        name="cap",
    )
    release_plunger.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=dark_grey,
        name="stem",
    )
    release_plunger.inertial = Inertial.from_geometry(Box((0.026, 0.022, 0.022)), mass=0.03)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(
                CHAMBER_X - LID_D / 2.0,
                CHAMBER_Y,
                BODY_H - 0.0005,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "body_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=paddle,
        origin=Origin(xyz=(PADDLE_X, PADDLE_Y, SPINDLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(KNOB_X, PANEL_FLOOR_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_release_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_plunger,
        origin=Origin(xyz=(BODY_D / 2.0 - 0.001, CHAMBER_Y, PLUNGER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    paddle = object_model.get_part("paddle")
    timer_knob = object_model.get_part("timer_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    release_plunger = object_model.get_part("release_plunger")
    lid_hinge = object_model.get_articulation("body_to_lid")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")
    plunger_joint = object_model.get_articulation("body_to_release_plunger")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0003,
        name="closed lid seats tightly on the body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.080,
        name="closed lid covers the chamber opening footprint",
    )
    ctx.expect_gap(
        paddle,
        body,
        axis="z",
        positive_elem="paddle_body",
        negative_elem="liner_floor",
        min_gap=0.0005,
        max_gap=0.010,
        name="paddle sits just above the chamber floor",
    )
    ctx.expect_overlap(
        paddle,
        body,
        axes="xy",
        elem_a="paddle_body",
        elem_b="liner_floor",
        min_overlap=0.014,
        name="paddle remains centered over the chamber floor",
    )

    liner_floor_aabb = ctx.part_element_world_aabb(body, elem="liner_floor")
    liner_floor_size = _aabb_size(liner_floor_aabb)
    ctx.check(
        "chamber floor is loaf pan sized",
        liner_floor_size is not None
        and 0.150 <= liner_floor_size[0] <= 0.170
        and 0.090 <= liner_floor_size[1] <= 0.110,
        details=f"liner_floor_size={liner_floor_size!r}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.090,
        details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
    )

    fin_center_closed = _aabb_center(ctx.part_element_world_aabb(paddle, elem="paddle_fin"))
    with ctx.pose(body_to_paddle=math.pi / 2.0):
        fin_center_quarter = _aabb_center(ctx.part_element_world_aabb(paddle, elem="paddle_fin"))
    ctx.check(
        "paddle rotates about the spindle axis",
        fin_center_closed is not None
        and fin_center_quarter is not None
        and (
            abs(fin_center_closed[0] - fin_center_quarter[0]) > 0.010
            or abs(fin_center_closed[1] - fin_center_quarter[1]) > 0.010
        ),
        details=f"closed={fin_center_closed!r}, quarter={fin_center_quarter!r}",
    )

    knob_pointer_closed = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
    with ctx.pose(body_to_timer_knob=math.pi / 2.0):
        knob_pointer_quarter = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
    ctx.check(
        "timer knob rotates on the side panel",
        knob_pointer_closed is not None
        and knob_pointer_quarter is not None
        and (
            abs(knob_pointer_closed[0] - knob_pointer_quarter[0]) > 0.008
            or abs(knob_pointer_closed[2] - knob_pointer_quarter[2]) > 0.008
        ),
        details=f"closed={knob_pointer_closed!r}, quarter={knob_pointer_quarter!r}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: button_0_joint.motion_limits.upper}):
        button_0_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "upper button presses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[1] < button_0_rest[1] - 0.003,
        details=f"rest={button_0_rest!r}, pressed={button_0_pressed!r}",
    )

    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: button_1_joint.motion_limits.upper}):
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "lower button presses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[1] < button_1_rest[1] - 0.003,
        details=f"rest={button_1_rest!r}, pressed={button_1_pressed!r}",
    )

    plunger_rest = ctx.part_world_position(release_plunger)
    with ctx.pose({plunger_joint: plunger_joint.motion_limits.upper}):
        plunger_pressed = ctx.part_world_position(release_plunger)
    ctx.check(
        "front release plunger slides into the shell",
        plunger_rest is not None
        and plunger_pressed is not None
        and plunger_pressed[0] < plunger_rest[0] - 0.006,
        details=f"rest={plunger_rest!r}, pressed={plunger_pressed!r}",
    )

    return ctx.report()


object_model = build_object_model()
