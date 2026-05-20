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


BODY_W = 0.236
BODY_D = 0.175
BODY_H = 0.565
BODY_WALL = 0.004

TANK_W = 0.198
TANK_D = 0.128
TANK_H = 0.166
TANK_WALL = 0.003
TANK_Z = 0.119
TANK_TRAVEL = 0.085

BAND_W = 0.114
BAND_H = 0.214
BAND_Z = 0.401
BAND_RECESS = 0.002
BAND_DEPTH = BAND_RECESS + 0.0008

DIAL_Z = 0.404
DIAL_R = 0.024

BUTTON_LAYOUT = (
    (-0.032, 0.446),
    (0.032, 0.446),
    (0.0, 0.362),
)

VENT_W = 0.148
VENT_D = 0.072
VENT_Y = 0.006


def _front_face_y() -> float:
    return -BODY_D * 0.5


def _band_face_y() -> float:
    return _front_face_y() + BAND_DEPTH


def _body_shape() -> cq.Workplane:
    inner_w = BODY_W - 2.0 * BODY_WALL
    opening_w = TANK_W + 0.008
    opening_h = TANK_H + 0.008
    opening_bottom = TANK_Z - opening_h * 0.5
    opening_top = TANK_Z + opening_h * 0.5
    jamb_w = (inner_w - opening_w) * 0.5
    front_y = _front_face_y() + BODY_WALL * 0.5

    shell = (
        cq.Workplane("XY")
        .box(BODY_WALL, BODY_D, BODY_H)
        .translate((-(BODY_W * 0.5 - BODY_WALL * 0.5), 0.0, BODY_H * 0.5))
    )
    shell = shell.union(
        cq.Workplane("XY")
        .box(BODY_WALL, BODY_D, BODY_H)
        .translate(((BODY_W * 0.5 - BODY_WALL * 0.5), 0.0, BODY_H * 0.5))
    )
    shell = shell.union(
        cq.Workplane("XY")
        .box(inner_w, BODY_WALL, BODY_H)
        .translate((0.0, BODY_D * 0.5 - BODY_WALL * 0.5, BODY_H * 0.5))
    )
    shell = shell.union(
        cq.Workplane("XY")
        .box(inner_w, BODY_D, BODY_WALL)
        .translate((0.0, 0.0, BODY_WALL * 0.5))
    )

    top_plate = (
        cq.Workplane("XY")
        .box(inner_w, BODY_D, BODY_WALL)
        .translate((0.0, 0.0, BODY_H - BODY_WALL * 0.5))
    )
    top_plate = top_plate.cut(
        cq.Workplane("XY")
        .box(VENT_W, VENT_D, BODY_WALL + 0.004)
        .translate((0.0, VENT_Y, BODY_H - BODY_WALL * 0.5))
    )
    shell = shell.union(top_plate)

    front_upper = (
        cq.Workplane("XY")
        .box(inner_w, BODY_WALL, BODY_H - opening_top)
        .translate((0.0, front_y, opening_top + (BODY_H - opening_top) * 0.5))
    )
    front_upper = front_upper.cut(
        cq.Workplane("XY")
        .box(BAND_W, BAND_DEPTH, BAND_H)
        .translate((0.0, _front_face_y() + BAND_DEPTH * 0.5, BAND_Z))
    )
    front_upper = front_upper.cut(
        cq.Workplane("XY")
        .box(0.026, 0.030, 0.026)
        .translate((0.0, _front_face_y() + 0.014, DIAL_Z))
    )
    for button_x, button_z in BUTTON_LAYOUT:
        front_upper = front_upper.cut(
            cq.Workplane("XY")
            .box(0.012, 0.030, 0.008)
            .translate((button_x, _front_face_y() + 0.014, button_z))
        )
    shell = shell.union(front_upper)

    shell = shell.union(
        cq.Workplane("XY")
        .box(inner_w, BODY_WALL, opening_bottom)
        .translate((0.0, front_y, opening_bottom * 0.5))
    )
    shell = shell.union(
        cq.Workplane("XY")
        .box(jamb_w, BODY_WALL, opening_h)
        .translate((opening_w * 0.5 + jamb_w * 0.5, front_y, TANK_Z))
    )
    shell = shell.union(
        cq.Workplane("XY")
        .box(jamb_w, BODY_WALL, opening_h)
        .translate((-(opening_w * 0.5 + jamb_w * 0.5), front_y, TANK_Z))
    )

    return shell


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_dehumidifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    tank_smoke = model.material("tank_smoke", rgba=(0.74, 0.79, 0.84, 0.62))
    dark_trim = model.material("dark_trim", rgba=(0.22, 0.24, 0.27, 1.0))
    mid_grey = model.material("mid_grey", rgba=(0.52, 0.54, 0.57, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=shell_white,
        name="body_shell",
    )

    tank = model.part("tank")
    tank.visual(
        Box((TANK_W + 0.020, 0.008, TANK_H + 0.012)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=tank_smoke,
        name="tank_face",
    )
    tank.visual(
        Box((TANK_WALL, TANK_D, TANK_H)),
        origin=Origin(xyz=(-(TANK_W * 0.5 - TANK_WALL * 0.5), TANK_D * 0.5, 0.0)),
        material=tank_smoke,
        name="side_wall_0",
    )
    tank.visual(
        Box((TANK_WALL, TANK_D, TANK_H)),
        origin=Origin(xyz=((TANK_W * 0.5 - TANK_WALL * 0.5), TANK_D * 0.5, 0.0)),
        material=tank_smoke,
        name="side_wall_1",
    )
    tank.visual(
        Box((TANK_W - 2.0 * TANK_WALL, TANK_D, TANK_WALL)),
        origin=Origin(xyz=(0.0, TANK_D * 0.5, -(TANK_H * 0.5 - TANK_WALL * 0.5))),
        material=tank_smoke,
        name="bottom_wall",
    )
    tank.visual(
        Box((TANK_W - 2.0 * TANK_WALL, TANK_WALL, TANK_H)),
        origin=Origin(xyz=(0.0, TANK_D - TANK_WALL * 0.5, 0.0)),
        material=tank_smoke,
        name="rear_wall",
    )
    tank.visual(
        Box((TANK_W - 2.0 * TANK_WALL, TANK_WALL, 0.030)),
        origin=Origin(xyz=(0.0, TANK_WALL * 0.5, 0.045)),
        material=tank_smoke,
        name="grip_bridge",
    )

    model.articulation(
        "tank_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank,
        origin=Origin(xyz=(0.0, _front_face_y(), TANK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=TANK_TRAVEL,
        ),
    )

    shutter = model.part("shutter")
    shutter.visual(
        Box((VENT_W + 0.010, VENT_D + 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -(VENT_D + 0.010) * 0.5, 0.002)),
        material=dark_trim,
        name="flap_panel",
    )
    shutter.visual(
        Cylinder(radius=0.0045, length=VENT_W - 0.010),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0045),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=mid_grey,
        name="hinge_barrel",
    )

    model.articulation(
        "shutter_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shutter,
        origin=Origin(xyz=(0.0, VENT_Y + VENT_D * 0.5, BODY_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_R, length=0.013),
        origin=Origin(xyz=(0.0, -0.0065, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_trim,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=mid_grey,
        name="shaft",
    )
    dial.visual(
        Box((0.006, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, -0.0145, 0.015)),
        material=mid_grey,
        name="pointer_ridge",
    )

    model.articulation(
        "selector_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, _band_face_y(), DIAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    for index, (button_x, button_z) in enumerate(BUTTON_LAYOUT):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=mid_grey,
            name="cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=dark_trim,
            name="stem",
        )

        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, _band_face_y(), button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tank = object_model.get_part("tank")
    shutter = object_model.get_part("shutter")
    dial = object_model.get_part("dial")

    tank_slide = object_model.get_articulation("tank_slide")
    shutter_hinge = object_model.get_articulation("shutter_hinge")
    selector_spin = object_model.get_articulation("selector_spin")

    ctx.expect_within(
        tank,
        body,
        axes="xz",
        margin=0.004,
        name="closed tank stays aligned in the lower opening",
    )
    ctx.expect_overlap(
        tank,
        body,
        axes="y",
        min_overlap=0.10,
        name="closed tank remains deeply inserted",
    )

    tank_rest = ctx.part_world_position(tank)
    with ctx.pose({tank_slide: TANK_TRAVEL}):
        ctx.expect_within(
            tank,
            body,
            axes="xz",
            margin=0.004,
            name="extended tank stays centered in the opening",
        )
        ctx.expect_overlap(
            tank,
            body,
            axes="y",
            min_overlap=0.035,
            name="extended tank retains insertion",
        )
        tank_extended = ctx.part_world_position(tank)
    ctx.check(
        "tank slides forward",
        tank_rest is not None
        and tank_extended is not None
        and tank_extended[1] < tank_rest[1] - 0.06,
        details=f"rest={tank_rest}, extended={tank_extended}",
    )

    shutter_closed = ctx.part_element_world_aabb(shutter, elem="flap_panel")
    with ctx.pose({shutter_hinge: 1.0}):
        shutter_open = ctx.part_element_world_aabb(shutter, elem="flap_panel")
    ctx.check(
        "shutter lifts above the top vent",
        shutter_closed is not None
        and shutter_open is not None
        and shutter_open[1][2] > shutter_closed[1][2] + 0.04,
        details=f"closed={shutter_closed}, open={shutter_open}",
    )

    dial_pointer_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer_ridge"))
    with ctx.pose({selector_spin: 1.2}):
        dial_pointer_turned = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer_ridge"))
    ctx.check(
        "selector dial rotates around its spindle",
        dial_pointer_rest is not None
        and dial_pointer_turned is not None
        and abs(dial_pointer_turned[0] - dial_pointer_rest[0]) > 0.012,
        details=f"rest={dial_pointer_rest}, turned={dial_pointer_turned}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_press = object_model.get_articulation(f"button_{index}_press")
        rest_aabb = ctx.part_element_world_aabb(button, elem="cap")
        with ctx.pose({button_press: 0.003}):
            pressed_aabb = ctx.part_element_world_aabb(button, elem="cap")
        ctx.check(
            f"button_{index} presses inward",
            rest_aabb is not None
            and pressed_aabb is not None
            and pressed_aabb[0][1] > rest_aabb[0][1] + 0.0025,
            details=f"rest={rest_aabb}, pressed={pressed_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
