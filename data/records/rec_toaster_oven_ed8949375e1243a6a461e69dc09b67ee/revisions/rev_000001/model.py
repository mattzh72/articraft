from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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


BODY_WIDTH = 0.42
BODY_DEPTH = 0.32
BODY_HEIGHT = 0.255
SIDE_WALL = 0.013
BACK_WALL = 0.012
BOTTOM_WALL = 0.018
TOP_WALL = 0.020
FRONT_FRAME_THICKNESS = 0.006
FOOT_HEIGHT = 0.006

DOOR_OPENING_WIDTH = 0.302
DOOR_OPENING_HEIGHT = 0.165
DOOR_OPENING_BOTTOM = 0.046
DOOR_OPENING_CENTER_X = -0.041

DOOR_WIDTH = 0.308
DOOR_HEIGHT = 0.172
DOOR_THICKNESS = 0.018


def _shape_box(
    size: tuple[float, float, float],
    *,
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> object:
    return cq.Workplane("XY").box(*size, centered=(True, True, False)).translate(translate).val()


def _body_shell_shape() -> object:
    body = _shape_box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT))

    inner_width = BODY_WIDTH - 2.0 * SIDE_WALL
    inner_depth = BODY_DEPTH - FRONT_FRAME_THICKNESS - BACK_WALL
    inner_height = BODY_HEIGHT - TOP_WALL - BOTTOM_WALL
    cavity = _shape_box(
        (inner_width, inner_depth, inner_height),
        translate=(0.0, (FRONT_FRAME_THICKNESS - BACK_WALL) * 0.5, BOTTOM_WALL),
    )
    body = body.cut(cavity)

    door_cut = _shape_box(
        (DOOR_OPENING_WIDTH, 0.080, DOOR_OPENING_HEIGHT),
        translate=(
            DOOR_OPENING_CENTER_X,
            BODY_DEPTH * 0.5 - 0.020,
            DOOR_OPENING_BOTTOM,
        ),
    )
    body = body.cut(door_cut)

    return body


def _door_shape() -> object:
    slab = _shape_box(
        (DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT),
        translate=(0.0, DOOR_THICKNESS * 0.5, 0.0),
    )
    window_cut = _shape_box(
        (0.246, DOOR_THICKNESS + 0.010, 0.094),
        translate=(0.0, DOOR_THICKNESS * 0.5, 0.050),
    )
    return slab.cut(window_cut)


def _door_handle_shape() -> object:
    handle = _shape_box(
        (0.190, 0.010, 0.012),
        translate=(0.0, DOOR_THICKNESS + 0.016, 0.125),
    )

    for support_x in (-0.084, 0.084):
        support = _shape_box(
            (0.014, 0.022, 0.020),
            translate=(support_x, DOOR_THICKNESS + 0.001, 0.121),
        )
        handle = handle.fuse(support)

    return handle


def _latch_shape() -> object:
    pivot = cq.Workplane("XZ").circle(0.0065).extrude(0.010).val()
    lever = _shape_box(
        (0.012, 0.010, 0.055),
        translate=(-0.012, 0.005, -0.044),
    )
    thumb = _shape_box(
        (0.018, 0.010, 0.012),
        translate=(-0.014, 0.005, -0.054),
    )
    return pivot.fuse(lever).fuse(thumb)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    shell_finish = model.material("shell_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    door_finish = model.material("door_finish", rgba=(0.43, 0.45, 0.47, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.20, 0.28, 0.33, 0.35))
    knob_finish = model.material("knob_finish", rgba=(0.09, 0.09, 0.10, 1.0))

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.022,
            body_style="skirted",
            top_diameter=0.028,
            edge_radius=0.001,
            skirt=KnobSkirt(0.040, 0.0045, flare=0.08),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "toaster_oven_dial",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "toaster_oven_body"),
        material=shell_finish,
        name="body_shell",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "toaster_oven_door"),
        material=door_finish,
        name="door_shell",
    )

    door_handle = model.part("door_handle")
    door_handle.visual(
        mesh_from_cadquery(_door_handle_shape(), "toaster_oven_door_handle"),
        material=door_finish,
        name="handle_shell",
    )

    door_glass = model.part("door_glass")
    door_glass.visual(
        Box((0.238, 0.004, 0.092)),
        origin=Origin(xyz=(0.0, DOOR_THICKNESS * 0.5, 0.098)),
        material=glass_finish,
        name="glass_panel",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="dial_shell",
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="dial_shell",
    )

    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(_latch_shape(), "toaster_oven_latch"),
        material=knob_finish,
        name="latch_shell",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(
                DOOR_OPENING_CENTER_X,
                BODY_DEPTH * 0.5,
                DOOR_OPENING_BOTTOM,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(88.0),
        ),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.FIXED,
        parent=door,
        child=door_handle,
        origin=Origin(),
    )
    model.articulation(
        "door_to_glass",
        ArticulationType.FIXED,
        parent=door,
        child=door_glass,
        origin=Origin(),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(0.142, BODY_DEPTH * 0.5, 0.176)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )
    model.articulation(
        "body_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=temperature_dial,
        origin=Origin(xyz=(0.142, BODY_DEPTH * 0.5, 0.108)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.138, BODY_DEPTH * 0.5 + 0.010, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    timer_dial = object_model.get_part("timer_dial")
    temperature_dial = object_model.get_part("temperature_dial")
    latch = object_model.get_part("latch")
    hinge = object_model.get_articulation("body_to_door")
    timer_joint = object_model.get_articulation("body_to_timer_dial")
    temperature_joint = object_model.get_articulation("body_to_temperature_dial")
    latch_joint = object_model.get_articulation("body_to_latch")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            max_gap=0.003,
            max_penetration=0.0,
            name="closed door sits near the front frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.15,
            name="closed door covers the oven opening",
        )
        closed_aabb = ctx.part_world_aabb(door)

    with ctx.pose({hinge: math.radians(80.0)}):
        open_aabb = ctx.part_world_aabb(door)

    door_opens_out = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.10
        and open_aabb[1][2] < closed_aabb[1][2] - 0.08
    )
    ctx.check(
        "door drops downward when opened",
        door_opens_out,
        details=f"closed_aabb={closed_aabb!r}, open_aabb={open_aabb!r}",
    )

    ctx.expect_gap(
        timer_dial,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="timer dial seats on the control panel",
    )
    ctx.expect_gap(
        temperature_dial,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="temperature dial seats on the control panel",
    )
    ctx.expect_origin_gap(
        timer_dial,
        temperature_dial,
        axis="z",
        min_gap=0.055,
        max_gap=0.075,
        name="timer dial sits above the temperature dial",
    )
    ctx.expect_origin_distance(
        timer_dial,
        temperature_dial,
        axes="x",
        max_dist=0.002,
        name="front dials stay in one vertical stack",
    )

    with ctx.pose({timer_joint: 1.4, temperature_joint: -1.1}):
        rotated_timer = ctx.part_world_aabb(timer_dial)
        rotated_temperature = ctx.part_world_aabb(temperature_dial)
    ctx.check(
        "dials remain mounted while rotating",
        rotated_timer is not None and rotated_temperature is not None,
        details=f"timer_aabb={rotated_timer!r}, temperature_aabb={rotated_temperature!r}",
    )

    latch_rest = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 0.55}):
        latch_swung = ctx.part_world_aabb(latch)
    ctx.check(
        "side latch swings across the door frame",
        latch_rest is not None
        and latch_swung is not None
        and latch_swung[0][0] < latch_rest[0][0] - 0.01,
        details=f"rest={latch_rest!r}, swung={latch_swung!r}",
    )

    return ctx.report()


object_model = build_object_model()
