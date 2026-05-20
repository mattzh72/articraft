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

BODY_DEPTH = 0.312
BODY_WIDTH = 0.274
BODY_HEIGHT = 0.332
DRAWER_OPENING_CENTER_Z = 0.134
DRAWER_JOINT_X = BODY_DEPTH * 0.5

DRAWER_BUCKET_DEPTH = 0.212
DRAWER_BUCKET_WIDTH = 0.216
DRAWER_BUCKET_HEIGHT = 0.116
DRAWER_PANEL_THICKNESS = 0.014
DRAWER_PANEL_WIDTH = 0.236
DRAWER_PANEL_HEIGHT = 0.136
DRAWER_HANDLE_DEPTH = 0.050
DRAWER_HANDLE_WIDTH = 0.120
DRAWER_HANDLE_HEIGHT = 0.030
DRAWER_TRAVEL = 0.122


def _filleted_box(size_x: float, size_y: float, size_z: float, radius: float):
    sketch = cq.Sketch().rect(size_y, size_z)
    if radius > 0.0:
        sketch = sketch.vertices().fillet(radius).reset()
    return (
        cq.Workplane("YZ")
        .placeSketch(sketch)
        .extrude(size_x)
        .translate((-size_x * 0.5, 0.0, 0.0))
    )


def _make_body_shell():
    side_thickness = 0.014
    shell_depth = BODY_DEPTH - 0.018
    side_height = BODY_HEIGHT - 0.044
    fascia_thickness = 0.018

    left_wall = _filleted_box(shell_depth, side_thickness, side_height, 0.006).translate(
        (0.0, -(BODY_WIDTH * 0.5 - side_thickness * 0.5), side_height * 0.5)
    )
    right_wall = _filleted_box(shell_depth, side_thickness, side_height, 0.006).translate(
        (0.0, BODY_WIDTH * 0.5 - side_thickness * 0.5, side_height * 0.5)
    )
    top_cover = _filleted_box(
        shell_depth,
        BODY_WIDTH - side_thickness * 2.0,
        0.060,
        0.020,
    ).translate((0.0, 0.0, BODY_HEIGHT - 0.030))
    back_wall = _filleted_box(
        0.018,
        BODY_WIDTH - side_thickness * 2.0,
        0.242,
        0.010,
    ).translate((-BODY_DEPTH * 0.5 + 0.009, 0.0, 0.121))
    rear_floor = _filleted_box(0.120, BODY_WIDTH - 0.050, 0.018, 0.006).translate(
        (-0.096, 0.0, 0.009)
    )
    front_fascia = _filleted_box(
        fascia_thickness,
        BODY_WIDTH - 0.034,
        0.112,
        0.014,
    ).translate((BODY_DEPTH * 0.5 - fascia_thickness * 0.5, 0.0, 0.256))
    bottom_lip = _filleted_box(
        fascia_thickness,
        BODY_WIDTH - 0.034,
        0.050,
        0.010,
    ).translate((BODY_DEPTH * 0.5 - fascia_thickness * 0.5, 0.0, 0.025))
    left_post = _filleted_box(fascia_thickness, 0.028, 0.150, 0.008).translate(
        (BODY_DEPTH * 0.5 - fascia_thickness * 0.5, -(BODY_WIDTH * 0.5 - 0.028), 0.125)
    )
    right_post = _filleted_box(fascia_thickness, 0.028, 0.150, 0.008).translate(
        (BODY_DEPTH * 0.5 - fascia_thickness * 0.5, BODY_WIDTH * 0.5 - 0.028, 0.125)
    )

    return (
        left_wall.union(right_wall)
        .union(top_cover)
        .union(back_wall)
        .union(rear_floor)
        .union(front_fascia)
        .union(bottom_lip)
        .union(left_post)
        .union(right_post)
    )


def _make_drawer_body():
    bucket_outer = _filleted_box(
        DRAWER_BUCKET_DEPTH,
        DRAWER_BUCKET_WIDTH,
        DRAWER_BUCKET_HEIGHT,
        0.012,
    ).translate((-DRAWER_BUCKET_DEPTH * 0.5, 0.0, 0.0))
    bucket_inner = _filleted_box(
        DRAWER_BUCKET_DEPTH - 0.018,
        DRAWER_BUCKET_WIDTH - 0.018,
        DRAWER_BUCKET_HEIGHT - 0.010,
        0.008,
    ).translate(
        (
            -DRAWER_BUCKET_DEPTH * 0.5 + 0.004,
            0.0,
            0.006,
        )
    )
    bucket_shell = bucket_outer.cut(bucket_inner)

    front_panel = _filleted_box(
        DRAWER_PANEL_THICKNESS,
        DRAWER_PANEL_WIDTH,
        DRAWER_PANEL_HEIGHT,
        0.016,
    ).translate((DRAWER_PANEL_THICKNESS * 0.5, 0.0, 0.0))
    handle = _filleted_box(
        DRAWER_HANDLE_DEPTH,
        DRAWER_HANDLE_WIDTH,
        DRAWER_HANDLE_HEIGHT,
        0.008,
    ).translate((0.035, 0.0, -0.006))
    release_recess = _filleted_box(0.036, 0.022, 0.008, 0.004).translate((0.037, 0.0, 0.006))

    return bucket_shell.union(front_panel).union(handle).cut(release_recess)


def _x_cylinder(radius: float, length: float):
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((-length * 0.5, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_air_fryer")

    shell_steel = model.material("shell_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.11, 0.11, 0.12, 1.0))
    chrome = model.material("chrome", rgba=(0.90, 0.91, 0.93, 1.0))
    signal_red = model.material("signal_red", rgba=(0.78, 0.12, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "air_fryer_body"),
        material=shell_steel,
        name="shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_body(), "air_fryer_drawer"),
        material=drawer_black,
        name="drawer_body",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.045,
                0.026,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.056, 0.006, flare=0.06),
                grip=KnobGrip(style="fluted", count=20, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "air_fryer_timer_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="dial",
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.045,
                0.026,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.056, 0.006, flare=0.06),
                grip=KnobGrip(style="fluted", count=20, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "air_fryer_temperature_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="dial",
    )

    power_button = model.part("power_button")
    power_button.visual(
        mesh_from_cadquery(_x_cylinder(0.0105, 0.006), "air_fryer_power_button_bezel"),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=chrome,
        name="button_bezel",
    )
    power_button.visual(
        mesh_from_cadquery(_x_cylinder(0.0068, 0.010), "air_fryer_power_button_lens"),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=signal_red,
        name="button_lens",
    )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(_filleted_box(0.032, 0.020, 0.008, 0.004), "air_fryer_release_button"),
        origin=Origin(xyz=(0.0, 0.0, 0.00168)),
        material=signal_red,
        name="release_cap",
    )
    release_button.visual(
        Box((0.010, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=signal_red,
        name="release_stem",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_JOINT_X, 0.0, DRAWER_OPENING_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, -0.068, 0.254)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "body_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=temperature_dial,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.068, 0.254)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, 0.252)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.004),
    )
    model.articulation(
        "drawer_to_release_button",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=release_button,
        origin=Origin(xyz=(0.037, 0.0, 0.009)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.0035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    power_button = object_model.get_part("power_button")
    release_button = object_model.get_part("release_button")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    power_button_slide = object_model.get_articulation("body_to_power_button")
    release_button_slide = object_model.get_articulation("drawer_to_release_button")
    limits = drawer_slide.motion_limits

    ctx.allow_overlap(
        body,
        drawer,
        reason=(
            "The basket drawer is intentionally nested inside the hollow heating chamber; "
            "the retained insertion fit is represented with mesh-backed open shell geometry."
        ),
    )
    ctx.allow_isolated_part(
        release_button,
        reason=(
            "The safety release button is guided by a tiny running clearance inside the "
            "drawer-handle recess rather than by zero-clearance resting contact."
        ),
    )

    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        min_overlap=0.10,
        name="drawer stays centered in the air fryer opening",
    )

    rest_position = ctx.part_world_position(drawer)
    if limits is not None and limits.upper is not None:
        with ctx.pose({drawer_slide: limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.10,
                name="drawer remains aligned when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.088,
                name="drawer keeps retained insertion at full travel",
            )
            extended_position = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends outward from the front",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.10,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    power_rest = ctx.part_world_position(power_button)
    with ctx.pose({power_button_slide: 0.004}):
        power_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button presses inward",
        power_rest is not None
        and power_pressed is not None
        and power_pressed[0] < power_rest[0] - 0.002,
        details=f"rest={power_rest}, pressed={power_pressed}",
    )

    release_rest = ctx.part_world_position(release_button)
    with ctx.pose({release_button_slide: 0.0035}):
        release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button depresses downward",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[2] < release_rest[2] - 0.002,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
