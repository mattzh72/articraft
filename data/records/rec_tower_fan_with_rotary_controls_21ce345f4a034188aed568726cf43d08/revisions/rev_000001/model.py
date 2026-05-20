from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.36
BASE_DEPTH = 0.245
BASE_HEIGHT = 0.030
BASE_BEARING_RADIUS = 0.038
BASE_BEARING_HEIGHT = 0.012

BODY_WIDTH = 0.155
BODY_DEPTH = 0.105
BODY_SHELL_BOTTOM = 0.055
BODY_SHELL_HEIGHT = 0.875
BODY_CORNER_RADIUS = 0.026
BODY_WALL = 0.0032
BODY_TOP = BODY_SHELL_BOTTOM + BODY_SHELL_HEIGHT

VENT_WIDTH = 0.112
VENT_HEIGHT = 0.640
VENT_BOTTOM = 0.115

GRILLE_WIDTH = 0.122
GRILLE_HEIGHT = 0.660
GRILLE_THICKNESS = 0.005
GRILLE_FRAME = 0.007

DECK_WIDTH = 0.132
DECK_DEPTH = 0.086
DECK_THICKNESS = 0.006
DECK_Y = 0.010

HANDLE_WIDTH = 0.088
HANDLE_DEPTH = 0.026
HANDLE_Y = -0.022
HANDLE_CUT_BOTTOM = BODY_SHELL_HEIGHT - 0.018

WHEEL_RADIUS = 0.039
WHEEL_INNER_RADIUS = 0.018
WHEEL_WIDTH = 0.640
WHEEL_CENTER_Y = 0.006
WHEEL_CENTER_Z = 0.435

SPEED_X = -0.034
TIMER_X = 0.034
DIAL_Y = DECK_Y
SPEED_DIAMETER = 0.032
SPEED_HEIGHT = 0.018
TIMER_DIAMETER = 0.036
TIMER_HEIGHT = 0.020


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _build_base_shape():
    base = cq.Workplane("XY").ellipse(BASE_WIDTH * 0.5, BASE_DEPTH * 0.5).extrude(BASE_HEIGHT)
    upper_pad = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT * 0.45)
        .ellipse(BASE_WIDTH * 0.44, BASE_DEPTH * 0.42)
        .extrude(BASE_HEIGHT * 0.55)
    )
    return base.union(upper_pad).edges(">Z").fillet(0.009)


def _build_body_shell():
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_SHELL_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
        .edges(">Z")
        .fillet(0.010)
        .faces("<Z")
        .shell(-BODY_WALL)
    )

    vent_cutter = (
        cq.Workplane("XY")
        .box(VENT_WIDTH, 0.032, VENT_HEIGHT, centered=(True, True, False))
        .translate((0.0, BODY_DEPTH * 0.5 - 0.016, VENT_BOTTOM))
    )
    handle_cutter = (
        cq.Workplane("XY")
        .box(HANDLE_WIDTH, HANDLE_DEPTH, 0.030, centered=(True, True, False))
        .translate((0.0, HANDLE_Y, HANDLE_CUT_BOTTOM))
    )
    return shell.cut(vent_cutter).cut(handle_cutter)


def _build_control_deck():
    return (
        cq.Workplane("XY")
        .box(DECK_WIDTH, DECK_DEPTH, DECK_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.0025)
    )


def _build_neck_fairing():
    return (
        cq.Workplane("XY")
        .ellipse(0.052, 0.036)
        .workplane(offset=0.022)
        .ellipse(0.064, 0.042)
        .workplane(offset=0.028)
        .ellipse(0.074, 0.049)
        .loft(combine=True)
    )


def _build_grille():
    outer = cq.Workplane("XY").box(GRILLE_WIDTH, GRILLE_THICKNESS, GRILLE_HEIGHT)
    inner = cq.Workplane("XY").box(
        GRILLE_WIDTH - 2.0 * GRILLE_FRAME,
        GRILLE_THICKNESS + 0.002,
        GRILLE_HEIGHT - 2.0 * GRILLE_FRAME,
    )
    grille = outer.cut(inner)

    slat_width = 0.004
    slat_span = GRILLE_WIDTH - 2.0 * (GRILLE_FRAME + 0.008)
    slat_count = 8
    for index in range(slat_count):
        if slat_count == 1:
            x = 0.0
        else:
            x = -slat_span * 0.5 + slat_span * index / (slat_count - 1)
        slat = cq.Workplane("XY").box(
            slat_width,
            GRILLE_THICKNESS,
            GRILLE_HEIGHT - 2.0 * GRILLE_FRAME,
        )
        grille = grille.union(slat.translate((x, 0.0, 0.0)))

    return grille.edges("|Y").fillet(0.0015)


def _add_dial_part(
    model: ArticulatedObject,
    *,
    name: str,
    mesh_name: str,
    diameter: float,
    height: float,
    body_material: str,
    pointer_material: str,
):
    dial = model.part(name)
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                diameter,
                height,
                body_style="skirted",
                top_diameter=diameter * 0.82,
                base_diameter=diameter * 1.06,
                edge_radius=0.0015,
                center=False,
            ),
            mesh_name,
        ),
        material=body_material,
        name="cap",
    )
    dial.visual(
        Box((diameter * 0.08, diameter * 0.36, 0.0018)),
        origin=Origin(xyz=(0.0, diameter * 0.26, height - 0.0009)),
        material=pointer_material,
        name="pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=diameter * 0.53, length=height),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )
    return dial


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedroom_tower_fan")

    base_dark = model.material("base_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    shell_white = model.material("shell_white", rgba=(0.90, 0.91, 0.88, 1.0))
    deck_black = model.material("deck_black", rgba=(0.10, 0.11, 0.12, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.23, 0.24, 0.25, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    dial_black = model.material("dial_black", rgba=(0.12, 0.12, 0.13, 1.0))
    pointer_silver = model.material("pointer_silver", rgba=(0.76, 0.77, 0.78, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_shape(), "tower_fan_base"), material=base_dark, name="base_shell")
    base.visual(
        Cylinder(radius=BASE_BEARING_RADIUS, length=BASE_BEARING_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + BASE_BEARING_HEIGHT * 0.5)),
        material=base_dark,
        name="base_bearing",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT + BASE_BEARING_HEIGHT)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + BASE_BEARING_HEIGHT) * 0.5)),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.030, length=BODY_SHELL_BOTTOM),
        origin=Origin(xyz=(0.0, 0.0, BODY_SHELL_BOTTOM * 0.5)),
        material=base_dark,
        name="pivot_housing",
    )
    body.visual(
        mesh_from_cadquery(_build_neck_fairing(), "tower_fan_neck_fairing"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=shell_white,
        name="neck",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.874),
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, 0.497)),
        material=base_dark,
        name="motor_spine",
    )
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "tower_fan_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_SHELL_BOTTOM)),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((0.110, 0.064, 0.004)),
        origin=Origin(xyz=(0.0, DECK_Y, BODY_TOP)),
        material=shell_white,
        name="deck_plinth",
    )
    body.visual(
        mesh_from_cadquery(_build_control_deck(), "tower_fan_control_deck"),
        origin=Origin(xyz=(0.0, DECK_Y, BODY_TOP)),
        material=deck_black,
        name="control_deck",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_TOP)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP * 0.5)),
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_cadquery(_build_grille(), "tower_fan_grille"),
        material=grille_dark,
        name="grille_face",
    )
    grille.inertial = Inertial.from_geometry(
        Box((GRILLE_WIDTH, GRILLE_THICKNESS, GRILLE_HEIGHT)),
        mass=0.18,
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                WHEEL_RADIUS,
                WHEEL_INNER_RADIUS,
                WHEEL_WIDTH,
                26,
                blade_thickness=0.0024,
                blade_sweep_deg=28.0,
            ),
            "tower_fan_blower_wheel",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wheel_dark,
        name="wheel",
    )
    blower_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.32,
    )

    speed_dial = _add_dial_part(
        model,
        name="speed_dial",
        mesh_name="tower_fan_speed_dial",
        diameter=SPEED_DIAMETER,
        height=SPEED_HEIGHT,
        body_material="dial_black",
        pointer_material="pointer_silver",
    )
    timer_knob = _add_dial_part(
        model,
        name="timer_knob",
        mesh_name="tower_fan_timer_knob",
        diameter=TIMER_DIAMETER,
        height=TIMER_HEIGHT,
        body_material="dial_black",
        pointer_material="pointer_silver",
    )

    oscillation = model.articulation(
        "body_oscillation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + BASE_BEARING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-math.radians(65.0),
            upper=math.radians(65.0),
        ),
    )

    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 + GRILLE_THICKNESS * 0.5,
                BODY_SHELL_BOTTOM + VENT_BOTTOM + VENT_HEIGHT * 0.5,
            )
        ),
    )
    blower_spin = model.articulation(
        "body_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=22.0),
    )
    speed_spin = model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=(SPEED_X, DIAL_Y, BODY_TOP + DECK_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.1, velocity=6.0),
    )
    timer_spin = model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(TIMER_X, DIAL_Y, BODY_TOP + DECK_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.1, velocity=6.0),
    )

    oscillation.meta["qc_samples"] = [0.0, math.radians(35.0), math.radians(-35.0)]
    blower_spin.meta["qc_samples"] = [0.0, math.pi / 5.0]
    speed_spin.meta["qc_samples"] = [0.0, math.pi / 2.0]
    timer_spin.meta["qc_samples"] = [0.0, math.pi / 2.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    grille = object_model.get_part("grille")
    blower_wheel = object_model.get_part("blower_wheel")
    speed_dial = object_model.get_part("speed_dial")
    timer_knob = object_model.get_part("timer_knob")

    oscillation = object_model.get_articulation("body_oscillation")
    blower_spin = object_model.get_articulation("body_to_blower_wheel")
    speed_spin = object_model.get_articulation("body_to_speed_dial")
    timer_spin = object_model.get_articulation("body_to_timer_knob")

    ctx.allow_overlap(
        body,
        blower_wheel,
        elem_a="motor_spine",
        elem_b="wheel",
        reason="The blower wheel is intentionally represented around a simplified central drive spine inside the tower shell.",
    )

    ctx.expect_contact(
        body,
        base,
        elem_a="pivot_housing",
        elem_b="base_bearing",
        name="body mounts on the base bearing",
    )
    ctx.expect_contact(
        grille,
        body,
        elem_a="grille_face",
        elem_b="body_shell",
        contact_tol=0.001,
        name="front grille mounts flush to the body shell",
    )
    ctx.expect_within(
        blower_wheel,
        body,
        axes="xy",
        inner_elem="wheel",
        outer_elem="body_shell",
        margin=0.012,
        name="blower wheel stays within the shell footprint",
    )
    ctx.expect_overlap(
        blower_wheel,
        body,
        axes="z",
        elem_a="wheel",
        elem_b="body_shell",
        min_overlap=0.58,
        name="blower wheel spans the tower outlet height",
    )
    ctx.expect_contact(
        speed_dial,
        body,
        elem_a="cap",
        elem_b="control_deck",
        contact_tol=0.001,
        name="speed dial seats on the control deck",
    )
    ctx.expect_contact(
        timer_knob,
        body,
        elem_a="cap",
        elem_b="control_deck",
        contact_tol=0.001,
        name="timer knob seats on the control deck",
    )
    ctx.expect_origin_distance(
        speed_dial,
        timer_knob,
        axes="x",
        min_dist=0.055,
        max_dist=0.080,
        name="top dials sit side by side on the control deck",
    )
    ctx.expect_origin_distance(
        speed_dial,
        timer_knob,
        axes="y",
        min_dist=0.0,
        max_dist=0.004,
        name="top dials share one control row",
    )

    grille_rest = ctx.part_world_position(grille)
    with ctx.pose({oscillation: math.radians(55.0)}):
        grille_turned = ctx.part_world_position(grille)
    moved = (
        grille_rest is not None
        and grille_turned is not None
        and math.hypot(grille_turned[0] - grille_rest[0], grille_turned[1] - grille_rest[1]) > 0.040
    )
    ctx.check(
        "body oscillates about the base",
        moved,
        details=f"rest={grille_rest}, turned={grille_turned}",
    )

    speed_pointer_rest = _aabb_center(ctx.part_element_world_aabb(speed_dial, elem="pointer"))
    with ctx.pose({speed_spin: math.pi / 2.0}):
        speed_pointer_turned = _aabb_center(ctx.part_element_world_aabb(speed_dial, elem="pointer"))
    speed_turns = (
        speed_pointer_rest is not None
        and speed_pointer_turned is not None
        and math.hypot(
            speed_pointer_turned[0] - speed_pointer_rest[0],
            speed_pointer_turned[1] - speed_pointer_rest[1],
        )
        > 0.008
    )
    ctx.check(
        "speed dial rotates visibly on its shaft",
        speed_turns,
        details=f"rest={speed_pointer_rest}, turned={speed_pointer_turned}",
    )

    timer_pointer_rest = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
    with ctx.pose({timer_spin: math.pi / 2.0}):
        timer_pointer_turned = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
    timer_turns = (
        timer_pointer_rest is not None
        and timer_pointer_turned is not None
        and math.hypot(
            timer_pointer_turned[0] - timer_pointer_rest[0],
            timer_pointer_turned[1] - timer_pointer_rest[1],
        )
        > 0.008
    )
    ctx.check(
        "timer knob rotates visibly on its shaft",
        timer_turns,
        details=f"rest={timer_pointer_rest}, turned={timer_pointer_turned}",
    )

    with ctx.pose({blower_spin: math.pi / 5.0}):
        ctx.expect_within(
            blower_wheel,
            body,
            axes="xy",
            inner_elem="wheel",
            outer_elem="body_shell",
            margin=0.012,
            name="blower wheel remains nested in the shell while spinning",
        )

    return ctx.report()


object_model = build_object_model()
