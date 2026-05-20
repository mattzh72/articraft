from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


BODY_W = 0.312
BODY_D = 0.392
BODY_H = 0.332

CAVITY_W = 0.188
CAVITY_D = 0.214
CAVITY_Y = -0.024
CAVITY_FLOOR_Z = 0.018

CHAMBER_W = 0.164
CHAMBER_D = 0.192
CHAMBER_H = 0.262
CHAMBER_WALL = 0.0028

LID_W = 0.246
LID_D = 0.244
LID_T = 0.025
WINDOW_W = 0.122
WINDOW_D = 0.148
WINDOW_Y0 = 0.046

BAND_ANGLE = math.radians(22.0)
BAND_CENTER_Y = 0.118
BAND_CENTER_Z = 0.276
BAND_THICKNESS = 0.020
BAND_FACE_Y = BAND_CENTER_Y + 0.5 * BAND_THICKNESS * math.sin(BAND_ANGLE)
BAND_FACE_Z = BAND_CENTER_Z + 0.5 * BAND_THICKNESS * math.cos(BAND_ANGLE)


def _body_shell_mesh():
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
    )
    cavity = (
        cq.Workplane("XY")
        .box(CAVITY_W, CAVITY_D, BODY_H - CAVITY_FLOOR_Z, centered=(True, True, False))
        .translate((0.0, CAVITY_Y, CAVITY_FLOOR_Z))
    )
    return mesh_from_cadquery(shell.cut(cavity), "bread_maker_shell")


def _chamber_mesh():
    outer = (
        cq.Workplane("XY")
        .box(CHAMBER_W, CHAMBER_D, CHAMBER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CHAMBER_W - 2.0 * CHAMBER_WALL,
            CHAMBER_D - 2.0 * CHAMBER_WALL,
            CHAMBER_H - CHAMBER_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CHAMBER_WALL))
    )
    return mesh_from_cadquery(outer.cut(inner), "baking_chamber")


def _lid_frame_mesh():
    lid = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.012)
    )
    window_cut = (
        cq.Workplane("XY")
        .box(WINDOW_W, WINDOW_D, LID_T + 0.004, centered=(True, False, False))
        .translate((0.0, WINDOW_Y0, -0.002))
    )
    front_handle = (
        cq.Workplane("XY")
        .box(0.092, 0.016, 0.010, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, LID_D - 0.012, 0.003))
    )
    return mesh_from_cadquery(lid.cut(window_cut).union(front_handle), "bread_maker_lid")


def _dial_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.021,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.064, 0.005, flare=0.06),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "program_dial",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_bread_maker")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.73, 0.75, 0.78, 1.0))
    chamber_metal = model.material("chamber_metal", rgba=(0.83, 0.85, 0.87, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.54, 0.56, 0.58, 1.0))
    button_green = model.material("button_green", rgba=(0.43, 0.66, 0.47, 1.0))
    button_red = model.material("button_red", rgba=(0.74, 0.36, 0.35, 1.0))
    latch_grey = model.material("latch_grey", rgba=(0.66, 0.68, 0.70, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.33, 0.40, 0.46, 0.35))
    foot_black = model.material("foot_black", rgba=(0.14, 0.14, 0.15, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((BODY_W - 0.036, BODY_D - 0.036, CAVITY_FLOOR_Z)),
        origin=Origin(xyz=(0.0, 0.0, CAVITY_FLOOR_Z * 0.5)),
        material=body_white,
        name="base_plate",
    )
    shell.visual(
        Box((0.030, BODY_D - 0.056, BODY_H - CAVITY_FLOOR_Z)),
        origin=Origin(xyz=(-BODY_W * 0.5 + 0.015, 0.0, (BODY_H + CAVITY_FLOOR_Z) * 0.5)),
        material=body_white,
        name="side_wall_0",
    )
    shell.visual(
        Box((0.030, BODY_D - 0.056, BODY_H - CAVITY_FLOOR_Z)),
        origin=Origin(xyz=(BODY_W * 0.5 - 0.015, 0.0, (BODY_H + CAVITY_FLOOR_Z) * 0.5)),
        material=body_white,
        name="side_wall_1",
    )
    shell.visual(
        Box((BODY_W - 0.060, 0.032, BODY_H - CAVITY_FLOOR_Z)),
        origin=Origin(
            xyz=(0.0, -BODY_D * 0.5 + 0.016, (BODY_H + CAVITY_FLOOR_Z) * 0.5)
        ),
        material=body_white,
        name="rear_wall",
    )
    shell.visual(
        Box((BODY_W - 0.060, 0.060, BODY_H - CAVITY_FLOOR_Z)),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - 0.030, (BODY_H + CAVITY_FLOOR_Z) * 0.5)
        ),
        material=body_white,
        name="front_wall",
    )
    shell.visual(
        Box((0.226, 0.056, 0.018)),
        origin=Origin(xyz=(0.0, 0.109, BODY_H - 0.009)),
        material=body_white,
        name="front_deck",
    )
    shell.visual(
        Box((0.172, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, -0.151, BODY_H - 0.007)),
        material=body_white,
        name="rear_bridge",
    )
    shell.visual(
        Box((0.024, CAVITY_D + 0.020, 0.012)),
        origin=Origin(
            xyz=(-CAVITY_W * 0.5 - 0.018, CAVITY_Y, BODY_H - 0.006)
        ),
        material=body_white,
        name="top_rail_0",
    )
    shell.visual(
        Box((0.024, CAVITY_D + 0.020, 0.012)),
        origin=Origin(
            xyz=(CAVITY_W * 0.5 + 0.018, CAVITY_Y, BODY_H - 0.006)
        ),
        material=body_white,
        name="top_rail_1",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shell.visual(
                Cylinder(radius=0.028, length=BODY_H),
                origin=Origin(
                    xyz=(
                        sx * (BODY_W * 0.5 - 0.028),
                        sy * (BODY_D * 0.5 - 0.028),
                        BODY_H * 0.5,
                    )
                ),
                material=body_white,
                name=f"corner_post_{int((sx + 1.0) * 0.5)}_{int((sy + 1.0) * 0.5)}",
            )
    shell.visual(
        Box((0.262, 0.072, BAND_THICKNESS)),
        origin=Origin(xyz=(0.0, BAND_CENTER_Y, BAND_CENTER_Z), rpy=(-BAND_ANGLE, 0.0, 0.0)),
        material=panel_grey,
        name="control_band",
    )
    shell.visual(
        Box((0.108, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, CAVITY_Y + CAVITY_D * 0.5 + 0.008, BODY_H - 0.005)),
        material=panel_grey,
        name="latch_lip",
    )
    shell.visual(
        Box((0.082, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, CAVITY_Y - CAVITY_D * 0.5 - 0.008, BODY_H - 0.002)),
        material=panel_grey,
        name="rear_hinge_cover",
    )
    for sx in (-0.112, 0.112):
        for sy in (-0.145, 0.145):
            shell.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(sx, sy, -0.004)),
                material=foot_black,
                name=f"foot_{int((sx > 0))}_{int((sy > 0))}",
            )

    chamber = model.part("chamber")
    chamber.visual(_chamber_mesh(), material=chamber_metal, name="chamber_shell")
    model.articulation(
        "shell_to_chamber",
        ArticulationType.FIXED,
        parent=shell,
        child=chamber,
        origin=Origin(xyz=(0.0, CAVITY_Y, CAVITY_FLOOR_Z)),
    )

    lid = model.part("lid")
    lid.visual(_lid_frame_mesh(), material=body_white, name="lid_frame")
    lid.visual(
        Box((0.142, 0.168, 0.002)),
        origin=Origin(xyz=(0.0, 0.084, 0.004)),
        material=glass_tint,
        name="window_pane",
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, CAVITY_Y - CAVITY_D * 0.5 - 0.014, BODY_H - 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    spindle = model.part("kneading_spindle")
    spindle.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=spindle_metal,
        name="spindle_collar",
    )
    spindle.visual(
        Cylinder(radius=0.0065, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=spindle_metal,
        name="shaft",
    )
    spindle.visual(
        Box((0.012, 0.006, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, 0.040)),
        material=spindle_metal,
        name="drive_tang",
    )
    model.articulation(
        "chamber_to_kneading_spindle",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_WALL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    dial = model.part("program_dial")
    dial.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=panel_grey,
        name="dial_collar",
    )
    dial.visual(
        _dial_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=knob_silver,
        name="dial_knob",
    )
    model.articulation(
        "shell_to_program_dial",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=dial,
        origin=Origin(xyz=(0.0, BAND_FACE_Y, BAND_FACE_Z), rpy=(-BAND_ANGLE, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=panel_grey,
        name="button_collar",
    )
    start_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=button_green,
        name="button_cap",
    )
    model.articulation(
        "shell_to_start_button",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=start_button,
        origin=Origin(xyz=(-0.083, BAND_FACE_Y, BAND_FACE_Z), rpy=(-BAND_ANGLE, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.005),
    )

    stop_button = model.part("stop_button")
    stop_button.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=panel_grey,
        name="button_collar",
    )
    stop_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=button_red,
        name="button_cap",
    )
    model.articulation(
        "shell_to_stop_button",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=stop_button,
        origin=Origin(xyz=(0.083, BAND_FACE_Y, BAND_FACE_Z), rpy=(-BAND_ANGLE, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.005),
    )

    latch_button = model.part("front_latch_button")
    latch_button.visual(
        Box((0.066, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=latch_grey,
        name="latch_button",
    )
    model.articulation(
        "shell_to_front_latch_button",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=latch_button,
        origin=Origin(xyz=(0.0, BODY_D * 0.5, BODY_H - 0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.06, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("program_dial")
    start_button = object_model.get_part("start_button")
    stop_button = object_model.get_part("stop_button")
    latch_button = object_model.get_part("front_latch_button")

    lid_hinge = object_model.get_articulation("shell_to_lid")
    spindle_joint = object_model.get_articulation("chamber_to_kneading_spindle")
    dial_joint = object_model.get_articulation("shell_to_program_dial")
    start_joint = object_model.get_articulation("shell_to_start_button")
    stop_joint = object_model.get_articulation("shell_to_stop_button")
    latch_joint = object_model.get_articulation("shell_to_front_latch_button")

    ctx.check(
        "lid articulation is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_hinge.articulation_type!r}",
    )
    ctx.check(
        "kneading spindle articulation is continuous",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spindle_joint.articulation_type!r}",
    )
    ctx.check(
        "program dial articulation is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "start button articulation is prismatic",
        start_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={start_joint.articulation_type!r}",
    )
    ctx.check(
        "stop button articulation is prismatic",
        stop_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={stop_joint.articulation_type!r}",
    )
    ctx.check(
        "front latch articulation is prismatic",
        latch_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={latch_joint.articulation_type!r}",
    )

    ctx.allow_overlap(
        shell,
        dial,
        elem_a="front_wall",
        elem_b="dial_knob",
        reason="The dial's rear skirt is intentionally represented as seating into the simplified front housing behind the sloped fascia.",
    )
    ctx.allow_overlap(
        shell,
        dial,
        elem_a="front_deck",
        elem_b="dial_knob",
        reason="The dial knob sits on the sloped control fascia while the simplified front deck stands in for the housing volume tucked beneath it.",
    )
    ctx.allow_overlap(
        shell,
        start_button,
        elem_a="control_band",
        elem_b="button_collar",
        reason="The start button collar is intentionally represented as seated into the thin control fascia.",
    )
    ctx.allow_overlap(
        shell,
        stop_button,
        elem_a="control_band",
        elem_b="button_collar",
        reason="The stop button collar is intentionally represented as seated into the thin control fascia.",
    )

    ctx.expect_within(
        chamber,
        shell,
        axes="xy",
        margin=0.008,
        name="chamber stays inside shell footprint",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="window_pane",
            negative_elem="chamber_shell",
            min_gap=0.020,
            max_gap=0.080,
            name="closed lid leaves visible headroom above chamber",
        )
        ctx.expect_overlap(
            lid,
            chamber,
            axes="xy",
            elem_a="lid_frame",
            elem_b="chamber_shell",
            min_overlap=0.120,
            name="lid covers the chamber opening",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_handle_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    ctx.check(
        "lid opens upward",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.14,
        details=f"closed={closed_handle_aabb!r}, open={open_handle_aabb!r}",
    )

    start_rest = ctx.part_world_position(start_button)
    with ctx.pose({start_joint: start_joint.motion_limits.upper}):
        start_pressed = ctx.part_world_position(start_button)
    ctx.check(
        "start button depresses into the control band",
        start_rest is not None
        and start_pressed is not None
        and start_pressed[1] < start_rest[1] - 0.001
        and start_pressed[2] < start_rest[2] - 0.003,
        details=f"rest={start_rest!r}, pressed={start_pressed!r}",
    )

    stop_rest = ctx.part_world_position(stop_button)
    with ctx.pose({stop_joint: stop_joint.motion_limits.upper}):
        stop_pressed = ctx.part_world_position(stop_button)
    ctx.check(
        "stop button depresses into the control band",
        stop_rest is not None
        and stop_pressed is not None
        and stop_pressed[1] < stop_rest[1] - 0.001
        and stop_pressed[2] < stop_rest[2] - 0.003,
        details=f"rest={stop_rest!r}, pressed={stop_pressed!r}",
    )

    latch_rest = ctx.part_world_position(latch_button)
    with ctx.pose({latch_joint: latch_joint.motion_limits.upper}):
        latch_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "front latch button slides into the shell",
        latch_rest is not None
        and latch_pressed is not None
        and latch_pressed[1] < latch_rest[1] - 0.006,
        details=f"rest={latch_rest!r}, pressed={latch_pressed!r}",
    )

    return ctx.report()


object_model = build_object_model()
