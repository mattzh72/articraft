from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
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


WIDTH = 0.69
DEPTH = 0.73
HEIGHT = 1.11
WALL = 0.018

OPENING_CENTER_X = 0.04
OPENING_DEPTH = 0.46
OPENING_WIDTH = 0.46

CONSOLE_FRONT_X = -0.205
CONSOLE_DEPTH = 0.155
CONSOLE_HEIGHT = 0.175
CONSOLE_CENTER_X = CONSOLE_FRONT_X - (CONSOLE_DEPTH * 0.5)

HINGE_X = -0.195
HINGE_Z = HEIGHT + 0.015

TUB_TOP_Z = HEIGHT - 0.026
TUB_DEPTH = 0.56

DIAL_Z = HEIGHT + 0.095
BUTTON_Z = HEIGHT + 0.095
BUTTON_Y_POSITIONS = (-0.215, -0.145, -0.080, 0.080, 0.145, 0.215)
RELEASE_BUTTON_X = 0.325
CONTROL_FACE_X = CONSOLE_FRONT_X + 0.005


def _filleted_box(
    sx: float,
    sy: float,
    sz: float,
    radius: float,
    *,
    centered: tuple[bool, bool, bool] = (True, True, False),
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz, centered=centered).edges("|Z").fillet(radius)


def _build_cabinet_shape() -> cq.Workplane:
    cabinet = _filleted_box(DEPTH, WIDTH, HEIGHT, 0.032).faces("<Z").shell(-WALL)

    opening = (
        cq.Workplane("XY")
        .box(OPENING_DEPTH, OPENING_WIDTH, 0.080, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.050)
        .translate((OPENING_CENTER_X, 0.0, HEIGHT - 0.040))
    )
    cabinet = cabinet.cut(opening)

    console = (
        _filleted_box(CONSOLE_DEPTH, WIDTH - 0.032, CONSOLE_HEIGHT, 0.016)
        .translate((CONSOLE_CENTER_X, 0.0, HEIGHT))
    )
    cabinet = cabinet.union(console)

    dial_pocket = (
        cq.Workplane("XY")
        .box(0.048, 0.065, 0.075, centered=(False, True, True))
        .translate((CONTROL_FACE_X - 0.048, 0.0, DIAL_Z))
    )
    cabinet = cabinet.cut(dial_pocket)

    for y_pos in BUTTON_Y_POSITIONS:
        button_pocket = (
            cq.Workplane("XY")
            .box(0.048, 0.036, 0.018, centered=(False, True, True))
            .translate((CONTROL_FACE_X - 0.048, y_pos, BUTTON_Z))
        )
        cabinet = cabinet.cut(button_pocket)

    release_slot = (
        cq.Workplane("XY")
        .box(0.052, 0.016, 0.030, centered=(True, True, False))
        .translate((RELEASE_BUTTON_X, 0.0, HEIGHT - 0.030))
    )
    cabinet = cabinet.cut(release_slot)

    return cabinet


def _build_lid_shape() -> cq.Workplane:
    lid_depth = 0.54
    lid_width = 0.60
    lid_thickness = 0.026
    window_depth = 0.320
    window_width = 0.460
    window_x = 0.080

    lid = (
        cq.Workplane("XY")
        .box(lid_depth, lid_width, lid_thickness, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.0, -0.010))
    )

    window_cut = (
        cq.Workplane("XY")
        .box(window_depth, window_width, 0.060, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.050)
        .translate((window_x, 0.0, -0.020))
    )
    lid = lid.cut(window_cut)

    rear_rib = (
        cq.Workplane("XY")
        .box(0.022, lid_width - 0.040, 0.016, centered=(False, True, False))
        .translate((0.000, 0.0, -0.010))
    )
    lid = lid.union(rear_rib)

    handle_bar = (
        cq.Workplane("XY")
        .box(0.022, 0.220, 0.018, centered=(True, True, False))
        .translate((0.475, 0.0, 0.030))
    )
    handle_post_0 = (
        cq.Workplane("XY")
        .box(0.018, 0.020, 0.022, centered=(True, True, False))
        .translate((0.455, -0.092, 0.010))
    )
    handle_post_1 = (
        cq.Workplane("XY")
        .box(0.018, 0.020, 0.022, centered=(True, True, False))
        .translate((0.455, 0.092, 0.010))
    )

    return lid.union(handle_bar).union(handle_post_0).union(handle_post_1)


def _build_console_fascia_shape() -> cq.Workplane:
    fascia = (
        cq.Workplane("XY")
        .box(0.005, WIDTH - 0.060, 0.112, centered=(False, True, True))
        .translate((CONSOLE_FRONT_X, 0.0, DIAL_Z))
    )

    dial_hole = (
        cq.Workplane("XY")
        .box(0.012, 0.026, 0.026, centered=(False, True, True))
        .translate((CONSOLE_FRONT_X - 0.003, 0.0, DIAL_Z))
    )
    fascia = fascia.cut(dial_hole)

    for y_pos in BUTTON_Y_POSITIONS:
        button_hole = (
            cq.Workplane("XY")
            .box(0.012, 0.034, 0.016, centered=(False, True, True))
            .translate((CONSOLE_FRONT_X - 0.003, y_pos, BUTTON_Z))
        )
        fascia = fascia.cut(button_hole)

    return fascia


def _build_tub_shape() -> cq.Workplane:
    outer_radius = 0.276
    inner_radius = 0.248
    bottom_thickness = 0.028

    shell = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(TUB_DEPTH)
        .translate((0.0, 0.0, -TUB_DEPTH))
    )
    cavity = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(TUB_DEPTH - bottom_thickness)
        .translate((0.0, 0.0, -(TUB_DEPTH - bottom_thickness)))
    )
    tub = shell.cut(cavity)

    top_ring = (
        cq.Workplane("XY")
        .circle(outer_radius + 0.010)
        .circle(inner_radius - 0.006)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.006))
    )
    impeller = (
        cq.Workplane("XY")
        .circle(0.105)
        .extrude(0.012)
        .translate((0.0, 0.0, -TUB_DEPTH + bottom_thickness))
    )
    hub = (
        cq.Workplane("XY")
        .circle(0.038)
        .extrude(0.028)
        .translate((0.0, 0.0, -TUB_DEPTH + bottom_thickness))
    )

    return tub.union(top_ring).union(impeller).union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.95, 0.96, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    button_dark = model.material("button_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.45, 0.52, 0.60, 0.42))
    knob_silver = model.material("knob_silver", rgba=(0.73, 0.74, 0.76, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shape(), "washer_cabinet"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_console_fascia_shape(), "console_fascia"),
        material=trim_dark,
        name="console_fascia",
    )
    cabinet.visual(
        Box((0.022, 0.022, 0.005)),
        origin=Origin(xyz=(0.245, -0.220, HEIGHT + 0.0025)),
        material=trim_dark,
        name="lid_bumper_0",
    )
    cabinet.visual(
        Box((0.022, 0.022, 0.005)),
        origin=Origin(xyz=(0.245, 0.220, HEIGHT + 0.0025)),
        material=trim_dark,
        name="lid_bumper_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "washer_lid"),
        material=trim_dark,
        name="lid_frame",
    )
    lid.visual(
        Box((0.380, 0.520, 0.005)),
        origin=Origin(xyz=(0.240, 0.0, -0.001)),
        material=glass_tint,
        name="glass_window",
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_build_tub_shape(), "washer_tub"),
        material=stainless,
        name="tub_shell",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.090,
                0.042,
                body_style="skirted",
                top_diameter=0.078,
                skirt=KnobSkirt(0.104, 0.006, flare=0.05),
                grip=KnobGrip(style="fluted", count=28, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.001),
                bore=KnobBore(style="round", diameter=0.010),
                center=False,
            ),
            "selector_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_silver,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="dial_shaft",
    )

    for index, y_pos in enumerate(BUTTON_Y_POSITIONS):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.008, 0.048, 0.020)),
            origin=Origin(xyz=(0.004, 0.0, 0.0)),
            material=button_dark,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.031, 0.014)),
            origin=Origin(xyz=(-0.009, 0.0, 0.0)),
            material=button_dark,
            name="button_stem",
        )

        model.articulation(
            f"cabinet_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(CONTROL_FACE_X, y_pos, BUTTON_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.070, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button_dark,
        name="release_cap",
    )
    release_button.visual(
        Box((0.048, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=button_dark,
        name="release_stem",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(OPENING_CENTER_X, 0.0, TUB_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=12.0),
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(CONTROL_FACE_X, 0.0, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "cabinet_to_release_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=release_button,
        origin=Origin(xyz=(RELEASE_BUTTON_X, 0.0, HEIGHT - 0.001)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    dial = object_model.get_part("dial")
    release_button = object_model.get_part("release_button")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    dial_joint = object_model.get_articulation("cabinet_to_dial")
    tub_joint = object_model.get_articulation("cabinet_to_tub")
    release_joint = object_model.get_articulation("cabinet_to_release_button")

    ctx.allow_isolated_part(
        tub,
        reason="The rotating wash basket is intentionally represented inside a hidden suspension and bearing assembly within the cabinet.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            tub,
            lid,
            axes="xy",
            elem_b="glass_window",
            min_overlap=0.30,
            name="window stays centered over the wash tub",
        )

    closed_lid_aabb = None
    open_lid_aabb = None
    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0}):
            closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "closed lid sits just above the top deck",
        closed_lid_aabb is not None and HEIGHT <= closed_lid_aabb[0][2] <= HEIGHT + 0.020,
        details=f"closed={closed_lid_aabb}, top_deck_z={HEIGHT}",
    )

    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    release_rest = None
    release_pressed = None
    release_limits = release_joint.motion_limits
    if release_limits is not None and release_limits.upper is not None:
        with ctx.pose({release_joint: 0.0}):
            release_rest = ctx.part_world_position(release_button)
        with ctx.pose({release_joint: release_limits.upper}):
            release_pressed = ctx.part_world_position(release_button)

    ctx.check(
        "release button depresses into the deck",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[2] < release_rest[2] - 0.002,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    button_positions = []
    program_joint_ok = True
    for index in range(6):
        part = object_model.get_part(f"program_button_{index}")
        joint = object_model.get_articulation(f"cabinet_to_program_button_{index}")
        pos = ctx.part_world_position(part)
        if pos is not None:
            button_positions.append(pos)
        limits = joint.motion_limits
        program_joint_ok = program_joint_ok and (
            joint.articulation_type == ArticulationType.PRISMATIC
            and limits is not None
            and limits.upper is not None
            and limits.upper > 0.0
        )

    dial_pos = ctx.part_world_position(dial)
    left_count = 0
    right_count = 0
    if dial_pos is not None:
        left_count = sum(1 for pos in button_positions if pos[1] < dial_pos[1])
        right_count = sum(1 for pos in button_positions if pos[1] > dial_pos[1])

    ctx.check(
        "six separate program buttons flank the dial",
        program_joint_ok and len(button_positions) == 6 and left_count == 3 and right_count == 3,
        details=f"dial={dial_pos}, buttons={button_positions}",
    )

    ctx.check(
        "tub and dial use continuous rotation",
        tub_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in tub_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in dial_joint.axis) == (1.0, 0.0, 0.0),
        details=f"tub_axis={tub_joint.axis}, dial_axis={dial_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
