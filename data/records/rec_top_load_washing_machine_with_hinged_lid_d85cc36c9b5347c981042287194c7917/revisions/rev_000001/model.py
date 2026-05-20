from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_D = 0.72
CABINET_W = 0.69
CABINET_H = 0.98
WALL = 0.016
DECK_T = 0.042

OPENING_X = 0.05
OPENING_R = 0.185

CONSOLE_D = 0.12
CONSOLE_W = 0.60
CONSOLE_H = 0.14
CONSOLE_X = -0.30
CONSOLE_FRONT_X = CONSOLE_X + CONSOLE_D / 2.0

TUB_BOTTOM_Z = 0.445
TUB_H = 0.46
TUB_R = 0.168
TUB_WALL = 0.010

LID_D = 0.45
LID_W = 0.50
LID_T = 0.028
LID_FRAME = 0.045
LID_GLASS_T = 0.006
LID_HINGE_X = -0.17
LID_HINGE_Z = CABINET_H + 0.002

DRAWER_L = 0.165
DRAWER_W = 0.070
DRAWER_H = 0.032
DRAWER_FRONT_X = 0.29
DRAWER_Y = -0.300
DRAWER_TRAVEL = 0.09

BLEACH_L = 0.090
BLEACH_W = 0.066
BLEACH_T = 0.006
BLEACH_Y = 0.255
BLEACH_HINGE_X = -0.21

BUTTON_T = 0.012
BUTTON_W = 0.055
BUTTON_H = 0.018
BUTTON_STEM_T = 0.010
BUTTON_STEM_W = 0.028
BUTTON_STEM_H = 0.010
BUTTON_TRAVEL = 0.008
BUTTON_LAYOUT = (
    (-0.20, CABINET_H + 0.020),
    (-0.10, CABINET_H + 0.020),
    (0.00, CABINET_H + 0.020),
    (0.10, CABINET_H + 0.020),
    (0.20, CABINET_H + 0.020),
)

DIAL_Z = CABINET_H + 0.084


def _cabinet_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(CABINET_D, CABINET_W, CABINET_H, centered=(True, True, False))
    shell = shell.cut(
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(CABINET_D - 2.0 * WALL, CABINET_W - 2.0 * WALL, CABINET_H, centered=(True, True, False))
    )

    deck = (
        cq.Workplane("XY")
        .workplane(offset=CABINET_H - DECK_T)
        .box(CABINET_D, CABINET_W, DECK_T, centered=(True, True, False))
    )
    cabinet = shell.union(deck)

    opening_cut = (
        cq.Workplane("XY")
        .workplane(offset=CABINET_H - DECK_T - 0.002)
        .center(OPENING_X, 0.0)
        .circle(OPENING_R)
        .extrude(DECK_T + 0.010)
    )
    cabinet = cabinet.cut(opening_cut)

    bleach_cut = (
        cq.Workplane("XY")
        .workplane(offset=CABINET_H - DECK_T - 0.002)
        .center(BLEACH_HINGE_X + BLEACH_L / 2.0, BLEACH_Y)
        .circle(0.026)
        .extrude(DECK_T + 0.012)
    )
    cabinet = cabinet.cut(bleach_cut)

    return cabinet


def _console_shape() -> cq.Workplane:
    console = (
        cq.Workplane("XY")
        .box(CONSOLE_D, CONSOLE_W, CONSOLE_H, centered=(True, True, False))
        .translate((CONSOLE_X, 0.0, CABINET_H))
    )

    for idx, (y_pos, z_pos) in enumerate(BUTTON_LAYOUT):
        pocket = (
            cq.Workplane("XY")
            .box(0.018, BUTTON_STEM_W + 0.004, BUTTON_STEM_H + 0.004, centered=(False, True, True))
            .translate((CONSOLE_FRONT_X - 0.018, y_pos, z_pos))
        )
        console = console.cut(pocket)

    return console


def _lid_frame_shape() -> cq.Workplane:
    lid = cq.Workplane("XY").box(LID_D, LID_W, LID_T, centered=(False, True, False))
    lid = lid.cut(
        cq.Workplane("XY")
        .center(LID_D / 2.0 + 0.010, 0.0)
        .box(
            LID_D - 2.0 * LID_FRAME,
            LID_W - 2.0 * LID_FRAME,
            LID_T + 0.004,
            centered=(True, True, False),
        )
    )
    return lid


def _tub_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(TUB_R).extrude(TUB_H)
    inner = cq.Workplane("XY").workplane(offset=0.028).circle(TUB_R - TUB_WALL).extrude(TUB_H)
    return outer.cut(inner)


def _drawer_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(DRAWER_L, DRAWER_W, DRAWER_H, centered=(False, True, False))
        .translate((-DRAWER_L, 0.0, 0.0))
    )


def _dial_mesh_name() -> str:
    return "dial_knob"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_top_load_washer")

    white = model.material("white_enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    graphite = model.material("graphite_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    control = model.material("control_panel", rgba=(0.11, 0.12, 0.14, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.75, 0.78, 1.0))
    chrome = model.material("chrome", rgba=(0.76, 0.78, 0.80, 1.0))
    glass = model.material("glass_tint", rgba=(0.22, 0.30, 0.35, 0.42))
    button_material = model.material("button_black", rgba=(0.09, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_cabinet_shell_shape(), "cabinet_shell"),
        material=white,
        name="cabinet_shell",
    )
    body.visual(
        mesh_from_cadquery(_console_shape(), "console"),
        material=control,
        name="console",
    )
    body.visual(
        Cylinder(radius=0.045, length=TUB_BOTTOM_Z),
        origin=Origin(xyz=(OPENING_X, 0.0, TUB_BOTTOM_Z / 2.0)),
        material=graphite,
        name="tub_pedestal",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame_shape(), "lid_frame"),
        material=graphite,
        name="lid_frame",
    )
    lid.visual(
        Box((LID_D - 2.0 * LID_FRAME + 0.004, LID_W - 2.0 * LID_FRAME + 0.004, LID_GLASS_T)),
        origin=Origin(
            xyz=(
                LID_FRAME + (LID_D - 2.0 * LID_FRAME) / 2.0,
                0.0,
                LID_T - LID_GLASS_T / 2.0 - 0.003,
            )
        ),
        material=glass,
        name="lid_glass",
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_tub_shape(), "wash_tub"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="drum",
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "detergent_drawer"),
        material=white,
        name="drawer_tray",
    )

    flap = model.part("bleach_flap")
    flap.visual(
        Box((BLEACH_L, BLEACH_W, BLEACH_T)),
        origin=Origin(xyz=(BLEACH_L / 2.0, 0.0, BLEACH_T / 2.0)),
        material=white,
        name="flap_cover",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.078,
                0.032,
                body_style="skirted",
                top_diameter=0.064,
                base_diameter=0.078,
                edge_radius=0.002,
                center=False,
            ),
            _dial_mesh_name(),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="dial_knob",
    )

    button_parts = []
    for idx, _ in enumerate(BUTTON_LAYOUT):
        button = model.part(f"program_button_{idx}")
        button.visual(
            Box((BUTTON_T, BUTTON_W, BUTTON_H)),
            origin=Origin(xyz=(BUTTON_T / 2.0, 0.0, 0.0)),
            material=button_material,
            name="cap",
        )
        button.visual(
            Box((BUTTON_STEM_T, BUTTON_STEM_W, BUTTON_STEM_H)),
            origin=Origin(xyz=(-BUTTON_STEM_T / 2.0, 0.0, 0.0)),
            material=button_material,
            name="stem",
        )
        button_parts.append(button)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=12.0, velocity=1.3),
    )
    model.articulation(
        "body_to_tub",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tub,
        origin=Origin(xyz=(OPENING_X, 0.0, TUB_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "body_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_FRONT_X, DRAWER_Y, CABINET_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=18.0, velocity=0.22),
    )
    model.articulation(
        "body_to_bleach_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(BLEACH_HINGE_X, BLEACH_Y, CABINET_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=2.0, velocity=1.5),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(CONSOLE_FRONT_X, 0.0, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    for idx, ((y_pos, z_pos), button) in enumerate(zip(BUTTON_LAYOUT, button_parts)):
        model.articulation(
            f"body_to_program_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(CONSOLE_FRONT_X, y_pos, z_pos)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=3.0, velocity=0.08),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    drawer = object_model.get_part("detergent_drawer")
    flap = object_model.get_part("bleach_flap")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("body_to_lid")
    drawer_slide = object_model.get_articulation("body_to_detergent_drawer")
    flap_hinge = object_model.get_articulation("body_to_bleach_flap")
    tub_spin = object_model.get_articulation("body_to_tub")
    dial_spin = object_model.get_articulation("body_to_dial")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="cabinet_shell",
        min_gap=0.0,
        max_gap=0.008,
        name="lid sits just above the top deck",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_frame",
        elem_b="cabinet_shell",
        min_overlap=0.30,
        name="lid covers the opening footprint",
    )
    tub_aabb = ctx.part_world_aabb(tub)
    tub_top_clearance = None if tub_aabb is None else CABINET_H - tub_aabb[1][2]
    ctx.check(
        "washer opening sits above the tub rim",
        tub_top_clearance is not None and 0.02 <= tub_top_clearance <= 0.10,
        details=f"tub_aabb={tub_aabb}, top_clearance={tub_top_clearance}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "detergent drawer slides forward",
        drawer_rest is not None and drawer_extended is not None and drawer_extended[0] > drawer_rest[0] + 0.08,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    flap_closed = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        flap_open = ctx.part_world_aabb(flap)
    ctx.check(
        "bleach flap lifts from the rear edge",
        flap_closed is not None and flap_open is not None and flap_open[1][2] > flap_closed[1][2] + 0.04,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    ctx.check(
        "tub spins on the vertical axis",
        tub_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(tub_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={tub_spin.articulation_type}, axis={tub_spin.axis}",
    )
    ctx.check(
        "dial is a continuous rotary control",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}",
    )

    for idx in range(len(BUTTON_LAYOUT)):
        button = object_model.get_part(f"program_button_{idx}")
        button_joint = object_model.get_articulation(f"body_to_program_button_{idx}")
        rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"program button {idx} presses inward",
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
