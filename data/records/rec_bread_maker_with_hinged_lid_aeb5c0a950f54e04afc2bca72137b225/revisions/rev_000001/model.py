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


BODY_WIDTH = 0.420
BODY_DEPTH = 0.305
BODY_HEIGHT = 0.295

CONSOLE_WIDTH = 0.360
CONSOLE_DEPTH = 0.092
CONSOLE_HEIGHT = 0.063
CONSOLE_BOTTOM_Z = BODY_HEIGHT - 0.010
CONSOLE_CENTER_Y = -BODY_DEPTH * 0.5 + CONSOLE_DEPTH * 0.5

PANEL_SURFACE_Y = -BODY_DEPTH * 0.5

CHAMBER_CENTER_Y = 0.035
CHAMBER_FLOOR_Z = 0.040
CHAMBER_CAVITY_WIDTH = 0.184
CHAMBER_CAVITY_DEPTH = 0.144
CHAMBER_CAVITY_HEIGHT = 0.270

PAN_WIDTH = 0.168
PAN_DEPTH = 0.128
PAN_HEIGHT = 0.220
PAN_WALL = 0.003
PAN_BOSS_RADIUS = 0.016
PAN_BOSS_HEIGHT = 0.012

LID_WIDTH = 0.328
LID_DEPTH = 0.199
LID_THICKNESS = 0.030
LID_TOP_SKIN = 0.006
LID_HINGE_Y = BODY_DEPTH * 0.5 - 0.012
LID_HINGE_Z = BODY_HEIGHT

POCKET_CENTER_Y = -0.090
POCKET_OUTER_WIDTH = 0.118
POCKET_OUTER_DEPTH = 0.080
POCKET_INNER_WIDTH = 0.110
POCKET_INNER_DEPTH = 0.072
POCKET_BOTTOM_Z = 0.006
POCKET_HEIGHT = LID_THICKNESS - POCKET_BOTTOM_Z

FLAP_WIDTH = POCKET_INNER_WIDTH
FLAP_DEPTH = POCKET_INNER_DEPTH
FLAP_THICKNESS = 0.005

DIAL_X = -0.050
DIAL_Z = 0.309
BUTTON_XS = (0.020, 0.055, 0.090)
BUTTON_Z = 0.316
BUTTON_SIZE = (0.022, 0.014, 0.008)
BUTTON_TRAVEL = 0.0032


def _filleted_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )

def _make_chamber_shape() -> cq.Workplane:
    pan = _filleted_box(PAN_WIDTH, PAN_DEPTH, PAN_HEIGHT, 0.014)
    pan = pan.faces(">Z").shell(-PAN_WALL)

    spindle_boss = cq.Workplane("XY").circle(PAN_BOSS_RADIUS).extrude(PAN_BOSS_HEIGHT)
    return pan.union(spindle_boss)


def _make_lid_shape() -> cq.Workplane:
    lid = _filleted_box(LID_WIDTH, LID_DEPTH, LID_THICKNESS, 0.012).translate(
        (0.0, -LID_DEPTH * 0.5, 0.0)
    )

    underside_cut = cq.Workplane("XY").box(
        LID_WIDTH - 0.026,
        LID_DEPTH - 0.030,
        LID_THICKNESS - LID_TOP_SKIN,
        centered=(True, True, False),
    ).translate((0.0, -LID_DEPTH * 0.5, 0.0))
    lid = lid.cut(underside_cut)

    pocket_opening = cq.Workplane("XY").box(
        POCKET_INNER_WIDTH,
        POCKET_INNER_DEPTH,
        LID_TOP_SKIN + 0.004,
        centered=(True, True, False),
    ).translate(
        (
            0.0,
            POCKET_CENTER_Y,
            LID_THICKNESS - LID_TOP_SKIN,
        )
    )
    lid = lid.cut(pocket_opening)

    pocket = (
        cq.Workplane("XY")
        .box(
            POCKET_OUTER_WIDTH,
            POCKET_OUTER_DEPTH,
            POCKET_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, POCKET_CENTER_Y, POCKET_BOTTOM_Z))
        .faces(">Z")
        .shell(-0.003)
    )
    return lid.union(pocket)


def _make_flap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FLAP_WIDTH, FLAP_DEPTH, FLAP_THICKNESS)
        .translate((0.0, -FLAP_DEPTH * 0.5, 0.0))
        .edges("|Z")
        .fillet(0.0015)
    )


def _make_paddle_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.011).extrude(0.012)

    blade = cq.Workplane("XY").box(0.050, 0.020, 0.016, centered=(True, True, False)).translate(
        (0.031, 0.0, 0.002)
    )
    tip = cq.Workplane("XY").circle(0.010).extrude(0.016).translate((0.055, 0.0, 0.002))
    return hub.union(blade).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_bread_maker")

    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.74, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    nonstick = model.material("nonstick", rgba=(0.35, 0.36, 0.38, 1.0))
    paddle_grey = model.material("paddle_grey", rgba=(0.62, 0.63, 0.66, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_black = model.material("button_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, CHAMBER_FLOOR_Z)),
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_FLOOR_Z * 0.5)),
        material=stainless,
        name="base_floor",
    )
    body.visual(
        Box((0.020, BODY_DEPTH, BODY_HEIGHT - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                -BODY_WIDTH * 0.5 + 0.010,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_HEIGHT - CHAMBER_FLOOR_Z) * 0.5,
            )
        ),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((0.020, BODY_DEPTH, BODY_HEIGHT - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - 0.010,
                0.0,
                CHAMBER_FLOOR_Z + (BODY_HEIGHT - CHAMBER_FLOOR_Z) * 0.5,
            )
        ),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 0.040, 0.020, BODY_HEIGHT - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - 0.010,
                CHAMBER_FLOOR_Z + (BODY_HEIGHT - CHAMBER_FLOOR_Z) * 0.5,
            )
        ),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_WIDTH, 0.100, CONSOLE_BOTTOM_Z - CHAMBER_FLOOR_Z)),
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH * 0.5 + 0.050,
                CHAMBER_FLOOR_Z + (CONSOLE_BOTTOM_Z - CHAMBER_FLOOR_Z) * 0.5,
            )
        ),
        material=stainless,
        name="front_tower",
    )
    body.visual(
        Box((CONSOLE_WIDTH, CONSOLE_DEPTH, CONSOLE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CONSOLE_CENTER_Y,
                CONSOLE_BOTTOM_Z + CONSOLE_HEIGHT * 0.5,
            )
        ),
        material=charcoal,
        name="console",
    )
    body.visual(
        Box((0.252, 0.002, 0.058)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 - 0.001, 0.313)),
        material=graphite,
        name="panel_bezel",
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(_make_chamber_shape(), "bread_maker_chamber"),
        material=nonstick,
        name="pan_shell",
    )
    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.0, CHAMBER_CENTER_Y, CHAMBER_FLOOR_Z)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shape(), "bread_maker_lid"),
        material=charcoal,
        name="lid_shell",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.35),
    )

    dispenser_flap = model.part("dispenser_flap")
    dispenser_flap.visual(
        mesh_from_cadquery(_make_flap_shape(), "bread_maker_dispenser_flap"),
        material=graphite,
        name="flap_shell",
    )
    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=dispenser_flap,
        origin=Origin(
            xyz=(
                0.0,
                POCKET_CENTER_Y + FLAP_DEPTH * 0.5,
                LID_THICKNESS - FLAP_THICKNESS * 0.5,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_make_paddle_shape(), "bread_maker_paddle"),
        material=paddle_grey,
        name="paddle",
    )
    model.articulation(
        "chamber_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=chamber,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, PAN_BOSS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.022,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.056, 0.006, flare=0.07),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=18.0),
                center=False,
            ),
            "bread_maker_dial",
        ),
        material=soft_black,
        name="dial_cap",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, PANEL_SURFACE_Y, DIAL_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_SIZE),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] * 0.5)),
            material=button_black,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, PANEL_SURFACE_Y, BUTTON_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    dispenser_flap = object_model.get_part("dispenser_flap")
    paddle = object_model.get_part("paddle")

    lid_hinge = object_model.get_articulation("body_to_lid")
    flap_hinge = object_model.get_articulation("lid_to_flap")

    ctx.expect_overlap(
        lid,
        chamber,
        axes="xy",
        min_overlap=0.120,
        name="lid covers the loaf chamber opening",
    )
    ctx.expect_within(
        paddle,
        chamber,
        axes="xy",
        margin=0.030,
        name="kneading paddle stays inside the loaf chamber footprint",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "main lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.120,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(dispenser_flap)
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        open_flap_aabb = ctx.part_world_aabb(dispenser_flap)
    ctx.check(
        "dispenser flap opens upward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.030,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
