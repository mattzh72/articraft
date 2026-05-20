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


CABINET_WIDTH = 0.55
CABINET_DEPTH = 0.58
BASE_HEIGHT = 0.82
CONSOLE_HEIGHT = 0.98
WALL_THICKNESS = 0.022
FLOOR_THICKNESS = 0.028

OPENING_WIDTH = 0.39
OPENING_DEPTH = 0.34
OPENING_CENTER_Y = -0.04

LID_WIDTH = 0.43
LID_DEPTH = 0.38
LID_THICKNESS = 0.018
LID_CLOSED_GAP = 0.0
LID_HINGE_Y = 0.145
LID_HINGE_Z = BASE_HEIGHT + LID_THICKNESS / 2.0 + LID_CLOSED_GAP

TUB_OUTER_RADIUS = 0.19
TUB_INNER_RADIUS = 0.172
TUB_DEPTH = 0.40
TUB_BOTTOM_THICKNESS = 0.020
TUB_TOP_Z = BASE_HEIGHT - 0.020

AGITATOR_RADIUS = 0.050
AGITATOR_TOP_RADIUS = 0.032
AGITATOR_HEIGHT = 0.280
SOFTENER_CAP_HEIGHT = 0.024
SOFTENER_CAP_RADIUS = 0.036
SOFTENER_CAP_BASE_Z = -0.100

CONSOLE_FACE_Y0 = 0.170
CONSOLE_FACE_Y1 = 0.255
CONSOLE_FACE_ROLL = math.atan2(CONSOLE_HEIGHT - BASE_HEIGHT, CONSOLE_FACE_Y1 - CONSOLE_FACE_Y0)
CONSOLE_FACE_LENGTH = math.hypot(CONSOLE_FACE_Y1 - CONSOLE_FACE_Y0, CONSOLE_HEIGHT - BASE_HEIGHT)

BUTTON_TRAVEL = 0.0015


def _console_origin(x: float, along: float, proud: float = 0.0) -> Origin:
    cos_a = math.cos(CONSOLE_FACE_ROLL)
    sin_a = math.sin(CONSOLE_FACE_ROLL)
    return Origin(
        xyz=(
            x,
            CONSOLE_FACE_Y0 + along * cos_a - proud * sin_a,
            BASE_HEIGHT + along * sin_a + proud * cos_a,
        ),
        rpy=(CONSOLE_FACE_ROLL, 0.0, 0.0),
    )


def _console_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (CONSOLE_FACE_Y0, BASE_HEIGHT),
                (CABINET_DEPTH / 2.0, BASE_HEIGHT),
                (CABINET_DEPTH / 2.0, CONSOLE_HEIGHT),
                (CONSOLE_FACE_Y1, CONSOLE_HEIGHT),
            ]
        )
        .close()
        .extrude(CABINET_WIDTH / 2.0, both=True)
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_top_load_washer")

    enamel_white = model.material("enamel_white", rgba=(0.94, 0.95, 0.97, 1.0))
    light_gray = model.material("light_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    soft_blue = model.material("soft_blue", rgba=(0.60, 0.74, 0.90, 1.0))

    body = model.part("body")
    opening_front_y = OPENING_CENTER_Y - OPENING_DEPTH / 2.0
    opening_rear_y = OPENING_CENTER_Y + OPENING_DEPTH / 2.0
    side_strip_width = (CABINET_WIDTH - OPENING_WIDTH) / 2.0
    front_strip_depth = opening_front_y - (-CABINET_DEPTH / 2.0)
    rear_strip_depth = CABINET_DEPTH / 2.0 - opening_rear_y
    panel_overlap = 0.002

    body.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS / 2.0)),
        material=enamel_white,
        name="base_panel",
    )
    body.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=(-(CABINET_WIDTH - WALL_THICKNESS) / 2.0, 0.0, BASE_HEIGHT / 2.0)),
        material=enamel_white,
        name="left_side",
    )
    body.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=((CABINET_WIDTH - WALL_THICKNESS) / 2.0, 0.0, BASE_HEIGHT / 2.0)),
        material=enamel_white,
        name="right_side",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS + panel_overlap, WALL_THICKNESS, BASE_HEIGHT - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH - WALL_THICKNESS) / 2.0,
                FLOOR_THICKNESS + (BASE_HEIGHT - FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=enamel_white,
        name="front_panel",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS + panel_overlap, WALL_THICKNESS, BASE_HEIGHT - FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (CABINET_DEPTH - WALL_THICKNESS) / 2.0,
                FLOOR_THICKNESS + (BASE_HEIGHT - FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=enamel_white,
        name="rear_panel",
    )
    body.visual(
        Box((CABINET_WIDTH, front_strip_depth + panel_overlap, WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + front_strip_depth / 2.0,
                BASE_HEIGHT - WALL_THICKNESS / 2.0,
            )
        ),
        material=enamel_white,
        name="deck_front",
    )
    body.visual(
        Box((CABINET_WIDTH, rear_strip_depth + panel_overlap, WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                opening_rear_y + rear_strip_depth / 2.0,
                BASE_HEIGHT - WALL_THICKNESS / 2.0,
            )
        ),
        material=enamel_white,
        name="deck_rear",
    )
    body.visual(
        Box((side_strip_width + panel_overlap, OPENING_DEPTH + panel_overlap, WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                -(OPENING_WIDTH + side_strip_width) / 2.0,
                OPENING_CENTER_Y,
                BASE_HEIGHT - WALL_THICKNESS / 2.0,
            )
        ),
        material=enamel_white,
        name="deck_left",
    )
    body.visual(
        Box((side_strip_width + panel_overlap, OPENING_DEPTH + panel_overlap, WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                (OPENING_WIDTH + side_strip_width) / 2.0,
                OPENING_CENTER_Y,
                BASE_HEIGHT - WALL_THICKNESS / 2.0,
            )
        ),
        material=enamel_white,
        name="deck_right",
    )
    console_panel_thickness = 0.012
    body.visual(
        Box((CABINET_WIDTH, CONSOLE_FACE_LENGTH, console_panel_thickness)),
        origin=_console_origin(0.0, CONSOLE_FACE_LENGTH / 2.0, proud=-console_panel_thickness / 2.0),
        material=enamel_white,
        name="console_face",
    )
    body.visual(
        Box((CABINET_WIDTH, 0.050, console_panel_thickness)),
        origin=Origin(xyz=(0.0, 0.265, CONSOLE_HEIGHT - console_panel_thickness / 2.0)),
        material=enamel_white,
        name="console_top",
    )
    body.visual(
        Box((console_panel_thickness, 0.090, 0.160)),
        origin=Origin(xyz=(-(CABINET_WIDTH - console_panel_thickness) / 2.0, 0.245, 0.900)),
        material=enamel_white,
        name="console_side_0",
    )
    body.visual(
        Box((console_panel_thickness, 0.090, 0.160)),
        origin=Origin(xyz=((CABINET_WIDTH - console_panel_thickness) / 2.0, 0.245, 0.900)),
        material=enamel_white,
        name="console_side_1",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        origin=Origin(xyz=(0.0, -LID_DEPTH / 2.0, 0.0)),
        material=enamel_white,
        name="lid_panel",
    )

    tub = model.part("tub")
    tub.visual(
        Cylinder(radius=TUB_OUTER_RADIUS, length=TUB_BOTTOM_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -TUB_DEPTH + TUB_BOTTOM_THICKNESS / 2.0)),
        material=light_gray,
        name="tub_bottom",
    )
    wall_height = TUB_DEPTH - TUB_BOTTOM_THICKNESS
    wall_mid_radius = (TUB_OUTER_RADIUS + TUB_INNER_RADIUS) / 2.0
    wall_thickness = TUB_OUTER_RADIUS - TUB_INNER_RADIUS
    wall_segment_width = 2.0 * math.pi * wall_mid_radius / 12.0 * 1.03
    for idx, angle_deg in enumerate(range(0, 360, 30)):
        tub.visual(
            Box((wall_segment_width, wall_thickness, wall_height)),
            origin=Origin(
                xyz=(0.0, wall_mid_radius, -wall_height / 2.0),
                rpy=(0.0, 0.0, math.radians(angle_deg)),
            ),
            material=light_gray,
            name=f"tub_wall_{idx}",
        )
    tub.visual(
        Cylinder(radius=AGITATOR_RADIUS, length=AGITATOR_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, -TUB_DEPTH + TUB_BOTTOM_THICKNESS + AGITATOR_HEIGHT / 2.0),
        ),
        material=light_gray,
        name="agitator_core",
    )
    tub.visual(
        Cylinder(radius=AGITATOR_TOP_RADIUS, length=0.060),
        origin=Origin(
            xyz=(0.0, 0.0, -TUB_DEPTH + TUB_BOTTOM_THICKNESS + AGITATOR_HEIGHT - 0.030),
        ),
        material=light_gray,
        name="agitator_top",
    )
    for idx, angle_deg in enumerate((0.0, 90.0, 180.0, 270.0)):
        tub.visual(
            Box((0.090, 0.012, 0.170)),
            origin=Origin(
                xyz=(0.0, 0.060, -TUB_DEPTH + TUB_BOTTOM_THICKNESS + 0.115),
                rpy=(0.0, 0.0, math.radians(angle_deg)),
            ),
            material=light_gray,
            name=f"agitator_fin_{idx}",
        )

    softener_cap = model.part("softener_cap")
    softener_cap.visual(
        Cylinder(radius=SOFTENER_CAP_RADIUS, length=0.016),
        material=soft_blue,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        name="cap_base",
    )
    softener_cap.visual(
        Cylinder(radius=SOFTENER_CAP_RADIUS * 0.72, length=0.008),
        material=soft_blue,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        name="cap_top",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.031, length=0.008),
        material=dark_gray,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        name="dial_skirt",
    )
    timer_dial.visual(
        Cylinder(radius=0.024, length=0.020),
        material=dark_gray,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        name="dial_cap",
    )
    timer_dial.visual(
        Box((0.003, 0.014, 0.004)),
        material=enamel_white,
        origin=Origin(xyz=(0.0, -0.018, 0.026)),
        name="dial_indicator",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=charcoal,
        name="start_cap",
    )

    mode_button_0 = model.part("mode_button_0")
    mode_button_0.visual(
        Box((0.036, 0.020, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=dark_gray,
        name="mode_cap",
    )

    mode_button_1 = model.part("mode_button_1")
    mode_button_1.visual(
        Box((0.036, 0.020, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=dark_gray,
        name="mode_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.32, effort=12.0, velocity=1.3),
    )
    model.articulation(
        "body_to_tub",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, TUB_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )
    model.articulation(
        "tub_to_softener_cap",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=softener_cap,
        origin=Origin(xyz=(0.0, 0.0, SOFTENER_CAP_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=_console_origin(0.150, 0.086),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=_console_origin(0.026, 0.074),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=20.0, velocity=0.06),
    )
    model.articulation(
        "body_to_mode_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_0,
        origin=_console_origin(-0.148, 0.072),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=15.0, velocity=0.06),
    )
    model.articulation(
        "body_to_mode_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button_1,
        origin=_console_origin(-0.086, 0.072),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=15.0, velocity=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    softener_cap = object_model.get_part("softener_cap")
    start_button = object_model.get_part("start_button")
    mode_button_0 = object_model.get_part("mode_button_0")

    lid_hinge = object_model.get_articulation("body_to_lid")
    start_slide = object_model.get_articulation("body_to_start_button")
    mode_slide = object_model.get_articulation("body_to_mode_button_0")

    ctx.allow_overlap(
        body,
        start_button,
        elem_a="console_face",
        elem_b="start_cap",
        reason="The start button cap is represented as seating into a shallow console bezel that is simplified as a flat panel.",
    )
    ctx.allow_overlap(
        body,
        object_model.get_part("timer_dial"),
        elem_a="console_face",
        elem_b="dial_skirt",
        reason="The timer dial skirt nests into a simplified shallow console opening around the shaft.",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="deck_rear",
        max_gap=0.004,
        max_penetration=0.0,
        name="lid sits flush on the cabinet deck",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.34,
        name="lid covers the top opening footprint",
    )
    ctx.expect_origin_distance(
        softener_cap,
        tub,
        axes="xy",
        max_dist=0.001,
        name="softener cap stays centered on the agitator axis",
    )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.18,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    tub_aabb = ctx.part_world_aabb(tub)
    ctx.check(
        "tub remains visibly deep",
        tub_aabb is not None and (tub_aabb[1][2] - tub_aabb[0][2]) >= 0.36,
        details=f"tub_aabb={tub_aabb}",
    )

    start_limits = start_slide.motion_limits
    if start_limits is not None and start_limits.upper is not None:
        start_rest = ctx.part_world_position(start_button)
        with ctx.pose({start_slide: start_limits.upper}):
            start_pressed = ctx.part_world_position(start_button)
        ctx.check(
            "start button presses inward",
            start_rest is not None
            and start_pressed is not None
            and start_pressed[1] > start_rest[1] + 0.001
            and start_pressed[2] < start_rest[2] - 0.0005,
            details=f"rest={start_rest}, pressed={start_pressed}",
        )

    mode_limits = mode_slide.motion_limits
    if mode_limits is not None and mode_limits.upper is not None:
        mode_rest = ctx.part_world_position(mode_button_0)
        with ctx.pose({mode_slide: mode_limits.upper}):
            mode_pressed = ctx.part_world_position(mode_button_0)
        ctx.check(
            "mode button presses inward",
            mode_rest is not None
            and mode_pressed is not None
            and mode_pressed[1] > mode_rest[1] + 0.001
            and mode_pressed[2] < mode_rest[2] - 0.0005,
            details=f"rest={mode_rest}, pressed={mode_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
