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


CABINET_WIDTH = 0.98
LOWER_FRONT_Y = 0.42
LOWER_TOP_Z = 0.62
UPPER_FRONT_Y = 0.12
UPPER_TOP_Z = 1.72
DECK_TOP_Z = 0.77

DOOR_WIDTH = 0.58
DOOR_HEIGHT = 0.40
DOOR_THICKNESS = 0.032
DOOR_BOTTOM_Z = 0.16

BUTTON_SURFACE_Y = 0.27
BUTTON_SURFACE_Z = 0.695
BUTTON_SEAT_OFFSET = 0.015
BUTTON_TRAVEL = 0.012
BUTTON_HOLE_RADIUS = 0.040
BUTTON_BEZEL_RADIUS = 0.057
BUTTON_BEZEL_HEIGHT = 0.012

DECK_ANGLE = math.atan2(DECK_TOP_Z - LOWER_TOP_Z, LOWER_FRONT_Y - UPPER_FRONT_Y)
DECK_ROLL = -DECK_ANGLE
DECK_NORMAL_Y = math.sin(DECK_ANGLE)
DECK_NORMAL_Z = math.cos(DECK_ANGLE)


def _deck_offset(distance: float) -> tuple[float, float, float]:
    return (0.0, DECK_NORMAL_Y * distance, DECK_NORMAL_Z * distance)


def _cabinet_body_mesh():
    side_profile = [
        (-0.42, 0.0),
        (LOWER_FRONT_Y, 0.0),
        (LOWER_FRONT_Y, LOWER_TOP_Z),
        (UPPER_FRONT_Y, DECK_TOP_Z),
        (UPPER_FRONT_Y, UPPER_TOP_Z),
        (-0.28, UPPER_TOP_Z),
        (-0.28, DECK_TOP_Z),
        (-0.42, LOWER_TOP_Z),
    ]

    body = cq.Workplane("YZ").polyline(side_profile).close().extrude(CABINET_WIDTH / 2.0, both=True)

    play_window_cut = cq.Workplane("XY").box(0.72, 0.050, 0.58).translate((0.0, 0.095, 1.10))
    prize_door_recess = cq.Workplane("XY").box(0.50, 0.030, 0.30).translate((0.0, 0.405, 0.33))

    hole_base = _deck_offset(-0.080)
    button_hole = (
        cq.Workplane("XY")
        .circle(BUTTON_HOLE_RADIUS)
        .extrude(0.160)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(DECK_ROLL))
        .translate((0.0, BUTTON_SURFACE_Y + hole_base[1], BUTTON_SURFACE_Z + hole_base[2]))
    )

    bezel_base = _deck_offset(-0.002)
    button_bezel = (
        cq.Workplane("XY")
        .circle(BUTTON_BEZEL_RADIUS)
        .circle(BUTTON_HOLE_RADIUS + 0.004)
        .extrude(BUTTON_BEZEL_HEIGHT)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(DECK_ROLL))
        .translate((0.0, BUTTON_SURFACE_Y + bezel_base[1], BUTTON_SURFACE_Z + bezel_base[2]))
    )

    body = body.cut(play_window_cut)
    body = body.cut(prize_door_recess)
    body = body.cut(button_hole)
    body = body.union(button_bezel)
    return mesh_from_cadquery(body, "redemption_arcade_cabinet_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="redemption_arcade_cabinet")

    cabinet_blue = model.material("cabinet_blue", rgba=(0.12, 0.29, 0.58, 1.0))
    window_tint = model.material("window_tint", rgba=(0.08, 0.12, 0.16, 0.72))
    marquee_light = model.material("marquee_light", rgba=(0.92, 0.95, 1.0, 0.90))
    door_blue = model.material("door_blue", rgba=(0.15, 0.33, 0.62, 1.0))
    accent_black = model.material("accent_black", rgba=(0.08, 0.08, 0.09, 1.0))
    lock_metal = model.material("lock_metal", rgba=(0.74, 0.77, 0.80, 1.0))
    button_red = model.material("button_red", rgba=(0.90, 0.18, 0.12, 0.92))
    button_base = model.material("button_base", rgba=(0.12, 0.12, 0.13, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(_cabinet_body_mesh(), material=cabinet_blue, name="body_shell")
    cabinet.visual(
        Box((0.74, 0.018, 0.56)),
        origin=Origin(xyz=(0.0, 0.113, 1.09)),
        material=window_tint,
        name="play_window",
    )
    cabinet.visual(
        Box((0.78, 0.012, 0.18)),
        origin=Origin(xyz=(0.0, 0.114, 1.58)),
        material=marquee_light,
        name="marquee_lens",
    )

    prize_door = model.part("prize_door")
    prize_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=door_blue,
        name="door_panel",
    )
    prize_door.visual(
        Box((DOOR_WIDTH - 0.08, 0.006, DOOR_HEIGHT - 0.08)),
        origin=Origin(
            xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS + 0.003, DOOR_HEIGHT / 2.0),
        ),
        material=accent_black,
        name="door_trim",
    )
    prize_door.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(
            xyz=(DOOR_WIDTH - 0.070, DOOR_THICKNESS + 0.008, DOOR_HEIGHT * 0.53),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=lock_metal,
        name="door_lock",
    )
    prize_door.visual(
        Box((0.018, 0.014, 0.085)),
        origin=Origin(
            xyz=(DOOR_WIDTH - 0.090, DOOR_THICKNESS + 0.007, DOOR_HEIGHT * 0.53),
        ),
        material=lock_metal,
        name="door_pull",
    )

    model.articulation(
        "cabinet_to_prize_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=prize_door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, LOWER_FRONT_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    push_button = model.part("push_button")
    push_button.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=button_base,
        name="button_skirt",
    )
    push_button.visual(
        Cylinder(radius=0.066, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=button_red,
        name="button_cap",
    )
    push_button.visual(
        Cylinder(radius=0.028, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=button_base,
        name="button_plunger",
    )

    button_origin_offset = _deck_offset(BUTTON_SEAT_OFFSET)
    model.articulation(
        "cabinet_to_push_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=push_button,
        origin=Origin(
            xyz=(
                0.0,
                BUTTON_SURFACE_Y + button_origin_offset[1],
                BUTTON_SURFACE_Z + button_origin_offset[2],
            ),
            rpy=(DECK_ROLL, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.12,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    prize_door = object_model.get_part("prize_door")
    push_button = object_model.get_part("push_button")
    door_hinge = object_model.get_articulation("cabinet_to_prize_door")
    button_slide = object_model.get_articulation("cabinet_to_push_button")

    ctx.allow_overlap(
        cabinet,
        push_button,
        elem_a="body_shell",
        elem_b="button_plunger",
        reason="The arcade button's internal plunger is intentionally modeled as nesting into the cabinet's simplified internal switch cavity.",
    )

    ctx.expect_gap(
        prize_door,
        cabinet,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="prize door closes onto the cabinet face",
    )

    door_aabb = ctx.part_element_world_aabb(prize_door, elem="door_panel")
    window_aabb = ctx.part_element_world_aabb(cabinet, elem="play_window")
    door_top = None if door_aabb is None else door_aabb[1][2]
    window_bottom = None if window_aabb is None else window_aabb[0][2]
    ctx.check(
        "prize door sits beneath the play window",
        door_top is not None and window_bottom is not None and window_bottom > door_top + 0.18,
        details=f"door_top={door_top}, window_bottom={window_bottom}",
    )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        closed_center = _aabb_center(ctx.part_element_world_aabb(prize_door, elem="door_panel"))
        with ctx.pose({door_hinge: door_limits.upper}):
            open_center = _aabb_center(ctx.part_element_world_aabb(prize_door, elem="door_panel"))
        ctx.check(
            "prize door swings outward on its side hinge",
            closed_center is not None
            and open_center is not None
            and open_center[1] > closed_center[1] + 0.18,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    button_rest = ctx.part_world_position(push_button)
    button_limits = button_slide.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_slide: button_limits.upper}):
            button_pressed = ctx.part_world_position(push_button)
        ctx.check(
            "push button presses downward into the sloped deck",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[2] < button_rest[2] - 0.007,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    ctx.check(
        "push button stays centered on the control deck",
        button_rest is not None and abs(button_rest[0]) < 0.01,
        details=f"button_rest={button_rest}",
    )

    return ctx.report()


object_model = build_object_model()
