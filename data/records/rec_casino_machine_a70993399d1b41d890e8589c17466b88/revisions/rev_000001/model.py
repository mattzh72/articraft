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


CABINET_WIDTH = 0.78
CABINET_DEPTH = 0.66
FRONT_THICKNESS = 0.03
SIDE_THICKNESS = 0.03
BODY_HEIGHT = 1.58
INNER_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS

DOOR_WIDTH = 0.48
DOOR_HEIGHT = 0.52
DOOR_THICKNESS = 0.018
DOOR_OPENING_WIDTH = 0.50
DOOR_OPENING_HEIGHT = 0.54
DOOR_OPENING_BOTTOM = 0.14
DOOR_RECESS = 0.010

REEL_OPENING_WIDTH = 0.46
REEL_OPENING_HEIGHT = 0.34
REEL_OPENING_BOTTOM = 0.96

SHELF_WIDTH = 0.58
SHELF_DEPTH = 0.16
SHELF_HEIGHT = 0.10
SHELF_TOP_THICKNESS = 0.014
SHELF_WALL_THICKNESS = 0.015
SHELF_FRONT_APRON = 0.028

BUTTON_TRAVEL = 0.008
BUTTON_CAP_RADIUS = 0.029
BUTTON_STEM_RADIUS = 0.011
BUTTON_TOP_CLEARANCE = 0.009
BUTTON_CAP_HEIGHT = 0.016
BUTTON_STEM_LENGTH = 0.050
BUTTON_Y = -0.010
BUTTON_X_OFFSETS = (-0.14, 0.0, 0.14)


def _build_top_housing() -> object:
    return (
        cq.Workplane("XY")
        .box(0.68, 0.44, 0.34)
        .edges("|X and >Z")
        .fillet(0.12)
    )


def _build_shelf_body() -> object:
    top = (
        cq.Workplane("XY")
        .box(SHELF_WIDTH, SHELF_DEPTH, SHELF_TOP_THICKNESS)
        .translate((0.0, 0.0, SHELF_HEIGHT - SHELF_TOP_THICKNESS / 2.0))
    )
    rear = (
        cq.Workplane("XY")
        .box(SHELF_WIDTH, SHELF_WALL_THICKNESS, SHELF_HEIGHT)
        .translate((0.0, SHELF_DEPTH / 2.0 - SHELF_WALL_THICKNESS / 2.0, SHELF_HEIGHT / 2.0))
    )
    front = (
        cq.Workplane("XY")
        .box(SHELF_WIDTH, SHELF_FRONT_APRON, SHELF_HEIGHT * 0.64)
        .translate((0.0, -SHELF_DEPTH / 2.0 + SHELF_FRONT_APRON / 2.0, SHELF_HEIGHT * 0.32))
    )
    left = (
        cq.Workplane("XY")
        .box(SHELF_WALL_THICKNESS, SHELF_DEPTH, SHELF_HEIGHT)
        .translate((-SHELF_WIDTH / 2.0 + SHELF_WALL_THICKNESS / 2.0, 0.0, SHELF_HEIGHT / 2.0))
    )
    right = (
        cq.Workplane("XY")
        .box(SHELF_WALL_THICKNESS, SHELF_DEPTH, SHELF_HEIGHT)
        .translate((SHELF_WIDTH / 2.0 - SHELF_WALL_THICKNESS / 2.0, 0.0, SHELF_HEIGHT / 2.0))
    )
    shelf = top.union(rear).union(front).union(left).union(right)
    shelf = (
        shelf.faces(">Z")
        .workplane()
        .pushPoints([(x_pos, BUTTON_Y) for x_pos in BUTTON_X_OFFSETS])
        .circle(BUTTON_STEM_RADIUS + 0.0015)
        .cutBlind(-(SHELF_TOP_THICKNESS + 0.008))
    )
    return shelf


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reel_slot_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.66, 0.08, 0.10, 1.0))
    cabinet_dark = model.material("cabinet_dark", rgba=(0.12, 0.12, 0.14, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.08, 0.08, 0.09, 1.0))
    gold_trim = model.material("gold_trim", rgba=(0.73, 0.60, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.14, 0.24, 0.28, 0.35))
    reel_white = model.material("reel_white", rgba=(0.93, 0.93, 0.90, 1.0))
    reel_black = model.material("reel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.12, 0.12, 1.0))
    button_amber = model.material("button_amber", rgba=(0.84, 0.57, 0.15, 1.0))
    button_blue = model.material("button_blue", rgba=(0.15, 0.36, 0.78, 1.0))
    marquee_glow = model.material("marquee_glow", rgba=(0.92, 0.63, 0.16, 0.55))

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH * 0.94, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cabinet_dark,
        name="base_plinth",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=cabinet_red,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=cabinet_red,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((INNER_WIDTH, SIDE_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0 - SIDE_THICKNESS / 2.0, BODY_HEIGHT / 2.0)),
        material=cabinet_red,
        name="back_wall",
    )
    cabinet.visual(
        Box((INNER_WIDTH, CABINET_DEPTH - FRONT_THICKNESS - SIDE_THICKNESS, SIDE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (CABINET_DEPTH / 2.0 - SIDE_THICKNESS / 2.0 + (-CABINET_DEPTH / 2.0 + FRONT_THICKNESS) / 2.0),
                SIDE_THICKNESS / 2.0,
            )
        ),
        material=cabinet_dark,
        name="floor",
    )
    cabinet.visual(
        Box((INNER_WIDTH, FRONT_THICKNESS, DOOR_OPENING_BOTTOM)),
        origin=Origin(
            xyz=(0.0, -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0, DOOR_OPENING_BOTTOM / 2.0)
        ),
        material=gold_trim,
        name="front_bottom_rail",
    )

    door_jamb_width = (INNER_WIDTH - DOOR_OPENING_WIDTH) / 2.0
    cabinet.visual(
        Box((door_jamb_width, FRONT_THICKNESS, DOOR_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DOOR_OPENING_WIDTH / 2.0 + door_jamb_width / 2.0),
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0,
                DOOR_OPENING_BOTTOM + DOOR_OPENING_HEIGHT / 2.0,
            )
        ),
        material=gold_trim,
        name="door_jamb_0",
    )
    cabinet.visual(
        Box((door_jamb_width, FRONT_THICKNESS, DOOR_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_OPENING_WIDTH / 2.0 + door_jamb_width / 2.0,
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0,
                DOOR_OPENING_BOTTOM + DOOR_OPENING_HEIGHT / 2.0,
            )
        ),
        material=gold_trim,
        name="door_jamb_1",
    )

    bridge_height = REEL_OPENING_BOTTOM - (DOOR_OPENING_BOTTOM + DOOR_OPENING_HEIGHT)
    cabinet.visual(
        Box((INNER_WIDTH, FRONT_THICKNESS, bridge_height)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0,
                DOOR_OPENING_BOTTOM + DOOR_OPENING_HEIGHT + bridge_height / 2.0,
            )
        ),
        material=cabinet_red,
        name="front_bridge",
    )

    reel_jamb_width = (INNER_WIDTH - REEL_OPENING_WIDTH) / 2.0
    cabinet.visual(
        Box((reel_jamb_width, FRONT_THICKNESS, REEL_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -(REEL_OPENING_WIDTH / 2.0 + reel_jamb_width / 2.0),
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0,
                REEL_OPENING_BOTTOM + REEL_OPENING_HEIGHT / 2.0,
            )
        ),
        material=gold_trim,
        name="reel_jamb_0",
    )
    cabinet.visual(
        Box((reel_jamb_width, FRONT_THICKNESS, REEL_OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                REEL_OPENING_WIDTH / 2.0 + reel_jamb_width / 2.0,
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0,
                REEL_OPENING_BOTTOM + REEL_OPENING_HEIGHT / 2.0,
            )
        ),
        material=gold_trim,
        name="reel_jamb_1",
    )

    top_rail_height = 0.18
    cabinet.visual(
        Box((INNER_WIDTH, FRONT_THICKNESS, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS / 2.0,
                REEL_OPENING_BOTTOM + REEL_OPENING_HEIGHT + top_rail_height / 2.0,
            )
        ),
        material=cabinet_red,
        name="reel_top_rail",
    )

    cabinet.visual(
        mesh_from_cadquery(_build_top_housing(), "slot_machine_top_housing"),
        origin=Origin(xyz=(0.0, -0.11, 1.64)),
        material=cabinet_red,
        name="top_housing",
    )
    cabinet.visual(
        Box((0.56, 0.006, 0.20)),
        origin=Origin(xyz=(0.0, -CABINET_DEPTH / 2.0 + 0.003, 1.63)),
        material=marquee_glow,
        name="marquee_panel",
    )

    reel_pack = model.part("reel_pack")
    reel_pack.visual(
        Box((REEL_OPENING_WIDTH - 0.016, 0.14, REEL_OPENING_HEIGHT - 0.020)),
        origin=Origin(xyz=(0.0, 0.072, 0.0)),
        material=reel_black,
        name="reel_box",
    )
    reel_pack.visual(
        Box((REEL_OPENING_WIDTH - 0.014, 0.004, REEL_OPENING_HEIGHT - 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=smoked_glass,
        name="window_glass",
    )
    reel_pack.visual(
        Box((INNER_WIDTH - 0.040, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.006, REEL_OPENING_HEIGHT / 2.0 + 0.025)),
        material=cabinet_shadow,
        name="reel_mount_top",
    )
    reel_pack.visual(
        Box((INNER_WIDTH - 0.040, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.006, -(REEL_OPENING_HEIGHT / 2.0 + 0.025))),
        material=cabinet_shadow,
        name="reel_mount_bottom",
    )
    reel_pack.visual(
        Box((0.070, 0.012, REEL_OPENING_HEIGHT + 0.060)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=cabinet_shadow,
        name="reel_mount_spine",
    )
    reel_pack.visual(
        Cylinder(radius=0.012, length=REEL_OPENING_WIDTH - 0.030),
        origin=Origin(xyz=(0.0, 0.072, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="reel_shaft",
    )
    for index, x_pos in enumerate((-0.14, 0.0, 0.14)):
        reel_pack.visual(
            Cylinder(radius=0.088, length=0.12),
            origin=Origin(xyz=(x_pos, 0.072, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=reel_white,
            name=f"reel_{index}",
        )

    model.articulation(
        "cabinet_to_reel_pack",
        ArticulationType.FIXED,
        parent=cabinet,
        child=reel_pack,
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + FRONT_THICKNESS,
                REEL_OPENING_BOTTOM + REEL_OPENING_HEIGHT / 2.0,
            )
        ),
    )

    shelf = model.part("shelf")
    shelf.visual(
        mesh_from_cadquery(_build_shelf_body(), "slot_machine_button_shelf"),
        material=cabinet_dark,
        name="shelf_body",
    )
    shelf.visual(
        Box((SHELF_WIDTH - 0.040, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -SHELF_DEPTH / 2.0 - 0.002, SHELF_HEIGHT * 0.62)),
        material=gold_trim,
        name="shelf_trim",
    )
    model.articulation(
        "cabinet_to_shelf",
        ArticulationType.FIXED,
        parent=cabinet,
        child=shelf,
        origin=Origin(xyz=(0.0, -CABINET_DEPTH / 2.0 - SHELF_DEPTH / 2.0, 0.72)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=cabinet_dark,
        name="door_panel",
    )
    service_door.visual(
        Box((0.060, 0.004, 0.028)),
        origin=Origin(
            xyz=(-DOOR_WIDTH + 0.055, -(DOOR_THICKNESS / 2.0 + 0.002), DOOR_HEIGHT * 0.58)
        ),
        material=chrome,
        name="door_pull",
    )
    service_door.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(
            xyz=(-DOOR_WIDTH + 0.095, -(DOOR_THICKNESS / 2.0 + 0.002), DOOR_HEIGHT * 0.58)
        ),
        material=chrome,
        name="door_lock",
    )
    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(
            xyz=(
                DOOR_WIDTH / 2.0,
                -CABINET_DEPTH / 2.0 + DOOR_RECESS + DOOR_THICKNESS / 2.0,
                DOOR_OPENING_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=20.0, velocity=1.4),
    )

    button_materials = (button_red, button_amber, button_blue)
    for index, (x_pos, cap_material) in enumerate(zip(BUTTON_X_OFFSETS, button_materials)):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_STEM_RADIUS, length=BUTTON_STEM_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_TOP_CLEARANCE - BUTTON_STEM_LENGTH / 2.0)),
            material=chrome,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=BUTTON_STEM_RADIUS + 0.0035, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -(SHELF_TOP_THICKNESS + 0.002))),
            material=chrome,
            name="button_collar",
        )
        button.visual(
            Cylinder(radius=BUTTON_CAP_RADIUS, length=BUTTON_CAP_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_TOP_CLEARANCE + BUTTON_CAP_HEIGHT / 2.0)),
            material=cap_material,
            name="button_cap",
        )
        model.articulation(
            f"shelf_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=shelf,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_Y, SHELF_HEIGHT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=BUTTON_TRAVEL,
                effort=12.0,
                velocity=0.08,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    shelf = object_model.get_part("shelf")
    service_door = object_model.get_part("service_door")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    door_hinge = object_model.get_articulation("cabinet_to_service_door")
    button_0_joint = object_model.get_articulation("shelf_to_button_0")

    bridge_aabb = ctx.part_element_world_aabb(cabinet, elem="front_bridge")
    shelf_aabb = ctx.part_element_world_aabb(shelf, elem="shelf_body")
    ctx.check(
        "shelf is a projecting ledge",
        bridge_aabb is not None
        and shelf_aabb is not None
        and abs(shelf_aabb[1][1] - bridge_aabb[0][1]) <= 0.003
        and shelf_aabb[0][1] < bridge_aabb[0][1] - 0.12,
        details=f"bridge={bridge_aabb}, shelf={shelf_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    jamb_aabb = ctx.part_element_world_aabb(cabinet, elem="door_jamb_1")
    ctx.check(
        "service door stays recessed at rest",
        closed_door_aabb is not None
        and jamb_aabb is not None
        and closed_door_aabb[0][1] > jamb_aabb[0][1] + 0.005,
        details=f"door={closed_door_aabb}, jamb={jamb_aabb}",
    )

    open_door_aabb = None
    if door_hinge.motion_limits is not None and door_hinge.motion_limits.upper is not None:
        with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
            open_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    ctx.check(
        "service door opens outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.15,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    ctx.expect_gap(
        button_0,
        shelf,
        axis="z",
        positive_elem="button_cap",
        negative_elem="shelf_body",
        min_gap=0.008,
        name="button cap stands above the shelf at rest",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    pressed_button_0 = None
    pressed_button_1 = None
    if button_0_joint.motion_limits is not None and button_0_joint.motion_limits.upper is not None:
        with ctx.pose({button_0_joint: button_0_joint.motion_limits.upper}):
            ctx.expect_gap(
                button_0,
                shelf,
                axis="z",
                positive_elem="button_cap",
                negative_elem="shelf_body",
                max_gap=0.003,
                max_penetration=0.0,
                name="button cap remains above the shelf when pressed",
            )
            pressed_button_0 = ctx.part_world_position(button_0)
            pressed_button_1 = ctx.part_world_position(button_1)

    ctx.check(
        "button press moves downward",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.006,
        details=f"rest={rest_button_0}, pressed={pressed_button_0}",
    )
    ctx.check(
        "buttons articulate independently",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and abs(pressed_button_1[2] - rest_button_1[2]) <= 1e-6,
        details=f"button_1_rest={rest_button_1}, button_1_pressed={pressed_button_1}",
    )

    return ctx.report()


object_model = build_object_model()
