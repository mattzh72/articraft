from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.70
CABINET_HEIGHT = 1.99

LOWER_DISPLAY_SIZE = (0.50, 0.040, 0.80)
LOWER_DISPLAY_CENTER = (0.0, -0.080, 1.12)

TOP_DISPLAY_SIZE = (0.46, 0.040, 0.26)
TOP_DISPLAY_CENTER = (0.0, -0.120, 1.74)

DOOR_WIDTH = 0.186
DOOR_HEIGHT = 0.300
DOOR_THICKNESS = 0.018
DOOR_CENTER_Z = 0.370
DOOR_HINGE_ORIGIN = (-0.104, -0.208, DOOR_CENTER_Z)

DECK_ROLL = math.radians(30.0)
DECK_CENTER = (0.0, -0.1320, 0.663)
DECK_SIZE = (0.56, 0.15, 0.030)

BUTTON_SIZE = (0.072, 0.040, 0.012)
BUTTON_XS = (-0.200, -0.100, 0.0, 0.100, 0.200)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _cq_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
):
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy, cz - (length * 0.5)))
    )


def _build_cabinet_shell():
    side_profile = [
        (-0.18, 0.00),
        (0.38, 0.00),
        (0.38, 0.84),
        (0.30, 1.24),
        (0.22, 1.60),
        (0.14, 1.88),
        (0.00, CABINET_HEIGHT),
        (-0.10, 1.92),
        (-0.09, 1.56),
        (-0.02, 1.52),
        (-0.02, 0.72),
        (-0.14, 0.65),
        (-0.18, 0.58),
        (-0.18, 0.00),
    ]
    shell = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(CABINET_WIDTH)
        .translate((-CABINET_WIDTH * 0.5, 0.0, 0.0))
    )

    lower_pocket = _cq_box(
        (LOWER_DISPLAY_SIZE[0] + 0.028, 0.050, LOWER_DISPLAY_SIZE[2] + 0.030),
        (0.0, 0.005, LOWER_DISPLAY_CENTER[2]),
    )
    top_pocket = _cq_box(
        (TOP_DISPLAY_SIZE[0] + 0.026, 0.046, TOP_DISPLAY_SIZE[2] + 0.028),
        (0.0, -0.067, TOP_DISPLAY_CENTER[2]),
    )
    door_pocket = _cq_box(
        (0.212, 0.030, 0.322),
        (-0.002, -0.165, DOOR_CENTER_Z),
    )

    return shell.cut(lower_pocket).cut(top_pocket).cut(door_pocket)


def _build_button_deck():
    main_slab = cq.Workplane("XY").box(*DECK_SIZE)
    front_fascia = _cq_box((DECK_SIZE[0], 0.020, 0.012), (0.0, -0.066, -0.009))
    rear_block = _cq_box((DECK_SIZE[0], 0.020, 0.020), (0.0, 0.060, -0.005))
    return main_slab.union(front_fascia).union(rear_block)


def _build_button():
    cap = cq.Workplane("XY").box(*BUTTON_SIZE).edges("|Z").fillet(0.006)
    light_bar = _cq_box(
        (BUTTON_SIZE[0] * 0.70, 0.006, 0.003),
        (0.0, 0.012, 0.0045),
    )
    return cap.union(light_bar)


def _build_bill_door():
    panel = _cq_box(
        (DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT),
        (0.103, 0.019, 0.0),
    )
    hinge_leaf = _cq_box((0.020, 0.012, DOOR_HEIGHT), (0.012, 0.015, 0.0))

    slot_trim = _cq_box((0.102, 0.004, 0.028), (0.115, 0.008, 0.066))
    pull = _cq_box((0.060, 0.010, 0.016), (0.132, 0.008, -0.082))

    barrels = None
    for z_center in (-0.105, 0.0, 0.105):
        barrel = _cq_cylinder(0.0075, 0.030, (0.004, 0.010, z_center))
        barrels = barrel if barrels is None else barrels.union(barrel)

    acceptor_slot = _cq_box((0.086, 0.024, 0.008), (0.118, 0.018, 0.066))

    return panel.union(hinge_leaf).union(slot_trim).union(pull).union(barrels).cut(acceptor_slot)


def _build_side_panel():
    thickness = 0.028
    profile = [
        (-0.18, 0.00),
        (0.24, 0.00),
        (0.30, 0.42),
        (0.29, 0.78),
        (0.23, 1.16),
        (0.18, 1.46),
        (0.12, 1.73),
        (0.08, 1.90),
        (-0.02, CABINET_HEIGHT),
        (-0.09, 1.88),
        (-0.11, 1.58),
        (-0.02, 1.50),
        (-0.02, 0.76),
        (-0.12, 0.68),
        (-0.18, 0.58),
        (-0.18, 0.00),
    ]
    return cq.Workplane("YZ").polyline(profile).close().extrude(thickness)


def _add_display_module(
    part,
    *,
    outer_size: tuple[float, float, float],
    glass_margin_x: float,
    glass_margin_z: float,
    frame_material,
    glass_material,
):
    width, depth, height = outer_size
    part.visual(
        Box(outer_size),
        material=frame_material,
        name="frame",
    )
    part.visual(
        Box((width - glass_margin_x, 0.0035, height - glass_margin_z)),
        origin=Origin(xyz=(0.0, -(depth * 0.5) + 0.00175, 0.0)),
        material=glass_material,
        name="glass",
    )
    part.visual(
        Box((width - 0.10, depth * 0.55, height - 0.10)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=frame_material,
        name="backer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_screen_slot_cabinet")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.10, 0.11, 0.13, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.20, 0.22, 0.25, 1.0))
    deck_finish = model.material("deck_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    display_glass = model.material("display_glass", rgba=(0.10, 0.23, 0.30, 0.45))
    display_frame = model.material("display_frame", rgba=(0.05, 0.06, 0.07, 1.0))
    button_finish = model.material("button_finish", rgba=(0.88, 0.28, 0.10, 1.0))
    button_light = model.material("button_light", rgba=(0.98, 0.62, 0.16, 1.0))
    chrome = model.material("chrome", rgba=(0.76, 0.79, 0.82, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.70, 0.56, 0.58)),
        origin=Origin(xyz=(0.0, 0.10, 0.29)),
        material=cabinet_finish,
        name="shell",
    )
    cabinet.visual(
        Box((0.70, 0.44, 1.02)),
        origin=Origin(xyz=(0.0, 0.16, 1.09)),
        material=cabinet_finish,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.66, 0.36, 0.39)),
        origin=Origin(xyz=(0.0, 0.08, 1.795)),
        material=cabinet_finish,
        name="upper_body",
    )
    cabinet.visual(
        Box((0.64, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.07, 1.54)),
        material=trim_finish,
        name="display_bridge",
    )
    cabinet.visual(
        Box((0.62, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, -0.07, 1.95)),
        material=trim_finish,
        name="top_cap",
    )
    cabinet.visual(
        Box((0.62, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.175, 0.020)),
        material=trim_finish,
        name="toe_trim",
    )
    side_panel_mesh = mesh_from_cadquery(_build_side_panel(), "slot_side_panel")
    cabinet.visual(
        side_panel_mesh,
        origin=Origin(xyz=(-CABINET_WIDTH * 0.5, 0.0, 0.0)),
        material=trim_finish,
        name="side_panel_0",
    )
    cabinet.visual(
        side_panel_mesh,
        origin=Origin(xyz=(CABINET_WIDTH * 0.5, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
        material=trim_finish,
        name="side_panel_1",
    )

    lower_display = model.part("lower_display")
    _add_display_module(
        lower_display,
        outer_size=LOWER_DISPLAY_SIZE,
        glass_margin_x=0.070,
        glass_margin_z=0.090,
        frame_material=display_frame,
        glass_material=display_glass,
    )
    model.articulation(
        "cabinet_to_lower_display",
        ArticulationType.FIXED,
        parent=cabinet,
        child=lower_display,
        origin=Origin(xyz=LOWER_DISPLAY_CENTER),
    )

    top_display = model.part("top_display")
    _add_display_module(
        top_display,
        outer_size=TOP_DISPLAY_SIZE,
        glass_margin_x=0.060,
        glass_margin_z=0.060,
        frame_material=display_frame,
        glass_material=display_glass,
    )
    model.articulation(
        "cabinet_to_top_display",
        ArticulationType.FIXED,
        parent=cabinet,
        child=top_display,
        origin=Origin(xyz=TOP_DISPLAY_CENTER),
    )

    button_deck = model.part("button_deck")
    button_deck.visual(
        Box(DECK_SIZE),
        material=deck_finish,
        name="deck_body",
    )
    button_deck.visual(
        Box((DECK_SIZE[0], 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -(DECK_SIZE[1] * 0.5) + 0.009, -0.010)),
        material=chrome,
        name="front_band",
    )
    model.articulation(
        "cabinet_to_button_deck",
        ArticulationType.FIXED,
        parent=cabinet,
        child=button_deck,
        origin=Origin(xyz=DECK_CENTER, rpy=(DECK_ROLL, 0.0, 0.0)),
    )

    bill_door = model.part("bill_door")
    bill_door.visual(
        mesh_from_cadquery(_build_bill_door(), "slot_bill_door"),
        material=trim_finish,
        name="door_panel",
    )
    model.articulation(
        "cabinet_to_bill_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=bill_door,
        origin=Origin(xyz=DOOR_HINGE_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )

    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_SIZE),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] * 0.5)),
            material=button_finish,
            name="cap",
        )
        button.visual(
            Box((BUTTON_SIZE[0] * 0.68, 0.004, 0.003)),
            origin=Origin(xyz=(0.0, 0.012, 0.009)),
            material=button_light,
            name="legend",
        )
        model.articulation(
            f"button_deck_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=button_deck,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, DECK_SIZE[2] * 0.5)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=-0.003,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lower_display = object_model.get_part("lower_display")
    top_display = object_model.get_part("top_display")
    button_deck = object_model.get_part("button_deck")
    bill_door = object_model.get_part("bill_door")
    bill_door_joint = object_model.get_articulation("cabinet_to_bill_door")

    ctx.expect_overlap(
        bill_door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="shell",
        min_overlap=0.12,
        name="bill door sits in the lower front cabinet region",
    )
    ctx.expect_origin_gap(
        lower_display,
        button_deck,
        axis="z",
        min_gap=0.34,
        name="lower display sits clearly above the button deck",
    )
    ctx.expect_origin_gap(
        top_display,
        lower_display,
        axis="z",
        min_gap=0.48,
        name="top display sits above the main lower display",
    )

    closed_door = ctx.part_element_world_aabb(bill_door, elem="door_panel")
    limits = bill_door_joint.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({bill_door_joint: limits.upper}):
            opened_door = ctx.part_element_world_aabb(bill_door, elem="door_panel")
        ctx.check(
            "bill door swings outward from the cabinet front",
            closed_door is not None
            and opened_door is not None
            and opened_door[0][1] < closed_door[0][1] - 0.06,
            details=f"closed={closed_door}, opened={opened_door}",
        )

    button_parts = [object_model.get_part(f"button_{index}") for index in range(len(BUTTON_XS))]
    button_joints = [
        object_model.get_articulation(f"button_deck_to_button_{index}")
        for index in range(len(BUTTON_XS))
    ]
    for button_part in button_parts:
        ctx.allow_overlap(
            button_part,
            button_deck,
            elem_a="cap",
            elem_b="deck_body",
            reason="The play buttons are represented as seated in a simplified solid deck without explicit button bores.",
        )
    rest_positions = {
        part.name: ctx.part_world_position(part)
        for part in button_parts
    }

    for active_index, (button_part, button_joint) in enumerate(zip(button_parts, button_joints)):
        limits = button_joint.motion_limits
        if limits is None or limits.lower is None:
            continue
        with ctx.pose({button_joint: limits.lower}):
            pressed_position = ctx.part_world_position(button_part)
            ctx.check(
                f"{button_part.name} presses into the deck",
                rest_positions[button_part.name] is not None
                and pressed_position is not None
                and pressed_position[1] > rest_positions[button_part.name][1] + 0.001
                and pressed_position[2] < rest_positions[button_part.name][2] - 0.001,
                details=f"rest={rest_positions[button_part.name]}, pressed={pressed_position}",
            )
            for other_index, other_part in enumerate(button_parts):
                if other_index == active_index:
                    continue
                other_pressed_position = ctx.part_world_position(other_part)
                rest_other = rest_positions[other_part.name]
                ctx.check(
                    f"{button_part.name} moves independently of {other_part.name}",
                    rest_other is not None
                    and other_pressed_position is not None
                    and max(
                        abs(other_pressed_position[0] - rest_other[0]),
                        abs(other_pressed_position[1] - rest_other[1]),
                        abs(other_pressed_position[2] - rest_other[2]),
                    )
                    < 1e-6,
                    details=f"rest={rest_other}, posed={other_pressed_position}",
                )

    return ctx.report()


object_model = build_object_model()
