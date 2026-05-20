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


BODY_W = 0.395
BODY_D = 0.305
BODY_H = 0.675
BODY_FRONT_Y = BODY_D * 0.5
BODY_BACK_Y = -BODY_D * 0.5
BODY_LOWER_FRONT_Z = 0.455
BODY_UPPER_FRONT_Y = 0.092
HEAD_PANEL_LENGTH = 0.212
HEAD_PANEL_ANGLE = 0.337
FRONT_PANEL_CENTER_Y = 0.122
FRONT_PANEL_CENTER_Z = 0.560
FRONT_PANEL_HALF_THICKNESS = 0.004

BIN_W = 0.326
BIN_D = 0.238
BIN_H = 0.392
BIN_WALL = 0.0035
BIN_TRAVEL = 0.165
BIN_ORIGIN_Z = 0.028
BIN_ORIGIN_Y = BODY_FRONT_Y - 0.015
def _build_top_deck() -> cq.Workplane:
    deck_center_y = (BODY_UPPER_FRONT_Y + BODY_BACK_Y) * 0.5
    deck_depth = BODY_UPPER_FRONT_Y - BODY_BACK_Y
    deck = (
        cq.Workplane("XY")
        .box(BODY_W - 0.016, deck_depth, 0.010, centered=(True, True, False))
        .translate((0.0, deck_center_y, BODY_H - 0.010))
    )
    paper_slot = (
        cq.Workplane("XY")
        .box(0.232, 0.015, 0.050, centered=(True, True, False))
        .translate((-0.040, 0.018, BODY_H - 0.020))
    )
    card_slot = (
        cq.Workplane("XY")
        .box(0.070, 0.009, 0.045, centered=(True, True, False))
        .translate((0.132, 0.018, BODY_H - 0.020))
    )
    return deck.cut(paper_slot).cut(card_slot)


def _build_bin() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BIN_W, BIN_D, BIN_H, centered=(True, True, False))
        .translate((0.0, -(BIN_D * 0.5), 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(BIN_W - (2.0 * BIN_WALL), BIN_D - (2.0 * BIN_WALL), BIN_H - BIN_WALL, centered=(True, True, False))
        .translate((0.0, -(BIN_D * 0.5), BIN_WALL))
    )
    fascia = (
        cq.Workplane("XY")
        .box(BIN_W + 0.016, 0.012, BIN_H + 0.018, centered=(True, True, False))
        .translate((0.0, 0.006, 0.0))
    )
    handle_cut = (
        cq.Workplane("XY")
        .box(0.146, 0.030, 0.034, centered=(True, True, True))
        .translate((0.0, 0.010, BIN_H * 0.74))
    )
    handle_finger_relief = (
        cq.Workplane("XZ")
        .center(0.0, BIN_H * 0.74)
        .rect(0.110, 0.018)
        .extrude(0.016)
        .translate((0.0, 0.012, 0.0))
    )
    return outer.cut(inner).union(fascia).cut(handle_cut).cut(handle_finger_relief)
def _front_panel_mount_origin(x: float, panel_z: float, *, standoff: float = 0.0) -> Origin:
    along_y = -math.sin(HEAD_PANEL_ANGLE)
    along_z = math.cos(HEAD_PANEL_ANGLE)
    normal_y = math.cos(HEAD_PANEL_ANGLE)
    normal_z = math.sin(HEAD_PANEL_ANGLE)
    return Origin(
        xyz=(
            x,
            FRONT_PANEL_CENTER_Y + (panel_z * along_y) + ((FRONT_PANEL_HALF_THICKNESS + standoff) * normal_y),
            FRONT_PANEL_CENTER_Z + (panel_z * along_z) + ((FRONT_PANEL_HALF_THICKNESS + standoff) * normal_z),
        ),
        rpy=(HEAD_PANEL_ANGLE - (math.pi * 0.5), 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_shredder")

    housing = model.material("housing", rgba=(0.14, 0.15, 0.17, 1.0))
    trim = model.material("trim", rgba=(0.24, 0.25, 0.27, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    cutter_metal = model.material("cutter_metal", rgba=(0.48, 0.49, 0.50, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    button_red = model.material("button_red", rgba=(0.63, 0.10, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.008, BODY_D, BODY_LOWER_FRONT_Z)),
        origin=Origin(xyz=(-(BODY_W * 0.5) + 0.004, 0.0, BODY_LOWER_FRONT_Z * 0.5)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((0.008, BODY_D, BODY_LOWER_FRONT_Z)),
        origin=Origin(xyz=((BODY_W * 0.5) - 0.004, 0.0, BODY_LOWER_FRONT_Z * 0.5)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 0.016, 0.008, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + 0.004, BODY_H * 0.5)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((0.008, BODY_UPPER_FRONT_Y - BODY_BACK_Y, BODY_H - BODY_LOWER_FRONT_Z)),
        origin=Origin(
            xyz=(
                -(BODY_W * 0.5) + 0.004,
                (BODY_UPPER_FRONT_Y + BODY_BACK_Y) * 0.5,
                BODY_LOWER_FRONT_Z + ((BODY_H - BODY_LOWER_FRONT_Z) * 0.5),
            )
        ),
        material=housing,
        name="left_head_cheek",
    )
    body.visual(
        Box((0.008, BODY_UPPER_FRONT_Y - BODY_BACK_Y, BODY_H - BODY_LOWER_FRONT_Z)),
        origin=Origin(
            xyz=(
                (BODY_W * 0.5) - 0.004,
                (BODY_UPPER_FRONT_Y + BODY_BACK_Y) * 0.5,
                BODY_LOWER_FRONT_Z + ((BODY_H - BODY_LOWER_FRONT_Z) * 0.5),
            )
        ),
        material=housing,
        name="right_head_cheek",
    )
    body.visual(
        mesh_from_cadquery(_build_top_deck(), "shredder_top_deck"),
        material=housing,
        name="top_deck",
    )
    body.visual(
        Box((BODY_W - 0.016, 0.008, HEAD_PANEL_LENGTH)),
        origin=Origin(
            xyz=(0.0, 0.122, 0.560),
            rpy=(HEAD_PANEL_ANGLE, 0.0, 0.0),
        ),
        material=housing,
        name="front_panel",
    )
    body.visual(
        Box((BODY_W - 0.070, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, BODY_UPPER_FRONT_Y - 0.022, BODY_H - 0.022)),
        material=trim,
        name="slot_bridge",
    )
    body.visual(
        Box((BODY_W - 0.005, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, BODY_BACK_Y + 0.014, 0.004)),
        material=trim,
        name="rear_base_rail",
    )
    body.visual(
        Box((BODY_W - 0.005, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.016, 0.004)),
        material=trim,
        name="front_base_rail",
    )
    body.visual(
        Box((0.012, BODY_D - 0.060, 0.008)),
        origin=Origin(xyz=(-(BODY_W * 0.5) + 0.006, 0.0, -0.004)),
        material=trim,
        name="left_skid",
    )
    body.visual(
        Box((0.012, BODY_D - 0.060, 0.008)),
        origin=Origin(xyz=((BODY_W * 0.5) - 0.006, 0.0, -0.004)),
        material=trim,
        name="right_skid",
    )
    body.visual(
        Box((BODY_W - 0.005, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.007, 0.456)),
        material=trim,
        name="opening_lintel",
    )
    body.visual(
        Box((BODY_W - 0.005, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.007, 0.024)),
        material=trim,
        name="bottom_lip",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_build_bin(), "waste_bin"),
        material=bin_plastic,
        name="bin_shell",
    )

    model.articulation(
        "body_to_bin",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bin_part,
        origin=Origin(xyz=(0.0, BIN_ORIGIN_Y, BIN_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.28,
            lower=0.0,
            upper=BIN_TRAVEL,
        ),
    )

    main_flap = model.part("main_flap")
    main_flap.visual(
        Cylinder(radius=0.0035, length=0.244),
        origin=Origin(xyz=(0.0, 0.0, 0.0035), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    main_flap.visual(
        Box((0.244, 0.033, 0.003)),
        origin=Origin(xyz=(0.0, 0.0165, 0.0015)),
        material=trim,
        name="flap_panel",
    )
    model.articulation(
        "body_to_main_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=main_flap,
        origin=Origin(xyz=(-0.040, 0.0105, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    card_flap = model.part("card_flap")
    card_flap.visual(
        Cylinder(radius=0.0030, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0030), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    card_flap.visual(
        Box((0.082, 0.021, 0.003)),
        origin=Origin(xyz=(0.0, 0.0105, 0.0015)),
        material=trim,
        name="flap_panel",
    )
    model.articulation(
        "body_to_card_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=card_flap,
        origin=Origin(xyz=(0.132, 0.0135, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    drum_0 = model.part("drum_0")
    drum_0.visual(
        Cylinder(radius=0.004, length=0.379),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=cutter_metal,
        name="shaft",
    )
    for index, x_offset in enumerate((-0.114, -0.078, -0.042, -0.006, 0.030, 0.066, 0.102)):
        drum_0.visual(
            Cylinder(radius=0.0095, length=0.010),
            origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=cutter_metal,
            name=f"cutter_{index}",
        )
    model.articulation(
        "body_to_drum_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum_0,
        origin=Origin(xyz=(0.0, 0.006, 0.626)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )

    drum_1 = model.part("drum_1")
    drum_1.visual(
        Cylinder(radius=0.004, length=0.379),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=cutter_metal,
        name="shaft",
    )
    for index, x_offset in enumerate((-0.096, -0.060, -0.024, 0.012, 0.048, 0.084, 0.120)):
        drum_1.visual(
            Cylinder(radius=0.0095, length=0.010),
            origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=cutter_metal,
            name=f"cutter_{index}",
        )
    model.articulation(
        "body_to_drum_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum_1,
        origin=Origin(xyz=(0.0, 0.028, 0.626)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=trim,
        name="dial_hub",
    )
    selector_dial.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim,
        name="dial_skirt",
    )
    selector_dial.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=control_black,
        name="dial_body",
    )
    selector_dial.visual(
        Box((0.004, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, 0.009, 0.018)),
        material=trim,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=_front_panel_mount_origin(-0.042, 0.014, standoff=0.001),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    reverse_bezel = model.part("reverse_bezel")
    reverse_bezel.visual(
        Box((0.036, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=trim,
        name="back_plate",
    )
    reverse_bezel.visual(
        Box((0.008, 0.022, 0.006)),
        origin=Origin(xyz=(-0.014, 0.0, 0.005)),
        material=trim,
        name="left_wall",
    )
    reverse_bezel.visual(
        Box((0.008, 0.022, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.005)),
        material=trim,
        name="right_wall",
    )
    reverse_bezel.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.008, 0.005)),
        material=trim,
        name="lower_wall",
    )
    reverse_bezel.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.008, 0.005)),
        material=trim,
        name="upper_wall",
    )
    model.articulation(
        "body_to_reverse_bezel",
        ArticulationType.FIXED,
        parent=body,
        child=reverse_bezel,
        origin=_front_panel_mount_origin(0.046, 0.014),
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        Box((0.014, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_red,
        name="button_stem",
    )
    reverse_button.visual(
        Box((0.024, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=button_red,
        name="button_cap",
    )
    model.articulation(
        "reverse_bezel_to_button",
        ArticulationType.PRISMATIC,
        parent=reverse_bezel,
        child=reverse_button,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.06,
            lower=0.0,
            upper=0.002,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bin_part = object_model.get_part("bin")
    bin_joint = object_model.get_articulation("body_to_bin")
    main_flap = object_model.get_part("main_flap")
    card_flap = object_model.get_part("card_flap")
    main_flap_joint = object_model.get_articulation("body_to_main_flap")
    card_flap_joint = object_model.get_articulation("body_to_card_flap")
    drum_0_joint = object_model.get_articulation("body_to_drum_0")
    drum_1_joint = object_model.get_articulation("body_to_drum_1")
    dial_joint = object_model.get_articulation("body_to_selector_dial")
    button_joint = object_model.get_articulation("reverse_bezel_to_button")
    reverse_button = object_model.get_part("reverse_button")

    ctx.allow_overlap(
        body,
        object_model.get_part("selector_dial"),
        elem_a="front_panel",
        elem_b="dial_hub",
        reason="The selector dial uses a short mounting boss that intentionally enters the simplified solid control-panel proxy.",
    )
    ctx.allow_overlap(
        body,
        object_model.get_part("reverse_bezel"),
        elem_a="front_panel",
        elem_b="back_plate",
        reason="The reverse-button bezel uses a thin mounting plate captured against the simplified solid control-panel proxy.",
    )

    ctx.expect_within(
        bin_part,
        body,
        axes="xz",
        margin=0.040,
        name="bin stays laterally and vertically within the shredder envelope",
    )

    rest_pos = ctx.part_world_position(bin_part)
    with ctx.pose({bin_joint: BIN_TRAVEL}):
        extended_pos = ctx.part_world_position(bin_part)

    ctx.check(
        "bin slides forward from the lower body",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({main_flap_joint: 0.0, card_flap_joint: 0.0}):
        ctx.expect_gap(
            main_flap,
            body,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="top_deck",
            max_gap=0.004,
            max_penetration=0.0,
            name="main flap sits on the top deck when closed",
        )
        ctx.expect_gap(
            card_flap,
            body,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="top_deck",
            max_gap=0.004,
            max_penetration=0.0,
            name="card flap sits on the top deck when closed",
        )
        main_closed = ctx.part_world_aabb(main_flap)
        card_closed = ctx.part_world_aabb(card_flap)
    with ctx.pose({main_flap_joint: 1.0, card_flap_joint: 0.9}):
        main_open = ctx.part_world_aabb(main_flap)
        card_open = ctx.part_world_aabb(card_flap)

    ctx.check(
        "slot flaps lift upward when opened",
        main_closed is not None
        and main_open is not None
        and card_closed is not None
        and card_open is not None
        and main_open[1][2] > main_closed[1][2] + 0.018
        and card_open[1][2] > card_closed[1][2] + 0.010,
        details=f"main_closed={main_closed}, main_open={main_open}, card_closed={card_closed}, card_open={card_open}",
    )

    button_rest = ctx.part_world_position(reverse_button)
    with ctx.pose({button_joint: 0.002}):
        button_pressed = ctx.part_world_position(reverse_button)
    ctx.check(
        "reverse button presses inward along the sloped panel",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] < button_rest[1] - 0.0003
        and button_pressed[2] < button_rest[2] - 0.0001,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    ctx.check(
        "requested mechanisms use the intended joint types",
        main_flap_joint.articulation_type == ArticulationType.REVOLUTE
        and card_flap_joint.articulation_type == ArticulationType.REVOLUTE
        and drum_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and drum_1_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"main={main_flap_joint.articulation_type}, "
            f"card={card_flap_joint.articulation_type}, "
            f"drum_0={drum_0_joint.articulation_type}, "
            f"drum_1={drum_1_joint.articulation_type}, "
            f"dial={dial_joint.articulation_type}, "
            f"button={button_joint.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
