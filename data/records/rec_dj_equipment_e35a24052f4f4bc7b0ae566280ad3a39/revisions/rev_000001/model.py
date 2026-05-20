from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TOP_Z = 0.051


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _add_tick_pair(part, x: float, y: float, z: float, material: Material) -> None:
    """Two small white scale ticks around a rotary channel control."""
    part.visual(
        Box((0.0026, 0.014, 0.0012)),
        origin=Origin(xyz=(x - 0.022, y, z), rpy=(0.0, 0.0, 0.55)),
        material=material,
        name=f"tick_{x:.2f}_{y:.2f}_a",
    )


def _aabb_center_z(aabb) -> float | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return 0.5 * (lower[2] + upper[2])
    part.visual(
        Box((0.0026, 0.014, 0.0012)),
        origin=Origin(xyz=(x + 0.022, y, z), rpy=(0.0, 0.0, -0.55)),
        material=material,
        name=f"tick_{x:.2f}_{y:.2f}_b",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_mixer")

    black = _mat(model, "mat_black_plastic", (0.008, 0.009, 0.011, 1.0))
    graphite = _mat(model, "mat_graphite_panel", (0.055, 0.058, 0.062, 1.0))
    dark = _mat(model, "mat_dark_recess", (0.0, 0.0, 0.0, 1.0))
    rubber = _mat(model, "mat_rubber", (0.015, 0.015, 0.017, 1.0))
    knob_gray = _mat(model, "mat_knob_gray", (0.24, 0.25, 0.26, 1.0))
    fader_silver = _mat(model, "mat_fader_silver", (0.72, 0.72, 0.70, 1.0))
    white = _mat(model, "mat_white_print", (0.92, 0.92, 0.86, 1.0))
    blue = _mat(model, "mat_cue_blue", (0.05, 0.20, 0.95, 1.0))
    red = _mat(model, "mat_red_led", (1.0, 0.05, 0.02, 1.0))
    yellow = _mat(model, "mat_yellow_led", (1.0, 0.78, 0.04, 1.0))
    green = _mat(model, "mat_green_led", (0.05, 0.90, 0.18, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.44, 0.32, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=black,
        name="chassis",
    )
    deck.visual(
        Box((0.42, 0.30, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=graphite,
        name="faceplate",
    )
    deck.visual(
        Box((0.006, 0.265, 0.0015)),
        origin=Origin(xyz=(0.0, 0.012, TOP_Z + 0.00075)),
        material=white,
        name="center_divider",
    )
    for i, x in enumerate((-0.105, 0.105)):
        deck.visual(
            Box((0.155, 0.226, 0.0014)),
            origin=Origin(xyz=(x, 0.035, TOP_Z + 0.0007)),
            material=Material(f"channel_tint_{i}", (0.038, 0.041, 0.045, 1.0)),
            name=f"channel_strip_{i}",
        )
        deck.visual(
            Box((0.090, 0.006, 0.0012)),
            origin=Origin(xyz=(x, 0.142, TOP_Z + 0.00055)),
            material=white,
            name=f"channel_label_{i}",
        )
        deck.visual(
            Box((0.070, 0.005, 0.0012)),
            origin=Origin(xyz=(x, -0.060, TOP_Z + 0.0014)),
            material=white,
            name=f"eq_label_{i}",
        )

    # Printed scale ticks for the channel rotary controls.
    knob_positions: list[tuple[str, float, float]] = []
    for col, x in enumerate((-0.105, 0.105)):
        for row, y in enumerate((0.115, 0.070, 0.025, -0.020)):
            knob_positions.append((f"channel_knob_{col}_{row}", x, y))
            _add_tick_pair(deck, x, y + 0.002, TOP_Z + 0.00145, white)

    # Crossfader channel and end stops.
    deck.visual(
        Box((0.335, 0.020, 0.003)),
        origin=Origin(xyz=(0.0, -0.124, TOP_Z + 0.0015)),
        material=dark,
        name="crossfader_channel",
    )
    for x in (-0.174, 0.174):
        deck.visual(
            Box((0.006, 0.032, 0.004)),
            origin=Origin(xyz=(x, -0.124, TOP_Z + 0.002)),
            material=rubber,
            name=f"crossfader_stop_{'a' if x < 0 else 'b'}",
        )

    # Meter LEDs and small square cue buttons make the panel read as a DJ mixer.
    for row, mat in enumerate((green, green, green, yellow, yellow, red)):
        y = 0.035 + row * 0.014
        for x in (-0.023, 0.023):
            deck.visual(
                Box((0.012, 0.006, 0.0018)),
                origin=Origin(xyz=(x, y, TOP_Z + 0.0019)),
                material=mat,
                name=f"meter_{row}_{'a' if x < 0 else 'b'}",
            )
    for i, x in enumerate((-0.156, -0.132, 0.132, 0.156)):
        deck.visual(
            Box((0.018, 0.018, 0.004)),
            origin=Origin(xyz=(x, -0.079, TOP_Z + 0.002)),
            material=blue if i in (1, 2) else rubber,
            name=f"cue_button_{i}",
        )

    # Cue lever yoke on the faceplate.
    cue_x = 0.0
    cue_y = -0.067
    cue_pivot_z = TOP_Z + 0.007
    deck.visual(
        Box((0.006, 0.020, 0.014)),
        origin=Origin(xyz=(-0.019, cue_y, TOP_Z + 0.007)),
        material=black,
        name="cue_yoke_0",
    )
    deck.visual(
        Box((0.006, 0.020, 0.014)),
        origin=Origin(xyz=(0.019, cue_y, TOP_Z + 0.007)),
        material=black,
        name="cue_yoke_1",
    )
    deck.visual(
        Box((0.048, 0.024, 0.002)),
        origin=Origin(xyz=(cue_x, cue_y, TOP_Z + 0.001)),
        material=dark,
        name="cue_slot_plate",
    )

    # Eight independent rotary channel knobs.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.020,
            body_style="faceted",
            base_diameter=0.037,
            top_diameter=0.029,
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0008, width=0.0014),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "channel_rotary_knob",
    )
    for name, x, y in knob_positions:
        knob = model.part(name)
        knob.visual(
            knob_mesh,
            origin=Origin(),
            material=knob_gray,
            name="knob_cap",
        )
        knob.visual(
            Box((0.0032, 0.016, 0.0016)),
            origin=Origin(xyz=(0.0, 0.0065, 0.0208)),
            material=white,
            name="pointer_line",
        )
        model.articulation(
            f"deck_to_{name}",
            ArticulationType.REVOLUTE,
            parent=deck,
            child=knob,
            origin=Origin(xyz=(x, y, TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=-2.35, upper=2.35),
        )

    # Crossfader moving cap: child frame is centered on the slot at the rail top.
    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.058, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=fader_silver,
        name="fader_cap",
    )
    crossfader.visual(
        Box((0.036, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=white,
        name="finger_ridge",
    )
    model.articulation(
        "deck_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.124, TOP_Z + 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.9, lower=-0.126, upper=0.126),
    )

    # Cue lever with an exposed hinge barrel captured by the yoke.
    cue_lever = model.part("cue_lever")
    cue_lever.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    lever_len = 0.062
    lever_angle = -1.05
    cue_lever.visual(
        Cylinder(radius=0.0032, length=lever_len),
        origin=Origin(
            xyz=(0.0, 0.5 * lever_len * math.sin(-lever_angle), 0.5 * lever_len * math.cos(lever_angle)),
            rpy=(lever_angle, 0.0, 0.0),
        ),
        material=fader_silver,
        name="lever_stem",
    )
    cue_lever.visual(
        Box((0.030, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.051, 0.029), rpy=(lever_angle, 0.0, 0.0)),
        material=blue,
        name="lever_tip",
    )
    model.articulation(
        "deck_to_cue_lever",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=cue_lever,
        origin=Origin(xyz=(cue_x, cue_y, cue_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    crossfader = object_model.get_part("crossfader")
    cue_lever = object_model.get_part("cue_lever")
    slide = object_model.get_articulation("deck_to_crossfader")
    cue_hinge = object_model.get_articulation("deck_to_cue_lever")

    for col in range(2):
        for row in range(4):
            knob = object_model.get_part(f"channel_knob_{col}_{row}")
            joint = object_model.get_articulation(f"deck_to_channel_knob_{col}_{row}")
            ctx.expect_gap(
                knob,
                deck,
                axis="z",
                max_gap=0.001,
                max_penetration=0.000001,
                elem_a="knob_cap",
                negative_elem="faceplate",
                name=f"rotary knob {col}-{row} seated on faceplate",
            )
            with ctx.pose({joint: 0.8}):
                ctx.expect_gap(
                    knob,
                    deck,
                    axis="z",
                    max_gap=0.001,
                    max_penetration=0.000001,
                    elem_a="knob_cap",
                    negative_elem="faceplate",
                    name=f"rotary knob {col}-{row} remains seated while turned",
                )

    ctx.expect_gap(
        crossfader,
        deck,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        elem_a="fader_cap",
        negative_elem="crossfader_channel",
        name="crossfader cap rides on the channel",
    )
    rest_pos = ctx.part_world_position(crossfader)
    with ctx.pose({slide: 0.110}):
        ctx.expect_within(
            crossfader,
            deck,
            axes="x",
            margin=0.003,
            inner_elem="fader_cap",
            outer_elem="crossfader_channel",
            name="crossfader remains captured at right travel",
        )
        right_pos = ctx.part_world_position(crossfader)
    with ctx.pose({slide: -0.110}):
        ctx.expect_within(
            crossfader,
            deck,
            axes="x",
            margin=0.003,
            inner_elem="fader_cap",
            outer_elem="crossfader_channel",
            name="crossfader remains captured at left travel",
        )
        left_pos = ctx.part_world_position(crossfader)
    ctx.check(
        "crossfader moves along the mixer width",
        rest_pos is not None
        and right_pos is not None
        and left_pos is not None
        and right_pos[0] > rest_pos[0] + 0.09
        and left_pos[0] < rest_pos[0] - 0.09,
        details=f"left={left_pos}, rest={rest_pos}, right={right_pos}",
    )

    ctx.expect_contact(
        cue_lever,
        deck,
        elem_a="hinge_barrel",
        elem_b="cue_yoke_0",
        name="cue lever barrel contacts one yoke cheek",
    )
    cue_rest_tip_z = _aabb_center_z(ctx.part_element_world_aabb(cue_lever, elem="lever_tip"))
    with ctx.pose({cue_hinge: 0.35}):
        cue_forward_tip_z = _aabb_center_z(ctx.part_element_world_aabb(cue_lever, elem="lever_tip"))
    ctx.check(
        "cue lever tip rises when hinged",
        cue_rest_tip_z is not None and cue_forward_tip_z is not None and cue_forward_tip_z > cue_rest_tip_z + 0.010,
        details=f"rest_tip_z={cue_rest_tip_z}, posed_tip_z={cue_forward_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
