from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_deck_dj_controller")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.055, 0.058, 0.064, 1.0))
    soft_black = model.material("soft_black", rgba=(0.006, 0.006, 0.007, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.018, 0.018, 0.020, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.56, 0.58, 0.58, 1.0))
    white_mark = model.material("white_mark", rgba=(0.92, 0.94, 0.92, 1.0))
    blue_led = model.material("blue_led", rgba=(0.05, 0.35, 1.0, 1.0))
    amber_led = model.material("amber_led", rgba=(1.0, 0.55, 0.05, 1.0))
    magenta_pad = model.material("magenta_pad", rgba=(0.82, 0.08, 0.45, 1.0))

    controller = model.part("controller")
    shell_profile = rounded_rect_profile(0.78, 0.38, 0.030, corner_segments=10)
    controller.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(shell_profile, 0.044),
            "rounded_controller_shell",
        ),
        material=matte_black,
        name="rounded_shell",
    )
    controller.visual(
        Box((0.745, 0.350, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=soft_black,
        name="top_plate",
    )

    # Separate overlay plates make the two decks and center mixer read clearly.
    deck_top = 0.052
    for i, x in enumerate((-0.235, 0.235)):
        controller.visual(
            Box((0.288, 0.322, 0.003)),
            origin=Origin(xyz=(x, 0.000, 0.0505)),
            material=charcoal,
            name=f"deck_panel_{i}",
        )
        controller.visual(
            Cylinder(radius=0.120, length=0.0025),
            origin=Origin(xyz=(x, 0.035, 0.05325)),
            material=soft_black,
            name=f"platter_well_{i}",
        )
        controller.visual(
            Box((0.018, 0.180, 0.0025)),
            origin=Origin(xyz=(x + (0.125 if x < 0.0 else -0.125), 0.020, 0.05325)),
            material=soft_black,
            name=f"pitch_slot_{i}",
        )
        controller.visual(
            Box((0.010, 0.040, 0.004)),
            origin=Origin(xyz=(x + (0.125 if x < 0.0 else -0.125), 0.020, 0.056)),
            material=blue_led,
            name=f"pitch_marker_{i}",
        )

    controller.visual(
        Box((0.150, 0.322, 0.003)),
        origin=Origin(xyz=(0.0, 0.000, 0.0505)),
        material=charcoal,
        name="mixer_panel",
    )
    controller.visual(
        Box((0.170, 0.014, 0.0025)),
        origin=Origin(xyz=(0.0, -0.126, 0.05325)),
        material=soft_black,
        name="crossfader_slot",
    )
    controller.visual(
        Box((0.118, 0.012, 0.0025)),
        origin=Origin(xyz=(0.0, -0.074, 0.05325)),
        material=soft_black,
        name="channel_fader_slots",
    )

    # Low raised meter bars and printed separators on the mixer face.
    for x in (-0.042, 0.042):
        for j in range(5):
            controller.visual(
                Box((0.008, 0.004, 0.0012)),
                origin=Origin(xyz=(x, 0.104 - j * 0.008, 0.0523)),
                material=blue_led if j < 3 else amber_led,
                name=f"meter_{'neg' if x < 0 else 'pos'}_{j}",
            )

    # Transport/performance pads are separate prismatic button parts below.
    pad_positions: list[tuple[float, float, str]] = []
    for deck_i, deck_x in enumerate((-0.235, 0.235)):
        for row, y in enumerate((-0.120, -0.094)):
            for col, dx in enumerate((-0.040, -0.012, 0.016, 0.044)):
                pad_positions.append((deck_x + dx, y, f"pad_{deck_i}_{row}_{col}"))

    # Two spinning jog platters, each with a rubber outside rim and metal top.
    for i, x in enumerate((-0.235, 0.235)):
        platter = model.part(f"platter_{i}")
        platter.visual(
            Cylinder(radius=0.108, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=dark_rubber,
            name="platter_base",
        )
        platter.visual(
            mesh_from_geometry(TorusGeometry(0.098, 0.006, radial_segments=18, tubular_segments=72), f"platter_rim_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=dark_rubber,
            name="rubber_rim",
        )
        platter.visual(
            Cylinder(radius=0.086, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=brushed_metal,
            name="metal_disk",
        )
        platter.visual(
            Cylinder(radius=0.032, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0195)),
            material=soft_black,
            name="center_label",
        )
        # White tick marks on the platter face make rotation perceptible.
        for mark in range(8):
            angle = mark * math.tau / 8.0
            radius = 0.067
            platter.visual(
                Box((0.004, 0.024, 0.001)),
                origin=Origin(
                    xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0184),
                    rpy=(0.0, 0.0, angle),
                ),
                material=white_mark,
                name=f"tick_{mark}",
            )

        model.articulation(
            f"controller_to_platter_{i}",
            ArticulationType.REVOLUTE,
            parent=controller,
            child=platter,
            origin=Origin(xyz=(x, 0.035, deck_top)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0, lower=-math.tau, upper=math.tau),
        )

    # Crossfader cap on a horizontal prismatic rail across the front of the mixer.
    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.014, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_metal,
        name="fader_stem",
    )
    crossfader.visual(
        Box((0.050, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_rubber,
        name="fader_cap",
    )
    crossfader.visual(
        Box((0.034, 0.003, 0.001)),
        origin=Origin(xyz=(0.0, 0.010, 0.0245)),
        material=white_mark,
        name="fader_mark",
    )
    model.articulation(
        "controller_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=controller,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.126, 0.0545)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.50, lower=-0.060, upper=0.060),
    )

    # Six articulated rotary EQ/gain knobs in the center mixer section.
    knob_geom = KnobGeometry(
        0.028,
        0.022,
        body_style="faceted",
        base_diameter=0.030,
        top_diameter=0.022,
        grip=KnobGrip(style="ribbed", count=14, depth=0.0008, width=0.0015),
        center=False,
    )
    for row, y in enumerate((0.112, 0.070, 0.028)):
        for col, x in enumerate((-0.037, 0.037)):
            knob = model.part(f"knob_{row}_{col}")
            knob.visual(
                mesh_from_geometry(knob_geom, f"mixer_knob_{row}_{col}"),
                material=dark_rubber,
                name="knob_cap",
            )
            knob.visual(
                Box((0.003, 0.015, 0.0012)),
                origin=Origin(xyz=(0.0, 0.0045, 0.0222)),
                material=white_mark,
                name="pointer_mark",
            )
            model.articulation(
                f"controller_to_knob_{row}_{col}",
                ArticulationType.REVOLUTE,
                parent=controller,
                child=knob,
                origin=Origin(xyz=(x, y, deck_top)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=-2.62, upper=2.62),
            )

    for x, y, name in pad_positions:
        pad = model.part(name)
        pad.visual(
            Box((0.022, 0.018, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=magenta_pad if "0_1" in name or "1_1" in name else blue_led,
            name="pad_cap",
        )
        model.articulation(
            f"controller_to_{name}",
            ArticulationType.PRISMATIC,
            parent=controller,
            child=pad,
            origin=Origin(xyz=(x, y, deck_top)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=-0.003, upper=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    controller = object_model.get_part("controller")
    crossfader = object_model.get_part("crossfader")
    crossfader_joint = object_model.get_articulation("controller_to_crossfader")

    platter_joints = [
        object_model.get_articulation("controller_to_platter_0"),
        object_model.get_articulation("controller_to_platter_1"),
    ]
    knob_joints = [
        object_model.get_articulation(f"controller_to_knob_{row}_{col}")
        for row in range(3)
        for col in range(2)
    ]

    ctx.check(
        "two revolute jog platters",
        all(j is not None and j.articulation_type == ArticulationType.REVOLUTE for j in platter_joints),
        details=f"platter_joints={platter_joints}",
    )
    ctx.check(
        "six articulated rotary knobs",
        all(j is not None and j.articulation_type == ArticulationType.REVOLUTE for j in knob_joints),
        details=f"knob_joints={knob_joints}",
    )
    ctx.check(
        "crossfader is prismatic",
        crossfader_joint is not None and crossfader_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"crossfader_joint={crossfader_joint}",
    )

    for i in range(2):
        platter = object_model.get_part(f"platter_{i}")
        ctx.expect_gap(
            platter,
            controller,
            axis="z",
            positive_elem="platter_base",
            negative_elem=f"deck_panel_{i}",
            max_gap=0.001,
            max_penetration=0.000001,
            name=f"platter_{i} sits on its deck",
        )

    ctx.expect_within(
        crossfader,
        controller,
        axes="x",
        inner_elem="fader_stem",
        outer_elem="crossfader_slot",
        margin=0.003,
        name="crossfader stem starts inside rail",
    )
    rest_pos = ctx.part_world_position(crossfader)
    with ctx.pose({crossfader_joint: 0.060}):
        ctx.expect_within(
            crossfader,
            controller,
            axes="x",
            inner_elem="fader_stem",
            outer_elem="crossfader_slot",
            margin=0.003,
            name="crossfader stem remains in rail at travel end",
        )
        end_pos = ctx.part_world_position(crossfader)
    ctx.check(
        "crossfader travels along rail",
        rest_pos is not None and end_pos is not None and end_pos[0] > rest_pos[0] + 0.050,
        details=f"rest={rest_pos}, end={end_pos}",
    )

    return ctx.report()


object_model = build_object_model()
