from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_screen_slot_cabinet")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.018, 1.0))
    charcoal = model.material("charcoal_side_panels", rgba=(0.055, 0.050, 0.060, 1.0))
    blue_glass = model.material("blue_black_display_glass", rgba=(0.015, 0.050, 0.095, 1.0))
    lit_blue = model.material("lit_blue_screen_art", rgba=(0.05, 0.30, 0.95, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.55, 0.52, 0.46, 1.0))
    dark_gap = model.material("black_shadow_gap", rgba=(0.0, 0.0, 0.0, 1.0))
    red_plastic = model.material("red_translucent_button", rgba=(0.90, 0.03, 0.02, 1.0))
    amber_plastic = model.material("amber_translucent_button", rgba=(1.00, 0.62, 0.07, 1.0))
    green_plastic = model.material("green_translucent_button", rgba=(0.04, 0.72, 0.20, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.86, 0.76, 0.18)),
        origin=Origin(xyz=(0.0, 0.02, 0.09)),
        material=charcoal,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.78, 0.64, 1.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=satin_black,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.76, 0.58, 0.58)),
        origin=Origin(xyz=(0.0, 0.04, 1.54)),
        material=satin_black,
        name="top_body",
    )
    cabinet.visual(
        Box((0.83, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, -0.27, 1.25)),
        material=charcoal,
        name="waist_band",
    )
    cabinet.visual(
        Box((0.88, 0.68, 0.08)),
        origin=Origin(xyz=(0.0, 0.02, 2.04)),
        material=charcoal,
        name="crown_cap",
    )
    cabinet.visual(
        Box((0.030, 0.050, 1.86)),
        origin=Origin(xyz=(-0.405, -0.285, 1.04)),
        material=brushed_metal,
        name="front_upright_0",
    )
    cabinet.visual(
        Box((0.030, 0.050, 1.86)),
        origin=Origin(xyz=(0.405, -0.285, 1.04)),
        material=brushed_metal,
        name="front_upright_1",
    )

    # The lower portrait display dominates the play field.
    cabinet.visual(
        Box((0.54, 0.014, 0.62)),
        origin=Origin(xyz=(0.0, -0.325, 1.37)),
        material=blue_glass,
        name="lower_display_glass",
    )
    cabinet.visual(
        Box((0.40, 0.016, 0.46)),
        origin=Origin(xyz=(0.0, -0.334, 1.38)),
        material=lit_blue,
        name="lower_display_art",
    )
    cabinet.visual(
        Box((0.62, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.333, 1.705)),
        material=brushed_metal,
        name="lower_bezel_top",
    )
    cabinet.visual(
        Box((0.62, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.333, 1.035)),
        material=brushed_metal,
        name="lower_bezel_bottom",
    )
    cabinet.visual(
        Box((0.045, 0.035, 0.66)),
        origin=Origin(xyz=(-0.315, -0.333, 1.37)),
        material=brushed_metal,
        name="lower_bezel_side_0",
    )
    cabinet.visual(
        Box((0.045, 0.035, 0.66)),
        origin=Origin(xyz=(0.315, -0.333, 1.37)),
        material=brushed_metal,
        name="lower_bezel_side_1",
    )

    # Smaller top display / marquee area.
    cabinet.visual(
        Box((0.50, 0.014, 0.28)),
        origin=Origin(xyz=(0.0, -0.255, 1.82)),
        material=blue_glass,
        name="top_display_glass",
    )
    cabinet.visual(
        Box((0.36, 0.016, 0.16)),
        origin=Origin(xyz=(0.0, -0.264, 1.82)),
        material=lit_blue,
        name="top_display_art",
    )
    cabinet.visual(
        Box((0.58, 0.034, 0.040)),
        origin=Origin(xyz=(0.0, -0.264, 1.985)),
        material=brushed_metal,
        name="top_bezel_top",
    )
    cabinet.visual(
        Box((0.58, 0.034, 0.040)),
        origin=Origin(xyz=(0.0, -0.264, 1.655)),
        material=brushed_metal,
        name="top_bezel_bottom",
    )
    cabinet.visual(
        Box((0.040, 0.034, 0.32)),
        origin=Origin(xyz=(-0.290, -0.264, 1.82)),
        material=brushed_metal,
        name="top_bezel_side_0",
    )
    cabinet.visual(
        Box((0.040, 0.034, 0.32)),
        origin=Origin(xyz=(0.290, -0.264, 1.82)),
        material=brushed_metal,
        name="top_bezel_side_1",
    )

    # Shallow play-button deck above the bill acceptor.
    cabinet.visual(
        Box((0.72, 0.26, 0.060)),
        origin=Origin(xyz=(0.0, -0.425, 0.925)),
        material=brushed_metal,
        name="button_deck",
    )
    cabinet.visual(
        Box((0.76, 0.030, 0.095)),
        origin=Origin(xyz=(0.0, -0.560, 0.910)),
        material=charcoal,
        name="deck_front_lip",
    )
    cabinet.visual(
        Box((0.78, 0.035, 0.095)),
        origin=Origin(xyz=(0.0, -0.300, 0.910)),
        material=charcoal,
        name="deck_rear_lip",
    )

    # Shadowed opening and fixed hinge leaf for the real hinged bill-door panel.
    cabinet.visual(
        Box((0.40, 0.006, 0.38)),
        origin=Origin(xyz=(0.0, -0.326, 0.60)),
        material=dark_gap,
        name="bill_door_recess",
    )
    cabinet.visual(
        Box((0.012, 0.040, 0.36)),
        origin=Origin(xyz=(-0.1980, -0.335, 0.60)),
        material=brushed_metal,
        name="bill_hinge_leaf",
    )
    cabinet.visual(
        Box((0.045, 0.010, 0.42)),
        origin=Origin(xyz=(-0.220, -0.352, 0.60)),
        material=dark_gap,
        name="bill_door_shadow_gap",
    )

    bill_door = model.part("bill_door")
    bill_door.visual(
        Box((0.34, 0.030, 0.32)),
        origin=Origin(xyz=(0.182, -0.016, 0.0)),
        material=brushed_metal,
        name="panel",
    )
    bill_door.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
        material=brushed_metal,
        name="hinge_barrel",
    )
    bill_door.visual(
        Box((0.22, 0.010, 0.045)),
        origin=Origin(xyz=(0.185, -0.043, 0.045)),
        material=dark_gap,
        name="bill_slot",
    )
    bill_door.visual(
        Box((0.26, 0.012, 0.070)),
        origin=Origin(xyz=(0.185, -0.034, 0.045)),
        material=charcoal,
        name="slot_bezel",
    )
    bill_door.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.292, -0.037, -0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gap,
        name="round_lock",
    )
    bill_door.visual(
        Box((0.080, 0.010, 0.035)),
        origin=Origin(xyz=(0.098, -0.027, -0.083)),
        material=green_plastic,
        name="status_lens",
    )

    model.articulation(
        "cabinet_to_bill_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=bill_door,
        origin=Origin(xyz=(-0.180, -0.345, 0.60)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.30),
    )

    button_xs = (-0.255, -0.145, -0.035, 0.075, 0.185, 0.295)
    button_materials: tuple[Material, ...] = (
        red_plastic,
        amber_plastic,
        amber_plastic,
        amber_plastic,
        amber_plastic,
        green_plastic,
    )
    for idx, (x_pos, material) in enumerate(zip(button_xs, button_materials)):
        button = model.part(f"button_{idx}")
        radius = 0.038 if idx in (0, 5) else 0.034
        button.visual(
            Cylinder(radius=radius, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=material,
            name="cap",
        )
        button.visual(
            Cylinder(radius=radius * 0.62, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=charcoal,
            name="stem_shadow",
        )
        model.articulation(
            f"cabinet_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, -0.455, 0.955)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    bill_door = object_model.get_part("bill_door")
    door_joint = object_model.get_articulation("cabinet_to_bill_door")

    ctx.allow_overlap(
        cabinet,
        bill_door,
        elem_a="bill_hinge_leaf",
        elem_b="hinge_barrel",
        reason=(
            "The hinge barrel is intentionally captured against the fixed hinge leaf "
            "with a tiny hidden overlap so the bill-acceptor door is visibly supported."
        ),
    )
    ctx.expect_contact(
        cabinet,
        bill_door,
        elem_a="bill_hinge_leaf",
        elem_b="hinge_barrel",
        contact_tol=0.002,
        name="bill door is carried by a physical vertical hinge",
    )
    ctx.expect_gap(
        cabinet,
        bill_door,
        axis="y",
        positive_elem="lower_body",
        negative_elem="panel",
        min_gap=0.004,
        max_gap=0.040,
        name="bill door is a separate proud panel on the lower front",
    )
    ctx.expect_overlap(
        bill_door,
        cabinet,
        axes="xz",
        elem_a="panel",
        elem_b="bill_door_recess",
        min_overlap=0.22,
        name="hinged bill door covers the lower-front acceptor opening",
    )

    closed_aabb = ctx.part_element_world_aabb(bill_door, elem="panel")
    with ctx.pose({door_joint: 1.05}):
        open_aabb = ctx.part_element_world_aabb(bill_door, elem="panel")
    ctx.check(
        "bill door swings outward about the vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.14,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    lower_display_aabb = ctx.part_element_world_aabb(cabinet, elem="lower_display_glass")
    top_display_aabb = ctx.part_element_world_aabb(cabinet, elem="top_display_glass")
    ctx.check(
        "lower display is a tall portrait screen",
        lower_display_aabb is not None
        and (lower_display_aabb[1][2] - lower_display_aabb[0][2])
        > (lower_display_aabb[1][0] - lower_display_aabb[0][0]),
        details=f"lower_display_aabb={lower_display_aabb}",
    )
    ctx.check(
        "secondary display sits above the lower display",
        lower_display_aabb is not None
        and top_display_aabb is not None
        and top_display_aabb[0][2] > lower_display_aabb[1][2] - 0.08,
        details=f"lower={lower_display_aabb}, top={top_display_aabb}",
    )

    button_joints = [object_model.get_articulation(f"cabinet_to_button_{idx}") for idx in range(6)]
    button_parts = [object_model.get_part(f"button_{idx}") for idx in range(6)]
    ctx.check(
        "six independent prismatic play buttons are present",
        len(button_joints) == 6 and len(button_parts) == 6,
    )
    for idx, button in enumerate(button_parts):
        ctx.allow_overlap(
            button,
            cabinet,
            elem_a="stem_shadow",
            elem_b="button_deck",
            reason=(
                "The push-button plunger sits in a shallow deck guide; the tiny hidden "
                "intersection is the captured sliding socket."
            ),
        )
        ctx.expect_gap(
            button,
            cabinet,
            axis="z",
            positive_elem="stem_shadow",
            negative_elem="button_deck",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"button {idx} plunger is seated in the deck guide",
        )
        ctx.expect_gap(
            button,
            cabinet,
            axis="z",
            positive_elem="cap",
            negative_elem="button_deck",
            min_gap=0.001,
            max_gap=0.006,
            name=f"button {idx} cap sits just proud of the shallow deck",
        )
        ctx.expect_overlap(
            button,
            cabinet,
            axes="xy",
            elem_a="cap",
            elem_b="button_deck",
            min_overlap=0.040,
            name=f"button {idx} is mounted within the button deck footprint",
        )

    rest_button_0 = ctx.part_world_position(button_parts[0])
    rest_button_1 = ctx.part_world_position(button_parts[1])
    with ctx.pose({button_joints[0]: 0.008}):
        pressed_button_0 = ctx.part_world_position(button_parts[0])
        still_button_1 = ctx.part_world_position(button_parts[1])
    ctx.check(
        "one play button depresses without dragging its neighbor",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and still_button_1 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.006
        and abs(still_button_1[2] - rest_button_1[2]) < 1e-6,
        details=(
            f"button0 rest={rest_button_0}, pressed={pressed_button_0}; "
            f"button1 rest={rest_button_1}, posed={still_button_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
