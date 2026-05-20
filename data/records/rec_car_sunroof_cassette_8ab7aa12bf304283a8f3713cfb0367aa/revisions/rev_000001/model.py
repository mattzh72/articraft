from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PANEL_L = 0.72
PANEL_W = 0.58
RAIL_Y = 0.31
SLIDE_TRAVEL = 0.33
TILT_UPPER = 0.16


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_slide_sunroof_cassette")

    model.material("black_e_coat", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("satin_rail", rgba=(0.46, 0.48, 0.50, 1.0))
    model.material("dark_glass", rgba=(0.11, 0.20, 0.25, 0.45))
    model.material("rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    model.material("zinc", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("acetal", rgba=(0.035, 0.038, 0.040, 1.0))

    cassette_frame = model.part("cassette_frame")
    # Rectangular stamped cassette tray with an open roof aperture.
    cassette_frame.visual(
        Box((1.46, 0.055, 0.035)),
        origin=Origin(xyz=(0.11, 0.41, 0.0175)),
        material="black_e_coat",
        name="side_frame_0",
    )
    cassette_frame.visual(
        Box((1.46, 0.055, 0.035)),
        origin=Origin(xyz=(0.11, -0.41, 0.0175)),
        material="black_e_coat",
        name="side_frame_1",
    )
    cassette_frame.visual(
        Box((0.08, 0.82, 0.035)),
        origin=Origin(xyz=(-0.62, 0.0, 0.0175)),
        material="black_e_coat",
        name="front_crossmember",
    )
    cassette_frame.visual(
        Box((0.08, 0.82, 0.035)),
        origin=Origin(xyz=(0.84, 0.0, 0.0175)),
        material="black_e_coat",
        name="rear_crossmember",
    )
    cassette_frame.visual(
        Box((1.42, 0.045, 0.030)),
        origin=Origin(xyz=(0.10, RAIL_Y, 0.050)),
        material="satin_rail",
        name="guide_rail_0",
    )
    cassette_frame.visual(
        Box((1.38, 0.010, 0.018)),
        origin=Origin(xyz=(0.10, RAIL_Y + 0.026, 0.073)),
        material="black_e_coat",
        name="rail_return_lip_0",
    )
    cassette_frame.visual(
        Box((1.42, 0.045, 0.030)),
        origin=Origin(xyz=(0.10, -RAIL_Y, 0.050)),
        material="satin_rail",
        name="guide_rail_1",
    )
    cassette_frame.visual(
        Box((1.38, 0.010, 0.018)),
        origin=Origin(xyz=(0.10, -RAIL_Y - 0.026, 0.073)),
        material="black_e_coat",
        name="rail_return_lip_1",
    )
    cassette_frame.visual(
        Box((1.40, 0.060, 0.024)),
        origin=Origin(xyz=(0.11, 0.0, 0.040)),
        material="black_e_coat",
        name="center_cable_housing",
    )
    cassette_frame.visual(
        Cylinder(radius=0.010, length=1.36),
        origin=Origin(xyz=(0.11, 0.0, 0.058), rpy=(0.0, pi / 2.0, 0.0)),
        material="zinc",
        name="drive_cable",
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        Box((0.040, 0.66, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material="zinc",
        name="front_tie_bar",
    )
    slide_carriage.visual(
        Box((0.040, 0.66, 0.018)),
        origin=Origin(xyz=(PANEL_L, 0.0, -0.016)),
        material="zinc",
        name="rear_tie_bar",
    )
    slide_carriage.visual(
        Box((0.085, 0.040, 0.025)),
        origin=Origin(xyz=(0.0, RAIL_Y, -0.0275)),
        material="acetal",
        name="front_shoe_0",
    )
    slide_carriage.visual(
        Cylinder(radius=0.016, length=0.038),
        origin=Origin(xyz=(0.0, RAIL_Y, -0.024), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="front_roller_0",
    )
    slide_carriage.visual(
        Box((0.085, 0.040, 0.025)),
        origin=Origin(xyz=(0.0, -RAIL_Y, -0.0275)),
        material="acetal",
        name="front_shoe_1",
    )
    slide_carriage.visual(
        Cylinder(radius=0.016, length=0.038),
        origin=Origin(xyz=(0.0, -RAIL_Y, -0.024), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="front_roller_1",
    )
    slide_carriage.visual(
        Box((0.085, 0.040, 0.025)),
        origin=Origin(xyz=(PANEL_L, RAIL_Y, -0.0275)),
        material="acetal",
        name="rear_shoe_0",
    )
    slide_carriage.visual(
        Cylinder(radius=0.016, length=0.038),
        origin=Origin(xyz=(PANEL_L, RAIL_Y, -0.024), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="rear_roller_0",
    )
    slide_carriage.visual(
        Box((0.085, 0.040, 0.025)),
        origin=Origin(xyz=(PANEL_L, -RAIL_Y, -0.0275)),
        material="acetal",
        name="rear_shoe_1",
    )
    slide_carriage.visual(
        Cylinder(radius=0.016, length=0.038),
        origin=Origin(xyz=(PANEL_L, -RAIL_Y, -0.024), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="rear_roller_1",
    )
    slide_carriage.visual(
        Box((PANEL_L, 0.035, 0.018)),
        origin=Origin(xyz=(PANEL_L / 2.0, 0.0, -0.016)),
        material="zinc",
        name="center_spine",
    )
    slide_carriage.visual(
        Box((0.13, 0.095, 0.032)),
        origin=Origin(xyz=(0.33, 0.0, -0.023)),
        material="black_e_coat",
        name="cable_clamp",
    )
    slide_carriage.visual(
        Cylinder(radius=0.006, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="front_hinge_bar",
    )
    for idx, y_pos in enumerate((0.24, -0.24)):
        slide_carriage.visual(
            Box((0.032, 0.044, 0.024)),
            origin=Origin(xyz=(0.500, y_pos, -0.041)),
            material="black_e_coat",
            name=f"pivot_stand_{idx}",
        )
        slide_carriage.visual(
            Box((0.208, 0.026, 0.016)),
            origin=Origin(xyz=(0.620, y_pos, -0.026)),
            material="black_e_coat",
            name=f"rear_lift_rail_{idx}",
        )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        Box((PANEL_L, PANEL_W, 0.010)),
        origin=Origin(xyz=(PANEL_L / 2.0, 0.0, 0.013)),
        material="dark_glass",
        name="glass_pane",
    )
    glass_panel.visual(
        Box((0.035, PANEL_W + 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="rubber",
        name="front_seal",
    )
    glass_panel.visual(
        Box((0.035, PANEL_W + 0.060, 0.018)),
        origin=Origin(xyz=(PANEL_L, 0.0, 0.009)),
        material="rubber",
        name="rear_seal",
    )
    for idx, y_pos in enumerate(((PANEL_W / 2.0) + 0.0175, -(PANEL_W / 2.0) - 0.0175)):
        glass_panel.visual(
            Box((PANEL_L, 0.035, 0.018)),
            origin=Origin(xyz=(PANEL_L / 2.0, y_pos, 0.009)),
            material="rubber",
            name=f"side_seal_{idx}",
        )
    glass_panel.visual(
        Box((0.11, 0.54, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, 0.012)),
        material="zinc",
        name="front_hinge_leaf",
    )
    glass_panel.visual(
        Box((0.13, 0.080, 0.012)),
        origin=Origin(xyz=(0.34, 0.0, 0.002)),
        material="black_e_coat",
        name="drive_cable_saddle",
    )

    for idx, y_pos in enumerate((0.24, -0.24)):
        arm = model.part(f"pivot_arm_{idx}")
        arm.visual(
            Cylinder(radius=0.012, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="zinc",
            name="lower_pivot_boss",
        )
        arm.visual(
            Box((0.185, 0.018, 0.010)),
            origin=Origin(xyz=(0.0925, 0.0, 0.008)),
            material="zinc",
            name="lift_link",
        )
        arm.visual(
            Box((0.026, 0.028, 0.020)),
            origin=Origin(xyz=(0.185, 0.0, 0.010)),
            material="acetal",
            name="panel_lift_pad",
        )

        model.articulation(
            f"pivot_arm_{idx}_tilt",
            ArticulationType.REVOLUTE,
            parent=slide_carriage,
            child=arm,
            origin=Origin(xyz=(0.500, y_pos, -0.017)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=TILT_UPPER, effort=8.0, velocity=1.0),
            mimic=Mimic(joint="panel_tilt", multiplier=1.0, offset=0.0),
        )

    model.articulation(
        "cassette_slide",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=slide_carriage,
        origin=Origin(xyz=(-0.35, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=180.0, velocity=0.35),
    )
    model.articulation(
        "panel_tilt",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=glass_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TILT_UPPER, effort=30.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    carriage = object_model.get_part("slide_carriage")
    panel = object_model.get_part("glass_panel")
    arm_0 = object_model.get_part("pivot_arm_0")
    slide = object_model.get_articulation("cassette_slide")
    tilt = object_model.get_articulation("panel_tilt")

    for seal_name in ("front_seal", "rear_seal", "side_seal_0", "side_seal_1"):
        ctx.check(
            f"{seal_name} is modeled",
            panel.get_visual(seal_name) is not None,
            details=f"missing visual {seal_name}",
        )

    with ctx.pose({slide: 0.0, tilt: 0.0}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="front_shoe_0",
            negative_elem="guide_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="front guide shoe sits on rail",
        )
        ctx.expect_within(
            carriage,
            frame,
            axes="y",
            inner_elem="front_shoe_0",
            outer_elem="guide_rail_0",
            margin=0.002,
            name="front guide shoe is captured laterally",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="rear_shoe_1",
            negative_elem="guide_rail_1",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear guide shoe sits on rail",
        )
        closed_panel_pos = ctx.part_world_position(panel)
        closed_rear = ctx.part_element_world_aabb(panel, elem="rear_seal")
        closed_pad = ctx.part_element_world_aabb(arm_0, elem="panel_lift_pad")

    with ctx.pose({slide: SLIDE_TRAVEL, tilt: 0.0}):
        slid_panel_pos = ctx.part_world_position(panel)
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="rear_shoe_0",
            elem_b="guide_rail_0",
            min_overlap=0.04,
            name="rear shoe remains engaged at full slide",
        )
        ctx.expect_within(
            carriage,
            frame,
            axes="y",
            inner_elem="rear_shoe_0",
            outer_elem="guide_rail_0",
            margin=0.002,
            name="slid shoe stays inside guide rail",
        )

    ctx.check(
        "glass panel slides backward on rails",
        closed_panel_pos is not None
        and slid_panel_pos is not None
        and slid_panel_pos[0] > closed_panel_pos[0] + 0.30,
        details=f"closed={closed_panel_pos}, slid={slid_panel_pos}",
    )

    with ctx.pose({slide: 0.0, tilt: TILT_UPPER}):
        tilted_rear = ctx.part_element_world_aabb(panel, elem="rear_seal")
        tilted_pad = ctx.part_element_world_aabb(arm_0, elem="panel_lift_pad")

    closed_rear_z = closed_rear[1][2] if closed_rear is not None else None
    tilted_rear_z = tilted_rear[1][2] if tilted_rear is not None else None
    ctx.check(
        "rear edge tilts upward",
        closed_rear_z is not None
        and tilted_rear_z is not None
        and tilted_rear_z > closed_rear_z + 0.06,
        details=f"closed rear z={closed_rear_z}, tilted rear z={tilted_rear_z}",
    )

    closed_pad_z = closed_pad[1][2] if closed_pad is not None else None
    tilted_pad_z = tilted_pad[1][2] if tilted_pad is not None else None
    ctx.check(
        "pivot arms lift with the tilt joint",
        closed_pad_z is not None
        and tilted_pad_z is not None
        and tilted_pad_z > closed_pad_z + 0.015,
        details=f"closed pad z={closed_pad_z}, tilted pad z={tilted_pad_z}",
    )

    return ctx.report()


object_model = build_object_model()
