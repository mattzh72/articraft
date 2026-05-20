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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_deli_display_freezer")

    white = model.material("warm_white_insulation", rgba=(0.86, 0.88, 0.85, 1.0))
    liner = model.material("dark_food_well", rgba=(0.05, 0.06, 0.065, 1.0))
    black = model.material("black_rubber_trim", rgba=(0.015, 0.016, 0.018, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.65, 0.66, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.78, 0.95, 0.36))
    wire = model.material("zinc_wire_basket", rgba=(0.72, 0.74, 0.70, 1.0))
    label = model.material("thermostat_label", rgba=(0.93, 0.93, 0.88, 1.0))
    dial_mat = model.material("black_thermostat_dial", rgba=(0.02, 0.022, 0.024, 1.0))
    package_mat = model.material("muted_package_colors", rgba=(0.50, 0.66, 0.80, 1.0))

    cabinet = model.part("cabinet")

    # Short, broad insulated tub: a real open well rather than a solid block.
    cabinet.visual(
        Box((1.25, 0.75, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=white,
        name="bottom_insulation",
    )
    cabinet.visual(
        Box((1.25, 0.07, 0.51)),
        origin=Origin(xyz=(0.0, -0.34, 0.335)),
        material=white,
        name="front_wall",
    )
    cabinet.visual(
        Box((1.25, 0.07, 0.51)),
        origin=Origin(xyz=(0.0, 0.34, 0.335)),
        material=white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((0.075, 0.75, 0.51)),
        origin=Origin(xyz=(-0.5875, 0.0, 0.335)),
        material=white,
        name="end_wall_0",
    )
    cabinet.visual(
        Box((0.075, 0.75, 0.51)),
        origin=Origin(xyz=(0.5875, 0.0, 0.335)),
        material=white,
        name="end_wall_1",
    )
    cabinet.visual(
        Box((1.10, 0.61, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material=liner,
        name="recessed_well_floor",
    )
    cabinet.visual(
        Box((1.08, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, -0.293, 0.461)),
        material=aluminum,
        name="front_basket_ledge",
    )
    cabinet.visual(
        Box((1.08, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.293, 0.461)),
        material=aluminum,
        name="rear_basket_ledge",
    )

    # Raised top frame and side guide tracks for the sliding glass lid.
    cabinet.visual(
        Box((1.25, 0.085, 0.065)),
        origin=Origin(xyz=(0.0, -0.3325, 0.6075)),
        material=black,
        name="top_front_frame",
    )
    cabinet.visual(
        Box((1.25, 0.085, 0.065)),
        origin=Origin(xyz=(0.0, 0.3325, 0.6075)),
        material=black,
        name="top_rear_frame",
    )
    cabinet.visual(
        Box((0.09, 0.75, 0.065)),
        origin=Origin(xyz=(-0.58, 0.0, 0.6075)),
        material=black,
        name="top_end_frame_0",
    )
    cabinet.visual(
        Box((0.09, 0.75, 0.065)),
        origin=Origin(xyz=(0.58, 0.0, 0.6075)),
        material=black,
        name="top_end_frame_1",
    )
    cabinet.visual(
        Box((1.15, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, -0.300, 0.637)),
        material=aluminum,
        name="front_guide_shelf",
    )
    cabinet.visual(
        Box((1.15, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, 0.300, 0.637)),
        material=aluminum,
        name="rear_guide_shelf",
    )
    cabinet.visual(
        Box((1.15, 0.018, 0.056)),
        origin=Origin(xyz=(0.0, -0.332, 0.665)),
        material=aluminum,
        name="front_guide_lip",
    )
    cabinet.visual(
        Box((1.15, 0.018, 0.056)),
        origin=Origin(xyz=(0.0, 0.332, 0.665)),
        material=aluminum,
        name="rear_guide_lip",
    )
    cabinet.visual(
        Box((0.48, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.378, 0.370)),
        material=label,
        name="control_badge_top",
    )
    cabinet.visual(
        Box((0.48, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.378, 0.250)),
        material=label,
        name="control_badge_bottom",
    )
    cabinet.visual(
        Box((0.025, 0.006, 0.105)),
        origin=Origin(xyz=(-0.115, -0.378, 0.310)),
        material=label,
        name="control_badge_side_0",
    )
    cabinet.visual(
        Box((0.025, 0.006, 0.105)),
        origin=Origin(xyz=(0.115, -0.378, 0.310)),
        material=label,
        name="control_badge_side_1",
    )
    cabinet.visual(
        Box((1.18, 0.026, 0.065)),
        origin=Origin(xyz=(0.0, -0.388, 0.085)),
        material=black,
        name="front_toe_kick",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.76, 0.49, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.845, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, -0.268, 0.0)),
        material=aluminum,
        name="front_lid_frame",
    )
    lid.visual(
        Box((0.845, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.268, 0.0)),
        material=aluminum,
        name="rear_lid_frame",
    )
    lid.visual(
        Box((0.045, 0.59, 0.024)),
        origin=Origin(xyz=(-0.400, 0.0, 0.0)),
        material=aluminum,
        name="end_lid_frame_0",
    )
    lid.visual(
        Box((0.045, 0.59, 0.024)),
        origin=Origin(xyz=(0.400, 0.0, 0.0)),
        material=aluminum,
        name="end_lid_frame_1",
    )
    lid.visual(
        Box((0.80, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.300, -0.017)),
        material=aluminum,
        name="front_runner",
    )
    lid.visual(
        Box((0.80, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.300, -0.017)),
        material=aluminum,
        name="rear_runner",
    )
    lid.visual(
        Box((0.040, 0.54, 0.040)),
        origin=Origin(xyz=(-0.440, 0.0, 0.018)),
        material=black,
        name="pull_rail",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(-0.10, 0.0, 0.672)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.28),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="shaft",
    )
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.078,
            0.026,
            body_style="skirted",
            top_diameter=0.060,
            edge_radius=0.0015,
            grip=KnobGrip(style="fluted", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        ),
        "thermostat_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_mat,
        name="dial_cap",
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(0.0, -0.375, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-2.35, upper=2.35),
    )

    for index, x_pos in enumerate((-0.285, 0.285)):
        basket = model.part(f"basket_{index}")
        basket.visual(
            Box((0.49, 0.035, 0.014)),
            origin=Origin(xyz=(0.0, -0.265, 0.045)),
            material=wire,
            name="front_rim",
        )
        basket.visual(
            Box((0.49, 0.035, 0.014)),
            origin=Origin(xyz=(0.0, 0.265, 0.045)),
            material=wire,
            name="rear_rim",
        )
        basket.visual(
            Box((0.014, 0.56, 0.014)),
            origin=Origin(xyz=(-0.245, 0.0, 0.045)),
            material=wire,
            name="side_rim_0",
        )
        basket.visual(
            Box((0.014, 0.56, 0.014)),
            origin=Origin(xyz=(0.245, 0.0, 0.045)),
            material=wire,
            name="side_rim_1",
        )
        basket.visual(
            Box((0.47, 0.50, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=wire,
            name="wire_floor_mat",
        )
        basket.visual(
            Box((0.36, 0.30, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=package_mat,
            name="product_load",
        )
        for sx in (-0.225, 0.225):
            for sy in (-0.245, 0.245):
                basket.visual(
                    Box((0.014, 0.014, 0.115)),
                    origin=Origin(xyz=(sx, sy, -0.010)),
                    material=wire,
                    name=f"corner_post_{sx}_{sy}",
                )
        model.articulation(
            f"cabinet_to_basket_{index}",
            ArticulationType.FIXED,
            parent=cabinet,
            child=basket,
            origin=Origin(xyz=(x_pos, 0.0, 0.435)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_slide = object_model.get_articulation("cabinet_to_lid")
    dial_turn = object_model.get_articulation("cabinet_to_dial")

    with ctx.pose({lid_slide: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="front_runner",
            negative_elem="front_guide_shelf",
            max_gap=0.001,
            max_penetration=0.0,
            name="front runner sits on its guide",
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="rear_runner",
            negative_elem="rear_guide_shelf",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear runner sits on its guide",
        )
        rest_position = ctx.part_world_position(lid)

    with ctx.pose({lid_slide: 0.28}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="front_runner",
            negative_elem="front_guide_shelf",
            max_gap=0.001,
            max_penetration=0.0,
            name="front runner stays on guide when slid",
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="rear_runner",
            negative_elem="rear_guide_shelf",
            max_gap=0.001,
            max_penetration=0.0,
            name="rear runner stays on guide when slid",
        )
        extended_position = ctx.part_world_position(lid)

    ctx.check(
        "lid translates along the top guides",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.25,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    ctx.expect_contact(
        dial,
        cabinet,
        elem_a="shaft",
        elem_b="front_wall",
        contact_tol=0.001,
        name="thermostat shaft enters the front face",
    )
    ctx.check(
        "thermostat dial has limited rotary travel",
        dial_turn.motion_limits is not None
        and dial_turn.motion_limits.lower is not None
        and dial_turn.motion_limits.upper is not None
        and 4.0 < dial_turn.motion_limits.upper - dial_turn.motion_limits.lower < 5.2,
        details=f"limits={dial_turn.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
