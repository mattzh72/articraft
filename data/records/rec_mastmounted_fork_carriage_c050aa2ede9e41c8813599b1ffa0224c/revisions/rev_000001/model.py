from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


MAST_HEIGHT = 1.55
TRAVEL = 0.45
CARRIAGE_HOME_Z = 0.36
RAIL_Y = 0.275
RAIL_SPACING = RAIL_Y * 2.0
CHANNEL_WIDTH = 0.110
CHANNEL_GAP_HALF = 0.039


def _box_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z))


def _fork_tip_geometry(length: float, width: float, height: float, tip_height: float) -> MeshGeometry:
    """A simple tapered pallet-fork nose with a flat lower face."""
    geom = MeshGeometry()
    verts = [
        (0.0, -width / 2.0, -height / 2.0),
        (0.0, width / 2.0, -height / 2.0),
        (0.0, width / 2.0, height / 2.0),
        (0.0, -width / 2.0, height / 2.0),
        (length, -width / 2.0, -height / 2.0),
        (length, width / 2.0, -height / 2.0),
        (length, width / 2.0, -height / 2.0 + tip_height),
        (length, -width / 2.0, -height / 2.0 + tip_height),
    ]
    for vertex in verts:
        geom.add_vertex(*vertex)
    for face in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (0, 3, 7),
        (0, 7, 4),
        (1, 5, 6),
        (1, 6, 2),
    ):
        geom.add_face(*face)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_fork_carriage_module")

    mast_paint = model.material("dark_mast_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    carriage_paint = model.material("safety_yellow_carriage", rgba=(0.92, 0.58, 0.08, 1.0))
    fork_steel = model.material("dark_fork_steel", rgba=(0.12, 0.12, 0.12, 1.0))
    wear_steel = model.material("bright_wear_steel", rgba=(0.65, 0.68, 0.66, 1.0))
    rubber = model.material("black_roller_rubber", rgba=(0.015, 0.015, 0.018, 1.0))

    mast = model.part("mast")

    web_x = -0.035
    web_thick = 0.026
    flange_x = -0.006
    flange_depth = 0.084
    flange_thick = 0.016
    flange_offset = CHANNEL_WIDTH / 2.0 - flange_thick / 2.0

    mast.visual(
        Box((web_thick, CHANNEL_WIDTH, MAST_HEIGHT)),
        origin=_box_origin(web_x, RAIL_Y, MAST_HEIGHT / 2.0),
        material=mast_paint,
        name="rail_0_web",
    )
    mast.visual(
        Box((flange_depth, flange_thick, MAST_HEIGHT)),
        origin=_box_origin(flange_x, RAIL_Y - flange_offset, MAST_HEIGHT / 2.0),
        material=mast_paint,
        name="rail_0_inner_flange",
    )
    mast.visual(
        Box((flange_depth, flange_thick, MAST_HEIGHT)),
        origin=_box_origin(flange_x, RAIL_Y + flange_offset, MAST_HEIGHT / 2.0),
        material=mast_paint,
        name="rail_0_outer_flange",
    )
    mast.visual(
        Box((0.035, 0.040, 0.080)),
        origin=_box_origin(0.045, RAIL_Y + flange_offset, 1.42),
        material=wear_steel,
        name="rail_0_upper_stop",
    )
    mast.visual(
        Box((web_thick, CHANNEL_WIDTH, MAST_HEIGHT)),
        origin=_box_origin(web_x, -RAIL_Y, MAST_HEIGHT / 2.0),
        material=mast_paint,
        name="rail_1_web",
    )
    mast.visual(
        Box((flange_depth, flange_thick, MAST_HEIGHT)),
        origin=_box_origin(flange_x, -RAIL_Y + flange_offset, MAST_HEIGHT / 2.0),
        material=mast_paint,
        name="rail_1_inner_flange",
    )
    mast.visual(
        Box((flange_depth, flange_thick, MAST_HEIGHT)),
        origin=_box_origin(flange_x, -RAIL_Y - flange_offset, MAST_HEIGHT / 2.0),
        material=mast_paint,
        name="rail_1_outer_flange",
    )
    mast.visual(
        Box((0.035, 0.040, 0.080)),
        origin=_box_origin(0.045, -RAIL_Y - flange_offset, 1.42),
        material=wear_steel,
        name="rail_1_upper_stop",
    )

    mast.visual(
        Box((0.125, RAIL_SPACING + 0.180, 0.070)),
        origin=_box_origin(-0.015, 0.0, 0.035),
        material=mast_paint,
        name="bottom_sill",
    )
    mast.visual(
        Box((0.118, RAIL_SPACING + 0.150, 0.085)),
        origin=_box_origin(-0.018, 0.0, MAST_HEIGHT + 0.0375),
        material=mast_paint,
        name="top_crosshead",
    )
    mast.visual(
        Box((0.036, RAIL_SPACING + 0.115, 0.030)),
        origin=_box_origin(0.034, 0.0, MAST_HEIGHT + 0.005),
        material=wear_steel,
        name="crosshead_front_wear_plate",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.050, 0.600, 0.300)),
        origin=_box_origin(0.105, 0.0, 0.000),
        material=carriage_paint,
        name="carriage_back_plate",
    )
    carriage.visual(
        Box((0.075, 0.660, 0.060)),
        origin=_box_origin(0.105, 0.0, 0.170),
        material=carriage_paint,
        name="carriage_top_bar",
    )
    carriage.visual(
        Box((0.075, 0.660, 0.060)),
        origin=_box_origin(0.105, 0.0, -0.180),
        material=carriage_paint,
        name="carriage_bottom_bar",
    )
    for idx, y in enumerate((0.300, -0.300)):
        carriage.visual(
            Box((0.060, 0.035, 0.440)),
            origin=_box_origin(0.138, y, -0.010),
            material=carriage_paint,
            name=f"side_rib_{idx}",
        )
    for idx, y in enumerate((-0.090, 0.090)):
        carriage.visual(
            Box((0.035, 0.030, 0.350)),
            origin=_box_origin(0.145, y, -0.010),
            material=carriage_paint,
            name=f"center_rib_{idx}",
        )
    for idx, y in enumerate((-0.180, 0.180)):
        carriage.visual(
            Box((0.016, 0.150, 0.320)),
            origin=_box_origin(0.137, y, 0.010),
            material=wear_steel,
            name=f"wear_plate_{idx}",
        )

    fork_tip = _fork_tip_geometry(0.120, 0.095, 0.055, 0.012)
    carriage.visual(
        Box((0.620, 0.095, 0.055)),
        origin=_box_origin(0.450, -0.180, -0.245),
        material=fork_steel,
        name="fork_0_tine",
    )
    carriage.visual(
        mesh_from_geometry(fork_tip, "fork_0_tapered_tip"),
        origin=_box_origin(0.758, -0.180, -0.245),
        material=fork_steel,
        name="fork_0_tip",
    )
    carriage.visual(
        Box((0.105, 0.120, 0.260)),
        origin=_box_origin(0.155, -0.180, -0.110),
        material=fork_steel,
        name="fork_0_heel_tab",
    )
    carriage.visual(
        Box((0.050, 0.145, 0.050)),
        origin=_box_origin(0.145, -0.180, 0.055),
        material=wear_steel,
        name="fork_0_hook_pad",
    )
    carriage.visual(
        Box((0.620, 0.095, 0.055)),
        origin=_box_origin(0.450, 0.180, -0.245),
        material=fork_steel,
        name="fork_1_tine",
    )
    carriage.visual(
        mesh_from_geometry(fork_tip, "fork_1_tapered_tip"),
        origin=_box_origin(0.758, 0.180, -0.245),
        material=fork_steel,
        name="fork_1_tip",
    )
    carriage.visual(
        Box((0.105, 0.120, 0.260)),
        origin=_box_origin(0.155, 0.180, -0.110),
        material=fork_steel,
        name="fork_1_heel_tab",
    )
    carriage.visual(
        Box((0.050, 0.145, 0.050)),
        origin=_box_origin(0.145, 0.180, 0.055),
        material=wear_steel,
        name="fork_1_hook_pad",
    )

    # Load backrest: an open grid tied into the carriage top bar rather than a loose screen.
    for idx, y in enumerate((-0.270, 0.270)):
        carriage.visual(
            Box((0.045, 0.035, 0.550)),
            origin=_box_origin(0.170, y, 0.410),
            material=carriage_paint,
            name=f"backrest_post_{idx}",
        )
    carriage.visual(
        Box((0.045, 0.585, 0.035)),
        origin=_box_origin(0.170, 0.0, 0.250),
        material=carriage_paint,
        name="backrest_lower_bar",
    )
    carriage.visual(
        Box((0.045, 0.585, 0.035)),
        origin=_box_origin(0.170, 0.0, 0.480),
        material=carriage_paint,
        name="backrest_middle_bar",
    )
    carriage.visual(
        Box((0.045, 0.585, 0.045)),
        origin=_box_origin(0.170, 0.0, 0.680),
        material=carriage_paint,
        name="backrest_top_bar",
    )
    for idx, y in enumerate((-0.135, 0.0, 0.135)):
        carriage.visual(
            Box((0.030, 0.024, 0.460)),
            origin=_box_origin(0.178, y, 0.450),
            material=carriage_paint,
            name=f"backrest_slat_{idx}",
        )

    # Four captured guide shoes ride in the voids of the two C-channels.
    for rail_idx, rail_y in enumerate((RAIL_Y, -RAIL_Y)):
        for shoe_idx, z in enumerate((-0.140, 0.150)):
            shoe_name = f"guide_shoe_{rail_idx}_{shoe_idx}"
            carriage.visual(
                Box((0.039, 0.045, 0.075)),
                origin=_box_origin(-0.0025, rail_y, z),
                material=wear_steel,
                name=shoe_name,
            )
            carriage.visual(
                Box((0.080, 0.025, 0.030)),
                origin=_box_origin(0.055, rail_y, z),
                material=carriage_paint,
                name=f"guide_arm_{rail_idx}_{shoe_idx}",
            )
            carriage.visual(
                Cylinder(radius=0.024, length=0.035),
                origin=Origin(xyz=(0.049, rail_y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name=f"guide_roller_{rail_idx}_{shoe_idx}",
            )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.35, lower=0.0, upper=TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    joint = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single_vertical_prismatic_joint",
        len(object_model.articulations) == 1
        and joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"joint_count={len(object_model.articulations)}, type={joint.articulation_type}, axis={joint.axis}",
    )

    for shoe in ("guide_shoe_0_0", "guide_shoe_0_1"):
        ctx.expect_gap(
            carriage,
            mast,
            axis="x",
            positive_elem=shoe,
            negative_elem="rail_0_web",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{shoe}_slides_on_web",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem=shoe,
            negative_elem="rail_0_inner_flange",
            min_gap=0.006,
            max_gap=0.030,
            name=f"{shoe}_clears_inner_flange",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            positive_elem="rail_0_outer_flange",
            negative_elem=shoe,
            min_gap=0.006,
            max_gap=0.030,
            name=f"{shoe}_clears_outer_flange",
        )
    for shoe in ("guide_shoe_1_0", "guide_shoe_1_1"):
        ctx.expect_gap(
            carriage,
            mast,
            axis="x",
            positive_elem=shoe,
            negative_elem="rail_1_web",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{shoe}_slides_on_web",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem=shoe,
            negative_elem="rail_1_outer_flange",
            min_gap=0.006,
            max_gap=0.030,
            name=f"{shoe}_clears_outer_flange",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            positive_elem="rail_1_inner_flange",
            negative_elem=shoe,
            min_gap=0.006,
            max_gap=0.030,
            name=f"{shoe}_clears_inner_flange",
        )

    ctx.expect_gap(
        carriage,
        mast,
        axis="z",
        positive_elem="fork_0_tine",
        negative_elem="bottom_sill",
        min_gap=0.010,
        name="lower_fork_clears_bottom_sill",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="z",
        positive_elem="fork_1_tine",
        negative_elem="bottom_sill",
        min_gap=0.010,
        name="upper_fork_clears_bottom_sill",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({joint: TRAVEL}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="top_crosshead",
            negative_elem="backrest_top_bar",
            min_gap=0.030,
            name="raised_backrest_clears_top_crosshead",
        )
        for rail_idx in (0, 1):
            for shoe_idx in (0, 1):
                ctx.expect_within(
                    carriage,
                    mast,
                    axes="z",
                    inner_elem=f"guide_shoe_{rail_idx}_{shoe_idx}",
                    outer_elem=f"rail_{rail_idx}_web",
                    margin=0.0,
                    name=f"guide_shoe_{rail_idx}_{shoe_idx}_retained_at_top",
                )

    ctx.check(
        "carriage_moves_upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + TRAVEL - 0.010,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
