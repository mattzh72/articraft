from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_PLINTH_RADIUS = 0.27
BASE_PLINTH_HEIGHT = 0.03
BASE_DRUM_RADIUS = 0.17
BASE_TOTAL_HEIGHT = 0.09

TURNTABLE_RADIUS = 0.23
TURNTABLE_THICKNESS = 0.04
TURNTABLE_COLLAR_WIDTH = 0.24
TURNTABLE_COLLAR_DEPTH = 0.16
TURNTABLE_COLLAR_HEIGHT = 0.07

MAST_WIDTH = 0.18
MAST_DEPTH = 0.12
MAST_HEIGHT = 0.81
MAST_BASE_Z = TURNTABLE_THICKNESS + TURNTABLE_COLLAR_HEIGHT

RAIL_OFFSET_X = 0.055
RAIL_CENTER_Y = 0.09
RAIL_SUPPORT_CENTER_Y = 0.075
RAIL_SUPPORT_WIDTH = 0.17
RAIL_SUPPORT_DEPTH = 0.03
RAIL_SUPPORT_HEIGHT = 0.05
LOWER_RAIL_SUPPORT_Z = 0.165
UPPER_RAIL_SUPPORT_Z = 0.765
GUIDE_ROD_RADIUS = 0.012
GUIDE_ROD_CLEAR_RADIUS = 0.013
GUIDE_ROD_Z0 = LOWER_RAIL_SUPPORT_Z + RAIL_SUPPORT_HEIGHT
GUIDE_ROD_LENGTH = UPPER_RAIL_SUPPORT_Z - GUIDE_ROD_Z0

WEAR_STRIP_WIDTH = 0.07
WEAR_STRIP_THICKNESS = 0.004
WEAR_STRIP_HEIGHT = 0.64
WEAR_STRIP_CENTER_Y = 0.062
WEAR_STRIP_CENTER_Z = 0.50

CARRIAGE_BLOCK_WIDTH = 0.05
CARRIAGE_BLOCK_DEPTH = 0.05
CARRIAGE_BLOCK_HEIGHT = 0.12
CARRIAGE_BRIDGE_WIDTH = 0.17
CARRIAGE_BRIDGE_DEPTH = 0.016
CARRIAGE_BRIDGE_HEIGHT = 0.18
CARRIAGE_SUPPORT_WIDTH = 0.09
CARRIAGE_SUPPORT_DEPTH = 0.08
CARRIAGE_SUPPORT_HEIGHT = 0.123

BACK_PAD_WIDTH = 0.09
BACK_PAD_THICKNESS = 0.004
BACK_PAD_HEIGHT = 0.18
BACK_PAD_ORIGIN = (0.0, -0.027, 0.02)

STAGE_WIDTH = 0.20
STAGE_DEPTH = 0.13
STAGE_THICKNESS = 0.018
STAGE_ORIGIN = (0.0, 0.10, 0.072)

CARRIAGE_JOINT_ORIGIN = (0.0, RAIL_CENTER_Y, 0.28)
LIFT_TRAVEL = 0.42


def make_stationary_base() -> cq.Workplane:
    plinth = cq.Workplane("XY").circle(BASE_PLINTH_RADIUS).extrude(BASE_PLINTH_HEIGHT)
    bearing_drum = cq.Workplane("XY").circle(BASE_DRUM_RADIUS).extrude(BASE_TOTAL_HEIGHT)
    return plinth.union(bearing_drum)


def make_turntable() -> cq.Workplane:
    disk = cq.Workplane("XY").circle(TURNTABLE_RADIUS).extrude(TURNTABLE_THICKNESS)
    collar = (
        cq.Workplane("XY")
        .box(
            TURNTABLE_COLLAR_WIDTH,
            TURNTABLE_COLLAR_DEPTH,
            TURNTABLE_COLLAR_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, TURNTABLE_THICKNESS))
    )
    return disk.union(collar)


def make_mast() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(MAST_WIDTH, MAST_DEPTH, MAST_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, MAST_BASE_Z))
    )


def make_guide_rails() -> cq.Workplane:
    lower_support = (
        cq.Workplane("XY")
        .box(
            RAIL_SUPPORT_WIDTH,
            RAIL_SUPPORT_DEPTH,
            RAIL_SUPPORT_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, RAIL_SUPPORT_CENTER_Y, LOWER_RAIL_SUPPORT_Z))
    )
    upper_support = (
        cq.Workplane("XY")
        .box(
            RAIL_SUPPORT_WIDTH,
            RAIL_SUPPORT_DEPTH,
            RAIL_SUPPORT_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, RAIL_SUPPORT_CENTER_Y, UPPER_RAIL_SUPPORT_Z))
    )
    rods = (
        cq.Workplane("XY")
        .pushPoints([(-RAIL_OFFSET_X, RAIL_CENTER_Y), (RAIL_OFFSET_X, RAIL_CENTER_Y)])
        .circle(GUIDE_ROD_RADIUS)
        .extrude(GUIDE_ROD_LENGTH)
        .translate((0.0, 0.0, GUIDE_ROD_Z0))
    )
    return lower_support.union(upper_support).union(rods)


def make_carriage_body() -> cq.Workplane:
    left_block = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_BLOCK_WIDTH,
            CARRIAGE_BLOCK_DEPTH,
            CARRIAGE_BLOCK_HEIGHT,
            centered=(True, True, True),
        )
        .translate((-RAIL_OFFSET_X, 0.0, 0.0))
    )
    right_block = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_BLOCK_WIDTH,
            CARRIAGE_BLOCK_DEPTH,
            CARRIAGE_BLOCK_HEIGHT,
            centered=(True, True, True),
        )
        .translate((RAIL_OFFSET_X, 0.0, 0.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_BRIDGE_WIDTH,
            CARRIAGE_BRIDGE_DEPTH,
            CARRIAGE_BRIDGE_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, 0.045, 0.0))
    )
    support = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_SUPPORT_WIDTH,
            CARRIAGE_SUPPORT_DEPTH,
            CARRIAGE_SUPPORT_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, 0.07, 0.0015))
    )
    carriage = left_block.union(right_block).union(bridge).union(support)
    rod_cuts = (
        cq.Workplane("XY")
        .pushPoints([(-RAIL_OFFSET_X, 0.0), (RAIL_OFFSET_X, 0.0)])
        .circle(GUIDE_ROD_CLEAR_RADIUS)
        .extrude(CARRIAGE_BLOCK_HEIGHT + 0.04, both=True)
    )
    return carriage.cut(rod_cuts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_indexer")

    dark_gray = model.material("dark_gray", color=(0.20, 0.21, 0.23))
    base_gray = model.material("base_gray", color=(0.33, 0.35, 0.38))
    rail_steel = model.material("rail_steel", color=(0.78, 0.80, 0.83))
    wear_black = model.material("wear_black", color=(0.08, 0.09, 0.10))
    carriage_gray = model.material("carriage_gray", color=(0.56, 0.58, 0.61))
    stage_aluminum = model.material("stage_aluminum", color=(0.86, 0.87, 0.89))

    stationary_base = model.part("stationary_base")
    stationary_base.visual(
        mesh_from_cadquery(make_stationary_base(), "stationary_base_shell"),
        material=base_gray,
        name="base_shell",
    )

    rotary_assembly = model.part("rotary_assembly")
    rotary_assembly.visual(
        mesh_from_cadquery(make_turntable(), "turntable_cluster"),
        material=dark_gray,
        name="turntable",
    )
    rotary_assembly.visual(
        mesh_from_cadquery(make_mast(), "mast_body"),
        material=dark_gray,
        name="mast_body",
    )
    rotary_assembly.visual(
        mesh_from_cadquery(make_guide_rails(), "guide_rails"),
        material=rail_steel,
        name="guide_rails",
    )
    rotary_assembly.visual(
        Box((WEAR_STRIP_WIDTH, WEAR_STRIP_THICKNESS, WEAR_STRIP_HEIGHT)),
        origin=Origin(xyz=(0.0, WEAR_STRIP_CENTER_Y, WEAR_STRIP_CENTER_Z)),
        material=wear_black,
        name="wear_strip",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_body(), "carriage_body"),
        material=carriage_gray,
        name="carriage_body",
    )
    carriage.visual(
        Box((BACK_PAD_WIDTH, BACK_PAD_THICKNESS, BACK_PAD_HEIGHT)),
        origin=Origin(xyz=BACK_PAD_ORIGIN),
        material=wear_black,
        name="back_pad",
    )
    carriage.visual(
        Box((STAGE_WIDTH, STAGE_DEPTH, STAGE_THICKNESS)),
        origin=Origin(xyz=STAGE_ORIGIN),
        material=stage_aluminum,
        name="stage_plate",
    )

    model.articulation(
        "pedestal_rotation",
        ArticulationType.REVOLUTE,
        parent=stationary_base,
        child=rotary_assembly,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=rotary_assembly,
        child=carriage,
        origin=Origin(xyz=CARRIAGE_JOINT_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.25,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stationary_base = object_model.get_part("stationary_base")
    rotary_assembly = object_model.get_part("rotary_assembly")
    carriage = object_model.get_part("carriage")
    pedestal_rotation = object_model.get_articulation("pedestal_rotation")
    carriage_lift = object_model.get_articulation("carriage_lift")

    mast_body = rotary_assembly.get_visual("mast_body")
    turntable = rotary_assembly.get_visual("turntable")
    wear_strip = rotary_assembly.get_visual("wear_strip")
    back_pad = carriage.get_visual("back_pad")
    stage_plate = carriage.get_visual("stage_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "pedestal_revolute_axis_is_vertical",
        pedestal_rotation.axis == (0.0, 0.0, 1.0),
        f"axis={pedestal_rotation.axis}",
    )
    ctx.check(
        "pedestal_revolute_has_indexing_range",
        pedestal_rotation.motion_limits is not None
        and pedestal_rotation.motion_limits.lower is not None
        and pedestal_rotation.motion_limits.upper is not None
        and pedestal_rotation.motion_limits.lower <= -3.0
        and pedestal_rotation.motion_limits.upper >= 3.0,
        f"limits={pedestal_rotation.motion_limits}",
    )
    ctx.check(
        "carriage_prismatic_axis_is_vertical",
        carriage_lift.axis == (0.0, 0.0, 1.0),
        f"axis={carriage_lift.axis}",
    )
    ctx.check(
        "carriage_has_real_vertical_travel",
        carriage_lift.motion_limits is not None
        and carriage_lift.motion_limits.lower == 0.0
        and carriage_lift.motion_limits.upper is not None
        and carriage_lift.motion_limits.upper >= 0.40,
        f"limits={carriage_lift.motion_limits}",
    )

    ctx.expect_contact(
        rotary_assembly,
        stationary_base,
        contact_tol=0.001,
        name="turntable_bearing_is_seated",
    )
    ctx.expect_contact(
        carriage,
        rotary_assembly,
        elem_a=back_pad,
        elem_b=wear_strip,
        contact_tol=1e-6,
        name="carriage_tracks_on_wear_strip",
    )

    with ctx.pose({carriage_lift: 0.0}):
        ctx.expect_gap(
            carriage,
            rotary_assembly,
            axis="y",
            positive_elem=stage_plate,
            negative_elem=mast_body,
            min_gap=0.05,
            max_gap=0.08,
            name="lift_stage_stands_proud_of_mast",
        )
        ctx.expect_gap(
            carriage,
            rotary_assembly,
            axis="z",
            positive_elem=stage_plate,
            negative_elem=turntable,
            min_gap=0.22,
            max_gap=0.26,
            name="low_pose_stage_clears_turntable",
        )

    with ctx.pose({carriage_lift: LIFT_TRAVEL}):
        ctx.expect_gap(
            carriage,
            rotary_assembly,
            axis="z",
            positive_elem=stage_plate,
            negative_elem=turntable,
            min_gap=0.64,
            max_gap=0.68,
            name="high_pose_stage_lifts_well_above_turntable",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
