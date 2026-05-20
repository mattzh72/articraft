from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_SIZE = 0.54
BASE_THICK = 0.05
PEDESTAL_RADIUS = 0.13
PEDESTAL_HEIGHT = 0.10
POST_RADIUS = 0.085
MAIN_POST_HEIGHT = 1.28
SUPPORT_FLANGE_RADIUS = 0.155
SUPPORT_FLANGE_THICKNESS = 0.035
SUPPORT_PLANE_Z = BASE_THICK + MAIN_POST_HEIGHT

COLLAR_INNER_RADIUS = 0.098
COLLAR_OUTER_RADIUS = 0.168
COLLAR_HEIGHT = 0.20
BEARING_RING_THICKNESS = 0.016

BEAM_START_X = 0.16
BEAM_LENGTH = 1.18
BEAM_WIDTH = 0.14
BEAM_HEIGHT = 0.18
BEAM_CENTER_Z = 0.11
GUSSET_THICKNESS = 0.014

TRUCK_HOME_X = 0.43
TRUCK_TRAVEL = 0.58
TRUCK_LENGTH = 0.30
TRUCK_DECK_WIDTH = 0.22
TRUCK_DECK_THICK = 0.04
TRUCK_RAIL_BLOCK_WIDTH = 0.04
TRUCK_RAIL_BLOCK_HEIGHT = 0.07
TRUCK_HOUSING_LENGTH = 0.22
TRUCK_HOUSING_WIDTH = 0.22
TRUCK_HOUSING_HEIGHT = 0.14
TRUCK_HOUSING_CENTER_Z = 0.11
TRUCK_MOTOR_LENGTH = 0.12
TRUCK_MOTOR_WIDTH = 0.10
TRUCK_MOTOR_HEIGHT = 0.08
TRUCK_MOTOR_CENTER = (-0.06, 0.0, 0.20)
TRUCK_CONTROL_BOX_LENGTH = 0.11
TRUCK_CONTROL_BOX_WIDTH = 0.14
TRUCK_CONTROL_BOX_HEIGHT = 0.06
TRUCK_CONTROL_BOX_CENTER = (0.07, 0.0, 0.21)


def _build_column_shape() -> cq.Workplane:
    base = cq.Workplane("XY").rect(BASE_SIZE, BASE_SIZE).extrude(BASE_THICK)
    pedestal = cq.Workplane("XY").circle(PEDESTAL_RADIUS).extrude(PEDESTAL_HEIGHT).translate(
        (0.0, 0.0, BASE_THICK)
    )
    post = cq.Workplane("XY").circle(POST_RADIUS).extrude(MAIN_POST_HEIGHT).translate(
        (0.0, 0.0, BASE_THICK)
    )
    support_flange = (
        cq.Workplane("XY")
        .circle(SUPPORT_FLANGE_RADIUS)
        .extrude(SUPPORT_FLANGE_THICKNESS)
        .translate((0.0, 0.0, SUPPORT_PLANE_Z - SUPPORT_FLANGE_THICKNESS))
    )
    return base.union(pedestal).union(post).union(support_flange)


def _build_arm_shape() -> cq.Workplane:
    bearing_ring = (
        cq.Workplane("XY")
        .circle(SUPPORT_FLANGE_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(BEARING_RING_THICKNESS)
    )
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT - BEARING_RING_THICKNESS)
        .translate((0.0, 0.0, BEARING_RING_THICKNESS))
    )

    beam = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT).translate(
        (BEAM_START_X + 0.5 * BEAM_LENGTH, 0.0, BEAM_CENTER_Z)
    )

    gusset_profile = [(0.03, 0.02), (0.38, 0.02), (0.11, 0.19)]
    side_gusset = cq.Workplane("XZ").polyline(gusset_profile).close().extrude(GUSSET_THICKNESS)
    gusset_pos = side_gusset.translate((0.0, 0.5 * BEAM_WIDTH - 0.5 * GUSSET_THICKNESS, 0.0))
    gusset_neg = side_gusset.translate(
        (0.0, -0.5 * BEAM_WIDTH - 0.5 * GUSSET_THICKNESS, 0.0)
    )

    return bearing_ring.union(collar).union(beam).union(gusset_pos).union(gusset_neg)


def _build_truck_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(TRUCK_LENGTH, TRUCK_DECK_WIDTH, TRUCK_DECK_THICK).translate(
        (0.0, 0.0, 0.5 * TRUCK_DECK_THICK)
    )
    rail_block_offset_y = 0.5 * (TRUCK_DECK_WIDTH - TRUCK_RAIL_BLOCK_WIDTH)
    rail_block_z = TRUCK_DECK_THICK + 0.5 * TRUCK_RAIL_BLOCK_HEIGHT
    left_rail_block = cq.Workplane("XY").box(
        0.24, TRUCK_RAIL_BLOCK_WIDTH, TRUCK_RAIL_BLOCK_HEIGHT
    ).translate((0.0, rail_block_offset_y, rail_block_z))
    right_rail_block = cq.Workplane("XY").box(
        0.24, TRUCK_RAIL_BLOCK_WIDTH, TRUCK_RAIL_BLOCK_HEIGHT
    ).translate((0.0, -rail_block_offset_y, rail_block_z))
    housing = cq.Workplane("XY").box(
        TRUCK_HOUSING_LENGTH, TRUCK_HOUSING_WIDTH, TRUCK_HOUSING_HEIGHT
    ).translate((0.0, 0.0, TRUCK_HOUSING_CENTER_Z))
    motor_pack = cq.Workplane("XY").box(
        TRUCK_MOTOR_LENGTH, TRUCK_MOTOR_WIDTH, TRUCK_MOTOR_HEIGHT
    ).translate(TRUCK_MOTOR_CENTER)
    control_box = cq.Workplane("XY").box(
        TRUCK_CONTROL_BOX_LENGTH, TRUCK_CONTROL_BOX_WIDTH, TRUCK_CONTROL_BOX_HEIGHT
    ).translate(TRUCK_CONTROL_BOX_CENTER)
    return (
        deck.union(left_rail_block)
        .union(right_rail_block)
        .union(housing)
        .union(motor_pack)
        .union(control_box)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="column_mounted_radial_arm_module")

    column_paint = model.material("column_paint", rgba=(0.27, 0.29, 0.32, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.83, 0.71, 0.18, 1.0))
    truck_paint = model.material("truck_paint", rgba=(0.87, 0.42, 0.15, 1.0))

    column = model.part("column")
    column.visual(
        mesh_from_cadquery(_build_column_shape(), "column_shell"),
        material=column_paint,
        name="column_shell",
    )
    column.inertial = Inertial.from_geometry(
        Box((BASE_SIZE, BASE_SIZE, SUPPORT_PLANE_Z)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * SUPPORT_PLANE_Z)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_build_arm_shape(), "arm_shell"),
        material=arm_paint,
        name="arm_shell",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.52, 0.34, 0.24)),
        mass=120.0,
        origin=Origin(xyz=(0.595, 0.0, 0.10)),
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_build_truck_shape(), "truck_shell"),
        material=truck_paint,
        name="truck_shell",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.30, 0.22, 0.26)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=column,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.9,
            lower=-2.5,
            upper=2.5,
        ),
    )

    model.articulation(
        "arm_to_truck",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=truck,
        origin=Origin(xyz=(TRUCK_HOME_X, 0.0, BEAM_CENTER_Z + 0.5 * BEAM_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.5,
            lower=0.0,
            upper=TRUCK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    arm = object_model.get_part("arm")
    truck = object_model.get_part("truck")
    turret = object_model.get_articulation("column_to_arm")
    slide = object_model.get_articulation("arm_to_truck")

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
        "turret_axis_vertical",
        turret.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical turret axis, got {turret.axis}",
    )
    ctx.check(
        "truck_axis_along_beam",
        slide.axis == (1.0, 0.0, 0.0),
        details=f"expected beamwise slide axis, got {slide.axis}",
    )

    with ctx.pose({turret: 0.0, slide: 0.0}):
        ctx.expect_contact(arm, column, name="arm_supported_on_column")
        ctx.expect_gap(
            truck,
            arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="truck_seats_on_beam_at_home",
        )
        ctx.expect_overlap(truck, arm, axes="xy", min_overlap=0.12, name="truck_home_overlap_on_beam")

    with ctx.pose({turret: 0.0, slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            truck,
            arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="truck_seats_on_beam_when_extended",
        )
        ctx.expect_overlap(
            truck,
            arm,
            axes="xy",
            min_overlap=0.12,
            name="truck_extended_overlap_on_beam",
        )

    with ctx.pose({turret: 0.0, slide: 0.0}):
        truck_home = ctx.part_world_position(truck)
    with ctx.pose({turret: 0.0, slide: slide.motion_limits.upper}):
        truck_out = ctx.part_world_position(truck)

    if truck_home is None or truck_out is None:
        ctx.fail("truck_prismatic_motion_measurable", "could not resolve truck world positions")
    else:
        dx = truck_out[0] - truck_home[0]
        dy = truck_out[1] - truck_home[1]
        dz = truck_out[2] - truck_home[2]
        ctx.check(
            "truck_extends_along_arm_axis",
            dx > 0.50 and abs(dy) < 1e-6 and abs(dz) < 1e-6,
            details=f"expected +X slide with no Y/Z drift, got dx={dx:.4f}, dy={dy:.6f}, dz={dz:.6f}",
        )

    mid_slide = 0.30
    with ctx.pose({turret: 0.0, slide: mid_slide}):
        truck_before = ctx.part_world_position(truck)
    with ctx.pose({turret: 1.0, slide: mid_slide}):
        truck_after = ctx.part_world_position(truck)

    if truck_before is None or truck_after is None:
        ctx.fail("turret_rotation_measurable", "could not resolve truck world positions through turret sweep")
    else:
        dx = truck_after[0] - truck_before[0]
        dy = truck_after[1] - truck_before[1]
        ctx.check(
            "positive_turret_rotation_swings_beam_counterclockwise",
            dx < -0.20 and dy > 0.55,
            details=f"expected +q turret motion to swing the arm into +Y, got dx={dx:.4f}, dy={dy:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
