from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_LENGTH = 0.240
BASE_WIDTH = 0.200
BASE_THICKNESS = 0.026
BASE_CAP_LENGTH = 0.180
BASE_CAP_WIDTH = 0.140
BASE_CAP_THICKNESS = 0.012
BEARING_RADIUS = 0.068
BEARING_HEIGHT = 0.010
SMALL_FUSE = 0.0005
YAW_JOINT_Z = BASE_THICKNESS + BASE_CAP_THICKNESS + BEARING_HEIGHT - SMALL_FUSE

TURNTABLE_RADIUS = 0.084
TURNTABLE_THICKNESS = 0.018
STAGE_BLOCK_LENGTH = 0.112
STAGE_BLOCK_DEPTH = 0.074
STAGE_BLOCK_HEIGHT = 0.036
STAGE_BLOCK_Y = -0.040
GUIDE_RAIL_WIDTH = 0.020
GUIDE_RAIL_DEPTH = 0.024
GUIDE_RAIL_HEIGHT = 0.126
GUIDE_RAIL_Y = -0.050
GUIDE_RAIL_X = 0.026
GUIDE_RAIL_BOTTOM_Z = 0.048
BACK_WEB_LENGTH = 0.086
BACK_WEB_DEPTH = 0.014
BACK_WEB_HEIGHT = 0.134
BACK_WEB_Y = -0.060
BRIDGE_LENGTH = 0.074
BRIDGE_DEPTH = 0.024
BRIDGE_HEIGHT = 0.014
BRIDGE_BOTTOM_Z = GUIDE_RAIL_BOTTOM_Z + GUIDE_RAIL_HEIGHT
DRIVE_HOUSING_LENGTH = 0.072
DRIVE_HOUSING_DEPTH = 0.042
DRIVE_HOUSING_HEIGHT = 0.022
DRIVE_HOUSING_Y = 0.024
MAST_BLOCK_LENGTH = 0.110
MAST_BLOCK_DEPTH = 0.084
MAST_BLOCK_HEIGHT = 0.040
MAST_BLOCK_Y = -0.038
MAST_COLUMN_WIDTH = 0.032
MAST_COLUMN_DEPTH = 0.030
MAST_COLUMN_HEIGHT = 0.165
MAST_COLUMN_Y = -0.045
MAST_COLUMN_BOTTOM_Z = TURNTABLE_THICKNESS + MAST_BLOCK_HEIGHT - SMALL_FUSE
REAR_BUTTRESS_LENGTH = 0.076
REAR_BUTTRESS_DEPTH = 0.016
REAR_BUTTRESS_HEIGHT = 0.112
REAR_BUTTRESS_Y = -0.060
REAR_BUTTRESS_Z = 0.096

CARRIAGE_HEAD_LENGTH = 0.096
CARRIAGE_MAX_DEPTH = 0.046
CARRIAGE_HEIGHT = 0.076
CARRIAGE_WINDOW_LENGTH = 0.046
CARRIAGE_WINDOW_DEPTH = 0.012
CARRIAGE_WINDOW_HEIGHT = 0.022
CARRIAGE_WINDOW_Y = 0.036
CARRIAGE_WINDOW_Z = 0.006

LIFT_JOINT_X = 0.0
LIFT_JOINT_Y = MAST_COLUMN_Y + (MAST_COLUMN_DEPTH / 2.0)
LIFT_JOINT_Z = 0.118
LIFT_TRAVEL = 0.050


def _base_foot_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.014)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    cap = (
        cq.Workplane("XY")
        .box(BASE_CAP_LENGTH, BASE_CAP_WIDTH, BASE_CAP_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
        .translate(
            (
                0.0,
                0.0,
                BASE_THICKNESS + (BASE_CAP_THICKNESS / 2.0) - SMALL_FUSE,
            )
        )
    )
    bearing = cq.Workplane("XY").circle(BEARING_RADIUS).extrude(BEARING_HEIGHT).translate(
        (0.0, 0.0, BASE_THICKNESS + BASE_CAP_THICKNESS - SMALL_FUSE)
    )
    return plinth.union(cap).union(bearing)


def _yaw_stage_shape() -> cq.Workplane:
    turntable = cq.Workplane("XY").circle(TURNTABLE_RADIUS).extrude(TURNTABLE_THICKNESS)
    stage_block = (
        cq.Workplane("XY")
        .box(MAST_BLOCK_LENGTH, MAST_BLOCK_DEPTH, MAST_BLOCK_HEIGHT)
        .edges("|Z")
        .fillet(0.007)
        .translate(
            (
                0.0,
                MAST_BLOCK_Y,
                TURNTABLE_THICKNESS + (MAST_BLOCK_HEIGHT / 2.0) - SMALL_FUSE,
            )
        )
    )
    mast_column = (
        cq.Workplane("XY")
        .box(MAST_COLUMN_WIDTH, MAST_COLUMN_DEPTH, MAST_COLUMN_HEIGHT)
        .edges("|Z")
        .fillet(0.003)
        .translate(
            (
                0.0,
                MAST_COLUMN_Y,
                MAST_COLUMN_BOTTOM_Z + (MAST_COLUMN_HEIGHT / 2.0),
            )
        )
    )
    rear_buttress = (
        cq.Workplane("XY")
        .box(REAR_BUTTRESS_LENGTH, REAR_BUTTRESS_DEPTH, REAR_BUTTRESS_HEIGHT)
        .translate(
            (
                0.0,
                REAR_BUTTRESS_Y,
                REAR_BUTTRESS_Z,
            )
        )
    )
    drive_housing = (
        cq.Workplane("XY")
        .box(DRIVE_HOUSING_LENGTH, DRIVE_HOUSING_DEPTH, DRIVE_HOUSING_HEIGHT)
        .edges("|Z")
        .fillet(0.005)
        .translate(
            (
                0.0,
                DRIVE_HOUSING_Y,
                TURNTABLE_THICKNESS + (DRIVE_HOUSING_HEIGHT / 2.0) - SMALL_FUSE,
            )
        )
    )
    return (
        turntable.union(stage_block)
        .union(mast_column)
        .union(rear_buttress)
        .union(drive_housing)
    )


def _carriage_shape() -> cq.Workplane:
    side_profile = (
        cq.Workplane("YZ")
        .moveTo(0.000, -0.034)
        .lineTo(0.018, -0.034)
        .lineTo(0.034, -0.022)
        .lineTo(0.046, -0.022)
        .lineTo(0.046, 0.026)
        .lineTo(0.032, 0.038)
        .lineTo(0.012, 0.038)
        .lineTo(0.000, 0.028)
        .close()
        .extrude(CARRIAGE_HEAD_LENGTH, both=True)
        .edges("|X")
        .fillet(0.003)
    )
    return side_profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_lift_module")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("stage_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("carriage_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    base_foot = model.part("base_foot")
    base_foot.visual(
        mesh_from_cadquery(_base_foot_shape(), "base_foot"),
        material="base_charcoal",
        name="base_shell",
    )
    base_foot.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, YAW_JOINT_Z)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z / 2.0)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage"),
        material="stage_gray",
        name="stage_shell",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((2.0 * TURNTABLE_RADIUS, 0.150, 0.188)),
        mass=2.3,
        origin=Origin(xyz=(0.0, -0.020, 0.094)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material="carriage_silver",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_HEAD_LENGTH, CARRIAGE_MAX_DEPTH, CARRIAGE_HEIGHT)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base_foot,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.4,
            upper=2.4,
            effort=18.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=yaw_stage,
        child=carriage,
        origin=Origin(xyz=(LIFT_JOINT_X, LIFT_JOINT_Y, LIFT_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=140.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_foot = object_model.get_part("base_foot")
    yaw_stage = object_model.get_part("yaw_stage")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("base_yaw")
    lift = object_model.get_articulation("mast_lift")

    ctx.allow_overlap(
        carriage,
        yaw_stage,
        reason="The lift carriage is a captured guide riding the mast column, so the simplified nested guide envelope is intentional.",
    )

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
        "joint_axes_are_vertical",
        yaw.axis == (0.0, 0.0, 1.0) and lift.axis == (0.0, 0.0, 1.0),
        details=f"yaw axis={yaw.axis}, lift axis={lift.axis}",
    )
    ctx.check(
        "rotating_stage_heavier_than_lifted_carriage",
        (
            yaw_stage.inertial is not None
            and carriage.inertial is not None
            and getattr(yaw_stage.inertial, "mass", 0.0) > getattr(carriage.inertial, "mass", 0.0)
        ),
        details=(
            f"yaw_stage mass={getattr(yaw_stage.inertial, 'mass', None)}, "
            f"carriage mass={getattr(carriage.inertial, 'mass', None)}"
        ),
    )

    base_aabb = ctx.part_world_aabb(base_foot)
    carriage_aabb = ctx.part_world_aabb(carriage)
    footprint_ok = False
    footprint_details = "missing AABB data"
    if base_aabb is not None and carriage_aabb is not None:
        base_x = base_aabb[1][0] - base_aabb[0][0]
        base_y = base_aabb[1][1] - base_aabb[0][1]
        carriage_x = carriage_aabb[1][0] - carriage_aabb[0][0]
        carriage_y = carriage_aabb[1][1] - carriage_aabb[0][1]
        footprint_ok = base_x > carriage_x and base_y > carriage_y
        footprint_details = (
            f"base footprint=({base_x:.4f}, {base_y:.4f}), "
            f"carriage footprint=({carriage_x:.4f}, {carriage_y:.4f})"
        )
    ctx.check("base_footprint_broader_than_carriage", footprint_ok, details=footprint_details)

    with ctx.pose({yaw: 0.0, lift: 0.0}):
        ctx.expect_contact(base_foot, yaw_stage, name="yaw_stage_bearing_seated")
        ctx.expect_contact(carriage, yaw_stage, name="carriage_guides_contact_at_home")

    with ctx.pose({lift: lift.motion_limits.upper}):
        ctx.expect_contact(carriage, yaw_stage, name="carriage_guides_contact_at_full_lift")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_lift")

    carriage_home = None
    carriage_raised = None
    carriage_yawed = None
    with ctx.pose({yaw: 0.0, lift: 0.0}):
        carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({yaw: 0.0, lift: lift.motion_limits.upper}):
        carriage_raised = ctx.part_world_position(carriage)
    with ctx.pose({yaw: 1.0, lift: 0.0}):
        carriage_yawed = ctx.part_world_position(carriage)

    lift_ok = False
    lift_details = "missing carriage positions"
    if carriage_home is not None and carriage_raised is not None:
        dz = carriage_raised[2] - carriage_home[2]
        lift_ok = dz > 0.045
        lift_details = f"carriage z change={dz:.4f}"
    ctx.check("positive_prismatic_motion_raises_carriage", lift_ok, details=lift_details)

    yaw_ok = False
    yaw_details = "missing carriage positions"
    if carriage_home is not None and carriage_yawed is not None:
        dx = carriage_yawed[0] - carriage_home[0]
        dy = carriage_yawed[1] - carriage_home[1]
        yaw_ok = (dx * dx + dy * dy) > 0.0002 and isclose(
            carriage_yawed[2],
            carriage_home[2],
            abs_tol=1e-6,
        )
        yaw_details = f"carriage xy shift=({dx:.4f}, {dy:.4f})"
    ctx.check("yaw_motion_swings_offset_mast", yaw_ok, details=yaw_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
