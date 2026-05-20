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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.22
BASE_Y = 0.14
BASE_Z = 0.05

COLUMN_X = 0.11
COLUMN_Y = 0.11
COLUMN_Z = 0.60

BACKPLATE_X = 0.028
BACKPLATE_Y = 0.20
BACKPLATE_Z = 0.74

HEAD_X = 0.11
HEAD_Y = 0.08
HEAD_Z = 0.12

BRACE_X = 0.05
BRACE_Y = 0.06
BRACE_Z = 0.22

BEAM_Y_START = 0.07
BEAM_Y_END = 0.50
BEAM_LENGTH = BEAM_Y_END - BEAM_Y_START
BEAM_Y_CENTER = (BEAM_Y_START + BEAM_Y_END) / 2.0
BEAM_Z = 0.60
UPPER_BEAM_ROD_Z = BEAM_Z + 0.03
LOWER_BEAM_ROD_Z = BEAM_Z - 0.03
BEAM_ROD_X = 0.02
BEAM_SCREW_X = -0.01
BEAM_ROD_R = 0.012
BEAM_SCREW_R = 0.008

OUTBOARD_SUPPORT_X = 0.08
OUTBOARD_SUPPORT_Y = 0.028
OUTBOARD_SUPPORT_Z = 0.10

Y_HOME = 0.18
Y_TRAVEL = 0.24

CARRIAGE_BLOCK_X = 0.09
CARRIAGE_BLOCK_Y = 0.12
CARRIAGE_BLOCK_Z = 0.15

CARRIAGE_BLOCK_CENTER_X = 0.02
CARRIAGE_UPPER_HOLE_Z = 0.03
CARRIAGE_LOWER_HOLE_Z = -0.03

GUIDE_CHEEK_X = 0.050
GUIDE_CHEEK_Y = 0.012
GUIDE_CHEEK_Z = 0.12
GUIDE_CHEEK_CENTER_X = 0.135
GUIDE_CHEEK_CENTER_Z = -0.170
GUIDE_CHEEK_Y_CENTER = 0.056

GUIDE_BRIDGE_X = 0.060
GUIDE_BRIDGE_Y = 0.124
GUIDE_BRIDGE_Z = 0.022
GUIDE_BRIDGE_CENTER_X = 0.125
GUIDE_BRIDGE_CENTER_Z = -0.108

VERTICAL_ROD_R = 0.010
VERTICAL_ROD_Y_OFFSET = 0.033
VERTICAL_ROD_LENGTH = 0.30
VERTICAL_ROD_CENTER_X = 0.10
VERTICAL_ROD_CENTER_Z = -0.28
VERTICAL_SCREW_R = 0.007

Z_HOME_OFFSET = -0.17
Z_TRAVEL = 0.20

SLIDE_BLOCK_X = 0.07
SLIDE_BLOCK_Y = 0.10
SLIDE_BLOCK_Z = 0.09

TOOL_CONNECTOR_X = 0.05
TOOL_CONNECTOR_Y = 0.07
TOOL_CONNECTOR_Z = 0.12

TOOL_PLATE_X = 0.018
TOOL_PLATE_Y = 0.18
TOOL_PLATE_Z = 0.22

TOOL_MOUNT_X = 0.045
TOOL_MOUNT_Y = 0.07
TOOL_MOUNT_Z = 0.036


def _cq_cylinder_y(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center_xyz
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - (length / 2.0), z))
    )


def _cq_cylinder_z(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center_xyz
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((x, y, z - (length / 2.0)))
    )


def _tower_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_Z).translate((0.03, 0.0, BASE_Z / 2.0))
    body = body.union(
        cq.Workplane("XY").box(COLUMN_X, COLUMN_Y, COLUMN_Z).translate((0.0, 0.0, BASE_Z + (COLUMN_Z / 2.0)))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(BACKPLATE_X, BACKPLATE_Y, BACKPLATE_Z)
        .translate((-0.055, 0.0, BACKPLATE_Z / 2.0))
    )
    body = body.union(
        cq.Workplane("XY").box(HEAD_X, HEAD_Y, HEAD_Z).translate((0.01, 0.07, BEAM_Z))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(BRACE_X, BRACE_Y, BRACE_Z)
        .translate((0.03, 0.035, 0.50))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(OUTBOARD_SUPPORT_X, OUTBOARD_SUPPORT_Y, OUTBOARD_SUPPORT_Z)
        .translate((0.01, BEAM_Y_END, BEAM_Z))
    )
    return body.combine().val()


def _beam_carriage_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_BLOCK_X, CARRIAGE_BLOCK_Y, CARRIAGE_BLOCK_Z)
        .translate((CARRIAGE_BLOCK_CENTER_X, 0.0, 0.0))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(GUIDE_MOUNT_X, GUIDE_MOUNT_Y, GUIDE_MOUNT_Z)
        .translate((GUIDE_MOUNT_CENTER_X, 0.0, GUIDE_MOUNT_CENTER_Z))
    )
    body = body.cut(
        _cq_cylinder_y(
            BEAM_ROD_R,
            CARRIAGE_BLOCK_Y + 0.02,
            (BEAM_ROD_X, 0.0, CARRIAGE_UPPER_HOLE_Z),
        )
    )
    body = body.cut(
        _cq_cylinder_y(
            BEAM_ROD_R,
            CARRIAGE_BLOCK_Y + 0.02,
            (BEAM_ROD_X, 0.0, CARRIAGE_LOWER_HOLE_Z),
        )
    )
    body = body.cut(
        _cq_cylinder_y(
            BEAM_SCREW_R + 0.001,
            CARRIAGE_BLOCK_Y + 0.02,
            (BEAM_SCREW_X, 0.0, 0.0),
        )
    )
    return body.combine().val()


def _tool_plate_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(SLIDE_BLOCK_X, SLIDE_BLOCK_Y, SLIDE_BLOCK_Z)
        .translate((0.03, 0.0, -(SLIDE_BLOCK_Z / 2.0)))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(0.055, 0.078, 0.13)
        .translate((0.042, 0.0, -0.135))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(TOOL_PLATE_X, TOOL_PLATE_Y, 0.24)
        .translate((0.064, 0.0, -0.19))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(0.05, TOOL_MOUNT_Y, 0.07)
        .translate((0.052, 0.0, -0.275))
    )
    body = body.cut(
        _cq_cylinder_z(
            VERTICAL_ROD_R,
            SLIDE_BLOCK_Z + 0.03,
            (0.0, VERTICAL_ROD_Y_OFFSET, -(SLIDE_BLOCK_Z / 2.0)),
        )
    )
    body = body.cut(
        _cq_cylinder_z(
            VERTICAL_ROD_R,
            SLIDE_BLOCK_Z + 0.03,
            (0.0, -VERTICAL_ROD_Y_OFFSET, -(SLIDE_BLOCK_Z / 2.0)),
        )
    )
    body = body.cut(
        _cq_cylinder_z(
            VERTICAL_SCREW_R + 0.001,
            SLIDE_BLOCK_Z + 0.03,
            (0.0, 0.0, -(SLIDE_BLOCK_Z / 2.0)),
        )
    )
    return body.combine().val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_yz_positioning_module")

    model.material("frame_charcoal", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_orange", rgba=(0.86, 0.46, 0.14, 1.0))
    model.material("tool_gray", rgba=(0.80, 0.82, 0.84, 1.0))

    tower = model.part("tower_frame")
    tower.visual(
        mesh_from_cadquery(_tower_body_shape(), "tower_frame_body"),
        material="frame_charcoal",
        name="tower_body",
    )
    tower.visual(
        Cylinder(radius=BEAM_ROD_R, length=BEAM_LENGTH),
        origin=Origin(xyz=(BEAM_ROD_X, BEAM_Y_CENTER, UPPER_BEAM_ROD_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="rail_steel",
        name="upper_beam_rod",
    )
    tower.visual(
        Cylinder(radius=BEAM_ROD_R, length=BEAM_LENGTH),
        origin=Origin(xyz=(BEAM_ROD_X, BEAM_Y_CENTER, LOWER_BEAM_ROD_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="rail_steel",
        name="lower_beam_rod",
    )
    tower.visual(
        Cylinder(radius=BEAM_SCREW_R, length=BEAM_LENGTH),
        origin=Origin(xyz=(BEAM_SCREW_X, BEAM_Y_CENTER, BEAM_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="rail_steel",
        name="beam_drive_screw",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.24, 0.52, 0.74)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.24, 0.37)),
    )

    beam_carriage = model.part("beam_carriage")
    beam_carriage.visual(
        Box((0.055, 0.12, 0.15)),
        origin=Origin(xyz=(0.0595, 0.0, 0.0)),
        material="carriage_orange",
        name="beam_carriage_body",
    )
    beam_carriage.visual(
        Box((0.040, 0.080, 0.035)),
        origin=Origin(xyz=(0.078, 0.0, -0.090)),
        material="carriage_orange",
        name="beam_carriage_neck",
    )
    beam_carriage.visual(
        Box((0.095, 0.10, 0.065)),
        origin=Origin(xyz=(0.100, 0.0, -0.130)),
        material="carriage_orange",
        name="vertical_guide_mount",
    )
    beam_carriage.visual(
        Cylinder(radius=VERTICAL_ROD_R, length=VERTICAL_ROD_LENGTH),
        origin=Origin(
            xyz=(VERTICAL_ROD_CENTER_X, VERTICAL_ROD_Y_OFFSET, VERTICAL_ROD_CENTER_Z),
            rpy=(0.0, 0.0, 0.0),
        ),
        material="rail_steel",
        name="left_vertical_rod",
    )
    beam_carriage.visual(
        Cylinder(radius=VERTICAL_ROD_R, length=VERTICAL_ROD_LENGTH),
        origin=Origin(
            xyz=(VERTICAL_ROD_CENTER_X, -VERTICAL_ROD_Y_OFFSET, VERTICAL_ROD_CENTER_Z),
            rpy=(0.0, 0.0, 0.0),
        ),
        material="rail_steel",
        name="right_vertical_rod",
    )
    beam_carriage.visual(
        Cylinder(radius=VERTICAL_SCREW_R, length=VERTICAL_ROD_LENGTH),
        origin=Origin(xyz=(VERTICAL_ROD_CENTER_X, 0.0, VERTICAL_ROD_CENTER_Z)),
        material="rail_steel",
        name="vertical_drive_screw",
    )
    beam_carriage.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.38)),
        mass=5.6,
        origin=Origin(xyz=(0.07, 0.0, -0.14)),
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((0.050, 0.10, 0.09)),
        origin=Origin(xyz=(0.035, 0.0, -0.045)),
        material="tool_gray",
        name="tool_slide_block",
    )
    tool_plate.visual(
        Box((0.040, 0.075, 0.10)),
        origin=Origin(xyz=(0.050, 0.0, -0.130)),
        material="tool_gray",
        name="tool_plate_spine",
    )
    tool_plate.visual(
        Box((0.018, 0.18, 0.22)),
        origin=Origin(xyz=(0.063, 0.0, -0.220)),
        material="tool_gray",
        name="tool_plate_body",
    )
    tool_plate.visual(
        Box((0.050, 0.07, 0.05)),
        origin=Origin(xyz=(0.050, 0.0, -0.295)),
        material="tool_gray",
        name="tool_mount_block",
    )
    tool_plate.inertial = Inertial.from_geometry(
        Box((0.08, 0.18, 0.33)),
        mass=3.2,
        origin=Origin(xyz=(0.04, 0.0, -0.17)),
    )

    model.articulation(
        "tower_to_beam_carriage",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=beam_carriage,
        origin=Origin(xyz=(0.0, Y_HOME, BEAM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.35,
            lower=0.0,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "beam_carriage_to_tool_plate",
        ArticulationType.PRISMATIC,
        parent=beam_carriage,
        child=tool_plate,
        origin=Origin(xyz=(VERTICAL_ROD_CENTER_X, 0.0, Z_HOME_OFFSET)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.25,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower_frame")
    beam_carriage = object_model.get_part("beam_carriage")
    tool_plate = object_model.get_part("tool_plate")
    y_axis = object_model.get_articulation("tower_to_beam_carriage")
    z_axis = object_model.get_articulation("beam_carriage_to_tool_plate")

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
        "horizontal_axis_is_lateral_prismatic",
        tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        details=f"expected lateral +Y axis, got {y_axis.axis}",
    )
    ctx.check(
        "vertical_axis_descends_on_positive_motion",
        tuple(z_axis.axis) == (0.0, 0.0, -1.0),
        details=f"expected descending -Z axis, got {z_axis.axis}",
    )

    with ctx.pose({y_axis: 0.0, z_axis: 0.0}):
        ctx.expect_contact(
            beam_carriage,
            tower,
            contact_tol=0.001,
            name="beam_carriage_is_supported_by_horizontal_rods",
        )
        ctx.expect_contact(
            tool_plate,
            beam_carriage,
            contact_tol=0.001,
            name="tool_plate_is_supported_by_vertical_rods",
        )
        ctx.expect_origin_gap(
            beam_carriage,
            tower,
            axis="y",
            min_gap=0.15,
            max_gap=0.22,
            name="carriage_home_position_is_near_tower",
        )
        ctx.expect_origin_gap(
            beam_carriage,
            tool_plate,
            axis="z",
            min_gap=0.16,
            max_gap=0.19,
            name="tool_plate_hangs_below_beam_carriage_at_home",
        )

    with ctx.pose({y_axis: Y_TRAVEL}):
        ctx.expect_origin_gap(
            beam_carriage,
            tower,
            axis="y",
            min_gap=0.38,
            max_gap=0.45,
            name="carriage_reaches_outboard_end_of_beam",
        )
        ctx.expect_contact(
            beam_carriage,
            tower,
            contact_tol=0.001,
            name="beam_carriage_remains_guided_at_outboard_travel",
        )

    with ctx.pose({z_axis: Z_TRAVEL}):
        ctx.expect_origin_gap(
            beam_carriage,
            tool_plate,
            axis="z",
            min_gap=0.35,
            max_gap=0.40,
            name="tool_plate_descends_when_z_axis_extends",
        )
        ctx.expect_contact(
            tool_plate,
            beam_carriage,
            contact_tol=0.001,
            name="tool_plate_remains_guided_at_full_drop",
        )

    with ctx.pose({y_axis: Y_TRAVEL, z_axis: Z_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_reach")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
