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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.34
BASE_HEIGHT = 0.08
PEDESTAL_RADIUS = 0.125
PEDESTAL_HEIGHT = 0.022
BEARING_RADIUS = 0.103
BEARING_HEIGHT = 0.010

TURN_TABLE_RADIUS = 0.110
TURN_TABLE_HEIGHT = 0.028
ROTARY_SADDLE_X = 0.170
ROTARY_SADDLE_Y = 0.170
ROTARY_SADDLE_HEIGHT = 0.020

COLUMN_DEPTH = 0.090
COLUMN_WIDTH = 0.130
COLUMN_HEIGHT = 0.580
COLUMN_BASE_Z = TURN_TABLE_HEIGHT + ROTARY_SADDLE_HEIGHT

GUIDE_RAIL_DEPTH = 0.015
GUIDE_RAIL_WIDTH = 0.024
GUIDE_RAIL_HEIGHT = 0.400
GUIDE_RAIL_CENTER_Y = 0.040
GUIDE_RAIL_BASE_Z = 0.120

SLIDE_HOME_Z = 0.200
LIFT_TRAVEL = 0.240

CARRIAGE_BLOCK_DEPTH = 0.040
CARRIAGE_BLOCK_WIDTH = 0.170
CARRIAGE_BLOCK_HEIGHT = 0.160
SLIDER_BLOCK_DEPTH = 0.032
SLIDER_BLOCK_WIDTH = 0.032
SLIDER_BLOCK_HEIGHT = 0.140
TOOL_FACE_DEPTH = 0.012
TOOL_FACE_WIDTH = 0.090
TOOL_FACE_HEIGHT = 0.100
TOOL_BOSS_RADIUS = 0.022
TOOL_BOSS_LENGTH = 0.015

YAW_JOINT_Z = BASE_HEIGHT + PEDESTAL_HEIGHT + BEARING_HEIGHT
SLIDE_X = (COLUMN_DEPTH / 2.0) + GUIDE_RAIL_DEPTH
YAW_TEST_ANGLE = pi / 2.0


def _build_base_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )

    upper_skirt = (
        cq.Workplane("XY")
        .box(0.290, 0.240, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_HEIGHT))
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )

    bearing_ring = (
        cq.Workplane("XY")
        .circle(BEARING_RADIUS)
        .extrude(BEARING_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT))
    )

    return plinth.union(upper_skirt).union(pedestal).union(bearing_ring)


def _build_rotary_stage_shape() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .circle(TURN_TABLE_RADIUS)
        .extrude(TURN_TABLE_HEIGHT)
        .edges("%CIRCLE")
        .fillet(0.006)
    )

    saddle = (
        cq.Workplane("XY")
        .box(
            ROTARY_SADDLE_X,
            0.150,
            ROTARY_SADDLE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, TURN_TABLE_HEIGHT))
    )

    left_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.070, TURN_TABLE_HEIGHT),
                (-0.040, TURN_TABLE_HEIGHT + ROTARY_SADDLE_HEIGHT),
                (-0.006, TURN_TABLE_HEIGHT + ROTARY_SADDLE_HEIGHT),
                (-0.038, TURN_TABLE_HEIGHT),
            ]
        )
        .close()
        .extrude(0.016)
        .translate((0.0, 0.055, 0.0))
    )

    right_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.070, TURN_TABLE_HEIGHT),
                (-0.040, TURN_TABLE_HEIGHT + ROTARY_SADDLE_HEIGHT),
                (-0.006, TURN_TABLE_HEIGHT + ROTARY_SADDLE_HEIGHT),
                (-0.038, TURN_TABLE_HEIGHT),
            ]
        )
        .close()
        .extrude(-0.016)
        .translate((0.0, -0.055, 0.0))
    )

    rear_boss = (
        cq.Workplane("XY")
        .box(0.042, 0.120, ROTARY_SADDLE_HEIGHT, centered=(True, True, False))
        .translate((-0.046, 0.0, TURN_TABLE_HEIGHT))
    )

    return turntable.union(saddle).union(left_rib).union(right_rib).union(rear_boss)


def _build_column_shape() -> cq.Workplane:
    column = cq.Workplane("XY").box(
        COLUMN_DEPTH,
        COLUMN_WIDTH,
        COLUMN_HEIGHT,
        centered=(True, True, False),
    )

    guide_rail_local_z = GUIDE_RAIL_BASE_Z - COLUMN_BASE_Z
    guide_rail_left = (
        cq.Workplane("XY")
        .box(
            GUIDE_RAIL_DEPTH,
            GUIDE_RAIL_WIDTH,
            GUIDE_RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                (COLUMN_DEPTH / 2.0) + (GUIDE_RAIL_DEPTH / 2.0),
                GUIDE_RAIL_CENTER_Y,
                guide_rail_local_z,
            )
        )
    )

    guide_rail_right = (
        cq.Workplane("XY")
        .box(
            GUIDE_RAIL_DEPTH,
            GUIDE_RAIL_WIDTH,
            GUIDE_RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                (COLUMN_DEPTH / 2.0) + (GUIDE_RAIL_DEPTH / 2.0),
                -GUIDE_RAIL_CENTER_Y,
                guide_rail_local_z,
            )
        )
    )

    top_cap = (
        cq.Workplane("XY")
        .box(0.100, 0.150, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, COLUMN_HEIGHT))
    )

    return column.union(guide_rail_left).union(guide_rail_right).union(top_cap)


def _build_carriage_shape() -> cq.Workplane:
    slider_left = (
        cq.Workplane("XY")
        .box(SLIDER_BLOCK_DEPTH, SLIDER_BLOCK_WIDTH, SLIDER_BLOCK_HEIGHT)
        .translate((SLIDER_BLOCK_DEPTH / 2.0, GUIDE_RAIL_CENTER_Y, 0.0))
    )
    slider_right = (
        cq.Workplane("XY")
        .box(SLIDER_BLOCK_DEPTH, SLIDER_BLOCK_WIDTH, SLIDER_BLOCK_HEIGHT)
        .translate((SLIDER_BLOCK_DEPTH / 2.0, -GUIDE_RAIL_CENTER_Y, 0.0))
    )

    carriage_block = (
        cq.Workplane("XY")
        .box(CARRIAGE_BLOCK_DEPTH, CARRIAGE_BLOCK_WIDTH, CARRIAGE_BLOCK_HEIGHT)
        .translate((SLIDER_BLOCK_DEPTH + (CARRIAGE_BLOCK_DEPTH / 2.0), 0.0, 0.0))
    )

    tool_face = (
        cq.Workplane("XY")
        .box(TOOL_FACE_DEPTH, TOOL_FACE_WIDTH, TOOL_FACE_HEIGHT)
        .translate(
            (
                SLIDER_BLOCK_DEPTH + CARRIAGE_BLOCK_DEPTH + (TOOL_FACE_DEPTH / 2.0),
                0.0,
                0.0,
            )
        )
    )

    tool_boss = (
        cq.Workplane("YZ")
        .workplane(offset=SLIDER_BLOCK_DEPTH + CARRIAGE_BLOCK_DEPTH + TOOL_FACE_DEPTH)
        .circle(TOOL_BOSS_RADIUS)
        .extrude(TOOL_BOSS_LENGTH)
    )

    return (
        slider_left.union(slider_right)
        .union(carriage_block)
        .union(tool_face)
        .union(tool_boss)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_base_vertical_slide")

    model.material("base_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_orange", rgba=(0.90, 0.48, 0.14, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_frame"),
        name="base_shell",
        material="base_graphite",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, YAW_JOINT_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z / 2.0)),
    )

    rotary = model.part("rotary_stage")
    rotary.visual(
        mesh_from_cadquery(_build_rotary_stage_shape(), "rotary_stage"),
        name="rotary_shell",
        material="machined_steel",
    )
    rotary.inertial = Inertial.from_geometry(
        Box((0.240, 0.240, COLUMN_BASE_Z)),
        mass=4.5,
        origin=Origin(
            xyz=(0.0, 0.0, COLUMN_BASE_Z / 2.0)
        ),
    )

    column = model.part("column_assembly")
    column.visual(
        mesh_from_cadquery(_build_column_shape(), "column_assembly"),
        name="column_shell",
        material="machined_steel",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.105, 0.150, COLUMN_HEIGHT + 0.018)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_HEIGHT + 0.018) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage"),
        name="carriage_shell",
        material="carriage_orange",
    )
    carriage.inertial = Inertial.from_geometry(
        Box(
            (
                SLIDER_BLOCK_DEPTH + CARRIAGE_BLOCK_DEPTH + TOOL_FACE_DEPTH + TOOL_BOSS_LENGTH,
                CARRIAGE_BLOCK_WIDTH,
                CARRIAGE_BLOCK_HEIGHT,
            )
        ),
        mass=3.2,
        origin=Origin(
            xyz=(
                (
                    SLIDER_BLOCK_DEPTH
                    + CARRIAGE_BLOCK_DEPTH
                    + TOOL_FACE_DEPTH
                    + TOOL_BOSS_LENGTH
                )
                / 2.0,
                0.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.6,
            upper=2.6,
            effort=60.0,
            velocity=1.2,
        ),
    )

    model.articulation(
        "rotary_to_column",
        ArticulationType.FIXED,
        parent=rotary,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BASE_Z)),
    )

    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(SLIDE_X, 0.0, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=900.0,
            velocity=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    rotary = object_model.get_part("rotary_stage")
    column = object_model.get_part("column_assembly")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("base_yaw")
    lift = object_model.get_articulation("column_lift")

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
        "base_yaw_axis_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "column_lift_axis_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lift.axis}",
    )

    ctx.expect_contact(
        rotary,
        base,
        contact_tol=5e-4,
        name="turntable_seats_on_grounded_base",
    )
    ctx.expect_contact(
        column,
        rotary,
        contact_tol=5e-4,
        name="column_is_mounted_to_turntable_stage",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_contact(
            carriage,
            column,
            contact_tol=5e-4,
            name="carriage_contacts_guides_at_home",
        )

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_contact(
            carriage,
            column,
            contact_tol=5e-4,
            name="carriage_contacts_guides_at_top",
        )

    with ctx.pose({yaw: 0.0, lift: 0.08}):
        carriage_pos_home = ctx.part_world_position(carriage)
    with ctx.pose({yaw: YAW_TEST_ANGLE, lift: 0.08}):
        carriage_pos_turned = ctx.part_world_position(carriage)

    yaw_ok = False
    yaw_details = "missing carriage world position"
    if carriage_pos_home is not None and carriage_pos_turned is not None:
        yaw_ok = (
            carriage_pos_home[0] > 0.05
            and abs(carriage_pos_home[1]) < 0.005
            and abs(carriage_pos_turned[0]) < 0.005
            and carriage_pos_turned[1] > 0.05
            and abs(carriage_pos_home[2] - carriage_pos_turned[2]) < 1e-6
        )
        yaw_details = (
            f"home={carriage_pos_home}, turned={carriage_pos_turned}, "
            f"expected quarter-turn orbit about z"
        )
    ctx.check("yaw_rotates_slide_assembly_about_vertical_axis", yaw_ok, yaw_details)

    with ctx.pose({yaw: 0.0, lift: 0.0}):
        carriage_low = ctx.part_world_position(carriage)
    with ctx.pose({yaw: 0.0, lift: LIFT_TRAVEL}):
        carriage_high = ctx.part_world_position(carriage)

    lift_ok = False
    lift_details = "missing carriage world position"
    if carriage_low is not None and carriage_high is not None:
        lift_ok = (
            abs(carriage_low[0] - carriage_high[0]) < 1e-6
            and abs(carriage_low[1] - carriage_high[1]) < 1e-6
            and abs((carriage_high[2] - carriage_low[2]) - LIFT_TRAVEL) < 1e-6
        )
        lift_details = (
            f"low={carriage_low}, high={carriage_high}, expected dz={LIFT_TRAVEL}"
        )
    ctx.check("prismatic_stage_lifts_carriage_along_column_axis", lift_ok, lift_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
