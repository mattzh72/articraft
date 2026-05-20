from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_RADIUS = 0.115
BASE_HEIGHT = 0.028
BASE_MID_HEIGHT = 0.018
BASE_TOP_RING_HEIGHT = 0.010
BASE_MID_RADIUS = 0.100
BASE_TOP_RADIUS = 0.082

ROTARY_RADIUS = 0.102
ROTARY_THICKNESS = 0.018

TOWER_CENTER_Y = -0.045
TOWER_WIDTH = 0.060
TOWER_DEPTH = 0.040
TOWER_HEIGHT = 0.112

SHOULDER_WIDTH = 0.090
SHOULDER_DEPTH = 0.058
SHOULDER_HEIGHT = 0.028

GUIDE_FRONT_Y = TOWER_CENTER_Y + TOWER_DEPTH * 0.5
GUIDE_SLOT_BOTTOM_Z = ROTARY_THICKNESS + SHOULDER_HEIGHT + 0.004
GUIDE_SLOT_TOP_Z = ROTARY_THICKNESS + SHOULDER_HEIGHT + TOWER_HEIGHT - 0.004
GUIDE_SLOT_HEIGHT = GUIDE_SLOT_TOP_Z - GUIDE_SLOT_BOTTOM_Z

STAGE_CARRIAGE_WIDTH = 0.030
STAGE_CARRIAGE_DEPTH = 0.008
STAGE_CARRIAGE_HEIGHT = 0.032
STAGE_MAST_WIDTH = 0.018
STAGE_MAST_DEPTH = 0.012
STAGE_MAST_HEIGHT = 0.074
STAGE_NECK_WIDTH = 0.016
STAGE_NECK_DEPTH = 0.064
STAGE_NECK_HEIGHT = 0.012
STAGE_NECK_Z = 0.068
PAD_SIZE = 0.056
PAD_THICKNESS = 0.010
PAD_FORWARD_OFFSET = 0.040
PAD_BOTTOM_Z = 0.076

TURNTABLE_LIMIT = math.pi
LIFT_TRAVEL = 0.045


def _make_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .extrude(BASE_MID_HEIGHT)
        .faces(">Z")
        .workplane()
        .circle(BASE_MID_RADIUS)
        .extrude(BASE_TOP_RING_HEIGHT * 0.4)
        .faces(">Z")
        .workplane()
        .circle(BASE_TOP_RADIUS)
        .extrude(BASE_TOP_RING_HEIGHT * 0.6)
    )


def _make_rotary_head_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(ROTARY_RADIUS).extrude(ROTARY_THICKNESS)

    pedestal = (
        cq.Workplane("XY")
        .box(
            SHOULDER_WIDTH,
            SHOULDER_DEPTH,
            SHOULDER_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, TOWER_CENTER_Y, ROTARY_THICKNESS))
    )

    tower = (
        cq.Workplane("XY")
        .box(
            TOWER_WIDTH,
            TOWER_DEPTH,
            TOWER_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, TOWER_CENTER_Y, ROTARY_THICKNESS + SHOULDER_HEIGHT))
    )

    cap = (
        cq.Workplane("XY")
        .box(
            TOWER_WIDTH + 0.012,
            0.020,
            0.014,
            centered=(True, True, False),
        )
        .translate((0.0, TOWER_CENTER_Y - 0.002, ROTARY_THICKNESS + SHOULDER_HEIGHT + TOWER_HEIGHT - 0.010))
    )

    left_gusset = (
        cq.Workplane("XY")
        .box(
            0.016,
            0.040,
            0.060,
            centered=(True, True, False),
        )
        .translate((-0.022, TOWER_CENTER_Y, ROTARY_THICKNESS))
    )

    right_gusset = (
        cq.Workplane("XY")
        .box(
            0.016,
            0.040,
            0.060,
            centered=(True, True, False),
        )
        .translate((0.022, TOWER_CENTER_Y, ROTARY_THICKNESS))
    )

    return platter.union(pedestal).union(tower).union(cap).union(left_gusset).union(right_gusset)


def _make_lift_stage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        STAGE_CARRIAGE_WIDTH,
        STAGE_CARRIAGE_DEPTH,
        STAGE_CARRIAGE_HEIGHT,
        centered=(True, False, False),
    )

    mast = (
        cq.Workplane("XY")
        .box(
            STAGE_MAST_WIDTH,
            STAGE_MAST_DEPTH,
            STAGE_MAST_HEIGHT,
            centered=(True, False, False),
        )
        .translate((0.0, 0.0, 0.010))
    )

    neck = (
        cq.Workplane("XY")
        .box(
            STAGE_NECK_WIDTH,
            STAGE_NECK_DEPTH,
            STAGE_NECK_HEIGHT,
            centered=(True, False, False),
        )
        .translate((0.0, 0.010, STAGE_NECK_Z))
    )

    pad = (
        cq.Workplane("XY")
        .box(
            PAD_SIZE,
            PAD_SIZE,
            PAD_THICKNESS,
            centered=(True, False, False),
        )
        .translate((0.0, PAD_FORWARD_OFFSET, PAD_BOTTOM_Z))
    )

    return carriage.union(mast).union(neck).union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_lift_platform")

    base_material = model.material("base_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    rotary_material = model.material("rotary_gray", rgba=(0.36, 0.37, 0.40, 1.0))
    stage_material = model.material("stage_metal", rgba=(0.74, 0.75, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material=base_material,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
    )

    rotary_head = model.part("rotary_head")
    rotary_head.visual(
        mesh_from_cadquery(_make_rotary_head_shape(), "rotary_head_shell"),
        material=rotary_material,
        name="rotary_head_shell",
    )
    rotary_head.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.17)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -0.02, 0.085)),
    )

    lift_stage = model.part("lift_stage")
    lift_stage.visual(
        mesh_from_cadquery(_make_lift_stage_shape(), "lift_stage_shell"),
        material=stage_material,
        name="lift_stage_shell",
    )
    lift_stage.inertial = Inertial.from_geometry(
        Box((PAD_SIZE, PAD_SIZE, PAD_BOTTOM_Z + PAD_THICKNESS)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (PAD_BOTTOM_Z + PAD_THICKNESS) * 0.5)),
    )

    model.articulation(
        "base_to_rotary_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_head,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-TURNTABLE_LIMIT,
            upper=TURNTABLE_LIMIT,
        ),
    )

    model.articulation(
        "rotary_head_to_lift_stage",
        ArticulationType.PRISMATIC,
        parent=rotary_head,
        child=lift_stage,
        origin=Origin(xyz=(0.0, GUIDE_FRONT_Y, GUIDE_SLOT_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotary_head = object_model.get_part("rotary_head")
    lift_stage = object_model.get_part("lift_stage")
    turntable = object_model.get_articulation("base_to_rotary_head")
    lift = object_model.get_articulation("rotary_head_to_lift_stage")

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
        "turntable_is_revolute",
        turntable.articulation_type == ArticulationType.REVOLUTE,
        details=f"expected revolute turntable, got {turntable.articulation_type}",
    )
    ctx.check(
        "lift_stage_is_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"expected prismatic lift, got {lift.articulation_type}",
    )
    ctx.check(
        "turntable_axis_is_vertical",
        tuple(turntable.axis) == (0.0, 0.0, 1.0),
        details=f"turntable axis is {turntable.axis}",
    )
    ctx.check(
        "lift_axis_is_vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"lift axis is {lift.axis}",
    )

    ctx.expect_contact(
        rotary_head,
        base,
        name="turntable_platform_seats_on_base",
    )
    ctx.expect_contact(
        lift_stage,
        rotary_head,
        name="lift_stage_seats_in_guide",
    )

    rest_stage_position = ctx.part_world_position(lift_stage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        extended_stage_position = ctx.part_world_position(lift_stage)
        ctx.expect_gap(
            lift_stage,
            base,
            axis="z",
            min_gap=0.06,
            name="extended_stage_clears_base",
        )

    if rest_stage_position is not None and extended_stage_position is not None:
        measured_travel = extended_stage_position[2] - rest_stage_position[2]
        ctx.check(
            "lift_stage_moves_up_by_joint_travel",
            abs(measured_travel - LIFT_TRAVEL) < 1e-6,
            details=(
                f"expected lift travel {LIFT_TRAVEL:.4f} m, "
                f"measured {measured_travel:.4f} m"
            ),
        )

    with ctx.pose({turntable: 0.0}):
        stage_at_zero = ctx.part_world_position(lift_stage)
    with ctx.pose({turntable: math.pi * 0.5}):
        stage_at_quarter_turn = ctx.part_world_position(lift_stage)

    if stage_at_zero is not None and stage_at_quarter_turn is not None:
        radius_zero = math.hypot(stage_at_zero[0], stage_at_zero[1])
        radius_quarter = math.hypot(
            stage_at_quarter_turn[0],
            stage_at_quarter_turn[1],
        )
        quarter_turn_ok = (
            abs(radius_zero - radius_quarter) < 1e-6
            and abs(stage_at_quarter_turn[0] + stage_at_zero[1]) < 0.002
            and abs(stage_at_quarter_turn[1] - stage_at_zero[0]) < 0.002
        )
        ctx.check(
            "turntable_sweeps_tower_around_center",
            quarter_turn_ok,
            details=(
                f"q0={stage_at_zero}, q90={stage_at_quarter_turn}, "
                f"r0={radius_zero:.4f}, r90={radius_quarter:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
