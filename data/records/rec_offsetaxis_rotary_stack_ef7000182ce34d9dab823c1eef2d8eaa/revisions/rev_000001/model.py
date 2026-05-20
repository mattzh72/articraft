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


BASE_LENGTH = 0.260
BASE_WIDTH = 0.190
BASE_PLINTH_HEIGHT = 0.014
BASE_DRUM_RADIUS = 0.102
BASE_DRUM_HEIGHT = 0.032
BASE_CAP_RADIUS = 0.086
BASE_CAP_HEIGHT = 0.006
BASE_TOP_Z = BASE_PLINTH_HEIGHT + BASE_DRUM_HEIGHT + BASE_CAP_HEIGHT

LOWER_STAGE_RADIUS = 0.092
LOWER_STAGE_HEIGHT = 0.022
LOWER_STAGE_CAP_RADIUS = 0.040
LOWER_STAGE_CAP_HEIGHT = 0.006
LOWER_STAGE_TOP_Z = LOWER_STAGE_HEIGHT + LOWER_STAGE_CAP_HEIGHT

ARM_BASE_LENGTH = 0.084
ARM_BASE_WIDTH = 0.090
ARM_BASE_HEIGHT = 0.012
ARM_BEAM_LENGTH = 0.090
ARM_BEAM_WIDTH = 0.070
ARM_BEAM_HEIGHT = 0.020
ARM_BEAM_BOTTOM_Z = 0.046
ARM_COLUMN_X = 0.114
ARM_COLUMN_WIDTH = 0.056
ARM_COLUMN_DEPTH = 0.082
ARM_COLUMN_BOTTOM_Z = ARM_BASE_HEIGHT
ARM_COLUMN_HEIGHT = 0.086
ARM_TOP_PLATE_RADIUS = 0.046
ARM_TOP_PLATE_HEIGHT = 0.010
ARM_TOP_PLATE_BOTTOM_Z = 0.086
ARM_BEARING_CAP_RADIUS = 0.036
ARM_BEARING_CAP_HEIGHT = 0.006
ARM_BEARING_CAP_BOTTOM_Z = 0.096
UPPER_AXIS_Z = ARM_BEARING_CAP_BOTTOM_Z + ARM_BEARING_CAP_HEIGHT
ARM_RIB_THICKNESS = 0.012

UPPER_HEAD_FLANGE_RADIUS = 0.048
UPPER_HEAD_FLANGE_HEIGHT = 0.012
UPPER_HEAD_BODY_RADIUS = 0.040
UPPER_HEAD_BODY_HEIGHT = 0.036
UPPER_HEAD_FACE_RADIUS = 0.045
UPPER_HEAD_FACE_HEIGHT = 0.008
UPPER_HEAD_SPINDLE_RADIUS = 0.015
UPPER_HEAD_SPINDLE_HEIGHT = 0.010
UPPER_HEAD_TOTAL_HEIGHT = (
    UPPER_HEAD_FLANGE_HEIGHT
    + UPPER_HEAD_BODY_HEIGHT
    + UPPER_HEAD_FACE_HEIGHT
    + UPPER_HEAD_SPINDLE_HEIGHT
)

BASE_SWING_LIMIT = 2.95
HEAD_SWING_LIMIT = 3.10


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name)


def _build_base_housing() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_PLINTH_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
    )

    drum = (
        cq.Workplane("XY")
        .circle(BASE_DRUM_RADIUS)
        .extrude(BASE_DRUM_HEIGHT)
        .translate((0.0, 0.0, BASE_PLINTH_HEIGHT))
    )

    cap = cq.Workplane("XY").circle(BASE_CAP_RADIUS).extrude(BASE_CAP_HEIGHT)
    cap = (
        cap.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.030)
        .cutBlind(-0.002)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.060, 45, 360, 4)
        .circle(0.0035)
        .cutBlind(-0.002)
        .translate((0.0, 0.0, BASE_PLINTH_HEIGHT + BASE_DRUM_HEIGHT))
    )

    return plinth.union(drum).union(cap)


def _build_lower_stage() -> cq.Workplane:
    stage = cq.Workplane("XY").circle(LOWER_STAGE_RADIUS).extrude(LOWER_STAGE_HEIGHT)
    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.074)
        .circle(0.068)
        .cutBlind(-0.0016)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.059)
        .circle(0.054)
        .cutBlind(-0.0014)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.050, 0, 360, 3)
        .circle(0.004)
        .cutBlind(-0.002)
    )

    cap = (
        cq.Workplane("XY")
        .circle(LOWER_STAGE_CAP_RADIUS)
        .extrude(LOWER_STAGE_CAP_HEIGHT)
        .translate((0.0, 0.0, LOWER_STAGE_HEIGHT))
    )

    return stage.union(cap)


def _build_support_arm() -> cq.Workplane:
    base_pad = cq.Workplane("XY").box(
        ARM_BASE_LENGTH,
        ARM_BASE_WIDTH,
        ARM_BASE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.034, 0.0, 0.0))

    beam = cq.Workplane("XY").box(
        ARM_BEAM_LENGTH,
        ARM_BEAM_WIDTH,
        ARM_BEAM_HEIGHT,
        centered=(True, True, False),
    ).translate((0.060, 0.0, ARM_BEAM_BOTTOM_Z))

    column = cq.Workplane("XY").box(
        ARM_COLUMN_WIDTH,
        ARM_COLUMN_DEPTH,
        ARM_COLUMN_HEIGHT,
        centered=(True, True, False),
    ).translate((ARM_COLUMN_X, 0.0, ARM_COLUMN_BOTTOM_Z))

    top_plate = (
        cq.Workplane("XY")
        .circle(ARM_TOP_PLATE_RADIUS)
        .extrude(ARM_TOP_PLATE_HEIGHT)
        .translate((ARM_COLUMN_X, 0.0, ARM_TOP_PLATE_BOTTOM_Z))
    )

    bearing_cap = cq.Workplane("XY").circle(ARM_BEARING_CAP_RADIUS).extrude(ARM_BEARING_CAP_HEIGHT)
    bearing_cap = (
        bearing_cap.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.024, 45, 360, 4)
        .circle(0.0028)
        .cutBlind(-0.0018)
        .translate((ARM_COLUMN_X, 0.0, ARM_BEARING_CAP_BOTTOM_Z))
    )

    rib_profile = (
        cq.Workplane("XZ")
        .moveTo(-0.004, ARM_BASE_HEIGHT)
        .lineTo(0.026, ARM_BASE_HEIGHT)
        .lineTo(0.090, 0.070)
        .lineTo(0.106, 0.070)
        .lineTo(0.106, 0.024)
        .close()
        .extrude(ARM_RIB_THICKNESS)
    )
    front_rib = rib_profile.translate((0.0, ARM_BASE_WIDTH / 2.0 - ARM_RIB_THICKNESS, 0.0))
    rear_rib = rib_profile.translate((0.0, -ARM_BASE_WIDTH / 2.0, 0.0))

    arm = (
        base_pad.union(beam)
        .union(column)
        .union(top_plate)
        .union(bearing_cap)
        .union(front_rib)
        .union(rear_rib)
    )

    relief_window = cq.Workplane("XY").box(
        0.060,
        0.032,
        0.028,
        centered=(True, True, False),
    ).translate((0.056, 0.0, 0.018))

    return arm.cut(relief_window)


def _build_upper_head() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(UPPER_HEAD_FLANGE_RADIUS).extrude(UPPER_HEAD_FLANGE_HEIGHT)
    body = (
        cq.Workplane("XY")
        .circle(UPPER_HEAD_BODY_RADIUS)
        .extrude(UPPER_HEAD_BODY_HEIGHT)
        .translate((0.0, 0.0, UPPER_HEAD_FLANGE_HEIGHT))
    )

    face = cq.Workplane("XY").circle(UPPER_HEAD_FACE_RADIUS).extrude(UPPER_HEAD_FACE_HEIGHT)
    face = (
        face.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.035)
        .circle(0.031)
        .cutBlind(-0.0014)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.026, 0, 360, 3)
        .circle(0.0032)
        .cutBlind(-0.002)
        .translate((0.0, 0.0, UPPER_HEAD_FLANGE_HEIGHT + UPPER_HEAD_BODY_HEIGHT))
    )

    spindle = (
        cq.Workplane("XY")
        .circle(UPPER_HEAD_SPINDLE_RADIUS)
        .extrude(UPPER_HEAD_SPINDLE_HEIGHT)
        .translate(
            (
                0.0,
                0.0,
                UPPER_HEAD_FLANGE_HEIGHT + UPPER_HEAD_BODY_HEIGHT + UPPER_HEAD_FACE_HEIGHT,
            )
        )
    )

    side_pod = (
        cq.Workplane("XY")
        .box(0.030, 0.040, 0.026, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.048, 0.0, 0.013))
    )

    rear_cover = (
        cq.Workplane("XY")
        .box(0.018, 0.022, 0.018, centered=(True, True, False))
        .translate((-0.040, 0.0, 0.030))
    )

    return flange.union(body).union(face).union(spindle).union(side_pod).union(rear_cover)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_indexing_head")

    model.material("base_coat", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("arm_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("machined_alloy", rgba=(0.74, 0.77, 0.80, 1.0))
    model.material("head_alloy", rgba=(0.68, 0.71, 0.75, 1.0))

    base_housing = model.part("base_housing")
    base_housing.visual(_mesh(_build_base_housing(), "base_housing"), material="base_coat", name="housing")
    base_housing.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_TOP_Z)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(_mesh(_build_lower_stage(), "lower_stage"), material="machined_alloy", name="turntable")
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=LOWER_STAGE_RADIUS, length=LOWER_STAGE_TOP_Z),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_TOP_Z / 2.0)),
    )

    support_arm = model.part("support_arm")
    support_arm.visual(_mesh(_build_support_arm(), "support_arm"), material="arm_gray", name="arm_body")
    support_arm.inertial = Inertial.from_geometry(
        Box((0.160, ARM_BASE_WIDTH, UPPER_AXIS_Z)),
        mass=4.1,
        origin=Origin(xyz=(0.060, 0.0, UPPER_AXIS_Z / 2.0)),
    )

    upper_head = model.part("upper_head")
    upper_head.visual(_mesh(_build_upper_head(), "upper_head"), material="head_alloy", name="spin_head")
    upper_head.inertial = Inertial.from_geometry(
        Cylinder(radius=UPPER_HEAD_FLANGE_RADIUS, length=UPPER_HEAD_TOTAL_HEIGHT),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, UPPER_HEAD_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_rotation",
        ArticulationType.REVOLUTE,
        parent=base_housing,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-BASE_SWING_LIMIT,
            upper=BASE_SWING_LIMIT,
            effort=60.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "stage_to_arm",
        ArticulationType.FIXED,
        parent=lower_stage,
        child=support_arm,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_TOP_Z - ARM_COLUMN_BOTTOM_Z)),
    )
    model.articulation(
        "head_rotation",
        ArticulationType.REVOLUTE,
        parent=support_arm,
        child=upper_head,
        origin=Origin(xyz=(ARM_COLUMN_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-HEAD_SWING_LIMIT,
            upper=HEAD_SWING_LIMIT,
            effort=25.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_housing = object_model.get_part("base_housing")
    lower_stage = object_model.get_part("lower_stage")
    support_arm = object_model.get_part("support_arm")
    upper_head = object_model.get_part("upper_head")
    base_rotation = object_model.get_articulation("base_rotation")
    head_rotation = object_model.get_articulation("head_rotation")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20, name="rotary articulation sweep remains clear")

    ctx.expect_contact(
        lower_stage,
        base_housing,
        contact_tol=0.001,
        name="lower stage sits on main bearing cap",
    )
    ctx.expect_contact(
        support_arm,
        lower_stage,
        contact_tol=0.001,
        name="offset arm is mounted to lower stage",
    )
    ctx.expect_contact(
        upper_head,
        support_arm,
        contact_tol=0.001,
        name="upper head seats on offset upright bearing",
    )
    ctx.expect_origin_distance(
        upper_head,
        lower_stage,
        axes="x",
        min_dist=0.10,
        max_dist=0.13,
        name="upper spindle axis is clearly offset from main axis",
    )
    ctx.expect_gap(
        upper_head,
        lower_stage,
        axis="z",
        min_gap=0.085,
        name="upper stage stands well above lower rotary stage",
    )

    stage_aabb = ctx.part_world_aabb(lower_stage)
    head_aabb = ctx.part_world_aabb(upper_head)
    if stage_aabb is not None and head_aabb is not None:
        stage_size = tuple(stage_aabb[1][i] - stage_aabb[0][i] for i in range(3))
        head_size = tuple(head_aabb[1][i] - head_aabb[0][i] for i in range(3))
        ctx.check(
            "lower mass remains visibly larger than upper mass",
            stage_size[0] > head_size[0] * 1.35 and stage_size[1] > head_size[1] * 1.6,
            f"lower_stage_xy={stage_size[:2]}, upper_head_xy={head_size[:2]}",
        )

    ctx.check(
        "both rotary axes are upright and parallel",
        tuple(base_rotation.axis) == (0.0, 0.0, 1.0) and tuple(head_rotation.axis) == (0.0, 0.0, 1.0),
        f"base_axis={base_rotation.axis}, head_axis={head_rotation.axis}",
    )

    with ctx.pose(base_rotation=pi / 2.0, head_rotation=pi / 2.0):
        ctx.fail_if_parts_overlap_in_current_pose(name="no clipping in combined quarter-turn pose")
        ctx.expect_gap(
            support_arm,
            base_housing,
            axis="z",
            min_gap=0.02,
            name="raised arm clears base housing through swing pose",
        )
        ctx.expect_gap(
            upper_head,
            base_housing,
            axis="z",
            min_gap=0.115,
            name="upper head stays clear of the main housing in swing pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
