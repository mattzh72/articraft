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


BASE_RADIUS = 0.36
BASE_THICKNESS = 0.055
PEDESTAL_RADIUS = 0.145
PEDESTAL_HEIGHT = 0.12
POST_RADIUS = 0.08
POST_HEIGHT = 2.15
HEAD_RADIUS = 0.13
HEAD_THICKNESS = 0.025

ARM_RING_OUTER_RADIUS = 0.15
ARM_RING_INNER_RADIUS = 0.095
ARM_RING_THICKNESS = 0.03
ARM_SLEEVE_HEIGHT = 0.16

ARM_ROOT_X = 0.12
ARM_TIP_X = 1.45
ARM_LENGTH = ARM_TIP_X - ARM_ROOT_X
ARM_CENTER_Z = 0.14
ARM_ROOT_WIDTH = 0.19
ARM_TIP_WIDTH = 0.11
ARM_ROOT_HEIGHT = 0.15
ARM_TIP_HEIGHT = 0.10
ARM_WALL = 0.012

TRACK_START_X = 0.26
TRACK_LENGTH = 1.02
TRACK_WIDTH = 0.082
TRACK_HEIGHT = 0.034
TRACK_BOTTOM_Z = 0.031
SLIDE_RAIL_WIDTH = 0.060

CARRIAGE_TOP_LENGTH = 0.12
CARRIAGE_TOP_WIDTH = 0.115
CARRIAGE_TOP_THICKNESS = 0.028
CARRIAGE_SIDE_THICKNESS = 0.016
CARRIAGE_SIDE_DROP = 0.145
CARRIAGE_BLOCK_LENGTH = 0.105
CARRIAGE_BLOCK_WIDTH = 0.072
CARRIAGE_BLOCK_HEIGHT = 0.065
CARRIAGE_SENSOR_RADIUS = 0.018
CARRIAGE_SENSOR_HEIGHT = 0.045

CARRIAGE_ORIGIN_X = 0.74
CARRIAGE_SLIDE_LOWER = -0.36
CARRIAGE_SLIDE_UPPER = 0.28
CARRIAGE_BLOCK_CENTER_Z = -(
    CARRIAGE_TOP_THICKNESS + CARRIAGE_SIDE_DROP + (CARRIAGE_BLOCK_HEIGHT / 2.0) - 0.01
)
CARRIAGE_BLOCK_BOTTOM_Z = CARRIAGE_BLOCK_CENTER_Z - (CARRIAGE_BLOCK_HEIGHT / 2.0)
CARRIAGE_SENSOR_CENTER_Z = CARRIAGE_BLOCK_BOTTOM_Z - (CARRIAGE_SENSOR_HEIGHT / 2.0)


def _annulus(outer_radius: float, inner_radius: float, thickness: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0))
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def _tapered_beam(
    start_x: float,
    length: float,
    center_z: float,
    root_width: float,
    tip_width: float,
    root_height: float,
    tip_height: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(start_x, 0.0, center_z))
        .rect(root_width, root_height)
        .workplane(offset=length)
        .rect(tip_width, tip_height)
        .loft(combine=True, ruled=False)
    )


def _build_arm_shape() -> cq.Workplane:
    ring = _annulus(
        ARM_RING_OUTER_RADIUS,
        ARM_RING_INNER_RADIUS,
        ARM_RING_THICKNESS,
        z0=0.0,
    )
    sleeve = _annulus(
        ARM_RING_OUTER_RADIUS - 0.012,
        ARM_RING_INNER_RADIUS,
        ARM_SLEEVE_HEIGHT,
        z0=ARM_RING_THICKNESS,
    )

    beam_outer = _tapered_beam(
        ARM_ROOT_X,
        ARM_LENGTH,
        ARM_CENTER_Z,
        ARM_ROOT_WIDTH,
        ARM_TIP_WIDTH,
        ARM_ROOT_HEIGHT,
        ARM_TIP_HEIGHT,
    )
    beam_inner = _tapered_beam(
        ARM_ROOT_X + 0.05,
        ARM_LENGTH - 0.10,
        ARM_CENTER_Z,
        ARM_ROOT_WIDTH - (2.0 * ARM_WALL),
        ARM_TIP_WIDTH - (2.0 * ARM_WALL),
        ARM_ROOT_HEIGHT - (2.0 * ARM_WALL),
        ARM_TIP_HEIGHT - (2.0 * ARM_WALL),
    )
    beam = beam_outer.cut(beam_inner)

    track = (
        cq.Workplane("XY")
        .box(
            TRACK_LENGTH,
            TRACK_WIDTH,
            TRACK_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                TRACK_START_X + (TRACK_LENGTH / 2.0),
                0.0,
                TRACK_BOTTOM_Z,
            )
        )
    )
    slide_rail = (
        cq.Workplane("XY")
        .box(
            TRACK_LENGTH,
            SLIDE_RAIL_WIDTH,
            TRACK_BOTTOM_Z,
            centered=(True, True, False),
        )
        .translate(
            (
                TRACK_START_X + (TRACK_LENGTH / 2.0),
                0.0,
                0.0,
            )
        )
    )

    root_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.00, ARM_RING_THICKNESS),
                (0.11, ARM_RING_THICKNESS),
                (0.24, ARM_CENTER_Z - (ARM_ROOT_HEIGHT / 2.0)),
                (0.17, ARM_CENTER_Z + 0.015),
                (0.05, ARM_CENTER_Z + 0.03),
            ]
        )
        .close()
        .extrude(0.10, both=True)
    )

    arm = ring.union(sleeve).union(beam).union(track).union(slide_rail).union(root_web)
    return arm


def _build_carriage_shape() -> cq.Workplane:
    top_shoe = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_TOP_LENGTH,
            CARRIAGE_TOP_WIDTH,
            CARRIAGE_TOP_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -CARRIAGE_TOP_THICKNESS))
    )

    side_z = -(CARRIAGE_TOP_THICKNESS + (CARRIAGE_SIDE_DROP / 2.0))
    side_y = (CARRIAGE_TOP_WIDTH / 2.0) - (CARRIAGE_SIDE_THICKNESS / 2.0)
    left_side = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_TOP_LENGTH * 0.92,
            CARRIAGE_SIDE_THICKNESS,
            CARRIAGE_SIDE_DROP,
        )
        .translate((0.0, side_y, side_z))
    )
    right_side = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_TOP_LENGTH * 0.92,
            CARRIAGE_SIDE_THICKNESS,
            CARRIAGE_SIDE_DROP,
        )
        .translate((0.0, -side_y, side_z))
    )

    block = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_BLOCK_LENGTH,
            CARRIAGE_BLOCK_WIDTH,
            CARRIAGE_BLOCK_HEIGHT,
        )
        .translate((0.0, 0.0, CARRIAGE_BLOCK_CENTER_Z))
    )

    return top_shoe.union(left_side).union(right_side).union(block)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_boom")

    base_gray = model.material("base_gray", rgba=(0.42, 0.45, 0.50, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.56, 0.58, 0.62, 1.0))
    boom_yellow = model.material("boom_yellow", rgba=(0.91, 0.73, 0.18, 1.0))
    carriage_black = model.material("carriage_black", rgba=(0.16, 0.17, 0.19, 1.0))
    sensor_blue = model.material("sensor_blue", rgba=(0.30, 0.48, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_gray,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + (PEDESTAL_HEIGHT / 2.0),
            )
        ),
        material=steel_gray,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + (POST_HEIGHT / 2.0),
            )
        ),
        material=steel_gray,
        name="post",
    )
    base.visual(
        Cylinder(radius=HEAD_RADIUS, length=HEAD_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + POST_HEIGHT + (HEAD_THICKNESS / 2.0),
            )
        ),
        material=steel_gray,
        name="head_plate",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_build_arm_shape(), "arm_body"),
        material=boom_yellow,
        name="arm_body",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.48, 0.22, 0.22)),
        mass=65.0,
        origin=Origin(xyz=(0.72, 0.0, 0.14)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage_body"),
        material=carriage_black,
        name="carriage_body",
    )
    carriage.visual(
        Cylinder(radius=CARRIAGE_SENSOR_RADIUS, length=CARRIAGE_SENSOR_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CARRIAGE_SENSOR_CENTER_Z,
            )
        ),
        material=sensor_blue,
        name="sensor_pod",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.24)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    arm_joint_z = BASE_THICKNESS + POST_HEIGHT + HEAD_THICKNESS
    model.articulation(
        "post_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, arm_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.35,
            lower=CARRIAGE_SLIDE_LOWER,
            upper=CARRIAGE_SLIDE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    arm_joint = object_model.get_articulation("post_to_arm")
    carriage_joint = object_model.get_articulation("arm_to_carriage")

    ctx.check(
        "arm_rotates_about_post_axis",
        tuple(arm_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected post-axis revolute joint, got axis={arm_joint.axis}",
    )
    ctx.check(
        "carriage_slides_along_arm_axis",
        tuple(carriage_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected arm-axis prismatic joint, got axis={carriage_joint.axis}",
    )
    ctx.check(
        "carriage_default_pose_is_midspan",
        carriage_joint.motion_limits is not None
        and carriage_joint.motion_limits.lower is not None
        and carriage_joint.motion_limits.upper is not None
        and carriage_joint.motion_limits.lower < 0.0 < carriage_joint.motion_limits.upper,
        details="prismatic limits should straddle zero so the carriage defaults near mid-arm.",
    )

    ctx.expect_contact(
        arm,
        base,
        elem_a="arm_body",
        elem_b="head_plate",
        name="arm_turntable_contacts_post_head",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        min_overlap=0.12,
        elem_a="arm_body",
        elem_b="head_plate",
        name="arm_turntable_has_bearing_footprint",
    )
    ctx.expect_contact(
        carriage,
        arm,
        elem_a="carriage_body",
        elem_b="arm_body",
        name="carriage_hangs_from_arm_underside",
    )
    with ctx.pose({arm_joint: 1.25}):
        ctx.expect_contact(
            arm,
            base,
            elem_a="arm_body",
            elem_b="head_plate",
            name="arm_stays_seated_when_swiveled",
        )
    with ctx.pose({carriage_joint: CARRIAGE_SLIDE_UPPER}):
        ctx.expect_contact(
            carriage,
            arm,
            elem_a="carriage_body",
            elem_b="arm_body",
            name="carriage_keeps_contact_at_outer_travel",
        )
    with ctx.pose({carriage_joint: CARRIAGE_SLIDE_LOWER}):
        ctx.expect_contact(
            carriage,
            arm,
            elem_a="carriage_body",
            elem_b="arm_body",
            name="carriage_keeps_contact_at_inner_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
