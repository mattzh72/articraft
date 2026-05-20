from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


BASE_RADIUS = 0.170
BASE_BODY_H = 0.046
BASE_TOP_H = 0.012
BASE_HEIGHT = BASE_BODY_H + BASE_TOP_H
BASE_TOP_RADIUS = 0.154
BASE_BOLT_RADIUS = 0.0065
BASE_BOLT_HEIGHT = 0.0045
BASE_BOLT_COUNT = 8
BASE_BOLT_RING_RADIUS = 0.144

ROTARY_PLATE_RADIUS = 0.132
ROTARY_PLATE_H = 0.026
ROTARY_HUB_RADIUS = 0.078
ROTARY_HUB_H = 0.030

COLUMN_W = 0.120
COLUMN_D = 0.092
COLUMN_FRONT = COLUMN_D / 2.0
COLUMN_H = 0.600
COLUMN_Z0 = ROTARY_PLATE_H + ROTARY_HUB_H

TRACK_W = 0.016
TRACK_D = 0.014
TRACK_H = 0.454
TRACK_Z0 = 0.122
TRACK_OFFSET_X = 0.028

STOP_W = 0.084
STOP_D = 0.010
STOP_H = 0.012
LOWER_STOP_Z = 0.273
UPPER_STOP_Z = 0.634

CARR_W = 0.158
CARR_GUIDE_W = 0.110
CARR_GUIDE_D = 0.022
CARR_BODY_D = 0.070
CARR_D = CARR_GUIDE_D + CARR_BODY_D
CARR_H = 0.142
CARR_HALF_H = CARR_H / 2.0

SLIDE_HOME_Z = 0.360
SLIDE_TRAVEL = 0.200

YAW_LOWER = -2.6
YAW_UPPER = 2.6


def _make_base_shape() -> cq.Workplane:
    lower_shell = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_BODY_H)
    upper_flange = (
        cq.Workplane("XY")
        .circle(BASE_TOP_RADIUS)
        .extrude(BASE_TOP_H)
        .translate((0.0, 0.0, BASE_BODY_H))
    )
    return lower_shell.union(upper_flange)


def _make_rotary_stage_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(ROTARY_PLATE_RADIUS).extrude(ROTARY_PLATE_H)
    hub = (
        cq.Workplane("XY")
        .circle(ROTARY_HUB_RADIUS)
        .extrude(ROTARY_HUB_H)
        .translate((0.0, 0.0, ROTARY_PLATE_H))
    )
    column_back = (
        cq.Workplane("XY")
        .box(COLUMN_W, 0.030, COLUMN_H)
        .translate((0.0, -0.031, COLUMN_Z0 + (COLUMN_H / 2.0)))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(0.026, 0.062, 0.520)
        .translate((-0.047, 0.015, COLUMN_Z0 + 0.260))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.026, 0.062, 0.520)
        .translate((0.047, 0.015, COLUMN_Z0 + 0.260))
    )
    bottom_saddle = (
        cq.Workplane("XY")
        .box(0.094, 0.078, 0.080)
        .translate((0.0, -0.007, COLUMN_Z0 + 0.040))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(0.050, 0.024, 0.180)
        .translate((0.0, -0.034, COLUMN_Z0 + 0.090))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(COLUMN_W, 0.062, 0.020)
        .translate((0.0, 0.015, COLUMN_Z0 + COLUMN_H - 0.010))
    )
    lower_stop_bridge = (
        cq.Workplane("XY")
        .box(STOP_W, 0.016, 0.020)
        .translate((0.0, 0.038, LOWER_STOP_Z + 0.010))
    )
    upper_stop_strut = (
        cq.Workplane("XY")
        .box(0.020, 0.022, 0.070)
        .translate((0.0, 0.024, 0.681))
    )
    upper_stop_bridge = (
        cq.Workplane("XY")
        .box(STOP_W, 0.016, 0.018)
        .translate((0.0, 0.038, UPPER_STOP_Z + 0.009))
    )

    return (
        plate.union(hub)
        .union(column_back)
        .union(left_cheek)
        .union(right_cheek)
        .union(bottom_saddle)
        .union(rear_rib)
        .union(top_bridge)
        .union(lower_stop_bridge)
        .union(upper_stop_strut)
        .union(upper_stop_bridge)
    )


def _make_carriage_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(0.01, 0.01, 0.01)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_mast_module")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machine_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    model.material("guide_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("carriage_orange", rgba=(0.78, 0.44, 0.18, 1.0))
    model.material("fastener_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("stop_pad", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material="base_charcoal",
        name="base_shell",
    )
    for bolt_index in range(BASE_BOLT_COUNT):
        angle = (2.0 * pi * bolt_index) / BASE_BOLT_COUNT
        base.visual(
            Cylinder(radius=BASE_BOLT_RADIUS, length=BASE_BOLT_HEIGHT),
            origin=Origin(
                xyz=(
                    BASE_BOLT_RING_RADIUS * cos(angle),
                    BASE_BOLT_RING_RADIUS * sin(angle),
                    BASE_HEIGHT + (BASE_BOLT_HEIGHT / 2.0),
                )
            ),
            material="fastener_dark",
            name=f"flange_bolt_{bolt_index + 1}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_RADIUS * 2.0, BASE_RADIUS * 2.0, BASE_HEIGHT + BASE_BOLT_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + BASE_BOLT_HEIGHT) / 2.0)),
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        mesh_from_cadquery(_make_rotary_stage_shape(), "rotary_stage_shell"),
        material="machine_gray",
        name="rotary_stage_shell",
    )
    rotary_stage.visual(
        Box((TRACK_W, TRACK_D, TRACK_H)),
        origin=Origin(
            xyz=(
                -TRACK_OFFSET_X,
                COLUMN_FRONT + (TRACK_D / 2.0),
                TRACK_Z0 + (TRACK_H / 2.0),
            )
        ),
        material="guide_steel",
        name="left_track",
    )
    rotary_stage.visual(
        Box((TRACK_W, TRACK_D, TRACK_H)),
        origin=Origin(
            xyz=(
                TRACK_OFFSET_X,
                COLUMN_FRONT + (TRACK_D / 2.0),
                TRACK_Z0 + (TRACK_H / 2.0),
            )
        ),
        material="guide_steel",
        name="right_track",
    )
    rotary_stage.visual(
        Box((STOP_W, STOP_D, STOP_H)),
        origin=Origin(
            xyz=(0.0, COLUMN_FRONT - 0.001, LOWER_STOP_Z + (STOP_H / 2.0))
        ),
        material="stop_pad",
        name="lower_stop",
    )
    rotary_stage.visual(
        Box((STOP_W, STOP_D, STOP_H)),
        origin=Origin(
            xyz=(0.0, COLUMN_FRONT - 0.001, UPPER_STOP_Z + (STOP_H / 2.0))
        ),
        material="stop_pad",
        name="upper_stop",
    )
    rotary_stage.inertial = Inertial.from_geometry(
        Box((ROTARY_PLATE_RADIUS * 2.0, 0.16, COLUMN_Z0 + COLUMN_H)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_Z0 + COLUMN_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.040, 0.030, 0.112)),
        origin=Origin(xyz=(-TRACK_OFFSET_X, 0.015, 0.0)),
        material="guide_steel",
        name="left_truck",
    )
    carriage.visual(
        Box((0.040, 0.030, 0.112)),
        origin=Origin(xyz=(TRACK_OFFSET_X, 0.015, 0.0)),
        material="guide_steel",
        name="right_truck",
    )
    carriage.visual(
        Box((0.116, 0.016, CARR_H)),
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        material="carriage_orange",
        name="bridge_web",
    )
    carriage.visual(
        Box((CARR_W, 0.036, CARR_H)),
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        material="carriage_orange",
        name="carriage_body",
    )
    carriage.visual(
        Box((0.128, 0.016, 0.120)),
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        material="carriage_orange",
        name="face_plate",
    )
    carriage.visual(
        Box((0.016, 0.024, 0.094)),
        origin=Origin(xyz=(-0.056, 0.055, 0.0)),
        material="carriage_orange",
        name="left_rib",
    )
    carriage.visual(
        Box((0.016, 0.024, 0.094)),
        origin=Origin(xyz=(0.056, 0.055, 0.0)),
        material="carriage_orange",
        name="right_rib",
    )
    carriage.visual(
        Box((0.084, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.093, -0.056)),
        material="stop_pad",
        name="lower_bumper",
    )
    carriage.visual(
        Box((0.084, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.093, 0.056)),
        material="stop_pad",
        name="upper_bumper",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_W, CARR_D, CARR_H)),
        mass=3.2,
        origin=Origin(xyz=(0.0, CARR_D / 2.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=YAW_LOWER,
            upper=YAW_UPPER,
            effort=220.0,
            velocity=1.3,
        ),
    )
    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, COLUMN_FRONT + TRACK_D, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=1800.0,
            velocity=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotary_stage = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("carriage")
    base_yaw = object_model.get_articulation("base_yaw")
    column_slide = object_model.get_articulation("column_slide")

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
        "yaw_joint_axis_is_vertical",
        tuple(base_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {base_yaw.axis}",
    )
    ctx.check(
        "slide_joint_axis_is_vertical",
        tuple(column_slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical slide axis, got {column_slide.axis}",
    )
    ctx.expect_contact(rotary_stage, base, name="rotary_stage_seated_on_base")
    ctx.expect_contact(
        carriage,
        rotary_stage,
        elem_a="left_truck",
        elem_b="left_track",
        name="left_truck_guided_on_left_track",
    )
    ctx.expect_contact(
        carriage,
        rotary_stage,
        elem_a="right_truck",
        elem_b="right_track",
        name="right_truck_guided_on_right_track",
    )

    with ctx.pose(column_slide=0.0):
        ctx.expect_gap(
            carriage,
            rotary_stage,
            axis="z",
            positive_elem="lower_bumper",
            negative_elem="lower_stop",
            min_gap=0.006,
            max_gap=0.020,
            name="lower_stop_clearance",
        )

    with ctx.pose(column_slide=SLIDE_TRAVEL):
        ctx.expect_gap(
            rotary_stage,
            carriage,
            axis="z",
            positive_elem="upper_stop",
            negative_elem="upper_bumper",
            min_gap=0.006,
            max_gap=0.020,
            name="upper_stop_clearance",
        )

    with ctx.pose(base_yaw=2.1, column_slide=SLIDE_TRAVEL):
        ctx.expect_contact(
            carriage,
            rotary_stage,
            elem_a="left_truck",
            elem_b="left_track",
            name="left_truck_guided_at_yaw_extreme",
        )
        ctx.expect_contact(
            carriage,
            rotary_stage,
            elem_a="right_truck",
            elem_b="right_track",
            name="right_truck_guided_at_yaw_extreme",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
