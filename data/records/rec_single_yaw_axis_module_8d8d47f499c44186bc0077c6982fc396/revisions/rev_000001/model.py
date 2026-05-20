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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.24
FOOT_WIDTH = 0.18
FOOT_THICKNESS = 0.028

LOWER_COLUMN_RADIUS = 0.038
LOWER_COLUMN_HEIGHT = 0.052
UPPER_COLUMN_RADIUS = 0.028
UPPER_COLUMN_HEIGHT = 0.033

HOUSING_RADIUS = 0.048
HOUSING_THICKNESS = 0.014
HOUSING_BORE_RADIUS = 0.022

RIB_LENGTH = 0.086
RIB_THICKNESS = 0.028
RIB_OFFSET_Y = 0.036
RIB_HEIGHT = 0.03
RIB_BASE_Z = FOOT_THICKNESS + LOWER_COLUMN_HEIGHT + UPPER_COLUMN_HEIGHT - 0.016

HOLE_SPACING_X = 0.16
HOLE_SPACING_Y = 0.11
MOUNT_HOLE_DIAMETER = 0.01

HOUSING_BASE_Z = FOOT_THICKNESS + LOWER_COLUMN_HEIGHT + UPPER_COLUMN_HEIGHT
JOINT_Z = HOUSING_BASE_Z + HOUSING_THICKNESS

TURNTABLE_RADIUS = 0.035
TURNTABLE_THICKNESS = 0.012
HEAD_NECK_RADIUS = 0.026
HEAD_NECK_HEIGHT = 0.02

HEAD_BODY_LENGTH = 0.11
HEAD_BODY_WIDTH = 0.072
HEAD_BODY_HEIGHT = 0.052
HEAD_BODY_X = 0.045
HEAD_BODY_Z = TURNTABLE_THICKNESS + HEAD_NECK_HEIGHT

NOSE_RADIUS = 0.016
NOSE_LENGTH = 0.022
NOSE_X = 0.096
NOSE_Z = 0.058


def _pedestal_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(
        FOOT_LENGTH,
        FOOT_WIDTH,
        FOOT_THICKNESS,
        centered=(True, True, False),
    )
    foot = foot.edges("|Z").fillet(0.012)
    foot = (
        foot.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-HOLE_SPACING_X / 2, -HOLE_SPACING_Y / 2),
                (-HOLE_SPACING_X / 2, HOLE_SPACING_Y / 2),
                (HOLE_SPACING_X / 2, -HOLE_SPACING_Y / 2),
                (HOLE_SPACING_X / 2, HOLE_SPACING_Y / 2),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
    )

    lower_column = (
        cq.Workplane("XY")
        .circle(LOWER_COLUMN_RADIUS)
        .extrude(LOWER_COLUMN_HEIGHT)
        .translate((0.0, 0.0, FOOT_THICKNESS))
    )
    upper_column = (
        cq.Workplane("XY")
        .circle(UPPER_COLUMN_RADIUS)
        .extrude(UPPER_COLUMN_HEIGHT)
        .translate((0.0, 0.0, FOOT_THICKNESS + LOWER_COLUMN_HEIGHT))
    )

    housing = (
        cq.Workplane("XY")
        .circle(HOUSING_RADIUS)
        .circle(HOUSING_BORE_RADIUS)
        .extrude(HOUSING_THICKNESS)
        .translate((0.0, 0.0, HOUSING_BASE_Z))
    )

    left_rib = cq.Workplane("XY").box(
        RIB_LENGTH,
        RIB_THICKNESS,
        RIB_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, RIB_OFFSET_Y, RIB_BASE_Z))
    right_rib = cq.Workplane("XY").box(
        RIB_LENGTH,
        RIB_THICKNESS,
        RIB_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, -RIB_OFFSET_Y, RIB_BASE_Z))

    pedestal = (
        foot.union(lower_column)
        .union(upper_column)
        .union(housing)
        .union(left_rib)
        .union(right_rib)
    )
    return pedestal


def _head_shell_shape() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .circle(TURNTABLE_RADIUS)
        .extrude(TURNTABLE_THICKNESS)
    )
    neck = (
        cq.Workplane("XY")
        .circle(HEAD_NECK_RADIUS)
        .extrude(HEAD_NECK_HEIGHT)
        .translate((0.0, 0.0, TURNTABLE_THICKNESS))
    )
    head_body = cq.Workplane("XY").box(
        HEAD_BODY_LENGTH,
        HEAD_BODY_WIDTH,
        HEAD_BODY_HEIGHT,
        centered=(True, True, False),
    )
    head_body = head_body.edges("|Z").fillet(0.008)
    head_body = head_body.translate((HEAD_BODY_X, 0.0, HEAD_BODY_Z))

    rear_pod = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.036)
        .translate((-0.018, 0.0, HEAD_BODY_Z + 0.02))
    )

    return turntable.union(neck).union(head_body).union(rear_pod)


def _head_nose_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(NOSE_RADIUS)
        .extrude(NOSE_LENGTH)
        .translate((NOSE_X, 0.0, NOSE_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_module")

    base_material = model.material("pedestal_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    head_material = model.material("head_silver", rgba=(0.68, 0.7, 0.73, 1.0))
    nose_material = model.material("sensor_black", rgba=(0.12, 0.13, 0.15, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_shell"),
        material=base_material,
        name="pedestal_shell",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, JOINT_Z)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z / 2)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "head_shell"),
        material=head_material,
        name="head_shell",
    )
    head.visual(
        mesh_from_cadquery(_head_nose_shape(), "sensor_nose"),
        material=nose_material,
        name="sensor_nose",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.12, 0.09, 0.09)),
        mass=2.2,
        origin=Origin(xyz=(0.04, 0.0, 0.045)),
    )

    model.articulation(
        "pedestal_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    head = object_model.get_part("head")
    head_yaw = object_model.get_articulation("pedestal_to_head")

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
        "parts_present",
        pedestal is not None and head is not None and head_yaw is not None,
        details="pedestal, head, and yaw articulation should all exist",
    )
    ctx.check(
        "yaw_axis_is_vertical",
        tuple(head_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {head_yaw.axis}",
    )
    ctx.check(
        "yaw_limits_are_bidirectional",
        head_yaw.motion_limits is not None
        and head_yaw.motion_limits.lower is not None
        and head_yaw.motion_limits.upper is not None
        and head_yaw.motion_limits.lower < 0.0 < head_yaw.motion_limits.upper
        and head_yaw.motion_limits.upper >= math.radians(150.0)
        and head_yaw.motion_limits.lower <= -math.radians(150.0),
        details="yaw travel should allow substantial left and right rotation",
    )

    ctx.expect_contact(
        head,
        pedestal,
        contact_tol=0.001,
        name="head_is_carried_by_bearing_housing",
    )

    with ctx.pose({head_yaw: math.pi / 2}):
        ctx.expect_contact(
            head,
            pedestal,
            contact_tol=0.001,
            name="bearing_contact_persists_at_quarter_turn",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_quarter_turn")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((low + high) / 2.0 for low, high in zip(aabb[0], aabb[1]))

    with ctx.pose({head_yaw: 0.0}):
        nose_center_0 = _aabb_center(ctx.part_element_world_aabb(head, elem="sensor_nose"))
    with ctx.pose({head_yaw: math.pi / 2}):
        nose_center_90 = _aabb_center(ctx.part_element_world_aabb(head, elem="sensor_nose"))

    nose_rotates_correctly = (
        nose_center_0 is not None
        and nose_center_90 is not None
        and nose_center_0[0] > 0.07
        and abs(nose_center_0[1]) < 0.025
        and nose_center_90[1] > 0.07
        and abs(nose_center_90[0]) < 0.025
    )
    ctx.check(
        "quarter_turn_swings_head_about_column_axis",
        nose_rotates_correctly,
        details=f"nose centers were q0={nose_center_0}, q90={nose_center_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
