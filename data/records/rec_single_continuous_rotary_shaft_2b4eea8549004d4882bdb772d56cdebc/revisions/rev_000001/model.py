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


FOOT_LENGTH = 0.34
FOOT_WIDTH = 0.17
FOOT_THICKNESS = 0.04

SUPPORT_CENTER_X = 0.072
SUPPORT_BODY_THICKNESS = 0.044
SUPPORT_WIDTH = 0.11
SUPPORT_HEIGHT = 0.165
SUPPORT_BASE_HEIGHT = 0.036
SUPPORT_AXIS_Z = 0.102

SUPPORT_BOSS_RADIUS = 0.05
SUPPORT_BOSS_PROJECTION = 0.008
SUPPORT_BOSS_LENGTH = SUPPORT_BODY_THICKNESS + 2.0 * SUPPORT_BOSS_PROJECTION

SPINDLE_AXIS_Z = FOOT_THICKNESS + SUPPORT_AXIS_Z
SPINDLE_RADIUS = 0.023
SPINDLE_JOURNAL_CLEARANCE_RADIUS = 0.0265
COLLAR_RADIUS = 0.04
COLLAR_LENGTH = 0.008

LEFT_SUPPORT_OUTER_FACE_X = -SUPPORT_CENTER_X - SUPPORT_BOSS_LENGTH / 2.0
RIGHT_SUPPORT_OUTER_FACE_X = SUPPORT_CENTER_X + SUPPORT_BOSS_LENGTH / 2.0

LEFT_COLLAR_CENTER_X = LEFT_SUPPORT_OUTER_FACE_X - COLLAR_LENGTH / 2.0
RIGHT_COLLAR_CENTER_X = RIGHT_SUPPORT_OUTER_FACE_X + COLLAR_LENGTH / 2.0

SHAFT_LEFT_END_X = -0.122
FACEPLATE_FRONT_X = 0.154
SHAFT_BODY_LENGTH = FACEPLATE_FRONT_X - SHAFT_LEFT_END_X
SHAFT_BODY_CENTER_X = 0.5 * (SHAFT_LEFT_END_X + FACEPLATE_FRONT_X)

FACEPLATE_HUB_LENGTH = 0.03
FACEPLATE_HUB_START_X = RIGHT_SUPPORT_OUTER_FACE_X + COLLAR_LENGTH
FACEPLATE_HUB_CENTER_X = FACEPLATE_HUB_START_X + FACEPLATE_HUB_LENGTH / 2.0
FACEPLATE_RADIUS = 0.092
FACEPLATE_THICKNESS = 0.014
FACEPLATE_CENTER_X = FACEPLATE_FRONT_X - FACEPLATE_THICKNESS / 2.0
FACEPLATE_HUB_RADIUS = 0.043


def _axial_origin(x_center: float) -> Origin:
    return Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0))


def _build_foot_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS)
        .edges("|Z")
        .fillet(0.008)
        .edges(">Z")
        .fillet(0.004)
        .translate((0.0, 0.0, FOOT_THICKNESS / 2.0))
    )
    return foot


def _build_support_shape() -> cq.Workplane:
    lower_block = (
        cq.Workplane("XY")
        .box(SUPPORT_BODY_THICKNESS, SUPPORT_WIDTH, SUPPORT_BASE_HEIGHT)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, 0.0, SUPPORT_BASE_HEIGHT / 2.0))
    )

    upper_height = SUPPORT_HEIGHT - SUPPORT_BASE_HEIGHT + 0.008
    upper_block = (
        cq.Workplane("XY")
        .box(SUPPORT_BODY_THICKNESS, SUPPORT_WIDTH * 0.82, upper_height)
        .edges("|Z")
        .fillet(0.005)
        .edges(">Z")
        .fillet(0.008)
        .translate((0.0, 0.0, SUPPORT_BASE_HEIGHT + upper_height / 2.0 - 0.004))
    )

    bearing_boss = (
        cq.Workplane("YZ")
        .center(0.0, SUPPORT_AXIS_Z)
        .circle(SUPPORT_BOSS_RADIUS)
        .extrude(SUPPORT_BOSS_LENGTH / 2.0, both=True)
    )

    support = lower_block.union(upper_block).union(bearing_boss)

    spindle_bore = (
        cq.Workplane("YZ")
        .center(0.0, SUPPORT_AXIS_Z)
        .circle(SPINDLE_JOURNAL_CLEARANCE_RADIUS)
        .extrude((SUPPORT_BOSS_LENGTH + 0.02) / 2.0, both=True)
    )

    return support.cut(spindle_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_pedestal_spindle_module")

    model.material("cast_iron", rgba=(0.22, 0.23, 0.24, 1.0))
    model.material("machined_steel", rgba=(0.67, 0.70, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_foot_shape(), "heavy_foot"),
        material="cast_iron",
        name="foot",
    )
    base.visual(
        mesh_from_cadquery(_build_support_shape(), "left_support"),
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, FOOT_THICKNESS)),
        material="cast_iron",
        name="left_support",
    )
    base.visual(
        mesh_from_cadquery(_build_support_shape(), "right_support"),
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, FOOT_THICKNESS)),
        material="cast_iron",
        name="right_support",
    )
    base.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS + SUPPORT_HEIGHT)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, (FOOT_THICKNESS + SUPPORT_HEIGHT) / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SHAFT_BODY_LENGTH),
        origin=_axial_origin(SHAFT_BODY_CENTER_X),
        material="machined_steel",
        name="shaft_body",
    )
    spindle.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=_axial_origin(LEFT_COLLAR_CENTER_X),
        material="machined_steel",
        name="left_collar",
    )
    spindle.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=_axial_origin(RIGHT_COLLAR_CENTER_X),
        material="machined_steel",
        name="right_collar",
    )
    spindle.visual(
        Cylinder(radius=FACEPLATE_HUB_RADIUS, length=FACEPLATE_HUB_LENGTH),
        origin=_axial_origin(FACEPLATE_HUB_CENTER_X),
        material="machined_steel",
        name="faceplate_hub",
    )
    spindle.visual(
        Cylinder(radius=FACEPLATE_RADIUS, length=FACEPLATE_THICKNESS),
        origin=_axial_origin(FACEPLATE_CENTER_X),
        material="machined_steel",
        name="faceplate",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((SHAFT_BODY_LENGTH, 2.0 * FACEPLATE_RADIUS, 2.0 * FACEPLATE_RADIUS)),
        mass=8.5,
        origin=Origin(xyz=(SHAFT_BODY_CENTER_X, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spindle = object_model.get_part("spindle")
    spindle_joint = object_model.get_articulation("base_to_spindle")

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
        "spindle_joint_is_continuous",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected CONTINUOUS joint, got {spindle_joint.articulation_type}",
    )
    ctx.check(
        "spindle_axis_runs_between_pedestals",
        spindle_joint.axis == (1.0, 0.0, 0.0),
        details=f"expected axis (1, 0, 0), got {spindle_joint.axis}",
    )

    ctx.expect_contact(
        base,
        spindle,
        elem_a="left_support",
        elem_b="left_collar",
        name="left_pedestal_contacts_left_collar",
    )
    ctx.expect_contact(
        base,
        spindle,
        elem_a="right_support",
        elem_b="right_collar",
        name="right_pedestal_contacts_right_collar",
    )
    ctx.expect_overlap(
        base,
        spindle,
        axes="yz",
        min_overlap=0.04,
        elem_a="left_support",
        elem_b="shaft_body",
        name="left_support_carries_shaft_axis",
    )
    ctx.expect_overlap(
        base,
        spindle,
        axes="yz",
        min_overlap=0.04,
        elem_a="right_support",
        elem_b="shaft_body",
        name="right_support_carries_shaft_axis",
    )
    ctx.expect_gap(
        spindle,
        base,
        axis="x",
        positive_elem="faceplate",
        negative_elem="right_support",
        min_gap=0.03,
        max_gap=0.05,
        name="faceplate_projects_past_right_support",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
