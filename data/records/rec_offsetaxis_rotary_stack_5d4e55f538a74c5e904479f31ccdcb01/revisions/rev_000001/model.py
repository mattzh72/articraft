from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.36
FOOT_WIDTH = 0.22
FOOT_HEIGHT = 0.026

PEDESTAL_RADIUS = 0.068
PEDESTAL_HEIGHT = 0.072
LOWER_SUPPORT_RADIUS = 0.098
LOWER_SUPPORT_HEIGHT = 0.010

LOWER_AXIS_Z = FOOT_HEIGHT + PEDESTAL_HEIGHT + LOWER_SUPPORT_HEIGHT

LOWER_TURNTABLE_RADIUS = 0.120
LOWER_TURNTABLE_HEIGHT = 0.018
LOWER_HUB_RADIUS = 0.046
LOWER_HUB_HEIGHT = 0.034

UPPER_AXIS_X = 0.165
UPPER_AXIS_Z = 0.150

SPINE_X = 0.292
SPINE_WIDTH_X = 0.034
SPINE_WIDTH_Y = 0.070
SPINE_HEIGHT = 0.238

ARM_START_X = 0.170
ARM_END_X = 0.292
LOWER_ARM_BOTTOM_Z = UPPER_AXIS_Z - 0.014
LOWER_ARM_HEIGHT = 0.014
UPPER_BRACE_START_X = 0.216
UPPER_BRACE_END_X = 0.292
UPPER_ARM_BOTTOM_Z = UPPER_AXIS_Z + 0.076
UPPER_ARM_HEIGHT = 0.014

UPPER_FACEPLATE_RADIUS = 0.085
UPPER_FACEPLATE_THICKNESS = 0.014
UPPER_THRUST_RADIUS = 0.024
UPPER_THRUST_HEIGHT = 0.008
UPPER_HUB_RADIUS = 0.034
UPPER_HUB_HEIGHT = 0.070
UPPER_CAP_RADIUS = 0.022
UPPER_CAP_HEIGHT = 0.016


def _build_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )
    foot = (
        foot.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(FOOT_LENGTH - 0.10, FOOT_WIDTH - 0.08, forConstruction=True)
        .vertices()
        .hole(0.012)
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .edges("%CIRCLE")
        .fillet(0.006)
        .translate((0.0, 0.0, FOOT_HEIGHT))
    )

    support = (
        cq.Workplane("XY")
        .circle(LOWER_SUPPORT_RADIUS)
        .extrude(LOWER_SUPPORT_HEIGHT)
        .edges("%CIRCLE")
        .fillet(0.003)
        .translate((0.0, 0.0, FOOT_HEIGHT + PEDESTAL_HEIGHT))
    )

    return foot.union(pedestal).union(support)


def _build_lower_carriage_shape() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .circle(LOWER_TURNTABLE_RADIUS)
        .extrude(LOWER_TURNTABLE_HEIGHT)
        .edges("%CIRCLE")
        .fillet(0.005)
    )
    turntable = (
        turntable.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.082, 0.0, 360.0, 6)
        .hole(0.012)
    )

    hub = (
        cq.Workplane("XY")
        .circle(LOWER_HUB_RADIUS)
        .extrude(LOWER_HUB_HEIGHT)
        .edges("%CIRCLE")
        .fillet(0.004)
    )

    base_block = (
        cq.Workplane("XY")
        .box(
            0.192,
            0.072,
            0.100,
            centered=(False, True, False),
        )
        .translate((0.100, 0.0, LOWER_TURNTABLE_HEIGHT))
        .edges("|Z")
        .fillet(0.006)
    )

    spine = (
        cq.Workplane("XY")
        .box(
            SPINE_WIDTH_X,
            SPINE_WIDTH_Y,
            SPINE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((SPINE_X, 0.0, LOWER_TURNTABLE_HEIGHT))
        .edges("|Z")
        .fillet(0.005)
    )

    lower_arm = (
        cq.Workplane("XY")
        .box(
            ARM_END_X - ARM_START_X,
            0.080,
            LOWER_ARM_HEIGHT,
            centered=(False, True, False),
        )
        .translate((ARM_START_X, 0.0, LOWER_ARM_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.003)
    )
    lower_pad = (
        cq.Workplane("XY")
        .circle(0.034)
        .extrude(LOWER_ARM_HEIGHT)
        .translate((UPPER_AXIS_X, 0.0, LOWER_ARM_BOTTOM_Z))
    )

    upper_arm = (
        cq.Workplane("XY")
        .box(
            UPPER_BRACE_END_X - UPPER_BRACE_START_X,
            0.060,
            UPPER_ARM_HEIGHT,
            centered=(False, True, False),
        )
        .translate((UPPER_BRACE_START_X, 0.0, UPPER_ARM_BOTTOM_Z))
    )
    upper_arm = upper_arm.edges("|Z").fillet(0.003)

    return (
        turntable.union(hub)
        .union(base_block)
        .union(spine)
        .union(lower_arm)
        .union(lower_pad)
        .union(upper_arm)
    )


def _build_upper_faceplate_shape() -> cq.Workplane:
    thrust = cq.Workplane("XY").circle(UPPER_THRUST_RADIUS).extrude(UPPER_THRUST_HEIGHT)

    hub = (
        cq.Workplane("XY")
        .circle(UPPER_HUB_RADIUS)
        .extrude(UPPER_HUB_HEIGHT)
        .edges("%CIRCLE")
        .fillet(0.003)
    )

    faceplate = (
        cq.Workplane("XY")
        .circle(UPPER_FACEPLATE_RADIUS)
        .extrude(UPPER_FACEPLATE_THICKNESS)
        .translate((0.0, 0.0, 0.026))
        .edges("%CIRCLE")
        .fillet(0.003)
    )
    faceplate = (
        faceplate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .polarArray(0.056, 45.0, 360.0, 4)
        .hole(0.018)
    )

    top_cap = (
        cq.Workplane("XY")
        .circle(UPPER_CAP_RADIUS)
        .extrude(UPPER_CAP_HEIGHT)
        .translate((0.0, 0.0, UPPER_HUB_HEIGHT))
        .edges("%CIRCLE")
        .fillet(0.002)
    )

    outer_rim = (
        cq.Workplane("XY")
        .circle(UPPER_FACEPLATE_RADIUS)
        .circle(UPPER_FACEPLATE_RADIUS - 0.012)
        .extrude(0.006)
        .translate((0.0, 0.0, 0.034))
    )

    return thrust.union(hub).union(faceplate).union(top_cap).union(outer_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_module")

    model.material("powder_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("machined_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("anodized_bluegray", rgba=(0.45, 0.52, 0.60, 1.0))

    base = model.part("grounded_foot")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "grounded_foot"),
        material="powder_black",
        name="base_shell",
    )

    lower = model.part("lower_carriage")
    lower.visual(
        mesh_from_cadquery(_build_lower_carriage_shape(), "lower_carriage"),
        material="machined_gray",
        name="lower_shell",
    )

    upper = model.part("upper_faceplate")
    upper.visual(
        mesh_from_cadquery(_build_upper_faceplate_shape(), "upper_faceplate"),
        material="anodized_bluegray",
        name="upper_shell",
    )

    model.articulation(
        "foot_to_lower_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.8,
            lower=-2.4,
            upper=2.4,
        ),
    )

    model.articulation(
        "lower_to_upper_faceplate",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("grounded_foot")
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_faceplate")
    lower_joint = object_model.get_articulation("foot_to_lower_turntable")
    upper_joint = object_model.get_articulation("lower_to_upper_faceplate")

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
        "parallel_revolute_axes",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0) and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"lower axis={lower_joint.axis}, upper axis={upper_joint.axis}",
    )
    ctx.check(
        "distinct_rotary_stages",
        lower_joint.parent == "grounded_foot"
        and lower_joint.child == "lower_carriage"
        and upper_joint.parent == "lower_carriage"
        and upper_joint.child == "upper_faceplate",
        details="expected grounded foot -> lower carriage -> upper faceplate chain",
    )

    ctx.expect_contact(base, lower, contact_tol=1e-6, name="lower_turntable_supported_on_base")
    ctx.expect_contact(lower, upper, contact_tol=1e-6, name="upper_faceplate_supported_by_bracket")
    ctx.expect_origin_gap(
        upper,
        lower,
        axis="x",
        min_gap=0.15,
        max_gap=0.18,
        name="upper_axis_is_offset_from_lower_axis",
    )

    with ctx.pose({lower_joint: 1.2}):
        ctx.expect_origin_distance(
            upper,
            base,
            axes="xy",
            min_dist=0.15,
            max_dist=0.18,
            name="upper_axis_orbits_lower_axis",
        )

    with ctx.pose({lower_joint: 0.8, upper_joint: -1.6}):
        ctx.expect_contact(
            lower,
            upper,
            contact_tol=1e-6,
            name="upper_stage_remains_supported_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
