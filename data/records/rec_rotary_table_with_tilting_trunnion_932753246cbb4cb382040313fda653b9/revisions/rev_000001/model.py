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


FOOT_LENGTH = 0.82
FOOT_WIDTH = 0.48
FOOT_HEIGHT = 0.06

PEDESTAL_LENGTH = 0.28
PEDESTAL_WIDTH = 0.24
PEDESTAL_HEIGHT = 0.44
BEARING_CAP_RADIUS = 0.18
BEARING_CAP_HEIGHT = 0.04
BASE_TOP_Z = FOOT_HEIGHT + PEDESTAL_HEIGHT + BEARING_CAP_HEIGHT

PLATFORM_RADIUS = 0.24
PLATFORM_THICKNESS = 0.035
PLATFORM_HUB_RADIUS = 0.115
PLATFORM_HUB_HEIGHT = 0.045
GEARBOX_LENGTH = 0.15
GEARBOX_WIDTH = 0.10
GEARBOX_HEIGHT = 0.05

TRUNNION_AXIS_Z = 0.255
SUPPORT_CENTER_X = 0.19
SUPPORT_THICKNESS = 0.030
SUPPORT_HOLE_RADIUS = 0.031
SUPPORT_AXIS_LOCAL_Z = TRUNNION_AXIS_Z - PLATFORM_THICKNESS

FACEPLATE_RADIUS = 0.155
FACEPLATE_THICKNESS = 0.028
FACEPLATE_HUB_RADIUS = 0.050
FACEPLATE_HUB_LENGTH = 0.120
JOURNAL_RADIUS = SUPPORT_HOLE_RADIUS
INNER_SUPPORT_FACE_X = SUPPORT_CENTER_X - (SUPPORT_THICKNESS / 2.0)
OUTER_SUPPORT_FACE_X = SUPPORT_CENTER_X + (SUPPORT_THICKNESS / 2.0)
TRUNNION_SPAN = OUTER_SUPPORT_FACE_X * 2.0
JOURNAL_CENTER_X = (INNER_SUPPORT_FACE_X + OUTER_SUPPORT_FACE_X) / 2.0


def _build_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT)
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, 0.0, FOOT_HEIGHT / 2.0))
    )

    anchor_pad = (
        cq.Workplane("XY")
        .box(0.13, 0.10, 0.016)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, 0.008))
    )
    foot = (
        foot.union(anchor_pad.translate((0.28, 0.16, 0.0)))
        .union(anchor_pad.translate((0.28, -0.16, 0.0)))
        .union(anchor_pad.translate((-0.28, 0.16, 0.0)))
        .union(anchor_pad.translate((-0.28, -0.16, 0.0)))
    )

    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT)
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.0, FOOT_HEIGHT + (PEDESTAL_HEIGHT / 2.0)))
    )

    cap = cq.Workplane("XY").circle(BEARING_CAP_RADIUS).extrude(BEARING_CAP_HEIGHT).translate(
        (0.0, 0.0, FOOT_HEIGHT + PEDESTAL_HEIGHT)
    )

    return foot.union(pedestal).union(cap)


def _build_support_shape() -> cq.Workplane:
    cheek = (
        cq.Workplane("YZ")
        .moveTo(-0.092, 0.0)
        .lineTo(-0.118, 0.0)
        .lineTo(-0.118, 0.046)
        .lineTo(-0.078, 0.118)
        .lineTo(-0.078, SUPPORT_AXIS_LOCAL_Z + 0.070)
        .threePointArc((0.0, SUPPORT_AXIS_LOCAL_Z + 0.116), (0.078, SUPPORT_AXIS_LOCAL_Z + 0.070))
        .lineTo(0.078, 0.118)
        .lineTo(0.118, 0.046)
        .lineTo(0.118, 0.0)
        .lineTo(0.092, 0.0)
        .close()
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
    )

    hole = (
        cq.Workplane("YZ")
        .center(0.0, SUPPORT_AXIS_LOCAL_Z)
        .circle(SUPPORT_HOLE_RADIUS)
        .extrude(SUPPORT_THICKNESS, both=True)
    )
    return cheek.cut(hole)


def _build_lower_platform_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").circle(PLATFORM_RADIUS).extrude(PLATFORM_THICKNESS)
    hub = cq.Workplane("XY").circle(PLATFORM_HUB_RADIUS).extrude(PLATFORM_HUB_HEIGHT).translate(
        (0.0, 0.0, PLATFORM_THICKNESS)
    )

    gearbox = (
        cq.Workplane("XY")
        .box(GEARBOX_LENGTH, GEARBOX_WIDTH, GEARBOX_HEIGHT)
        .edges("|Z")
        .fillet(0.010)
        .translate(
            (
                0.0,
                -0.115,
                PLATFORM_THICKNESS + (GEARBOX_HEIGHT / 2.0),
            )
        )
    )

    return deck.union(hub).union(gearbox)


def _build_faceplate_shape() -> cq.Workplane:
    plate = cq.Workplane("YZ").circle(FACEPLATE_RADIUS).extrude(FACEPLATE_THICKNESS / 2.0, both=True)

    bolt_radius = 0.102
    bolt_points = [
        (bolt_radius * math.cos(angle), bolt_radius * math.sin(angle))
        for angle in [i * (math.pi / 4.0) for i in range(8)]
    ]
    bolt_holes = (
        cq.Workplane("YZ")
        .pushPoints(bolt_points)
        .circle(0.013)
        .extrude(FACEPLATE_THICKNESS * 2.0, both=True)
    )
    plate = plate.cut(bolt_holes)

    hub = cq.Workplane("YZ").circle(FACEPLATE_HUB_RADIUS).extrude(INNER_SUPPORT_FACE_X, both=True)
    left_journal = (
        cq.Workplane("YZ")
        .circle(JOURNAL_RADIUS)
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
        .translate((JOURNAL_CENTER_X, 0.0, 0.0))
    )
    right_journal = (
        cq.Workplane("YZ")
        .circle(JOURNAL_RADIUS)
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
        .translate((-JOURNAL_CENTER_X, 0.0, 0.0))
    )

    return plate.union(hub).union(left_journal).union(right_journal)


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-9 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_trunnion_positioner")

    model.material("machine_blue", rgba=(0.23, 0.31, 0.40, 1.0))
    model.material("signal_orange", rgba=(0.83, 0.40, 0.12, 1.0))
    model.material("steel_gray", rgba=(0.70, 0.72, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base"),
        material="machine_blue",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, BASE_TOP_Z)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z / 2.0)),
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        mesh_from_cadquery(_build_lower_platform_shape(), "lower_platform"),
        material="signal_orange",
        name="lower_platform_shell",
    )
    lower_platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_RADIUS * 2.0, PLATFORM_RADIUS * 2.0, 0.40)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    left_support = model.part("left_trunnion_support")
    left_support.visual(
        mesh_from_cadquery(_build_support_shape(), "left_trunnion_support"),
        material="machine_blue",
        name="left_support_shell",
    )
    left_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_THICKNESS, 0.24, TRUNNION_AXIS_Z + 0.14)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (TRUNNION_AXIS_Z + 0.14) / 2.0)),
    )

    right_support = model.part("right_trunnion_support")
    right_support.visual(
        mesh_from_cadquery(_build_support_shape(), "right_trunnion_support"),
        material="machine_blue",
        name="right_support_shell",
    )
    right_support.inertial = Inertial.from_geometry(
        Box((SUPPORT_THICKNESS, 0.24, TRUNNION_AXIS_Z + 0.14)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (TRUNNION_AXIS_Z + 0.14) / 2.0)),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        mesh_from_cadquery(_build_faceplate_shape(), "faceplate"),
        material="steel_gray",
        name="faceplate_shell",
    )
    faceplate.inertial = Inertial.from_geometry(
        Box((TRUNNION_SPAN, FACEPLATE_RADIUS * 2.0, FACEPLATE_RADIUS * 2.0)),
        mass=42.0,
    )

    model.articulation(
        "base_to_lower_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_platform,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "lower_platform_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=lower_platform,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "lower_platform_to_left_support",
        ArticulationType.FIXED,
        parent=lower_platform,
        child=left_support,
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, PLATFORM_THICKNESS)),
    )
    model.articulation(
        "lower_platform_to_right_support",
        ArticulationType.FIXED,
        parent=lower_platform,
        child=right_support,
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, PLATFORM_THICKNESS)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_platform = object_model.get_part("lower_platform")
    left_support = object_model.get_part("left_trunnion_support")
    right_support = object_model.get_part("right_trunnion_support")
    faceplate = object_model.get_part("faceplate")
    yaw = object_model.get_articulation("base_to_lower_platform")
    tilt = object_model.get_articulation("lower_platform_to_faceplate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        faceplate,
        left_support,
        reason="Trunnion journals are intentionally modeled as a captive bearing interface inside the left support bore.",
    )
    ctx.allow_overlap(
        faceplate,
        right_support,
        reason="Trunnion journals are intentionally modeled as a captive bearing interface inside the right support bore.",
    )

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
        "expected_parts_present",
        all(part is not None for part in (base, lower_platform, left_support, right_support, faceplate)),
        "Base, lower platform, both trunnion supports, and the faceplate must all be authored as separate parts.",
    )
    ctx.check(
        "vertical_yaw_axis_configured",
        _axis_matches(yaw.axis, (0.0, 0.0, 1.0))
        and yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower < 0.0 < yaw.motion_limits.upper,
        "The lower platform should revolve about the vertical Z axis with bidirectional travel.",
    )
    ctx.check(
        "horizontal_trunnion_axis_configured",
        _axis_matches(tilt.axis, (1.0, 0.0, 0.0))
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        "The faceplate should revolve about the horizontal X axis between the trunnion supports.",
    )

    with ctx.pose({yaw: 0.0, tilt: 0.0}):
        ctx.expect_contact(
            lower_platform,
            base,
            contact_tol=0.0015,
            name="turntable_seats_on_pedestal_cap",
        )
        ctx.expect_contact(
            left_support,
            lower_platform,
            contact_tol=0.0015,
            name="left_support_is_bolted_to_platform",
        )
        ctx.expect_contact(
            right_support,
            lower_platform,
            contact_tol=0.0015,
            name="right_support_is_bolted_to_platform",
        )
        ctx.expect_contact(
            faceplate,
            left_support,
            contact_tol=0.0015,
            name="faceplate_contacts_left_trunnion_support",
        )
        ctx.expect_contact(
            faceplate,
            right_support,
            contact_tol=0.0015,
            name="faceplate_contacts_right_trunnion_support",
        )
        ctx.expect_gap(
            faceplate,
            base,
            axis="z",
            min_gap=0.08,
            name="faceplate_clears_fixed_pedestal",
        )

    with ctx.pose({yaw: 0.75, tilt: 1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_combined_rotated_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
