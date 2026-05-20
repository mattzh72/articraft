from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FOOT_RADIUS = 0.085
FOOT_THICKNESS = 0.014

PEDESTAL_HEIGHT = 0.090
PEDESTAL_BOTTOM_DIAMETER = 0.112
PEDESTAL_TOP_DIAMETER = 0.082

RING_OUTER_RADIUS = 0.088
RING_INNER_RADIUS = 0.048
LOWER_RING_HEIGHT = 0.024
UPPER_RING_HEIGHT = 0.016

HUB_RADIUS = 0.030
HUB_HEIGHT = 0.010

DECK_SIZE = 0.190
DECK_THICKNESS = 0.012
DECK_CORNER_RADIUS = 0.010
CENTER_HOLE_DIAMETER = 0.026
MOUNT_HOLE_DIAMETER = 0.007
MOUNT_PATTERN_OFFSET = 0.055

YAW_ORIGIN_Z = FOOT_THICKNESS + PEDESTAL_HEIGHT + LOWER_RING_HEIGHT


def make_annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def make_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(FOOT_RADIUS).extrude(FOOT_THICKNESS)

    pedestal = (
        cq.Workplane("XY")
        .polygon(6, PEDESTAL_BOTTOM_DIAMETER)
        .workplane(offset=PEDESTAL_HEIGHT)
        .polygon(6, PEDESTAL_TOP_DIAMETER)
        .loft(combine=True)
        .translate((0.0, 0.0, FOOT_THICKNESS))
    )

    lower_ring = make_annulus(
        outer_radius=RING_OUTER_RADIUS,
        inner_radius=RING_INNER_RADIUS,
        height=LOWER_RING_HEIGHT,
    ).translate((0.0, 0.0, FOOT_THICKNESS + PEDESTAL_HEIGHT))

    return foot.union(pedestal).union(lower_ring)


def make_stage_shape() -> cq.Workplane:
    upper_ring = make_annulus(
        outer_radius=RING_OUTER_RADIUS,
        inner_radius=RING_INNER_RADIUS,
        height=UPPER_RING_HEIGHT,
    )

    hub = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS)
        .extrude(HUB_HEIGHT)
        .translate((0.0, 0.0, UPPER_RING_HEIGHT))
    )

    deck = cq.Workplane("XY").rect(DECK_SIZE, DECK_SIZE).extrude(DECK_THICKNESS)
    deck = deck.edges("|Z").fillet(DECK_CORNER_RADIUS)
    deck = deck.faces(">Z").workplane(centerOption="CenterOfMass").hole(CENTER_HOLE_DIAMETER)
    deck = (
        deck.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_PATTERN_OFFSET, -MOUNT_PATTERN_OFFSET),
                (-MOUNT_PATTERN_OFFSET, MOUNT_PATTERN_OFFSET),
                (MOUNT_PATTERN_OFFSET, -MOUNT_PATTERN_OFFSET),
                (MOUNT_PATTERN_OFFSET, MOUNT_PATTERN_OFFSET),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
        .translate((0.0, 0.0, UPPER_RING_HEIGHT + HUB_HEIGHT))
    )

    return upper_ring.union(hub).union(deck)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_base_turntable")

    support_material = model.material(
        "support_anodized",
        rgba=(0.18, 0.20, 0.23, 1.0),
    )
    stage_material = model.material(
        "stage_aluminum",
        rgba=(0.74, 0.77, 0.80, 1.0),
    )

    base_support = model.part("base_support")
    base_support.visual(
        mesh_from_cadquery(make_base_shape(), "base_support"),
        origin=Origin(),
        material=support_material,
        name="support_shell",
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_cadquery(make_stage_shape(), "top_stage"),
        origin=Origin(),
        material=stage_material,
        name="stage_shell",
    )

    model.articulation(
        "support_to_stage_yaw",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    top_stage = object_model.get_part("top_stage")
    yaw_joint = object_model.get_articulation("support_to_stage_yaw")
    yaw_limits = yaw_joint.motion_limits

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
        base_support.name == "base_support" and top_stage.name == "top_stage",
        "Expected static base support and moving top stage parts.",
    )
    ctx.check(
        "yaw_joint_is_vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical yaw axis, got {yaw_joint.axis}.",
    )
    ctx.check(
        "yaw_joint_limits_are_full_pan",
        yaw_limits is not None
        and yaw_limits.lower is not None
        and yaw_limits.upper is not None
        and abs(yaw_limits.lower + pi) < 1e-6
        and abs(yaw_limits.upper - pi) < 1e-6,
        f"Expected ±pi yaw limits, got {yaw_limits}.",
    )
    ctx.expect_origin_distance(
        top_stage,
        base_support,
        axes="xy",
        max_dist=1e-6,
        name="stage_is_coaxial_with_support",
    )
    ctx.expect_origin_gap(
        top_stage,
        base_support,
        axis="z",
        min_gap=YAW_ORIGIN_Z,
        max_gap=YAW_ORIGIN_Z,
        name="yaw_origin_sits_above_static_support",
    )
    ctx.expect_gap(
        top_stage,
        base_support,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        name="stage_seats_on_turntable_ring",
    )
    ctx.expect_contact(
        top_stage,
        base_support,
        contact_tol=0.001,
        name="stage_is_physically_supported",
    )
    ctx.expect_overlap(
        top_stage,
        base_support,
        axes="xy",
        min_overlap=0.090,
        name="moving_stage_stays_over_base_footprint",
    )

    with ctx.pose({yaw_joint: pi / 2.0}):
        ctx.expect_gap(
            top_stage,
            base_support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            name="stage_keeps_bearing_seat_at_quarter_turn",
        )
        ctx.expect_contact(
            top_stage,
            base_support,
            contact_tol=0.001,
            name="stage_remains_supported_while_yawing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
