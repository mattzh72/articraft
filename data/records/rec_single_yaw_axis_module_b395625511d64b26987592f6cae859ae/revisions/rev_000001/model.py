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


BASE_FLANGE_AF = 0.128
BASE_FLANGE_H = 0.010
PEDESTAL_AF = 0.118
PEDESTAL_H = 0.074
PEDESTAL_CAVITY_AF = 0.092
PEDESTAL_CAVITY_H = 0.064
LOWER_RING_R = 0.075
LOWER_RING_H = 0.020
JOINT_Z = BASE_FLANGE_H + PEDESTAL_H + LOWER_RING_H
LOWER_RING_RELIEF_R = 0.042
LOWER_RING_RELIEF_D = 0.002
PILOT_R = 0.020
PILOT_H = 0.008
FIXED_BOLT_R = 0.064
FIXED_BOLT_HEAD_D = 0.010
FIXED_BOLT_HEAD_H = 0.003
FIXED_RING_GROOVE_R = 0.058
FIXED_RING_GROOVE_W = 0.008

STAGE_BORE_R = 0.022
CARRIER_R = 0.056
CARRIER_H = 0.014
TRIM_COVER_R = 0.068
TRIM_COVER_H = 0.004
TOP_DECK_SIZE = 0.176
TOP_DECK_H = 0.018
TOP_DECK_CORNER_R = 0.010
STAGE_TOTAL_H = CARRIER_H + TOP_DECK_H
UNDERSIDE_POCKET_SIZE = 0.128
UNDERSIDE_POCKET_D = 0.010
TOP_RECESS_SIZE = 0.132
TOP_RECESS_D = 0.002
SLOT_OFFSET = 0.040
SLOT_LEN = 0.032
SLOT_W = 0.008
SLOT_D = 0.006
MOVING_BOLT_R = 0.041
MOVING_BOLT_D = 0.009
MOVING_BOLT_DPT = 0.003
CENTER_RECESS_R = 0.026
CENTER_RECESS_D = 0.0015


def _hex_points(across_flats: float, angle_offset: float = math.pi / 6.0) -> list[tuple[float, float]]:
    radius = across_flats / math.sqrt(3.0)
    return [
        (
            radius * math.cos(angle_offset + (math.pi / 3.0) * index),
            radius * math.sin(angle_offset + (math.pi / 3.0) * index),
        )
        for index in range(6)
    ]


def _radial_points(count: int, radius: float, angle_offset: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + (2.0 * math.pi * index / count)),
            radius * math.sin(angle_offset + (2.0 * math.pi * index / count)),
        )
        for index in range(count)
    ]


def _hex_prism(across_flats: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(_hex_points(across_flats)).close().extrude(height)


def _cylinder(radius: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _annulus(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _build_base_shape() -> cq.Workplane:
    body = _hex_prism(BASE_FLANGE_AF, BASE_FLANGE_H)
    body = body.union(_hex_prism(PEDESTAL_AF, PEDESTAL_H).translate((0.0, 0.0, BASE_FLANGE_H)))
    body = body.union(_cylinder(LOWER_RING_R, LOWER_RING_H, BASE_FLANGE_H + PEDESTAL_H))
    body = body.cut(
        _hex_prism(PEDESTAL_CAVITY_AF, PEDESTAL_CAVITY_H).translate((0.0, 0.0, BASE_FLANGE_H))
    )
    body = body.cut(
        _cylinder(
            LOWER_RING_RELIEF_R,
            LOWER_RING_RELIEF_D + 0.0005,
            JOINT_Z - LOWER_RING_RELIEF_D,
        )
    )
    body = body.cut(
        _cylinder(PILOT_R + 0.0015, PILOT_H, JOINT_Z - PILOT_H)
    )
    body = body.cut(
        _annulus(
            FIXED_RING_GROOVE_R + (FIXED_RING_GROOVE_W / 2.0),
            FIXED_RING_GROOVE_R - (FIXED_RING_GROOVE_W / 2.0),
            0.0015,
            JOINT_Z - 0.0015,
        )
    )

    fixed_bolt_counterbores = (
        cq.Workplane("XY")
        .pushPoints(_radial_points(6, FIXED_BOLT_R, math.pi / 6.0))
        .circle(FIXED_BOLT_HEAD_D / 2.0)
        .extrude(FIXED_BOLT_HEAD_H + 0.0005)
        .translate((0.0, 0.0, JOINT_Z - FIXED_BOLT_HEAD_H))
    )
    body = body.cut(fixed_bolt_counterbores)
    return body


def _build_stage_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(TOP_DECK_SIZE, TOP_DECK_SIZE, TOP_DECK_H, centered=(True, True, False))
        .translate((0.0, 0.0, CARRIER_H))
        .edges("|Z")
        .fillet(TOP_DECK_CORNER_R)
    )

    body = _annulus(CARRIER_R, STAGE_BORE_R, CARRIER_H, 0.0)
    body = body.union(_cylinder(TRIM_COVER_R, TRIM_COVER_H, CARRIER_H - TRIM_COVER_H))
    body = body.union(deck)
    body = body.cut(
        cq.Workplane("XY")
        .box(
            UNDERSIDE_POCKET_SIZE,
            UNDERSIDE_POCKET_SIZE,
            UNDERSIDE_POCKET_D,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIER_H))
    )
    body = body.cut(
        cq.Workplane("XY")
        .box(TOP_RECESS_SIZE, TOP_RECESS_SIZE, TOP_RECESS_D, centered=(True, True, False))
        .translate((0.0, 0.0, STAGE_TOTAL_H - TOP_RECESS_D))
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(((0.0, SLOT_OFFSET), (0.0, -SLOT_OFFSET)))
        .slot2D(SLOT_LEN, SLOT_W, angle=0)
        .cutBlind(-SLOT_D)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(((SLOT_OFFSET, 0.0), (-SLOT_OFFSET, 0.0)))
        .slot2D(SLOT_LEN, SLOT_W, angle=90)
        .cutBlind(-SLOT_D)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_radial_points(8, MOVING_BOLT_R, math.pi / 8.0))
        .circle(MOVING_BOLT_D / 2.0)
        .cutBlind(-MOVING_BOLT_DPT)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(CENTER_RECESS_R)
        .cutBlind(-CENTER_RECESS_D)
    )
    body = body.cut(_cylinder(STAGE_BORE_R, STAGE_TOTAL_H + 0.002, -0.001))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pan_slewing_base")

    model.material("base_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("stage_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base"),
        material="base_graphite",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=LOWER_RING_R, length=JOINT_Z),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z / 2.0)),
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_cadquery(_build_stage_shape(), "top_stage"),
        material="stage_aluminum",
        name="stage_shell",
    )
    top_stage.inertial = Inertial.from_geometry(
        Box((TOP_DECK_SIZE, TOP_DECK_SIZE, STAGE_TOTAL_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, STAGE_TOTAL_H / 2.0)),
    )

    model.articulation(
        "base_to_top_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.0,
            lower=-3.0,
            upper=3.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_stage = object_model.get_part("top_stage")
    yaw = object_model.get_articulation("base_to_top_stage")
    yaw_limits = yaw.motion_limits

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
        "yaw_joint_is_vertical_revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "yaw_joint_has_realistic_range",
        yaw_limits is not None
        and yaw_limits.lower is not None
        and yaw_limits.upper is not None
        and yaw_limits.lower <= -2.9
        and yaw_limits.upper >= 2.9,
        f"limits={yaw_limits}",
    )
    ctx.expect_origin_distance(
        base,
        top_stage,
        axes="xy",
        max_dist=1e-6,
        name="stage_is_coaxial_with_base",
    )
    ctx.expect_origin_gap(
        top_stage,
        base,
        axis="z",
        min_gap=JOINT_Z - 1e-4,
        max_gap=JOINT_Z + 1e-4,
        name="stage_origin_sits_at_bearing_height",
    )
    ctx.expect_contact(
        top_stage,
        base,
        name="stage_is_seated_on_the_bearing_interface",
    )
    ctx.expect_overlap(
        top_stage,
        base,
        axes="xy",
        min_overlap=0.11,
        name="stage_projects_over_the_turntable_support",
    )

    with ctx.pose({yaw: math.pi / 4.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_clipping_at_45_degree_yaw")
        ctx.expect_contact(
            top_stage,
            base,
            name="bearing_contact_is_retained_at_45_degree_yaw",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
