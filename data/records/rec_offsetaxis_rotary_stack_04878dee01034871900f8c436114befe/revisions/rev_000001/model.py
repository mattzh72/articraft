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


BASE_L = 0.260
BASE_W = 0.220
BASE_H = 0.046

PLATFORM_R = 0.150
PLATFORM_HUB_R = 0.056
PLATFORM_HUB_H = 0.018
PLATFORM_TABLE_T = 0.026
PLATFORM_TOP_Z = PLATFORM_HUB_H + PLATFORM_TABLE_T

BRIDGE_MOUNT_X = -0.115
HEAD_AXIS_X = 0.160
HEAD_AXIS_Z = 0.130


def _make_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.016)
    )


def _make_platform_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(PLATFORM_HUB_R).extrude(PLATFORM_HUB_H)
    table = (
        cq.Workplane("XY")
        .circle(PLATFORM_R)
        .extrude(PLATFORM_TABLE_T)
        .translate((0.0, 0.0, PLATFORM_HUB_H))
    )
    body = hub.union(table)
    return (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.104)
        .cutBlind(-0.005)
    )


def _make_bridge_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.056, 0.094, 0.012, centered=(True, True, False))
    column = (
        cq.Workplane("XY")
        .box(0.026, 0.070, 0.160, centered=(True, True, False))
        .translate((0.0, 0.0, 0.012))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.145, 0.070, 0.030, centered=(False, True, False))
        .translate((-0.005, 0.0, 0.118))
    )
    web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.013, 0.012),
                (0.045, 0.012),
                (0.132, 0.118),
                (-0.005, 0.118),
            ]
        )
        .close()
        .extrude(0.016, both=True)
    )
    support_plate = (
        cq.Workplane("XY")
        .box(0.022, 0.072, 0.094, centered=(True, True, True))
        .translate((HEAD_AXIS_X - 0.011, 0.0, HEAD_AXIS_Z))
    )
    return foot.union(column).union(beam).union(web).union(support_plate)


def _make_head_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, 0.031)
        .lineTo(0.012, 0.031)
        .lineTo(0.018, 0.045)
        .lineTo(0.044, 0.045)
        .lineTo(0.050, 0.052)
        .lineTo(0.058, 0.052)
        .lineTo(0.058, 0.038)
        .lineTo(0.062, 0.038)
        .lineTo(0.068, 0.018)
        .lineTo(0.076, 0.018)
        .lineTo(0.076, 0.0)
        .close()
        .revolve(360, (0.0, 0.0), (1.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_fixture")

    base_mat = model.material("base_cast", rgba=(0.28, 0.29, 0.31, 1.0))
    platform_mat = model.material("platform_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    bridge_mat = model.material("bridge_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    head_mat = model.material("head_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "fixture_base"),
        material=base_mat,
        name="base_body",
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        mesh_from_cadquery(_make_platform_shape(), "fixture_platform"),
        material=platform_mat,
        name="platform_body",
    )

    bridge_arm = model.part("bridge_arm")
    bridge_arm.visual(
        mesh_from_cadquery(_make_bridge_shape(), "fixture_bridge"),
        material=bridge_mat,
        name="bridge_body",
    )

    rotary_head = model.part("rotary_head")
    rotary_head.visual(
        mesh_from_cadquery(_make_head_shape(), "fixture_head"),
        material=head_mat,
        name="head_body",
    )

    model.articulation(
        "base_to_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_platform,
        origin=Origin(xyz=(0.0, 0.0, BASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "platform_to_bridge",
        ArticulationType.FIXED,
        parent=lower_platform,
        child=bridge_arm,
        origin=Origin(xyz=(BRIDGE_MOUNT_X, 0.0, PLATFORM_TOP_Z)),
    )

    model.articulation(
        "bridge_to_head",
        ArticulationType.REVOLUTE,
        parent=bridge_arm,
        child=rotary_head,
        origin=Origin(xyz=(HEAD_AXIS_X, 0.0, HEAD_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_platform = object_model.get_part("lower_platform")
    bridge_arm = object_model.get_part("bridge_arm")
    rotary_head = object_model.get_part("rotary_head")
    base_to_platform = object_model.get_articulation("base_to_platform")
    bridge_to_head = object_model.get_articulation("bridge_to_head")

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

    ctx.expect_contact(lower_platform, base, name="platform seats on base turntable")
    ctx.expect_contact(bridge_arm, lower_platform, name="bridge foot mounts on platform")
    ctx.expect_contact(rotary_head, bridge_arm, name="head mounts against the offset bridge support")
    ctx.expect_overlap(
        lower_platform,
        base,
        axes="xy",
        min_overlap=0.18,
        name="platform remains broadly centered over base",
    )

    with ctx.pose({base_to_platform: 0.0}):
        bridge_rest = ctx.part_world_position(bridge_arm)
    with ctx.pose({base_to_platform: math.pi / 2.0}):
        bridge_quarter_turn = ctx.part_world_position(bridge_arm)
        ctx.expect_contact(
            lower_platform,
            base,
            name="platform stays seated through quarter turn",
        )
        ctx.expect_contact(
            rotary_head,
            bridge_arm,
            name="head remains mounted while platform turns",
        )

    bridge_swings_about_z = (
        bridge_rest is not None
        and bridge_quarter_turn is not None
        and bridge_rest[0] < -0.08
        and abs(bridge_rest[1]) < 0.015
        and abs(bridge_quarter_turn[0]) < 0.04
        and bridge_quarter_turn[1] < -0.08
    )
    ctx.check(
        "platform joint rotates the edge bridge around vertical axis",
        bridge_swings_about_z,
        details=(
            f"rest={bridge_rest}, quarter_turn={bridge_quarter_turn}; "
            "expected the bridge mount to move from the -X edge toward -Y."
        ),
    )

    with ctx.pose({bridge_to_head: 0.0}):
        head_rest_aabb = ctx.part_world_aabb(rotary_head)
    with ctx.pose({bridge_to_head: 1.0}):
        head_tilted_aabb = ctx.part_world_aabb(rotary_head)
        ctx.expect_contact(
            rotary_head,
            bridge_arm,
            name="head stays supported when tilted upward",
        )

    head_lifts_on_positive_rotation = (
        head_rest_aabb is not None
        and head_tilted_aabb is not None
        and head_tilted_aabb[1][2] > head_rest_aabb[1][2] + 0.025
    )
    ctx.check(
        "positive head rotation lifts the offset faceplate",
        head_lifts_on_positive_rotation,
        details=(
            f"rest_aabb={head_rest_aabb}, tilted_aabb={head_tilted_aabb}; "
            "expected the forward head mass to swing upward around the bridge trunnion."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
