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


BEZEL_HEIGHT = 0.026
BEZEL_OUTER_RADIUS = 0.041
BEZEL_INNER_RADIUS = 0.025
PIVOT_Z = 0.016
OUTER_LIMIT = 0.48
INNER_LIMIT = 0.48


def _octagon_points(radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((math.pi / 8.0) + (i * math.pi / 4.0)),
            radius * math.sin((math.pi / 8.0) + (i * math.pi / 4.0)),
        )
        for i in range(8)
    ]


def _octagon_prism(radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(_octagon_points(radius)).close().extrude(height)


def _build_bezel() -> cq.Workplane:
    upper_ring = _octagon_prism(BEZEL_OUTER_RADIUS, 0.012).cut(
        _octagon_prism(BEZEL_INNER_RADIUS, 0.012)
    ).translate((0.0, 0.0, 0.014))

    lower_ring = _octagon_prism(BEZEL_OUTER_RADIUS * 0.90, 0.006).cut(
        _octagon_prism(BEZEL_INNER_RADIUS * 1.06, 0.006)
    ).translate((0.0, 0.0, 0.008))

    tower_thickness = 0.005
    tower_width = 0.018
    tower_height = 0.014
    tower_center_x = 0.0225
    tower_center_z = PIVOT_Z

    left_tower = cq.Workplane("XY").box(
        tower_thickness, tower_width, tower_height
    ).translate((-tower_center_x, 0.0, tower_center_z))
    right_tower = cq.Workplane("XY").box(
        tower_thickness, tower_width, tower_height
    ).translate((tower_center_x, 0.0, tower_center_z))

    left_rib = cq.Workplane("XY").box(0.010, 0.006, 0.008).translate(
        (-0.028, 0.0, 0.014)
    )
    right_rib = cq.Workplane("XY").box(0.010, 0.006, 0.008).translate(
        (0.028, 0.0, 0.014)
    )

    return (
        upper_ring.union(lower_ring)
        .union(left_tower)
        .union(right_tower)
        .union(left_rib)
        .union(right_rib)
    )


def _build_outer_fork() -> cq.Workplane:
    cheek_x = 0.018
    cheek_t = 0.003
    cheek_z = 0.016
    cheek_center_y = 0.0105
    cheek_center_z = -0.001

    left_cheek = cq.Workplane("XY").box(cheek_x, cheek_t, cheek_z).translate(
        (0.0, -cheek_center_y, cheek_center_z)
    )
    right_cheek = cq.Workplane("XY").box(cheek_x, cheek_t, cheek_z).translate(
        (0.0, cheek_center_y, cheek_center_z)
    )

    lower_bridge = cq.Workplane("XY").box(0.016, 0.021, 0.003).translate(
        (0.0, 0.0, -0.0135)
    )
    left_pivot = (
        cq.Workplane("YZ")
        .circle(0.0025)
        .extrude(0.003, both=True)
        .translate((-0.017, 0.0, 0.0))
    )
    right_pivot = (
        cq.Workplane("YZ")
        .circle(0.0025)
        .extrude(0.003, both=True)
        .translate((0.017, 0.0, 0.0))
    )

    return (
        left_cheek.union(right_cheek)
        .union(lower_bridge)
        .union(left_pivot)
        .union(right_pivot)
    )


def _build_inner_ring() -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .circle(0.0095)
        .circle(0.0068)
        .extrude(0.0012, both=True)
    )

    left_trunnion = (
        cq.Workplane("XZ")
        .circle(0.0022)
        .extrude(0.004, both=True)
        .translate((0.0, -0.0052, 0.0))
    )
    right_trunnion = (
        cq.Workplane("XZ")
        .circle(0.0022)
        .extrude(0.004, both=True)
        .translate((0.0, 0.0052, 0.0))
    )

    x_web = cq.Workplane("XY").box(0.0145, 0.0024, 0.002).translate((0.0, 0.0, 0.001))
    z_web = cq.Workplane("XY").box(0.0024, 0.0024, 0.0145).translate((0.0, 0.0, 0.001))
    hub = cq.Workplane("XY").circle(0.0038).extrude(0.010)

    return ring.union(left_trunnion).union(right_trunnion).union(x_web).union(z_web).union(hub)


def _build_stick() -> cq.Workplane:
    mount_pad = cq.Workplane("XY").circle(0.0038).extrude(0.001)
    stem = cq.Workplane("XY").circle(0.0034).extrude(0.006).translate((0.0, 0.0, 0.001))
    grip = cq.Workplane("XY").circle(0.0054).extrude(0.0055).translate((0.0, 0.0, 0.007))
    cap = cq.Workplane("XY").circle(0.0046).extrude(0.0025).translate((0.0, 0.0, 0.0125))
    return mount_pad.union(stem).union(grip).union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guarded_thumb_stick")

    bezel_mat = model.material("bezel_finish", color=(0.23, 0.24, 0.26, 1.0))
    fork_mat = model.material("fork_finish", color=(0.42, 0.43, 0.45, 1.0))
    ring_mat = model.material("ring_finish", color=(0.16, 0.17, 0.18, 1.0))
    stick_mat = model.material("stick_finish", color=(0.08, 0.08, 0.09, 1.0))

    bezel = model.part("bezel")
    bezel.visual(
        mesh_from_cadquery(_build_bezel(), "thumb_stick_bezel"),
        material=bezel_mat,
        name="bezel_shell",
    )

    outer_fork = model.part("outer_fork")
    outer_fork.visual(
        mesh_from_cadquery(_build_outer_fork(), "thumb_stick_outer_fork"),
        material=fork_mat,
        name="outer_fork_shell",
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        mesh_from_cadquery(_build_inner_ring(), "thumb_stick_inner_ring"),
        material=ring_mat,
        name="inner_ring_shell",
    )

    stick = model.part("control_post")
    stick.visual(
        mesh_from_cadquery(_build_stick(), "thumb_stick_control_post"),
        material=stick_mat,
        name="control_post_shell",
    )

    model.articulation(
        "bezel_to_outer_fork",
        ArticulationType.REVOLUTE,
        parent=bezel,
        child=outer_fork,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-OUTER_LIMIT,
            upper=OUTER_LIMIT,
        ),
    )

    model.articulation(
        "outer_fork_to_inner_ring",
        ArticulationType.REVOLUTE,
        parent=outer_fork,
        child=inner_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-INNER_LIMIT,
            upper=INNER_LIMIT,
        ),
    )

    model.articulation(
        "inner_ring_to_control_post",
        ArticulationType.FIXED,
        parent=inner_ring,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bezel = object_model.get_part("bezel")
    outer_fork = object_model.get_part("outer_fork")
    inner_ring = object_model.get_part("inner_ring")
    control_post = object_model.get_part("control_post")
    fork_joint = object_model.get_articulation("bezel_to_outer_fork")
    ring_joint = object_model.get_articulation("outer_fork_to_inner_ring")

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
    ctx.allow_overlap(
        bezel,
        outer_fork,
        reason=(
            "Outer fork trunnions are represented as captured hinge pins inside "
            "solid bezel bearing towers in this compact console control."
        ),
    )
    ctx.allow_overlap(
        outer_fork,
        inner_ring,
        reason=(
            "Inner ring trunnions are represented as captured pivots inside the "
            "fork cheeks, with bearing bores left implicit."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_primary_parts_present",
        all(part is not None for part in (bezel, outer_fork, inner_ring, control_post)),
        "Missing one or more required thumb-stick parts.",
    )
    ctx.check(
        "outer_fork_uses_horizontal_x_axis",
        tuple(fork_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected outer fork axis (1, 0, 0), got {fork_joint.axis!r}.",
    )
    ctx.check(
        "inner_ring_uses_orthogonal_horizontal_y_axis",
        tuple(ring_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected inner ring axis (0, 1, 0), got {ring_joint.axis!r}.",
    )
    ctx.check(
        "compact_motion_limits",
        fork_joint.motion_limits is not None
        and ring_joint.motion_limits is not None
        and fork_joint.motion_limits.lower == -OUTER_LIMIT
        and fork_joint.motion_limits.upper == OUTER_LIMIT
        and ring_joint.motion_limits.lower == -INNER_LIMIT
        and ring_joint.motion_limits.upper == INNER_LIMIT,
        "Expected compact symmetric gimbal limits for both articulated axes.",
    )

    ctx.expect_within(
        outer_fork,
        bezel,
        axes="xy",
        margin=0.0,
        name="outer_fork_stays_inside_bezel_guard",
    )
    ctx.expect_within(
        inner_ring,
        outer_fork,
        axes="xy",
        margin=0.0,
        name="inner_ring_stays_inside_outer_fork_envelope",
    )
    ctx.expect_contact(inner_ring, control_post, name="control_post_is_seated_on_inner_ring")

    with ctx.pose({fork_joint: 0.34, ring_joint: -0.31}):
        ctx.expect_within(
            outer_fork,
            bezel,
            axes="xy",
            margin=0.0,
            name="outer_fork_remains_guarded_when_tilted",
        )
        ctx.expect_within(
            inner_ring,
            outer_fork,
            axes="xy",
            margin=0.0,
            name="inner_ring_remains_nested_when_tilted",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_deflected_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
