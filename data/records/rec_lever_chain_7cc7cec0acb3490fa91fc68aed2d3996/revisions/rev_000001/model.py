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


CHEEK_THICKNESS = 0.008
LINK_THICKNESS = 0.006
TAB_THICKNESS = 0.005

PIN_HOLE_RADIUS = 0.0045
TIP_HOLE_RADIUS = 0.0032

CHEEK_BOSS_RADIUS = 0.015
LINK1_BOSS_RADIUS = 0.012
LINK2_BOSS_RADIUS = 0.0115
TAB_BOSS_RADIUS = 0.0105

CHEEK_OUTER_BOSS = 0.004
LINK_OUTER_BOSS = 0.004
TAB_OUTER_BOSS = 0.0035

LINK1_VECTOR = (0.095, 0.018)
LINK2_VECTOR = (0.082, -0.015)
TAB_VECTOR = (0.045, 0.006)

LINK1_Z = CHEEK_THICKNESS / 2.0 + CHEEK_OUTER_BOSS + LINK_THICKNESS / 2.0
LINK2_Z_OFFSET = LINK_THICKNESS / 2.0 + LINK_OUTER_BOSS + LINK_THICKNESS / 2.0
TAB_Z_OFFSET = LINK_THICKNESS / 2.0 + TAB_OUTER_BOSS + TAB_THICKNESS / 2.0


def _disc(center_xy: tuple[float, float], radius: float, thickness: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center_xy[0], center_xy[1])
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z0))
    )


def _beam(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    width: float,
    thickness: float,
    z0: float,
) -> cq.Workplane:
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    length = math.hypot(dx, dy)
    angle_deg = math.degrees(math.atan2(dy, dx))
    mid_x = (start_xy[0] + end_xy[0]) / 2.0
    mid_y = (start_xy[1] + end_xy[1]) / 2.0
    return (
        cq.Workplane("XY")
        .transformed(offset=(mid_x, mid_y, 0.0), rotate=(0.0, 0.0, angle_deg))
        .rect(length, width)
        .extrude(thickness)
        .translate((0.0, 0.0, z0))
    )


def _hole(center_xy: tuple[float, float], radius: float, length: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center_xy[0], center_xy[1])
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, z0))
    )


def _annular_boss(
    center_xy: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    z0: float,
) -> cq.Workplane:
    boss = _disc(center_xy, outer_radius, thickness, z0)
    return boss.cut(_hole(center_xy, inner_radius, thickness + 0.002, z0 - 0.001))


def _build_link_plate(
    *,
    distal_xy: tuple[float, float],
    thickness: float,
    body_width: float,
    prox_radius: float,
    dist_radius: float,
    prox_outer_boss: float,
    dist_outer_boss: float,
    tip_hole: float | None = None,
) -> cq.Workplane:
    z0 = -thickness / 2.0
    shape = _disc((0.0, 0.0), prox_radius, thickness, z0)
    shape = shape.union(_disc(distal_xy, dist_radius, thickness, z0))
    shape = shape.union(_beam((0.0, 0.0), distal_xy, body_width, thickness, z0))

    if prox_outer_boss > 0.0:
        shape = shape.union(
            _annular_boss(
                (0.0, 0.0),
                prox_radius,
                PIN_HOLE_RADIUS,
                prox_outer_boss,
                thickness / 2.0,
            )
        )
    if dist_outer_boss > 0.0:
        shape = shape.union(
            _annular_boss(
                distal_xy,
                dist_radius,
                PIN_HOLE_RADIUS,
                dist_outer_boss,
                thickness / 2.0,
            )
        )

    max_outer = max(prox_outer_boss, dist_outer_boss, 0.0)
    hole_len = thickness + max_outer + 0.004
    hole_z0 = -thickness / 2.0 - 0.002
    shape = shape.cut(_hole((0.0, 0.0), PIN_HOLE_RADIUS, hole_len, hole_z0))
    shape = shape.cut(_hole(distal_xy, PIN_HOLE_RADIUS, hole_len, hole_z0))

    if tip_hole is not None:
        shape = shape.cut(_hole(distal_xy, tip_hole, hole_len, hole_z0))

    return shape


def _build_cheek_shape() -> cq.Workplane:
    z0 = -CHEEK_THICKNESS / 2.0
    pivot = (0.0, 0.0)
    upper_mount = (-0.082, 0.028)
    lower_mount = (-0.088, -0.026)
    rear_bridge = (-0.102, 0.002)

    shape = _disc(pivot, 0.016, CHEEK_THICKNESS, z0)
    shape = shape.union(_disc(upper_mount, 0.018, CHEEK_THICKNESS, z0))
    shape = shape.union(_disc(lower_mount, 0.016, CHEEK_THICKNESS, z0))
    shape = shape.union(_disc(rear_bridge, 0.013, CHEEK_THICKNESS, z0))
    shape = shape.union(_beam(pivot, upper_mount, 0.020, CHEEK_THICKNESS, z0))
    shape = shape.union(_beam(pivot, lower_mount, 0.018, CHEEK_THICKNESS, z0))
    shape = shape.union(_beam(upper_mount, lower_mount, 0.020, CHEEK_THICKNESS, z0))
    shape = shape.union(_beam(upper_mount, rear_bridge, 0.014, CHEEK_THICKNESS, z0))
    shape = shape.union(_beam(lower_mount, rear_bridge, 0.014, CHEEK_THICKNESS, z0))
    shape = shape.union(
        _annular_boss(
            pivot,
            CHEEK_BOSS_RADIUS,
            PIN_HOLE_RADIUS,
            CHEEK_OUTER_BOSS,
            CHEEK_THICKNESS / 2.0,
        )
    )

    hole_len = CHEEK_THICKNESS + CHEEK_OUTER_BOSS + 0.004
    hole_z0 = -CHEEK_THICKNESS / 2.0 - 0.002
    shape = shape.cut(_hole(pivot, PIN_HOLE_RADIUS, hole_len, hole_z0))
    shape = shape.cut(_hole(upper_mount, 0.0042, hole_len, hole_z0))
    shape = shape.cut(_hole(lower_mount, 0.0042, hole_len, hole_z0))
    return shape


def _build_link1_shape() -> cq.Workplane:
    return _build_link_plate(
        distal_xy=LINK1_VECTOR,
        thickness=LINK_THICKNESS,
        body_width=0.022,
        prox_radius=LINK1_BOSS_RADIUS,
        dist_radius=LINK1_BOSS_RADIUS,
        prox_outer_boss=LINK_OUTER_BOSS,
        dist_outer_boss=LINK_OUTER_BOSS,
    )


def _build_link2_shape() -> cq.Workplane:
    return _build_link_plate(
        distal_xy=LINK2_VECTOR,
        thickness=LINK_THICKNESS,
        body_width=0.020,
        prox_radius=LINK2_BOSS_RADIUS,
        dist_radius=LINK2_BOSS_RADIUS,
        prox_outer_boss=LINK_OUTER_BOSS,
        dist_outer_boss=TAB_OUTER_BOSS,
    )


def _build_end_tab_shape() -> cq.Workplane:
    z0 = -TAB_THICKNESS / 2.0
    tip_center = TAB_VECTOR
    shape = _disc((0.0, 0.0), TAB_BOSS_RADIUS, TAB_THICKNESS, z0)
    shape = shape.union(_disc(tip_center, 0.0075, TAB_THICKNESS, z0))
    shape = shape.union(_beam((0.0, 0.0), tip_center, 0.013, TAB_THICKNESS, z0))
    shape = shape.union(
        _annular_boss(
            (0.0, 0.0),
            TAB_BOSS_RADIUS,
            PIN_HOLE_RADIUS,
            TAB_OUTER_BOSS,
            TAB_THICKNESS / 2.0,
        )
    )

    hole_len = TAB_THICKNESS + TAB_OUTER_BOSS + 0.004
    hole_z0 = -TAB_THICKNESS / 2.0 - 0.002
    shape = shape.cut(_hole((0.0, 0.0), PIN_HOLE_RADIUS, hole_len, hole_z0))
    shape = shape.cut(_hole(tip_center, TIP_HOLE_RADIUS, hole_len, hole_z0))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_lever_chain")

    model.material("cheek_gray", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("link_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("tab_dark", rgba=(0.28, 0.30, 0.33, 1.0))

    cheek = model.part("fixed_cheek")
    cheek.visual(
        mesh_from_cadquery(_build_cheek_shape(), "fixed_cheek"),
        origin=Origin(),
        material="cheek_gray",
        name="body",
    )

    link_1 = model.part("primary_link")
    link_1.visual(
        mesh_from_cadquery(_build_link1_shape(), "primary_link"),
        origin=Origin(),
        material="link_steel",
        name="body",
    )

    link_2 = model.part("secondary_link")
    link_2.visual(
        mesh_from_cadquery(_build_link2_shape(), "secondary_link"),
        origin=Origin(),
        material="link_steel",
        name="body",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_build_end_tab_shape(), "end_tab"),
        origin=Origin(),
        material="tab_dark",
        name="body",
    )

    model.articulation(
        "cheek_to_primary",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, LINK1_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.7, upper=1.05),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK1_VECTOR[0], LINK1_VECTOR[1], LINK2_Z_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-1.15, upper=1.1),
    )
    model.articulation(
        "secondary_to_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(LINK2_VECTOR[0], LINK2_VECTOR[1], TAB_Z_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.8, lower=-0.95, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cheek = object_model.get_part("fixed_cheek")
    link_1 = object_model.get_part("primary_link")
    link_2 = object_model.get_part("secondary_link")
    end_tab = object_model.get_part("end_tab")
    joint_1 = object_model.get_articulation("cheek_to_primary")
    joint_2 = object_model.get_articulation("primary_to_secondary")
    joint_3 = object_model.get_articulation("secondary_to_tab")

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
        "expected_parts_present",
        all(part is not None for part in (cheek, link_1, link_2, end_tab)),
        "Missing one or more authored parts.",
    )
    ctx.check(
        "parallel_joint_axes",
        joint_1.axis == joint_2.axis == joint_3.axis == (0.0, 0.0, 1.0),
        f"Joint axes were {joint_1.axis}, {joint_2.axis}, and {joint_3.axis}.",
    )

    ctx.expect_contact(link_1, cheek, name="primary_link_supported_by_cheek")
    ctx.expect_contact(link_2, link_1, name="secondary_link_supported_by_primary")
    ctx.expect_contact(end_tab, link_2, name="end_tab_supported_by_secondary")

    ctx.expect_origin_gap(
        link_1,
        cheek,
        axis="z",
        min_gap=0.009,
        max_gap=0.013,
        name="primary_link_stands_off_from_cheek_plane",
    )
    ctx.expect_origin_gap(
        link_2,
        link_1,
        axis="z",
        min_gap=0.008,
        max_gap=0.012,
        name="secondary_link_is_outboard_of_primary",
    )
    ctx.expect_origin_gap(
        end_tab,
        link_2,
        axis="z",
        min_gap=0.007,
        max_gap=0.011,
        name="end_tab_is_outboard_of_secondary",
    )

    def _center_xy_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.0}):
        base_center = _center_xy_from_aabb(ctx.part_element_world_aabb(end_tab, elem="body"))
    with ctx.pose({joint_1: 0.45, joint_2: 0.0, joint_3: 0.0}):
        joint_1_center = _center_xy_from_aabb(ctx.part_element_world_aabb(end_tab, elem="body"))
    with ctx.pose({joint_1: 0.0, joint_2: 0.45, joint_3: 0.0}):
        joint_2_center = _center_xy_from_aabb(ctx.part_element_world_aabb(end_tab, elem="body"))
    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.55}):
        joint_3_center = _center_xy_from_aabb(ctx.part_element_world_aabb(end_tab, elem="body"))

    ctx.check(
        "joint_1_rotates_chain_in_plane",
        abs(joint_1_center[2] - base_center[2]) < 1e-6 and joint_1_center[1] > base_center[1] + 0.02,
        f"End tab center moved from {base_center} to {joint_1_center}.",
    )
    ctx.check(
        "joint_2_rotates_chain_in_plane",
        abs(joint_2_center[2] - base_center[2]) < 1e-6 and joint_2_center[1] > base_center[1] + 0.01,
        f"End tab center moved from {base_center} to {joint_2_center}.",
    )
    ctx.check(
        "joint_3_rotates_end_tab_in_plane",
        abs(joint_3_center[2] - base_center[2]) < 1e-6 and joint_3_center[1] > base_center[1] + 0.004,
        f"End tab center moved from {base_center} to {joint_3_center}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
