from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.014
PLATE_WIDTH = 0.140
PLATE_HEIGHT = 0.260
EAR_LENGTH = 0.030
EAR_WIDTH = 0.052
EAR_THICKNESS = 0.026
HINGE_BOSS_RADIUS = 0.010
HINGE_BOSS_LENGTH = 0.024
HINGE_ORIGIN_X = PLATE_THICKNESS + EAR_LENGTH + 0.011

BODY_ROOT_LENGTH = 0.020
BODY_RAIL_START_X = BODY_ROOT_LENGTH
BODY_RAIL_END_X = 0.236
BODY_HALF_WIDTH = 0.018
BODY_RAIL_INNER_Y = 0.011
BODY_RAIL_OUTER_Y = 0.017
BODY_RAIL_HALF_Z = 0.010
BODY_FRONT_BRIDGE_LENGTH = 0.016

SLIDER_SIDE_Y = 0.008
SLIDER_EAR_LENGTH = 0.020

SLIDE_TRAVEL = 0.080
SLIDER_TONGUE_LENGTH = 0.130
SLIDER_PIVOT_X = 0.120
SLIDER_PIVOT_Z = 0.0

SWING_UPPER = 1.15
FORK_LOWER = -0.55
FORK_UPPER = 0.95


def span_box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0, centered=(False, False, False))
        .translate((x0, y0, z0))
    )


def make_side_plate() -> cq.Workplane:
    plate = span_box(
        0.0,
        PLATE_THICKNESS,
        -PLATE_WIDTH / 2.0,
        PLATE_WIDTH / 2.0,
        -PLATE_HEIGHT / 2.0,
        PLATE_HEIGHT / 2.0,
    )

    upper_ear = span_box(
        PLATE_THICKNESS,
        PLATE_THICKNESS + EAR_LENGTH,
        -EAR_WIDTH / 2.0,
        EAR_WIDTH / 2.0,
        HINGE_BOSS_LENGTH / 2.0,
        HINGE_BOSS_LENGTH / 2.0 + EAR_THICKNESS,
    )
    lower_ear = span_box(
        PLATE_THICKNESS,
        PLATE_THICKNESS + EAR_LENGTH,
        -EAR_WIDTH / 2.0,
        EAR_WIDTH / 2.0,
        -HINGE_BOSS_LENGTH / 2.0 - EAR_THICKNESS,
        -HINGE_BOSS_LENGTH / 2.0,
    )

    bracket = plate.union(upper_ear).union(lower_ear)

    for z_pos in (-0.080, 0.080):
        hole = (
            cq.Workplane("YZ")
            .center(0.0, z_pos)
            .circle(0.006)
            .extrude(PLATE_THICKNESS + 0.004)
            .translate((-0.002, 0.0, 0.0))
        )
        bracket = bracket.cut(hole)

    return bracket


def make_body() -> cq.Workplane:
    root_boss = (
        cq.Workplane("XY")
        .center(HINGE_BOSS_RADIUS, 0.0)
        .circle(HINGE_BOSS_RADIUS)
        .extrude(HINGE_BOSS_LENGTH / 2.0, both=True)
    )
    root_plate = span_box(0.000, BODY_ROOT_LENGTH, -BODY_HALF_WIDTH, BODY_HALF_WIDTH, -0.018, 0.018)
    left_rail = span_box(
        BODY_ROOT_LENGTH,
        BODY_RAIL_END_X,
        -BODY_RAIL_OUTER_Y,
        -BODY_RAIL_INNER_Y,
        -BODY_RAIL_HALF_Z,
        BODY_RAIL_HALF_Z,
    )
    right_rail = span_box(
        BODY_ROOT_LENGTH,
        BODY_RAIL_END_X,
        BODY_RAIL_INNER_Y,
        BODY_RAIL_OUTER_Y,
        -BODY_RAIL_HALF_Z,
        BODY_RAIL_HALF_Z,
    )
    front_nose = span_box(BODY_RAIL_END_X, BODY_RAIL_END_X + 0.018, -0.012, 0.012, -0.009, 0.009)

    return root_boss.union(root_plate).union(left_rail).union(right_rail).union(front_nose)


def make_slider() -> cq.Workplane:
    core = span_box(0.000, SLIDER_TONGUE_LENGTH, -0.004, 0.004, -SLIDER_CORE_HALF_Z, SLIDER_CORE_HALF_Z)
    left_shoe = span_box(0.006, 0.100, -SLIDER_WIDTH_HALF, -0.004, -0.007, 0.007)
    right_shoe = span_box(0.006, 0.100, 0.004, SLIDER_WIDTH_HALF, -0.007, 0.007)
    front_block = span_box(0.116, SLIDER_TONGUE_LENGTH, -0.010, 0.010, -0.008, 0.008)

    return core.union(left_shoe).union(right_shoe).union(front_block)


def make_tip_fork() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .center(0.008, 0.0)
        .circle(0.008)
        .extrude(0.012 / 2.0, both=True)
    )
    bridge = span_box(0.012, 0.024, -0.010, 0.010, -0.004, 0.004)
    left_tine = span_box(0.024, 0.072, -0.010, -0.004, -0.003, 0.003)
    right_tine = span_box(0.024, 0.072, 0.004, 0.010, -0.003, 0.003)
    return hub.union(bridge).union(left_tine).union(right_tine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_swing_slide_rotary")

    plate_mat = model.material("plate_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    body_mat = model.material("body_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    slider_mat = model.material("slider_steel", rgba=(0.43, 0.46, 0.49, 1.0))
    fork_mat = model.material("fork_black", rgba=(0.20, 0.18, 0.15, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(make_side_plate(), "side_plate"),
        material=plate_mat,
        name="plate_shell",
    )

    body = model.part("body")
    body.visual(
        Cylinder(HINGE_BOSS_RADIUS, HINGE_BOSS_LENGTH),
        origin=Origin(),
        material=body_mat,
        name="hinge_boss",
    )
    body.visual(
        Box((0.011, 0.024, 0.024)),
        origin=Origin(xyz=(-0.0055, 0.0, 0.0)),
        material=body_mat,
        name="mount_tab",
    )
    body.visual(
        Box((BODY_ROOT_LENGTH, 0.036, 0.036)),
        origin=Origin(xyz=(BODY_ROOT_LENGTH / 2.0, 0.0, 0.0)),
        material=body_mat,
        name="root_block",
    )
    body.visual(
        Box((BODY_RAIL_END_X - BODY_RAIL_START_X, BODY_RAIL_OUTER_Y - BODY_RAIL_INNER_Y, 0.020)),
        origin=Origin(
            xyz=(
                (BODY_RAIL_START_X + BODY_RAIL_END_X) / 2.0,
                -(BODY_RAIL_INNER_Y + BODY_RAIL_OUTER_Y) / 2.0,
                0.0,
            )
        ),
        material=body_mat,
        name="left_rail",
    )
    body.visual(
        Box((BODY_RAIL_END_X - BODY_RAIL_START_X, BODY_RAIL_OUTER_Y - BODY_RAIL_INNER_Y, 0.020)),
        origin=Origin(
            xyz=(
                (BODY_RAIL_START_X + BODY_RAIL_END_X) / 2.0,
                (BODY_RAIL_INNER_Y + BODY_RAIL_OUTER_Y) / 2.0,
                0.0,
            )
        ),
        material=body_mat,
        name="right_rail",
    )
    body.visual(
        Box((BODY_FRONT_BRIDGE_LENGTH, 0.036, 0.020)),
        origin=Origin(xyz=(BODY_RAIL_END_X + BODY_FRONT_BRIDGE_LENGTH / 2.0, 0.0, 0.0)),
        material=body_mat,
        name="front_bridge",
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.110, 0.006, 0.014)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=slider_mat,
        name="spine",
    )
    slider.visual(
        Box((0.094, 0.006, 0.018)),
        origin=Origin(xyz=(0.053, -SLIDER_SIDE_Y, 0.0)),
        material=slider_mat,
        name="left_shoe",
    )
    slider.visual(
        Box((0.094, 0.006, 0.018)),
        origin=Origin(xyz=(0.053, SLIDER_SIDE_Y, 0.0)),
        material=slider_mat,
        name="right_shoe",
    )
    slider.visual(
        Box((0.010, 0.010, 0.016)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=slider_mat,
        name="clevis_bridge",
    )
    slider.visual(
        Box((SLIDER_EAR_LENGTH, 0.006, 0.016)),
        origin=Origin(xyz=(0.120, -SLIDER_SIDE_Y, 0.0)),
        material=slider_mat,
        name="left_ear",
    )
    slider.visual(
        Box((SLIDER_EAR_LENGTH, 0.006, 0.016)),
        origin=Origin(xyz=(0.120, SLIDER_SIDE_Y, 0.0)),
        material=slider_mat,
        name="right_ear",
    )

    tip_fork = model.part("tip_fork")
    tip_fork.visual(
        Cylinder(0.005, 0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=fork_mat,
        name="pivot_hub",
    )
    tip_fork.visual(
        Box((0.018, 0.008, 0.008)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=fork_mat,
        name="fork_neck",
    )
    tip_fork.visual(
        Box((0.012, 0.020, 0.008)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=fork_mat,
        name="fork_root",
    )
    tip_fork.visual(
        Box((0.044, 0.006, 0.006)),
        origin=Origin(xyz=(0.050, -0.007, 0.0)),
        material=fork_mat,
        name="left_tine",
    )
    tip_fork.visual(
        Box((0.044, 0.006, 0.006)),
        origin=Origin(xyz=(0.050, 0.007, 0.0)),
        material=fork_mat,
        name="right_tine",
    )

    model.articulation(
        "side_plate_to_body",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=body,
        origin=Origin(xyz=(HINGE_ORIGIN_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.4, lower=0.0, upper=SWING_UPPER),
    )

    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(BODY_ROOT_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.18, lower=0.0, upper=SLIDE_TRAVEL),
    )

    model.articulation(
        "slider_to_tip_fork",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=tip_fork,
        origin=Origin(xyz=(SLIDER_PIVOT_X, 0.0, SLIDER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=FORK_LOWER, upper=FORK_UPPER),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    body = object_model.get_part("body")
    slider = object_model.get_part("slider")
    tip_fork = object_model.get_part("tip_fork")
    swing = object_model.get_articulation("side_plate_to_body")
    slide = object_model.get_articulation("body_to_slider")
    tip = object_model.get_articulation("slider_to_tip_fork")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=1e-5)
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

    with ctx.pose({swing: 0.0, slide: 0.0, tip: 0.0}):
        ctx.expect_contact(body, side_plate, name="body_supported_by_side_plate_hinge")
        ctx.expect_contact(slider, body, name="slider_retracted_against_body_stop")
        ctx.expect_contact(tip_fork, slider, contact_tol=1e-5, name="fork_supported_by_slider_clevis")
        ctx.expect_overlap(slider, body, axes="yz", min_overlap=0.016, name="slider_is_carried_inside_body_guides")

    with ctx.pose({swing: 0.0, slide: SLIDE_TRAVEL, tip: 0.0}):
        ctx.expect_overlap(slider, body, axes="yz", min_overlap=0.016, name="extended_slider_remains_guide_aligned")

    with ctx.pose({swing: 0.0, slide: 0.0, tip: 0.0}):
        closed_fork_pos = ctx.part_world_position(tip_fork)
        closed_fork_aabb = ctx.part_world_aabb(tip_fork)

    with ctx.pose({swing: SWING_UPPER, slide: 0.0, tip: 0.0}):
        swung_fork_pos = ctx.part_world_position(tip_fork)

    with ctx.pose({swing: 0.0, slide: SLIDE_TRAVEL, tip: 0.0}):
        extended_fork_pos = ctx.part_world_position(tip_fork)

    with ctx.pose({swing: 0.0, slide: 0.0, tip: FORK_UPPER}):
        raised_fork_aabb = ctx.part_world_aabb(tip_fork)

    if closed_fork_pos is None or swung_fork_pos is None:
        ctx.fail("body_swings_outward", "could not measure fork position in closed and swung poses")
    else:
        ctx.check(
            "body_swings_outward",
            swung_fork_pos[1] > closed_fork_pos[1] + 0.050,
            f"expected fork y to increase by > 0.050 m, got closed={closed_fork_pos} swung={swung_fork_pos}",
        )

    if closed_fork_pos is None or extended_fork_pos is None:
        ctx.fail("slider_extends_outward", "could not measure fork position in retracted and extended poses")
    else:
        ctx.check(
            "slider_extends_outward",
            extended_fork_pos[0] > closed_fork_pos[0] + 0.060,
            f"expected fork x to increase by > 0.060 m, got closed={closed_fork_pos} extended={extended_fork_pos}",
        )

    if closed_fork_aabb is None or raised_fork_aabb is None:
        ctx.fail("fork_rotates_upward", "could not measure fork bounds in neutral and raised poses")
    else:
        ctx.check(
            "fork_rotates_upward",
            raised_fork_aabb[1][2] > closed_fork_aabb[1][2] + 0.020,
            f"expected fork top z to rise by > 0.020 m, got closed={closed_fork_aabb} raised={raised_fork_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
