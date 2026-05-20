from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.22
BASE_WIDTH = 0.10
BASE_T = 0.012

REAR_WALL_T = 0.014
REAR_WALL_W = 0.080
REAR_WALL_H = 0.070
REAR_WALL_X = -0.103

BRIDGE_LEN = 0.151
BRIDGE_W = 0.070
BRIDGE_T = 0.014
BRIDGE_BOTTOM_Z = 0.056
BRIDGE_CENTER_X = -0.0205

CHEEK_LEN = 0.095
CHEEK_T = 0.006
CHEEK_H = 0.038
CHEEK_CENTER_X = 0.0125
CHEEK_CENTER_Y = 0.023

CARRIAGE_LEN = 0.070
CARRIAGE_W = 0.036
CARRIAGE_H = 0.038
CARRIAGE_HOME_X = 0.010
CARRIAGE_CENTER_Z = BASE_T + CARRIAGE_H / 2.0
SLIDE_TRAVEL = 0.065

FORK_GAP = 0.014
FORK_CUT_LEN = 0.020
FORK_CUT_H = 0.032
HINGE_X = CARRIAGE_LEN / 2.0 - 0.006
HINGE_Z = 0.006

PIN_R = 0.003
PIN_BORE_R = 0.0033
PIN_LEN = FORK_GAP
BORE_LEN = CARRIAGE_W + 0.004

PADDLE_LEN = 0.022
PADDLE_W = 0.006
PADDLE_HEIGHT = 0.018
PADDLE_NECK_LEN = 0.012
PADDLE_NECK_W = 0.010
PADDLE_NECK_H = 0.008
PADDLE_OPEN = 1.0


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").transformed(offset=center).box(*size)


def _y_cylinder(
    center: tuple[float, float, float],
    radius: float,
    length: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _make_support_frame() -> cq.Workplane:
    rear_wall = _box(
        (REAR_WALL_X, 0.0, BASE_T + REAR_WALL_H / 2.0),
        (REAR_WALL_T, REAR_WALL_W, REAR_WALL_H),
    )
    bridge = _box(
        (BRIDGE_CENTER_X, 0.0, BRIDGE_BOTTOM_Z + BRIDGE_T / 2.0),
        (BRIDGE_LEN, BRIDGE_W, BRIDGE_T),
    )
    left_cheek = _box(
        (CHEEK_CENTER_X, CHEEK_CENTER_Y, BASE_T + CHEEK_H / 2.0),
        (CHEEK_LEN, CHEEK_T, CHEEK_H),
    )
    right_cheek = _box(
        (CHEEK_CENTER_X, -CHEEK_CENTER_Y, BASE_T + CHEEK_H / 2.0),
        (CHEEK_LEN, CHEEK_T, CHEEK_H),
    )
    return rear_wall.union(bridge).union(left_cheek).union(right_cheek)


def _make_carriage() -> cq.Workplane:
    body = cq.Workplane("XY").box(CARRIAGE_LEN, CARRIAGE_W, CARRIAGE_H)
    fork_slot = _box(
        (CARRIAGE_LEN / 2.0 - FORK_CUT_LEN / 2.0, 0.0, 0.0),
        (FORK_CUT_LEN, FORK_GAP, FORK_CUT_H),
    )
    hinge_bore = _y_cylinder((HINGE_X, 0.0, HINGE_Z), PIN_BORE_R, BORE_LEN)
    return body.cut(fork_slot).cut(hinge_bore)


def _make_paddle() -> cq.Workplane:
    barrel = _y_cylinder((0.0, 0.0, 0.0), PIN_R, PIN_LEN)
    neck = _box(
        (PADDLE_NECK_LEN / 2.0, 0.0, -0.004),
        (PADDLE_NECK_LEN, PADDLE_NECK_W, PADDLE_NECK_H),
    )
    plate = _box(
        (0.020, 0.0, -0.016),
        (PADDLE_LEN, PADDLE_W, PADDLE_HEIGHT),
    )
    toe = _y_cylinder((0.031, 0.0, -0.016), PADDLE_HEIGHT / 2.0, PADDLE_W)
    return barrel.union(neck).union(plate).union(toe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_slide")

    support_gray = model.material("support_gray", color=(0.34, 0.36, 0.39))
    carriage_gray = model.material("carriage_gray", color=(0.69, 0.71, 0.74))
    paddle_orange = model.material("paddle_orange", color=(0.84, 0.40, 0.14))

    support = model.part("support")
    support.visual(
        Box((BASE_LEN, BASE_WIDTH, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=support_gray,
        name="base_plate",
    )
    support.visual(
        mesh_from_cadquery(_make_support_frame(), "support_frame"),
        material=support_gray,
        name="support_frame",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage(), "carriage_body"),
        material=carriage_gray,
        name="carriage_body",
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_make_paddle(), "paddle_tip"),
        material=paddle_orange,
        name="paddle_tip",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, 0.0, CARRIAGE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_paddle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=paddle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    carriage = object_model.get_part("carriage")
    paddle = object_model.get_part("paddle")
    slide = object_model.get_articulation("support_to_carriage")
    tip = object_model.get_articulation("carriage_to_paddle")

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
        "slide_joint_configuration",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic +X slide, got type={slide.articulation_type!r} axis={slide.axis!r}",
    )
    ctx.check(
        "tip_joint_configuration",
        tip.articulation_type == ArticulationType.REVOLUTE and tuple(tip.axis) == (0.0, -1.0, 0.0),
        f"expected revolute -Y tip hinge, got type={tip.articulation_type!r} axis={tip.axis!r}",
    )

    ctx.expect_contact(
        carriage,
        support,
        elem_b="base_plate",
        contact_tol=1e-5,
        name="carriage_supported_on_base",
    )
    ctx.expect_origin_distance(
        paddle,
        carriage,
        axes="y",
        max_dist=1e-6,
        name="paddle_centered_in_fork",
    )
    ctx.expect_origin_gap(
        paddle,
        carriage,
        axis="x",
        min_gap=HINGE_X - 1e-6,
        max_gap=HINGE_X + 1e-6,
        name="paddle_hinge_x_location",
    )
    ctx.expect_origin_gap(
        paddle,
        carriage,
        axis="z",
        min_gap=HINGE_Z - 1e-6,
        max_gap=HINGE_Z + 1e-6,
        name="paddle_hinge_z_location",
    )
    ctx.expect_within(
        paddle,
        carriage,
        axes="y",
        margin=0.0,
        name="paddle_captured_between_fork_cheeks",
    )
    ctx.expect_within(
        carriage,
        support,
        axes="y",
        margin=0.01,
        name="carriage_kept_within_support_span",
    )

    closed_carriage_x = ctx.part_world_position(carriage)[0]
    with ctx.pose({slide: SLIDE_TRAVEL}):
        opened_carriage_x = ctx.part_world_position(carriage)[0]
    ctx.check(
        "carriage_moves_forward",
        opened_carriage_x > closed_carriage_x + 0.9 * SLIDE_TRAVEL,
        f"expected forward slide travel near {SLIDE_TRAVEL:.3f} m, got {opened_carriage_x - closed_carriage_x:.4f} m",
    )

    closed_tip_top_z = ctx.part_world_aabb(paddle)[1][2]
    with ctx.pose({tip: PADDLE_OPEN}):
        raised_tip_top_z = ctx.part_world_aabb(paddle)[1][2]
    ctx.check(
        "paddle_rotates_upward",
        raised_tip_top_z > closed_tip_top_z + 0.010,
        f"expected opened paddle to rise, got closed zmax={closed_tip_top_z:.4f} open zmax={raised_tip_top_z:.4f}",
    )

    with ctx.pose({slide: SLIDE_TRAVEL, tip: PADDLE_OPEN}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_extended_open_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
