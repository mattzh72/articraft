from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BODY_X = 0.090
BODY_Y = 0.042
BODY_Z = 0.050
BACK_BOSS_X = 0.014
BACK_BOSS_Y = 0.030
BACK_BOSS_Z = 0.030
FRONT_WINDOW_X = 0.024
FRONT_WINDOW_Y = 0.022
FRONT_WINDOW_Z = 0.020

RAIL_X = 0.014
RAIL_Y = 0.030
RAIL_Z = 0.010
RAIL_CENTER_X = BODY_X / 2.0 + RAIL_X / 2.0

JAW_X = 0.034
JAW_Y = 0.028
JAW_Z = 0.026
JAW_TRAVEL = 0.009

FINGER_BASE_X = 0.040
FINGER_BASE_Y = 0.016
FINGER_BASE_Z = 0.022
FINGER_BASE_OFFSET_Y = 0.014
FINGER_TIP_X = 0.016
FINGER_TIP_Y = 0.012
FINGER_TIP_Z = 0.016
FINGER_TIP_OFFSET_Y = 0.020

LEFT_JAW_OPEN_Y = BODY_Y / 2.0 + RAIL_Y - JAW_Y / 2.0
RIGHT_JAW_OPEN_Y = -LEFT_JAW_OPEN_Y
JAW_CENTER_X = BODY_X / 2.0 + RAIL_X + JAW_X / 2.0


def _housing_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_X, BODY_Y, BODY_Z)

    back_boss = (
        cq.Workplane("XY")
        .box(BACK_BOSS_X, BACK_BOSS_Y, BACK_BOSS_Z)
        .translate((-BODY_X / 2.0 - BACK_BOSS_X / 2.0 + 0.0002, 0.0, 0.0))
    )

    left_rail = (
        cq.Workplane("XY")
        .box(RAIL_X, RAIL_Y, RAIL_Z)
        .translate((RAIL_CENTER_X, BODY_Y / 2.0 + RAIL_Y / 2.0, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(RAIL_X, RAIL_Y, RAIL_Z)
        .translate((RAIL_CENTER_X, -BODY_Y / 2.0 - RAIL_Y / 2.0, 0.0))
    )

    front_window = (
        cq.Workplane("XY")
        .box(FRONT_WINDOW_X, FRONT_WINDOW_Y, FRONT_WINDOW_Z)
        .translate((BODY_X / 2.0 - FRONT_WINDOW_X / 2.0 + 0.001, 0.0, 0.0))
    )

    top_relief = (
        cq.Workplane("XY")
        .box(0.040, 0.026, 0.010)
        .translate((0.0, 0.0, BODY_Z / 2.0 - 0.005))
    )
    bottom_relief = (
        cq.Workplane("XY")
        .box(0.040, 0.026, 0.010)
        .translate((0.0, 0.0, -BODY_Z / 2.0 + 0.005))
    )

    housing = body.union(back_boss).union(left_rail).union(right_rail)
    housing = housing.cut(front_window).cut(top_relief).cut(bottom_relief)
    return housing


def _jaw_shape(side: str) -> cq.Workplane:
    side_sign = 1.0 if side == "left" else -1.0
    inward = -side_sign

    sleeve = cq.Workplane("XY").box(JAW_X, JAW_Y, JAW_Z)

    finger_base = (
        cq.Workplane("XY")
        .box(FINGER_BASE_X, FINGER_BASE_Y, FINGER_BASE_Z)
        .translate(
            (
                JAW_X / 2.0 + FINGER_BASE_X / 2.0 - 0.001,
                inward * FINGER_BASE_OFFSET_Y,
                0.0,
            )
        )
    )
    finger_tip = (
        cq.Workplane("XY")
        .box(FINGER_TIP_X, FINGER_TIP_Y, FINGER_TIP_Z)
        .translate(
            (
                JAW_X / 2.0 + FINGER_BASE_X + FINGER_TIP_X / 2.0 - 0.002,
                inward * FINGER_TIP_OFFSET_Y,
                0.0,
            )
        )
    )

    return sleeve.union(finger_base).union(finger_tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_rail_twin_slide_gripper")

    housing_mat = model.material("housing_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    jaw_mat = model.material("jaw_black", rgba=(0.14, 0.15, 0.17, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell"),
        material=housing_mat,
        name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_X + BACK_BOSS_X, BODY_Y + 2.0 * RAIL_Y, BODY_Z)),
        mass=1.4,
        origin=Origin(),
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_jaw_shape("left"), "left_jaw"),
        material=jaw_mat,
        name="left_jaw_shell",
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((JAW_X + FINGER_BASE_X + FINGER_TIP_X, JAW_Y, JAW_Z)),
        mass=0.22,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_jaw_shape("right"), "right_jaw"),
        material=jaw_mat,
        name="right_jaw_shell",
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((JAW_X + FINGER_BASE_X + FINGER_TIP_X, JAW_Y, JAW_Z)),
        mass=0.22,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_jaw,
        origin=Origin(xyz=(JAW_CENTER_X, LEFT_JAW_OPEN_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    model.articulation(
        "housing_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_jaw,
        origin=Origin(xyz=(JAW_CENTER_X, RIGHT_JAW_OPEN_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("housing_to_left_jaw")
    right_slide = object_model.get_articulation("housing_to_right_jaw")

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

    ctx.expect_contact(
        left_jaw,
        housing,
        name="left_jaw_block_is_carried_by_left_side_rail",
    )
    ctx.expect_contact(
        right_jaw,
        housing,
        name="right_jaw_block_is_carried_by_right_side_rail",
    )
    ctx.expect_overlap(
        left_jaw,
        housing,
        axes="yz",
        min_overlap=0.012,
        name="left_jaw_aligned_with_rail_section",
    )
    ctx.expect_overlap(
        right_jaw,
        housing,
        axes="yz",
        min_overlap=0.012,
        name="right_jaw_aligned_with_rail_section",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            left_jaw,
            right_jaw,
            axis="y",
            min_gap=0.018,
            max_gap=0.028,
            name="open_pose_finger_gap",
        )

    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        ctx.expect_gap(
            left_jaw,
            right_jaw,
            axis="y",
            min_gap=0.002,
            max_gap=0.008,
            name="closed_pose_fingers_nearly_meet_without_overlap",
        )

    left_open_y = ctx.part_world_position(left_jaw)[1]
    right_open_y = ctx.part_world_position(right_jaw)[1]
    with ctx.pose({left_slide: JAW_TRAVEL}):
        left_closed_y = ctx.part_world_position(left_jaw)[1]
    with ctx.pose({right_slide: JAW_TRAVEL}):
        right_closed_y = ctx.part_world_position(right_jaw)[1]

    ctx.check(
        "left_positive_prismatic_motion_moves_toward_center",
        left_closed_y < left_open_y - 0.005,
        details=f"open y={left_open_y:.4f}, closed y={left_closed_y:.4f}",
    )
    ctx.check(
        "right_positive_prismatic_motion_moves_toward_center",
        right_closed_y > right_open_y + 0.005,
        details=f"open y={right_open_y:.4f}, closed y={right_closed_y:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
