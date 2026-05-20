from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

try:
    os.chdir("/")
except FileNotFoundError:
    pass

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd

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
)

HERE = os.getcwd()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chalkboard_easel")

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.43, 0.39, 1.0))
    chalkboard = model.material("chalkboard", rgba=(0.08, 0.10, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.14, 1.0))

    crown_z = 1.58
    foot_z = 0.018

    front_hinge_x = -0.005
    front_hinge_y = 0.40
    front_foot_y = 0.58
    front_drop = crown_z - foot_z
    front_spread = front_foot_y - front_hinge_y
    front_leg_len = math.hypot(front_drop, front_spread)
    front_leg_roll = math.atan2(front_spread, front_drop)

    rear_hinge_x = -0.050
    rear_foot_x = -0.63
    rear_backset = abs(rear_foot_x - rear_hinge_x)
    rear_drop = crown_z - foot_z
    rear_leg_len = math.hypot(rear_backset, rear_drop)
    rear_leg_pitch = math.atan2(rear_backset, rear_drop)
    front_leg_center_y = 0.5 * front_spread
    front_leg_center_z = -0.5 * front_drop
    front_leg_body_x = 0.045
    rear_leg_center_x = -0.5 * rear_backset
    rear_leg_center_z = -0.5 * rear_drop
    rear_leg_body_x = -0.050

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.22, 0.98, 1.66)),
        mass=17.5,
        origin=Origin(xyz=(-0.01, 0.0, 0.83)),
    )
    front_frame.visual(
        Box((0.05, 0.94, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, crown_z)),
        material=painted_steel,
        name="crown_header",
    )
    front_frame.visual(
        Box((0.04, 0.04, 1.12)),
        origin=Origin(xyz=(0.0, -0.31, 1.02)),
        material=painted_steel,
        name="left_board_rail",
    )
    front_frame.visual(
        Box((0.04, 0.04, 1.12)),
        origin=Origin(xyz=(0.0, 0.31, 1.02)),
        material=painted_steel,
        name="right_board_rail",
    )
    front_frame.visual(
        Box((0.04, 0.62, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.44)),
        material=painted_steel,
        name="top_frame_rail",
    )
    front_frame.visual(
        Box((0.04, 0.62, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=painted_steel,
        name="bottom_frame_rail",
    )
    front_frame.visual(
        Box((0.014, 0.58, 0.92)),
        origin=Origin(xyz=(-0.010, 0.0, 0.99)),
        material=chalkboard,
        name="chalkboard_panel",
    )
    front_frame.visual(
        Box((0.04, 0.08, 0.05)),
        origin=Origin(xyz=(front_hinge_x, -front_hinge_y, crown_z)),
        material=painted_steel,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.04, 0.08, 0.05)),
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, crown_z)),
        material=painted_steel,
        name="right_hinge_block",
    )
    front_frame.visual(
        Box((0.06, 0.12, 0.06)),
        origin=Origin(xyz=(rear_hinge_x, 0.0, crown_z)),
        material=painted_steel,
        name="rear_hinge_block",
    )

    left_leg = model.part("left_front_leg")
    left_leg.inertial = Inertial.from_geometry(
        Box((0.09, 0.10, front_leg_len)),
        mass=2.8,
        origin=Origin(xyz=(front_leg_body_x, -front_leg_center_y, front_leg_center_z)),
    )
    left_leg.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(front_leg_body_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="left_hinge_collar",
    )
    left_leg.visual(
        Box((0.035, 0.04, front_leg_len)),
        origin=Origin(
            xyz=(front_leg_body_x, -front_leg_center_y, front_leg_center_z),
            rpy=(-front_leg_roll, 0.0, 0.0),
        ),
        material=painted_steel,
        name="left_leg_body",
    )
    left_leg.visual(
        Box((0.07, 0.09, 0.036)),
        origin=Origin(xyz=(front_leg_body_x, -front_spread, -front_drop)),
        material=rubber,
        name="left_foot",
    )

    right_leg = model.part("right_front_leg")
    right_leg.inertial = Inertial.from_geometry(
        Box((0.09, 0.10, front_leg_len)),
        mass=2.8,
        origin=Origin(xyz=(front_leg_body_x, front_leg_center_y, front_leg_center_z)),
    )
    right_leg.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(front_leg_body_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="right_hinge_collar",
    )
    right_leg.visual(
        Box((0.035, 0.04, front_leg_len)),
        origin=Origin(
            xyz=(front_leg_body_x, front_leg_center_y, front_leg_center_z),
            rpy=(front_leg_roll, 0.0, 0.0),
        ),
        material=painted_steel,
        name="right_leg_body",
    )
    right_leg.visual(
        Box((0.07, 0.09, 0.036)),
        origin=Origin(xyz=(front_leg_body_x, front_spread, -front_drop)),
        material=rubber,
        name="right_foot",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.16, 0.80, 0.05)),
        mass=1.8,
        origin=Origin(xyz=(0.09, 0.0, 0.525)),
    )
    tray.visual(
        Box((0.14, 0.80, 0.012)),
        origin=Origin(xyz=(0.09, 0.0, 0.52)),
        material=painted_steel,
        name="tray_plate",
    )
    tray.visual(
        Box((0.02, 0.56, 0.045)),
        origin=Origin(xyz=(0.03, 0.0, 0.5365)),
        material=painted_steel,
        name="tray_back_lip",
    )
    tray.visual(
        Box((0.02, 0.80, 0.032)),
        origin=Origin(xyz=(0.15, 0.0, 0.526)),
        material=painted_steel,
        name="tray_front_lip",
    )

    rear_leg = model.part("rear_leg")
    rear_leg.inertial = Inertial.from_geometry(
        Box((rear_backset + 0.08, 0.12, rear_leg_len)),
        mass=2.6,
        origin=Origin(xyz=(rear_leg_body_x - 0.5 * rear_backset, 0.0, rear_leg_center_z)),
    )
    rear_leg.visual(
        Cylinder(radius=0.024, length=0.06),
        origin=Origin(xyz=(rear_leg_body_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="rear_hinge_collar",
    )
    rear_leg.visual(
        Box((0.035, 0.05, rear_leg_len)),
        origin=Origin(
            xyz=(rear_leg_body_x - 0.5 * rear_backset, 0.0, rear_leg_center_z),
            rpy=(0.0, rear_leg_pitch, 0.0),
        ),
        material=painted_steel,
        name="rear_leg_body",
    )
    rear_leg.visual(
        Box((0.08, 0.11, 0.036)),
        origin=Origin(xyz=(rear_leg_body_x - rear_backset, 0.0, -rear_drop)),
        material=rubber,
        name="rear_foot",
    )

    model.articulation(
        "front_frame_to_left_front_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_leg,
        origin=Origin(xyz=(front_hinge_x, -front_hinge_y, crown_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.08, upper=0.08),
    )
    model.articulation(
        "front_frame_to_right_front_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_leg,
        origin=Origin(xyz=(front_hinge_x, front_hinge_y, crown_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.08, upper=0.08),
    )
    model.articulation(
        "front_frame_to_tray",
        ArticulationType.FIXED,
        parent=front_frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "front_frame_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(rear_hinge_x, 0.0, crown_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.18, upper=0.12),
    )

    return model


def _foot_center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        0.5 * (aabb[0][0] + aabb[1][0]),
        0.5 * (aabb[0][1] + aabb[1][1]),
        0.5 * (aabb[0][2] + aabb[1][2]),
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    front_frame = object_model.get_part("front_frame")
    left_leg = object_model.get_part("left_front_leg")
    right_leg = object_model.get_part("right_front_leg")
    tray = object_model.get_part("tray")
    rear_leg = object_model.get_part("rear_leg")

    left_hinge = object_model.get_articulation("front_frame_to_left_front_leg")
    right_hinge = object_model.get_articulation("front_frame_to_right_front_leg")
    rear_hinge = object_model.get_articulation("front_frame_to_rear_leg")

    crown_header = front_frame.get_visual("crown_header")
    bottom_frame_rail = front_frame.get_visual("bottom_frame_rail")
    chalkboard_panel = front_frame.get_visual("chalkboard_panel")
    left_hinge_block = front_frame.get_visual("left_hinge_block")
    right_hinge_block = front_frame.get_visual("right_hinge_block")
    rear_hinge_block = front_frame.get_visual("rear_hinge_block")

    left_hinge_collar = left_leg.get_visual("left_hinge_collar")
    left_leg_body = left_leg.get_visual("left_leg_body")
    left_foot = left_leg.get_visual("left_foot")
    right_hinge_collar = right_leg.get_visual("right_hinge_collar")
    right_leg_body = right_leg.get_visual("right_leg_body")
    right_foot = right_leg.get_visual("right_foot")

    tray_plate = tray.get_visual("tray_plate")
    tray_back_lip = tray.get_visual("tray_back_lip")

    rear_hinge_collar = rear_leg.get_visual("rear_hinge_collar")
    rear_foot = rear_leg.get_visual("rear_foot")

    ctx.allow_overlap(
        front_frame,
        left_leg,
        elem_a=left_hinge_block,
        elem_b=left_hinge_collar,
        reason="the left front hinge collar nests inside the welded crown block",
    )
    ctx.allow_overlap(
        front_frame,
        left_leg,
        elem_a=crown_header,
        elem_b=left_hinge_collar,
        reason="the crown cap closes over the left hinge collar at the pivot",
    )
    ctx.allow_overlap(
        front_frame,
        right_leg,
        elem_a=right_hinge_block,
        elem_b=right_hinge_collar,
        reason="the right front hinge collar nests inside the welded crown block",
    )
    ctx.allow_overlap(
        front_frame,
        right_leg,
        elem_a=crown_header,
        elem_b=right_hinge_collar,
        reason="the crown cap closes over the right hinge collar at the pivot",
    )
    ctx.allow_overlap(
        front_frame,
        rear_leg,
        elem_a=rear_hinge_block,
        elem_b=rear_hinge_collar,
        reason="the rear hinge collar nests into the central crown hinge block",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_gap(
        right_leg,
        left_leg,
        axis="y",
        min_gap=1.00,
        positive_elem=right_foot,
        negative_elem=left_foot,
        name="front_feet_form_a_wide_a_frame",
    )
    ctx.expect_gap(
        front_frame,
        left_leg,
        axis="y",
        min_gap=0.02,
        positive_elem=chalkboard_panel,
        negative_elem=left_leg_body,
        name="chalkboard_sits_inside_left_leg",
    )
    ctx.expect_gap(
        right_leg,
        front_frame,
        axis="y",
        min_gap=0.02,
        positive_elem=right_leg_body,
        negative_elem=chalkboard_panel,
        name="chalkboard_sits_inside_right_leg",
    )
    ctx.expect_overlap(
        tray,
        front_frame,
        axes="y",
        elem_a=tray_plate,
        elem_b=crown_header,
        min_overlap=0.74,
        name="tray_runs_nearly_full_width",
    )
    ctx.expect_overlap(
        tray,
        front_frame,
        axes="y",
        elem_a=tray_plate,
        elem_b=chalkboard_panel,
        min_overlap=0.58,
        name="tray_spans_the_writing_area",
    )
    ctx.expect_gap(
        tray,
        front_frame,
        axis="x",
        min_gap=-0.001,
        max_gap=0.005,
        positive_elem=tray_back_lip,
        negative_elem=bottom_frame_rail,
        name="tray_is_fixed_to_the_front_face",
    )
    ctx.expect_gap(
        front_frame,
        tray,
        axis="z",
        min_gap=0.0,
        max_gap=0.03,
        positive_elem=chalkboard_panel,
        negative_elem=tray_plate,
        name="tray_sits_below_the_writing_surface",
    )
    ctx.expect_overlap(
        rear_leg,
        front_frame,
        axes="yz",
        elem_a=rear_hinge_collar,
        elem_b=rear_hinge_block,
        min_overlap=0.045,
        name="rear_hinge_stays_seated",
    )
    ctx.expect_gap(
        front_frame,
        rear_leg,
        axis="x",
        max_gap=0.02,
        max_penetration=0.06,
        positive_elem=rear_hinge_block,
        negative_elem=rear_hinge_collar,
        name="rear_leg_is_seated_at_the_crown_hinge",
    )
    ctx.expect_gap(
        front_frame,
        rear_leg,
        axis="x",
        min_gap=0.53,
        positive_elem=bottom_frame_rail,
        negative_elem=rear_foot,
        name="rear_foot_stands_behind_the_front_face",
    )

    for hinge in (left_hinge, right_hinge, rear_hinge):
        limits = hinge.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({hinge: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{hinge.name}_lower_no_floating")
            with ctx.pose({hinge: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{hinge.name}_upper_no_floating")

    left_foot_aabb = ctx.part_element_world_aabb(left_leg, elem=left_foot.name)
    right_foot_aabb = ctx.part_element_world_aabb(right_leg, elem=right_foot.name)
    rear_foot_aabb = ctx.part_element_world_aabb(rear_leg, elem=rear_foot.name)

    if left_foot_aabb is not None:
        ctx.check(
            "left_foot_reaches_floor_plane",
            abs(left_foot_aabb[0][2]) <= 0.004,
            details=f"left foot min z was {left_foot_aabb[0][2]:.4f} m",
        )
    if right_foot_aabb is not None:
        ctx.check(
            "right_foot_reaches_floor_plane",
            abs(right_foot_aabb[0][2]) <= 0.004,
            details=f"right foot min z was {right_foot_aabb[0][2]:.4f} m",
        )
    if rear_foot_aabb is not None:
        ctx.check(
            "rear_foot_reaches_floor_plane",
            abs(rear_foot_aabb[0][2]) <= 0.004,
            details=f"rear foot min z was {rear_foot_aabb[0][2]:.4f} m",
        )

    assert left_foot_aabb is not None and right_foot_aabb is not None and rear_foot_aabb is not None
    left_rest_center = _foot_center_from_aabb(left_foot_aabb)
    right_rest_center = _foot_center_from_aabb(right_foot_aabb)
    rear_rest_center = _foot_center_from_aabb(rear_foot_aabb)

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    rear_limits = rear_hinge.motion_limits
    assert left_limits is not None and left_limits.lower is not None and left_limits.upper is not None
    assert right_limits is not None and right_limits.lower is not None and right_limits.upper is not None
    assert rear_limits is not None and rear_limits.lower is not None and rear_limits.upper is not None

    with ctx.pose({left_hinge: left_limits.lower, right_hinge: right_limits.lower}):
        left_folded = ctx.part_element_world_aabb(left_leg, elem=left_foot.name)
        right_folded = ctx.part_element_world_aabb(right_leg, elem=right_foot.name)
        assert left_folded is not None and right_folded is not None
        left_folded_center = _foot_center_from_aabb(left_folded)
        right_folded_center = _foot_center_from_aabb(right_folded)
        ctx.check(
            "front_legs_fold_inward_together",
            left_folded_center[1] > left_rest_center[1] + 0.05
            and right_folded_center[1] < right_rest_center[1] - 0.05,
            details=(
                "front feet did not move inward enough: "
                f"left y {left_folded_center[1]:.3f}, right y {right_folded_center[1]:.3f}"
            ),
        )

    with ctx.pose({left_hinge: left_limits.upper, right_hinge: right_limits.upper}):
        left_open = ctx.part_element_world_aabb(left_leg, elem=left_foot.name)
        right_open = ctx.part_element_world_aabb(right_leg, elem=right_foot.name)
        assert left_open is not None and right_open is not None
        left_open_center = _foot_center_from_aabb(left_open)
        right_open_center = _foot_center_from_aabb(right_open)
        ctx.check(
            "front_legs_open_wider_at_upper_limit",
            left_open_center[1] < left_rest_center[1] - 0.015
            and right_open_center[1] > right_rest_center[1] + 0.015,
            details=(
                "front feet did not open wider enough: "
                f"left y {left_open_center[1]:.3f}, right y {right_open_center[1]:.3f}"
            ),
        )

    with ctx.pose({rear_hinge: rear_limits.lower}):
        rear_folded = ctx.part_element_world_aabb(rear_leg, elem=rear_foot.name)
        assert rear_folded is not None
        rear_folded_center = _foot_center_from_aabb(rear_folded)
        ctx.expect_gap(
            front_frame,
            rear_leg,
            axis="x",
            min_gap=0.10,
            positive_elem=bottom_frame_rail,
            negative_elem=rear_foot,
            name="rear_leg_can_fold_forward_without_crossing_the_frame",
        )
        ctx.check(
            "rear_leg_lower_limit_moves_foot_forward",
            rear_folded_center[0] > rear_rest_center[0] + 0.20,
            details=f"rear foot x was {rear_folded_center[0]:.3f} m at the folded pose",
        )

    with ctx.pose({rear_hinge: rear_limits.upper}):
        rear_open = ctx.part_element_world_aabb(rear_leg, elem=rear_foot.name)
        assert rear_open is not None
        rear_open_center = _foot_center_from_aabb(rear_open)
        ctx.expect_gap(
            front_frame,
            rear_leg,
            axis="x",
            min_gap=0.66,
            positive_elem=bottom_frame_rail,
            negative_elem=rear_foot,
            name="rear_leg_opens_the_a_frame_wider",
        )
        ctx.check(
            "rear_leg_upper_limit_moves_foot_back",
            rear_open_center[0] < rear_rest_center[0] - 0.05,
            details=f"rear foot x was {rear_open_center[0]:.3f} m at the open pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
