from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

DESK_W = 1.80
DESK_D = 0.78
DESK_H = 1.03
TOP_T = 0.055

FRAME_T = 0.04
TOP_UNDERSIDE_Z = DESK_H - TOP_T
TOP_RAIL_Z = TOP_UNDERSIDE_Z - FRAME_T / 2.0
LOWER_STRETCHER_Z = 0.095
LEG_FOOT_H = 0.014
LEG_CENTER_Z = (LEG_FOOT_H + (TOP_UNDERSIDE_Z - FRAME_T)) / 2.0
LEG_LEN = (TOP_UNDERSIDE_Z - FRAME_T) - LEG_FOOT_H

HALF_W = DESK_W / 2.0
HALF_D = DESK_D / 2.0
LEG_X = HALF_W - FRAME_T / 2.0
LEG_Y = HALF_D - FRAME_T / 2.0
TOP_RAIL_LEN_X = DESK_W - 2.0 * FRAME_T
SIDE_RAIL_LEN_Y = DESK_D - 2.0 * FRAME_T

PEDESTAL_W = 0.42
PEDESTAL_D = 0.70
PEDESTAL_H = 0.935
PEDESTAL_X = HALF_W - FRAME_T - PEDESTAL_W / 2.0 - 0.02
PEDESTAL_FRONT_Y = PEDESTAL_D / 2.0

PED_WALL_T = 0.018
PED_TOP_T = 0.020
PED_BOTTOM_T = 0.020
PED_BACK_T = 0.016
PED_DIVIDER_T = 0.018
PED_INNER_W = PEDESTAL_W - 2.0 * PED_WALL_T
PED_INNER_D = PEDESTAL_D - PED_BACK_T

DRAWER_FRONT_T = 0.018
DRAWER_RAIL_T = 0.016
DRAWER_RAIL_H = 0.018
DRAWER_SLIDE_T = 0.006
DRAWER_SLIDE_H = 0.018
DRAWER_BODY_W = 0.340
DRAWER_FRONT_W = PED_INNER_W - 0.010
LEFT_RAIL_X = -(PED_INNER_W / 2.0 - DRAWER_RAIL_T / 2.0)
RIGHT_RAIL_X = -LEFT_RAIL_X
LEFT_SLIDE_X = -(DRAWER_BODY_W / 2.0 + DRAWER_SLIDE_T / 2.0)
RIGHT_SLIDE_X = -LEFT_SLIDE_X
DRAWER_JOINT_X = LEFT_RAIL_X
DRAWER_JOINT_Y = -0.02

TOP_DRAWER_Z = -0.095
MID_DRAWER_Z = -0.297
BOTTOM_DRAWER_Z = -0.635


def _box_visual(part, size, xyz, name, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder_visual(part, radius, length, xyz, name, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _drawer_part(
    model: ArticulatedObject,
    *,
    name: str,
    front_h: float,
    body_h: float,
    body_d: float,
    slide_len: float,
    mass: float,
    shell_material,
    pull_material,
    pull_w: float,
    pull_h: float,
):
    drawer = model.part(name)
    _box_visual(
        drawer,
        (DRAWER_FRONT_W, DRAWER_FRONT_T, front_h),
        (-DRAWER_JOINT_X, -DRAWER_JOINT_Y - DRAWER_FRONT_T / 2.0, 0.0),
        "drawer_front",
        shell_material,
    )
    _box_visual(
        drawer,
        (DRAWER_BODY_W, body_d, body_h),
        (-DRAWER_JOINT_X, -DRAWER_JOINT_Y - DRAWER_FRONT_T - body_d / 2.0, 0.0),
        "drawer_body",
        shell_material,
    )
    _box_visual(
        drawer,
        (DRAWER_SLIDE_T, slide_len, DRAWER_SLIDE_H),
        (LEFT_SLIDE_X - DRAWER_JOINT_X, -slide_len / 2.0, 0.0),
        "left_slide",
        shell_material,
    )
    _box_visual(
        drawer,
        (DRAWER_SLIDE_T, slide_len, DRAWER_SLIDE_H),
        (RIGHT_SLIDE_X - DRAWER_JOINT_X, -slide_len / 2.0, 0.0),
        "right_slide",
        shell_material,
    )
    _box_visual(
        drawer,
        (pull_w, 0.008, pull_h),
        (-DRAWER_JOINT_X, -DRAWER_JOINT_Y - 0.013, front_h * 0.22),
        "finger_pull",
        pull_material,
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_W, DRAWER_FRONT_T + body_d, front_h)),
        mass=mass,
        origin=Origin(
            xyz=(
                -DRAWER_JOINT_X,
                -DRAWER_JOINT_Y - (DRAWER_FRONT_T + body_d) / 2.0,
                0.0,
            )
        ),
    )
    return drawer


def _pedestal_rail_set(
    pedestal,
    *,
    prefix: str,
    rail_z: float,
    rail_len: float,
    material,
) -> None:
    rail_y = -(0.02 + rail_len / 2.0)
    front_bracket_y = -0.05
    rear_bracket_y = 0.03 - rail_len

    _box_visual(
        pedestal,
        (DRAWER_RAIL_T, rail_len, DRAWER_RAIL_H),
        (LEFT_RAIL_X, rail_y, rail_z),
        f"{prefix}_left_rail",
        material,
    )
    _box_visual(
        pedestal,
        (DRAWER_RAIL_T, rail_len, DRAWER_RAIL_H),
        (RIGHT_RAIL_X, rail_y, rail_z),
        f"{prefix}_right_rail",
        material,
    )

    for side_name, bracket_x in (("left", -0.187), ("right", 0.187)):
        for end_name, bracket_y in (("front", front_bracket_y), ("rear", rear_bracket_y)):
            _box_visual(
                pedestal,
                (0.010, 0.024, 0.024),
                (bracket_x, bracket_y, rail_z - 0.021),
                f"{prefix}_{side_name}_{end_name}_bracket",
                material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_loft_workdesk", assets=ASSETS)

    raw_steel = model.material("raw_steel", rgba=(0.40, 0.41, 0.42, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.29, 0.20, 0.12, 1.0))
    foot_black = model.material("foot_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pull_shadow = model.material("pull_shadow", rgba=(0.11, 0.11, 0.12, 1.0))

    frame = model.part("frame")
    _box_visual(
        frame,
        (TOP_RAIL_LEN_X, FRAME_T, FRAME_T),
        (0.0, LEG_Y, TOP_RAIL_Z),
        "front_top_rail",
        raw_steel,
    )
    _box_visual(
        frame,
        (TOP_RAIL_LEN_X, FRAME_T, FRAME_T),
        (0.0, -LEG_Y, TOP_RAIL_Z),
        "rear_top_rail",
        raw_steel,
    )
    _box_visual(
        frame,
        (FRAME_T, SIDE_RAIL_LEN_Y, FRAME_T),
        (-LEG_X, 0.0, TOP_RAIL_Z),
        "left_top_side_rail",
        raw_steel,
    )
    _box_visual(
        frame,
        (FRAME_T, SIDE_RAIL_LEN_Y, FRAME_T),
        (LEG_X, 0.0, TOP_RAIL_Z),
        "right_top_side_rail",
        raw_steel,
    )
    _box_visual(
        frame,
        (FRAME_T, SIDE_RAIL_LEN_Y, FRAME_T),
        (0.0, 0.0, TOP_RAIL_Z),
        "center_top_support",
        raw_steel,
    )
    _box_visual(
        frame,
        (TOP_RAIL_LEN_X, FRAME_T, FRAME_T),
        (0.0, LEG_Y, LOWER_STRETCHER_Z),
        "front_lower_stretcher",
        raw_steel,
    )
    _box_visual(
        frame,
        (TOP_RAIL_LEN_X, FRAME_T, FRAME_T),
        (0.0, -LEG_Y, LOWER_STRETCHER_Z),
        "rear_lower_stretcher",
        raw_steel,
    )

    for leg_name, x_sign, y_sign in (
        ("left_front_leg", -1.0, 1.0),
        ("left_rear_leg", -1.0, -1.0),
        ("right_front_leg", 1.0, 1.0),
        ("right_rear_leg", 1.0, -1.0),
    ):
        _box_visual(
            frame,
            (FRAME_T, FRAME_T, LEG_LEN),
            (x_sign * LEG_X, y_sign * LEG_Y, LEG_CENTER_Z),
            leg_name,
            raw_steel,
        )
        _cylinder_visual(
            frame,
            radius=0.017,
            length=LEG_FOOT_H,
            xyz=(x_sign * LEG_X, y_sign * LEG_Y, LEG_FOOT_H / 2.0),
            name=f"{leg_name}_foot",
            material=foot_black,
        )

    brace_start_a = (-0.84, -0.35)
    brace_end_a = (0.30, 0.35)
    brace_len = math.hypot(brace_end_a[0] - brace_start_a[0], brace_end_a[1] - brace_start_a[1])
    brace_angle = math.atan2(brace_end_a[1] - brace_start_a[1], brace_end_a[0] - brace_start_a[0])
    _box_visual(
        frame,
        (brace_len, 0.028, 0.028),
        ((brace_start_a[0] + brace_end_a[0]) / 2.0, 0.0, TOP_RAIL_Z),
        "left_cross_brace",
        dark_steel,
        rpy=(0.0, 0.0, brace_angle),
    )
    _box_visual(
        frame,
        (brace_len, 0.028, 0.028),
        ((brace_start_a[0] + brace_end_a[0]) / 2.0, 0.0, TOP_RAIL_Z),
        "right_cross_brace",
        dark_steel,
        rpy=(0.0, 0.0, -brace_angle),
    )
    frame.inertial = Inertial.from_geometry(
        Box((DESK_W, DESK_D, DESK_H)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, DESK_H / 2.0)),
    )

    work_surface = model.part("work_surface")
    _box_visual(
        work_surface,
        (DESK_W, DESK_D, TOP_T),
        (0.0, 0.0, TOP_T / 2.0),
        "work_surface",
        dark_wood,
    )
    work_surface.inertial = Inertial.from_geometry(
        Box((DESK_W, DESK_D, TOP_T)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_T / 2.0)),
    )

    pedestal = model.part("pedestal")
    _box_visual(
        pedestal,
        (PEDESTAL_W, PEDESTAL_D, PED_TOP_T),
        (0.0, -PEDESTAL_D / 2.0, -PED_TOP_T / 2.0),
        "top_sheet",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (PEDESTAL_W, PEDESTAL_D, PED_BOTTOM_T),
        (0.0, -PEDESTAL_D / 2.0, -PEDESTAL_H + PED_BOTTOM_T / 2.0),
        "bottom_sheet",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (PED_WALL_T, PEDESTAL_D, PEDESTAL_H),
        (-PEDESTAL_W / 2.0 + PED_WALL_T / 2.0, -PEDESTAL_D / 2.0, -PEDESTAL_H / 2.0),
        "left_wall",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (PED_WALL_T, PEDESTAL_D, PEDESTAL_H),
        (PEDESTAL_W / 2.0 - PED_WALL_T / 2.0, -PEDESTAL_D / 2.0, -PEDESTAL_H / 2.0),
        "right_wall",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (PED_INNER_W, PED_BACK_T, PEDESTAL_H),
        (0.0, -PEDESTAL_D + PED_BACK_T / 2.0, -PEDESTAL_H / 2.0),
        "back_sheet",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (0.050, 0.060, 0.010),
        (PEDESTAL_W / 2.0 - 0.005, -0.18, -0.070),
        "front_mount_tab",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (0.050, 0.060, 0.010),
        (PEDESTAL_W / 2.0 - 0.005, -0.54, -0.070),
        "rear_mount_tab",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (0.012, 0.012, 0.008),
        (PEDESTAL_W / 2.0 + 0.019, -0.18, -0.066),
        "front_mount_bolt",
        dark_steel,
    )
    _box_visual(
        pedestal,
        (0.012, 0.012, 0.008),
        (PEDESTAL_W / 2.0 + 0.019, -0.54, -0.066),
        "rear_mount_bolt",
        dark_steel,
    )
    _box_visual(
        pedestal,
        (PED_INNER_W, PED_INNER_D, PED_DIVIDER_T),
        (0.0, -PED_INNER_D / 2.0, -0.179),
        "upper_divider",
        raw_steel,
    )
    _box_visual(
        pedestal,
        (PED_INNER_W, PED_INNER_D, PED_DIVIDER_T),
        (0.0, -PED_INNER_D / 2.0, -0.415),
        "lower_divider",
        raw_steel,
    )
    _pedestal_rail_set(pedestal, prefix="top", rail_z=TOP_DRAWER_Z, rail_len=0.32, material=dark_steel)
    _pedestal_rail_set(pedestal, prefix="middle", rail_z=MID_DRAWER_Z, rail_len=0.45, material=dark_steel)
    _pedestal_rail_set(pedestal, prefix="bottom", rail_z=BOTTOM_DRAWER_Z, rail_len=0.60, material=dark_steel)
    pedestal.inertial = Inertial.from_geometry(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        mass=32.0,
        origin=Origin(xyz=(0.0, -PEDESTAL_D / 2.0, -PEDESTAL_H / 2.0)),
    )

    top_drawer = _drawer_part(
        model,
        name="top_drawer",
        front_h=0.145,
        body_h=0.105,
        body_d=0.300,
        slide_len=0.300,
        mass=4.0,
        shell_material=dark_steel,
        pull_material=pull_shadow,
        pull_w=0.200,
        pull_h=0.026,
    )
    middle_drawer = _drawer_part(
        model,
        name="middle_drawer",
        front_h=0.215,
        body_h=0.170,
        body_d=0.430,
        slide_len=0.430,
        mass=6.0,
        shell_material=dark_steel,
        pull_material=pull_shadow,
        pull_w=0.230,
        pull_h=0.030,
    )
    bottom_drawer = _drawer_part(
        model,
        name="bottom_drawer",
        front_h=0.420,
        body_h=0.330,
        body_d=0.580,
        slide_len=0.580,
        mass=9.0,
        shell_material=dark_steel,
        pull_material=pull_shadow,
        pull_w=0.260,
        pull_h=0.036,
    )

    model.articulation(
        "frame_to_work_surface",
        ArticulationType.FIXED,
        parent=frame,
        child=work_surface,
        origin=Origin(xyz=(0.0, 0.0, TOP_UNDERSIDE_Z)),
    )
    model.articulation(
        "frame_to_pedestal",
        ArticulationType.FIXED,
        parent=frame,
        child=pedestal,
        origin=Origin(xyz=(PEDESTAL_X, PEDESTAL_FRONT_Y, TOP_UNDERSIDE_Z)),
    )
    model.articulation(
        "pedestal_to_top_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=top_drawer,
        origin=Origin(xyz=(DRAWER_JOINT_X, DRAWER_JOINT_Y, TOP_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.22),
    )
    model.articulation(
        "pedestal_to_middle_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=middle_drawer,
        origin=Origin(xyz=(DRAWER_JOINT_X, DRAWER_JOINT_Y, MID_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.45, lower=0.0, upper=0.28),
    )
    model.articulation(
        "pedestal_to_bottom_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=bottom_drawer,
        origin=Origin(xyz=(DRAWER_JOINT_X, DRAWER_JOINT_Y, BOTTOM_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.40, lower=0.0, upper=0.34),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    work_surface = object_model.get_part("work_surface")
    pedestal = object_model.get_part("pedestal")
    top_drawer = object_model.get_part("top_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    bottom_drawer = object_model.get_part("bottom_drawer")

    top_joint = object_model.get_articulation("pedestal_to_top_drawer")
    middle_joint = object_model.get_articulation("pedestal_to_middle_drawer")
    bottom_joint = object_model.get_articulation("pedestal_to_bottom_drawer")

    top_panel = work_surface.get_visual("work_surface")
    front_top_rail = frame.get_visual("front_top_rail")
    rear_top_rail = frame.get_visual("rear_top_rail")
    left_cross_brace = frame.get_visual("left_cross_brace")
    right_cross_brace = frame.get_visual("right_cross_brace")
    right_front_leg = frame.get_visual("right_front_leg")
    right_rear_leg = frame.get_visual("right_rear_leg")
    right_top_side_rail = frame.get_visual("right_top_side_rail")

    pedestal_top = pedestal.get_visual("top_sheet")
    pedestal_right_wall = pedestal.get_visual("right_wall")
    pedestal_back = pedestal.get_visual("back_sheet")
    front_mount_tab = pedestal.get_visual("front_mount_tab")
    rear_mount_tab = pedestal.get_visual("rear_mount_tab")
    top_left_rail = pedestal.get_visual("top_left_rail")
    top_right_rail = pedestal.get_visual("top_right_rail")
    middle_left_rail = pedestal.get_visual("middle_left_rail")
    middle_right_rail = pedestal.get_visual("middle_right_rail")
    bottom_left_rail = pedestal.get_visual("bottom_left_rail")
    bottom_right_rail = pedestal.get_visual("bottom_right_rail")

    top_front = top_drawer.get_visual("drawer_front")
    top_body = top_drawer.get_visual("drawer_body")
    top_left_slide = top_drawer.get_visual("left_slide")
    top_right_slide = top_drawer.get_visual("right_slide")
    middle_front = middle_drawer.get_visual("drawer_front")
    middle_body = middle_drawer.get_visual("drawer_body")
    middle_left_slide = middle_drawer.get_visual("left_slide")
    middle_right_slide = middle_drawer.get_visual("right_slide")
    bottom_front = bottom_drawer.get_visual("drawer_front")
    bottom_body = bottom_drawer.get_visual("drawer_body")
    bottom_left_slide = bottom_drawer.get_visual("left_slide")
    bottom_right_slide = bottom_drawer.get_visual("right_slide")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.allow_coplanar_surfaces(
        pedestal,
        work_surface,
        reason="pedestal top is intentionally flush with the work surface",
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(work_surface, frame, axes="xy", max_dist=0.01)
    ctx.expect_gap(
        work_surface,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=top_panel,
        negative_elem=front_top_rail,
    )
    ctx.expect_gap(
        work_surface,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=top_panel,
        negative_elem=rear_top_rail,
    )
    ctx.expect_gap(
        work_surface,
        frame,
        axis="z",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem=top_panel,
        negative_elem=left_cross_brace,
    )
    ctx.expect_gap(
        work_surface,
        frame,
        axis="z",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem=top_panel,
        negative_elem=right_cross_brace,
    )

    ctx.expect_within(pedestal, work_surface, axes="xy")
    ctx.expect_gap(
        work_surface,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=top_panel,
        negative_elem=pedestal_top,
    )
    ctx.expect_gap(
        frame,
        pedestal,
        axis="x",
        min_gap=0.0,
        max_gap=0.05,
        positive_elem=right_front_leg,
        negative_elem=pedestal_right_wall,
    )
    ctx.expect_gap(
        frame,
        pedestal,
        axis="x",
        min_gap=0.0,
        max_gap=0.05,
        positive_elem=right_rear_leg,
        negative_elem=pedestal_right_wall,
    )
    ctx.expect_gap(
        frame,
        pedestal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_top_side_rail,
        negative_elem=front_mount_tab,
    )
    ctx.expect_gap(
        frame,
        pedestal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_top_side_rail,
        negative_elem=rear_mount_tab,
    )

    ctx.expect_gap(
        top_drawer,
        middle_drawer,
        axis="z",
        min_gap=0.012,
        max_gap=0.030,
        positive_elem=top_front,
        negative_elem=middle_front,
    )
    ctx.expect_gap(
        middle_drawer,
        bottom_drawer,
        axis="z",
        min_gap=0.012,
        max_gap=0.030,
        positive_elem=middle_front,
        negative_elem=bottom_front,
    )
    ctx.expect_gap(
        top_drawer,
        pedestal,
        axis="y",
        min_gap=0.34,
        max_gap=0.43,
        positive_elem=top_body,
        negative_elem=pedestal_back,
    )
    ctx.expect_gap(
        middle_drawer,
        pedestal,
        axis="y",
        min_gap=0.23,
        max_gap=0.31,
        positive_elem=middle_body,
        negative_elem=pedestal_back,
    )
    ctx.expect_gap(
        bottom_drawer,
        pedestal,
        axis="y",
        min_gap=0.08,
        max_gap=0.14,
        positive_elem=bottom_body,
        negative_elem=pedestal_back,
    )
    ctx.expect_overlap(top_drawer, pedestal, axes="xz", min_overlap=0.14)
    ctx.expect_overlap(middle_drawer, pedestal, axes="xz", min_overlap=0.20)
    ctx.expect_overlap(bottom_drawer, pedestal, axes="xz", min_overlap=0.24)

    ctx.expect_gap(
        top_drawer,
        pedestal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=top_left_slide,
        negative_elem=top_left_rail,
    )
    ctx.expect_gap(
        pedestal,
        top_drawer,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=top_right_rail,
        negative_elem=top_right_slide,
    )
    ctx.expect_gap(
        middle_drawer,
        pedestal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=middle_left_slide,
        negative_elem=middle_left_rail,
    )
    ctx.expect_gap(
        pedestal,
        middle_drawer,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=middle_right_rail,
        negative_elem=middle_right_slide,
    )
    ctx.expect_gap(
        bottom_drawer,
        pedestal,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=bottom_left_slide,
        negative_elem=bottom_left_rail,
    )
    ctx.expect_gap(
        pedestal,
        bottom_drawer,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0002,
        positive_elem=bottom_right_rail,
        negative_elem=bottom_right_slide,
    )

    with ctx.pose({top_joint: 0.18}):
        ctx.expect_within(top_drawer, pedestal, axes="xz")
        ctx.expect_gap(
            top_drawer,
            pedestal,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem=top_left_slide,
            negative_elem=top_left_rail,
        )
        ctx.expect_gap(
            pedestal,
            top_drawer,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem=top_right_rail,
            negative_elem=top_right_slide,
        )

    with ctx.pose({middle_joint: 0.24}):
        ctx.expect_within(middle_drawer, pedestal, axes="xz")
        ctx.expect_gap(
            middle_drawer,
            pedestal,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem=middle_left_slide,
            negative_elem=middle_left_rail,
        )
        ctx.expect_gap(
            pedestal,
            middle_drawer,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem=middle_right_rail,
            negative_elem=middle_right_slide,
        )

    with ctx.pose({bottom_joint: 0.30}):
        ctx.expect_within(bottom_drawer, pedestal, axes="xz")
        ctx.expect_gap(
            bottom_drawer,
            pedestal,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem=bottom_left_slide,
            negative_elem=bottom_left_rail,
        )
        ctx.expect_gap(
            pedestal,
            bottom_drawer,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem=bottom_right_rail,
            negative_elem=bottom_right_slide,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
