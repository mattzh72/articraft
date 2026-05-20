from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_X = 0.28
BASE_Y = 0.22
BASE_T = 0.028
COL_R = 0.032
COL_H = 0.74
TOP_CAP_H = 0.018

LOW_Z = 0.175
MID_Z = 0.395
TOP_Z = 0.615
HUB_X = 0.086

CHEEK_T = 0.012
CHEEK_GAP = 0.040
CHEEK_CENTER_Y = CHEEK_GAP / 2.0 + CHEEK_T / 2.0
HOUSING_FACE_Y = CHEEK_GAP / 2.0 + CHEEK_T

JOURNAL_R = 0.012
HUB_R = 0.033
HUB_W = 0.032
COLLAR_R = 0.020
COLLAR_T = 0.004
COLLAR_CENTER_Y = HOUSING_FACE_Y + COLLAR_T / 2.0

STOP_PLATE_R = 0.024
STOP_TAB_T = 0.006
STOP_TAB_CENTER_Y = HOUSING_FACE_Y + COLLAR_T + STOP_TAB_T / 2.0
STOP_LUG_T = 0.008
STOP_LUG_CENTER_Y = STOP_TAB_CENTER_Y + 0.009

PI = 3.141592653589793
HALF_PI = PI / 2.0

CHEEK_BLOCK_Z = 0.024
CHEEK_BLOCK_OFFSET_Z = 0.032
CHEEK_X = 0.032
BODY_X = 0.044
HOUSING_CORE_Y = 0.018
HOUSING_RIB_Y = 0.014

BRANCH_HUB_R = 0.017
BRANCH_HUB_W = 0.036
BRANCH_JOURNAL_R = 0.011
BRANCH_JOURNAL_W = CHEEK_GAP
COLLAR_CLEAR_Y = CHEEK_CENTER_Y + CHEEK_T / 2.0 + COLLAR_T / 2.0


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
    name: str,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder_z(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_cylinder_x(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, HALF_PI, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-HALF_PI, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _sx(sign: int, x: float) -> float:
    return float(sign) * x


def _add_frame_housing(part, material, prefix: str, sign: int, hub_z: float):
    sx = lambda value: _sx(sign, value)

    _add_box(
        part,
        (0.034, HOUSING_CORE_Y, 0.064),
        (sx(BODY_X), 0.0, hub_z - 0.006),
        material,
        f"{prefix}_core",
    )
    _add_box(
        part,
        (0.024, HOUSING_CORE_Y, 0.022),
        (sx(0.038), 0.0, hub_z + 0.040),
        material,
        f"{prefix}_top_tie",
    )
    _add_box(
        part,
        (0.020, HOUSING_CORE_Y, 0.018),
        (sx(0.036), 0.0, hub_z - 0.046),
        material,
        f"{prefix}_lower_tie",
    )

    for side_name, y_sign in (("pos", 1.0), ("neg", -1.0)):
        y_center = y_sign * CHEEK_CENTER_Y
        _add_box(
            part,
            (0.018, 0.018, 0.068),
            (sx(0.058), y_sign * 0.0175, hub_z),
            material,
            f"{prefix}_{side_name}_bridge",
        )
        _add_box(
            part,
            (0.024, CHEEK_T, 0.088),
            (sx(0.067), y_center, hub_z),
            material,
            f"{prefix}_{side_name}_spine",
        )
        _add_box(
            part,
            (0.034, CHEEK_T, CHEEK_BLOCK_Z),
            (sx(HUB_X), y_center, hub_z + CHEEK_BLOCK_OFFSET_Z),
            material,
            f"{prefix}_{side_name}_upper_cheek",
        )
        _add_box(
            part,
            (0.034, CHEEK_T, CHEEK_BLOCK_Z),
            (sx(HUB_X), y_center, hub_z - CHEEK_BLOCK_OFFSET_Z),
            material,
            f"{prefix}_{side_name}_lower_cheek",
        )

    _add_box(
        part,
        (0.060, HOUSING_RIB_Y, 0.012),
        (sx(0.056), 0.0, hub_z + 0.018),
        material,
        f"{prefix}_side_rail",
        rpy=(0.0, -sign * 0.42, 0.0),
    )

    _add_box(
        part,
        (0.016, STOP_LUG_T, 0.014),
        (sx(HUB_X - 0.028), STOP_LUG_CENTER_Y + 0.005, hub_z + 0.040),
        material,
        f"{prefix}_upper_lug",
    )
    _add_box(
        part,
        (0.016, 0.044, 0.012),
        (sx(HUB_X - 0.028), 0.031, hub_z + 0.040),
        material,
        f"{prefix}_upper_lug_arm",
    )
    _add_box(
        part,
        (0.018, STOP_LUG_T, 0.014),
        (sx(HUB_X - 0.026), STOP_LUG_CENTER_Y + 0.005, hub_z - 0.043),
        material,
        f"{prefix}_lower_lug",
    )
    _add_box(
        part,
        (0.018, 0.044, 0.012),
        (sx(HUB_X - 0.026), 0.031, hub_z - 0.043),
        material,
        f"{prefix}_lower_lug_arm",
    )


def _add_branch_root(part, material, prefix: str, sign: int):
    _add_cylinder_y(part, BRANCH_JOURNAL_R, BRANCH_JOURNAL_W, (0.0, 0.0, 0.0), material, f"{prefix}_journal")
    _add_cylinder_y(part, BRANCH_HUB_R, BRANCH_HUB_W, (0.0, 0.0, 0.0), material, f"{prefix}_hub")
    _add_cylinder_y(part, 0.008, 0.012, (0.0, 0.026, 0.0), material, f"{prefix}_outer_spacer")
    _add_cylinder_y(part, 0.008, 0.012, (0.0, -0.026, 0.0), material, f"{prefix}_inner_spacer")
    _add_cylinder_y(part, COLLAR_R, COLLAR_T, (0.0, COLLAR_CLEAR_Y, 0.0), material, f"{prefix}_outer_collar")
    _add_cylinder_y(part, COLLAR_R, COLLAR_T, (0.0, -COLLAR_CLEAR_Y, 0.0), material, f"{prefix}_inner_collar")
    _add_cylinder_y(
        part,
        STOP_PLATE_R,
        STOP_TAB_T,
        (0.0, STOP_TAB_CENTER_Y, 0.0),
        material,
        f"{prefix}_stop_plate",
    )
    _add_box(
        part,
        (0.015, STOP_TAB_T, 0.012),
        (_sx(sign, -0.022), STOP_TAB_CENTER_Y, -0.021),
        material,
        f"{prefix}_stop_tab",
    )


def _build_frame_geometry(frame, material):
    _add_box(frame, (BASE_X, BASE_Y, BASE_T), (0.0, 0.0, BASE_T / 2.0), material, "base_plate")
    _add_cylinder_z(
        frame,
        COL_R,
        COL_H,
        (0.0, 0.0, BASE_T + COL_H / 2.0),
        material,
        "center_column",
    )
    _add_cylinder_z(
        frame,
        0.025,
        TOP_CAP_H,
        (0.0, 0.0, BASE_T + COL_H + TOP_CAP_H / 2.0),
        material,
        "top_cap",
    )

    for sign in (-1, 1):
        _add_box(
            frame,
            (0.088, 0.018, 0.090),
            (_sx(sign, 0.050), 0.0, BASE_T + 0.046),
            material,
            f"base_rib_x_{'p' if sign > 0 else 'n'}",
            rpy=(0.0, -sign * 0.54, 0.0),
        )
        _add_box(
            frame,
            (0.018, 0.078, 0.082),
            (0.0, _sx(sign, 0.046), BASE_T + 0.043),
            material,
            f"base_rib_y_{'p' if sign > 0 else 'n'}",
            rpy=(sign * 0.48, 0.0, 0.0),
        )

    for label, z in (("low", LOW_Z), ("mid", MID_Z), ("top", TOP_Z)):
        _add_cylinder_z(frame, COL_R + 0.010, 0.018, (0.0, 0.0, z), material, f"{label}_column_band")

    _add_frame_housing(frame, material, "low_housing", 1, LOW_Z)
    _add_frame_housing(frame, material, "mid_housing", -1, MID_Z)
    _add_frame_housing(frame, material, "top_housing", 1, TOP_Z)


def _build_low_branch(part, material):
    _add_branch_root(part, material, "low_root", 1)
    _add_box(part, (0.036, 0.026, 0.024), (0.020, 0.0, 0.0), material, "low_knuckle")
    _add_box(part, (0.122, 0.026, 0.020), (0.094, 0.0, 0.0), material, "low_beam")
    _add_cylinder_x(part, 0.011, 0.028, (0.157, 0.0, 0.0), material, "low_pad_stem")
    _add_cylinder_x(part, 0.027, 0.010, (0.174, 0.0, 0.0), material, "low_pad_disc")


def _build_mid_branch(part, material):
    _add_branch_root(part, material, "mid_root", -1)
    _add_box(part, (0.040, 0.028, 0.024), (-0.022, 0.0, 0.0), material, "mid_knuckle")
    _add_box(part, (0.182, 0.028, 0.020), (-0.126, 0.0, 0.0), material, "mid_beam")
    _add_box(part, (0.044, 0.054, 0.020), (-0.238, 0.0, 0.0), material, "mid_pad")
    _add_box(part, (0.016, 0.032, 0.010), (-0.258, 0.0, 0.0), material, "mid_pad_step")


def _build_top_branch(part, material):
    _add_branch_root(part, material, "top_root", 1)
    _add_box(part, (0.050, 0.026, 0.024), (0.026, 0.0, 0.0), material, "top_knuckle")
    _add_box(
        part,
        (0.168, 0.026, 0.020),
        (0.118, 0.0, 0.060),
        material,
        "top_main_beam",
        rpy=(0.0, -0.62, 0.0),
    )
    _add_box(part, (0.028, 0.026, 0.032), (0.197, 0.0, 0.115), material, "top_fork_base")
    _add_box(part, (0.046, 0.008, 0.016), (0.228, 0.010, 0.128), material, "top_fork_tine_upper")
    _add_box(part, (0.046, 0.008, 0.016), (0.228, -0.010, 0.128), material, "top_fork_tine_lower")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metrology_fixture_tree")

    frame_mat = model.material("frame_gray", color=(0.58, 0.60, 0.62))
    low_mat = model.material("low_arm_black", color=(0.20, 0.22, 0.24))
    mid_mat = model.material("mid_arm_bluegray", color=(0.34, 0.39, 0.44))
    top_mat = model.material("top_arm_olive", color=(0.33, 0.38, 0.31))

    frame = model.part("frame")
    _build_frame_geometry(frame, frame_mat)

    low_branch = model.part("low_branch")
    _build_low_branch(low_branch, low_mat)

    mid_branch = model.part("mid_branch")
    _build_mid_branch(mid_branch, mid_mat)

    top_branch = model.part("top_branch")
    _build_top_branch(top_branch, top_mat)

    model.articulation(
        "low_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=low_branch,
        origin=Origin(xyz=(HUB_X, 0.0, LOW_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-0.60, upper=0.62),
    )

    model.articulation(
        "mid_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=mid_branch,
        origin=Origin(xyz=(-HUB_X, 0.0, MID_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.6, lower=-0.48, upper=0.52),
    )

    model.articulation(
        "top_branch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top_branch,
        origin=Origin(xyz=(HUB_X, 0.0, TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.44, upper=0.54),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    low_branch = object_model.get_part("low_branch")
    mid_branch = object_model.get_part("mid_branch")
    top_branch = object_model.get_part("top_branch")

    low_joint = object_model.get_articulation("low_branch_hinge")
    mid_joint = object_model.get_articulation("mid_branch_hinge")
    top_joint = object_model.get_articulation("top_branch_hinge")

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

    for part in (frame, low_branch, mid_branch, top_branch):
        ctx.check(f"{part.name}_present", part is not None, f"Missing part {part.name}")

    for joint in (low_joint, mid_joint, top_joint):
        ctx.check(
            f"{joint.name}_axis_supported",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} should rotate about a horizontal y-axis, got {joint.axis}",
        )

    ctx.expect_contact(
        low_branch,
        frame,
        contact_tol=0.001,
        name="low_branch_grounded_by_housing",
    )
    ctx.expect_contact(
        mid_branch,
        frame,
        contact_tol=0.001,
        name="mid_branch_grounded_by_housing",
    )
    ctx.expect_contact(
        top_branch,
        frame,
        contact_tol=0.001,
        name="top_branch_grounded_by_housing",
    )

    ctx.expect_origin_gap(
        mid_branch,
        low_branch,
        axis="z",
        min_gap=0.18,
        max_gap=0.28,
        name="mid_hub_stays_above_low_hub",
    )
    ctx.expect_origin_gap(
        top_branch,
        mid_branch,
        axis="z",
        min_gap=0.18,
        max_gap=0.28,
        name="top_hub_stays_above_mid_hub",
    )

    low_aabb = ctx.part_world_aabb(low_branch)
    mid_aabb = ctx.part_world_aabb(mid_branch)
    top_aabb = ctx.part_world_aabb(top_branch)
    top_pos = ctx.part_world_position(top_branch)

    if low_aabb is not None and mid_aabb is not None:
        low_dx = low_aabb[1][0] - low_aabb[0][0]
        mid_dx = mid_aabb[1][0] - mid_aabb[0][0]
        ctx.check(
            "mid_branch_longer_than_low_branch",
            mid_dx > low_dx + 0.08,
            f"Expected the mid branch to read longer than the low branch, got {mid_dx:.3f} vs {low_dx:.3f}",
        )

    if top_aabb is not None and top_pos is not None:
        top_rise = top_aabb[1][2] - top_pos[2]
        ctx.check(
            "top_branch_reads_angled_upward",
            top_rise > 0.09,
            f"Top branch should rise clearly above its hub, got {top_rise:.3f} m",
        )

    with ctx.pose({low_joint: 0.52, mid_joint: -0.42, top_joint: -0.30}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spread_pose_clearance")

    with ctx.pose({low_joint: -0.48, mid_joint: 0.42, top_joint: 0.46}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clearance")

    ctx.warn_if_articulation_overlaps(max_pose_samples=16, name="articulation_overlap_review")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
