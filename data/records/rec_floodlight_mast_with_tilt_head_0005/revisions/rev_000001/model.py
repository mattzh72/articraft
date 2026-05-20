from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="construction_flood_mast")

    base_yellow = model.material("base_yellow", rgba=(0.87, 0.72, 0.18, 1.0))
    mast_metal = model.material("mast_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_housing = model.material("dark_housing", rgba=(0.18, 0.19, 0.21, 1.0))
    bezel_metal = model.material("bezel_metal", rgba=(0.44, 0.47, 0.51, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.88, 0.92, 0.95, 0.45))

    base = model.part("base")
    base.visual(
        Box((0.68, 0.44, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=base_yellow,
        name="generator_body",
    )
    base.visual(
        Box((0.24, 0.22, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=base_yellow,
        name="tower_base",
    )
    base.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=mast_metal,
        name="mast_socket",
    )
    base.visual(
        Box((0.012, 0.18, 0.16)),
        origin=Origin(xyz=(0.346, 0.0, 0.11)),
        material=bezel_metal,
        name="service_door",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.68, 0.44, 0.56)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.065, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=mast_metal,
        name="lower_sleeve",
    )
    column.visual(
        Cylinder(radius=0.05, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=mast_metal,
        name="main_tube",
    )
    column.visual(
        Cylinder(radius=0.07, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=mast_metal,
        name="top_cap",
    )
    column.visual(
        Box((0.04, 0.36, 0.06)),
        origin=Origin(xyz=(0.015, 0.0, 0.94)),
        material=mast_metal,
        name="light_bar",
    )
    column.visual(
        Box((0.084, 0.14, 0.03)),
        origin=Origin(xyz=(0.037, -0.115, 0.91)),
        material=mast_metal,
        name="left_mount_beam",
    )
    column.visual(
        Box((0.084, 0.14, 0.03)),
        origin=Origin(xyz=(0.037, 0.115, 0.91)),
        material=mast_metal,
        name="right_mount_beam",
    )
    column.visual(
        Box((0.03, 0.01, 0.07)),
        origin=Origin(xyz=(0.09, -0.05, 0.96)),
        material=mast_metal,
        name="left_inner_plate",
    )
    column.visual(
        Box((0.03, 0.01, 0.07)),
        origin=Origin(xyz=(0.09, -0.18, 0.96)),
        material=mast_metal,
        name="left_outer_plate",
    )
    column.visual(
        Box((0.03, 0.01, 0.07)),
        origin=Origin(xyz=(0.09, 0.05, 0.96)),
        material=mast_metal,
        name="right_inner_plate",
    )
    column.visual(
        Box((0.03, 0.01, 0.07)),
        origin=Origin(xyz=(0.09, 0.18, 0.96)),
        material=mast_metal,
        name="right_outer_plate",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.07, length=1.04),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
    )

    def add_floodlight_head(part_name: str) -> None:
        head = model.part(part_name)
        head.visual(
            Box((0.10, 0.10, 0.078)),
            origin=Origin(xyz=(0.05, 0.0, 0.01)),
            material=dark_housing,
            name="housing",
        )
        head.visual(
            Box((0.014, 0.09, 0.012)),
            origin=Origin(xyz=(0.092, 0.0, 0.028)),
            material=bezel_metal,
            name="top_frame",
        )
        head.visual(
            Box((0.014, 0.09, 0.012)),
            origin=Origin(xyz=(0.092, 0.0, -0.006)),
            material=bezel_metal,
            name="bottom_frame",
        )
        head.visual(
            Box((0.014, 0.010, 0.056)),
            origin=Origin(xyz=(0.092, -0.040, 0.011)),
            material=bezel_metal,
            name="left_frame",
        )
        head.visual(
            Box((0.014, 0.010, 0.056)),
            origin=Origin(xyz=(0.092, 0.040, 0.011)),
            material=bezel_metal,
            name="right_frame",
        )
        head.visual(
            Box((0.004, 0.072, 0.048)),
            origin=Origin(xyz=(0.083, 0.0, 0.011)),
            material=lens_glass,
            name="lens",
        )
        head.visual(
            Box((0.008, 0.076, 0.006)),
            origin=Origin(xyz=(-0.004, 0.0, 0.024)),
            material=bezel_metal,
            name="upper_fin",
        )
        head.visual(
            Box((0.008, 0.076, 0.006)),
            origin=Origin(xyz=(-0.004, 0.0, 0.010)),
            material=bezel_metal,
            name="middle_fin",
        )
        head.visual(
            Box((0.008, 0.076, 0.006)),
            origin=Origin(xyz=(-0.004, 0.0, -0.004)),
            material=bezel_metal,
            name="lower_fin",
        )
        head.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mast_metal,
            name="neg_y_trunnion",
        )
        head.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mast_metal,
            name="pos_y_trunnion",
        )
        head.inertial = Inertial.from_geometry(
            Box((0.11, 0.11, 0.09)),
            mass=1.6,
            origin=Origin(xyz=(0.05, 0.0, 0.012)),
        )

    add_floodlight_head("left_head")
    add_floodlight_head("right_head")

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )
    model.articulation(
        "left_head_tilt",
        ArticulationType.REVOLUTE,
        parent=column,
        child="left_head",
        origin=Origin(xyz=(0.09, -0.115, 0.96)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.85,
            upper=0.75,
        ),
    )
    model.articulation(
        "right_head_tilt",
        ArticulationType.REVOLUTE,
        parent=column,
        child="right_head",
        origin=Origin(xyz=(0.09, 0.115, 0.96)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.85,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    left_head = object_model.get_part("left_head")
    right_head = object_model.get_part("right_head")
    left_head_tilt = object_model.get_articulation("left_head_tilt")
    right_head_tilt = object_model.get_articulation("right_head_tilt")

    mast_socket = base.get_visual("mast_socket")
    tower_base = base.get_visual("tower_base")
    lower_sleeve = column.get_visual("lower_sleeve")
    light_bar = column.get_visual("light_bar")
    left_inner_plate = column.get_visual("left_inner_plate")
    left_outer_plate = column.get_visual("left_outer_plate")
    right_inner_plate = column.get_visual("right_inner_plate")
    right_outer_plate = column.get_visual("right_outer_plate")
    left_housing = left_head.get_visual("housing")
    right_housing = right_head.get_visual("housing")
    left_neg_trunnion = left_head.get_visual("neg_y_trunnion")
    left_pos_trunnion = left_head.get_visual("pos_y_trunnion")
    right_neg_trunnion = right_head.get_visual("neg_y_trunnion")
    right_pos_trunnion = right_head.get_visual("pos_y_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.expect_gap(
        column,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_sleeve,
        negative_elem=mast_socket,
        name="column_sits_on_mast_socket",
    )
    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem=lower_sleeve,
        outer_elem=tower_base,
        name="column_centered_on_rectangular_tower_base",
    )
    ctx.expect_contact(
        left_head,
        column,
        elem_a=left_pos_trunnion,
        elem_b=left_inner_plate,
        name="left_head_inboard_pin_contact",
    )
    ctx.expect_contact(
        left_head,
        column,
        elem_a=left_neg_trunnion,
        elem_b=left_outer_plate,
        name="left_head_outboard_pin_contact",
    )
    ctx.expect_contact(
        right_head,
        column,
        elem_a=right_neg_trunnion,
        elem_b=right_inner_plate,
        name="right_head_inboard_pin_contact",
    )
    ctx.expect_contact(
        right_head,
        column,
        elem_a=right_pos_trunnion,
        elem_b=right_outer_plate,
        name="right_head_outboard_pin_contact",
    )
    ctx.expect_gap(
        right_head,
        left_head,
        axis="y",
        min_gap=0.10,
        name="paired_floodlights_have_clear_side_by_side_spacing",
    )
    ctx.expect_overlap(
        left_head,
        right_head,
        axes="xz",
        min_overlap=0.06,
        elem_a=left_housing,
        elem_b=right_housing,
        name="paired_floodlights_share_common_depth_and_height",
    )
    ctx.expect_gap(
        left_head,
        column,
        axis="x",
        min_gap=0.015,
        positive_elem=left_housing,
        negative_elem=light_bar,
        name="left_head_projects_forward_of_bar",
    )
    ctx.expect_gap(
        right_head,
        column,
        axis="x",
        min_gap=0.015,
        positive_elem=right_housing,
        negative_elem=light_bar,
        name="right_head_projects_forward_of_bar",
    )
    ctx.expect_origin_distance(
        left_head,
        right_head,
        axes="xz",
        max_dist=0.001,
        name="both_heads_share_same_top_mount_plane",
    )

    def housing_center(part_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="housing")
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    left_rest_center = housing_center(left_head)
    right_rest_center = housing_center(right_head)

    left_limits = left_head_tilt.motion_limits
    right_limits = right_head_tilt.motion_limits

    assert left_limits is not None
    assert right_limits is not None
    assert left_limits.lower is not None and left_limits.upper is not None
    assert right_limits.lower is not None and right_limits.upper is not None

    with ctx.pose({left_head_tilt: left_limits.lower}):
        left_low_center = housing_center(left_head)
        ctx.fail_if_parts_overlap_in_current_pose(name="left_head_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="left_head_lower_no_floating")
    with ctx.pose({right_head_tilt: right_limits.upper}):
        right_high_center = housing_center(right_head)
        ctx.fail_if_parts_overlap_in_current_pose(name="right_head_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="right_head_upper_no_floating")

    left_motion_ok = (
        left_rest_center is not None
        and left_low_center is not None
        and abs(left_low_center[1] - left_rest_center[1]) <= 1e-4
        and math.hypot(
            left_low_center[0] - left_rest_center[0],
            left_low_center[2] - left_rest_center[2],
        )
        >= 0.03
    )
    ctx.check(
        "left_head_rotates_in_xz_plane_about_y_axis",
        left_motion_ok,
        details=(
            f"rest={left_rest_center}, posed={left_low_center}; "
            "expected y to stay fixed while the housing center moves through the xz plane."
        ),
    )

    right_motion_ok = (
        right_rest_center is not None
        and right_high_center is not None
        and abs(right_high_center[1] - right_rest_center[1]) <= 1e-4
        and math.hypot(
            right_high_center[0] - right_rest_center[0],
            right_high_center[2] - right_rest_center[2],
        )
        >= 0.03
    )
    ctx.check(
        "right_head_rotates_in_xz_plane_about_y_axis",
        right_motion_ok,
        details=(
            f"rest={right_rest_center}, posed={right_high_center}; "
            "expected y to stay fixed while the housing center moves through the xz plane."
        ),
    )

    with ctx.pose({left_head_tilt: -0.65, right_head_tilt: 0.55}):
        ctx.expect_contact(
            left_head,
            column,
            elem_a=left_pos_trunnion,
            elem_b=left_inner_plate,
            name="left_head_stays_pinned_when_tilted",
        )
        ctx.expect_contact(
            right_head,
            column,
            elem_a=right_neg_trunnion,
            elem_b=right_inner_plate,
            name="right_head_stays_pinned_when_tilted",
        )
        ctx.expect_gap(
            right_head,
            left_head,
            axis="y",
            min_gap=0.10,
            name="heads_stay_clear_in_opposed_tilt_pose",
        )
        ctx.expect_gap(
            left_head,
            column,
            axis="x",
            min_gap=0.01,
            positive_elem=left_housing,
            negative_elem=light_bar,
            name="left_head_keeps_forward_clearance_in_tilt_pose",
        )
        ctx.expect_gap(
            right_head,
            column,
            axis="x",
            min_gap=0.01,
            positive_elem=right_housing,
            negative_elem=light_bar,
            name="right_head_keeps_forward_clearance_in_tilt_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
