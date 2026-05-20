from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


CASE_W = 0.24
CASE_D = 0.52
CASE_H = 0.56
BOTTOM_T = 0.016
TOP_T = 0.014
WALL_T = 0.014
FRONT_T = 0.012

DOOR_T = 0.010
DOOR_FRAME = 0.012
DOOR_Y0 = -CASE_D / 2.0 + 0.018
DOOR_D = CASE_D - 0.036
DOOR_Z0 = 0.055
DOOR_H = 0.470
LEFT_REAR_SPINE_D = 0.030
LEFT_FRONT_SPINE_D = 0.030
LEFT_FRAME_RUN_D = CASE_D - LEFT_REAR_SPINE_D - LEFT_FRONT_SPINE_D
HINGE_BARREL_R = 0.005
HINGE_BARREL_L = 0.085

BAY_W = 0.170
BAY_FACE_W = 0.164
BAY_FACE_H = 0.020
TRAY_BODY_W = 0.158
TRAY_BODY_H = 0.018
TRAY_BODY_D = 0.340
TRAY_FACE_T = 0.006
TRAY_TRAVEL = 0.130
UPPER_BAY_Z = 0.472
LOWER_BAY_Z = 0.438


def _box_visual(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _axis_matches(actual, expected, tol=1e-9):
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_tower_gaming_pc", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.22, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.34, 0.42, 0.48, 0.22))
    tray_black = model.material("tray_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_cyan = model.material("accent_cyan", rgba=(0.12, 0.72, 0.95, 1.0))

    chassis = model.part("chassis")

    # Main tower shell with an open left side for the glass door.
    _box_visual(
        chassis,
        (CASE_W, CASE_D, BOTTOM_T),
        (0.0, 0.0, BOTTOM_T / 2.0),
        dark_metal,
        "bottom_panel",
    )
    _box_visual(
        chassis,
        (CASE_W, CASE_D, TOP_T),
        (0.0, 0.0, CASE_H - TOP_T / 2.0),
        dark_metal,
        "top_panel",
    )
    _box_visual(
        chassis,
        (WALL_T, CASE_D, CASE_H - BOTTOM_T - TOP_T),
        (CASE_W / 2.0 - WALL_T / 2.0, 0.0, (CASE_H + BOTTOM_T - TOP_T) / 2.0),
        dark_metal,
        "right_wall",
    )
    _box_visual(
        chassis,
        (CASE_W, WALL_T, CASE_H - BOTTOM_T - TOP_T),
        (0.0, -CASE_D / 2.0 + WALL_T / 2.0, (CASE_H + BOTTOM_T - TOP_T) / 2.0),
        matte_black,
        "rear_wall",
    )

    # Left-side perimeter structure around the glass opening.
    _box_visual(
        chassis,
        (DOOR_FRAME, LEFT_REAR_SPINE_D, CASE_H - BOTTOM_T - TOP_T),
        (
            -CASE_W / 2.0 + DOOR_FRAME / 2.0,
            -CASE_D / 2.0 + LEFT_REAR_SPINE_D / 2.0,
            (CASE_H + BOTTOM_T - TOP_T) / 2.0,
        ),
        dark_metal,
        "left_rear_spine",
    )
    _box_visual(
        chassis,
        (DOOR_FRAME, LEFT_FRONT_SPINE_D, CASE_H - BOTTOM_T - TOP_T),
        (
            -CASE_W / 2.0 + DOOR_FRAME / 2.0,
            CASE_D / 2.0 - LEFT_FRONT_SPINE_D / 2.0,
            (CASE_H + BOTTOM_T - TOP_T) / 2.0,
        ),
        dark_metal,
        "left_front_spine",
    )
    _box_visual(
        chassis,
        (DOOR_FRAME, LEFT_FRAME_RUN_D, DOOR_FRAME),
        (
            -CASE_W / 2.0 + DOOR_FRAME / 2.0,
            0.0,
            DOOR_Z0 + DOOR_H - DOOR_FRAME / 2.0,
        ),
        dark_metal,
        "left_top_rail",
    )
    _box_visual(
        chassis,
        (DOOR_FRAME, LEFT_FRAME_RUN_D, DOOR_FRAME),
        (-CASE_W / 2.0 + DOOR_FRAME / 2.0, 0.0, DOOR_Z0 + DOOR_FRAME / 2.0),
        dark_metal,
        "left_bottom_rail",
    )
    _box_visual(
        chassis,
        (DOOR_FRAME, LEFT_FRONT_SPINE_D, CASE_H - BOTTOM_T - TOP_T),
        (
            -CASE_W / 2.0 + DOOR_FRAME / 2.0,
            CASE_D / 2.0 - LEFT_FRONT_SPINE_D / 2.0,
            (CASE_H + BOTTOM_T - TOP_T) / 2.0,
        ),
        dark_metal,
        "left_front_rail",
    )

    # Front fascia with two optical drive openings.
    side_strip_w = (CASE_W - BAY_W) / 2.0
    _box_visual(
        chassis,
        (side_strip_w, FRONT_T, CASE_H - BOTTOM_T),
        (-CASE_W / 2.0 + side_strip_w / 2.0, CASE_D / 2.0 - FRONT_T / 2.0, (CASE_H + BOTTOM_T) / 2.0),
        matte_black,
        "front_left_strip",
    )
    _box_visual(
        chassis,
        (side_strip_w, FRONT_T, CASE_H - BOTTOM_T),
        (CASE_W / 2.0 - side_strip_w / 2.0, CASE_D / 2.0 - FRONT_T / 2.0, (CASE_H + BOTTOM_T) / 2.0),
        matte_black,
        "front_right_strip",
    )
    _box_visual(
        chassis,
        (BAY_W, FRONT_T, 0.030),
        (0.0, CASE_D / 2.0 - FRONT_T / 2.0, 0.512),
        matte_black,
        "front_top_strip",
    )
    _box_visual(
        chassis,
        (BAY_W, FRONT_T, 0.012),
        (0.0, CASE_D / 2.0 - FRONT_T / 2.0, 0.455),
        matte_black,
        "front_bay_divider",
    )
    _box_visual(
        chassis,
        (BAY_W, FRONT_T, 0.400),
        (0.0, CASE_D / 2.0 - FRONT_T / 2.0, 0.213),
        matte_black,
        "front_lower_panel",
    )

    # Gaming-style front accents.
    _box_visual(
        chassis,
        (0.008, 0.004, 0.240),
        (-0.088, CASE_D / 2.0 + 0.002, 0.180),
        accent_cyan,
        "front_rgb_left",
    )
    _box_visual(
        chassis,
        (0.008, 0.004, 0.240),
        (0.088, CASE_D / 2.0 + 0.002, 0.180),
        accent_cyan,
        "front_rgb_right",
    )

    # Internal tray guides and a visible PSU shroud floor to help the tower read through glass.
    guide_z_offset = TRAY_BODY_H / 2.0 + 0.0015
    _box_visual(
        chassis,
        (TRAY_BODY_W, TRAY_BODY_D, 0.003),
        (0.0, CASE_D / 2.0 - TRAY_BODY_D / 2.0, UPPER_BAY_Z - guide_z_offset),
        dark_metal,
        "upper_bay_floor",
    )
    _box_visual(
        chassis,
        (TRAY_BODY_W, TRAY_BODY_D, 0.003),
        (0.0, CASE_D / 2.0 - TRAY_BODY_D / 2.0, LOWER_BAY_Z - guide_z_offset),
        dark_metal,
        "lower_bay_floor",
    )
    bay_cage_x = TRAY_BODY_W / 2.0 + 0.002
    bay_cage_h = (UPPER_BAY_Z - guide_z_offset - 0.0015) - (LOWER_BAY_Z - guide_z_offset + 0.0015)
    bay_cage_z = ((UPPER_BAY_Z - guide_z_offset - 0.0015) + (LOWER_BAY_Z - guide_z_offset + 0.0015)) / 2.0
    _box_visual(
        chassis,
        (0.004, TRAY_BODY_D, bay_cage_h),
        (bay_cage_x, CASE_D / 2.0 - TRAY_BODY_D / 2.0, bay_cage_z),
        dark_metal,
        "bay_cage_right",
    )
    _box_visual(
        chassis,
        (0.004, TRAY_BODY_D, bay_cage_h),
        (-bay_cage_x, CASE_D / 2.0 - TRAY_BODY_D / 2.0, bay_cage_z),
        dark_metal,
        "bay_cage_left",
    )
    _box_visual(
        chassis,
        (CASE_W - WALL_T, CASE_D - 0.090, 0.090),
        (0.0, -0.010, 0.061),
        dark_metal,
        "psu_shroud",
    )
    _box_visual(
        chassis,
        (0.150, 0.018, 0.295),
        (-0.010, -CASE_D / 2.0 + WALL_T + 0.009, 0.282),
        matte_black,
        "motherboard_tray",
    )
    _box_visual(
        chassis,
        (0.080, 0.020, 0.080),
        (-0.012, -CASE_D / 2.0 + WALL_T + 0.028, 0.330),
        dark_metal,
        "cpu_cooler",
    )
    _box_visual(
        chassis,
        (0.145, 0.278, 0.034),
        (0.008, -0.089, 0.292),
        matte_black,
        "gpu_card",
    )
    _box_visual(
        chassis,
        (0.140, 0.006, 0.020),
        (0.010, 0.053, 0.300),
        accent_cyan,
        "gpu_accent_bar",
    )
    _box_visual(
        chassis,
        (0.090, 0.036, 0.010),
        (0.020, 0.130, CASE_H - TOP_T - 0.005),
        dark_metal,
        "top_io_cluster",
    )
    chassis.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(-0.050, 0.135, CASE_H + 0.002)),
        material=accent_cyan,
        name="power_button",
    )

    # Feet.
    foot_xy = 0.028
    foot_h = 0.012
    for index, (x, y) in enumerate(
        (
            (-0.085, -0.200),
            (0.085, -0.200),
            (-0.085, 0.200),
            (0.085, 0.200),
        ),
        start=1,
    ):
        _box_visual(
            chassis,
            (foot_xy, foot_xy, foot_h),
            (x, y, foot_h / 2.0),
            matte_black,
            f"foot_{index}",
        )

    side_panel = model.part("side_panel")
    door_z_center = DOOR_Z0 + DOOR_H / 2.0
    _box_visual(
        side_panel,
        (DOOR_T, DOOR_D, DOOR_FRAME),
        (0.0, DOOR_D / 2.0, DOOR_H / 2.0 - DOOR_FRAME / 2.0),
        matte_black,
        "door_top_frame",
    )
    _box_visual(
        side_panel,
        (DOOR_T, DOOR_D, DOOR_FRAME),
        (0.0, DOOR_D / 2.0, -DOOR_H / 2.0 + DOOR_FRAME / 2.0),
        matte_black,
        "door_bottom_frame",
    )
    _box_visual(
        side_panel,
        (DOOR_T, DOOR_FRAME, DOOR_H),
        (0.0, DOOR_FRAME / 2.0, 0.0),
        matte_black,
        "door_rear_frame",
    )
    _box_visual(
        side_panel,
        (DOOR_T, DOOR_FRAME, DOOR_H),
        (0.0, DOOR_D - DOOR_FRAME / 2.0, 0.0),
        matte_black,
        "door_front_frame",
    )
    _box_visual(
        side_panel,
        (0.005, DOOR_D - 2.0 * DOOR_FRAME, DOOR_H - 2.0 * DOOR_FRAME),
        (0.0, DOOR_D / 2.0, 0.0),
        smoked_glass,
        "door_glass",
    )
    _box_visual(
        side_panel,
        (0.012, 0.090, 0.010),
        (-0.003, DOOR_D - 0.026, 0.0),
        matte_black,
        "door_handle",
    )
    side_panel.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=Origin(xyz=(0.0, DOOR_FRAME / 2.0, DOOR_H / 2.0 - 0.070)),
        material=dark_metal,
        name="upper_hinge_barrel",
    )
    side_panel.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=Origin(xyz=(0.0, DOOR_FRAME / 2.0, -DOOR_H / 2.0 + 0.070)),
        material=dark_metal,
        name="lower_hinge_barrel",
    )

    upper_tray = model.part("upper_optical_tray")
    _box_visual(
        upper_tray,
        (BAY_FACE_W, TRAY_FACE_T, BAY_FACE_H),
        (0.0, TRAY_FACE_T / 2.0, 0.0),
        tray_black,
        "upper_faceplate",
    )
    _box_visual(
        upper_tray,
        (TRAY_BODY_W, TRAY_BODY_D, TRAY_BODY_H),
        (0.0, -TRAY_BODY_D / 2.0, 0.0),
        dark_metal,
        "upper_body",
    )
    _box_visual(
        upper_tray,
        (0.014, 0.003, 0.004),
        (BAY_FACE_W / 2.0 - 0.012, TRAY_FACE_T + 0.0015, -BAY_FACE_H / 2.0 + 0.004),
        tray_black,
        "upper_eject_button",
    )

    lower_tray = model.part("lower_optical_tray")
    _box_visual(
        lower_tray,
        (BAY_FACE_W, TRAY_FACE_T, BAY_FACE_H),
        (0.0, TRAY_FACE_T / 2.0, 0.0),
        tray_black,
        "lower_faceplate",
    )
    _box_visual(
        lower_tray,
        (TRAY_BODY_W, TRAY_BODY_D, TRAY_BODY_H),
        (0.0, -TRAY_BODY_D / 2.0, 0.0),
        dark_metal,
        "lower_body",
    )
    _box_visual(
        lower_tray,
        (0.014, 0.003, 0.004),
        (BAY_FACE_W / 2.0 - 0.012, TRAY_FACE_T + 0.0015, -BAY_FACE_H / 2.0 + 0.004),
        tray_black,
        "lower_eject_button",
    )

    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(-CASE_W / 2.0 - DOOR_T / 2.0, DOOR_Y0, door_z_center)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=0.0, upper=1.30),
    )
    model.articulation(
        "upper_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=upper_tray,
        origin=Origin(xyz=(0.0, CASE_D / 2.0, UPPER_BAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.30, lower=0.0, upper=TRAY_TRAVEL),
    )
    model.articulation(
        "lower_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=lower_tray,
        origin=Origin(xyz=(0.0, CASE_D / 2.0, LOWER_BAY_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.30, lower=0.0, upper=TRAY_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    upper_tray = object_model.get_part("upper_optical_tray")
    lower_tray = object_model.get_part("lower_optical_tray")

    side_panel_hinge = object_model.get_articulation("side_panel_hinge")
    upper_tray_slide = object_model.get_articulation("upper_tray_slide")
    lower_tray_slide = object_model.get_articulation("lower_tray_slide")

    left_rear_spine = chassis.get_visual("left_rear_spine")
    left_top_rail = chassis.get_visual("left_top_rail")
    left_bottom_rail = chassis.get_visual("left_bottom_rail")
    front_top_strip = chassis.get_visual("front_top_strip")
    front_bay_divider = chassis.get_visual("front_bay_divider")
    upper_bay_floor = chassis.get_visual("upper_bay_floor")
    lower_bay_floor = chassis.get_visual("lower_bay_floor")
    bay_cage_right = chassis.get_visual("bay_cage_right")
    bay_cage_left = chassis.get_visual("bay_cage_left")

    door_top_frame = side_panel.get_visual("door_top_frame")
    door_bottom_frame = side_panel.get_visual("door_bottom_frame")
    door_glass = side_panel.get_visual("door_glass")
    door_handle = side_panel.get_visual("door_handle")
    upper_hinge_barrel = side_panel.get_visual("upper_hinge_barrel")
    lower_hinge_barrel = side_panel.get_visual("lower_hinge_barrel")

    upper_faceplate = upper_tray.get_visual("upper_faceplate")
    upper_body = upper_tray.get_visual("upper_body")
    lower_faceplate = lower_tray.get_visual("lower_faceplate")
    lower_body = lower_tray.get_visual("lower_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64, overlap_tol=0.0005, overlap_volume_tol=0.0)

    ctx.check(
        "side_panel_hinge_axis_is_vertical",
        _axis_matches(side_panel_hinge.axis, (0.0, 0.0, 1.0)),
        details=f"Expected vertical hinge axis, got {side_panel_hinge.axis!r}",
    )
    ctx.check(
        "upper_tray_slide_axis_is_forward",
        _axis_matches(upper_tray_slide.axis, (0.0, 1.0, 0.0)),
        details=f"Expected +Y tray slide axis, got {upper_tray_slide.axis!r}",
    )
    ctx.check(
        "lower_tray_slide_axis_is_forward",
        _axis_matches(lower_tray_slide.axis, (0.0, 1.0, 0.0)),
        details=f"Expected +Y tray slide axis, got {lower_tray_slide.axis!r}",
    )

    side_limits = side_panel_hinge.motion_limits
    upper_limits = upper_tray_slide.motion_limits
    lower_limits = lower_tray_slide.motion_limits
    ctx.check(
        "side_panel_hinge_range_is_realistic",
        side_limits is not None
        and side_limits.lower == 0.0
        and side_limits.upper is not None
        and 1.10 <= side_limits.upper <= 1.35,
        details=f"Unexpected side-panel limits: {side_limits!r}",
    )
    ctx.check(
        "upper_tray_travel_is_realistic",
        upper_limits is not None and upper_limits.lower == 0.0 and upper_limits.upper == TRAY_TRAVEL,
        details=f"Unexpected upper-tray limits: {upper_limits!r}",
    )
    ctx.check(
        "lower_tray_travel_is_realistic",
        lower_limits is not None and lower_limits.lower == 0.0 and lower_limits.upper == TRAY_TRAVEL,
        details=f"Unexpected lower-tray limits: {lower_limits!r}",
    )

    # Full-tower envelope and dominant side opening.
    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.18)
    ctx.expect_contact(side_panel, chassis, elem_a=door_top_frame, elem_b=left_top_rail)
    ctx.expect_contact(side_panel, chassis, elem_a=door_bottom_frame, elem_b=left_bottom_rail)
    ctx.expect_contact(side_panel, chassis, elem_a=door_top_frame, elem_b=left_rear_spine)
    ctx.expect_contact(side_panel, chassis, elem_a=upper_hinge_barrel, elem_b=left_rear_spine)
    ctx.expect_contact(side_panel, chassis, elem_a=lower_hinge_barrel, elem_b=left_rear_spine)

    # Both optical trays are mounted in the upper-front drive stack.
    ctx.expect_within(upper_tray, chassis, axes="xz")
    ctx.expect_within(lower_tray, chassis, axes="xz")
    ctx.expect_contact(upper_tray, chassis, elem_a=upper_body, elem_b=upper_bay_floor)
    ctx.expect_contact(lower_tray, chassis, elem_a=lower_body, elem_b=lower_bay_floor)
    ctx.expect_contact(chassis, chassis, elem_a=lower_bay_floor, elem_b=bay_cage_right)
    ctx.expect_contact(chassis, chassis, elem_a=lower_bay_floor, elem_b=bay_cage_left)
    ctx.expect_contact(chassis, chassis, elem_a=upper_bay_floor, elem_b=bay_cage_right)
    ctx.expect_contact(chassis, chassis, elem_a=upper_bay_floor, elem_b=bay_cage_left)
    ctx.expect_gap(
        upper_tray,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_faceplate,
        negative_elem=front_top_strip,
    )
    ctx.expect_gap(
        lower_tray,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_faceplate,
        negative_elem=front_bay_divider,
    )
    ctx.expect_gap(
        upper_tray,
        lower_tray,
        axis="z",
        min_gap=0.010,
        positive_elem=upper_faceplate,
        negative_elem=lower_faceplate,
    )

    # Key articulated poses: hinged glass swings outward and both optical trays eject forward.
    with ctx.pose({side_panel_hinge: side_limits.upper}):
        ctx.expect_gap(
            chassis,
            side_panel,
            axis="x",
            min_gap=0.180,
            negative_elem=door_handle,
        )
        ctx.expect_overlap(side_panel, chassis, axes="z", min_overlap=0.40, elem_a=door_glass)
    with ctx.pose({upper_tray_slide: upper_limits.upper}):
        ctx.expect_gap(
            upper_tray,
            chassis,
            axis="y",
            min_gap=TRAY_TRAVEL - 0.010,
            positive_elem=upper_faceplate,
            negative_elem=front_top_strip,
        )
        ctx.expect_contact(upper_tray, chassis, elem_a=upper_body, elem_b=upper_bay_floor)
        ctx.expect_within(upper_tray, chassis, axes="xz", margin=0.001)
    with ctx.pose({lower_tray_slide: lower_limits.upper}):
        ctx.expect_gap(
            lower_tray,
            chassis,
            axis="y",
            min_gap=TRAY_TRAVEL - 0.010,
            positive_elem=lower_faceplate,
            negative_elem=front_bay_divider,
        )
        ctx.expect_contact(lower_tray, chassis, elem_a=lower_body, elem_b=lower_bay_floor)
        ctx.expect_within(lower_tray, chassis, axes="xz", margin=0.001)

    for articulation in (side_panel_hinge, upper_tray_slide, lower_tray_slide):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
