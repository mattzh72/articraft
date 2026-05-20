from __future__ import annotations

from pathlib import Path

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
HERE = Path(__file__).resolve().parent

CASE_W = 0.180
CASE_D = 0.350
CASE_H = 0.365
STEEL_T = 0.0012
BEZEL_T = 0.010
LEFT_FRAME_T = 0.004
PANEL_T = 0.0010
HINGE_R = 0.0035

BAY_W = 0.146
BAY_H = 0.042
BAY_DEPTH = 0.170
BAY_FLOOR_T = 0.002
TRAY_W = 0.134
TRAY_D = 0.162
TRAY_BOTTOM_T = 0.002
TRAY_FACE_T = 0.004
TRAY_FACE_H = 0.038


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_tower_pc", assets=ASSETS)

    half_w = CASE_W * 0.5
    half_d = CASE_D * 0.5

    panel_axis_x = -half_w + PANEL_T + HINGE_R
    panel_axis_y = -half_d + 0.010
    panel_height = CASE_H - 0.024
    panel_depth = CASE_D - 0.020 - HINGE_R

    bay_bottom_z = 0.286
    bay_top_z = bay_bottom_z + BAY_H
    bay_center_y = half_d - BEZEL_T - BAY_DEPTH * 0.5
    tray_closed_y = half_d - (TRAY_D * 0.5 + TRAY_FACE_T * 0.5)
    tray_origin_z = bay_bottom_z + BAY_FLOOR_T
    left_frame_x = panel_axis_x + HINGE_R + LEFT_FRAME_T * 0.5

    steel = model.material("steel_case", rgba=(0.58, 0.60, 0.63, 1.0))
    bezel = model.material("front_bezel", rgba=(0.13, 0.14, 0.15, 1.0))
    tray_black = model.material("tray_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.06, 0.06, 0.07, 1.0))
    led_blue = model.material("led_blue", rgba=(0.32, 0.58, 0.92, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((CASE_W, CASE_D, STEEL_T)),
        origin=Origin(xyz=(0.0, 0.0, STEEL_T * 0.5)),
        material=steel,
        name="bottom_floor",
    )
    chassis.visual(
        Box((CASE_W, CASE_D, STEEL_T)),
        origin=Origin(xyz=(0.0, 0.0, CASE_H - STEEL_T * 0.5)),
        material=steel,
        name="top_cover",
    )
    chassis.visual(
        Box((STEEL_T, CASE_D, CASE_H - 2.0 * STEEL_T)),
        origin=Origin(xyz=(half_w - STEEL_T * 0.5, 0.0, CASE_H * 0.5)),
        material=steel,
        name="right_wall",
    )
    chassis.visual(
        Box((CASE_W - STEEL_T, STEEL_T, CASE_H - 2.0 * STEEL_T)),
        origin=Origin(xyz=(0.0, -half_d + STEEL_T * 0.5, CASE_H * 0.5)),
        material=steel,
        name="rear_wall",
    )
    chassis.visual(
        Box((LEFT_FRAME_T, CASE_D - 0.020, 0.010)),
        origin=Origin(xyz=(left_frame_x, 0.0, STEEL_T + 0.005)),
        material=steel,
        name="left_bottom_rail",
    )
    chassis.visual(
        Box((LEFT_FRAME_T, CASE_D - 0.020, 0.010)),
        origin=Origin(xyz=(left_frame_x, 0.0, CASE_H - STEEL_T - 0.005)),
        material=steel,
        name="left_top_rail",
    )
    chassis.visual(
        Box((LEFT_FRAME_T, 0.010, panel_height)),
        origin=Origin(xyz=(left_frame_x, half_d - 0.015, CASE_H * 0.5)),
        material=steel,
        name="left_front_rail",
    )
    chassis.visual(
        Box((LEFT_FRAME_T, 0.010, CASE_H - 2.0 * STEEL_T)),
        origin=Origin(xyz=(left_frame_x, panel_axis_y + 0.005, CASE_H * 0.5)),
        material=steel,
        name="hinge_jamb",
    )
    chassis.visual(
        Box((CASE_W - STEEL_T, BEZEL_T, bay_bottom_z - 0.002)),
        origin=Origin(xyz=(0.0, half_d - BEZEL_T * 0.5, (bay_bottom_z - 0.002) * 0.5)),
        material=bezel,
        name="front_lower_bezel",
    )
    chassis.visual(
        Box((CASE_W - STEEL_T, BEZEL_T, CASE_H - bay_top_z)),
        origin=Origin(
            xyz=(0.0, half_d - BEZEL_T * 0.5, bay_top_z + (CASE_H - bay_top_z) * 0.5)
        ),
        material=bezel,
        name="front_upper_strip",
    )
    chassis.visual(
        Box(((CASE_W - BAY_W) * 0.5, BEZEL_T, BAY_H)),
        origin=Origin(
            xyz=(-(CASE_W - BAY_W) * 0.25, half_d - BEZEL_T * 0.5, bay_bottom_z + BAY_H * 0.5)
        ),
        material=bezel,
        name="front_left_bay_post",
    )
    chassis.visual(
        Box(((CASE_W - BAY_W) * 0.5, BEZEL_T, BAY_H)),
        origin=Origin(
            xyz=(((CASE_W - BAY_W) * 0.25), half_d - BEZEL_T * 0.5, bay_bottom_z + BAY_H * 0.5)
        ),
        material=bezel,
        name="front_right_bay_post",
    )
    chassis.visual(
        Box((BAY_W, BAY_DEPTH, BAY_FLOOR_T)),
        origin=Origin(xyz=(0.0, bay_center_y, bay_bottom_z - BAY_FLOOR_T * 0.5)),
        material=steel,
        name="bay_floor",
    )
    chassis.visual(
        Box((BAY_W, BAY_DEPTH, BAY_FLOOR_T)),
        origin=Origin(xyz=(0.0, bay_center_y, bay_top_z - BAY_FLOOR_T * 0.5)),
        material=steel,
        name="bay_ceiling",
    )
    chassis.visual(
        Box((BAY_FLOOR_T, BAY_DEPTH, BAY_H - 2.0 * BAY_FLOOR_T)),
        origin=Origin(xyz=(-BAY_W * 0.5 - BAY_FLOOR_T * 0.5, bay_center_y, bay_bottom_z + BAY_H * 0.5)),
        material=steel,
        name="bay_left_wall",
    )
    chassis.visual(
        Box((BAY_FLOOR_T, BAY_DEPTH, BAY_H - 2.0 * BAY_FLOOR_T)),
        origin=Origin(xyz=(BAY_W * 0.5 + BAY_FLOOR_T * 0.5, bay_center_y, bay_bottom_z + BAY_H * 0.5)),
        material=steel,
        name="bay_right_wall",
    )
    chassis.visual(
        Box((0.010, BAY_DEPTH - 0.020, BAY_FLOOR_T)),
        origin=Origin(xyz=(-0.054, bay_center_y, tray_origin_z - BAY_FLOOR_T * 0.5)),
        material=steel,
        name="bay_runner_left",
    )
    chassis.visual(
        Box((0.010, BAY_DEPTH - 0.020, BAY_FLOOR_T)),
        origin=Origin(xyz=(0.054, bay_center_y, tray_origin_z - BAY_FLOOR_T * 0.5)),
        material=steel,
        name="bay_runner_right",
    )
    chassis.visual(
        Box((0.100, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, tray_closed_y - TRAY_D * 0.5 - 0.002, tray_origin_z + 0.004)),
        material=dark_trim,
        name="bay_rear_stop",
    )
    chassis.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(
            xyz=(0.055, half_d - 0.002, 0.258),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_trim,
        name="power_button",
    )
    chassis.visual(
        Box((0.012, 0.0015, 0.005)),
        origin=Origin(xyz=(0.055, half_d - 0.00075, 0.246)),
        material=led_blue,
        name="power_led",
    )
    for foot_x in (-0.050, 0.050):
        for foot_y in (-0.120, 0.120):
            chassis.visual(
                Box((0.020, 0.020, 0.006)),
                origin=Origin(xyz=(foot_x, foot_y, -0.003)),
                material=dark_trim,
                name=f"foot_{'l' if foot_x < 0 else 'r'}_{'rear' if foot_y < 0 else 'front'}",
            )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, CASE_H * 0.5)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((PANEL_T, panel_depth, panel_height)),
        origin=Origin(
            xyz=(-HINGE_R, HINGE_R * 0.5 + panel_depth * 0.5, 0.0)
        ),
        material=steel,
        name="panel_leaf",
    )
    side_panel.visual(
        Cylinder(radius=HINGE_R, length=panel_height - 0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_trim,
        name="panel_hinge_barrel",
    )
    side_panel.visual(
        Box((0.008, 0.018, 0.100)),
        origin=Origin(
            xyz=(-(HINGE_R + PANEL_T + 0.0035), HINGE_R + panel_depth - 0.020, 0.0)
        ),
        material=dark_trim,
        name="panel_grip",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((0.014, panel_depth, panel_height)),
        mass=0.80,
        origin=Origin(xyz=(-0.007, HINGE_R + panel_depth * 0.5, 0.0)),
    )

    tray = model.part("optical_tray")
    tray.visual(
        Box((TRAY_W, TRAY_D, TRAY_BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_BOTTOM_T * 0.5)),
        material=tray_black,
        name="tray_bottom",
    )
    tray.visual(
        Box((0.003, TRAY_D - 0.014, 0.012)),
        origin=Origin(xyz=(-TRAY_W * 0.5 + 0.0015, 0.0, 0.007)),
        material=tray_black,
        name="tray_left_lip",
    )
    tray.visual(
        Box((0.003, TRAY_D - 0.014, 0.012)),
        origin=Origin(xyz=(TRAY_W * 0.5 - 0.0015, 0.0, 0.007)),
        material=tray_black,
        name="tray_right_lip",
    )
    tray.visual(
        Box((TRAY_W, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -TRAY_D * 0.5 + 0.0015, 0.007)),
        material=tray_black,
        name="tray_rear_wall",
    )
    tray.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, -0.010, 0.002)),
        material=dark_trim,
        name="tray_spindle",
    )
    tray.visual(
        Box((BAY_W - 0.002, TRAY_FACE_T, TRAY_FACE_H)),
        origin=Origin(xyz=(0.0, TRAY_D * 0.5 + TRAY_FACE_T * 0.5, TRAY_FACE_H * 0.5)),
        material=tray_black,
        name="tray_face",
    )
    tray.visual(
        Box((0.010, 0.002, 0.005)),
        origin=Origin(xyz=(0.052, TRAY_D * 0.5 + TRAY_FACE_T + 0.001, 0.007)),
        material=dark_trim,
        name="eject_button",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_W, TRAY_D + TRAY_FACE_T, TRAY_FACE_H)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, TRAY_FACE_H * 0.5)),
    )

    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(panel_axis_x, panel_axis_y, CASE_H * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.92,
        ),
    )
    model.articulation(
        "optical_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=tray,
        origin=Origin(xyz=(0.0, tray_closed_y, tray_origin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    tray = object_model.get_part("optical_tray")
    side_panel_hinge = object_model.get_articulation("side_panel_hinge")
    optical_tray_slide = object_model.get_articulation("optical_tray_slide")

    panel_leaf = side_panel.get_visual("panel_leaf")
    panel_hinge_barrel = side_panel.get_visual("panel_hinge_barrel")
    panel_grip = side_panel.get_visual("panel_grip")
    tray_bottom = tray.get_visual("tray_bottom")
    tray_face = tray.get_visual("tray_face")
    tray_rear_wall = tray.get_visual("tray_rear_wall")
    hinge_jamb = chassis.get_visual("hinge_jamb")
    left_front_rail = chassis.get_visual("left_front_rail")
    bay_floor = chassis.get_visual("bay_floor")
    bay_runner_left = chassis.get_visual("bay_runner_left")
    bay_runner_right = chassis.get_visual("bay_runner_right")
    bay_rear_stop = chassis.get_visual("bay_rear_stop")
    front_left_bay_post = chassis.get_visual("front_left_bay_post")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "side_panel_hinge_axis",
        tuple(side_panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical hinge axis, got {side_panel_hinge.axis!r}",
    )
    ctx.check(
        "tray_slide_axis",
        tuple(optical_tray_slide.axis) == (0.0, 1.0, 0.0),
        details=f"Expected front-to-back tray slide axis, got {optical_tray_slide.axis!r}",
    )
    panel_limits = side_panel_hinge.motion_limits
    tray_limits = optical_tray_slide.motion_limits
    ctx.check(
        "side_panel_opening_range",
        panel_limits is not None
        and panel_limits.lower == 0.0
        and panel_limits.upper is not None
        and 1.7 <= panel_limits.upper <= 2.0,
        details=f"Unexpected side-panel motion limits: {panel_limits!r}",
    )
    ctx.check(
        "tray_extension_range",
        tray_limits is not None
        and tray_limits.lower == 0.0
        and tray_limits.upper is not None
        and 0.07 <= tray_limits.upper <= 0.10,
        details=f"Unexpected tray motion limits: {tray_limits!r}",
    )

    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.26)
    ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_barrel, elem_b=hinge_jamb)
    ctx.expect_gap(
        chassis,
        side_panel,
        axis="x",
        max_gap=0.008,
        max_penetration=0.0,
        positive_elem=hinge_jamb,
        negative_elem=panel_leaf,
        name="panel_closed_flush_to_case",
    )

    ctx.expect_origin_distance(tray, chassis, axes="x", max_dist=0.001)
    ctx.expect_overlap(tray, chassis, axes="x", min_overlap=0.13)
    ctx.expect_overlap(tray, chassis, axes="z", min_overlap=0.03)
    ctx.expect_contact(tray, chassis, elem_a=tray_bottom, elem_b=bay_runner_left)
    ctx.expect_contact(tray, chassis, elem_a=tray_bottom, elem_b=bay_runner_right)
    ctx.expect_contact(tray, chassis, elem_a=tray_rear_wall, elem_b=bay_rear_stop)
    ctx.expect_gap(
        tray,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_bottom,
        negative_elem=bay_runner_left,
        name="tray_seated_on_runner_height",
    )
    ctx.expect_within(tray, chassis, axes="x", inner_elem=tray_face, outer_elem=bay_floor, margin=0.0)

    with ctx.pose({side_panel_hinge: 1.60}):
        ctx.expect_contact(side_panel, chassis, elem_a=panel_hinge_barrel, elem_b=hinge_jamb)
        ctx.expect_gap(
            chassis,
            side_panel,
            axis="x",
            min_gap=0.060,
            positive_elem=left_front_rail,
            negative_elem=panel_grip,
            name="panel_opens_clear_of_left_side",
        )

    with ctx.pose({optical_tray_slide: 0.080}):
        ctx.expect_contact(tray, chassis, elem_a=tray_bottom, elem_b=bay_runner_left)
        ctx.expect_gap(
            tray,
            chassis,
            axis="y",
            min_gap=0.070,
            positive_elem=tray_face,
            negative_elem=front_left_bay_post,
            name="tray_extends_beyond_front_bezel",
        )

    for joint in (side_panel_hinge, optical_tray_slide):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
