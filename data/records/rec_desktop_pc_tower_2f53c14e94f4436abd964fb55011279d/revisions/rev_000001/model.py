from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="rackmount_desktop_pc")

    brushed_aluminum = model.material(
        "brushed_aluminum",
        rgba=(0.74, 0.76, 0.79, 1.0),
    )
    dark_anodized = model.material(
        "dark_anodized",
        rgba=(0.20, 0.22, 0.24, 1.0),
    )
    tray_black = model.material(
        "tray_black",
        rgba=(0.11, 0.12, 0.13, 1.0),
    )
    hinge_steel = model.material(
        "hinge_steel",
        rgba=(0.38, 0.40, 0.42, 1.0),
    )
    indicator_blue = model.material(
        "indicator_blue",
        rgba=(0.25, 0.54, 0.90, 1.0),
    )
    button_black = model.material(
        "button_black",
        rgba=(0.08, 0.09, 0.10, 1.0),
    )

    body_depth = 0.420
    body_width = 0.444
    rack_width = 0.483
    body_height = 0.088

    floor_thickness = 0.003
    side_thickness = 0.0025
    front_thickness = 0.004
    rear_thickness = 0.003
    ear_overlap = 0.0005

    front_x = body_depth * 0.5
    rear_x = -body_depth * 0.5
    top_z = body_height * 0.5
    bottom_z = -body_height * 0.5

    wall_height = body_height - floor_thickness
    wall_center_z = bottom_z + floor_thickness + wall_height * 0.5
    ear_width = (rack_width - body_width) * 0.5 + ear_overlap

    drive_slot_center_y = -0.105
    drive_slot_center_z = 0.007
    drive_slot_width = 0.148
    drive_slot_height = 0.018

    slot_bottom_z = drive_slot_center_z - drive_slot_height * 0.5
    slot_top_z = drive_slot_center_z + drive_slot_height * 0.5
    lower_rail_height = slot_bottom_z - bottom_z
    upper_rail_height = top_z - slot_top_z
    left_panel_width = drive_slot_center_y - drive_slot_width * 0.5 + body_width * 0.5
    right_panel_width = body_width * 0.5 - (drive_slot_center_y + drive_slot_width * 0.5)

    chassis = model.part("chassis")
    chassis.visual(
        Box((body_depth, body_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_z + floor_thickness * 0.5)),
        material=brushed_aluminum,
        name="floor_pan",
    )
    chassis.visual(
        Box((body_depth, side_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, -body_width * 0.5 + side_thickness * 0.5, wall_center_z)
        ),
        material=brushed_aluminum,
        name="left_side_wall",
    )
    chassis.visual(
        Box((body_depth, side_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, body_width * 0.5 - side_thickness * 0.5, wall_center_z)
        ),
        material=brushed_aluminum,
        name="right_side_wall",
    )
    chassis.visual(
        Box((rear_thickness, body_width - 0.004, wall_height)),
        origin=Origin(xyz=(rear_x + rear_thickness * 0.5, 0.0, wall_center_z)),
        material=brushed_aluminum,
        name="rear_wall",
    )
    chassis.visual(
        Box((front_thickness, body_width, lower_rail_height)),
        origin=Origin(
            xyz=(
                front_x - front_thickness * 0.5,
                0.0,
                bottom_z + lower_rail_height * 0.5,
            )
        ),
        material=dark_anodized,
        name="lower_front_rail",
    )
    chassis.visual(
        Box((front_thickness, body_width, upper_rail_height)),
        origin=Origin(
            xyz=(
                front_x - front_thickness * 0.5,
                0.0,
                slot_top_z + upper_rail_height * 0.5,
            )
        ),
        material=dark_anodized,
        name="upper_front_rail",
    )
    chassis.visual(
        Box((front_thickness, left_panel_width, drive_slot_height)),
        origin=Origin(
            xyz=(
                front_x - front_thickness * 0.5,
                -body_width * 0.5 + left_panel_width * 0.5,
                drive_slot_center_z,
            )
        ),
        material=dark_anodized,
        name="left_drive_stile",
    )
    chassis.visual(
        Box((front_thickness, right_panel_width, drive_slot_height)),
        origin=Origin(
            xyz=(
                front_x - front_thickness * 0.5,
                drive_slot_center_y
                + drive_slot_width * 0.5
                + right_panel_width * 0.5,
                drive_slot_center_z,
            )
        ),
        material=dark_anodized,
        name="right_front_panel",
    )
    chassis.visual(
        Box((front_thickness, ear_width, body_height)),
        origin=Origin(
            xyz=(
                front_x - front_thickness * 0.5,
                -body_width * 0.5 - ear_width * 0.5 + ear_overlap,
                0.0,
            )
        ),
        material=brushed_aluminum,
        name="left_rack_ear",
    )
    chassis.visual(
        Box((front_thickness, ear_width, body_height)),
        origin=Origin(
            xyz=(
                front_x - front_thickness * 0.5,
                body_width * 0.5 + ear_width * 0.5 - ear_overlap,
                0.0,
            )
        ),
        material=brushed_aluminum,
        name="right_rack_ear",
    )
    chassis.visual(
        Box((0.030, side_thickness, body_height)),
        origin=Origin(
            xyz=(front_x - 0.015, -rack_width * 0.5 + side_thickness * 0.5, 0.0)
        ),
        material=brushed_aluminum,
        name="left_ear_return",
    )
    chassis.visual(
        Box((0.030, side_thickness, body_height)),
        origin=Origin(
            xyz=(front_x - 0.015, rack_width * 0.5 - side_thickness * 0.5, 0.0)
        ),
        material=brushed_aluminum,
        name="right_ear_return",
    )
    chassis.visual(
        Cylinder(radius=0.008, length=front_thickness),
        origin=Origin(
            xyz=(front_x - front_thickness * 0.5, 0.155, 0.004),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=button_black,
        name="power_button",
    )
    chassis.visual(
        Box((0.0015, 0.010, 0.003)),
        origin=Origin(xyz=(front_x + 0.0005, 0.135, -0.010)),
        material=indicator_blue,
        name="status_led",
    )

    drive_housing_length = 0.150
    drive_housing_width = 0.146
    drive_housing_height = 0.021
    drive_housing_x = 0.125
    drive_shell_thickness = 0.0015
    drive_housing_left_y = (
        drive_slot_center_y - drive_housing_width * 0.5 + drive_shell_thickness * 0.5
    )
    drive_housing_right_y = (
        drive_slot_center_y + drive_housing_width * 0.5 - drive_shell_thickness * 0.5
    )
    drive_mount_height = (
        drive_slot_center_z - drive_housing_height * 0.5 - (bottom_z + floor_thickness)
    )
    drive_mount_center_z = bottom_z + floor_thickness + drive_mount_height * 0.5

    chassis.visual(
        Box((drive_housing_length, drive_shell_thickness, drive_housing_height)),
        origin=Origin(
            xyz=(
                drive_housing_x,
                drive_housing_left_y,
                drive_slot_center_z,
            )
        ),
        material=dark_anodized,
        name="drive_housing_left",
    )
    chassis.visual(
        Box((drive_housing_length, drive_shell_thickness, drive_housing_height)),
        origin=Origin(
            xyz=(
                drive_housing_x,
                drive_housing_right_y,
                drive_slot_center_z,
            )
        ),
        material=dark_anodized,
        name="drive_housing_right",
    )
    chassis.visual(
        Box((drive_housing_length, drive_housing_width, drive_shell_thickness)),
        origin=Origin(
            xyz=(
                drive_housing_x,
                drive_slot_center_y,
                drive_slot_center_z + drive_housing_height * 0.5 - drive_shell_thickness * 0.5,
            )
        ),
        material=dark_anodized,
        name="drive_housing_top",
    )
    chassis.visual(
        Box((drive_shell_thickness, drive_housing_width, drive_housing_height)),
        origin=Origin(
            xyz=(
                drive_housing_x - drive_housing_length * 0.5 + drive_shell_thickness * 0.5,
                drive_slot_center_y,
                drive_slot_center_z,
            )
        ),
        material=dark_anodized,
        name="drive_housing_rear",
    )
    chassis.visual(
        Box((0.135, 0.004, 0.010)),
        origin=Origin(
            xyz=(0.132, drive_slot_center_y - 0.072, drive_slot_center_z - 0.001)
        ),
        material=hinge_steel,
        name="left_tray_guide",
    )
    chassis.visual(
        Box((0.135, 0.004, 0.010)),
        origin=Origin(
            xyz=(0.132, drive_slot_center_y + 0.072, drive_slot_center_z - 0.001)
        ),
        material=hinge_steel,
        name="right_tray_guide",
    )
    chassis.visual(
        Box((0.120, drive_shell_thickness, drive_mount_height)),
        origin=Origin(xyz=(0.125, drive_housing_left_y, drive_mount_center_z)),
        material=dark_anodized,
        name="left_drive_mount",
    )
    chassis.visual(
        Box((0.120, drive_shell_thickness, drive_mount_height)),
        origin=Origin(xyz=(0.125, drive_housing_right_y, drive_mount_center_z)),
        material=dark_anodized,
        name="right_drive_mount",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((body_depth, rack_width, body_height)),
        mass=6.2,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid_length = 0.419
    lid_width = 0.450
    lid_top_thickness = 0.0016
    lid_skirt_thickness = 0.0018
    lid_skirt_height = 0.016
    lid_hinge_x = rear_x - 0.0035
    lid_hinge_z = top_z + 0.001

    lid.visual(
        Cylinder(radius=0.0022, length=body_width - 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.014, body_width - 0.010, 0.003)),
        origin=Origin(xyz=(0.007, 0.0, -0.0010)),
        material=brushed_aluminum,
        name="rear_leaf",
    )
    lid.visual(
        Box((lid_length, lid_width, lid_top_thickness)),
        origin=Origin(xyz=(lid_length * 0.5, 0.0, 0.0008)),
        material=brushed_aluminum,
        name="top_panel",
    )
    lid.visual(
        Box((lid_length - 0.016, lid_skirt_thickness, lid_skirt_height)),
        origin=Origin(
            xyz=(
                lid_length * 0.5,
                -lid_width * 0.5 + lid_skirt_thickness * 0.5,
                -0.0072,
            )
        ),
        material=brushed_aluminum,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_length - 0.016, lid_skirt_thickness, lid_skirt_height)),
        origin=Origin(
            xyz=(
                lid_length * 0.5,
                lid_width * 0.5 - lid_skirt_thickness * 0.5,
                -0.0072,
            )
        ),
        material=brushed_aluminum,
        name="right_skirt",
    )
    lid.visual(
        Box((0.012, lid_width - 0.010, 0.013)),
        origin=Origin(xyz=(0.4175, 0.0, -0.0055)),
        material=brushed_aluminum,
        name="front_lip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.018)),
        mass=1.0,
        origin=Origin(xyz=(lid_length * 0.5, 0.0, -0.002)),
    )

    optical_tray = model.part("optical_tray")
    optical_tray.visual(
        Box((0.146, 0.130, 0.0015)),
        origin=Origin(xyz=(-0.067, 0.0, -0.00625)),
        material=tray_black,
        name="tray_floor",
    )
    optical_tray.visual(
        Box((0.132, 0.004, 0.012)),
        origin=Origin(xyz=(-0.074, -0.067, -0.0005)),
        material=tray_black,
        name="left_tray_rail",
    )
    optical_tray.visual(
        Box((0.132, 0.004, 0.012)),
        origin=Origin(xyz=(-0.074, 0.067, -0.0005)),
        material=tray_black,
        name="right_tray_rail",
    )
    optical_tray.visual(
        Box((0.004, 0.128, 0.010)),
        origin=Origin(xyz=(-0.140, 0.0, -0.0015)),
        material=tray_black,
        name="rear_stop",
    )
    optical_tray.visual(
        Cylinder(radius=0.012, length=0.0035),
        origin=Origin(xyz=(-0.060, 0.0, -0.00475)),
        material=hinge_steel,
        name="spindle_hub",
    )
    optical_tray.visual(
        Box((0.008, 0.150, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=tray_black,
        name="tray_bezel",
    )
    optical_tray.visual(
        Box((0.002, 0.010, 0.004)),
        origin=Origin(xyz=(0.015, 0.054, -0.003)),
        material=button_black,
        name="eject_button",
    )
    optical_tray.visual(
        Box((0.0015, 0.004, 0.002)),
        origin=Origin(xyz=(0.01475, -0.055, -0.004)),
        material=indicator_blue,
        name="activity_led",
    )
    optical_tray.inertial = Inertial.from_geometry(
        Box((0.150, 0.150, 0.020)),
        mass=0.25,
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lid,
        origin=Origin(xyz=(lid_hinge_x, 0.0, lid_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "optical_tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=optical_tray,
        origin=Origin(xyz=(0.204, drive_slot_center_y, drive_slot_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.25,
            lower=0.0,
            upper=0.165,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    primary_part_names = {part.name for part in object_model.parts}
    primary_joint_names = {joint.name for joint in object_model.articulations}

    ctx.check(
        "primary_parts_present",
        {"chassis", "lid", "optical_tray"} <= primary_part_names,
        details=f"Found parts: {sorted(primary_part_names)}",
    )
    ctx.check(
        "primary_articulations_present",
        {"rear_lid_hinge", "optical_tray_slide"} <= primary_joint_names,
        details=f"Found articulations: {sorted(primary_joint_names)}",
    )

    chassis = object_model.get_part("chassis")
    lid = object_model.get_part("lid")
    optical_tray = object_model.get_part("optical_tray")
    lid_hinge = object_model.get_articulation("rear_lid_hinge")
    tray_slide = object_model.get_articulation("optical_tray_slide")

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

    lid_axis = lid_hinge.axis
    tray_axis = tray_slide.axis
    lid_limits = lid_hinge.motion_limits
    tray_limits = tray_slide.motion_limits

    ctx.check(
        "lid_hinge_runs_across_rear_width",
        math.isclose(lid_axis[0], 0.0, abs_tol=1e-6)
        and math.isclose(abs(lid_axis[1]), 1.0, abs_tol=1e-6)
        and math.isclose(lid_axis[2], 0.0, abs_tol=1e-6)
        and lid_limits is not None
        and lid_limits.upper is not None
        and lid_limits.upper >= 1.2,
        details=f"axis={lid_axis}, limits={lid_limits}",
    )
    ctx.check(
        "tray_slide_runs_front_to_back",
        math.isclose(abs(tray_axis[0]), 1.0, abs_tol=1e-6)
        and math.isclose(tray_axis[1], 0.0, abs_tol=1e-6)
        and math.isclose(tray_axis[2], 0.0, abs_tol=1e-6)
        and tray_limits is not None
        and tray_limits.upper is not None
        and tray_limits.upper >= 0.15,
        details=f"axis={tray_axis}, limits={tray_limits}",
    )

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_overlap(
            lid,
            chassis,
            axes="xy",
            min_overlap=0.35,
            name="lid_covers_chassis_planform",
        )
        ctx.expect_gap(
            lid,
            chassis,
            axis="z",
            positive_elem="top_panel",
            negative_elem="upper_front_rail",
            max_gap=0.0035,
            max_penetration=0.0,
            name="lid_rests_just_above_front_rail",
        )
        ctx.expect_gap(
            optical_tray,
            chassis,
            axis="x",
            positive_elem="tray_bezel",
            negative_elem="upper_front_rail",
            max_gap=0.0015,
            max_penetration=0.0,
            name="optical_bezel_seats_against_front_plane",
        )
        ctx.expect_within(
            optical_tray,
            chassis,
            axes="yz",
            margin=0.002,
            name="tray_stays_within_body_width_and_height",
        )

    def _elem_aabb(part, elem: str):
        return ctx.part_element_world_aabb(part, elem=elem)

    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_front = _elem_aabb(lid, "front_lip")
    with ctx.pose({lid_hinge: 1.10}):
        opened_lid_front = _elem_aabb(lid, "front_lip")
    if closed_lid_front is None or opened_lid_front is None:
        ctx.fail("lid_front_lip_aabb_available", "Could not resolve lid front lip AABB.")
    else:
        closed_front_z = closed_lid_front[1][2]
        opened_front_z = opened_lid_front[1][2]
        ctx.check(
            "lid_front_edge_lifts_when_opened",
            opened_front_z > closed_front_z + 0.12,
            details=f"closed_z={closed_front_z:.4f}, opened_z={opened_front_z:.4f}",
        )

    with ctx.pose({tray_slide: 0.0}):
        closed_bezel = _elem_aabb(optical_tray, "tray_bezel")
    with ctx.pose({tray_slide: 0.160}):
        open_bezel = _elem_aabb(optical_tray, "tray_bezel")
        ctx.expect_within(
            optical_tray,
            chassis,
            axes="yz",
            margin=0.002,
            name="tray_remains_aligned_when_extended",
        )
    if closed_bezel is None or open_bezel is None:
        ctx.fail("tray_bezel_aabb_available", "Could not resolve tray bezel AABB.")
    else:
        closed_front_x = closed_bezel[1][0]
        open_front_x = open_bezel[1][0]
        ctx.check(
            "tray_bezel_moves_forward_when_opened",
            open_front_x > closed_front_x + 0.12,
            details=f"closed_x={closed_front_x:.4f}, opened_x={open_front_x:.4f}",
        )

    left_ear = ctx.part_element_world_aabb(chassis, elem="left_rack_ear")
    right_ear = ctx.part_element_world_aabb(chassis, elem="right_rack_ear")
    left_wall = ctx.part_element_world_aabb(chassis, elem="left_side_wall")
    right_wall = ctx.part_element_world_aabb(chassis, elem="right_side_wall")
    if None in (left_ear, right_ear, left_wall, right_wall):
        ctx.fail("rack_ear_aabbs_available", "Could not resolve rack ear or side wall AABBs.")
    else:
        ctx.check(
            "rack_ears_extend_beyond_chassis_sides",
            left_ear[0][1] < left_wall[0][1] - 0.015
            and right_ear[1][1] > right_wall[1][1] + 0.015,
            details=(
                f"left ear min y={left_ear[0][1]:.4f}, left wall min y={left_wall[0][1]:.4f}; "
                f"right ear max y={right_ear[1][1]:.4f}, right wall max y={right_wall[1][1]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
