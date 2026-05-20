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
    model = ArticulatedObject(name="full_tower_pc_case")

    chassis_black = model.material("chassis_black", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.55, 0.57, 0.60, 1.0))
    bay_black = model.material("bay_black", rgba=(0.14, 0.14, 0.15, 1.0))
    handle_gray = model.material("handle_gray", rgba=(0.35, 0.37, 0.40, 1.0))
    drive_gray = model.material("drive_gray", rgba=(0.42, 0.44, 0.46, 1.0))

    outer_width = 0.24
    outer_depth = 0.52
    outer_height = 0.56
    frame_t = 0.012
    panel_t = 0.008
    rear_edge_y = 0.252
    front_edge_y = -0.252
    left_panel_x = -(outer_width / 2.0 + panel_t / 2.0)
    right_panel_x = outer_width / 2.0 + panel_t / 2.0

    chassis = model.part("chassis")

    # Main lower frame and corner structure.
    chassis.visual(
        Box((0.216, 0.48, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chassis_black,
        name="bottom_floor",
    )
    chassis.visual(
        Box((frame_t, 0.50, 0.020)),
        origin=Origin(xyz=(-0.114, 0.0, 0.010)),
        material=chassis_black,
        name="left_lower_rail",
    )
    chassis.visual(
        Box((frame_t, 0.50, 0.020)),
        origin=Origin(xyz=(0.114, 0.0, 0.010)),
        material=chassis_black,
        name="right_lower_rail",
    )
    chassis.visual(
        Box((frame_t, 0.020, 0.540)),
        origin=Origin(xyz=(-0.114, -0.250, 0.290)),
        material=chassis_black,
        name="front_left_post",
    )
    chassis.visual(
        Box((frame_t, 0.020, 0.540)),
        origin=Origin(xyz=(0.114, -0.250, 0.290)),
        material=chassis_black,
        name="front_right_post",
    )
    chassis.visual(
        Box((frame_t, 0.020, 0.540)),
        origin=Origin(xyz=(-0.114, 0.250, 0.290)),
        material=chassis_black,
        name="rear_left_post",
    )
    chassis.visual(
        Box((frame_t, 0.020, 0.540)),
        origin=Origin(xyz=(0.114, 0.250, 0.290)),
        material=chassis_black,
        name="rear_right_post",
    )
    chassis.visual(
        Box((frame_t, 0.50, 0.006)),
        origin=Origin(xyz=(-0.114, 0.0, 0.557)),
        material=chassis_black,
        name="left_top_outer_rail",
    )
    chassis.visual(
        Box((frame_t, 0.50, 0.006)),
        origin=Origin(xyz=(0.114, 0.0, 0.557)),
        material=chassis_black,
        name="right_top_outer_rail",
    )
    chassis.visual(
        Box((0.216, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.254, 0.557)),
        material=chassis_black,
        name="top_front_rail",
    )
    chassis.visual(
        Box((0.036, 0.012, 0.006)),
        origin=Origin(xyz=(-0.092, 0.254, 0.557)),
        material=chassis_black,
        name="top_rear_left_rail",
    )
    chassis.visual(
        Box((0.110, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.254, 0.557)),
        material=chassis_black,
        name="top_rear_center_rail",
    )
    chassis.visual(
        Box((0.036, 0.012, 0.006)),
        origin=Origin(xyz=(0.092, 0.254, 0.557)),
        material=chassis_black,
        name="top_rear_right_rail",
    )

    # Internal motherboard tray and rear structure.
    chassis.visual(
        Box((0.004, 0.44, 0.500)),
        origin=Origin(xyz=(0.045, 0.010, 0.270)),
        material=steel_gray,
        name="motherboard_tray",
    )
    chassis.visual(
        Box((0.228, 0.012, 0.180)),
        origin=Origin(xyz=(0.0, 0.254, 0.380)),
        material=chassis_black,
        name="rear_io_panel",
    )

    # Front bezel with a tall hot-swap bay opening and lower intake area.
    chassis.visual(
        Box((0.188, 0.020, 0.220)),
        origin=Origin(xyz=(0.0, -0.250, 0.110)),
        material=chassis_black,
        name="front_lower_intake",
    )
    chassis.visual(
        Box((0.188, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.250, 0.230)),
        material=chassis_black,
        name="front_bay_bottom_bar",
    )
    chassis.visual(
        Box((0.188, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.250, 0.500)),
        material=chassis_black,
        name="front_bay_top_bar",
    )
    chassis.visual(
        Box((0.014, 0.020, 0.240)),
        origin=Origin(xyz=(-0.101, -0.250, 0.370)),
        material=chassis_black,
        name="front_bay_left_bar",
    )
    chassis.visual(
        Box((0.014, 0.020, 0.240)),
        origin=Origin(xyz=(0.101, -0.250, 0.370)),
        material=chassis_black,
        name="front_bay_right_bar",
    )

    # Three-bay hot-swap cage.
    chassis.visual(
        Box((0.008, 0.240, 0.200)),
        origin=Origin(xyz=(-0.090, -0.120, 0.370)),
        material=steel_gray,
        name="cage_left_wall",
    )
    chassis.visual(
        Box((0.008, 0.240, 0.200)),
        origin=Origin(xyz=(0.090, -0.120, 0.370)),
        material=steel_gray,
        name="cage_right_wall",
    )
    chassis.visual(
        Box((0.188, 0.240, 0.006)),
        origin=Origin(xyz=(0.0, -0.120, 0.267)),
        material=steel_gray,
        name="cage_bottom_shelf",
    )
    chassis.visual(
        Box((0.188, 0.240, 0.006)),
        origin=Origin(xyz=(0.0, -0.120, 0.329)),
        material=steel_gray,
        name="cage_mid_shelf_1",
    )
    chassis.visual(
        Box((0.188, 0.240, 0.006)),
        origin=Origin(xyz=(0.0, -0.120, 0.391)),
        material=steel_gray,
        name="cage_mid_shelf_2",
    )
    chassis.visual(
        Box((0.188, 0.240, 0.006)),
        origin=Origin(xyz=(0.0, -0.120, 0.473)),
        material=steel_gray,
        name="cage_top_shelf",
    )
    chassis.visual(
        Box((0.188, 0.008, 0.200)),
        origin=Origin(xyz=(0.0, 0.004, 0.370)),
        material=steel_gray,
        name="cage_back_strap",
    )

    bay_runner_zs = (0.296, 0.358, 0.420)
    for bay_index, runner_z in enumerate(bay_runner_zs, start=1):
        chassis.visual(
            Box((0.008, 0.240, 0.004)),
            origin=Origin(xyz=(-0.074, -0.120, runner_z - 0.002)),
            material=steel_gray,
            name=f"bay_{bay_index}_left_rail",
        )
        chassis.visual(
            Box((0.008, 0.240, 0.004)),
            origin=Origin(xyz=(0.074, -0.120, runner_z - 0.002)),
            material=steel_gray,
            name=f"bay_{bay_index}_right_rail",
        )

    # Top exhaust opening surround.
    chassis.visual(
        Box((0.028, 0.200, 0.006)),
        origin=Origin(xyz=(-0.100, 0.154, 0.557)),
        material=chassis_black,
        name="top_grill_left_cover",
    )
    chassis.visual(
        Box((0.028, 0.200, 0.006)),
        origin=Origin(xyz=(0.100, 0.154, 0.557)),
        material=chassis_black,
        name="top_grill_right_cover",
    )
    chassis.visual(
        Box((0.012, 0.200, 0.006)),
        origin=Origin(xyz=(-0.086, 0.154, 0.557)),
        material=chassis_black,
        name="top_grill_left_support",
    )
    chassis.visual(
        Box((0.012, 0.200, 0.006)),
        origin=Origin(xyz=(0.086, 0.154, 0.557)),
        material=chassis_black,
        name="top_grill_right_support",
    )
    chassis.visual(
        Box((0.184, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.048, 0.557)),
        material=chassis_black,
        name="top_grill_front_strip",
    )

    # Rear hinge brackets for the side panels and top grill.
    chassis.visual(
        Box((0.008, 0.012, 0.160)),
        origin=Origin(xyz=(-0.114, 0.254, 0.100)),
        material=steel_gray,
        name="left_panel_lower_bracket",
    )
    chassis.visual(
        Box((0.008, 0.012, 0.380)),
        origin=Origin(xyz=(-0.114, 0.254, 0.370)),
        material=steel_gray,
        name="left_panel_upper_bracket",
    )
    chassis.visual(
        Box((0.008, 0.012, 0.160)),
        origin=Origin(xyz=(0.114, 0.254, 0.100)),
        material=steel_gray,
        name="right_panel_lower_bracket",
    )
    chassis.visual(
        Box((0.008, 0.012, 0.380)),
        origin=Origin(xyz=(0.114, 0.254, 0.370)),
        material=steel_gray,
        name="right_panel_upper_bracket",
    )
    chassis.visual(
        Box((0.030, 0.020, 0.010)),
        origin=Origin(xyz=(-0.060, 0.256, 0.555)),
        material=steel_gray,
        name="top_grill_left_bracket",
    )
    chassis.visual(
        Box((0.030, 0.020, 0.010)),
        origin=Origin(xyz=(0.060, 0.256, 0.555)),
        material=steel_gray,
        name="top_grill_right_bracket",
    )
    chassis.visual(
        Cylinder(radius=0.0035, length=0.024),
        origin=Origin(
            xyz=(-0.055, 0.266, 0.562),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_gray,
        name="top_grill_left_pin",
    )
    chassis.visual(
        Cylinder(radius=0.0035, length=0.024),
        origin=Origin(
            xyz=(0.055, 0.266, 0.562),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_gray,
        name="top_grill_right_pin",
    )

    chassis.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, outer_height)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, outer_height / 2.0)),
    )

    def build_side_panel(name: str, outward_sign: float) -> None:
        panel = model.part(name)
        panel.visual(
            Box((panel_t, 0.504, 0.500)),
            origin=Origin(xyz=(0.0, -0.252, 0.250)),
            material=panel_black,
            name="panel_sheet",
        )
        panel.visual(
            Box((0.010, 0.060, 0.120)),
            origin=Origin(xyz=(outward_sign * 0.008, -0.180, 0.270)),
            material=handle_gray,
            name="pull_handle",
        )
        panel.visual(
            Cylinder(radius=0.006, length=0.460),
            origin=Origin(xyz=(0.0, 0.0, 0.250)),
            material=steel_gray,
            name="hinge_barrel",
        )
        panel.inertial = Inertial.from_geometry(
            Box((panel_t, 0.504, 0.500)),
            mass=1.8,
            origin=Origin(xyz=(0.0, -0.252, 0.250)),
        )

    build_side_panel("left_side_panel", outward_sign=-1.0)
    build_side_panel("right_side_panel", outward_sign=1.0)

    left_panel = model.get_part("left_side_panel")
    right_panel = model.get_part("right_side_panel")

    model.articulation(
        "left_side_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=left_panel,
        origin=Origin(xyz=(left_panel_x, rear_edge_y, 0.030)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "right_side_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=right_panel,
        origin=Origin(xyz=(right_panel_x, rear_edge_y, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    top_grill = model.part("top_exhaust_grill")
    top_grill.visual(
        Box((0.160, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=panel_black,
        name="grill_rear_bar",
    )
    top_grill.visual(
        Box((0.160, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.214, 0.0)),
        material=panel_black,
        name="grill_front_bar",
    )
    top_grill.visual(
        Box((0.008, 0.202, 0.004)),
        origin=Origin(xyz=(-0.076, -0.117, 0.0)),
        material=panel_black,
        name="grill_left_bar",
    )
    top_grill.visual(
        Box((0.008, 0.202, 0.004)),
        origin=Origin(xyz=(0.076, -0.117, 0.0)),
        material=panel_black,
        name="grill_right_bar",
    )
    for slat_index, x_pos in enumerate((-0.048, -0.024, 0.0, 0.024, 0.048), start=1):
        top_grill.visual(
            Box((0.004, 0.186, 0.003)),
            origin=Origin(xyz=(x_pos, -0.117, 0.0)),
            material=steel_gray,
            name=f"slat_{slat_index}",
        )
    top_grill.visual(
        Box((0.024, 0.016, 0.004)),
        origin=Origin(xyz=(-0.055, -0.008, 0.0)),
        material=steel_gray,
        name="left_hinge_bridge",
    )
    top_grill.visual(
        Box((0.024, 0.016, 0.004)),
        origin=Origin(xyz=(0.055, -0.008, 0.0)),
        material=steel_gray,
        name="right_hinge_bridge",
    )
    top_grill.visual(
        Cylinder(radius=0.0037, length=0.024),
        origin=Origin(
            xyz=(-0.055, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_gray,
        name="left_hinge_barrel",
    )
    top_grill.visual(
        Cylinder(radius=0.0037, length=0.024),
        origin=Origin(
            xyz=(0.055, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_gray,
        name="right_hinge_barrel",
    )
    top_grill.inertial = Inertial.from_geometry(
        Box((0.160, 0.220, 0.020)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.107, 0.0)),
    )

    model.articulation(
        "top_grill_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_grill,
        origin=Origin(xyz=(0.0, 0.266, 0.562)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    def build_drive_sled(name: str) -> None:
        sled = model.part(name)
        sled.visual(
            Box((0.008, 0.180, 0.004)),
            origin=Origin(xyz=(-0.074, 0.090, 0.002)),
            material=steel_gray,
            name="left_runner",
        )
        sled.visual(
            Box((0.008, 0.180, 0.004)),
            origin=Origin(xyz=(0.074, 0.090, 0.002)),
            material=steel_gray,
            name="right_runner",
        )
        sled.visual(
            Box((0.148, 0.180, 0.002)),
            origin=Origin(xyz=(0.0, 0.090, 0.005)),
            material=bay_black,
            name="tray_floor",
        )
        sled.visual(
            Box((0.002, 0.180, 0.018)),
            origin=Origin(xyz=(-0.077, 0.090, 0.015)),
            material=bay_black,
            name="left_wall",
        )
        sled.visual(
            Box((0.002, 0.180, 0.018)),
            origin=Origin(xyz=(0.077, 0.090, 0.015)),
            material=bay_black,
            name="right_wall",
        )
        sled.visual(
            Box((0.154, 0.002, 0.018)),
            origin=Origin(xyz=(0.0, 0.179, 0.015)),
            material=bay_black,
            name="rear_wall",
        )
        sled.visual(
            Box((0.156, 0.008, 0.032)),
            origin=Origin(xyz=(0.0, -0.004, 0.016)),
            material=handle_gray,
            name="front_face",
        )
        sled.visual(
            Box((0.060, 0.014, 0.012)),
            origin=Origin(xyz=(0.0, -0.011, 0.018)),
            material=steel_gray,
            name="handle_grip",
        )
        sled.visual(
            Box((0.102, 0.146, 0.014)),
            origin=Origin(xyz=(0.0, 0.088, 0.013)),
            material=drive_gray,
            name="drive_block",
        )
        sled.inertial = Inertial.from_geometry(
            Box((0.156, 0.190, 0.032)),
            mass=0.9,
            origin=Origin(xyz=(0.0, 0.085, 0.016)),
        )

    for sled_name in ("drive_sled_1", "drive_sled_2", "drive_sled_3"):
        build_drive_sled(sled_name)

    for sled_index, runner_z in enumerate(bay_runner_zs, start=1):
        model.articulation(
            f"drive_sled_{sled_index}_slide",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=model.get_part(f"drive_sled_{sled_index}"),
            origin=Origin(xyz=(0.0, -0.240, runner_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.25,
                lower=0.0,
                upper=0.100,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    chassis = object_model.get_part("chassis")
    left_panel = object_model.get_part("left_side_panel")
    right_panel = object_model.get_part("right_side_panel")
    top_grill = object_model.get_part("top_exhaust_grill")
    sleds = [object_model.get_part(f"drive_sled_{index}") for index in (1, 2, 3)]

    left_hinge = object_model.get_articulation("left_side_hinge")
    right_hinge = object_model.get_articulation("right_side_hinge")
    top_hinge = object_model.get_articulation("top_grill_hinge")
    sled_joints = [object_model.get_articulation(f"drive_sled_{index}_slide") for index in (1, 2, 3)]

    left_sheet = left_panel.get_visual("panel_sheet")
    right_sheet = right_panel.get_visual("panel_sheet")
    left_front_post = chassis.get_visual("front_left_post")
    right_front_post = chassis.get_visual("front_right_post")
    grill_left_pin = chassis.get_visual("top_grill_left_pin")
    grill_right_pin = chassis.get_visual("top_grill_right_pin")
    grill_front_bar = top_grill.get_visual("grill_front_bar")
    grill_left_barrel = top_grill.get_visual("left_hinge_barrel")
    grill_right_barrel = top_grill.get_visual("right_hinge_barrel")
    grill_front_strip = chassis.get_visual("top_grill_front_strip")

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
    ctx.allow_overlap(
        chassis,
        top_grill,
        elem_a=grill_left_pin,
        elem_b=grill_left_barrel,
        reason="Top exhaust panel rotates on a left steel hinge pin captured inside its hinge barrel.",
    )
    ctx.allow_overlap(
        chassis,
        top_grill,
        elem_a=grill_right_pin,
        elem_b=grill_right_barrel,
        reason="Top exhaust panel rotates on a right steel hinge pin captured inside its hinge barrel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.fail_if_isolated_parts(max_pose_samples=16, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.check(
        "left_side_hinge_axis",
        tuple(left_hinge.axis) == (0.0, 0.0, -1.0),
        f"Expected left hinge axis (0, 0, -1), got {left_hinge.axis}",
    )
    ctx.check(
        "right_side_hinge_axis",
        tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        f"Expected right hinge axis (0, 0, 1), got {right_hinge.axis}",
    )
    ctx.check(
        "top_grill_hinge_axis",
        tuple(top_hinge.axis) == (-1.0, 0.0, 0.0),
        f"Expected top hinge axis (-1, 0, 0), got {top_hinge.axis}",
    )
    for index, sled_joint in enumerate(sled_joints, start=1):
        ctx.check(
            f"drive_sled_{index}_axis",
            tuple(sled_joint.axis) == (0.0, -1.0, 0.0),
            f"Expected sled axis (0, -1, 0), got {sled_joint.axis}",
        )

    chassis_aabb = ctx.part_world_aabb(chassis)
    if chassis_aabb is None:
        ctx.fail("chassis_aabb_available", "Chassis world AABB is unavailable.")
    else:
        (cmin_x, cmin_y, cmin_z), (cmax_x, cmax_y, cmax_z) = chassis_aabb
        ctx.check(
            "chassis_full_tower_width",
            0.20 <= (cmax_x - cmin_x) <= 0.26,
            f"Chassis width was {cmax_x - cmin_x:.3f} m",
        )
        ctx.check(
            "chassis_full_tower_depth",
            0.48 <= (cmax_y - cmin_y) <= 0.54,
            f"Chassis depth was {cmax_y - cmin_y:.3f} m",
        )
        ctx.check(
            "chassis_full_tower_height",
            0.54 <= (cmax_z - cmin_z) <= 0.57,
            f"Chassis height was {cmax_z - cmin_z:.3f} m",
        )

    ctx.expect_gap(
        chassis,
        left_panel,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=left_front_post,
        negative_elem=left_sheet,
        name="left_panel_closed_seat",
    )
    ctx.expect_gap(
        right_panel,
        chassis,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=right_sheet,
        negative_elem=right_front_post,
        name="right_panel_closed_seat",
    )
    ctx.expect_gap(
        top_grill,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=grill_front_bar,
        negative_elem=grill_front_strip,
        name="top_grill_closed_seat",
    )

    ctx.expect_contact(left_panel, chassis, name="left_panel_contact")
    ctx.expect_contact(right_panel, chassis, name="right_panel_contact")
    ctx.expect_contact(top_grill, chassis, name="top_grill_contact")
    for index, sled in enumerate(sleds, start=1):
        ctx.expect_contact(sled, chassis, name=f"drive_sled_{index}_rest_contact")

    left_rest = ctx.part_world_aabb(left_panel)
    right_rest = ctx.part_world_aabb(right_panel)
    grill_rest = ctx.part_world_aabb(top_grill)
    sled_rest_positions = [ctx.part_world_position(sled) for sled in sleds]

    if left_rest is not None:
        with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
            left_open = ctx.part_world_aabb(left_panel)
            if left_open is None:
                ctx.fail("left_panel_open_aabb", "Left panel open-pose AABB is unavailable.")
            else:
                ctx.check(
                    "left_panel_swings_outward",
                    left_open[0][0] < left_rest[0][0] - 0.12,
                    f"Left panel min x only moved from {left_rest[0][0]:.3f} to {left_open[0][0]:.3f}",
                )
                ctx.expect_contact(left_panel, chassis, name="left_panel_open_contact")

    if right_rest is not None:
        with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
            right_open = ctx.part_world_aabb(right_panel)
            if right_open is None:
                ctx.fail("right_panel_open_aabb", "Right panel open-pose AABB is unavailable.")
            else:
                ctx.check(
                    "right_panel_swings_outward",
                    right_open[1][0] > right_rest[1][0] + 0.12,
                    f"Right panel max x only moved from {right_rest[1][0]:.3f} to {right_open[1][0]:.3f}",
                )
                ctx.expect_contact(right_panel, chassis, name="right_panel_open_contact")

    if grill_rest is not None:
        with ctx.pose({top_hinge: top_hinge.motion_limits.upper}):
            grill_open = ctx.part_world_aabb(top_grill)
            if grill_open is None:
                ctx.fail("top_grill_open_aabb", "Top grill open-pose AABB is unavailable.")
            else:
                ctx.check(
                    "top_grill_lifts_upward",
                    grill_open[1][2] > grill_rest[1][2] + 0.12,
                    f"Top grill max z only moved from {grill_rest[1][2]:.3f} to {grill_open[1][2]:.3f}",
                )
                ctx.expect_contact(top_grill, chassis, name="top_grill_open_contact")

    for index, (sled, joint, rest_pos) in enumerate(zip(sleds, sled_joints, sled_rest_positions), start=1):
        if rest_pos is None:
            ctx.fail(f"drive_sled_{index}_rest_position", "Drive sled rest position is unavailable.")
            continue
        with ctx.pose({joint: joint.motion_limits.upper}):
            open_pos = ctx.part_world_position(sled)
            if open_pos is None:
                ctx.fail(f"drive_sled_{index}_open_position", "Drive sled open position is unavailable.")
            else:
                ctx.check(
                    f"drive_sled_{index}_slides_forward",
                    open_pos[1] < rest_pos[1] - 0.09,
                    f"Sled y only moved from {rest_pos[1]:.3f} to {open_pos[1]:.3f}",
                )
                ctx.expect_contact(sled, chassis, name=f"drive_sled_{index}_extended_contact")

    rest_zs = [ctx.part_world_position(sled) for sled in sleds]
    if all(pos is not None for pos in rest_zs):
        z_values = [pos[2] for pos in rest_zs if pos is not None]
        ctx.check(
            "drive_sleds_stack_vertically",
            z_values[0] < z_values[1] < z_values[2],
            f"Expected ascending sled z positions, got {z_values}",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for joint in [left_hinge, right_hinge, top_hinge, *sled_joints]:
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
