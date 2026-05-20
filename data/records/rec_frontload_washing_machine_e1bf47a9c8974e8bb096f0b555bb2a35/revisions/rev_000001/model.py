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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.17, 0.19, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.11, 0.11, 0.12, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.53, 0.66, 0.76, 0.42))
    drawer_gray = model.material("drawer_gray", rgba=(0.82, 0.84, 0.86, 1.0))

    width = 0.60
    depth = 0.66
    overall_height = 0.85
    foot_height = 0.03
    body_height = overall_height - foot_height
    panel_t = 0.015
    front_t = 0.020
    top_t = 0.018
    bottom_t = 0.020
    back_t = 0.012
    front_y = depth / 2.0 - front_t / 2.0
    side_z_center = foot_height + body_height / 2.0

    door_center_z = 0.435
    door_outer_radius = 0.215
    door_center_offset = 0.225
    door_center_y = 0.363
    gasket_center_y = 0.308
    drum_center_y = 0.095

    drawer_center_x = -0.18
    drawer_center_y = 0.172
    drawer_center_z = 0.758

    axis_to_y_rpy = (math.pi / 2.0, 0.0, 0.0)

    def lathed_shell_mesh(outer_profile, inner_profile, name: str):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=72,
            ),
            name,
        )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((panel_t, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + panel_t / 2.0, 0.0, side_z_center)),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((panel_t, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - panel_t / 2.0, 0.0, side_z_center)),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((width - 2.0 * panel_t, depth - back_t, bottom_t)),
        origin=Origin(
            xyz=(0.0, back_t / 2.0, foot_height + bottom_t / 2.0),
        ),
        material=cabinet_white,
        name="bottom_pan",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, overall_height - top_t / 2.0)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * panel_t, back_t, body_height - bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                -depth / 2.0 + back_t / 2.0,
                foot_height + bottom_t + (body_height - bottom_t) / 2.0,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((width, front_t, 0.120)),
        origin=Origin(xyz=(0.0, front_y, 0.060)),
        material=cabinet_white,
        name="lower_rail",
    )
    cabinet.visual(
        Box((0.080, front_t, 0.540)),
        origin=Origin(xyz=(-0.260, front_y, 0.390)),
        material=cabinet_white,
        name="left_stile",
    )
    cabinet.visual(
        Box((0.080, front_t, 0.540)),
        origin=Origin(xyz=(0.260, front_y, 0.390)),
        material=cabinet_white,
        name="right_stile",
    )
    cabinet.visual(
        Box((width, front_t, 0.080)),
        origin=Origin(xyz=(0.0, front_y, 0.670)),
        material=cabinet_white,
        name="upper_rail",
    )
    cabinet.visual(
        Box((0.310, front_t, 0.140)),
        origin=Origin(xyz=(0.145, front_y, 0.750)),
        material=cabinet_white,
        name="control_console",
    )
    cabinet.visual(
        Box((0.240, front_t, 0.020)),
        origin=Origin(xyz=(drawer_center_x, front_y, 0.680)),
        material=cabinet_white,
        name="drawer_slot_sill",
    )
    cabinet.visual(
        Box((0.240, front_t, 0.030)),
        origin=Origin(xyz=(drawer_center_x, front_y, 0.820)),
        material=cabinet_white,
        name="drawer_slot_header",
    )
    cabinet.visual(
        Box((0.020, front_t, 0.120)),
        origin=Origin(xyz=(-0.060, front_y, 0.750)),
        material=cabinet_white,
        name="drawer_slot_right_jamb",
    )
    cabinet.visual(
        Box((0.015, 0.380, 0.078)),
        origin=Origin(xyz=(-0.2775, 0.120, drawer_center_z)),
        material=drawer_gray,
        name="drawer_left_rail",
    )
    cabinet.visual(
        Box((0.020, 0.380, 0.078)),
        origin=Origin(xyz=(-0.080, 0.120, drawer_center_z)),
        material=drawer_gray,
        name="drawer_right_rail",
    )
    cabinet.visual(
        Box((0.036, 0.040, 0.080)),
        origin=Origin(xyz=(-0.253, 0.340, door_center_z + 0.120)),
        material=cabinet_white,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.036, 0.040, 0.080)),
        origin=Origin(xyz=(-0.253, 0.340, door_center_z - 0.120)),
        material=cabinet_white,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.029, 0.030, 0.240)),
        origin=Origin(xyz=(-0.2055, 0.308, door_center_z)),
        material=cabinet_white,
        name="left_gasket_bridge",
    )
    cabinet.visual(
        Box((0.029, 0.030, 0.240)),
        origin=Origin(xyz=(0.2055, 0.308, door_center_z)),
        material=cabinet_white,
        name="right_gasket_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(-door_center_offset - 0.003, door_center_y, door_center_z + 0.115)),
        material=dark_trim,
        name="upper_hinge_pin_support",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(-door_center_offset - 0.003, door_center_y, door_center_z - 0.115)),
        material=dark_trim,
        name="lower_hinge_pin_support",
    )
    cabinet.visual(
        Cylinder(radius=0.055, length=0.140),
        origin=Origin(xyz=(0.0, -0.254, door_center_z), rpy=axis_to_y_rpy),
        material=dark_trim,
        name="rear_bearing_housing",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.160),
        origin=Origin(xyz=(0.0, -0.185, door_center_z), rpy=axis_to_y_rpy),
        material=dark_trim,
        name="rear_support_axle",
    )
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.230, 0.235),
        ("front_right_foot", 0.230, 0.235),
        ("rear_left_foot", -0.230, -0.235),
        ("rear_right_foot", 0.230, -0.235),
    ):
        cabinet.visual(
            Cylinder(radius=0.020, length=foot_height),
            origin=Origin(xyz=(foot_x, foot_y, foot_height / 2.0)),
            material=dark_trim,
            name=foot_name,
        )
    cabinet.inertial = Inertial.from_geometry(
        Box((width, depth, overall_height)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, overall_height / 2.0)),
    )

    gasket = model.part("gasket")
    gasket.visual(
        lathed_shell_mesh(
            [(0.185, -0.030), (0.191, 0.000), (0.183, 0.030)],
            [(0.148, -0.024), (0.157, 0.000), (0.150, 0.024)],
            "washer_gasket_shell",
        ),
        origin=Origin(rpy=axis_to_y_rpy),
        material=gasket_black,
        name="gasket_shell",
    )
    gasket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.185, length=0.060),
        mass=1.2,
    )

    drum = model.part("drum")
    drum.visual(
        lathed_shell_mesh(
            [(0.225, -0.150), (0.225, 0.150)],
            [(0.214, -0.144), (0.214, 0.144)],
            "washer_drum_shell",
        ),
        origin=Origin(rpy=axis_to_y_rpy),
        material=drum_steel,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.216, length=0.018),
        origin=Origin(xyz=(0.0, -0.141, 0.0), rpy=axis_to_y_rpy),
        material=drum_steel,
        name="drum_back_plate",
    )
    drum.visual(
        Cylinder(radius=0.038, length=0.052),
        origin=Origin(xyz=(0.0, -0.176, 0.0), rpy=axis_to_y_rpy),
        material=dark_trim,
        name="drum_hub",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.300),
        mass=8.0,
    )

    door = model.part("door")
    door.visual(
        lathed_shell_mesh(
            [(door_outer_radius, -0.025), (0.222, 0.000), (door_outer_radius, 0.025)],
            [(0.150, -0.019), (0.142, 0.000), (0.150, 0.019)],
            "washer_door_ring",
        ),
        origin=Origin(xyz=(door_center_offset, 0.0, 0.0), rpy=axis_to_y_rpy),
        material=dark_trim,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.145, length=0.016),
        origin=Origin(xyz=(door_center_offset, -0.008, 0.0), rpy=axis_to_y_rpy),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.160),
        material=dark_trim,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.024, 0.024, 0.140)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_bridge",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.350, 0.040, 0.0)),
        material=handle_steel,
        name="pull_handle_bar",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.350, 0.022, 0.055), rpy=axis_to_y_rpy),
        material=handle_steel,
        name="pull_handle_upper_standoff",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.350, 0.022, -0.055), rpy=axis_to_y_rpy),
        material=handle_steel,
        name="pull_handle_lower_standoff",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.430, 0.090, 0.440)),
        mass=4.0,
        origin=Origin(xyz=(0.225, 0.020, 0.0)),
    )

    drawer = model.part("drawer")
    tray_width = 0.180
    tray_depth = 0.280
    tray_height = 0.078
    tray_wall = 0.004
    front_width = 0.200
    front_height = 0.072
    front_thickness = 0.018
    drawer.visual(
        Box((tray_width - 2.0 * tray_wall, tray_depth, tray_wall)),
        origin=Origin(xyz=(0.0, 0.0, -tray_height / 2.0 + tray_wall / 2.0)),
        material=drawer_gray,
        name="drawer_floor",
    )
    drawer.visual(
        Box((tray_wall, tray_depth, tray_height)),
        origin=Origin(xyz=(-tray_width / 2.0 + tray_wall / 2.0, 0.0, 0.0)),
        material=drawer_gray,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((tray_wall, tray_depth, tray_height)),
        origin=Origin(xyz=(tray_width / 2.0 - tray_wall / 2.0, 0.0, 0.0)),
        material=drawer_gray,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((tray_width - 2.0 * tray_wall, tray_wall, tray_height)),
        origin=Origin(xyz=(0.0, -tray_depth / 2.0 + tray_wall / 2.0, 0.0)),
        material=drawer_gray,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((front_width, front_thickness, front_height)),
        origin=Origin(xyz=(0.0, tray_depth / 2.0 + front_thickness / 2.0, 0.0)),
        material=drawer_gray,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.008, length=0.135),
        origin=Origin(
            xyz=(0.0, tray_depth / 2.0 + front_thickness + 0.008, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_steel,
        name="drawer_pull",
    )
    drawer.visual(
        Box((tray_wall, 0.170, 0.055)),
        origin=Origin(xyz=(0.026, -0.015, -0.008)),
        material=drawer_gray,
        name="drawer_divider",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((front_width, tray_depth + front_thickness, front_height)),
        mass=1.3,
    )

    model.articulation(
        "cabinet_to_gasket",
        ArticulationType.FIXED,
        parent=cabinet,
        child=gasket,
        origin=Origin(xyz=(0.0, gasket_center_y, door_center_z)),
    )
    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, drum_center_y, door_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=22.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_center_offset, door_center_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(drawer_center_x, drawer_center_y, drawer_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.35,
            lower=0.0,
            upper=0.200,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    gasket = object_model.get_part("gasket")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")

    drum_joint = object_model.get_articulation("cabinet_to_drum")
    door_joint = object_model.get_articulation("cabinet_to_door")
    drawer_joint = object_model.get_articulation("cabinet_to_drawer")

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
        "door_hinge_axis_vertical",
        tuple(round(v, 3) for v in door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "door_hinge_limit_realistic",
        door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and 1.8 <= door_joint.motion_limits.upper <= 2.1,
        details=f"limits={door_joint.motion_limits}",
    )
    ctx.check(
        "drawer_slide_axis_front_back",
        tuple(round(v, 3) for v in drawer_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drawer_joint.axis}",
    )
    ctx.check(
        "drawer_travel_realistic",
        drawer_joint.motion_limits is not None
        and drawer_joint.motion_limits.lower == 0.0
        and drawer_joint.motion_limits.upper is not None
        and 0.16 <= drawer_joint.motion_limits.upper <= 0.24,
        details=f"limits={drawer_joint.motion_limits}",
    )
    ctx.check(
        "drum_spin_axis_front_back",
        tuple(round(v, 3) for v in drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drum_joint.axis}",
    )

    ctx.expect_origin_distance(drum, gasket, axes="xz", max_dist=0.001, name="drum_centered_in_opening")
    ctx.expect_gap(
        door,
        gasket,
        axis="y",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="door_ring",
        negative_elem="gasket_shell",
        name="door_ring_seats_against_gasket",
    )
    ctx.expect_overlap(
        door,
        gasket,
        axes="xz",
        min_overlap=0.300,
        elem_a="door_ring",
        elem_b="gasket_shell",
        name="door_aligned_to_gasket",
    )
    ctx.expect_overlap(
        drum,
        gasket,
        axes="xz",
        min_overlap=0.300,
        elem_a="drum_shell",
        elem_b="gasket_shell",
        name="drum_aligned_to_gasket",
    )
    ctx.expect_gap(
        gasket,
        drum,
        axis="y",
        min_gap=0.010,
        max_gap=0.050,
        positive_elem="gasket_shell",
        negative_elem="drum_shell",
        name="gasket_clear_of_rotating_drum",
    )
    ctx.expect_within(drawer, cabinet, axes="x", margin=0.040, name="drawer_stays_within_cabinet_width")
    ctx.expect_within(drum, cabinet, axes="xz", margin=0.030, name="drum_within_cabinet_envelope")

    drawer_front_closed = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    assert drawer_front_closed is not None
    drawer_slot_sill = ctx.part_element_world_aabb(cabinet, elem="drawer_slot_sill")
    assert drawer_slot_sill is not None
    ctx.check(
        "drawer_front_closed_flush",
        abs(drawer_front_closed[1][1] - drawer_slot_sill[1][1]) <= 0.003,
        details=f"drawer_front_max_y={drawer_front_closed[1][1]:.4f}, slot_front_max_y={drawer_slot_sill[1][1]:.4f}",
    )

    door_limits = door_joint.motion_limits
    assert door_limits is not None and door_limits.upper is not None
    closed_door_ring = ctx.part_element_world_aabb(door, elem="door_ring")
    assert closed_door_ring is not None
    with ctx.pose({door_joint: door_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")
    with ctx.pose({door_joint: door_limits.upper}):
        open_door_ring = ctx.part_element_world_aabb(door, elem="door_ring")
        assert open_door_ring is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.check(
            "door_swings_forward",
            open_door_ring[1][1] > closed_door_ring[1][1] + 0.120,
            details=f"closed_front={closed_door_ring[1][1]:.4f}, open_front={open_door_ring[1][1]:.4f}",
        )

    drawer_limits = drawer_joint.motion_limits
    assert drawer_limits is not None and drawer_limits.upper is not None
    with ctx.pose({drawer_joint: drawer_limits.upper}):
        open_drawer_front = ctx.part_element_world_aabb(drawer, elem="drawer_front")
        assert open_drawer_front is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="drawer_open_no_overlap")
        ctx.fail_if_isolated_parts(name="drawer_open_no_floating")
        ctx.check(
            "drawer_extends_forward",
            open_drawer_front[1][1] - drawer_front_closed[1][1] >= 0.180,
            details=f"closed_front={drawer_front_closed[1][1]:.4f}, open_front={open_drawer_front[1][1]:.4f}",
        )
        ctx.expect_within(drawer, cabinet, axes="x", margin=0.040, name="drawer_open_keeps_side_guidance")

    with ctx.pose({drum_joint: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drum_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="drum_spin_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
