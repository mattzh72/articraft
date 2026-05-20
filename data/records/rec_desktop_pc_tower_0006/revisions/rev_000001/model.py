from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import pathlib
import tempfile

_REAL_GETCWD = os.getcwd
_REAL_PATH_ABSOLUTE = pathlib.Path.absolute
_REAL_PATH_RESOLVE = pathlib.Path.resolve


def _stable_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        fallback = tempfile.gettempdir()
        try:
            os.chdir(fallback)
        except FileNotFoundError:
            os.chdir("/")
        return _REAL_GETCWD()


os.getcwd = _stable_getcwd
os.chdir(_stable_getcwd())


def _stable_path_absolute(self):
    try:
        return _REAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        if self.is_absolute():
            return self
        return pathlib.Path(_stable_getcwd()) / self


def _stable_path_resolve(self, strict=False):
    try:
        return _REAL_PATH_RESOLVE(self, strict=strict)
    except FileNotFoundError:
        return _stable_path_absolute(self)


for _path_cls_name in ("Path", "PosixPath", "WindowsPath"):
    _path_cls = getattr(pathlib, _path_cls_name, None)
    if _path_cls is not None:
        _path_cls.absolute = _stable_path_absolute
        _path_cls.resolve = _stable_path_resolve

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

OUTER_W = 0.190
OUTER_D = 0.580
OUTER_H = 0.465
WALL = 0.006

FRAME_W = 0.012
FRAME_D = 0.020

DOOR_T = 0.012
DOOR_W = 0.192
DOOR_H = OUTER_H - 0.012
DOOR_FRAME_W = 0.018
DOOR_OPEN_MAX = 1.85

HINGE_RADIUS = 0.006
HINGE_X = -OUTER_W / 2 - 0.001
HINGE_Y = OUTER_D / 2 + DOOR_T / 2
HINGE_DOOR_SEGMENTS = (
    (-0.150, 0.075, "lower_hinge"),
    (0.000, 0.085, "middle_hinge"),
    (0.150, 0.075, "upper_hinge"),
)
DRAWER_W = 0.160
DRAWER_D = 0.185
DRAWER_H = 0.068
DRAWER_WALL = 0.003
DRAWER_BEZEL_W = 0.158
DRAWER_BEZEL_H = 0.076
DRAWER_BEZEL_T = 0.010
DRAWER_TRAVEL = 0.150
DRAWER_CLOSED_Y = 0.180

LOWER_DRAWER_Z = 0.056
MID_SHELF_Z = 0.103
UPPER_DRAWER_Z = 0.148
MID_SHELF_T = 0.006

TRACK_T = 0.006
RUNNER_T = 0.004
RUNNER_H = 0.018
TRACK_H = 0.020
TRACK_LEN = 0.220
TRACK_CENTER_Y = 0.150
RUNNER_LEN = 0.180
RUNNER_CENTER_Y = -0.030


def _add_drive_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    bezel_color: str,
    led_color: str,
):
    drawer = model.part(name)

    floor_t = 0.003
    wall_h = DRAWER_H - 0.012

    drawer.visual(
        Box((DRAWER_W - 2 * DRAWER_WALL, DRAWER_D, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, -DRAWER_H / 2 + floor_t / 2)),
        material="tray_metal",
        name="tray_floor",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_D, wall_h)),
        origin=Origin(
            xyz=(-DRAWER_W / 2 + DRAWER_WALL / 2, 0.0, -DRAWER_H / 2 + wall_h / 2)
        ),
        material="tray_metal",
        name="left_wall",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_D, wall_h)),
        origin=Origin(
            xyz=(DRAWER_W / 2 - DRAWER_WALL / 2, 0.0, -DRAWER_H / 2 + wall_h / 2)
        ),
        material="tray_metal",
        name="right_wall",
    )
    drawer.visual(
        Box((DRAWER_W - 2 * DRAWER_WALL, DRAWER_WALL, wall_h)),
        origin=Origin(
            xyz=(0.0, -DRAWER_D / 2 + DRAWER_WALL / 2, -DRAWER_H / 2 + wall_h / 2)
        ),
        material="tray_metal",
        name="rear_wall",
    )
    drawer.visual(
        Box((DRAWER_W - 0.028, DRAWER_D * 0.68, 0.028)),
        origin=Origin(xyz=(0.0, -0.012, -DRAWER_H / 2 + floor_t + 0.014)),
        material="drive_pack",
        name="drive_pack",
    )
    drawer.visual(
        Box((DRAWER_BEZEL_W, DRAWER_BEZEL_T, DRAWER_BEZEL_H)),
        origin=Origin(xyz=(0.0, DRAWER_D / 2 + DRAWER_BEZEL_T / 2, 0.0)),
        material=bezel_color,
        name="bezel",
    )
    drawer.visual(
        Box((0.008, 0.008, 0.020)),
        origin=Origin(xyz=(-0.032, DRAWER_D / 2 + 0.010, 0.0)),
        material="handle_metal",
        name="handle_left_post",
    )
    drawer.visual(
        Box((0.008, 0.008, 0.020)),
        origin=Origin(xyz=(0.032, DRAWER_D / 2 + 0.010, 0.0)),
        material="handle_metal",
        name="handle_right_post",
    )
    drawer.visual(
        Cylinder(radius=0.005, length=0.078),
        origin=Origin(
            xyz=(0.0, DRAWER_D / 2 + 0.014, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="handle_metal",
        name="handle",
    )
    drawer.visual(
        Box((0.012, 0.003, 0.010)),
        origin=Origin(xyz=(0.060, DRAWER_D / 2 + 0.007, 0.0)),
        material=led_color,
        name="status_led",
    )
    drawer.visual(
        Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(-0.081, RUNNER_CENTER_Y, 0.0)),
        material="rail_dark",
        name="left_runner",
    )
    drawer.visual(
        Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
        origin=Origin(xyz=(0.081, RUNNER_CENTER_Y, 0.0)),
        material="rail_dark",
        name="right_runner",
    )

    return drawer


def build_object_model() -> ArticulatedObject:
    os.chdir(_stable_getcwd())
    model = ArticulatedObject(name="tower_server_chassis")

    model.material("chassis_shell", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("chassis_inner", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("rail_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("door_shell", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("door_mesh", rgba=(0.24, 0.27, 0.30, 0.70))
    model.material("tray_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("drive_pack", rgba=(0.31, 0.32, 0.34, 1.0))
    model.material("drawer_bezel_upper", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("drawer_bezel_lower", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("handle_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("status_blue", rgba=(0.18, 0.47, 0.87, 1.0))
    model.material("status_amber", rgba=(0.94, 0.64, 0.16, 1.0))
    model.material("foot_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((WALL, OUTER_D, OUTER_H)),
        origin=Origin(xyz=(-OUTER_W / 2 + WALL / 2, 0.0, OUTER_H / 2)),
        material="chassis_shell",
        name="left_side",
    )
    chassis.visual(
        Box((WALL, OUTER_D, OUTER_H)),
        origin=Origin(xyz=(OUTER_W / 2 - WALL / 2, 0.0, OUTER_H / 2)),
        material="chassis_shell",
        name="right_side",
    )
    chassis.visual(
        Box((OUTER_W - 2 * WALL, OUTER_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2)),
        material="chassis_shell",
        name="bottom_pan",
    )
    chassis.visual(
        Box((OUTER_W - 2 * WALL, OUTER_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_H - WALL / 2)),
        material="chassis_shell",
        name="top_cover",
    )
    chassis.visual(
        Box((OUTER_W - 2 * WALL, WALL, OUTER_H - 2 * WALL)),
        origin=Origin(xyz=(0.0, -OUTER_D / 2 + WALL / 2, OUTER_H / 2)),
        material="chassis_shell",
        name="rear_panel",
    )
    chassis.visual(
        Box((FRAME_W, FRAME_D, OUTER_H - 2 * WALL)),
        origin=Origin(
            xyz=(-OUTER_W / 2 + FRAME_W / 2, OUTER_D / 2 - FRAME_D / 2, OUTER_H / 2)
        ),
        material="chassis_shell",
        name="front_left_stile",
    )
    chassis.visual(
        Box((FRAME_W, FRAME_D, OUTER_H - 2 * WALL)),
        origin=Origin(
            xyz=(OUTER_W / 2 - FRAME_W / 2, OUTER_D / 2 - FRAME_D / 2, OUTER_H / 2)
        ),
        material="chassis_shell",
        name="front_right_stile",
    )
    chassis.visual(
        Box((OUTER_W - 2 * FRAME_W, FRAME_D, FRAME_W)),
        origin=Origin(
            xyz=(0.0, OUTER_D / 2 - FRAME_D / 2, OUTER_H - FRAME_W / 2)
        ),
        material="chassis_shell",
        name="front_top_rail",
    )
    chassis.visual(
        Box((OUTER_W - 2 * FRAME_W, FRAME_D, FRAME_W)),
        origin=Origin(xyz=(0.0, OUTER_D / 2 - FRAME_D / 2, FRAME_W / 2)),
        material="chassis_shell",
        name="front_bottom_rail",
    )
    chassis.visual(
        Box((OUTER_W - 2 * FRAME_W, FRAME_D, 0.010)),
        origin=Origin(xyz=(0.0, OUTER_D / 2 - FRAME_D / 2, 0.102)),
        material="chassis_shell",
        name="bay_divider",
    )
    chassis.visual(
        Box((OUTER_W - 2 * WALL, 0.380, MID_SHELF_T)),
        origin=Origin(xyz=(0.0, -0.030, MID_SHELF_Z)),
        material="chassis_inner",
        name="mid_shelf",
    )
    chassis.visual(
        Box((OUTER_W - 2 * WALL - 0.016, 0.175, 0.120)),
        origin=Origin(xyz=(0.0, -0.202, 0.265)),
        material="chassis_inner",
        name="psu_block",
    )
    chassis.visual(
        Cylinder(radius=0.034, length=WALL),
        origin=Origin(
            xyz=(-0.044, -OUTER_D / 2 + WALL / 2, 0.338),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="rail_dark",
        name="rear_fan_left",
    )
    chassis.visual(
        Cylinder(radius=0.034, length=WALL),
        origin=Origin(
            xyz=(0.044, -OUTER_D / 2 + WALL / 2, 0.338),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material="rail_dark",
        name="rear_fan_right",
    )
    chassis.visual(
        Box((0.028, 0.135, 0.012)),
        origin=Origin(xyz=(-0.054, 0.0, -0.006)),
        material="foot_rubber",
        name="left_foot",
    )
    chassis.visual(
        Box((0.028, 0.135, 0.012)),
        origin=Origin(xyz=(0.054, 0.0, -0.006)),
        material="foot_rubber",
        name="right_foot",
    )

    for x_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x_track = x_sign * 0.086
        chassis.visual(
            Box((TRACK_T, TRACK_LEN, TRACK_H)),
            origin=Origin(xyz=(x_track, TRACK_CENTER_Y, LOWER_DRAWER_Z)),
            material="rail_dark",
            name=f"lower_{side_name}_track",
        )
        chassis.visual(
            Box((TRACK_T, TRACK_LEN, TRACK_H)),
            origin=Origin(xyz=(x_track, TRACK_CENTER_Y, UPPER_DRAWER_Z)),
            material="rail_dark",
            name=f"upper_{side_name}_track",
        )

    door = model.part("front_door")
    door.visual(
        Box((DOOR_FRAME_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_FRAME_W / 2 + 0.002, 0.0, 0.0)),
        material="door_shell",
        name="left_stile",
    )
    door.visual(
        Box((DOOR_FRAME_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W - DOOR_FRAME_W / 2, 0.0, 0.0)),
        material="door_shell",
        name="right_stile",
    )
    door.visual(
        Box((DOOR_W - 2 * DOOR_FRAME_W, DOOR_T, DOOR_FRAME_W)),
        origin=Origin(xyz=(DOOR_W / 2, 0.0, DOOR_H / 2 - DOOR_FRAME_W / 2)),
        material="door_shell",
        name="top_rail",
    )
    door.visual(
        Box((DOOR_W - 2 * DOOR_FRAME_W, DOOR_T, DOOR_FRAME_W)),
        origin=Origin(xyz=(DOOR_W / 2, 0.0, -DOOR_H / 2 + DOOR_FRAME_W / 2)),
        material="door_shell",
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_W - 0.028, 0.004, DOOR_H - 0.092)),
        origin=Origin(xyz=(DOOR_W / 2, -0.002, 0.0)),
        material="door_mesh",
        name="mesh_panel",
    )
    for idx, x_pos in enumerate((0.041, 0.067, 0.093, 0.119, 0.145), start=1):
        door.visual(
            Box((0.006, 0.003, DOOR_H - 2 * DOOR_FRAME_W)),
            origin=Origin(xyz=(x_pos, 0.0015, 0.0)),
            material="door_shell",
            name=f"mesh_slat_{idx}",
        )
    door.visual(
        Box((0.012, 0.014, 0.140)),
        origin=Origin(xyz=(DOOR_W - 0.024, 0.010, 0.0)),
        material="handle_metal",
        name="handle",
    )
    for z_center, length, name in HINGE_DOOR_SEGMENTS:
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material="door_shell",
            name=name,
        )

    lower_drawer = _add_drive_drawer(
        model,
        name="lower_drawer",
        bezel_color="drawer_bezel_lower",
        led_color="status_amber",
    )
    upper_drawer = _add_drive_drawer(
        model,
        name="upper_drawer",
        bezel_color="drawer_bezel_upper",
        led_color="status_blue",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, OUTER_H / 2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=DOOR_OPEN_MAX,
        ),
    )
    model.articulation(
        "lower_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=lower_drawer,
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, LOWER_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "upper_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=upper_drawer,
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, UPPER_DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    os.chdir(_stable_getcwd())
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    door = object_model.get_part("front_door")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_drawer = object_model.get_part("upper_drawer")

    door_hinge = object_model.get_articulation("door_hinge")
    lower_slide = object_model.get_articulation("lower_slide")
    upper_slide = object_model.get_articulation("upper_slide")

    rear_panel = chassis.get_visual("rear_panel")
    bottom_pan = chassis.get_visual("bottom_pan")
    mid_shelf = chassis.get_visual("mid_shelf")
    front_top_rail = chassis.get_visual("front_top_rail")
    front_left_stile = chassis.get_visual("front_left_stile")
    lower_left_track = chassis.get_visual("lower_left_track")
    upper_right_track = chassis.get_visual("upper_right_track")
    door_mesh = door.get_visual("mesh_panel")
    door_handle = door.get_visual("handle")
    door_left_stile = door.get_visual("left_stile")
    lower_floor = lower_drawer.get_visual("tray_floor")
    upper_floor = upper_drawer.get_visual("tray_floor")
    lower_bezel = lower_drawer.get_visual("bezel")
    upper_bezel = upper_drawer.get_visual("bezel")
    lower_left_runner = lower_drawer.get_visual("left_runner")
    upper_right_runner = upper_drawer.get_visual("right_runner")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.expect_contact(
        door,
        chassis,
        elem_a=door_left_stile,
        elem_b=front_left_stile,
        name="door_resting_against_front_frame",
    )
    ctx.expect_contact(
        lower_drawer,
        chassis,
        elem_a=lower_left_runner,
        elem_b=lower_left_track,
        name="lower_drawer_runner_contact_at_rest",
    )
    ctx.expect_contact(
        upper_drawer,
        chassis,
        elem_a=upper_right_runner,
        elem_b=upper_right_track,
        name="upper_drawer_runner_contact_at_rest",
    )

    ctx.expect_gap(
        door,
        upper_drawer,
        axis="y",
        min_gap=0.009,
        max_gap=0.030,
        positive_elem=door_mesh,
        negative_elem=upper_bezel,
        name="door_to_upper_bezel_closed_gap",
    )
    ctx.expect_gap(
        door,
        lower_drawer,
        axis="y",
        min_gap=0.009,
        max_gap=0.030,
        positive_elem=door_mesh,
        negative_elem=lower_bezel,
        name="door_to_lower_bezel_closed_gap",
    )
    ctx.expect_gap(
        lower_drawer,
        chassis,
        axis="z",
        min_gap=0.012,
        max_gap=0.030,
        positive_elem=lower_floor,
        negative_elem=bottom_pan,
        name="lower_drawer_bottom_clearance",
    )
    ctx.expect_gap(
        upper_drawer,
        chassis,
        axis="z",
        min_gap=0.006,
        max_gap=0.020,
        positive_elem=upper_floor,
        negative_elem=mid_shelf,
        name="upper_drawer_shelf_clearance",
    )
    ctx.expect_gap(
        lower_drawer,
        chassis,
        axis="y",
        min_gap=0.020,
        positive_elem=lower_floor,
        negative_elem=rear_panel,
        name="lower_drawer_rear_clearance",
    )
    ctx.expect_gap(
        upper_drawer,
        chassis,
        axis="y",
        min_gap=0.020,
        positive_elem=upper_floor,
        negative_elem=rear_panel,
        name="upper_drawer_rear_clearance",
    )
    ctx.expect_within(
        lower_drawer,
        chassis,
        axes="xz",
        inner_elem=lower_bezel,
        margin=0.0,
        name="lower_bezel_within_chassis_face",
    )
    ctx.expect_within(
        upper_drawer,
        chassis,
        axes="xz",
        inner_elem=upper_bezel,
        margin=0.0,
        name="upper_bezel_within_chassis_face",
    )
    ctx.expect_within(
        door,
        chassis,
        axes="xz",
        inner_elem=door_mesh,
        margin=0.0,
        name="door_mesh_within_chassis_front_footprint",
    )

    lower_rest = ctx.part_world_position(lower_drawer)
    upper_rest = ctx.part_world_position(upper_drawer)
    door_handle_rest = ctx.part_element_world_aabb(door, elem=door_handle)

    lower_limits = lower_slide.motion_limits
    upper_limits = upper_slide.motion_limits
    door_limits = door_hinge.motion_limits

    if lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose({lower_slide: lower_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lower_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lower_slide_upper_no_floating")
            ctx.expect_contact(
                lower_drawer,
                chassis,
                elem_a=lower_left_runner,
                elem_b=lower_left_track,
                name="lower_drawer_runner_contact_open",
            )
            ctx.expect_gap(
                lower_drawer,
                chassis,
                axis="y",
                min_gap=0.120,
                positive_elem=lower_bezel,
                negative_elem=front_top_rail,
                name="lower_drawer_projects_forward_when_open",
            )
            lower_open = ctx.part_world_position(lower_drawer)
            ctx.check(
                "lower_drawer_travel_distance",
                lower_rest is not None
                and lower_open is not None
                and lower_open[1] >= lower_rest[1] + DRAWER_TRAVEL - 1e-4,
                "Lower drawer did not translate forward through its full rail travel.",
            )

    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({upper_slide: upper_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="upper_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="upper_slide_upper_no_floating")
            ctx.expect_contact(
                upper_drawer,
                chassis,
                elem_a=upper_right_runner,
                elem_b=upper_right_track,
                name="upper_drawer_runner_contact_open",
            )
            ctx.expect_gap(
                upper_drawer,
                chassis,
                axis="y",
                min_gap=0.120,
                positive_elem=upper_bezel,
                negative_elem=front_top_rail,
                name="upper_drawer_projects_forward_when_open",
            )
            upper_open = ctx.part_world_position(upper_drawer)
            ctx.check(
                "upper_drawer_travel_distance",
                upper_rest is not None
                and upper_open is not None
                and upper_open[1] >= upper_rest[1] + DRAWER_TRAVEL - 1e-4,
                "Upper drawer did not translate forward through its full rail travel.",
            )

    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")
            ctx.expect_gap(
                door,
                chassis,
                axis="y",
                min_gap=0.120,
                positive_elem=door_handle,
                negative_elem=front_top_rail,
                name="door_handle_swings_forward",
            )
            door_handle_open = ctx.part_element_world_aabb(door, elem=door_handle)
            ctx.check(
                "door_handle_motion_arc",
                door_handle_rest is not None
                and door_handle_open is not None
                and door_handle_open[0][1] > door_handle_rest[0][1] + 0.120,
                "Door handle did not move forward enough to read as an opening swing.",
            )

    if (
        door_limits is not None
        and door_limits.upper is not None
        and lower_limits is not None
        and lower_limits.upper is not None
        and upper_limits is not None
        and upper_limits.upper is not None
    ):
        with ctx.pose(
            {
                door_hinge: door_limits.upper,
                lower_slide: lower_limits.upper,
                upper_slide: upper_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="all_articulations_open_no_overlap")
            ctx.fail_if_isolated_parts(name="all_articulations_open_no_floating")
            ctx.expect_contact(
                lower_drawer,
                chassis,
                elem_a=lower_left_runner,
                elem_b=lower_left_track,
                name="lower_runner_contact_all_open",
            )
            ctx.expect_contact(
                upper_drawer,
                chassis,
                elem_a=upper_right_runner,
                elem_b=upper_right_track,
                name="upper_runner_contact_all_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
