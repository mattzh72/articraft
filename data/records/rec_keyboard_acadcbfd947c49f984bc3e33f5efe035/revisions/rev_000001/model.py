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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trackpoint_keyboard")

    body_dark = model.material("body_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    body_trim = model.material("body_trim", rgba=(0.22, 0.23, 0.25, 1.0))
    key_dark = model.material("key_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    key_top = model.material("key_top", rgba=(0.28, 0.29, 0.31, 1.0))
    stem_dark = model.material("stem_dark", rgba=(0.11, 0.11, 0.12, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.10, 0.14, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_mark = model.material("wheel_mark", rgba=(0.75, 0.76, 0.78, 1.0))

    body_w = 0.236
    body_d = 0.124
    floor_t = 0.004
    wall_t = 0.005
    front_h = 0.018
    side_h = 0.021
    rear_h = 0.026
    key_joint_z = 0.0138
    key_travel = 0.0035

    chassis = model.part("chassis")
    chassis.visual(
        Box((body_w, body_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=body_dark,
        name="tray_floor",
    )
    chassis.visual(
        Box((body_w, wall_t, front_h)),
        origin=Origin(xyz=(0.0, -(body_d * 0.5) + (wall_t * 0.5), front_h * 0.5)),
        material=body_dark,
        name="front_wall",
    )
    chassis.visual(
        Box((body_w, wall_t, rear_h)),
        origin=Origin(xyz=(0.0, (body_d * 0.5) - (wall_t * 0.5), rear_h * 0.5)),
        material=body_dark,
        name="rear_wall",
    )
    chassis.visual(
        Box((wall_t, body_d, side_h)),
        origin=Origin(xyz=(-(body_w * 0.5) + (wall_t * 0.5), 0.0, side_h * 0.5)),
        material=body_dark,
        name="left_wall",
    )
    chassis.visual(
        Box((wall_t, body_d, side_h)),
        origin=Origin(xyz=((body_w * 0.5) - (wall_t * 0.5), 0.0, side_h * 0.5)),
        material=body_dark,
        name="right_wall",
    )
    chassis.visual(
        Box((body_w - 0.012, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, -(body_d * 0.5) + 0.010, 0.0155)),
        material=body_trim,
        name="front_inner_lip",
    )
    chassis.visual(
        Box((body_w - 0.012, 0.009, 0.003)),
        origin=Origin(xyz=(0.0, (body_d * 0.5) - 0.011, 0.0195)),
        material=body_trim,
        name="rear_inner_lip",
    )
    chassis.visual(
        Box((0.009, body_d - 0.014, 0.003)),
        origin=Origin(xyz=(-(body_w * 0.5) + 0.010, 0.0, 0.0170)),
        material=body_trim,
        name="left_inner_lip",
    )
    chassis.visual(
        Box((0.009, body_d - 0.014, 0.003)),
        origin=Origin(xyz=((body_w * 0.5) - 0.010, 0.0, 0.0170)),
        material=body_trim,
        name="right_inner_lip",
    )

    chassis.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.0, 0.013, 0.009)),
        material=stem_dark,
        name="trackpoint_pedestal",
    )
    chassis.visual(
        Cylinder(radius=0.0062, length=0.0032),
        origin=Origin(xyz=(0.0, 0.013, 0.0156)),
        material=body_trim,
        name="trackpoint_boot",
    )

    wheel_center = (0.091, 0.0465, 0.0240)
    chassis.visual(
        Box((0.030, 0.024, 0.011)),
        origin=Origin(xyz=(wheel_center[0], wheel_center[1], 0.0097)),
        material=body_trim,
        name="wheel_pod_base",
    )
    chassis.visual(
        Box((0.003, 0.022, 0.014)),
        origin=Origin(xyz=(wheel_center[0] - 0.011, wheel_center[1], 0.0220)),
        material=body_dark,
        name="wheel_pod_left_cheek",
    )
    chassis.visual(
        Box((0.003, 0.022, 0.014)),
        origin=Origin(xyz=(wheel_center[0] + 0.011, wheel_center[1], 0.0220)),
        material=body_dark,
        name="wheel_pod_right_cheek",
    )
    chassis.visual(
        Box((0.030, 0.006, 0.014)),
        origin=Origin(xyz=(wheel_center[0], wheel_center[1] + 0.010, 0.0220)),
        material=body_dark,
        name="wheel_pod_rear_bridge",
    )

    x_positions = (-0.090, -0.071, -0.052, -0.033, -0.014, 0.014, 0.033, 0.052, 0.071, 0.090)
    y_positions = (0.033, 0.013, -0.007, -0.027)
    row_profiles = (
        {"top_z": 0.0068, "top_y": 0.0013, "roll": 0.10, "top_h": 0.0030},
        {"top_z": 0.0066, "top_y": 0.0005, "roll": 0.04, "top_h": 0.0031},
        {"top_z": 0.0064, "top_y": -0.0004, "roll": -0.04, "top_h": 0.0032},
        {"top_z": 0.0062, "top_y": -0.0013, "roll": -0.10, "top_h": 0.0030},
    )

    guide_open = 0.0046
    guide_t = 0.0018
    guide_outer = guide_open + (2.0 * guide_t)
    guide_h = 0.0080
    guide_z = 0.0080

    for y_pos in y_positions:
        for x_pos in x_positions:
            chassis.visual(
                Box((guide_t, guide_outer, guide_h)),
                origin=Origin(xyz=(x_pos - ((guide_open * 0.5) + (guide_t * 0.5)), y_pos, guide_z)),
                material=stem_dark,
                name=None,
            )
            chassis.visual(
                Box((guide_t, guide_outer, guide_h)),
                origin=Origin(xyz=(x_pos + ((guide_open * 0.5) + (guide_t * 0.5)), y_pos, guide_z)),
                material=stem_dark,
                name=None,
            )
            chassis.visual(
                Box((guide_outer, guide_t, guide_h)),
                origin=Origin(xyz=(x_pos, y_pos - ((guide_open * 0.5) + (guide_t * 0.5)), guide_z)),
                material=stem_dark,
                name=None,
            )
            chassis.visual(
                Box((guide_outer, guide_t, guide_h)),
                origin=Origin(xyz=(x_pos, y_pos + ((guide_open * 0.5) + (guide_t * 0.5)), guide_z)),
                material=stem_dark,
                name=None,
            )

    lower_w = 0.0164
    lower_d = 0.0164
    lower_h = 0.0054
    stem_w = 0.0046
    stem_h = 0.0052

    for row_index, y_pos in enumerate(y_positions):
        row_profile = row_profiles[row_index]
        for col_index, x_pos in enumerate(x_positions):
            key = model.part(f"key_r{row_index}_c{col_index}")
            key.visual(
                Box((stem_w, stem_w, stem_h)),
                origin=Origin(xyz=(0.0, 0.0, -0.0014)),
                material=stem_dark,
                name="stem",
            )
            key.visual(
                Box((lower_w, lower_d, lower_h)),
                origin=Origin(xyz=(0.0, 0.0, 0.0027)),
                material=key_dark,
                name="key_body",
            )
            key.visual(
                Box((0.0136, 0.0132, row_profile["top_h"])),
                origin=Origin(
                    xyz=(0.0, row_profile["top_y"], row_profile["top_z"]),
                    rpy=(row_profile["roll"], 0.0, 0.0),
                ),
                material=key_top,
                name="key_top",
            )
            model.articulation(
                f"chassis_to_key_r{row_index}_c{col_index}",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=key,
                origin=Origin(xyz=(x_pos, y_pos, key_joint_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.2,
                    velocity=0.08,
                    lower=0.0,
                    upper=key_travel,
                ),
            )

    trackpoint = model.part("trackpoint_nub")
    trackpoint.visual(
        Cylinder(radius=0.0038, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
        material=accent_red,
        name="nub_shaft",
    )
    trackpoint.visual(
        Sphere(radius=0.0037),
        origin=Origin(xyz=(0.0, 0.0, 0.0062)),
        material=accent_red,
        name="nub_cap",
    )
    model.articulation(
        "chassis_to_trackpoint_nub",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=trackpoint,
        origin=Origin(xyz=(0.0, 0.013, 0.0160)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=1.5,
            lower=-0.18,
            upper=0.18,
        ),
    )

    wheel = model.part("media_wheel")
    wheel.visual(
        Cylinder(radius=0.0085, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=wheel_dark,
        name="wheel_tire",
    )
    wheel.visual(
        Box((0.010, 0.004, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0088)),
        material=wheel_mark,
        name="wheel_marker",
    )
    model.articulation(
        "chassis_to_media_wheel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=wheel,
        origin=Origin(xyz=wheel_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.10,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    home_left = object_model.get_part("key_r1_c4")
    home_right = object_model.get_part("key_r1_c5")
    sample_key = object_model.get_part("key_r2_c5")
    trackpoint = object_model.get_part("trackpoint_nub")
    wheel = object_model.get_part("media_wheel")

    sample_key_joint = object_model.get_articulation("chassis_to_key_r2_c5")
    trackpoint_joint = object_model.get_articulation("chassis_to_trackpoint_nub")
    wheel_joint = object_model.get_articulation("chassis_to_media_wheel")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.check("chassis exists", chassis is not None, details="Expected a root tray part.")
    ctx.check("trackpoint exists", trackpoint is not None, details="Expected central pointing nub.")
    ctx.check("media wheel exists", wheel is not None, details="Expected corner media wheel.")

    home_left_pos = ctx.part_world_position(home_left)
    home_right_pos = ctx.part_world_position(home_right)
    trackpoint_pos = ctx.part_world_position(trackpoint)
    wheel_pos = ctx.part_world_position(wheel)

    centered_ok = (
        home_left_pos is not None
        and home_right_pos is not None
        and trackpoint_pos is not None
        and home_left_pos[0] < trackpoint_pos[0] < home_right_pos[0]
        and abs(trackpoint_pos[1] - home_left_pos[1]) <= 0.0015
    )
    ctx.check(
        "trackpoint sits between the home-row keys",
        centered_ok,
        details=f"left={home_left_pos}, trackpoint={trackpoint_pos}, right={home_right_pos}",
    )

    wheel_corner_ok = (
        wheel_pos is not None
        and home_right_pos is not None
        and wheel_pos[0] > home_right_pos[0]
        and wheel_pos[1] > home_right_pos[1] + 0.020
    )
    ctx.check(
        "media wheel sits at the rear corner",
        wheel_corner_ok,
        details=f"wheel={wheel_pos}, right_home_key={home_right_pos}",
    )

    key_rest = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: 0.0032}):
        key_pressed = ctx.part_world_position(sample_key)
    key_motion_ok = (
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.0025
    )
    ctx.check(
        "sample key plunges downward",
        key_motion_ok,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    track_rest_center = _center_from_aabb(ctx.part_world_aabb(trackpoint))
    with ctx.pose({trackpoint_joint: 0.14}):
        track_tilted_center = _center_from_aabb(ctx.part_world_aabb(trackpoint))
    track_tilt_ok = (
        track_rest_center is not None
        and track_tilted_center is not None
        and (
            abs(track_tilted_center[1] - track_rest_center[1]) > 0.0004
            or abs(track_tilted_center[2] - track_rest_center[2]) > 0.0004
        )
    )
    ctx.check(
        "trackpoint nub tilts on its compliant mount",
        track_tilt_ok,
        details=f"rest={track_rest_center}, tilted={track_tilted_center}",
    )

    marker_rest_center = _center_from_aabb(ctx.part_element_world_aabb(wheel, elem="wheel_marker"))
    with ctx.pose({wheel_joint: math.pi * 0.5}):
        marker_spun_center = _center_from_aabb(ctx.part_element_world_aabb(wheel, elem="wheel_marker"))
    wheel_spin_ok = (
        marker_rest_center is not None
        and marker_spun_center is not None
        and abs(marker_spun_center[0] - marker_rest_center[0]) > 0.005
        and abs(marker_spun_center[2] - marker_rest_center[2]) > 0.005
    )
    ctx.check(
        "media wheel rotates about its local axle",
        wheel_spin_ok,
        details=f"rest={marker_rest_center}, spun={marker_spun_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
