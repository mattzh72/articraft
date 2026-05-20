from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


BASE_X = 0.320
BASE_Y = 0.260
BASE_Z = 0.118
BASE_FLANGE_Z = 0.014
GIMBAL_Z = 0.272

TOWER_X = 0.024
TOWER_Y = 0.120
TOWER_Z = 0.164
TOWER_CX = 0.126

FRAME_Y = 0.060
LEVER_SHAFT_H = 0.440


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder_x(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_z(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_cardan_joystick")

    housing_color = model.material("housing_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    yoke_color = model.material("yoke_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    inner_color = model.material("inner_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    grip_color = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base_housing")
    _add_box(base, (BASE_X + 0.036, BASE_Y + 0.036, BASE_FLANGE_Z), (0.0, 0.0, BASE_FLANGE_Z / 2.0), housing_color, "mount_flange")
    _add_box(base, (BASE_X, BASE_Y, BASE_Z), (0.0, 0.0, BASE_FLANGE_Z + BASE_Z / 2.0), housing_color, "housing_shell")
    _add_box(base, (0.208, 0.156, 0.012), (0.0, 0.0, BASE_FLANGE_Z + BASE_Z + 0.006), housing_color, "interface_flange")
    _add_box(base, (TOWER_X, TOWER_Y, TOWER_Z), (-TOWER_CX, 0.0, BASE_FLANGE_Z + BASE_Z + TOWER_Z / 2.0), housing_color, "left_bearing_tower")
    _add_box(base, (TOWER_X, TOWER_Y, TOWER_Z), (TOWER_CX, 0.0, BASE_FLANGE_Z + BASE_Z + TOWER_Z / 2.0), housing_color, "right_bearing_tower")
    _add_box(base, (0.022, 0.036, 0.064), (-0.112, 0.036, BASE_FLANGE_Z + BASE_Z + 0.032), housing_color, "left_front_brace")
    _add_box(base, (0.022, 0.036, 0.064), (-0.112, -0.036, BASE_FLANGE_Z + BASE_Z + 0.032), housing_color, "left_rear_brace")
    _add_box(base, (0.022, 0.036, 0.064), (0.112, 0.036, BASE_FLANGE_Z + BASE_Z + 0.032), housing_color, "right_front_brace")
    _add_box(base, (0.022, 0.036, 0.064), (0.112, -0.036, BASE_FLANGE_Z + BASE_Z + 0.032), housing_color, "right_rear_brace")
    _add_box(base, (0.136, 0.010, 0.010), (0.0, 0.080, BASE_FLANGE_Z + BASE_Z + 0.005), housing_color, "front_guard_plate")
    _add_box(base, (0.136, 0.010, 0.010), (0.0, -0.080, BASE_FLANGE_Z + BASE_Z + 0.005), housing_color, "rear_guard_plate")
    _add_box(base, (0.014, 0.164, 0.012), (-0.090, 0.0, BASE_FLANGE_Z + BASE_Z + 0.006), housing_color, "left_deck_rail")
    _add_box(base, (0.014, 0.164, 0.012), (0.090, 0.0, BASE_FLANGE_Z + BASE_Z + 0.006), housing_color, "right_deck_rail")
    base.inertial = Inertial.from_geometry(
        Box((BASE_X + 0.036, BASE_Y + 0.036, BASE_FLANGE_Z + BASE_Z + TOWER_Z)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_FLANGE_Z + BASE_Z + TOWER_Z) / 2.0)),
    )

    outer_yoke = model.part("outer_yoke")
    for prefix, sign_y in (("front", 1.0), ("rear", -1.0)):
        y = sign_y * FRAME_Y
        _add_box(outer_yoke, (0.208, 0.012, 0.014), (0.0, y, 0.070), yoke_color, f"{prefix}_top_bar")
        _add_box(outer_yoke, (0.188, 0.012, 0.016), (0.0, y, -0.050), yoke_color, f"{prefix}_bottom_bar")
        _add_box(outer_yoke, (0.014, 0.012, 0.150), (-0.068, y, 0.010), yoke_color, f"{prefix}_left_upright")
        _add_box(outer_yoke, (0.014, 0.012, 0.150), (0.068, y, 0.010), yoke_color, f"{prefix}_right_upright")
    _add_box(outer_yoke, (0.012, 0.120, 0.016), (-0.104, 0.0, 0.070), yoke_color, "left_cap_strap")
    _add_box(outer_yoke, (0.012, 0.120, 0.016), (0.104, 0.0, 0.070), yoke_color, "right_cap_strap")
    _add_box(outer_yoke, (0.018, 0.120, 0.018), (-0.092, 0.0, -0.050), yoke_color, "left_bottom_bridge")
    _add_box(outer_yoke, (0.018, 0.120, 0.018), (0.092, 0.0, -0.050), yoke_color, "right_bottom_bridge")
    _add_box(outer_yoke, (0.024, 0.124, 0.060), (-0.084, 0.0, 0.0), yoke_color, "left_trunnion_block")
    _add_box(outer_yoke, (0.024, 0.124, 0.060), (0.084, 0.0, 0.0), yoke_color, "right_trunnion_block")
    _add_box(outer_yoke, (0.136, 0.012, 0.032), (0.0, 0.060, 0.0), yoke_color, "front_axis_plate")
    _add_box(outer_yoke, (0.136, 0.012, 0.032), (0.0, -0.060, 0.0), yoke_color, "rear_axis_plate")
    _add_cylinder_x(outer_yoke, 0.013, 0.031, (-0.0905, 0.0, 0.0), yoke_color, "left_trunnion_stub")
    _add_cylinder_x(outer_yoke, 0.013, 0.031, (0.0905, 0.0, 0.0), yoke_color, "right_trunnion_stub")
    _add_cylinder_x(outer_yoke, 0.019, 0.008, (-0.110, 0.0, 0.0), yoke_color, "left_trunnion_cap")
    _add_cylinder_x(outer_yoke, 0.019, 0.008, (0.110, 0.0, 0.0), yoke_color, "right_trunnion_cap")
    _add_cylinder_y(outer_yoke, 0.017, 0.012, (0.0, 0.060, 0.0), yoke_color, "front_roll_boss")
    _add_cylinder_y(outer_yoke, 0.017, 0.012, (0.0, -0.060, 0.0), yoke_color, "rear_roll_boss")
    outer_yoke.inertial = Inertial.from_geometry(
        Box((0.228, 0.132, 0.180)),
        mass=2.4,
        origin=Origin(),
    )

    inner_frame = model.part("inner_frame")
    _add_box(inner_frame, (0.012, 0.020, 0.104), (-0.030, 0.0, 0.002), inner_color, "left_cradle_post")
    _add_box(inner_frame, (0.012, 0.020, 0.104), (0.030, 0.0, 0.002), inner_color, "right_cradle_post")
    _add_box(inner_frame, (0.072, 0.020, 0.012), (0.0, 0.0, 0.046), inner_color, "top_cradle_bridge")
    _add_box(inner_frame, (0.068, 0.024, 0.018), (0.0, 0.0, -0.036), inner_color, "bottom_cradle_bridge")
    _add_box(inner_frame, (0.060, 0.020, 0.018), (0.0, 0.0, 0.0), inner_color, "central_hub_block")
    _add_cylinder_y(inner_frame, 0.010, 0.096, (0.0, 0.0, 0.0), inner_color, "roll_tube")
    _add_cylinder_y(inner_frame, 0.016, 0.006, (0.0, 0.051, 0.0), inner_color, "front_roll_flange")
    _add_cylinder_y(inner_frame, 0.016, 0.006, (0.0, -0.051, 0.0), inner_color, "rear_roll_flange")
    _add_cylinder_z(inner_frame, 0.024, 0.050, (0.0, 0.0, -0.035), inner_color, "lever_socket")
    _add_cylinder_z(inner_frame, 0.028, 0.008, (0.0, 0.0, -0.006), inner_color, "socket_collar")
    _add_cylinder_z(inner_frame, 0.0135, 0.120, (0.0, 0.0, 0.060), inner_color, "lever_lower_shaft")
    _add_cylinder_z(inner_frame, 0.0105, 0.170, (0.0, 0.0, 0.205), inner_color, "lever_mid_shaft")
    _add_cylinder_z(inner_frame, 0.0085, 0.150, (0.0, 0.0, 0.365), inner_color, "lever_upper_shaft")
    _add_cylinder_z(inner_frame, 0.014, 0.060, (0.0, 0.0, 0.470), grip_color, "grip")
    inner_frame.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.509)),
        material=grip_color,
        name="grip_cap",
    )
    inner_frame.inertial = Inertial.from_geometry(
        Box((0.100, 0.120, 0.560)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
    )

    model.articulation(
        "outer_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.42, upper=0.48),
    )
    model.articulation(
        "inner_roll",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_frame,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base_housing")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_frame = object_model.get_part("inner_frame")
    outer_pitch = object_model.get_articulation("outer_pitch")
    inner_roll = object_model.get_articulation("inner_roll")

    ctx.check(
        "outer_pitch_axis_is_transverse",
        tuple(round(v, 4) for v in outer_pitch.axis) == (1.0, 0.0, 0.0),
        f"outer_pitch axis was {outer_pitch.axis}",
    )
    ctx.check(
        "inner_roll_axis_is_perpendicular",
        tuple(round(v, 4) for v in inner_roll.axis) == (0.0, 1.0, 0.0),
        f"inner_roll axis was {inner_roll.axis}",
    )
    ctx.expect_origin_gap(
        outer_yoke,
        base,
        axis="z",
        min_gap=0.268,
        max_gap=0.276,
        name="gimbal_center_height_above_base",
    )
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulation_clearance_sweep")

    with ctx.pose({outer_pitch: outer_pitch.motion_limits.upper, inner_roll: inner_roll.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="positive_extreme_pose_clear")

    with ctx.pose({outer_pitch: outer_pitch.motion_limits.lower, inner_roll: inner_roll.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="negative_extreme_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
