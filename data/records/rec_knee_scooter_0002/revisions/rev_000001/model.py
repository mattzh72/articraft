from __future__ import annotations

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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
WHEEL_RADIUS = 0.102
WHEEL_WIDTH = 0.036
HUB_LENGTH = 0.024
TRACK_HALF = 0.122
FRAME_HALF_WIDTH = 0.082
REAR_WHEEL_Y = -0.225
STEERING_ORIGIN = (0.0, 0.120, 0.254)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _rounded_rect_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _wheel_tire_mesh():
    half_width = WHEEL_WIDTH * 0.5
    profile = [
        (WHEEL_RADIUS * 0.62, -half_width * 0.98),
        (WHEEL_RADIUS * 0.86, -half_width),
        (WHEEL_RADIUS * 0.97, -half_width * 0.62),
        (WHEEL_RADIUS, -half_width * 0.16),
        (WHEEL_RADIUS, half_width * 0.16),
        (WHEEL_RADIUS * 0.97, half_width * 0.62),
        (WHEEL_RADIUS * 0.86, half_width),
        (WHEEL_RADIUS * 0.62, half_width * 0.98),
        (WHEEL_RADIUS * 0.48, half_width * 0.36),
        (WHEEL_RADIUS * 0.44, 0.0),
        (WHEEL_RADIUS * 0.48, -half_width * 0.36),
        (WHEEL_RADIUS * 0.62, -half_width * 0.98),
    ]
    return _save_mesh(
        "knee_scooter_tire.obj",
        LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0),
    )


def _add_wheel_visuals(part, tire_mesh, rim_material, hub_material, tire_material) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(Cylinder(radius=0.072, length=0.012), origin=spin_origin, material=rim_material, name="rim")
    part.visual(Cylinder(radius=0.046, length=0.018), origin=spin_origin, material=rim_material, name="hub_flange")
    part.visual(Cylinder(radius=0.020, length=HUB_LENGTH), origin=spin_origin, material=hub_material, name="hub_core")
    part.visual(Cylinder(radius=0.010, length=HUB_LENGTH * 0.84), origin=spin_origin, material=rim_material, name="axle_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter", assets=ASSETS)

    frame_metal = model.material("frame_metal", rgba=(0.76, 0.79, 0.82, 1.0))
    fork_metal = model.material("fork_metal", rgba=(0.30, 0.33, 0.36, 1.0))
    rim_metal = model.material("rim_metal", rgba=(0.66, 0.69, 0.72, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.22, 0.24, 0.26, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.12, 0.13, 0.14, 1.0))

    tire_mesh = _wheel_tire_mesh()

    base_frame = model.part("base_frame")
    base_frame.inertial = Inertial.from_geometry(
        Box((0.34, 0.76, 0.58)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -0.02, 0.29)),
    )
    base_frame.visual(
        _save_mesh(
            "knee_scooter_left_lower_rail.obj",
            tube_from_spline_points(
                [(FRAME_HALF_WIDTH, -0.250, 0.182), (FRAME_HALF_WIDTH, -0.020, 0.182)],
                radius=0.016,
                samples_per_segment=4,
                radial_segments=18,
            ),
        ),
        material=frame_metal,
    )
    base_frame.visual(
        _save_mesh(
            "knee_scooter_right_lower_rail.obj",
            tube_from_spline_points(
                [(-FRAME_HALF_WIDTH, -0.250, 0.182), (-FRAME_HALF_WIDTH, -0.020, 0.182)],
                radius=0.016,
                samples_per_segment=4,
                radial_segments=18,
            ),
        ),
        material=frame_metal,
    )
    base_frame.visual(
        Cylinder(radius=0.016, length=FRAME_HALF_WIDTH * 2.0),
        origin=Origin(xyz=(0.0, -0.250, 0.182), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_metal,
    )
    base_frame.visual(
        Cylinder(radius=0.016, length=0.150),
        origin=Origin(xyz=(0.0, -0.020, 0.182), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_metal,
    )
    base_frame.visual(
        Box((0.150, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, -0.080, 0.190)),
        material=frame_metal,
        name="center_brace",
    )
    base_frame.visual(
        Box((0.120, 0.065, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, 0.190)),
        material=frame_metal,
    )
    base_frame.visual(
        Cylinder(radius=0.018, length=0.300),
        origin=Origin(xyz=(0.0, -0.03, 0.331)),
        material=frame_metal,
        name="knee_support_post",
    )
    base_frame.visual(
        Box((0.080, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.03, 0.483)),
        material=frame_metal,
        name="knee_post_cap",
    )
    base_frame.visual(
        _save_mesh(
            "knee_scooter_left_pad_brace.obj",
            tube_from_spline_points(
                [(0.060, -0.115, 0.190), (0.035, -0.070, 0.250), (0.0, -0.030, 0.324)],
                radius=0.012,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_metal,
    )
    base_frame.visual(
        _save_mesh(
            "knee_scooter_right_pad_brace.obj",
            tube_from_spline_points(
                _mirror_x([(0.060, -0.115, 0.190), (0.035, -0.070, 0.250), (0.0, -0.030, 0.324)]),
                radius=0.012,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_metal,
    )
    base_frame.visual(
        Cylinder(radius=0.028, length=0.120),
        origin=Origin(xyz=STEERING_ORIGIN),
        material=fork_metal,
        name="steering_head",
    )
    base_frame.visual(
        _save_mesh(
            "knee_scooter_left_head_brace.obj",
            tube_from_spline_points(
                [(0.055, -0.020, 0.190), (0.030, 0.045, 0.220), (0.0, 0.120, 0.244)],
                radius=0.012,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_metal,
    )
    base_frame.visual(
        _save_mesh(
            "knee_scooter_right_head_brace.obj",
            tube_from_spline_points(
                _mirror_x([(0.055, -0.020, 0.190), (0.030, 0.045, 0.220), (0.0, 0.120, 0.244)]),
                radius=0.012,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=frame_metal,
    )
    base_frame.visual(
        Box((0.018, 0.060, 0.122)),
        origin=Origin(xyz=(0.096, REAR_WHEEL_Y, 0.121)),
        material=fork_metal,
    )
    base_frame.visual(
        Box((0.018, 0.060, 0.122)),
        origin=Origin(xyz=(-0.096, REAR_WHEEL_Y, 0.121)),
        material=fork_metal,
    )
    base_frame.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.098, REAR_WHEEL_Y, WHEEL_RADIUS), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="rear_left_axle_stub",
    )
    base_frame.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(-0.098, REAR_WHEEL_Y, WHEEL_RADIUS), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="rear_right_axle_stub",
    )

    knee_platform = model.part("knee_platform")
    knee_platform.inertial = Inertial.from_geometry(
        Box((0.220, 0.140, 0.070)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )
    knee_platform.visual(
        Box((0.140, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=fork_metal,
        name="mounting_plate",
    )
    knee_platform.visual(
        _save_mesh(
            "knee_scooter_pad.obj",
            section_loft(
                [
                    _rounded_rect_section(0.190, 0.110, 0.028, 0.006),
                    _rounded_rect_section(0.212, 0.132, 0.040, 0.028),
                    _rounded_rect_section(0.208, 0.128, 0.044, 0.054),
                    _rounded_rect_section(0.196, 0.120, 0.046, 0.066),
                ]
            ),
        ),
        material=pad_vinyl,
        name="knee_pad",
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.inertial = Inertial.from_geometry(
        Box((0.460, 0.180, 0.700)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.04, 0.20)),
    )
    steering_assembly.visual(
        Cylinder(radius=0.024, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=fork_metal,
        name="steering_collar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.019, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.338)),
        material=fork_metal,
        name="steering_stem",
    )
    steering_assembly.visual(
        Box((0.060, 0.028, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=fork_metal,
        name="handlebar_clamp",
    )
    steering_assembly.visual(
        Box((0.420, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=fork_metal,
        name="handlebar_bar",
    )
    steering_assembly.visual(
        Box((0.026, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.040, 0.082)),
        material=fork_metal,
        name="fork_neck_link",
    )
    steering_assembly.visual(
        Box((0.100, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.080, 0.058)),
        material=fork_metal,
        name="fork_crown",
    )
    steering_assembly.visual(
        _save_mesh(
            "knee_scooter_left_fork_arm.obj",
            tube_from_spline_points(
                [(0.040, 0.088, 0.058), (0.066, 0.118, 0.032), (0.094, 0.160, 0.006)],
                radius=0.012,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=fork_metal,
        name="left_fork_arm",
    )
    steering_assembly.visual(
        _save_mesh(
            "knee_scooter_right_fork_arm.obj",
            tube_from_spline_points(
                _mirror_x([(0.040, 0.088, 0.058), (0.066, 0.118, 0.032), (0.094, 0.160, 0.006)]),
                radius=0.012,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=fork_metal,
        name="right_fork_arm",
    )
    steering_assembly.visual(
        Box((0.018, 0.048, 0.154)),
        origin=Origin(xyz=(0.094, 0.170, -0.070)),
        material=fork_metal,
        name="left_fork_leg",
    )
    steering_assembly.visual(
        Box((0.018, 0.048, 0.154)),
        origin=Origin(xyz=(-0.094, 0.170, -0.070)),
        material=fork_metal,
        name="right_fork_leg",
    )
    steering_assembly.visual(
        Box((0.190, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.170, -0.144)),
        material=fork_metal,
        name="front_axle_crossbar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.098, 0.170, -0.152), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="front_left_axle_stub",
    )
    steering_assembly.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(-0.098, 0.170, -0.152), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="front_right_axle_stub",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(rear_left_wheel, tire_mesh, rim_metal, hub_metal, tire_rubber)

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(rear_right_wheel, tire_mesh, rim_metal, hub_metal, tire_rubber)

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(front_left_wheel, tire_mesh, rim_metal, hub_metal, tire_rubber)

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(front_right_wheel, tire_mesh, rim_metal, hub_metal, tire_rubber)

    model.articulation(
        "knee_platform_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=knee_platform,
        origin=Origin(xyz=(0.0, -0.03, 0.485)),
    )
    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=steering_assembly,
        origin=Origin(xyz=STEERING_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(TRACK_HALF, REAR_WHEEL_Y, WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-TRACK_HALF, REAR_WHEEL_Y, WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_left_wheel,
        origin=Origin(xyz=(TRACK_HALF, 0.170, -0.152)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_right_wheel,
        origin=Origin(xyz=(-TRACK_HALF, 0.170, -0.152)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    knee_platform = object_model.get_part("knee_platform")
    steering_assembly = object_model.get_part("steering_assembly")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    steering_yaw = object_model.get_articulation("steering_yaw")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")

    knee_post_cap = base_frame.get_visual("knee_post_cap")
    steering_head = base_frame.get_visual("steering_head")
    rear_left_stub = base_frame.get_visual("rear_left_axle_stub")
    rear_right_stub = base_frame.get_visual("rear_right_axle_stub")
    mounting_plate = knee_platform.get_visual("mounting_plate")
    knee_pad = knee_platform.get_visual("knee_pad")
    steering_collar = steering_assembly.get_visual("steering_collar")
    handlebar_bar = steering_assembly.get_visual("handlebar_bar")
    front_left_stub = steering_assembly.get_visual("front_left_axle_stub")
    front_right_stub = steering_assembly.get_visual("front_right_axle_stub")
    rear_left_hub = rear_left_wheel.get_visual("hub_core")
    rear_right_hub = rear_right_wheel.get_visual("hub_core")
    front_left_hub = front_left_wheel.get_visual("hub_core")
    front_right_hub = front_right_wheel.get_visual("hub_core")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(knee_platform, base_frame, elem_a=mounting_plate, elem_b=knee_post_cap, name="knee_platform_mounted")
    ctx.expect_contact(steering_assembly, base_frame, elem_a=steering_collar, elem_b=steering_head, name="steering_head_seated")
    ctx.expect_contact(rear_left_wheel, base_frame, elem_a=rear_left_hub, elem_b=rear_left_stub, name="rear_left_wheel_on_stub")
    ctx.expect_contact(rear_right_wheel, base_frame, elem_a=rear_right_hub, elem_b=rear_right_stub, name="rear_right_wheel_on_stub")
    ctx.expect_contact(front_left_wheel, steering_assembly, elem_a=front_left_hub, elem_b=front_left_stub, name="front_left_wheel_on_stub")
    ctx.expect_contact(front_right_wheel, steering_assembly, elem_a=front_right_hub, elem_b=front_right_stub, name="front_right_wheel_on_stub")

    ctx.expect_origin_distance(knee_platform, base_frame, axes="x", max_dist=0.02, name="knee_platform_centered")
    ctx.expect_overlap(knee_platform, base_frame, axes="xy", min_overlap=0.08, name="knee_platform_over_frame")

    steering_axis = tuple(round(value, 4) for value in steering_yaw.axis)
    ctx.check(
        "steering_joint_is_vertical_yaw",
        steering_yaw.articulation_type == ArticulationType.REVOLUTE and steering_axis == (0.0, 0.0, 1.0),
        details=f"Expected revolute z-axis steering, got type={steering_yaw.articulation_type} axis={steering_yaw.axis}",
    )
    steering_limits = steering_yaw.motion_limits
    ctx.check(
        "steering_limits_match_prompt",
        steering_limits is not None
        and abs(steering_limits.lower + math.pi / 4.0) < 1e-6
        and abs(steering_limits.upper - math.pi / 4.0) < 1e-6,
        details=f"Steering limits were {steering_limits}",
    )

    for joint_name, joint in (
        ("rear_left_spin", rear_left_spin),
        ("rear_right_spin", rear_right_spin),
        ("front_left_spin", front_left_spin),
        ("front_right_spin", front_right_spin),
    ):
        axis = tuple(round(value, 4) for value in joint.axis)
        ctx.check(
            f"{joint_name}_continuous_spin",
            joint.articulation_type == ArticulationType.CONTINUOUS and axis == (1.0, 0.0, 0.0),
            details=f"Expected continuous x-axis spin for {joint_name}, got type={joint.articulation_type} axis={joint.axis}",
        )

    frame_pos = ctx.part_world_position(base_frame)
    knee_pos = ctx.part_world_position(knee_platform)
    front_left_rest = ctx.part_world_position(front_left_wheel)
    front_right_rest = ctx.part_world_position(front_right_wheel)
    rear_left_rest = ctx.part_world_position(rear_left_wheel)
    rear_right_rest = ctx.part_world_position(rear_right_wheel)
    core_positions_ok = all(
        position is not None
        for position in (frame_pos, knee_pos, front_left_rest, front_right_rest, rear_left_rest, rear_right_rest)
    )
    ctx.check("core_positions_available", core_positions_ok, details="Expected world positions for frame, pad, and all wheels.")
    if core_positions_ok:
        assert frame_pos is not None
        assert knee_pos is not None
        assert front_left_rest is not None
        assert front_right_rest is not None
        assert rear_left_rest is not None
        assert rear_right_rest is not None
        ctx.check(
            "knee_platform_height",
            knee_pos[2] > frame_pos[2] + 0.46,
            details=f"Knee platform origin {knee_pos} was not high above frame origin {frame_pos}.",
        )
        wheelbase = front_left_rest[1] - rear_left_rest[1]
        front_track = front_left_rest[0] - front_right_rest[0]
        rear_track = rear_left_rest[0] - rear_right_rest[0]
        ctx.check(
            "wheelbase_proportion",
            0.48 < wheelbase < 0.56,
            details=f"Expected knee-scooter wheelbase near 0.52 m, got {wheelbase:.4f} m.",
        )
        ctx.check(
            "wheel_tracks_match",
            abs(front_track - rear_track) < 0.02 and 0.24 < front_track < 0.29,
            details=f"Front/rear tracks were {front_track:.4f} and {rear_track:.4f} m.",
        )
        ctx.check(
            "wheel_centers_on_ground_plane",
            max(
                abs(front_left_rest[2] - WHEEL_RADIUS),
                abs(front_right_rest[2] - WHEEL_RADIUS),
                abs(rear_left_rest[2] - WHEEL_RADIUS),
                abs(rear_right_rest[2] - WHEEL_RADIUS),
            )
            < 0.003,
            details=(
                f"Wheel center heights were {front_left_rest[2]:.4f}, {front_right_rest[2]:.4f}, "
                f"{rear_left_rest[2]:.4f}, {rear_right_rest[2]:.4f}."
            ),
        )

    handlebar_aabb = ctx.part_element_world_aabb(steering_assembly, elem=handlebar_bar)
    knee_pad_aabb = ctx.part_element_world_aabb(knee_platform, elem=knee_pad)
    aabbs_ok = handlebar_aabb is not None and knee_pad_aabb is not None
    ctx.check("feature_aabbs_available", aabbs_ok, details="Expected measurable AABBs for handlebar and knee pad visuals.")
    if aabbs_ok:
        assert handlebar_aabb is not None
        assert knee_pad_aabb is not None
        bar_width = handlebar_aabb[1][0] - handlebar_aabb[0][0]
        ctx.check(
            "handlebar_width_reads_bicycle_style",
            0.34 < bar_width < 0.50,
            details=f"Handlebar width was {bar_width:.4f} m.",
        )
        ctx.check(
            "handlebar_above_knee_pad",
            handlebar_aabb[1][2] > knee_pad_aabb[1][2] + 0.25,
            details=(
                f"Handlebar top z {handlebar_aabb[1][2]:.4f} was not well above "
                f"knee pad top z {knee_pad_aabb[1][2]:.4f}."
            ),
        )

    turned_left_pos = None
    with ctx.pose({steering_yaw: math.radians(40.0)}):
        ctx.expect_contact(
            front_left_wheel,
            steering_assembly,
            elem_a=front_left_hub,
            elem_b=front_left_stub,
            name="front_left_contact_at_left_lock",
        )
        ctx.expect_contact(
            front_right_wheel,
            steering_assembly,
            elem_a=front_right_hub,
            elem_b=front_right_stub,
            name="front_right_contact_at_left_lock",
        )
        turned_left_pos = ctx.part_world_position(front_left_wheel)
    if core_positions_ok and turned_left_pos is not None:
        assert front_left_rest is not None
        ctx.check(
            "front_left_wheel_moves_forward_when_steered_left",
            turned_left_pos[1] > front_left_rest[1] + 0.02 and turned_left_pos[0] < front_left_rest[0] - 0.08,
            details=f"Front-left wheel y moved from {front_left_rest[1]:.4f} to {turned_left_pos[1]:.4f}.",
        )

    turned_right_pos = None
    with ctx.pose({steering_yaw: -math.radians(40.0)}):
        ctx.expect_contact(
            front_left_wheel,
            steering_assembly,
            elem_a=front_left_hub,
            elem_b=front_left_stub,
            name="front_left_contact_at_right_lock",
        )
        ctx.expect_contact(
            front_right_wheel,
            steering_assembly,
            elem_a=front_right_hub,
            elem_b=front_right_stub,
            name="front_right_contact_at_right_lock",
        )
        turned_right_pos = ctx.part_world_position(front_right_wheel)
    if core_positions_ok and turned_right_pos is not None:
        assert front_right_rest is not None
        ctx.check(
            "front_right_wheel_moves_forward_when_steered_right",
            turned_right_pos[1] > front_right_rest[1] + 0.02 and turned_right_pos[0] > front_right_rest[0] + 0.08,
            details=f"Front-right wheel y moved from {front_right_rest[1]:.4f} to {turned_right_pos[1]:.4f}.",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
