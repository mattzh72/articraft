from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

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
    rounded_rect_profile,
    section_loft,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rounded_rect_section(
    width: float, depth: float, radius: float, z: float
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _build_wheel_visuals(
    part,
    mesh_name: str,
    *,
    tire_radius: float,
    tire_width: float,
    rubber,
    rim_metal,
    hub_metal,
    add_side_collars: bool = False,
) -> None:
    half_width = tire_width * 0.5
    hub_length = tire_width + 0.008
    tire_profile = [
        (tire_radius * 0.58, -half_width * 1.00),
        (tire_radius * 0.76, -half_width * 0.98),
        (tire_radius * 0.90, -half_width * 0.82),
        (tire_radius * 0.98, -half_width * 0.42),
        (tire_radius, -half_width * 0.10),
        (tire_radius, half_width * 0.10),
        (tire_radius * 0.98, half_width * 0.42),
        (tire_radius * 0.90, half_width * 0.82),
        (tire_radius * 0.76, half_width * 0.98),
        (tire_radius * 0.58, half_width * 1.00),
        (tire_radius * 0.47, half_width * 0.38),
        (tire_radius * 0.43, 0.0),
        (tire_radius * 0.47, -half_width * 0.38),
        (tire_radius * 0.58, -half_width * 1.00),
    ]
    tire_mesh = mesh_from_geometry(
        LatheGeometry(tire_profile, segments=64).rotate_x(pi / 2.0),
        f"{mesh_name}_tire",
    )
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))

    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=tire_radius * 0.70, length=tire_width * 0.58),
        origin=spin_origin,
        material=rim_metal,
        name="rim_shell",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.24, length=hub_length),
        origin=spin_origin,
        material=hub_metal,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.08, length=tire_width * 0.28),
        origin=spin_origin,
        material=rim_metal,
        name="axle_boss",
    )
    if add_side_collars:
        collar_length = 0.007
        collar_offset = hub_length * 0.5 + collar_length * 0.5
        part.visual(
            Cylinder(radius=tire_radius * 0.12, length=collar_length),
            origin=Origin(
                xyz=(0.0, -collar_offset, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hub_metal,
            name="inner_bearing_collar",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.12, length=collar_length),
            origin=Origin(
                xyz=(0.0, collar_offset, 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hub_metal,
            name="outer_bearing_collar",
        )


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="all_terrain_knee_scooter")

    frame_coat = model.material("frame_coat", rgba=(0.26, 0.28, 0.30, 1.0))
    fork_coat = model.material("fork_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    pad_base = model.material("pad_base", rgba=(0.10, 0.10, 0.11, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.14, 0.14, 0.15, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    wheel_radius = 0.16
    wheel_width = 0.072
    rear_wheel_x = -0.29
    front_wheel_local_x = 0.10
    wheel_center_z = wheel_radius
    wheel_track_half = 0.175
    steering_origin = (0.21, 0.0, 0.34)

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.44, 0.62)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
    )

    main_frame.visual(
        Box((0.42, 0.04, 0.03)),
        origin=Origin(xyz=(-0.03, 0.095, 0.27)),
        material=frame_coat,
        name="left_frame_rail",
    )
    main_frame.visual(
        Box((0.42, 0.04, 0.03)),
        origin=Origin(xyz=(-0.03, -0.095, 0.27)),
        material=frame_coat,
        name="right_frame_rail",
    )
    main_frame.visual(
        Box((0.06, 0.24, 0.03)),
        origin=Origin(xyz=(0.14, 0.0, 0.27)),
        material=frame_coat,
        name="front_crossmember",
    )
    main_frame.visual(
        Box((0.06, 0.20, 0.03)),
        origin=Origin(xyz=(-0.18, 0.0, 0.27)),
        material=frame_coat,
        name="rear_crossmember",
    )
    main_frame.visual(
        Box((0.06, 0.08, 0.07)),
        origin=Origin(xyz=(0.14, 0.0, 0.30)),
        material=frame_coat,
        name="head_lug",
    )
    head_tube_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.034, -0.045), (0.034, 0.045)],
            [(0.028, -0.042), (0.028, 0.042)],
            segments=56,
        ),
        "head_tube_shell",
    )
    main_frame.visual(
        head_tube_mesh,
        origin=Origin(xyz=(steering_origin[0] - 0.006, steering_origin[1], steering_origin[2])),
        material=fork_coat,
        name="head_tube",
    )
    main_frame.visual(
        Box((0.24, 0.08, 0.02)),
        origin=Origin(xyz=(-0.08, 0.0, 0.50)),
        material=frame_coat,
        name="knee_pad_support",
    )
    main_frame.visual(
        Cylinder(radius=0.018, length=0.22),
        origin=Origin(xyz=(-0.08, 0.0, 0.39)),
        material=frame_coat,
        name="center_pad_post",
    )
    _add_tube(
        main_frame,
        (-0.11, 0.095, 0.275),
        (-0.08, 0.05, 0.49),
        radius=0.012,
        material=frame_coat,
        name="left_pad_brace",
    )
    _add_tube(
        main_frame,
        (-0.11, -0.095, 0.275),
        (-0.08, -0.05, 0.49),
        radius=0.012,
        material=frame_coat,
        name="right_pad_brace",
    )
    _add_tube(
        main_frame,
        (-0.18, 0.0, 0.27),
        (-0.08, 0.0, 0.49),
        radius=0.014,
        material=frame_coat,
        name="center_pad_brace",
    )
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        wheel_y = sign * wheel_track_half
        inner_plate_y = sign * (wheel_track_half - 0.044)
        main_frame.visual(
            Box((0.024, 0.008, 0.30)),
            origin=Origin(xyz=(rear_wheel_x, inner_plate_y, 0.25)),
            material=frame_coat,
            name=f"{side_name}_rear_support_plate",
        )
        _add_tube(
            main_frame,
            (-0.15, sign * 0.115, 0.285),
            (rear_wheel_x, inner_plate_y, 0.36),
            radius=0.012,
            material=frame_coat,
            name=f"{side_name}_rear_support_brace",
        )
        main_frame.visual(
            Box((0.05, 0.04, 0.03)),
            origin=Origin(xyz=(-0.16, sign * 0.112, 0.285)),
            material=frame_coat,
            name=f"{side_name}_rear_support_lug",
        )

    knee_pad = model.part("knee_pad")
    knee_pad.inertial = Inertial.from_geometry(
        Box((0.36, 0.19, 0.08)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )
    knee_pad.visual(
        Box((0.34, 0.16, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=pad_base,
        name="pad_tray",
    )
    knee_pad_cushion = mesh_from_geometry(
        section_loft(
            [
                _rounded_rect_section(0.34, 0.18, 0.035, 0.012),
                _rounded_rect_section(0.36, 0.19, 0.050, 0.030),
                _rounded_rect_section(0.34, 0.17, 0.052, 0.054),
                _rounded_rect_section(0.30, 0.14, 0.048, 0.074),
            ]
        ),
        "knee_pad_cushion",
    )
    knee_pad.visual(knee_pad_cushion, material=pad_vinyl, name="pad_cushion")

    steering_fork = model.part("steering_fork")
    steering_fork.inertial = Inertial.from_geometry(
        Box((0.54, 0.46, 0.96)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )
    steering_fork.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=aluminum,
        name="upper_bearing_flange",
    )
    steering_fork.visual(
        Cylinder(radius=0.024, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="steering_stem",
    )
    steering_fork.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=aluminum,
        name="lower_bearing_flange",
    )
    _add_tube(
        steering_fork,
        (0.0, 0.0, 0.053),
        (-0.08, 0.0, 0.60),
        radius=0.018,
        material=fork_coat,
        name="steering_column_lower",
    )
    _add_tube(
        steering_fork,
        (-0.08, 0.0, 0.60),
        (-0.10, 0.0, 0.72),
        radius=0.015,
        material=aluminum,
        name="steering_column_upper",
    )
    steering_fork.visual(
        Box((0.08, 0.05, 0.04)),
        origin=Origin(xyz=(-0.10, 0.0, 0.74)),
        material=fork_coat,
        name="handlebar_clamp",
    )
    _add_tube(
        steering_fork,
        (-0.10, -0.22, 0.74),
        (-0.10, 0.22, 0.74),
        radius=0.014,
        material=aluminum,
        name="handlebar",
    )
    _add_tube(
        steering_fork,
        (-0.10, -0.29, 0.74),
        (-0.10, -0.22, 0.74),
        radius=0.018,
        material=grip_rubber,
        name="left_grip",
    )
    _add_tube(
        steering_fork,
        (-0.10, 0.22, 0.74),
        (-0.10, 0.29, 0.74),
        radius=0.018,
        material=grip_rubber,
        name="right_grip",
    )
    _add_tube(
        steering_fork,
        (0.06, 0.0, -0.055),
        (0.06, 0.0, 0.05),
        radius=0.020,
        material=fork_coat,
        name="fork_spine",
    )
    _add_tube(
        steering_fork,
        (0.0, 0.0, -0.055),
        (0.06, 0.0, -0.055),
        radius=0.010,
        material=fork_coat,
        name="fork_crown_bar",
    )
    steering_fork.visual(
        Box((0.08, 0.18, 0.014)),
        origin=Origin(xyz=(0.09, 0.0, 0.047)),
        material=fork_coat,
        name="fork_crown",
    )
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        wheel_y = sign * wheel_track_half
        inner_plate_y = sign * (wheel_track_half - 0.051)
        outer_plate_y = sign * (wheel_track_half + 0.051)
        steering_fork.visual(
            Box((0.07, 0.056, 0.014)),
            origin=Origin(xyz=(0.09, sign * 0.118, 0.047)),
            material=fork_coat,
            name=f"{side_name}_fork_arm",
        )
        steering_fork.visual(
            Box((0.07, 0.094, 0.014)),
            origin=Origin(xyz=(0.10, wheel_y, 0.047)),
            material=fork_coat,
            name=f"{side_name}_yoke_bridge",
        )
        steering_fork.visual(
            Box((0.018, 0.008, 0.24)),
            origin=Origin(xyz=(front_wheel_local_x, inner_plate_y, -0.08)),
            material=fork_coat,
            name=f"{side_name}_inner_yoke_plate",
        )
        steering_fork.visual(
            Box((0.018, 0.008, 0.24)),
            origin=Origin(xyz=(front_wheel_local_x, outer_plate_y, -0.08)),
            material=fork_coat,
            name=f"{side_name}_outer_yoke_plate",
        )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=dark_steel,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=dark_steel,
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_wheel_visuals(
        front_left_wheel,
        "front_left_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=dark_steel,
        add_side_collars=True,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_wheel_visuals(
        front_right_wheel,
        "front_right_wheel",
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        rubber=rubber,
        rim_metal=aluminum,
        hub_metal=dark_steel,
        add_side_collars=True,
    )

    model.articulation(
        "main_frame_to_knee_pad",
        ArticulationType.FIXED,
        parent=main_frame,
        child=knee_pad,
        origin=Origin(xyz=(-0.10, 0.0, 0.51)),
    )
    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=steering_fork,
        origin=Origin(xyz=steering_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(rear_wheel_x, wheel_track_half, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(rear_wheel_x, -wheel_track_half, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(front_wheel_local_x, wheel_track_half, -0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(front_wheel_local_x, -wheel_track_half, -0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    main_frame = object_model.get_part("main_frame")
    knee_pad = object_model.get_part("knee_pad")
    steering_fork = object_model.get_part("steering_fork")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    steering_yaw = object_model.get_articulation("steering_yaw")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        main_frame,
        steering_fork,
        elem_a="head_tube",
        elem_b="steering_stem",
        reason="The steering stem is intentionally captured inside the main frame head tube.",
    )
    ctx.allow_overlap(
        front_left_wheel,
        steering_fork,
        elem_a="inner_bearing_collar",
        reason="The simplified fork tabs stand in for slotted dropouts, so the left bearing collar is allowed to seat slightly into the tab faces while remaining captured.",
    )
    ctx.allow_overlap(
        front_right_wheel,
        steering_fork,
        elem_a="outer_bearing_collar",
        reason="The simplified fork tabs stand in for slotted dropouts, so the right bearing collar is allowed to seat slightly into the tab faces while remaining captured.",
    )
    ctx.allow_overlap(
        front_left_wheel,
        steering_fork,
        elem_a="outer_bearing_collar",
        reason="The simplified fork tabs stand in for slotted dropouts, so the left bearing collar is allowed to seat slightly into the tab faces while remaining captured.",
    )
    ctx.allow_overlap(
        front_right_wheel,
        steering_fork,
        elem_a="inner_bearing_collar",
        reason="The simplified fork tabs stand in for slotted dropouts, so the right bearing collar is allowed to seat slightly into the tab faces while remaining captured.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(knee_pad, main_frame, name="knee_pad_is_mounted")
    ctx.expect_contact(steering_fork, main_frame, name="steering_fork_is_captured")
    ctx.expect_contact(rear_left_wheel, main_frame, name="rear_left_wheel_supported")
    ctx.expect_contact(rear_right_wheel, main_frame, name="rear_right_wheel_supported")
    ctx.expect_contact(front_left_wheel, steering_fork, name="front_left_wheel_captured")
    ctx.expect_contact(front_right_wheel, steering_fork, name="front_right_wheel_captured")

    ctx.expect_origin_distance(
        front_left_wheel,
        rear_left_wheel,
        axes="x",
        min_dist=0.55,
        max_dist=0.65,
        name="wheelbase_is_all_terrain_length",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.33,
        max_dist=0.37,
        name="front_track_is_wide_and_stable",
    )

    ctx.check(
        "steering_axis_is_vertical",
        _axis_matches(steering_yaw.axis, (0.0, 0.0, 1.0)),
        details=f"steering axis was {steering_yaw.axis}",
    )
    ctx.check(
        "wheel_spin_axes_are_transverse",
        _axis_matches(rear_left_wheel_spin.axis, (0.0, 1.0, 0.0))
        and _axis_matches(rear_right_wheel_spin.axis, (0.0, 1.0, 0.0))
        and _axis_matches(front_left_wheel_spin.axis, (0.0, 1.0, 0.0))
        and _axis_matches(front_right_wheel_spin.axis, (0.0, 1.0, 0.0)),
        details=(
            f"rear_left={rear_left_wheel_spin.axis}, rear_right={rear_right_wheel_spin.axis}, "
            f"front_left={front_left_wheel_spin.axis}, front_right={front_right_wheel_spin.axis}"
        ),
    )

    rest_left = ctx.part_world_position(front_left_wheel)
    turned_left = None
    with ctx.pose({steering_yaw: 0.45}):
        turned_left = ctx.part_world_position(front_left_wheel)
        ctx.expect_contact(
            front_left_wheel,
            steering_fork,
            name="front_left_wheel_stays_captured_while_steering",
        )
        ctx.expect_contact(
            front_right_wheel,
            steering_fork,
            name="front_right_wheel_stays_captured_while_steering",
        )
        ctx.expect_contact(
            front_left_wheel,
            steering_fork,
            elem_a="outer_bearing_collar",
            elem_b="left_outer_yoke_plate",
            name="left_outer_collar_remains_seated_at_steering_lock",
        )
        ctx.expect_contact(
            front_right_wheel,
            steering_fork,
            elem_a="inner_bearing_collar",
            elem_b="right_outer_yoke_plate",
            name="right_outer_collar_remains_seated_at_steering_lock",
        )

    moved_with_steer = False
    if rest_left is not None and turned_left is not None:
        moved_with_steer = turned_left[1] > rest_left[1] + 0.02 and turned_left[0] < rest_left[0] - 0.04
    ctx.check(
        "front_pair_follows_steering_yaw",
        moved_with_steer,
        details=f"rest={rest_left}, turned={turned_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
