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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _rect_tube_mesh(
    name: str,
    *,
    outer_x: float,
    outer_y: float,
    wall: float,
    height: float,
    radius: float,
):
    inner_x = outer_x - 2.0 * wall
    inner_y = outer_y - 2.0 * wall
    inner_radius = max(0.001, min(radius - wall * 0.5, inner_x * 0.25, inner_y * 0.25))
    outer_profile = rounded_rect_profile(outer_x, outer_y, radius)
    inner_profile = rounded_rect_profile(inner_x, inner_y, inner_radius)
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            height=height,
            center=True,
        ),
    )


def _rounded_pad_mesh(name: str, *, width: float, depth: float, thickness: float, radius: float):
    return _save_mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius),
            height=thickness,
            center=True,
        ),
    )


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return _save_mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=16,
        ),
    )


def _wheel_tire_mesh(name: str, *, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.42, -half_width * 0.94),
        (radius * 0.72, -half_width),
        (radius * 0.90, -half_width * 0.80),
        (radius * 0.985, -half_width * 0.36),
        (radius, 0.0),
        (radius * 0.985, half_width * 0.36),
        (radius * 0.90, half_width * 0.80),
        (radius * 0.72, half_width),
        (radius * 0.42, half_width * 0.94),
        (radius * 0.33, half_width * 0.30),
        (radius * 0.33, -half_width * 0.30),
        (radius * 0.42, -half_width * 0.94),
    ]
    return _save_mesh(name, LatheGeometry(profile, segments=60).rotate_y(math.pi / 2.0))


def _add_small_wheel(
    part,
    *,
    mesh_prefix: str,
    tire_radius: float,
    tire_width: float,
    rubber,
    rim_metal,
    hub_metal,
) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        _wheel_tire_mesh(f"{mesh_prefix}_tire", radius=tire_radius, width=tire_width),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.74, length=0.032),
        origin=spin_origin,
        material=rim_metal,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.30, length=0.038),
        origin=spin_origin,
        material=hub_metal,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.12, length=0.056),
        origin=spin_origin,
        material=hub_metal,
        name="axle_spacer",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.28, length=0.010),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="left_bearing_cap",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.28, length=0.010),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_metal,
        name="right_bearing_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter")

    frame_blue = model.material("frame_blue", rgba=(0.17, 0.36, 0.63, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    rim_metal = model.material("rim_metal", rgba=(0.78, 0.80, 0.84, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.43, 0.46, 0.50, 1.0))
    pad_black = model.material("pad_black", rgba=(0.09, 0.09, 0.10, 1.0))

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.44, 0.80, 0.46)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    main_frame.visual(
        Box((0.060, 0.480, 0.036)),
        origin=Origin(xyz=(0.0, -0.015, 0.215)),
        material=frame_blue,
        name="center_spine",
    )
    main_frame.visual(
        _tube_mesh(
            "left_lower_rail",
            [
                (0.094, -0.220, 0.190),
                (0.104, -0.060, 0.220),
                (0.096, 0.090, 0.215),
                (0.056, 0.215, 0.235),
            ],
            0.017,
        ),
        material=frame_blue,
        name="left_lower_rail",
    )
    main_frame.visual(
        _tube_mesh(
            "right_lower_rail",
            _mirror_x(
                [
                    (0.094, -0.220, 0.190),
                    (0.104, -0.060, 0.220),
                    (0.096, 0.090, 0.215),
                    (0.056, 0.215, 0.235),
                ]
            ),
            0.017,
        ),
        material=frame_blue,
        name="right_lower_rail",
    )
    main_frame.visual(
        Cylinder(radius=0.016, length=0.210),
        origin=Origin(xyz=(0.0, -0.220, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_blue,
        name="rear_cross_tube",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.170),
        origin=Origin(xyz=(0.0, 0.100, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_blue,
        name="front_cross_tube",
    )
    main_frame.visual(
        _tube_mesh(
            "left_head_brace",
            [(0.060, 0.180, 0.225), (0.040, 0.225, 0.280), (0.024, 0.240, 0.335)],
            0.014,
        ),
        material=frame_blue,
        name="left_head_brace",
    )
    main_frame.visual(
        _tube_mesh(
            "right_head_brace",
            _mirror_x([(0.060, 0.180, 0.225), (0.040, 0.225, 0.280), (0.024, 0.240, 0.335)]),
            0.014,
        ),
        material=frame_blue,
        name="right_head_brace",
    )
    main_frame.visual(
        _tube_mesh(
            "left_knee_brace",
            [(0.078, -0.045, 0.220), (0.050, -0.040, 0.280), (0.028, -0.030, 0.335)],
            0.014,
        ),
        material=frame_blue,
        name="left_knee_brace",
    )
    main_frame.visual(
        _tube_mesh(
            "right_knee_brace",
            _mirror_x([(0.078, -0.045, 0.220), (0.050, -0.040, 0.280), (0.028, -0.030, 0.335)]),
            0.014,
        ),
        material=frame_blue,
        name="right_knee_brace",
    )
    main_frame.visual(
        _tube_mesh(
            "left_rear_leg",
            [(0.098, -0.205, 0.195), (0.136, -0.228, 0.216), (0.170, -0.250, 0.220)],
            0.014,
        ),
        material=frame_blue,
        name="left_rear_leg",
    )
    main_frame.visual(
        _tube_mesh(
            "right_rear_leg",
            _mirror_x([(0.098, -0.205, 0.195), (0.136, -0.228, 0.216), (0.170, -0.250, 0.220)]),
            0.014,
        ),
        material=frame_blue,
        name="right_rear_leg",
    )
    main_frame.visual(
        Box((0.080, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.030, 0.235)),
        material=frame_blue,
        name="knee_sleeve_mount",
    )
    main_frame.visual(
        _rect_tube_mesh(
            "knee_sleeve_shell",
            outer_x=0.056,
            outer_y=0.066,
            wall=0.006,
            height=0.160,
            radius=0.010,
        ),
        origin=Origin(xyz=(0.0, -0.030, 0.340)),
        material=charcoal,
        name="knee_sleeve_shell",
    )
    main_frame.visual(
        Box((0.070, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.014, 0.418)),
        material=charcoal,
        name="knee_clamp_block",
    )
    main_frame.visual(
        Box((0.014, 0.060, 0.044)),
        origin=Origin(xyz=(0.030, 0.214, 0.255)),
        material=frame_blue,
        name="head_tube_mount",
    )
    main_frame.visual(
        Box((0.014, 0.060, 0.044)),
        origin=Origin(xyz=(-0.030, 0.214, 0.255)),
        material=frame_blue,
        name="head_tube_right_mount",
    )
    main_frame.visual(
        _rect_tube_mesh(
            "head_tube_shell",
            outer_x=0.058,
            outer_y=0.068,
            wall=0.006,
            height=0.180,
            radius=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.240, 0.327)),
        material=charcoal,
        name="head_tube_shell",
    )
    main_frame.visual(
        _rect_tube_mesh(
            "head_bearing_collar",
            outer_x=0.048,
            outer_y=0.058,
            wall=0.007,
            height=0.010,
            radius=0.008,
        ),
        origin=Origin(xyz=(0.0, 0.240, 0.235)),
        material=charcoal,
        name="head_bearing_collar",
    )
    main_frame.visual(
        Box((0.008, 0.024, 0.240)),
        origin=Origin(xyz=(0.138, -0.250, 0.120)),
        material=charcoal,
        name="rear_left_inner_plate",
    )
    main_frame.visual(
        Box((0.008, 0.024, 0.240)),
        origin=Origin(xyz=(0.202, -0.250, 0.120)),
        material=charcoal,
        name="rear_left_outer_plate",
    )
    main_frame.visual(
        Box((0.072, 0.024, 0.020)),
        origin=Origin(xyz=(0.170, -0.250, 0.230)),
        material=charcoal,
        name="rear_left_hanger_bridge",
    )
    main_frame.visual(
        Box((0.008, 0.024, 0.240)),
        origin=Origin(xyz=(-0.138, -0.250, 0.120)),
        material=charcoal,
        name="rear_right_inner_plate",
    )
    main_frame.visual(
        Box((0.008, 0.024, 0.240)),
        origin=Origin(xyz=(-0.202, -0.250, 0.120)),
        material=charcoal,
        name="rear_right_outer_plate",
    )
    main_frame.visual(
        Box((0.072, 0.024, 0.020)),
        origin=Origin(xyz=(-0.170, -0.250, 0.230)),
        material=charcoal,
        name="rear_right_hanger_bridge",
    )

    steering_fork = model.part("steering_fork")
    steering_fork.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.56)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.10, 0.08)),
    )
    steering_fork.visual(
        Box((0.030, 0.040, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=charcoal,
        name="steering_shaft",
    )
    steering_fork.visual(
        Box((0.042, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=charcoal,
        name="lower_bearing_collar",
    )
    steering_fork.visual(
        _rect_tube_mesh(
            "steering_sleeve_shell",
            outer_x=0.060,
            outer_y=0.070,
            wall=0.006,
            height=0.180,
            radius=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=charcoal,
        name="steering_sleeve_shell",
    )
    steering_fork.visual(
        Box((0.086, 0.076, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=charcoal,
        name="stem_clamp_block",
    )
    steering_fork.visual(
        Box((0.180, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, 0.078, -0.036)),
        material=frame_blue,
        name="fork_crown",
    )
    steering_fork.visual(
        Box((0.020, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.045, -0.019)),
        material=frame_blue,
        name="crown_neck",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.240)),
        origin=Origin(xyz=(0.157, 0.090, -0.080)),
        material=frame_blue,
        name="left_outer_fork_arm",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.240)),
        origin=Origin(xyz=(0.093, 0.090, -0.080)),
        material=frame_blue,
        name="left_inner_fork_arm",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.240)),
        origin=Origin(xyz=(-0.157, 0.090, -0.080)),
        material=frame_blue,
        name="right_outer_fork_arm",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.240)),
        origin=Origin(xyz=(-0.093, 0.090, -0.080)),
        material=frame_blue,
        name="right_inner_fork_arm",
    )
    steering_fork.visual(
        Box((0.072, 0.026, 0.020)),
        origin=Origin(xyz=(0.125, 0.082, -0.022)),
        material=frame_blue,
        name="left_fork_bridge",
    )
    steering_fork.visual(
        Box((0.072, 0.026, 0.020)),
        origin=Origin(xyz=(-0.125, 0.082, -0.022)),
        material=frame_blue,
        name="right_fork_bridge",
    )
    steering_fork.visual(
        Cylinder(radius=0.010, length=0.300),
        origin=Origin(xyz=(0.0, 0.103, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_blue,
        name="fork_tie_bar",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.120)),
        origin=Origin(xyz=(0.093, 0.090, -0.140)),
        material=charcoal,
        name="front_left_inner_plate",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.120)),
        origin=Origin(xyz=(0.157, 0.090, -0.140)),
        material=charcoal,
        name="front_left_outer_plate",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.120)),
        origin=Origin(xyz=(-0.093, 0.090, -0.140)),
        material=charcoal,
        name="front_right_inner_plate",
    )
    steering_fork.visual(
        Box((0.008, 0.026, 0.120)),
        origin=Origin(xyz=(-0.157, 0.090, -0.140)),
        material=charcoal,
        name="front_right_outer_plate",
    )

    handle_stem = model.part("handle_stem")
    handle_stem.inertial = Inertial.from_geometry(
        Box((0.52, 0.12, 0.42)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.01, 0.22)),
    )
    handle_stem.visual(
        Box((0.028, 0.038, 0.260)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=charcoal,
        name="stem_post",
    )
    handle_stem.visual(
        Box((0.052, 0.062, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        material=charcoal,
        name="stem_stop_collar",
    )
    handle_stem.visual(
        Box((0.050, 0.050, 0.078)),
        origin=Origin(xyz=(0.0, -0.010, 0.265)),
        material=charcoal,
        name="handlebar_clamp",
    )
    handle_stem.visual(
        _tube_mesh(
            "handlebar_tube",
            [
                (-0.235, -0.030, 0.300),
                (-0.170, -0.015, 0.308),
                (-0.080, 0.000, 0.314),
                (0.080, 0.000, 0.314),
                (0.170, -0.015, 0.308),
                (0.235, -0.030, 0.300),
            ],
            0.012,
        ),
        material=charcoal,
        name="handlebar_tube",
    )
    handle_stem.visual(
        Cylinder(radius=0.015, length=0.100),
        origin=Origin(xyz=(0.245, -0.030, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    handle_stem.visual(
        Cylinder(radius=0.015, length=0.100),
        origin=Origin(xyz=(-0.245, -0.030, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )

    knee_post = model.part("knee_post")
    knee_post.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.34)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.020, 0.220)),
    )
    knee_post.visual(
        Box((0.034, 0.044, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=charcoal,
        name="guide_post",
    )
    knee_post.visual(
        Box((0.048, 0.058, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=charcoal,
        name="height_stop_collar",
    )
    knee_post.visual(
        Box((0.084, 0.110, 0.070)),
        origin=Origin(xyz=(0.0, -0.020, 0.215)),
        material=charcoal,
        name="pad_support_head",
    )
    knee_post.visual(
        Box((0.110, 0.150, 0.024)),
        origin=Origin(xyz=(0.0, -0.020, 0.260)),
        material=satin_black,
        name="pad_base_deck",
    )
    knee_post.visual(
        _rounded_pad_mesh("knee_tray", width=0.170, depth=0.190, thickness=0.014, radius=0.018),
        origin=Origin(xyz=(0.0, -0.020, 0.274)),
        material=satin_black,
        name="knee_tray",
    )
    knee_post.visual(
        _rounded_pad_mesh("left_knee_cushion", width=0.072, depth=0.150, thickness=0.036, radius=0.016),
        origin=Origin(xyz=(-0.040, -0.020, 0.294)),
        material=pad_black,
        name="left_knee_cushion",
    )
    knee_post.visual(
        _rounded_pad_mesh("right_knee_cushion", width=0.072, depth=0.150, thickness=0.036, radius=0.016),
        origin=Origin(xyz=(0.040, -0.020, 0.294)),
        material=pad_black,
        name="right_knee_cushion",
    )

    for wheel_name in ("rear_left_wheel", "rear_right_wheel", "front_left_wheel", "front_right_wheel"):
        wheel_part = model.part(wheel_name)
        wheel_part.inertial = Inertial.from_geometry(
            Cylinder(radius=0.100, length=0.040),
            mass=0.9,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        _add_small_wheel(
            wheel_part,
            mesh_prefix=wheel_name,
            tire_radius=0.100,
            tire_width=0.040,
            rubber=rubber,
            rim_metal=rim_metal,
            hub_metal=hub_metal,
        )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=steering_fork,
        origin=Origin(xyz=(0.0, 0.240, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "handle_stem_height",
        ArticulationType.PRISMATIC,
        parent=steering_fork,
        child=handle_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.18,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "knee_post_height",
        ArticulationType.PRISMATIC,
        parent=main_frame,
        child=knee_post,
        origin=Origin(xyz=(0.0, -0.030, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.12,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child="rear_left_wheel",
        origin=Origin(xyz=(0.170, -0.250, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=22.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child="rear_right_wheel",
        origin=Origin(xyz=(-0.170, -0.250, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=22.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child="front_left_wheel",
        origin=Origin(xyz=(0.125, 0.090, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=22.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child="front_right_wheel",
        origin=Origin(xyz=(-0.125, 0.090, -0.150)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    steering_fork = object_model.get_part("steering_fork")
    handle_stem = object_model.get_part("handle_stem")
    knee_post = object_model.get_part("knee_post")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    steering_yaw = object_model.get_articulation("steering_yaw")
    handle_stem_height = object_model.get_articulation("handle_stem_height")
    knee_post_height = object_model.get_articulation("knee_post_height")
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
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        main_frame,
        steering_fork,
        elem_a="head_bearing_collar",
        elem_b="crown_neck",
        reason="The fork crown neck intentionally nests into the lower headset bearing collar around the steering axis.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(steering_fork, main_frame, elem_a="lower_bearing_collar", elem_b="head_bearing_collar")
    ctx.expect_contact(handle_stem, steering_fork, elem_a="stem_stop_collar", elem_b="steering_sleeve_shell")
    ctx.expect_contact(knee_post, main_frame, elem_a="height_stop_collar", elem_b="knee_sleeve_shell")
    ctx.expect_contact(rear_left_wheel, main_frame)
    ctx.expect_contact(rear_right_wheel, main_frame)
    ctx.expect_contact(front_left_wheel, steering_fork)
    ctx.expect_contact(front_right_wheel, steering_fork)

    ctx.expect_origin_gap(front_left_wheel, rear_left_wheel, axis="y", min_gap=0.50)
    ctx.expect_origin_distance(rear_left_wheel, rear_right_wheel, axes="x", min_dist=0.30, max_dist=0.40)
    ctx.expect_origin_distance(front_left_wheel, front_right_wheel, axes="x", min_dist=0.20, max_dist=0.30)

    ctx.check(
        "steering_axis_is_vertical",
        tuple(steering_yaw.axis) == (0.0, 0.0, 1.0),
        f"steering axis was {steering_yaw.axis}",
    )
    ctx.check(
        "knee_post_axis_is_vertical",
        tuple(knee_post_height.axis) == (0.0, 0.0, 1.0),
        f"knee post axis was {knee_post_height.axis}",
    )
    ctx.check(
        "handle_stem_axis_is_vertical",
        tuple(handle_stem_height.axis) == (0.0, 0.0, 1.0),
        f"handle stem axis was {handle_stem_height.axis}",
    )
    for joint_name, joint_obj in (
        ("rear_left_wheel_spin_axis", rear_left_wheel_spin),
        ("rear_right_wheel_spin_axis", rear_right_wheel_spin),
        ("front_left_wheel_spin_axis", front_left_wheel_spin),
        ("front_right_wheel_spin_axis", front_right_wheel_spin),
    ):
        ctx.check(joint_name, tuple(joint_obj.axis) == (1.0, 0.0, 0.0), f"{joint_name} was {joint_obj.axis}")

    knee_rest = ctx.part_world_position(knee_post)
    handle_rest = ctx.part_world_position(handle_stem)
    front_left_rest = ctx.part_world_position(front_left_wheel)
    assert knee_rest is not None
    assert handle_rest is not None
    assert front_left_rest is not None

    with ctx.pose({knee_post_height: 0.10}):
        knee_raised = ctx.part_world_position(knee_post)
        assert knee_raised is not None
        ctx.check(
            "knee_post_raises_in_z",
            knee_raised[2] > knee_rest[2] + 0.095,
            f"knee post z changed from {knee_rest[2]:.4f} to {knee_raised[2]:.4f}",
        )
        ctx.expect_within(
            knee_post,
            main_frame,
            axes="xy",
            margin=0.002,
            inner_elem="guide_post",
            outer_elem="knee_sleeve_shell",
        )
        ctx.expect_overlap(
            knee_post,
            main_frame,
            axes="z",
            min_overlap=0.055,
            elem_a="guide_post",
            elem_b="knee_sleeve_shell",
        )

    with ctx.pose({handle_stem_height: 0.10}):
        handle_high = ctx.part_world_position(handle_stem)
        assert handle_high is not None
        ctx.check(
            "handle_stem_raises_in_z",
            handle_high[2] > handle_rest[2] + 0.095,
            f"handle stem z changed from {handle_rest[2]:.4f} to {handle_high[2]:.4f}",
        )
        ctx.expect_within(
            handle_stem,
            steering_fork,
            axes="xy",
            margin=0.002,
            inner_elem="stem_post",
            outer_elem="steering_sleeve_shell",
        )
        ctx.expect_overlap(
            handle_stem,
            steering_fork,
            axes="z",
            min_overlap=0.075,
            elem_a="stem_post",
            elem_b="steering_sleeve_shell",
        )

    with ctx.pose({steering_yaw: 0.55}):
        front_left_turned = ctx.part_world_position(front_left_wheel)
        assert front_left_turned is not None
        ctx.check(
            "front_fork_yaws",
            abs(front_left_turned[0] - front_left_rest[0]) > 0.03
            and abs(front_left_turned[1] - front_left_rest[1]) > 0.03,
            f"front wheel moved from {front_left_rest} to {front_left_turned}",
        )
        ctx.expect_contact(front_left_wheel, steering_fork)
        ctx.expect_contact(front_right_wheel, steering_fork)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
