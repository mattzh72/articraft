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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _pipe_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    half = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def _square_tube_mesh(
    name: str,
    *,
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    length: float,
):
    geom = ExtrudeWithHolesGeometry(
        _rect_profile(outer_x, outer_y),
        [_rect_profile(inner_x, inner_y)],
        height=length,
        center=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def _wheel_mesh(
    name: str,
    *,
    radius: float,
    width: float,
    hub_radius: float,
    bore_radius: float,
):
    half = width * 0.5
    bore_half = half * 0.66
    outer_profile = [
        (hub_radius, -bore_half),
        (hub_radius * 1.06, -half * 0.38),
        (radius * 0.78, -half * 0.46),
        (radius * 0.96, -half * 0.18),
        (radius, 0.0),
        (radius * 0.96, half * 0.18),
        (radius * 0.78, half * 0.46),
        (hub_radius * 1.06, half * 0.38),
        (hub_radius, bore_half),
    ]
    inner_profile = [(bore_radius, -bore_half), (bore_radius, bore_half)]
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(math.pi * 0.5)
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_knee_walker", assets=ASSETS)

    frame = model.material("frame", rgba=(0.23, 0.24, 0.26, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    accent = model.material("accent", rgba=(0.65, 0.17, 0.15, 1.0))

    rear_wheel_shell = _wheel_mesh(
        "rear_wheel_shell.obj",
        radius=0.115,
        width=0.045,
        hub_radius=0.030,
        bore_radius=0.012,
    )
    front_wheel_shell = _wheel_mesh(
        "front_wheel_shell.obj",
        radius=0.085,
        width=0.040,
        hub_radius=0.024,
        bore_radius=0.008,
    )
    head_tube_shell = _pipe_mesh(
        "head_tube_shell.obj",
        outer_radius=0.028,
        inner_radius=0.020,
        length=0.140,
    )
    pad_sleeve_shell = _square_tube_mesh(
        "pad_sleeve_shell.obj",
        outer_x=0.052,
        outer_y=0.036,
        inner_x=0.038,
        inner_y=0.022,
        length=0.160,
    )

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.62, 0.38, 0.42)),
        mass=7.4,
        origin=Origin(xyz=(0.02, 0.0, 0.21)),
    )
    chassis.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(-0.150, 0.0, 0.115), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_steel,
        name="rear_axle",
    )
    _add_tube(
        chassis,
        (-0.150, 0.090, 0.115),
        (-0.080, 0.075, 0.155),
        radius=0.011,
        material=frame,
        name="left_rear_upright",
    )
    _add_tube(
        chassis,
        (-0.150, -0.090, 0.115),
        (-0.080, -0.075, 0.155),
        radius=0.011,
        material=frame,
        name="right_rear_upright",
    )
    _add_tube(
        chassis,
        (-0.080, -0.075, 0.155),
        (-0.080, 0.075, 0.155),
        radius=0.010,
        material=frame,
        name="rear_crossbrace",
    )
    _add_tube(
        chassis,
        (-0.080, 0.075, 0.155),
        (0.060, 0.070, 0.160),
        radius=0.012,
        material=frame,
        name="left_lower_rail",
    )
    _add_tube(
        chassis,
        (-0.080, -0.075, 0.155),
        (0.060, -0.070, 0.160),
        radius=0.012,
        material=frame,
        name="right_lower_rail",
    )
    _add_tube(
        chassis,
        (0.060, -0.070, 0.160),
        (0.060, 0.070, 0.160),
        radius=0.010,
        material=frame,
        name="front_crossbrace",
    )
    _add_tube(
        chassis,
        (-0.080, 0.050, 0.155),
        (-0.020, 0.018, 0.190),
        radius=0.008,
        material=frame,
        name="left_sleeve_brace",
    )
    _add_tube(
        chassis,
        (-0.080, -0.050, 0.155),
        (-0.020, -0.018, 0.190),
        radius=0.008,
        material=frame,
        name="right_sleeve_brace",
    )
    chassis.visual(
        pad_sleeve_shell,
        origin=Origin(xyz=(-0.020, 0.0, 0.270)),
        material=satin_steel,
        name="pad_sleeve",
    )
    _add_tube(
        chassis,
        (0.060, 0.070, 0.160),
        (0.240, 0.028, 0.130),
        radius=0.009,
        material=frame,
        name="left_head_brace",
    )
    _add_tube(
        chassis,
        (0.060, -0.070, 0.160),
        (0.240, -0.028, 0.130),
        radius=0.009,
        material=frame,
        name="right_head_brace",
    )
    chassis.visual(
        head_tube_shell,
        origin=Origin(xyz=(0.240, 0.0, 0.180)),
        material=satin_steel,
        name="head_tube",
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.inertial = Inertial.from_geometry(
        Box((0.30, 0.40, 0.92)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )
    steering_assembly.visual(
        Cylinder(radius=0.020, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="steering_stem",
    )
    steering_assembly.visual(
        Box((0.050, 0.085, 0.016)),
        origin=Origin(xyz=(0.060, 0.0, -0.090)),
        material=frame,
        name="fork_crown",
    )
    _add_tube(
        steering_assembly,
        (0.020, 0.000, -0.082),
        (0.090, 0.000, -0.095),
        radius=0.008,
        material=frame,
        name="fork_spine",
    )
    steering_assembly.visual(
        Cylinder(radius=0.008, length=0.218),
        origin=Origin(xyz=(0.090, 0.0, -0.095), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=satin_steel,
        name="front_axle",
    )
    _add_tube(
        steering_assembly,
        (0.055, 0.040, -0.084),
        (0.090, 0.075, -0.095),
        radius=0.009,
        material=frame,
        name="left_fork_leg",
    )
    _add_tube(
        steering_assembly,
        (0.055, -0.040, -0.084),
        (0.090, -0.075, -0.095),
        radius=0.009,
        material=frame,
        name="right_fork_leg",
    )
    _add_tube(
        steering_assembly,
        (0.000, 0.000, 0.070),
        (-0.080, 0.000, 0.680),
        radius=0.015,
        material=frame,
        name="upper_column",
    )
    _add_tube(
        steering_assembly,
        (-0.080, -0.150, 0.680),
        (-0.080, 0.150, 0.680),
        radius=0.014,
        material=frame,
        name="handlebar",
    )
    _add_tube(
        steering_assembly,
        (-0.080, 0.150, 0.680),
        (-0.080, 0.190, 0.680),
        radius=0.017,
        material=grip_black,
        name="left_grip",
    )
    _add_tube(
        steering_assembly,
        (-0.080, -0.150, 0.680),
        (-0.080, -0.190, 0.680),
        radius=0.017,
        material=grip_black,
        name="right_grip",
    )

    knee_platform = model.part("knee_platform")
    knee_platform.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.32)),
        mass=1.5,
        origin=Origin(xyz=(0.06, 0.0, 0.08)),
    )
    knee_platform.visual(
        Box((0.038, 0.022, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_steel,
        name="guide_post",
    )
    _add_tube(
        knee_platform,
        (0.000, 0.011, 0.090),
        (0.080, 0.045, 0.145),
        radius=0.008,
        material=frame,
        name="left_pad_support",
    )
    _add_tube(
        knee_platform,
        (0.000, -0.011, 0.090),
        (0.080, -0.045, 0.145),
        radius=0.008,
        material=frame,
        name="right_pad_support",
    )
    _add_tube(
        knee_platform,
        (0.080, -0.045, 0.145),
        (0.080, 0.045, 0.145),
        radius=0.007,
        material=frame,
        name="front_pad_support",
    )
    knee_platform.visual(
        Box((0.240, 0.160, 0.018)),
        origin=Origin(xyz=(0.070, 0.0, 0.145)),
        material=frame,
        name="pad_base",
    )
    knee_platform.visual(
        Box((0.220, 0.140, 0.050)),
        origin=Origin(xyz=(0.070, 0.0, 0.178)),
        material=pad_vinyl,
        name="pad_cushion",
    )
    knee_platform.visual(
        Box((0.030, 0.120, 0.040)),
        origin=Origin(xyz=(0.175, 0.0, 0.166)),
        material=accent,
        name="front_lip",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.045),
        mass=0.8,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    rear_left_wheel.visual(rear_wheel_shell, material=rubber, name="wheel_shell")

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.045),
        mass=0.8,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    rear_right_wheel.visual(rear_wheel_shell, material=rubber, name="wheel_shell")

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.040),
        mass=0.5,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    front_left_wheel.visual(front_wheel_shell, material=rubber, name="wheel_shell")

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.040),
        mass=0.5,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    front_right_wheel.visual(front_wheel_shell, material=rubber, name="wheel_shell")

    model.articulation(
        "chassis_to_steering",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=steering_assembly,
        origin=Origin(xyz=(0.240, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=-math.radians(40.0),
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "chassis_to_knee_platform",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=knee_platform,
        origin=Origin(xyz=(-0.020, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.150, 0.160, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.150, -0.160, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_left_wheel,
        origin=Origin(xyz=(0.090, 0.095, -0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_right_wheel,
        origin=Origin(xyz=(0.090, -0.095, -0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    steering_assembly = object_model.get_part("steering_assembly")
    knee_platform = object_model.get_part("knee_platform")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")

    steering_joint = object_model.get_articulation("chassis_to_steering")
    knee_height_joint = object_model.get_articulation("chassis_to_knee_platform")
    rear_left_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_spin = object_model.get_articulation("rear_right_wheel_spin")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_spin = object_model.get_articulation("front_right_wheel_spin")

    head_tube = chassis.get_visual("head_tube")
    rear_axle = chassis.get_visual("rear_axle")
    pad_sleeve = chassis.get_visual("pad_sleeve")
    rear_crossbrace = chassis.get_visual("rear_crossbrace")
    steering_stem = steering_assembly.get_visual("steering_stem")
    front_axle = steering_assembly.get_visual("front_axle")
    guide_post = knee_platform.get_visual("guide_post")
    pad_base = knee_platform.get_visual("pad_base")
    rear_left_shell = rear_left_wheel.get_visual("wheel_shell")
    rear_right_shell = rear_right_wheel.get_visual("wheel_shell")
    front_left_shell = front_left_wheel.get_visual("wheel_shell")
    front_right_shell = front_right_wheel.get_visual("wheel_shell")

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
    ctx.allow_overlap(
        steering_assembly,
        chassis,
        elem_a=steering_stem,
        elem_b=head_tube,
        reason="The steering stem runs concentrically inside the head tube as a bearing sleeve.",
    )
    ctx.allow_overlap(
        knee_platform,
        chassis,
        elem_a=guide_post,
        elem_b=pad_sleeve,
        reason="The knee pad mast telescopes inside the fixed sleeve for height adjustment.",
    )
    ctx.allow_overlap(
        rear_left_wheel,
        chassis,
        elem_a=rear_left_shell,
        elem_b=rear_axle,
        reason="The rear wheel hub rotates around the through-axle.",
    )
    ctx.allow_overlap(
        rear_right_wheel,
        chassis,
        elem_a=rear_right_shell,
        elem_b=rear_axle,
        reason="The rear wheel hub rotates around the through-axle.",
    )
    ctx.allow_overlap(
        front_left_wheel,
        steering_assembly,
        elem_a=front_left_shell,
        elem_b=front_axle,
        reason="The front wheel hub rotates around the fork axle.",
    )
    ctx.allow_overlap(
        front_right_wheel,
        steering_assembly,
        elem_a=front_right_shell,
        elem_b=front_axle,
        reason="The front wheel hub rotates around the fork axle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        steering_assembly,
        chassis,
        elem_a=steering_stem,
        elem_b=head_tube,
        name="steering_stem_seated_in_head_tube",
    )
    ctx.expect_within(
        steering_assembly,
        chassis,
        axes="xy",
        inner_elem=steering_stem,
        outer_elem=head_tube,
        margin=0.0005,
        name="steering_stem_centered_in_head_tube",
    )
    ctx.expect_contact(
        knee_platform,
        chassis,
        elem_a=guide_post,
        elem_b=pad_sleeve,
        name="guide_post_runs_in_pad_sleeve",
    )
    ctx.expect_within(
        knee_platform,
        chassis,
        axes="xy",
        inner_elem=guide_post,
        outer_elem=pad_sleeve,
        margin=0.0005,
        name="guide_post_captured_by_pad_sleeve",
    )
    ctx.expect_gap(
        knee_platform,
        chassis,
        axis="z",
        min_gap=0.05,
        positive_elem=pad_base,
        negative_elem=pad_sleeve,
        name="knee_pad_sits_above_height_sleeve",
    )

    ctx.expect_contact(
        rear_left_wheel,
        chassis,
        elem_a=rear_left_shell,
        elem_b=rear_axle,
        name="rear_left_wheel_on_axle",
    )
    ctx.expect_contact(
        rear_right_wheel,
        chassis,
        elem_a=rear_right_shell,
        elem_b=rear_axle,
        name="rear_right_wheel_on_axle",
    )
    ctx.expect_contact(
        front_left_wheel,
        steering_assembly,
        elem_a=front_left_shell,
        elem_b=front_axle,
        name="front_left_wheel_on_fork_axle",
    )
    ctx.expect_contact(
        front_right_wheel,
        steering_assembly,
        elem_a=front_right_shell,
        elem_b=front_axle,
        name="front_right_wheel_on_fork_axle",
    )

    def _axis_tuple(articulation) -> tuple[float, float, float]:
        return tuple(float(value) for value in articulation.axis or (0.0, 0.0, 0.0))

    ctx.check(
        "steering_axis_and_range",
        _axis_tuple(steering_joint) == (0.0, 0.0, 1.0)
        and steering_joint.motion_limits is not None
        and steering_joint.motion_limits.lower <= -math.radians(39.5)
        and steering_joint.motion_limits.upper >= math.radians(39.5),
        details=f"axis={_axis_tuple(steering_joint)} limits={steering_joint.motion_limits}",
    )
    for label, articulation in (
        ("rear_left_spin_is_continuous", rear_left_spin),
        ("rear_right_spin_is_continuous", rear_right_spin),
        ("front_left_spin_is_continuous", front_left_spin),
        ("front_right_spin_is_continuous", front_right_spin),
    ):
        ctx.check(
            label,
            articulation.articulation_type == ArticulationType.CONTINUOUS
            and _axis_tuple(articulation) == (0.0, 1.0, 0.0),
            details=f"type={articulation.articulation_type} axis={_axis_tuple(articulation)}",
        )

    def _bottom_z(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        return None if aabb is None else float(aabb[0][2])

    for label, wheel in (
        ("rear_left_rolls_on_ground", rear_left_wheel),
        ("rear_right_rolls_on_ground", rear_right_wheel),
        ("front_left_rolls_on_ground", front_left_wheel),
        ("front_right_rolls_on_ground", front_right_wheel),
    ):
        bottom = _bottom_z(wheel)
        ctx.check(
            label,
            bottom is not None and abs(bottom) <= 0.003,
            details=f"wheel bottom z={bottom}",
        )

    rest_front_left = ctx.part_world_position(front_left_wheel)
    rest_front_right = ctx.part_world_position(front_right_wheel)
    rest_platform = ctx.part_world_position(knee_platform)

    with ctx.pose({steering_joint: math.radians(35.0)}):
        steer_front_left = ctx.part_world_position(front_left_wheel)
        steer_front_right = ctx.part_world_position(front_right_wheel)
        ctx.expect_contact(front_left_wheel, steering_assembly, elem_a=front_left_shell, elem_b=front_axle)
        ctx.expect_contact(front_right_wheel, steering_assembly, elem_a=front_right_shell, elem_b=front_axle)
        ctx.expect_contact(steering_assembly, chassis, elem_a=steering_stem, elem_b=head_tube)
        ctx.check(
            "steering_swings_fork_pair",
            rest_front_left is not None
            and rest_front_right is not None
            and steer_front_left is not None
            and steer_front_right is not None
            and steer_front_left[1] > rest_front_left[1] + 0.02
            and steer_front_right[1] > rest_front_right[1] + 0.02
            and steer_front_left[0] < rest_front_left[0] - 0.02
            and steer_front_right[0] > rest_front_right[0] + 0.02,
            details=(
                f"rest_left={rest_front_left} steer_left={steer_front_left} "
                f"rest_right={rest_front_right} steer_right={steer_front_right}"
            ),
        )

    with ctx.pose({knee_height_joint: 0.10}):
        raised_platform = ctx.part_world_position(knee_platform)
        ctx.expect_contact(knee_platform, chassis, elem_a=guide_post, elem_b=pad_sleeve)
        ctx.expect_within(
            knee_platform,
            chassis,
            axes="xy",
            inner_elem=guide_post,
            outer_elem=pad_sleeve,
            margin=0.0005,
        )
        ctx.expect_gap(
            knee_platform,
            chassis,
            axis="z",
            min_gap=0.15,
            positive_elem=pad_base,
            negative_elem=rear_crossbrace,
            name="raised_knee_pad_stays_above_frame",
        )
        ctx.check(
            "knee_platform_lifts_vertically",
            rest_platform is not None
            and raised_platform is not None
            and raised_platform[2] > rest_platform[2] + 0.095,
            details=f"rest={rest_platform} raised={raised_platform}",
        )

    all_parts = (
        chassis,
        steering_assembly,
        knee_platform,
        rear_left_wheel,
        rear_right_wheel,
        front_left_wheel,
        front_right_wheel,
    )
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    bounds_ok = True
    for part in all_parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            bounds_ok = False
            break
        for axis in range(3):
            mins[axis] = min(mins[axis], float(aabb[0][axis]))
            maxs[axis] = max(maxs[axis], float(aabb[1][axis]))
    if bounds_ok:
        dims = [maxs[index] - mins[index] for index in range(3)]
        ctx.check(
            "compact_knee_walker_overall_proportions",
            0.55 <= dims[0] <= 0.70 and 0.34 <= dims[1] <= 0.42 and 0.82 <= dims[2] <= 0.96,
            details=f"overall dims={dims}",
        )
    else:
        ctx.fail("compact_knee_walker_overall_proportions", "Could not resolve world bounds for all parts.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
