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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _wheel_visuals(part, *, mesh_prefix: str, tire_radius: float, tire_width: float, rubber, rim_steel, hub_steel) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.54, -half_width),
        (tire_radius * 0.76, -half_width * 0.98),
        (tire_radius * 0.92, -half_width * 0.74),
        (tire_radius * 1.00, -half_width * 0.34),
        (tire_radius, half_width * 0.34),
        (tire_radius * 0.92, half_width * 0.74),
        (tire_radius * 0.76, half_width * 0.98),
        (tire_radius * 0.54, half_width),
        (tire_radius * 0.42, half_width * 0.40),
        (tire_radius * 0.38, 0.0),
        (tire_radius * 0.42, -half_width * 0.40),
        (tire_radius * 0.54, -half_width),
    ]
    tire_mesh = _mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0),
    )
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=tire_radius * 0.72, length=tire_width * 0.70),
        origin=spin_origin,
        material=rim_steel,
        name="rim_disc",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.42, length=tire_width * 0.92),
        origin=spin_origin,
        material=hub_steel,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.18, length=tire_width),
        origin=spin_origin,
        material=rim_steel,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_mover_hand_truck")

    frame_yellow = model.material("frame_yellow", rgba=(0.88, 0.72, 0.10, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    rim_steel = model.material("rim_steel", rgba=(0.74, 0.76, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    clamp_pad = model.material("clamp_pad", rgba=(0.11, 0.12, 0.13, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.76, 0.58, 1.54)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.02, 0.77)),
    )

    left_rail_path = [
        (0.245, -0.060, 0.110),
        (0.248, -0.050, 0.300),
        (0.247, -0.030, 0.820),
        (0.242, -0.060, 1.200),
        (0.225, -0.105, 1.420),
        (0.140, -0.180, 1.490),
    ]
    right_rail_path = _mirror_x(left_rail_path)
    chassis.visual(
        _mesh(
            "drum_mover_left_rail",
            tube_from_spline_points(left_rail_path, radius=0.025, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_yellow,
        name="left_rail",
    )
    chassis.visual(
        _mesh(
            "drum_mover_right_rail",
            tube_from_spline_points(right_rail_path, radius=0.025, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_yellow,
        name="right_rail",
    )

    chassis.visual(
        Cylinder(radius=0.022, length=0.280),
        origin=Origin(xyz=(0.0, -0.180, 1.490), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_crossbar",
    )
    chassis.visual(
        Box((0.450, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.070, 1.220)),
        material=frame_yellow,
        name="upper_frame_crossbar",
    )
    chassis.visual(
        Box((0.480, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, -0.010, 0.860)),
        material=frame_yellow,
        name="mid_frame_crossbar",
    )
    chassis.visual(
        Box((0.500, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.010, 0.340)),
        material=frame_yellow,
        name="lower_frame_crossbar",
    )
    chassis.visual(
        Box((0.180, 0.080, 0.440)),
        origin=Origin(xyz=(0.0, 0.000, 0.620)),
        material=dark_steel,
        name="center_web",
    )
    chassis.visual(
        Box((0.460, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.035, 0.560)),
        material=dark_steel,
        name="lower_drum_pad",
    )
    chassis.visual(
        Box((0.460, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.035, 0.760)),
        material=dark_steel,
        name="upper_drum_pad",
    )
    chassis.visual(
        Box((0.500, 0.220, 0.012)),
        origin=Origin(xyz=(0.0, 0.110, 0.030)),
        material=frame_yellow,
        name="toe_plate",
    )
    chassis.visual(
        Box((0.500, 0.015, 0.050)),
        origin=Origin(xyz=(0.0, 0.217, 0.025)),
        material=frame_yellow,
        name="toe_plate_lip",
    )
    chassis.visual(
        Box((0.015, 0.220, 0.040)),
        origin=Origin(xyz=(0.2425, 0.110, 0.020)),
        material=frame_yellow,
        name="toe_left_flange",
    )
    chassis.visual(
        Box((0.015, 0.220, 0.040)),
        origin=Origin(xyz=(-0.2425, 0.110, 0.020)),
        material=frame_yellow,
        name="toe_right_flange",
    )
    chassis.visual(
        Box((0.520, 0.180, 0.140)),
        origin=Origin(xyz=(0.0, 0.015, 0.100)),
        material=frame_yellow,
        name="lower_gusset",
    )
    chassis.visual(
        Box((0.440, 0.080, 0.080)),
        origin=Origin(xyz=(0.0, -0.100, 0.180)),
        material=dark_steel,
        name="axle_housing",
    )
    chassis.visual(
        Box((0.070, 0.080, 0.200)),
        origin=Origin(xyz=(0.245, -0.070, 0.200)),
        material=dark_steel,
        name="left_axle_bracket",
    )
    chassis.visual(
        Box((0.070, 0.080, 0.200)),
        origin=Origin(xyz=(-0.245, -0.070, 0.200)),
        material=dark_steel,
        name="right_axle_bracket",
    )
    chassis.visual(
        Cylinder(radius=0.016, length=0.042),
        origin=Origin(xyz=(0.301, -0.100, 0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_steel,
        name="left_axle_stub",
    )
    chassis.visual(
        Cylinder(radius=0.016, length=0.042),
        origin=Origin(xyz=(-0.301, -0.100, 0.170), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_steel,
        name="right_axle_stub",
    )
    chassis.visual(
        Box((0.040, 0.062, 0.080)),
        origin=Origin(xyz=(0.190, -0.065, 1.230)),
        material=dark_steel,
        name="left_hinge_bracket",
    )
    chassis.visual(
        Box((0.040, 0.062, 0.080)),
        origin=Origin(xyz=(-0.190, -0.065, 1.230)),
        material=dark_steel,
        name="right_hinge_bracket",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.090),
        mass=5.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        left_wheel,
        mesh_prefix="drum_mover_left_wheel",
        tire_radius=0.170,
        tire_width=0.090,
        rubber=rubber,
        rim_steel=rim_steel,
        hub_steel=dark_steel,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.090),
        mass=5.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        right_wheel,
        mesh_prefix="drum_mover_right_wheel",
        tire_radius=0.170,
        tire_width=0.090,
        rubber=rubber,
        rim_steel=rim_steel,
        hub_steel=dark_steel,
    )

    clamp_arm = model.part("clamp_arm")
    clamp_arm.inertial = Inertial.from_geometry(
        Box((0.460, 0.360, 0.420)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.160, -0.170)),
    )
    clamp_arm.visual(
        Cylinder(radius=0.024, length=0.340),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    left_clamp_arm_path = [
        (0.170, 0.000, 0.000),
        (0.190, 0.090, -0.030),
        (0.195, 0.185, -0.120),
        (0.185, 0.265, -0.240),
        (0.170, 0.305, -0.340),
    ]
    right_clamp_arm_path = _mirror_x(left_clamp_arm_path)
    clamp_arm.visual(
        _mesh(
            "drum_mover_clamp_left_arm",
            tube_from_spline_points(left_clamp_arm_path, radius=0.018, samples_per_segment=14, radial_segments=16),
        ),
        material=dark_steel,
        name="left_clamp_tube",
    )
    clamp_arm.visual(
        _mesh(
            "drum_mover_clamp_right_arm",
            tube_from_spline_points(right_clamp_arm_path, radius=0.018, samples_per_segment=14, radial_segments=16),
        ),
        material=dark_steel,
        name="right_clamp_tube",
    )
    clamp_arm.visual(
        Cylinder(radius=0.018, length=0.340),
        origin=Origin(xyz=(0.0, 0.305, -0.340), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="clamp_crossbar",
    )
    clamp_arm.visual(
        Box((0.220, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.285, -0.310)),
        material=clamp_pad,
        name="clamp_pad_block",
    )
    clamp_arm.visual(
        Box((0.070, 0.045, 0.022)),
        origin=Origin(xyz=(0.130, 0.272, -0.286)),
        material=clamp_pad,
        name="left_pad",
    )
    clamp_arm.visual(
        Box((0.070, 0.045, 0.022)),
        origin=Origin(xyz=(-0.130, 0.272, -0.286)),
        material=clamp_pad,
        name="right_pad",
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=left_wheel,
        origin=Origin(xyz=(0.367, -0.100, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=right_wheel,
        origin=Origin(xyz=(-0.367, -0.100, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=18.0),
    )
    model.articulation(
        "upper_clamp_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=clamp_arm,
        origin=Origin(xyz=(0.0, -0.010, 1.230)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.90, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    chassis = object_model.get_part("chassis")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    clamp_arm = object_model.get_part("clamp_arm")

    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    clamp_hinge = object_model.get_articulation("upper_clamp_hinge")

    ctx.expect_origin_gap(
        left_wheel,
        right_wheel,
        axis="x",
        min_gap=0.68,
        max_gap=0.76,
        name="rear wheels are spaced across the chassis",
    )

    wheel_axes_ok = (
        left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_wheel_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(right_wheel_spin.axis) == (1.0, 0.0, 0.0)
    )
    ctx.check(
        "rear wheels spin about the transverse axle",
        wheel_axes_ok,
        details=f"left_axis={left_wheel_spin.axis}, right_axis={right_wheel_spin.axis}",
    )

    clamp_limits = clamp_hinge.motion_limits
    clamp_joint_ok = (
        clamp_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(clamp_hinge.axis) == (-1.0, 0.0, 0.0)
        and clamp_limits is not None
        and clamp_limits.lower is not None
        and clamp_limits.upper is not None
        and clamp_limits.lower < 0.0 < clamp_limits.upper
    )
    ctx.check(
        "clamp arm uses an upper transverse hinge",
        clamp_joint_ok,
        details=f"axis={clamp_hinge.axis}, limits={clamp_limits}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_crossbar = ctx.part_element_world_aabb(clamp_arm, elem="clamp_crossbar")
    with ctx.pose({clamp_hinge: clamp_limits.lower if clamp_limits is not None else -0.9}):
        open_crossbar = ctx.part_element_world_aabb(clamp_arm, elem="clamp_crossbar")
    with ctx.pose({clamp_hinge: clamp_limits.upper if clamp_limits is not None else 0.45}):
        closed_crossbar = ctx.part_element_world_aabb(clamp_arm, elem="clamp_crossbar")

    rest_center = _aabb_center(rest_crossbar)
    open_center = _aabb_center(open_crossbar)
    closed_center = _aabb_center(closed_crossbar)
    clamp_motion_ok = (
        rest_center is not None
        and open_center is not None
        and closed_center is not None
        and open_center[2] > rest_center[2] + 0.18
        and closed_center[2] < rest_center[2] - 0.05
        and closed_center[1] < rest_center[1] - 0.10
    )
    ctx.check(
        "clamp arm swings up to open and down over the load area",
        clamp_motion_ok,
        details=f"open={open_center}, rest={rest_center}, closed={closed_center}",
    )

    chassis_aabb = ctx.part_world_aabb(chassis)
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    wheels_below_frame = (
        chassis_aabb is not None
        and left_wheel_aabb is not None
        and right_wheel_aabb is not None
        and left_wheel_aabb[1][2] < chassis_aabb[1][2] - 0.90
        and right_wheel_aabb[1][2] < chassis_aabb[1][2] - 0.90
    )
    ctx.check(
        "main wheels sit low on the rear of the tall chassis",
        wheels_below_frame,
        details=f"chassis={chassis_aabb}, left={left_wheel_aabb}, right={right_wheel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
