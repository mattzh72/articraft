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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _ring_profile(major_radius: float, tube_radius: float, *, segments: int = 24) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    for index in range(segments + 1):
        angle = 2.0 * math.pi * index / segments
        profile.append(
            (
                major_radius + tube_radius * math.cos(angle),
                tube_radius * math.sin(angle),
            )
        )
    return profile


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_segment(part, start, end, *, radius: float, material, name: str) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_rear_wheel(part, mesh_prefix: str, *, side_sign: float, tire_material, rim_material) -> None:
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(_ring_profile(0.289, 0.016, segments=28), segments=64).rotate_y(math.pi / 2.0),
    )
    rim_mesh = _save_mesh(
        f"{mesh_prefix}_rim",
        LatheGeometry(_ring_profile(0.270, 0.006, segments=20), segments=56).rotate_y(math.pi / 2.0),
    )
    pushrim_mesh = _save_mesh(
        f"{mesh_prefix}_pushrim",
        LatheGeometry(_ring_profile(0.282, 0.005, segments=20), segments=56).rotate_y(math.pi / 2.0),
    )
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(rim_mesh, material=rim_material, name="rim")
    part.visual(
        pushrim_mesh,
        origin=Origin(xyz=(0.036 * side_sign, 0.0, 0.0)),
        material=rim_material,
        name="pushrim",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim_material,
        name="hub_shell",
    )
    for flange_sign in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=0.040, length=0.008),
            origin=Origin(
                xyz=(0.010 * flange_sign, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rim_material,
            name=f"hub_flange_{'left' if flange_sign < 0.0 else 'right'}",
        )

    spoke_radius = 0.0016
    for spoke_index in range(8):
        angle = 2.0 * math.pi * spoke_index / 8.0
        rim_angle = angle + 0.22
        for flange_sign in (-1.0, 1.0):
            start = (
                0.010 * flange_sign,
                0.040 * math.cos(angle),
                0.040 * math.sin(angle),
            )
            end = (
                0.0,
                0.264 * math.cos(rim_angle),
                0.264 * math.sin(rim_angle),
            )
            _add_segment(
                part,
                start,
                end,
                radius=spoke_radius,
                material=rim_material,
                name=f"spoke_{spoke_index}_{'in' if flange_sign < 0.0 else 'out'}",
            )

    for bracket_index, angle in enumerate((0.35, 2.55, 4.65)):
        start = (
            0.006 * side_sign,
            0.266 * math.cos(angle),
            0.266 * math.sin(angle),
        )
        end = (
            0.036 * side_sign,
            0.282 * math.cos(angle),
            0.282 * math.sin(angle),
        )
        _add_segment(
            part,
            start,
            end,
            radius=0.0035,
            material=rim_material,
            name=f"pushrim_bracket_{bracket_index}",
        )


def _build_caster_fork(part, *, material) -> None:
    part.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=material,
        name="swivel_head",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=material,
        name="stem",
    )
    part.visual(
        Box((0.050, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.013, -0.045)),
        material=material,
        name="fork_bridge",
    )
    part.visual(
        Box((0.012, 0.078, 0.070)),
        origin=Origin(xyz=(0.020, -0.039, -0.090)),
        material=material,
        name="left_fork_leg",
    )
    part.visual(
        Box((0.012, 0.078, 0.070)),
        origin=Origin(xyz=(-0.020, -0.039, -0.090)),
        material=material,
        name="right_fork_leg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_metal = model.material("frame_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.24, 0.25, 0.27, 1.0))
    upholstery = model.material("upholstery", rgba=(0.13, 0.14, 0.16, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.84, 0.85, 0.87, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 1.02, 1.00)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.06, 0.50)),
    )

    side_path = [
        (0.205, -0.18, 0.15),
        (0.205, -0.13, 0.33),
        (0.205, -0.22, 0.93),
        (0.205, 0.18, 0.54),
        (0.205, 0.40, 0.16),
    ]
    lower_side_path = [
        (0.185, -0.13, 0.33),
        (0.185, 0.06, 0.20),
        (0.185, 0.34, 0.15),
    ]
    frame.visual(
        _save_mesh(
            "wheelchair_left_side_frame",
            tube_from_spline_points(side_path, radius=0.012, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_metal,
        name="left_side_frame",
    )
    frame.visual(
        _save_mesh(
            "wheelchair_right_side_frame",
            tube_from_spline_points(_mirror_x(side_path), radius=0.012, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_metal,
        name="right_side_frame",
    )
    frame.visual(
        _save_mesh(
            "wheelchair_left_lower_frame",
            tube_from_spline_points(lower_side_path, radius=0.010, samples_per_segment=12, radial_segments=16),
        ),
        material=dark_frame,
        name="left_lower_frame",
    )
    frame.visual(
        _save_mesh(
            "wheelchair_right_lower_frame",
            tube_from_spline_points(_mirror_x(lower_side_path), radius=0.010, samples_per_segment=12, radial_segments=16),
        ),
        material=dark_frame,
        name="right_lower_frame",
    )

    frame.visual(
        Cylinder(radius=0.014, length=0.46),
        origin=Origin(xyz=(0.0, -0.13, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_frame,
        name="axle_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.40),
        origin=Origin(xyz=(0.0, 0.395, 0.15), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_metal,
        name="front_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.38),
        origin=Origin(xyz=(0.0, -0.19, 0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_metal,
        name="backrest_cross_tube",
    )

    frame.visual(
        Box((0.43, 0.40, 0.018)),
        origin=Origin(xyz=(0.0, -0.01, 0.50)),
        material=upholstery,
        name="seat_sling",
    )
    frame.visual(
        Box((0.40, 0.020, 0.30)),
        origin=Origin(xyz=(0.0, -0.19, 0.72)),
        material=upholstery,
        name="backrest_sling",
    )
    frame.visual(
        Box((0.07, 0.26, 0.04)),
        origin=Origin(xyz=(0.235, -0.01, 0.62)),
        material=dark_frame,
        name="left_armrest_pad",
    )
    frame.visual(
        Box((0.07, 0.26, 0.04)),
        origin=Origin(xyz=(-0.235, -0.01, 0.62)),
        material=dark_frame,
        name="right_armrest_pad",
    )

    footrest_hanger = [
        (0.185, 0.34, 0.15),
        (0.125, 0.39, 0.12),
        (0.105, 0.455, 0.065),
    ]
    frame.visual(
        _save_mesh(
            "wheelchair_left_footrest_hanger",
            tube_from_spline_points(footrest_hanger, radius=0.009, samples_per_segment=10, radial_segments=16),
        ),
        material=frame_metal,
        name="left_footrest_hanger",
    )
    frame.visual(
        _save_mesh(
            "wheelchair_right_footrest_hanger",
            tube_from_spline_points(_mirror_x(footrest_hanger), radius=0.009, samples_per_segment=10, radial_segments=16),
        ),
        material=frame_metal,
        name="right_footrest_hanger",
    )
    frame.visual(
        Box((0.14, 0.08, 0.014)),
        origin=Origin(xyz=(0.085, 0.475, 0.058), rpy=(0.18, 0.0, 0.0)),
        material=dark_frame,
        name="left_footplate",
    )
    frame.visual(
        Box((0.14, 0.08, 0.014)),
        origin=Origin(xyz=(-0.085, 0.475, 0.058), rpy=(0.18, 0.0, 0.0)),
        material=dark_frame,
        name="right_footplate",
    )

    frame.visual(
        Box((0.016, 0.018, 0.040)),
        origin=Origin(xyz=(0.208, 0.400, 0.173)),
        material=dark_frame,
        name="left_caster_mount",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.225, 0.40, 0.198)),
        material=dark_frame,
        name="left_caster_headset",
    )
    frame.visual(
        Box((0.016, 0.018, 0.040)),
        origin=Origin(xyz=(-0.208, 0.400, 0.173)),
        material=dark_frame,
        name="right_caster_mount",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(-0.225, 0.40, 0.198)),
        material=dark_frame,
        name="right_caster_headset",
    )

    frame.visual(
        Cylinder(radius=0.011, length=0.108),
        origin=Origin(xyz=(0.260, -0.13, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_frame,
        name="left_axle_mount",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.108),
        origin=Origin(xyz=(-0.260, -0.13, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_frame,
        name="right_axle_mount",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.025),
        mass=2.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_rear_wheel(
        rear_left_wheel,
        "rear_left_wheel",
        side_sign=1.0,
        tire_material=tire_rubber,
        rim_material=wheel_metal,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.025),
        mass=2.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_rear_wheel(
        rear_right_wheel,
        "rear_right_wheel",
        side_sign=-1.0,
        tire_material=tire_rubber,
        rim_material=wheel_metal,
    )

    for side_name in ("left", "right"):
        caster = model.part(f"front_{side_name}_caster")
        caster.inertial = Inertial.from_geometry(
            Box((0.05, 0.05, 0.14)),
            mass=0.35,
            origin=Origin(xyz=(0.0, -0.01, -0.07)),
        )
        _build_caster_fork(caster, material=dark_frame)

        caster_wheel = model.part(f"front_{side_name}_caster_wheel")
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.062, length=0.028),
            mass=0.45,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        caster_wheel.visual(
            Cylinder(radius=0.062, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tire_rubber,
            name="caster_tire",
        )
        caster_wheel.visual(
            Cylinder(radius=0.030, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_metal,
            name="caster_hub",
        )

        caster_x = 0.225 if side_name == "left" else -0.225
        model.articulation(
            f"frame_to_front_{side_name}_caster_swivel",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(caster_x, 0.40, 0.188)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        )
        model.articulation(
            f"front_{side_name}_caster_to_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, -0.074, -0.118)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    model.articulation(
        "frame_to_rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.335, -0.13, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.335, -0.13, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
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

    frame = object_model.get_part("frame")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    front_left_caster_wheel = object_model.get_part("front_left_caster_wheel")
    front_right_caster_wheel = object_model.get_part("front_right_caster_wheel")

    rear_left_spin = object_model.get_articulation("frame_to_rear_left_wheel_spin")
    front_left_swivel = object_model.get_articulation("frame_to_front_left_caster_swivel")
    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel_spin")

    ctx.expect_origin_gap(
        rear_left_wheel,
        frame,
        axis="x",
        min_gap=0.33,
        max_gap=0.34,
        name="left rear wheel sits well outboard for a wide stance",
    )
    ctx.expect_origin_gap(
        frame,
        rear_right_wheel,
        axis="x",
        min_gap=0.33,
        max_gap=0.34,
        name="right rear wheel sits well outboard for a wide stance",
    )

    ctx.expect_gap(
        front_left_caster,
        front_left_caster_wheel,
        axis="z",
        positive_elem="fork_bridge",
        negative_elem="caster_tire",
        min_gap=0.0005,
        max_gap=0.015,
        name="left caster wheel clears the fork bridge",
    )
    ctx.expect_gap(
        front_right_caster,
        front_right_caster_wheel,
        axis="z",
        positive_elem="fork_bridge",
        negative_elem="caster_tire",
        min_gap=0.0005,
        max_gap=0.015,
        name="right caster wheel clears the fork bridge",
    )
    ctx.expect_within(
        front_left_caster_wheel,
        front_left_caster,
        axes="x",
        margin=0.0,
        inner_elem="caster_tire",
        outer_elem="fork_bridge",
        name="left caster wheel stays centered under the fork",
    )
    ctx.expect_within(
        front_right_caster_wheel,
        front_right_caster,
        axes="x",
        margin=0.0,
        inner_elem="caster_tire",
        outer_elem="fork_bridge",
        name="right caster wheel stays centered under the fork",
    )

    rear_left_rest = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({rear_left_spin: 1.3}):
        rear_left_spun = ctx.part_world_position(rear_left_wheel)
    ctx.check(
        "left rear wheel spins about its axle without translating",
        rear_left_rest is not None
        and rear_left_spun is not None
        and max(abs(a - b) for a, b in zip(rear_left_rest, rear_left_spun)) < 1e-6,
        details=f"rest={rear_left_rest}, spun={rear_left_spun}",
    )

    caster_wheel_rest = ctx.part_world_position(front_left_caster_wheel)
    with ctx.pose({front_left_spin: 1.4}):
        caster_wheel_spun = ctx.part_world_position(front_left_caster_wheel)
    ctx.check(
        "left front caster wheel spins about its axle without translating",
        caster_wheel_rest is not None
        and caster_wheel_spun is not None
        and max(abs(a - b) for a, b in zip(caster_wheel_rest, caster_wheel_spun)) < 1e-6,
        details=f"rest={caster_wheel_rest}, spun={caster_wheel_spun}",
    )

    caster_rest = ctx.part_world_position(front_left_caster_wheel)
    with ctx.pose({front_left_swivel: 0.9}):
        caster_swiveled = ctx.part_world_position(front_left_caster_wheel)
    ctx.check(
        "left caster swivel rotates the wheel around a vertical stem",
        caster_rest is not None
        and caster_swiveled is not None
        and caster_swiveled[0] > caster_rest[0] + 0.04
        and abs(caster_swiveled[2] - caster_rest[2]) < 1e-6,
        details=f"rest={caster_rest}, swiveled={caster_swiveled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
