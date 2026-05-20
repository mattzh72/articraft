from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    TorusGeometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _line_rpy(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    planar = sqrt(dx * dx + dy * dy)
    yaw = atan2(dy, dx)
    pitch = atan2(planar, dz)
    return (0.0, pitch, yaw)


def _add_cylinder_between(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str | None = None,
) -> None:
    mid = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    length = sqrt(
        (end[0] - start[0]) ** 2
        + (end[1] - start[1]) ** 2
        + (end[2] - start[2]) ** 2
    )
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=_line_rpy(start, end)),
        material=material,
        name=name,
    )


def _add_rear_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    side_sign: float,
    tire_material,
    rim_material,
    dark_material,
) -> None:
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(radius=0.293, tube=0.015, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0),
    )
    rim_mesh = _save_mesh(
        f"{mesh_prefix}_rim",
        TorusGeometry(radius=0.270, tube=0.010, radial_segments=16, tubular_segments=48).rotate_y(pi / 2.0),
    )
    handrim_mesh = _save_mesh(
        f"{mesh_prefix}_handrim",
        TorusGeometry(radius=0.286, tube=0.005, radial_segments=14, tubular_segments=48).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(rim_mesh, material=rim_material, name="rim")
    part.visual(
        handrim_mesh,
        origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.0)),
        material=rim_material,
        name="handrim",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.072),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.072),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_material,
        name="axle_sleeve",
    )
    spoke_angles = [index * (2.0 * pi / 8.0) for index in range(8)]
    for index, angle in enumerate(spoke_angles):
        start = (0.0, cos(angle) * 0.026, sin(angle) * 0.026)
        end = (0.0, cos(angle + pi / 8.0) * 0.266, sin(angle + pi / 8.0) * 0.266)
        _add_cylinder_between(
            part,
            start=start,
            end=end,
            radius=0.0032,
            material=rim_material,
            name=f"spoke_{index}",
        )
    for index, angle in enumerate((pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0)):
        _add_cylinder_between(
            part,
            start=(side_sign * 0.010, cos(angle) * 0.270, sin(angle) * 0.270),
            end=(side_sign * 0.030, cos(angle) * 0.286, sin(angle) * 0.286),
            radius=0.0030,
            material=rim_material,
            name=f"handrim_strut_{index}",
        )
    part.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(side_sign * 0.044, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_material,
        name="quick_release_button",
    )


def _add_caster_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    tire_material,
    rim_material,
    dark_material,
) -> None:
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        TorusGeometry(radius=0.063, tube=0.011, radial_segments=14, tubular_segments=36).rotate_y(pi / 2.0),
    )
    rim_mesh = _save_mesh(
        f"{mesh_prefix}_rim",
        TorusGeometry(radius=0.051, tube=0.006, radial_segments=12, tubular_segments=32).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(rim_mesh, material=rim_material, name="rim")
    part.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_material,
        name="wheel_disc",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_material,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.19, 0.21, 0.24, 1.0))
    upholstery = model.material("upholstery", rgba=(0.11, 0.12, 0.14, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    rim_metal = model.material("rim_metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.37, 0.38, 0.40, 1.0))
    footplate_finish = model.material("footplate_finish", rgba=(0.28, 0.29, 0.31, 1.0))

    chair = model.part("chair_frame")
    chair.inertial = Inertial.from_geometry(
        Box((0.72, 1.02, 0.88)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.12, 0.10)),
    )

    tube_profile = rounded_rect_profile(0.034, 0.024, 0.007, corner_segments=6)
    side_points = [
        (0.235, -0.230, 0.455),
        (0.235, -0.225, 0.220),
        (0.232, -0.020, 0.205),
        (0.225, 0.235, 0.185),
        (0.214, 0.362, 0.055),
        (0.210, 0.392, -0.005),
    ]
    left_side_frame = sweep_profile_along_spline(
        side_points,
        profile=tube_profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    right_side_frame = sweep_profile_along_spline(
        _mirror_x(side_points),
        profile=tube_profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    chair.visual(_save_mesh("left_side_frame", left_side_frame), material=frame_paint, name="left_side_frame")
    chair.visual(_save_mesh("right_side_frame", right_side_frame), material=frame_paint, name="right_side_frame")

    left_rear_strut = sweep_profile_along_spline(
        [(0.232, -0.120, 0.188), (0.257, -0.060, 0.084), (0.286, -0.015, 0.002)],
        profile=rounded_rect_profile(0.030, 0.022, 0.006, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )
    right_rear_strut = sweep_profile_along_spline(
        _mirror_x([(0.232, -0.120, 0.188), (0.257, -0.060, 0.084), (0.286, -0.015, 0.002)]),
        profile=rounded_rect_profile(0.030, 0.022, 0.006, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )
    chair.visual(_save_mesh("left_rear_strut", left_rear_strut), material=frame_paint, name="left_rear_strut")
    chair.visual(_save_mesh("right_rear_strut", right_rear_strut), material=frame_paint, name="right_rear_strut")

    left_hanger = sweep_profile_along_spline(
        [(0.214, 0.320, 0.090), (0.180, 0.412, -0.035), (0.118, 0.455, -0.138)],
        profile=rounded_rect_profile(0.028, 0.020, 0.005, corner_segments=5),
        samples_per_segment=14,
        cap_profile=True,
    )
    right_hanger = sweep_profile_along_spline(
        _mirror_x([(0.214, 0.320, 0.090), (0.180, 0.412, -0.035), (0.118, 0.455, -0.138)]),
        profile=rounded_rect_profile(0.028, 0.020, 0.005, corner_segments=5),
        samples_per_segment=14,
        cap_profile=True,
    )
    chair.visual(_save_mesh("left_hanger", left_hanger), material=frame_paint, name="left_hanger")
    chair.visual(_save_mesh("right_hanger", right_hanger), material=frame_paint, name="right_hanger")

    chair.visual(
        Box((0.430, 0.370, 0.020)),
        origin=Origin(xyz=(0.0, 0.045, 0.195)),
        material=upholstery,
        name="seat_sling",
    )
    chair.visual(
        Box((0.492, 0.040, 0.320)),
        origin=Origin(xyz=(0.0, -0.205, 0.300)),
        material=upholstery,
        name="backrest_panel",
    )
    chair.visual(
        Box((0.486, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, -0.165, 0.218)),
        material=frame_paint,
        name="backrest_crossbar",
    )
    chair.visual(
        Box((0.402, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, 0.105, 0.195)),
        material=frame_paint,
        name="front_seat_crossbar",
    )
    chair.visual(
        Box((0.444, 0.040, 0.034)),
        origin=Origin(xyz=(0.0, 0.278, 0.115)),
        material=frame_paint,
        name="legrest_crossbar",
    )

    chair.visual(
        Box((0.090, 0.110, 0.100)),
        origin=Origin(xyz=(0.286, -0.012, 0.000)),
        material=dark_metal,
        name="left_axle_block",
    )
    chair.visual(
        Box((0.090, 0.110, 0.100)),
        origin=Origin(xyz=(-0.286, -0.012, 0.000)),
        material=dark_metal,
        name="right_axle_block",
    )
    chair.visual(
        Box((0.100, 0.070, 0.036)),
        origin=Origin(xyz=(0.230, 0.422, -0.001)),
        material=dark_metal,
        name="left_caster_receiver",
    )
    chair.visual(
        Box((0.100, 0.070, 0.036)),
        origin=Origin(xyz=(-0.230, 0.422, -0.001)),
        material=dark_metal,
        name="right_caster_receiver",
    )
    chair.visual(
        Box((0.118, 0.105, 0.018)),
        origin=Origin(xyz=(0.092, 0.458, -0.148)),
        material=footplate_finish,
        name="left_footplate",
    )
    chair.visual(
        Box((0.118, 0.105, 0.018)),
        origin=Origin(xyz=(-0.092, 0.458, -0.148)),
        material=footplate_finish,
        name="right_footplate",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.308, length=0.032),
        mass=4.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        left_rear_wheel,
        mesh_prefix="left_rear_wheel",
        side_sign=1.0,
        tire_material=tire_rubber,
        rim_material=rim_metal,
        dark_material=dark_metal,
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.308, length=0.032),
        mass=4.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        right_rear_wheel,
        mesh_prefix="right_rear_wheel",
        side_sign=-1.0,
        tire_material=tire_rubber,
        rim_material=rim_metal,
        dark_material=dark_metal,
    )

    def _build_caster_swivel(name: str) -> None:
        caster = model.part(name)
        caster.inertial = Inertial.from_geometry(
            Box((0.090, 0.090, 0.270)),
            mass=1.2,
            origin=Origin(xyz=(0.0, -0.014, -0.126)),
        )
        caster.visual(
            Cylinder(radius=0.018, length=0.102),
            origin=Origin(xyz=(0.0, 0.0, -0.084)),
            material=dark_metal,
            name="stem",
        )
        caster.visual(
            Cylinder(radius=0.026, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.044)),
            material=dark_metal,
            name="top_collar",
        )
        caster.visual(
            Box((0.082, 0.050, 0.032)),
            origin=Origin(xyz=(0.0, -0.004, -0.094)),
            material=dark_metal,
            name="fork_crown",
        )
        caster.visual(
            Box((0.016, 0.058, 0.170)),
            origin=Origin(xyz=(0.034, -0.028, -0.162)),
            material=dark_metal,
            name="outer_fork_leg",
        )
        caster.visual(
            Box((0.016, 0.058, 0.170)),
            origin=Origin(xyz=(-0.034, -0.028, -0.162)),
            material=dark_metal,
            name="inner_fork_leg",
        )

    _build_caster_swivel("left_caster_swivel")
    _build_caster_swivel("right_caster_swivel")

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.042),
        mass=0.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        left_caster_wheel,
        mesh_prefix="left_caster_wheel",
        tire_material=tire_rubber,
        rim_material=rim_metal,
        dark_material=dark_metal,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.042),
        mass=0.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        right_caster_wheel,
        mesh_prefix="right_caster_wheel",
        tire_material=tire_rubber,
        rim_material=rim_metal,
        dark_material=dark_metal,
    )

    model.articulation(
        "left_rear_wheel_spin_joint",
        ArticulationType.CONTINUOUS,
        parent=chair,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.367, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "right_rear_wheel_spin_joint",
        ArticulationType.CONTINUOUS,
        parent=chair,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.367, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "left_caster_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=chair,
        child="left_caster_swivel",
        origin=Origin(xyz=(0.236, 0.424, 0.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "right_caster_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=chair,
        child="right_caster_swivel",
        origin=Origin(xyz=(-0.236, 0.424, 0.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "left_caster_wheel_spin_joint",
        ArticulationType.CONTINUOUS,
        parent="left_caster_swivel",
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, -0.028, -0.234)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_wheel_spin_joint",
        ArticulationType.CONTINUOUS,
        parent="right_caster_swivel",
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, -0.028, -0.234)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
