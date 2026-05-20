from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
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


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _rounded_rect_section(
    width: float,
    length: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, length, radius)]


def _wheel_profile(radius: float, width: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    return [
        (radius * 0.55, -half_width * 0.98),
        (radius * 0.75, -half_width),
        (radius * 0.90, -half_width * 0.82),
        (radius * 0.97, -half_width * 0.46),
        (radius, -half_width * 0.14),
        (radius, half_width * 0.14),
        (radius * 0.97, half_width * 0.46),
        (radius * 0.90, half_width * 0.82),
        (radius * 0.75, half_width),
        (radius * 0.55, half_width * 0.98),
        (radius * 0.47, half_width * 0.30),
        (radius * 0.43, 0.0),
        (radius * 0.47, -half_width * 0.30),
        (radius * 0.55, -half_width * 0.98),
    ]


def _add_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    radius: float,
    tire_width: float,
    hub_width: float,
    rubber,
    metal,
) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.68, length=hub_width),
        origin=spin_origin,
        material=metal,
        name="rim",
    )
    cap_offset = max(0.0, min(tire_width * 0.30, (tire_width - hub_width) * 0.5 + hub_width * 0.5))
    cap_length = max(0.004, tire_width * 0.10)
    part.visual(
        Cylinder(radius=radius * 0.18, length=cap_length),
        origin=Origin(xyz=(cap_offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=radius * 0.18, length=cap_length),
        origin=Origin(xyz=(-cap_offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="inner_cap",
    )


def _build_frame_geometry():
    main_radius = 0.016
    brace_radius = 0.011

    frame_geom = CylinderGeometry(radius=0.010, height=0.395).rotate_y(math.pi / 2.0).translate(0.0, -0.035, 0.10)
    frame_geom.merge(
        CylinderGeometry(radius=0.010, height=0.035).rotate_y(math.pi / 2.0).translate(-0.215, -0.035, 0.10)
    )
    frame_geom.merge(
        CylinderGeometry(radius=0.010, height=0.035).rotate_y(math.pi / 2.0).translate(0.215, -0.035, 0.10)
    )

    left_rail = [
        (-0.195, -0.035, 0.145),
        (-0.170, 0.10, 0.18),
        (-0.148, 0.30, 0.18),
        (-0.140, 0.48, 0.18),
        (-0.120, 0.555, 0.195),
    ]
    frame_geom.merge(
        tube_from_spline_points(
            left_rail,
            radius=main_radius,
            samples_per_segment=14,
            radial_segments=18,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(left_rail),
            radius=main_radius,
            samples_per_segment=14,
            radial_segments=18,
        )
    )

    frame_geom.merge(
        CylinderGeometry(radius=0.012, height=0.16).rotate_y(math.pi / 2.0).translate(0.0, 0.18, 0.18)
    )

    frame_geom.merge(CylinderGeometry(radius=0.013, height=0.346).translate(-0.070, 0.23, 0.353))
    frame_geom.merge(CylinderGeometry(radius=0.013, height=0.346).translate(0.070, 0.23, 0.353))
    frame_geom.merge(BoxGeometry((0.18, 0.22, 0.008)).translate(0.0, 0.23, 0.522))
    frame_geom.merge(BoxGeometry((0.16, 0.05, 0.018)).translate(0.0, 0.23, 0.34))

    frame_geom.merge(
        tube_from_spline_points(
            [
                (-0.145, 0.28, 0.18),
                (-0.105, 0.245, 0.30),
                (-0.075, 0.23, 0.46),
            ],
            radius=brace_radius,
            samples_per_segment=10,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(
                [
                    (-0.145, 0.28, 0.18),
                    (-0.105, 0.245, 0.30),
                    (-0.075, 0.23, 0.46),
                ]
            ),
            radius=brace_radius,
            samples_per_segment=10,
            radial_segments=16,
        )
    )

    frame_geom.merge(BoxGeometry((0.32, 0.026, 0.012)).translate(0.0, 0.605, 0.223))
    frame_geom.merge(CylinderGeometry(radius=0.016, height=0.012).translate(-0.13, 0.605, 0.223))
    frame_geom.merge(CylinderGeometry(radius=0.016, height=0.012).translate(0.13, 0.605, 0.223))

    frame_geom.merge(
        tube_from_spline_points(
            [
                (-0.120, 0.555, 0.195),
                (-0.095, 0.585, 0.212),
                (-0.075, 0.600, 0.223),
            ],
            radius=brace_radius,
            samples_per_segment=8,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(
                [
                    (-0.120, 0.555, 0.195),
                    (-0.095, 0.585, 0.212),
                    (-0.075, 0.600, 0.223),
                ]
            ),
            radius=brace_radius,
            samples_per_segment=8,
            radial_segments=16,
        )
    )

    frame_geom.merge(
        tube_from_spline_points(
            [
                (0.0, 0.605, 0.223),
                (0.0, 0.575, 0.48),
                (0.0, 0.545, 0.72),
                (0.0, 0.515, 0.97),
            ],
            radius=0.019,
            samples_per_segment=16,
            radial_segments=20,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            [
                (-0.075, 0.600, 0.223),
                (-0.035, 0.565, 0.35),
                (0.0, 0.545, 0.50),
            ],
            radius=brace_radius,
            samples_per_segment=10,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            _mirror_x(
                [
                    (-0.075, 0.600, 0.223),
                    (-0.035, 0.565, 0.35),
                    (0.0, 0.545, 0.50),
                ]
            ),
            radius=brace_radius,
            samples_per_segment=10,
            radial_segments=16,
        )
    )
    frame_geom.merge(
        tube_from_spline_points(
            [
                (-0.25, 0.47, 0.95),
                (-0.15, 0.49, 0.964),
                (0.0, 0.50, 0.972),
                (0.15, 0.49, 0.964),
                (0.25, 0.47, 0.95),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=18,
        )
    )

    return frame_geom


def _build_caster_geometry():
    caster_geom = CylinderGeometry(radius=0.018, height=0.008).translate(0.0, 0.0, -0.004)
    caster_geom.merge(CylinderGeometry(radius=0.009, height=0.050).translate(0.0, 0.0, -0.033))
    caster_geom.merge(BoxGeometry((0.030, 0.016, 0.012)).translate(0.0, -0.038, -0.060))
    caster_geom.merge(BoxGeometry((0.006, 0.012, 0.070)).translate(-0.016, -0.052, -0.103))
    caster_geom.merge(BoxGeometry((0.006, 0.012, 0.070)).translate(0.016, -0.052, -0.103))
    return caster_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter_twin_caster")

    frame_blue = model.material("frame_blue", rgba=(0.21, 0.43, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.76, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.12, 0.12, 0.13, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.16, 0.17, 0.18, 1.0))

    main_frame = model.part("main_frame")
    main_frame.visual(
        Cylinder(radius=0.011, length=0.39),
        origin=Origin(xyz=(0.0, -0.05, 0.13), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_blue,
        name="rear_cross_tube",
    )
    main_frame.visual(
        Box((0.028, 0.62, 0.060)),
        origin=Origin(xyz=(-0.15, 0.25, 0.155), rpy=(-0.08, 0.0, 0.0)),
        material=frame_blue,
        name="left_lower_rail",
    )
    main_frame.visual(
        Box((0.028, 0.62, 0.060)),
        origin=Origin(xyz=(0.15, 0.25, 0.155), rpy=(-0.08, 0.0, 0.0)),
        material=frame_blue,
        name="right_lower_rail",
    )
    main_frame.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, 0.18, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_blue,
        name="mid_cross_tube",
    )
    main_frame.visual(
        Box((0.08, 0.70, 0.120)),
        origin=Origin(xyz=(0.0, 0.285, 0.16)),
        material=frame_blue,
        name="center_spine",
    )
    main_frame.visual(
        Box((0.12, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.23, 0.28)),
        material=frame_blue,
        name="pad_mast",
    )
    main_frame.visual(
        Cylinder(radius=0.013, length=0.35),
        origin=Origin(xyz=(-0.075, 0.23, 0.351)),
        material=frame_blue,
        name="left_pad_upright",
    )
    main_frame.visual(
        Cylinder(radius=0.013, length=0.35),
        origin=Origin(xyz=(0.075, 0.23, 0.351)),
        material=frame_blue,
        name="right_pad_upright",
    )
    main_frame.visual(
        Box((0.16, 0.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.23, 0.34)),
        material=frame_blue,
        name="pad_support_crossbar",
    )
    main_frame.visual(
        Box((0.18, 0.24, 0.006)),
        origin=Origin(xyz=(0.0, 0.23, 0.523)),
        material=dark_steel,
        name="pad_support_deck",
    )
    main_frame.visual(
        Box((0.030, 0.16, 0.16)),
        origin=Origin(xyz=(-0.14, 0.23, 0.27)),
        material=frame_blue,
        name="left_pad_brace",
    )
    main_frame.visual(
        Box((0.030, 0.16, 0.16)),
        origin=Origin(xyz=(0.14, 0.23, 0.27)),
        material=frame_blue,
        name="right_pad_brace",
    )
    main_frame.visual(
        Box((0.30, 0.12, 0.030)),
        origin=Origin(xyz=(0.0, 0.575, 0.223)),
        material=frame_blue,
        name="front_nose_beam",
    )
    main_frame.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.12, 0.62, 0.246)),
        material=frame_blue,
        name="left_caster_receiver",
    )
    main_frame.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.12, 0.62, 0.246)),
        material=frame_blue,
        name="right_caster_receiver",
    )
    main_frame.visual(
        Cylinder(radius=0.019, length=0.76),
        origin=Origin(xyz=(0.0, 0.58, 0.60)),
        material=frame_blue,
        name="handle_column",
    )
    main_frame.visual(
        Box((0.12, 0.12, 0.70)),
        origin=Origin(xyz=(0.0, 0.58, 0.54)),
        material=frame_blue,
        name="steering_tower_core",
    )
    main_frame.visual(
        Box((0.080, 0.080, 0.28)),
        origin=Origin(xyz=(-0.040, 0.575, 0.36)),
        material=frame_blue,
        name="left_column_brace",
    )
    main_frame.visual(
        Box((0.080, 0.080, 0.28)),
        origin=Origin(xyz=(0.040, 0.575, 0.36)),
        material=frame_blue,
        name="right_column_brace",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.52),
        origin=Origin(xyz=(0.0, 0.58, 0.95), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_blue,
        name="handlebar_tube",
    )
    main_frame.visual(
        Cylinder(radius=0.018, length=0.095),
        origin=Origin(xyz=(-0.285, 0.58, 0.95), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    main_frame.visual(
        Cylinder(radius=0.018, length=0.095),
        origin=Origin(xyz=(0.285, 0.58, 0.95), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    main_frame.visual(
        Box((0.050, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.56, 0.83)),
        material=clamp_black,
        name="column_clamp",
    )
    main_frame.visual(
        Box((0.10, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.58, 0.91)),
        material=frame_blue,
        name="handlebar_mount",
    )
    main_frame.visual(
        Box((0.14, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.58, 0.92)),
        material=frame_blue,
        name="upper_bridge",
    )
    main_frame.visual(
        Box((0.040, 0.040, 0.040)),
        origin=Origin(xyz=(-0.175, -0.045, 0.130)),
        material=frame_blue,
        name="left_rear_gusset",
    )
    main_frame.visual(
        Box((0.040, 0.040, 0.040)),
        origin=Origin(xyz=(0.175, -0.045, 0.130)),
        material=frame_blue,
        name="right_rear_gusset",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(-0.1975, -0.05, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_rear_axle_stub",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.1975, -0.05, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_rear_axle_stub",
    )
    main_frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.80, 0.96)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.30, 0.48)),
    )

    knee_pad = model.part("knee_pad")
    knee_pad.visual(
        Box((0.22, 0.30, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_steel,
        name="pad_base",
    )
    knee_pad.visual(
        Box((0.24, 0.32, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=pad_vinyl,
        name="pad_cushion",
    )
    knee_pad.visual(
        Box((0.20, 0.26, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=pad_vinyl,
        name="pad_top",
    )
    knee_pad.inertial = Inertial.from_geometry(
        Box((0.24, 0.32, 0.08)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    left_rear_wheel = model.part("left_rear_wheel")
    _add_wheel_visuals(
        left_rear_wheel,
        mesh_prefix="left_rear_wheel",
        radius=0.10,
        tire_width=0.045,
        hub_width=0.040,
        rubber=rubber,
        metal=steel,
    )
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.045),
        mass=1.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_rear_wheel = model.part("right_rear_wheel")
    _add_wheel_visuals(
        right_rear_wheel,
        mesh_prefix="right_rear_wheel",
        radius=0.10,
        tire_width=0.045,
        hub_width=0.040,
        rubber=rubber,
        metal=steel,
    )
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.045),
        mass=1.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_caster = model.part("left_caster")
    left_caster.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=dark_steel,
        name="top_cap",
    )
    left_caster.visual(Cylinder(radius=0.009, length=0.054), origin=Origin(xyz=(0.0, 0.0, -0.031)), material=dark_steel, name="pivot_stem")
    left_caster.visual(Box((0.050, 0.050, 0.014)), origin=Origin(xyz=(0.0, -0.033, -0.055)), material=dark_steel, name="fork_bridge")
    left_caster.visual(Box((0.006, 0.012, 0.072)), origin=Origin(xyz=(-0.020, -0.052, -0.096)), material=dark_steel, name="left_fork_leg")
    left_caster.visual(Box((0.006, 0.012, 0.072)), origin=Origin(xyz=(0.020, -0.052, -0.096)), material=dark_steel, name="right_fork_leg")
    left_caster.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.017, -0.052, -0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_axle_cap",
    )
    left_caster.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.017, -0.052, -0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_axle_cap",
    )
    left_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.070, 0.145)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.030, -0.072)),
    )

    right_caster = model.part("right_caster")
    right_caster.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=dark_steel,
        name="top_cap",
    )
    right_caster.visual(Cylinder(radius=0.009, length=0.054), origin=Origin(xyz=(0.0, 0.0, -0.031)), material=dark_steel, name="pivot_stem")
    right_caster.visual(Box((0.050, 0.050, 0.014)), origin=Origin(xyz=(0.0, -0.033, -0.055)), material=dark_steel, name="fork_bridge")
    right_caster.visual(Box((0.006, 0.012, 0.072)), origin=Origin(xyz=(-0.020, -0.052, -0.096)), material=dark_steel, name="left_fork_leg")
    right_caster.visual(Box((0.006, 0.012, 0.072)), origin=Origin(xyz=(0.020, -0.052, -0.096)), material=dark_steel, name="right_fork_leg")
    right_caster.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.017, -0.052, -0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_axle_cap",
    )
    right_caster.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.017, -0.052, -0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_axle_cap",
    )
    right_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.070, 0.145)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.030, -0.072)),
    )

    left_front_wheel = model.part("left_front_wheel")
    _add_wheel_visuals(
        left_front_wheel,
        mesh_prefix="left_front_wheel",
        radius=0.075,
        tire_width=0.028,
        hub_width=0.028,
        rubber=rubber,
        metal=steel,
    )
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.028),
        mass=0.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_front_wheel = model.part("right_front_wheel")
    _add_wheel_visuals(
        right_front_wheel,
        mesh_prefix="right_front_wheel",
        radius=0.075,
        tire_width=0.028,
        hub_width=0.028,
        rubber=rubber,
        metal=steel,
    )
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.028),
        mass=0.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_knee_pad",
        ArticulationType.FIXED,
        parent=main_frame,
        child=knee_pad,
        origin=Origin(xyz=(0.0, 0.23, 0.526)),
    )
    model.articulation(
        "left_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.23, -0.05, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "right_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(0.23, -0.05, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_caster,
        origin=Origin(xyz=(-0.12, 0.62, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_caster,
        origin=Origin(xyz=(0.12, 0.62, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )
    model.articulation(
        "left_front_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, -0.052, -0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )
    model.articulation(
        "right_front_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, -0.052, -0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    main_frame = object_model.get_part("main_frame")
    knee_pad = object_model.get_part("knee_pad")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    left_caster = object_model.get_part("left_caster")
    right_caster = object_model.get_part("right_caster")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")

    left_rear_spin = object_model.get_articulation("left_rear_spin")
    right_rear_spin = object_model.get_articulation("right_rear_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_front_spin = object_model.get_articulation("left_front_spin")
    right_front_spin = object_model.get_articulation("right_front_spin")

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

    ctx.expect_contact(knee_pad, main_frame, contact_tol=0.002)
    ctx.expect_contact(left_rear_wheel, main_frame, contact_tol=0.002)
    ctx.expect_contact(right_rear_wheel, main_frame, contact_tol=0.002)
    ctx.expect_contact(left_caster, main_frame, contact_tol=0.002)
    ctx.expect_contact(right_caster, main_frame, contact_tol=0.002)
    ctx.expect_contact(left_front_wheel, left_caster, contact_tol=0.002)
    ctx.expect_contact(right_front_wheel, right_caster, contact_tol=0.002)

    ctx.expect_overlap(knee_pad, main_frame, axes="xy", min_overlap=0.08)

    ctx.check(
        "rear wheels spin on axle axis",
        left_rear_spin.axis == (1.0, 0.0, 0.0) and right_rear_spin.axis == (1.0, 0.0, 0.0),
        f"rear axes were {left_rear_spin.axis} and {right_rear_spin.axis}",
    )
    ctx.check(
        "front wheels spin on axle axis",
        left_front_spin.axis == (1.0, 0.0, 0.0) and right_front_spin.axis == (1.0, 0.0, 0.0),
        f"front axes were {left_front_spin.axis} and {right_front_spin.axis}",
    )
    ctx.check(
        "casters swivel vertically",
        left_caster_swivel.axis == (0.0, 0.0, 1.0) and right_caster_swivel.axis == (0.0, 0.0, 1.0),
        f"caster axes were {left_caster_swivel.axis} and {right_caster_swivel.axis}",
    )

    frame_aabb = ctx.part_world_aabb(main_frame)
    assert frame_aabb is not None
    frame_length = frame_aabb[1][1] - frame_aabb[0][1]
    frame_height = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "knee scooter proportions",
        frame_length > 0.65 and frame_height > 0.88,
        f"frame length={frame_length:.3f}, height={frame_height:.3f}",
    )

    left_front_rest = ctx.part_world_position(left_front_wheel)
    assert left_front_rest is not None
    with ctx.pose({left_caster_swivel: math.pi / 2.0}):
        left_front_swiveled = ctx.part_world_position(left_front_wheel)
        assert left_front_swiveled is not None
        ctx.check(
            "left caster swivel moves trailing wheel center",
            left_front_swiveled[0] > left_front_rest[0] + 0.02 and left_front_swiveled[1] > left_front_rest[1] + 0.02,
            f"rest={left_front_rest}, swiveled={left_front_swiveled}",
        )
        ctx.expect_contact(left_front_wheel, left_caster, contact_tol=0.002)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
