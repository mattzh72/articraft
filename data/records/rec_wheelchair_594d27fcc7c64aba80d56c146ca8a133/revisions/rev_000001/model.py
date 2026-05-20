from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _tube_segment(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    radial_segments: int = 14,
):
    return tube_from_spline_points(
        [start, end],
        radius=radius,
        samples_per_segment=1,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _add_drive_wheel(
    part,
    *,
    prefix: str,
    side_sign: float,
    rim_material,
    tire_material,
    hardware_material,
    marker_material,
) -> None:
    tire = TorusGeometry(radius=0.279, tube=0.026, radial_segments=18, tubular_segments=40).rotate_y(pi / 2.0)
    rim = TorusGeometry(radius=0.246, tube=0.010, radial_segments=16, tubular_segments=40).rotate_y(pi / 2.0)
    pushrim = (
        TorusGeometry(radius=0.274, tube=0.006, radial_segments=12, tubular_segments=40)
        .rotate_y(pi / 2.0)
        .translate(side_sign * 0.028, 0.0, 0.0)
    )

    part.visual(_save_mesh(f"{prefix}_tire", tire), material=tire_material, name="tire")
    part.visual(_save_mesh(f"{prefix}_rim", rim), material=rim_material, name="rim")
    part.visual(_save_mesh(f"{prefix}_pushrim", pushrim), material=rim_material, name="pushrim")

    part.visual(
        Cylinder(radius=0.062, length=0.010),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_material,
        name="outer_flange",
    )
    part.visual(
        Cylinder(radius=0.062, length=0.010),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_material,
        name="inner_flange",
    )
    part.visual(
        Cylinder(radius=0.052, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.066),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="axle_bore_shell",
    )

    spoke_angles = [index * (pi / 4.0) for index in range(8)]
    for spoke_index, angle in enumerate(spoke_angles):
        start = (0.0, cos(angle) * 0.046, sin(angle) * 0.046)
        end = (0.0, cos(angle + 0.14) * 0.238, sin(angle + 0.14) * 0.238)
        part.visual(
            _save_mesh(f"{prefix}_spoke_{spoke_index}", _tube_segment(start, end, radius=0.0036)),
            material=rim_material,
        )

    for strut_index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        strut = _tube_segment(
            (side_sign * 0.010, cos(angle) * 0.242, sin(angle) * 0.242),
            (side_sign * 0.028, cos(angle) * 0.274, sin(angle) * 0.274),
            radius=0.003,
            radial_segments=12,
        )
        part.visual(_save_mesh(f"{prefix}_pushrim_strut_{strut_index}", strut), material=hardware_material)

    part.visual(
        Box((0.004, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.246, 0.0)),
        material=marker_material,
        name="index_mark",
    )


def _add_caster_wheel(
    part,
    *,
    prefix: str,
    rim_material,
    tire_material,
    hardware_material,
) -> None:
    tire = TorusGeometry(radius=0.054, tube=0.016, radial_segments=14, tubular_segments=28).rotate_y(pi / 2.0)
    rim = TorusGeometry(radius=0.043, tube=0.006, radial_segments=12, tubular_segments=28).rotate_y(pi / 2.0)
    part.visual(_save_mesh(f"{prefix}_tire", tire), material=tire_material, name="tire")
    part.visual(_save_mesh(f"{prefix}_rim", rim), material=rim_material, name="rim")
    part.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="outer_bushing",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="inner_bushing",
    )
    part.visual(
        Cylinder(radius=0.0055, length=0.078),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="axle_pin",
    )
    for spoke_index, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        part.visual(
            _save_mesh(
                f"{prefix}_spoke_{spoke_index}",
                _tube_segment(
                    (0.0, cos(angle) * 0.020, sin(angle) * 0.020),
                    (0.0, cos(angle + 0.10) * 0.040, sin(angle + 0.10) * 0.040),
                    radius=0.0025,
                    radial_segments=10,
                ),
            ),
            material=rim_material,
        )


def _build_footrest(part, *, prefix: str, side_sign: float, frame_material, pad_material, marker_material, knob_material) -> None:
    hanger_path = [
        (0.0, 0.0, -0.006),
        (0.0, 0.040, -0.090),
        (0.0, 0.070, -0.205),
        (0.0, 0.072, -0.322),
    ]
    part.visual(
        Box((0.034, 0.036, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=frame_material,
        name="mount_clamp",
    )
    part.visual(
        _save_mesh(f"{prefix}_hanger", tube_from_spline_points(hanger_path, radius=0.012, samples_per_segment=10, radial_segments=14)),
        material=frame_material,
        name="hanger",
    )
    part.visual(
        Box((0.040, 0.040, 0.078)),
        origin=Origin(xyz=(0.0, 0.032, -0.082)),
        material=frame_material,
        name="adjuster_sleeve",
    )
    part.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(xyz=(side_sign * 0.020, 0.032, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=knob_material,
        name="clamp_knob",
    )
    for mark_index, local_z in enumerate((-0.045, -0.065, -0.085, -0.105)):
        part.visual(
            Box((0.006, 0.002, 0.012)),
            origin=Origin(xyz=(side_sign * 0.018, 0.050, local_z)),
            material=marker_material,
            name=f"height_mark_{mark_index}",
        )
    part.visual(
        Cylinder(radius=0.008, length=0.090),
        origin=Origin(xyz=(0.0, 0.082, -0.330), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="plate_spindle",
    )
    part.visual(
        Box((0.024, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, 0.078, -0.325)),
        material=frame_material,
        name="plate_support_block",
    )
    part.visual(
        Box((0.120, 0.200, 0.012)),
        origin=Origin(xyz=(0.0, 0.110, -0.340)),
        material=pad_material,
        name="datum_plate",
    )
    part.visual(
        Box((0.120, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.203, -0.330)),
        material=frame_material,
        name="front_datum_lip",
    )
    part.visual(
        Box((0.012, 0.200, 0.018)),
        origin=Origin(xyz=(side_sign * 0.054, 0.110, -0.331)),
        material=frame_material,
        name="outer_datum_lip",
    )
    part.visual(
        Box((0.008, 0.160, 0.002)),
        origin=Origin(xyz=(0.0, 0.110, -0.339)),
        material=marker_material,
        name="plate_centerline",
    )
    for tick_index, local_y in enumerate((0.060, 0.110, 0.160)):
        part.visual(
            Box((0.040, 0.004, 0.002)),
            origin=Origin(xyz=(0.0, local_y, -0.339)),
            material=marker_material,
            name=f"plate_tick_{tick_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_wheelchair")

    frame_material = model.material("frame_satin_titanium", rgba=(0.66, 0.69, 0.72, 1.0))
    rim_material = model.material("rim_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    hardware_material = model.material("hardware_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    pad_material = model.material("pad_black", rgba=(0.11, 0.12, 0.13, 1.0))
    tire_material = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    marker_material = model.material("marker_amber", rgba=(0.92, 0.72, 0.16, 1.0))
    sling_material = model.material("sling_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.64, 0.96, 0.92)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.04, 0.46)),
    )

    for name, points in (
        ("left_lower_rail", [(0.215, 0.245, 0.200), (0.230, 0.180, 0.192), (0.232, 0.020, 0.192), (0.240, -0.090, 0.215), (0.250, -0.180, 0.252)]),
        ("right_lower_rail", _mirror_x([(0.215, 0.245, 0.200), (0.230, 0.180, 0.192), (0.232, 0.020, 0.192), (0.240, -0.090, 0.215), (0.250, -0.180, 0.252)])),
        ("left_upper_rail", [(0.230, 0.180, 0.490), (0.230, 0.030, 0.490), (0.222, -0.130, 0.492), (0.202, -0.220, 0.512)]),
        ("right_upper_rail", _mirror_x([(0.230, 0.180, 0.490), (0.230, 0.030, 0.490), (0.222, -0.130, 0.492), (0.202, -0.220, 0.512)])),
        ("left_rear_brace", [(0.232, -0.055, 0.198), (0.224, -0.135, 0.340), (0.203, -0.220, 0.512)]),
        ("right_rear_brace", _mirror_x([(0.232, -0.055, 0.198), (0.224, -0.135, 0.340), (0.203, -0.220, 0.512)])),
    ):
        main_frame.visual(
            _save_mesh(f"wheelchair_{name}", tube_from_spline_points(points, radius=0.016, samples_per_segment=12, radial_segments=16)),
            material=frame_material,
            name=name,
        )

    main_frame.visual(
        Cylinder(radius=0.015, length=0.460),
        origin=Origin(xyz=(0.0, 0.245, 0.192), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="front_cross_tube",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.420),
        origin=Origin(xyz=(0.0, 0.193, 0.500), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="footrest_bridge",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.420),
        origin=Origin(xyz=(0.0, 0.170, 0.490), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="seat_front_cross",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.360),
        origin=Origin(xyz=(0.0, -0.020, 0.490), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="seat_mid_cross",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.480),
        origin=Origin(xyz=(0.0, -0.085, 0.252), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="axle_crossmember",
    )
    main_frame.visual(
        Box((0.440, 0.360, 0.045)),
        origin=Origin(xyz=(0.0, 0.000, 0.523)),
        material=sling_material,
        name="seat_sling",
    )
    main_frame.visual(
        Box((0.050, 0.320, 0.030)),
        origin=Origin(xyz=(0.285, -0.030, 0.625)),
        material=pad_material,
        name="left_armrest_pad",
    )
    main_frame.visual(
        Box((0.050, 0.320, 0.030)),
        origin=Origin(xyz=(-0.285, -0.030, 0.625)),
        material=pad_material,
        name="right_armrest_pad",
    )
    main_frame.visual(
        Box((0.028, 0.060, 0.140)),
        origin=Origin(xyz=(0.255, -0.030, 0.570)),
        material=frame_material,
        name="left_armrest_post",
    )
    main_frame.visual(
        Box((0.028, 0.060, 0.140)),
        origin=Origin(xyz=(-0.255, -0.030, 0.570)),
        material=frame_material,
        name="right_armrest_post",
    )

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        main_frame.visual(
            Cylinder(radius=0.015, length=0.300),
            origin=Origin(xyz=(side_sign * 0.215, 0.193, 0.340), rpy=(0.0, 0.0, 0.0)),
            material=frame_material,
            name=f"{side_name}_front_upright",
        )
        main_frame.visual(
            Cylinder(radius=0.020, length=0.040),
            origin=Origin(xyz=(side_sign * 0.215, 0.390, 0.200), rpy=(0.0, 0.0, 0.0)),
            material=frame_material,
            name=f"{side_name}_caster_receiver",
        )
        main_frame.visual(
            _save_mesh(
                f"wheelchair_{side_name}_caster_boom",
                tube_from_spline_points(
                    [
                        (side_sign * 0.215, 0.245, 0.200),
                        (side_sign * 0.220, 0.320, 0.194),
                        (side_sign * 0.215, 0.390, 0.194),
                    ],
                    radius=0.014,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
            ),
            material=frame_material,
            name=f"{side_name}_caster_boom",
        )
        main_frame.visual(
            Box((0.030, 0.040, 0.020)),
            origin=Origin(xyz=(side_sign * 0.110, 0.210, 0.500)),
            material=frame_material,
            name=f"{side_name}_footrest_receiver",
        )
        main_frame.visual(
            Box((0.070, 0.110, 0.110)),
            origin=Origin(xyz=(side_sign * 0.275, -0.075, 0.285)),
            material=frame_material,
            name=f"{side_name}_axle_plate",
        )
        main_frame.visual(
            Cylinder(radius=0.011, length=0.026),
            origin=Origin(xyz=(side_sign * 0.304, -0.030, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware_material,
            name=f"{side_name}_axle_stub",
        )
        main_frame.visual(
            Box((0.012, 0.130, 0.100)),
            origin=Origin(xyz=(side_sign * 0.298, -0.030, 0.315)),
            material=rim_material,
            name=f"{side_name}_axle_scale_plate",
        )
        for mark_index, local_y in enumerate((-0.060, -0.030, 0.000, 0.030, 0.060)):
            main_frame.visual(
                Box((0.002, 0.010, 0.010 if mark_index != 2 else 0.020)),
                origin=Origin(xyz=(side_sign * 0.305, local_y - 0.030, 0.315)),
                material=marker_material,
                name=f"{side_name}_axle_mark_{mark_index}",
            )
        main_frame.visual(
            Box((0.040, 0.040, 0.040)),
            origin=Origin(xyz=(side_sign * 0.200, -0.220, 0.520)),
            material=frame_material,
            name=f"{side_name}_backrest_socket",
        )

    backrest = model.part("backrest_frame")
    backrest.inertial = Inertial.from_geometry(
        Box((0.46, 0.14, 0.52)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.02, 0.26)),
    )
    backrest.visual(
        Box((0.040, 0.030, 0.040)),
        origin=Origin(xyz=(0.200, 0.000, 0.020)),
        material=frame_material,
        name="left_mount_lug",
    )
    backrest.visual(
        Box((0.040, 0.030, 0.040)),
        origin=Origin(xyz=(-0.200, 0.000, 0.020)),
        material=frame_material,
        name="right_mount_lug",
    )
    for name, points in (
        ("left_upright", [(0.200, 0.000, 0.000), (0.200, -0.025, 0.220), (0.192, -0.050, 0.470)]),
        ("right_upright", _mirror_x([(0.200, 0.000, 0.000), (0.200, -0.025, 0.220), (0.192, -0.050, 0.470)])),
    ):
        backrest.visual(
            _save_mesh(f"wheelchair_{name}", tube_from_spline_points(points, radius=0.014, samples_per_segment=14, radial_segments=16)),
            material=frame_material,
            name=name,
        )
    backrest.visual(
        Cylinder(radius=0.012, length=0.340),
        origin=Origin(xyz=(0.0, -0.015, 0.110), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="lower_back_cross",
    )
    backrest.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.0, -0.040, 0.410), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="upper_back_cross",
    )
    backrest.visual(
        Box((0.400, 0.012, 0.270)),
        origin=Origin(xyz=(0.0, -0.020, 0.245)),
        material=sling_material,
        name="back_sling",
    )
    backrest.visual(
        Box((0.080, 0.020, 0.060)),
        origin=Origin(xyz=(0.190, -0.040, 0.470)),
        material=pad_material,
        name="left_push_handle_grip",
    )
    backrest.visual(
        Box((0.080, 0.020, 0.060)),
        origin=Origin(xyz=(-0.190, -0.040, 0.470)),
        material=pad_material,
        name="right_push_handle_grip",
    )

    left_footrest = model.part("left_footrest")
    left_footrest.inertial = Inertial.from_geometry(
        Box((0.18, 0.24, 0.38)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.10, -0.19)),
    )
    _build_footrest(
        left_footrest,
        prefix="left_footrest",
        side_sign=1.0,
        frame_material=frame_material,
        pad_material=pad_material,
        marker_material=marker_material,
        knob_material=hardware_material,
    )

    right_footrest = model.part("right_footrest")
    right_footrest.inertial = Inertial.from_geometry(
        Box((0.18, 0.24, 0.38)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.10, -0.19)),
    )
    _build_footrest(
        right_footrest,
        prefix="right_footrest",
        side_sign=-1.0,
        frame_material=frame_material,
        pad_material=pad_material,
        marker_material=marker_material,
        knob_material=hardware_material,
    )

    left_drive_wheel = model.part("left_drive_wheel")
    left_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.060),
        mass=2.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_drive_wheel(
        left_drive_wheel,
        prefix="left_drive_wheel",
        side_sign=1.0,
        rim_material=rim_material,
        tire_material=tire_material,
        hardware_material=hardware_material,
        marker_material=marker_material,
    )

    right_drive_wheel = model.part("right_drive_wheel")
    right_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.060),
        mass=2.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_drive_wheel(
        right_drive_wheel,
        prefix="right_drive_wheel",
        side_sign=-1.0,
        rim_material=rim_material,
        tire_material=tire_material,
        hardware_material=hardware_material,
        marker_material=marker_material,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.080, 0.040, 0.180)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )
    left_caster_fork.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=hardware_material,
        name="swivel_collar",
    )
    left_caster_fork.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=hardware_material,
        name="stem",
    )
    left_caster_fork.visual(
        Box((0.070, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=hardware_material,
        name="crown",
    )
    left_caster_fork.visual(
        Box((0.010, 0.024, 0.082)),
        origin=Origin(xyz=(0.038, 0.0, -0.146)),
        material=hardware_material,
        name="outer_arm",
    )
    left_caster_fork.visual(
        Box((0.010, 0.024, 0.082)),
        origin=Origin(xyz=(-0.038, 0.0, -0.146)),
        material=hardware_material,
        name="inner_arm",
    )
    left_caster_fork.visual(
        Box((0.010, 0.024, 0.024)),
        origin=Origin(xyz=(0.045, 0.0, -0.188)),
        material=hardware_material,
        name="outer_drop_out",
    )
    left_caster_fork.visual(
        Box((0.010, 0.024, 0.024)),
        origin=Origin(xyz=(-0.045, 0.0, -0.188)),
        material=hardware_material,
        name="inner_drop_out",
    )
    left_caster_fork.visual(
        Box((0.020, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.012, -0.014)),
        material=marker_material,
        name="swivel_pointer",
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.080, 0.040, 0.180)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )
    right_caster_fork.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=hardware_material,
        name="swivel_collar",
    )
    right_caster_fork.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=hardware_material,
        name="stem",
    )
    right_caster_fork.visual(
        Box((0.070, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=hardware_material,
        name="crown",
    )
    right_caster_fork.visual(
        Box((0.010, 0.024, 0.082)),
        origin=Origin(xyz=(0.038, 0.0, -0.146)),
        material=hardware_material,
        name="outer_arm",
    )
    right_caster_fork.visual(
        Box((0.010, 0.024, 0.082)),
        origin=Origin(xyz=(-0.038, 0.0, -0.146)),
        material=hardware_material,
        name="inner_arm",
    )
    right_caster_fork.visual(
        Box((0.010, 0.024, 0.024)),
        origin=Origin(xyz=(0.045, 0.0, -0.188)),
        material=hardware_material,
        name="outer_drop_out",
    )
    right_caster_fork.visual(
        Box((0.010, 0.024, 0.024)),
        origin=Origin(xyz=(-0.045, 0.0, -0.188)),
        material=hardware_material,
        name="inner_drop_out",
    )
    right_caster_fork.visual(
        Box((0.020, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.012, -0.014)),
        material=marker_material,
        name="swivel_pointer",
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.038),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(
        left_caster_wheel,
        prefix="left_caster_wheel",
        rim_material=rim_material,
        tire_material=tire_material,
        hardware_material=hardware_material,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.038),
        mass=0.45,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_caster_wheel(
        right_caster_wheel,
        prefix="right_caster_wheel",
        rim_material=rim_material,
        tire_material=tire_material,
        hardware_material=hardware_material,
    )

    model.articulation(
        "main_to_backrest",
        ArticulationType.FIXED,
        parent=main_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.220, 0.540)),
    )
    model.articulation(
        "main_to_left_footrest",
        ArticulationType.FIXED,
        parent=main_frame,
        child=left_footrest,
        origin=Origin(xyz=(0.110, 0.210, 0.490)),
    )
    model.articulation(
        "main_to_right_footrest",
        ArticulationType.FIXED,
        parent=main_frame,
        child=right_footrest,
        origin=Origin(xyz=(-0.110, 0.210, 0.490)),
    )
    model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(0.347, -0.030, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.347, -0.030, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.215, 0.390, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.215, 0.390, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
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

    frame = object_model.get_part("main_frame")
    backrest = object_model.get_part("backrest_frame")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")

    ctx.expect_contact(backrest, frame, name="backrest is physically mounted to frame")
    ctx.expect_contact(left_footrest, frame, name="left footrest is physically mounted to frame")
    ctx.expect_contact(right_footrest, frame, name="right footrest is physically mounted to frame")
    ctx.expect_contact(left_drive_wheel, frame, name="left drive wheel is supported by axle hardware")
    ctx.expect_contact(right_drive_wheel, frame, name="right drive wheel is supported by axle hardware")
    ctx.expect_contact(left_caster_fork, frame, name="left caster swivel stack is supported by frame")
    ctx.expect_contact(right_caster_fork, frame, name="right caster swivel stack is supported by frame")
    ctx.expect_contact(left_caster_wheel, left_caster_fork, name="left caster wheel is carried by fork")
    ctx.expect_contact(right_caster_wheel, right_caster_fork, name="right caster wheel is carried by fork")

    ctx.expect_gap(
        left_drive_wheel,
        frame,
        axis="x",
        positive_elem="tire",
        min_gap=0.003,
        max_gap=0.020,
        name="left drive tire clears frame with controlled side gap",
    )
    ctx.expect_gap(
        frame,
        right_drive_wheel,
        axis="x",
        negative_elem="tire",
        min_gap=0.003,
        max_gap=0.020,
        name="right drive tire clears frame with controlled side gap",
    )
    ctx.expect_origin_gap(
        left_caster_fork,
        left_drive_wheel,
        axis="y",
        min_gap=0.250,
        max_gap=0.450,
        name="front caster stack leads the drive wheel",
    )
    ctx.expect_origin_distance(
        left_footrest,
        right_footrest,
        axes="z",
        max_dist=0.002,
        name="datum footrests stay at matched height",
    )

    with ctx.pose({left_caster_swivel: 0.8, right_caster_swivel: -0.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with casters in steering pose")
        ctx.expect_contact(left_caster_wheel, left_caster_fork, name="left caster remains carried when swiveled")
        ctx.expect_contact(right_caster_wheel, right_caster_fork, name="right caster remains carried when swiveled")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
