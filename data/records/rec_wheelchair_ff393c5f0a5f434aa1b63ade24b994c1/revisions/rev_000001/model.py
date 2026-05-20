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


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples: int = 12,
    radial_segments: int = 16,
):
    return _save_mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
    )


def _torus_mesh(name: str, major_radius: float, tube_radius: float):
    return _save_mesh(
        name,
        TorusGeometry(
            radius=major_radius,
            tube=tube_radius,
            radial_segments=18,
            tubular_segments=48,
        ).rotate_x(pi / 2.0),
    )


def _add_rear_wheel_visuals(part, prefix: str, *, side_sign: float, tire_material, rim_material, pushrim_material):
    part.visual(
        _torus_mesh(f"{prefix}_tire", 0.291, 0.014),
        material=tire_material,
        name="tire",
    )
    part.visual(
        _torus_mesh(f"{prefix}_rim", 0.278, 0.010),
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.034),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="hub",
    )
    for spoke_index in range(12):
        angle = spoke_index * (pi / 6.0)
        part.visual(
            Box((0.270, 0.005, 0.005)),
            origin=Origin(
                xyz=(0.135 * cos(angle), 0.0, 0.135 * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=rim_material,
            name=f"spoke_{spoke_index}",
        )
    pushrim_y = side_sign * 0.040
    part.visual(
        _torus_mesh(f"{prefix}_pushrim", 0.278, 0.005),
        origin=Origin(xyz=(0.0, pushrim_y, 0.0)),
        material=pushrim_material,
        name="pushrim",
    )
    for standoff_index in range(4):
        angle = standoff_index * (pi / 2.0) + (pi / 4.0)
        part.visual(
            Cylinder(radius=0.0038, length=abs(pushrim_y)),
            origin=Origin(
                xyz=(0.278 * cos(angle), pushrim_y * 0.5, 0.278 * sin(angle)),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=pushrim_material,
            name=f"pushrim_standoff_{standoff_index}",
        )


def _add_caster_wheel_visuals(part, *, tire_material, hub_material):
    part.visual(
        _torus_mesh(f"{part.name}_caster_tire", 0.070, 0.015),
        material=tire_material,
        name="caster_tire",
    )
    part.visual(
        Cylinder(radius=0.055, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="caster_hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_gray = model.material("frame_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    upholstery = model.material("upholstery", rgba=(0.08, 0.09, 0.10, 1.0))
    footplate_dark = model.material("footplate_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    caster_dark = model.material("caster_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 1.00)),
        mass=16.0,
        origin=Origin(xyz=(0.08, 0.0, 0.50)),
    )

    left_lower_rail_pts = [
        (-0.20, 0.23, 0.47),
        (-0.03, 0.23, 0.47),
        (0.14, 0.24, 0.45),
        (0.25, 0.245, 0.36),
        (0.31, 0.25, 0.24),
    ]
    frame.visual(
        _tube_mesh("wheelchair_left_lower_rail", left_lower_rail_pts, radius=0.012),
        material=frame_gray,
        name="left_lower_rail",
    )
    frame.visual(
        _tube_mesh("wheelchair_right_lower_rail", _mirror_y(left_lower_rail_pts), radius=0.012),
        material=frame_gray,
        name="right_lower_rail",
    )

    left_back_post_pts = [
        (-0.03, 0.23, 0.47),
        (-0.12, 0.24, 0.60),
        (-0.19, 0.25, 0.77),
        (-0.22, 0.26, 0.92),
    ]
    frame.visual(
        _tube_mesh("wheelchair_left_back_post", left_back_post_pts, radius=0.012, samples=10),
        material=frame_gray,
        name="left_back_post",
    )
    frame.visual(
        _tube_mesh("wheelchair_right_back_post", _mirror_y(left_back_post_pts), radius=0.012, samples=10),
        material=frame_gray,
        name="right_back_post",
    )

    left_armrest_pts = [
        (-0.12, 0.25, 0.60),
        (0.01, 0.25, 0.585),
        (0.13, 0.25, 0.575),
        (0.19, 0.25, 0.56),
    ]
    frame.visual(
        _tube_mesh("wheelchair_left_armrest", left_armrest_pts, radius=0.011, samples=10),
        material=frame_gray,
        name="left_armrest_rail",
    )
    frame.visual(
        _tube_mesh("wheelchair_right_armrest", _mirror_y(left_armrest_pts), radius=0.011, samples=10),
        material=frame_gray,
        name="right_armrest_rail",
    )

    left_front_upright_pts = [
        (0.18, 0.25, 0.56),
        (0.17, 0.245, 0.50),
        (0.15, 0.24, 0.44),
    ]
    frame.visual(
        _tube_mesh("wheelchair_left_front_upright", left_front_upright_pts, radius=0.011, samples=8),
        material=frame_gray,
        name="left_front_upright",
    )
    frame.visual(
        _tube_mesh("wheelchair_right_front_upright", _mirror_y(left_front_upright_pts), radius=0.011, samples=8),
        material=frame_gray,
        name="right_front_upright",
    )
    left_footrest_receiver_pts = [
        (0.160, 0.243, 0.448),
        (0.182, 0.205, 0.445),
        (0.210, 0.150, 0.440),
    ]
    frame.visual(
        _tube_mesh("wheelchair_left_footrest_receiver", left_footrest_receiver_pts, radius=0.010, samples=8),
        material=frame_gray,
        name="left_footrest_receiver",
    )
    frame.visual(
        _tube_mesh(
            "wheelchair_right_footrest_receiver",
            _mirror_y(left_footrest_receiver_pts),
            radius=0.010,
            samples=8,
        ),
        material=frame_gray,
        name="right_footrest_receiver",
    )

    left_rear_brace_pts = [
        (-0.17, 0.22, 0.54),
        (-0.10, 0.22, 0.44),
        (-0.02, 0.22, 0.34),
    ]
    frame.visual(
        _tube_mesh("wheelchair_left_rear_brace", left_rear_brace_pts, radius=0.010, samples=8),
        material=frame_gray,
        name="left_rear_brace",
    )
    frame.visual(
        _tube_mesh("wheelchair_right_rear_brace", _mirror_y(left_rear_brace_pts), radius=0.010, samples=8),
        material=frame_gray,
        name="right_rear_brace",
    )

    frame.visual(
        Cylinder(radius=0.015, length=0.48),
        origin=Origin(xyz=(-0.02, 0.0, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="rear_axle_tube",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.46),
        origin=Origin(xyz=(0.16, 0.0, 0.46), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="front_seat_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.42),
        origin=Origin(xyz=(-0.04, 0.0, 0.47), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="rear_seat_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.36),
        origin=Origin(xyz=(-0.19, 0.0, 0.74), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="backrest_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.52),
        origin=Origin(xyz=(0.31, 0.0, 0.24), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="front_frame_spreader",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.50),
        origin=Origin(xyz=(-0.14, 0.0, 0.60), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="lower_backrest_cross_tube",
    )

    frame.visual(
        Box((0.38, 0.43, 0.016)),
        origin=Origin(xyz=(0.05, 0.0, 0.490)),
        material=upholstery,
        name="seat_sling",
    )
    frame.visual(
        Box((0.025, 0.50, 0.30)),
        origin=Origin(xyz=(-0.19, 0.0, 0.70)),
        material=upholstery,
        name="backrest_panel",
    )

    for side_sign, prefix in ((1.0, "left"), (-1.0, "right")):
        frame.visual(
            Box((0.12, 0.10, 0.012)),
            origin=Origin(xyz=(0.22, side_sign * 0.25, 0.575)),
            material=upholstery,
            name=f"{prefix}_arm_pad",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(xyz=(0.31, side_sign * 0.25, 0.220)),
            material=caster_dark,
            name=f"{prefix}_caster_sleeve",
        )
        frame.visual(
            Cylinder(radius=0.013, length=0.080),
            origin=Origin(xyz=(-0.02, side_sign * 0.275, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
            material=frame_gray,
            name=f"{prefix}_rear_axle_stub",
        )

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        footrest = model.part(f"{side_name}_footrest")
        footrest.inertial = Inertial.from_geometry(
            Box((0.18, 0.12, 0.34)),
            mass=1.2,
            origin=Origin(xyz=(0.06, 0.0, -0.17)),
        )
        footrest.visual(
            Box((0.05, 0.04, 0.020)),
            material=frame_gray,
            name="mount_block",
        )
        footrest.visual(
            Cylinder(radius=0.010, length=0.26),
            origin=Origin(xyz=(0.0, 0.0, -0.14)),
            material=frame_gray,
            name="hanger_tube",
        )
        footrest.visual(
            _tube_mesh(
                f"wheelchair_{side_name}_footrest_tube",
                [(0.0, 0.0, -0.27), (0.04, 0.0, -0.29), (0.10, 0.0, -0.31)],
                radius=0.010,
                samples=6,
                radial_segments=12,
            ),
            material=frame_gray,
            name="support_tube",
        )
        footrest.visual(
            Box((0.12, 0.10, 0.012)),
            origin=Origin(xyz=(0.12, 0.0, -0.315)),
            material=footplate_dark,
            name="footplate",
        )
        model.articulation(
            f"frame_to_{side_name}_footrest",
            ArticulationType.FIXED,
            parent=frame,
            child=footrest,
            origin=Origin(xyz=(0.21, side_sign * 0.13, 0.44)),
        )

    rear_wheel_tire = model.material("rear_wheel_tire", rgba=(0.06, 0.06, 0.07, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    pushrim_silver = model.material("pushrim_silver", rgba=(0.83, 0.84, 0.86, 1.0))

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        rear_wheel = model.part(f"rear_{side_name}_wheel")
        rear_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.305, length=0.030),
            mass=2.4,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        _add_rear_wheel_visuals(
            rear_wheel,
            f"wheelchair_rear_{side_name}_wheel",
            side_sign=side_sign,
            tire_material=rear_wheel_tire,
            rim_material=rim_silver,
            pushrim_material=pushrim_silver,
        )
        model.articulation(
            f"rear_{side_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=rear_wheel,
            origin=Origin(xyz=(-0.02, side_sign * 0.33, 0.34)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
        )

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        caster_swivel = model.part(f"{side_name}_caster_swivel")
        caster_swivel.inertial = Inertial.from_geometry(
            Box((0.06, 0.08, 0.24)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.0, -0.12)),
        )
        caster_swivel.visual(
            Cylinder(radius=0.009, length=0.12),
            origin=Origin(xyz=(0.0, 0.0, -0.06)),
            material=frame_gray,
            name="stem",
        )
        caster_swivel.visual(
            Box((0.035, 0.060, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.105)),
            material=frame_gray,
            name="fork_crown",
        )
        caster_swivel.visual(
            Box((0.012, 0.012, 0.110)),
            origin=Origin(xyz=(0.0, 0.024, -0.160)),
            material=frame_gray,
            name="fork_blade_outer",
        )
        caster_swivel.visual(
            Box((0.012, 0.012, 0.110)),
            origin=Origin(xyz=(0.0, -0.024, -0.160)),
            material=frame_gray,
            name="fork_blade_inner",
        )
        caster_wheel = model.part(f"{side_name}_caster_wheel")
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.085, length=0.035),
            mass=0.55,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        _add_caster_wheel_visuals(caster_wheel, tire_material=caster_dark, hub_material=rim_silver)

        model.articulation(
            f"{side_name}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster_swivel,
            origin=Origin(xyz=(0.31, side_sign * 0.25, 0.21)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=8.0),
        )
        model.articulation(
            f"{side_name}_caster_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster_swivel,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.210)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_footrest = object_model.get_part("left_footrest")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    ctx.expect_gap(
        frame,
        frame,
        axis="z",
        positive_elem="seat_sling",
        negative_elem="front_seat_cross_tube",
        max_gap=0.04,
        max_penetration=0.01,
        name="seat sling stays supported by the front cross tube",
    )
    ctx.expect_gap(
        left_footrest,
        frame,
        axis="x",
        positive_elem="footplate",
        negative_elem="backrest_panel",
        min_gap=0.44,
        name="footrests project forward of the backrest",
    )
    ctx.expect_origin_gap(
        rear_left_wheel,
        frame,
        axis="y",
        min_gap=0.30,
        name="left rear wheel sits outside the frame side rail",
    )
    ctx.expect_origin_gap(
        frame,
        rear_right_wheel,
        axis="y",
        min_gap=0.30,
        name="right rear wheel sits outside the frame side rail",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
