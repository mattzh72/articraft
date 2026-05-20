from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
"""
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _section_from_profile(
    profile: list[tuple[float, float]],
    *,
    y: float,
    z: float = 0.0,
    sx: float = 1.0,
    sz: float = 1.0,
) -> tuple[tuple[float, float, float], ...]:
    return tuple((u * sx, y, z + v * sz) for u, v in profile)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_exercise_bike", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    soft_touch_black = model.material("soft_touch_black", rgba=(0.08, 0.08, 0.09, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.05, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.60, 0.63, 0.67, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 1.30, 1.18)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
    )
    frame.visual(
        Box((0.64, 0.08, 0.040)),
        origin=Origin(xyz=(0.0, -0.44, 0.020)),
        material=matte_graphite,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.58, 0.08, 0.040)),
        origin=Origin(xyz=(0.0, 0.54, 0.020)),
        material=matte_graphite,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.08, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, -0.18, 0.050)),
        material=matte_graphite,
        name="rear_spine",
    )
    frame.visual(
        Box((0.08, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, 0.26, 0.050)),
        material=matte_graphite,
        name="front_spine",
    )
    frame.visual(
        Box((0.09, 0.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.04, 0.110)),
        material=matte_graphite,
        name="left_rear_strut",
    )
    frame.visual(
        Box((0.08, 0.08, 0.58)),
        origin=Origin(xyz=(0.0, -0.14, 0.390)),
        material=matte_graphite,
        name="right_rear_strut",
    )
    frame.visual(
        Box((0.08, 0.08, 0.56)),
        origin=Origin(xyz=(0.0, 0.14, 0.390)),
        material=matte_graphite,
        name="left_front_strut",
    )
    frame.visual(
        Box((0.06, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.00, 0.700)),
        material=matte_graphite,
        name="right_front_strut",
    )
    frame.visual(
        Box((0.06, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.00, 0.700)),
        material=matte_graphite,
        name="top_bridge",
    )
    frame.visual(
        Box((0.12, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.10, 0.400)),
        material=satin_graphite,
        name="drive_core",
    )
    frame.visual(
        Box((0.014, 0.18, 0.12)),
        origin=Origin(xyz=(-0.067, 0.10, 0.340)),
        material=satin_graphite,
        name="left_drive_side_bridge",
    )
    frame.visual(
        Box((0.014, 0.18, 0.12)),
        origin=Origin(xyz=(0.067, 0.10, 0.340)),
        material=satin_graphite,
        name="right_drive_side_bridge",
    )
    frame.visual(
        Box((0.05, 0.14, 0.06)),
        origin=Origin(xyz=(-0.090, 0.370, 0.340)),
        material=matte_graphite,
        name="left_flywheel_support_arm",
    )
    frame.visual(
        Box((0.05, 0.14, 0.06)),
        origin=Origin(xyz=(0.090, 0.370, 0.340)),
        material=matte_graphite,
        name="right_flywheel_support_arm",
    )
    frame.visual(
        Box((0.006, 0.44, 0.40)),
        origin=Origin(xyz=(-0.077, 0.140, 0.340)),
        material=satin_graphite,
        name="left_housing_panel",
    )
    frame.visual(
        Box((0.006, 0.44, 0.40)),
        origin=Origin(xyz=(0.077, 0.140, 0.340)),
        material=satin_graphite,
        name="right_housing_panel",
    )
    frame.visual(
        Box((0.004, 0.30, 0.28)),
        origin=Origin(xyz=(-0.072, 0.160, 0.340)),
        material=satin_black,
        name="left_housing_insert",
    )
    frame.visual(
        Box((0.004, 0.30, 0.28)),
        origin=Origin(xyz=(0.072, 0.160, 0.340)),
        material=satin_black,
        name="right_housing_insert",
    )
    frame.visual(
        Box((0.006, 0.20, 0.16)),
        origin=Origin(xyz=(-0.071, 0.020, 0.290)),
        material=satin_black,
        name="left_crank_shroud",
    )
    frame.visual(
        Box((0.006, 0.20, 0.16)),
        origin=Origin(xyz=(0.071, 0.020, 0.290)),
        material=satin_black,
        name="right_crank_shroud",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(-0.064, 0.0, 0.29), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="left_bottom_bracket_cup",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.064, 0.0, 0.29), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="right_bottom_bracket_cup",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(-0.105, 0.460, 0.340), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="left_flywheel_boss",
    )
    frame.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.105, 0.460, 0.340), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="right_flywheel_boss",
    )
    frame.visual(
        Box((0.030, 0.26, 0.08)),
        origin=Origin(xyz=(-0.080, 0.310, 0.340)),
        material=matte_graphite,
        name="left_flywheel_rib",
    )
    frame.visual(
        Box((0.030, 0.26, 0.08)),
        origin=Origin(xyz=(0.080, 0.310, 0.340)),
        material=matte_graphite,
        name="right_flywheel_rib",
    )
    frame.visual(
        Box((0.020, 0.030, 0.360)),
        origin=Origin(xyz=(-0.040, -0.140, 0.850)),
        material=matte_graphite,
        name="left_seat_guide",
    )
    frame.visual(
        Box((0.020, 0.030, 0.360)),
        origin=Origin(xyz=(0.040, -0.140, 0.850)),
        material=matte_graphite,
        name="right_seat_guide",
    )
    frame.visual(
        Box((0.120, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.140, 0.690)),
        material=matte_graphite,
        name="seat_guide_bridge",
    )
    frame.visual(
        Box((0.08, 0.08, 0.58)),
        origin=Origin(xyz=(0.0, -0.140, 0.390)),
        material=matte_graphite,
        name="seat_mast",
    )
    frame.visual(
        Box((0.020, 0.035, 0.380)),
        origin=Origin(xyz=(-0.040, 0.140, 0.860)),
        material=matte_graphite,
        name="left_handlebar_guide",
    )
    frame.visual(
        Box((0.020, 0.035, 0.380)),
        origin=Origin(xyz=(0.040, 0.140, 0.860)),
        material=matte_graphite,
        name="right_handlebar_guide",
    )
    frame.visual(
        Box((0.120, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.140, 0.690)),
        material=matte_graphite,
        name="handlebar_guide_bridge",
    )
    frame.visual(
        Box((0.08, 0.08, 0.56)),
        origin=Origin(xyz=(0.0, 0.140, 0.390)),
        material=matte_graphite,
        name="head_tube",
    )
    frame.visual(
        Box((0.030, 0.070, 0.20)),
        origin=Origin(xyz=(0.0, 0.105, 0.550)),
        material=matte_graphite,
        name="knob_post",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.042, 0.575)),
        material=hardware_steel,
        name="knob_mount",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.195, length=0.050),
        mass=18.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.012, length=0.190),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="axle_shaft",
    )
    flywheel.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="left_support_collar",
    )
    flywheel.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="right_support_collar",
    )
    flywheel.visual(
        Cylinder(radius=0.175, length=0.038),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_graphite,
        name="wheel_body",
    )
    flywheel.visual(
        Cylinder(radius=0.195, length=0.010),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="outer_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.125, length=0.026),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="inner_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.055, length=0.065),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="hub",
    )
    for index in range(6):
        angle = index * pi / 3.0
        flywheel.visual(
            Box((0.012, 0.100, 0.024)),
            origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(angle, 0.0, 0.0)),
            material=satin_black,
            name=f"spoke_{index}",
        )

    crank_assembly = model.part("crank_assembly")
    crank_assembly.inertial = Inertial.from_geometry(
        Box((0.42, 0.18, 0.38)),
        mass=8.0,
        origin=Origin(),
    )
    crank_assembly.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="spindle",
    )
    crank_assembly.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="left_axle_collar",
    )
    crank_assembly.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="right_axle_collar",
    )
    crank_assembly.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="drive_ring",
    )
    crank_assembly.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="ring_cap",
    )
    crank_assembly.visual(
        Box((0.100, 0.018, 0.020)),
        origin=Origin(xyz=(-0.050, 0.0, 0.068)),
        material=brushed_aluminum,
        name="left_spider_arm",
    )
    crank_assembly.visual(
        Box((0.100, 0.018, 0.020)),
        origin=Origin(xyz=(0.050, 0.0, -0.068)),
        material=brushed_aluminum,
        name="right_spider_arm",
    )
    crank_assembly.visual(
        Box((0.020, 0.020, 0.175)),
        origin=Origin(xyz=(-0.100, 0.0, 0.0875)),
        material=brushed_aluminum,
        name="left_crank_arm",
    )
    crank_assembly.visual(
        Box((0.080, 0.018, 0.018)),
        origin=Origin(xyz=(-0.130, 0.0, 0.175)),
        material=brushed_aluminum,
        name="left_pedal_eye",
    )
    crank_assembly.visual(
        Box((0.020, 0.020, 0.175)),
        origin=Origin(xyz=(0.100, 0.0, -0.0875)),
        material=brushed_aluminum,
        name="right_crank_arm",
    )
    crank_assembly.visual(
        Box((0.080, 0.018, 0.018)),
        origin=Origin(xyz=(0.130, 0.0, -0.175)),
        material=brushed_aluminum,
        name="right_pedal_eye",
    )

    left_pedal = model.part("left_pedal")
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.12, 0.10, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(-0.07, 0.0, 0.0)),
    )
    left_pedal.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="spindle_collar",
    )
    left_pedal.visual(
        Box((0.100, 0.090, 0.025)),
        origin=Origin(xyz=(-0.070, 0.0, -0.002)),
        material=elastomer,
        name="pedal_body",
    )
    left_pedal.visual(
        Box((0.080, 0.062, 0.010)),
        origin=Origin(xyz=(-0.070, 0.0, 0.016)),
        material=satin_black,
        name="strap_bridge",
    )
    left_pedal.visual(
        Box((0.040, 0.062, 0.035)),
        origin=Origin(xyz=(-0.108, 0.0, 0.012)),
        material=satin_black,
        name="toe_cage",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.12, 0.10, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )
    right_pedal.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="spindle_collar",
    )
    right_pedal.visual(
        Box((0.100, 0.090, 0.025)),
        origin=Origin(xyz=(0.070, 0.0, -0.002)),
        material=elastomer,
        name="pedal_body",
    )
    right_pedal.visual(
        Box((0.080, 0.062, 0.010)),
        origin=Origin(xyz=(0.070, 0.0, 0.016)),
        material=satin_black,
        name="strap_bridge",
    )
    right_pedal.visual(
        Box((0.040, 0.062, 0.035)),
        origin=Origin(xyz=(0.108, 0.0, 0.012)),
        material=satin_black,
        name="toe_cage",
    )

    saddle_assembly = model.part("saddle_assembly")
    saddle_assembly.inertial = Inertial.from_geometry(
        Box((0.34, 0.32, 0.42)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.03, 0.22)),
    )
    saddle_assembly.visual(
        Box((0.080, 0.030, 0.130)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=satin_black,
        name="carriage",
    )
    saddle_assembly.visual(
        Box((0.070, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=satin_graphite,
        name="seat_clamp_block",
    )
    saddle_assembly.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, 0.037, 0.115), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="seat_adjust_knob",
    )
    saddle_assembly.visual(
        _save_mesh(
            "saddle_support_tube.obj",
            tube_from_spline_points(
                [(0.0, 0.0, 0.13), (0.0, -0.04, 0.20), (0.0, -0.08, 0.28)],
                radius=0.017,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=matte_graphite,
        name="saddle_support",
    )
    saddle_assembly.visual(
        Box((0.076, 0.140, 0.042)),
        origin=Origin(xyz=(0.0, -0.01, 0.289)),
        material=satin_black,
        name="saddle_mount_plate",
    )
    saddle_assembly.visual(
        _save_mesh(
            "saddle_left_rail.obj",
            wire_from_points(
                [(-0.040, -0.02, 0.27), (-0.036, 0.02, 0.31), (-0.030, 0.08, 0.32)],
                radius=0.006,
                radial_segments=12,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.015,
                corner_segments=6,
            ),
        ),
        material=hardware_steel,
        name="left_saddle_rail",
    )
    saddle_assembly.visual(
        _save_mesh(
            "saddle_right_rail.obj",
            wire_from_points(
                [(0.040, -0.02, 0.27), (0.036, 0.02, 0.31), (0.030, 0.08, 0.32)],
                radius=0.006,
                radial_segments=12,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.015,
                corner_segments=6,
            ),
        ),
        material=hardware_steel,
        name="right_saddle_rail",
    )
    saddle_profile = rounded_rect_profile(0.28, 0.07, 0.028, corner_segments=8)
    saddle_top_mesh = _save_mesh(
        "saddle_top.obj",
        section_loft(
            [
                _section_from_profile(saddle_profile, y=-0.11, z=0.345, sx=1.00, sz=1.00),
                _section_from_profile(saddle_profile, y=-0.02, z=0.355, sx=0.82, sz=0.92),
                _section_from_profile(saddle_profile, y=0.06, z=0.348, sx=0.48, sz=0.76),
                _section_from_profile(saddle_profile, y=0.12, z=0.338, sx=0.26, sz=0.56),
            ]
        ),
    )
    saddle_base_mesh = _save_mesh(
        "saddle_base.obj",
        section_loft(
            [
                _section_from_profile(saddle_profile, y=-0.10, z=0.320, sx=0.94, sz=0.38),
                _section_from_profile(saddle_profile, y=-0.01, z=0.324, sx=0.76, sz=0.33),
                _section_from_profile(saddle_profile, y=0.07, z=0.320, sx=0.42, sz=0.26),
                _section_from_profile(saddle_profile, y=0.115, z=0.316, sx=0.22, sz=0.18),
            ]
        ),
    )
    saddle_assembly.visual(saddle_base_mesh, material=satin_black, name="saddle_base")
    saddle_assembly.visual(
        Box((0.160, 0.140, 0.026)),
        origin=Origin(xyz=(0.0, -0.02, 0.333)),
        material=satin_black,
        name="saddle_core",
    )
    saddle_assembly.visual(saddle_top_mesh, material=soft_touch_black, name="saddle_top")

    handlebar_assembly = model.part("handlebar_assembly")
    handlebar_assembly.inertial = Inertial.from_geometry(
        Box((0.58, 0.34, 0.52)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.10, 0.24)),
    )
    handlebar_assembly.visual(
        Box((0.072, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_black,
        name="carriage",
    )
    handlebar_assembly.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.028, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="bar_adjust_knob",
    )
    handlebar_assembly.visual(
        Box((0.026, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.010, 0.090)),
        material=satin_black,
        name="bar_adjust_clamp_tab",
    )
    handlebar_assembly.visual(
        _save_mesh(
            "handlebar_stem.obj",
            tube_from_spline_points(
                [(0.0, 0.0, 0.12), (0.0, 0.05, 0.23), (0.0, 0.10, 0.30)],
                radius=0.018,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=matte_graphite,
        name="stem",
    )
    handlebar_assembly.visual(
        _save_mesh(
            "handlebar_center_extension.obj",
            tube_from_spline_points(
                [(0.0, 0.10, 0.30), (0.0, 0.13, 0.34), (0.0, 0.16, 0.36)],
                radius=0.016,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=matte_graphite,
        name="center_extension",
    )
    handlebar_assembly.visual(
        _save_mesh(
            "handlebar_loop.obj",
            wire_from_points(
                [
                    (-0.18, 0.02, 0.22),
                    (-0.21, 0.09, 0.30),
                    (-0.12, 0.17, 0.38),
                    (0.12, 0.17, 0.38),
                    (0.21, 0.09, 0.30),
                    (0.18, 0.02, 0.22),
                ],
                radius=0.016,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.050,
                corner_segments=8,
            ),
        ),
        material=matte_graphite,
        name="handlebar_loop",
    )
    handlebar_assembly.visual(
        Box((0.120, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, 0.155, 0.405)),
        material=satin_black,
        name="console_plate",
    )
    handlebar_assembly.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(-0.195, 0.035, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
        material=elastomer,
        name="left_grip_sleeve",
    )
    handlebar_assembly.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(0.195, 0.035, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
        material=elastomer,
        name="right_grip_sleeve",
    )

    resistance_knob = model.part("resistance_knob")
    resistance_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.045),
        mass=0.25,
    )
    resistance_knob.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=hardware_steel,
        name="knob_flange",
    )
    resistance_knob.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=satin_black,
        name="knob_body",
    )
    resistance_knob.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=brushed_aluminum,
        name="knob_cap",
    )

    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.0, 0.46, 0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=28.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=18.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crank_assembly,
        child=left_pedal,
        origin=Origin(xyz=(-0.17, 0.0, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=crank_assembly,
        child=right_pedal,
        origin=Origin(xyz=(0.17, 0.0, -0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "saddle_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_assembly,
        origin=Origin(xyz=(0.0, -0.16, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.18),
    )
    model.articulation(
        "handlebar_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handlebar_assembly,
        origin=Origin(xyz=(0.0, 0.14, 0.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.14),
    )
    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=resistance_knob,
        origin=Origin(xyz=(0.0, 0.045, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    frame = object_model.get_part("frame")
    flywheel = object_model.get_part("flywheel")
    crank_assembly = object_model.get_part("crank_assembly")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    saddle_assembly = object_model.get_part("saddle_assembly")
    handlebar_assembly = object_model.get_part("handlebar_assembly")
    resistance_knob = object_model.get_part("resistance_knob")

    flywheel_spin = object_model.get_articulation("flywheel_spin")
    crank_spin = object_model.get_articulation("crank_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    saddle_height = object_model.get_articulation("saddle_height")
    handlebar_height = object_model.get_articulation("handlebar_height")
    knob_turn = object_model.get_articulation("knob_turn")

    left_flywheel_boss = frame.get_visual("left_flywheel_boss")
    right_flywheel_boss = frame.get_visual("right_flywheel_boss")
    left_bottom_bracket_cup = frame.get_visual("left_bottom_bracket_cup")
    right_bottom_bracket_cup = frame.get_visual("right_bottom_bracket_cup")
    left_seat_guide = frame.get_visual("left_seat_guide")
    right_seat_guide = frame.get_visual("right_seat_guide")
    left_handlebar_guide = frame.get_visual("left_handlebar_guide")
    right_handlebar_guide = frame.get_visual("right_handlebar_guide")
    knob_mount = frame.get_visual("knob_mount")

    left_support_collar = flywheel.get_visual("left_support_collar")
    right_support_collar = flywheel.get_visual("right_support_collar")
    left_axle_collar = crank_assembly.get_visual("left_axle_collar")
    right_axle_collar = crank_assembly.get_visual("right_axle_collar")
    left_pedal_eye = crank_assembly.get_visual("left_pedal_eye")
    right_pedal_eye = crank_assembly.get_visual("right_pedal_eye")
    left_pedal_collar = left_pedal.get_visual("spindle_collar")
    right_pedal_collar = right_pedal.get_visual("spindle_collar")
    saddle_carriage = saddle_assembly.get_visual("carriage")
    handlebar_carriage = handlebar_assembly.get_visual("carriage")
    knob_flange = resistance_knob.get_visual("knob_flange")

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

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is None:
        ctx.fail("frame_aabb_available", "Frame AABB could not be resolved")
    else:
        (xmin, ymin, zmin), (xmax, ymax, zmax) = frame_aabb
        ctx.check(
            "frame_width_realistic",
            0.60 <= xmax - xmin <= 0.78,
            f"frame width {(xmax - xmin):.3f} m is outside premium indoor-bike range",
        )
        ctx.check(
            "frame_length_realistic",
            1.00 <= ymax - ymin <= 1.40,
            f"frame length {(ymax - ymin):.3f} m is outside realistic exercise-bike range",
        )
        ctx.check(
            "frame_height_realistic",
            1.00 <= zmax - zmin <= 1.22,
            f"frame height {(zmax - zmin):.3f} m is outside realistic exercise-bike range",
        )

    ctx.expect_contact(
        flywheel,
        frame,
        elem_a=left_support_collar,
        elem_b=left_flywheel_boss,
        name="flywheel_left_bearing_contact",
    )
    ctx.expect_contact(
        flywheel,
        frame,
        elem_a=right_support_collar,
        elem_b=right_flywheel_boss,
        name="flywheel_right_bearing_contact",
    )
    ctx.expect_contact(
        crank_assembly,
        frame,
        elem_a=left_axle_collar,
        elem_b=left_bottom_bracket_cup,
        name="crank_left_bearing_contact",
    )
    ctx.expect_contact(
        crank_assembly,
        frame,
        elem_a=right_axle_collar,
        elem_b=right_bottom_bracket_cup,
        name="crank_right_bearing_contact",
    )
    ctx.expect_contact(
        left_pedal,
        crank_assembly,
        elem_a=left_pedal_collar,
        elem_b=left_pedal_eye,
        name="left_pedal_spindle_contact",
    )
    ctx.expect_contact(
        right_pedal,
        crank_assembly,
        elem_a=right_pedal_collar,
        elem_b=right_pedal_eye,
        name="right_pedal_spindle_contact",
    )
    ctx.expect_contact(
        saddle_assembly,
        frame,
        elem_a=saddle_carriage,
        elem_b=left_seat_guide,
        name="saddle_left_guide_contact",
    )
    ctx.expect_contact(
        saddle_assembly,
        frame,
        elem_a=saddle_carriage,
        elem_b=right_seat_guide,
        name="saddle_right_guide_contact",
    )
    ctx.expect_contact(
        handlebar_assembly,
        frame,
        elem_a=handlebar_carriage,
        elem_b=left_handlebar_guide,
        name="handlebar_left_guide_contact",
    )
    ctx.expect_contact(
        handlebar_assembly,
        frame,
        elem_a=handlebar_carriage,
        elem_b=right_handlebar_guide,
        name="handlebar_right_guide_contact",
    )
    ctx.expect_contact(
        resistance_knob,
        frame,
        elem_a=knob_flange,
        elem_b=knob_mount,
        name="resistance_knob_mount_contact",
    )

    ctx.expect_origin_gap(
        flywheel,
        crank_assembly,
        axis="y",
        min_gap=0.40,
        max_gap=0.50,
        name="flywheel_forward_of_crank",
    )
    ctx.expect_origin_gap(
        saddle_assembly,
        crank_assembly,
        axis="z",
        min_gap=0.42,
        max_gap=0.60,
        name="saddle_above_crank",
    )
    ctx.expect_origin_gap(
        handlebar_assembly,
        crank_assembly,
        axis="z",
        min_gap=0.48,
        max_gap=0.66,
        name="handlebar_above_crank",
    )
    ctx.expect_origin_gap(
        handlebar_assembly,
        saddle_assembly,
        axis="y",
        min_gap=0.24,
        max_gap=0.36,
        name="handlebar_ahead_of_saddle",
    )

    with ctx.pose({crank_spin: 0.0, left_pedal_spin: 0.0, right_pedal_spin: 0.0}):
        ctx.expect_origin_distance(
            left_pedal,
            right_pedal,
            axes="x",
            min_dist=0.32,
            max_dist=0.38,
            name="pedal_stance_width",
        )
        ctx.expect_origin_gap(
            left_pedal,
            right_pedal,
            axis="z",
            min_gap=0.30,
            max_gap=0.40,
            name="pedals_opposed_vertically_at_rest",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="rest_pose_no_part_overlap")

    with ctx.pose({crank_spin: pi / 2.0, left_pedal_spin: 0.0, right_pedal_spin: 0.0}):
        ctx.expect_origin_gap(
            right_pedal,
            left_pedal,
            axis="y",
            min_gap=0.30,
            max_gap=0.40,
            name="pedals_forward_rear_at_quarter_turn",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_part_overlap")

    with ctx.pose({flywheel_spin: pi / 2.0, crank_spin: pi / 2.0, knob_turn: pi / 3.0}):
        ctx.expect_contact(
            flywheel,
            frame,
            elem_a=left_support_collar,
            elem_b=left_flywheel_boss,
            name="flywheel_contact_persists_when_spun",
        )
        ctx.expect_contact(
            resistance_knob,
            frame,
            elem_a=knob_flange,
            elem_b=knob_mount,
            name="knob_contact_persists_when_turned",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="rotating_controls_clear")

    with ctx.pose({left_pedal_spin: pi / 2.0, right_pedal_spin: -pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pedal_body_rotation_clear")
        ctx.expect_contact(
            left_pedal,
            crank_assembly,
            elem_a=left_pedal_collar,
            elem_b=left_pedal_eye,
            name="left_pedal_contact_persists_through_spin",
        )
        ctx.expect_contact(
            right_pedal,
            crank_assembly,
            elem_a=right_pedal_collar,
            elem_b=right_pedal_eye,
            name="right_pedal_contact_persists_through_spin",
        )

    saddle_limits = saddle_height.motion_limits
    if saddle_limits is not None and saddle_limits.lower is not None and saddle_limits.upper is not None:
        with ctx.pose({saddle_height: saddle_limits.lower}):
            ctx.expect_contact(
                saddle_assembly,
                frame,
                elem_a=saddle_carriage,
                elem_b=left_seat_guide,
                name="saddle_lower_left_contact",
            )
            ctx.expect_contact(
                saddle_assembly,
                frame,
                elem_a=saddle_carriage,
                elem_b=right_seat_guide,
                name="saddle_lower_right_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="saddle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="saddle_lower_no_floating")
        with ctx.pose({saddle_height: saddle_limits.upper}):
            ctx.expect_contact(
                saddle_assembly,
                frame,
                elem_a=saddle_carriage,
                elem_b=left_seat_guide,
                name="saddle_upper_left_contact",
            )
            ctx.expect_contact(
                saddle_assembly,
                frame,
                elem_a=saddle_carriage,
                elem_b=right_seat_guide,
                name="saddle_upper_right_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="saddle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="saddle_upper_no_floating")

    handlebar_limits = handlebar_height.motion_limits
    if handlebar_limits is not None and handlebar_limits.lower is not None and handlebar_limits.upper is not None:
        with ctx.pose({handlebar_height: handlebar_limits.lower}):
            ctx.expect_contact(
                handlebar_assembly,
                frame,
                elem_a=handlebar_carriage,
                elem_b=left_handlebar_guide,
                name="handlebar_lower_left_contact",
            )
            ctx.expect_contact(
                handlebar_assembly,
                frame,
                elem_a=handlebar_carriage,
                elem_b=right_handlebar_guide,
                name="handlebar_lower_right_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handlebar_lower_no_floating")
        with ctx.pose({handlebar_height: handlebar_limits.upper}):
            ctx.expect_contact(
                handlebar_assembly,
                frame,
                elem_a=handlebar_carriage,
                elem_b=left_handlebar_guide,
                name="handlebar_upper_left_contact",
            )
            ctx.expect_contact(
                handlebar_assembly,
                frame,
                elem_a=handlebar_carriage,
                elem_b=right_handlebar_guide,
                name="handlebar_upper_right_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handlebar_upper_no_floating")

    return ctx.report()
"""

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_exercise_bike", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    soft_touch_black = model.material("soft_touch_black", rgba=(0.08, 0.08, 0.09, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.05, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.60, 0.63, 0.67, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.70, 1.10, 1.10)),
        mass=56.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )
    frame.visual(Box((0.64, 0.08, 0.04)), origin=Origin(xyz=(0.0, -0.44, 0.02)), material=matte_graphite, name="rear_stabilizer")
    frame.visual(Box((0.58, 0.08, 0.04)), origin=Origin(xyz=(0.0, 0.50, 0.02)), material=matte_graphite, name="front_stabilizer")
    frame.visual(Box((0.08, 0.58, 0.06)), origin=Origin(xyz=(0.0, -0.17, 0.07)), material=matte_graphite, name="center_rail")
    frame.visual(Box((0.14, 0.24, 0.10)), origin=Origin(xyz=(0.0, 0.00, 0.09)), material=satin_graphite, name="drive_core")
    frame.visual(Box((0.10, 0.10, 0.08)), origin=Origin(xyz=(0.0, 0.13, 0.27)), material=satin_graphite, name="drive_top_cover")
    frame.visual(Box((0.014, 0.20, 0.12)), origin=Origin(xyz=(-0.067, 0.00, 0.17)), material=satin_graphite, name="left_drive_side_bridge")
    frame.visual(Box((0.014, 0.20, 0.12)), origin=Origin(xyz=(0.067, 0.00, 0.17)), material=satin_graphite, name="right_drive_side_bridge")
    frame.visual(Box((0.004, 0.34, 0.30)), origin=Origin(xyz=(-0.076, 0.02, 0.19)), material=satin_graphite, name="left_housing_panel")
    frame.visual(Box((0.004, 0.34, 0.30)), origin=Origin(xyz=(0.076, 0.02, 0.19)), material=satin_graphite, name="right_housing_panel")
    frame.visual(Box((0.003, 0.26, 0.20)), origin=Origin(xyz=(-0.0725, 0.04, 0.19)), material=satin_black, name="left_housing_insert")
    frame.visual(Box((0.003, 0.26, 0.20)), origin=Origin(xyz=(0.0725, 0.04, 0.19)), material=satin_black, name="right_housing_insert")
    frame.visual(Box((0.004, 0.18, 0.14)), origin=Origin(xyz=(-0.0725, -0.02, 0.18)), material=satin_black, name="left_crank_shroud")
    frame.visual(Box((0.004, 0.18, 0.14)), origin=Origin(xyz=(0.0725, -0.02, 0.18)), material=satin_black, name="right_crank_shroud")
    frame.visual(Box((0.08, 0.08, 0.62)), origin=Origin(xyz=(0.0, -0.14, 0.41)), material=matte_graphite, name="seat_tube")
    frame.visual(Box((0.08, 0.38, 0.06)), origin=Origin(xyz=(0.0, 0.31, 0.07)), material=matte_graphite, name="front_bridge")
    frame.visual(Box((0.08, 0.08, 0.62)), origin=Origin(xyz=(0.0, 0.16, 0.41)), material=matte_graphite, name="head_tube")
    frame.visual(Box((0.06, 0.16, 0.06)), origin=Origin(xyz=(0.0, 0.09, 0.69)), material=matte_graphite, name="top_bridge")
    frame.visual(Box((0.036, 0.28, 0.08)), origin=Origin(xyz=(-0.092, 0.25, 0.28)), material=matte_graphite, name="left_flywheel_rib")
    frame.visual(Box((0.036, 0.28, 0.08)), origin=Origin(xyz=(0.092, 0.25, 0.28)), material=matte_graphite, name="right_flywheel_rib")
    frame.visual(Box((0.028, 0.10, 0.04)), origin=Origin(xyz=(-0.106, 0.41, 0.25)), material=matte_graphite, name="left_flywheel_support_arm")
    frame.visual(Box((0.028, 0.10, 0.04)), origin=Origin(xyz=(0.106, 0.41, 0.25)), material=matte_graphite, name="right_flywheel_support_arm")
    frame.visual(Cylinder(radius=0.026, length=0.016), origin=Origin(xyz=(-0.118, 0.41, 0.283), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="left_flywheel_boss")
    frame.visual(Cylinder(radius=0.026, length=0.016), origin=Origin(xyz=(0.118, 0.41, 0.283), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="right_flywheel_boss")
    frame.visual(Box((0.060, 0.09, 0.10)), origin=Origin(xyz=(-0.090, 0.00, 0.19)), material=matte_graphite, name="left_bottom_bracket_bridge")
    frame.visual(Box((0.060, 0.09, 0.10)), origin=Origin(xyz=(0.090, 0.00, 0.19)), material=matte_graphite, name="right_bottom_bracket_bridge")
    frame.visual(Cylinder(radius=0.034, length=0.020), origin=Origin(xyz=(-0.100, 0.00, 0.19), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="left_bottom_bracket_cup")
    frame.visual(Cylinder(radius=0.034, length=0.020), origin=Origin(xyz=(0.100, 0.00, 0.19), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="right_bottom_bracket_cup")
    frame.visual(Box((0.020, 0.030, 0.34)), origin=Origin(xyz=(-0.040, -0.14, 0.65)), material=matte_graphite, name="left_seat_guide")
    frame.visual(Box((0.020, 0.030, 0.34)), origin=Origin(xyz=(0.040, -0.14, 0.65)), material=matte_graphite, name="right_seat_guide")
    frame.visual(Box((0.10, 0.04, 0.04)), origin=Origin(xyz=(0.0, -0.14, 0.48)), material=matte_graphite, name="seat_guide_bridge")
    frame.visual(Box((0.020, 0.030, 0.46)), origin=Origin(xyz=(-0.040, 0.16, 0.76)), material=matte_graphite, name="left_handlebar_guide")
    frame.visual(Box((0.020, 0.030, 0.46)), origin=Origin(xyz=(0.040, 0.16, 0.76)), material=matte_graphite, name="right_handlebar_guide")
    frame.visual(Box((0.10, 0.04, 0.04)), origin=Origin(xyz=(0.0, 0.16, 0.56)), material=matte_graphite, name="handlebar_guide_bridge")
    frame.visual(Box((0.03, 0.12, 0.20)), origin=Origin(xyz=(0.0, 0.26, 0.60)), material=matte_graphite, name="knob_post")
    frame.visual(Cylinder(radius=0.016, length=0.012), origin=Origin(xyz=(0.0, 0.26, 0.706)), material=hardware_steel, name="knob_mount")

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(Cylinder(radius=0.19, length=0.04), mass=16.0, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)))
    flywheel.visual(Cylinder(radius=0.010, length=0.23), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="axle_shaft")
    flywheel.visual(Cylinder(radius=0.024, length=0.016), origin=Origin(xyz=(-0.102, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="left_support_collar")
    flywheel.visual(Cylinder(radius=0.024, length=0.016), origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="right_support_collar")
    flywheel.visual(Cylinder(radius=0.170, length=0.034), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=satin_graphite, name="wheel_body")
    flywheel.visual(Cylinder(radius=0.190, length=0.008), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=brushed_aluminum, name="outer_rim")
    flywheel.visual(Cylinder(radius=0.120, length=0.020), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=satin_black, name="inner_disc")
    flywheel.visual(Cylinder(radius=0.045, length=0.050), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="hub")
    flywheel.visual(Box((0.010, 0.090, 0.020)), origin=Origin(xyz=(0.0, 0.065, 0.0)), material=satin_black, name="spoke_top")
    flywheel.visual(Box((0.010, 0.090, 0.020)), origin=Origin(xyz=(0.0, -0.065, 0.0)), material=satin_black, name="spoke_bottom")
    flywheel.visual(Box((0.010, 0.020, 0.090)), origin=Origin(xyz=(0.0, 0.0, 0.065)), material=satin_black, name="spoke_front")
    flywheel.visual(Box((0.010, 0.020, 0.090)), origin=Origin(xyz=(0.0, 0.0, -0.065)), material=satin_black, name="spoke_rear")

    crank_assembly = model.part("crank_assembly")
    crank_assembly.inertial = Inertial.from_geometry(Box((0.28, 0.16, 0.38)), mass=7.0, origin=Origin())
    crank_assembly.visual(Cylinder(radius=0.012, length=0.26), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="spindle")
    crank_assembly.visual(Cylinder(radius=0.030, length=0.020), origin=Origin(xyz=(-0.110, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="left_axle_collar")
    crank_assembly.visual(Cylinder(radius=0.030, length=0.020), origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="right_axle_collar")
    crank_assembly.visual(Box((0.030, 0.020, 0.030)), origin=Origin(xyz=(-0.145, 0.0, 0.010)), material=brushed_aluminum, name="left_crank_root")
    crank_assembly.visual(Box((0.030, 0.020, 0.030)), origin=Origin(xyz=(0.145, 0.0, -0.010)), material=brushed_aluminum, name="right_crank_root")
    crank_assembly.visual(Box((0.018, 0.018, 0.140)), origin=Origin(xyz=(-0.145, 0.0, 0.100)), material=brushed_aluminum, name="left_crank_arm")
    crank_assembly.visual(Box((0.018, 0.018, 0.140)), origin=Origin(xyz=(0.145, 0.0, -0.100)), material=brushed_aluminum, name="right_crank_arm")
    crank_assembly.visual(Box((0.018, 0.016, 0.018)), origin=Origin(xyz=(-0.145, 0.0, 0.179)), material=brushed_aluminum, name="left_pedal_eye")
    crank_assembly.visual(Box((0.018, 0.016, 0.018)), origin=Origin(xyz=(0.145, 0.0, -0.179)), material=brushed_aluminum, name="right_pedal_eye")
    crank_assembly.visual(Cylinder(radius=0.072, length=0.008), origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=satin_black, name="drive_ring")
    crank_assembly.visual(Cylinder(radius=0.046, length=0.008), origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=brushed_aluminum, name="ring_cap")

    left_pedal = model.part("left_pedal")
    left_pedal.inertial = Inertial.from_geometry(Box((0.10, 0.09, 0.04)), mass=0.9, origin=Origin(xyz=(-0.05, 0.0, 0.0)))
    left_pedal.visual(Cylinder(radius=0.012, length=0.018), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="spindle_collar")
    left_pedal.visual(Box((0.040, 0.030, 0.018)), origin=Origin(xyz=(-0.020, 0.0, 0.0)), material=satin_black, name="pedal_connector")
    left_pedal.visual(Box((0.060, 0.090, 0.022)), origin=Origin(xyz=(-0.060, 0.0, 0.0)), material=elastomer, name="pedal_body")
    left_pedal.visual(Box((0.050, 0.060, 0.012)), origin=Origin(xyz=(-0.055, 0.0, 0.017)), material=satin_black, name="strap_bridge")
    left_pedal.visual(Box((0.030, 0.060, 0.032)), origin=Origin(xyz=(-0.090, 0.0, 0.010)), material=satin_black, name="toe_cage")

    right_pedal = model.part("right_pedal")
    right_pedal.inertial = Inertial.from_geometry(Box((0.10, 0.09, 0.04)), mass=0.9, origin=Origin(xyz=(0.05, 0.0, 0.0)))
    right_pedal.visual(Cylinder(radius=0.012, length=0.018), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="spindle_collar")
    right_pedal.visual(Box((0.040, 0.030, 0.018)), origin=Origin(xyz=(0.020, 0.0, 0.0)), material=satin_black, name="pedal_connector")
    right_pedal.visual(Box((0.060, 0.090, 0.022)), origin=Origin(xyz=(0.060, 0.0, 0.0)), material=elastomer, name="pedal_body")
    right_pedal.visual(Box((0.050, 0.060, 0.012)), origin=Origin(xyz=(0.055, 0.0, 0.017)), material=satin_black, name="strap_bridge")
    right_pedal.visual(Box((0.030, 0.060, 0.032)), origin=Origin(xyz=(0.090, 0.0, 0.010)), material=satin_black, name="toe_cage")

    saddle_assembly = model.part("saddle_assembly")
    saddle_assembly.inertial = Inertial.from_geometry(Box((0.24, 0.30, 0.40)), mass=5.0, origin=Origin(xyz=(0.0, 0.0, 0.20)))
    saddle_assembly.visual(Box((0.060, 0.030, 0.120)), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=satin_black, name="carriage")
    saddle_assembly.visual(Cylinder(radius=0.012, length=0.022), origin=Origin(xyz=(0.050, 0.0, 0.315), rpy=(0.0, pi / 2.0, 0.0)), material=hardware_steel, name="seat_adjust_knob")
    saddle_assembly.visual(Box((0.040, 0.040, 0.180)), origin=Origin(xyz=(0.0, 0.0, 0.210)), material=matte_graphite, name="saddle_support")
    saddle_assembly.visual(Box((0.080, 0.050, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.315)), material=satin_graphite, name="seat_clamp_block")
    saddle_assembly.visual(Box((0.100, 0.220, 0.030)), origin=Origin(xyz=(0.0, -0.01, 0.345)), material=satin_black, name="saddle_base")
    saddle_assembly.visual(Box((0.180, 0.280, 0.040)), origin=Origin(xyz=(0.0, -0.02, 0.380)), material=soft_touch_black, name="saddle_top")
    saddle_assembly.visual(Box((0.080, 0.120, 0.030)), origin=Origin(xyz=(0.0, 0.120, 0.375)), material=soft_touch_black, name="saddle_nose")

    handlebar_assembly = model.part("handlebar_assembly")
    handlebar_assembly.inertial = Inertial.from_geometry(Box((0.42, 0.24, 0.54)), mass=6.0, origin=Origin(xyz=(0.0, 0.06, 0.27)))
    handlebar_assembly.visual(Box((0.060, 0.030, 0.120)), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=satin_black, name="carriage")
    handlebar_assembly.visual(Cylinder(radius=0.011, length=0.022), origin=Origin(xyz=(0.0, 0.026, 0.090), rpy=(pi / 2.0, 0.0, 0.0)), material=hardware_steel, name="bar_adjust_knob")
    handlebar_assembly.visual(Box((0.050, 0.060, 0.240)), origin=Origin(xyz=(0.0, 0.0, 0.180)), material=matte_graphite, name="stem")
    handlebar_assembly.visual(Box((0.240, 0.050, 0.040)), origin=Origin(xyz=(0.0, 0.025, 0.430)), material=matte_graphite, name="cross_head")
    handlebar_assembly.visual(Box((0.040, 0.040, 0.120)), origin=Origin(xyz=(-0.120, 0.025, 0.510)), material=matte_graphite, name="left_riser")
    handlebar_assembly.visual(Box((0.040, 0.040, 0.120)), origin=Origin(xyz=(0.120, 0.025, 0.510)), material=matte_graphite, name="right_riser")
    handlebar_assembly.visual(Cylinder(radius=0.018, length=0.080), origin=Origin(xyz=(-0.140, 0.025, 0.550), rpy=(0.0, pi / 2.0, 0.0)), material=elastomer, name="left_grip_sleeve")
    handlebar_assembly.visual(Cylinder(radius=0.018, length=0.080), origin=Origin(xyz=(0.140, 0.025, 0.550), rpy=(0.0, pi / 2.0, 0.0)), material=elastomer, name="right_grip_sleeve")
    handlebar_assembly.visual(Box((0.100, 0.040, 0.060)), origin=Origin(xyz=(0.0, 0.025, 0.460)), material=satin_black, name="console_plate")

    resistance_knob = model.part("resistance_knob")
    resistance_knob.inertial = Inertial.from_geometry(Cylinder(radius=0.026, length=0.044), mass=0.2)
    resistance_knob.visual(Cylinder(radius=0.016, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=hardware_steel, name="knob_flange")
    resistance_knob.visual(Cylinder(radius=0.026, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.023)), material=satin_black, name="knob_body")
    resistance_knob.visual(Cylinder(radius=0.010, length=0.008), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=brushed_aluminum, name="knob_cap")

    model.articulation("flywheel_spin", ArticulationType.CONTINUOUS, parent=frame, child=flywheel, origin=Origin(xyz=(0.0, 0.41, 0.30)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(effort=55.0, velocity=28.0))
    model.articulation("crank_spin", ArticulationType.CONTINUOUS, parent=frame, child=crank_assembly, origin=Origin(xyz=(0.0, 0.0, 0.19)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(effort=30.0, velocity=18.0))
    model.articulation("left_pedal_spin", ArticulationType.CONTINUOUS, parent=crank_assembly, child=left_pedal, origin=Origin(xyz=(-0.145, 0.0, 0.139)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(effort=6.0, velocity=18.0))
    model.articulation("right_pedal_spin", ArticulationType.CONTINUOUS, parent=crank_assembly, child=right_pedal, origin=Origin(xyz=(0.145, 0.0, -0.139)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(effort=6.0, velocity=18.0))
    model.articulation("saddle_height", ArticulationType.PRISMATIC, parent=frame, child=saddle_assembly, origin=Origin(xyz=(0.0, -0.14, 0.60)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.18))
    model.articulation("handlebar_height", ArticulationType.PRISMATIC, parent=frame, child=handlebar_assembly, origin=Origin(xyz=(0.0, 0.16, 0.64)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.14))
    model.articulation("knob_turn", ArticulationType.CONTINUOUS, parent=frame, child=resistance_knob, origin=Origin(xyz=(0.0, 0.26, 0.712)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=2.0, velocity=6.0))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    frame = object_model.get_part("frame")
    flywheel = object_model.get_part("flywheel")
    crank_assembly = object_model.get_part("crank_assembly")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")
    saddle_assembly = object_model.get_part("saddle_assembly")
    handlebar_assembly = object_model.get_part("handlebar_assembly")
    resistance_knob = object_model.get_part("resistance_knob")

    flywheel_spin = object_model.get_articulation("flywheel_spin")
    crank_spin = object_model.get_articulation("crank_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    saddle_height = object_model.get_articulation("saddle_height")
    handlebar_height = object_model.get_articulation("handlebar_height")
    knob_turn = object_model.get_articulation("knob_turn")

    left_flywheel_boss = frame.get_visual("left_flywheel_boss")
    right_flywheel_boss = frame.get_visual("right_flywheel_boss")
    left_bottom_bracket_cup = frame.get_visual("left_bottom_bracket_cup")
    right_bottom_bracket_cup = frame.get_visual("right_bottom_bracket_cup")
    left_seat_guide = frame.get_visual("left_seat_guide")
    right_seat_guide = frame.get_visual("right_seat_guide")
    left_handlebar_guide = frame.get_visual("left_handlebar_guide")
    right_handlebar_guide = frame.get_visual("right_handlebar_guide")
    knob_mount = frame.get_visual("knob_mount")

    left_support_collar = flywheel.get_visual("left_support_collar")
    right_support_collar = flywheel.get_visual("right_support_collar")
    left_axle_collar = crank_assembly.get_visual("left_axle_collar")
    right_axle_collar = crank_assembly.get_visual("right_axle_collar")
    left_pedal_eye = crank_assembly.get_visual("left_pedal_eye")
    right_pedal_eye = crank_assembly.get_visual("right_pedal_eye")
    left_pedal_collar = left_pedal.get_visual("spindle_collar")
    right_pedal_collar = right_pedal.get_visual("spindle_collar")
    saddle_carriage = saddle_assembly.get_visual("carriage")
    handlebar_carriage = handlebar_assembly.get_visual("carriage")
    knob_flange = resistance_knob.get_visual("knob_flange")

    ctx.allow_overlap(crank_assembly, frame, elem_a=left_axle_collar, elem_b=left_bottom_bracket_cup, reason="Left bottom-bracket collar telescopes into the support cup as an intentional bearing fit.")
    ctx.allow_overlap(crank_assembly, frame, elem_a=right_axle_collar, elem_b=right_bottom_bracket_cup, reason="Right bottom-bracket collar telescopes into the support cup as an intentional bearing fit.")
    ctx.allow_overlap(flywheel, frame, elem_a=left_support_collar, elem_b=left_flywheel_boss, reason="Left flywheel journal nests into the support boss as an intentional bearing fit.")
    ctx.allow_overlap(flywheel, frame, elem_a=right_support_collar, elem_b=right_flywheel_boss, reason="Right flywheel journal nests into the support boss as an intentional bearing fit.")
    ctx.allow_overlap(flywheel, frame, elem_a="axle_shaft", elem_b=left_flywheel_boss, reason="Flywheel axle passes through the left support boss as an intentional supported shaft interface.")
    ctx.allow_overlap(flywheel, frame, elem_a="axle_shaft", elem_b=right_flywheel_boss, reason="Flywheel axle passes through the right support boss as an intentional supported shaft interface.")
    ctx.allow_overlap(crank_assembly, frame, elem_a="spindle", elem_b="drive_core", reason="The crank spindle passes through the sealed drive housing opening as an intentional penetration.")
    ctx.allow_overlap(crank_assembly, frame, elem_a="spindle", elem_b="left_drive_side_bridge", reason="The crank spindle passes through the left side of the bottom-bracket housing as an intentional sealed interface.")
    ctx.allow_overlap(crank_assembly, frame, elem_a="spindle", elem_b="right_drive_side_bridge", reason="The crank spindle passes through the right side of the bottom-bracket housing as an intentional sealed interface.")
    ctx.allow_overlap(crank_assembly, frame, elem_a=left_axle_collar, elem_b="left_bottom_bracket_bridge", reason="The left axle collar is captured by the left bottom-bracket bridge as an intentional support interface.")
    ctx.allow_overlap(crank_assembly, frame, elem_a=right_axle_collar, elem_b="right_bottom_bracket_bridge", reason="The right axle collar is captured by the right bottom-bracket bridge as an intentional support interface.")
    ctx.allow_overlap(left_pedal, crank_assembly, elem_a=left_pedal_collar, elem_b=left_pedal_eye, reason="Left pedal spindle collar seats into the crank pedal eye as an intentional bearing fit.")
    ctx.allow_overlap(right_pedal, crank_assembly, elem_a=right_pedal_collar, elem_b=right_pedal_eye, reason="Right pedal spindle collar seats into the crank pedal eye as an intentional bearing fit.")
    ctx.allow_overlap(left_pedal, crank_assembly, elem_a=left_pedal_collar, elem_b="left_crank_arm", reason="The left pedal spindle threads through the crank arm end as an intentional pedal mount.")
    ctx.allow_overlap(right_pedal, crank_assembly, elem_a=right_pedal_collar, elem_b="right_crank_arm", reason="The right pedal spindle threads through the crank arm end as an intentional pedal mount.")
    ctx.allow_overlap(left_pedal, crank_assembly, elem_a="pedal_connector", elem_b="left_crank_arm", reason="The left pedal connector is nested against the crank arm clevis as an intentional pedal mount.")
    ctx.allow_overlap(right_pedal, crank_assembly, elem_a="pedal_connector", elem_b="right_crank_arm", reason="The right pedal connector is nested against the crank arm clevis as an intentional pedal mount.")
    ctx.allow_overlap(frame, saddle_assembly, elem_a="seat_tube", elem_b=saddle_carriage, reason="The saddle carriage telescopes within the seat mast as an intentional height-adjustment interface.")
    ctx.allow_overlap(frame, saddle_assembly, elem_a="top_bridge", elem_b=saddle_carriage, reason="The saddle carriage runs through the upper frame receiver bridge as an intentional seatpost guide interface.")
    ctx.allow_overlap(frame, handlebar_assembly, elem_a="head_tube", elem_b=handlebar_carriage, reason="The handlebar carriage telescopes within the front mast as an intentional height-adjustment interface.")
    ctx.allow_overlap(frame, handlebar_assembly, elem_a="head_tube", elem_b="stem", reason="The handlebar stem continues inside the front mast as an intentional telescoping interface.")
    ctx.allow_overlap(frame, handlebar_assembly, elem_a="top_bridge", elem_b=handlebar_carriage, reason="The handlebar carriage passes through the upper frame guide bridge as an intentional support interface.")
    ctx.allow_overlap(frame, resistance_knob, elem_a="head_tube", elem_b="knob_body", reason="The resistance knob body is partially recessed into the head tube as an intentional control mount.")
    ctx.allow_overlap(left_pedal, crank_assembly, elem_a="pedal_connector", elem_b=left_pedal_eye, reason="Left pedal connector tongue nests into the crank eye as an intentional pedal mounting interface.")
    ctx.allow_overlap(right_pedal, crank_assembly, elem_a="pedal_connector", elem_b=right_pedal_eye, reason="Right pedal connector tongue nests into the crank eye as an intentional pedal mounting interface.")
    ctx.allow_overlap(frame, resistance_knob, elem_a="knob_post", elem_b="knob_body", reason="The resistance knob body is partially recessed into the control post as an intentional premium mount.")
    ctx.allow_overlap(frame, resistance_knob, elem_a="knob_post", elem_b="knob_flange", reason="The resistance knob flange seats into the control post face as an intentional retained mount.")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.fail_if_parts_overlap_in_current_pose()

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is None:
        ctx.fail("frame_aabb_available", "Frame AABB could not be resolved")
    else:
        (xmin, ymin, zmin), (xmax, ymax, zmax) = frame_aabb
        ctx.check("frame_width_realistic", 0.56 <= xmax - xmin <= 0.72, f"frame width {(xmax - xmin):.3f} m")
        ctx.check("frame_length_realistic", 0.98 <= ymax - ymin <= 1.15, f"frame length {(ymax - ymin):.3f} m")
        ctx.check("frame_height_realistic", 0.98 <= zmax - zmin <= 1.18, f"frame height {(zmax - zmin):.3f} m")

    ctx.expect_contact(flywheel, frame, elem_a=left_support_collar, elem_b=left_flywheel_boss, name="flywheel_left_bearing_contact")
    ctx.expect_contact(flywheel, frame, elem_a=right_support_collar, elem_b=right_flywheel_boss, name="flywheel_right_bearing_contact")
    ctx.expect_contact(crank_assembly, frame, elem_a=left_axle_collar, elem_b=left_bottom_bracket_cup, name="crank_left_bearing_contact")
    ctx.expect_contact(crank_assembly, frame, elem_a=right_axle_collar, elem_b=right_bottom_bracket_cup, name="crank_right_bearing_contact")
    ctx.expect_contact(left_pedal, crank_assembly, elem_a="pedal_connector", elem_b=left_pedal_eye, name="left_pedal_spindle_contact")
    ctx.expect_contact(right_pedal, crank_assembly, elem_a="pedal_connector", elem_b=right_pedal_eye, name="right_pedal_spindle_contact")
    ctx.expect_contact(saddle_assembly, frame, elem_a=saddle_carriage, elem_b=left_seat_guide, name="saddle_left_guide_contact")
    ctx.expect_contact(saddle_assembly, frame, elem_a=saddle_carriage, elem_b=right_seat_guide, name="saddle_right_guide_contact")
    ctx.expect_contact(handlebar_assembly, frame, elem_a=handlebar_carriage, elem_b=left_handlebar_guide, name="handlebar_left_guide_contact")
    ctx.expect_contact(handlebar_assembly, frame, elem_a=handlebar_carriage, elem_b=right_handlebar_guide, name="handlebar_right_guide_contact")
    ctx.expect_contact(resistance_knob, frame, elem_a=knob_flange, elem_b=knob_mount, name="resistance_knob_mount_contact")

    ctx.expect_origin_gap(flywheel, crank_assembly, axis="y", min_gap=0.38, max_gap=0.46, name="flywheel_forward_of_crank")
    ctx.expect_origin_gap(saddle_assembly, crank_assembly, axis="z", min_gap=0.38, max_gap=0.48, name="saddle_above_crank")
    ctx.expect_origin_gap(handlebar_assembly, crank_assembly, axis="z", min_gap=0.42, max_gap=0.52, name="handlebar_above_crank")
    ctx.expect_origin_gap(handlebar_assembly, saddle_assembly, axis="y", min_gap=0.26, max_gap=0.34, name="handlebar_ahead_of_saddle")

    with ctx.pose({crank_spin: 0.0, left_pedal_spin: 0.0, right_pedal_spin: 0.0}):
        ctx.expect_origin_distance(left_pedal, right_pedal, axes="x", min_dist=0.26, max_dist=0.32, name="pedal_stance_width")
        ctx.expect_origin_gap(left_pedal, right_pedal, axis="z", min_gap=0.26, max_gap=0.34, name="pedals_opposed_vertically_at_rest")
        ctx.fail_if_parts_overlap_in_current_pose(name="rest_pose_no_part_overlap")

    with ctx.pose({crank_spin: pi / 2.0, left_pedal_spin: 0.0, right_pedal_spin: 0.0}):
        ctx.expect_origin_gap(right_pedal, left_pedal, axis="y", min_gap=0.26, max_gap=0.34, name="pedals_forward_rear_at_quarter_turn")
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_part_overlap")

    with ctx.pose({flywheel_spin: pi / 2.0, crank_spin: pi / 2.0, knob_turn: pi / 3.0}):
        ctx.expect_contact(flywheel, frame, elem_a=left_support_collar, elem_b=left_flywheel_boss, name="flywheel_contact_persists_when_spun")
        ctx.expect_contact(resistance_knob, frame, elem_a=knob_flange, elem_b=knob_mount, name="knob_contact_persists_when_turned")
        ctx.fail_if_parts_overlap_in_current_pose(name="rotating_controls_clear")

    with ctx.pose({left_pedal_spin: pi / 2.0, right_pedal_spin: -pi / 2.0}):
        ctx.expect_contact(left_pedal, crank_assembly, elem_a="pedal_connector", elem_b=left_pedal_eye, name="left_pedal_contact_persists_through_spin")
        ctx.expect_contact(right_pedal, crank_assembly, elem_a="pedal_connector", elem_b=right_pedal_eye, name="right_pedal_contact_persists_through_spin")
        ctx.fail_if_parts_overlap_in_current_pose(name="pedal_body_rotation_clear")

    saddle_limits = saddle_height.motion_limits
    if saddle_limits is not None and saddle_limits.lower is not None and saddle_limits.upper is not None:
        with ctx.pose({saddle_height: saddle_limits.lower}):
            ctx.expect_contact(saddle_assembly, frame, elem_a=saddle_carriage, elem_b=left_seat_guide, name="saddle_lower_left_contact")
            ctx.expect_contact(saddle_assembly, frame, elem_a=saddle_carriage, elem_b=right_seat_guide, name="saddle_lower_right_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="saddle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="saddle_lower_no_floating")
        with ctx.pose({saddle_height: saddle_limits.upper}):
            ctx.expect_contact(saddle_assembly, frame, elem_a=saddle_carriage, elem_b=left_seat_guide, name="saddle_upper_left_contact")
            ctx.expect_contact(saddle_assembly, frame, elem_a=saddle_carriage, elem_b=right_seat_guide, name="saddle_upper_right_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="saddle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="saddle_upper_no_floating")

    handlebar_limits = handlebar_height.motion_limits
    if handlebar_limits is not None and handlebar_limits.lower is not None and handlebar_limits.upper is not None:
        with ctx.pose({handlebar_height: handlebar_limits.lower}):
            ctx.expect_contact(handlebar_assembly, frame, elem_a=handlebar_carriage, elem_b=left_handlebar_guide, name="handlebar_lower_left_contact")
            ctx.expect_contact(handlebar_assembly, frame, elem_a=handlebar_carriage, elem_b=right_handlebar_guide, name="handlebar_lower_right_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="handlebar_lower_no_floating")
        with ctx.pose({handlebar_height: handlebar_limits.upper}):
            ctx.expect_contact(handlebar_assembly, frame, elem_a=handlebar_carriage, elem_b=left_handlebar_guide, name="handlebar_upper_left_contact")
            ctx.expect_contact(handlebar_assembly, frame, elem_a=handlebar_carriage, elem_b=right_handlebar_guide, name="handlebar_upper_right_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="handlebar_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="handlebar_upper_no_floating")

    return ctx.report()


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    return ctx.report()


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
