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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _translated_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    segments: int = 40,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (dx + radius * cos((2.0 * pi * index) / segments), dy + radius * sin((2.0 * pi * index) / segments))
        for index in range(segments)
    ]


def _tray_section(
    length: float,
    width: float,
    z: float,
    *,
    x_shift: float,
    corner_radius: float,
    corner_segments: int = 8,
) -> tuple[tuple[float, float, float], ...]:
    profile = rounded_rect_profile(length, width, corner_radius, corner_segments=corner_segments)
    return tuple((x + x_shift, y, z) for x, y in profile)


def _build_tray_shell_mesh(
    sections: list[tuple[tuple[float, float, float], ...]],
    *,
    name: str,
):
    spec = SectionLoftSpec(
        sections=tuple(LoftSection(points=section) for section in sections),
        cap=False,
        solid=False,
    )
    return _save_mesh(name, section_loft(spec))


def _build_tire_mesh(*, radius: float, width: float, name: str):
    half_width = width * 0.5
    profile = [
        (radius * 0.48, -half_width * 0.98),
        (radius * 0.72, -half_width),
        (radius * 0.90, -half_width * 0.82),
        (radius * 0.98, -half_width * 0.44),
        (radius, -half_width * 0.12),
        (radius, half_width * 0.12),
        (radius * 0.98, half_width * 0.44),
        (radius * 0.90, half_width * 0.82),
        (radius * 0.72, half_width),
        (radius * 0.48, half_width * 0.98),
        (radius * 0.40, half_width * 0.32),
        (radius * 0.36, 0.0),
        (radius * 0.40, -half_width * 0.32),
        (radius * 0.48, -half_width * 0.98),
    ]
    return _save_mesh(name, LatheGeometry(profile, segments=64).rotate_x(pi / 2.0))


def _build_wheel_disc_mesh(name: str):
    outer = _circle_profile(0.145, segments=44)
    holes: list[list[tuple[float, float]]] = [_circle_profile(0.040, segments=24)]
    for hole_index in range(5):
        angle = (2.0 * pi * hole_index) / 5.0
        holes.append(
            _circle_profile(
                0.022,
                segments=20,
                dx=0.076 * cos(angle),
                dy=0.076 * sin(angle),
            )
        )
    return _save_mesh(name, ExtrudeWithHolesGeometry(outer, holes, height=0.005, center=True).rotate_x(pi / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_wheelbarrow")

    frame_powder = model.material("frame_powder", rgba=(0.19, 0.21, 0.22, 1.0))
    tray_coat = model.material("tray_coat", rgba=(0.37, 0.43, 0.30, 1.0))
    tray_shadow = model.material("tray_shadow", rgba=(0.30, 0.35, 0.26, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    zinc = model.material("zinc", rgba=(0.67, 0.69, 0.71, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.13, 0.13, 0.13, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.52, 0.64, 0.62)),
        mass=15.0,
        origin=Origin(xyz=(0.00, 0.00, 0.31)),
    )

    rail_radius = 0.017
    left_main_rail_pts = [
        (-0.74, 0.27, 0.56),
        (-0.61, 0.25, 0.47),
        (-0.50, 0.23, 0.38),
        (-0.40, 0.21, 0.295),
        (-0.20, 0.19, 0.280),
        (0.05, 0.17, 0.278),
        (0.32, 0.12, 0.255),
        (0.56, 0.08, 0.27),
    ]
    frame.visual(
        _save_mesh(
            "wheelbarrow_left_main_rail",
            tube_from_spline_points(left_main_rail_pts, radius=rail_radius, samples_per_segment=16, radial_segments=18),
        ),
        material=frame_powder,
        name="left_main_rail",
    )
    frame.visual(
        _save_mesh(
            "wheelbarrow_right_main_rail",
            tube_from_spline_points(_mirror_y(left_main_rail_pts), radius=rail_radius, samples_per_segment=16, radial_segments=18),
        ),
        material=frame_powder,
        name="right_main_rail",
    )

    left_leg_pts = [
        (-0.20, 0.19, 0.280),
        (-0.29, 0.195, 0.16),
        (-0.385, 0.20, 0.018),
    ]
    frame.visual(
        _save_mesh(
            "wheelbarrow_left_leg",
            tube_from_spline_points(left_leg_pts, radius=0.016, samples_per_segment=10, radial_segments=16),
        ),
        material=frame_powder,
        name="left_leg",
    )
    frame.visual(
        _save_mesh(
            "wheelbarrow_right_leg",
            tube_from_spline_points(_mirror_y(left_leg_pts), radius=0.016, samples_per_segment=10, radial_segments=16),
        ),
        material=frame_powder,
        name="right_leg",
    )

    left_fork_pts = [
        (0.56, 0.08, 0.27),
        (0.55, 0.078, 0.43),
        (0.64, 0.070, 0.34),
        (0.69, 0.060, 0.205),
    ]
    frame.visual(
        _save_mesh(
            "wheelbarrow_left_fork",
            tube_from_spline_points(left_fork_pts, radius=0.015, samples_per_segment=10, radial_segments=16),
        ),
        material=frame_powder,
        name="left_fork_blade",
    )
    frame.visual(
        _save_mesh(
            "wheelbarrow_right_fork",
            tube_from_spline_points(_mirror_y(left_fork_pts), radius=0.015, samples_per_segment=10, radial_segments=16),
        ),
        material=frame_powder,
        name="right_fork_blade",
    )

    frame.visual(
        Cylinder(radius=0.019, length=0.165),
        origin=Origin(xyz=(0.55, 0.0, 0.432), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_powder,
        name="fork_crown",
    )
    frame.visual(
        Box((0.18, 0.34, 0.020)),
        origin=Origin(xyz=(0.03, 0.0, 0.286)),
        material=frame_powder,
        name="mid_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.46),
        origin=Origin(xyz=(-0.50, 0.0, 0.382), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_powder,
        name="rear_handle_brace",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(-0.31, 0.0, 0.155), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_powder,
        name="leg_cross_brace",
    )

    frame.visual(
        Box((0.11, 0.40, 0.008)),
        origin=Origin(xyz=(-0.08, 0.0, 0.286)),
        material=zinc,
        name="rear_saddle_plate",
    )
    frame.visual(
        Box((0.11, 0.40, 0.004)),
        origin=Origin(xyz=(-0.08, 0.0, 0.290)),
        material=gasket_rubber,
        name="rear_gasket_strip",
    )
    frame.visual(
        Box((0.07, 0.18, 0.008)),
        origin=Origin(xyz=(0.14, 0.0, 0.286)),
        material=zinc,
        name="front_saddle_plate",
    )
    frame.visual(
        Box((0.07, 0.18, 0.004)),
        origin=Origin(xyz=(0.14, 0.0, 0.290)),
        material=gasket_rubber,
        name="front_gasket_strip",
    )

    frame.visual(
        Box((0.10, 0.035, 0.020)),
        origin=Origin(xyz=(-0.40, 0.20, 0.010)),
        material=stainless,
        name="left_rear_foot",
    )
    frame.visual(
        Box((0.10, 0.035, 0.020)),
        origin=Origin(xyz=(-0.40, -0.20, 0.010)),
        material=stainless,
        name="right_rear_foot",
    )

    frame.visual(
        Cylinder(radius=0.021, length=0.18),
        origin=Origin(xyz=(-0.75, 0.27, 0.56), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.18),
        origin=Origin(xyz=(-0.75, -0.27, 0.56), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    frame.visual(
        Box((0.03, 0.020, 0.090)),
        origin=Origin(xyz=(0.69, 0.055, 0.205)),
        material=frame_powder,
        name="left_dropout",
    )
    frame.visual(
        Box((0.03, 0.020, 0.090)),
        origin=Origin(xyz=(0.69, -0.055, 0.205)),
        material=frame_powder,
        name="right_dropout",
    )
    frame.visual(
        Cylinder(radius=0.027, length=0.020),
        origin=Origin(xyz=(0.695, 0.055, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_bearing_cap",
    )
    frame.visual(
        Cylinder(radius=0.027, length=0.020),
        origin=Origin(xyz=(0.695, -0.055, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_bearing_cap",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.90, 0.66, 0.24)),
        mass=11.0,
        origin=Origin(xyz=(0.03, 0.0, 0.41)),
    )

    outer_sections = [
        _tray_section(0.70, 0.42, 0.302, x_shift=0.02, corner_radius=0.038),
        _tray_section(0.78, 0.54, 0.400, x_shift=0.04, corner_radius=0.048),
        _tray_section(0.86, 0.66, 0.510, x_shift=0.05, corner_radius=0.060),
    ]
    inner_sections = [
        _tray_section(0.686, 0.406, 0.310, x_shift=0.02, corner_radius=0.032),
        _tray_section(0.762, 0.522, 0.400, x_shift=0.04, corner_radius=0.042),
        _tray_section(0.812, 0.608, 0.504, x_shift=0.05, corner_radius=0.052),
    ]
    tray.visual(
        _build_tray_shell_mesh(outer_sections, name="wheelbarrow_tray_outer_shell"),
        material=tray_coat,
        name="tray_outer_shell",
    )
    tray.visual(
        _build_tray_shell_mesh(inner_sections, name="wheelbarrow_tray_inner_shell"),
        material=tray_shadow,
        name="tray_inner_shell",
    )

    rim_outer = _translated_profile(rounded_rect_profile(0.86, 0.66, 0.060, corner_segments=8), dx=0.05)
    rim_inner = _translated_profile(rounded_rect_profile(0.812, 0.608, 0.052, corner_segments=8), dx=0.05)
    tray.visual(
        _save_mesh(
            "wheelbarrow_tray_rim",
            ExtrudeWithHolesGeometry(rim_outer, [rim_inner], height=0.010, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.509)),
        material=tray_coat,
        name="tray_rim",
    )
    tray.visual(
        Box((0.70, 0.42, 0.008)),
        origin=Origin(xyz=(0.02, 0.0, 0.306)),
        material=tray_coat,
        name="tray_floor",
    )
    tray.visual(
        Box((0.016, 0.54, 0.030)),
        origin=Origin(xyz=(0.475, 0.0, 0.492)),
        material=tray_coat,
        name="front_drip_lip",
    )
    tray.visual(
        Box((0.11, 0.11, 0.010)),
        origin=Origin(xyz=(-0.08, 0.0, 0.297)),
        material=zinc,
        name="rear_mount_pad",
    )
    tray.visual(
        Box((0.07, 0.11, 0.010)),
        origin=Origin(xyz=(0.14, 0.0, 0.297)),
        material=zinc,
        name="front_mount_pad",
    )
    tray.visual(
        Box((0.14, 0.07, 0.028)),
        origin=Origin(xyz=(0.33, 0.0, 0.288)),
        material=tray_shadow,
        name="nose_stiffener",
    )

    for mount_name, mount_x in (("rear", -0.08), ("front", 0.14)):
        for side_index, mount_y in enumerate((-0.07, 0.07)):
            tray.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(mount_x, mount_y, 0.312), rpy=(0.0, 0.0, 0.0)),
                material=stainless,
                name=f"{mount_name}_bolt_{side_index}",
            )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.205, length=0.082),
        mass=4.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    tire_mesh = _build_tire_mesh(radius=0.205, width=0.082, name="wheelbarrow_tire")
    disc_mesh = _build_wheel_disc_mesh("wheelbarrow_disc")
    wheel.visual(tire_mesh, material=tire_rubber, name="tire")
    wheel.visual(
        Cylinder(radius=0.125, length=0.056),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="rim_barrel",
    )
    wheel.visual(
        disc_mesh,
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=zinc,
        name="left_disc",
    )
    wheel.visual(
        disc_mesh,
        origin=Origin(xyz=(0.0, -0.019, 0.0)),
        material=zinc,
        name="right_disc",
    )
    wheel.visual(
        Cylinder(radius=0.046, length=0.060),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_powder,
        name="hub_shell",
    )
    wheel.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, 0.037, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hub_cap_left",
    )
    wheel.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hub_cap_right",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.69, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("frame_to_wheel")

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

    ctx.check(
        "parts_present",
        frame is not None and tray is not None and wheel is not None,
        "Frame, tray, and wheel must all exist.",
    )
    limits = wheel_spin.motion_limits
    ctx.check(
        "wheel_spin_joint_is_continuous_y_axis",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (0.0, 1.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        "Wheel should spin continuously about the transverse axle axis.",
    )

    ctx.expect_contact(
        tray,
        frame,
        elem_a="rear_mount_pad",
        elem_b="rear_gasket_strip",
        name="rear_mount_pad_seals_to_frame",
    )
    ctx.expect_contact(
        tray,
        frame,
        elem_a="front_mount_pad",
        elem_b="front_gasket_strip",
        name="front_mount_pad_seals_to_frame",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="hub_cap_left",
        elem_b="left_bearing_cap",
        name="left_hub_seated_in_fork",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="hub_cap_right",
        elem_b="right_bearing_cap",
        name="right_hub_seated_in_fork",
    )
    ctx.expect_gap(
        wheel,
        tray,
        axis="x",
        min_gap=0.003,
        positive_elem="tire",
        negative_elem="tray_outer_shell",
        name="wheel_clears_tray_front",
    )

    left_foot_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_foot")
    right_foot_aabb = ctx.part_element_world_aabb(frame, elem="right_rear_foot")
    tire_aabb = ctx.part_element_world_aabb(wheel, elem="tire")
    stance_ok = False
    if left_foot_aabb is not None and right_foot_aabb is not None and tire_aabb is not None:
        left_foot_min, left_foot_max = left_foot_aabb
        right_foot_min, right_foot_max = right_foot_aabb
        tire_min, _ = tire_aabb
        rear_feet_level = abs(left_foot_min[2] - tire_min[2]) <= 0.02 and abs(right_foot_min[2] - tire_min[2]) <= 0.02
        rear_feet_behind_wheel = left_foot_max[0] < tire_min[0] and right_foot_max[0] < tire_min[0]
        stance_ok = rear_feet_level and rear_feet_behind_wheel
    ctx.check(
        "three_point_stance_is_readable",
        stance_ok,
        "Rear feet should sit near the wheel contact plane and clearly behind the axle.",
    )

    with ctx.pose({wheel_spin: 1.4}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="hub_cap_left",
            elem_b="left_bearing_cap",
            name="left_hub_remains_seated_when_spun",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="hub_cap_right",
            elem_b="right_bearing_cap",
            name="right_hub_remains_seated_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
