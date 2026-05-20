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
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * step / segments),
            radius * sin(2.0 * pi * step / segments),
        )
        for step in range(segments)
    ]


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oval_condiment_lazy_susan")

    tray_glaze = model.material("tray_glaze", rgba=(0.95, 0.93, 0.89, 1.0))
    walnut = model.material("walnut", rgba=(0.37, 0.26, 0.18, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.76, 0.78, 0.80, 1.0))

    tray_length = 0.38
    tray_width = 0.24
    tray_floor_thickness = 0.006
    tray_rim_width = 0.020
    tray_rim_height = 0.032

    base_length = 0.23
    base_width = 0.15
    base_foot_height = 0.012
    base_cap_height = 0.010
    pedestal_base_z = base_foot_height + base_cap_height
    pedestal_height = 0.036
    disc_radius = 0.052
    disc_thickness = 0.008
    disc_top_z = pedestal_base_z + pedestal_height + disc_thickness

    lower_base = model.part("lower_base")
    lower_base.inertial = Inertial.from_geometry(
        Box((base_length, base_width, disc_top_z)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, disc_top_z * 0.5)),
    )

    base_profile = superellipse_profile(base_length, base_width, exponent=2.5, segments=64)
    base_cap_profile = superellipse_profile(0.18, 0.11, exponent=2.6, segments=64)
    base_shell = ExtrudeGeometry.from_z0(base_profile, base_foot_height)
    base_shell.merge(ExtrudeGeometry.from_z0(base_cap_profile, base_cap_height).translate(0.0, 0.0, base_foot_height))

    pedestal_profile = [
        (0.044, 0.0),
        (0.042, 0.004),
        (0.036, 0.020),
        (0.038, 0.030),
        (0.043, pedestal_height),
    ]
    pedestal_shell = LatheGeometry(pedestal_profile, segments=56).translate(0.0, 0.0, pedestal_base_z)
    base_shell.merge(pedestal_shell)

    lower_base.visual(_save_mesh("lazy_susan_base_shell", base_shell), material=walnut, name="base_shell")
    lower_base.visual(
        Cylinder(radius=disc_radius, length=disc_thickness),
        origin=Origin(xyz=(0.0, 0.0, disc_top_z - disc_thickness * 0.5)),
        material=brushed_metal,
        name="turntable_disc",
    )

    upper_tray = model.part("upper_tray")
    upper_tray.inertial = Inertial.from_geometry(
        Box((tray_length, tray_width, 0.055)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    outer_tray_profile = superellipse_profile(tray_length, tray_width, exponent=2.3, segments=72)
    inner_tray_profile = superellipse_profile(
        tray_length - 2.0 * tray_rim_width,
        tray_width - 2.0 * tray_rim_width,
        exponent=2.45,
        segments=72,
    )
    tray_floor = ExtrudeGeometry.from_z0(outer_tray_profile, tray_floor_thickness)

    tray_shell = ExtrudeWithHolesGeometry(
        outer_tray_profile,
        [inner_tray_profile],
        tray_rim_height,
        center=False,
    ).translate(0.0, 0.0, tray_floor_thickness - 0.0005)

    collar_outer_radius = 0.067
    collar_inner_radius = 0.054
    collar_height = 0.011
    tray_shell.merge(
        ExtrudeWithHolesGeometry(
            _circle_profile(collar_outer_radius, segments=56),
            [_circle_profile(collar_inner_radius, segments=56)],
            collar_height,
            center=False,
        ).translate(0.0, 0.0, -collar_height + 0.0005)
    )

    handle_radius = 0.0045
    left_handle_points = [
        (-0.184, -0.050, 0.028),
        (-0.212, -0.028, 0.037),
        (-0.222, 0.000, 0.044),
        (-0.212, 0.028, 0.037),
        (-0.184, 0.050, 0.028),
    ]
    right_handle_points = _mirror_x(left_handle_points)

    left_handle = tube_from_spline_points(
        left_handle_points,
        radius=handle_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    right_handle = tube_from_spline_points(
        right_handle_points,
        radius=handle_radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )

    upper_tray.visual(_save_mesh("lazy_susan_tray_floor", tray_floor), material=tray_glaze, name="tray_floor")
    upper_tray.visual(_save_mesh("lazy_susan_tray_shell", tray_shell), material=tray_glaze, name="tray_shell")
    upper_tray.visual(_save_mesh("lazy_susan_left_handle", left_handle), material=brushed_metal, name="left_handle")
    upper_tray.visual(_save_mesh("lazy_susan_right_handle", right_handle), material=brushed_metal, name="right_handle")
    for post_name, x_pos, y_pos in [
        ("left_handle_front_post", -0.184, -0.050),
        ("left_handle_rear_post", -0.184, 0.050),
        ("right_handle_front_post", 0.184, -0.050),
        ("right_handle_rear_post", 0.184, 0.050),
    ]:
        upper_tray.visual(
            Cylinder(radius=0.0055, length=0.032),
            origin=Origin(xyz=(x_pos, y_pos, 0.016)),
            material=brushed_metal,
            name=post_name,
        )

    model.articulation(
        "tray_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_base,
        child=upper_tray,
        origin=Origin(xyz=(0.0, 0.0, disc_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_base = object_model.get_part("lower_base")
    upper_tray = object_model.get_part("upper_tray")
    tray_spin = object_model.get_articulation("tray_spin")
    tray_floor = upper_tray.get_visual("tray_floor")
    turntable_disc = lower_base.get_visual("turntable_disc")
    upper_tray.get_visual("left_handle")
    upper_tray.get_visual("right_handle")

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
        "tray_spin_is_continuous",
        tray_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous rotation, got {tray_spin.articulation_type!r}",
    )
    ctx.check(
        "tray_spin_axis_vertical",
        tuple(tray_spin.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {tray_spin.axis!r}",
    )
    ctx.check(
        "tray_spin_has_no_angle_bounds",
        tray_spin.motion_limits is not None
        and tray_spin.motion_limits.lower is None
        and tray_spin.motion_limits.upper is None,
        details=f"unexpected continuous-joint bounds: {tray_spin.motion_limits!r}",
    )

    with ctx.pose({tray_spin: 0.0}):
        ctx.expect_contact(
            upper_tray,
            lower_base,
            elem_a=tray_floor,
            elem_b=turntable_disc,
            contact_tol=0.001,
            name="tray_floor_seats_on_turntable",
        )
        ctx.expect_origin_distance(
            upper_tray,
            lower_base,
            axes="xy",
            max_dist=0.001,
            name="tray_centered_at_rest",
        )
        ctx.expect_overlap(
            upper_tray,
            lower_base,
            axes="xy",
            elem_a=tray_floor,
            elem_b=turntable_disc,
            min_overlap=0.09,
            name="turntable_stays_under_tray",
        )

    with ctx.pose({tray_spin: pi / 2.0}):
        ctx.expect_contact(
            upper_tray,
            lower_base,
            elem_a=tray_floor,
            elem_b=turntable_disc,
            contact_tol=0.001,
            name="tray_floor_seats_on_turntable_rotated",
        )
        ctx.expect_origin_distance(
            upper_tray,
            lower_base,
            axes="xy",
            max_dist=0.001,
            name="tray_centered_while_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
