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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    axis: str = "z",
    material=None,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _circle_points_x_plane(x: float, radius: float, samples: int = 18) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius * cos((2.0 * pi * i) / samples),
            radius * sin((2.0 * pi * i) / samples),
        )
        for i in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_spotlight")

    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_base = model.material("dark_base", rgba=(0.14, 0.14, 0.15, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.72, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.64, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.68, 0.75, 0.82, 0.45))

    stand = model.part("stand")
    lamp = model.part("lamp_can")

    # Stable base and support structure.
    stand.visual(
        Box((0.38, 0.26, 0.02)),
        origin=Origin(xyz=(0.02, 0.0, 0.01)),
        material=dark_base,
        name="base_plate",
    )
    stand.visual(
        Box((0.14, 0.18, 0.018)),
        origin=Origin(xyz=(0.18, 0.0, 0.009)),
        material=dark_base,
        name="front_ballast",
    )
    stand.visual(
        Box((0.26, 0.05, 0.03)),
        origin=Origin(xyz=(0.02, 0.10, 0.015)),
        material=dark_base,
        name="left_skid",
    )
    stand.visual(
        Box((0.26, 0.05, 0.03)),
        origin=Origin(xyz=(0.02, -0.10, 0.015)),
        material=dark_base,
        name="right_skid",
    )
    stand.visual(
        Box((0.10, 0.12, 0.18)),
        origin=Origin(xyz=(-0.10, 0.0, 0.11)),
        material=graphite,
        name="mast_block",
    )
    stand.visual(
        Box((0.16, 0.28, 0.055)),
        origin=Origin(xyz=(-0.03, 0.0, 0.0575)),
        material=graphite,
        name="lower_saddle",
    )
    stand.visual(
        Box((0.15, 0.012, 0.34)),
        origin=Origin(xyz=(-0.005, 0.136, 0.20)),
        material=safety_yellow,
        name="left_yoke",
    )
    stand.visual(
        Box((0.15, 0.012, 0.34)),
        origin=Origin(xyz=(-0.005, -0.136, 0.20)),
        material=safety_yellow,
        name="right_yoke",
    )
    stand.visual(
        Box((0.08, 0.018, 0.10)),
        origin=Origin(xyz=(-0.055, 0.136, 0.10)),
        material=safety_yellow,
        name="left_rear_gusset",
    )
    stand.visual(
        Box((0.08, 0.018, 0.10)),
        origin=Origin(xyz=(-0.055, -0.136, 0.10)),
        material=safety_yellow,
        name="right_rear_gusset",
    )
    stand.visual(
        Box((0.06, 0.018, 0.09)),
        origin=Origin(xyz=(0.035, 0.136, 0.10)),
        material=safety_yellow,
        name="left_front_gusset",
    )
    stand.visual(
        Box((0.06, 0.018, 0.09)),
        origin=Origin(xyz=(0.035, -0.136, 0.10)),
        material=safety_yellow,
        name="right_front_gusset",
    )
    stand.visual(
        Box((0.04, 0.30, 0.05)),
        origin=Origin(xyz=(-0.09, 0.0, 0.365)),
        material=graphite,
        name="top_tie",
    )
    stand.visual(
        Box((0.07, 0.018, 0.09)),
        origin=Origin(xyz=(0.015, 0.139, 0.305)),
        material=safety_yellow,
        name="left_yoke_doubler",
    )
    stand.visual(
        Box((0.07, 0.018, 0.09)),
        origin=Origin(xyz=(0.015, -0.139, 0.305)),
        material=safety_yellow,
        name="right_yoke_doubler",
    )
    _add_cylinder(
        stand,
        radius=0.028,
        length=0.038,
        xyz=(0.02, 0.149, 0.305),
        axis="y",
        material=steel,
        name="left_pivot_pad",
    )
    _add_cylinder(
        stand,
        radius=0.028,
        length=0.038,
        xyz=(0.02, -0.149, 0.305),
        axis="y",
        material=steel,
        name="right_pivot_pad",
    )
    _add_cylinder(
        stand,
        radius=0.044,
        length=0.016,
        xyz=(0.02, 0.172, 0.305),
        axis="y",
        material=dark_base,
        name="left_lock_wheel",
    )
    _add_cylinder(
        stand,
        radius=0.044,
        length=0.016,
        xyz=(0.02, -0.172, 0.305),
        axis="y",
        material=dark_base,
        name="right_lock_wheel",
    )
    stand.visual(
        Box((0.078, 0.010, 0.010)),
        origin=Origin(xyz=(0.02, 0.178, 0.305)),
        material=steel,
        name="left_lock_handle",
    )
    stand.visual(
        Box((0.078, 0.010, 0.010)),
        origin=Origin(xyz=(0.02, -0.178, 0.305)),
        material=steel,
        name="right_lock_handle",
    )
    stand.visual(
        Box((0.035, 0.020, 0.035)),
        origin=Origin(xyz=(-0.015, 0.120, 0.215)),
        material=steel,
        name="left_lower_stop",
    )
    stand.visual(
        Box((0.035, 0.020, 0.035)),
        origin=Origin(xyz=(-0.015, -0.120, 0.215)),
        material=steel,
        name="right_lower_stop",
    )
    stand.visual(
        Box((0.035, 0.020, 0.035)),
        origin=Origin(xyz=(-0.010, 0.126, 0.3875)),
        material=steel,
        name="left_upper_stop",
    )
    stand.visual(
        Box((0.035, 0.020, 0.035)),
        origin=Origin(xyz=(-0.010, -0.126, 0.3875)),
        material=steel,
        name="right_upper_stop",
    )

    for x in (-0.11, 0.13):
        for y in (-0.08, 0.08):
            _add_cylinder(
                stand,
                radius=0.010,
                length=0.008,
                xyz=(x, y, 0.018),
                axis="z",
                material=steel,
            )

    for z in (0.275, 0.335):
        _add_cylinder(
            stand,
            radius=0.008,
            length=0.010,
            xyz=(0.000, 0.145, z),
            axis="y",
            material=steel,
        )
        _add_cylinder(
            stand,
            radius=0.008,
            length=0.010,
            xyz=(0.000, -0.145, z),
            axis="y",
            material=steel,
        )

    stand.inertial = Inertial.from_geometry(
        Box((0.38, 0.26, 0.24)),
        mass=22.0,
        origin=Origin(xyz=(0.00, 0.0, 0.12)),
    )

    # Spotlight can carried by side-yoke trunnions.
    body_shell_mesh = mesh_from_geometry(
        CylinderGeometry(0.115, 0.24, radial_segments=48, closed=False),
        "lamp_body_shell",
    )
    pivot_band_mesh = mesh_from_geometry(
        CylinderGeometry(0.122, 0.06, radial_segments=40, closed=False),
        "lamp_pivot_band",
    )
    rear_band_mesh = mesh_from_geometry(
        CylinderGeometry(0.102, 0.05, radial_segments=40, closed=True),
        "lamp_rear_cap",
    )
    bezel_mesh = mesh_from_geometry(
        CylinderGeometry(0.138, 0.06, radial_segments=48, closed=False),
        "lamp_front_bezel",
    )
    guard_ring_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _circle_points_x_plane(0.295, 0.155, samples=18),
            radius=0.006,
            samples_per_segment=8,
            closed_spline=True,
            radial_segments=18,
            cap_ends=False,
        ),
        "lamp_guard_ring",
    )

    lamp.visual(
        body_shell_mesh,
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="body_shell",
    )
    lamp.visual(
        pivot_band_mesh,
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_yellow,
        name="pivot_band",
    )
    lamp.visual(
        rear_band_mesh,
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="rear_cap",
    )
    _add_cylinder(
        lamp,
        radius=0.112,
        length=0.032,
        xyz=(-0.030, 0.0, 0.0),
        axis="x",
        material=graphite,
        name="rear_mount_ring",
    )
    lamp.visual(
        Box((0.05, 0.10, 0.06)),
        origin=Origin(xyz=(-0.085, 0.0, 0.0)),
        material=dark_base,
        name="rear_service_box",
    )
    lamp.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.23, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_yellow,
        name="front_bezel",
    )
    _add_cylinder(
        lamp,
        radius=0.138,
        length=0.032,
        xyz=(0.214, 0.0, 0.0),
        axis="x",
        material=safety_yellow,
        name="front_mount_ring",
    )
    _add_cylinder(
        lamp,
        radius=0.108,
        length=0.016,
        xyz=(0.212, 0.0, 0.0),
        axis="x",
        material=lens_tint,
        name="lens",
    )
    _add_cylinder(
        lamp,
        radius=0.024,
        length=0.020,
        xyz=(0.000, 0.120, 0.0),
        axis="y",
        material=steel,
        name="left_trunnion",
    )
    _add_cylinder(
        lamp,
        radius=0.024,
        length=0.020,
        xyz=(0.000, -0.120, 0.0),
        axis="y",
        material=steel,
        name="right_trunnion",
    )
    lamp.visual(
        Box((0.04, 0.010, 0.18)),
        origin=Origin(xyz=(0.000, 0.107, 0.0)),
        material=steel,
        name="left_lock_sector",
    )
    lamp.visual(
        Box((0.04, 0.010, 0.18)),
        origin=Origin(xyz=(0.000, -0.107, 0.0)),
        material=steel,
        name="right_lock_sector",
    )
    lamp.visual(
        Box((0.025, 0.016, 0.020)),
        origin=Origin(xyz=(-0.006, 0.112, 0.080)),
        material=steel,
        name="left_upper_stop_ear",
    )
    lamp.visual(
        Box((0.025, 0.016, 0.020)),
        origin=Origin(xyz=(-0.006, 0.112, -0.080)),
        material=steel,
        name="left_lower_stop_ear",
    )
    lamp.visual(
        Box((0.025, 0.016, 0.020)),
        origin=Origin(xyz=(-0.006, -0.112, 0.080)),
        material=steel,
        name="right_upper_stop_ear",
    )
    lamp.visual(
        Box((0.025, 0.016, 0.020)),
        origin=Origin(xyz=(-0.006, -0.112, -0.080)),
        material=steel,
        name="right_lower_stop_ear",
    )
    lamp.visual(
        guard_ring_mesh,
        material=safety_yellow,
        name="guard_ring",
    )
    _add_cylinder(
        lamp,
        radius=0.005,
        length=0.302,
        xyz=(0.295, 0.0, 0.0),
        axis="y",
        material=steel,
        name="guard_bar_horizontal",
    )
    _add_cylinder(
        lamp,
        radius=0.005,
        length=0.302,
        xyz=(0.295, 0.0, 0.0),
        axis="z",
        material=steel,
        name="guard_bar_vertical",
    )
    _add_cylinder(
        lamp,
        radius=0.008,
        length=0.050,
        xyz=(0.270, 0.0, 0.145),
        axis="x",
        material=steel,
        name="guard_standoff_top",
    )
    _add_cylinder(
        lamp,
        radius=0.008,
        length=0.050,
        xyz=(0.270, 0.0, -0.145),
        axis="x",
        material=steel,
        name="guard_standoff_bottom",
    )
    _add_cylinder(
        lamp,
        radius=0.008,
        length=0.050,
        xyz=(0.270, 0.145, 0.0),
        axis="x",
        material=steel,
        name="guard_standoff_left",
    )
    _add_cylinder(
        lamp,
        radius=0.008,
        length=0.050,
        xyz=(0.270, -0.145, 0.0),
        axis="x",
        material=steel,
        name="guard_standoff_right",
    )

    lamp.inertial = Inertial.from_geometry(
        Box((0.34, 0.27, 0.27)),
        mass=9.5,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_lamp",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lamp,
        origin=Origin(xyz=(0.02, 0.0, 0.305)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-0.42,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    lamp = object_model.get_part("lamp_can")
    tilt = object_model.get_articulation("stand_to_lamp")
    limits = tilt.motion_limits

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
        "tilt_joint_is_yoke_carried_and_bidirectional",
        (
            tilt.axis == (0.0, -1.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        ),
        details=f"axis={tilt.axis}, lower={getattr(limits, 'lower', None)}, upper={getattr(limits, 'upper', None)}",
    )

    lower = -0.42 if limits is None or limits.lower is None else limits.lower
    upper = 0.95 if limits is None or limits.upper is None else limits.upper

    with ctx.pose({tilt: 0.0}):
        ctx.expect_contact(
            stand,
            lamp,
            elem_a="left_pivot_pad",
            elem_b="left_trunnion",
            contact_tol=0.0015,
            name="left_trunnion_seated_in_yoke",
        )
        ctx.expect_contact(
            stand,
            lamp,
            elem_a="right_pivot_pad",
            elem_b="right_trunnion",
            contact_tol=0.0015,
            name="right_trunnion_seated_in_yoke",
        )
        ctx.expect_gap(
            lamp,
            stand,
            axis="z",
            positive_elem="guard_ring",
            negative_elem="base_plate",
            min_gap=0.12,
            name="rest_pose_guard_clears_base",
        )
        rest_guard = ctx.part_element_world_aabb(lamp, elem="guard_ring")

    with ctx.pose({tilt: lower}):
        ctx.expect_gap(
            lamp,
            stand,
            axis="z",
            positive_elem="guard_ring",
            negative_elem="base_plate",
            min_gap=0.01,
            name="downward_limit_keeps_guard_off_base",
        )
        low_guard = ctx.part_element_world_aabb(lamp, elem="guard_ring")

    with ctx.pose({tilt: upper}):
        ctx.expect_gap(
            lamp,
            stand,
            axis="z",
            positive_elem="guard_ring",
            negative_elem="base_plate",
            min_gap=0.12,
            name="upward_limit_keeps_guard_off_base",
        )
        high_guard = ctx.part_element_world_aabb(lamp, elem="guard_ring")

    ctx.check(
        "positive_tilt_raises_front_of_can",
        (
            rest_guard is not None
            and high_guard is not None
            and high_guard[0][2] > rest_guard[0][2] + 0.10
        ),
        details=f"rest={rest_guard}, high={high_guard}",
    )
    ctx.check(
        "negative_tilt_lowers_front_of_can",
        (
            rest_guard is not None
            and low_guard is not None
            and low_guard[0][2] < rest_guard[0][2] - 0.08
        ),
        details=f"rest={rest_guard}, low={low_guard}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
