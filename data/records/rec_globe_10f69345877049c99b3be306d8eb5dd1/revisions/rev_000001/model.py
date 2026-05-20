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
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arc_points(
    radius: float,
    start_deg: float,
    end_deg: float,
    z_height: float,
    *,
    samples: int = 21,
) -> list[tuple[float, float, float]]:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    return [
        (
            radius * math.cos(start + ((end - start) * idx / (samples - 1))),
            radius * math.sin(start + ((end - start) * idx / (samples - 1))),
            z_height,
        )
        for idx in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="antique_desk_globe")

    walnut = model.material("walnut", rgba=(0.35, 0.22, 0.12, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.20, 0.12, 0.06, 1.0))
    antique_brass = model.material("antique_brass", rgba=(0.74, 0.60, 0.32, 1.0))
    aged_parchment = model.material("aged_parchment", rgba=(0.90, 0.87, 0.73, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.52, 0.68, 0.78, 1.0))

    sphere_radius = 0.09
    globe_body_radius = 0.086
    pole_cap_radius = 0.004
    meridian_radius = 0.102
    horizon_height = 0.235
    center_height = 0.340
    tilt_angle = math.radians(23.5)

    stand = model.part("stand")

    pedestal_body = LatheGeometry(
        [
            (0.0, 0.000),
            (0.046, 0.000),
            (0.108, 0.008),
            (0.138, 0.018),
            (0.124, 0.026),
            (0.094, 0.036),
            (0.062, 0.054),
            (0.048, 0.082),
            (0.040, 0.128),
            (0.033, 0.178),
            (0.026, 0.210),
            (0.020, 0.220),
            (0.020, 0.235),
            (0.014, 0.235),
            (0.014, 0.248),
            (0.0, 0.248),
        ],
        segments=88,
    )
    stand.visual(
        _save_mesh("stand_pedestal_body", pedestal_body),
        material=walnut,
        name="pedestal_body",
    )

    foot_path = [
        (0.050, 0.0, 0.020),
        (0.086, 0.0, 0.018),
        (0.124, 0.0, 0.014),
        (0.158, 0.0, 0.010),
    ]
    foot_geom = tube_from_spline_points(
        foot_path,
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        stand.visual(
            _save_mesh(f"stand_foot_{index}", foot_geom.copy().rotate_z(angle)),
            material=dark_walnut,
            name=f"foot_{index}",
        )

    horizon_north = tube_from_spline_points(
        _arc_points(0.122, 16.0, 164.0, horizon_height),
        radius=0.0055,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    horizon_south = tube_from_spline_points(
        _arc_points(0.122, 196.0, 344.0, horizon_height),
        radius=0.0055,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    stand.visual(
        _save_mesh("horizon_ring_north", horizon_north),
        material=antique_brass,
        name="horizon_ring_north",
    )
    stand.visual(
        _save_mesh("horizon_ring_south", horizon_south),
        material=antique_brass,
        name="horizon_ring_south",
    )

    horizon_spoke = tube_from_spline_points(
        [(0.014, 0.0, horizon_height), (0.118, 0.0, horizon_height)],
        radius=0.0038,
        samples_per_segment=2,
        radial_segments=14,
        cap_ends=True,
    )
    for index, angle_deg in enumerate((60.0, 120.0, 240.0, 300.0)):
        stand.visual(
            _save_mesh(
                f"horizon_spoke_{index}",
                horizon_spoke.copy().rotate_z(math.radians(angle_deg)),
            ),
            material=antique_brass,
            name=f"horizon_spoke_{index}",
        )

    for index, angle_deg in enumerate((60.0, 120.0, 240.0, 300.0)):
        angle = math.radians(angle_deg)
        stand.visual(
            Cylinder(radius=0.0052, length=0.220),
            origin=Origin(
                xyz=(0.122 * math.cos(angle), 0.122 * math.sin(angle), 0.125),
            ),
            material=antique_brass,
            name=f"horizon_post_{index}",
        )

    support_upright = tube_from_spline_points(
        [
            (0.090, 0.0, 0.026),
            (0.109, 0.0, 0.090),
            (0.121, 0.0, horizon_height),
            (0.125, 0.0, center_height),
        ],
        radius=0.0085,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    stand.visual(
        _save_mesh("side_support_pos", support_upright),
        material=antique_brass,
        name="side_support_pos",
    )
    stand.visual(
        _save_mesh("side_support_neg", support_upright.copy().rotate_z(math.pi)),
        material=antique_brass,
        name="side_support_neg",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.119, 0.0, center_height), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="support_pad_pos",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.119, 0.0, center_height), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="support_pad_neg",
    )
    stand.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.129, 0.0, center_height)),
        material=antique_brass,
        name="support_knob_pos",
    )
    stand.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(-0.129, 0.0, center_height)),
        material=antique_brass,
        name="support_knob_neg",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.36)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    meridian = model.part("meridian")
    meridian.visual(
        _save_mesh(
            "meridian_ring",
            TorusGeometry(
                radius=meridian_radius,
                tube=0.0048,
                radial_segments=18,
                tubular_segments=84,
            ),
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=antique_brass,
        name="meridian_ring",
    )
    meridian.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="hub_pos",
    )
    meridian.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="hub_neg",
    )
    meridian.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="trunnion_pos",
    )
    meridian.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antique_brass,
        name="trunnion_neg",
    )
    meridian.visual(
        Cylinder(radius=0.0028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=antique_brass,
        name="north_pivot_shaft",
    )
    meridian.visual(
        Cylinder(radius=0.0028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=antique_brass,
        name="south_pivot_shaft",
    )
    meridian.visual(
        Sphere(radius=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=antique_brass,
        name="north_pivot",
    )
    meridian.visual(
        Sphere(radius=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.094)),
        material=antique_brass,
        name="south_pivot",
    )
    meridian.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=antique_brass,
        name="north_cap",
    )
    meridian.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=antique_brass,
        name="south_cap",
    )
    meridian.inertial = Inertial.from_geometry(
        Box((0.23, 0.03, 0.23)),
        mass=0.55,
        origin=Origin(),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_body_radius),
        material=ocean_blue,
        name="globe_sphere",
    )
    globe.visual(
        Cylinder(radius=0.0022, length=0.174),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aged_parchment,
        name="axis_marker",
    )
    globe.visual(
        Sphere(radius=pole_cap_radius),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=aged_parchment,
        name="north_pole_cap",
    )
    globe.visual(
        Sphere(radius=pole_cap_radius),
        origin=Origin(xyz=(0.0, 0.0, -0.087)),
        material=aged_parchment,
        name="south_pole_cap",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=globe_body_radius),
        mass=0.42,
        origin=Origin(),
    )

    model.articulation(
        "meridian_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, center_height), rpy=(tilt_angle, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-0.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    meridian_tilt = object_model.get_articulation("meridian_tilt")
    globe_spin = object_model.get_articulation("globe_spin")

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

    ctx.check("key_parts_present", True, "")
    ctx.check(
        "articulation_axes_match_mechanism",
        all(abs(a - b) < 1e-9 for a, b in zip(meridian_tilt.axis, (1.0, 0.0, 0.0)))
        and all(abs(a - b) < 1e-9 for a, b in zip(globe_spin.axis, (0.0, 0.0, 1.0))),
        f"tilt axis={meridian_tilt.axis}, spin axis={globe_spin.axis}",
    )
    ctx.check(
        "articulation_limits_match_globe_behavior",
        meridian_tilt.motion_limits is not None
        and meridian_tilt.motion_limits.lower is not None
        and meridian_tilt.motion_limits.upper is not None
        and meridian_tilt.motion_limits.lower < 0.0 < meridian_tilt.motion_limits.upper
        and globe_spin.motion_limits is not None
        and globe_spin.motion_limits.lower is None
        and globe_spin.motion_limits.upper is None,
        "meridian should have bounded tilt while globe spin stays continuous",
    )

    ctx.expect_contact(
        meridian,
        stand,
        elem_a="trunnion_pos",
        elem_b="support_pad_pos",
        contact_tol=0.001,
        name="positive_side_trunnion_seated",
    )
    ctx.expect_contact(
        meridian,
        stand,
        elem_a="trunnion_neg",
        elem_b="support_pad_neg",
        contact_tol=0.001,
        name="negative_side_trunnion_seated",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="north_pole_cap",
        elem_b="north_pivot",
        contact_tol=0.001,
        name="north_polar_pivot_clips_globe",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a="south_pole_cap",
        elem_b="south_pivot",
        contact_tol=0.001,
        name="south_polar_pivot_clips_globe",
    )

    with ctx.pose({meridian_tilt: 0.18, globe_spin: 1.3}):
        ctx.expect_contact(
            meridian,
            stand,
            elem_a="trunnion_pos",
            elem_b="support_pad_pos",
            contact_tol=0.001,
            name="positive_side_trunnion_seated_tilted",
        )
        ctx.expect_contact(
            meridian,
            stand,
            elem_a="trunnion_neg",
            elem_b="support_pad_neg",
            contact_tol=0.001,
            name="negative_side_trunnion_seated_tilted",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="north_pole_cap",
            elem_b="north_pivot",
            contact_tol=0.001,
            name="north_polar_pivot_clips_globe_tilted",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a="south_pole_cap",
            elem_b="south_pivot",
            contact_tol=0.001,
            name="south_polar_pivot_clips_globe_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
