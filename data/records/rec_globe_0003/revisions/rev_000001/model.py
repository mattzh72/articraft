from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    wrap_profile_onto_surface,
)

ASSETS = AssetContext.from_script(__file__)

GLOBE_RADIUS = 0.108
MERIDIAN_OUTER_RADIUS = 0.138
MERIDIAN_INNER_RADIUS = 0.124
MERIDIAN_THICKNESS = 0.0035
RING_CENTER_Z = 0.205


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _annulus_band(*, outer_radius: float, inner_radius: float, thickness: float):
    outer = CylinderGeometry(radius=outer_radius, height=thickness, radial_segments=72)
    inner = CylinderGeometry(radius=inner_radius, height=thickness + 0.002, radial_segments=72)
    return boolean_difference(outer, inner)


def _unit(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(sum(component * component for component in vec))
    return (vec[0] / mag, vec[1] / mag, vec[2] / mag)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _closed_profile(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return sample_catmull_rom_spline_2d(points, samples_per_segment=10, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_globe_stand", assets=ASSETS)

    bronze = model.material("bronze", rgba=(0.48, 0.34, 0.20, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.63, 0.54, 0.33, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.19, 0.19, 0.20, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.24, 0.46, 0.67, 1.0))
    parchment = model.material("parchment", rgba=(0.92, 0.88, 0.72, 1.0))
    land_olive = model.material("land_olive", rgba=(0.56, 0.62, 0.33, 1.0))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.115, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=bronze,
        name="pedestal_foot",
    )
    support.visual(
        Cylinder(radius=0.090, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=bronze,
        name="pedestal_step",
    )
    support.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=bronze,
        name="crown_plate",
    )
    support.visual(
        Cylinder(radius=0.024, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=bronze,
        name="center_column",
    )
    support.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=bronze,
        name="column_collar",
    )
    support.visual(
        Box((0.056, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=bronze,
        name="fork_bridge",
    )

    support.visual(
        Box((0.170, 0.018, 0.012)),
        origin=Origin(xyz=(0.098, 0.0, 0.120), rpy=(0.0, -0.64, 0.0)),
        material=bronze,
        name="right_fork_arm",
    )
    support.visual(
        Box((0.170, 0.018, 0.012)),
        origin=Origin(xyz=(-0.098, 0.0, 0.120), rpy=(0.0, 0.64, 0.0)),
        material=bronze,
        name="left_fork_arm",
    )
    support.visual(
        Box((0.014, 0.038, 0.076)),
        origin=Origin(xyz=(0.160, 0.0, 0.167)),
        material=bronze,
        name="right_cheek_plate",
    )
    support.visual(
        Box((0.014, 0.038, 0.076)),
        origin=Origin(xyz=(-0.160, 0.0, 0.167)),
        material=bronze,
        name="left_cheek_plate",
    )
    support.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.155, 0.0, RING_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="right_bushing",
    )
    support.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(-0.155, 0.0, RING_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="left_bushing",
    )
    support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.150),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    meridian = model.part("meridian")
    meridian.visual(
        _save_mesh(
            "meridian_ring.obj",
            _annulus_band(
                outer_radius=MERIDIAN_OUTER_RADIUS,
                inner_radius=MERIDIAN_INNER_RADIUS,
                thickness=MERIDIAN_THICKNESS,
            ).rotate_x(math.pi / 2.0),
        ),
        material=aged_brass,
        name="ring_band",
    )
    meridian.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(0.144, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="right_hub",
    )
    meridian.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(xyz=(-0.144, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="left_hub",
    )
    meridian.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=dark_iron,
        name="top_bearing",
    )
    meridian.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.126)),
        material=dark_iron,
        name="bottom_bearing",
    )
    meridian.inertial = Inertial.from_geometry(
        Box((0.304, 0.020, 0.282)),
        mass=0.75,
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=ocean_blue,
        name="globe_shell",
    )
    globe.visual(
        _save_mesh(
            "equator_band.obj",
            TorusGeometry(
                radius=GLOBE_RADIUS - 0.0035,
                tube=0.0035,
                radial_segments=16,
                tubular_segments=72,
            ),
        ),
        material=parchment,
        name="equator_band",
    )
    globe.visual(
        _save_mesh(
            "legend_patch.obj",
            wrap_profile_onto_surface(
                rounded_rect_profile(0.032, 0.018, radius=0.003, corner_segments=8),
                Sphere(radius=GLOBE_RADIUS),
                thickness=0.0008,
                direction=_unit((1.0, 0.35, 0.25)),
                mapping="intrinsic",
                visible_relief=0.0001,
                surface_max_edge=0.004,
            ),
        ),
        material=parchment,
        name="legend_patch",
    )
    globe.visual(
        _save_mesh(
            "continent_americas.obj",
            wrap_profile_onto_surface(
                _closed_profile(
                    [
                        (-0.010, 0.026),
                        (-0.018, 0.014),
                        (-0.020, -0.003),
                        (-0.014, -0.018),
                        (-0.010, -0.034),
                        (0.001, -0.040),
                        (0.010, -0.026),
                        (0.010, -0.008),
                        (0.016, 0.008),
                        (0.010, 0.026),
                    ]
                ),
                Sphere(radius=GLOBE_RADIUS),
                thickness=0.0009,
                direction=_unit((-0.70, -0.58, 0.20)),
                mapping="intrinsic",
                visible_relief=0.00012,
                surface_max_edge=0.004,
            ),
        ),
        material=land_olive,
        name="continent_americas",
    )
    globe.visual(
        _save_mesh(
            "continent_eurafrica.obj",
            wrap_profile_onto_surface(
                _closed_profile(
                    [
                        (-0.022, 0.020),
                        (-0.010, 0.028),
                        (0.010, 0.026),
                        (0.026, 0.018),
                        (0.022, 0.004),
                        (0.012, -0.004),
                        (0.016, -0.016),
                        (0.008, -0.032),
                        (-0.004, -0.036),
                        (-0.014, -0.020),
                        (-0.020, -0.004),
                    ]
                ),
                Sphere(radius=GLOBE_RADIUS),
                thickness=0.0009,
                direction=_unit((0.74, 0.22, 0.30)),
                mapping="intrinsic",
                visible_relief=0.00012,
                surface_max_edge=0.004,
            ),
        ),
        material=land_olive,
        name="continent_eurafrica",
    )
    globe.visual(
        _save_mesh(
            "continent_australia.obj",
            wrap_profile_onto_surface(
                _closed_profile(
                    [
                        (-0.012, 0.004),
                        (-0.004, 0.012),
                        (0.010, 0.010),
                        (0.014, -0.002),
                        (0.006, -0.010),
                        (-0.008, -0.010),
                    ]
                ),
                Sphere(radius=GLOBE_RADIUS),
                thickness=0.0008,
                direction=_unit((0.50, -0.72, -0.12)),
                mapping="intrinsic",
                visible_relief=0.00012,
                surface_max_edge=0.004,
            ),
        ),
        material=land_olive,
        name="continent_australia",
    )
    globe.visual(
        Cylinder(radius=0.003, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.0045)),
        material=aged_brass,
        name="north_pin",
    )
    globe.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=aged_brass,
        name="north_cap",
    )
    globe.visual(
        Cylinder(radius=0.003, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, -(GLOBE_RADIUS + 0.0045))),
        material=aged_brass,
        name="south_pin",
    )
    globe.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.119)),
        material=aged_brass,
        name="south_cap",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=1.1,
    )

    model.articulation(
        "support_to_meridian",
        ArticulationType.REVOLUTE,
        parent=support,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, RING_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support = object_model.get_part("support")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    meridian_tilt = object_model.get_articulation("support_to_meridian")
    globe_spin = object_model.get_articulation("meridian_to_globe")

    crown_plate = support.get_visual("crown_plate")
    left_bushing = support.get_visual("left_bushing")
    right_bushing = support.get_visual("right_bushing")

    ring_band = meridian.get_visual("ring_band")
    left_hub = meridian.get_visual("left_hub")
    right_hub = meridian.get_visual("right_hub")
    top_bearing = meridian.get_visual("top_bearing")
    bottom_bearing = meridian.get_visual("bottom_bearing")

    globe_shell = globe.get_visual("globe_shell")
    legend_patch = globe.get_visual("legend_patch")
    north_cap = globe.get_visual("north_cap")
    south_cap = globe.get_visual("south_cap")

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

    ctx.check(
        "meridian_tilt_axis_is_side_to_side",
        tuple(getattr(meridian_tilt, "axis", (None, None, None))) == (1.0, 0.0, 0.0),
        details=f"axis={getattr(meridian_tilt, 'axis', None)}",
    )
    ctx.check(
        "meridian_tilt_reaches_plus_minus_45_deg",
        meridian_tilt.motion_limits is not None
        and meridian_tilt.motion_limits.lower <= -math.radians(45.0) + 1e-6
        and meridian_tilt.motion_limits.upper >= math.radians(45.0) - 1e-6,
        details=f"limits={getattr(meridian_tilt, 'motion_limits', None)}",
    )
    ctx.check(
        "globe_spin_is_continuous",
        getattr(globe_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(globe_spin, 'articulation_type', None)}",
    )
    ctx.check(
        "globe_spin_axis_is_vertical_north_south",
        tuple(getattr(globe_spin, "axis", (None, None, None))) == (0.0, 0.0, 1.0),
        details=f"axis={getattr(globe_spin, 'axis', None)}",
    )

    ctx.expect_origin_distance(
        meridian,
        support,
        axes="xy",
        max_dist=1e-4,
        name="meridian_is_centered_over_pedestal",
    )
    ctx.expect_origin_distance(
        globe,
        meridian,
        axes="xy",
        max_dist=1e-4,
        name="globe_is_centered_inside_meridian",
    )
    ctx.expect_contact(
        meridian,
        support,
        elem_a=left_hub,
        elem_b=left_bushing,
        name="left_side_trunnion_is_supported",
    )
    ctx.expect_contact(
        meridian,
        support,
        elem_a=right_hub,
        elem_b=right_bushing,
        name="right_side_trunnion_is_supported",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a=north_cap,
        elem_b=top_bearing,
        name="north_polar_trunnion_contacts_meridian",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a=south_cap,
        elem_b=bottom_bearing,
        name="south_polar_trunnion_contacts_meridian",
    )
    ctx.expect_gap(
        meridian,
        support,
        axis="z",
        min_gap=0.028,
        positive_elem=ring_band,
        negative_elem=crown_plate,
        name="meridian_clears_pedestal_top",
    )
    ctx.expect_gap(
        globe,
        support,
        axis="z",
        min_gap=0.055,
        positive_elem=globe_shell,
        negative_elem=crown_plate,
        name="globe_clears_pedestal_top",
    )
    ctx.expect_overlap(
        globe,
        meridian,
        axes="xz",
        min_overlap=0.20,
        elem_a=globe_shell,
        elem_b=ring_band,
        name="globe_reads_nested_within_meridian",
    )

    meridian_rest_aabb = ctx.part_world_aabb(meridian)
    legend_rest_aabb = ctx.part_element_world_aabb(globe, elem=legend_patch)
    assert meridian_rest_aabb is not None
    assert legend_rest_aabb is not None

    meridian_rest_y = meridian_rest_aabb[1][1] - meridian_rest_aabb[0][1]
    meridian_rest_z = meridian_rest_aabb[1][2] - meridian_rest_aabb[0][2]
    legend_rest_center = _aabb_center(legend_rest_aabb)

    with ctx.pose({meridian_tilt: math.radians(45.0)}):
        ctx.expect_contact(
            meridian,
            support,
            elem_a=left_hub,
            elem_b=left_bushing,
            name="left_trunnion_contact_at_max_positive_tilt",
        )
        ctx.expect_contact(
            meridian,
            support,
            elem_a=right_hub,
            elem_b=right_bushing,
            name="right_trunnion_contact_at_max_positive_tilt",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=north_cap,
            elem_b=top_bearing,
            name="north_trunnion_contact_at_max_positive_tilt",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=south_cap,
            elem_b=bottom_bearing,
            name="south_trunnion_contact_at_max_positive_tilt",
        )
        ctx.expect_gap(
            globe,
            support,
            axis="z",
            min_gap=0.055,
            positive_elem=globe_shell,
            negative_elem=crown_plate,
            name="globe_clears_pedestal_when_tilted_forward",
        )
        meridian_tilted_aabb = ctx.part_world_aabb(meridian)
        assert meridian_tilted_aabb is not None
        meridian_tilted_y = meridian_tilted_aabb[1][1] - meridian_tilted_aabb[0][1]
        meridian_tilted_z = meridian_tilted_aabb[1][2] - meridian_tilted_aabb[0][2]
        ctx.check(
            "meridian_rotation_changes_ring_profile_as_expected",
            meridian_tilted_y > meridian_rest_y + 0.10 and meridian_tilted_z < meridian_rest_z - 0.05,
            details=(
                f"rest_y={meridian_rest_y:.4f}, tilted_y={meridian_tilted_y:.4f}, "
                f"rest_z={meridian_rest_z:.4f}, tilted_z={meridian_tilted_z:.4f}"
            ),
        )

    with ctx.pose({globe_spin: math.pi / 2.0}):
        legend_spin_aabb = ctx.part_element_world_aabb(globe, elem=legend_patch)
        assert legend_spin_aabb is not None
        legend_spin_center = _aabb_center(legend_spin_aabb)
        ctx.check(
            "globe_continuous_spin_moves_surface_detail",
            abs(legend_spin_center[0] - legend_rest_center[0]) > 0.02
            and abs(legend_spin_center[1] - legend_rest_center[1]) > 0.02,
            details=f"rest={legend_rest_center}, spun={legend_spin_center}",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=north_cap,
            elem_b=top_bearing,
            name="north_trunnion_contact_during_globe_spin",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=south_cap,
            elem_b=bottom_bearing,
            name="south_trunnion_contact_during_globe_spin",
        )

    with ctx.pose({meridian_tilt: -math.radians(45.0), globe_spin: math.pi}):
        ctx.expect_contact(
            meridian,
            support,
            elem_a=left_hub,
            elem_b=left_bushing,
            name="left_trunnion_contact_at_max_negative_tilt",
        )
        ctx.expect_contact(
            meridian,
            support,
            elem_a=right_hub,
            elem_b=right_bushing,
            name="right_trunnion_contact_at_max_negative_tilt",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=north_cap,
            elem_b=top_bearing,
            name="north_trunnion_contact_at_combined_pose",
        )
        ctx.expect_contact(
            globe,
            meridian,
            elem_a=south_cap,
            elem_b=bottom_bearing,
            name="south_trunnion_contact_at_combined_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
