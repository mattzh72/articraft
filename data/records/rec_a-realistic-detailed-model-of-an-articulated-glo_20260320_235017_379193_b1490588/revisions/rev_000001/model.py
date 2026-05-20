from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
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
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
    wrap_profile_onto_surface,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

GLOBE_RADIUS = 0.09
EARTH_TILT = math.radians(23.5)
MERIDIAN_RADIUS = 0.106
MERIDIAN_TUBE_RADIUS = 0.005
MERIDIAN_COLLAR_HEIGHT = 0.012
MERIDIAN_CENTER_Z = MERIDIAN_COLLAR_HEIGHT + MERIDIAN_RADIUS


def _axis_vector() -> tuple[float, float, float]:
    return (math.sin(EARTH_TILT), 0.0, math.cos(EARTH_TILT))


def _lat_lon_direction(lat_deg: float, lon_deg: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    return (
        math.cos(lat) * math.cos(lon),
        math.cos(lat) * math.sin(lon),
        math.sin(lat),
    )


def _mesh_name(name: str) -> str:
    return str(ASSETS.mesh_path(name))


def _build_meridian_segment(start_deg: float, end_deg: float, name: str):
    arc_points = []
    for step in range(7):
        t = start_deg + (end_deg - start_deg) * step / 6.0
        angle = math.radians(t)
        arc_points.append(
            (
                MERIDIAN_RADIUS * math.cos(angle),
                0.0,
                MERIDIAN_CENTER_Z + MERIDIAN_RADIUS * math.sin(angle),
            )
        )
    geom = tube_from_spline_points(
        arc_points,
        radius=MERIDIAN_TUBE_RADIUS,
        samples_per_segment=8,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, _mesh_name(name))


def _build_continent_mesh(
    name: str,
    points: list[tuple[float, float]],
    lat_deg: float,
    lon_deg: float,
):
    outline = sample_catmull_rom_spline_2d(
        points,
        samples_per_segment=12,
        closed=True,
    )
    wrapped = wrap_profile_onto_surface(
        outline,
        Sphere(radius=GLOBE_RADIUS),
        thickness=0.0012,
        direction=_lat_lon_direction(lat_deg, lon_deg),
        mapping="intrinsic",
        visible_relief=0.0005,
        surface_max_edge=0.005,
    )
    return mesh_from_geometry(wrapped, _mesh_name(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_globe", assets=ASSETS)

    wood = model.material("wood", rgba=(0.38, 0.24, 0.14, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.24, 0.15, 0.09, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.66, 0.34, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.63, 0.53, 0.28, 1.0))
    ocean = model.material("ocean", rgba=(0.16, 0.38, 0.62, 1.0))
    land = model.material("land", rgba=(0.70, 0.73, 0.55, 1.0))
    paper = model.material("paper", rgba=(0.91, 0.88, 0.77, 1.0))

    axis = _axis_vector()

    base = model.part("base")
    base_profile = [
        (0.0, 0.0),
        (0.048, 0.0),
        (0.092, 0.006),
        (0.108, 0.016),
        (0.104, 0.026),
        (0.086, 0.039),
        (0.061, 0.049),
        (0.036, 0.054),
        (0.0, 0.054),
    ]
    base_mesh = mesh_from_geometry(LatheGeometry(base_profile, segments=56), _mesh_name("globe_base.obj"))
    base.visual(base_mesh, material=wood)
    base.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=aged_brass,
        name="collar",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=brass,
        name="stem",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=aged_brass,
        name="socket",
    )
    base.visual(
        Box((0.050, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, 0.084, 0.030), rpy=(0.0, 0.0, 0.0)),
        material=brass,
        name="nameplate",
    )
    base.visual(
        Box((0.056, 0.002, 0.024)),
        origin=Origin(xyz=(0.0, 0.083, 0.030)),
        material=paper,
        name="nameplate_inlay",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.054),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
    )

    meridian = model.part("meridian")
    meridian.visual(
        Cylinder(radius=0.013, length=MERIDIAN_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, MERIDIAN_COLLAR_HEIGHT * 0.5)),
        material=aged_brass,
        name="meridian_collar",
    )
    for idx, (start, end) in enumerate(
        [
            (-90.0, -30.0),
            (-30.0, 30.0),
            (30.0, 90.0),
            (90.0, 150.0),
            (150.0, 210.0),
            (210.0, 270.0),
        ],
        start=1,
    ):
        meridian.visual(
            _build_meridian_segment(start, end, f"meridian_segment_{idx}.obj"),
            material=brass,
            name=f"meridian_segment_{idx}",
        )
    meridian.visual(
        Box((0.028, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, MERIDIAN_COLLAR_HEIGHT + 0.004)),
        material=aged_brass,
        name="saddle",
    )
    for idx, sign in enumerate((1.0, -1.0), start=1):
        meridian.visual(
            Cylinder(radius=0.0065, length=0.016),
            origin=Origin(
                xyz=(
                    sign * axis[0] * 0.099,
                    0.0,
                    MERIDIAN_CENTER_Z + sign * axis[2] * 0.099,
                ),
                rpy=(0.0, EARTH_TILT, 0.0),
            ),
            material=aged_brass,
            name=f"bearing_cup_{idx}",
        )
    meridian.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.015),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, MERIDIAN_CENTER_Z)),
    )

    globe = model.part("globe")
    globe.visual(Sphere(radius=GLOBE_RADIUS), material=ocean, name="ocean_shell")

    north_america = _build_continent_mesh(
        "north_america.obj",
        [
            (-0.021, -0.014),
            (-0.018, 0.002),
            (-0.010, 0.014),
            (0.002, 0.018),
            (0.016, 0.012),
            (0.021, 0.002),
            (0.017, -0.010),
            (0.006, -0.020),
            (-0.008, -0.022),
            (-0.018, -0.018),
        ],
        lat_deg=42.0,
        lon_deg=-102.0,
    )
    south_america = _build_continent_mesh(
        "south_america.obj",
        [
            (-0.009, 0.018),
            (0.002, 0.014),
            (0.010, 0.006),
            (0.010, -0.004),
            (0.005, -0.016),
            (-0.001, -0.028),
            (-0.007, -0.034),
            (-0.013, -0.022),
            (-0.014, -0.008),
            (-0.012, 0.006),
        ],
        lat_deg=-17.0,
        lon_deg=-60.0,
    )
    eurasia = _build_continent_mesh(
        "eurasia.obj",
        [
            (-0.034, -0.014),
            (-0.028, 0.000),
            (-0.020, 0.012),
            (-0.008, 0.020),
            (0.008, 0.022),
            (0.024, 0.014),
            (0.036, 0.010),
            (0.042, -0.002),
            (0.032, -0.012),
            (0.014, -0.016),
            (0.002, -0.012),
            (-0.012, -0.020),
            (-0.026, -0.020),
        ],
        lat_deg=48.0,
        lon_deg=58.0,
    )
    africa = _build_continent_mesh(
        "africa.obj",
        [
            (-0.012, 0.018),
            (0.000, 0.024),
            (0.012, 0.016),
            (0.014, 0.002),
            (0.008, -0.016),
            (0.000, -0.028),
            (-0.008, -0.032),
            (-0.016, -0.014),
            (-0.016, 0.004),
        ],
        lat_deg=5.0,
        lon_deg=20.0,
    )
    australia = _build_continent_mesh(
        "australia.obj",
        [
            (-0.014, 0.008),
            (-0.004, 0.014),
            (0.010, 0.012),
            (0.016, 0.004),
            (0.014, -0.006),
            (0.004, -0.012),
            (-0.008, -0.012),
            (-0.016, -0.002),
        ],
        lat_deg=-25.0,
        lon_deg=135.0,
    )
    greenland = _build_continent_mesh(
        "greenland.obj",
        [
            (-0.008, 0.010),
            (-0.002, 0.016),
            (0.006, 0.012),
            (0.008, 0.002),
            (0.004, -0.008),
            (-0.004, -0.010),
            (-0.010, 0.000),
        ],
        lat_deg=73.0,
        lon_deg=-42.0,
    )

    for name, mesh in [
        ("north_america", north_america),
        ("south_america", south_america),
        ("eurasia", eurasia),
        ("africa", africa),
        ("australia", australia),
        ("greenland", greenland),
    ]:
        globe.visual(mesh, material=land, name=name)

    for idx, sign in enumerate((1.0, -1.0), start=1):
        globe.visual(
            Cylinder(radius=0.0048, length=0.010),
            origin=Origin(
                xyz=(
                    sign * axis[0] * 0.094,
                    0.0,
                    sign * axis[2] * 0.094,
                ),
                rpy=(0.0, EARTH_TILT, 0.0),
            ),
            material=aged_brass,
            name=f"polar_pin_{idx}",
        )

    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_meridian",
        ArticulationType.FIXED,
        parent="base",
        child="meridian",
        origin=Origin(xyz=(0.0, 0.0, 0.1265)),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent="meridian",
        child="globe",
        origin=Origin(xyz=(0.0, 0.0, MERIDIAN_CENTER_Z)),
        axis=axis,
        motion_limits=MotionLimits(effort=2.5, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.100)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.allow_overlap(
        "globe",
        "meridian",
        reason="the meridian ring encloses the globe and generated collision hulls can conservatively intrude into the ring opening",
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("globe", "base", axes="xy", max_dist=0.002)
    ctx.expect_origin_distance("meridian", "base", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("meridian", "base", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_gap("meridian", "base", axis="z", max_gap=0.003, max_penetration=0.002)
    ctx.expect_aabb_overlap("globe", "base", axes="xy", min_overlap=0.16)
    ctx.expect_aabb_gap("globe", "base", axis="z", max_gap=0.04, max_penetration=0.0)
    ctx.expect_aabb_overlap("globe", "meridian", axes="xz", min_overlap=0.17)

    for angle in (0.0, math.pi * 0.5, math.pi):
        with ctx.pose(globe_spin=angle):
            ctx.expect_origin_distance("globe", "base", axes="xy", max_dist=0.002)
            ctx.expect_aabb_overlap("globe", "base", axes="xy", min_overlap=0.16)
            ctx.expect_aabb_overlap("globe", "meridian", axes="xz", min_overlap=0.17)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
