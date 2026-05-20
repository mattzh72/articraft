from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
    wrap_profile_onto_surface,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.090
BASE_THICKNESS = 0.018
BASE_STEP_RADIUS = 0.062
BASE_STEP_THICKNESS = 0.010
COLUMN_RADIUS = 0.017
COLUMN_HEIGHT = 0.052
COLUMN_OFFSET_Y = -0.020
CROWN_RADIUS = 0.016
CROWN_LENGTH = 0.026
YOKE_SHOULDER_RADIUS = 0.009
YOKE_SHOULDER_HEIGHT = 0.014
YOKE_ARM_RADIUS = 0.0045
MERIDIAN_CENTER_Z = 0.146
MERIDIAN_RING_RADIUS = 0.071
MERIDIAN_RING_TUBE = 0.003
TRUNNION_RADIUS = 0.0045
TRUNNION_LENGTH = 0.018
YOKE_CAP_RADIUS = 0.010
YOKE_CAP_THICKNESS = 0.004
GLOBE_RADIUS = 0.055
PIVOT_BUTTON_RADIUS = 0.0045
PIVOT_BUTTON_THICKNESS = 0.003
POLAR_CAP_RADIUS = 0.006
POLAR_CAP_THICKNESS = 0.004
POLAR_SUPPORT_RADIUS = 0.0026
AXIAL_TILT = math.radians(23.5)
REST_MERIDIAN_TILT = math.radians(18.0)
MERIDIAN_TILT_LIMIT = math.radians(30.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _surface_direction(lat_deg: float, lon_deg: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    return (
        math.cos(lat) * math.cos(lon),
        math.cos(lat) * math.sin(lon),
        math.sin(lat),
    )


def _axis_vector(tilt_rad: float) -> tuple[float, float, float]:
    return (math.sin(tilt_rad), 0.0, math.cos(tilt_rad))


def _scaled(vec: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _continent_mesh(name: str, points: list[tuple[float, float]], lat_deg: float, lon_deg: float):
    profile = sample_catmull_rom_spline_2d(points, samples_per_segment=12, closed=True)
    wrapped = wrap_profile_onto_surface(
        profile,
        Sphere(radius=GLOBE_RADIUS),
        thickness=0.0008,
        direction=_surface_direction(lat_deg, lon_deg),
        mapping="intrinsic",
        visible_relief=0.00005,
        surface_max_edge=0.004,
    )
    return _save_mesh(name, wrapped)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_world_globe", assets=ASSETS)

    dark_wood = model.material("dark_wood", rgba=(0.30, 0.20, 0.12, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.76, 0.63, 0.31, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.84, 0.73, 0.43, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.17, 0.41, 0.71, 1.0))
    land_green = model.material("land_green", rgba=(0.45, 0.58, 0.27, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.22, 0.24, 1.0))

    right_arm_geom = tube_from_spline_points(
        [
            (0.044, -0.014, BASE_THICKNESS + BASE_STEP_THICKNESS + 0.008),
            (0.048, -0.014, 0.064),
            (0.061, -0.012, 0.102),
            (0.078, -0.010, 0.127),
            (0.092, -0.008, 0.138),
        ],
        radius=YOKE_ARM_RADIUS,
        samples_per_segment=20,
        radial_segments=20,
        cap_ends=True,
    )
    left_arm_geom = right_arm_geom.copy().scale(-1.0, 1.0, 1.0)
    right_arm_mesh = _save_mesh("right_yoke_arm.obj", right_arm_geom)
    left_arm_mesh = _save_mesh("left_yoke_arm.obj", left_arm_geom)
    meridian_ring_mesh = _save_mesh(
        "meridian_ring.obj",
        TorusGeometry(
            radius=MERIDIAN_RING_RADIUS,
            tube=MERIDIAN_RING_TUBE,
            radial_segments=18,
            tubular_segments=88,
        ),
    )

    americas_mesh = _continent_mesh(
        "americas.obj",
        [
            (-0.012, 0.028),
            (-0.020, 0.020),
            (-0.018, 0.006),
            (-0.010, -0.006),
            (-0.014, -0.020),
            (-0.006, -0.030),
            (0.004, -0.022),
            (0.010, -0.008),
            (0.006, 0.006),
            (0.010, 0.020),
            (0.000, 0.030),
        ],
        lat_deg=8.0,
        lon_deg=-90.0,
    )
    eurasia_mesh = _continent_mesh(
        "eurasia_africa.obj",
        [
            (-0.028, 0.018),
            (-0.010, 0.026),
            (0.010, 0.020),
            (0.026, 0.012),
            (0.022, -0.002),
            (0.010, -0.006),
            (0.016, -0.018),
            (0.006, -0.028),
            (-0.008, -0.026),
            (-0.018, -0.010),
            (-0.026, 0.002),
        ],
        lat_deg=16.0,
        lon_deg=28.0,
    )
    australia_mesh = _continent_mesh(
        "australia.obj",
        [
            (-0.014, 0.006),
            (-0.004, 0.012),
            (0.010, 0.008),
            (0.014, -0.002),
            (0.006, -0.010),
            (-0.008, -0.010),
        ],
        lat_deg=-26.0,
        lon_deg=134.0,
    )

    pivot_distance = GLOBE_RADIUS + (PIVOT_BUTTON_THICKNESS / 2.0)
    polar_button_face = GLOBE_RADIUS + PIVOT_BUTTON_THICKNESS
    polar_cap_center_distance = polar_button_face + (POLAR_CAP_THICKNESS / 2.0)
    polar_support_length = MERIDIAN_RING_RADIUS - polar_button_face - POLAR_CAP_THICKNESS
    polar_support_center_distance = polar_button_face + POLAR_CAP_THICKNESS + (polar_support_length / 2.0)
    axis_dir = _axis_vector(AXIAL_TILT)

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=dark_wood,
        name="foot_disk",
    )
    base.visual(
        Cylinder(radius=BASE_STEP_RADIUS, length=BASE_STEP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (BASE_STEP_THICKNESS / 2.0))),
        material=dark_wood,
        name="foot_step",
    )
    base.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                COLUMN_OFFSET_Y,
                BASE_THICKNESS + BASE_STEP_THICKNESS + (COLUMN_HEIGHT / 2.0),
            )
        ),
        material=dark_wood,
        name="center_column",
    )
    base.visual(
        Cylinder(radius=CROWN_RADIUS, length=CROWN_LENGTH),
        origin=Origin(
            xyz=(0.0, COLUMN_OFFSET_Y, 0.058),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=warm_brass,
        name="crown_hub",
    )
    base.visual(
        Cylinder(radius=YOKE_SHOULDER_RADIUS, length=YOKE_SHOULDER_HEIGHT),
        origin=Origin(
            xyz=(
                -(0.044),
                -0.014,
                BASE_THICKNESS + BASE_STEP_THICKNESS + (YOKE_SHOULDER_HEIGHT / 2.0),
            )
        ),
        material=warm_brass,
        name="left_yoke_shoulder",
    )
    base.visual(
        Cylinder(radius=YOKE_SHOULDER_RADIUS, length=YOKE_SHOULDER_HEIGHT),
        origin=Origin(
            xyz=(
                0.044,
                -0.014,
                BASE_THICKNESS + BASE_STEP_THICKNESS + (YOKE_SHOULDER_HEIGHT / 2.0),
            )
        ),
        material=warm_brass,
        name="right_yoke_shoulder",
    )
    base.visual(left_arm_mesh, material=warm_brass, name="left_yoke_arm")
    base.visual(right_arm_mesh, material=warm_brass, name="right_yoke_arm")
    base.visual(
        Cylinder(radius=YOKE_CAP_RADIUS, length=YOKE_CAP_THICKNESS),
        origin=Origin(
            xyz=(
                -(MERIDIAN_RING_RADIUS + TRUNNION_LENGTH + (YOKE_CAP_THICKNESS / 2.0)),
                0.0,
                MERIDIAN_CENTER_Z,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_brass,
        name="left_pivot_cap",
    )
    base.visual(
        Cylinder(radius=YOKE_CAP_RADIUS, length=YOKE_CAP_THICKNESS),
        origin=Origin(
            xyz=(
                MERIDIAN_RING_RADIUS + TRUNNION_LENGTH + (YOKE_CAP_THICKNESS / 2.0),
                0.0,
                MERIDIAN_CENTER_Z,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_brass,
        name="right_pivot_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=0.090),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    meridian = model.part("meridian")
    meridian.visual(
        meridian_ring_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="meridian_ring",
    )
    meridian.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(-(MERIDIAN_RING_RADIUS + (TRUNNION_LENGTH / 2.0)), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_brass,
        name="left_trunnion",
    )
    meridian.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(MERIDIAN_RING_RADIUS + (TRUNNION_LENGTH / 2.0), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_brass,
        name="right_trunnion",
    )
    meridian.visual(
        Cylinder(radius=POLAR_SUPPORT_RADIUS, length=polar_support_length),
        origin=Origin(
            xyz=_scaled(axis_dir, polar_support_center_distance),
            rpy=(0.0, AXIAL_TILT, 0.0),
        ),
        material=warm_brass,
        name="north_support",
    )
    meridian.visual(
        Cylinder(radius=POLAR_SUPPORT_RADIUS, length=polar_support_length),
        origin=Origin(
            xyz=_scaled(axis_dir, -polar_support_center_distance),
            rpy=(0.0, AXIAL_TILT, 0.0),
        ),
        material=warm_brass,
        name="south_support",
    )
    meridian.visual(
        Cylinder(radius=POLAR_CAP_RADIUS, length=POLAR_CAP_THICKNESS),
        origin=Origin(
            xyz=_scaled(axis_dir, polar_cap_center_distance),
            rpy=(0.0, AXIAL_TILT, 0.0),
        ),
        material=satin_brass,
        name="north_cap",
    )
    meridian.visual(
        Cylinder(radius=POLAR_CAP_RADIUS, length=POLAR_CAP_THICKNESS),
        origin=Origin(
            xyz=_scaled(axis_dir, -polar_cap_center_distance),
            rpy=(0.0, AXIAL_TILT, 0.0),
        ),
        material=satin_brass,
        name="south_cap",
    )
    meridian.inertial = Inertial.from_geometry(
        Cylinder(radius=MERIDIAN_RING_RADIUS + TRUNNION_LENGTH, length=0.012),
        mass=0.45,
        origin=Origin(),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=ocean_blue,
        name="ocean_sphere",
    )
    globe.visual(americas_mesh, material=land_green, name="americas")
    globe.visual(eurasia_mesh, material=land_green, name="eurasia_africa")
    globe.visual(australia_mesh, material=land_green, name="australia")
    globe.visual(
        Cylinder(radius=PIVOT_BUTTON_RADIUS, length=PIVOT_BUTTON_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, pivot_distance)),
        material=dark_metal,
        name="north_button",
    )
    globe.visual(
        Cylinder(radius=PIVOT_BUTTON_RADIUS, length=PIVOT_BUTTON_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -pivot_distance)),
        material=dark_metal,
        name="south_button",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=0.38,
        origin=Origin(),
    )

    model.articulation(
        "base_to_meridian",
        ArticulationType.REVOLUTE,
        parent=base,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, MERIDIAN_CENTER_Z), rpy=(REST_MERIDIAN_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-MERIDIAN_TILT_LIMIT,
            upper=MERIDIAN_TILT_LIMIT,
        ),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, AXIAL_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")
    meridian_tilt = object_model.get_articulation("base_to_meridian")
    globe_spin = object_model.get_articulation("meridian_to_globe")

    foot_disk = base.get_visual("foot_disk")
    left_pivot_cap = base.get_visual("left_pivot_cap")
    right_pivot_cap = base.get_visual("right_pivot_cap")
    meridian_ring = meridian.get_visual("meridian_ring")
    left_trunnion = meridian.get_visual("left_trunnion")
    right_trunnion = meridian.get_visual("right_trunnion")
    north_cap = meridian.get_visual("north_cap")
    south_cap = meridian.get_visual("south_cap")
    ocean_sphere = globe.get_visual("ocean_sphere")
    north_button = globe.get_visual("north_button")
    south_button = globe.get_visual("south_button")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=18, ignore_adjacent=True, ignore_fixed=True)

    ctx.check("base exists", base is not None, "base part missing")
    ctx.check("meridian exists", meridian is not None, "meridian part missing")
    ctx.check("globe exists", globe is not None, "globe part missing")
    ctx.check("meridian tilt articulation exists", meridian_tilt is not None, "tilt joint missing")
    ctx.check("globe spin articulation exists", globe_spin is not None, "globe spin joint missing")

    ctx.expect_origin_gap(
        meridian,
        base,
        axis="z",
        min_gap=0.140,
        max_gap=0.152,
        name="meridian center sits above base",
    )
    ctx.expect_origin_distance(
        globe,
        meridian,
        axes="xy",
        max_dist=0.001,
        name="globe centered in meridian",
    )
    ctx.expect_contact(
        base,
        meridian,
        elem_a=left_pivot_cap,
        elem_b=left_trunnion,
        name="left side trunnion seated in yoke cap",
    )
    ctx.expect_contact(
        base,
        meridian,
        elem_a=right_pivot_cap,
        elem_b=right_trunnion,
        name="right side trunnion seated in yoke cap",
    )
    ctx.expect_contact(
        meridian,
        globe,
        elem_a=north_cap,
        elem_b=north_button,
        name="north polar pivot seated",
    )
    ctx.expect_contact(
        meridian,
        globe,
        elem_a=south_cap,
        elem_b=south_button,
        name="south polar pivot seated",
    )
    ctx.expect_within(
        globe,
        meridian,
        axes="xz",
        margin=0.040,
        inner_elem=ocean_sphere,
        outer_elem=meridian_ring,
        name="globe stays framed by meridian in front view",
    )
    ctx.expect_overlap(
        globe,
        meridian,
        axes="xz",
        min_overlap=0.100,
        elem_a=ocean_sphere,
        elem_b=meridian_ring,
        name="meridian ring visibly encircles globe",
    )
    ctx.expect_within(
        meridian,
        base,
        axes="x",
        margin=0.001,
        inner_elem=meridian_ring,
        outer_elem=foot_disk,
        name="meridian stays within base footprint",
    )

    with ctx.pose({meridian_tilt: MERIDIAN_TILT_LIMIT * 0.8}):
        ctx.expect_contact(
            base,
            meridian,
            elem_a=left_pivot_cap,
            elem_b=left_trunnion,
            name="left trunnion remains seated at forward tilt",
        )
        ctx.expect_contact(
            base,
            meridian,
            elem_a=right_pivot_cap,
            elem_b=right_trunnion,
            name="right trunnion remains seated at forward tilt",
        )
        ctx.expect_contact(
            meridian,
            globe,
            elem_a=north_cap,
            elem_b=north_button,
            name="north pivot remains seated at forward tilt",
        )
        ctx.expect_contact(
            meridian,
            globe,
            elem_a=south_cap,
            elem_b=south_button,
            name="south pivot remains seated at forward tilt",
        )

    with ctx.pose({meridian_tilt: -MERIDIAN_TILT_LIMIT * 0.8, globe_spin: math.pi / 2.0}):
        ctx.expect_contact(
            base,
            meridian,
            elem_a=left_pivot_cap,
            elem_b=left_trunnion,
            name="left trunnion remains seated at reverse tilt",
        )
        ctx.expect_contact(
            base,
            meridian,
            elem_a=right_pivot_cap,
            elem_b=right_trunnion,
            name="right trunnion remains seated at reverse tilt",
        )
        ctx.expect_contact(
            meridian,
            globe,
            elem_a=north_cap,
            elem_b=north_button,
            name="north pivot remains seated while globe spins",
        )
        ctx.expect_contact(
            meridian,
            globe,
            elem_a=south_cap,
            elem_b=south_button,
            name="south pivot remains seated while globe spins",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
