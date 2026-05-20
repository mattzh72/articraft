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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satcom_dish_az_el_mount", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.70, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.43, 0.46, 0.50, 1.0))
    off_white = model.material("off_white", rgba=(0.90, 0.92, 0.94, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    horn_beige = model.material("horn_beige", rgba=(0.73, 0.69, 0.55, 1.0))

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    reflector_shell = _mesh(
        "reflector_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0, -0.205),
                (0.070, -0.202),
                (0.200, -0.190),
                (0.380, -0.160),
                (0.550, -0.115),
                (0.670, -0.060),
                (0.725, 0.0),
            ],
            [
                (0.0, -0.193),
                (0.060, -0.191),
                (0.180, -0.181),
                (0.350, -0.154),
                (0.520, -0.112),
                (0.655, -0.065),
                (0.713, -0.012),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    rim_ring = _mesh(
        "rim_ring.obj",
        TorusGeometry(radius=0.719, tube=0.012, radial_segments=16, tubular_segments=72),
    )
    rear_rib = _mesh(
        "rear_rib.obj",
        tube_from_spline_points(
            [
                (0.120, 0.0, 0.095),
                (0.155, 0.0, 0.120),
                (0.205, 0.0, 0.190),
                (0.245, 0.0, 0.265),
                (0.278, 0.0, 0.340),
            ],
            radius=0.013,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    feed_boom = _mesh(
        "feed_boom.obj",
        tube_from_spline_points(
            [
                (0.030, 0.0, -0.015),
                (0.240, 0.0, -0.010),
                (0.500, 0.0, -0.004),
                (0.800, 0.0, 0.000),
            ],
            radius=0.013,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    feed_brace = _mesh(
        "feed_brace.obj",
        tube_from_spline_points(
            [
                (0.285, 0.0, 0.255),
                (0.430, 0.0, 0.125),
                (0.610, 0.0, 0.020),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    feed_horn_mesh = _mesh(
        "feed_horn.obj",
        LatheGeometry(
            [
                (0.0, -0.090),
                (0.026, -0.090),
                (0.032, -0.040),
                (0.042, 0.015),
                (0.052, 0.065),
                (0.057, 0.090),
                (0.0, 0.090),
            ],
            segments=48,
        ),
    )
    yoke_arm = _mesh(
        "yoke_arm.obj",
        sweep_profile_along_spline(
            [
                (-0.060, 0.0, 0.150),
                (-0.010, 0.0, 0.310),
                (0.065, 0.0, 0.500),
                (0.140, 0.0, 0.635),
            ],
            profile=rounded_rect_profile(0.085, 0.120, radius=0.018, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((1.20, 1.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="foundation",
    )
    pedestal.visual(
        Cylinder(radius=0.180, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=mount_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.28, 0.22, 0.42)),
        origin=Origin(xyz=(-0.26, 0.0, 0.33)),
        material=dark_steel,
        name="service_box",
    )
    pedestal.visual(
        Cylinder(radius=0.280, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.32)),
        material=dark_steel,
        name="azimuth_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.200, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.43)),
        material=aluminum,
        name="bearing_cap",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 1.46)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
    )

    azimuth_yoke = model.part("azimuth_yoke")
    azimuth_yoke.visual(
        Cylinder(radius=0.190, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="turntable_drum",
    )
    azimuth_yoke.visual(
        Cylinder(radius=0.300, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=mount_gray,
        name="upper_deck",
    )
    azimuth_yoke.visual(
        Box((0.16, 0.40, 0.38)),
        origin=Origin(xyz=(-0.020, 0.0, 0.33)),
        material=mount_gray,
        name="mast_block",
    )
    azimuth_yoke.visual(
        Box((0.12, 0.92, 0.12)),
        origin=Origin(xyz=(-0.020, 0.0, 0.36)),
        material=dark_steel,
        name="crosshead",
    )
    azimuth_yoke.visual(
        yoke_arm,
        origin=Origin(xyz=(0.0, 0.470, 0.0)),
        material=off_white,
        name="positive_yoke_arm",
    )
    azimuth_yoke.visual(
        yoke_arm,
        origin=Origin(xyz=(0.0, -0.470, 0.0)),
        material=off_white,
        name="negative_yoke_arm",
    )
    azimuth_yoke.visual(
        Cylinder(radius=0.090, length=0.18),
        origin=Origin(xyz=(0.140, 0.480, 0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="positive_bearing",
    )
    azimuth_yoke.visual(
        Cylinder(radius=0.090, length=0.18),
        origin=Origin(xyz=(0.140, -0.480, 0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="negative_bearing",
    )
    azimuth_yoke.inertial = Inertial.from_geometry(
        Box((0.70, 1.12, 0.78)),
        mass=95.0,
        origin=Origin(xyz=(0.020, 0.0, 0.39)),
    )

    reflector = model.part("reflector")
    reflector.visual(
        Cylinder(radius=0.060, length=0.28),
        origin=Origin(xyz=(0.0, 0.250, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="positive_trunnion",
    )
    reflector.visual(
        Cylinder(radius=0.060, length=0.28),
        origin=Origin(xyz=(0.0, -0.250, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="negative_trunnion",
    )
    reflector.visual(
        Box((0.16, 0.46, 0.20)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=mount_gray,
        name="hub_block",
    )
    reflector.visual(
        Cylinder(radius=0.100, length=0.22),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    for index in range(6):
        angle = (2.0 * math.pi * index) / 6.0
        reflector.visual(
            rear_rib,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=dark_steel,
            name=f"rear_rib_{index}",
        )
    reflector.visual(
        reflector_shell,
        origin=Origin(xyz=(0.480, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="reflector_shell",
    )
    reflector.visual(
        rim_ring,
        origin=Origin(xyz=(0.480, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="rim_ring",
    )
    reflector.visual(
        feed_boom,
        material=aluminum,
        name="feed_boom",
    )
    reflector.visual(
        feed_brace,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="feed_brace_positive",
    )
    reflector.visual(
        feed_brace,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="feed_brace_negative",
    )
    reflector.visual(
        Box((0.06, 0.10, 0.08)),
        origin=Origin(xyz=(0.760, 0.0, 0.0)),
        material=dark_steel,
        name="feed_support_block",
    )
    reflector.visual(
        feed_horn_mesh,
        origin=Origin(xyz=(0.860, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=horn_beige,
        name="feed_horn",
    )
    reflector.inertial = Inertial.from_geometry(
        Box((1.05, 1.46, 1.46)),
        mass=52.0,
        origin=Origin(xyz=(0.470, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2),
    )
    model.articulation(
        "dish_elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_yoke,
        child=reflector,
        origin=Origin(xyz=(0.140, 0.0, 0.640)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.9,
            lower=math.radians(10.0),
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    azimuth_yoke = object_model.get_part("azimuth_yoke")
    reflector = object_model.get_part("reflector")
    azimuth_yaw = object_model.get_articulation("azimuth_yaw")
    dish_elevation = object_model.get_articulation("dish_elevation")

    bearing_cap = pedestal.get_visual("bearing_cap")
    turntable_drum = azimuth_yoke.get_visual("turntable_drum")
    positive_bearing = azimuth_yoke.get_visual("positive_bearing")
    negative_bearing = azimuth_yoke.get_visual("negative_bearing")
    positive_trunnion = reflector.get_visual("positive_trunnion")
    negative_trunnion = reflector.get_visual("negative_trunnion")
    reflector_shell = reflector.get_visual("reflector_shell")
    feed_horn = reflector.get_visual("feed_horn")

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

    ctx.expect_contact(
        azimuth_yoke,
        pedestal,
        elem_a=turntable_drum,
        elem_b=bearing_cap,
        name="turntable_contacts_bearing_cap",
    )
    ctx.expect_gap(
        azimuth_yoke,
        pedestal,
        axis="z",
        positive_elem=turntable_drum,
        negative_elem=bearing_cap,
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable_seats_on_pedestal_cap",
    )
    ctx.expect_overlap(
        azimuth_yoke,
        pedestal,
        axes="xy",
        min_overlap=0.18,
        elem_a=turntable_drum,
        elem_b=bearing_cap,
        name="turntable_centered_over_pedestal_cap",
    )

    ctx.expect_gap(
        azimuth_yoke,
        reflector,
        axis="y",
        positive_elem=positive_bearing,
        negative_elem=positive_trunnion,
        max_gap=0.001,
        max_penetration=0.0,
        name="positive_trunnion_seats_in_bearing",
    )
    ctx.expect_gap(
        reflector,
        azimuth_yoke,
        axis="y",
        positive_elem=negative_trunnion,
        negative_elem=negative_bearing,
        max_gap=0.001,
        max_penetration=0.0,
        name="negative_trunnion_seats_in_bearing",
    )
    ctx.expect_overlap(
        reflector,
        azimuth_yoke,
        axes="xz",
        min_overlap=0.10,
        elem_a=positive_trunnion,
        elem_b=positive_bearing,
        name="positive_trunnion_aligned_with_bearing",
    )
    ctx.expect_overlap(
        reflector,
        azimuth_yoke,
        axes="xz",
        min_overlap=0.10,
        elem_a=negative_trunnion,
        elem_b=negative_bearing,
        name="negative_trunnion_aligned_with_bearing",
    )
    ctx.expect_overlap(
        reflector,
        azimuth_yoke,
        axes="z",
        min_overlap=0.12,
        elem_a=reflector_shell,
        elem_b=positive_bearing,
        name="reflector_axis_runs_through_yoke_height",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    with ctx.pose({azimuth_yaw: 0.0, dish_elevation: math.radians(10.0)}):
        low_feed = _center_from_aabb(ctx.part_element_world_aabb(reflector, elem=feed_horn))
    with ctx.pose({azimuth_yaw: 0.0, dish_elevation: math.radians(80.0)}):
        high_feed = _center_from_aabb(ctx.part_element_world_aabb(reflector, elem=feed_horn))

    ctx.check(
        "elevation_raises_feed_horn",
        low_feed is not None and high_feed is not None and high_feed[2] > low_feed[2] + 0.55,
        details=f"low={low_feed}, high={high_feed}",
    )
    ctx.check(
        "elevation_swings_feed_back_over_axis",
        low_feed is not None and high_feed is not None and high_feed[0] < low_feed[0] - 0.55,
        details=f"low={low_feed}, high={high_feed}",
    )
    ctx.check(
        "elevation_preserves_feed_midplane",
        low_feed is not None
        and high_feed is not None
        and abs(low_feed[1]) < 0.02
        and abs(high_feed[1]) < 0.02,
        details=f"low={low_feed}, high={high_feed}",
    )

    with ctx.pose({azimuth_yaw: 0.0, dish_elevation: math.radians(35.0)}):
        yaw_zero_feed = _center_from_aabb(ctx.part_element_world_aabb(reflector, elem=feed_horn))
    with ctx.pose({azimuth_yaw: math.pi / 2.0, dish_elevation: math.radians(35.0)}):
        yaw_quarter_feed = _center_from_aabb(ctx.part_element_world_aabb(reflector, elem=feed_horn))

    zero_radius = None if yaw_zero_feed is None else math.hypot(yaw_zero_feed[0], yaw_zero_feed[1])
    quarter_radius = (
        None if yaw_quarter_feed is None else math.hypot(yaw_quarter_feed[0], yaw_quarter_feed[1])
    )
    ctx.check(
        "azimuth_rotates_feed_around_vertical_axis",
        yaw_zero_feed is not None
        and yaw_quarter_feed is not None
        and abs(yaw_zero_feed[2] - yaw_quarter_feed[2]) < 1e-3
        and abs(yaw_quarter_feed[0]) < 0.12
        and yaw_quarter_feed[1] > 0.55,
        details=f"yaw0={yaw_zero_feed}, yaw90={yaw_quarter_feed}",
    )
    ctx.check(
        "azimuth_preserves_horizontal_radius",
        zero_radius is not None
        and quarter_radius is not None
        and abs(zero_radius - quarter_radius) < 0.01,
        details=f"r0={zero_radius}, r90={quarter_radius}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
