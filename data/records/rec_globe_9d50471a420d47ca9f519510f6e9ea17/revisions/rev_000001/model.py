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
    wrap_profile_onto_surface,
)


GLOBE_RADIUS = 0.120
GLOBE_CENTER_HEIGHT = 0.265
MERIDIAN_RADIUS = 0.152
MERIDIAN_TUBE_RADIUS = 0.0055
POLAR_TILT = math.radians(23.5)
MERIDIAN_REST_TILT = math.radians(-14.0)
MESH_REV = "r4"


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, f"{name}_{MESH_REV}")


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _continent_profile(points: list[tuple[float, float]], *, scale: float) -> list[tuple[float, float]]:
    return [(x * scale, y * scale) for x, y in points]


def _wrapped_continent(
    *,
    name: str,
    profile: list[tuple[float, float]],
    direction: tuple[float, float, float],
    spin: float = 0.0,
):
    return _mesh(
        name,
        wrap_profile_onto_surface(
            profile,
            Sphere(radius=GLOBE_RADIUS),
            thickness=0.0014,
            direction=direction,
            visible_relief=0.00025,
            spin=spin,
            mapping="intrinsic",
            surface_max_edge=0.010,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_world_globe")

    wood = model.material("wood", rgba=(0.33, 0.20, 0.12, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.23, 0.13, 0.08, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.58, 0.24, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.56, 0.43, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    ocean = model.material("ocean", rgba=(0.18, 0.42, 0.69, 1.0))
    land = model.material("land", rgba=(0.77, 0.79, 0.58, 1.0))
    ink = model.material("ink", rgba=(0.09, 0.11, 0.12, 1.0))

    base = model.part("base")

    base_profile = [
        (0.0, 0.000),
        (0.112, 0.000),
        (0.112, 0.010),
        (0.100, 0.016),
        (0.093, 0.040),
        (0.070, 0.056),
        (0.058, 0.066),
        (0.052, 0.070),
        (0.0, 0.070),
    ]
    base.visual(
        _mesh("turned_base", LatheGeometry(base_profile, segments=72)),
        material=wood,
        name="base_plinth",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=dark_wood,
        name="base_collar",
    )

    base.visual(
        Box((0.340, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, -0.018, 0.091)),
        material=brass_dark,
        name="yoke_frame",
    )
    base.visual(
        Box((0.020, 0.020, 0.176)),
        origin=Origin(xyz=(0.170, -0.018, 0.179)),
        material=brass_dark,
        name="left_upright",
    )
    base.visual(
        Box((0.020, 0.020, 0.176)),
        origin=Origin(xyz=(-0.170, -0.018, 0.179)),
        material=brass_dark,
        name="right_upright",
    )
    base.visual(
        Box((0.024, 0.020, 0.048)),
        origin=Origin(xyz=(0.172, -0.010, 0.257)),
        material=brass_dark,
        name="left_support_cheek",
    )
    base.visual(
        Box((0.024, 0.020, 0.048)),
        origin=Origin(xyz=(-0.172, -0.010, 0.257)),
        material=brass_dark,
        name="right_support_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.082),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    meridian = model.part("meridian")
    meridian.visual(
        _mesh(
            "meridian_ring",
            TorusGeometry(
                radius=MERIDIAN_RADIUS,
                tube=MERIDIAN_TUBE_RADIUS,
                radial_segments=18,
                tubular_segments=96,
            ).rotate_x(math.pi / 2.0),
        ),
        material=brass,
        name="meridian_ring",
    )
    meridian.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.152, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    meridian.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(-0.152, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )

    polar_axis = (math.sin(POLAR_TILT), 0.0, math.cos(POLAR_TILT))
    meridian.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=Origin(
            xyz=(polar_axis[0] * 0.136, 0.0, polar_axis[2] * 0.136),
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=steel,
        name="north_receiver",
    )
    meridian.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=Origin(
            xyz=(-polar_axis[0] * 0.136, 0.0, -polar_axis[2] * 0.136),
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=steel,
        name="south_receiver",
    )
    meridian.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.032),
        mass=0.65,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=ocean,
        name="globe_sphere",
    )
    globe.visual(
        Cylinder(radius=0.0036, length=0.018),
        origin=Origin(
            xyz=(polar_axis[0] * 0.117, 0.0, polar_axis[2] * 0.117),
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=steel,
        name="north_pivot",
    )
    globe.visual(
        Cylinder(radius=0.0036, length=0.018),
        origin=Origin(
            xyz=(-polar_axis[0] * 0.117, 0.0, -polar_axis[2] * 0.117),
            rpy=(0.0, POLAR_TILT, 0.0),
        ),
        material=steel,
        name="south_pivot",
    )
    globe.visual(
        _wrapped_continent(
            name="continent_americas",
            profile=_continent_profile(
                [
                    (-0.32, 0.62),
                    (-0.10, 0.78),
                    (0.10, 0.72),
                    (0.20, 0.54),
                    (0.12, 0.28),
                    (0.04, 0.10),
                    (0.10, -0.20),
                    (0.02, -0.52),
                    (-0.12, -0.78),
                    (-0.28, -0.60),
                    (-0.20, -0.18),
                    (-0.36, 0.12),
                    (-0.42, 0.38),
                ],
                scale=0.040,
            ),
            direction=(-0.78, -0.16, 0.20),
            spin=0.20,
        ),
        material=land,
        name="continent_americas",
    )
    globe.visual(
        _wrapped_continent(
            name="continent_eurafrica",
            profile=_continent_profile(
                [
                    (-0.76, 0.46),
                    (-0.40, 0.70),
                    (0.06, 0.72),
                    (0.58, 0.50),
                    (0.72, 0.20),
                    (0.44, 0.06),
                    (0.24, -0.08),
                    (0.34, -0.34),
                    (0.12, -0.70),
                    (-0.10, -0.46),
                    (-0.28, -0.12),
                    (-0.52, 0.02),
                    (-0.76, 0.20),
                ],
                scale=0.042,
            ),
            direction=(0.66, 0.22, 0.18),
            spin=-0.30,
        ),
        material=land,
        name="continent_eurafrica",
    )
    globe.visual(
        _wrapped_continent(
            name="continent_australia",
            profile=_continent_profile(
                [
                    (-0.36, 0.18),
                    (-0.10, 0.34),
                    (0.24, 0.24),
                    (0.38, 0.04),
                    (0.22, -0.22),
                    (-0.08, -0.30),
                    (-0.34, -0.08),
                ],
                scale=0.024,
            ),
            direction=(0.52, -0.60, -0.28),
            spin=0.25,
        ),
        material=land,
        name="continent_australia",
    )
    globe.visual(
        _mesh(
            "equator_line",
            TorusGeometry(
                radius=GLOBE_RADIUS * 0.998,
                tube=0.0014,
                radial_segments=10,
                tubular_segments=80,
            ),
        ),
        origin=Origin(rpy=(0.0, POLAR_TILT, 0.0)),
        material=ink,
        name="equator_line",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=GLOBE_RADIUS),
        mass=0.9,
    )

    model.articulation(
        "base_to_meridian",
        ArticulationType.REVOLUTE,
        parent=base,
        child=meridian,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_HEIGHT), rpy=(MERIDIAN_REST_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=math.radians(-32.0),
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "meridian_to_globe",
        ArticulationType.CONTINUOUS,
        parent=meridian,
        child=globe,
        origin=Origin(),
        axis=polar_axis,
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    meridian = object_model.get_part("meridian")
    globe = object_model.get_part("globe")

    meridian_joint = object_model.get_articulation("base_to_meridian")
    globe_joint = object_model.get_articulation("meridian_to_globe")

    left_cheek = base.get_visual("left_support_cheek")
    right_cheek = base.get_visual("right_support_cheek")
    left_trunnion = meridian.get_visual("left_trunnion")
    right_trunnion = meridian.get_visual("right_trunnion")
    north_receiver = meridian.get_visual("north_receiver")
    south_receiver = meridian.get_visual("south_receiver")
    globe_shell = globe.get_visual("globe_sphere")
    north_pivot = globe.get_visual("north_pivot")
    south_pivot = globe.get_visual("south_pivot")
    base_plinth = base.get_visual("base_plinth")
    meridian_ring = meridian.get_visual("meridian_ring")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a=north_pivot,
        elem_b=north_receiver,
        reason="North polar pivot pin is intentionally captured inside the meridian receiver.",
    )
    ctx.allow_overlap(
        globe,
        meridian,
        elem_a=south_pivot,
        elem_b=south_receiver,
        reason="South polar pivot pin is intentionally captured inside the meridian receiver.",
    )
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
        "meridian_axis_horizontal",
        abs(meridian_joint.axis[0] - 1.0) < 1e-6
        and abs(meridian_joint.axis[1]) < 1e-6
        and abs(meridian_joint.axis[2]) < 1e-6,
        details=f"Unexpected meridian axis: {meridian_joint.axis}",
    )
    ctx.check(
        "globe_axis_is_tilted_polar_axis",
        0.39 < globe_joint.axis[0] < 0.41
        and abs(globe_joint.axis[1]) < 1e-6
        and 0.91 < globe_joint.axis[2] < 0.92,
        details=f"Unexpected globe spin axis: {globe_joint.axis}",
    )

    ctx.expect_contact(
        meridian,
        base,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        contact_tol=0.0015,
        name="left_trunnion_cheek_contact",
    )
    ctx.expect_contact(
        meridian,
        base,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        contact_tol=0.0015,
        name="right_trunnion_cheek_contact",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a=north_pivot,
        elem_b=north_receiver,
        contact_tol=0.0015,
        name="north_pivot_receiver_contact",
    )
    ctx.expect_contact(
        globe,
        meridian,
        elem_a=south_pivot,
        elem_b=south_receiver,
        contact_tol=0.0015,
        name="south_pivot_receiver_contact",
    )
    ctx.expect_origin_gap(
        globe,
        base,
        axis="z",
        min_gap=0.25,
        max_gap=0.28,
        name="globe_center_height",
    )
    ctx.expect_gap(
        globe,
        base,
        axis="z",
        positive_elem=globe_shell,
        negative_elem=base_plinth,
        min_gap=0.070,
        max_gap=0.090,
        name="globe_clears_round_base",
    )
    ctx.expect_overlap(
        globe,
        meridian,
        axes="xz",
        elem_a=globe_shell,
        elem_b=meridian_ring,
        min_overlap=0.22,
        name="meridian_encircles_globe_projection",
    )

    meridian_limits = meridian_joint.motion_limits
    if meridian_limits is not None and meridian_limits.lower is not None and meridian_limits.upper is not None:
        with ctx.pose({meridian_joint: meridian_limits.lower}):
            ctx.expect_contact(meridian, base, elem_a=left_trunnion, elem_b=left_cheek, contact_tol=0.0015)
            ctx.expect_contact(meridian, base, elem_a=right_trunnion, elem_b=right_cheek, contact_tol=0.0015)
            ctx.expect_contact(globe, meridian, elem_a=north_pivot, elem_b=north_receiver, contact_tol=0.0015)
            ctx.expect_contact(globe, meridian, elem_a=south_pivot, elem_b=south_receiver, contact_tol=0.0015)
            ctx.fail_if_parts_overlap_in_current_pose(name="meridian_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="meridian_lower_no_floating")

        with ctx.pose({meridian_joint: meridian_limits.upper}):
            ctx.expect_contact(meridian, base, elem_a=left_trunnion, elem_b=left_cheek, contact_tol=0.0015)
            ctx.expect_contact(meridian, base, elem_a=right_trunnion, elem_b=right_cheek, contact_tol=0.0015)
            ctx.expect_contact(globe, meridian, elem_a=north_pivot, elem_b=north_receiver, contact_tol=0.0015)
            ctx.expect_contact(globe, meridian, elem_a=south_pivot, elem_b=south_receiver, contact_tol=0.0015)
            ctx.fail_if_parts_overlap_in_current_pose(name="meridian_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="meridian_upper_no_floating")

    for index, spin_angle in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        with ctx.pose({globe_joint: spin_angle}):
            ctx.expect_contact(
                globe,
                meridian,
                elem_a=north_pivot,
                elem_b=north_receiver,
                contact_tol=0.0015,
                name=f"north_pivot_contact_spin_pose_{index}",
            )
            ctx.expect_contact(
                globe,
                meridian,
                elem_a=south_pivot,
                elem_b=south_receiver,
                contact_tol=0.0015,
                name=f"south_pivot_contact_spin_pose_{index}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"spin_pose_{index}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"spin_pose_{index}_no_floating")

    if meridian_limits is not None and meridian_limits.upper is not None:
        with ctx.pose({meridian_joint: meridian_limits.upper, globe_joint: math.pi / 2.0}):
            ctx.expect_contact(globe, meridian, elem_a=north_pivot, elem_b=north_receiver, contact_tol=0.0015)
            ctx.expect_contact(globe, meridian, elem_a=south_pivot, elem_b=south_receiver, contact_tol=0.0015)
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
