from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, radians, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.120
POLAR_TILT = radians(23.5)


def _sphere_point(radius: float, latitude_deg: float, longitude_deg: float) -> tuple[float, float, float]:
    lat = radians(latitude_deg)
    lon = radians(longitude_deg)
    xy = radius * cos(lat)
    return (xy * cos(lon), xy * sin(lon), radius * sin(lat))


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _distance(a, b) -> float:
    return sqrt(sum((a[idx] - b[idx]) ** 2 for idx in range(3)))


def _midpoint(a, b):
    return tuple((a[idx] + b[idx]) * 0.5 for idx in range(3))


def _segment_pose(start, end) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    planar = sqrt(dx * dx + dy * dy)
    return length, Origin(
        xyz=_midpoint(start, end),
        rpy=(0.0, atan2(planar, dz), atan2(dy, dx)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="celestial_globe")

    base_black = model.material("base_black", rgba=(0.10, 0.09, 0.08, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.57, 0.30, 1.0))
    antique_gold = model.material("antique_gold", rgba=(0.79, 0.69, 0.38, 1.0))
    globe_blue = model.material("globe_blue", rgba=(0.11, 0.18, 0.34, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.160, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=base_black,
        name="base_plinth",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, 0.044)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brass,
        name="turntable_plate",
    )
    cradle.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brass,
        name="turntable_hub",
    )

    lower_pivot_center = (0.0, 0.147 * sin(POLAR_TILT), 0.205 - 0.147 * cos(POLAR_TILT))
    upper_pivot_center = (0.0, -0.147 * sin(POLAR_TILT), 0.205 + 0.147 * cos(POLAR_TILT))

    lower_post_base = (-0.148, lower_pivot_center[1], 0.025)
    upper_post_base = (-0.148, upper_pivot_center[1], 0.025)
    lower_foot_anchor = (-0.018, 0.022, 0.025)
    upper_foot_anchor = (-0.018, -0.022, 0.025)

    lower_foot_length, lower_foot_origin = _segment_pose(lower_foot_anchor, lower_post_base)
    cradle.visual(
        Cylinder(radius=0.010, length=lower_foot_length),
        origin=lower_foot_origin,
        material=brass,
        name="lower_foot",
    )
    upper_foot_length, upper_foot_origin = _segment_pose(upper_foot_anchor, upper_post_base)
    cradle.visual(
        Cylinder(radius=0.010, length=upper_foot_length),
        origin=upper_foot_origin,
        material=brass,
        name="upper_foot",
    )

    cradle.visual(
        Cylinder(radius=0.010, length=lower_pivot_center[2] - lower_post_base[2]),
        origin=Origin(
            xyz=(
                lower_post_base[0],
                lower_post_base[1],
                0.5 * (lower_post_base[2] + lower_pivot_center[2]),
            )
        ),
        material=brass,
        name="lower_support",
    )
    cradle.visual(
        Cylinder(radius=0.010, length=upper_pivot_center[2] - upper_post_base[2]),
        origin=Origin(
            xyz=(
                upper_post_base[0],
                upper_post_base[1],
                0.5 * (upper_post_base[2] + upper_pivot_center[2]),
            )
        ),
        material=brass,
        name="upper_support",
    )

    arm_radius = 0.0075
    arm_length = abs(lower_post_base[0] - lower_pivot_center[0])
    cradle.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(
                0.5 * (lower_post_base[0] + lower_pivot_center[0]),
                lower_pivot_center[1],
                lower_pivot_center[2],
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=brass,
        name="lower_arm",
    )
    cradle.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(
                0.5 * (upper_post_base[0] + upper_pivot_center[0]),
                upper_pivot_center[1],
                upper_pivot_center[2],
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=brass,
        name="upper_arm",
    )
    cradle.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=lower_pivot_center, rpy=(POLAR_TILT, 0.0, 0.0)),
        material=antique_gold,
        name="lower_pivot",
    )
    cradle.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=upper_pivot_center, rpy=(POLAR_TILT, 0.0, 0.0)),
        material=antique_gold,
        name="upper_pivot",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.120, 0.160, 0.360)),
        mass=1.4,
        origin=Origin(xyz=(-0.010, -0.005, 0.180)),
    )

    globe = model.part("globe")
    globe.visual(Sphere(radius=GLOBE_RADIUS), material=globe_blue, name="globe_shell")
    globe.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=antique_gold,
        name="north_stub",
    )
    globe.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=antique_gold,
        name="south_stub",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(radius=0.1185, tube=0.0018), "equator_ring"),
        material=antique_gold,
        name="equator_ring",
    )
    globe.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=_sphere_point(0.114, 18.0, 32.0)),
        material=antique_gold,
        name="constellation_marker",
    )
    globe.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=_sphere_point(0.116, -22.0, -65.0)),
        material=antique_gold,
        name="southern_marker",
    )
    globe.inertial = Inertial.from_geometry(Sphere(radius=0.122), mass=0.9)

    model.articulation(
        "base_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "cradle_to_globe",
        ArticulationType.CONTINUOUS,
        parent=cradle,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.205), rpy=(POLAR_TILT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    base = object_model.get_part("base")
    cradle = object_model.get_part("cradle")
    globe = object_model.get_part("globe")
    base_to_cradle = object_model.get_articulation("base_to_cradle")
    cradle_to_globe = object_model.get_articulation("cradle_to_globe")

    ctx.check(
        "primary articulations are continuous",
        base_to_cradle.articulation_type == ArticulationType.CONTINUOUS
        and cradle_to_globe.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"base_to_cradle={base_to_cradle.articulation_type}, "
            f"cradle_to_globe={cradle_to_globe.articulation_type}"
        ),
    )
    ctx.check(
        "turntable axis is vertical",
        tuple(base_to_cradle.axis) == (0.0, 0.0, 1.0),
        details=f"axis={base_to_cradle.axis}",
    )
    ctx.check(
        "globe polar axis is tilted in the cradle",
        tuple(cradle_to_globe.axis) == (0.0, 0.0, 1.0)
        and abs(cradle_to_globe.origin.rpy[0] - POLAR_TILT) < 1e-6,
        details=f"origin_rpy={cradle_to_globe.origin.rpy}, axis={cradle_to_globe.axis}",
    )
    ctx.expect_gap(
        cradle,
        base,
        axis="z",
        positive_elem="turntable_plate",
        negative_elem="base_plinth",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable plate sits on the base plinth",
    )
    ctx.expect_contact(
        globe,
        cradle,
        elem_a="north_stub",
        elem_b="upper_pivot",
        contact_tol=0.0015,
        name="north polar stub reaches the upper pivot",
    )
    ctx.expect_contact(
        globe,
        cradle,
        elem_a="south_stub",
        elem_b="lower_pivot",
        contact_tol=0.0015,
        name="south polar stub reaches the lower pivot",
    )

    globe_origin_rest = ctx.part_world_position(globe)
    marker_rest = _aabb_center(ctx.part_element_world_aabb(globe, elem="constellation_marker"))
    with ctx.pose({cradle_to_globe: pi / 2.0}):
        globe_origin_spun = ctx.part_world_position(globe)
        marker_spun = _aabb_center(ctx.part_element_world_aabb(globe, elem="constellation_marker"))
    ctx.check(
        "globe spins about its polar axis in place",
        globe_origin_rest is not None
        and globe_origin_spun is not None
        and marker_rest is not None
        and marker_spun is not None
        and _distance(globe_origin_rest, globe_origin_spun) < 1e-6
        and _distance(marker_rest, marker_spun) > 0.040,
        details=(
            f"globe_origin_rest={globe_origin_rest}, globe_origin_spun={globe_origin_spun}, "
            f"marker_rest={marker_rest}, marker_spun={marker_spun}"
        ),
    )

    upper_pivot_rest = _aabb_center(ctx.part_element_world_aabb(cradle, elem="upper_pivot"))
    with ctx.pose({base_to_cradle: pi / 2.0}):
        upper_pivot_turned = _aabb_center(ctx.part_element_world_aabb(cradle, elem="upper_pivot"))
    ctx.check(
        "cradle turns about the vertical turntable axis",
        upper_pivot_rest is not None
        and upper_pivot_turned is not None
        and upper_pivot_turned[0] > 0.040
        and abs(upper_pivot_turned[1]) < 0.012
        and abs(upper_pivot_turned[2] - upper_pivot_rest[2]) < 1e-6,
        details=f"rest={upper_pivot_rest}, turned={upper_pivot_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
