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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member_visual(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_cylinder(start, end)),
        material=material,
        name=name,
    )


def _vec_scale(
    v: tuple[float, float, float], s: float
) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _vec_add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_globe")

    walnut = model.material("walnut", rgba=(0.30, 0.18, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.63, 0.32, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.56, 0.45, 0.22, 1.0))
    globe_ocean = model.material("globe_ocean", rgba=(0.18, 0.31, 0.48, 1.0))
    globe_land = model.material("globe_land", rgba=(0.76, 0.71, 0.58, 1.0))

    base_radius = 0.072
    base_height = 0.022
    top_bead_radius = 0.056
    top_bead_height = 0.006

    tilt = math.radians(23.5)
    globe_radius = 0.086
    bushing_radius = 0.006
    bushing_length = 0.008
    globe_center = (0.0, 0.0, 0.138)
    meridian_radius = 0.110
    support_axis = (math.sin(tilt), 0.0, math.cos(tilt))
    rear_vector = (-math.cos(tilt), 0.0, math.sin(tilt))

    lower_hub = _vec_add(globe_center, _vec_scale(support_axis, -meridian_radius))
    upper_hub = _vec_add(globe_center, _vec_scale(support_axis, meridian_radius))

    upper_pin_end = _vec_add(
        globe_center, _vec_scale(support_axis, globe_radius + bushing_length)
    )
    lower_pin_end = _vec_add(
        globe_center, _vec_scale(support_axis, -(globe_radius + bushing_length))
    )
    upper_pin_start = _vec_add(
        globe_center, _vec_scale(support_axis, meridian_radius - 0.006)
    )
    lower_pin_start = _vec_add(
        globe_center, _vec_scale(support_axis, -(meridian_radius - 0.006))
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=walnut,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=top_bead_radius, length=top_bead_height),
        origin=Origin(xyz=(0.0, 0.0, base_height + top_bead_height * 0.5)),
        material=dark_brass,
        name="base_bead",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_radius * 2.0, base_radius * 2.0, 0.040)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.042, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brass,
        name="collar_shell",
    )
    support.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_brass,
        name="collar_cap",
    )

    _add_member_visual(
        support,
        start=(0.0, 0.0, 0.010),
        end=(lower_hub[0] + 0.004, lower_hub[1], lower_hub[2] + 0.004),
        radius=0.0085,
        material=brass,
        name="support_stem",
    )

    meridian_points = []
    for step in range(11):
        theta = -math.pi * 0.5 + math.pi * step / 10.0
        ring_offset = _vec_add(
            _vec_scale(rear_vector, math.cos(theta) * meridian_radius),
            _vec_scale(support_axis, math.sin(theta) * meridian_radius),
        )
        meridian_points.append(_vec_add(globe_center, ring_offset))
    meridian_mesh = mesh_from_geometry(
        tube_from_spline_points(
            meridian_points,
            radius=0.0065,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "executive_globe_meridian",
    )
    support.visual(
        meridian_mesh,
        material=brass,
        name="meridian_arc",
    )

    support.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=lower_hub),
        material=dark_brass,
        name="lower_hub",
    )
    support.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=upper_hub),
        material=dark_brass,
        name="upper_hub",
    )

    _add_member_visual(
        support,
        start=lower_pin_start,
        end=lower_pin_end,
        radius=0.0045,
        material=dark_brass,
        name="lower_pivot_pin",
    )
    _add_member_visual(
        support,
        start=upper_pin_start,
        end=upper_pin_end,
        radius=0.0045,
        material=dark_brass,
        name="upper_pivot_pin",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.24, 0.09, 0.26)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        material=globe_ocean,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=bushing_radius, length=bushing_length),
        origin=Origin(xyz=(0.0, 0.0, globe_radius + bushing_length * 0.5)),
        material=brass,
        name="north_bushing",
    )
    globe.visual(
        Cylinder(radius=bushing_radius, length=bushing_length),
        origin=Origin(xyz=(0.0, 0.0, -(globe_radius + bushing_length * 0.5))),
        material=brass,
        name="south_bushing",
    )
    globe.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.058, 0.020, 0.033)),
        material=globe_land,
        name="land_marker",
    )
    globe.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.032, -0.028, -0.010)),
        material=globe_land,
        name="land_marker_secondary",
    )
    globe.inertial = Inertial.from_geometry(
        Sphere(radius=globe_radius),
        mass=0.8,
    )

    model.articulation(
        "base_to_support",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, base_height + top_bead_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.6),
    )
    model.articulation(
        "support_to_globe",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=globe,
        origin=Origin(xyz=globe_center, rpy=(0.0, tilt, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    support = object_model.get_part("support")
    globe = object_model.get_part("globe")
    swivel = object_model.get_articulation("base_to_support")
    spin = object_model.get_articulation("support_to_globe")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({swivel: 0.0, spin: 0.0}):
        ctx.expect_gap(
            support,
            base,
            axis="z",
            positive_elem="collar_shell",
            negative_elem="base_bead",
            max_gap=0.001,
            max_penetration=0.0,
            name="swivel collar seats cleanly on the base",
        )
        ctx.expect_contact(
            globe,
            support,
            elem_a="north_bushing",
            elem_b="upper_pivot_pin",
            contact_tol=0.001,
            name="north pivot pin meets the globe bushing",
        )
        ctx.expect_contact(
            globe,
            support,
            elem_a="south_bushing",
            elem_b="lower_pivot_pin",
            contact_tol=0.001,
            name="south pivot pin meets the globe bushing",
        )

        upper_pin_rest = aabb_center(ctx.part_element_world_aabb(support, elem="upper_pivot_pin"))
        marker_rest = aabb_center(ctx.part_element_world_aabb(globe, elem="land_marker"))
        globe_origin_rest = ctx.part_world_position(globe)

    with ctx.pose({swivel: math.pi * 0.5, spin: 0.0}):
        upper_pin_swiveled = aabb_center(ctx.part_element_world_aabb(support, elem="upper_pivot_pin"))

    ctx.check(
        "support fork swivels about the vertical base collar",
        upper_pin_rest is not None
        and upper_pin_swiveled is not None
        and abs(upper_pin_rest[0] - upper_pin_swiveled[0]) > 0.020
        and abs(upper_pin_rest[1] - upper_pin_swiveled[1]) > 0.020,
        details=f"rest={upper_pin_rest}, swiveled={upper_pin_swiveled}",
    )

    with ctx.pose({swivel: 0.0, spin: math.pi * 0.5}):
        marker_spun = aabb_center(ctx.part_element_world_aabb(globe, elem="land_marker"))
        globe_origin_spun = ctx.part_world_position(globe)

    ctx.check(
        "globe spins about its tilted polar axis",
        marker_rest is not None
        and marker_spun is not None
        and (
            abs(marker_rest[0] - marker_spun[0]) > 0.012
            or abs(marker_rest[1] - marker_spun[1]) > 0.012
            or abs(marker_rest[2] - marker_spun[2]) > 0.012
        ),
        details=f"rest={marker_rest}, spun={marker_spun}",
    )
    ctx.check(
        "globe spin keeps the globe centered between the pivots",
        globe_origin_rest is not None
        and globe_origin_spun is not None
        and abs(globe_origin_rest[0] - globe_origin_spun[0]) < 1e-6
        and abs(globe_origin_rest[1] - globe_origin_spun[1]) < 1e-6
        and abs(globe_origin_rest[2] - globe_origin_spun[2]) < 1e-6,
        details=f"rest={globe_origin_rest}, spun={globe_origin_spun}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
