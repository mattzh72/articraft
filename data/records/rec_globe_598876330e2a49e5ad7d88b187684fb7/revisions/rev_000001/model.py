from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canted_single_arm_globe")

    graphite = Material("soft_graphite", rgba=(0.055, 0.058, 0.060, 1.0))
    dark_metal = Material("dark_burnished_metal", rgba=(0.16, 0.15, 0.13, 1.0))
    brass = Material("warm_brushed_brass", rgba=(0.78, 0.58, 0.30, 1.0))
    ocean = Material("matte_deep_ocean", rgba=(0.05, 0.18, 0.32, 1.0))
    pale_line = Material("pale_engraved_lines", rgba=(0.78, 0.82, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.205, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=graphite,
        name="fixed_foot",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.176, tube=0.010, radial_segments=18, tubular_segments=72), "base_outer_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_metal,
        name="outer_ring",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.172, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_metal,
        name="turntable_disc",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.158, tube=0.008, radial_segments=18, tubular_segments=72), "pedestal_raised_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=brass,
        name="raised_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=brass,
        name="center_boss",
    )

    globe_radius = 0.130
    globe_center = (0.120, 0.0, 0.420)
    tilt = math.radians(25.0)
    polar_axis = (-math.sin(tilt), 0.0, math.cos(tilt))

    def along_polar(distance: float) -> tuple[float, float, float]:
        return (
            globe_center[0] + polar_axis[0] * distance,
            globe_center[1] + polar_axis[1] * distance,
            globe_center[2] + polar_axis[2] * distance,
        )

    pin_inner = along_polar(globe_radius + 0.022)
    pin_outer = along_polar(globe_radius + 0.078)
    pin_center = along_polar(globe_radius + 0.050)

    support_arm = tube_from_spline_points(
        [
            (-0.035, -0.030, 0.086),
            (-0.076, -0.098, 0.245),
            (-0.050, -0.092, 0.455),
            pin_outer,
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    pedestal.visual(
        mesh_from_geometry(support_arm, "canted_support_arm"),
        material=brass,
        name="support_arm",
    )
    pedestal.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=pin_outer),
        material=brass,
        name="bearing_knuckle",
    )
    pedestal.visual(
        Cylinder(radius=0.012, length=0.056),
        origin=Origin(xyz=pin_center, rpy=(0.0, -tilt, 0.0)),
        material=brass,
        name="spindle_pin",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=globe_radius),
        material=ocean,
        name="sphere_shell",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(radius=globe_radius * 1.006, tube=0.0032, radial_segments=12, tubular_segments=80), "globe_equator_ring"),
        material=pale_line,
        name="equator_ring",
    )
    meridian_mesh = mesh_from_geometry(
        TorusGeometry(radius=globe_radius * 1.008, tube=0.0030, radial_segments=12, tubular_segments=80),
        "globe_meridian_ring",
    )
    globe.visual(
        meridian_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pale_line,
        name="meridian_ring_0",
    )
    globe.visual(
        meridian_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, math.radians(60.0))),
        material=pale_line,
        name="meridian_ring_1",
    )
    globe.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, globe_radius + 0.011)),
        material=brass,
        name="polar_socket",
    )
    globe.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -(globe_radius + 0.005))),
        material=brass,
        name="polar_cap",
    )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "pedestal_to_globe",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=globe,
        origin=Origin(xyz=globe_center, rpy=(0.0, -tilt, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    globe = object_model.get_part("globe")
    base_spin = object_model.get_articulation("base_to_pedestal")
    globe_spin = object_model.get_articulation("pedestal_to_globe")

    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        positive_elem="turntable_disc",
        negative_elem="fixed_foot",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on fixed foot",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="xy",
        elem_a="turntable_disc",
        elem_b="fixed_foot",
        min_overlap=0.25,
        name="pedestal is centered over base bearing",
    )
    ctx.expect_contact(
        globe,
        pedestal,
        elem_a="polar_socket",
        elem_b="spindle_pin",
        contact_tol=0.003,
        name="single polar socket is carried by spindle",
    )

    rest_pedestal_position = ctx.part_world_position(pedestal)
    with ctx.pose({base_spin: math.pi / 2.0}):
        spun_pedestal_position = ctx.part_world_position(pedestal)
    ctx.check(
        "pedestal spins about vertical base axis",
        rest_pedestal_position is not None
        and spun_pedestal_position is not None
        and abs(rest_pedestal_position[0] - spun_pedestal_position[0]) < 1e-6
        and abs(rest_pedestal_position[1] - spun_pedestal_position[1]) < 1e-6,
        details=f"rest={rest_pedestal_position}, spun={spun_pedestal_position}",
    )

    meridian_rest = ctx.part_element_world_aabb(globe, elem="meridian_ring_0")
    with ctx.pose({globe_spin: math.pi / 2.0}):
        meridian_spun = ctx.part_element_world_aabb(globe, elem="meridian_ring_0")
    ctx.check(
        "globe markings rotate about canted polar axis",
        meridian_rest is not None
        and meridian_spun is not None
        and abs((meridian_rest[1][1] - meridian_rest[0][1]) - (meridian_spun[1][1] - meridian_spun[0][1])) > 0.01,
        details=f"rest={meridian_rest}, spun={meridian_spun}",
    )

    return ctx.report()


object_model = build_object_model()
