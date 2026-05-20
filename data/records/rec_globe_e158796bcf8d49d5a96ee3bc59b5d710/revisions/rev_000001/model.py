from __future__ import annotations

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
    TorusGeometry,
    mesh_from_geometry,
)


GLOBE_RADIUS = 0.25
GLOBE_CENTER_Z = 0.52


def _torus_mesh(name: str, radius: float, tube: float):
    return mesh_from_geometry(
        TorusGeometry(radius=radius, tube=tube, radial_segments=14, tubular_segments=72),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="celestial_globe_cradle")

    walnut = model.material("dark_walnut", rgba=(0.28, 0.16, 0.08, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.82, 0.62, 0.28, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.57, 0.43, 0.20, 1.0))
    night_blue = model.material("night_sky_blue", rgba=(0.02, 0.07, 0.22, 1.0))
    star_ink = model.material("warm_star_ink", rgba=(1.00, 0.92, 0.72, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.03, 0.03, 0.03, 1.0))

    equator_mesh = _torus_mesh("equator_band", GLOBE_RADIUS + 0.002, 0.0045)
    meridian_mesh = _torus_mesh("meridian_band", GLOBE_RADIUS + 0.001, 0.0035)
    ecliptic_mesh = _torus_mesh("ecliptic_band", GLOBE_RADIUS + 0.004, 0.0035)
    bearing_ring_mesh = _torus_mesh("fixed_bearing_ring_mesh", 0.30, 0.010)
    latitude_meshes = {
        "latitude_30": _torus_mesh("latitude_30_band", GLOBE_RADIUS * math.cos(math.radians(30.0)), 0.0026),
        "latitude_60": _torus_mesh("latitude_60_band", GLOBE_RADIUS * math.cos(math.radians(60.0)), 0.0024),
    }

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.43, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=walnut,
        name="round_plinth",
    )
    base.visual(
        bearing_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=satin_brass,
        name="fixed_bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
        material=bearing_black,
        name="center_bearing",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.43, length=0.11),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.35, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin_brass,
        name="rotating_plate",
    )
    cradle.visual(
        Cylinder(radius=0.028, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=bearing_black,
        name="turntable_spigot",
    )
    for x_pos, suffix in [(-0.32, "0"), (0.32, "1")]:
        cradle.visual(
            Cylinder(radius=0.019, length=0.792),
            origin=Origin(xyz=(x_pos, 0.0, 0.446)),
            material=satin_brass,
            name=f"post_{suffix}",
        )
        cradle.visual(
            Box((0.075, 0.060, 0.040)),
            origin=Origin(xyz=(x_pos, 0.0, 0.070)),
            material=aged_brass,
            name=f"post_foot_{suffix}",
        )
        cradle.visual(
            Cylinder(radius=0.027, length=0.035),
            origin=Origin(xyz=(x_pos, 0.0, 0.842), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aged_brass,
            name=f"top_knuckle_{suffix}",
        )
    cradle.visual(
        Cylinder(radius=0.020, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.842), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_brass,
        name="top_crossbar",
    )
    cradle.visual(
        Cylinder(radius=0.020, length=0.197),
        origin=Origin(xyz=(0.0, 0.0, 0.1485)),
        material=satin_brass,
        name="bottom_pivot",
    )
    cradle.visual(
        Cylinder(radius=0.015, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.819)),
        material=satin_brass,
        name="top_pivot",
    )
    cradle.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=aged_brass,
        name="lower_pivot_boss",
    )
    cradle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.86),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        material=night_blue,
        name="globe_sphere",
    )
    globe.visual(equator_mesh, material=satin_brass, name="equator_band")
    globe.visual(
        ecliptic_mesh,
        origin=Origin(rpy=(math.radians(23.5), 0.0, 0.0)),
        material=aged_brass,
        name="ecliptic_band",
    )
    for index, yaw in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
        globe.visual(
            meridian_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, yaw)),
            material=satin_brass,
            name=f"meridian_{index}",
        )
    for lat_name, z_sign in (("latitude_30", -1.0), ("latitude_30", 1.0), ("latitude_60", -1.0), ("latitude_60", 1.0)):
        lat_deg = 30.0 if lat_name == "latitude_30" else 60.0
        globe.visual(
            latitude_meshes[lat_name],
            origin=Origin(xyz=(0.0, 0.0, z_sign * GLOBE_RADIUS * math.sin(math.radians(lat_deg)))),
            material=aged_brass,
            name=f"{lat_name}_{'north' if z_sign > 0.0 else 'south'}",
        )
    globe.visual(
        Cylinder(radius=0.033, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.267)),
        material=satin_brass,
        name="top_socket",
    )
    globe.visual(
        Cylinder(radius=0.033, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.267)),
        material=satin_brass,
        name="bottom_socket",
    )
    star_positions = [
        (-120, 25), (-92, -18), (-68, 48), (-40, 4), (-16, -36), (12, 36),
        (38, -8), (63, 55), (88, -30), (115, 12), (142, -46), (168, 34),
        (-154, -58), (-132, 62), (28, 8), (74, 22), (104, -5), (-8, 68),
    ]
    for index, (lon_deg, lat_deg) in enumerate(star_positions):
        lon = math.radians(lon_deg)
        lat = math.radians(lat_deg)
        radius = GLOBE_RADIUS + 0.002
        globe.visual(
            Sphere(radius=0.006 if index % 4 else 0.008),
            origin=Origin(
                xyz=(
                    radius * math.cos(lat) * math.cos(lon),
                    radius * math.cos(lat) * math.sin(lon),
                    radius * math.sin(lat),
                )
            ),
            material=star_ink,
            name=f"star_{index}",
        )
    globe.inertial = Inertial.from_geometry(Sphere(radius=GLOBE_RADIUS), mass=0.7)

    model.articulation(
        "cradle_turn",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=cradle,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cradle = object_model.get_part("cradle")
    globe = object_model.get_part("globe")
    cradle_turn = object_model.get_articulation("cradle_turn")
    globe_spin = object_model.get_articulation("globe_spin")

    ctx.allow_overlap(
        cradle,
        globe,
        elem_a="top_pivot",
        elem_b="top_socket",
        reason="The top polar pin is intentionally seated inside the globe socket so the globe is captured while spinning.",
    )
    ctx.allow_overlap(
        cradle,
        globe,
        elem_a="bottom_pivot",
        elem_b="bottom_socket",
        reason="The lower polar pin is intentionally seated inside the globe socket so the globe is captured while spinning.",
    )
    ctx.allow_overlap(
        base,
        cradle,
        elem_a="center_bearing",
        elem_b="turntable_spigot",
        reason="The rotating turntable spigot is intentionally represented as seated inside the fixed bearing socket.",
    )

    ctx.expect_contact(
        base,
        cradle,
        elem_a="fixed_bearing_ring",
        elem_b="rotating_plate",
        contact_tol=0.006,
        name="turntable plate sits on fixed bearing ring",
    )
    ctx.expect_within(
        cradle,
        base,
        axes="xy",
        inner_elem="turntable_spigot",
        outer_elem="center_bearing",
        margin=0.001,
        name="turntable spigot is centered in bearing",
    )
    ctx.expect_overlap(
        base,
        cradle,
        axes="z",
        elem_a="center_bearing",
        elem_b="turntable_spigot",
        min_overlap=0.020,
        name="turntable spigot remains inserted",
    )
    ctx.expect_within(
        globe,
        cradle,
        axes="xy",
        inner_elem="globe_sphere",
        outer_elem="top_crossbar",
        margin=0.36,
        name="globe centered under cradle crossbar",
    )
    for pivot_elem, socket_elem in (("top_pivot", "top_socket"), ("bottom_pivot", "bottom_socket")):
        ctx.expect_within(
            cradle,
            globe,
            axes="xy",
            inner_elem=pivot_elem,
            outer_elem=socket_elem,
            margin=0.001,
            name=f"{pivot_elem} stays coaxial in socket",
        )
        ctx.expect_overlap(
            cradle,
            globe,
            axes="z",
            elem_a=pivot_elem,
            elem_b=socket_elem,
            min_overlap=0.010,
            name=f"{pivot_elem} retains socket insertion",
        )

    with ctx.pose({cradle_turn: 1.25, globe_spin: 2.1}):
        for pivot_elem, socket_elem in (("top_pivot", "top_socket"), ("bottom_pivot", "bottom_socket")):
            ctx.expect_within(
                cradle,
                globe,
                axes="xy",
                inner_elem=pivot_elem,
                outer_elem=socket_elem,
                margin=0.001,
                name=f"{pivot_elem} remains captured after rotation",
            )
            ctx.expect_overlap(
                cradle,
                globe,
                axes="z",
                elem_a=pivot_elem,
                elem_b=socket_elem,
                min_overlap=0.010,
                name=f"{pivot_elem} remains inserted after rotation",
            )

    return ctx.report()


object_model = build_object_model()
