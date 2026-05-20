from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _sgn_pow(value: float, exponent: float) -> float:
    return math.copysign(abs(value) ** exponent, value)


def _rounded_box_loft_x(
    sections: list[tuple[float, float, float, float]],
    *,
    segments: int = 36,
    exponent: float = 0.58,
) -> MeshGeometry:
    """Loft rounded-rectangle sections in planes normal to local X."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, width_y, height_z, center_z in sections:
        ring: list[int] = []
        for i in range(segments):
            t = 2.0 * math.pi * i / segments
            y = 0.5 * width_y * _sgn_pow(math.cos(t), exponent)
            z = center_z + 0.5 * height_z * _sgn_pow(math.sin(t), exponent)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    for ring in (rings[0], rings[-1]):
        pts = [geom.vertices[i] for i in ring]
        cx = sum(p[0] for p in pts) / segments
        cy = sum(p[1] for p in pts) / segments
        cz = sum(p[2] for p in pts) / segments
        c = geom.add_vertex(cx, cy, cz)
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(c, ring[j], ring[i])
    return geom


def _frustum_z(
    radius_bottom: float,
    radius_top: float,
    z_bottom: float,
    z_top: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    geom = MeshGeometry()
    lower: list[int] = []
    upper: list[int] = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        lower.append(geom.add_vertex(radius_bottom * c, radius_bottom * s, z_bottom))
        upper.append(geom.add_vertex(radius_top * c, radius_top * s, z_top))
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(lower[i], upper[i], upper[j])
        geom.add_face(lower[i], upper[j], lower[j])
    c0 = geom.add_vertex(0.0, 0.0, z_bottom)
    c1 = geom.add_vertex(0.0, 0.0, z_top)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(c0, lower[j], lower[i])
        geom.add_face(c1, upper[i], upper[j])
    return geom


def _circular_loft_x(
    sections: list[tuple[float, float]],
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Loft circular cross sections along local X as (x, radius)."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, radius in sections:
        ring: list[int] = []
        for i in range(segments):
            t = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(x, radius * math.cos(t), radius * math.sin(t)))
        rings.append(ring)
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])
    for ring in (rings[0], rings[-1]):
        x = geom.vertices[ring[0]][0]
        c = geom.add_vertex(x, 0.0, 0.0)
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(c, ring[i], ring[j])
    return geom


def _blade_mesh() -> MeshGeometry:
    """A thick utility-turbine blade extending from its pitch bearing along +Y."""
    geom = MeshGeometry()
    radial_stations = [0.42, 0.72, 1.20, 1.85, 2.60, 3.35, 4.15, 4.70]
    segments = 28
    rings: list[list[int]] = []
    max_y = radial_stations[-1]
    for y in radial_stations:
        s = (y - radial_stations[0]) / (max_y - radial_stations[0])
        chord = 0.82 * (1.0 - s) ** 1.15 + 0.16 * s
        thickness = 0.21 * (1.0 - s) ** 1.1 + 0.040 * s
        twist = math.radians(16.0 * (1.0 - s) - 5.0 * s)
        sweep_x = 0.10 * s * s
        ring: list[int] = []
        for i in range(segments):
            t = 2.0 * math.pi * i / segments
            # Broad rounded leading side and a slightly flatter, durable trailing side.
            z0 = 0.5 * chord * math.cos(t)
            x0 = 0.5 * thickness * math.sin(t) * (1.0 - 0.18 * math.cos(t))
            x0 += 0.018 * math.sin(math.pi * s)  # gentle camber
            x = sweep_x + x0 * math.cos(twist) - z0 * math.sin(twist)
            z = x0 * math.sin(twist) + z0 * math.cos(twist)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])
    for ring in (rings[0], rings[-1]):
        pts = [geom.vertices[i] for i in ring]
        cx = sum(p[0] for p in pts) / segments
        cy = sum(p[1] for p in pts) / segments
        cz = sum(p[2] for p in pts) / segments
        c = geom.add_vertex(cx, cy, cz)
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(c, ring[j], ring[i])
    return geom


def _bearing_disk_origin(angle: float, radius: float, offset_x: float = 0.0) -> tuple[float, float, float]:
    return (offset_x, radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wind_turbine")

    galvanized = model.material("galvanized_painted_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_steel = model.material("dark_blasted_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow_service_marks", rgba=(0.95, 0.68, 0.12, 1.0))
    off_white = model.material("molded_off_white_composite", rgba=(0.88, 0.90, 0.86, 1.0))
    utility_grey = model.material("rugged_utility_grey_paint", rgba=(0.42, 0.46, 0.47, 1.0))
    rubber_black = model.material("black_rubber_seals", rgba=(0.025, 0.025, 0.023, 1.0))
    bolt_finish = model.material("dark_zinc_fasteners", rgba=(0.055, 0.055, 0.050, 1.0))

    # Root structural tower and service foundation.
    tower = model.part("tower")
    tower.visual(
        Box((2.80, 2.80, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=galvanized,
        name="foundation_slab",
    )
    tower.visual(
        Cylinder(radius=0.62, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=dark_steel,
        name="base_flange",
    )
    tower.visual(
        mesh_from_geometry(_frustum_z(0.43, 0.19, 0.28, 11.42, segments=64), "tapered_tower_shell"),
        material=galvanized,
        name="tapered_tower_shell",
    )
    tower.visual(
        Cylinder(radius=0.33, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 11.52)),
        material=dark_steel,
        name="yaw_support_flange",
    )
    tower.visual(
        Cylinder(radius=0.49, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 11.56)),
        material=bolt_finish,
        name="yaw_bolt_ring",
    )
    tower.visual(
        Box((0.040, 0.38, 0.86)),
        origin=Origin(xyz=(-0.402, 0.0, 1.08)),
        material=utility_grey,
        name="tower_access_hatch",
    )
    tower.visual(
        Box((0.018, 0.42, 0.036)),
        origin=Origin(xyz=(-0.431, 0.0, 1.43)),
        material=safety_yellow,
        name="hatch_handle_bar",
    )
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        tower.visual(
            Cylinder(radius=0.035, length=0.12),
            origin=Origin(xyz=(0.74 * math.cos(a), 0.74 * math.sin(a), 0.18)),
            material=bolt_finish,
            name=f"foundation_anchor_{i}",
        )

    # Yawing nacelle with rugged bolted service covers and front bearing support.
    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.47, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="yaw_bearing_skirt",
    )
    nacelle.visual(
        mesh_from_geometry(
            _rounded_box_loft_x(
                [
                    (-1.52, 0.76, 0.58, 0.52),
                    (-0.70, 1.02, 0.82, 0.55),
                    (0.58, 0.96, 0.76, 0.55),
                    (1.16, 0.62, 0.54, 0.55),
                ],
                segments=44,
            ),
            "nacelle_rounded_housing",
        ),
        material=utility_grey,
        name="nacelle_rounded_housing",
    )
    nacelle.visual(
        Cylinder(radius=0.37, length=0.32),
        origin=Origin(xyz=(1.20, 0.0, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bearing_collar",
    )
    nacelle.visual(
        Cylinder(radius=0.49, length=0.055),
        origin=Origin(xyz=(1.005, 0.0, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_finish,
        name="front_bolt_flange",
    )
    nacelle.visual(
        Box((0.58, 0.060, 0.36)),
        origin=Origin(xyz=(-0.48, 0.505, 0.56)),
        material=galvanized,
        name="service_panel_0",
    )
    nacelle.visual(
        Box((0.58, 0.060, 0.36)),
        origin=Origin(xyz=(-0.48, -0.505, 0.56)),
        material=galvanized,
        name="service_panel_1",
    )
    nacelle.visual(
        Box((0.72, 0.050, 0.055)),
        origin=Origin(xyz=(-1.03, 0.0, 0.91)),
        material=rubber_black,
        name="roof_lifting_rail",
    )
    for side in (-1.0, 1.0):
        for ix, x in enumerate((-0.72, -0.24)):
            for iz, z in enumerate((0.42, 0.70)):
                nacelle.visual(
                    Cylinder(radius=0.026, length=0.032),
                    origin=Origin(
                        xyz=(x, side * 0.538, z),
                        rpy=(-side * math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=bolt_finish,
                    name=f"service_panel_bolt_{int(side > 0)}_{ix}_{iz}",
                )

    yaw = model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 11.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.16, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=140.0, friction=70.0),
    )
    yaw.meta["description"] = "Slow serviceable yaw bearing on top of the utility tower."

    # Spinning rotor hub: axis points along local +X, carried by the nacelle collar.
    hub = model.part("rotor_hub")
    hub.visual(
        Cylinder(radius=0.38, length=0.64),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    hub.visual(
        Cylinder(radius=0.47, length=0.16),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_finish,
        name="rotor_bearing_ring",
    )
    hub.visual(
        Sphere(radius=0.34),
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        material=dark_steel,
        name="spherical_hub_casting",
    )
    hub.visual(
        mesh_from_geometry(
            _circular_loft_x([(0.50, 0.36), (0.70, 0.24), (0.88, 0.05)], segments=48),
            "rounded_spinner_nose",
        ),
        material=utility_grey,
        name="rounded_spinner_nose",
    )

    blade_angles = [math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0]
    pitch_face_names = ["pitch_bearing_face_0", "pitch_bearing_face_1", "pitch_bearing_face_2"]
    for idx, angle in enumerate(blade_angles):
        radial_rpy = (angle - math.pi / 2.0, 0.0, 0.0)
        y, z = 0.46 * math.cos(angle), 0.46 * math.sin(angle)
        hub.visual(
            Cylinder(radius=0.255, length=0.54),
            origin=Origin(xyz=(0.34, y, z), rpy=radial_rpy),
            material=dark_steel,
            name=f"pitch_socket_{idx}",
        )
        hub.visual(
            Cylinder(radius=0.31, length=0.055),
            origin=Origin(xyz=_bearing_disk_origin(angle, 0.735, offset_x=0.34), rpy=radial_rpy),
            material=bolt_finish,
            name=pitch_face_names[idx],
        )
        tangent = (0.0, -math.sin(angle), math.cos(angle))
        radial = (0.0, math.cos(angle), math.sin(angle))
        for b in range(8):
            t = 2.0 * math.pi * b / 8.0
            bolt_circle = 0.255
            x = 0.34 + bolt_circle * math.cos(t)
            yb = 0.735 * radial[1] + bolt_circle * math.sin(t) * tangent[1]
            zb = 0.735 * radial[2] + bolt_circle * math.sin(t) * tangent[2]
            hub.visual(
                Cylinder(radius=0.025, length=0.045),
                origin=Origin(xyz=(x, yb, zb), rpy=radial_rpy),
                material=bolt_finish,
                name=f"pitch_bolt_{idx}_{b}",
            )

    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(1.36, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=1.8),
        motion_properties=MotionProperties(damping=35.0, friction=8.0),
    )

    blade_mesh = mesh_from_geometry(_blade_mesh(), "thick_twisted_blade")
    for idx, angle in enumerate(blade_angles):
        blade = model.part(f"blade_{idx}")
        blade.visual(
            Cylinder(radius=0.185, length=0.62),
            origin=Origin(xyz=(0.0, 0.31, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="root_cuff",
        )
        blade.visual(
            Cylinder(radius=0.210, length=0.08),
            origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_finish,
            name="pitch_bearing_ring",
        )
        blade.visual(
            blade_mesh,
            material=off_white,
            name="thick_twisted_blade",
        )
        blade.visual(
            Box((0.026, 2.40, 0.034)),
            origin=Origin(xyz=(0.05, 2.50, 0.16)),
            material=rubber_black,
            name="leading_edge_guard",
        )
        blade.visual(
            Cylinder(radius=0.192, length=0.055),
            origin=Origin(xyz=(0.0, 0.45, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name="root_index_band",
        )
        model.articulation(
            f"hub_to_blade_{idx}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=blade,
            origin=Origin(xyz=(0.34, 0.7625 * math.cos(angle), 0.7625 * math.sin(angle)), rpy=(angle, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1800.0, velocity=0.45, lower=math.radians(-8.0), upper=math.radians(28.0)),
            motion_properties=MotionProperties(damping=22.0, friction=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("rotor_hub")
    rotor = object_model.get_articulation("nacelle_to_rotor")

    ctx.expect_contact(
        tower,
        nacelle,
        elem_a="yaw_support_flange",
        elem_b="yaw_bearing_skirt",
        contact_tol=0.003,
        name="yaw bearing stack is seated on tower",
    )
    ctx.expect_contact(
        nacelle,
        hub,
        elem_a="front_bearing_collar",
        elem_b="rotor_bearing_ring",
        contact_tol=0.003,
        name="rotor hub is carried by front bearing collar",
    )

    pitch_face_names = ["pitch_bearing_face_0", "pitch_bearing_face_1", "pitch_bearing_face_2"]
    blade_names = ["blade_0", "blade_1", "blade_2"]
    for blade_name, face_name in zip(blade_names, pitch_face_names):
        blade = object_model.get_part(blade_name)
        ctx.allow_overlap(
            blade,
            hub,
            elem_a="pitch_bearing_ring",
            elem_b=face_name,
            reason="The bolted pitch bearing ring is intentionally seated into the hub-side bearing face proxy.",
        )
        ctx.allow_overlap(
            blade,
            hub,
            elem_a="root_cuff",
            elem_b=face_name,
            reason="The thick blade root cuff intentionally passes through the pitch-bearing face into the reinforced socket.",
        )
        ctx.expect_overlap(
            blade,
            hub,
            axes="x",
            elem_a="pitch_bearing_ring",
            elem_b=face_name,
            min_overlap=0.35,
            name=f"{blade_name} pitch bearing face is centered on the hub flange",
        )
        ctx.expect_overlap(
            blade,
            hub,
            axes="x",
            elem_a="root_cuff",
            elem_b=face_name,
            min_overlap=0.30,
            name=f"{blade_name} root cuff is captured by the pitch flange",
        )

    blade0 = object_model.get_part("blade_0")
    ctx.expect_gap(
        blade0,
        hub,
        axis="z",
        positive_elem="pitch_bearing_ring",
        negative_elem="pitch_bearing_face_0",
        max_gap=0.002,
        max_penetration=0.002,
        name="blade root pitch bearing is seated on the hub face",
    )

    p0 = ctx.part_world_position(blade0)
    with ctx.pose({rotor: math.pi / 3.0}):
        p1 = ctx.part_world_position(blade0)
    ctx.check(
        "rotor spin moves blade around supported hub axis",
        p0 is not None and p1 is not None and abs(p1[1] - p0[1]) + abs(p1[2] - p0[2]) > 0.20,
        details=f"closed={p0}, spun={p1}",
    )

    return ctx.report()


object_model = build_object_model()
