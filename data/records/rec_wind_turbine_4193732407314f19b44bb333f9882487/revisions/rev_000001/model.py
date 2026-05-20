from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WHITE = Material("gelcoat_white", rgba=(0.86, 0.88, 0.86, 1.0))
WARM_WHITE = Material("warm_composite_white", rgba=(0.92, 0.91, 0.86, 1.0))
LIGHT_GRAY = Material("light_galvanized_gray", rgba=(0.64, 0.66, 0.66, 1.0))
CONCRETE = Material("weathered_concrete", rgba=(0.45, 0.45, 0.42, 1.0))
DARK = Material("dark_service_gray", rgba=(0.08, 0.09, 0.10, 1.0))
RED = Material("aviation_red", rgba=(0.75, 0.07, 0.04, 1.0))


def _tower_shell() -> MeshGeometry:
    """Utility-scale tubular tower: broad at grade, slim at the yaw bearing."""
    height = 79.0
    base_radius = 2.45
    top_radius = 1.42
    # Solid capped frustum; the visible utility tower is a painted shell, but
    # the closed caps hide the wall thickness at this scale.
    profile = [
        (0.0, 0.0),
        (base_radius, 0.0),
        (top_radius, height),
        (0.0, height),
    ]
    return LatheGeometry(profile, segments=72, closed=True)


def _bearing_ring(inner_radius: float, outer_radius: float, length: float) -> MeshGeometry:
    """Short hollow ring centered on local Z, used for the main rotor bearing."""
    profile = [
        (inner_radius, -0.5 * length),
        (outer_radius, -0.5 * length),
        (outer_radius, 0.5 * length),
        (inner_radius, 0.5 * length),
    ]
    return LatheGeometry(profile, segments=64, closed=True)


def _blade_mesh() -> MeshGeometry:
    """Three long tapered, twisted wind-turbine blades in the rotor XZ plane."""
    geom = MeshGeometry()

    station_count = 15
    upper_us = [-0.50, -0.34, -0.18, 0.00, 0.22, 0.40, 0.50]
    lower_us = [0.40, 0.22, 0.00, -0.18, -0.34]
    profile_us = upper_us + lower_us

    axial = (0.0, -1.0, 0.0)
    root_r = 2.0
    tip_r = 38.0

    def add_vec(a, b):
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

    def scale_vec(v, s):
        return (v[0] * s, v[1] * s, v[2] * s)

    for blade_index, theta in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        radial = (math.cos(theta), 0.0, math.sin(theta))
        tangent = (-math.sin(theta), 0.0, math.cos(theta))
        sections: list[list[int]] = []

        for i in range(station_count):
            s = i / (station_count - 1)
            radius = root_r + (tip_r - root_r) * s
            chord = 4.55 * (1.0 - s) ** 1.35 + 0.72 * s
            thickness = 0.78 * (1.0 - s) ** 1.15 + 0.13 * s
            pitch = math.radians(16.0 * (1.0 - s) + 3.2 * s)
            sweep = 1.25 * s * s
            coning = 0.55 * s * s

            center = add_vec(
                add_vec(scale_vec(radial, radius), scale_vec(tangent, sweep)),
                scale_vec(axial, coning),
            )

            chord_dir = add_vec(scale_vec(tangent, math.cos(pitch)), scale_vec(axial, math.sin(pitch)))
            thick_dir = add_vec(scale_vec(tangent, -math.sin(pitch)), scale_vec(axial, math.cos(pitch)))

            loop: list[int] = []
            for j, u in enumerate(profile_us):
                edge_falloff = max(0.08, 1.0 - (2.0 * u) ** 2)
                local_t = thickness * math.sqrt(edge_falloff)
                camber = 0.10 * thickness * (1.0 - (2.0 * u) ** 2)
                sign = 1.0 if j < len(upper_us) else -1.0
                normal_offset = camber + sign * 0.50 * local_t

                p = add_vec(
                    add_vec(center, scale_vec(chord_dir, u * chord)),
                    scale_vec(thick_dir, normal_offset),
                )
                loop.append(geom.add_vertex(*p))
            sections.append(loop)

        # Skin between stations.
        for a, b in zip(sections[:-1], sections[1:]):
            n = len(a)
            for k in range(n):
                a0 = a[k]
                a1 = a[(k + 1) % n]
                b0 = b[k]
                b1 = b[(k + 1) % n]
                geom.add_face(a0, b0, b1)
                geom.add_face(a0, b1, a1)

        # Root and tip caps.
        for loop, reverse in ((sections[0], True), (sections[-1], False)):
            center_pt = (0.0, 0.0, 0.0)
            for idx in loop:
                vx, vy, vz = geom.vertices[idx]
                center_pt = (center_pt[0] + vx, center_pt[1] + vy, center_pt[2] + vz)
            center_pt = (center_pt[0] / len(loop), center_pt[1] / len(loop), center_pt[2] / len(loop))
            c = geom.add_vertex(*center_pt)
            for k in range(len(loop)):
                if reverse:
                    geom.add_face(c, loop[(k + 1) % len(loop)], loop[k])
                else:
                    geom.add_face(c, loop[k], loop[(k + 1) % len(loop)])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_scale_wind_turbine")

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=6.2, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        material=CONCRETE,
        name="foundation",
    )
    tower.visual(
        mesh_from_geometry(_tower_shell(), "tapered_tower"),
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        material=WHITE,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=2.75, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
        material=LIGHT_GRAY,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=1.65, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 80.0)),
        material=LIGHT_GRAY,
        name="top_flange",
    )
    for z, radius in ((12.5, 2.30), (31.5, 2.05), (50.5, 1.80), (68.5, 1.57)):
        tower.visual(
            mesh_from_geometry(
                TorusGeometry(radius=radius, tube=0.055, radial_segments=18, tubular_segments=72),
                f"section_ring_{int(z)}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=LIGHT_GRAY,
            name=f"section_ring_{int(z)}",
        )
    tower.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.95, tube=0.075, radial_segments=18, tubular_segments=72),
            "warning_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, 42.0)),
        material=RED,
        name="warning_band",
    )
    tower.visual(
        Box((1.05, 0.10, 2.15)),
        origin=Origin(xyz=(0.0, -2.50, 2.25)),
        material=DARK,
        name="access_door",
    )
    tower.visual(
        Box((0.68, 0.08, 0.28)),
        origin=Origin(xyz=(0.0, -2.58, 3.12)),
        material=LIGHT_GRAY,
        name="door_lintel",
    )
    for i in range(16):
        a = 2.0 * math.pi * i / 16.0
        tower.visual(
            Cylinder(radius=0.12, length=0.24),
            origin=Origin(xyz=(4.95 * math.cos(a), 4.95 * math.sin(a), 1.12)),
            material=LIGHT_GRAY,
            name=f"anchor_bolt_{i}",
        )

    nacelle = model.part("nacelle")
    nacelle_body = (
        cq.Workplane("XY")
        .box(4.6, 11.3, 4.0)
        .edges()
        .fillet(0.34)
        .translate((0.0, 1.25, 2.78))
    )
    nacelle.visual(
        mesh_from_cadquery(nacelle_body, "nacelle_shell", tolerance=0.035, angular_tolerance=0.18),
        material=WHITE,
        name="nacelle_shell",
    )
    nacelle.visual(
        Cylinder(radius=2.05, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=LIGHT_GRAY,
        name="yaw_ring",
    )
    nacelle.visual(
        Box((3.85, 3.10, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        material=LIGHT_GRAY,
        name="bedplate_skirt",
    )
    nacelle.visual(
        mesh_from_geometry(_bearing_ring(1.08, 1.55, 0.75), "nose_fairing"),
        origin=Origin(xyz=(0.0, -4.775, 2.80), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="nose_fairing",
    )
    nacelle.visual(
        mesh_from_geometry(_bearing_ring(1.12, 1.45, 0.35), "front_bearing"),
        origin=Origin(xyz=(0.0, -5.10, 2.80), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK,
        name="front_bearing",
    )
    for side_x in (-2.34, 2.34):
        for j in range(6):
            nacelle.visual(
                Box((0.09, 0.62, 0.14)),
                origin=Origin(xyz=(side_x, 0.2 + j * 0.72, 3.72)),
                material=DARK,
                name=f"side_vent_{'a' if side_x < 0 else 'b'}_{j}",
            )
    for j in range(5):
        nacelle.visual(
            Box((3.25, 0.08, 0.13)),
            origin=Origin(xyz=(0.0, 6.93, 2.15 + j * 0.34)),
            material=DARK,
            name=f"rear_louver_{j}",
        )
    nacelle.visual(
        Cylinder(radius=0.055, length=1.25),
        origin=Origin(xyz=(0.0, 4.10, 5.39)),
        material=DARK,
        name="weather_mast",
    )
    nacelle.visual(
        Box((0.10, 1.30, 0.08)),
        origin=Origin(xyz=(0.0, 4.10, 6.05)),
        material=DARK,
        name="wind_vane_boom",
    )
    nacelle.visual(
        Box((0.70, 0.09, 0.36)),
        origin=Origin(xyz=(0.0, 4.74, 6.05)),
        material=DARK,
        name="wind_vane_tail",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_blade_mesh(), "rotor_blades"),
        material=WARM_WHITE,
        name="blade_set",
    )
    rotor.visual(
        Cylinder(radius=2.10, length=2.45),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE,
        name="hub_shell",
    )
    rotor.visual(
        Sphere(radius=1.80),
        origin=Origin(xyz=(0.0, -1.65, 0.0)),
        material=WHITE,
        name="spinner_cap",
    )
    rotor.visual(
        Cylinder(radius=1.00, length=0.35),
        origin=Origin(xyz=(0.0, 1.40, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=LIGHT_GRAY,
        name="rear_collar",
    )
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        rotor.visual(
            Cylinder(radius=0.12, length=0.10),
            origin=Origin(
                xyz=(1.30 * math.cos(a), -1.95, 1.30 * math.sin(a)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=DARK,
            name=f"hub_bolt_{i}",
        )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 80.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=750000.0, velocity=0.08, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=9000.0, friction=1200.0),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(0.0, -6.675, 2.80)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=950000.0, velocity=1.6),
        motion_properties=MotionProperties(damping=350.0, friction=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("yaw")
    spin = object_model.get_articulation("rotor_spin")

    ctx.check(
        "rotor spins about horizontal shaft",
        tuple(round(v, 3) for v in spin.axis) == (0.0, -1.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.check(
        "nacelle has yaw articulation",
        tuple(round(v, 3) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.expect_contact(
        nacelle,
        tower,
        elem_a="yaw_ring",
        elem_b="top_flange",
        contact_tol=0.004,
        name="yaw bearing seated on tower",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="xz",
        inner_elem="rear_collar",
        outer_elem="front_bearing",
        margin=0.01,
        name="rotor shaft centered in bearing",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="y",
        elem_a="rear_collar",
        elem_b="front_bearing",
        min_overlap=0.15,
        name="rotor shaft remains inserted",
    )
    rotor_aabb = ctx.part_world_aabb(rotor)
    ctx.check(
        "blade tips clear ground",
        rotor_aabb is not None and rotor_aabb[0][2] > 40.0,
        details=f"rotor_aabb={rotor_aabb}",
    )
    with ctx.pose({spin: 0.9, yaw: 0.35}):
        ctx.expect_within(
            rotor,
            nacelle,
            axes="xz",
            inner_elem="rear_collar",
            outer_elem="front_bearing",
            margin=0.02,
            name="spinning rotor remains centered",
        )

    return ctx.report()


object_model = build_object_model()
