from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _body_of_revolution_along_x(stations, *, segments: int = 56, z_scale: float = 0.93) -> MeshGeometry:
    """Closed streamlined body with circular-ish stations along the airship X axis."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, radius in stations:
        ring: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(x, radius * math.cos(a), z_scale * radius * math.sin(a)))
        rings.append(ring)

    for ring_a, ring_b in zip(rings, rings[1:]):
        for i in range(segments):
            a0 = ring_a[i]
            a1 = ring_a[(i + 1) % segments]
            b0 = ring_b[i]
            b1 = ring_b[(i + 1) % segments]
            geom.add_face(a0, b0, b1)
            geom.add_face(a0, b1, a1)

    # Cap the small end rings so exact geometry remains a closed solid.
    for ring, reverse in ((rings[0], True), (rings[-1], False)):
        center_x = stations[0][0] if reverse else stations[-1][0]
        c = geom.add_vertex(center_x, 0.0, 0.0)
        for i in range(segments):
            a = ring[i]
            b = ring[(i + 1) % segments]
            if reverse:
                geom.add_face(c, b, a)
            else:
                geom.add_face(c, a, b)
    return geom


def _superellipse_loop_x(
    x: float,
    *,
    center_z: float,
    width: float,
    height: float,
    samples: int = 36,
    exponent: float = 3.2,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    half_w = width * 0.5
    half_h = height * 0.5
    p = 2.0 / exponent
    for i in range(samples):
        t = 2.0 * math.pi * i / samples
        ct = math.cos(t)
        st = math.sin(t)
        y = half_w * (1.0 if ct >= 0.0 else -1.0) * (abs(ct) ** p)
        z = center_z + half_h * (1.0 if st >= 0.0 else -1.0) * (abs(st) ** p)
        points.append((x, y, z))
    return points


def _loft_between_loops(loops: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    geom = MeshGeometry()
    indices: list[list[int]] = []
    count = len(loops[0])
    for loop in loops:
        indices.append([geom.add_vertex(*p) for p in loop])

    for loop_a, loop_b in zip(indices, indices[1:]):
        for i in range(count):
            a0 = loop_a[i]
            a1 = loop_a[(i + 1) % count]
            b0 = loop_b[i]
            b1 = loop_b[(i + 1) % count]
            geom.add_face(a0, b0, b1)
            geom.add_face(a0, b1, a1)

    # End caps.
    for loop, reverse in ((indices[0], True), (indices[-1], False)):
        center = [0.0, 0.0, 0.0]
        for idx in loop:
            vx, vy, vz = geom.vertices[idx]
            center[0] += vx
            center[1] += vy
            center[2] += vz
        n = float(len(loop))
        c = geom.add_vertex(center[0] / n, center[1] / n, center[2] / n)
        for i in range(count):
            a = loop[i]
            b = loop[(i + 1) % count]
            if reverse:
                geom.add_face(c, b, a)
            else:
                geom.add_face(c, a, b)
    return geom


def _passenger_cabin_mesh() -> MeshGeometry:
    center_x = 1.5
    center_z = -8.65
    length = 18.0
    width = 3.35
    height = 2.20
    # Tiny end sections avoid a flat boxy cap while keeping the mesh closed.
    section_scales = (
        (-0.500, 0.08),
        (-0.455, 0.42),
        (-0.365, 0.78),
        (-0.250, 0.98),
        (0.250, 1.00),
        (0.365, 0.82),
        (0.455, 0.45),
        (0.500, 0.10),
    )
    loops = [
        _superellipse_loop_x(
            center_x + length * rel,
            center_z=center_z,
            width=width * scale,
            height=height * (0.72 + 0.28 * scale),
        )
        for rel, scale in section_scales
    ]
    return _loft_between_loops(loops)


def _vertical_prism(points_xz, *, thickness_y: float) -> MeshGeometry:
    geom = MeshGeometry()
    half = thickness_y * 0.5
    front = [geom.add_vertex(x, -half, z) for x, z in points_xz]
    back = [geom.add_vertex(x, half, z) for x, z in points_xz]
    n = len(points_xz)
    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _horizontal_prism(points_xy, *, thickness_z: float) -> MeshGeometry:
    geom = MeshGeometry()
    half = thickness_z * 0.5
    lower = [geom.add_vertex(x, y, -half) for x, y in points_xy]
    upper = [geom.add_vertex(x, y, half) for x, y in points_xy]
    n = len(points_xy)
    for i in range(1, n - 1):
        geom.add_face(lower[0], lower[i + 1], lower[i])
        geom.add_face(upper[0], upper[i], upper[i + 1])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(lower[i], lower[j], upper[j])
        geom.add_face(lower[i], upper[j], upper[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sightseeing_blimp")

    hull_fabric = model.material("warm_white_fabric", rgba=(0.88, 0.91, 0.88, 1.0))
    tail_blue = model.material("deep_tail_blue", rgba=(0.06, 0.18, 0.34, 1.0))
    cabin_white = model.material("cabin_white", rgba=(0.92, 0.93, 0.89, 1.0))
    glass = model.material("smoked_blue_glass", rgba=(0.05, 0.16, 0.23, 0.78))
    dark = model.material("matte_graphite", rgba=(0.015, 0.017, 0.018, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.58, 0.60, 0.58, 1.0))
    red = model.material("safety_red", rgba=(0.78, 0.07, 0.04, 1.0))

    hull = model.part("hull")

    envelope = _body_of_revolution_along_x(
        (
            (-27.5, 0.06),
            (-26.2, 0.75),
            (-24.0, 2.10),
            (-20.0, 4.35),
            (-14.0, 6.05),
            (-6.0, 7.10),
            (3.5, 7.55),
            (12.0, 7.10),
            (20.5, 5.05),
            (25.6, 2.05),
            (27.5, 0.08),
        )
    )
    hull.visual(mesh_from_geometry(envelope, "streamlined_envelope"), material=hull_fabric, name="envelope")

    # A long belly keel, passenger gondola, and four structural hangers make the
    # cabin visibly carried by the envelope rather than floating below it.
    hull.visual(
        Box((17.5, 0.82, 0.38)),
        origin=Origin(xyz=(1.5, 0.0, -7.15)),
        material=metal,
        name="belly_keel",
    )
    hull.visual(mesh_from_geometry(_passenger_cabin_mesh(), "passenger_cabin"), material=cabin_white, name="cabin")
    for i, x in enumerate((-5.8, -1.6, 2.6, 6.8)):
        hull.visual(
            Box((0.46, 0.28, 0.72)),
            origin=Origin(xyz=(x, 0.0, -7.62)),
            material=metal,
            name=f"cabin_hanger_{i}",
        )
    hull.visual(
        Box((13.4, 0.075, 0.44)),
        origin=Origin(xyz=(1.55, 1.70, -8.34)),
        material=glass,
        name="port_window_band",
    )
    hull.visual(
        Box((13.4, 0.075, 0.44)),
        origin=Origin(xyz=(1.55, -1.70, -8.34)),
        material=glass,
        name="starboard_window_band",
    )
    hull.visual(
        Box((12.0, 0.18, 0.11)),
        origin=Origin(xyz=(1.8, 0.0, -9.78)),
        material=metal,
        name="landing_skid",
    )

    # Side engine pylons are part of the fixed hull/gondola structure.  Their
    # outer tip faces are tangent to the rotating pod pivot lugs at q=0.
    hull.visual(
        Box((0.72, 2.90, 0.34)),
        origin=Origin(xyz=(1.0, 3.15, -8.62)),
        material=metal,
        name="port_pylon",
    )
    hull.visual(
        Box((0.86, 0.18, 0.86)),
        origin=Origin(xyz=(1.0, 4.51, -8.62)),
        material=metal,
        name="port_pylon_tip",
    )
    hull.visual(
        Box((1.10, 0.18, 0.64)),
        origin=Origin(xyz=(1.0, 1.73, -8.62)),
        material=metal,
        name="port_pylon_root",
    )
    hull.visual(
        Box((0.72, 2.90, 0.34)),
        origin=Origin(xyz=(1.0, -3.15, -8.62)),
        material=metal,
        name="starboard_pylon",
    )
    hull.visual(
        Box((0.86, 0.18, 0.86)),
        origin=Origin(xyz=(1.0, -4.51, -8.62)),
        material=metal,
        name="starboard_pylon_tip",
    )
    hull.visual(
        Box((1.10, 0.18, 0.64)),
        origin=Origin(xyz=(1.0, -1.73, -8.62)),
        material=metal,
        name="starboard_pylon_root",
    )

    # Cruciform tail: upper/lower vertical fins and port/starboard horizontal
    # tailplanes are fixed to the aft envelope.  Separate rudder/elevator parts
    # are mounted on the visible trailing hinge rods.
    top_fin = _vertical_prism(
        [(-22.2, 2.7), (-27.75, 0.55), (-23.8, 9.25), (-19.7, 3.25)],
        thickness_y=0.34,
    )
    bottom_fin = _vertical_prism(
        [(-22.2, -2.7), (-19.7, -3.25), (-23.8, -9.25), (-27.75, -0.55)],
        thickness_y=0.34,
    )
    hull.visual(mesh_from_geometry(top_fin, "top_fin"), material=tail_blue, name="top_fin")
    hull.visual(mesh_from_geometry(bottom_fin, "bottom_fin"), material=tail_blue, name="bottom_fin")

    port_tailplane = _horizontal_prism(
        [(-22.0, 3.55), (-25.40, 1.18), (-23.10, 8.85), (-19.7, 4.25)],
        thickness_z=0.30,
    )
    star_tailplane = _horizontal_prism(
        [(-22.0, -3.55), (-19.7, -4.25), (-23.10, -8.85), (-25.40, -1.18)],
        thickness_z=0.30,
    )
    hull.visual(mesh_from_geometry(port_tailplane, "port_tailplane"), material=tail_blue, name="port_tailplane")
    hull.visual(
        mesh_from_geometry(star_tailplane, "starboard_tailplane"),
        material=tail_blue,
        name="starboard_tailplane",
    )
    hull.visual(
        Cylinder(radius=0.060, length=7.20),
        origin=Origin(xyz=(-27.75, 0.0, 0.0)),
        material=metal,
        name="rudder_hinge",
    )
    hull.visual(
        Box((0.62, 0.26, 1.25)),
        origin=Origin(xyz=(-27.48, 0.0, 0.0)),
        material=metal,
        name="tail_post_fairing",
    )
    hull.visual(
        Box((1.30, 2.45, 0.32)),
        origin=Origin(xyz=(-24.72, 2.36, 0.0)),
        material=tail_blue,
        name="port_tailplane_root",
    )
    hull.visual(
        Box((1.30, 2.45, 0.32)),
        origin=Origin(xyz=(-24.72, -2.36, 0.0)),
        material=tail_blue,
        name="starboard_tailplane_root",
    )
    hull.visual(
        Cylinder(radius=0.055, length=5.50),
        origin=Origin(xyz=(-25.40, 3.98, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="port_elevator_hinge",
    )
    hull.visual(
        Cylinder(radius=0.055, length=5.50),
        origin=Origin(xyz=(-25.40, -3.98, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="starboard_elevator_hinge",
    )

    def add_engine(side_name: str, sign: float) -> None:
        pivot_y = sign * 4.60
        pod = model.part(f"{side_name}_pod")
        pod.visual(
            Cylinder(radius=0.44, length=1.78),
            origin=Origin(xyz=(0.02, sign * 0.64, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="nacelle",
        )
        pod.visual(
            Box((0.70, 0.20, 0.78)),
            origin=Origin(xyz=(0.02, sign * 0.10, 0.0)),
            material=metal,
            name="pivot_lug",
        )
        pod.visual(
            mesh_from_geometry(TorusGeometry(0.44, 0.045, radial_segments=18, tubular_segments=40), f"{side_name}_cowl_ring"),
            origin=Origin(xyz=(0.952, sign * 0.64, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="cowl_ring",
        )
        pod.visual(
            Cylinder(radius=0.28, length=0.30),
            origin=Origin(xyz=(-0.95, sign * 0.64, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="aft_exhaust",
        )
        model.articulation(
            f"{side_name}_vector",
            ArticulationType.REVOLUTE,
            parent=hull,
            child=pod,
            origin=Origin(xyz=(1.0, pivot_y, -8.62)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=220.0, velocity=0.9, lower=-0.55, upper=0.55),
        )

        propeller = model.part(f"{side_name}_propeller")
        rotor = FanRotorGeometry(
            1.07,
            0.22,
            5,
            thickness=0.085,
            blade_pitch_deg=34.0,
            blade_sweep_deg=28.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.12),
            hub=FanRotorHub(style="spinner", bore_diameter=0.10),
        )
        propeller.visual(mesh_from_geometry(rotor, f"{side_name}_propeller"), material=dark, name="rotor")
        propeller.visual(
            Cylinder(radius=0.070, length=0.46),
            origin=Origin(xyz=(0.0, 0.0, -0.23)),
            material=metal,
            name="shaft",
        )
        model.articulation(
            f"{side_name}_propeller_spin",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=propeller,
            origin=Origin(xyz=(1.22, sign * 0.64, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=90.0, velocity=120.0),
        )

    add_engine("port", 1.0)
    add_engine("starboard", -1.0)

    rudder = model.part("rudder")
    rudder_mesh = _vertical_prism(
        [(-0.060, -3.20), (-1.45, -2.35), (-1.55, 2.35), (-0.060, 3.20)],
        thickness_y=0.18,
    )
    rudder.visual(mesh_from_geometry(rudder_mesh, "rudder_panel"), material=tail_blue, name="panel")
    rudder.visual(
        Box((0.055, 0.20, 2.90)),
        origin=Origin(xyz=(-1.52, 0.0, 0.0)),
        material=red,
        name="red_trailing_edge",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-27.75, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.75, lower=-0.44, upper=0.44),
    )

    def add_elevator(side_name: str, sign: float) -> None:
        elevator = model.part(f"{side_name}_elevator")
        points = [
            (-0.055, sign * 1.22),
            (-2.85, sign * 1.46),
            (-3.40, sign * 6.10),
            (-0.055, sign * 6.72),
        ]
        elevator.visual(
            mesh_from_geometry(_horizontal_prism(points, thickness_z=0.16), f"{side_name}_elevator_panel"),
            material=tail_blue,
            name="panel",
        )
        elevator.visual(
            Box((0.40, 0.78, 0.09)),
            origin=Origin(xyz=(-3.20, sign * 4.85, 0.0)),
            material=red,
            name="red_tip",
        )
        model.articulation(
            f"{side_name}_elevator_hinge",
            ArticulationType.REVOLUTE,
            parent=hull,
            child=elevator,
            origin=Origin(xyz=(-25.40, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=180.0, velocity=0.8, lower=-0.35, upper=0.35),
        )

    add_elevator("port", 1.0)
    add_elevator("starboard", -1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hull = object_model.get_part("hull")
    port_pod = object_model.get_part("port_pod")
    starboard_pod = object_model.get_part("starboard_pod")
    port_propeller = object_model.get_part("port_propeller")
    starboard_propeller = object_model.get_part("starboard_propeller")
    rudder = object_model.get_part("rudder")
    port_elevator = object_model.get_part("port_elevator")
    starboard_elevator = object_model.get_part("starboard_elevator")

    port_vector = object_model.get_articulation("port_vector")
    starboard_vector = object_model.get_articulation("starboard_vector")
    port_spin = object_model.get_articulation("port_propeller_spin")
    starboard_spin = object_model.get_articulation("starboard_propeller_spin")
    rudder_hinge = object_model.get_articulation("rudder_hinge")
    port_elevator_hinge = object_model.get_articulation("port_elevator_hinge")
    starboard_elevator_hinge = object_model.get_articulation("starboard_elevator_hinge")

    ctx.check(
        "two propellers use continuous shaft joints",
        port_spin.articulation_type == ArticulationType.CONTINUOUS
        and starboard_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"port={port_spin.articulation_type}, starboard={starboard_spin.articulation_type}",
    )
    ctx.check(
        "engine pods have bounded vectoring pivots",
        port_vector.motion_limits is not None
        and starboard_vector.motion_limits is not None
        and port_vector.motion_limits.lower < -0.40
        and port_vector.motion_limits.upper > 0.40
        and starboard_vector.motion_limits.lower < -0.40
        and starboard_vector.motion_limits.upper > 0.40,
        details=f"port={port_vector.motion_limits}, starboard={starboard_vector.motion_limits}",
    )
    ctx.check(
        "tail control surfaces are hinged",
        rudder_hinge.articulation_type == ArticulationType.REVOLUTE
        and port_elevator_hinge.articulation_type == ArticulationType.REVOLUTE
        and starboard_elevator_hinge.articulation_type == ArticulationType.REVOLUTE,
        details="rudder and both elevators should be revolute controls",
    )

    ctx.expect_contact(
        hull,
        port_pod,
        elem_a="port_pylon_tip",
        elem_b="pivot_lug",
        contact_tol=0.006,
        name="port pod is carried by pylon tip",
    )
    ctx.expect_contact(
        hull,
        starboard_pod,
        elem_a="starboard_pylon_tip",
        elem_b="pivot_lug",
        contact_tol=0.006,
        name="starboard pod is carried by pylon tip",
    )
    ctx.expect_gap(
        port_propeller,
        port_pod,
        axis="x",
        min_gap=0.12,
        max_gap=0.35,
        positive_elem="rotor",
        negative_elem="cowl_ring",
        name="port propeller sits forward of nacelle cowl",
    )
    ctx.expect_gap(
        starboard_propeller,
        starboard_pod,
        axis="x",
        min_gap=0.12,
        max_gap=0.35,
        positive_elem="rotor",
        negative_elem="cowl_ring",
        name="starboard propeller sits forward of nacelle cowl",
    )
    ctx.allow_overlap(
        port_pod,
        port_propeller,
        elem_a="nacelle",
        elem_b="shaft",
        reason="The rotating propeller shaft is intentionally captured inside the simplified solid nacelle nose bearing.",
    )
    ctx.expect_overlap(
        port_propeller,
        port_pod,
        axes="x",
        elem_a="shaft",
        elem_b="nacelle",
        min_overlap=0.10,
        name="port propeller shaft remains inserted in nacelle bearing",
    )
    ctx.allow_overlap(
        starboard_pod,
        starboard_propeller,
        elem_a="nacelle",
        elem_b="shaft",
        reason="The rotating propeller shaft is intentionally captured inside the simplified solid nacelle nose bearing.",
    )
    ctx.expect_overlap(
        starboard_propeller,
        starboard_pod,
        axes="x",
        elem_a="shaft",
        elem_b="nacelle",
        min_overlap=0.10,
        name="starboard propeller shaft remains inserted in nacelle bearing",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_port_nacelle = _aabb_center(ctx.part_element_world_aabb(port_pod, elem="nacelle"))
    with ctx.pose({port_vector: 0.42}):
        vectored_port_nacelle = _aabb_center(ctx.part_element_world_aabb(port_pod, elem="nacelle"))
    ctx.check(
        "port pod visibly vectors about pylon",
        rest_port_nacelle is not None
        and vectored_port_nacelle is not None
        and vectored_port_nacelle[0] < rest_port_nacelle[0] - 0.18,
        details=f"rest={rest_port_nacelle}, vectored={vectored_port_nacelle}",
    )

    rest_rudder = _aabb_center(ctx.part_world_aabb(rudder))
    with ctx.pose({rudder_hinge: 0.35}):
        yawed_rudder = _aabb_center(ctx.part_world_aabb(rudder))
    ctx.check(
        "rudder swings on vertical tail hinge",
        rest_rudder is not None and yawed_rudder is not None and yawed_rudder[1] < rest_rudder[1] - 0.15,
        details=f"rest={rest_rudder}, yawed={yawed_rudder}",
    )

    rest_elevator_aabb = ctx.part_world_aabb(port_elevator)
    with ctx.pose({port_elevator_hinge: 0.28, starboard_elevator_hinge: 0.28}):
        raised_port_aabb = ctx.part_world_aabb(port_elevator)
        raised_starboard_aabb = ctx.part_world_aabb(starboard_elevator)
    ctx.check(
        "elevators pitch about horizontal hinges",
        rest_elevator_aabb is not None
        and raised_port_aabb is not None
        and raised_starboard_aabb is not None
        and raised_port_aabb[1][2] > rest_elevator_aabb[1][2] + 0.45
        and raised_starboard_aabb[1][2] > rest_elevator_aabb[1][2] + 0.45,
        details=f"rest={rest_elevator_aabb}, port={raised_port_aabb}, starboard={raised_starboard_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
