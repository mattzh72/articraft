from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _partial_tube(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 56,
) -> MeshGeometry:
    """Closed C-section tube mesh, useful for cutaway service collars."""
    if end_angle <= start_angle:
        end_angle += 2.0 * math.pi

    geom = MeshGeometry()
    rings: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        t = start_angle + (end_angle - start_angle) * i / segments
        c = math.cos(t)
        s = math.sin(t)
        ob = geom.add_vertex(outer_radius * c, outer_radius * s, z_min)
        ot = geom.add_vertex(outer_radius * c, outer_radius * s, z_max)
        ib = geom.add_vertex(inner_radius * c, inner_radius * s, z_min)
        it = geom.add_vertex(inner_radius * c, inner_radius * s, z_max)
        rings.append((ob, ot, ib, it))

    for i in range(segments):
        ob0, ot0, ib0, it0 = rings[i]
        ob1, ot1, ib1, it1 = rings[i + 1]
        # Outer cylindrical wall.
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        # Inner cylindrical wall.
        geom.add_face(ib0, it1, ib1)
        geom.add_face(ib0, it0, it1)
        # Top and bottom annular faces.
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob0, ib1, ob1)
        geom.add_face(ob0, ib0, ib1)

    # Radial cut faces at both ends of the C section.
    for idx in (0, segments):
        ob, ot, ib, it = rings[idx]
        geom.add_face(ob, ot, it)
        geom.add_face(ob, it, ib)

    return geom


def _helical_band(
    inner_radius: float,
    outer_radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    *,
    width: float,
    start_angle: float = 0.0,
    samples_per_turn: int = 36,
) -> MeshGeometry:
    """A raised rectangular helical thread band around the Z axis."""
    steps = max(12, int(abs(turns) * samples_per_turn))
    geom = MeshGeometry()
    sections: list[tuple[int, int, int, int]] = []
    for i in range(steps + 1):
        u = i / steps
        angle = start_angle + turns * 2.0 * math.pi * u
        z = z_start + (z_end - z_start) * u
        c = math.cos(angle)
        s = math.sin(angle)
        verts = (
            geom.add_vertex(inner_radius * c, inner_radius * s, z - width / 2.0),
            geom.add_vertex(outer_radius * c, outer_radius * s, z - width / 2.0),
            geom.add_vertex(outer_radius * c, outer_radius * s, z + width / 2.0),
            geom.add_vertex(inner_radius * c, inner_radius * s, z + width / 2.0),
        )
        sections.append(verts)

    for i in range(steps):
        a0, b0, c0, d0 = sections[i]
        a1, b1, c1, d1 = sections[i + 1]
        geom.add_face(a0, a1, b1)
        geom.add_face(a0, b1, b0)
        geom.add_face(b0, b1, c1)
        geom.add_face(b0, c1, c0)
        geom.add_face(c0, c1, d1)
        geom.add_face(c0, d1, d0)
        geom.add_face(d0, d1, a1)
        geom.add_face(d0, a1, a0)

    for idx in (0, steps):
        a, b, c, d = sections[idx]
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    return geom


def _socket_shell_geometry() -> MeshGeometry:
    # A broad front window exposes the brass liner and thread path for service.
    return _partial_tube(
        inner_radius=0.040,
        outer_radius=0.052,
        z_min=0.054,
        z_max=0.150,
        start_angle=math.radians(138.0),
        end_angle=math.radians(402.0),
        segments=64,
    )


def _thread_liner_geometry() -> MeshGeometry:
    sleeve = _partial_tube(
        inner_radius=0.034,
        outer_radius=0.039,
        z_min=0.062,
        z_max=0.150,
        start_angle=math.radians(138.0),
        end_angle=math.radians(402.0),
        segments=64,
    )
    # A replaceable top lip seats on the steel socket shell.
    flange = _partial_tube(
        inner_radius=0.032,
        outer_radius=0.048,
        z_min=0.150,
        z_max=0.157,
        start_angle=math.radians(138.0),
        end_angle=math.radians(402.0),
        segments=64,
    )
    internal_thread = _helical_band(
        inner_radius=0.0318,
        outer_radius=0.0345,
        z_start=0.070,
        z_end=0.141,
        turns=4.2,
        width=0.0036,
        start_angle=math.radians(22.0),
        samples_per_turn=40,
    )
    sleeve.merge(flange)
    sleeve.merge(internal_thread)
    return sleeve


def _bulb_screw_base_geometry() -> MeshGeometry:
    base_profile = [
        (0.000, 0.000),
        (0.013, 0.000),
        (0.020, 0.004),
        (0.025, 0.010),
        (0.027, 0.072),
        (0.023, 0.082),
        (0.017, 0.088),
        (0.000, 0.088),
    ]
    base = LatheGeometry(base_profile, segments=72, closed=True)
    thread = _helical_band(
        inner_radius=0.0265,
        outer_radius=0.0304,
        z_start=0.013,
        z_end=0.074,
        turns=4.2,
        width=0.0034,
        start_angle=math.radians(202.0),
        samples_per_turn=40,
    )
    base.merge(thread)
    return base


def _bulb_glass_geometry() -> MeshGeometry:
    # Pear-shaped A-series envelope, translucent enough to reveal the serviceable lamp core.
    profile = [
        (0.000, 0.084),
        (0.015, 0.084),
        (0.020, 0.094),
        (0.035, 0.116),
        (0.051, 0.148),
        (0.056, 0.182),
        (0.049, 0.218),
        (0.031, 0.246),
        (0.011, 0.262),
        (0.000, 0.264),
    ]
    return LatheGeometry(profile, segments=80, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_screw_in_bulb_socket")

    steel = model.material("dark_zinc_steel", rgba=(0.16, 0.17, 0.17, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    brass = model.material("service_brass", rgba=(0.88, 0.61, 0.24, 1.0))
    nickel = model.material("brushed_nickel", rgba=(0.72, 0.72, 0.68, 1.0))
    glass = model.material("frosted_glass", rgba=(0.78, 0.92, 1.00, 0.42))
    ceramic = model.material("white_ceramic", rgba=(0.92, 0.90, 0.82, 1.0))
    tungsten = model.material("warm_tungsten", rgba=(1.0, 0.60, 0.20, 1.0))

    socket = model.part("socket_body")
    socket.visual(
        Box((0.240, 0.300, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=steel,
        name="base_plate",
    )
    socket.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=steel,
        name="welded_pedestal",
    )
    socket.visual(
        mesh_from_geometry(_socket_shell_geometry(), "socket_cutaway_shell"),
        material=steel,
        name="socket_shell",
    )
    # Four gussets weld the collar to the base so the socket reads as a rugged service fixture.
    for idx, angle in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        socket.visual(
            Box((0.024, 0.010, 0.078)),
            origin=Origin(
                xyz=(0.064 * math.cos(angle), 0.064 * math.sin(angle), 0.083),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"collar_gusset_{idx}",
        )

    socket.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=ceramic,
        name="ceramic_contact_seat",
    )
    socket.visual(
        Cylinder(radius=0.009, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.06725)),
        material=brass,
        name="spring_contact",
    )

    socket.visual(
        Box((0.174, 0.090, 0.025)),
        origin=Origin(xyz=(0.0, -0.105, 0.0365)),
        material=steel,
        name="terminal_box",
    )
    socket.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(0.0, -0.166, 0.039), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="strain_relief",
    )
    for x in (-0.034, 0.034):
        socket.visual(
            Box((0.031, 0.020, 0.007)),
            origin=Origin(xyz=(x, -0.105, 0.0525)),
            material=brass,
            name=f"terminal_lug_{0 if x < 0 else 1}",
        )

    # Alternating fixed hinge knuckles and their welded leaves for the service cover.
    for idx, (x, length) in enumerate(((-0.075, 0.022), (0.0, 0.036), (0.075, 0.022))):
        socket.visual(
            Cylinder(radius=0.0058, length=length),
            origin=Origin(xyz=(x, -0.150, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"body_hinge_knuckle_{idx}",
        )
        socket.visual(
            Box((length, 0.024, 0.005)),
            origin=Origin(xyz=(x, -0.138, 0.051)),
            material=steel,
            name=f"body_hinge_leaf_{idx}",
        )

    # Captive set-screw bosses hold the brass liner but stay outside the thread clearance.
    for idx, angle in enumerate((math.radians(0.0), math.radians(180.0))):
        socket.visual(
            Cylinder(radius=0.007, length=0.018),
            origin=Origin(
                xyz=(0.058 * math.cos(angle), 0.058 * math.sin(angle), 0.112),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=steel,
            name=f"liner_set_screw_{idx}",
        )

    for idx, (x, y) in enumerate(((-0.090, 0.105), (0.090, 0.105), (-0.090, -0.105), (0.090, -0.105))):
        socket.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.027)),
            material=steel,
            name=f"mount_boss_{idx}",
        )
        socket.visual(
            Box((0.016, 0.003, 0.002)),
            origin=Origin(xyz=(x, y, 0.0308), rpy=(0.0, 0.0, math.pi / 4.0)),
            material=black,
            name=f"screw_slot_{idx}",
        )

    liner = model.part("thread_liner")
    liner.visual(
        mesh_from_geometry(_thread_liner_geometry(), "replaceable_thread_liner"),
        material=brass,
        name="liner_threads",
    )
    model.articulation(
        "socket_to_liner",
        ArticulationType.FIXED,
        parent=socket,
        child=liner,
        origin=Origin(),
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_bulb_screw_base_geometry(), "bulb_rolled_screw_base"),
        material=nickel,
        name="screw_base",
    )
    bulb.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=brass,
        name="bottom_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=ceramic,
        name="glass_stem",
    )
    for x in (-0.008, 0.008):
        bulb.visual(
            Cylinder(radius=0.0012, length=0.046),
            origin=Origin(xyz=(x, 0.0, 0.116)),
            material=tungsten,
            name=f"filament_support_{0 if x < 0 else 1}",
        )
    bulb.visual(
        Cylinder(radius=0.0016, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tungsten,
        name="filament_bridge",
    )
    bulb.visual(
        mesh_from_geometry(_bulb_glass_geometry(), "frosted_bulb_envelope"),
        material=glass,
        name="glass_envelope",
    )
    model.articulation(
        "liner_to_bulb",
        ArticulationType.REVOLUTE,
        parent=liner,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.5, lower=-2.0 * math.pi, upper=0.0),
    )

    cover = model.part("service_cover")
    cover.visual(
        Box((0.160, 0.068, 0.008)),
        origin=Origin(xyz=(0.0, 0.046, 0.004)),
        material=steel,
        name="cover_plate",
    )
    for idx, x in enumerate((-0.040, 0.040)):
        cover.visual(
            Cylinder(radius=0.0060, length=0.038),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"cover_hinge_knuckle_{idx}",
        )
        cover.visual(
            Box((0.038, 0.024, 0.005)),
            origin=Origin(xyz=(x, 0.012, 0.0025)),
            material=steel,
            name=f"cover_hinge_leaf_{idx}",
        )
    for y in (0.022, 0.058):
        cover.visual(
            Box((0.135, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.0105)),
            material=steel,
            name=f"cover_rib_{0 if y < 0.04 else 1}",
        )
    cover.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.065, 0.011)),
        material=steel,
        name="latch_boss",
    )
    cover.visual(
        Box((0.018, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.065, 0.0148)),
        material=black,
        name="latch_slot",
    )
    model.articulation(
        "socket_to_cover",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=cover,
        origin=Origin(xyz=(0.0, -0.150, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    liner = object_model.get_part("thread_liner")
    bulb = object_model.get_part("bulb")
    socket = object_model.get_part("socket_body")
    cover = object_model.get_part("service_cover")
    screw = object_model.get_articulation("liner_to_bulb")
    cover_hinge = object_model.get_articulation("socket_to_cover")

    ctx.expect_within(
        bulb,
        liner,
        axes="xy",
        inner_elem="screw_base",
        outer_elem="liner_threads",
        margin=0.002,
        name="screw base is coaxially captured by liner",
    )
    ctx.expect_overlap(
        bulb,
        liner,
        axes="z",
        elem_a="screw_base",
        elem_b="liner_threads",
        min_overlap=0.055,
        name="bulb threads remain deeply engaged",
    )
    ctx.check(
        "screw joint uses socket axis",
        tuple(round(v, 4) for v in screw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={screw.axis}",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="bottom_contact",
        negative_elem="spring_contact",
        max_gap=0.001,
        max_penetration=0.001,
        name="center contact seats without hard collision",
    )

    closed_box = ctx.part_element_world_aabb(cover, elem="cover_plate")
    closed_z = (closed_box[0][2] + closed_box[1][2]) / 2.0 if closed_box else None
    with ctx.pose({cover_hinge: 1.15}):
        open_box = ctx.part_element_world_aabb(cover, elem="cover_plate")
        open_z = (open_box[0][2] + open_box[1][2]) / 2.0 if open_box else None
    ctx.check(
        "maintenance cover opens upward",
        closed_z is not None and open_z is not None and open_z > closed_z + 0.020,
        details=f"closed_z={closed_z}, open_z={open_z}",
    )
    ctx.expect_contact(
        liner,
        socket,
        elem_a="liner_threads",
        elem_b="socket_shell",
        contact_tol=0.002,
        name="replaceable liner lip seats on socket shell",
    )

    return ctx.report()


object_model = build_object_model()
