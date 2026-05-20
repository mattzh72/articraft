from __future__ import annotations

from math import cos, pi, sin

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
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _helical_band(
    inner_radius: float,
    outer_radius: float,
    z_start: float,
    z_end: float,
    pitch: float,
    *,
    width: float = 0.0018,
    segments_per_turn: int = 18,
    phase: float = 0.0,
) -> MeshGeometry:
    """A low, broad rectangular-section helical strip used as visible screw thread."""

    turns = (z_end - z_start) / pitch
    steps = max(12, int(abs(turns) * segments_per_turn))
    half_width = width / 2.0
    geom = MeshGeometry()
    rings: list[tuple[int, int, int, int]] = []

    for i in range(steps + 1):
        t = i / steps
        angle = phase + 2.0 * pi * turns * t
        z = z_start + (z_end - z_start) * t
        ca = cos(angle)
        sa = sin(angle)
        rings.append(
            (
                geom.add_vertex(inner_radius * ca, inner_radius * sa, z - half_width),
                geom.add_vertex(outer_radius * ca, outer_radius * sa, z - half_width),
                geom.add_vertex(outer_radius * ca, outer_radius * sa, z + half_width),
                geom.add_vertex(inner_radius * ca, inner_radius * sa, z + half_width),
            )
        )

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for current, nxt in zip(rings, rings[1:]):
        quad(current[0], nxt[0], nxt[1], current[1])
        quad(current[1], nxt[1], nxt[2], current[2])
        quad(current[2], nxt[2], nxt[3], current[3])
        quad(current[3], nxt[3], nxt[0], current[0])
    quad(*rings[0])
    quad(rings[-1][3], rings[-1][2], rings[-1][1], rings[-1][0])
    return geom


def _glass_envelope() -> LatheGeometry:
    """A broad, shallow frosted envelope profile revolved around the socket axis."""

    return LatheGeometry(
        [
            (0.0, 0.014),
            (0.019, 0.014),
            (0.045, 0.026),
            (0.080, 0.048),
            (0.087, 0.066),
            (0.081, 0.082),
            (0.056, 0.096),
            (0.0, 0.104),
        ],
        segments=72,
        closed=True,
    )


def _bulb_threaded_base() -> cq.Workplane:
    core_radius = 0.0142
    z_min = -0.063
    height = 0.056

    core = cq.Workplane("XY").circle(core_radius).extrude(height + 0.008).translate((0, 0, z_min))
    lower_crimp = _annular_cylinder(0.0158, 0.0065, 0.004, z_min)
    upper_crimp = _annular_cylinder(0.0162, 0.0065, 0.004, -0.006)
    return core.union(lower_crimp).union(upper_crimp)


def _socket_collar() -> cq.Workplane:
    collar = _annular_cylinder(0.0220, 0.0172, 0.064, -0.064)
    top_roll = _annular_cylinder(0.0232, 0.0172, 0.004, -0.004)
    lower_roll = _annular_cylinder(0.0224, 0.0168, 0.004, -0.064)
    return collar.union(top_roll).union(lower_roll)


def _socket_body() -> cq.Workplane:
    shell = _annular_cylinder(0.039, 0.022, 0.074, -0.074)
    top_lip = _annular_cylinder(0.043, 0.022, 0.008, -0.004)
    lower_body = cq.Workplane("XY").circle(0.033).extrude(0.018).translate((0.0, 0.0, -0.092))
    foot = _annular_cylinder(0.047, 0.024, 0.006, -0.092)
    return shell.union(top_lip).union(lower_body).union(foot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_bulb_socket")

    ceramic = Material("warm_white_ceramic", rgba=(0.86, 0.82, 0.72, 1.0))
    brushed_metal = Material("brushed_silver", rgba=(0.74, 0.72, 0.67, 1.0))
    brass = Material("warm_brass", rgba=(0.95, 0.67, 0.25, 1.0))
    frosted_glass = Material("frosted_glass", rgba=(0.88, 0.96, 1.0, 0.58))

    socket = model.part("socket")
    socket.visual(
        mesh_from_cadquery(_socket_body(), "socket_body", tolerance=0.0008, angular_tolerance=0.08),
        material=ceramic,
        name="body",
    )
    socket.visual(
        mesh_from_cadquery(_socket_collar(), "socket_collar", tolerance=0.0005, angular_tolerance=0.06),
        material=brushed_metal,
        name="collar",
    )
    socket.visual(
        mesh_from_geometry(
            _helical_band(0.0168, 0.0186, -0.058, -0.010, 0.010, phase=pi / 7.0),
            "collar_thread",
        ),
        material=brushed_metal,
        name="collar_thread",
    )
    socket.visual(
        Box((0.030, 0.004, 0.0015)),
        origin=Origin(xyz=(0.011, 0.0, -0.06675)),
        material=brass,
        name="contact_tab",
    )
    socket.visual(
        Cylinder(radius=0.0070, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.0670)),
        material=brass,
        name="bottom_contact",
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_cadquery(_bulb_threaded_base(), "bulb_threaded_base", tolerance=0.00045, angular_tolerance=0.05),
        material=brushed_metal,
        name="threaded_base",
    )
    bulb.visual(
        mesh_from_geometry(_helical_band(0.0140, 0.0160, -0.058, -0.010, 0.010), "bulb_spiral_thread"),
        material=brushed_metal,
        name="spiral_thread",
    )
    bulb.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_metal,
        name="seat_band",
    )
    bulb.visual(
        Cylinder(radius=0.0060, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, -0.06425)),
        material=brass,
        name="contact_button",
    )
    bulb.visual(
        mesh_from_geometry(_glass_envelope(), "glass_envelope"),
        material=frosted_glass,
        name="envelope",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    joint = object_model.get_articulation("socket_to_bulb")

    ctx.check(
        "bulb rotates continuously on socket axis",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="collar",
        margin=0.001,
        name="threaded base stays centered inside collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="collar",
        min_overlap=0.050,
        name="threaded base remains seated in collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="seat_band",
        negative_elem="collar",
        min_gap=0.0,
        max_gap=0.001,
        name="bulb shoulder seats on collar lip",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="contact_button",
        elem_b="bottom_contact",
        contact_tol=0.001,
        name="center contact meets socket contact",
    )

    envelope_aabb = ctx.part_element_world_aabb(bulb, elem="envelope")
    if envelope_aabb is None:
        ctx.fail("envelope has measurable broad shallow shape", "missing envelope AABB")
    else:
        env_min, env_max = envelope_aabb
        dx = env_max[0] - env_min[0]
        dy = env_max[1] - env_min[1]
        dz = env_max[2] - env_min[2]
        ctx.check(
            "envelope is broad and shallow",
            dx > 0.14 and dy > 0.14 and dz < 0.10 and min(dx, dy) > 1.5 * dz,
            details=f"envelope dims=({dx:.3f}, {dy:.3f}, {dz:.3f})",
        )

    with ctx.pose({joint: pi / 2.0}):
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="contact_button",
            elem_b="bottom_contact",
            contact_tol=0.001,
            name="contact remains seated after quarter turn",
        )

    return ctx.report()


object_model = build_object_model()
