from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _stepped_tube_x(
    stations: list[tuple[float, float]],
    inner_radius: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Hollow tube with varying outside radius, aligned to global X."""
    mesh = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x, outer_radius in stations:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            y = math.cos(angle)
            z = math.sin(angle)
            outer_ring.append(mesh.add_vertex(x, outer_radius * y, outer_radius * z))
            inner_ring.append(mesh.add_vertex(x, inner_radius * y, inner_radius * z))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for ring in range(len(stations) - 1):
        for i in range(segments):
            j = (i + 1) % segments
            _add_quad(mesh, outer[ring][i], outer[ring + 1][i], outer[ring + 1][j], outer[ring][j])
            _add_quad(mesh, inner[ring][j], inner[ring + 1][j], inner[ring + 1][i], inner[ring][i])

    first = 0
    last = len(stations) - 1
    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, outer[first][j], outer[first][i], inner[first][i], inner[first][j])
        _add_quad(mesh, outer[last][i], outer[last][j], inner[last][j], inner[last][i])
    return mesh


def _annular_disk_x(
    thickness: float,
    outer_radius: float,
    inner_radius: float,
    *,
    segments: int = 96,
    tooth_count: int | None = None,
    tooth_depth: float = 0.0,
) -> MeshGeometry:
    """Flat annulus in the YZ plane, with optional small radial teeth."""
    mesh = MeshGeometry()
    x_values = (-thickness / 2.0, thickness / 2.0)
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x in x_values:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            radius = outer_radius
            if tooth_count:
                # Alternate every half-tooth so the perimeter reads as a real chainring.
                phase = (angle * tooth_count / (2.0 * math.pi)) % 1.0
                crown = 1.0 - abs(phase - 0.5) * 2.0
                radius += tooth_depth * max(0.0, crown)
            y = math.cos(angle)
            z = math.sin(angle)
            outer_ring.append(mesh.add_vertex(x, radius * y, radius * z))
            inner_ring.append(mesh.add_vertex(x, inner_radius * y, inner_radius * z))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, outer[0][i], outer[1][i], outer[1][j], outer[0][j])
        _add_quad(mesh, inner[0][j], inner[1][j], inner[1][i], inner[0][i])
        _add_quad(mesh, outer[1][i], inner[1][i], inner[1][j], outer[1][j])
        _add_quad(mesh, outer[0][j], inner[0][j], inner[0][i], outer[0][i])
    return mesh


def _solid_cylinder_x(length: float, radius: float, *, segments: int = 72) -> MeshGeometry:
    mesh = MeshGeometry()
    xs = (-length / 2.0, length / 2.0)
    rings: list[list[int]] = []
    for x in xs:
        ring: list[int] = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            ring.append(mesh.add_vertex(x, radius * math.cos(angle), radius * math.sin(angle)))
        rings.append(ring)
    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, rings[0][i], rings[1][i], rings[1][j], rings[0][j])
    c0 = mesh.add_vertex(xs[0], 0.0, 0.0)
    c1 = mesh.add_vertex(xs[1], 0.0, 0.0)
    for i in range(segments):
        j = (i + 1) % segments
        mesh.add_face(c1, rings[1][i], rings[1][j])
        mesh.add_face(c0, rings[0][j], rings[0][i])
    return mesh


def _capsule_prism_x(
    p0_yz: tuple[float, float],
    p1_yz: tuple[float, float],
    width: float,
    thickness: float,
    *,
    x_center: float = 0.0,
    arc_segments: int = 12,
) -> MeshGeometry:
    """Rounded-end bar between two YZ points, extruded along X."""
    y0, z0 = p0_yz
    y1, z1 = p1_yz
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    if length <= 1e-9:
        raise ValueError("capsule endpoints must be distinct")
    theta = math.atan2(dz, dy)
    radius = width / 2.0
    profile: list[tuple[float, float]] = []

    # End cap at p1, from +normal to -normal through the forward side.
    for step in range(arc_segments + 1):
        angle = theta + math.pi / 2.0 - step * math.pi / arc_segments
        profile.append((y1 + radius * math.cos(angle), z1 + radius * math.sin(angle)))
    # Start cap at p0, from -normal to +normal through the rear side.
    for step in range(arc_segments + 1):
        angle = theta - math.pi / 2.0 - step * math.pi / arc_segments
        profile.append((y0 + radius * math.cos(angle), z0 + radius * math.sin(angle)))

    mesh = MeshGeometry()
    back: list[int] = []
    front: list[int] = []
    for y, z in profile:
        back.append(mesh.add_vertex(x_center - thickness / 2.0, y, z))
    for y, z in profile:
        front.append(mesh.add_vertex(x_center + thickness / 2.0, y, z))
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        _add_quad(mesh, back[i], front[i], front[j], back[j])
    back_center = mesh.add_vertex(x_center - thickness / 2.0, sum(p[0] for p in profile) / n, sum(p[1] for p in profile) / n)
    front_center = mesh.add_vertex(x_center + thickness / 2.0, sum(p[0] for p in profile) / n, sum(p[1] for p in profile) / n)
    for i in range(n):
        j = (i + 1) % n
        mesh.add_face(front_center, front[i], front[j])
        mesh.add_face(back_center, back[j], back[i])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="downhill_freeride_crankset")

    model.material("bb_black", rgba=(0.025, 0.027, 0.030, 1.0))
    model.material("matte_black", rgba=(0.035, 0.037, 0.040, 1.0))
    model.material("gunmetal", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("guard_black", rgba=(0.01, 0.012, 0.014, 1.0))
    model.material("bolt_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("rubber", rgba=(0.012, 0.012, 0.013, 1.0))

    shell = model.part("bottom_shell")
    shell.visual(
        mesh_from_geometry(
            _stepped_tube_x(
                [
                    (-0.066, 0.036),
                    (-0.054, 0.036),
                    (-0.048, 0.029),
                    (0.048, 0.029),
                    (0.054, 0.036),
                    (0.066, 0.036),
                ],
                inner_radius=0.020,
            ),
            "wide_bottom_bracket_shell",
        ),
        material="bb_black",
        name="bb_shell",
    )
    shell.visual(
        Box((0.128, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="gunmetal",
        name="bearing_pad_top",
    )
    shell.visual(
        Box((0.128, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material="gunmetal",
        name="bearing_pad_bottom",
    )
    crank = model.part("crank")
    crank.visual(
        mesh_from_geometry(_solid_cylinder_x(0.245, 0.012), "hollowtech_spindle"),
        material="brushed_steel",
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.033, length=0.050),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="gunmetal",
        name="drive_hub",
    )
    crank.visual(
        Cylinder(radius=0.031, length=0.036),
        origin=Origin(xyz=(-0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="gunmetal",
        name="arm_hub",
    )
    crank.visual(
        mesh_from_geometry(
            _capsule_prism_x((0.0, -0.006), (0.0, -0.166), 0.036, 0.019, x_center=0.082),
            "oversized_crank_arm_down",
        ),
        material="matte_black",
        name="arm_down",
    )
    crank.visual(
        mesh_from_geometry(
            _capsule_prism_x((0.0, 0.006), (0.0, 0.166), 0.036, 0.019, x_center=-0.082),
            "oversized_crank_arm_up",
        ),
        material="matte_black",
        name="arm_up",
    )
    crank.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.082, 0.0, -0.166), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="pedal_boss_down",
    )
    crank.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(-0.082, 0.0, 0.166), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="pedal_boss_up",
    )
    crank.visual(
        mesh_from_geometry(
            _annular_disk_x(0.007, 0.101, 0.065, segments=120),
            "bash_guard_outer_ring",
        ),
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        material="guard_black",
        name="bash_guard",
    )
    crank.visual(
        mesh_from_geometry(
            _annular_disk_x(0.004, 0.077, 0.050, segments=128, tooth_count=32, tooth_depth=0.005),
            "toothed_chainring",
        ),
        origin=Origin(xyz=(0.101, 0.0, 0.0)),
        material="brushed_steel",
        name="chainring",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        end = (0.074 * math.cos(angle), 0.074 * math.sin(angle))
        crank.visual(
            mesh_from_geometry(
                _capsule_prism_x((0.0, 0.0), end, 0.014, 0.012, x_center=0.104),
                f"spider_spoke_{index}",
            ),
            material="gunmetal",
            name=f"spider_spoke_{index}",
        )
        crank.visual(
            Cylinder(radius=0.0055, length=0.005),
            origin=Origin(
                xyz=(0.1175, end[0], end[1]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="bolt_steel",
            name=f"guard_bolt_{index}",
        )

    for name, side in (("pedal_0", 1.0), ("pedal_1", -1.0)):
        pedal = model.part(name)
        pedal.visual(
            Cylinder(radius=0.0053, length=0.052),
            origin=Origin(xyz=(side * 0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name="axle",
        )
        pedal.visual(
            Box((0.022, 0.106, 0.056)),
            origin=Origin(xyz=(side * 0.058, 0.0, 0.0)),
            material="rubber",
            name="platform_body",
        )
        pedal.visual(
            Box((0.026, 0.010, 0.064)),
            origin=Origin(xyz=(side * 0.058, 0.050, 0.0)),
            material="matte_black",
            name="outer_rail_0",
        )
        pedal.visual(
            Box((0.026, 0.010, 0.064)),
            origin=Origin(xyz=(side * 0.058, -0.050, 0.0)),
            material="matte_black",
            name="outer_rail_1",
        )
        pedal.visual(
            Box((0.026, 0.088, 0.008)),
            origin=Origin(xyz=(side * 0.058, 0.0, 0.030)),
            material="matte_black",
            name="grip_bar_0",
        )
        pedal.visual(
            Box((0.026, 0.088, 0.008)),
            origin=Origin(xyz=(side * 0.058, 0.0, -0.030)),
            material="matte_black",
            name="grip_bar_1",
        )

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=crank,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "pedal_0_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child="pedal_0",
        origin=Origin(xyz=(0.093, 0.0, -0.166)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "pedal_1_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child="pedal_1",
        origin=Origin(xyz=(-0.093, 0.0, 0.166)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crank = object_model.get_part("crank")
    shell = object_model.get_part("bottom_shell")
    pedal_0 = object_model.get_part("pedal_0")
    pedal_1 = object_model.get_part("pedal_1")
    spindle_spin = object_model.get_articulation("spindle_spin")
    pedal_0_spin = object_model.get_articulation("pedal_0_spin")
    pedal_1_spin = object_model.get_articulation("pedal_1_spin")

    ctx.check(
        "crank and pedals use continuous rotation",
        spindle_spin.articulation_type == ArticulationType.CONTINUOUS
        and pedal_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and pedal_1_spin.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.expect_within(
        crank,
        shell,
        axes="yz",
        inner_elem="spindle",
        outer_elem="bb_shell",
        margin=0.0,
        name="spindle is centered inside the wide shell projection",
    )
    ctx.expect_overlap(
        crank,
        shell,
        axes="x",
        elem_a="spindle",
        elem_b="bb_shell",
        min_overlap=0.120,
        name="spindle passes fully through shell width",
    )
    ctx.expect_contact(
        pedal_0,
        crank,
        elem_a="axle",
        elem_b="pedal_boss_down",
        contact_tol=0.001,
        name="lower platform pedal axle seats on crank boss",
    )
    ctx.expect_contact(
        pedal_1,
        crank,
        elem_a="axle",
        elem_b="pedal_boss_up",
        contact_tol=0.001,
        name="upper platform pedal axle seats on crank boss",
    )

    rest_pos = ctx.part_world_position(pedal_0)
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(pedal_0)
    ctx.check(
        "crank spin carries pedal around bottom bracket",
        rest_pos is not None
        and spun_pos is not None
        and spun_pos[1] > rest_pos[1] + 0.12
        and abs(spun_pos[2]) < 0.020,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    rest_aabb = ctx.part_world_aabb(pedal_0)
    with ctx.pose({pedal_0_spin: math.pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(pedal_0)
    if rest_aabb is not None and spun_aabb is not None:
        rest_size_y = rest_aabb[1][1] - rest_aabb[0][1]
        rest_size_z = rest_aabb[1][2] - rest_aabb[0][2]
        spun_size_y = spun_aabb[1][1] - spun_aabb[0][1]
        spun_size_z = spun_aabb[1][2] - spun_aabb[0][2]
        pedal_rotates = spun_size_y < rest_size_y - 0.020 and spun_size_z > rest_size_z + 0.020
    else:
        pedal_rotates = False
    ctx.check("platform pedal rotates about its axle", pedal_rotates)

    return ctx.report()


object_model = build_object_model()
