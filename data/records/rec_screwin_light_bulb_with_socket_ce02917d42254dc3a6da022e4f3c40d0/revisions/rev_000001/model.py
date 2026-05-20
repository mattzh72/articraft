from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_cylinder_mesh(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Connected open tube with annular top and bottom lips."""
    geom = MeshGeometry()
    rings: dict[tuple[str, str], list[int]] = {}
    for z_name, z in (("bottom", z_min), ("top", z_max)):
        for r_name, r in (("outer", outer_radius), ("inner", inner_radius)):
            ring: list[int] = []
            for i in range(segments):
                theta = 2.0 * math.pi * i / segments
                ring.append(geom.add_vertex(r * math.cos(theta), r * math.sin(theta), z))
            rings[(z_name, r_name)] = ring

    bottom_outer = rings[("bottom", "outer")]
    top_outer = rings[("top", "outer")]
    bottom_inner = rings[("bottom", "inner")]
    top_inner = rings[("top", "inner")]
    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(bottom_outer[i], bottom_outer[j], top_outer[j])
        geom.add_face(bottom_outer[i], top_outer[j], top_outer[i])
        # Inner wall.
        geom.add_face(bottom_inner[j], bottom_inner[i], top_inner[i])
        geom.add_face(bottom_inner[j], top_inner[i], top_inner[j])
        # Top annular lip.
        geom.add_face(top_outer[i], top_outer[j], top_inner[j])
        geom.add_face(top_outer[i], top_inner[j], top_inner[i])
        # Bottom annular lip.
        geom.add_face(bottom_outer[j], bottom_outer[i], bottom_inner[i])
        geom.add_face(bottom_outer[j], bottom_inner[i], bottom_inner[j])
    return geom


def _helix_points(
    radius: float,
    z_min: float,
    z_max: float,
    *,
    turns: float,
    samples: int,
    phase: float = 0.0,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples + 1):
        t = i / samples
        theta = phase + 2.0 * math.pi * turns * t
        z = z_min + (z_max - z_min) * t
        points.append((radius * math.cos(theta), radius * math.sin(theta), z))
    return points


def _thread_tube(
    radius: float,
    z_min: float,
    z_max: float,
    *,
    turns: float,
    tube_radius: float,
    name_samples: int = 80,
    phase: float = 0.0,
) -> MeshGeometry:
    return tube_from_spline_points(
        _helix_points(radius, z_min, z_max, turns=turns, samples=name_samples, phase=phase),
        radius=tube_radius,
        samples_per_segment=2,
        radial_segments=10,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _filament_mesh() -> MeshGeometry:
    """One continuous supported lead-wire path with a vintage glowing filament."""
    path: list[tuple[float, float, float]] = []
    path.extend([(0.0, 0.0, 0.006), (0.0, 0.0, 0.045), (-0.006, 0.0, 0.049), (-0.006, 0.0, 0.079)])
    for i in range(41):
        t = i / 40
        x = -0.006 + 0.012 * t
        y = 0.0014 * math.sin(2.0 * math.pi * 5.0 * t)
        z = 0.080 + 0.0008 * math.cos(2.0 * math.pi * 5.0 * t)
        path.append((x, y, z))
    path.extend([(0.006, 0.0, 0.079), (0.006, 0.0, 0.049), (0.0, 0.0, 0.045), (0.0, 0.0, 0.006)])
    return tube_from_spline_points(
        path,
        radius=0.00055,
        samples_per_segment=3,
        radial_segments=8,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_screw_bulb_socket")

    bakelite = model.material("aged_bakelite", rgba=(0.08, 0.055, 0.035, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    brass = model.material("brushed_brass", rgba=(0.78, 0.57, 0.25, 1.0))
    dull_brass = model.material("aged_brass", rgba=(0.55, 0.39, 0.16, 1.0))
    steel = model.material("blued_steel", rgba=(0.25, 0.27, 0.28, 1.0))
    ceramic = model.material("warm_ceramic", rgba=(0.86, 0.80, 0.66, 1.0))
    glass = model.material("slightly_smoky_glass", rgba=(0.86, 0.95, 1.0, 0.38))
    glow = model.material("warm_filament", rgba=(1.0, 0.55, 0.12, 1.0))
    gasket = model.material("red_fiber_gasket", rgba=(0.48, 0.06, 0.035, 1.0))

    socket = model.part("socket")

    socket.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.021, 0.034, 0.006, 0.064), "socket_body"),
        material=bakelite,
        name="socket_body",
    )
    socket.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.019, 0.026, 0.040, 0.078), "thread_collar"),
        material=brass,
        name="thread_collar",
    )
    socket.visual(
        mesh_from_geometry(
            _thread_tube(0.01855, 0.044, 0.074, turns=2.45, tube_radius=0.00085, phase=0.35),
            "socket_internal_thread",
        ),
        material=dull_brass,
        name="internal_thread",
    )
    socket.visual(
        Cylinder(radius=0.044, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=black,
        name="adapter_disk",
    )
    for i, x in enumerate((-0.055, 0.055)):
        socket.visual(
            Box((0.034, 0.020, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.003)),
            material=black,
            name=f"adapter_ear_{i}",
        )
        socket.visual(
            Cylinder(radius=0.0048, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0073)),
            material=steel,
            name=f"adapter_bolt_{i}",
        )
    for i, theta in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        r = 0.0315
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        socket.visual(
            Box((0.006, 0.004, 0.032)),
            origin=Origin(xyz=(x, y, 0.054), rpy=(0.0, 0.0, theta)),
            material=steel,
            name=f"collar_strap_{i}",
        )
        socket.visual(
            Cylinder(radius=0.0027, length=0.0018),
            origin=Origin(xyz=(0.034 * math.cos(theta), 0.034 * math.sin(theta), 0.039), rpy=(math.pi / 2.0, 0.0, theta)),
            material=steel,
            name=f"strap_rivet_{i}",
        )
    socket.visual(
        Box((0.047, 0.006, 0.041)),
        origin=Origin(xyz=(0.0, -0.0365, 0.038)),
        material=bakelite,
        name="hatch_boss",
    )
    socket.visual(
        Box((0.044, 0.005, 0.034)),
        origin=Origin(xyz=(0.0, 0.0365, 0.038)),
        material=bakelite,
        name="terminal_hatch",
    )
    for i, x in enumerate((-0.015, 0.015)):
        socket.visual(
            Cylinder(radius=0.0032, length=0.0022),
            origin=Origin(xyz=(x, 0.0397, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"terminal_screw_{i}",
        )
    socket.visual(
        Box((0.044, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=ceramic,
        name="contact_bridge",
    )
    socket.visual(
        Cylinder(radius=0.0040, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material=ceramic,
        name="contact_post",
    )
    socket.visual(
        Cylinder(radius=0.0060, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0365)),
        material=brass,
        name="spring_contact",
    )

    service_hatch = model.part("service_hatch")
    service_hatch.visual(
        Box((0.041, 0.003, 0.034)),
        origin=Origin(xyz=(0.0205, -0.0015, 0.0)),
        material=bakelite,
        name="hatch_panel",
    )
    service_hatch.visual(
        Cylinder(radius=0.0026, length=0.038),
        origin=Origin(xyz=(0.0, -0.0050, 0.0)),
        material=steel,
        name="hatch_barrel",
    )
    service_hatch.visual(
        Cylinder(radius=0.0031, length=0.002),
        origin=Origin(xyz=(0.034, -0.0036, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="latch_screw",
    )
    service_hatch.visual(
        Box((0.020, 0.0012, 0.003)),
        origin=Origin(xyz=(0.020, -0.0032, -0.012)),
        material=gasket,
        name="hatch_gasket",
    )

    thread_carriage = model.part("thread_carriage")
    thread_carriage.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.0168, 0.0190, -0.040, -0.034, segments=48), "thread_carriage"),
        material=ceramic,
        name="guide_ring",
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0147, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=brass,
        name="screw_core",
    )
    bulb.visual(
        mesh_from_geometry(
            _thread_tube(0.01525, -0.031, -0.004, turns=2.55, tube_radius=0.00105, phase=0.05),
            "bulb_external_thread",
        ),
        material=dull_brass,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass,
        name="base_flange",
    )
    bulb.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=ceramic,
        name="ceramic_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0068, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0355)),
        material=brass,
        name="contact_button",
    )
    bulb_profile = LatheGeometry.from_shell_profiles(
        [
            (0.0115, 0.006),
            (0.0175, 0.018),
            (0.0285, 0.040),
            (0.0340, 0.066),
            (0.0300, 0.094),
            (0.0180, 0.119),
            (0.0060, 0.129),
            (0.0020, 0.131),
        ],
        [
            (0.0100, 0.008),
            (0.0158, 0.020),
            (0.0268, 0.041),
            (0.0320, 0.066),
            (0.0283, 0.093),
            (0.0164, 0.117),
            (0.0048, 0.127),
            (0.0008, 0.129),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    bulb.visual(
        mesh_from_geometry(bulb_profile, "glass_envelope"),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        mesh_from_geometry(_filament_mesh(), "filament_assembly"),
        material=glow,
        name="filament_assembly",
    )
    bulb.visual(
        Box((0.006, 0.0013, 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, -0.015), rpy=(0.0, 0.0, math.radians(16))),
        material=steel,
        name="base_seam",
    )

    model.articulation(
        "socket_to_thread_carriage",
        ArticulationType.FIXED,
        parent=socket,
        child=thread_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )
    model.articulation(
        "thread_carriage_to_bulb",
        ArticulationType.PRISMATIC,
        parent=thread_carriage,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.05, lower=0.0, upper=0.026),
    )
    model.articulation(
        "socket_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=service_hatch,
        origin=Origin(xyz=(-0.024, -0.0395, 0.038)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    carriage = object_model.get_part("thread_carriage")
    hatch = object_model.get_part("service_hatch")
    slide = object_model.get_articulation("thread_carriage_to_bulb")
    hatch_joint = object_model.get_articulation("socket_to_service_hatch")

    ctx.expect_origin_distance(
        bulb,
        socket,
        axes="xy",
        max_dist=0.001,
        name="bulb and socket share the screw axis",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_core",
        outer_elem="thread_collar",
        margin=0.0,
        name="bulb screw base is constrained inside the socket collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_core",
        elem_b="thread_collar",
        min_overlap=0.030,
        name="seated screw base is deeply engaged in the threaded collar",
    )
    ctx.expect_within(
        carriage,
        socket,
        axes="xy",
        inner_elem="guide_ring",
        outer_elem="thread_collar",
        margin=0.0,
        name="coaxial guide ring runs inside the collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="contact_button",
        negative_elem="spring_contact",
        max_gap=0.0015,
        max_penetration=0.0,
        name="bulb contact button seats on the socket spring contact",
    )
    ctx.expect_gap(
        socket,
        hatch,
        axis="y",
        positive_elem="hatch_boss",
        negative_elem="hatch_panel",
        max_gap=0.0001,
        max_penetration=0.0,
        name="closed service hatch seats against its mounting boss",
    )

    rest_pos = ctx.part_world_position(bulb)
    hatch_rest_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({slide: 0.026}):
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="screw_core",
            elem_b="thread_collar",
            min_overlap=0.008,
            name="unscrewed bulb still retains thread engagement",
        )
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="screw_core",
            outer_elem="thread_collar",
            margin=0.0,
            name="unscrewing motion remains coaxial",
        )
        raised_pos = ctx.part_world_position(bulb)
    ctx.check(
        "screw action raises the bulb along the socket axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.020,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({hatch_joint: 1.0}):
        hatch_open_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    hatch_rest_y = None
    hatch_open_y = None
    if hatch_rest_aabb is not None and hatch_open_aabb is not None:
        hatch_rest_y = 0.5 * (hatch_rest_aabb[0][1] + hatch_rest_aabb[1][1])
        hatch_open_y = 0.5 * (hatch_open_aabb[0][1] + hatch_open_aabb[1][1])
    ctx.check(
        "service hatch swings outward from the socket body",
        hatch_rest_y is not None and hatch_open_y is not None and hatch_open_y < hatch_rest_y - 0.010,
        details=f"closed_y={hatch_rest_y}, open_y={hatch_open_y}",
    )

    return ctx.report()


object_model = build_object_model()
