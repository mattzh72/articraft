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
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _parabolic_reflector_geometry(
    *,
    radius: float = 0.55,
    depth: float = 0.32,
    thickness: float = 0.018,
    x_vertex: float = 0.12,
    radial_rings: int = 14,
    segments: int = 72,
) -> MeshGeometry:
    """Build a thin parabolic reflector shell opening along local +X."""
    geom = MeshGeometry()
    inner: list[list[int]] = []
    outer: list[list[int]] = []

    hub_radius = 0.045
    for ring in range(radial_rings + 1):
        t = ring / radial_rings
        r = hub_radius + (radius - hub_radius) * t
        x_inner = x_vertex + depth * (r / radius) ** 2
        # The outer face is slightly behind the reflective face, giving the rim a real lip.
        x_outer = x_inner - thickness * (0.85 + 0.15 * t)
        inner_ring = []
        outer_ring = []
        for seg in range(segments):
            a = 2.0 * math.pi * seg / segments
            y = r * math.cos(a)
            z = r * math.sin(a)
            inner_ring.append(geom.add_vertex(x_inner, y, z))
            outer_ring.append(geom.add_vertex(x_outer, y, z))
        inner.append(inner_ring)
        outer.append(outer_ring)

    for ring in range(radial_rings):
        for seg in range(segments):
            n = (seg + 1) % segments
            # Concave reflective face.
            geom.add_face(inner[ring][seg], inner[ring + 1][seg], inner[ring + 1][n])
            geom.add_face(inner[ring][seg], inner[ring + 1][n], inner[ring][n])
            # Convex rear face.
            geom.add_face(outer[ring][seg], outer[ring + 1][n], outer[ring + 1][seg])
            geom.add_face(outer[ring][seg], outer[ring][n], outer[ring + 1][n])

    # Connect the front rim and the small central service opening into a single shell.
    for seg in range(segments):
        n = (seg + 1) % segments
        geom.add_face(inner[-1][seg], outer[-1][seg], outer[-1][n])
        geom.add_face(inner[-1][seg], outer[-1][n], inner[-1][n])
        geom.add_face(outer[0][seg], inner[0][seg], inner[0][n])
        geom.add_face(outer[0][seg], inner[0][n], outer[0][n])

    return geom


def _frustum_geometry(
    *,
    x0: float,
    x1: float,
    r0: float,
    r1: float,
    segments: int = 40,
    cap_small_end: bool = True,
    cap_large_end: bool = False,
) -> MeshGeometry:
    """Conical feed-horn frustum along local X."""
    geom = MeshGeometry()
    start = []
    end = []
    for seg in range(segments):
        a = 2.0 * math.pi * seg / segments
        ca = math.cos(a)
        sa = math.sin(a)
        start.append(geom.add_vertex(x0, r0 * ca, r0 * sa))
        end.append(geom.add_vertex(x1, r1 * ca, r1 * sa))
    for seg in range(segments):
        n = (seg + 1) % segments
        geom.add_face(start[seg], end[seg], end[n])
        geom.add_face(start[seg], end[n], start[n])
    if cap_small_end:
        c = geom.add_vertex(x0, 0.0, 0.0)
        for seg in range(segments):
            geom.add_face(c, start[(seg + 1) % segments], start[seg])
    if cap_large_end:
        c = geom.add_vertex(x1, 0.0, 0.0)
        for seg in range(segments):
            geom.add_face(c, end[seg], end[(seg + 1) % segments])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_communications_dish")

    dark_paint = model.material("dark_paint", rgba=(0.08, 0.09, 0.095, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.57, 0.58, 1.0))
    light_reflector = model.material("light_reflector", rgba=(0.82, 0.84, 0.82, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.67, 0.08, 1.0))

    base_frame = model.part("base_frame")
    # A low skid-style transport base, wide enough to stabilize a meter-class dish.
    for y, name in ((-0.34, "skid_0"), (0.34, "skid_1")):
        base_frame.visual(
            Box((0.96, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.04)),
            material=dark_paint,
            name=name,
        )
    for x, name in ((-0.40, "crossbar_0"), (0.40, "crossbar_1")):
        base_frame.visual(
            Box((0.11, 0.76, 0.065)),
            origin=Origin(xyz=(x, 0.0, 0.065)),
            material=dark_paint,
            name=name,
        )
    base_frame.visual(
        Box((1.08, 0.10, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_paint,
        name="center_spine",
    )
    base_frame.visual(
        Box((0.46, 0.46, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_paint,
        name="center_plate",
    )
    base_frame.visual(
        Cylinder(radius=0.19, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=satin_metal,
        name="azimuth_bearing",
    )
    base_frame.visual(
        Box((0.12, 0.25, 0.035)),
        origin=Origin(xyz=(-0.52, 0.0, 0.215)),
        material=dark_paint,
        name="leg_hinge_bridge",
    )
    for y, name in ((-0.22, "hinge_support_0"), (0.22, "hinge_support_1")):
        base_frame.visual(
            Box((0.080, 0.280, 0.065)),
            origin=Origin(xyz=(-0.52, y, 0.085)),
            material=dark_paint,
            name=name,
        )
    for y, name in ((-0.095, "leg_hinge_lug_0"), (0.095, "leg_hinge_lug_1")):
        base_frame.visual(
            Box((0.090, 0.050, 0.105)),
            origin=Origin(xyz=(-0.52, y, 0.160)),
            material=dark_paint,
            name=name,
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.170, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=satin_metal,
        name="turntable",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_paint,
        name="rotating_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.065, length=0.790),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=dark_paint,
        name="main_mast",
    )
    pedestal.visual(
        Box((0.28, 0.82, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        material=dark_paint,
        name="yoke_base",
    )
    for y, name in ((-0.34, "yoke_cheek_0"), (0.34, "yoke_cheek_1")):
        pedestal.visual(
            Box((0.22, 0.060, 0.480)),
            origin=Origin(xyz=(0.0, y, 1.160)),
            material=dark_paint,
            name=name,
        )
    pedestal.visual(
        Box((0.20, 0.055, 0.18)),
        origin=Origin(xyz=(0.105, -0.135, 0.48)),
        material=safety_yellow,
        name="service_box",
    )
    pedestal.visual(
        Box((0.13, 0.055, 0.080)),
        origin=Origin(xyz=(0.075, -0.088, 0.48)),
        material=dark_paint,
        name="service_bracket",
    )

    elevation_frame = model.part("elevation_frame")
    elevation_frame.visual(
        Cylinder(radius=0.035, length=0.780),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="trunnion_pin",
    )
    elevation_frame.visual(
        Cylinder(radius=0.073, length=0.120),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_paint,
        name="rear_hub",
    )
    elevation_frame.visual(
        mesh_from_geometry(_parabolic_reflector_geometry(), "reflector_shell"),
        material=light_reflector,
        name="reflector_shell",
    )
    rim = TorusGeometry(radius=0.55, tube=0.015, radial_segments=24, tubular_segments=96)
    rim.rotate_y(math.pi / 2.0).translate(0.44, 0.0, 0.0)
    elevation_frame.visual(
        mesh_from_geometry(rim, "rim_ring"),
        material=satin_metal,
        name="rim_ring",
    )
    elevation_frame.visual(
        mesh_from_geometry(
            wire_from_points(
                [(0.0, 0.0, 0.0), (0.18, 0.0, 0.0)],
                radius=0.026,
                radial_segments=18,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "backbone_tube",
        ),
        material=dark_paint,
        name="backbone_tube",
    )
    strut_points = [
        ((0.435, 0.43, 0.36), (0.725, 0.030, 0.025)),
        ((0.435, -0.43, 0.36), (0.725, -0.030, 0.025)),
        ((0.435, 0.43, -0.36), (0.725, 0.030, -0.025)),
        ((0.435, -0.43, -0.36), (0.725, -0.030, -0.025)),
    ]
    for idx, (start, end) in enumerate(strut_points):
        elevation_frame.visual(
            mesh_from_geometry(
                wire_from_points(
                    [start, end],
                    radius=0.010,
                    radial_segments=12,
                    cap_ends=True,
                    up_hint=(0.0, 0.0, 1.0),
                ),
                f"feed_strut_{idx}",
            ),
            material=satin_metal,
            name=f"feed_strut_{idx}",
        )
    elevation_frame.visual(
        Cylinder(radius=0.045, length=0.105),
        origin=Origin(xyz=(0.735, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_paint,
        name="feed_can",
    )
    elevation_frame.visual(
        mesh_from_geometry(
            _frustum_geometry(x0=0.775, x1=0.900, r0=0.040, r1=0.085),
            "feed_horn",
        ),
        material=satin_metal,
        name="feed_horn",
    )

    transport_leg = model.part("transport_leg")
    transport_leg.visual(
        Cylinder(radius=0.025, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    transport_leg.visual(
        mesh_from_geometry(
            wire_from_points(
                [(-0.018, 0.0, -0.018), (-0.36, 0.0, -0.130)],
                radius=0.019,
                radial_segments=16,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "leg_tube",
        ),
        material=dark_paint,
        name="leg_tube",
    )
    transport_leg.visual(
        Box((0.20, 0.16, 0.035)),
        origin=Origin(xyz=(-0.43, 0.0, -0.150)),
        material=black_rubber,
        name="foot_pad",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.75, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=elevation_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=0.55, lower=0.0, upper=1.15),
    )
    model.articulation(
        "leg_hinge",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=transport_leg,
        origin=Origin(xyz=(-0.52, 0.0, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    pedestal = object_model.get_part("pedestal")
    elevation = object_model.get_part("elevation_frame")
    leg = object_model.get_part("transport_leg")
    azimuth = object_model.get_articulation("azimuth_axis")
    elevation_axis = object_model.get_articulation("elevation_axis")
    leg_hinge = object_model.get_articulation("leg_hinge")

    # The elevation pin is intentionally captured in the two pedestal yoke cheeks.
    for cheek in ("yoke_cheek_0", "yoke_cheek_1"):
        ctx.allow_overlap(
            pedestal,
            elevation,
            elem_a=cheek,
            elem_b="trunnion_pin",
            reason="The trunnion pin is intentionally seated through the elevation yoke cheek.",
        )
        ctx.expect_overlap(
            pedestal,
            elevation,
            elem_a=cheek,
            elem_b="trunnion_pin",
            axes="yz",
            min_overlap=0.030,
            name=f"{cheek} captures trunnion pin",
        )

    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        positive_elem="turntable",
        negative_elem="azimuth_bearing",
        max_gap=0.002,
        max_penetration=0.001,
        name="turntable seats on azimuth bearing",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="xy",
        elem_a="turntable",
        elem_b="azimuth_bearing",
        min_overlap=0.16,
        name="azimuth bearing footprint is centered",
    )
    ctx.expect_contact(
        leg,
        base,
        elem_a="hinge_barrel",
        elem_b="leg_hinge_lug_0",
        contact_tol=0.012,
        name="leg hinge barrel is carried by first lug",
    )
    ctx.expect_contact(
        leg,
        base,
        elem_a="hinge_barrel",
        elem_b="leg_hinge_lug_1",
        contact_tol=0.012,
        name="leg hinge barrel is carried by second lug",
    )

    feed_rest = ctx.part_element_world_aabb(elevation, elem="feed_horn")
    with ctx.pose({elevation_axis: 0.80}):
        feed_raised = ctx.part_element_world_aabb(elevation, elem="feed_horn")
    ctx.check(
        "elevation raises feed",
        feed_rest is not None
        and feed_raised is not None
        and feed_raised[0][2] > feed_rest[0][2] + 0.35,
        details=f"rest={feed_rest}, raised={feed_raised}",
    )

    foot_deployed = ctx.part_element_world_aabb(leg, elem="foot_pad")
    with ctx.pose({leg_hinge: 1.20}):
        foot_folded = ctx.part_element_world_aabb(leg, elem="foot_pad")
    ctx.check(
        "transport leg folds upward",
        foot_deployed is not None
        and foot_folded is not None
        and foot_folded[0][2] > foot_deployed[0][2] + 0.25,
        details=f"deployed={foot_deployed}, folded={foot_folded}",
    )

    with ctx.pose({azimuth: 1.0}):
        ctx.expect_gap(
            pedestal,
            base,
            axis="z",
            positive_elem="turntable",
            negative_elem="azimuth_bearing",
            max_gap=0.002,
            max_penetration=0.001,
            name="azimuth rotation remains seated",
        )

    return ctx.report()


object_model = build_object_model()
