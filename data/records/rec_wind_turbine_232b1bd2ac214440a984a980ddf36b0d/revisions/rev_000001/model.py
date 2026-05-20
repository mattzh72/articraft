from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_cylinder_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Tube/adapter ring centered on the local X axis."""
    geom = MeshGeometry()
    x0 = -length / 2.0
    x1 = length / 2.0

    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer0.append(geom.add_vertex(x0, outer_radius * ca, outer_radius * sa))
        outer1.append(geom.add_vertex(x1, outer_radius * ca, outer_radius * sa))
        inner0.append(geom.add_vertex(x0, inner_radius * ca, inner_radius * sa))
        inner1.append(geom.add_vertex(x1, inner_radius * ca, inner_radius * sa))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Inner wall, wound opposite so the tube has a real bore.
        geom.add_face(inner0[i], inner1[j], inner0[j])
        geom.add_face(inner0[i], inner1[i], inner1[j])
        # Rear and front annular faces.
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        geom.add_face(outer1[i], outer1[j], inner1[i])
        geom.add_face(outer1[j], inner1[j], inner1[i])
    return geom


def _tapered_tower_mesh(
    *,
    bottom_radius: float,
    top_radius: float,
    height: float,
    base_z: float,
    segments: int = 64,
) -> MeshGeometry:
    """A riveted legacy-style tapered tubular tower shell, closed for export."""
    geom = MeshGeometry()
    bottom: list[int] = []
    top: list[int] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        bottom.append(geom.add_vertex(bottom_radius * ca, bottom_radius * sa, base_z))
        top.append(geom.add_vertex(top_radius * ca, top_radius * sa, base_z + height))

    bottom_center = geom.add_vertex(0.0, 0.0, base_z)
    top_center = geom.add_vertex(0.0, 0.0, base_z + height)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return geom


def _blade_mesh(theta: float) -> MeshGeometry:
    """Three-section tapered blade loft with pitch twist around its radial axis."""
    geom = MeshGeometry()
    # Radial basis in the rotor YZ plane; +X is the shaft/wind axis.
    u = (0.0, math.cos(theta), math.sin(theta))
    tangent = (0.0, -math.sin(theta), math.cos(theta))
    x_axis = (1.0, 0.0, 0.0)

    def add_vec(a, b):
        return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

    def scale(v, s):
        return (v[0] * s, v[1] * s, v[2] * s)

    sections = [
        # radius, chord, thickness, tangential sweep, axial sweep, pitch degrees
        (0.92, 0.68, 0.135, 0.00, 0.02, 18.0),
        (3.05, 0.40, 0.085, 0.18, -0.02, 10.0),
        (5.85, 0.17, 0.045, 0.34, -0.08, 4.0),
    ]
    # A modest airfoil-like loop in local chord/thickness coordinates.
    profile = [
        (-0.50, 0.00),
        (-0.18, 0.50),
        (0.24, 0.34),
        (0.50, 0.00),
        (0.24, -0.34),
        (-0.18, -0.50),
    ]

    rings: list[list[int]] = []
    for radius, chord, thickness, sweep, axial, pitch_deg in sections:
        pitch = math.radians(pitch_deg)
        chord_axis = add_vec(scale(tangent, math.cos(pitch)), scale(x_axis, -math.sin(pitch)))
        thick_axis = add_vec(scale(tangent, math.sin(pitch)), scale(x_axis, math.cos(pitch)))
        center = add_vec(scale(u, radius), add_vec(scale(tangent, sweep), scale(x_axis, axial)))
        ring: list[int] = []
        for chord_unit, thick_unit in profile:
            p = add_vec(
                center,
                add_vec(
                    scale(chord_axis, chord_unit * chord),
                    scale(thick_axis, thick_unit * thickness),
                ),
            )
            ring.append(geom.add_vertex(*p))
        rings.append(ring)

    n = len(profile)
    for r in range(len(rings) - 1):
        a_ring = rings[r]
        b_ring = rings[r + 1]
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(a_ring[i], a_ring[j], b_ring[j])
            geom.add_face(a_ring[i], b_ring[j], b_ring[i])

    # Root and tip caps.
    root_center = geom.add_vertex(*scale(u, sections[0][0] - 0.02))
    tip_center = geom.add_vertex(*add_vec(scale(u, sections[-1][0]), scale(tangent, sections[-1][3])))
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(root_center, rings[0][j], rings[0][i])
        geom.add_face(tip_center, rings[-1][i], rings[-1][j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_wind_turbine")

    aged_paint = model.material("aged_ivory_paint", rgba=(0.78, 0.76, 0.66, 1.0))
    weathered_steel = model.material("weathered_galvanized_steel", rgba=(0.42, 0.45, 0.43, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    red_oxide = model.material("red_oxide_adapter", rgba=(0.55, 0.16, 0.10, 1.0))
    concrete = model.material("aged_concrete", rgba=(0.50, 0.50, 0.46, 1.0))
    hatch_blue = model.material("faded_blue_hatch", rgba=(0.18, 0.30, 0.38, 1.0))
    blade_paint = model.material("matte_blade_composite", rgba=(0.86, 0.84, 0.75, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((5.2, 5.2, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=concrete,
        name="foundation_block",
    )
    tower.visual(
        Cylinder(radius=1.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=dark_steel,
        name="base_plate",
    )
    tower.visual(
        mesh_from_geometry(
            _tapered_tower_mesh(bottom_radius=0.70, top_radius=0.36, height=16.95, base_z=0.55),
            "tapered_tower_shell",
        ),
        material=aged_paint,
        name="tapered_shell",
    )
    tower.visual(
        Cylinder(radius=0.95, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        material=weathered_steel,
        name="base_flange",
    )
    tower.visual(
        Cylinder(radius=0.56, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 8.75)),
        material=weathered_steel,
        name="lap_joint_band",
    )
    tower.visual(
        Cylinder(radius=0.85, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 17.58)),
        material=weathered_steel,
        name="top_flange",
    )

    # Legacy external service door: a practical retrofit hatch, hinge rod, and handle.
    tower.visual(
        Box((0.08, 0.58, 1.05)),
        origin=Origin(xyz=(0.66, 0.0, 2.25)),
        material=hatch_blue,
        name="tower_service_hatch",
    )
    tower.visual(
        Cylinder(radius=0.035, length=1.08),
        origin=Origin(xyz=(0.705, -0.32, 2.25)),
        material=dark_steel,
        name="hatch_hinge_bar",
    )
    tower.visual(
        Box((0.055, 0.16, 0.08)),
        origin=Origin(xyz=(0.72, 0.18, 2.20)),
        material=dark_steel,
        name="hatch_handle",
    )

    # Four welded gusset plates at the base; simple but credible load paths.
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = 0.63 * math.cos(angle)
        y = 0.63 * math.sin(angle)
        tower.visual(
            Box((0.12, 0.44, 0.90)),
            origin=Origin(xyz=(x, y, 0.93), rpy=(0.0, 0.0, angle)),
            material=red_oxide,
            name=f"base_gusset_{i}",
        )

    # Bolt heads on the bottom and top flanges, slightly seated into their plates.
    for i in range(16):
        a = 2.0 * math.pi * i / 16
        tower.visual(
            Cylinder(radius=0.055, length=0.055),
            origin=Origin(xyz=(0.78 * math.cos(a), 0.78 * math.sin(a), 0.758)),
            material=dark_steel,
            name=f"base_bolt_{i}",
        )
    for i in range(12):
        a = 2.0 * math.pi * i / 12
        tower.visual(
            Cylinder(radius=0.038, length=0.050),
            origin=Origin(xyz=(0.74 * math.cos(a), 0.74 * math.sin(a), 17.725)),
            material=dark_steel,
            name=f"top_bolt_{i}",
        )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.62, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=weathered_steel,
        name="yaw_turntable",
    )
    nacelle.visual(
        Box((2.60, 1.40, 0.16)),
        origin=Origin(xyz=(0.65, 0.0, 0.28)),
        material=dark_steel,
        name="machined_bedplate",
    )
    nacelle.visual(
        Box((2.85, 1.18, 0.90)),
        origin=Origin(xyz=(0.65, 0.0, 0.75)),
        material=aged_paint,
        name="riveted_housing",
    )
    nacelle.visual(
        Cylinder(radius=0.48, length=0.65),
        origin=Origin(xyz=(-0.90, 0.0, 0.76), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_paint,
        name="rear_generator_cover",
    )
    nacelle.visual(
        mesh_from_geometry(_annular_cylinder_x(0.55, 0.25, 0.22), "front_adapter_ring"),
        origin=Origin(xyz=(2.15, 0.0, 0.76)),
        material=red_oxide,
        name="front_adapter",
    )
    nacelle.visual(
        mesh_from_geometry(_annular_cylinder_x(0.35, 0.20, 0.65), "nose_bearing_ring"),
        origin=Origin(xyz=(2.43, 0.0, 0.76)),
        material=dark_steel,
        name="nose_bearing",
    )
    nacelle.visual(
        Box((0.92, 0.08, 0.48)),
        origin=Origin(xyz=(0.58, 0.63, 0.82)),
        material=hatch_blue,
        name="side_service_hatch",
    )
    nacelle.visual(
        Box((0.78, 0.62, 0.06)),
        origin=Origin(xyz=(0.35, 0.0, 1.23)),
        material=hatch_blue,
        name="top_service_hatch",
    )
    for i, y in enumerate((-0.47, 0.47)):
        nacelle.visual(
            Box((1.45, 0.10, 0.38)),
            origin=Origin(xyz=(1.18, y, 0.46)),
            material=red_oxide,
            name=f"bearing_web_{i}",
        )
    for i in range(10):
        a = 2.0 * math.pi * i / 10
        nacelle.visual(
            Cylinder(radius=0.032, length=0.060),
            origin=Origin(
                xyz=(2.28, 0.43 * math.cos(a), 0.76 + 0.43 * math.sin(a)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"adapter_bolt_{i}",
        )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 17.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=0.12, lower=-1.57, upper=1.57),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.42, length=0.62),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_steel,
        name="hub_shell",
    )
    rotor.visual(
        Sphere(radius=0.38),
        origin=Origin(xyz=(0.32, 0.0, 0.0)),
        material=aged_paint,
        name="spinner_cap",
    )
    rotor.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weathered_steel,
        name="rear_collar",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.86),
        origin=Origin(xyz=(-0.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )

    blade_angles = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    for i, theta in enumerate(blade_angles):
        u_y = math.cos(theta)
        u_z = math.sin(theta)
        roll = theta - math.pi / 2.0
        rotor.visual(
            Cylinder(radius=0.18, length=0.82),
            origin=Origin(xyz=(0.0, 0.62 * u_y, 0.62 * u_z), rpy=(roll, 0.0, 0.0)),
            material=weathered_steel,
            name=f"root_cuff_{i}",
        )
        rotor.visual(
            Cylinder(radius=0.22, length=0.14),
            origin=Origin(xyz=(0.0, 0.98 * u_y, 0.98 * u_z), rpy=(roll, 0.0, 0.0)),
            material=red_oxide,
            name=f"root_band_{i}",
        )
        rotor.visual(
            mesh_from_geometry(_blade_mesh(theta), f"tapered_blade_{i}"),
            material=blade_paint,
            name=f"blade_{i}",
        )
        tangent = (0.0, -math.sin(theta), math.cos(theta))
        for j, (sx, st) in enumerate(((-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0))):
            rotor.visual(
                Sphere(radius=0.045),
                origin=Origin(
                    xyz=(
                        0.12 * sx,
                        0.42 * u_y + 0.14 * st * tangent[1],
                        0.42 * u_z + 0.14 * st * tangent[2],
                    )
                ),
                material=dark_steel,
                name=f"root_bolt_{i}_{j}",
            )

    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(3.08, 0.0, 0.76)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.expect_contact(
        nacelle,
        tower,
        elem_a="yaw_turntable",
        elem_b="top_flange",
        contact_tol=0.002,
        name="yaw turntable is seated on the tower top flange",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="yz",
        inner_elem="shaft",
        outer_elem="nose_bearing",
        margin=0.002,
        name="rotor shaft stays centered inside the nose bearing envelope",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="x",
        elem_a="shaft",
        elem_b="nose_bearing",
        min_overlap=0.55,
        name="rotor shaft remains inserted through the bearing",
    )

    rest_blade = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_blade = ctx.part_element_world_aabb(rotor, elem="blade_0")
    ctx.check(
        "rotor spin rotates the blade plane about the shaft",
        rest_blade is not None
        and spun_blade is not None
        and rest_blade[1][2] > spun_blade[1][2] + 3.0
        and spun_blade[0][1] < rest_blade[0][1] - 3.0,
        details=f"rest_blade={rest_blade}, spun_blade={spun_blade}",
    )

    rest_body = ctx.part_element_world_aabb(nacelle, elem="riveted_housing")
    with ctx.pose({yaw: 0.50}):
        yawed_body = ctx.part_element_world_aabb(nacelle, elem="riveted_housing")
    ctx.check(
        "nacelle yaws on the tower bearing",
        rest_body is not None
        and yawed_body is not None
        and (yawed_body[0][1] + yawed_body[1][1]) / 2.0 > (rest_body[0][1] + rest_body[1][1]) / 2.0 + 0.20,
        details=f"rest_body={rest_body}, yawed_body={yawed_body}",
    )

    return ctx.report()


object_model = build_object_model()
