from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _merge_meshes(meshes: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 64,
) -> MeshGeometry:
    """Closed annular cylinder centered on local Z."""
    mesh = MeshGeometry()
    z0 = -0.5 * length
    z1 = 0.5 * length
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []
    for i in range(segments):
        angle = math.tau * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(mesh.add_vertex(outer_radius * c, outer_radius * s, z0))
        outer_top.append(mesh.add_vertex(outer_radius * c, outer_radius * s, z1))
        inner_bottom.append(mesh.add_vertex(inner_radius * c, inner_radius * s, z0))
        inner_top.append(mesh.add_vertex(inner_radius * c, inner_radius * s, z1))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        mesh.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        mesh.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner wall, reversed so normals face the bore.
        mesh.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        mesh.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        # Top and bottom annular caps.
        mesh.add_face(outer_top[i], outer_top[j], inner_top[j])
        mesh.add_face(outer_top[i], inner_top[j], inner_top[i])
        mesh.add_face(outer_bottom[j], outer_bottom[i], inner_bottom[i])
        mesh.add_face(outer_bottom[j], inner_bottom[i], inner_bottom[j])
    return mesh


def _superellipse_section(
    x: float,
    width: float,
    height: float,
    *,
    center_z: float,
    exponent: float = 2.7,
    samples: int = 40,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = math.tau * i / samples
        c = math.cos(t)
        s = math.sin(t)
        y = 0.5 * width * math.copysign(abs(c) ** (2.0 / exponent), c)
        z = center_z + 0.5 * height * math.copysign(abs(s) ** (2.0 / exponent), s)
        points.append((x, y, z))
    return points


def _build_tower_mesh() -> MeshGeometry:
    # A subtly tapered, large utility-scale steel tower with a widened top flange.
    return LatheGeometry(
        [
            (0.0, 0.50),
            (2.20, 0.50),
            (2.05, 5.0),
            (1.55, 34.0),
            (1.10, 68.0),
            (0.82, 82.8),
            (1.05, 84.2),
            (0.0, 84.2),
        ],
        segments=96,
    )


def _build_nacelle_shell_mesh() -> MeshGeometry:
    sections = [
        _superellipse_section(-5.4, 2.45, 2.55, center_z=2.55),
        _superellipse_section(-4.6, 3.55, 3.25, center_z=2.55),
        _superellipse_section(-2.2, 4.35, 3.95, center_z=2.55),
        _superellipse_section(2.2, 4.15, 3.90, center_z=2.55),
        _superellipse_section(4.8, 3.15, 3.25, center_z=2.55),
        _superellipse_section(5.65, 1.65, 2.10, center_z=2.55),
    ]
    return repair_loft(section_loft(sections))


def _build_hub_mesh() -> MeshGeometry:
    # Lathed around local Z, then rotated so its rotary axis is local +X.
    hub = LatheGeometry(
        [
            (0.0, -0.46),
            (0.70, -0.46),
            (1.02, -0.30),
            (1.25, 0.06),
            (1.17, 0.40),
            (0.86, 0.82),
            (0.36, 1.22),
            (0.0, 1.34),
        ],
        segments=80,
    )
    hub.rotate_y(math.pi / 2.0)
    rear_flange = _ring_mesh(outer_radius=1.34, inner_radius=0.46, length=0.16, segments=72)
    rear_flange.rotate_y(math.pi / 2.0).translate(-0.48, 0.0, 0.0)
    nose_band = _ring_mesh(outer_radius=0.97, inner_radius=0.84, length=0.12, segments=56)
    nose_band.rotate_y(math.pi / 2.0).translate(0.82, 0.0, 0.0)
    return _merge_meshes([hub, rear_flange, nose_band])


def _airfoil_loop(
    span: float,
    chord: float,
    thickness: float,
    *,
    twist_deg: float,
    sweep_y: float = 0.0,
    cone_x: float = 0.0,
) -> list[tuple[float, float, float]]:
    # Airfoil lives in local X/Y at a fixed span. X is axial thickness, Y is chord.
    raw = [
        (0.00, -0.50),
        (0.46, -0.36),
        (0.62, -0.12),
        (0.50, 0.20),
        (0.16, 0.47),
        (-0.10, 0.50),
        (-0.48, 0.24),
        (-0.58, -0.08),
        (-0.40, -0.36),
    ]
    twist = math.radians(twist_deg)
    ct = math.cos(twist)
    st = math.sin(twist)
    points: list[tuple[float, float, float]] = []
    for nx, ny in raw:
        x0 = nx * thickness
        y0 = ny * chord
        x = cone_x + x0 * ct - y0 * st
        y = sweep_y + x0 * st + y0 * ct
        points.append((x, y, span))
    return points


def _build_blade_airfoil_mesh() -> MeshGeometry:
    sections = [
        _airfoil_loop(2.55, 4.50, 0.68, twist_deg=18.0, sweep_y=0.00, cone_x=0.00),
        _airfoil_loop(6.50, 4.10, 0.54, twist_deg=13.0, sweep_y=-0.12, cone_x=0.05),
        _airfoil_loop(15.0, 3.15, 0.38, twist_deg=8.0, sweep_y=-0.42, cone_x=0.16),
        _airfoil_loop(27.0, 2.05, 0.24, twist_deg=4.5, sweep_y=-0.74, cone_x=0.31),
        _airfoil_loop(40.0, 0.88, 0.12, twist_deg=1.5, sweep_y=-0.92, cone_x=0.50),
    ]
    return repair_loft(section_loft(sections))


def _orient_about_rotor(mesh: MeshGeometry, angle: float) -> MeshGeometry:
    return mesh.rotate_x(-angle)


def _build_pitch_socket_mesh(angle: float) -> MeshGeometry:
    bearing_sleeve = CylinderGeometry(radius=0.62, height=1.35, radial_segments=48).translate(
        0.0, 0.0, 1.78
    )
    outer_lip = _ring_mesh(outer_radius=0.78, inner_radius=0.43, length=0.30, segments=48).translate(
        0.0, 0.0, 2.34
    )
    inner_lip = _ring_mesh(outer_radius=0.70, inner_radius=0.40, length=0.24, segments=48).translate(
        0.0, 0.0, 1.21
    )
    return _orient_about_rotor(_merge_meshes([bearing_sleeve, outer_lip, inner_lip]), angle)


def _build_blade_root_mesh(angle: float) -> MeshGeometry:
    spindle = CylinderGeometry(radius=0.41, height=1.78, radial_segments=48).translate(
        0.0, 0.0, 2.36
    )
    root_flange = CylinderGeometry(radius=0.56, height=0.28, radial_segments=56).translate(
        0.0, 0.0, 1.63
    )
    cuff = LatheGeometry(
        [
            (0.0, 2.28),
            (0.46, 2.28),
            (0.78, 2.60),
            (0.92, 3.10),
            (0.72, 3.64),
            (0.38, 3.92),
            (0.0, 3.92),
        ],
        segments=48,
    )
    return _orient_about_rotor(_merge_meshes([spindle, root_flange, cuff]), angle)


def _build_blade_shell_mesh(angle: float) -> MeshGeometry:
    return _orient_about_rotor(_build_blade_airfoil_mesh(), angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wind_turbine")

    matte_white = model.material("matte_warm_white", rgba=(0.86, 0.87, 0.84, 1.0))
    satin_white = model.material("satin_shell_white", rgba=(0.93, 0.94, 0.91, 1.0))
    satin_metal = model.material("satin_burnished_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_seal = model.material("dark_seam_seal", rgba=(0.10, 0.12, 0.13, 1.0))
    concrete = model.material("pale_concrete", rgba=(0.60, 0.58, 0.54, 1.0))
    blade_white = model.material("blade_satin_white", rgba=(0.95, 0.95, 0.92, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=4.35, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=concrete,
        name="foundation_disc",
    )
    tower.visual(
        Cylinder(radius=2.85, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=concrete,
        name="base_plinth",
    )
    tower.visual(
        mesh_from_geometry(_build_tower_mesh(), "tapered_tower"),
        material=matte_white,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(radius=1.18, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 84.04)),
        material=satin_metal,
        name="tower_yaw_flange",
    )
    tower.visual(
        Box((0.07, 0.86, 1.85)),
        origin=Origin(xyz=(2.08, 0.0, 2.05), rpy=(0.0, 0.0, 0.0)),
        material=dark_seal,
        name="tower_access_seam",
    )
    for index in range(16):
        angle = math.tau * index / 16
        tower.visual(
            Cylinder(radius=0.085, length=0.075),
            origin=Origin(
                xyz=(2.18 * math.cos(angle), 2.18 * math.sin(angle), 1.325),
            ),
            material=satin_metal,
            name=f"anchor_bolt_{index}",
        )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_build_nacelle_shell_mesh(), "nacelle_shell"),
        material=satin_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        mesh_from_geometry(
            _ring_mesh(outer_radius=1.46, inner_radius=0.74, length=0.40, segments=72),
            "yaw_bearing_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=satin_metal,
        name="yaw_bearing_ring",
    )
    nacelle.visual(
        Cylinder(radius=1.18, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=satin_white,
        name="yaw_fairing_pedestal",
    )
    front_bearing = _ring_mesh(outer_radius=1.18, inner_radius=0.72, length=0.70, segments=72)
    front_bearing.rotate_y(math.pi / 2.0)
    nacelle.visual(
        mesh_from_geometry(front_bearing, "front_bearing_collar"),
        origin=Origin(xyz=(5.98, 0.0, 2.55)),
        material=satin_metal,
        name="front_bearing_collar",
    )
    nacelle.visual(
        Box((0.10, 4.10, 0.08)),
        origin=Origin(xyz=(-0.8, 0.0, 4.58)),
        material=dark_seal,
        name="roof_long_seam",
    )
    nacelle.visual(
        Box((0.08, 0.08, 2.16)),
        origin=Origin(xyz=(2.72, 2.04, 2.48)),
        material=dark_seal,
        name="side_panel_seam_0",
    )
    nacelle.visual(
        Box((0.08, 0.08, 2.16)),
        origin=Origin(xyz=(-2.86, -2.07, 2.48)),
        material=dark_seal,
        name="side_panel_seam_1",
    )
    nacelle.visual(
        Box((0.08, 2.02, 0.10)),
        origin=Origin(xyz=(-5.39, 0.0, 2.62)),
        material=dark_seal,
        name="rear_vent_header",
    )
    for row in range(5):
        nacelle.visual(
            Box((0.075, 1.72, 0.045)),
            origin=Origin(xyz=(-5.39, 0.0, 1.88 + row * 0.24)),
            material=dark_seal,
            name=f"rear_vent_slat_{row}",
        )
    for index, y in enumerate((-0.74, 0.74)):
        nacelle.visual(
            Box((0.55, 0.10, 0.18)),
            origin=Origin(xyz=(-1.4, y, 4.50)),
            material=satin_metal,
            name=f"roof_lift_lug_{index}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_build_hub_mesh(), "hub_shell"),
        material=satin_metal,
        name="hub_shell",
    )
    main_spindle = CylinderGeometry(radius=0.72, height=1.60, radial_segments=56)
    main_spindle.rotate_y(math.pi / 2.0).translate(-0.68, 0.0, 0.0)
    rotor.visual(
        mesh_from_geometry(main_spindle, "main_spindle"),
        material=dark_seal,
        name="main_spindle",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            mesh_from_geometry(_build_pitch_socket_mesh(angle), f"pitch_socket_{index}"),
            material=satin_metal,
            name=f"pitch_socket_{index}",
        )

    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(blade_angles):
        blade = model.part(f"blade_{index}")
        blade.visual(
            mesh_from_geometry(_build_blade_root_mesh(angle), f"root_spindle_{index}"),
            material=satin_metal,
            name=f"root_spindle_{index}",
        )
        blade.visual(
            mesh_from_geometry(_build_blade_shell_mesh(angle), f"blade_shell_{index}"),
            material=blade_white,
            name=f"blade_shell_{index}",
        )

    yaw = model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 84.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850000.0, velocity=0.08, lower=-math.pi, upper=math.pi),
    )
    yaw.meta["description"] = "Slow yaw bearing on the tower crown."

    spin = model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(7.15, 0.0, 2.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=480000.0, velocity=1.45),
    )
    spin.meta["description"] = "Main rotor spindle supported by the front bearing collar."

    rotor_part = model.get_part("rotor")
    for index, angle in enumerate(blade_angles):
        model.articulation(
            f"rotor_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=rotor_part,
            child=model.get_part(f"blade_{index}"),
            origin=Origin(),
            axis=(0.0, math.sin(angle), math.cos(angle)),
            motion_limits=MotionLimits(effort=70000.0, velocity=0.30, lower=-0.35, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.allow_overlap(
        nacelle,
        rotor,
        elem_a="front_bearing_collar",
        elem_b="main_spindle",
        reason="The main rotor spindle is intentionally seated through the bearing-collar proxy.",
    )
    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem="yaw_bearing_ring",
        negative_elem="tower_yaw_flange",
        name="yaw bearing is seated on tower flange",
    )
    ctx.expect_within(
        rotor,
        nacelle,
        axes="yz",
        inner_elem="main_spindle",
        outer_elem="front_bearing_collar",
        margin=0.04,
        name="main spindle is centered in bearing collar",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="x",
        elem_a="main_spindle",
        elem_b="front_bearing_collar",
        min_overlap=0.20,
        name="main spindle remains inserted through front bearing",
    )

    for index in range(3):
        blade = object_model.get_part(f"blade_{index}")
        ctx.allow_overlap(
            rotor,
            blade,
            elem_a=f"pitch_socket_{index}",
            elem_b=f"root_spindle_{index}",
            reason="The blade root spindle is intentionally captured inside the pitch-bearing sleeve.",
        )
        ctx.expect_overlap(
            rotor,
            blade,
            axes="xyz",
            elem_a=f"pitch_socket_{index}",
            elem_b=f"root_spindle_{index}",
            min_overlap=0.12,
            name=f"blade {index} root is retained in pitch socket",
        )

    blade_0 = object_model.get_part("blade_0")
    rest_aabb = ctx.part_world_aabb(blade_0)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(blade_0)
    rest_center_z = None if rest_aabb is None else 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
    spun_center_y = None if spun_aabb is None else 0.5 * (spun_aabb[0][1] + spun_aabb[1][1])
    ctx.check(
        "rotor spin moves blade through swept plane",
        rest_center_z is not None
        and spun_center_y is not None
        and rest_center_z > 15.0
        and spun_center_y < -15.0,
        details=f"rest_center_z={rest_center_z}, spun_center_y={spun_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
