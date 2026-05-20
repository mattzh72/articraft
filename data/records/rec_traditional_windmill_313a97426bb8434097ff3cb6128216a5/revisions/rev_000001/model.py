from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


TOWER_HEIGHT = 3.20
YAW_ORIGIN_Z = 3.30
SHAFT_X = 0.82
SHAFT_Z = 0.38


def _octagonal_frustum(
    bottom_radius: float,
    top_radius: float,
    z_min: float,
    z_max: float,
    *,
    sides: int = 8,
) -> MeshGeometry:
    """Low-cost tapered tower shell: one molded/extruded octagonal frustum."""
    mesh = MeshGeometry()
    bottom = []
    top = []
    # Rotate by half a side so the front/back/side faces are flat for trim.
    offset = math.pi / sides
    for i in range(sides):
        angle = offset + i * math.tau / sides
        bx = bottom_radius * math.cos(angle)
        by = bottom_radius * math.sin(angle)
        tx = top_radius * math.cos(angle)
        ty = top_radius * math.sin(angle)
        bottom.append(mesh.add_vertex(bx, by, z_min))
        top.append(mesh.add_vertex(tx, ty, z_max))

    bottom_center = mesh.add_vertex(0.0, 0.0, z_min)
    top_center = mesh.add_vertex(0.0, 0.0, z_max)
    for i in range(sides):
        j = (i + 1) % sides
        mesh.add_face(bottom[i], bottom[j], top[j])
        mesh.add_face(bottom[i], top[j], top[i])
        mesh.add_face(bottom_center, bottom[i], bottom[j])
        mesh.add_face(top_center, top[j], top[i])
    return mesh


def _add_blade_lattice(part, material, angle: float) -> None:
    """Four repeated stamped/screwed blade lattices around the hub."""
    roll = angle
    # A single straight spar is cheapest to extrude and gives each lattice a
    # clear load path into the hub.
    part.visual(
        Box((0.036, 0.062, 0.92)),
        origin=Origin(xyz=(0.165, 0.0, 0.595), rpy=(roll, 0.0, 0.0)),
        material=material,
        name=f"blade_spar_{int(round(angle / (math.pi / 2))) % 4}",
    )
    # Side rails and repeated slats make the traditional sail lattice without
    # introducing separate parts.
    for side, y in enumerate((-0.18, 0.18)):
        part.visual(
            Box((0.026, 0.032, 0.70)),
            origin=Origin(xyz=(0.160, y, 0.665), rpy=(roll, 0.0, 0.0)),
            material=material,
            name=f"blade_side_{int(round(angle / (math.pi / 2))) % 4}_{side}",
        )
    for index, (z, width) in enumerate(((0.30, 0.28), (0.47, 0.36), (0.64, 0.43), (0.81, 0.50), (0.98, 0.55))):
        part.visual(
            Box((0.026, width, 0.036)),
            origin=Origin(xyz=(0.158, 0.0, z), rpy=(roll, 0.0, 0.0)),
            material=material,
            name=f"blade_slats_{int(round(angle / (math.pi / 2))) % 4}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_traditional_windmill")

    stone = model.material("warm_stone_composite", rgba=(0.74, 0.70, 0.62, 1.0))
    base_concrete = model.material("cast_concrete", rgba=(0.52, 0.50, 0.46, 1.0))
    roof = model.material("oxide_red_cap", rgba=(0.54, 0.12, 0.07, 1.0))
    wood = model.material("sealed_blade_laminate", rgba=(0.64, 0.46, 0.27, 1.0))
    dark = model.material("dark_bearing_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    galvanized = model.material("galvanized_fasteners", rgba=(0.58, 0.62, 0.64, 1.0))

    tower = model.part(
        "tower",
        meta={
            "manufacturing": "One tapered octagonal shell plus molded plinth and top bearing race.",
            "assembly_order": "Anchor plinth, place tower shell, drop cap race onto top race, insert rotor shaft.",
        },
    )
    tower.visual(
        Box((1.88, 1.88, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=base_concrete,
        name="plinth",
    )
    tower.visual(
        mesh_from_geometry(_octagonal_frustum(0.82, 0.48, 0.16, TOWER_HEIGHT), "tower_shell"),
        material=stone,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=0.33, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 3.245)),
        material=stone,
        name="top_boss",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=0.170, tube=0.032, radial_segments=18, tubular_segments=56), "tower_race"),
        origin=Origin(xyz=(0.0, 0.0, 3.286)),
        material=dark,
        name="tower_race",
    )
    for index, (x, y) in enumerate(((0.170, 0.0), (-0.170, 0.0), (0.0, 0.170), (0.0, -0.170))):
        tower.visual(
            Box((0.075, 0.075, 0.030)),
            origin=Origin(xyz=(x, y, 3.303)),
            material=dark,
            name=f"tower_thrust_pad_{index}",
        )
    tower.visual(
        Cylinder(radius=0.105, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 3.260)),
        material=galvanized,
        name="yaw_post",
    )
    # Surface-mounted door/window/bolt details are shallow, molded-in trim.
    tower.visual(
        Box((0.035, 0.34, 0.62)),
        origin=Origin(xyz=(0.675, 0.0, 0.62)),
        material=wood,
        name="service_door",
    )
    tower.visual(
        Box((0.050, 0.26, 0.22)),
        origin=Origin(xyz=(0.595, 0.0, 1.95)),
        material=dark,
        name="front_louver",
    )

    cap = model.part(
        "cap",
        meta={
            "manufacturing": "Symmetric side panels, flat deck, two stamped roof halves, and common bearing rings.",
            "interface": "Lower yaw race drops over the tower post and rests on the tower race.",
        },
    )
    cap.visual(
        mesh_from_geometry(TorusGeometry(radius=0.170, tube=0.032, radial_segments=18, tubular_segments=56), "cap_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark,
        name="cap_race",
    )
    for index, (x, y) in enumerate(((0.170, 0.0), (-0.170, 0.0), (0.0, 0.170), (0.0, -0.170))):
        cap.visual(
            Box((0.075, 0.075, 0.014)),
            origin=Origin(xyz=(x, y, 0.025)),
            material=dark,
            name=f"cap_thrust_pad_{index}",
        )
    cap.visual(
        Box((1.15, 0.76, 0.08)),
        origin=Origin(xyz=(0.295, 0.0, 0.120)),
        material=roof,
        name="cap_deck",
    )
    for side, y in enumerate((-0.390, 0.390)):
        cap.visual(
            Box((1.04, 0.070, 0.42)),
            origin=Origin(xyz=(0.300, y, 0.360)),
            material=roof,
            name=f"side_panel_{side}",
        )
        cap.visual(
            Cylinder(radius=0.028, length=0.080),
            origin=Origin(xyz=(-0.080, y, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"side_bolt_{side}_0",
        )
        cap.visual(
            Cylinder(radius=0.028, length=0.080),
            origin=Origin(xyz=(0.560, y, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"side_bolt_{side}_1",
        )
    cap.visual(
        Box((1.08, 0.52, 0.055)),
        origin=Origin(xyz=(0.300, -0.180, 0.635), rpy=(0.50, 0.0, 0.0)),
        material=roof,
        name="roof_panel_0",
    )
    cap.visual(
        Box((1.08, 0.52, 0.055)),
        origin=Origin(xyz=(0.300, 0.180, 0.635), rpy=(-0.50, 0.0, 0.0)),
        material=roof,
        name="roof_panel_1",
    )
    for index, x in enumerate((0.420, SHAFT_X)):
        cap.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.115, tube=0.028, radial_segments=18, tubular_segments=56).rotate_y(math.pi / 2.0),
                f"bearing_ring_{index}",
            ),
            origin=Origin(xyz=(x, 0.0, SHAFT_Z)),
            material=dark,
            name="rear_bearing" if index == 0 else "front_bearing",
        )
        cap.visual(
            Box((0.080, 0.090, 0.230)),
            origin=Origin(xyz=(x, 0.0, 0.220)),
            material=dark,
            name=f"bearing_pedestal_{index}",
        )
        cap.visual(
            Box((0.065, 0.040, 0.27)),
            origin=Origin(xyz=(x, -0.150, SHAFT_Z)),
            material=dark,
            name=f"bearing_cheek_{index}_0",
        )
        cap.visual(
            Box((0.065, 0.040, 0.27)),
            origin=Origin(xyz=(x, 0.150, SHAFT_Z)),
            material=dark,
            name=f"bearing_cheek_{index}_1",
        )

    rotor = model.part(
        "rotor",
        meta={
            "manufacturing": "One shaft/hub subassembly with four identical bolted lattice sail stampings.",
            "interface": "Shaft slides through rear bearing, front bearing, stop collar, then hub and blade plate bolt on.",
        },
    )
    rotor.visual(
        Cylinder(radius=0.045, length=0.600),
        origin=Origin(xyz=(-0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.035),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="front_collar",
    )
    rotor.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(-0.345, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="rear_collar",
    )
    rotor.visual(
        Cylinder(radius=0.155, length=0.120),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.115, length=0.045),
        origin=Origin(xyz=(0.260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="hub_cap",
    )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        _add_blade_lattice(rotor, wood, angle)

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.35, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=2.0, friction=0.3),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(SHAFT_X, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_rotor")

    ctx.expect_gap(
        cap,
        tower,
        axis="z",
        positive_elem="cap_race",
        negative_elem="tower_race",
        max_gap=0.003,
        max_penetration=0.0,
        name="cap yaw race rests on tower race",
    )
    for bearing_name in ("front_bearing", "rear_bearing"):
        ctx.expect_within(
            rotor,
            cap,
            axes="yz",
            inner_elem="shaft",
            outer_elem=bearing_name,
            margin=0.002,
            name=f"shaft is centered inside {bearing_name}",
        )
        ctx.expect_overlap(
            rotor,
            cap,
            axes="x",
            elem_a="shaft",
            elem_b=bearing_name,
            min_overlap=0.040,
            name=f"shaft passes through {bearing_name}",
        )

    ctx.check(
        "cap has limited yaw bearing",
        yaw is not None and yaw.motion_limits is not None and yaw.motion_limits.lower <= -3.0 and yaw.motion_limits.upper >= 3.0,
        details="Cap should yaw on the top bearing through nearly a full turn.",
    )
    ctx.check(
        "rotor has continuous shaft spin",
        spin is not None and str(spin.articulation_type).lower().endswith("continuous"),
        details="Rotor should be a continuous revolute stage around the shaft.",
    )

    rest = ctx.part_world_position(rotor)
    with ctx.pose({yaw: 0.50}):
        yawed = ctx.part_world_position(rotor)
    ctx.check(
        "yaw stage carries rotor around tower axis",
        rest is not None and yawed is not None and abs(yawed[1] - rest[1]) > 0.25,
        details=f"rest={rest}, yawed={yawed}",
    )

    return ctx.report()


object_model = build_object_model()
