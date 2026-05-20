from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        min(radius, 0.49 * width, 0.49 * height),
        corner_segments=corner_segments,
    )
    return [(x_pos, y_pos, z_center + z_pos) for y_pos, z_pos in profile]


def _build_airframe_mesh():
    sections = [
        _yz_section(1.62, width=0.18, height=0.26, radius=0.06, z_center=1.33),
        _yz_section(1.34, width=0.54, height=0.72, radius=0.14, z_center=1.38),
        _yz_section(1.02, width=1.10, height=1.20, radius=0.20, z_center=1.38),
        _yz_section(0.40, width=1.88, height=1.60, radius=0.24, z_center=1.35),
        _yz_section(-0.35, width=1.95, height=1.64, radius=0.26, z_center=1.33),
        _yz_section(-0.95, width=1.72, height=1.34, radius=0.22, z_center=1.28),
        _yz_section(-1.55, width=1.08, height=0.84, radius=0.16, z_center=1.22),
        _yz_section(-2.10, width=0.66, height=0.52, radius=0.11, z_center=1.20),
        _yz_section(-3.00, width=0.42, height=0.34, radius=0.08, z_center=1.22),
        _yz_section(-3.90, width=0.28, height=0.24, radius=0.06, z_center=1.25),
        _yz_section(-4.55, width=0.14, height=0.14, radius=0.03, z_center=1.34),
    ]
    return repair_loft(section_loft(sections))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, *, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_nose_gear(part, *, metal, rubber) -> None:
    part.visual(
        Box((0.10, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=metal,
        name="mount_block",
    )
    _add_member(
        part,
        (0.0, 0.0, -0.04),
        (0.08, 0.0, -0.26),
        0.035,
        material=metal,
        name="oleo",
    )
    _add_member(
        part,
        (0.02, 0.0, -0.08),
        (0.12, 0.0, -0.39),
        0.018,
        material=metal,
        name="drag_brace",
    )
    _add_member(
        part,
        (0.08, 0.0, -0.26),
        (0.12, 0.05, -0.43),
        0.015,
        material=metal,
        name="fork_0",
    )
    _add_member(
        part,
        (0.08, 0.0, -0.26),
        (0.12, -0.05, -0.43),
        0.015,
        material=metal,
        name="fork_1",
    )
    part.visual(
        Box((0.05, 0.14, 0.05)),
        origin=Origin(xyz=(0.12, 0.0, -0.43)),
        material=metal,
        name="axle_block",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.16),
        origin=Origin(xyz=(0.12, 0.0, -0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.12, 0.0, -0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel",
    )


def _build_main_gear(part, *, side_sign: float, metal, rubber) -> None:
    part.visual(
        Box((0.14, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=metal,
        name="mount_block",
    )
    lower_joint = (0.05, side_sign * 0.02, -0.22)
    wheel_center = (0.16, side_sign * 0.05, -0.46)
    _add_member(
        part,
        (0.0, 0.0, -0.04),
        lower_joint,
        0.040,
        material=metal,
        name="strut",
    )
    _add_member(
        part,
        (-0.10, 0.0, -0.04),
        wheel_center,
        0.018,
        material=metal,
        name="brace",
    )
    _add_member(part, lower_joint, wheel_center, 0.018, material=metal, name="lower_leg")
    part.visual(
        Box((0.06, 0.18, 0.05)),
        origin=Origin(xyz=wheel_center),
        material=metal,
        name="axle_block",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.20),
        origin=Origin(xyz=wheel_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axle",
    )
    part.visual(
        Cylinder(radius=0.22, length=0.13),
        origin=Origin(xyz=wheel_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_helicopter")

    fuselage_paint = model.material("fuselage_paint", rgba=(0.40, 0.46, 0.36, 1.0))
    cabin_glass = model.material("cabin_glass", rgba=(0.34, 0.46, 0.56, 0.45))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.18, 0.19, 0.20, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        _save_mesh("airframe_shell", _build_airframe_mesh()),
        material=fuselage_paint,
        name="shell",
    )
    airframe.visual(
        Box((1.36, 0.94, 0.26)),
        origin=Origin(xyz=(-0.42, 0.0, 1.94)),
        material=fuselage_paint,
        name="engine_cowling",
    )
    airframe.visual(
        Box((0.72, 0.12, 1.18)),
        origin=Origin(xyz=(-4.02, 0.0, 1.85)),
        material=fuselage_paint,
        name="tail_fin",
    )
    airframe.visual(
        Box((0.54, 1.52, 0.08)),
        origin=Origin(xyz=(-3.52, 0.0, 1.16)),
        material=fuselage_paint,
        name="stabilizer",
    )
    airframe.visual(
        Box((0.76, 0.18, 0.56)),
        origin=Origin(xyz=(-3.62, 0.0, 1.46)),
        material=fuselage_paint,
        name="fin_fillet",
    )
    airframe.visual(
        Box((0.78, 0.86, 0.32)),
        origin=Origin(xyz=(1.08, 0.0, 0.98)),
        material=fuselage_paint,
        name="nose_keel",
    )
    airframe.visual(
        Box((1.04, 0.36, 0.24)),
        origin=Origin(xyz=(-1.02, 0.0, 0.68)),
        material=fuselage_paint,
        name="belly_fairing",
    )
    airframe.visual(
        Box((0.18, 0.16, 0.08)),
        origin=Origin(xyz=(0.78, 0.0, 0.64)),
        material=fuselage_paint,
        name="nose_gear_mount_block",
    )
    airframe.visual(
        Box((0.74, 0.34, 0.22)),
        origin=Origin(xyz=(-0.74, 0.98, 0.78)),
        material=fuselage_paint,
        name="port_sponson",
    )
    airframe.visual(
        Box((0.74, 0.34, 0.22)),
        origin=Origin(xyz=(-0.74, -0.98, 0.78)),
        material=fuselage_paint,
        name="starboard_sponson",
    )
    airframe.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(-0.08, 0.0, 2.10)),
        material=dark_metal,
        name="mast_fairing",
    )
    airframe.visual(
        Box((0.14, 0.10, 0.14)),
        origin=Origin(xyz=(-4.18, 0.08, 1.72)),
        material=fuselage_paint,
        name="tail_rotor_gearbox",
    )
    airframe.visual(
        Cylinder(radius=0.026, length=0.07),
        origin=Origin(xyz=(-4.18, 0.115, 1.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tail_rotor_quill",
    )
    airframe.visual(
        Box((0.34, 1.02, 0.56)),
        origin=Origin(xyz=(1.02, 0.0, 1.52), rpy=(0.0, -0.44, 0.0)),
        material=cabin_glass,
        name="windshield",
    )
    airframe.visual(
        Box((0.98, 0.06, 0.64)),
        origin=Origin(xyz=(0.66, 0.82, 1.42)),
        material=cabin_glass,
        name="port_cockpit_glass",
    )
    airframe.visual(
        Box((0.98, 0.06, 0.64)),
        origin=Origin(xyz=(0.66, -0.82, 1.42)),
        material=cabin_glass,
        name="starboard_cockpit_glass",
    )
    airframe.visual(
        Box((1.10, 0.06, 0.62)),
        origin=Origin(xyz=(-0.14, 0.95, 1.40)),
        material=cabin_glass,
        name="port_cabin_glass",
    )
    airframe.visual(
        Box((1.10, 0.06, 0.62)),
        origin=Origin(xyz=(-0.14, -0.95, 1.40)),
        material=cabin_glass,
        name="starboard_cabin_glass",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((6.4, 2.2, 2.3)),
        mass=2600.0,
        origin=Origin(xyz=(-1.10, 0.0, 1.18)),
    )

    nose_gear = model.part("nose_gear")
    _build_nose_gear(nose_gear, metal=steel, rubber=rubber)
    nose_gear.inertial = Inertial.from_geometry(
        Box((0.42, 0.24, 0.78)),
        mass=28.0,
        origin=Origin(xyz=(0.12, 0.0, -0.39)),
    )

    port_main_gear = model.part("port_main_gear")
    _build_main_gear(port_main_gear, side_sign=1.0, metal=steel, rubber=rubber)
    port_main_gear.inertial = Inertial.from_geometry(
        Box((0.40, 0.48, 0.82)),
        mass=42.0,
        origin=Origin(xyz=(0.05, 0.10, -0.38)),
    )

    starboard_main_gear = model.part("starboard_main_gear")
    _build_main_gear(starboard_main_gear, side_sign=-1.0, metal=steel, rubber=rubber)
    starboard_main_gear.inertial = Inertial.from_geometry(
        Box((0.40, 0.48, 0.82)),
        mass=42.0,
        origin=Origin(xyz=(0.05, -0.10, -0.38)),
    )

    port_door = model.part("port_baggage_door")
    port_door.visual(
        Box((0.68, 0.020, 0.70)),
        origin=Origin(xyz=(0.34, -0.010, 0.35)),
        material=fuselage_paint,
        name="panel",
    )
    port_door.inertial = Inertial.from_geometry(
        Box((0.68, 0.02, 0.70)),
        mass=18.0,
        origin=Origin(xyz=(0.34, -0.010, 0.35)),
    )

    starboard_door = model.part("starboard_baggage_door")
    starboard_door.visual(
        Box((0.68, 0.020, 0.70)),
        origin=Origin(xyz=(0.34, 0.010, 0.35)),
        material=fuselage_paint,
        name="panel",
    )
    starboard_door.inertial = Inertial.from_geometry(
        Box((0.68, 0.02, 0.70)),
        mass=18.0,
        origin=Origin(xyz=(0.34, 0.010, 0.35)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.042, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_metal,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=dark_metal,
        name="hub",
    )
    blade_length = 3.40
    blade_chord = 0.26
    blade_thickness = 0.045
    blade_center = 0.08 + blade_length * 0.5
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        main_rotor.visual(
            Box((blade_length, blade_chord, blade_thickness)),
            origin=Origin(
                xyz=(blade_center * math.cos(angle), blade_center * math.sin(angle), 0.22),
                rpy=(math.radians(2.5), 0.0, angle),
            ),
            material=rotor_gray,
            name=f"blade_{index}",
        )
        main_rotor.visual(
            Box((0.34, 0.18, 0.06)),
            origin=Origin(
                xyz=(0.17 * math.cos(angle), 0.17 * math.sin(angle), 0.22),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_metal,
            name=f"grip_{index}",
        )
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=4.3, length=0.32),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    tail_rotor.visual(
        Box((0.72, 0.028, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rotor_gray,
        name="blade_0",
    )
    tail_rotor.visual(
        Box((0.72, 0.028, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_gray,
        name="blade_1",
    )
    tail_rotor.visual(
        Box((0.12, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="spider",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.80, 0.14, 0.80)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "nose_gear_mount",
        ArticulationType.FIXED,
        parent=airframe,
        child=nose_gear,
        origin=Origin(xyz=(0.78, 0.0, 0.60)),
    )
    model.articulation(
        "port_main_gear_mount",
        ArticulationType.FIXED,
        parent=airframe,
        child=port_main_gear,
        origin=Origin(xyz=(-0.74, 1.12, 0.67)),
    )
    model.articulation(
        "starboard_main_gear_mount",
        ArticulationType.FIXED,
        parent=airframe,
        child=starboard_main_gear,
        origin=Origin(xyz=(-0.74, -1.12, 0.67)),
    )
    model.articulation(
        "port_baggage_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=port_door,
        origin=Origin(xyz=(-1.12, 1.00, 0.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "starboard_baggage_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=starboard_door,
        origin=Origin(xyz=(-1.12, -1.00, 0.92)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.08, 0.0, 2.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=35.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-4.18, 0.20, 1.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=55.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    port_door = object_model.get_part("port_baggage_door")
    starboard_door = object_model.get_part("starboard_baggage_door")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    port_hinge = object_model.get_articulation("port_baggage_hinge")
    starboard_hinge = object_model.get_articulation("starboard_baggage_hinge")
    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")

    ctx.expect_overlap(
        port_door,
        airframe,
        axes="xz",
        min_overlap=0.45,
        name="port baggage door covers aft side opening",
    )
    ctx.expect_overlap(
        starboard_door,
        airframe,
        axes="xz",
        min_overlap=0.45,
        name="starboard baggage door covers aft side opening",
    )
    ctx.expect_overlap(
        main_rotor,
        airframe,
        axes="xy",
        min_overlap=0.20,
        name="main rotor mast sits over the cabin roof",
    )
    ctx.expect_overlap(
        tail_rotor,
        airframe,
        axes="xz",
        min_overlap=0.10,
        name="tail rotor sits on the fin zone",
    )

    closed_port_aabb = ctx.part_world_aabb(port_door)
    closed_starboard_aabb = ctx.part_world_aabb(starboard_door)
    with ctx.pose({port_hinge: math.radians(105.0), starboard_hinge: math.radians(105.0)}):
        open_port_aabb = ctx.part_world_aabb(port_door)
        open_starboard_aabb = ctx.part_world_aabb(starboard_door)

    ctx.check(
        "port baggage door opens outward",
        closed_port_aabb is not None
        and open_port_aabb is not None
        and open_port_aabb[1][1] > closed_port_aabb[1][1] + 0.20,
        details=f"closed={closed_port_aabb}, open={open_port_aabb}",
    )
    ctx.check(
        "starboard baggage door opens outward",
        closed_starboard_aabb is not None
        and open_starboard_aabb is not None
        and open_starboard_aabb[0][1] < closed_starboard_aabb[0][1] - 0.20,
        details=f"closed={closed_starboard_aabb}, open={open_starboard_aabb}",
    )
    ctx.check(
        "rotor articulation axes match prompt",
        main_spin.axis == (0.0, 0.0, 1.0) and tail_spin.axis == (0.0, 1.0, 0.0),
        details=f"main={main_spin.axis}, tail={tail_spin.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
