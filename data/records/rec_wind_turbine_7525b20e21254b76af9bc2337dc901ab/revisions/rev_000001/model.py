from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


HUB_X = 1.284
HUB_Z = 0.55
PITCH_ROOT_RADIUS = 0.56


def _cylinder_x_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _cylinder_between_origin(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> Origin:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    horizontal = math.hypot(dx, dy)
    return Origin(
        xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
        rpy=(0.0, math.atan2(horizontal, dz), math.atan2(dy, dx)),
    )


def _distance(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> float:
    return math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2 + (p1[2] - p0[2]) ** 2)


def _radial_origin(angle: float, radial_center: float, *, roll_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=(0.0, radial_center * math.cos(angle), radial_center * math.sin(angle)),
        rpy=(angle - math.pi / 2.0 + roll_offset, 0.0, 0.0),
    )


def _rotated_box_origin(angle: float, radial_center: float, z_offset: float = 0.0) -> Origin:
    return Origin(
        xyz=(radial_center * math.cos(angle), radial_center * math.sin(angle), z_offset),
        rpy=(0.0, 0.0, angle),
    )


def _tower_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.50, 0.46),
            (0.47, 1.40),
            (0.42, 3.50),
            (0.36, 5.80),
            (0.31, 7.96),
        ],
        [
            (0.42, 0.46),
            (0.39, 1.40),
            (0.34, 3.50),
            (0.29, 5.80),
            (0.25, 7.96),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _spinner_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.66),
            (0.06, 0.63),
            (0.18, 0.54),
            (0.30, 0.36),
            (0.32, 0.24),
            (0.0, 0.22),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)


def _guard_ring_mesh() -> MeshGeometry:
    return TorusGeometry(radius=0.95, tube=0.035, radial_segments=18, tubular_segments=72).rotate_y(
        math.pi / 2.0
    )


def _bearing_ring_mesh() -> MeshGeometry:
    return TorusGeometry(radius=0.43, tube=0.055, radial_segments=18, tubular_segments=64).rotate_y(
        math.pi / 2.0
    )


def _airfoil_section(z: float, chord: float, thickness: float, camber: float) -> list[tuple[float, float, float]]:
    return [
        (-0.50 * thickness, -0.48 * chord + camber, z),
        (0.12 * thickness, -0.26 * chord + camber, z),
        (0.50 * thickness, 0.04 * chord + camber, z),
        (0.38 * thickness, 0.38 * chord + camber, z),
        (0.00 * thickness, 0.52 * chord + camber, z),
        (-0.42 * thickness, 0.30 * chord + camber, z),
        (-0.56 * thickness, -0.04 * chord + camber, z),
        (-0.28 * thickness, -0.36 * chord + camber, z),
    ]


def _blade_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _airfoil_section(0.30, 0.56, 0.110, -0.015),
                _airfoil_section(0.72, 0.50, 0.090, 0.020),
                _airfoil_section(1.30, 0.40, 0.070, 0.040),
                _airfoil_section(1.95, 0.30, 0.050, 0.030),
                _airfoil_section(2.58, 0.18, 0.030, 0.008),
            ]
        )
    )
    return blade


def _add_bolt_circle(
    part,
    *,
    radius: float,
    z: float,
    count: int,
    bolt_radius: float,
    length: float,
    material,
    name_prefix: str,
) -> None:
    for index in range(count):
        angle = math.tau * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=length),
            origin=Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), z)),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def _add_blade_root_bolts(part, *, material) -> None:
    for index in range(10):
        angle = math.tau * index / 10
        part.visual(
            Cylinder(radius=0.022, length=0.050),
            origin=Origin(
                xyz=(0.22 * math.cos(angle), 0.22 * math.sin(angle), 0.115),
            ),
            material=material,
            name=f"root_bolt_{index}",
        )


def _add_blade_part(model: ArticulatedObject, hub, index: int, angle: float, blade_material, metal, red):
    blade = model.part(f"blade_{index}")
    blade.visual(
        Cylinder(radius=0.34, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=metal,
        name="root_flange",
    )
    blade.visual(
        Cylinder(radius=0.25, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=metal,
        name="root_bearing",
    )
    blade.visual(
        mesh_from_geometry(_blade_mesh(), f"blade_{index}_airfoil"),
        material=blade_material,
        name="airfoil",
    )
    blade.visual(
        Box((0.050, 0.36, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=metal,
        name="root_doubler",
    )
    blade.visual(
        Box((0.070, 0.080, 0.080)),
        origin=Origin(xyz=(0.0, -0.30, 0.085)),
        material=red,
        name="pitch_stop_tab",
    )
    _add_blade_root_bolts(blade, material=metal)
    blade.inertial = Inertial.from_geometry(Box((0.12, 0.56, 2.70)), mass=95.0)

    model.articulation(
        f"hub_to_blade_{index}",
        ArticulationType.REVOLUTE,
        parent=hub,
        child=blade,
        origin=_radial_origin(angle, PITCH_ROOT_RADIUS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65000.0, velocity=0.25, lower=-0.18, upper=0.42),
        motion_properties=MotionProperties(damping=75.0, friction=25.0),
    )
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wind_turbine")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_structural_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.72, 0.74, 0.75, 1.0))
    blade_white = model.material("reinforced_blade_white", rgba=(0.86, 0.88, 0.84, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.82, 0.05, 0.03, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    tower = model.part("tower")
    tower.visual(Box((2.60, 2.60, 0.28)), origin=Origin(xyz=(0.0, 0.0, 0.14)), material=dark_steel, name="foundation")
    tower.visual(Cylinder(radius=0.86, length=0.18), origin=Origin(xyz=(0.0, 0.0, 0.37)), material=dark_steel, name="base_flange")
    tower.visual(mesh_from_geometry(_tower_shell_mesh(), "tapered_tower_shell"), material=galvanized, name="tower_shell")
    tower.visual(Cylinder(radius=0.56, length=0.16), origin=Origin(xyz=(0.0, 0.0, 8.04)), material=dark_steel, name="top_yaw_flange")
    tower.visual(Cylinder(radius=0.40, length=0.08), origin=Origin(xyz=(0.0, 0.0, 8.16)), material=black, name="yaw_seal")
    for index in range(8):
        angle = math.tau * index / 8
        tower.visual(
            Box((0.46, 0.045, 0.56)),
            origin=_rotated_box_origin(angle, 0.62, 0.66),
            material=dark_steel,
            name=f"base_gusset_{index}",
        )
    _add_bolt_circle(
        tower,
        radius=0.70,
        z=0.48,
        count=12,
        bolt_radius=0.055,
        length=0.070,
        material=galvanized,
        name_prefix="base_bolt",
    )
    for index, angle in enumerate((-0.75, 0.75)):
        tower.visual(
            Box((0.28, 0.16, 0.14)),
            origin=_rotated_box_origin(angle, 0.70, 8.18),
            material=lockout_red,
            name=f"yaw_stop_{index}",
        )
    tower.inertial = Inertial.from_geometry(Cylinder(radius=0.52, length=8.2), mass=6200.0, origin=Origin(xyz=(0.0, 0.0, 4.1)))

    nacelle = model.part("nacelle")
    nacelle.visual(Cylinder(radius=0.52, length=0.18), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=dark_steel, name="yaw_ring")
    nacelle.visual(Box((1.65, 0.92, 0.82)), origin=Origin(xyz=(-0.02, 0.0, 0.56)), material=nacelle_paint, name="gearbox_housing")
    nacelle.visual(Box((0.78, 0.70, 0.56)), origin=Origin(xyz=(0.48, 0.0, 0.56)), material=nacelle_paint, name="bearing_nose")
    nacelle.visual(mesh_from_geometry(_bearing_ring_mesh().translate(0.92, 0.0, HUB_Z), "main_bearing_ring"), material=dark_steel, name="main_bearing_ring")
    nacelle.visual(Box((0.42, 0.12, 0.18)), origin=Origin(xyz=(0.20, 0.46, 0.17)), material=lockout_red, name="yaw_stop_lug")
    nacelle.visual(Box((0.56, 0.065, 0.10)), origin=Origin(xyz=(0.55, 0.50, HUB_Z + 0.535)), material=safety_yellow, name="lock_upper_rail")
    nacelle.visual(Box((0.56, 0.065, 0.10)), origin=Origin(xyz=(0.55, 0.50, HUB_Z + 0.365)), material=safety_yellow, name="lock_lower_rail")
    nacelle.visual(Box((0.18, 0.10, 0.10)), origin=Origin(xyz=(0.58, 0.45, HUB_Z + 0.45)), material=safety_yellow, name="lock_mount_plate")
    nacelle.inertial = Inertial.from_geometry(Box((1.70, 0.95, 0.90)), mass=2100.0, origin=Origin(xyz=(0.0, 0.0, 0.56)))

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 8.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180000.0, velocity=0.08, lower=-2.10, upper=2.10),
        motion_properties=MotionProperties(damping=180.0, friction=70.0),
    )

    guard = model.part("root_guard")
    guard.visual(mesh_from_geometry(_guard_ring_mesh().translate(0.98, 0.0, HUB_Z), "root_guard_ring"), material=safety_yellow, name="guard_ring")
    for strut_name, y, z in (
        ("guard_strut_0", 0.495, 0.74),
        ("guard_strut_1", -0.495, 0.74),
        ("guard_strut_2", 0.495, 0.28),
        ("guard_strut_3", -0.495, 0.28),
    ):
        guard.visual(Box((0.38, 0.070, 0.10)), origin=Origin(xyz=(0.64, y, z)), material=safety_yellow, name=strut_name)
    for index, (p0, p1) in enumerate(
        (
            ((0.83, 0.54, 0.78), (0.98, 0.78, HUB_Z + 0.55)),
            ((0.83, -0.54, 0.78), (0.98, -0.78, HUB_Z + 0.55)),
            ((0.83, 0.53, 0.28), (0.98, 0.67, HUB_Z - 0.67)),
            ((0.83, -0.53, 0.28), (0.98, -0.67, HUB_Z - 0.67)),
        )
    ):
        guard.visual(
            Cylinder(radius=0.035, length=_distance(p0, p1)),
            origin=_cylinder_between_origin(p0, p1),
            material=safety_yellow,
            name=f"guard_brace_{index}",
        )
    guard.visual(Box((0.12, 1.42, 0.08)), origin=Origin(xyz=(0.98, 0.0, HUB_Z + 0.92)), material=safety_yellow, name="upper_guard_bar")
    guard.visual(Box((0.12, 1.42, 0.08)), origin=Origin(xyz=(0.98, 0.0, HUB_Z - 0.92)), material=safety_yellow, name="lower_guard_bar")
    guard.visual(Box((0.12, 0.08, 1.86)), origin=Origin(xyz=(0.965, 0.71, HUB_Z)), material=safety_yellow, name="guard_side_0")
    guard.visual(Box((0.12, 0.08, 1.86)), origin=Origin(xyz=(0.965, -0.71, HUB_Z)), material=safety_yellow, name="guard_side_1")
    guard.inertial = Inertial.from_geometry(Box((0.62, 2.0, 2.0)), mass=180.0, origin=Origin(xyz=(0.85, 0.0, HUB_Z)))
    model.articulation("nacelle_to_guard", ArticulationType.FIXED, parent=nacelle, child=guard, origin=Origin())

    hub = model.part("hub")
    hub.visual(Cylinder(radius=0.33, length=0.56), origin=_cylinder_x_origin(0.0, 0.0, 0.0), material=dark_steel, name="hub_barrel")
    hub.visual(Cylinder(radius=0.43, length=0.10), origin=_cylinder_x_origin(-0.26, 0.0, 0.0), material=dark_steel, name="rear_flange")
    hub.visual(Cylinder(radius=0.40, length=0.10), origin=_cylinder_x_origin(0.24, 0.0, 0.0), material=dark_steel, name="front_flange")
    hub.visual(mesh_from_geometry(_spinner_mesh(), "spinner"), material=galvanized, name="spinner")
    hub.visual(Box((0.10, 0.20, 0.20)), origin=Origin(xyz=(-0.18, 0.56, 0.45)), material=lockout_red, name="lock_receiver")
    hub.visual(Box((0.12, 0.26, 0.20)), origin=Origin(xyz=(-0.18, 0.43, 0.32)), material=dark_steel, name="lock_receiver_web")
    for index in range(3):
        angle = math.tau * index / 3
        hub.visual(
            Cylinder(radius=0.31, length=PITCH_ROOT_RADIUS),
            origin=_radial_origin(angle, PITCH_ROOT_RADIUS / 2.0),
            material=dark_steel,
            name=f"blade_socket_{index}",
        )
        hub.visual(
            Box((0.15, 0.08, 0.48)),
            origin=_radial_origin(angle, 0.30),
            material=dark_steel,
            name=f"root_gusset_{index}",
        )
        hub.visual(
            Box((0.11, 0.12, 0.11)),
            origin=_radial_origin(angle, 0.28, roll_offset=0.38),
            material=lockout_red,
            name=f"pitch_stop_{index}",
        )
    hub.inertial = Inertial.from_geometry(Cylinder(radius=0.55, length=0.95), mass=760.0)

    model.articulation(
        "nacelle_to_hub",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=hub,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=1.4),
        motion_properties=MotionProperties(damping=12.0, friction=4.0),
    )

    for index in range(3):
        _add_blade_part(model, hub, index, math.tau * index / 3, blade_white, galvanized, lockout_red)

    lock_pin = model.part("lock_pin")
    lock_pin.visual(Cylinder(radius=0.045, length=0.50), origin=_cylinder_x_origin(0.18, 0.0, 0.0), material=lockout_red, name="pin_bar")
    lock_pin.visual(Box((0.08, 0.12, 0.08)), origin=Origin(xyz=(-0.105, 0.0, 0.0)), material=lockout_red, name="pull_handle")
    lock_pin.visual(Cylinder(radius=0.055, length=0.055), origin=_cylinder_x_origin(-0.165, 0.0, 0.0), material=black, name="handle_grip")
    lock_pin.inertial = Inertial.from_geometry(Cylinder(radius=0.06, length=0.58), mass=16.0)
    model.articulation(
        "nacelle_to_lock_pin",
        ArticulationType.PRISMATIC,
        parent=nacelle,
        child=lock_pin,
        origin=Origin(xyz=(0.38, 0.56, HUB_Z + 0.45)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.12, lower=0.0, upper=0.32),
        motion_properties=MotionProperties(damping=40.0, friction=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    hub = object_model.get_part("hub")
    guard = object_model.get_part("root_guard")
    lock_pin = object_model.get_part("lock_pin")
    yaw = object_model.get_articulation("tower_to_nacelle")
    rotor = object_model.get_articulation("nacelle_to_hub")
    lock_slide = object_model.get_articulation("nacelle_to_lock_pin")

    for name in ("tower", "nacelle", "hub", "root_guard", "lock_pin", "blade_0", "blade_1", "blade_2"):
        ctx.check(f"{name}_present", object_model.get_part(name) is not None, f"Missing {name}")

    ctx.expect_contact(nacelle, tower, elem_a="yaw_ring", elem_b="yaw_seal", contact_tol=0.002, name="yaw bearing stack is seated on tower")
    ctx.expect_contact(guard, nacelle, elem_a="guard_strut_0", elem_b="gearbox_housing", contact_tol=0.010, name="guard frame is braced back to nacelle")

    for index in range(3):
        blade = object_model.get_part(f"blade_{index}")
        ctx.expect_contact(
            blade,
            hub,
            elem_a="root_flange",
            elem_b=f"blade_socket_{index}",
            contact_tol=0.004,
            name=f"blade_{index} root flange bears on hub socket",
        )
        pitch = object_model.get_articulation(f"hub_to_blade_{index}")
        limits = pitch.motion_limits if pitch is not None else None
        ctx.check(
            f"blade_{index}_pitch_limits",
            limits is not None and limits.lower <= -0.17 and limits.upper >= 0.40,
            details=f"limits={limits}",
        )

    yaw_limits = yaw.motion_limits if yaw is not None else None
    ctx.check(
        "yaw_has_limited_service_sweep",
        yaw_limits is not None and yaw_limits.lower < -2.0 and yaw_limits.upper > 2.0,
        details=f"limits={yaw_limits}",
    )
    ctx.check("rotor_is_continuous", rotor is not None and rotor.articulation_type == ArticulationType.CONTINUOUS, "Rotor hub should spin continuously.")

    ctx.expect_gap(
        hub,
        lock_pin,
        axis="x",
        min_gap=0.04,
        positive_elem="lock_receiver",
        negative_elem="pin_bar",
        name="lock pin is clear of receiver when retracted",
    )

    ctx.allow_overlap(
        lock_pin,
        nacelle,
        elem_a="pin_bar",
        elem_b="lock_upper_rail",
        reason="The lockout pin is intentionally captured with slight preload between the yellow guide rails.",
    )
    ctx.allow_overlap(
        lock_pin,
        nacelle,
        elem_a="pin_bar",
        elem_b="lock_lower_rail",
        reason="The lockout pin is intentionally captured with slight preload between the yellow guide rails.",
    )
    ctx.expect_gap(
        nacelle,
        lock_pin,
        axis="z",
        max_penetration=0.012,
        positive_elem="lock_upper_rail",
        negative_elem="pin_bar",
        name="upper guide rail lightly captures lock pin",
    )
    ctx.expect_gap(
        lock_pin,
        nacelle,
        axis="z",
        max_penetration=0.012,
        positive_elem="pin_bar",
        negative_elem="lock_lower_rail",
        name="lower guide rail lightly captures lock pin",
    )

    ctx.allow_overlap(
        lock_pin,
        hub,
        elem_a="pin_bar",
        elem_b="lock_receiver",
        reason="At full lockout travel the red pin intentionally enters the rotor lock receiver to immobilize the hub.",
    )
    with ctx.pose({lock_slide: 0.32}):
        ctx.expect_overlap(
            lock_pin,
            hub,
            axes="x",
            elem_a="pin_bar",
            elem_b="lock_receiver",
            min_overlap=0.07,
            name="lockout pin inserts into receiver at full stroke",
        )
        ctx.expect_within(
            lock_pin,
            hub,
            axes="yz",
            inner_elem="pin_bar",
            outer_elem="lock_receiver",
            margin=0.08,
            name="lockout pin aligns with receiver bore",
        )

    blade_0 = object_model.get_part("blade_0")
    rest_blade_origin = ctx.part_world_position(blade_0)
    with ctx.pose({rotor: math.pi / 2.0}):
        spun_blade_origin = ctx.part_world_position(blade_0)
    ctx.check(
        "rotor_spin_moves_blade_root",
        rest_blade_origin is not None
        and spun_blade_origin is not None
        and abs(rest_blade_origin[1] - spun_blade_origin[1]) > 0.30
        and abs(rest_blade_origin[2] - spun_blade_origin[2]) > 0.30,
        details=f"rest={rest_blade_origin}, spun={spun_blade_origin}",
    )

    rest_hub_origin = ctx.part_world_position(hub)
    with ctx.pose({yaw: 0.75}):
        yawed_hub_origin = ctx.part_world_position(hub)
    ctx.check(
        "yaw_sweep_moves_rotor_axis_around_tower",
        rest_hub_origin is not None
        and yawed_hub_origin is not None
        and yawed_hub_origin[1] > rest_hub_origin[1] + 0.40,
        details=f"rest={rest_hub_origin}, yawed={yawed_hub_origin}",
    )

    return ctx.report()


object_model = build_object_model()
