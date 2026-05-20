from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _blade_outline(length: float, root_width: float, tip_width: float) -> list[tuple[float, float]]:
    tip_radius = tip_width * 0.5
    tip_center_x = length - tip_radius
    profile = [
        (0.0, -root_width * 0.5),
        (0.10, -root_width * 0.49),
        (0.26, -0.051),
        (0.44, -0.046),
        (tip_center_x, -tip_radius),
    ]
    for index in range(1, 9):
        angle = -math.pi * 0.5 + (index * math.pi / 8.0)
        profile.append(
            (
                tip_center_x + math.cos(angle) * tip_radius,
                math.sin(angle) * tip_radius,
            )
        )
    profile.extend(
        [
            (0.44, 0.046),
            (0.26, 0.051),
            (0.10, root_width * 0.49),
            (0.0, root_width * 0.5),
            (0.0, -root_width * 0.5),
        ]
    )
    return profile


def _build_canopy_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.014, 0.000),
            (0.050, 0.004),
            (0.078, 0.016),
            (0.074, 0.042),
            (0.052, 0.058),
            (0.028, 0.066),
            (0.018, 0.068),
        ],
        segments=64,
    )


def _build_motor_shell_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.033, 0.000),
            (0.052, 0.012),
            (0.088, 0.028),
            (0.126, 0.052),
            (0.146, 0.088),
            (0.142, 0.122),
            (0.114, 0.150),
            (0.080, 0.168),
            (0.050, 0.178),
        ],
        segments=72,
    )


def _build_rotor_hardware_mesh(blade_count: int) -> MeshGeometry:
    hardware = MeshGeometry()
    hardware.merge(CylinderGeometry(radius=0.028, height=0.034, radial_segments=36).translate(0.0, 0.0, 0.017))
    hardware.merge(CylinderGeometry(radius=0.102, height=0.018, radial_segments=48).translate(0.0, 0.0, 0.043))
    hardware.merge(CylinderGeometry(radius=0.062, height=0.014, radial_segments=40).translate(0.0, 0.0, 0.060))

    iron_pitch = -0.18
    for index in range(blade_count):
        angle = index * math.tau / blade_count
        inner_strap = BoxGeometry((0.152, 0.026, 0.008)).rotate_y(iron_pitch).translate(0.094, 0.0, 0.047)
        outer_plate = BoxGeometry((0.098, 0.036, 0.005)).rotate_y(iron_pitch).translate(0.188, 0.0, 0.060)
        reinforcement = BoxGeometry((0.034, 0.014, 0.022)).rotate_y(iron_pitch).translate(0.145, 0.0, 0.052)
        iron = MeshGeometry()
        iron.merge(inner_strap)
        iron.merge(outer_plate)
        iron.merge(reinforcement)
        iron.rotate_z(angle)
        hardware.merge(iron)

    return hardware


def _build_blade_mesh() -> MeshGeometry:
    blade = ExtrudeGeometry.from_z0(_blade_outline(length=0.67, root_width=0.115, tip_width=0.084), 0.008)
    blade.translate(0.0, 0.0, -0.004)
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windmill_ceiling_fan")

    dark_bronze = model.material("dark_bronze", rgba=(0.24, 0.20, 0.16, 1.0))
    aged_steel = model.material("aged_steel", rgba=(0.32, 0.33, 0.35, 1.0))
    vent_shadow = model.material("vent_shadow", rgba=(0.14, 0.13, 0.12, 1.0))
    oak_veneer = model.material("oak_veneer", rgba=(0.69, 0.54, 0.34, 1.0))

    support = model.part("support_assembly")
    support.visual(_save_mesh("canopy_shell", _build_canopy_mesh()), material=dark_bronze, name="canopy_shell")
    support.visual(
        Cylinder(radius=0.017, length=0.490),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=dark_bronze,
        name="downrod",
    )
    support.visual(
        Cylinder(radius=0.026, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=dark_bronze,
        name="upper_collar",
    )
    support.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.530)),
        material=dark_bronze,
        name="lower_coupler",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.60)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )

    motor = model.part("motor_housing")
    motor.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_bronze,
        name="upper_neck",
    )
    motor.visual(
        Cylinder(radius=0.063, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_bronze,
        name="upper_bell",
    )
    motor.visual(_save_mesh("motor_shell", _build_motor_shell_mesh()), material=dark_bronze, name="main_shell")
    motor.visual(
        Cylinder(radius=0.118, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=vent_shadow,
        name="vent_band",
    )
    motor.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=aged_steel,
        name="lower_bearing_cap",
    )
    motor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.180),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    rotor = model.part("rotor_assembly")
    rotor.visual(
        Cylinder(radius=0.028, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=aged_steel,
        name="rotor_hardware",
    )
    rotor.visual(
        Cylinder(radius=0.102, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=aged_steel,
        name="hub_plate",
    )
    rotor.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=aged_steel,
        name="hub_cap",
    )
    iron_pitch = -0.18
    for index in range(6):
        angle = index * math.tau / 6.0
        rotor.visual(
            Box((0.230, 0.024, 0.008)),
            origin=Origin(
                xyz=(0.136 * math.cos(angle), 0.136 * math.sin(angle), 0.054),
                rpy=(0.0, iron_pitch, angle),
            ),
            material=aged_steel,
            name=f"iron_{index:02d}",
        )

    base_blade = _build_blade_mesh()
    blade_pitch = -0.13
    for index in range(6):
        angle = index * math.tau / 6.0
        blade_geom = base_blade.copy()
        blade_geom.rotate_y(blade_pitch)
        blade_geom.translate(0.162, 0.0, 0.063)
        blade_geom.rotate_z(angle)
        rotor.visual(
            _save_mesh(f"blade_{index:02d}", blade_geom),
            material=oak_veneer,
            name=f"blade_{index:02d}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.82, length=0.090),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    model.articulation(
        "support_to_motor",
        ArticulationType.FIXED,
        parent=support,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
    )
    model.articulation(
        "motor_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=14.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_assembly")
    motor = object_model.get_part("motor_housing")
    rotor = object_model.get_part("rotor_assembly")
    spin = object_model.get_articulation("motor_to_rotor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    blade_visuals = [visual for visual in rotor.visuals if visual.name and visual.name.startswith("blade_")]
    ctx.check("six long blades are authored", len(blade_visuals) == 6, details=f"blade_count={len(blade_visuals)}")

    ctx.expect_contact(
        support,
        motor,
        elem_a="lower_coupler",
        elem_b="upper_neck",
        name="downrod coupler seats on the motor neck",
    )
    ctx.expect_contact(
        motor,
        rotor,
        elem_a="lower_bearing_cap",
        elem_b="rotor_hardware",
        name="rotor assembly meets the motor bearing plane",
    )
    ctx.expect_overlap(
        motor,
        rotor,
        axes="xy",
        elem_a="lower_bearing_cap",
        elem_b="rotor_hardware",
        min_overlap=0.050,
        name="rotor stays centered under the motor housing",
    )

    support_aabb = ctx.part_world_aabb(support)
    motor_aabb = ctx.part_world_aabb(motor)
    rotor_aabb = ctx.part_world_aabb(rotor)
    support_height = None if support_aabb is None else support_aabb[1][2] - support_aabb[0][2]
    motor_width = None if motor_aabb is None else motor_aabb[1][0] - motor_aabb[0][0]
    rotor_span = None if rotor_aabb is None else rotor_aabb[1][0] - rotor_aabb[0][0]
    ctx.check(
        "downrod reads as long suspended support",
        support_height is not None and 0.55 <= support_height <= 0.70,
        details=f"support_height={support_height}",
    )
    ctx.check(
        "motor housing reads wide and substantial",
        motor_width is not None and 0.26 <= motor_width <= 0.34,
        details=f"motor_width={motor_width}",
    )
    ctx.check(
        "blade sweep reads as full-size six-blade ceiling fan",
        rotor_span is not None and 1.45 <= rotor_span <= 1.70,
        details=f"rotor_span={rotor_span}",
    )

    rest_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="blade_00"))
    with ctx.pose({spin: math.pi / 3.0}):
        turned_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="blade_00"))
    moved_in_plane = None
    z_stable = None
    if rest_center is not None and turned_center is not None:
        moved_in_plane = math.hypot(turned_center[0] - rest_center[0], turned_center[1] - rest_center[1])
        z_stable = abs(turned_center[2] - rest_center[2])
    ctx.check(
        "continuous axle spins blade sweep in plane",
        moved_in_plane is not None and z_stable is not None and moved_in_plane > 0.45 and z_stable < 0.03,
        details=f"rest_center={rest_center}, turned_center={turned_center}, moved_in_plane={moved_in_plane}, z_stable={z_stable}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
