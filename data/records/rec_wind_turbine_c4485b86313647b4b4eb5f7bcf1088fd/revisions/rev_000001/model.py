from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_shift, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_center) for y, z in rounded_rect_profile(width, height, radius)]


def _rotated_xy(points: list[tuple[float, float]], angle: float) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def _airfoil_loop(
    *,
    chord: float,
    thickness: float,
    z: float,
    center_x: float = 0.0,
    center_y: float = 0.0,
    twist: float = 0.0,
) -> list[tuple[float, float, float]]:
    profile = [
        (-0.48 * chord, 0.00 * thickness),
        (-0.18 * chord, 0.42 * thickness),
        (0.18 * chord, 0.54 * thickness),
        (0.50 * chord, 0.12 * thickness),
        (0.46 * chord, -0.08 * thickness),
        (0.12 * chord, -0.46 * thickness),
        (-0.20 * chord, -0.38 * thickness),
        (-0.50 * chord, -0.08 * thickness),
    ]
    rotated = _rotated_xy(profile, twist)
    return [(x + center_x, y + center_y, z) for x, y in rotated]


def _build_tower_shell_mesh():
    return section_loft(
        [
            _xy_section(0.056, 0.050, 0.016, 0.016),
            _xy_section(0.042, 0.038, 0.012, 0.160),
            _xy_section(0.028, 0.026, 0.009, 0.334),
        ]
    )


def _build_nacelle_shell_mesh():
    return section_loft(
        [
            _yz_section(0.040, 0.048, 0.010, 0.000, z_center=0.040),
            _yz_section(0.074, 0.082, 0.018, 0.052, z_center=0.046),
            _yz_section(0.062, 0.066, 0.015, 0.108, z_center=0.044),
            _yz_section(0.028, 0.030, 0.007, 0.158, z_center=0.040),
        ]
    )


def _build_blade_shell_mesh():
    return section_loft(
        [
            _airfoil_loop(chord=0.046, thickness=0.012, z=0.012, center_x=-0.004, twist=0.34),
            _airfoil_loop(chord=0.035, thickness=0.008, z=0.052, center_x=-0.007, twist=0.20),
            _airfoil_loop(chord=0.024, thickness=0.005, z=0.098, center_x=-0.011, twist=0.09),
            _airfoil_loop(chord=0.014, thickness=0.003, z=0.145, center_x=-0.015, twist=0.02),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wind_turbine")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    tower_white = model.material("tower_white", rgba=(0.90, 0.91, 0.92, 1.0))
    nacelle_white = model.material("nacelle_white", rgba=(0.95, 0.95, 0.96, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.44, 0.47, 0.50, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.66, 0.70, 1.0))
    blade_white = model.material("blade_white", rgba=(0.97, 0.97, 0.96, 1.0))

    tower_base = model.part("tower_base")
    tower_base.visual(
        _save_mesh(
            "desktop_turbine_base_pod",
            ExtrudeGeometry(rounded_rect_profile(0.180, 0.140, 0.022), 0.024).translate(0.0, 0.0, 0.012),
        ),
        material=base_dark,
        name="base_pod",
    )
    tower_base.visual(
        Box((0.052, 0.046, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=base_dark,
        name="tower_socket",
    )
    tower_base.visual(
        _save_mesh("desktop_turbine_tower_shell", _build_tower_shell_mesh()),
        material=tower_white,
        name="tower_shell",
    )
    tower_base.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
        material=steel,
        name="tower_bearing",
    )
    tower_base.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.360)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="yaw_ring",
    )
    nacelle.visual(
        _save_mesh("desktop_turbine_nacelle_shell", _build_nacelle_shell_mesh()),
        material=nacelle_white,
        name="shell",
    )
    nacelle.visual(
        Cylinder(radius=0.0175, length=0.022),
        origin=Origin(xyz=(0.143, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_bearing",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((0.168, 0.082, 0.084)),
        mass=0.6,
        origin=Origin(xyz=(0.084, 0.0, 0.034)),
    )

    rotor_hub = model.part("rotor_hub")
    rotor_hub.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_flange",
    )
    rotor_hub.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_grey,
        name="hub_body",
    )
    rotor_hub.visual(
        _save_mesh(
            "desktop_turbine_spinner",
            ConeGeometry(radius=0.028, height=0.040, radial_segments=36).rotate_y(math.pi / 2.0).translate(
                0.070,
                0.0,
                0.0,
            ),
        ),
        material=nacelle_white,
        name="spinner",
    )

    blade_root_radius = 0.043
    blade_rolls = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]
    blade_mesh = _save_mesh("desktop_turbine_blade_shell", _build_blade_shell_mesh())
    blade_parts = []

    for index, roll in enumerate(blade_rolls, start=1):
        radial_y = -math.sin(roll)
        radial_z = math.cos(roll)
        collar_radius = blade_root_radius - 0.010
        ring_radius = blade_root_radius - 0.003

        rotor_hub.visual(
            Cylinder(radius=0.0125, length=0.020),
            origin=Origin(
                xyz=(0.014, collar_radius * radial_y, collar_radius * radial_z),
                rpy=(roll, 0.0, 0.0),
            ),
            material=hub_grey,
            name=f"root_collar_{index}",
        )
        rotor_hub.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(
                xyz=(0.014, ring_radius * radial_y, ring_radius * radial_z),
                rpy=(roll, 0.0, 0.0),
            ),
            material=steel,
            name=f"pitch_ring_{index}",
        )

        blade = model.part(f"blade_{index}")
        blade.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=steel,
            name="root_sleeve",
        )
        blade.visual(
            blade_mesh,
            material=blade_white,
            name="blade_shell",
        )
        blade.inertial = Inertial.from_geometry(
            Box((0.050, 0.018, 0.148)),
            mass=0.08,
            origin=Origin(xyz=(-0.008, 0.0, 0.074)),
        )
        blade_parts.append((blade, roll, radial_y, radial_z))

    rotor_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.092),
        mass=0.28,
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    nacelle_yaw = model.articulation(
        "tower_to_nacelle_yaw",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-2.80,
            upper=2.80,
        ),
    )

    rotor_spin = model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor_hub,
        origin=Origin(xyz=(0.154, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=15.0),
    )

    for index, (blade, roll, radial_y, radial_z) in enumerate(blade_parts, start=1):
        model.articulation(
            f"hub_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=rotor_hub,
            child=blade,
            origin=Origin(
                xyz=(0.014, blade_root_radius * radial_y, blade_root_radius * radial_z),
                rpy=(roll, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=1.0,
                lower=-0.35,
                upper=1.45,
            ),
        )

    nacelle_yaw.meta["role"] = "stow_yaw"
    rotor_spin.meta["role"] = "rotor_spin"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    tower_base = object_model.get_part("tower_base")
    nacelle = object_model.get_part("nacelle")
    rotor_hub = object_model.get_part("rotor_hub")
    blade_1 = object_model.get_part("blade_1")
    blade_2 = object_model.get_part("blade_2")
    blade_3 = object_model.get_part("blade_3")

    yaw_joint = object_model.get_articulation("tower_to_nacelle_yaw")
    rotor_joint = object_model.get_articulation("nacelle_to_rotor")
    blade_1_joint = object_model.get_articulation("hub_to_blade_1")
    blade_2_joint = object_model.get_articulation("hub_to_blade_2")
    blade_3_joint = object_model.get_articulation("hub_to_blade_3")

    ctx.check(
        "yaw_axis_is_vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        f"expected vertical yaw axis, got {yaw_joint.axis}",
    )
    ctx.check(
        "rotor_axis_is_forward",
        rotor_joint.axis == (1.0, 0.0, 0.0),
        f"expected rotor axis along +X, got {rotor_joint.axis}",
    )
    ctx.check(
        "rotor_joint_is_continuous",
        rotor_joint.motion_limits is not None
        and rotor_joint.motion_limits.lower is None
        and rotor_joint.motion_limits.upper is None,
        "rotor should spin continuously without bounded limits",
    )

    for index, blade_joint in enumerate((blade_1_joint, blade_2_joint, blade_3_joint), start=1):
        limits = blade_joint.motion_limits
        ctx.check(
            f"blade_{index}_pitch_supports_feathering",
            limits is not None and limits.upper is not None and limits.upper >= 1.30,
            f"blade {index} needs a strong feather/stow pitch limit, got {limits}",
        )

    ctx.expect_contact(nacelle, tower_base, elem_a="yaw_ring", elem_b="tower_bearing", name="nacelle_mounts_to_tower")
    ctx.expect_contact(rotor_hub, nacelle, elem_a="rear_flange", elem_b="front_bearing", name="rotor_mounts_to_nacelle")
    ctx.expect_contact(blade_1, rotor_hub, elem_a="root_sleeve", elem_b="root_collar_1", name="blade_1_root_load_path")
    ctx.expect_contact(blade_2, rotor_hub, elem_a="root_sleeve", elem_b="root_collar_2", name="blade_2_root_load_path")
    ctx.expect_contact(blade_3, rotor_hub, elem_a="root_sleeve", elem_b="root_collar_3", name="blade_3_root_load_path")

    ctx.expect_gap(rotor_hub, tower_base, axis="x", min_gap=0.050, negative_elem="tower_shell", name="rotor_clears_tower_column")
    ctx.expect_gap(blade_1, tower_base, axis="x", min_gap=0.030, negative_elem="tower_shell", name="blade_1_clears_tower")
    ctx.expect_gap(blade_2, tower_base, axis="x", min_gap=0.030, negative_elem="tower_shell", name="blade_2_clears_tower")
    ctx.expect_gap(blade_3, tower_base, axis="x", min_gap=0.030, negative_elem="tower_shell", name="blade_3_clears_tower")
    ctx.expect_gap(blade_1, tower_base, axis="z", min_gap=0.170, negative_elem="base_pod", name="blade_1_clears_base")
    ctx.expect_gap(blade_2, tower_base, axis="z", min_gap=0.170, negative_elem="base_pod", name="blade_2_clears_base")
    ctx.expect_gap(blade_3, tower_base, axis="z", min_gap=0.170, negative_elem="base_pod", name="blade_3_clears_base")

    blade_aabb = ctx.part_element_world_aabb(blade_1, elem="blade_shell")
    feathered_aabb = None
    stow_pitch = 1.35
    if blade_1_joint.motion_limits is not None and blade_1_joint.motion_limits.upper is not None:
        stow_pitch = blade_1_joint.motion_limits.upper
    with ctx.pose({blade_1_joint: stow_pitch}):
        feathered_aabb = ctx.part_element_world_aabb(blade_1, elem="blade_shell")
    if blade_aabb is not None and feathered_aabb is not None:
        deployed_x_depth = blade_aabb[1][0] - blade_aabb[0][0]
        feathered_x_depth = feathered_aabb[1][0] - feathered_aabb[0][0]
        ctx.check(
            "blade_1_feathers_for_stowage",
            feathered_x_depth < deployed_x_depth * 0.60,
            f"expected full-stow blade pitch to shrink x-depth from {deployed_x_depth:.4f} to below 60%, got {feathered_x_depth:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
