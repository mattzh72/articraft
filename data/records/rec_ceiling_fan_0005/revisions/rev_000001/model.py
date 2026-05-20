from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _blade_section(
    x_pos: float,
    width: float,
    thickness: float,
    *,
    sweep: float = 0.0,
    camber: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_thickness = thickness * 0.5
    return [
        (x_pos, sweep - 0.98 * half_width, camber + 0.00 * half_thickness),
        (x_pos, sweep - 0.66 * half_width, camber + 1.00 * half_thickness),
        (x_pos, sweep + 0.14 * half_width, camber + 0.92 * half_thickness),
        (x_pos, sweep + 0.98 * half_width, camber + 0.04 * half_thickness),
        (x_pos, sweep + 0.72 * half_width, camber - 0.84 * half_thickness),
        (x_pos, sweep + 0.12 * half_width, camber - 1.00 * half_thickness),
        (x_pos, sweep - 0.34 * half_width, camber - 0.82 * half_thickness),
        (x_pos, sweep - 0.92 * half_width, camber - 0.16 * half_thickness),
    ]


def _build_blade_mesh():
    return repair_loft(
        section_loft(
            [
                _blade_section(0.000, 0.060, 0.0052, sweep=0.000),
                _blade_section(0.180, 0.056, 0.0048, sweep=-0.002),
                _blade_section(0.380, 0.048, 0.0040, sweep=-0.007, camber=0.0003),
                _blade_section(0.540, 0.022, 0.0030, sweep=-0.013, camber=0.0008),
            ]
        )
    )


def _bracket_section(
    x_pos: float,
    width: float,
    thickness: float,
    *,
    z_base: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (x_pos, -half_width, z_base),
        (x_pos, -half_width, z_base + thickness),
        (x_pos, half_width, z_base + thickness),
        (x_pos, half_width, z_base),
    ]


def _build_blade_bracket_mesh():
    return repair_loft(
        section_loft(
            [
                _bracket_section(0.034, 0.016, 0.0060, z_base=0.0000),
                _bracket_section(0.062, 0.015, 0.0058, z_base=0.0001),
                _bracket_section(0.092, 0.013, 0.0054, z_base=0.0003),
            ]
        )
    )


def _build_light_dome_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.088, 0.000),
            (0.086, 0.007),
            (0.076, 0.018),
            (0.057, 0.029),
            (0.030, 0.037),
            (0.006, 0.041),
        ],
        [
            (0.079, 0.002),
            (0.078, 0.009),
            (0.068, 0.020),
            (0.051, 0.030),
            (0.027, 0.036),
            (0.002, 0.038),
        ],
        segments=64,
        start_cap="flat",
        end_cap="round",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_ceiling_fan", assets=ASSETS)

    housing_white = model.material("housing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_nickel = model.material("trim_nickel", rgba=(0.70, 0.72, 0.74, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.55, 0.53, 0.49, 1.0))
    blade_bracket = model.material("blade_bracket", rgba=(0.67, 0.68, 0.70, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.94, 0.96, 0.97, 0.72))

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.150, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=housing_white,
        name="ceiling_plate",
    )
    mount.visual(
        Cylinder(radius=0.172, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=housing_white,
        name="motor_disc",
    )
    mount.visual(
        Cylinder(radius=0.176, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=trim_nickel,
        name="lower_trim",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.176, length=0.060),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.094, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=trim_nickel,
        name="hub_plate",
    )
    rotor.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=trim_nickel,
        name="hub_cap",
    )
    rotor.visual(
        Cylinder(radius=0.101, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=housing_white,
        name="light_seat",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.101, length=0.024),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    light = model.part("light")
    light.visual(
        Cylinder(radius=0.094, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=trim_nickel,
        name="light_trim",
    )
    light.visual(
        _save_mesh(_build_light_dome_mesh(), "light_dome.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=frosted_glass,
        name="light_dome_shell",
    )
    light.inertial = Inertial.from_geometry(
        Cylinder(radius=0.094, length=0.048),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    blade_surface_mesh = _save_mesh(_build_blade_mesh(), "blade_surface.obj")
    blade_bracket_mesh = _save_mesh(_build_blade_bracket_mesh(), "blade_bracket.obj")
    blade_angles = (
        0.0,
        math.pi / 2.0,
        math.pi,
        3.0 * math.pi / 2.0,
    )
    for index, angle in enumerate(blade_angles):
        blade = model.part(f"blade_{index}")
        blade.visual(
            blade_bracket_mesh,
            origin=Origin(),
            material=blade_bracket,
            name="blade_bracket",
        )
        blade.visual(
            blade_surface_mesh,
            origin=Origin(xyz=(0.082, 0.0, 0.0045)),
            material=blade_finish,
            name="blade_surface",
        )
        blade.inertial = Inertial.from_geometry(
            Box((0.622, 0.060, 0.012)),
            mass=0.55,
            origin=Origin(xyz=(0.311, 0.0, 0.006)),
        )
        model.articulation(
            f"rotor_to_blade_{index}",
            ArticulationType.FIXED,
            parent=rotor,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, 0.008), rpy=(0.0, 0.0, angle)),
        )

    model.articulation(
        "mount_to_rotor",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=18.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "rotor_to_light",
        ArticulationType.FIXED,
        parent=rotor,
        child=light,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    mount = object_model.get_part("mount")
    rotor = object_model.get_part("rotor")
    light = object_model.get_part("light")
    blade_0 = object_model.get_part("blade_0")
    blade_1 = object_model.get_part("blade_1")
    blade_2 = object_model.get_part("blade_2")
    blade_3 = object_model.get_part("blade_3")
    fan_spin = object_model.get_articulation("mount_to_rotor")

    motor_disc = mount.get_visual("motor_disc")
    lower_trim = mount.get_visual("lower_trim")
    hub_plate = rotor.get_visual("hub_plate")
    light_seat = rotor.get_visual("light_seat")
    light_trim = light.get_visual("light_trim")
    light_dome_shell = light.get_visual("light_dome_shell")
    blade_parts = (blade_0, blade_1, blade_2, blade_3)
    blade_brackets = tuple(blade.get_visual("blade_bracket") for blade in blade_parts)
    blade_surfaces = tuple(blade.get_visual("blade_surface") for blade in blade_parts)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts(max_pose_samples=4)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "fan_spin_axis_vertical",
        fan_spin.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical spin axis, got {fan_spin.axis!r}",
    )

    with ctx.pose({fan_spin: 0.0}):
        ctx.expect_contact(rotor, mount, elem_a=hub_plate, elem_b=lower_trim)
        ctx.expect_contact(light, rotor, elem_a=light_trim, elem_b=light_seat)
        ctx.expect_within(
            light,
            mount,
            axes="xy",
            inner_elem=light_dome_shell,
            outer_elem=motor_disc,
            margin=0.0,
        )

        for blade, bracket, surface in zip(blade_parts, blade_brackets, blade_surfaces):
            ctx.expect_contact(
                blade,
                rotor,
                elem_a=bracket,
                elem_b=hub_plate,
                name=f"{blade.name}_mounted_to_rotor",
            )
            ctx.expect_gap(
                blade,
                mount,
                axis="z",
                min_gap=0.006,
                max_gap=0.016,
                positive_elem=surface,
                negative_elem=lower_trim,
                name=f"{blade.name}_below_motor_housing",
            )

        def aabb_center(aabb):
            return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

        blade_0_center = aabb_center(ctx.part_world_aabb(blade_0))
        blade_1_center = aabb_center(ctx.part_world_aabb(blade_1))
        blade_2_center = aabb_center(ctx.part_world_aabb(blade_2))
        blade_3_center = aabb_center(ctx.part_world_aabb(blade_3))

        ctx.check(
            "blade_0_points_positive_x",
            blade_0_center[0] > 0.22 and abs(blade_0_center[1]) < 0.05,
            details=f"blade_0 center={blade_0_center!r}",
        )
        ctx.check(
            "blade_1_points_positive_y",
            blade_1_center[1] > 0.22 and abs(blade_1_center[0]) < 0.05,
            details=f"blade_1 center={blade_1_center!r}",
        )
        ctx.check(
            "blade_2_points_negative_x",
            blade_2_center[0] < -0.22 and abs(blade_2_center[1]) < 0.05,
            details=f"blade_2 center={blade_2_center!r}",
        )
        ctx.check(
            "blade_3_points_negative_y",
            blade_3_center[1] < -0.22 and abs(blade_3_center[0]) < 0.05,
            details=f"blade_3 center={blade_3_center!r}",
        )

    with ctx.pose({fan_spin: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_world_aabb(blade_0)
        blade_0_quarter_center = tuple(
            (quarter_turn_aabb[0][axis] + quarter_turn_aabb[1][axis]) * 0.5 for axis in range(3)
        )
        ctx.check(
            "blade_0_rotates_with_hub",
            abs(blade_0_quarter_center[0]) < 0.05 and blade_0_quarter_center[1] > 0.22,
            details=f"blade_0 quarter-turn center={blade_0_quarter_center!r}",
        )
        ctx.expect_contact(blade_0, rotor, elem_a=blade_brackets[0], elem_b=hub_plate)
        ctx.fail_if_parts_overlap_in_current_pose(name="fan_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="fan_quarter_turn_no_floating")

    limits = fan_spin.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({fan_spin: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fan_spin_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="fan_spin_lower_no_floating")
        with ctx.pose({fan_spin: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="fan_spin_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="fan_spin_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
