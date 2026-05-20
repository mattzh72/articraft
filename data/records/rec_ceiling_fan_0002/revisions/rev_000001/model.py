from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_blade_mesh():
    blade_profile = [
        (-0.005, -0.050),
        (0.010, -0.074),
        (0.120, -0.076),
        (0.300, -0.068),
        (0.420, -0.055),
        (0.490, -0.038),
        (0.520, -0.016),
        (0.530, 0.000),
        (0.520, 0.016),
        (0.490, 0.038),
        (0.420, 0.055),
        (0.300, 0.068),
        (0.120, 0.076),
        (0.010, 0.074),
        (-0.005, 0.050),
    ]
    return ExtrudeGeometry(blade_profile, 0.008, center=True)


def _build_globe_shell_mesh():
    outer_profile = [
        (0.037, 0.000),
        (0.058, 0.014),
        (0.086, 0.048),
        (0.090, 0.086),
        (0.064, 0.118),
        (0.000, 0.134),
    ]
    inner_profile = [
        (0.033, 0.000),
        (0.054, 0.015),
        (0.082, 0.048),
        (0.086, 0.085),
        (0.060, 0.115),
        (0.000, 0.130),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_ceiling_fan", assets=ASSETS)

    satin_nickel = model.material("satin_nickel", rgba=(0.63, 0.66, 0.69, 1.0))
    charcoal_metal = model.material("charcoal_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    walnut = model.material("walnut", rgba=(0.48, 0.33, 0.20, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.93, 0.94, 0.96, 0.72))

    blade_mesh = _save_mesh("blade_panel.obj", _build_blade_mesh())
    globe_mesh = _save_mesh("globe_shell.obj", _build_globe_shell_mesh())
    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.075, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=satin_nickel,
        name="canopy",
    )
    mount.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=charcoal_metal,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        material=charcoal_metal,
        name="coupler",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=charcoal_metal,
        name="rotor_collar",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=charcoal_metal,
        name="motor_housing",
    )
    rotor.visual(
        Cylinder(radius=0.072, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=charcoal_metal,
        name="switch_housing",
    )
    rotor.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.174)),
        material=satin_nickel,
        name="light_fitter",
    )

    blade_mount_radius = 0.090
    blade_mount_z = -0.079
    blade_joint_radius = blade_mount_radius + 0.045
    for index in range(5):
        angle = index * (math.tau / 5.0)
        rotor.visual(
            Box((0.090, 0.030, 0.008)),
            origin=Origin(
                xyz=(
                    blade_mount_radius * math.cos(angle),
                    blade_mount_radius * math.sin(angle),
                    blade_mount_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_nickel,
            name=f"blade_arm_{index + 1}",
        )

    globe = model.part("globe")
    globe.visual(
        Cylinder(radius=0.037, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=satin_nickel,
        name="globe_neck",
    )
    globe.visual(
        globe_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.014), rpy=(math.pi, 0.0, 0.0)),
        material=frosted_glass,
        name="globe_shell",
    )

    for index in range(5):
        blade = model.part(f"blade_{index + 1}")
        blade.visual(
            Box((0.084, 0.032, 0.008)),
            origin=Origin(xyz=(0.042, 0.0, 0.0)),
            material=satin_nickel,
            name="blade_iron",
        )
        blade.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=walnut,
            name="blade_panel",
        )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=8.0),
    )
    model.articulation(
        "rotor_to_globe",
        ArticulationType.FIXED,
        parent=rotor,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, -0.182)),
    )

    for index in range(5):
        angle = index * (math.tau / 5.0)
        model.articulation(
            f"rotor_to_blade_{index + 1}",
            ArticulationType.FIXED,
            parent=rotor,
            child=f"blade_{index + 1}",
            origin=Origin(
                xyz=(
                    blade_joint_radius * math.cos(angle),
                    blade_joint_radius * math.sin(angle),
                    blade_mount_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    mount = object_model.get_part("mount")
    rotor = object_model.get_part("rotor")
    globe = object_model.get_part("globe")
    fan_spin = object_model.get_articulation("fan_spin")

    coupler = mount.get_visual("coupler")
    downrod = mount.get_visual("downrod")
    rotor_collar = rotor.get_visual("rotor_collar")
    motor_housing = rotor.get_visual("motor_housing")
    light_fitter = rotor.get_visual("light_fitter")
    globe_neck = globe.get_visual("globe_neck")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "five_blades_exist",
        all(object_model.get_part(f"blade_{index + 1}") is not None for index in range(5)),
        "Expected five blade parts.",
    )
    ctx.expect_origin_distance(rotor, mount, axes="xy", max_dist=0.001, name="rotor_centered")
    ctx.expect_gap(
        mount,
        rotor,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=coupler,
        negative_elem=rotor_collar,
        name="coupler_to_rotor_seated",
    )
    ctx.expect_gap(
        rotor,
        globe,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=light_fitter,
        negative_elem=globe_neck,
        name="light_fitter_to_globe_seated",
    )
    ctx.expect_overlap(globe, rotor, axes="xy", min_overlap=0.070, name="globe_centered_under_motor")

    motor_aabb = ctx.part_element_world_aabb(rotor, elem=motor_housing)
    if motor_aabb is not None:
        motor_dx = motor_aabb[1][0] - motor_aabb[0][0]
        motor_dy = motor_aabb[1][1] - motor_aabb[0][1]
        ctx.check(
            "motor_housing_plan_is_round",
            0.18 <= motor_dx <= 0.20 and abs(motor_dx - motor_dy) <= 0.005,
            f"Motor housing plan dims were {(motor_dx, motor_dy)}.",
        )

    blade_1 = object_model.get_part("blade_1")
    blade_1_aabb = ctx.part_world_aabb(blade_1)
    if blade_1_aabb is not None:
        blade_tip_x = blade_1_aabb[1][0]
        ctx.check(
            "fan_radius_realistic",
            0.64 <= blade_tip_x <= 0.68,
            f"Blade tip x was {blade_tip_x:.3f} m; expected about a 1.3 m overall fan span.",
        )

    for index in range(5):
        blade = object_model.get_part(f"blade_{index + 1}")
        blade_arm = rotor.get_visual(f"blade_arm_{index + 1}")
        blade_iron = blade.get_visual("blade_iron")
        ctx.expect_contact(
            blade,
            rotor,
            elem_a=blade_iron,
            elem_b=blade_arm,
            name=f"blade_{index + 1}_mounted_to_rotor",
        )
        ctx.expect_gap(
            blade,
            globe,
            axis="z",
            min_gap=0.090,
            name=f"blade_{index + 1}_clears_globe",
        )

    rest_position = ctx.part_world_position(blade_1)
    with ctx.pose({fan_spin: math.radians(72.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spun_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="spun_pose_no_floating")
        ctx.expect_gap(
            mount,
            rotor,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=coupler,
            negative_elem=rotor_collar,
            name="spun_pose_coupler_to_rotor_seated",
        )
        ctx.expect_gap(
            rotor,
            globe,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=light_fitter,
            negative_elem=globe_neck,
            name="spun_pose_globe_stays_seated",
        )
        ctx.expect_contact(
            blade_1,
            rotor,
            elem_a=blade_1.get_visual("blade_iron"),
            elem_b=rotor.get_visual("blade_arm_1"),
            name="spun_pose_blade_1_stays_mounted",
        )
        spun_position = ctx.part_world_position(blade_1)
        if rest_position is not None and spun_position is not None:
            rest_radius = math.hypot(rest_position[0], rest_position[1])
            spun_radius = math.hypot(spun_position[0], spun_position[1])
            moved_xy = math.hypot(
                spun_position[0] - rest_position[0],
                spun_position[1] - rest_position[1],
            )
            ctx.check(
                "blade_1_moves_when_fan_spins",
                moved_xy >= 0.14 and abs(rest_radius - spun_radius) <= 0.002,
                (
                    f"Blade root moved {moved_xy:.3f} m with radii "
                    f"{rest_radius:.3f} and {spun_radius:.3f}."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
