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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_ceiling_fan")

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    canopy_metal = model.material("canopy_metal", rgba=(0.41, 0.34, 0.25, 1.0))
    housing_metal = model.material("housing_metal", rgba=(0.30, 0.24, 0.18, 1.0))
    iron_metal = model.material("iron_metal", rgba=(0.16, 0.14, 0.12, 1.0))
    blade_wood = model.material("blade_wood", rgba=(0.47, 0.31, 0.19, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.52, 0.47, 0.39, 1.0))

    canopy_shell = save_mesh(
        "ceiling_fan_canopy_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.030, 0.000),
                (0.040, 0.010),
                (0.058, 0.028),
                (0.072, 0.054),
                (0.076, 0.072),
            ],
            [
                (0.022, 0.000),
                (0.028, 0.010),
                (0.046, 0.028),
                (0.062, 0.054),
                (0.068, 0.068),
            ],
            segments=64,
        ),
    )
    blade_profile = [
        (0.000, -0.060),
        (0.075, -0.061),
        (0.185, -0.058),
        (0.300, -0.051),
        (0.385, -0.040),
        (0.410, 0.000),
        (0.385, 0.040),
        (0.300, 0.051),
        (0.185, 0.058),
        (0.075, 0.061),
        (0.000, 0.060),
    ]
    blade_mesh = save_mesh(
        "ceiling_fan_blade_paddle",
        ExtrudeGeometry(blade_profile, 0.008, center=True),
    )
    canopy = model.part("canopy")
    canopy.visual(canopy_shell, material=canopy_metal, name="canopy_shell")
    canopy.visual(
        Cylinder(radius=0.070, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=canopy_metal,
        name="canopy_top_plate",
    )
    canopy.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=canopy_metal,
        name="hanger_socket",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.072),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    adapter = model.part("adapter")
    adapter.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=canopy_metal,
        name="adapter_collar",
    )
    adapter.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=spindle_metal,
        name="hanger_ball",
    )
    adapter.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=spindle_metal,
        name="downrod",
    )
    adapter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.090),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )
    model.articulation(
        "canopy_to_adapter",
        ArticulationType.FIXED,
        parent=canopy,
        child=adapter,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=canopy_metal,
        name="upper_neck",
    )
    motor_housing.visual(
        Cylinder(radius=0.128, length=0.142),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=housing_metal,
        name="drum_shell",
    )
    motor_housing.visual(
        Cylinder(radius=0.136, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=canopy_metal,
        name="lower_band",
    )
    motor_housing.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
        material=canopy_metal,
        name="switch_cap",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.136, length=0.186),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
    )
    model.articulation(
        "adapter_to_motor_housing",
        ArticulationType.FIXED,
        parent=adapter,
        child=motor_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.070, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=housing_metal,
        name="hub_plate",
    )
    hub.visual(
        Cylinder(radius=0.044, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=canopy_metal,
        name="hub_cap",
    )
    hub.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=spindle_metal,
        name="hub_nut",
    )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.072, length=0.060),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    model.articulation(
        "housing_to_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=hub,
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )

    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0

        blade_iron = model.part(f"blade_iron_{index}")
        blade_iron.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron_metal,
            name="pivot_barrel",
        )
        blade_iron.visual(
            Box((0.020, 0.012, 0.012)),
            origin=Origin(xyz=(0.010, 0.0, 0.0)),
            material=iron_metal,
            name="root_shank",
        )
        blade_iron.visual(
            Box((0.118, 0.014, 0.010)),
            origin=Origin(xyz=(0.074, 0.0, -0.004)),
            material=iron_metal,
            name="main_arm",
        )
        blade_iron.visual(
            Box((0.040, 0.070, 0.006)),
            origin=Origin(xyz=(0.144, 0.0, -0.010)),
            material=iron_metal,
            name="blade_plate",
        )
        blade_iron.visual(
            Box((0.040, 0.074, 0.008)),
            origin=Origin(xyz=(0.180, 0.0, -0.014)),
            material=blade_wood,
            name="blade_root",
        )
        blade_iron.visual(
            blade_mesh,
            origin=Origin(xyz=(0.180, 0.0, -0.014)),
            material=blade_wood,
            name="blade_paddle",
        )
        blade_iron.inertial = Inertial.from_geometry(
            Box((0.410, 0.122, 0.040)),
            mass=0.67,
            origin=Origin(xyz=(0.205, 0.0, -0.012)),
        )
        model.articulation(
            f"hub_to_blade_iron_{index}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=blade_iron,
            origin=Origin(
                xyz=(0.065 * math.cos(angle), 0.065 * math.sin(angle), 0.004),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=1.0,
                lower=-0.08,
                upper=0.14,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    canopy = object_model.get_part("canopy")
    adapter = object_model.get_part("adapter")
    motor_housing = object_model.get_part("motor_housing")
    hub = object_model.get_part("hub")
    hub_spin = object_model.get_articulation("housing_to_hub_spin")

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
    ctx.allow_overlap(
        adapter,
        canopy,
        elem_a="hanger_ball",
        elem_b="hanger_socket",
        reason="The hanger ball is captured inside a simplified solid socket instead of a modeled hollow cavity.",
    )
    ctx.allow_overlap(
        adapter,
        motor_housing,
        elem_a="downrod",
        elem_b="upper_neck",
        reason="The downrod is seated into a simplified solid motor yoke neck rather than a bored receiver.",
    )
    for index in range(5):
        ctx.allow_overlap(
            hub,
            object_model.get_part(f"blade_iron_{index}"),
            reason="The blade-iron pivot knuckle is simplified as a concealed nested seat within the hub rim.",
        )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(adapter, canopy)
    ctx.expect_contact(motor_housing, adapter)
    ctx.expect_contact(hub, motor_housing)
    ctx.expect_overlap(adapter, canopy, axes="xy", min_overlap=0.030)
    ctx.expect_overlap(motor_housing, canopy, axes="xy", min_overlap=0.070)

    for index in range(5):
        blade_iron = object_model.get_part(f"blade_iron_{index}")
        pitch_joint = object_model.get_articulation(f"hub_to_blade_iron_{index}")

        ctx.expect_contact(blade_iron, hub, name=f"blade_iron_{index}_mounted_to_hub")
        ctx.expect_origin_distance(
            blade_iron,
            hub,
            axes="xy",
            min_dist=0.064,
            max_dist=0.066,
            name=f"blade_iron_{index}_pivot_radius",
        )

        limits = pitch_joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None

        with ctx.pose({pitch_joint: limits.lower}):
            lower_aabb = ctx.part_world_aabb(blade_iron)
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pitch_joint.name}_lower_clear")
            ctx.fail_if_isolated_parts(name=f"{pitch_joint.name}_lower_connected")
        with ctx.pose({pitch_joint: limits.upper}):
            upper_aabb = ctx.part_world_aabb(blade_iron)
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pitch_joint.name}_upper_clear")
            ctx.fail_if_isolated_parts(name=f"{pitch_joint.name}_upper_connected")

        assert lower_aabb is not None
        assert upper_aabb is not None
        ctx.check(
            f"{pitch_joint.name}_articulates",
            upper_aabb[1][2] < lower_aabb[1][2] - 0.02,
            details=(
                f"Expected blade {index} tip envelope to move downward with positive pitch; "
                f"lower max z={lower_aabb[1][2]:.4f}, upper max z={upper_aabb[1][2]:.4f}"
            ),
        )

    blade_0 = object_model.get_part("blade_iron_0")
    blade_0_rest = ctx.part_world_position(blade_0)
    assert blade_0_rest is not None
    with ctx.pose({hub_spin: math.pi / 2.0}):
        blade_0_quarter = ctx.part_world_position(blade_0)
        ctx.fail_if_parts_overlap_in_current_pose(name="hub_spin_quarter_turn_clear")
        ctx.fail_if_isolated_parts(name="hub_spin_quarter_turn_connected")
    assert blade_0_quarter is not None
    ctx.check(
        "hub_spin_rotates_blade_pattern",
        blade_0_quarter[1] > blade_0_rest[0] - 0.03 and abs(blade_0_quarter[0]) < 0.03,
        details=(
            "Expected blade_0 root to move from the +X side to the +Y side under a quarter-turn "
            f"spin; rest={blade_0_rest}, quarter={blade_0_quarter}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
