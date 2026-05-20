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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mini_ceiling_fan")

    housing_white = model.material("housing_white", rgba=(0.96, 0.96, 0.97, 1.0))
    blade_oak = model.material("blade_oak", rgba=(0.75, 0.69, 0.57, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.66, 0.34, 1.0))

    canopy_radius = 0.072
    canopy_height = 0.018
    collar_radius = 0.048
    collar_height = 0.014
    motor_radius = 0.058
    motor_height = 0.050
    base_radius = 0.046
    base_height = 0.024
    housing_bottom_z = canopy_height + collar_height + motor_height + base_height

    housing = model.part("motor_housing")
    housing.visual(
        Cylinder(radius=canopy_radius, length=canopy_height),
        origin=Origin(xyz=(0.0, 0.0, canopy_height / 2.0)),
        material=housing_white,
        name="elem_ceiling_canopy",
    )
    housing.visual(
        Cylinder(radius=collar_radius, length=collar_height),
        origin=Origin(xyz=(0.0, 0.0, canopy_height + collar_height / 2.0)),
        material=housing_white,
        name="elem_motor_collar",
    )
    housing.visual(
        Cylinder(radius=motor_radius, length=motor_height),
        origin=Origin(
            xyz=(0.0, 0.0, canopy_height + collar_height + motor_height / 2.0)
        ),
        material=housing_white,
        name="elem_motor_shell",
    )
    housing.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                canopy_height + collar_height + motor_height + base_height / 2.0,
            )
        ),
        material=housing_white,
        name="elem_motor_base",
    )

    blade_length = 0.230
    blade_width = 0.092
    blade_thickness = 0.008
    blade_profile = rounded_rect_profile(
        blade_length,
        blade_width,
        radius=0.016,
        corner_segments=8,
    )
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(blade_profile, blade_thickness),
        "mini_fan_blade",
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=bracket_gray,
        name="elem_rotor_hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=bracket_gray,
        name="elem_rotor_plate",
    )
    blade_assembly.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bracket_gray,
        name="elem_rotor_spindle",
    )
    blade_assembly.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=bracket_gray,
        name="elem_rotor_cap",
    )

    blade_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(blade_angles):
        bracket_radius = 0.049
        blade_radius = 0.145
        blade_assembly.visual(
            Box((0.060, 0.022, 0.006)),
            origin=Origin(
                xyz=(
                    bracket_radius * math.cos(angle),
                    bracket_radius * math.sin(angle),
                    0.012,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=bracket_gray,
            name=f"elem_blade_bracket_{index}",
        )
        blade_assembly.visual(
            blade_mesh,
            origin=Origin(
                xyz=(
                    blade_radius * math.cos(angle),
                    blade_radius * math.sin(angle),
                    0.013,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=blade_oak,
            name=f"elem_blade_{index}",
        )

    pull_chain_pulley = model.part("pull_chain_pulley")
    pull_chain_pulley.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass,
        name="elem_pulley_wheel",
    )
    pull_chain_pulley.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=brass,
        name="elem_pulley_cap",
    )
    pull_chain_pulley.visual(
        Cylinder(radius=0.0012, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=brass,
        name="elem_chain_stem",
    )
    for index, bead_z in enumerate((0.018, 0.036, 0.054, 0.072)):
        pull_chain_pulley.visual(
            Sphere(radius=0.003),
            origin=Origin(xyz=(0.0, 0.0, bead_z)),
            material=brass,
            name=f"elem_chain_bead_{index}",
        )
    pull_chain_pulley.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=brass,
        name="elem_pull_weight",
    )

    model.articulation(
        "motor_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, housing_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "motor_to_pull_pulley",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=pull_chain_pulley,
        origin=Origin(xyz=(0.016, 0.028, housing_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("motor_housing")
    blade_assembly = object_model.get_part("blade_assembly")
    pull_chain_pulley = object_model.get_part("pull_chain_pulley")
    blade_joint = object_model.get_articulation("motor_to_blade_assembly")
    pull_joint = object_model.get_articulation("motor_to_pull_pulley")

    ctx.check(
        "fan has one housing and two mounted subassemblies",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=(
            f"parts={[part.name for part in object_model.parts]}, "
            f"articulations={[joint.name for joint in object_model.articulations]}"
        ),
    )
    ctx.check(
        "blade assembly spins continuously around the vertical axis",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(blade_joint.axis) == (0.0, 0.0, 1.0)
        and blade_joint.motion_limits is not None
        and blade_joint.motion_limits.lower is None
        and blade_joint.motion_limits.upper is None,
        details=(
            f"type={blade_joint.articulation_type}, axis={blade_joint.axis}, "
            f"limits={blade_joint.motion_limits}"
        ),
    )
    ctx.check(
        "pull pulley also uses continuous vertical rotation",
        pull_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(pull_joint.axis) == (0.0, 0.0, 1.0)
        and pull_joint.motion_limits is not None
        and pull_joint.motion_limits.lower is None
        and pull_joint.motion_limits.upper is None,
        details=(
            f"type={pull_joint.articulation_type}, axis={pull_joint.axis}, "
            f"limits={pull_joint.motion_limits}"
        ),
    )
    ctx.expect_gap(
        blade_assembly,
        housing,
        axis="z",
        positive_elem="elem_rotor_hub",
        negative_elem="elem_motor_base",
        min_gap=0.005,
        max_gap=0.007,
        name="blade hub stays just below the flush motor base",
    )
    ctx.expect_gap(
        pull_chain_pulley,
        housing,
        axis="z",
        positive_elem="elem_pulley_wheel",
        negative_elem="elem_motor_base",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="pull pulley seats against the motor base",
    )
    ctx.expect_gap(
        pull_chain_pulley,
        housing,
        axis="z",
        positive_elem="elem_pull_weight",
        negative_elem="elem_motor_base",
        min_gap=0.070,
        max_gap=0.110,
        name="pull chain hangs clearly below the housing",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    blade_aabb = ctx.part_world_aabb(blade_assembly)
    if housing_aabb is not None and blade_aabb is not None:
        housing_span_x = housing_aabb[1][0] - housing_aabb[0][0]
        housing_span_y = housing_aabb[1][1] - housing_aabb[0][1]
        blade_span_x = blade_aabb[1][0] - blade_aabb[0][0]
        blade_span_y = blade_aabb[1][1] - blade_aabb[0][1]
        ctx.check(
            "short wide blades still sweep much wider than the tiny motor housing",
            max(blade_span_x, blade_span_y) > max(housing_span_x, housing_span_y) * 2.6,
            details=(
                f"housing_span_x={housing_span_x}, housing_span_y={housing_span_y}, "
                f"blade_span_x={blade_span_x}, blade_span_y={blade_span_y}"
            ),
        )
    else:
        ctx.fail(
            "fan span AABBs are available",
            details=f"housing_aabb={housing_aabb}, blade_aabb={blade_aabb}",
        )

    with ctx.pose({blade_joint: 1.1, pull_joint: 2.3}):
        ctx.expect_gap(
            blade_assembly,
            housing,
            axis="z",
            positive_elem="elem_rotor_hub",
            negative_elem="elem_motor_base",
            min_gap=0.005,
            max_gap=0.007,
            name="spinning blade hub keeps the same flush clearance",
        )
        ctx.expect_gap(
            pull_chain_pulley,
            housing,
            axis="z",
            positive_elem="elem_pulley_wheel",
            negative_elem="elem_motor_base",
            max_gap=0.0005,
            max_penetration=1e-6,
            name="rotated pull pulley stays mounted under the motor base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
