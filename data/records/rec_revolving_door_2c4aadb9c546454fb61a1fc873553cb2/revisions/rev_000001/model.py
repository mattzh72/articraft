from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frameless_three_wing_revolving_door")

    glass = Material("pale_blue_clear_glass", rgba=(0.72, 0.92, 1.0, 0.32))
    edge_glass = Material("polished_glass_edge", rgba=(0.58, 0.86, 0.96, 0.55))
    steel = Material("brushed_stainless_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_threshold = Material("dark_recessed_threshold", rgba=(0.06, 0.055, 0.05, 1.0))

    storefront = model.part("storefront")
    storefront.visual(
        Cylinder(radius=1.25, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_threshold,
        name="floor_threshold",
    )
    storefront.visual(
        Box((0.020, 2.35, 2.42)),
        origin=Origin(xyz=(-1.20, 0.0, 1.27)),
        material=glass,
        name="fixed_glass_sidelight",
    )
    storefront.visual(
        Box((1.20, 0.08, 0.04)),
        origin=Origin(xyz=(-0.60, 0.0, 2.50)),
        material=steel,
        name="overhead_patch_arm",
    )
    storefront.visual(
        Cylinder(radius=0.075, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 2.50)),
        material=steel,
        name="top_bearing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.032, length=2.42),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=steel,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=steel,
        name="lower_hub_collar",
    )
    rotor.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        material=steel,
        name="upper_hub_collar",
    )

    wing_length = 0.95
    wing_thickness = 0.014
    wing_height = 2.30
    wing_center_x = 0.032 + wing_length / 2.0
    wing_center_z = 1.18
    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Box((wing_length, wing_thickness, wing_height)),
            origin=Origin(xyz=(wing_center_x, 0.0, wing_center_z), rpy=(0.0, 0.0, yaw)),
            material=glass,
            name=f"glass_wing_{index}",
        )
        rotor.visual(
            Box((0.025, 0.020, wing_height)),
            origin=Origin(xyz=(0.032 + wing_length, 0.0, wing_center_z), rpy=(0.0, 0.0, yaw)),
            material=edge_glass,
            name=f"polished_edge_{index}",
        )
        for height_name, z in (("lower", 0.58), ("upper", 1.88)):
            rotor.visual(
                Box((0.12, 0.046, 0.08)),
                origin=Origin(xyz=(0.070, 0.0, z), rpy=(0.0, 0.0, yaw)),
                material=steel,
                name=f"{height_name}_clamp_{index}",
            )

    model.articulation(
        "storefront_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=storefront,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    storefront = object_model.get_part("storefront")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("storefront_to_rotor")

    ctx.expect_contact(
        rotor,
        storefront,
        elem_a="central_post",
        elem_b="floor_threshold",
        name="steel post sits on the floor bearing",
    )
    ctx.expect_contact(
        rotor,
        storefront,
        elem_a="central_post",
        elem_b="top_bearing",
        name="steel post reaches the overhead bearing",
    )
    ctx.expect_overlap(
        rotor,
        storefront,
        axes="xy",
        elem_a="central_post",
        elem_b="top_bearing",
        min_overlap=0.05,
        name="post is centered in the bearing footprint",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="glass_wing_0")
    with ctx.pose({spin: math.pi / 3.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="glass_wing_0")

    ctx.check(
        "continuous joint rotates the glass wings",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(((rest_aabb[0][0] + rest_aabb[1][0]) * 0.5) - ((turned_aabb[0][0] + turned_aabb[1][0]) * 0.5)) > 0.05
        and abs(((rest_aabb[0][1] + rest_aabb[1][1]) * 0.5) - ((turned_aabb[0][1] + turned_aabb[1][1]) * 0.5)) > 0.05,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
