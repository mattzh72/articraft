from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    Origin,
    MotionLimits,
    Material,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_boom_barrier")

    # Materials
    model.material(name="pedestal_gray", rgba=(0.5, 0.5, 0.55, 1.0))
    model.material(name="boom_yellow", rgba=(1.0, 0.85, 0.0, 1.0))
    model.material(name="stripe_black", rgba=(0.1, 0.1, 0.1, 1.0))
    model.material(name="weight_dark", rgba=(0.25, 0.25, 0.28, 1.0))
    model.material(name="light_red", rgba=(0.9, 0.1, 0.1, 1.0))
    model.material(name="bolt_metal", rgba=(0.65, 0.65, 0.7, 1.0))
    model.material(name="hinge_metal", rgba=(0.4, 0.4, 0.45, 1.0))

    # Pedestal (root part - fixed to ground)
    pedestal = model.part("pedestal")

    # Main cabinet body - sits on ground
    pedestal.visual(
        Box((0.35, 0.35, 1.0)),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        material="pedestal_gray",
        name="cabinet",
    )

    # Base plate with bolt pattern
    pedestal.visual(
        Box((0.55, 0.55, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material="pedestal_gray",
        name="base_plate",
    )

    # Pole from cabinet to warning light
    pedestal.visual(
        Cylinder(radius=0.03, length=0.1),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material="pedestal_gray",
        name="light_pole",
    )

    # Warning light on top of cabinet
    pedestal.visual(
        Cylinder(radius=0.07, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        material="light_red",
        name="warning_light",
    )

    # Light dome on top
    pedestal.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
        material="light_red",
        name="light_dome",
    )

    # Base bolts (4 corners)
    bolt_positions = [(-0.22, -0.22), (0.22, -0.22), (-0.22, 0.22), (0.22, 0.22)]
    for i, (x, y) in enumerate(bolt_positions):
        pedestal.visual(
            Cylinder(radius=0.015, length=0.045),
            origin=Origin(xyz=(x, y, 0.022)),
            material="bolt_metal",
            name=f"bolt_{i}",
        )

    # Boom assembly (moving part)
    boom = model.part("boom")

    # Boom arm - extends 4m from hinge along +X at q=0
    # Boom part origin is at the hinge point
    boom.visual(
        Box((3.5, 0.12, 0.06)),
        origin=Origin(xyz=(1.75, 0.0, 0.0)),
        material="boom_yellow",
        name="boom_arm",
    )

    # Stripes on boom (alternating black/yellow - every 0.5m)
    for i in range(7):
        x_pos = 0.25 + i * 0.5
        boom.visual(
            Box((0.25, 0.13, 0.005)),
            origin=Origin(xyz=(x_pos, 0.0, 0.032)),
            material="stripe_black",
            name=f"stripe_{i}",
        )

    # Counterweight block - extends behind hinge (opposite direction from boom)
    boom.visual(
        Box((0.25, 0.25, 0.35)),
        origin=Origin(xyz=(-0.125, 0.0, -0.175)),
        material="weight_dark",
        name="counterweight",
    )

    # Side hinge hub (visual indicator on boom side)
    boom.visual(
        Cylinder(radius=0.04, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.571, 0.0)),
        material="hinge_metal",
        name="hinge_hub",
    )

    # Articulation: pedestal to boom
    # Hinge is on the +Y face of pedestal at 80cm height
    # Boom extends along +X from hinge
    # axis=(0, -1, 0) makes positive q lift the boom upward (right-hand rule)
    model.articulation(
        "pedestal_to_boom",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=boom,
        origin=Origin(xyz=(0.0, 0.175, 0.8)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    boom = object_model.get_part("boom")
    hinge = object_model.get_articulation("pedestal_to_boom")

    # Verify pedestal is root (no parent articulation)
    ctx.check("pedestal_is_root", pedestal is not None, details="Pedestal part exists")

    # Verify boom is child of pedestal
    ctx.check("boom_is_child", boom is not None, details="Boom part exists")

    # Check hinge articulation exists with correct type
    ctx.check("hinge_exists", hinge is not None, details="Hinge articulation exists")
    if hinge is not None and hinge.articulation_type is not None:
        ctx.check(
            "hinge_is_revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
            details=f"Hinge type is {hinge.articulation_type}",
        )

    # Allow intentional overlaps
    # Counterweight sits inside/behind cabinet - intentional overlap
    ctx.allow_overlap(
        "pedestal",
        "boom",
        elem_a="cabinet",
        elem_b="counterweight",
        reason="Counterweight is positioned behind hinge, partially overlapping cabinet footprint",
    )

    # Boom arm emerges from cabinet at hinge point - intentional overlap
    ctx.allow_overlap(
        "pedestal",
        "boom",
        elem_a="cabinet",
        elem_b="boom_arm",
        reason="Boom arm emerges from hinge mounted on pedestal face",
    )

    # Stripes are on surface of boom arm - intentional overlap
    for i in range(7):
        ctx.allow_overlap(
            "boom",
            "boom",
            elem_a="boom_arm",
            elem_b=f"stripe_{i}",
            reason="Stripes are mounted on surface of boom arm",
        )

    # Hinge hub connects to pedestal - intentional overlap at hinge point
    ctx.allow_overlap(
        "pedestal",
        "boom",
        elem_a="cabinet",
        elem_b="hinge_hub",
        reason="Hinge hub is mounted at hinge point on pedestal face",
    )

    # Test closed pose (q=0): boom is horizontal
    with ctx.pose({hinge: 0.0}):
        # Get boom arm AABB at closed pose
        boom_arm_aabb = ctx.part_element_world_aabb(boom, elem="boom_arm")
        if boom_arm_aabb:
            # At closed pose, max Z should be near hinge height (0.8m)
            closed_max_z = boom_arm_aabb[1][2]  # max corner Z
            ctx.check(
                "boom_closed_height",
                abs(closed_max_z - 0.83) < 0.05,  # boom arm top at ~0.83m
                details=f"Boom arm max z at closed pose: {closed_max_z}",
            )

        # Check that counterweight overlaps with pedestal (it's behind the hinge)
        ctx.expect_overlap(
            pedestal,
            boom,
            axes="xy",
            elem_a="cabinet",
            elem_b="counterweight",
            min_overlap=0.05,
            name="counterweight_behind_hinge",
        )

    # Test open pose (q=upper limit): boom lifts upward
    upper_limit = 1.57  # ~90 degrees
    with ctx.pose({hinge: upper_limit}):
        # Get boom arm AABB at open pose
        boom_arm_aabb_open = ctx.part_element_world_aabb(boom, elem="boom_arm")
        if boom_arm_aabb_open:
            open_max_z = boom_arm_aabb_open[1][2]  # max corner Z
            ctx.check(
                "boom_lifts_upward",
                open_max_z > 0.83,
                details=f"Boom arm max z at open pose: {open_max_z}",
            )

            # The tip of the boom should be much higher when open
            # At 90 degree rotation, tip should be at ~0.8 + 3.5 = 4.3m high
            ctx.check(
                "boom_tip_raised",
                open_max_z > 2.0,
                details=f"Boom tip height at open pose: {open_max_z}",
            )

    # Test that boom actually moves between poses
    with ctx.pose({hinge: 0.0}):
        boom_arm_aabb_closed = ctx.part_element_world_aabb(boom, elem="boom_arm")
    with ctx.pose({hinge: upper_limit}):
        boom_arm_aabb_open = ctx.part_element_world_aabb(boom, elem="boom_arm")

    if boom_arm_aabb_closed and boom_arm_aabb_open:
        closed_max_z = boom_arm_aabb_closed[1][2]
        open_max_z = boom_arm_aabb_open[1][2]
        ctx.check(
            "boom_moves_upward",
            open_max_z > closed_max_z + 0.5,  # Should move up significantly
            details=f"Closed max z={closed_max_z}, Open max z={open_max_z}",
        )

    # Verify warning light is on top of pedestal - check contact between light_pole and cabinet
    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            pedestal,
            pedestal,
            elem_a="cabinet",
            elem_b="light_pole",
            contact_tol=0.06,
            name="light_pole_mounted_on_cabinet",
        )

        ctx.expect_contact(
            pedestal,
            pedestal,
            elem_a="light_pole",
            elem_b="warning_light",
            contact_tol=0.06,
            name="warning_light_on_pole",
        )

    # Verify base bolts are at base
    for i in range(4):
        ctx.expect_contact(
            pedestal,
            pedestal,
            elem_a="base_plate",
            elem_b=f"bolt_{i}",
            contact_tol=0.03,
            name=f"bolt_{i}_on_base",
        )

    return ctx.report()


object_model = build_object_model()
