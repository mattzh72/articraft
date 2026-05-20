from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_pottery_wheel")

    # Root part: heavy circular base
    base = model.part("base")
    # Thick cylindrical base, 0.4m diameter, 0.1m tall, sits on ground (z=0)
    base.visual(
        Cylinder(radius=0.2, height=0.1),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Centered vertically: bottom at z=0, top at z=0.1
        name="base_shell",
        color=(0.2, 0.2, 0.2),  # Dark gray heavy metal look
    )
    # Splash pan rim: fixed shallow tray around base to catch clay
    # 0.5m diameter, 0.03m tall, mounted on top of base
    base.visual(
        Cylinder(radius=0.25, height=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.1 + 0.015)),  # Top of base + half splash pan height
        name="splash_pan_rim",
        color=(0.9, 0.9, 0.9),  # Light gray plastic
    )
    # Foot pedal: fixed flat pedal on ground next to base
    base.visual(
        Box((0.15, 0.1, 0.02)),  # 15cm x 10cm x 2cm
        origin=Origin(xyz=(0.3, 0.0, 0.01)),  # Adjacent to base, half height above ground
        name="foot_pedal",
        color=(0.1, 0.1, 0.1),  # Black rubber
    )
    # Connector between base and foot pedal to avoid disconnected geometry
    base.visual(
        Cylinder(radius=0.01, height=0.1),  # Runs from base edge (x=0.2) to pedal center (x=0.3)
        origin=Origin(xyz=(0.25, 0.0, 0.01)),  # Midpoint x=0.25, z=0.01 (half height)
        name="pedal_connector",
        color=(0.2, 0.2, 0.2),  # Match base color
    )

    # Rotating wheel head assembly
    wheel_head = model.part("wheel_head")
    # Central hub: connects wheel head to base bearing
    wheel_head.visual(
        Cylinder(radius=0.05, height=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),  # Centered in part frame: extends 0-0.15m up from joint
        name="wheel_hub",
        color=(0.7, 0.7, 0.7),  # Silver metal
    )
    # Rotating wheel plate: flat work surface for clay
    wheel_head.visual(
        Cylinder(radius=0.15, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.15 + 0.01)),  # Sits on top of hub + half plate height
        name="wheel_plate",
        color=(0.95, 0.95, 0.9),  # Off-white smooth finish
    )

    # Primary articulation: vertical revolute joint for wheel head rotation
    model.articulation(
        "base_to_wheel_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wheel_head,
        origin=Origin(xyz=(0.0, 0.0, 0.1)),  # Rotation axis at top center of base
        axis=(0.0, 0.0, 1.0),  # Vertical Z-axis rotation
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=15.0,
            lower=-6.283,  # -360 degrees
            upper=6.283,   # +360 degrees
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wheel_head = object_model.get_part("wheel_head")
    joint = object_model.get_articulation("base_to_wheel_head")

    # Allow intentional overlap between splash pan rim and wheel hub
    ctx.allow_overlap(
        "base", "wheel_head",
        elem_a="splash_pan_rim", elem_b="wheel_hub",
        reason="Wheel hub rotates inside splash pan rim; overlap is intentional as hub passes through rim footprint"
    )
    # Proof check for overlap allowance
    ctx.expect_overlap(
        "base", "wheel_head",
        elem_a="splash_pan_rim", elem_b="wheel_hub",
        axes="xy", min_overlap=0.05,
        name="wheel hub inside splash pan rim"
    )

    # 1. Main mechanism: wheel head rotates around vertical axis
    with ctx.pose({joint: 0.0}):
        ctx.expect_contact(wheel_head, base, name="wheel head contacts base at rest")
        ctx.expect_within(wheel_head, base, axes="xy", margin=0.001, name="wheel head centered over base at rest")

    with ctx.pose({joint: 1.5708}):  # 90 degrees rotation
        ctx.expect_within(wheel_head, base, axes="xy", margin=0.001, name="wheel head stays centered when rotated")
        # Verify wheel hub is raised above base shell
        base_shell_aabb = ctx.part_element_world_aabb(base, elem="base_shell")
        wheel_hub_aabb = ctx.part_element_world_aabb(wheel_head, elem="wheel_hub")
        if base_shell_aabb and wheel_hub_aabb:
            ctx.check(
                "wheel hub raised above base shell",
                wheel_hub_aabb[0][2] >= base_shell_aabb[1][2] - 0.001,
                details=f"base shell max z={base_shell_aabb[1][2]:.3f}, wheel hub min z={wheel_hub_aabb[0][2]:.3f}",
            )

    # 2. Support: wheel head is physically supported by base
    ctx.expect_contact(wheel_head, base, name="wheel head supported by base")

    # 3. Visible details verification
    # Splash pan rim is larger than base and overlaps in XY
    ctx.expect_overlap(
        base, base,
        elem_a="splash_pan_rim", elem_b="base_shell",
        axes="xy", min_overlap=0.1,
        name="splash pan overlaps base footprint",
    )
    # Foot pedal is at ground level
    pedal_aabb = ctx.part_element_world_aabb(base, elem="foot_pedal")
    if pedal_aabb:
        ctx.check(
            "foot pedal at ground level",
            pedal_aabb[0][2] <= 0.02,
            details=f"foot pedal min z={pedal_aabb[0][2]:.3f}",
        )
    # Wheel head has required visual elements
    wheel_visual_names = [v.name for v in wheel_head.visuals]
    ctx.check(
        "wheel head has hub and plate",
        "wheel_hub" in wheel_visual_names and "wheel_plate" in wheel_visual_names,
        details=f"wheel visuals: {wheel_visual_names}",
    )

    return ctx.report()


object_model = build_object_model()
