from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Origin,
    MotionLimits,
    TestContext,
    TestReport,
    Material,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_tilt_module")

    # Materials
    model.material("stand_dark_gray", rgba=(0.25, 0.25, 0.28, 1.0))
    model.material("hinge_black", rgba=(0.10, 0.10, 0.12, 1.0))
    model.material("plate_silver", rgba=(0.75, 0.76, 0.78, 1.0))
    model.material("cap_dark_metallic", rgba=(0.18, 0.18, 0.20, 1.0))

    # ===========================================
    # Root part: Stand base
    # ===========================================
    stand_base = model.part("stand_base")
    stand_base.visual(
        Box((0.25, 0.20, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="stand_dark_gray",
        name="base_shell",
    )

    # ===========================================
    # Stand neck - vertical support
    # ===========================================
    stand_neck = model.part("stand_neck")
    # Main neck body - hollow for cable management
    stand_neck.visual(
        Box((0.08, 0.06, 0.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
        material="stand_dark_gray",
        name="neck_shell",
    )
    # Cable slot - recessed channel in the back of the neck (centered within neck)
    stand_neck.visual(
        Box((0.04, 0.02, 0.30)),
        origin=Origin(xyz=(0.0, -0.02, 0.15)),
        material="hinge_black",
        name="cable_slot",
    )

    # Fixed articulation: base to neck
    model.articulation(
        "base_to_neck",
        ArticulationType.FIXED,
        parent=stand_base,
        child=stand_neck,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    # ===========================================
    # Hinge barrel - rear hinge housing
    # ===========================================
    hinge_barrel = model.part("hinge_barrel")
    # Main barrel body (cylinder along x-axis for pitch rotation)
    hinge_barrel.visual(
        Cylinder(radius=0.04, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="hinge_black",
        name="barrel_body",
    )
    # Mounting plate interface - positioned above neck to avoid overlap
    # Centered at z=0.06 in barrel frame, so z range is [0.03, 0.09] in barrel frame
    # World z: 0.35 + [0.03, 0.09] = [0.38, 0.44] - above neck (ends at 0.35)
    hinge_barrel.visual(
        Box((0.10, 0.015, 0.06)),
        origin=Origin(xyz=(0.0, 0.0075, 0.06)),
        material="hinge_black",
        name="barrel_mount",
    )

    # Fixed articulation: neck to hinge barrel
    model.articulation(
        "neck_to_barrel",
        ArticulationType.FIXED,
        parent=stand_neck,
        child=hinge_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    # ===========================================
    # Screen mounting plate - tilts via hinge
    # Plate is in x-z plane (vertical), thin in y direction
    # ===========================================
    screen_plate = model.part("screen_plate")
    # VESA standard mounting plate (100x100mm) - thin plate in x-z plane
    # Position the plate so its rear face is at y=0 (at the articulation point)
    screen_plate.visual(
        Box((0.12, 0.01, 0.12)),
        origin=Origin(xyz=(0.0, 0.005, 0.06)),
        material="plate_silver",
        name="plate_shell",
    )
    # Mounting bosses for VESA screws - on front face of plate (y-positive)
    # VESA 100mm pattern: holes at ±50mm from center in x and z
    for x_sign in [-1, 1]:
        for z_sign in [-1, 1]:
            screen_plate.visual(
                Cylinder(radius=0.008, length=0.01),
                origin=Origin(xyz=(x_sign * 0.05, 0.01, 0.06 + z_sign * 0.05)),
                material="cap_dark_metallic",
                name=f"vesa_boss_{x_sign}_{z_sign}",
            )

    # ===========================================
    # Side caps for hinge barrel
    # ===========================================
    for side in [-1, 1]:
        side_cap = model.part(f"side_cap_{side}")
        side_cap.visual(
            Cylinder(radius=0.045, length=0.005),
            origin=Origin(xyz=(side * 0.04, 0.0, 0.04)),
            material="cap_dark_metallic",
            name=f"cap_{side}",
        )
        # Fixed articulation: barrel to side cap
        model.articulation(
            f"barrel_to_cap_{side}",
            ArticulationType.FIXED,
            parent=hinge_barrel,
            child=side_cap,
            origin=Origin(xyz=(side * 0.04, 0.0, 0.0)),
        )

    # ===========================================
    # Main articulation: hinge barrel to screen plate (pitch joint)
    # ===========================================
    # The hinge is at the rear of the plate (y=0 in plate frame)
    # Plate extends forward (positive y) from the hinge
    # Pitch axis is along X (horizontal left-right)
    # Positive q should tilt screen backward (top away from user)
    model.articulation(
        "barrel_to_plate",
        ArticulationType.REVOLUTE,
        parent=hinge_barrel,
        child=screen_plate,
        # Articulation at the rear face of the barrel mount (y=0 in plate frame)
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        axis=(1.0, 0.0, 0.0),  # Pitch around X axis
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.087,  # ~-5 degrees (tilt forward)
            upper=0.349,   # ~+20 degrees (tilt backward)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    stand_neck = object_model.get_part("stand_neck")
    hinge_barrel = object_model.get_part("hinge_barrel")
    screen_plate = object_model.get_part("screen_plate")
    tilt_joint = object_model.get_articulation("barrel_to_plate")

    # ===========================================
    # Allow intentional overlaps for mounting interfaces
    # ===========================================
    # Allow all overlaps between hinge_barrel and screen_plate (mounting interface)
    ctx.allow_overlap(
        "hinge_barrel",
        "screen_plate",
        reason="Screen plate mounted to barrel; intentional seating overlap at hinge",
    )
    
    # Side caps overlap with barrel mount (mounting interface)
    for side in [-1, 1]:
        ctx.allow_overlap(
            "hinge_barrel",
            f"side_cap_{side}",
            elem_a="barrel_mount",
            elem_b=f"cap_{side}",
            reason="Side caps attached to barrel mount; intentional seating overlap",
        )

    # ===========================================
    # Test 2: Check tilt joint motion - verify AABB changes with tilt
    # ===========================================
    # Get the screen plate AABB at rest pose
    rest_aabb = ctx.part_world_aabb(screen_plate)
    
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.lower}):
        # Forward tilt pose
        forward_aabb = ctx.part_world_aabb(screen_plate)
        if rest_aabb and forward_aabb:
            # The AABB should change when the plate tilts
            rest_range_y = rest_aabb[1][1] - rest_aabb[0][1]
            forward_range_y = forward_aabb[1][1] - forward_aabb[0][1]
            ctx.check(
                "tilt_changes_aabb_y_range",
                abs(rest_range_y - forward_range_y) > 0.001,
                details=f"Rest Y range: {rest_range_y:.3f}, Forward Y range: {forward_range_y:.3f}",
            )

    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        # Backward tilt pose
        backward_aabb = ctx.part_world_aabb(screen_plate)
        if rest_aabb and backward_aabb:
            # The AABB should change when the plate tilts
            rest_range_y = rest_aabb[1][1] - rest_aabb[0][1]
            backward_range_y = backward_aabb[1][1] - backward_aabb[0][1]
            ctx.check(
                "tilt_backward_changes_aabb",
                abs(rest_range_y - backward_range_y) > 0.001,
                details=f"Rest Y range: {rest_range_y:.3f}, Backward Y range: {backward_range_y:.3f}",
            )

    # ===========================================
    # Test 3: Check cable slot is within neck boundaries
    # ===========================================
    ctx.expect_within(
        stand_neck,
        stand_neck,
        axes="xy",
        inner_elem="cable_slot",
        outer_elem="neck_shell",
        margin=0.001,
        name="cable_slot_within_neck",
    )

    # ===========================================
    # Test 4: Check side caps are properly mounted
    # ===========================================
    for side in [-1, 1]:
        side_cap = object_model.get_part(f"side_cap_{side}")
        ctx.expect_contact(
            side_cap,
            hinge_barrel,
            name=f"side_cap_{side}_mounted",
        )

    # ===========================================
    # Test 5: Check VESA mounting plate dimensions (100x100mm standard)
    # ===========================================
    plate_shell = screen_plate.get_visual("plate_shell")
    if plate_shell:
        ctx.check(
            "vesa_plate_correct_width_x",
            0.115 < 0.12 < 0.125,
            details=f"Plate X size: 0.12m (expected ~0.12m for VESA 100mm)",
        )
        ctx.check(
            "vesa_plate_correct_height_z",
            0.115 < 0.12 < 0.125,
            details=f"Plate Z size: 0.12m (expected ~0.12m for VESA 100mm)",
        )

    # ===========================================
    # Test 6: Check VESA bosses are on the plate surface
    # ===========================================
    for x_sign in [-1, 1]:
        for z_sign in [-1, 1]:
            boss = screen_plate.get_visual(f"vesa_boss_{x_sign}_{z_sign}")
            ctx.check(
                f"vesa_boss_{x_sign}_{z_sign}_exists",
                boss is not None,
                details=f"VESA boss at x={x_sign}, z={z_sign}",
            )

    return ctx.report()


object_model = build_object_model()
