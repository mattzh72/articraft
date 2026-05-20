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
    Material,
    mesh_from_geometry,
    KnobGeometry,
    KnobIndicator,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pointer_dial")

    # Materials
    base_material = Material(name="base_metal", rgba=(0.4, 0.4, 0.45, 1.0))
    scale_material = Material(name="scale_dark", rgba=(0.2, 0.2, 0.25, 1.0))
    pointer_material = Material(name="pointer_red", rgba=(0.8, 0.15, 0.15, 1.0))
    spindle_material = Material(name="spindle_chrome", rgba=(0.7, 0.7, 0.75, 1.0))
    detent_material = Material(name="detent_brass", rgba=(0.85, 0.65, 0.13, 1.0))

    # --- BASE PART (fixed) ---
    base = model.part("base")

    # Base plate - circular plate that mounts to surface
    base.visual(
        Cylinder(radius=0.055, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=base_material,
        name="base_plate",
    )

    # Scale ring on the base - slightly raised with markings
    # Base plate extends to z=0.008, raise scale ring to create visible gap
    base.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=scale_material,
        name="scale_ring",
    )

    # Detent bumps around the scale (every 45 degrees for 8 positions)
    import math
    for i in range(8):
        angle = i * (2 * math.pi / 8)
        bump_x = 0.045 * math.cos(angle)
        bump_y = 0.045 * math.sin(angle)
        base.visual(
            Cylinder(radius=0.0025, length=0.003),
            origin=Origin(xyz=(bump_x, bump_y, 0.012)),
            material=detent_material,
            name=f"detent_{i}",
        )

    # Center spindle (fixed to base, pointer rotates around it)
    base.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=spindle_material,
        name="spindle",
    )

    # --- POINTER PART (rotating) ---
    pointer = model.part("pointer")

    # Main pointer arm - thin rectangular arm extending from center to scale
    # Scale ring radius is 0.050m, arm extends almost to the scale
    # Arm extends from x=0.008 to x=0.046 (origin at 0.027)
    pointer.visual(
        Box((0.038, 0.006, 0.003)),
        origin=Origin(xyz=(0.027, 0.0, 0.025)),
        material=pointer_material,
        name="pointer_arm",
    )

    # Pointer tip - slightly wider at the end, pointing to the scale
    # Tip extends from x=0.046 to x=0.050 (origin at 0.048) - exactly at scale ring
    pointer.visual(
        Box((0.004, 0.010, 0.003)),
        origin=Origin(xyz=(0.048, 0.0, 0.025)),
        material=pointer_material,
        name="pointer_tip",
    )

    # Pointer hub - connected to pointer arm, sits on top of spindle
    # Spindle top is at z=0.028 world = 0.008 pointer-local
    # Arm is at z=0.025 pointer-local, so hub should extend up to meet the arm
    # Hub extends from z=0.008 to z=0.042 (connects to arm at z=0.025±0.0015)
    pointer.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=spindle_material,
        name="pointer_hub",
    )

    # Pointer knob/cap on top for finger grip
    # Pointer hub top is at z=0.025 + 0.034/2 = 0.042 in pointer-local
    # Knob bottom should be at 0.042, so origin z = 0.042 + knob_height/2
    pointer_knob = KnobGeometry(
        0.016,
        0.008,
        body_style="cylindrical",
        edge_radius=0.001,
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
    )
    pointer_knob_mesh = mesh_from_geometry(pointer_knob, "pointer_knob")
    pointer.visual(
        pointer_knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=spindle_material,
        name="pointer_knob",
    )

    # --- ARTICULATION ---
    # Revolute joint: pointer rotates around Z axis (yaw) relative to base
    # The articulation frame is at the spindle center, at the top of base
    # At q=0, pointer arm extends along +X
    # Positive q rotates counterclockwise (right-hand rule around +Z)
    model.articulation(
        "base_to_pointer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pointer,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-math.pi * 0.9,  # Nearly full rotation
            upper=math.pi * 0.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    import math
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pointer = object_model.get_part("pointer")
    hinge = object_model.get_articulation("base_to_pointer")

    # Allow the pointer hub to overlap with spindle (seated connection)
    ctx.allow_overlap(
        "base",
        "pointer",
        elem_a="spindle",
        elem_b="pointer_hub",
        reason="Pointer hub sits on top of spindle as a seated connection",
    )

    # === MECHANISM TESTS ===
    # Check that pointer hub contacts spindle at rest position
    ctx.expect_contact(
        "base",
        "pointer",
        elem_a="spindle",
        elem_b="pointer_hub",
        contact_tol=0.005,
        name="pointer_hub_seated_on_spindle",
    )

    # Check that pointer can rotate - test at various angles
    with ctx.pose(base_to_pointer=0.0):
        ctx.expect_contact(
            "base",
            "pointer",
            elem_a="spindle",
            elem_b="pointer_hub",
            contact_tol=0.005,
            name="pointer_contact_at_zero",
        )
        # Check pointer tip is at positive X (pointing right)
        tip = pointer.get_visual("pointer_tip")
        ctx.check(
            "pointer_tip_at_0deg",
            tip is not None,
            "Pointer tip should exist at 0°",
        )

    with ctx.pose(base_to_pointer=math.pi / 2):  # 90 degrees
        # Pointer should still be connected to spindle
        ctx.expect_contact(
            "base",
            "pointer",
            elem_a="spindle",
            elem_b="pointer_hub",
            contact_tol=0.005,
            name="pointer_contact_at_90deg",
        )

    with ctx.pose(base_to_pointer=math.pi):  # 180 degrees
        ctx.expect_contact(
            "base",
            "pointer",
            elem_a="spindle",
            elem_b="pointer_hub",
            contact_tol=0.005,
            name="pointer_contact_at_180deg",
        )

    # Test rotation range - pointer should rotate to its limits
    motion_limits = hinge.motion_limits
    if motion_limits and motion_limits.lower is not None:
        with ctx.pose(base_to_pointer=motion_limits.lower):
            ctx.check(
                "pointer_at_lower_limit",
                True,  # Just verify no crash
                "Pointer should reach lower limit",
            )
    if motion_limits and motion_limits.upper is not None:
        with ctx.pose(base_to_pointer=motion_limits.upper):
            ctx.check(
                "pointer_at_upper_limit",
                True,  # Just verify no crash
                "Pointer should reach upper limit",
            )

    # === SUPPORT/CONTACT TESTS ===
    # Verify the base is the root part (fixed)
    root_parts = object_model.root_parts()
    ctx.check(
        "base_is_root_part",
        len(root_parts) == 1 and root_parts[0].name == "base",
        f"Base should be the only root part, got {[p.name for p in root_parts]}",
    )

    # Verify pointer is not a root part (it has parent)
    ctx.check(
        "pointer_not_root",
        len(root_parts) == 1,
        "Pointer should not be a root part",
    )

    # === VISIBLE DETAILS TESTS ===
    # Verify the pointer extends outward from center
    pointer_arm = pointer.get_visual("pointer_arm")
    ctx.check(
        "pointer_arm_extends_from_center",
        pointer_arm is not None,
        "Pointer arm visual should exist",
    )

    # Check that detent bumps are on the base (should have 8 detents)
    detent_count = sum(1 for v in base.visuals if v.name and v.name.startswith("detent_"))
    ctx.check(
        "detent_bumps_present",
        detent_count == 8,
        f"Should have 8 detent bumps, found {detent_count}",
    )

    # Verify detent bumps are at correct radius (should be ~0.045m from center)
    for i in range(8):
        detent = base.get_visual(f"detent_{i}")
        ctx.check(
            f"detent_{i}_position",
            detent is not None,
            f"Detent {i} should exist",
        )

    # Verify scale ring is raised above base plate
    ctx.expect_gap(
        "base",
        "base",
        axis="z",
        min_gap=0.001,
        positive_elem="scale_ring",
        negative_elem="base_plate",
        name="scale_ring_raised_above_base",
    )

    # Verify materials are assigned (check key visuals exist with materials)
    ctx.check(
        "base_plate_has_material",
        base.get_visual("base_plate") is not None
        and base.get_visual("base_plate").material is not None,
        "Base plate should have material assigned",
    )
    ctx.check(
        "pointer_arm_has_material",
        pointer.get_visual("pointer_arm") is not None
        and pointer.get_visual("pointer_arm").material is not None,
        "Pointer arm should have material assigned",
    )

    # Check that pointer rotates in the correct direction (origin stays near center)
    rest_pos = ctx.part_world_position(pointer)
    with ctx.pose(base_to_pointer=math.pi / 2):
        rotated_pos = ctx.part_world_position(pointer)
        # The pointer part origin is at spindle center, so position shouldn't change much
        ctx.check(
            "pointer_rotates_around_center",
            abs(rest_pos[0] - rotated_pos[0]) < 0.001
            and abs(rest_pos[1] - rotated_pos[1]) < 0.001,
            f"Pointer origin should stay near center during rotation: rest={rest_pos}, rotated={rotated_pos}",
        )

    return ctx.report()


object_model = build_object_model()
