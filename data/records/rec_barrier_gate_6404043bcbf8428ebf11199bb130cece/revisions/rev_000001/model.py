from math import pi
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
    model = ArticulatedObject(name="teaching_boom_gate")

    # Materials
    model.material("concrete", rgba=(0.55, 0.55, 0.52, 1.0))
    model.material("transparent_housing", rgba=(0.7, 0.85, 0.9, 0.4))
    model.material("boom_yellow", rgba=(1.0, 0.85, 0.0, 1.0))
    model.material("warning_black", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("axle_metal", rgba=(0.6, 0.62, 0.65, 1.0))
    model.material("arrow_red", rgba=(0.9, 0.15, 0.15, 1.0))

    # PIVOT HEIGHT - the height where boom rotates
    PIVOT_Z = 0.6

    # === FIXED BASE ===
    base = model.part("base")
    # Concrete base that sits on the ground
    base.visual(
        Box((0.5, 0.5, 0.4)),
        origin=Origin(xyz=(0.0, 0.0, 0.2)),
        material="concrete",
        name="base_block",
    )
    # Mounting post that extends up to just below pivot height
    base.visual(
        Box((0.2, 0.2, PIVOT_Z - 0.05)),
        origin=Origin(xyz=(0.0, 0.0, (PIVOT_Z - 0.05) / 2)),
        material="concrete",
        name="mounting_post",
    )

    # === PIVOT HOUSING (Transparent) ===
    # Housing sits on top of the base at pivot height
    pivot_housing = model.part("pivot_housing")
    # Housing is a boxy transparent enclosure at pivot height
    # Center the housing at PIVOT_Z
    pivot_housing.visual(
        Box((0.18, 0.25, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="transparent_housing",
        name="housing_shell",
    )
    # The housing part frame will be at PIVOT_Z via the articulation

    model.articulation(
        "base_to_housing",
        ArticulationType.FIXED,
        parent=base,
        child=pivot_housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),  # Housing part frame at PIVOT_Z
    )

    # === AXLE CYLINDER ===
    # Axle is at pivot height, runs horizontally through housing
    axle = model.part("axle")
    # Axle cylinder centered in the housing
    axle.visual(
        Cylinder(radius=0.025, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi/2, 0.0)),
        material="axle_metal",
        name="axle_cylinder",
    )
    # Axle part frame same as housing (at PIVOT_Z)

    model.articulation(
        "housing_to_axle",
        ArticulationType.FIXED,
        parent=pivot_housing,
        child=axle,
        origin=Origin(),  # Same position as housing
    )

    # === BOOM ARM (Primary moving part) ===
    boom = model.part("boom_arm")
    boom_length = 3.5
    boom_width = 0.08
    boom_height = 0.06

    # Main boom body - extends along +X from the pivot point
    # At q=0 (closed/horizontal), boom extends along +X from pivot
    boom.visual(
        Box((boom_length, boom_width, boom_height)),
        origin=Origin(xyz=(boom_length / 2, 0.0, 0.0)),
        material="boom_yellow",
        name="boom_body",
    )

    # Add black warning stripes on the boom (every 0.3m)
    stripe_count = int(boom_length / 0.3)
    for i in range(stripe_count):
        stripe_x = 0.15 + i * 0.3
        if stripe_x + 0.15 < boom_length:
            boom.visual(
                Box((0.15, boom_width + 0.005, boom_height + 0.005)),
                origin=Origin(xyz=(stripe_x, 0.0, 0.0)),
                material="warning_black",
                name=f"stripe_{i}",
            )

    # Add end cap to boom
    boom.visual(
        Box((0.1, boom_width, boom_height)),
        origin=Origin(xyz=(boom_length + 0.05, 0.0, 0.0)),
        material="warning_black",
        name="boom_end_cap",
    )

    # === DIRECTION ARROWS on the boom ===
    # Arrows on top surface of boom, pointing along +X (traffic direction)
    # Center arrows on boom width (y=0) and place on top surface (z = height/2 + small offset)
    arrow_z = boom_height / 2 + 0.002  # On top surface
    for i, x_pos in enumerate([0.8, 1.6, 2.6]):
        # Arrow body (rectangle) - centered on boom
        boom.visual(
            Box((0.15, 0.03, 0.003)),
            origin=Origin(xyz=(x_pos, 0.0, arrow_z)),
            material="arrow_red",
            name=f"arrow_body_{i}",
        )
        # Arrow head - centered on boom
        boom.visual(
            Box((0.06, 0.06, 0.003)),
            origin=Origin(xyz=(x_pos + 0.1, 0.0, arrow_z)),
            material="arrow_red",
            name=f"arrow_head_{i}",
        )

    # === BOOM ARTICULATION ===
    # Boom rotates around Y axis at the pivot point (PIVOT_Z)
    # At q=0: boom is horizontal (extends along +X)
    # Positive q: boom pitches up (rotates toward +Z)
    # Using axis=(0, -1, 0): right-hand rule with thumb in -Y
    # fingers curl from +X toward +Z (upward)

    model.articulation(
        "axle_to_boom",
        ArticulationType.REVOLUTE,
        parent=axle,
        child=boom,
        origin=Origin(),  # Boom part frame = axle part frame = pivot point
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.5,
            lower=0.0,
            upper=pi / 2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    boom = object_model.get_part("boom_arm")
    pivot_housing = object_model.get_part("pivot_housing")
    axle = object_model.get_part("axle")
    hinge = object_model.get_articulation("axle_to_boom")

    # === Allow intentional overlaps (full part pairs) ===
    # Axle passes through housing and boom - these are intentional
    ctx.allow_overlap(
        "axle", "pivot_housing",
        reason="Axle intentionally passes through the pivot housing",
    )
    # Axle passes through boom at pivot point
    ctx.allow_overlap(
        "axle", "boom_arm",
        reason="Boom rotates around axle at pivot point - axle passes through boom",
    )
    # Housing seated on base mounting post
    ctx.allow_overlap(
        "base", "pivot_housing",
        reason="Housing seated on base mounting post",
    )
    # Boom pivot point adjacent to housing
    ctx.allow_overlap(
        "boom_arm", "pivot_housing",
        reason="Boom pivot point adjacent to housing",
    )

    # === Test 1: Verify articulation exists and has correct type ===
    ctx.check(
        "articulation_is_revolute",
        hinge is not None and hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Boom gate must have a revolute joint",
    )

    # === Test 2: Verify motion limits ===
    limits = hinge.motion_limits
    ctx.check(
        "motion_limits_valid",
        limits is not None and limits.lower == 0.0 and limits.upper == pi / 2,
        details=f"Motion limits should be 0 to pi/2, got {limits.lower if limits else None}, {limits.upper if limits else None}",
    )

    # === Test 3: Verify axis is horizontal (Y axis) for pitch ===
    ctx.check(
        "axis_is_horizontal",
        hinge.axis is not None and abs(hinge.axis[1]) > 0.99,
        details=f"Axis should be along Y for pitch, got {hinge.axis}",
    )

    # === Test 4: Check closed position (q=0) ===
    with ctx.pose({hinge: 0.0}):
        # Boom should be horizontal - check AABB
        boom_aabb = ctx.part_world_aabb(boom)
        if boom_aabb:
            # At q=0, boom extends along X, so X range should be large
            x_range = boom_aabb[1][0] - boom_aabb[0][0]
            ctx.check(
                "boom_horizontal_at_closed",
                x_range > 3.0,  # Boom should extend more than 3m in X
                details=f"Boom X range at closed: {x_range}",
            )

    # === Test 5: Check open position (q=pi/2) ===
    with ctx.pose({hinge: pi / 2}):
        boom_aabb_open = ctx.part_world_aabb(boom)
        if boom_aabb_open:
            # At q=pi/2, boom should extend upward - Z range should be large
            z_range = boom_aabb_open[1][2] - boom_aabb_open[0][2]
            ctx.check(
                "boom_vertical_at_open",
                z_range > 3.0,  # Boom should extend more than 3m in Z
                details=f"Boom Z range at open: {z_range}",
            )

    # === Test 6: Check that boom raises on positive q ===
    # Compare Z position of boom's far end
    with ctx.pose({hinge: 0.0}):
        aabb_closed = ctx.part_world_aabb(boom)
        z_max_closed = aabb_closed[1][2] if aabb_closed else 0

    with ctx.pose({hinge: pi / 2}):
        aabb_open = ctx.part_world_aabb(boom)
        z_max_open = aabb_open[1][2] if aabb_open else 0

    ctx.check(
        "boom_raises_on_positive_q",
        z_max_open > z_max_closed + 1.0,
        details=f"Z max: closed={z_max_closed}, open={z_max_open}",
    )

    # === Test 7: Verify housing is mounted to base ===
    ctx.expect_contact(pivot_housing, base, name="housing_mounted_to_base")

    # === Test 8: Verify axle is mounted to housing ===
    ctx.expect_contact(axle, pivot_housing, name="axle_mounted_to_housing")

    # === Test 9: Verify visual details ===
    ctx.check(
        "boom_has_stripes",
        any("stripe" in (v.name or "") for v in boom.visuals),
        details="Boom should have warning stripes",
    )
    ctx.check(
        "boom_has_arrows",
        any("arrow" in (v.name or "") for v in boom.visuals),
        details="Boom should have direction arrows",
    )

    # === Test 10: Verify transparent housing material ===
    housing_visuals = [v for v in pivot_housing.visuals if v.name == "housing_shell"]
    if housing_visuals:
        ctx.check(
            "housing_is_transparent",
            housing_visuals[0].material == "transparent_housing",
            details="Pivot housing should use transparent material",
        )

    return ctx.report()


object_model = build_object_model()
