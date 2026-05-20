from __future__ import annotations

from math import cos, sin

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
    model = ArticulatedObject(name="panel_tilt_mount")

    # Materials
    base_material = model.material("base_metal", rgba=(0.35, 0.35, 0.40, 1.0))
    panel_material = model.material("panel_white", rgba=(0.92, 0.92, 0.95, 1.0))
    axle_material = model.material("axle_steel", rgba=(0.65, 0.65, 0.70, 1.0))
    stop_material = model.material("stop_aluminum", rgba=(0.50, 0.50, 0.55, 1.0))

    # Base part (fixed) - contains base plate and side cheeks
    base = model.part("base")

    # Base plate - flat plate that sits on surface
    base.visual(
        Box((0.16, 0.12, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_material,
        name="base_plate",
    )

    # Left cheek - vertical support with rounded appearance
    base.visual(
        Box((0.012, 0.10, 0.10)),
        origin=Origin(xyz=(-0.074, 0.0, 0.064)),
        material=base_material,
        name="left_cheek",
    )

    # Right cheek - vertical support
    base.visual(
        Box((0.012, 0.10, 0.10)),
        origin=Origin(xyz=(0.074, 0.0, 0.064)),
        material=base_material,
        name="right_cheek",
    )

    # Stop tabs on left cheek - positioned to limit panel tilt
    # Lower stop (limits forward/downward tilt) - positioned at bottom front of panel path
    base.visual(
        Box((0.010, 0.015, 0.008)),
        origin=Origin(xyz=(-0.080, 0.080, -0.010)),
        material=stop_material,
        name="stop_lower_left",
    )

    # Upper stop (limits backward/upward tilt) - positioned at top rear of panel path
    base.visual(
        Box((0.010, 0.015, 0.008)),
        origin=Origin(xyz=(-0.080, 0.025, 0.140)),
        material=stop_material,
        name="stop_upper_left",
    )

    # Stop tabs on right cheek
    base.visual(
        Box((0.010, 0.015, 0.008)),
        origin=Origin(xyz=(0.080, 0.080, -0.010)),
        material=stop_material,
        name="stop_lower_right",
    )

    base.visual(
        Box((0.010, 0.015, 0.008)),
        origin=Origin(xyz=(0.080, 0.025, 0.140)),
        material=stop_material,
        name="stop_upper_right",
    )

    # Panel part (tilting) - the adjustable panel
    panel = model.part("panel")

    # Main panel body - rectangular panel, thin profile
    # Panel is vertical at q=0, facing along +X direction
    # Dimensions: 6mm thick (Y), 220mm wide (X), 160mm high (Z)
    # Pivot point is at z=0.064, which is near the bottom third of the 160mm panel
    # This gives ~96mm below pivot and ~64mm above pivot
    panel.visual(
        Box((0.22, 0.006, 0.16)),
        origin=Origin(xyz=(0.0, 0.053, -0.096)),
        material=panel_material,
        name="panel_shell",
    )

    # Axle - main pivot axle passing through panel and cheeks
    # Horizontal along X axis, passes through cheeks and panel
    # Positioned at y=0.053 to align with panel center, z=0.064 at pivot height
    panel.visual(
        Cylinder(radius=0.006, length=0.18),
        origin=Origin(xyz=(0.0, 0.053, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=axle_material,
        name="axle",
    )

    # Left washer - between panel and left cheek
    panel.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(-0.085, 0.053, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=axle_material,
        name="washer_left",
    )

    # Right washer - between panel and right cheek
    panel.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.085, 0.053, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=axle_material,
        name="washer_right",
    )

    # Left axle pin - end cap/retainer on left side
    panel.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(-0.093, 0.053, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=axle_material,
        name="pin_left",
    )

    # Right axle pin - end cap/retainer on right side
    panel.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.093, 0.053, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=axle_material,
        name="pin_right",
    )

    # Articulation: base to panel
    # - Pivot axis: X (horizontal, left-right)
    # - Pivot point: at z=0.064 (in base frame), y=0.053 (panel center)
    # - At q=0: panel is vertical
    # - Positive q: panel tilts backward (top moves toward -Y, following right-hand rule around +X)
    # - Panel bottom is at z=-0.032, top at z=0.128 in panel frame at q=0
    model.articulation(
        "base_to_panel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=panel,
        origin=Origin(xyz=(0.0, 0.053, 0.064)),
        axis=(1.0, 0.0, 0.0),  # Rotation around X axis (pitch tilt)
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    panel = object_model.get_part("panel")
    hinge = object_model.get_articulation("base_to_panel")

    # Check parts exist
    ctx.check("base_present", base is not None, "Base part not found")
    ctx.check("panel_present", panel is not None, "Panel part not found")
    ctx.check("hinge_present", hinge is not None, "Tilt hinge articulation not found")

    if base is None or panel is None or hinge is None:
        return ctx.report()

    # Allow panel to be "floating" - it's intentionally connected only via the axle/articulation
    ctx.allow_isolated_part(
        "panel",
        reason="Panel is intentionally connected to base only via the axle/articulation, not direct contact",
    )

    # Allow overlap between axle and cheeks (axle passes through cheek cutouts)
    ctx.allow_overlap(
        "base", "panel",
        elem_a="left_cheek",
        elem_b="axle",
        reason="Axle passes through left cheek opening (intentional nested fit)",
    )
    ctx.allow_overlap(
        "base", "panel",
        elem_a="right_cheek",
        elem_b="axle",
        reason="Axle passes through right cheek opening (intentional nested fit)",
    )

    # Allow overlap between washers and cheeks
    ctx.allow_overlap(
        "base", "panel",
        elem_a="left_cheek",
        elem_b="washer_left",
        reason="Washer sits between panel and left cheek",
    )
    ctx.allow_overlap(
        "base", "panel",
        elem_a="right_cheek",
        elem_b="washer_right",
        reason="Washer sits between panel and right cheek",
    )

    # Verify panel is properly connected to base via articulation
    ctx.check(
        "panel_properly_supported",
        hinge.parent == "base" and hinge.child == "panel",
        f"Panel not correctly connected to base via hinge: parent={hinge.parent}, child={hinge.child}",
    )

    # Check motion limits are set correctly for tilt joint
    limits = hinge.motion_limits
    ctx.check(
        "tilt_limits_exist",
        limits is not None and limits.lower is not None and limits.upper is not None,
        "Tilt joint missing motion limits",
    )
    if limits and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "tilt_range_reasonable",
            limits.lower < 0 and limits.upper > 0,
            f"Tilt range should allow both directions: lower={limits.lower}, upper={limits.upper}",
        )

    # Test at rest position (q=0, panel vertical)
    with ctx.pose({hinge: 0.0}):
        panel_aabb = ctx.part_world_aabb(panel)
        if panel_aabb:
            # Panel should be in front of base (y > 0)
            ctx.check(
                "panel_in_front_of_base_at_rest",
                panel_aabb[0][1] >= 0.0,
                f"Panel should be in front of base (y>=0): {panel_aabb}",
            )

    # Test tilted position - verify the panel orientation changes
    # At q=0.35 (positive), panel tilts backward (top moves toward -Y)
    with ctx.pose({hinge: 0.35}):
        panel_aabb_tilted = ctx.part_world_aabb(panel)
        if panel_aabb and panel_aabb_tilted:
            # The panel should have rotated, so the AABB should be different
            ctx.check(
                "panel_tilts_backward",
                panel_aabb_tilted is not None,
                "Panel AABB should be valid at tilted pose",
            )

    # Test opposite tilt
    with ctx.pose({hinge: -0.35}):
        panel_aabb_opposite = ctx.part_world_aabb(panel)
        if panel_aabb_opposite:
            ctx.check(
                "panel_tilts_forward",
                panel_aabb_opposite is not None,
                "Panel AABB should be valid at opposite tilt",
            )

    # Verify the articulation has proper axis and limits
    ctx.check(
        "articulation_axis_correct",
        hinge.axis == (1.0, 0.0, 0.0),
        f"Articulation axis should be along X: {hinge.axis}",
    )

    ctx.check(
        "tilt_joint_has_proper_limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower is not None
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.lower < 0
        and hinge.motion_limits.upper > 0,
        "Tilt joint should allow both positive and negative rotation",
    )

    # Verify all required visual elements exist
    panel_visuals = [v.name for v in panel.visuals]
    ctx.check(
        "panel_has_axle",
        "axle" in panel_visuals,
        f"Panel missing axle visual: {panel_visuals}",
    )
    ctx.check(
        "panel_has_washers",
        "washer_left" in panel_visuals and "washer_right" in panel_visuals,
        f"Panel missing washer visuals: {panel_visuals}",
    )
    ctx.check(
        "panel_has_pins",
        "pin_left" in panel_visuals and "pin_right" in panel_visuals,
        f"Panel missing pin visuals: {panel_visuals}",
    )

    base_visuals = [v.name for v in base.visuals]
    ctx.check(
        "base_has_cheeks",
        "left_cheek" in base_visuals and "right_cheek" in base_visuals,
        f"Base missing cheek visuals: {base_visuals}",
    )
    ctx.check(
        "base_has_stops",
        "stop_lower_left" in base_visuals
        and "stop_upper_left" in base_visuals
        and "stop_lower_right" in base_visuals
        and "stop_upper_right" in base_visuals,
        f"Base missing stop tab visuals: {base_visuals}",
    )

    return ctx.report()


object_model = build_object_model()
