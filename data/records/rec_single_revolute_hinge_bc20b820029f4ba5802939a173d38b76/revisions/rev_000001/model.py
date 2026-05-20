from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_hinge_sample")
    model.meta["description"] = "Laptop-style hinge with base, screen, two hinge barrels, central axle, end caps."

    # ---------- Root Part: Base ----------
    base = model.part("base")
    base_width = 0.3  # 30cm wide (x-axis)
    base_depth = 0.22  # 22cm deep (y-axis)
    base_thickness = 0.015  # 1.5cm thick (z-axis)
    fillet_radius = 0.002  # 2mm fillet on vertical edges

    # Base strip with rounded edges using CadQuery
    base_cq = (
        cq.Workplane("XY")
        .rect(base_width, base_depth)
        .extrude(base_thickness)
        .edges("|Z")  # Select edges parallel to Z-axis (vertical edges)
        .fillet(fillet_radius)
    )
    base.visual(
        mesh_from_cadquery(base_cq, "base_strip"),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2)),  # Center base at z=0.0075, sits on z=0
        name="base_strip",
    )

    # ---------- Hinge Barrels (fixed to base) ----------
    barrel_radius = 0.005  # 5mm radius
    barrel_length = 0.02  # 2cm long (x-axis)
    barrel_offset_x = 0.12  # 12cm left/right of center
    hinge_y = 0.11  # Top edge of base (y-axis)
    hinge_z = base_thickness  # Top of base (z-axis)

    for i, x_pos in enumerate([-barrel_offset_x, barrel_offset_x]):
        # Cylinder along x-axis, centered at (x_pos, hinge_y, hinge_z)
        barrel_cq = (
            cq.Workplane("YZ")  # Workplane normal to x-axis
            .workplane(offset=x_pos - barrel_length / 2)  # Start at x_pos - 0.01m
            .center(hinge_y, hinge_z)  # Center barrel at hinge y/z position
            .circle(barrel_radius)
            .extrude(barrel_length)  # Extrude 0.02m along x-axis
        )
        base.visual(
            mesh_from_cadquery(barrel_cq, f"hinge_barrel_{i}"),
            name=f"hinge_barrel_{i}",
        )

    # ---------- Screen Part ----------
    screen = model.part("screen")
    screen_width = 0.3  # Same width as base
    screen_height = 0.2  # 20cm tall (y-axis when open)
    screen_thickness = 0.005  # 5mm thick
    screen_fillet_radius = 0.001  # 1mm fillet for screen edges

    # 1. Screen strip with rounded edges, extends along +y from hinge line
    screen_cq = (
        cq.Workplane("XY")
        .rect(screen_width, screen_height)
        .extrude(screen_thickness)
        .edges("|Z")
        .fillet(screen_fillet_radius)
    )
    # Visual origin: center at (0, screen_height/2, 0.01) in screen's local frame
    # - y: screen extends from y=0 (hinge) to y=screen_height (front edge) in local y
    # - z: screen bottom face at base top (z=0.0225): 
    #   local z min = 0.01 - 0.0025 = 0.0075; world z min = 0.015 + 0.0075 = 0.0225 (matches base max z)
    screen.visual(
        mesh_from_cadquery(screen_cq, "screen_strip"),
        origin=Origin(xyz=(0.0, screen_height / 2, 0.01)),
        name="screen_strip",
    )

    # 2. Central Axle (rotates with screen)
    axle_radius = 0.002  # 2mm radius
    axle_length = 0.28  # 28cm long, runs through both barrels

    # Axle is along x-axis in screen's local frame (origin at hinge line)
    axle_cq = (
        cq.Workplane("YZ")  # Workplane normal to x-axis (local frame)
        .workplane(offset=-axle_length / 2)  # Start at x=-0.14m in local x
        .center(0, 0)  # Centered at y=0, z=0 in screen's local frame (hinge line)
        .circle(axle_radius)
        .extrude(axle_length)  # Extrude to x=0.14m in local x
    )
    screen.visual(
        mesh_from_cadquery(axle_cq, "central_axle"),
        name="central_axle",
    )

    # 3. End Caps (rotates with screen)
    end_cap_radius = 0.003  # 3mm radius
    end_cap_length = 0.002  # 2mm thick

    for i, x_pos in enumerate([-0.14, 0.14]):  # Ends of the axle in screen's local x
        end_cap_cq = (
            cq.Workplane("YZ")  # Workplane normal to x-axis (local frame)
            .workplane(offset=x_pos - end_cap_length / 2)  # Offset along local x
            .center(0, 0)  # Centered at y=0, z=0 in screen's local frame
            .circle(end_cap_radius)
            .extrude(end_cap_length)
        )
        screen.visual(
            mesh_from_cadquery(end_cap_cq, f"end_cap_{i}"),
            name=f"end_cap_{i}",
        )

    # ---------- Primary Articulation: Base to Screen ----------
    model.articulation(
        "base_to_screen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=screen,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),  # Hinge line at top center of base
        axis=(1.0, 0.0, 0.0),  # Rotate around x-axis (horizontal hinge)
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=2.094),  # 0-120 degrees
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    screen = object_model.get_part("screen")
    hinge = object_model.get_articulation("base_to_screen")

    # Allow intentional nested overlap: axle rotates inside hinge barrels
    ctx.allow_overlap(
        "base",
        "screen",
        elem_a="hinge_barrel_0",
        elem_b="central_axle",
        reason="Axle rotates inside left hinge barrel (intentional nested fit)",
    )
    ctx.allow_overlap(
        "base",
        "screen",
        elem_a="hinge_barrel_1",
        elem_b="central_axle",
        reason="Axle rotates inside right hinge barrel (intentional nested fit)",
    )

    # Proof checks for allowed overlaps: contact between barrels and axle
    ctx.expect_contact(
        "base", "screen",
        elem_a="hinge_barrel_0", elem_b="central_axle",
        name="left barrel contacts axle",
    )
    ctx.expect_contact(
        "base", "screen",
        elem_a="hinge_barrel_1", elem_b="central_axle",
        name="right barrel contacts axle",
    )

    # Validate motion limits (0 to ~120 degrees)
    ctx.check(
        "hinge motion limits are 0-120 degrees",
        hinge.motion_limits.lower == 0.0 and abs(hinge.motion_limits.upper - 2.094) < 0.001,
        details=f"lower={hinge.motion_limits.lower}, upper={hinge.motion_limits.upper}",
    )

    # Closed pose (q=0): screen sits on base
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            positive_link=screen, negative_link=base,
            axis="z",
            min_gap=0.0, max_gap=0.005,
            positive_elem="screen_strip", negative_elem="base_strip",
            name="closed screen sits on base with minimal gap",
        )
        ctx.check(
            "base has two hinge barrels",
            sum(1 for v in base.visuals if "hinge_barrel" in v.name) == 2,
            details="Incorrect number of hinge barrels on base",
        )

    # Open pose (q=120 degrees): screen lifts upward
    with ctx.pose({hinge: 2.094}):
        # Check screen strip's max z is above base's top (base max z = 0.0225)
        screen_strip_aabb = ctx.part_element_world_aabb(screen, elem="screen_strip")
        base_strip_aabb = ctx.part_element_world_aabb(base, elem="base_strip")
        screen_max_z = screen_strip_aabb[1][2]  # max z of screen strip
        base_max_z = base_strip_aabb[1][2]  # max z of base strip (0.0225)
        ctx.check(
            "open screen is above base",
            screen_max_z > base_max_z + 0.05,
            details=f"screen max z={screen_max_z:.3f}, base max z={base_max_z:.3f}",
        )
        # Re-check axle-barrel contact at open pose
        ctx.expect_contact(
            "base", "screen",
            elem_a="hinge_barrel_0", elem_b="central_axle",
            name="left barrel contacts axle at open pose",
        )

    # Validate visible details
    ctx.check(
        "screen has two end caps",
        sum(1 for v in screen.visuals if "end_cap" in v.name) == 2,
        details="Incorrect number of end caps on screen",
    )

    return ctx.report()


object_model = build_object_model()
