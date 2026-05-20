from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    place_on_face,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_scanner_pan_module")

    # ==================== BASE PART ====================
    base = model.part("base")

    # --- Base main body (low circular base with cable notch and rubber feet) ---
    base_wp = cq.Workplane("XY").circle(0.1).extrude(0.03)  # 0.2m diameter, 0.03m height

    # Add rubber feet as part of base body (small cylinders on bottom)
    for x, y in [(0.09, 0.0), (0.0, 0.09), (-0.09, 0.0), (0.0, -0.09)]:
        base_wp = (
            base_wp
            .union(
                cq.Workplane("XY")
                .workplane(offset=-0.005)  # Below base bottom
                .center(x, y)
                .circle(0.01)  # 0.02m diameter
                .extrude(0.005)  # 0.005m height (protruding below base)
            )
        )

    # Cable relief notch: create a small rectangular cutout at the edge
    notch_wp = (
        cq.Workplane("XY")
        .workplane(offset=0.0125)  # Middle height of base
        .center(0.07, 0.0)  # Position near edge
        .rect(0.08, 0.01)  # Wide rectangle that extends past edge
        .extrude(0.005)  # Through-thickness cut
    )
    base_wp = base_wp.cut(notch_wp)
    base_wp = base_wp.combine()  # Combine all components

    base_body_mesh = mesh_from_cadquery(base_wp, "base_body")
    base.visual(
        base_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),  # Center of base body (bottom at z=0)
        name="base_body",
        color=(0.9, 0.9, 0.9),  # Light gray
    )

    # --- Central pivot boss (fixed to base, supports upper plate) ---
    boss_wp = (
        cq.Workplane("XY")
        .circle(0.01)  # 0.02m diameter
        .extrude(0.01)  # 0.01m height
    )
    boss_mesh = mesh_from_cadquery(boss_wp, "pivot_boss")
    base.visual(
        boss_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),  # Top of base body + half boss height
        name="pivot_boss",
        color=(0.9, 0.9, 0.9),
    )

    # ==================== UPPER PLATE PART ====================
    upper_plate = model.part("upper_plate")

    # Upper plate: 0.18m diameter, 0.02m height, central hole for pivot boss
    plate_wp = cq.Workplane("XY").circle(0.09).extrude(0.02)  # 0.18m diameter
    plate_wp = plate_wp.cut(
        cq.Workplane("XY").circle(0.0105).extrude(0.02)  # 0.021m diameter hole (clearance fit)
    )

    plate_mesh = mesh_from_cadquery(plate_wp, "upper_plate_shell")
    upper_plate.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Bottom of plate at z=0.045 (boss top)
        name="plate",
        color=(0.2, 0.2, 0.2),  # Dark gray
    )

    # ==================== ARTICULATION (YAW AXIS) ====================
    model.articulation(
        "base_to_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),  # Pivot point (center of boss)
        axis=(0.0, 0.0, 1.0),  # Z-axis rotation (yaw)
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.0,
            lower=0.0,
            upper=4.71238898,  # ~270 degrees (3π/2) to prevent cable overtwist
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_plate")
    joint = object_model.get_articulation("base_to_upper")

    # ==================== OVERLAP / NESTING CHECKS ====================
    # Pivot boss is nested inside upper plate's central hole (clearance fit)
    # Proof: boss is within plate hole (radial constraint)
    ctx.expect_within(
        "base",
        "upper_plate",
        axes="xy",
        inner_elem="pivot_boss",
        outer_elem="plate",
        margin=0.001,
        name="pivot boss contained within plate hole",
    )

    # Proof that plate sits on boss (axial support)
    ctx.expect_gap(
        "upper_plate",
        "base",
        axis="z",
        negative_elem="pivot_boss",
        positive_elem="plate",
        max_penetration=0.001,
        name="plate sits on pivot boss with minimal gap",
    )

    # ==================== MAIN MECHANISM TESTS ====================
    rest_pos = ctx.part_world_position(upper)
    with ctx.pose({joint: 0.0}):
        ctx.expect_origin_distance(
            base, upper, axes="xy", min_dist=0.0, max_dist=0.01, name="upper plate centered on base at rest"
        )

    with ctx.pose({joint: 4.71238898}):  # Full rotation limit
        upper_pos = ctx.part_world_position(upper)
        ctx.check(
            "upper plate rotates around Z axis (no vertical translation)",
            abs(rest_pos[2] - upper_pos[2]) < 0.001,
            details=f"Rest Z: {rest_pos[2]:.3f}, Rotated Z: {upper_pos[2]:.3f}",
        )
        ctx.expect_origin_distance(
            base, upper, axes="xy", min_dist=0.0, max_dist=0.01, name="upper plate remains centered at full rotation"
        )

    # ==================== VISIBLE DETAIL TESTS ====================
    # Base dimensions (0.2m diameter, 0.03m height)
    base_aabb = ctx.part_world_aabb(base)
    if base_aabb:
        base_min, base_max = base_aabb
        base_width = base_max[0] - base_min[0]
        base_depth = base_max[1] - base_min[1]
        base_height = base_max[2] - base_min[2]
        ctx.check(
            "base has correct diameter (~0.2m)",
            abs(base_width - 0.2) < 0.01,
            details=f"Base dims: {base_width:.3f}x{base_depth:.3f}x{base_height:.3f}m",
        )

    # Upper plate dimensions (0.18m diameter, 0.02m height)
    upper_aabb = ctx.part_world_aabb(upper)
    if upper_aabb:
        upper_min, upper_max = upper_aabb
        upper_width = upper_max[0] - upper_min[0]
        upper_depth = upper_max[1] - upper_min[1]
        upper_height = upper_max[2] - upper_min[2]
        ctx.check(
            "upper plate has correct diameter (~0.18m)",
            abs(upper_width - 0.18) < 0.01,
            details=f"Upper plate dims: {upper_width:.3f}x{upper_depth:.3f}x{upper_height:.3f}m",
        )

    return ctx.report()


object_model = build_object_model()
