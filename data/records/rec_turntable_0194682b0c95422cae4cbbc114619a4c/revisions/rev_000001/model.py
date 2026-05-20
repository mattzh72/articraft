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
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_turntable")

    # --- Register materials ---
    base_mat = model.material("base_material", rgba=(0.2, 0.2, 0.2, 1.0))  # Dark gray plastic
    rubber_mat = model.material("rubber_material", rgba=(0.0, 0.0, 0.0, 1.0))  # Black rubber
    switch_mat = model.material("switch_material", rgba=(1.0, 0.0, 0.0, 1.0))  # Red switch
    platter_mat = model.material("platter_material", rgba=(0.8, 0.8, 0.8, 1.0))  # Light gray plastic
    cap_mat = model.material("cap_material", rgba=(0.7, 0.7, 0.7, 1.0))  # Silver center cap

    # --- Root part: Base ---
    base = model.part("base")

    # Low circular base shell (0.3m diameter = 0.15m radius, 0.05m tall)
    base.visual(
        Cylinder(radius=0.15, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_shell",
        material=base_mat,
    )

    # Rubber feet (4x small cylinders on bottom of base)
    foot_offset = 0.14  # Distance from center to each foot (base radius 0.15m - 0.01m foot radius)
    for i, (x, y) in enumerate([
        (foot_offset, 0.0),
        (0.0, foot_offset),
        (-foot_offset, 0.0),
        (0.0, -foot_offset),
    ]):
        base.visual(
            Cylinder(radius=0.005, length=0.01),
            origin=Origin(xyz=(x, y, -0.005)),  # Extends from z=-0.01 to z=0 (base bottom surface)
            name=f"rubber_foot_{i}",
            material=rubber_mat,
        )

    # Power switch detail on side of base
    base.visual(
        Box((0.02, 0.01, 0.01)),  # 2cm x 1cm x 1cm
        origin=Origin(xyz=(0.16, 0.0, 0.025)),  # Mounted on right side of base (0.15m radius + 0.01m half-width)
        name="power_switch",
        material=switch_mat,
    )

    # --- Rotating part: Top Disk ---
    top_disk = model.part("top_disk")

    # Main platter with beveled rim (CadQuery for chamfered edge)
    platter_cq = (
        cq.Workplane("XY")
        .cylinder(height=0.02, radius=0.14)  # 0.28m diameter, 0.02m thick
        .faces("+Z")  # Top face
        .edges()  # Outer edges of top face
        .chamfer(0.002)  # 2mm beveled rim
    )
    top_disk.visual(
        mesh_from_cadquery(platter_cq, "platter"),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Centered 0.01m above top_disk frame (sits on base top at z=0.05)
        name="platter",
        material=platter_mat,
    )

    # Center cap on platter
    top_disk.visual(
        Cylinder(radius=0.02, length=0.01),  # 4cm diameter = 2cm radius, 1cm tall
        origin=Origin(xyz=(0.0, 0.0, 0.025)),  # Sits on top of platter (platter top at z=0.02 relative to top_disk)
        name="center_cap",
        material=cap_mat,
    )

    # --- Primary articulation: Vertical yaw joint (continuous rotation) ---
    model.articulation(
        "base_to_top",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_disk,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Top surface of base
        axis=(0.0, 0.0, 1.0),  # Vertical Z-axis rotation
        motion_limits=MotionLimits(effort=1.0, velocity=6.28),  # ~1 rev/sec max speed
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_disk = object_model.get_part("top_disk")
    joint = object_model.get_articulation("base_to_top")

    # Rest pose (q=0) checks
    with ctx.pose({joint: 0.0}):
        ctx.expect_contact(top_disk, base, name="rest_pose_contact")
        ctx.expect_within(top_disk, base, axes="xy", margin=0.01, name="platter_centered")

    # Rotated pose (q=π/2) checks
    with ctx.pose({joint: 1.5708}):
        top_pos = ctx.part_world_position(top_disk)
        ctx.check(
            "rotated_pose_centered",
            abs(top_pos[0]) < 0.01 and abs(top_pos[1]) < 0.01,
            details=f"Top disk position at π/2: {top_pos}",
        )

    # Visible detail checks
    base_visuals = [v.name for v in base.visuals]
    ctx.check(
        "base_has_rubber_feet",
        sum(1 for v in base_visuals if "rubber_foot" in v) == 4,
        details=f"Base visuals: {base_visuals}",
    )
    ctx.check(
        "base_has_switch",
        "power_switch" in base_visuals,
        details=f"Base visuals: {base_visuals}",
    )

    top_visuals = [v.name for v in top_disk.visuals]
    ctx.check(
        "top_has_platter",
        "platter" in top_visuals,
        details=f"Top visuals: {top_visuals}",
    )
    ctx.check(
        "top_has_center_cap",
        "center_cap" in top_visuals,
        details=f"Top visuals: {top_visuals}",
    )

    # Mechanism check
    ctx.check(
        "joint_is_continuous_yaw",
        joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 0.0, 1.0),
        details=f"Joint type: {joint.articulation_type}, axis: {joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
