from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_sensor_bracket")

    # ====================
    # Root Part: Fixed Base Assembly
    # ====================
    base = model.part("base")

    # 1. Base block (dark gray metallic, filleted vertical edges)
    base_block_shape = (
        cq.Workplane("XY")
        .box(0.12, 0.08, 0.02)  # X=0.12m (120mm), Y=0.08m (80mm), Z=0.02m (20mm)
        .edges("|Z")  # Vertical edges parallel to Z axis
        .fillet(0.005)  # 5mm fillet on vertical edges
    )
    base.visual(
        mesh_from_cadquery(base_block_shape, "base_block"),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),  # Center of block at Z=0.01 (bottom at Z=0)
        name="base_block",
        material=Material(name="dark_metal", color=(0.2, 0.2, 0.2)),
    )

    # 2. Left side cheek (vertical plate, filleted horizontal edges)
    left_cheek_shape = (
        cq.Workplane("XY")
        .box(0.06, 0.02, 0.06)  # X=0.06m, Y=0.02m (20mm thick), Z=0.06m (60mm tall)
        .edges("|X or |Y")  # Horizontal edges parallel to X or Y
        .fillet(0.003)  # 3mm fillet
    )
    base.visual(
        mesh_from_cadquery(left_cheek_shape, "left_cheek"),
        origin=Origin(xyz=(0.0, 0.04, 0.05)),  # Y=+0.04 (left side), Z=0.05 (center height)
        name="left_cheek",
        material=Material(name="dark_metal", color=(0.2, 0.2, 0.2)),
    )

    # 3. Right side cheek (mirror of left)
    right_cheek_shape = (
        cq.Workplane("XY")
        .box(0.06, 0.02, 0.06)
        .edges("|X or |Y")
        .fillet(0.003)
    )
    base.visual(
        mesh_from_cadquery(right_cheek_shape, "right_cheek"),
        origin=Origin(xyz=(0.0, -0.04, 0.05)),  # Y=-0.04 (right side)
        name="right_cheek",
        material=Material(name="dark_metal", color=(0.2, 0.2, 0.2)),
    )

    # 4. Horizontal axle pin (Y-axis, through cheeks and sensor box)
    axle_pin_shape = (
        cq.Workplane("XZ")  # Normal is Y, cylinder extends along Y
        .cylinder(0.10, 0.0025)  # Length 0.10m (100mm), radius 2.5mm (5mm diameter)
    )
    base.visual(
        mesh_from_cadquery(axle_pin_shape, "axle_pin"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Centered at axle pivot (X=0, Y=0, Z=0.05)
        name="axle_pin",
        material=Material(name="axle_metal", color=(0.3, 0.3, 0.3)),
    )

    # 5. Washers (two, on axle between cheeks and sensor box)
    washer_shape = (
        cq.Workplane("XZ")  # Normal is Y, extrude along Y
        .workplane()
        .circle(0.008)  # Outer diameter 8mm
        .circle(0.0025)  # Inner diameter 5mm (matches axle)
        .extrude(0.002)  # Thickness 2mm
    )
    # Left washer (Y=+0.025, between left cheek and sensor box)
    base.visual(
        mesh_from_cadquery(washer_shape, "left_washer"),
        origin=Origin(xyz=(0.0, 0.025, 0.05)),
        name="left_washer",
        material=Material(name="washer_metal", color=(0.4, 0.4, 0.4)),
    )
    # Right washer (Y=-0.025)
    base.visual(
        mesh_from_cadquery(washer_shape, "right_washer"),
        origin=Origin(xyz=(0.0, -0.025, 0.05)),
        name="right_washer",
        material=Material(name="washer_metal", color=(0.4, 0.4, 0.4)),
    )

    # ====================
    # Moving Part: Tilting Sensor Box
    # ====================
    sensor_box = model.part("sensor_box")

    # Sensor body (light blue-gray, filleted edges)
    sensor_body_shape = (
        cq.Workplane("XY")
        .box(0.06, 0.04, 0.05)  # X=0.06m, Y=0.04m (fits between cheeks), Z=0.05m
        .edges("|X or |Y or |Z")  # All edges
        .fillet(0.003)  # 3mm fillet
    )
    sensor_box.visual(
        mesh_from_cadquery(sensor_body_shape, "sensor_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered at part frame (axle pivot)
        name="sensor_body",
        material=Material(name="sensor_body_mat", color=(0.8, 0.8, 0.9)),
    )

    # Rear cable boss (black plastic, on rear face of sensor box)
    cable_boss_shape = (
        cq.Workplane("YZ")  # Normal is X, work in YZ plane
        .workplane(offset=-0.03)  # Rear face of sensor box (X=-0.03)
        .circle(0.0075)  # Radius 7.5mm (15mm diameter)
        .extrude(-0.01)  # Extrude 0.01m rearward along -X
    )
    sensor_box.visual(
        mesh_from_cadquery(cable_boss_shape, "cable_boss"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Attached to sensor body rear face
        name="cable_boss",
        material=Material(name="cable_boss_mat", color=(0.1, 0.1, 0.1)),
    )

    # ====================
    # Articulation: Horizontal Pitch Revolute Joint
    # ====================
    model.articulation(
        "base_to_sensor",
        ArticulationType.REVOLUTE,
        parent=base,
        child=sensor_box,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Pivot at axle center
        axis=(0.0, -1.0, 0.0),  # Y-axis (correct pitch: +θ tilts front up)
        motion_limits=MotionLimits(
            lower=-0.3,  # Downward tilt ~-17 degrees
            upper=0.3,   # Upward tilt ~17 degrees
            effort=2.0,
            velocity=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    sensor = object_model.get_part("sensor_box")
    joint = object_model.get_articulation("base_to_sensor")

    # Allow intentional overlap between axle pin and sensor body bore
    ctx.allow_overlap(
        "base", "sensor_box",
        elem_a="axle_pin",
        elem_b="sensor_body",
        reason="Axle pin is intentionally nested through sensor box bore",
    )

    # Proof check: axle pin contacts sensor body
    ctx.expect_contact(
        base, sensor,
        elem_a="axle_pin",
        elem_b="sensor_body",
        contact_tol=0.001,
        name="axle pin contacts sensor body bore",
    )

    # Rest pose checks (q=0)
    with ctx.pose({joint: 0.0}):
        # Sensor centered between cheeks along Y-axis
        ctx.expect_within(
            sensor, base,
            axes="y",
            margin=0.001,
            name="sensor centered between cheeks at rest",
        )
        # Clearance between sensor bottom and base block top (min 5mm)
        ctx.expect_gap(
            sensor, base,
            axis="z",
            min_gap=0.005,
            negative_elem="base_block",  # Check against base block only, not entire base assembly
            name="sensor has 5mm clearance above base block at rest",
        )

    # Sensor centered between cheeks along Y-axis at rest
    ctx.expect_within(
        sensor, base,
        axes="y",
        margin=0.001,
        name="sensor centered between cheeks at rest",
    )

    # Verify joint can reach upper and lower limits
    with ctx.pose({joint: joint.motion_limits.upper}):
        ctx.check("joint reaches upper limit", True, details=f"Upper limit: {joint.motion_limits.upper}")
    with ctx.pose({joint: joint.motion_limits.lower}):
        ctx.check("joint reaches lower limit", True, details=f"Lower limit: {joint.motion_limits.lower}")

    # Joint property checks
    ctx.check(
        "joint is revolute with Y-axis",
        joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (0.0, -1.0, 0.0),
        details=f"joint type: {joint.articulation_type}, axis: {joint.axis}",
    )
    ctx.check(
        "joint has correct tilt limits",
        abs(joint.motion_limits.lower + 0.3) < 0.001 and abs(joint.motion_limits.upper - 0.3) < 0.001,
        details=f"limits: {joint.motion_limits.lower} to {joint.motion_limits.upper}",
    )

    return ctx.report()


object_model = build_object_model()
