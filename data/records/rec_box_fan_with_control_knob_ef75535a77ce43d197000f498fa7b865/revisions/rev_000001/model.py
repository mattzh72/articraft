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
    place_on_surface,
    place_on_face,
    proud_for_flush_mount,
)
import cadquery as cq

def make_clamp_base():
    # A C-clamp shape.
    # We will build it on the XY plane and extrude.
    # The clamp sits on the desk, so the gap is around Z=0.
    # Let's make it open towards +Y.
    # Back spine along -Y.
    # Top jaw at Z = +0.03, bottom jaw at Z = -0.03.
    # Thickness = 0.015.
    
    # Profile in YZ plane, extruded along X.
    # Let's just use boolean boxes for simplicity.
    base = cq.Workplane("XY").box(0.06, 0.08, 0.08)
    # Cut out the inner part to make a C shape.
    # The opening will be on the +Y side.
    cutout = cq.Workplane("XY").center(0, 0.015).box(0.07, 0.06, 0.05)
    base = base.cut(cutout)
    
    # Add a small cylinder boss on top for the shoulder joint.
    boss = cq.Workplane("XY").workplane(offset=0.04).cylinder(0.01, 0.02)
    base = base.union(boss)
    return base

def make_capsule(length, radius=0.01):
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .faces(">Z").sphere(radius)
        .faces("<Z").sphere(radius)
    )

def make_fan_head():
    # A round fan head.
    # Let's make it a cylinder along Y axis, so it faces +Y.
    head = (
        cq.Workplane("XZ")
        .cylinder(0.05, 0.06)  # radius 0.06, length 0.05 along Y
    )
    # Hollow it out slightly from the front (+Y)
    cutout = (
        cq.Workplane("XZ")
        .workplane(offset=0.005)
        .cylinder(0.05, 0.055)
    )
    head = head.cut(cutout)
    
    # Add a mounting bracket on the bottom (-Z)
    bracket = (
        cq.Workplane("XY")
        .workplane(offset=-0.06)
        .box(0.02, 0.02, 0.02)
    )
    head = head.union(bracket)
    return head

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_gooseneck_usb_fan")

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_clamp_base(), "clamp_base_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="clamp_base",
    )

    # The top of the clamp base boss is at Z = 0.04 + 0.01/2 = 0.045.
    # Wait, boss is cylinder(0.01, 0.02).
    # In CadQuery, cylinder(height, radius) -> wait, cylinder signature is `cylinder(height, radius)`.
    # Let me double check CadQuery cylinder signature. It's `cylinder(distance, radius)`.
    # So `cylinder(0.01, 0.02)` means height 0.01, radius 0.02.
    # The workplane was offset=0.04. The cylinder is centered on the workplane.
    # So it goes from Z=0.035 to Z=0.045.
    # The top of the base box is at Z=0.04. So the boss sticks out to 0.045.
    
    arm_radius = 0.01
    arm_length = 0.12
    
    arm_1 = model.part("arm_1")
    # Shift the visual so the bottom end is at the part origin.
    arm_1.visual(
        mesh_from_cadquery(make_capsule(arm_length, arm_radius), "arm_1_geom"),
        origin=Origin(xyz=(0.0, 0.0, arm_length / 2)),
        name="arm_1_shell",
    )

    arm_2 = model.part("arm_2")
    arm_2.visual(
        mesh_from_cadquery(make_capsule(arm_length, arm_radius), "arm_2_geom"),
        origin=Origin(xyz=(0.0, 0.0, arm_length / 2)),
        name="arm_2_shell",
    )

    arm_3 = model.part("arm_3")
    arm_3.visual(
        mesh_from_cadquery(make_capsule(arm_length, arm_radius), "arm_3_geom"),
        origin=Origin(xyz=(0.0, 0.0, arm_length / 2)),
        name="arm_3_shell",
    )

    fan_head = model.part("fan_head")
    fan_head.visual(
        mesh_from_cadquery(make_fan_head(), "fan_head_geom"),
        # The bracket bottom is at Z = -0.07. We want the origin to be at the bracket bottom.
        # Wait, bracket is offset=-0.06, size=0.02. So it goes from -0.05 to -0.07.
        # Let's shift the visual so the origin is exactly at Z=-0.07.
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        name="fan_head_shell",
    )

    dial = model.part("fan_dial")
    dial.visual(
        Cylinder(radius=0.015, length=0.01),
        # Origin at the front face of the dial so it sits flush on the rear face.
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="dial_shell",
    )

    # Articulations
    # Shoulder joint: base to arm_1
    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_1,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 1.0, 0.0), # Pitch joint
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=1.0),
    )

    # Elbow 1: arm_1 to arm_2
    model.articulation(
        "elbow_1",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=arm_2,
        origin=Origin(xyz=(0.0, 0.0, arm_length)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.5, upper=1.5),
    )

    # Elbow 2: arm_2 to arm_3
    model.articulation(
        "elbow_2",
        ArticulationType.REVOLUTE,
        parent=arm_2,
        child=arm_3,
        origin=Origin(xyz=(0.0, 0.0, arm_length)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.5, upper=1.5),
    )

    # Wrist: arm_3 to fan_head
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=arm_3,
        child=fan_head,
        origin=Origin(xyz=(0.0, 0.0, arm_length)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=1.0),
    )

    # Dial joint: fan_head to dial
    # The fan head cylinder is along Y. The rear face is at Y = -0.025.
    model.articulation(
        "dial_spin",
        ArticulationType.REVOLUTE,
        parent=fan_head,
        child=dial,
        # Place it on the rear face. The dial's origin is at its flush face.
        # So we place it at Y = -0.025, and orient it so its local Z points along -Y.
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(-1.5708, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.1, velocity=1.0, lower=-3.14, upper=3.14),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allow overlaps for the overlapping capsule joints
    ctx.allow_overlap("base", "arm_1", reason="Capsule base overlaps with mounting boss")
    ctx.allow_overlap("arm_1", "arm_2", reason="Capsule ends overlap to form a continuous bendable joint")
    ctx.allow_overlap("arm_2", "arm_3", reason="Capsule ends overlap to form a continuous bendable joint")
    ctx.allow_overlap("arm_3", "fan_head", reason="Capsule end overlaps with fan head bracket")
    ctx.allow_overlap("fan_head", "fan_dial", reason="Dial is flush mounted on fan head")

    return ctx.report()

object_model = build_object_model()
