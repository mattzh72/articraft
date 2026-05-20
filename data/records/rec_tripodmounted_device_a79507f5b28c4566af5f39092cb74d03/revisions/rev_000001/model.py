import math
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
    model = ArticulatedObject(name="tripod_camera")
    
    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.03, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="crown_body"
    )
    
    for i in range(3):
        angle = math.radians(i * 120)
        c = math.cos(angle)
        s = math.sin(angle)
        
        # Hinge mount on the crown
        crown.visual(
            Box((0.013, 0.01, 0.02)),
            origin=Origin(xyz=(0.0315 * c, 0.0315 * s, 0.0), rpy=(0.0, 0.0, angle)),
            name=f"hinge_mount_{i}"
        )
        
        leg = model.part(f"leg_{i}")
        # Hinge barrel
        leg.visual(
            Cylinder(radius=0.01, length=0.02),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.radians(90), 0.0, 0.0)),
            name=f"leg_hinge_{i}"
        )
        # Leg body
        leg.visual(
            Box((0.015, 0.02, 0.20)),
            origin=Origin(xyz=(0.0075, 0.0, -0.10)),
            name=f"leg_body_{i}"
        )
        
        model.articulation(
            f"crown_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(0.040 * c, 0.040 * s, 0.0), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0), # Positive rotation folds outward
            motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=math.radians(60))
        )
        
    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.025, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)), # 0.015 above the joint origin (which is at Z=0.01)
        name="pan_head_body"
    )
    
    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.01)), # Top of crown
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0)
    )
    
    device_body = model.part("device_body")
    # Tilt plate visual
    device_body.visual(
        Box((0.04, 0.04, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="tilt_plate"
    )
    # Camera body visual
    device_body.visual(
        Box((0.10, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)), # 0.01 (plate) + 0.02 (half height)
        name="camera_shell"
    )
    # Camera lens visual
    device_body.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(0.0, 0.04, 0.03), rpy=(math.radians(90), 0.0, 0.0)),
        name="camera_lens"
    )
    
    model.articulation(
        "pan_head_to_device",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_body,
        origin=Origin(xyz=(0.0, 0.0, 0.03)), # Top of pan_head
        axis=(1.0, 0.0, 0.0), # Tilt around X axis
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-math.radians(90), upper=math.radians(90))
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    device_body = object_model.get_part("device_body")
    
    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            crown, leg,
            elem_a=f"hinge_mount_{i}",
            elem_b=f"leg_hinge_{i}",
            reason="Hinge mount is captured inside the leg barrel."
        )
        ctx.expect_overlap(
            crown, leg,
            elem_a=f"hinge_mount_{i}",
            elem_b=f"leg_hinge_{i}",
            axes="xy",
            name=f"leg_{i}_hinge_captured"
        )
        
    ctx.expect_contact(crown, pan_head, name="pan_head_sits_on_crown")
    ctx.expect_contact(pan_head, device_body, name="device_sits_on_pan_head")
    
    return ctx.report()

object_model = build_object_model()
