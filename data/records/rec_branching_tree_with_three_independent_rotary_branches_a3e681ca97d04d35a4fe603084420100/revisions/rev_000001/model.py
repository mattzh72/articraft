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
    model = ArticulatedObject(name="tooling_tree")
    
    pedestal = model.part("pedestal")
    # Base
    pedestal.visual(
        Cylinder(radius=0.3, height=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base"
    )
    # Column
    pedestal.visual(
        Cylinder(radius=0.08, height=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="column"
    )
    
    reaches = [0.3, 0.4, 0.5]
    heights = [0.4, 0.6, 0.8]
    angles_deg = [0, 120, 240]
    
    for i in range(3):
        angle_rad = math.radians(angles_deg[i])
        reach = reaches[i]
        h = heights[i]
        
        # Bracket on the pedestal to support the branch
        bx = 0.1 * math.cos(angle_rad)
        by = 0.1 * math.sin(angle_rad)
        pedestal.visual(
            Box((0.2, 0.08, 0.08)),
            origin=Origin(xyz=(bx, by, h), rpy=(0, 0, angle_rad)),
            name=f"bracket_{i}"
        )
        
        # Branch part
        branch = model.part(f"branch_{i}")
        
        # Hub
        branch.visual(
            Cylinder(radius=0.04, height=0.06),
            origin=Origin(xyz=(0.0, 0.0, 0.03)),
            name=f"hub_{i}"
        )
        
        # Arm
        branch.visual(
            Box((reach, 0.04, 0.04)),
            origin=Origin(xyz=(reach / 2, 0.0, 0.03)),
            name=f"arm_{i}"
        )
        
        # End face
        branch.visual(
            Box((0.02, 0.08, 0.08)),
            origin=Origin(xyz=(reach + 0.01, 0.0, 0.03)),
            name=f"end_face_{i}"
        )
        
        # Joint
        jx = 0.16 * math.cos(angle_rad)
        jy = 0.16 * math.sin(angle_rad)
        jz = h + 0.04  # top of bracket
        
        model.articulation(
            f"pedestal_to_branch_{i}",
            ArticulationType.REVOLUTE,
            parent=pedestal,
            child=branch,
            origin=Origin(xyz=(jx, jy, jz), rpy=(0, 0, angle_rad)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi/2, upper=math.pi/2)
        )
        
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    pedestal = object_model.get_part("pedestal")
    
    for i in range(3):
        branch = object_model.get_part(f"branch_{i}")
        bracket_name = f"bracket_{i}"
        hub_name = f"hub_{i}"
        
        # Check contact between bracket and hub at rest
        ctx.expect_contact(pedestal, branch, elem_a=bracket_name, elem_b=hub_name, name=f"branch_{i}_rests_on_bracket")
        
        # Test a rotated pose
        joint = object_model.get_articulation(f"pedestal_to_branch_{i}")
        with ctx.pose({joint: math.pi/3}):
            ctx.expect_contact(pedestal, branch, elem_a=bracket_name, elem_b=hub_name, name=f"branch_{i}_remains_supported_in_rotated_pose")

    return ctx.report()


object_model = build_object_model()