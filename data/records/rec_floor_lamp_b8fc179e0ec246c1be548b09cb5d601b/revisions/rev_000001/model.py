from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_floor_lamp")

    stand = model.part("stand")
    
    # Hub
    stand.visual(Cylinder(radius=0.04, height=0.05), origin=Origin(xyz=(0.0, 0.0, 0.4)), name="hub")
    
    # Post
    stand.visual(Cylinder(radius=0.02, height=1.0), origin=Origin(xyz=(0.0, 0.0, 0.9)), name="post")
    
    # Legs
    pitch = math.atan2(0.3, 0.4)
    length = math.hypot(0.3, 0.4)
    for i in range(3):
        yaw = math.radians(120 * i)
        stand.visual(
            Cylinder(radius=0.015, height=length),
            origin=Origin(
                xyz=(0.15 * math.cos(yaw), 0.15 * math.sin(yaw), 0.2),
                rpy=(0.0, -pitch, yaw)
            ),
            name=f"leg_{i}"
        )
        
    # Yoke attached near the top of the post
    # Yoke stem
    stand.visual(
        Cylinder(radius=0.015, height=0.05),
        origin=Origin(xyz=(0.045, 0.0, 1.35), rpy=(0.0, math.pi/2, 0.0)),
        name="yoke_stem"
    )
    # Yoke back plate
    stand.visual(
        Box((0.02, 0.14, 0.04)),
        origin=Origin(xyz=(0.08, 0.0, 1.35)),
        name="yoke_back"
    )
    # Yoke arms
    stand.visual(
        Box((0.14, 0.02, 0.04)),
        origin=Origin(xyz=(0.16, 0.06, 1.35)),
        name="yoke_arm_left"
    )
    stand.visual(
        Box((0.14, 0.02, 0.04)),
        origin=Origin(xyz=(0.16, -0.06, 1.35)),
        name="yoke_arm_right"
    )
    
    # Spotlight head
    head = model.part("spotlight_head")
    # Main body
    head.visual(
        Cylinder(radius=0.045, height=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi/2, 0.0)),
        name="head_body"
    )
    # Back dome
    head.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(-0.07, 0.0, 0.0)),
        name="head_back"
    )
    # Front bezel
    head.visual(
        Cylinder(radius=0.048, height=0.02),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, math.pi/2, 0.0)),
        name="head_bezel"
    )
    
    # Hinge pins
    head.visual(
        Cylinder(radius=0.005, height=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="hinge_pins"
    )
    
    # Articulation
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.21, 0.0, 1.35)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-math.pi/3, upper=math.pi/3)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    stand = object_model.get_part("stand")
    head = object_model.get_part("spotlight_head")
    
    ctx.allow_overlap(
        head, stand,
        elem_a="hinge_pins",
        elem_b="yoke_arm_left",
        reason="Hinge pins embed into the yoke arm."
    )
    ctx.allow_overlap(
        head, stand,
        elem_a="hinge_pins",
        elem_b="yoke_arm_right",
        reason="Hinge pins embed into the yoke arm."
    )
    
    ctx.expect_within(
        head, stand,
        axes="y",
        margin=0.005,
        name="head_fits_between_yoke_arms"
    )
    
    ctx.expect_gap(
        head, stand,
        axis="x",
        negative_elem="yoke_back",
        positive_elem="head_back",
        min_gap=0.001,
        name="head_clears_yoke_back_plate"
    )
    
    return ctx.report()

object_model = build_object_model()
