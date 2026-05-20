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
    model = ArticulatedObject(name="camera_flash")

    body = model.part("body")
    body.visual(
        Box((0.018, 0.018, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        name="foot",
    )
    body.visual(
        Box((0.060, 0.040, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        name="body_case",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        name="neck_cylinder",
    )

    head = model.part("head")
    head.visual(
        Box((0.065, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, -0.040, 0.0175)),
        name="head_case",
    )
    head.visual(
        Box((0.055, 0.002, 0.025)),
        origin=Origin(xyz=(0.0, -0.081, 0.0175)),
        name="flash_window",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.015, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_neck")
    tilt_joint = object_model.get_articulation("neck_to_head")

    ctx.expect_gap(neck, body, axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_gap(head, neck, axis="z", max_gap=0.001, max_penetration=0.001)

    head_aabb_rest = ctx.part_world_aabb(head)

    with ctx.pose({tilt_joint: 1.57}):
        head_aabb_tilted = ctx.part_world_aabb(head)
        if head_aabb_rest and head_aabb_tilted:
            ctx.check(
                "head_tilts_upward",
                head_aabb_tilted[1][2] > head_aabb_rest[1][2] + 0.03,
                "The head should raise significantly along Z when tilted by 90 degrees."
            )
            ctx.check(
                "head_clears_neck_when_tilted",
                head_aabb_tilted[0][2] >= ctx.part_world_aabb(neck)[1][2] - 0.001,
                "The tilted head should remain above or touching the neck, not penetrating."
            )

    return ctx.report()

object_model = build_object_model()
