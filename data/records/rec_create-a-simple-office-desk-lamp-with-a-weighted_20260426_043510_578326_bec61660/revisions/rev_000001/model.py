import math
import cadquery as cq
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


def build_shade():
    outer = cq.Workplane("XY").circle(0.02).workplane(offset=0.12).circle(0.07).loft()
    inner = cq.Workplane("XY").workplane(offset=0.002).circle(0.018).workplane(offset=0.12).circle(0.068).loft()
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_lamp")

    # Base
    base = model.part("base")
    base.visual(Cylinder(radius=0.08, height=0.02), origin=Origin(xyz=(0, 0, 0.01)), name="pad")
    base.visual(Box((0.02, 0.01, 0.047)), origin=Origin(xyz=(0, -0.016, 0.0415)), name="bracket_left")
    base.visual(Box((0.02, 0.01, 0.047)), origin=Origin(xyz=(0, 0.016, 0.0415)), name="bracket_right")

    # Lower arm
    lower_arm = model.part("lower_arm")
    lower_arm.visual(Cylinder(radius=0.015, height=0.022), origin=Origin(rpy=(math.pi / 2, 0, 0)), name="hinge_bottom")
    lower_arm.visual(Box((0.02, 0.02, 0.29)), origin=Origin(xyz=(0, 0, 0.14)), name="arm")
    lower_arm.visual(Box((0.02, 0.011, 0.045)), origin=Origin(xyz=(0, -0.0155, 0.3025)), name="fork_left")
    lower_arm.visual(Box((0.02, 0.011, 0.045)), origin=Origin(xyz=(0, 0.0155, 0.3025)), name="fork_right")

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0, 0, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.5, upper=1.5),
    )

    # Upper arm
    upper_arm = model.part("upper_arm")
    upper_arm.visual(Cylinder(radius=0.015, height=0.020), origin=Origin(rpy=(math.pi / 2, 0, 0)), name="hinge_bottom")
    upper_arm.visual(Box((0.02, 0.02, 0.29)), origin=Origin(xyz=(0, 0, 0.14)), name="arm")
    upper_arm.visual(Box((0.02, 0.011, 0.045)), origin=Origin(xyz=(0, -0.0155, 0.3025)), name="fork_left")
    upper_arm.visual(Box((0.02, 0.011, 0.045)), origin=Origin(xyz=(0, 0.0155, 0.3025)), name="fork_right")

    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0, 0, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.5, upper=2.5),
    )

    # Head
    head = model.part("head")
    head.visual(Cylinder(radius=0.015, height=0.020), origin=Origin(rpy=(math.pi / 2, 0, 0)), name="hinge")
    head.visual(Box((0.02, 0.02, 0.025)), origin=Origin(xyz=(0, 0, 0.0075)), name="tab")
    head.visual(mesh_from_cadquery(build_shade(), "shade"), origin=Origin(xyz=(0, 0, 0.02)), name="shade")

    model.articulation(
        "upper_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(0, 0, 0.31)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")

    # Confirm contact between adjacent parts at rest
    ctx.expect_contact(base, lower_arm, elem_a="bracket_right", elem_b="hinge_bottom")
    ctx.expect_contact(lower_arm, upper_arm, elem_a="fork_right", elem_b="hinge_bottom")
    ctx.expect_contact(upper_arm, head, elem_a="fork_right", elem_b="hinge")

    # Test articulation poses
    with ctx.pose({"base_to_lower": 1.0}):
        upper_pos = ctx.part_world_position(upper_arm)
        ctx.check("lower_arm_tilts_forward", upper_pos is not None and upper_pos[0] > 0.2)

    with ctx.pose({"base_to_lower": 1.0, "lower_to_upper": -2.0}):
        head_pos = ctx.part_world_position(head)
        upper_pos = ctx.part_world_position(upper_arm)
        ctx.check("upper_arm_folds_back", head_pos is not None and upper_pos is not None and head_pos[0] < upper_pos[0])

    return ctx.report()


object_model = build_object_model()
