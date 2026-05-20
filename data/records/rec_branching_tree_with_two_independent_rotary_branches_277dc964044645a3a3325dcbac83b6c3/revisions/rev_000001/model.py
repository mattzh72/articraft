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
    model = ArticulatedObject(name="inspection_tree")

    base = model.part("base")
    base.visual(Box((0.4, 0.4, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="base_plate")
    base.visual(Box((0.1, 0.1, 1.05)), origin=Origin(xyz=(0.0, 0.0, 0.575)), name="spine")
    
    # Right bracket (lower)
    base.visual(Box((0.08, 0.04, 0.08)), origin=Origin(xyz=(0.09, 0.04, 0.55)), name="bracket_1")
    base.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.11, 0.01, 0.55), rpy=(1.570796, 0.0, 0.0)),
        name="pin_1",
    )
    
    # Left bracket (higher)
    base.visual(Box((0.08, 0.04, 0.08)), origin=Origin(xyz=(-0.09, -0.04, 0.85)), name="bracket_2")
    base.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(-0.11, -0.01, 0.85), rpy=(1.570796, 0.0, 0.0)),
        name="pin_2",
    )

    # Branch 1
    branch_1 = model.part("branch_1")
    branch_1.visual(
        Cylinder(radius=0.04, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.570796, 0.0, 0.0)),
        name="hub",
    )
    branch_1.visual(Box((0.4, 0.03, 0.04)), origin=Origin(xyz=(0.2, -0.01, 0.0)), name="arm")
    branch_1.visual(Box((0.06, 0.06, 0.06)), origin=Origin(xyz=(0.4, -0.01, -0.02)), name="sensor")

    model.articulation(
        "base_to_branch_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=branch_1,
        origin=Origin(xyz=(0.11, -0.03, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    # Branch 2
    branch_2 = model.part("branch_2")
    branch_2.visual(
        Cylinder(radius=0.04, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.570796, 0.0, 0.0)),
        name="hub",
    )
    branch_2.visual(Box((0.3, 0.03, 0.04)), origin=Origin(xyz=(-0.15, 0.01, 0.0)), name="arm")
    branch_2.visual(Box((0.06, 0.06, 0.06)), origin=Origin(xyz=(-0.3, 0.01, -0.02)), name="sensor")

    model.articulation(
        "base_to_branch_2",
        ArticulationType.REVOLUTE,
        parent=base,
        child=branch_2,
        origin=Origin(xyz=(-0.11, 0.03, 0.85)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    branch_1 = object_model.get_part("branch_1")
    branch_2 = object_model.get_part("branch_2")
    
    ctx.allow_overlap(
        base, branch_1,
        elem_a="pin_1", elem_b="hub",
        reason="The pin from the base is captured inside the branch hub to form the revolute joint."
    )
    ctx.allow_overlap(
        base, branch_2,
        elem_a="pin_2", elem_b="hub",
        reason="The pin from the base is captured inside the branch hub to form the revolute joint."
    )
    
    ctx.expect_gap(
        base, branch_1,
        axis="y",
        positive_elem="bracket_1",
        negative_elem="hub",
        min_gap=0.01,
        max_gap=0.03,
        name="branch_1_hub_clears_bracket"
    )
    ctx.expect_gap(
        branch_2, base,
        axis="y",
        positive_elem="hub",
        negative_elem="bracket_2",
        min_gap=0.01,
        max_gap=0.03,
        name="branch_2_hub_clears_bracket"
    )
    
    joint_1 = object_model.get_articulation("base_to_branch_1")
    joint_2 = object_model.get_articulation("base_to_branch_2")
    
    with ctx.pose({joint_1: 1.57, joint_2: -1.57}):
        ctx.expect_gap(
            branch_1, base,
            axis="z",
            positive_elem="arm",
            negative_elem="base_plate",
            min_gap=0.05,
            name="branch_1_clears_base_when_rotated"
        )
        ctx.expect_gap(
            branch_2, base,
            axis="z",
            positive_elem="arm",
            negative_elem="base_plate",
            min_gap=0.05,
            name="branch_2_clears_base_when_rotated"
        )

    return ctx.report()

object_model = build_object_model()