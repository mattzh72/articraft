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
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_mount")

    # 1. Wall Plate (Grounded)
    wall_plate = model.part("wall_plate")
    wall_plate.visual(Box((0.02, 0.1, 0.4)), origin=Origin(xyz=(0.01, 0, 0)), name="plate")
    wall_plate.visual(Box((0.06, 0.04, 0.01)), origin=Origin(xyz=(0.03, 0, 0.075)), name="top_tab")
    wall_plate.visual(Box((0.06, 0.04, 0.01)), origin=Origin(xyz=(0.03, 0, 0.025)), name="bottom_tab")

    # 2. Arm 1
    arm_1 = model.part("arm_1")
    arm_1.visual(Cylinder(radius=0.02, length=0.04), origin=Origin(xyz=(0, 0, 0)), name="hinge_barrel")
    arm_1.visual(Box((0.26, 0.04, 0.04)), origin=Origin(xyz=(0.15, 0, 0)), name="link")
    arm_1.visual(Box((0.06, 0.02, 0.1)), origin=Origin(xyz=(0.27, -0.03, -0.03)), name="clevis_back")
    arm_1.visual(Box((0.04, 0.06, 0.01)), origin=Origin(xyz=(0.3, -0.01, -0.025)), name="top_tab")
    arm_1.visual(Box((0.04, 0.06, 0.01)), origin=Origin(xyz=(0.3, -0.01, -0.075)), name="bottom_tab")

    # 3. Arm 2
    arm_2 = model.part("arm_2")
    arm_2.visual(Cylinder(radius=0.02, length=0.04), origin=Origin(xyz=(0, 0, 0)), name="start_barrel")
    arm_2.visual(Box((0.26, 0.04, 0.04)), origin=Origin(xyz=(0.15, 0, 0)), name="link")
    arm_2.visual(Cylinder(radius=0.02, length=0.04), origin=Origin(xyz=(0.3, 0, 0)), name="end_barrel")

    # 4. Head Swivel
    head_swivel = model.part("head_swivel")
    head_swivel.visual(Box((0.06, 0.04, 0.01)), origin=Origin(xyz=(0.03, 0, 0.025)), name="top_tab")
    head_swivel.visual(Box((0.06, 0.04, 0.01)), origin=Origin(xyz=(0.03, 0, -0.025)), name="bottom_tab")
    head_swivel.visual(Box((0.04, 0.04, 0.06)), origin=Origin(xyz=(0.04, 0, 0)), name="back_plate")
    head_swivel.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.07, 0, 0), rpy=(math.pi / 2, 0, 0)),
        name="tilt_barrel"
    )

    # 5. Head Tilt
    head_tilt = model.part("head_tilt")
    head_tilt.visual(Box((0.03, 0.01, 0.03)), origin=Origin(xyz=(0.005, 0.025, 0)), name="left_tab")
    head_tilt.visual(Box((0.03, 0.01, 0.03)), origin=Origin(xyz=(0.005, -0.025, 0)), name="right_tab")
    head_tilt.visual(Box((0.01, 0.15, 0.15)), origin=Origin(xyz=(0.025, 0, 0)), name="vesa_plate")

    # Articulations
    model.articulation(
        "wall_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=arm_1,
        origin=Origin(xyz=(0.04, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi / 2, upper=math.pi / 2),
    )

    model.articulation(
        "arm_1_to_arm_2",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=arm_2,
        origin=Origin(xyz=(0.3, 0.0, -0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=math.pi),
    )

    model.articulation(
        "arm_2_to_head_swivel",
        ArticulationType.REVOLUTE,
        parent=arm_2,
        child=head_swivel,
        origin=Origin(xyz=(0.3, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi / 2, upper=math.pi / 2),
    )

    model.articulation(
        "head_swivel_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=head_tilt,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.26, upper=0.26),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.expect_contact("wall_plate", "arm_1", elem_a="top_tab", elem_b="hinge_barrel")
    ctx.expect_contact("arm_1", "arm_2", elem_a="top_tab", elem_b="start_barrel")
    ctx.expect_contact("arm_2", "head_swivel", elem_a="end_barrel", elem_b="top_tab")
    ctx.expect_contact("head_swivel", "head_tilt", elem_a="tilt_barrel", elem_b="left_tab")

    return ctx.report()

object_model = build_object_model()
