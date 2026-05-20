from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
import math

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microphone_boom", assets=ASSETS)

    # Materials
    matte_metal = model.material("matte_metal", color=(0.15, 0.15, 0.15, 1.0))
    plastic_collar = model.material("plastic_collar", color=(0.05, 0.05, 0.05, 1.0))
    mic_finish = model.material("mic_finish", color=(0.7, 0.7, 0.7, 1.0))
    mic_grille = model.material("mic_grille", color=(0.2, 0.2, 0.2, 1.0))

    # Constants
    ARM_L = 0.4
    ARM_W = 0.02
    ARM_T = 0.01
    HUB_R = 0.018
    HUB_W = 0.024
    BLOCK_H = 0.06

    # --- Base Link (Clamp + Mounting Block + Brackets) ---
    base = model.part("base")
    # Clamp Top
    base.visual(Box((0.08, 0.06, 0.01)), origin=Origin(xyz=(0.0, 0.0, -0.005)), material=matte_metal)
    # Clamp Back
    base.visual(Box((0.01, 0.06, 0.08)), origin=Origin(xyz=(-0.045, 0.0, -0.04)), material=matte_metal)
    # Clamp Bottom (simplified screw)
    base.visual(Box((0.04, 0.04, 0.01)), origin=Origin(xyz=(-0.02, 0.0, -0.075)), material=matte_metal)
    # Mounting Block
    base.visual(Box((0.04, 0.04, BLOCK_H)), origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2)), material=matte_metal)
    # Shoulder Brackets
    bracket_h = 0.04
    base.visual(Box((0.04, 0.005, bracket_h)), origin=Origin(xyz=(0.0, 0.015, BLOCK_H + bracket_h / 2)), material=matte_metal, name="shoulder_bracket_l")
    base.visual(Box((0.04, 0.005, bracket_h)), origin=Origin(xyz=(0.0, -0.015, BLOCK_H + bracket_h / 2)), material=matte_metal, name="shoulder_bracket_r")

    base.inertial = Inertial.from_geometry(Box((0.08, 0.06, 0.15)), mass=2.0, origin=Origin(xyz=(0.0, 0.0, 0.0)))

    # --- Lower Arm ---
    lower_arm = model.part("lower_arm")
    # Shoulder Hub (Fits between base brackets)
    lower_arm.visual(Cylinder(radius=HUB_R, length=0.024), origin=Origin(rpy=(math.pi / 2, 0, 0)), material=matte_metal, name="shoulder_hub")
    # Adjustment Collar (Plastic)
    lower_arm.visual(Cylinder(radius=HUB_R + 0.002, length=0.01), origin=Origin(xyz=(0, 0.016, 0), rpy=(math.pi / 2, 0, 0)), material=plastic_collar)
    # Main Beam
    lower_arm.visual(Box((ARM_L, ARM_W, ARM_T)), origin=Origin(xyz=(ARM_L / 2, 0, 0)), material=matte_metal)
    # Elbow Brackets (parent side)
    lower_arm.visual(Box((0.04, 0.005, 0.04)), origin=Origin(xyz=(ARM_L, 0.015, 0)), material=matte_metal, name="elbow_bracket_l")
    lower_arm.visual(Box((0.04, 0.005, 0.04)), origin=Origin(xyz=(ARM_L, -0.015, 0)), material=matte_metal, name="elbow_bracket_r")

    lower_arm.inertial = Inertial.from_geometry(Box((ARM_L, ARM_W, ARM_T)), mass=0.5, origin=Origin(xyz=(ARM_L / 2, 0, 0)))

    # --- Upper Arm ---
    upper_arm = model.part("upper_arm")
    # Elbow Hub (child side - fits between lower arm brackets)
    upper_arm.visual(Cylinder(radius=HUB_R, length=0.024), origin=Origin(rpy=(math.pi / 2, 0, 0)), material=matte_metal, name="elbow_hub")
    # Adjustment Collar
    upper_arm.visual(Cylinder(radius=HUB_R + 0.002, length=0.01), origin=Origin(xyz=(0, -0.016, 0), rpy=(math.pi / 2, 0, 0)), material=plastic_collar)
    # Main Beam
    upper_arm.visual(Box((ARM_L, ARM_W, ARM_T)), origin=Origin(xyz=(ARM_L / 2, 0, 0)), material=matte_metal)
    # Wrist Brackets (parent side)
    upper_arm.visual(Box((0.04, 0.005, 0.04)), origin=Origin(xyz=(ARM_L, 0.015, 0)), material=matte_metal, name="wrist_bracket_l")
    upper_arm.visual(Box((0.04, 0.005, 0.04)), origin=Origin(xyz=(ARM_L, -0.015, 0)), material=matte_metal, name="wrist_bracket_r")

    upper_arm.inertial = Inertial.from_geometry(Box((ARM_L, ARM_W, ARM_T)), mass=0.4, origin=Origin(xyz=(ARM_L / 2, 0, 0)))

    # --- Microphone ---
    microphone = model.part("microphone")
    # Wrist Hub (child side)
    microphone.visual(Cylinder(radius=HUB_R, length=0.024), origin=Origin(rpy=(math.pi / 2, 0, 0)), material=matte_metal, name="wrist_hub")
    # Adjustment Collar
    microphone.visual(Cylinder(radius=HUB_R + 0.002, length=0.01), origin=Origin(xyz=(0, 0.016, 0), rpy=(math.pi / 2, 0, 0)), material=plastic_collar)
    # Mic Mount / Bracket
    microphone.visual(Box((0.04, 0.02, 0.02)), origin=Origin(xyz=(0.03, 0, 0)), material=matte_metal)
    # Mic Body
    microphone.visual(Cylinder(radius=0.022, length=0.10), origin=Origin(xyz=(0.05, 0, 0), rpy=(0, math.pi / 2, 0)), material=mic_finish)
    # Mic Grille
    microphone.visual(Cylinder(radius=0.022, length=0.04), origin=Origin(xyz=(0.12, 0, 0), rpy=(0, math.pi / 2, 0)), material=mic_grille)

    microphone.inertial = Inertial.from_geometry(Cylinder(radius=0.022, length=0.15), mass=0.3, origin=Origin(xyz=(0.08, 0, 0)))

    # --- Articulations ---
    # Shoulder: Base to Lower Arm
    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lower_arm",
        origin=Origin(xyz=(0, 0, BLOCK_H + bracket_h / 2)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-math.pi / 2, upper=math.pi / 2),
    )

    # Elbow: Lower Arm to Upper Arm
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_arm",
        origin=Origin(xyz=(ARM_L, 0, 0)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.1, upper=math.pi * 0.8),
    )

    # Wrist: Upper Arm to Microphone
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="microphone",
        origin=Origin(xyz=(ARM_L, 0, 0)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-math.pi / 2, upper=math.pi / 2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    # Allow overlaps for the hubs which are nested by design
    ctx.allow_overlap("base", "lower_arm", reason="shoulder hub overlaps mounting block top")
    ctx.allow_overlap("lower_arm", "upper_arm", reason="elbow hubs overlap")
    ctx.allow_overlap("upper_arm", "microphone", reason="wrist hubs overlap")

    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.fail_if_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Intent checks
    # 1. Base is at the bottom, and lower_arm is mounted to it.
    # We expect penetration because of the brackets.
    ctx.expect_aabb_gap("lower_arm", "base", axis="z", max_gap=0.01, max_penetration=0.05)
    ctx.expect_aabb_contact("lower_arm", "base")

    # 2. Sequential joints move the end effector (microphone)
    # Check shoulder motion
    with ctx.pose(shoulder=0.5):
        pos1 = ctx.part_world_position("microphone")
    with ctx.pose(shoulder=-0.5):
        pos2 = ctx.part_world_position("microphone")
    assert pos1[2] > pos2[2], f"Shoulder joint should move microphone vertically (pos1[2]={pos1[2]}, pos2[2]={pos2[2]})"

    # 3. Check elbow folding
    with ctx.pose(elbow=0.0):
        reach1 = ctx.part_world_position("microphone")[0]
    with ctx.pose(elbow=1.2):
        reach2 = ctx.part_world_position("microphone")[0]
    assert reach1 > reach2, "Elbow joint should change horizontal reach"

    # 4. Check attachment at joints
    ctx.expect_aabb_gap("upper_arm", "lower_arm", axis="x", max_gap=0.01, max_penetration=0.05)
    ctx.expect_aabb_gap("microphone", "upper_arm", axis="x", max_gap=0.01, max_penetration=0.05)
    ctx.expect_aabb_contact("upper_arm", "lower_arm")
    ctx.expect_aabb_contact("microphone", "upper_arm")

    # 4. Check microphone position relative to wrist
    ctx.expect_aabb_overlap("microphone", "upper_arm", axes="y", min_overlap=0.01)

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
