from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LEN = 0.18
BASE_W = 0.10
BASE_T = 0.016

COLUMN_LEN = 0.050
COLUMN_W = 0.072
COLUMN_H = 0.052
COLUMN_CENTER_X = -0.030

HINGE_Z = 0.104
EAR_THICK = 0.014
EAR_GAP = 0.040
EAR_LEN = 0.034
EAR_H = 0.058
EAR_BASE_Z = BASE_T + COLUMN_H
ARM_W = 0.034
ARM_H = 0.046
HUB_R = 0.018
HUB_LEN = EAR_GAP
ROOT_BLOCK_LEN = 0.075
ARM_OUTER_LEN = 0.315
GUIDE_SIDE_T = 0.006
GUIDE_CAP_T = 0.011
GUIDE_LEN = ARM_OUTER_LEN - ROOT_BLOCK_LEN

SLIDE_HOME = ROOT_BLOCK_LEN
SLIDE_TRAVEL = 0.180
BAR_LEN = 0.220
BAR_W = 0.016
BAR_H = 0.018
COLLAR_LEN = 0.020
COLLAR_W = 0.020
COLLAR_H = 0.022
HEAD_LEN = 0.050
HEAD_W = 0.028
HEAD_H = 0.030
NOSE_LEN = 0.012
NOSE_R = 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_module")

    model.material("support_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("arm_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("output_anodized", rgba=(0.36, 0.39, 0.43, 1.0))

    support = model.part("support")
    support.visual(
        Box((BASE_LEN, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="support_steel",
        name="base_plate",
    )
    support.visual(
        Box((COLUMN_LEN, COLUMN_W, COLUMN_H)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, BASE_T + COLUMN_H / 2.0)),
        material="support_steel",
        name="column_block",
    )
    ear_offset = EAR_GAP / 2.0 + EAR_THICK / 2.0
    for side, suffix in ((1.0, "left"), (-1.0, "right")):
        support.visual(
            Box((EAR_LEN, EAR_THICK, EAR_H)),
            origin=Origin(xyz=(0.0, side * ear_offset, EAR_BASE_Z + EAR_H / 2.0)),
            material="support_steel",
            name=f"{suffix}_ear",
        )

    arm = model.part("arm_link")
    arm.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="arm_aluminum",
        name="hinge_barrel",
    )
    arm.visual(
        Box((ROOT_BLOCK_LEN, ARM_W, ARM_H)),
        origin=Origin(xyz=(ROOT_BLOCK_LEN / 2.0, 0.0, 0.0)),
        material="arm_aluminum",
        name="root_block",
    )
    side_rail_height = ARM_H - 2.0 * GUIDE_CAP_T
    rail_center_x = ROOT_BLOCK_LEN + GUIDE_LEN / 2.0
    rail_center_y = ARM_W / 2.0 - GUIDE_SIDE_T / 2.0
    cap_center_z = ARM_H / 2.0 - GUIDE_CAP_T / 2.0
    arm.visual(
        Box((GUIDE_LEN, GUIDE_SIDE_T, side_rail_height)),
        origin=Origin(xyz=(rail_center_x, rail_center_y, 0.0)),
        material="arm_aluminum",
        name="left_guide_rail",
    )
    arm.visual(
        Box((GUIDE_LEN, GUIDE_SIDE_T, side_rail_height)),
        origin=Origin(xyz=(rail_center_x, -rail_center_y, 0.0)),
        material="arm_aluminum",
        name="right_guide_rail",
    )
    arm.visual(
        Box((GUIDE_LEN, ARM_W, GUIDE_CAP_T)),
        origin=Origin(xyz=(rail_center_x, 0.0, cap_center_z)),
        material="arm_aluminum",
        name="top_guide_cap",
    )
    arm.visual(
        Box((GUIDE_LEN, ARM_W, GUIDE_CAP_T)),
        origin=Origin(xyz=(rail_center_x, 0.0, -cap_center_z)),
        material="arm_aluminum",
        name="bottom_guide_cap",
    )

    output = model.part("output_member")
    output.visual(
        Box((BAR_LEN, BAR_W, BAR_H)),
        origin=Origin(xyz=(BAR_LEN / 2.0, 0.0, 0.0)),
        material="output_anodized",
        name="slider_bar",
    )
    output.visual(
        Box((COLLAR_LEN, COLLAR_W, COLLAR_H)),
        origin=Origin(xyz=(BAR_LEN + COLLAR_LEN / 2.0, 0.0, 0.0)),
        material="output_anodized",
        name="guide_collar",
    )
    output.visual(
        Box((HEAD_LEN, HEAD_W, HEAD_H)),
        origin=Origin(xyz=(BAR_LEN + COLLAR_LEN + HEAD_LEN / 2.0, 0.0, 0.0)),
        material="output_anodized",
        name="service_head",
    )
    output.visual(
        Cylinder(radius=NOSE_R, length=NOSE_LEN),
        origin=Origin(
            xyz=(BAR_LEN + COLLAR_LEN + HEAD_LEN + NOSE_LEN / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="output_anodized",
        name="tool_nose",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.4, lower=0.0, upper=1.15),
    )
    model.articulation(
        "arm_extension",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=output,
        origin=Origin(xyz=(SLIDE_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    arm = object_model.get_part("arm_link")
    output = object_model.get_part("output_member")
    base_hinge = object_model.get_articulation("base_hinge")
    arm_extension = object_model.get_articulation("arm_extension")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "base hinge uses lateral axis",
        tuple(base_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={base_hinge.axis!r}",
    )
    ctx.check(
        "extension stage uses arm centerline axis",
        tuple(arm_extension.axis) == (1.0, 0.0, 0.0),
        details=f"axis={arm_extension.axis!r}",
    )

    with ctx.pose(base_hinge=0.0, arm_extension=0.0):
        ctx.expect_contact(arm, support, name="arm hub seats in support clevis")
        ctx.expect_within(
            output,
            arm,
            axes="yz",
            margin=0.003,
            name="output stage stays centered inside arm guide",
        )
        ctx.expect_overlap(
            output,
            arm,
            axes="x",
            min_overlap=0.20,
            name="output stage remains deeply engaged in guide at home",
        )

        retracted_aabb = ctx.part_world_aabb(output)

    with ctx.pose(base_hinge=0.0, arm_extension=SLIDE_TRAVEL):
        ctx.expect_within(
            output,
            arm,
            axes="yz",
            margin=0.003,
            name="extended output remains laterally guided",
        )
        extended_aabb = ctx.part_world_aabb(output)

    if retracted_aabb is not None and extended_aabb is not None:
        ctx.check(
            "prismatic stage extends outward",
            extended_aabb[1][0] > retracted_aabb[1][0] + 0.16,
            details=f"retracted_max_x={retracted_aabb[1][0]:.4f}, extended_max_x={extended_aabb[1][0]:.4f}",
        )

    with ctx.pose(base_hinge=0.0, arm_extension=SLIDE_TRAVEL):
        closed_aabb = ctx.part_world_aabb(output)
    with ctx.pose(base_hinge=1.0, arm_extension=SLIDE_TRAVEL):
        opened_aabb = ctx.part_world_aabb(output)

    if closed_aabb is not None and opened_aabb is not None:
        ctx.check(
            "revolute stage lifts the extended output upward",
            opened_aabb[1][2] > closed_aabb[1][2] + 0.20,
            details=f"closed_max_z={closed_aabb[1][2]:.4f}, opened_max_z={opened_aabb[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
