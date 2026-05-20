from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_LENGTH = 0.290
SUPPORT_WIDTH = 0.160
SUPPORT_TOP_THICKNESS = 0.024
SUPPORT_TOP_CENTER_Z = 0.095
SUPPORT_TOP_CENTER_X = 0.030
SUPPORT_HANGER_LENGTH = 0.094
SUPPORT_HANGER_WIDTH = 0.094
SUPPORT_HANGER_HEIGHT = 0.088
SUPPORT_SLOT_WIDTH = 0.060
SUPPORT_SLOT_HEIGHT = 0.062
SUPPORT_SLOT_LENGTH = 0.050
SUPPORT_SLOT_CENTER_X = 0.020
SUPPORT_SLOT_CENTER_Z = 0.018

UPPER_ARM_LENGTH = 0.280
UPPER_ARM_ROOT_LENGTH = 0.040
UPPER_ARM_ROOT_WIDTH = SUPPORT_SLOT_WIDTH
UPPER_ARM_ROOT_HEIGHT = 0.036
UPPER_ARM_ROOT_CENTER_X = 0.012
UPPER_ARM_ROOT_CENTER_Z = -0.018
UPPER_ARM_BEAM_LENGTH = 0.190
UPPER_ARM_BEAM_WIDTH = 0.044
UPPER_ARM_BEAM_HEIGHT = 0.036
UPPER_ARM_BEAM_CENTER_X = 0.125
UPPER_ARM_BEAM_CENTER_Z = -0.053
UPPER_ARM_RIB_LENGTH = 0.120
UPPER_ARM_RIB_WIDTH = 0.030
UPPER_ARM_RIB_HEIGHT = 0.022
UPPER_ARM_RIB_CENTER_X = 0.105
UPPER_ARM_RIB_CENTER_Z = -0.030
UPPER_ARM_KNUCKLE_LENGTH = 0.060
UPPER_ARM_KNUCKLE_WIDTH = 0.072
UPPER_ARM_KNUCKLE_HEIGHT = 0.040
UPPER_ARM_KNUCKLE_CENTER_X = 0.250
UPPER_ARM_KNUCKLE_CENTER_Z = -0.020

FOREARM_LENGTH = 0.242
FOREARM_ROOT_LENGTH = 0.036
FOREARM_ROOT_WIDTH = 0.044
FOREARM_ROOT_HEIGHT = 0.040
FOREARM_ROOT_CENTER_X = 0.018
FOREARM_ROOT_CENTER_Z = -0.020
FOREARM_BEAM_LENGTH = 0.180
FOREARM_BEAM_WIDTH = 0.040
FOREARM_BEAM_HEIGHT = 0.034
FOREARM_BEAM_CENTER_X = 0.107
FOREARM_BEAM_CENTER_Z = -0.048
FOREARM_RIB_LENGTH = 0.110
FOREARM_RIB_WIDTH = 0.028
FOREARM_RIB_HEIGHT = 0.020
FOREARM_RIB_CENTER_X = 0.085
FOREARM_RIB_CENTER_Z = -0.030
FOREARM_WRIST_BLOCK_LENGTH = 0.046
FOREARM_WRIST_BLOCK_WIDTH = 0.056
FOREARM_WRIST_BLOCK_HEIGHT = 0.046
FOREARM_WRIST_BLOCK_CENTER_X = 0.214
FOREARM_WRIST_BLOCK_CENTER_Z = -0.029
FOREARM_FLANGE_LENGTH = 0.014
FOREARM_FLANGE_WIDTH = 0.060
FOREARM_FLANGE_HEIGHT = 0.060
FOREARM_FLANGE_CENTER_X = 0.235
FOREARM_FLANGE_CENTER_Z = -0.006
OUTPUT_FACE_RADIUS = 0.028
OUTPUT_FACE_THICKNESS = 0.004

SHOULDER_LIMITS = (0.0, 1.00)
ELBOW_LIMITS = (0.0, 1.05)


def _support_shape() -> cq.Workplane:
    top_plate = cq.Workplane("XY").box(
        SUPPORT_LENGTH,
        SUPPORT_WIDTH,
        SUPPORT_TOP_THICKNESS,
    ).translate((SUPPORT_TOP_CENTER_X, 0.0, SUPPORT_TOP_CENTER_Z))

    motor_pod = cq.Workplane("XY").box(0.118, 0.094, 0.038).translate((-0.025, 0.0, 0.118))
    hanger = cq.Workplane("XY").box(
        SUPPORT_HANGER_LENGTH,
        SUPPORT_HANGER_WIDTH,
        SUPPORT_HANGER_HEIGHT,
    ).translate((0.0, 0.0, SUPPORT_HANGER_HEIGHT / 2.0))
    slot = cq.Workplane("XY").box(
        SUPPORT_SLOT_LENGTH,
        SUPPORT_SLOT_WIDTH,
        SUPPORT_SLOT_HEIGHT,
    ).translate((SUPPORT_SLOT_CENTER_X, 0.0, SUPPORT_SLOT_CENTER_Z))
    hanger = hanger.cut(slot)

    rear_spine = cq.Workplane("XY").box(0.040, 0.060, 0.090).translate((-0.030, 0.0, 0.048))
    shoulder_cap = cq.Workplane("XY").box(0.040, 0.078, 0.020).translate((-0.027, 0.0, 0.090))

    return top_plate.union(motor_pod).union(hanger).union(rear_spine).union(shoulder_cap)


def _upper_arm_shape() -> cq.Workplane:
    shoulder_root = cq.Workplane("XY").box(
        UPPER_ARM_ROOT_LENGTH,
        UPPER_ARM_ROOT_WIDTH,
        UPPER_ARM_ROOT_HEIGHT,
    ).translate((UPPER_ARM_ROOT_CENTER_X, 0.0, UPPER_ARM_ROOT_CENTER_Z))

    beam = cq.Workplane("XY").box(
        UPPER_ARM_BEAM_LENGTH,
        UPPER_ARM_BEAM_WIDTH,
        UPPER_ARM_BEAM_HEIGHT,
    ).translate((UPPER_ARM_BEAM_CENTER_X, 0.0, UPPER_ARM_BEAM_CENTER_Z))

    root_web = cq.Workplane("XY").box(0.090, 0.052, 0.056).translate((0.055, 0.0, -0.030))
    rib = cq.Workplane("XY").box(
        UPPER_ARM_RIB_LENGTH,
        UPPER_ARM_RIB_WIDTH,
        UPPER_ARM_RIB_HEIGHT,
    ).translate((UPPER_ARM_RIB_CENTER_X, 0.0, UPPER_ARM_RIB_CENTER_Z))
    knuckle = cq.Workplane("XY").box(
        UPPER_ARM_KNUCKLE_LENGTH,
        UPPER_ARM_KNUCKLE_WIDTH,
        UPPER_ARM_KNUCKLE_HEIGHT,
    ).translate((UPPER_ARM_KNUCKLE_CENTER_X, 0.0, UPPER_ARM_KNUCKLE_CENTER_Z))

    return shoulder_root.union(beam).union(root_web).union(rib).union(knuckle)


def _forearm_shape() -> cq.Workplane:
    elbow_root = cq.Workplane("XY").box(
        FOREARM_ROOT_LENGTH,
        FOREARM_ROOT_WIDTH,
        FOREARM_ROOT_HEIGHT,
    ).translate((FOREARM_ROOT_CENTER_X, 0.0, FOREARM_ROOT_CENTER_Z))

    root_web = cq.Workplane("XY").box(0.082, 0.046, 0.050).translate((0.052, 0.0, -0.022))
    beam = cq.Workplane("XY").box(
        FOREARM_BEAM_LENGTH,
        FOREARM_BEAM_WIDTH,
        FOREARM_BEAM_HEIGHT,
    ).translate((FOREARM_BEAM_CENTER_X, 0.0, FOREARM_BEAM_CENTER_Z))
    rib = cq.Workplane("XY").box(
        FOREARM_RIB_LENGTH,
        FOREARM_RIB_WIDTH,
        FOREARM_RIB_HEIGHT,
    ).translate((FOREARM_RIB_CENTER_X, 0.0, FOREARM_RIB_CENTER_Z))

    wrist_block = cq.Workplane("XY").box(
        FOREARM_WRIST_BLOCK_LENGTH,
        FOREARM_WRIST_BLOCK_WIDTH,
        FOREARM_WRIST_BLOCK_HEIGHT,
    ).translate((FOREARM_WRIST_BLOCK_CENTER_X, 0.0, FOREARM_WRIST_BLOCK_CENTER_Z))

    flange = cq.Workplane("XY").box(
        FOREARM_FLANGE_LENGTH,
        FOREARM_FLANGE_WIDTH,
        FOREARM_FLANGE_HEIGHT,
    ).translate((FOREARM_FLANGE_CENTER_X, 0.0, FOREARM_FLANGE_CENTER_Z))

    return elbow_root.union(root_web).union(beam).union(rib).union(wrist_block).union(flange)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_elbow_arm")

    model.material("support_charcoal", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("arm_aluminum", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("joint_steel", rgba=(0.54, 0.57, 0.61, 1.0))
    model.material("tool_dark", rgba=(0.13, 0.14, 0.16, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "support_body"),
        material="support_charcoal",
        name="support_body",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.280, 0.145, 0.150)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm_body"),
        material="arm_aluminum",
        name="upper_arm_body",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.290, 0.080, 0.090)),
        mass=3.1,
        origin=Origin(xyz=(0.145, 0.0, -0.028)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_forearm_shape(), "forearm_body"),
        material="arm_aluminum",
        name="forearm_body",
    )
    forearm.visual(
        Cylinder(radius=OUTPUT_FACE_RADIUS, length=OUTPUT_FACE_THICKNESS),
        origin=Origin(
            xyz=(FOREARM_LENGTH + OUTPUT_FACE_THICKNESS / 2.0, 0.0, FOREARM_FLANGE_CENTER_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="tool_dark",
        name="output_face",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.248, 0.060, 0.060)),
        mass=2.0,
        origin=Origin(xyz=(0.124, 0.0, -0.020)),
    )

    model.articulation(
        "support_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.3,
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
        ),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.6,
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("support_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    forearm.get_visual("output_face")

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

    ctx.expect_contact(support, upper_arm, name="shoulder_joint_is_physically_carried")
    ctx.expect_contact(upper_arm, forearm, name="elbow_joint_is_physically_carried")

    support_aabb = ctx.part_world_aabb(support)
    upper_aabb = ctx.part_world_aabb(upper_arm)
    forearm_aabb = ctx.part_world_aabb(forearm)
    output_face_aabb = ctx.part_element_world_aabb(forearm, elem="output_face")

    support_center = _aabb_center(support_aabb)
    upper_center = _aabb_center(upper_aabb)
    forearm_center = _aabb_center(forearm_aabb)
    output_face_size = _aabb_size(output_face_aabb)

    ctx.check(
        "arm_chain_hangs_below_support",
        upper_center[2] < support_center[2] - 0.045 and forearm_center[2] < support_center[2] - 0.040,
        details=(
            f"support_center_z={support_center[2]:.4f}, "
            f"upper_center_z={upper_center[2]:.4f}, "
            f"forearm_center_z={forearm_center[2]:.4f}"
        ),
    )
    ctx.check(
        "output_face_is_compact",
        output_face_size[0] <= 0.008 and output_face_size[1] <= 0.060 and output_face_size[2] <= 0.060,
        details=f"output_face_size={output_face_size}",
    )

    rest_face_center = _aabb_center(output_face_aabb)
    with ctx.pose({shoulder: 0.35}):
        deployed_face_center = _aabb_center(ctx.part_element_world_aabb(forearm, elem="output_face"))
    ctx.check(
        "positive_shoulder_rotation_deploys_arm_downward",
        deployed_face_center[2] < rest_face_center[2] - 0.050,
        details=f"rest_z={rest_face_center[2]:.4f}, deployed_z={deployed_face_center[2]:.4f}",
    )

    with ctx.pose({elbow: 0.70}):
        bent_face_center = _aabb_center(ctx.part_element_world_aabb(forearm, elem="output_face"))
    ctx.check(
        "positive_elbow_rotation_raises_output_face",
        bent_face_center[2] > rest_face_center[2] + 0.050,
        details=f"rest_z={rest_face_center[2]:.4f}, bent_z={bent_face_center[2]:.4f}",
    )

    with ctx.pose({shoulder: 0.25, elbow: 0.80}):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
