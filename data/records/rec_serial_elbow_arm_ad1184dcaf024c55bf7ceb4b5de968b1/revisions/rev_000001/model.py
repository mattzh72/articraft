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


BASE_PLATE_X = 0.18
BASE_PLATE_Y = 0.14
BASE_PLATE_Z = 0.016

COLUMN_X = 0.085
COLUMN_Y = 0.075
COLUMN_Z = 0.118

SHOULDER_BLOCK_X = 0.060
SHOULDER_BLOCK_Y = 0.080
SHOULDER_BLOCK_Z = 0.032

SHOULDER_AXIS_Z = BASE_PLATE_Z + COLUMN_Z + 0.051
SHOULDER_GAP_Y = 0.034
SHOULDER_CHEEK_Y = 0.014
SHOULDER_CHEEK_X = 0.050
SHOULDER_CHEEK_Z = 0.075
SHOULDER_BARREL_R = 0.020
SHOULDER_BARREL_Y = SHOULDER_GAP_Y

UPPER_ARM_LENGTH = 0.310
UPPER_ROOT_X = 0.032
UPPER_ROOT_Y = 0.030
UPPER_ROOT_Z = 0.034
UPPER_BEAM_X = 0.240
UPPER_BEAM_Y = 0.050
UPPER_BEAM_Z = 0.082
ELBOW_GAP_Y = 0.028
ELBOW_CHEEK_Y = 0.016
ELBOW_CHEEK_X = 0.060
ELBOW_CHEEK_Z = 0.078
ELBOW_ROOT_X = 0.020
ELBOW_ROOT_Y = UPPER_BEAM_Y
ELBOW_ROOT_Z = 0.068
ELBOW_BARREL_R = 0.018
ELBOW_BARREL_Y = ELBOW_GAP_Y

FOREARM_ROOT_X = 0.036
FOREARM_ROOT_Y = 0.026
FOREARM_ROOT_Z = 0.044
FOREARM_BEAM_X = 0.210
FOREARM_BEAM_Y = 0.036
FOREARM_BEAM_Z = 0.054
ADAPTER_X = 0.030
ADAPTER_Y = 0.050
ADAPTER_Z = 0.074
TOOL_FACE_X = 0.012
TOOL_FACE_Y = 0.082
TOOL_FACE_Z = 0.104


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_mounted_elbow_arm")

    base_gray = model.material("base_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    arm_gray = model.material("arm_gray", rgba=(0.60, 0.63, 0.67, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.77, 0.80, 1.0))
    tool_dark = model.material("tool_dark", rgba=(0.26, 0.29, 0.32, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_Z / 2.0)),
        material=base_gray,
        name="base_plate",
    )
    base.visual(
        Box((COLUMN_X, COLUMN_Y, COLUMN_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_Z + COLUMN_Z / 2.0)),
        material=base_gray,
        name="base_column",
    )
    base.visual(
        Box((SHOULDER_BLOCK_X, SHOULDER_BLOCK_Y, SHOULDER_BLOCK_Z)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_PLATE_Z + COLUMN_Z + SHOULDER_BLOCK_Z / 2.0,
            )
        ),
        material=base_gray,
        name="shoulder_block",
    )
    cheek_y = SHOULDER_GAP_Y / 2.0 + SHOULDER_CHEEK_Y / 2.0
    base.visual(
        Box((SHOULDER_CHEEK_X, SHOULDER_CHEEK_Y, SHOULDER_CHEEK_Z)),
        origin=Origin(xyz=(0.0, cheek_y, SHOULDER_AXIS_Z)),
        material=steel,
        name="shoulder_cheek_left",
    )
    base.visual(
        Box((SHOULDER_CHEEK_X, SHOULDER_CHEEK_Y, SHOULDER_CHEEK_Z)),
        origin=Origin(xyz=(0.0, -cheek_y, SHOULDER_AXIS_Z)),
        material=steel,
        name="shoulder_cheek_right",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=SHOULDER_BARREL_R, length=SHOULDER_BARREL_Y),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Box((UPPER_ROOT_X, UPPER_ROOT_Y, UPPER_ROOT_Z)),
        origin=Origin(xyz=(UPPER_ROOT_X / 2.0, 0.0, 0.0)),
        material=steel,
        name="upper_root_knuckle",
    )
    upper_arm.visual(
        Box((UPPER_BEAM_X, UPPER_BEAM_Y, UPPER_BEAM_Z)),
        origin=Origin(xyz=(UPPER_ROOT_X + UPPER_BEAM_X / 2.0, 0.0, 0.0)),
        material=arm_gray,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((ELBOW_ROOT_X, ELBOW_ROOT_Y, ELBOW_ROOT_Z)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.028, 0.0, 0.0)),
        material=arm_gray,
        name="elbow_root_block",
    )
    elbow_cheek_y = ELBOW_GAP_Y / 2.0 + ELBOW_CHEEK_Y / 2.0
    upper_arm.visual(
        Box((ELBOW_CHEEK_X, ELBOW_CHEEK_Y, ELBOW_CHEEK_Z)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, elbow_cheek_y, 0.0)),
        material=steel,
        name="elbow_cheek_left",
    )
    upper_arm.visual(
        Box((ELBOW_CHEEK_X, ELBOW_CHEEK_Y, ELBOW_CHEEK_Z)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH, -elbow_cheek_y, 0.0)),
        material=steel,
        name="elbow_cheek_right",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=ELBOW_BARREL_R, length=ELBOW_BARREL_Y),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((FOREARM_ROOT_X, FOREARM_ROOT_Y, FOREARM_ROOT_Z)),
        origin=Origin(xyz=(FOREARM_ROOT_X / 2.0, 0.0, 0.0)),
        material=steel,
        name="forearm_root_knuckle",
    )
    forearm.visual(
        Box((FOREARM_BEAM_X, FOREARM_BEAM_Y, FOREARM_BEAM_Z)),
        origin=Origin(
            xyz=(FOREARM_ROOT_X + FOREARM_BEAM_X / 2.0, 0.0, 0.0)
        ),
        material=arm_gray,
        name="forearm_beam",
    )
    adapter_center_x = FOREARM_ROOT_X + FOREARM_BEAM_X + ADAPTER_X / 2.0
    forearm.visual(
        Box((ADAPTER_X, ADAPTER_Y, ADAPTER_Z)),
        origin=Origin(xyz=(adapter_center_x, 0.0, 0.0)),
        material=arm_gray,
        name="tool_adapter",
    )
    tool_face_center_x = (
        FOREARM_ROOT_X + FOREARM_BEAM_X + ADAPTER_X + TOOL_FACE_X / 2.0
    )
    forearm.visual(
        Box((TOOL_FACE_X, TOOL_FACE_Y, TOOL_FACE_Z)),
        origin=Origin(xyz=(tool_face_center_x, 0.0, 0.0)),
        material=tool_dark,
        name="tool_face",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-0.35,
            upper=1.20,
        ),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.8,
            lower=0.0,
            upper=2.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

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

    ctx.expect_contact(
        upper_arm,
        base,
        contact_tol=1e-6,
        name="upper_arm_shoulder_is_borne_by_base_clevis",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        contact_tol=1e-6,
        name="forearm_elbow_is_borne_by_upper_arm_clevis",
    )
    ctx.expect_gap(
        forearm,
        base,
        axis="x",
        min_gap=0.18,
        name="forearm_projects_forward_of_pedestal",
    )

    rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.80}):
        lifted_elbow_pos = ctx.part_world_position(forearm)
        shoulder_ok = (
            rest_elbow_pos is not None
            and lifted_elbow_pos is not None
            and lifted_elbow_pos[2] > rest_elbow_pos[2] + 0.18
        )
        ctx.check(
            "shoulder_positive_rotation_lifts_upper_structure",
            shoulder_ok,
            details=(
                f"rest elbow position={rest_elbow_pos}, "
                f"lifted elbow position={lifted_elbow_pos}"
            ),
        )

    with ctx.pose({elbow: 1.00}):
        tool_face_aabb = ctx.part_element_world_aabb(forearm, elem="tool_face")
        elbow_pos = ctx.part_world_position(forearm)
        tool_face_center_z = None
        if tool_face_aabb is not None:
            tool_face_center_z = (tool_face_aabb[0][2] + tool_face_aabb[1][2]) / 2.0
        elbow_ok = (
            elbow_pos is not None
            and tool_face_center_z is not None
            and tool_face_center_z > elbow_pos[2] + 0.10
        )
        ctx.check(
            "elbow_positive_rotation_raises_tool_face",
            elbow_ok,
            details=(
                f"elbow origin={elbow_pos}, "
                f"tool face center z={tool_face_center_z}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
