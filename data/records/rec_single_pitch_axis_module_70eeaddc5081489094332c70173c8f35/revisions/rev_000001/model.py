from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_X = 0.320
BASE_Y = 0.220
BASE_T = 0.016

SUPPORT_BODY_X = 0.140
SUPPORT_BODY_Y = 0.110
SUPPORT_BODY_H = 0.075

CHEEK_X = 0.090
CHEEK_T = 0.018
CHEEK_H = 0.096
CHEEK_CENTER_Y = 0.069
CHEEK_SPAN_Y = 2.0 * CHEEK_CENTER_Y + CHEEK_T
YOKE_BASE_H = 0.026

AXIS_Z = BASE_T + SUPPORT_BODY_H + YOKE_BASE_H + 0.028

BOSS_R = 0.025
BOSS_T = 0.010

TRUNNION_CORE_R = 0.011
TRUNNION_CORE_LEN = 0.084
TRUNNION_COLLAR_R = 0.019
TRUNNION_COLLAR_LEN = 0.018
TRUNNION_COLLAR_CENTER_Y = 0.051

HEAD_HUB_X = 0.026
HEAD_HUB_Y = 0.068
HEAD_HUB_Z = 0.046
HEAD_HUB_CENTER_X = 0.008

ARM_X = 0.064
ARM_T = 0.014
ARM_Z = 0.052
ARM_CENTER_X = 0.040
ARM_CENTER_Y = 0.035
ARM_CENTER_Z = 0.032

FACE_BACKER_X = 0.012
FACE_BACKER_Y = 0.070
FACE_BACKER_Z = 0.036
FACE_BACKER_CENTER_X = 0.078

TOOL_FACE_X = 0.094
TOOL_FACE_T = 0.020
TOOL_FACE_Y = 0.095
TOOL_FACE_Z = 0.105
TOOL_HOLE_D = 0.010

HALF_PI = 1.5707963267948966
TILT_LOWER = 0.0
TILT_UPPER = 1.10


def _tool_face_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(TOOL_FACE_T, TOOL_FACE_Y, TOOL_FACE_Z).translate(
        (TOOL_FACE_X, 0.0, 0.0)
    )
    return (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-TOOL_FACE_Y * 0.26, -TOOL_FACE_Z * 0.26),
                (-TOOL_FACE_Y * 0.26, TOOL_FACE_Z * 0.26),
                (TOOL_FACE_Y * 0.26, -TOOL_FACE_Z * 0.26),
                (TOOL_FACE_Y * 0.26, TOOL_FACE_Z * 0.26),
            ]
        )
        .hole(TOOL_HOLE_D)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_pitch_trunnion_module")

    base_finish = model.material("base_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    head_finish = model.material("head_finish", rgba=(0.66, 0.69, 0.72, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.82, 0.84, 0.86, 1.0))

    support = model.part("support")
    support.visual(
        Box((BASE_X, BASE_Y, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=base_finish,
        name="base_plate",
    )
    support.visual(
        Box((SUPPORT_BODY_X, SUPPORT_BODY_Y, SUPPORT_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + SUPPORT_BODY_H / 2.0)),
        material=base_finish,
        name="support_body",
    )
    support.visual(
        Box((CHEEK_X, CHEEK_SPAN_Y, YOKE_BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + SUPPORT_BODY_H + YOKE_BASE_H / 2.0)),
        material=base_finish,
        name="yoke_base",
    )
    support.visual(
        Box((CHEEK_X, CHEEK_T, CHEEK_H)),
        origin=Origin(
            xyz=(0.0, CHEEK_CENTER_Y, BASE_T + SUPPORT_BODY_H + YOKE_BASE_H + CHEEK_H / 2.0)
        ),
        material=base_finish,
        name="left_cheek",
    )
    support.visual(
        Box((CHEEK_X, CHEEK_T, CHEEK_H)),
        origin=Origin(
            xyz=(0.0, -CHEEK_CENTER_Y, BASE_T + SUPPORT_BODY_H + YOKE_BASE_H + CHEEK_H / 2.0)
        ),
        material=base_finish,
        name="right_cheek",
    )
    support.visual(
        Cylinder(radius=BOSS_R, length=BOSS_T),
        origin=Origin(
            xyz=(0.0, CHEEK_CENTER_Y + CHEEK_T / 2.0 + BOSS_T / 2.0, AXIS_Z),
            rpy=(HALF_PI, 0.0, 0.0),
        ),
        material=base_finish,
        name="left_bearing",
    )
    support.visual(
        Cylinder(radius=BOSS_R, length=BOSS_T),
        origin=Origin(
            xyz=(0.0, -(CHEEK_CENTER_Y + CHEEK_T / 2.0 + BOSS_T / 2.0), AXIS_Z),
            rpy=(HALF_PI, 0.0, 0.0),
        ),
        material=base_finish,
        name="right_bearing",
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Box((HEAD_HUB_X, HEAD_HUB_Y, HEAD_HUB_Z)),
        origin=Origin(xyz=(HEAD_HUB_CENTER_X, 0.0, 0.0)),
        material=head_finish,
        name="head_hub",
    )
    tilt_head.visual(
        Box((ARM_X, ARM_T, ARM_Z)),
        origin=Origin(xyz=(ARM_CENTER_X, ARM_CENTER_Y, ARM_CENTER_Z)),
        material=head_finish,
        name="left_arm",
    )
    tilt_head.visual(
        Box((ARM_X, ARM_T, ARM_Z)),
        origin=Origin(xyz=(ARM_CENTER_X, -ARM_CENTER_Y, ARM_CENTER_Z)),
        material=head_finish,
        name="right_arm",
    )
    tilt_head.visual(
        Cylinder(radius=TRUNNION_CORE_R, length=TRUNNION_CORE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=head_finish,
        name="trunnion_core",
    )
    tilt_head.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_LEN),
        origin=Origin(
            xyz=(0.0, TRUNNION_COLLAR_CENTER_Y, 0.0),
            rpy=(HALF_PI, 0.0, 0.0),
        ),
        material=head_finish,
        name="left_trunnion",
    )
    tilt_head.visual(
        Cylinder(radius=TRUNNION_COLLAR_R, length=TRUNNION_COLLAR_LEN),
        origin=Origin(
            xyz=(0.0, -TRUNNION_COLLAR_CENTER_Y, 0.0),
            rpy=(HALF_PI, 0.0, 0.0),
        ),
        material=head_finish,
        name="right_trunnion",
    )
    tilt_head.visual(
        Box((FACE_BACKER_X, FACE_BACKER_Y, FACE_BACKER_Z)),
        origin=Origin(xyz=(FACE_BACKER_CENTER_X, 0.0, 0.0)),
        material=head_finish,
        name="face_backer",
    )
    tilt_head.visual(
        Box((0.032, 0.050, 0.022)),
        origin=Origin(xyz=(0.066, 0.0, 0.020)),
        material=head_finish,
        name="top_rib",
    )
    tilt_head.visual(
        Box((0.032, 0.050, 0.022)),
        origin=Origin(xyz=(0.066, 0.0, -0.020)),
        material=head_finish,
        name="bottom_rib",
    )
    tilt_head.visual(
        Box((0.018, 0.022, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=head_finish,
        name="center_web",
    )
    tilt_head.visual(
        mesh_from_cadquery(_tool_face_shape(), "tool_face_plate"),
        material=plate_finish,
        name="tool_face_plate",
    )

    model.articulation(
        "support_to_head",
        ArticulationType.REVOLUTE,
        parent=support,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=TILT_LOWER,
            upper=TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    tilt_head = object_model.get_part("tilt_head")
    tilt_joint = object_model.get_articulation("support_to_head")
    tool_face = tilt_head.get_visual("tool_face_plate")
    left_cheek = support.get_visual("left_cheek")
    right_cheek = support.get_visual("right_cheek")
    left_trunnion = tilt_head.get_visual("left_trunnion")
    right_trunnion = tilt_head.get_visual("right_trunnion")

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
        tilt_head,
        support,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        name="left_trunnion_is_borne_in_support",
    )
    ctx.expect_contact(
        tilt_head,
        support,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        name="right_trunnion_is_borne_in_support",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        mins, maxs = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))

    closed_face_aabb = ctx.part_element_world_aabb(tilt_head, elem=tool_face)
    upper_limit = tilt_joint.motion_limits.upper if tilt_joint.motion_limits is not None else None
    lower_limit = tilt_joint.motion_limits.lower if tilt_joint.motion_limits is not None else None

    if closed_face_aabb is None or upper_limit is None or lower_limit is None:
        ctx.fail("tool_face_pose_queries_available", "Could not resolve tooling face AABB or joint limits.")
        return ctx.report()

    closed_face_center = aabb_center(closed_face_aabb)

    with ctx.pose({tilt_joint: upper_limit}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_tilt")
        upper_face_aabb = ctx.part_element_world_aabb(tilt_head, elem=tool_face)

    with ctx.pose({tilt_joint: lower_limit}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lower_tilt")

    if upper_face_aabb is None:
        ctx.fail("upper_tilt_face_available", "Could not resolve tooling face AABB at the upper tilt limit.")
        return ctx.report()

    upper_face_center = aabb_center(upper_face_aabb)
    ctx.check(
        "positive_tilt_raises_tool_face",
        upper_face_center[2] > closed_face_center[2] + 0.045
        and upper_face_center[0] < closed_face_center[0] - 0.020,
        details=f"closed_center={closed_face_center}, upper_center={upper_face_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
