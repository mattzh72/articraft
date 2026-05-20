from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


PLATE_X = 0.18
PLATE_Y = 0.12
PLATE_T = 0.012

CHEEK_X = 0.078
CHEEK_T = 0.014
CHEEK_CENTER_Y = 0.039
CHEEK_INNER_FACE_Y = CHEEK_CENTER_Y - CHEEK_T / 2.0
CHEEK_OUTER_FACE_Y = CHEEK_CENTER_Y + CHEEK_T / 2.0
CHEEK_WEB_H = 0.073
CHEEK_BOSS_R = 0.018

TRUNNION_AXIS_X = 0.0
TRUNNION_AXIS_Z = 0.070
HOLE_R = 0.008
PIN_R = 0.0065

BODY_REAR_X = -0.010
BODY_FRONT_X = 0.042
BODY_LEN = BODY_FRONT_X - BODY_REAR_X
BODY_CENTER_X = 0.5 * (BODY_FRONT_X + BODY_REAR_X)
BODY_W = 0.050
BODY_H = 0.040

JOURNAL_LEN = CHEEK_INNER_FACE_Y - BODY_W / 2.0
JOURNAL_R = 0.009
PIN_HALF_SPAN = CHEEK_OUTER_FACE_Y

OUTPUT_FACE_T = 0.004
OUTPUT_FACE_W = 0.048
OUTPUT_FACE_H = 0.036
OUTPUT_FACE_CENTER_X = BODY_FRONT_X + OUTPUT_FACE_T / 2.0


def _center_of_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_trunnion_module")

    base_finish = model.material("base_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    head_finish = model.material("head_finish", rgba=(0.71, 0.73, 0.76, 1.0))
    face_finish = model.material("face_finish", rgba=(0.82, 0.84, 0.87, 1.0))

    base = model.part("base")
    base.visual(
        Box((PLATE_X, PLATE_Y, PLATE_T)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T / 2.0)),
        material=base_finish,
        name="base_plate",
    )
    base.visual(
        Box((CHEEK_X, CHEEK_T, CHEEK_WEB_H)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, PLATE_T + CHEEK_WEB_H / 2.0)),
        material=base_finish,
        name="left_cheek",
    )
    base.visual(
        Box((CHEEK_X, CHEEK_T, CHEEK_WEB_H)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, PLATE_T + CHEEK_WEB_H / 2.0)),
        material=base_finish,
        name="right_cheek",
    )
    base.visual(
        Cylinder(radius=CHEEK_BOSS_R, length=CHEEK_T),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, TRUNNION_AXIS_Z), rpy=(-1.5707963267948966, 0.0, 0.0)),
        material=base_finish,
        name="left_boss",
    )
    base.visual(
        Cylinder(radius=CHEEK_BOSS_R, length=CHEEK_T),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, TRUNNION_AXIS_Z), rpy=(-1.5707963267948966, 0.0, 0.0)),
        material=base_finish,
        name="right_boss",
    )

    head = model.part("moving_head")
    head.visual(
        Box((BODY_LEN, BODY_W, BODY_H)),
        origin=Origin(xyz=(BODY_CENTER_X, 0.0, 0.0)),
        material=head_finish,
        name="head_body",
    )
    head.visual(
        Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
        origin=Origin(
            xyz=(0.0, BODY_W / 2.0 + JOURNAL_LEN / 2.0, 0.0),
            rpy=(-1.5707963267948966, 0.0, 0.0),
        ),
        material=head_finish,
        name="left_journal",
    )
    head.visual(
        Cylinder(radius=JOURNAL_R, length=JOURNAL_LEN),
        origin=Origin(
            xyz=(0.0, -(BODY_W / 2.0 + JOURNAL_LEN / 2.0), 0.0),
            rpy=(-1.5707963267948966, 0.0, 0.0),
        ),
        material=head_finish,
        name="right_journal",
    )
    head.visual(
        Box((OUTPUT_FACE_T, OUTPUT_FACE_W, OUTPUT_FACE_H)),
        origin=Origin(xyz=(OUTPUT_FACE_CENTER_X, 0.0, 0.0)),
        material=face_finish,
        name="output_face",
    )

    model.articulation(
        "base_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(TRUNNION_AXIS_X, 0.0, TRUNNION_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.45,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("moving_head")
    pitch = object_model.get_articulation("base_to_head_pitch")
    output_face = head.get_visual("output_face")

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
        "pitch_axis_is_horizontal",
        abs(pitch.axis[0]) < 1e-9 and abs(abs(pitch.axis[1]) - 1.0) < 1e-9 and abs(pitch.axis[2]) < 1e-9,
        details=f"expected horizontal trunnion axis, got {pitch.axis}",
    )
    ctx.expect_contact(base, head, name="head_is_supported_by_split_cheeks")

    closed_face_aabb = ctx.part_element_world_aabb(head, elem=output_face)
    ctx.check(
        "output_face_visual_present",
        closed_face_aabb is not None,
        details="named output face visual did not resolve to world geometry",
    )

    with ctx.pose({pitch: pitch.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_pitch_limit")
        open_face_aabb = ctx.part_element_world_aabb(head, elem=output_face)

    with ctx.pose({pitch: pitch.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lower_pitch_limit")

    if closed_face_aabb is not None and open_face_aabb is not None:
        closed_center = _center_of_aabb(closed_face_aabb)
        open_center = _center_of_aabb(open_face_aabb)
        ctx.check(
            "positive_pitch_raises_output_face",
            open_center[2] > closed_center[2] + 0.020,
            details=(
                f"expected output face to lift when pitched positive, "
                f"closed_z={closed_center[2]:.4f}, open_z={open_center[2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
