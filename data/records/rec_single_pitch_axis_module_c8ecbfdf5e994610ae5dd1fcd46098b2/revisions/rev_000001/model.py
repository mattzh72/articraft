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


BASE_WIDTH = 0.220
BASE_DEPTH = 0.180
BASE_THICKNESS = 0.022
SADDLE_LIP_WIDTH = 0.180
FRONT_LIP_DEPTH = 0.018
FRONT_LIP_HEIGHT = 0.018
REAR_LIP_DEPTH = 0.030
REAR_LIP_HEIGHT = 0.028
SADDLE_PAD_WIDTH = 0.036
SADDLE_PAD_DEPTH = 0.040
SADDLE_PAD_HEIGHT = 0.024
SADDLE_PAD_CENTER_Y = -0.026
INNER_SPAN = 0.144
CHEEK_THICKNESS = 0.028
CHEEK_DEPTH = 0.104
CHEEK_HEIGHT = 0.124
CHEEK_CENTER_Y = -0.002
BRIDGE_DEPTH = 0.020
BRIDGE_THICKNESS = 0.010
BRIDGE_CENTER_Y = -0.024
PIVOT_Z = 0.093

CORE_WIDTH = 0.060
CORE_DEPTH = 0.060
CORE_HEIGHT = 0.074
CORE_CENTER_Y = 0.010
CORE_CENTER_Z = -0.020
BEAM_WIDTH = 0.116
BEAM_DEPTH = 0.036
BEAM_HEIGHT = 0.028
BEAM_CENTER_Y = 0.000
BEAM_CENTER_Z = 0.000
JOURNAL_RADIUS = 0.018
JOURNAL_LENGTH = 0.026
JOURNAL_CENTER_X = INNER_SPAN / 2.0 - JOURNAL_LENGTH / 2.0
FACE_WIDTH = 0.090
FACE_HEIGHT = 0.094
FACE_THICKNESS = 0.012
FACE_CENTER_Y = CORE_CENTER_Y + CORE_DEPTH / 2.0 + FACE_THICKNESS / 2.0
FACE_CENTER_Z = -0.010
HALF_PI = 1.5707963267948966


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_trunnion")

    model.material("frame_gray", color=(0.22, 0.24, 0.27, 1.0))
    model.material("head_gray", color=(0.48, 0.50, 0.53, 1.0))
    model.material("face_metal", color=(0.72, 0.75, 0.78, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        material="frame_gray",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        name="base_plate",
    )
    frame.visual(
        Box((SADDLE_LIP_WIDTH, FRONT_LIP_DEPTH, FRONT_LIP_HEIGHT)),
        material="frame_gray",
        origin=Origin(
            xyz=(
                0.0,
                0.052,
                BASE_THICKNESS + FRONT_LIP_HEIGHT / 2.0,
            )
        ),
        name="front_lip",
    )
    frame.visual(
        Box((SADDLE_PAD_WIDTH, SADDLE_PAD_DEPTH, SADDLE_PAD_HEIGHT)),
        material="frame_gray",
        origin=Origin(
            xyz=(
                INNER_SPAN / 2.0 - SADDLE_PAD_WIDTH / 2.0,
                SADDLE_PAD_CENTER_Y,
                BASE_THICKNESS + SADDLE_PAD_HEIGHT / 2.0,
            )
        ),
        name="left_saddle_pad",
    )
    frame.visual(
        Box((SADDLE_PAD_WIDTH, SADDLE_PAD_DEPTH, SADDLE_PAD_HEIGHT)),
        material="frame_gray",
        origin=Origin(
            xyz=(
                -(INNER_SPAN / 2.0 - SADDLE_PAD_WIDTH / 2.0),
                SADDLE_PAD_CENTER_Y,
                BASE_THICKNESS + SADDLE_PAD_HEIGHT / 2.0,
            )
        ),
        name="right_saddle_pad",
    )
    frame.visual(
        Box((CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT)),
        material="frame_gray",
        origin=Origin(
            xyz=(
                INNER_SPAN / 2.0 + CHEEK_THICKNESS / 2.0,
                CHEEK_CENTER_Y,
                BASE_THICKNESS + CHEEK_HEIGHT / 2.0,
            )
        ),
        name="left_cheek",
    )
    frame.visual(
        Box((CHEEK_THICKNESS, CHEEK_DEPTH, CHEEK_HEIGHT)),
        material="frame_gray",
        origin=Origin(
            xyz=(
                -(INNER_SPAN / 2.0 + CHEEK_THICKNESS / 2.0),
                CHEEK_CENTER_Y,
                BASE_THICKNESS + CHEEK_HEIGHT / 2.0,
            )
        ),
        name="right_cheek",
    )
    frame.visual(
        Box((INNER_SPAN, BRIDGE_DEPTH, BRIDGE_THICKNESS)),
        material="frame_gray",
        origin=Origin(
            xyz=(
                0.0,
                BRIDGE_CENTER_Y,
                BASE_THICKNESS + CHEEK_HEIGHT - BRIDGE_THICKNESS / 2.0,
            )
        ),
        name="bridge",
    )

    head = model.part("tilting_head")
    head.visual(
        Box((CORE_WIDTH, CORE_DEPTH, CORE_HEIGHT)),
        material="head_gray",
        origin=Origin(xyz=(0.0, CORE_CENTER_Y, CORE_CENTER_Z)),
        name="head_core",
    )
    head.visual(
        Box((BEAM_WIDTH, BEAM_DEPTH, BEAM_HEIGHT)),
        material="head_gray",
        origin=Origin(xyz=(0.0, BEAM_CENTER_Y, BEAM_CENTER_Z)),
        name="trunnion_beam",
    )
    head.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        material="head_gray",
        origin=Origin(
            xyz=(JOURNAL_CENTER_X, 0.0, 0.0),
            rpy=(0.0, HALF_PI, 0.0),
        ),
        name="left_journal",
    )
    head.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        material="head_gray",
        origin=Origin(
            xyz=(-JOURNAL_CENTER_X, 0.0, 0.0),
            rpy=(0.0, HALF_PI, 0.0),
        ),
        name="right_journal",
    )
    head.visual(
        Box((FACE_WIDTH, FACE_THICKNESS, FACE_HEIGHT)),
        material="face_metal",
        origin=Origin(xyz=(0.0, FACE_CENTER_Y, 0.0)),
        name="carried_face",
    )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.30,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("tilting_head")
    pitch = object_model.get_articulation("pitch_axis")

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

    axis = pitch.axis
    ctx.check(
        "pitch axis is horizontal",
        abs(abs(axis[0]) - 1.0) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9,
        f"expected x-axis trunnion, got axis={axis}",
    )

    with ctx.pose({pitch: 0.0}):
        ctx.expect_contact(
            frame,
            head,
            elem_a="left_cheek",
            elem_b="left_journal",
            name="left trunnion cheek supports left journal",
        )
        ctx.expect_contact(
            frame,
            head,
            elem_a="right_cheek",
            elem_b="right_journal",
            name="right trunnion cheek supports right journal",
        )
        ctx.expect_within(
            head,
            frame,
            axes="x",
            margin=0.0,
            name="head journals remain between side cheeks",
        )
        ctx.expect_gap(
            frame,
            head,
            axis="z",
            positive_elem="bridge",
            negative_elem="trunnion_beam",
            min_gap=0.020,
            max_gap=0.032,
            name="bridge cap leaves small clearance above head",
        )
        ctx.expect_gap(
            head,
            frame,
            axis="z",
            positive_elem="head_core",
            negative_elem="base_plate",
            min_gap=0.010,
            max_gap=0.022,
            name="head clears lower saddle base",
        )
        ctx.expect_overlap(
            head,
            frame,
            axes="xy",
            min_overlap=0.070,
            name="head sits deeply within frame footprint",
        )

    with ctx.pose({pitch: pitch.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at lower pitch stop")

    with ctx.pose({pitch: pitch.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at upper pitch stop")
        raised_face_aabb = ctx.part_element_world_aabb(head, elem="carried_face")

    with ctx.pose({pitch: 0.0}):
        rest_face_aabb = ctx.part_element_world_aabb(head, elem="carried_face")

    if raised_face_aabb is not None and rest_face_aabb is not None:
        raised_z = _aabb_center(raised_face_aabb)[2]
        rest_z = _aabb_center(rest_face_aabb)[2]
        ctx.check(
            "positive pitch lifts carried face",
            raised_z > rest_z + 0.015,
            f"expected carried face to rise under positive pitch, rest_z={rest_z:.4f}, raised_z={raised_z:.4f}",
        )
    else:
        ctx.fail("positive pitch lifts carried face", "missing carried_face visual AABB for pose comparison")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
