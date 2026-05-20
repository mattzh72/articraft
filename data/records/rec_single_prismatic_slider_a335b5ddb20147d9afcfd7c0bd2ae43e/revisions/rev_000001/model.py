from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.220
FRAME_DEPTH = 0.076
FRAME_HEIGHT = 0.430

BACK_PLATE_DEPTH = 0.014
COLUMN_WIDTH = 0.034
COLUMN_DEPTH = 0.054
COLUMN_HEIGHT = 0.322
COLUMN_OFFSET_X = 0.078

TOP_BRIDGE_WIDTH = 0.194
TOP_BRIDGE_DEPTH = 0.056
TOP_BRIDGE_HEIGHT = 0.030

GUIDE_WIDTH = 0.072
GUIDE_DEPTH = 0.036
GUIDE_HEIGHT = 0.306
GUIDE_SUPPORT_WIDTH = 0.090
GUIDE_SUPPORT_DEPTH = 0.020
GUIDE_SUPPORT_HEIGHT = 0.074

STOP_WIDTH = 0.092
STOP_DEPTH = 0.014
STOP_HEIGHT = 0.014
LOWER_STOP_Z = 0.077
UPPER_STOP_Z = 0.368
STOP_Y = -0.007

RUNNER_BODY_WIDTH = 0.112
RUNNER_BODY_DEPTH = 0.050
RUNNER_BODY_HEIGHT = 0.094
RUNNER_BODY_Y = 0.020

RUNNER_NECK_WIDTH = 0.116
RUNNER_NECK_DEPTH = 0.018
RUNNER_NECK_HEIGHT = 0.102
RUNNER_NECK_Y = 0.044

MOUNT_FACE_WIDTH = 0.168
MOUNT_FACE_DEPTH = 0.014
MOUNT_FACE_HEIGHT = 0.110
MOUNT_FACE_Y = 0.060

RUNNER_SLOT_WIDTH = 0.086
RUNNER_SLOT_DEPTH = 0.070
RUNNER_SLOT_HEIGHT = 0.080

GUIDE_PAD_WIDTH = 0.010
GUIDE_PAD_DEPTH = 0.022
GUIDE_PAD_HEIGHT = 0.084
GUIDE_PAD_X = 0.056
GUIDE_PAD_Y = 0.012

CARRIAGE_REST_Z = 0.136
CARRIAGE_TRAVEL = 0.173


def _frame_shell() -> cq.Workplane:
    back_plate = cq.Workplane("XY").box(
        FRAME_WIDTH,
        BACK_PLATE_DEPTH,
        FRAME_HEIGHT,
    ).translate((0.0, -0.021, FRAME_HEIGHT / 2.0))

    base = cq.Workplane("XY").box(
        0.206,
        FRAME_DEPTH,
        0.024,
    ).translate((0.0, 0.0, 0.012))

    left_column = cq.Workplane("XY").box(
        COLUMN_WIDTH,
        COLUMN_DEPTH,
        COLUMN_HEIGHT,
    ).translate((-COLUMN_OFFSET_X, 0.006, 0.222))
    right_column = cq.Workplane("XY").box(
        COLUMN_WIDTH,
        COLUMN_DEPTH,
        COLUMN_HEIGHT,
    ).translate((COLUMN_OFFSET_X, 0.006, 0.222))

    top_bridge = cq.Workplane("XY").box(
        TOP_BRIDGE_WIDTH,
        TOP_BRIDGE_DEPTH,
        TOP_BRIDGE_HEIGHT,
    ).translate((0.0, 0.006, 0.403))
    guide_support = cq.Workplane("XY").box(
        GUIDE_SUPPORT_WIDTH,
        GUIDE_SUPPORT_DEPTH,
        GUIDE_SUPPORT_HEIGHT,
    ).translate((0.0, -0.020, 0.049))
    guide_spine = cq.Workplane("XY").box(
        GUIDE_WIDTH,
        GUIDE_DEPTH,
        GUIDE_HEIGHT,
    ).translate((0.0, -0.002, 0.239))

    shell = (
        back_plate
        .union(base)
        .union(left_column)
        .union(right_column)
        .union(top_bridge)
        .union(guide_support)
        .union(guide_spine)
    )
    return shell


def _runner_body() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        RUNNER_BODY_WIDTH,
        RUNNER_BODY_DEPTH,
        RUNNER_BODY_HEIGHT,
    ).translate((0.0, RUNNER_BODY_Y, 0.0))

    neck = cq.Workplane("XY").box(
        RUNNER_NECK_WIDTH,
        RUNNER_NECK_DEPTH,
        RUNNER_NECK_HEIGHT,
    ).translate((0.0, RUNNER_NECK_Y, 0.0))

    slot = cq.Workplane("XY").box(
        RUNNER_SLOT_WIDTH,
        RUNNER_SLOT_DEPTH,
        RUNNER_SLOT_HEIGHT,
    )

    return body.union(neck).cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_slide_unit")

    model.material("frame_paint", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("carriage_alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("stop_black", rgba=(0.08, 0.08, 0.09, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        Box((FRAME_WIDTH, BACK_PLATE_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.021, FRAME_HEIGHT / 2.0)),
        material="frame_paint",
        name="back_plate",
    )
    back_frame.visual(
        Box((0.206, FRAME_DEPTH, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="frame_paint",
        name="base_shoe",
    )
    back_frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_OFFSET_X, 0.006, 0.222)),
        material="frame_paint",
        name="left_guide",
    )
    back_frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(COLUMN_OFFSET_X, 0.006, 0.222)),
        material="frame_paint",
        name="right_guide",
    )
    back_frame.visual(
        Box((TOP_BRIDGE_WIDTH, TOP_BRIDGE_DEPTH, TOP_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.006, 0.403)),
        material="frame_paint",
        name="top_bridge",
    )
    back_frame.visual(
        Box((STOP_WIDTH, STOP_DEPTH, STOP_HEIGHT)),
        origin=Origin(xyz=(0.0, STOP_Y, LOWER_STOP_Z)),
        material="stop_black",
        name="lower_stop",
    )
    back_frame.visual(
        Box((STOP_WIDTH, STOP_DEPTH, STOP_HEIGHT)),
        origin=Origin(xyz=(0.0, STOP_Y, UPPER_STOP_Z)),
        material="stop_black",
        name="upper_stop",
    )
    back_frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_runner_body(), "carriage_runner_body"),
        material="carriage_alloy",
        name="runner_body",
    )
    carriage.visual(
        Box((MOUNT_FACE_WIDTH, MOUNT_FACE_DEPTH, MOUNT_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, MOUNT_FACE_Y, 0.0)),
        material="carriage_alloy",
        name="mount_face",
    )
    carriage.visual(
        Box((GUIDE_PAD_WIDTH, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_PAD_X, GUIDE_PAD_Y, 0.0)),
        material="carriage_alloy",
        name="left_pad",
    )
    carriage.visual(
        Box((GUIDE_PAD_WIDTH, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(GUIDE_PAD_X, GUIDE_PAD_Y, 0.0)),
        material="carriage_alloy",
        name="right_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((MOUNT_FACE_WIDTH, 0.067, MOUNT_FACE_HEIGHT)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=1200.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

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
        carriage,
        back_frame,
        elem_a="left_pad",
        elem_b="left_guide",
        name="left guide pad carries the carriage",
    )
    ctx.expect_contact(
        carriage,
        back_frame,
        elem_a="right_pad",
        elem_b="right_guide",
        name="right guide pad carries the carriage",
    )
    ctx.expect_gap(
        carriage,
        back_frame,
        axis="y",
        positive_elem="mount_face",
        min_gap=0.015,
        name="mounting face stays proud of the back frame",
    )
    ctx.expect_gap(
        carriage,
        back_frame,
        axis="z",
        positive_elem="runner_body",
        negative_elem="lower_stop",
        min_gap=0.0005,
        max_gap=0.0035,
        name="lower stop sits just below the carriage at home",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.expect_gap(
            back_frame,
            carriage,
            axis="z",
            positive_elem="upper_stop",
            negative_elem="runner_body",
            min_gap=0.0005,
            max_gap=0.0035,
            name="upper stop caps the carriage at full extension",
        )
        ctx.expect_gap(
            carriage,
            back_frame,
            axis="y",
            positive_elem="mount_face",
            min_gap=0.015,
            name="mounting face remains in front when extended",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates upward along the guide path",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
