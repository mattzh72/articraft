from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.74
BODY_WIDTH = 0.60
BODY_HEIGHT = 0.24
BODY_WALL = 0.02
BODY_FLOOR = 0.02
BODY_RUNNER_WIDTH = 0.035
BODY_RUNNER_HEIGHT = 0.006
BODY_RUNNER_TOP = BODY_FLOOR + BODY_RUNNER_HEIGHT

STAGE1_DEPTH = 0.58
STAGE1_WIDTH = 0.54
STAGE1_HEIGHT = 0.08
STAGE1_WALL = 0.012
STAGE1_FLOOR = 0.006
STAGE1_FRONT_HEIGHT = 0.085
STAGE1_TOP_RUNNER_WIDTH = 0.04
STAGE1_TOP_RUNNER_HEIGHT = 0.006
STAGE1_TOP_RUNNER_TOP = STAGE1_HEIGHT + STAGE1_TOP_RUNNER_HEIGHT

STAGE2_DEPTH = 0.42
STAGE2_WIDTH = 0.46
STAGE2_HEIGHT = 0.07
STAGE2_WALL = 0.010
STAGE2_FLOOR = 0.006
STAGE2_FRONT_HEIGHT = 0.11

BODY_TO_STAGE1_X = 0.05
STAGE1_TO_STAGE2_X = 0.08

STAGE1_TRAVEL = 0.24
STAGE2_TRAVEL = 0.26


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_body_shape() -> cq.Workplane:
    outer = _box(
        (BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT),
        (BODY_DEPTH / 2.0, 0.0, BODY_HEIGHT / 2.0),
    )
    cavity = _box(
        (
            BODY_DEPTH - BODY_WALL + 0.02,
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - BODY_FLOOR + 0.02,
        ),
        (
            (BODY_WALL + BODY_DEPTH + 0.02) / 2.0,
            0.0,
            (BODY_FLOOR + BODY_HEIGHT + 0.02) / 2.0,
        ),
    )
    body = outer.cut(cavity)

    runner_length = BODY_DEPTH - BODY_WALL - 0.14
    runner_center_x = BODY_WALL + 0.05 + runner_length / 2.0
    runner_center_y = BODY_WIDTH / 2.0 - BODY_WALL - BODY_RUNNER_WIDTH / 2.0
    runner_center_z = BODY_FLOOR + BODY_RUNNER_HEIGHT / 2.0

    left_runner = _box(
        (runner_length, BODY_RUNNER_WIDTH, BODY_RUNNER_HEIGHT),
        (runner_center_x, runner_center_y, runner_center_z),
    )
    right_runner = _box(
        (runner_length, BODY_RUNNER_WIDTH, BODY_RUNNER_HEIGHT),
        (runner_center_x, -runner_center_y, runner_center_z),
    )

    return body.union(left_runner).union(right_runner)


def _make_stage1_shape() -> cq.Workplane:
    outer = _box(
        (STAGE1_DEPTH, STAGE1_WIDTH, STAGE1_HEIGHT),
        (STAGE1_DEPTH / 2.0, 0.0, STAGE1_HEIGHT / 2.0),
    )
    cavity = _box(
        (
            STAGE1_DEPTH - 2.0 * STAGE1_WALL,
            STAGE1_WIDTH - 2.0 * STAGE1_WALL,
            STAGE1_HEIGHT - STAGE1_FLOOR + 0.02,
        ),
        (
            STAGE1_DEPTH / 2.0,
            0.0,
            (STAGE1_FLOOR + STAGE1_HEIGHT + 0.02) / 2.0,
        ),
    )
    tray = outer.cut(cavity)

    front_face = _box(
        (STAGE1_WALL, STAGE1_WIDTH, STAGE1_FRONT_HEIGHT),
        (
            STAGE1_DEPTH - STAGE1_WALL / 2.0,
            0.0,
            STAGE1_FRONT_HEIGHT / 2.0,
        ),
    )

    upper_runner_length = STAGE1_DEPTH - 0.20
    upper_runner_center_x = STAGE1_WALL + 0.08 + upper_runner_length / 2.0
    upper_runner_center_y = STAGE1_WIDTH / 2.0 - STAGE1_TOP_RUNNER_WIDTH / 2.0
    upper_runner_center_z = STAGE1_HEIGHT + STAGE1_TOP_RUNNER_HEIGHT / 2.0

    left_upper_runner = _box(
        (
            upper_runner_length,
            STAGE1_TOP_RUNNER_WIDTH,
            STAGE1_TOP_RUNNER_HEIGHT,
        ),
        (
            upper_runner_center_x,
            upper_runner_center_y,
            upper_runner_center_z,
        ),
    )
    right_upper_runner = _box(
        (
            upper_runner_length,
            STAGE1_TOP_RUNNER_WIDTH,
            STAGE1_TOP_RUNNER_HEIGHT,
        ),
        (
            upper_runner_center_x,
            -upper_runner_center_y,
            upper_runner_center_z,
        ),
    )

    return tray.union(front_face).union(left_upper_runner).union(right_upper_runner)


def _make_stage2_shape() -> cq.Workplane:
    outer = _box(
        (STAGE2_DEPTH, STAGE2_WIDTH, STAGE2_HEIGHT),
        (STAGE2_DEPTH / 2.0, 0.0, STAGE2_HEIGHT / 2.0),
    )
    cavity = _box(
        (
            STAGE2_DEPTH - 2.0 * STAGE2_WALL,
            STAGE2_WIDTH - 2.0 * STAGE2_WALL,
            STAGE2_HEIGHT - STAGE2_FLOOR + 0.02,
        ),
        (
            STAGE2_DEPTH / 2.0,
            0.0,
            (STAGE2_FLOOR + STAGE2_HEIGHT + 0.02) / 2.0,
        ),
    )
    tray = outer.cut(cavity)

    front_face = _box(
        (STAGE2_WALL, STAGE2_WIDTH, STAGE2_FRONT_HEIGHT),
        (
            STAGE2_DEPTH - STAGE2_WALL / 2.0,
            0.0,
            STAGE2_FRONT_HEIGHT / 2.0,
        ),
    )

    return tray.union(front_face)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_three_stage_slide")

    body_color = model.material("body_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    stage1_color = model.material("stage1_gray", rgba=(0.48, 0.50, 0.52, 1.0))
    stage2_color = model.material("stage2_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "body_shell"),
        material=body_color,
        name="body_shell",
    )

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_make_stage1_shape(), "stage1_carriage"),
        material=stage1_color,
        name="stage1_carriage",
    )

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_make_stage2_shape(), "stage2_carriage"),
        material=stage2_color,
        name="stage2_carriage",
    )

    model.articulation(
        "body_to_stage1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage1,
        origin=Origin(xyz=(BODY_TO_STAGE1_X, 0.0, BODY_RUNNER_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )

    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(STAGE1_TO_STAGE2_X, 0.0, STAGE1_TOP_RUNNER_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    articulation_names = {joint.name for joint in object_model.articulations}

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
        "required_parts_present",
        {"body", "stage1", "stage2"} <= part_names,
        f"found parts: {sorted(part_names)}",
    )
    ctx.check(
        "required_articulations_present",
        {"body_to_stage1", "stage1_to_stage2"} <= articulation_names,
        f"found articulations: {sorted(articulation_names)}",
    )

    body = object_model.get_part("body")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    body_to_stage1 = object_model.get_articulation("body_to_stage1")
    stage1_to_stage2 = object_model.get_articulation("stage1_to_stage2")

    ctx.check(
        "serial_prismatic_configuration",
        (
            body_to_stage1.articulation_type == ArticulationType.PRISMATIC
            and stage1_to_stage2.articulation_type == ArticulationType.PRISMATIC
            and tuple(body_to_stage1.axis) == (1.0, 0.0, 0.0)
            and tuple(stage1_to_stage2.axis) == (1.0, 0.0, 0.0)
            and body_to_stage1.parent == "body"
            and body_to_stage1.child == "stage1"
            and stage1_to_stage2.parent == "stage1"
            and stage1_to_stage2.child == "stage2"
        ),
        "expected two serial +X prismatic joints: body -> stage1 -> stage2",
    )

    ctx.expect_contact(stage1, body, name="stage1_supported_by_body_at_rest")
    ctx.expect_contact(stage2, stage1, name="stage2_supported_by_stage1_at_rest")
    ctx.expect_within(
        stage1,
        body,
        axes="y",
        margin=0.0,
        name="stage1_nested_within_body_width",
    )
    ctx.expect_within(
        stage2,
        stage1,
        axes="xy",
        margin=0.0,
        name="stage2_nested_within_stage1_at_rest",
    )

    with ctx.pose({body_to_stage1: STAGE1_TRAVEL}):
        ctx.expect_origin_gap(
            stage1,
            body,
            axis="x",
            min_gap=STAGE1_TRAVEL - 0.01,
            name="stage1_extends_forward_from_body",
        )

    with ctx.pose({body_to_stage1: STAGE1_TRAVEL, stage1_to_stage2: STAGE2_TRAVEL}):
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=STAGE2_TRAVEL - 0.01,
            name="stage2_extends_forward_from_stage1",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
