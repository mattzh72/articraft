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


BASE_LENGTH = 0.40
BASE_WIDTH = 0.32
BASE_THICKNESS = 0.02
BASE_RAIL_LENGTH = 0.38
BASE_RAIL_WIDTH = 0.032
BASE_RAIL_HEIGHT = 0.012
BASE_RAIL_Y = 0.09

LOWER_TRAVEL = 0.04
LOWER_RUNNER_LENGTH = 0.30
LOWER_RUNNER_WIDTH = 0.038
LOWER_RUNNER_HEIGHT = 0.014
LOWER_RUNNER_Y = BASE_RAIL_Y
LOWER_BODY_LENGTH = 0.30
LOWER_BODY_WIDTH = 0.20
LOWER_BODY_HEIGHT = 0.018
UPPER_RAIL_LENGTH = 0.26
UPPER_RAIL_WIDTH = 0.028
UPPER_RAIL_HEIGHT = 0.010
UPPER_RAIL_X = 0.045

UPPER_TRAVEL = 0.04
UPPER_RUNNER_LENGTH = 0.18
UPPER_RUNNER_WIDTH = 0.032
UPPER_RUNNER_HEIGHT = 0.014
UPPER_RUNNER_X = UPPER_RAIL_X
UPPER_BODY_LENGTH_X = 0.18
UPPER_BODY_LENGTH_Y = 0.18
UPPER_BODY_HEIGHT = 0.016
UPPER_PAD_X = 0.11
UPPER_PAD_Y = 0.11
UPPER_PAD_HEIGHT = 0.006

TABLE_LENGTH = 0.18
TABLE_WIDTH = 0.18
TABLE_THICKNESS = 0.012


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            BASE_LENGTH,
            BASE_WIDTH,
            BASE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.005)
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.145, -0.105),
                (-0.145, 0.105),
                (0.145, -0.105),
                (0.145, 0.105),
            ]
        )
        .hole(0.008)
    )

    rail = (
        cq.Workplane("XY")
        .box(
            BASE_RAIL_LENGTH,
            BASE_RAIL_WIDTH,
            BASE_RAIL_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.0025)
    )

    left_rail = rail.translate((0.0, BASE_RAIL_Y, BASE_THICKNESS))
    right_rail = rail.translate((0.0, -BASE_RAIL_Y, BASE_THICKNESS))

    return plate.union(left_rail).union(right_rail)


def _lower_carriage_shape() -> cq.Workplane:
    runner = cq.Workplane("XY").box(
        LOWER_RUNNER_LENGTH,
        LOWER_RUNNER_WIDTH,
        LOWER_RUNNER_HEIGHT,
        centered=(True, True, False),
    )
    bridge = cq.Workplane("XY").box(
        LOWER_BODY_LENGTH,
        LOWER_BODY_WIDTH,
        LOWER_BODY_HEIGHT,
        centered=(True, True, False),
    )
    cross_rail = cq.Workplane("XY").box(
        UPPER_RAIL_WIDTH,
        UPPER_RAIL_LENGTH,
        UPPER_RAIL_HEIGHT,
        centered=(True, True, False),
    )

    body = (
        runner.translate((0.0, LOWER_RUNNER_Y, 0.0))
        .union(runner.translate((0.0, -LOWER_RUNNER_Y, 0.0)))
        .union(bridge.translate((0.0, 0.0, LOWER_RUNNER_HEIGHT)))
        .union(cross_rail.translate((UPPER_RAIL_X, 0.0, LOWER_RUNNER_HEIGHT + LOWER_BODY_HEIGHT)))
        .union(cross_rail.translate((-UPPER_RAIL_X, 0.0, LOWER_RUNNER_HEIGHT + LOWER_BODY_HEIGHT)))
    )

    body = (
        body.faces(">Z")
        .workplane()
        .rect(0.17, 0.08)
        .cutBlind(-0.007)
    )

    return body.edges("|Z").fillet(0.003)


def _upper_carriage_shape() -> cq.Workplane:
    runner = cq.Workplane("XY").box(
        UPPER_RUNNER_WIDTH,
        UPPER_RUNNER_LENGTH,
        UPPER_RUNNER_HEIGHT,
        centered=(True, True, False),
    )
    bridge = cq.Workplane("XY").box(
        UPPER_BODY_LENGTH_X,
        UPPER_BODY_LENGTH_Y,
        UPPER_BODY_HEIGHT,
        centered=(True, True, False),
    )
    pad = cq.Workplane("XY").box(
        UPPER_PAD_X,
        UPPER_PAD_Y,
        UPPER_PAD_HEIGHT,
        centered=(True, True, False),
    )

    body = (
        runner.translate((UPPER_RUNNER_X, 0.0, 0.0))
        .union(runner.translate((-UPPER_RUNNER_X, 0.0, 0.0)))
        .union(bridge.translate((0.0, 0.0, UPPER_RUNNER_HEIGHT)))
        .union(pad.translate((0.0, 0.0, UPPER_RUNNER_HEIGHT + UPPER_BODY_HEIGHT)))
    )

    body = (
        body.faces(">Z")
        .workplane()
        .rect(0.12, 0.05)
        .cutBlind(-0.004)
    )

    return body.edges("|Z").fillet(0.003)


def _table_shape() -> cq.Workplane:
    table = (
        cq.Workplane("XY")
        .box(TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    table = (
        table.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.055, -0.055),
                (-0.055, 0.055),
                (0.055, -0.055),
                (0.055, 0.055),
            ]
        )
        .hole(0.008)
    )

    return table


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_plate_xy_stage")

    model.material("base_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carriage_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("table_silver", rgba=(0.83, 0.85, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "xy_stage_base"),
        material="base_black",
        name="base_body",
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "xy_stage_lower_carriage"),
        material="carriage_gray",
        name="lower_body",
    )

    upper_carriage = model.part("upper_carriage")
    upper_carriage.visual(
        mesh_from_cadquery(_upper_carriage_shape(), "xy_stage_upper_carriage"),
        material="carriage_gray",
        name="upper_body",
    )

    table = model.part("table")
    table.visual(
        mesh_from_cadquery(_table_shape(), "xy_stage_table"),
        material="table_silver",
        name="table_plate",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
            effort=250.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_RUNNER_HEIGHT + LOWER_BODY_HEIGHT + UPPER_RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_TRAVEL,
            upper=UPPER_TRAVEL,
            effort=180.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "upper_to_table",
        ArticulationType.FIXED,
        parent=upper_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, UPPER_RUNNER_HEIGHT + UPPER_BODY_HEIGHT + UPPER_PAD_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_carriage = object_model.get_part("upper_carriage")
    table = object_model.get_part("table")
    lower_slide = object_model.get_articulation("base_to_lower")
    upper_slide = object_model.get_articulation("lower_to_upper")
    table_mount = object_model.get_articulation("upper_to_table")

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
        "lower slide axis runs along x",
        tuple(lower_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {lower_slide.axis}",
    )
    ctx.check(
        "upper slide axis runs along y",
        tuple(upper_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {upper_slide.axis}",
    )
    ctx.check(
        "slides are perpendicular and horizontal",
        (
            abs(lower_slide.axis[2]) < 1e-9
            and abs(upper_slide.axis[2]) < 1e-9
            and sum(a * b for a, b in zip(lower_slide.axis, upper_slide.axis)) == 0.0
        ),
        details=(
            f"lower axis={lower_slide.axis}, upper axis={upper_slide.axis}; "
            "expected perpendicular horizontal directions"
        ),
    )
    ctx.check(
        "table mount is fixed",
        table_mount.articulation_type == ArticulationType.FIXED,
        details=f"expected FIXED, got {table_mount.articulation_type}",
    )

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        ctx.expect_contact(
            lower_carriage,
            base,
            name="lower carriage is supported by base rails",
        )
        ctx.expect_contact(
            upper_carriage,
            lower_carriage,
            name="upper carriage is supported by lower carriage rails",
        )
        ctx.expect_contact(
            table,
            upper_carriage,
            name="table is mounted to upper carriage",
        )
        ctx.expect_gap(
            table,
            base,
            axis="z",
            min_gap=0.045,
            name="table stays above grounded base",
        )

    rest_lower = ctx.part_world_position(lower_carriage)
    with ctx.pose({lower_slide: lower_slide.motion_limits.upper}):
        moved_lower = ctx.part_world_position(lower_carriage)
    ctx.check(
        "lower carriage translates only along x",
        (
            rest_lower is not None
            and moved_lower is not None
            and moved_lower[0] > rest_lower[0] + 0.03
            and abs(moved_lower[1] - rest_lower[1]) < 1e-6
            and abs(moved_lower[2] - rest_lower[2]) < 1e-6
        ),
        details=f"rest={rest_lower}, moved={moved_lower}",
    )

    rest_upper = ctx.part_world_position(upper_carriage)
    with ctx.pose({upper_slide: upper_slide.motion_limits.upper}):
        moved_upper = ctx.part_world_position(upper_carriage)
    ctx.check(
        "upper carriage translates only along y",
        (
            rest_upper is not None
            and moved_upper is not None
            and moved_upper[1] > rest_upper[1] + 0.03
            and abs(moved_upper[0] - rest_upper[0]) < 1e-6
            and abs(moved_upper[2] - rest_upper[2]) < 1e-6
        ),
        details=f"rest={rest_upper}, moved={moved_upper}",
    )

    with ctx.pose(
        {
            lower_slide: lower_slide.motion_limits.upper,
            upper_slide: upper_slide.motion_limits.lower,
        }
    ):
        ctx.expect_contact(
            lower_carriage,
            base,
            name="lower carriage stays supported at x-limit",
        )
        ctx.expect_contact(
            upper_carriage,
            lower_carriage,
            name="upper carriage stays supported at diagonal travel limit",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at diagonal travel limit")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
