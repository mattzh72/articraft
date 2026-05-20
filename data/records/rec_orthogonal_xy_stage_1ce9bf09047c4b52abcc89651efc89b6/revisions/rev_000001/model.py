from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.320
BASE_WIDTH = 0.180
BASE_PLATE_THICKNESS = 0.016
LOWER_RAIL_LENGTH = 0.260
LOWER_RAIL_WIDTH = 0.028
LOWER_RAIL_HEIGHT = 0.012
LOWER_RAIL_Y = 0.052

X_CARRIAGE_LENGTH = 0.220
X_CARRIAGE_WIDTH = 0.148
X_CARRIAGE_BODY_HEIGHT = 0.020
X_CARRIAGE_PEDESTAL_LENGTH = 0.152
X_CARRIAGE_PEDESTAL_WIDTH = 0.108
X_CARRIAGE_PEDESTAL_HEIGHT = 0.012
UPPER_RAIL_X = 0.036
UPPER_RAIL_WIDTH = 0.018
UPPER_RAIL_LENGTH = 0.120
UPPER_RAIL_HEIGHT = 0.010

Y_CARRIAGE_BLOCK_LENGTH = 0.108
Y_CARRIAGE_BLOCK_WIDTH = 0.100
Y_CARRIAGE_BLOCK_HEIGHT = 0.018
Y_CARRIAGE_PEDESTAL_LENGTH = 0.074
Y_CARRIAGE_PEDESTAL_WIDTH = 0.066
Y_CARRIAGE_PEDESTAL_HEIGHT = 0.010
TOP_PLATE_LENGTH = 0.150
TOP_PLATE_WIDTH = 0.116
TOP_PLATE_THICKNESS = 0.008

LOWER_TRAVEL = 0.050
UPPER_TRAVEL = 0.035

UPPER_STAGE_Z = (
    X_CARRIAGE_BODY_HEIGHT + X_CARRIAGE_PEDESTAL_HEIGHT + UPPER_RAIL_HEIGHT
)


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_carriage_body_shape() -> cq.Workplane:
    body = _box_shape(
        (X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, X_CARRIAGE_BODY_HEIGHT),
        (0.0, 0.0, X_CARRIAGE_BODY_HEIGHT / 2.0),
    )
    body = body.edges("|Z").fillet(0.003)

    pedestal = _box_shape(
        (
            X_CARRIAGE_PEDESTAL_LENGTH,
            X_CARRIAGE_PEDESTAL_WIDTH,
            X_CARRIAGE_PEDESTAL_HEIGHT,
        ),
        (
            0.0,
            0.0,
            X_CARRIAGE_BODY_HEIGHT + X_CARRIAGE_PEDESTAL_HEIGHT / 2.0,
        ),
    ).edges("|Z").fillet(0.002)

    side_relief_left = _box_shape((0.060, 0.024, 0.010), (-0.058, 0.0, 0.015))
    side_relief_right = _box_shape((0.060, 0.024, 0.010), (0.058, 0.0, 0.015))

    return body.cut(side_relief_left).cut(side_relief_right).union(pedestal)


def _y_carriage_shape() -> cq.Workplane:
    lower_block = _box_shape(
        (Y_CARRIAGE_BLOCK_LENGTH, Y_CARRIAGE_BLOCK_WIDTH, Y_CARRIAGE_BLOCK_HEIGHT),
        (0.0, 0.0, Y_CARRIAGE_BLOCK_HEIGHT / 2.0),
    ).edges("|Z").fillet(0.0025)

    pedestal = _box_shape(
        (
            Y_CARRIAGE_PEDESTAL_LENGTH,
            Y_CARRIAGE_PEDESTAL_WIDTH,
            Y_CARRIAGE_PEDESTAL_HEIGHT,
        ),
        (
            0.0,
            0.0,
            Y_CARRIAGE_BLOCK_HEIGHT + Y_CARRIAGE_PEDESTAL_HEIGHT / 2.0,
        ),
    ).edges("|Z").fillet(0.0015)

    top_plate = _box_shape(
        (TOP_PLATE_LENGTH, TOP_PLATE_WIDTH, TOP_PLATE_THICKNESS),
        (
            0.0,
            0.0,
            Y_CARRIAGE_BLOCK_HEIGHT
            + Y_CARRIAGE_PEDESTAL_HEIGHT
            + TOP_PLATE_THICKNESS / 2.0,
        ),
    ).edges("|Z").fillet(0.0015)

    return lower_block.union(pedestal).union(top_plate)


def _check_axis(
    ctx: TestContext,
    *,
    actual: tuple[float, float, float],
    expected: tuple[float, float, float],
    name: str,
) -> bool:
    return ctx.check(
        name,
        all(abs(a - e) < 1e-9 for a, e in zip(actual, expected)),
        f"expected axis {expected}, got {actual}",
    )


def _check_position_delta(
    ctx: TestContext,
    *,
    before: tuple[float, float, float] | None,
    after: tuple[float, float, float] | None,
    expected_delta: tuple[float, float, float],
    name: str,
    tol: float = 1e-6,
) -> bool:
    if before is None or after is None:
        return ctx.fail(name, "missing world position")
    delta = tuple(after[i] - before[i] for i in range(3))
    ok = all(abs(delta[i] - expected_delta[i]) <= tol for i in range(3))
    return ctx.check(
        name,
        ok,
        f"expected delta {expected_delta}, got {delta}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_level_xy_stage")

    model.material("base_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, -(LOWER_RAIL_HEIGHT + BASE_PLATE_THICKNESS / 2.0))
        ),
        material="base_black",
        name="base_plate",
    )
    base.visual(
        Box((LOWER_RAIL_LENGTH, LOWER_RAIL_WIDTH, LOWER_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -LOWER_RAIL_Y, -LOWER_RAIL_HEIGHT / 2.0)),
        material="rail_steel",
        name="lower_rail_left",
    )
    base.visual(
        Box((LOWER_RAIL_LENGTH, LOWER_RAIL_WIDTH, LOWER_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, LOWER_RAIL_Y, -LOWER_RAIL_HEIGHT / 2.0)),
        material="rail_steel",
        name="lower_rail_right",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_body_shape(), "x_carriage_body"),
        material="machined_aluminum",
        name="x_carriage_body",
    )
    x_carriage.visual(
        Box((UPPER_RAIL_WIDTH, UPPER_RAIL_LENGTH, UPPER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                -UPPER_RAIL_X,
                0.0,
                X_CARRIAGE_BODY_HEIGHT
                + X_CARRIAGE_PEDESTAL_HEIGHT
                + UPPER_RAIL_HEIGHT / 2.0,
            )
        ),
        material="rail_steel",
        name="upper_rail_left",
    )
    x_carriage.visual(
        Box((UPPER_RAIL_WIDTH, UPPER_RAIL_LENGTH, UPPER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                UPPER_RAIL_X,
                0.0,
                X_CARRIAGE_BODY_HEIGHT
                + X_CARRIAGE_PEDESTAL_HEIGHT
                + UPPER_RAIL_HEIGHT / 2.0,
            )
        ),
        material="rail_steel",
        name="upper_rail_right",
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_y_carriage_shape(), "y_carriage"),
        material="machined_aluminum",
        name="y_carriage_body",
    )

    model.articulation(
        "base_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
        ),
    )
    model.articulation(
        "x_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STAGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.16,
            lower=-UPPER_TRAVEL,
            upper=UPPER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    x_slide = object_model.get_articulation("base_to_x_slide")
    y_slide = object_model.get_articulation("x_to_y_slide")

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

    _check_axis(
        ctx,
        actual=x_slide.axis,
        expected=(1.0, 0.0, 0.0),
        name="lower_slide_axis_is_horizontal_x",
    )
    _check_axis(
        ctx,
        actual=y_slide.axis,
        expected=(0.0, 1.0, 0.0),
        name="upper_slide_axis_is_horizontal_y",
    )

    ctx.expect_contact(x_carriage, base, name="x_carriage_supported_on_lower_stage")
    ctx.expect_contact(y_carriage, x_carriage, name="y_carriage_supported_on_upper_stage")
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="xy",
        min_overlap=0.12,
        name="lower_stage_has_broad_support_overlap",
    )
    ctx.expect_overlap(
        y_carriage,
        x_carriage,
        axes="xy",
        min_overlap=0.09,
        name="upper_stage_has_broad_support_overlap",
    )
    ctx.expect_origin_gap(
        y_carriage,
        x_carriage,
        axis="z",
        min_gap=UPPER_STAGE_Z - 1e-6,
        max_gap=UPPER_STAGE_Z + 1e-6,
        name="upper_stage_is_split_above_lower_stage",
    )

    x_home = ctx.part_world_position(x_carriage)
    y_home = ctx.part_world_position(y_carriage)

    with ctx.pose({x_slide: LOWER_TRAVEL, y_slide: 0.0}):
        x_shifted = ctx.part_world_position(x_carriage)
        y_following_x = ctx.part_world_position(y_carriage)
        _check_position_delta(
            ctx,
            before=x_home,
            after=x_shifted,
            expected_delta=(LOWER_TRAVEL, 0.0, 0.0),
            name="lower_slide_moves_only_along_x",
        )
        _check_position_delta(
            ctx,
            before=y_home,
            after=y_following_x,
            expected_delta=(LOWER_TRAVEL, 0.0, 0.0),
            name="upper_stage_rides_with_lower_slide_in_x",
        )
        ctx.expect_contact(
            x_carriage,
            base,
            name="x_carriage_remains_supported_at_x_travel_limit",
        )
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="xy",
            min_overlap=0.10,
            name="lower_stage_overlap_persists_at_x_travel_limit",
        )

    with ctx.pose({x_slide: 0.0, y_slide: UPPER_TRAVEL}):
        y_shifted = ctx.part_world_position(y_carriage)
        _check_position_delta(
            ctx,
            before=y_home,
            after=y_shifted,
            expected_delta=(0.0, UPPER_TRAVEL, 0.0),
            name="upper_slide_moves_only_along_y",
        )
        ctx.expect_contact(
            y_carriage,
            x_carriage,
            name="y_carriage_remains_supported_at_y_travel_limit",
        )
        ctx.expect_overlap(
            y_carriage,
            x_carriage,
            axes="xy",
            min_overlap=0.08,
            name="upper_stage_overlap_persists_at_y_travel_limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
