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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.22
BASE_THICKNESS = 0.022
LOWER_RAIL_LENGTH = 0.34
LOWER_RAIL_WIDTH = 0.028
LOWER_RAIL_HEIGHT = 0.018
LOWER_RAIL_SPAN = 0.14

X_CARRIAGE_LENGTH = 0.17
X_CARRIAGE_WIDTH = 0.19
X_CARRIAGE_PLATE_THICKNESS = 0.020
X_CARRIAGE_BEARING_BLOCK_WIDTH = 0.042
X_CARRIAGE_BEARING_BLOCK_HEIGHT = 0.012
X_CARRIAGE_SIDE_WEB_THICKNESS = 0.008

UPPER_RAIL_LENGTH = 0.14
UPPER_RAIL_WIDTH = 0.022
UPPER_RAIL_HEIGHT = 0.014
UPPER_RAIL_SPAN = 0.092

PLATFORM_LENGTH_X = 0.12
PLATFORM_LENGTH_Y = 0.13
PLATFORM_THICKNESS = 0.012
PLATFORM_RUNNER_WIDTH = 0.030
PLATFORM_RUNNER_LENGTH = 0.102
PLATFORM_RUNNER_HEIGHT = 0.008

X_TRAVEL = 0.10
Y_TRAVEL = 0.035


def _box_on_floor(length: float, width: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False)).translate(
        (0.0, 0.0, z0)
    )


def _build_lower_rail_shape() -> cq.Workplane:
    shape = _box_on_floor(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
    rail_z0 = BASE_THICKNESS
    for y_pos in (-LOWER_RAIL_SPAN / 2.0, LOWER_RAIL_SPAN / 2.0):
        shape = shape.union(
            _box_on_floor(LOWER_RAIL_LENGTH, LOWER_RAIL_WIDTH, LOWER_RAIL_HEIGHT, rail_z0).translate(
                (0.0, y_pos, 0.0)
            )
        )
    return shape


def _build_x_carriage_shape() -> cq.Workplane:
    plate_z0 = X_CARRIAGE_BEARING_BLOCK_HEIGHT
    upper_rail_z0 = plate_z0 + X_CARRIAGE_PLATE_THICKNESS

    shape = cq.Workplane("XY")
    for y_pos in (-LOWER_RAIL_SPAN / 2.0, LOWER_RAIL_SPAN / 2.0):
        shape = shape.union(
            _box_on_floor(
                X_CARRIAGE_LENGTH,
                X_CARRIAGE_BEARING_BLOCK_WIDTH,
                X_CARRIAGE_BEARING_BLOCK_HEIGHT,
                0.0,
            ).translate((0.0, y_pos, 0.0))
        )

    shape = shape.union(
        _box_on_floor(
            X_CARRIAGE_LENGTH,
            X_CARRIAGE_WIDTH,
            X_CARRIAGE_PLATE_THICKNESS,
            plate_z0,
        )
    )

    web_y = LOWER_RAIL_SPAN / 2.0 + X_CARRIAGE_BEARING_BLOCK_WIDTH / 2.0 - X_CARRIAGE_SIDE_WEB_THICKNESS / 2.0
    for y_pos in (-web_y, web_y):
        shape = shape.union(
            _box_on_floor(
                X_CARRIAGE_LENGTH,
                X_CARRIAGE_SIDE_WEB_THICKNESS,
                upper_rail_z0,
                0.0,
            ).translate((0.0, y_pos, 0.0))
        )

    for x_pos in (-UPPER_RAIL_SPAN / 2.0, UPPER_RAIL_SPAN / 2.0):
        shape = shape.union(
            _box_on_floor(
                UPPER_RAIL_WIDTH,
                UPPER_RAIL_LENGTH,
                UPPER_RAIL_HEIGHT,
                upper_rail_z0,
            ).translate((x_pos, 0.0, 0.0))
        )

    return shape


def _build_y_platform_shape() -> cq.Workplane:
    plate_z0 = PLATFORM_RUNNER_HEIGHT

    shape = cq.Workplane("XY")

    for x_pos in (-UPPER_RAIL_SPAN / 2.0, UPPER_RAIL_SPAN / 2.0):
        shape = shape.union(
            _box_on_floor(
                PLATFORM_RUNNER_WIDTH,
                PLATFORM_RUNNER_LENGTH,
                PLATFORM_RUNNER_HEIGHT,
                0.0,
            ).translate((x_pos, 0.0, 0.0))
        )

    shape = shape.union(
        _box_on_floor(
            PLATFORM_LENGTH_X,
            PLATFORM_LENGTH_Y,
            PLATFORM_THICKNESS,
            plate_z0,
        )
    )

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xy_cross_stage")

    model.material("base_black", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("carriage_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("platform_aluminum", rgba=(0.82, 0.84, 0.87, 1.0))

    lower_rail = model.part("lower_rail")
    lower_rail.visual(
        mesh_from_cadquery(_build_lower_rail_shape(), "lower_rail_body"),
        material="base_black",
        name="lower_rail_body",
    )
    lower_rail.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + LOWER_RAIL_HEIGHT)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + LOWER_RAIL_HEIGHT) / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_build_x_carriage_shape(), "x_carriage_body"),
        material="carriage_aluminum",
        name="x_carriage_body",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, 0.046)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    y_platform = model.part("y_platform")
    y_platform.visual(
        mesh_from_cadquery(_build_y_platform_shape(), "y_platform_body"),
        material="platform_aluminum",
        name="y_platform_body",
    )
    y_platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_LENGTH_X, PLATFORM_LENGTH_Y, PLATFORM_RUNNER_HEIGHT + PLATFORM_THICKNESS)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, (PLATFORM_RUNNER_HEIGHT + PLATFORM_THICKNESS) / 2.0)),
    )

    lower_to_x = model.articulation(
        "lower_to_x",
        ArticulationType.PRISMATIC,
        parent=lower_rail,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + LOWER_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=180.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_platform,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                X_CARRIAGE_BEARING_BLOCK_HEIGHT + X_CARRIAGE_PLATE_THICKNESS + UPPER_RAIL_HEIGHT,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=90.0,
            velocity=0.18,
        ),
    )

    model.meta["primary_joint"] = lower_to_x.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    lower_rail = object_model.get_part("lower_rail")
    x_carriage = object_model.get_part("x_carriage")
    y_platform = object_model.get_part("y_platform")
    lower_to_x = object_model.get_articulation("lower_to_x")
    x_to_y = object_model.get_articulation("x_to_y")

    ctx.expect_within(
        x_carriage,
        lower_rail,
        axes="y",
        margin=0.0,
        name="x carriage stays laterally over the lower rail",
    )
    ctx.expect_overlap(
        x_carriage,
        lower_rail,
        axes="x",
        min_overlap=0.15,
        name="x carriage has substantial support length at rest",
    )
    ctx.expect_within(
        y_platform,
        x_carriage,
        axes="x",
        margin=0.0,
        name="y platform stays centered between the upper guides",
    )
    ctx.expect_overlap(
        y_platform,
        x_carriage,
        axes="y",
        min_overlap=0.08,
        name="y platform remains supported on the short slide at rest",
    )
    ctx.expect_contact(
        x_carriage,
        lower_rail,
        name="x carriage bears on the lower guide rails",
    )
    ctx.expect_contact(
        y_platform,
        x_carriage,
        name="y platform bears on the upper guide rails",
    )

    x_rest = ctx.part_world_position(x_carriage)
    with ctx.pose({lower_to_x: X_TRAVEL}):
        ctx.expect_within(
            x_carriage,
            lower_rail,
            axes="y",
            margin=0.0,
            name="x carriage stays laterally aligned at full x travel",
        )
        ctx.expect_overlap(
            x_carriage,
            lower_rail,
            axes="x",
            min_overlap=0.12,
            name="x carriage keeps guide engagement at full x travel",
        )
        x_extended = ctx.part_world_position(x_carriage)

    y_rest = ctx.part_world_position(y_platform)
    with ctx.pose({x_to_y: Y_TRAVEL}):
        ctx.expect_within(
            y_platform,
            x_carriage,
            axes="x",
            margin=0.0,
            name="y platform stays centered in x at full y travel",
        )
        ctx.expect_overlap(
            y_platform,
            x_carriage,
            axes="y",
            min_overlap=0.06,
            name="y platform keeps guide engagement at full y travel",
        )
        y_extended = ctx.part_world_position(y_platform)

    ctx.check(
        "x carriage moves along +X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.05,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "y platform moves along +Y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.02,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
