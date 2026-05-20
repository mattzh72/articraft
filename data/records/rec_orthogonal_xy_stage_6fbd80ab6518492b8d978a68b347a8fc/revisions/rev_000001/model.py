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


BASE_LENGTH = 0.68
BASE_WIDTH = 0.40
BED_HEIGHT = 0.028
RAIL_PAD_LENGTH = 0.62
RAIL_PAD_WIDTH = 0.070
RAIL_PAD_HEIGHT = 0.022
RAIL_LENGTH = 0.60
RAIL_WIDTH = 0.040
RAIL_HEIGHT = 0.025
RAIL_CENTER_Y = 0.125
BASE_TOTAL_HEIGHT = BED_HEIGHT + RAIL_PAD_HEIGHT + RAIL_HEIGHT

X_CARRIAGE_LENGTH = 0.50
X_CARRIAGE_WIDTH = 0.32
X_RUNNER_LENGTH = 0.46
X_RUNNER_WIDTH = 0.050
X_RUNNER_HEIGHT = 0.045
X_CARRIAGE_BODY_HEIGHT = 0.034
Y_GUIDE_X = 0.180
Y_GUIDE_Y = 0.300
Y_GUIDE_HEIGHT = 0.018
X_CARRIAGE_TOTAL_HEIGHT = X_RUNNER_HEIGHT + X_CARRIAGE_BODY_HEIGHT + Y_GUIDE_HEIGHT
X_CARRIAGE_BOTTOM_Z = BASE_TOTAL_HEIGHT
X_TRAVEL = 0.12

Y_SLIDE_LENGTH = 0.30
Y_SLIDE_WIDTH = 0.25
Y_SLIDE_HEIGHT = 0.080
Y_SLIDE_SHOE_LENGTH = 0.16
Y_SLIDE_SHOE_WIDTH = 0.22
Y_SLIDE_SHOE_HEIGHT = 0.040
Y_SLIDE_BOTTOM_Z = X_RUNNER_HEIGHT + X_CARRIAGE_BODY_HEIGHT + Y_GUIDE_HEIGHT
Y_TRAVEL = 0.07


def _base_shape() -> cq.Workplane:
    bed = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BED_HEIGHT,
        centered=(True, True, False),
    )

    for rail_sign in (-1.0, 1.0):
        rail_y = rail_sign * RAIL_CENTER_Y
        pad = (
            cq.Workplane("XY")
            .center(0.0, rail_y)
            .box(
                RAIL_PAD_LENGTH,
                RAIL_PAD_WIDTH,
                RAIL_PAD_HEIGHT,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, BED_HEIGHT))
        )
        rail = (
            cq.Workplane("XY")
            .center(0.0, rail_y)
            .box(
                RAIL_LENGTH,
                RAIL_WIDTH,
                RAIL_HEIGHT,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, BED_HEIGHT + RAIL_PAD_HEIGHT))
        )
        bed = bed.union(pad).union(rail)

    return bed


def _x_carriage_shape() -> cq.Workplane:
    top_plate = cq.Workplane("XY").box(
        X_CARRIAGE_LENGTH,
        X_CARRIAGE_WIDTH,
        X_CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, X_RUNNER_HEIGHT))
    carriage = top_plate
    for rail_sign in (-1.0, 1.0):
        runner = (
            cq.Workplane("XY")
            .center(0.0, rail_sign * RAIL_CENTER_Y)
            .box(
                X_RUNNER_LENGTH,
                X_RUNNER_WIDTH,
                X_RUNNER_HEIGHT,
                centered=(True, True, False),
            )
        )
        carriage = carriage.union(runner)
    guide = (
        cq.Workplane("XY")
        .box(
            Y_GUIDE_X,
            Y_GUIDE_Y,
            Y_GUIDE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, X_RUNNER_HEIGHT + X_CARRIAGE_BODY_HEIGHT))
    )
    return carriage.union(guide)


def _y_slide_shape() -> cq.Workplane:
    tooling_plate = cq.Workplane("XY").box(
        Y_SLIDE_LENGTH,
        Y_SLIDE_WIDTH,
        Y_SLIDE_HEIGHT - Y_SLIDE_SHOE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, Y_SLIDE_SHOE_HEIGHT))
    shoe = cq.Workplane("XY").box(
        Y_SLIDE_SHOE_LENGTH,
        Y_SLIDE_SHOE_WIDTH,
        Y_SLIDE_SHOE_HEIGHT,
        centered=(True, True, False),
    )
    return tooling_plate.union(shoe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_table_xy_stage")

    model.material("cast_base", rgba=(0.42, 0.45, 0.48, 1.0))
    model.material("ground_steel", rgba=(0.68, 0.71, 0.75, 1.0))
    model.material("slide_finish", rgba=(0.78, 0.80, 0.84, 1.0))

    base = model.part("base_rail_set")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_rail_set"),
        material="cast_base",
        name="base_casting",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_TOTAL_HEIGHT)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_shape(), "x_carriage"),
        material="ground_steel",
        name="x_saddle",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, X_CARRIAGE_TOTAL_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_TOTAL_HEIGHT / 2.0)),
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_y_slide_shape(), "y_slide"),
        material="slide_finish",
        name="y_slide_body",
    )
    y_slide.inertial = Inertial.from_geometry(
        Box((Y_SLIDE_LENGTH, Y_SLIDE_WIDTH, Y_SLIDE_HEIGHT)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, Y_SLIDE_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2800.0,
            velocity=0.35,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, Y_SLIDE_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.30,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

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

    base = object_model.get_part("base_rail_set")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")

    ctx.check(
        "orthogonal prismatic slide stack",
        base_to_x.articulation_type == ArticulationType.PRISMATIC
        and x_to_y.articulation_type == ArticulationType.PRISMATIC
        and tuple(base_to_x.axis) == (1.0, 0.0, 0.0)
        and tuple(x_to_y.axis) == (0.0, 1.0, 0.0),
        details=f"base_to_x axis={base_to_x.axis}, x_to_y axis={x_to_y.axis}",
    )

    ctx.expect_within(
        x_carriage,
        base,
        axes="y",
        margin=0.0,
        name="x carriage stays captured between the base rails in Y",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="x",
        min_overlap=0.44,
        name="x carriage retains long engagement on the base rail set at center",
    )
    ctx.expect_within(
        y_slide,
        x_carriage,
        axes="x",
        margin=0.0,
        name="y slide stays centered on the x carriage in X",
    )
    ctx.expect_overlap(
        y_slide,
        x_carriage,
        axes="y",
        min_overlap=0.24,
        name="y slide retains engagement on the x carriage guide at center",
    )

    x_rest = ctx.part_world_position(x_carriage)
    with ctx.pose({base_to_x: X_TRAVEL}):
        ctx.expect_within(
            x_carriage,
            base,
            axes="y",
            margin=0.0,
            name="x carriage remains laterally captured at full +X travel",
        )
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            min_overlap=0.38,
            name="x carriage still overlaps the fixed rail set at full +X travel",
        )
        x_extended = ctx.part_world_position(x_carriage)

    y_rest = ctx.part_world_position(y_slide)
    with ctx.pose({base_to_x: X_TRAVEL, x_to_y: Y_TRAVEL}):
        ctx.expect_within(
            y_slide,
            x_carriage,
            axes="x",
            margin=0.0,
            name="y slide stays centered on the x carriage at combined travel",
        )
        ctx.expect_overlap(
            y_slide,
            x_carriage,
            axes="y",
            min_overlap=0.16,
            name="y slide keeps retained insertion on the y guide at full +Y travel",
        )
        y_extended = ctx.part_world_position(y_slide)

    ctx.check(
        "x carriage moves toward +X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.10,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "y slide moves toward +Y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.05,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
