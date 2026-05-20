from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.420
OUTER_WIDTH = 0.090
OUTER_HEIGHT = 0.060
OUTER_WALL = 0.004

MIDDLE_LENGTH = 0.380
MIDDLE_WIDTH = 0.078
MIDDLE_HEIGHT = 0.048
MIDDLE_WALL = 0.0035
MIDDLE_FRONT_NOSE = 0.015

INNER_LENGTH = 0.340
INNER_WIDTH = 0.066
INNER_HEIGHT = 0.036
INNER_WALL = 0.003
INNER_FRONT_NOSE = 0.020

OUTER_TO_MIDDLE_TRAVEL = 0.190
MIDDLE_TO_INNER_TRAVEL = 0.150

OUTER_BASE_PLATE_LENGTH = 0.390
OUTER_BASE_PLATE_WIDTH = 0.070
OUTER_BASE_PLATE_THICKNESS = 0.006

MIDDLE_Z_OFFSET = OUTER_WALL
INNER_Z_OFFSET = MIDDLE_WALL


def _tube_center_x(length: float, max_x: float) -> float:
    return max_x - (length / 2.0)


def _add_box_tube_visuals(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    max_x: float,
    material: str,
) -> None:
    center_x = _tube_center_x(length, max_x)

    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(center_x, 0.0, wall / 2.0)),
        material=material,
        name=f"{prefix}_bottom_wall",
    )
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(center_x, 0.0, height - (wall / 2.0))),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, wall, height - (2.0 * wall))),
        origin=Origin(
            xyz=(center_x, -(width / 2.0) + (wall / 2.0), height / 2.0),
        ),
        material=material,
        name=f"{prefix}_left_wall",
    )
    part.visual(
        Box((length, wall, height - (2.0 * wall))),
        origin=Origin(
            xyz=(center_x, (width / 2.0) - (wall / 2.0), height / 2.0),
        ),
        material=material,
        name=f"{prefix}_right_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_box_telescoping_slide")

    model.material("outer_finish", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("middle_finish", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("inner_finish", rgba=(0.76, 0.78, 0.80, 1.0))

    outer = model.part("outer_channel")
    _add_box_tube_visuals(
        outer,
        prefix="outer_channel",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        max_x=0.0,
        material="outer_finish",
    )
    outer.visual(
        Box(
            (
                OUTER_BASE_PLATE_LENGTH,
                OUTER_BASE_PLATE_WIDTH,
                OUTER_BASE_PLATE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                -OUTER_BASE_PLATE_LENGTH / 2.0,
                0.0,
                -OUTER_BASE_PLATE_THICKNESS / 2.0,
            )
        ),
        material="outer_finish",
        name="outer_mounting_base",
    )

    middle = model.part("middle_member")
    _add_box_tube_visuals(
        middle,
        prefix="middle_member",
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        max_x=MIDDLE_FRONT_NOSE,
        material="middle_finish",
    )

    inner = model.part("inner_member")
    _add_box_tube_visuals(
        inner,
        prefix="inner_member",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        max_x=INNER_FRONT_NOSE,
        material="inner_finish",
    )
    inner.visual(
        Box((0.010, 0.072, 0.040)),
        origin=Origin(
            xyz=(INNER_FRONT_NOSE + 0.004, 0.0, INNER_HEIGHT / 2.0),
        ),
        material="inner_finish",
        name="inner_front_plate",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_Z_OFFSET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=120.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_FRONT_NOSE, 0.0, INNER_Z_OFFSET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=90.0,
            velocity=0.45,
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

    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "outer stage slides on +X",
        tuple(outer_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={outer_slide.axis}",
    )
    ctx.check(
        "inner stage slides on +X",
        tuple(inner_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={inner_slide.axis}",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        name="middle member stays nested within outer channel",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.160,
        name="middle member remains inserted in outer channel at rest",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        name="inner member stays nested within middle member",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.260,
        name="inner member remains inserted in middle member at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    inner_rest = ctx.part_world_position(inner)

    with ctx.pose({outer_slide: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            name="extended middle member stays nested within outer channel",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.150,
            name="extended middle member retains insertion in outer channel",
        )
        middle_extended = ctx.part_world_position(middle)

    with ctx.pose({inner_slide: MIDDLE_TO_INNER_TRAVEL}):
        inner_only_extended = ctx.part_world_position(inner)

    with ctx.pose(
        {
            outer_slide: OUTER_TO_MIDDLE_TRAVEL,
            inner_slide: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            name="fully extended inner member stays nested within middle member",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.150,
            name="fully extended inner member retains insertion in middle member",
        )
        inner_fully_extended = ctx.part_world_position(inner)

    ctx.check(
        "middle member moves outward",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner member moves outward on its own stage",
        inner_rest is not None
        and inner_only_extended is not None
        and inner_only_extended[0] > inner_rest[0] + 0.08,
        details=f"rest={inner_rest}, extended={inner_only_extended}",
    )
    ctx.check(
        "inner member reaches farther than middle member under combined extension",
        middle_rest is not None
        and inner_rest is not None
        and inner_fully_extended is not None
        and inner_fully_extended[0] > inner_rest[0] + 0.20,
        details=f"rest={inner_rest}, combined_extended={inner_fully_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
