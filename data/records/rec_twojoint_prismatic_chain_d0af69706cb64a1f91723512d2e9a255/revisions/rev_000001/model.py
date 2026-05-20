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


OUTER_LENGTH = 0.58
OUTER_WIDTH = 0.086
OUTER_HEIGHT = 0.056
OUTER_WALL = 0.0035

MIDDLE_LENGTH = 0.42
MIDDLE_WIDTH = 0.072
MIDDLE_HEIGHT = 0.040
MIDDLE_WALL = 0.003

INNER_LENGTH = 0.33
INNER_WIDTH = 0.050
INNER_HEIGHT = 0.024

OUTER_TO_MIDDLE_TRAVEL = 0.34
MIDDLE_TO_INNER_TRAVEL = 0.24
OUTER_GUIDE_WIDTH = (OUTER_WIDTH - 2.0 * OUTER_WALL - MIDDLE_WIDTH) / 2.0
OUTER_GUIDE_HEIGHT = 0.008
OUTER_GUIDE_Z = 0.028

INNER_GUIDE_WIDTH = (MIDDLE_WIDTH - 2.0 * MIDDLE_WALL - INNER_WIDTH) / 2.0
INNER_GUIDE_HEIGHT = 0.010
INNER_GUIDE_LENGTH = INNER_LENGTH - 0.04
INNER_GUIDE_Z = 0.012


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_guide_extension_chain")

    outer_mat = model.material("outer_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    middle_mat = model.material("middle_finish", rgba=(0.50, 0.53, 0.57, 1.0))
    inner_mat = model.material("inner_finish", rgba=(0.74, 0.76, 0.79, 1.0))

    outer_channel = model.part("outer_channel")
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_WALL)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT - OUTER_WALL / 2.0)),
        material=outer_mat,
        name="outer_roof",
    )
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0,
                (OUTER_HEIGHT - OUTER_WALL) / 2.0,
            )
        ),
        material=outer_mat,
        name="outer_left_web",
    )
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                -OUTER_WIDTH / 2.0 + OUTER_WALL / 2.0,
                (OUTER_HEIGHT - OUTER_WALL) / 2.0,
            )
        ),
        material=outer_mat,
        name="outer_right_web",
    )

    middle_runner = model.part("middle_runner")
    middle_runner.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_WALL)),
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_WALL / 2.0)),
        material=middle_mat,
        name="middle_floor",
    )
    middle_runner.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - MIDDLE_WALL)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                MIDDLE_WIDTH / 2.0 - MIDDLE_WALL / 2.0,
                MIDDLE_WALL + (MIDDLE_HEIGHT - MIDDLE_WALL) / 2.0,
            )
        ),
        material=middle_mat,
        name="middle_left_flange",
    )
    middle_runner.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - MIDDLE_WALL)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                -MIDDLE_WIDTH / 2.0 + MIDDLE_WALL / 2.0,
                MIDDLE_WALL + (MIDDLE_HEIGHT - MIDDLE_WALL) / 2.0,
            )
        ),
        material=middle_mat,
        name="middle_right_flange",
    )
    middle_runner.visual(
        Box((MIDDLE_LENGTH, OUTER_GUIDE_WIDTH, OUTER_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                MIDDLE_WIDTH / 2.0 + OUTER_GUIDE_WIDTH / 2.0,
                OUTER_GUIDE_Z,
            )
        ),
        material=middle_mat,
        name="middle_left_outer_guide",
    )
    middle_runner.visual(
        Box((MIDDLE_LENGTH, OUTER_GUIDE_WIDTH, OUTER_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                -MIDDLE_WIDTH / 2.0 - OUTER_GUIDE_WIDTH / 2.0,
                OUTER_GUIDE_Z,
            )
        ),
        material=middle_mat,
        name="middle_right_outer_guide",
    )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
        material=inner_mat,
        name="inner_runner_body",
    )
    inner_runner.visual(
        Box((INNER_GUIDE_LENGTH, INNER_GUIDE_WIDTH, INNER_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_GUIDE_LENGTH / 2.0,
                INNER_WIDTH / 2.0 + INNER_GUIDE_WIDTH / 2.0,
                INNER_GUIDE_Z,
            )
        ),
        material=inner_mat,
        name="inner_left_guide",
    )
    inner_runner.visual(
        Box((INNER_GUIDE_LENGTH, INNER_GUIDE_WIDTH, INNER_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_GUIDE_LENGTH / 2.0,
                -INNER_WIDTH / 2.0 - INNER_GUIDE_WIDTH / 2.0,
                INNER_GUIDE_Z,
            )
        ),
        material=inner_mat,
        name="inner_right_guide",
    )

    outer_to_middle_z = 0.008
    middle_to_inner_z = 0.006

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=middle_runner,
        origin=Origin(xyz=(0.0, 0.0, outer_to_middle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.5,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(xyz=(0.0, 0.0, middle_to_inner_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.5,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
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

    outer_channel = object_model.get_part("outer_channel")
    middle_runner = object_model.get_part("middle_runner")
    inner_runner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "all expected parts exist",
        all(part is not None for part in (outer_channel, middle_runner, inner_runner)),
    )

    ctx.expect_within(
        middle_runner,
        outer_channel,
        axes="yz",
        margin=0.0,
        name="middle runner stays laterally contained by outer channel",
    )
    ctx.expect_overlap(
        middle_runner,
        outer_channel,
        axes="x",
        min_overlap=0.20,
        name="collapsed middle runner remains deeply inserted in outer channel",
    )
    ctx.expect_contact(
        middle_runner,
        outer_channel,
        elem_a="middle_left_outer_guide",
        elem_b="outer_left_web",
        name="middle runner is supported by the outer channel guide web",
    )

    ctx.expect_within(
        inner_runner,
        middle_runner,
        axes="yz",
        inner_elem="inner_runner_body",
        margin=0.0,
        name="inner runner stays laterally contained by middle runner",
    )
    ctx.expect_overlap(
        inner_runner,
        middle_runner,
        axes="x",
        elem_a="inner_runner_body",
        min_overlap=0.20,
        name="collapsed inner runner remains deeply inserted in middle runner",
    )
    ctx.expect_contact(
        inner_runner,
        middle_runner,
        elem_a="inner_left_guide",
        elem_b="middle_left_flange",
        name="inner runner is supported by the middle runner guide flange",
    )

    rest_middle_pos = ctx.part_world_position(middle_runner)
    rest_inner_pos = ctx.part_world_position(inner_runner)

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle_runner,
            outer_channel,
            axes="yz",
            margin=0.0,
            name="extended middle runner stays guided by outer channel",
        )
        ctx.expect_overlap(
            middle_runner,
            outer_channel,
            axes="x",
            min_overlap=0.08,
            name="extended middle runner still retains insertion in outer channel",
        )
        ctx.expect_contact(
            middle_runner,
            outer_channel,
            elem_a="middle_left_outer_guide",
            elem_b="outer_left_web",
            name="extended middle runner stays on the outer guide web",
        )
        carried_inner_pos = ctx.part_world_position(inner_runner)
        extended_middle_pos = ctx.part_world_position(middle_runner)

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_within(
            inner_runner,
            middle_runner,
            axes="yz",
            inner_elem="inner_runner_body",
            margin=0.0,
            name="extended inner runner stays guided by middle runner",
        )
        ctx.expect_overlap(
            inner_runner,
            middle_runner,
            axes="x",
            elem_a="inner_runner_body",
            min_overlap=0.08,
            name="extended inner runner still retains insertion in middle runner",
        )
        ctx.expect_contact(
            inner_runner,
            middle_runner,
            elem_a="inner_left_guide",
            elem_b="middle_left_flange",
            name="extended inner runner stays on the middle guide flange",
        )
        fully_extended_inner_pos = ctx.part_world_position(inner_runner)
        outer_aabb = ctx.part_world_aabb(outer_channel)
        inner_aabb = ctx.part_world_aabb(inner_runner)

    ctx.check(
        "middle runner extends along +X",
        rest_middle_pos is not None
        and extended_middle_pos is not None
        and extended_middle_pos[0] > rest_middle_pos[0] + 0.25,
        details=f"rest={rest_middle_pos}, extended={extended_middle_pos}",
    )
    ctx.check(
        "middle stage carries inner stage outward",
        rest_inner_pos is not None
        and carried_inner_pos is not None
        and carried_inner_pos[0] > rest_inner_pos[0] + 0.25,
        details=f"rest={rest_inner_pos}, carried={carried_inner_pos}",
    )
    ctx.check(
        "inner runner extends further from the middle stage",
        carried_inner_pos is not None
        and fully_extended_inner_pos is not None
        and fully_extended_inner_pos[0] > carried_inner_pos[0] + 0.15,
        details=f"carried={carried_inner_pos}, fully_extended={fully_extended_inner_pos}",
    )
    ctx.check(
        "plain inner end face projects past outer channel at full extension",
        outer_aabb is not None
        and inner_aabb is not None
        and inner_aabb[1][0] > outer_aabb[1][0] + 0.20,
        details=f"outer_aabb={outer_aabb}, inner_aabb={inner_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
