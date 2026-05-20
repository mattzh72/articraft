from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.180
BASE_WIDTH = 0.160
BASE_THICK = 0.012

GUIDE_LENGTH = 0.220
REAR_BLOCK_LENGTH = 0.038
ARM_THICK = 0.018
INNER_WIDTH = 0.060
OUTER_WIDTH = INNER_WIDTH + 2.0 * ARM_THICK

SIDE_HEIGHT = 0.072
GUIDE_CENTER_Z = 0.050
INNER_HEIGHT = 0.036
FLOOR_THICK = 0.010
ROOF_THICK = 0.010
ROOF_LENGTH = 0.205

FLOOR_TOP_Z = GUIDE_CENTER_Z - INNER_HEIGHT / 2.0
FLOOR_BOTTOM_Z = FLOOR_TOP_Z - FLOOR_THICK
ROOF_BOTTOM_Z = GUIDE_CENTER_Z + INNER_HEIGHT / 2.0

RUNNER_BAR_LENGTH = 0.240
RUNNER_BAR_WIDTH = 0.050
RUNNER_BAR_HEIGHT = 0.026
END_PLATE_THICK = 0.008
END_PLATE_WIDTH = 0.078
END_PLATE_HEIGHT = 0.080
PLATE_OVERLAP = 0.002

TRAVEL = 0.130


def _origin_xyz(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_captured_linear_runner")

    model.material("frame_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("runner_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("plate_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICK)),
        origin=_origin_xyz(-0.050 + BASE_LENGTH / 2.0, 0.0, BASE_THICK / 2.0),
        material="frame_dark",
        name="base_plate",
    )
    frame.visual(
        Box((REAR_BLOCK_LENGTH, OUTER_WIDTH, SIDE_HEIGHT)),
        origin=_origin_xyz(
            -REAR_BLOCK_LENGTH / 2.0,
            0.0,
            BASE_THICK + SIDE_HEIGHT / 2.0,
        ),
        material="frame_dark",
        name="rear_block",
    )
    frame.visual(
        Box((GUIDE_LENGTH, ARM_THICK, SIDE_HEIGHT)),
        origin=_origin_xyz(
            GUIDE_LENGTH / 2.0,
            INNER_WIDTH / 2.0 + ARM_THICK / 2.0,
            BASE_THICK + SIDE_HEIGHT / 2.0,
        ),
        material="frame_dark",
        name="right_arm",
    )
    frame.visual(
        Box((GUIDE_LENGTH, ARM_THICK, SIDE_HEIGHT)),
        origin=_origin_xyz(
            GUIDE_LENGTH / 2.0,
            -(INNER_WIDTH / 2.0 + ARM_THICK / 2.0),
            BASE_THICK + SIDE_HEIGHT / 2.0,
        ),
        material="frame_dark",
        name="left_arm",
    )
    frame.visual(
        Box((GUIDE_LENGTH, INNER_WIDTH, FLOOR_THICK)),
        origin=_origin_xyz(
            GUIDE_LENGTH / 2.0,
            0.0,
            FLOOR_BOTTOM_Z + FLOOR_THICK / 2.0,
        ),
        material="frame_dark",
        name="floor_strip",
    )
    frame.visual(
        Box((ROOF_LENGTH, INNER_WIDTH, ROOF_THICK)),
        origin=_origin_xyz(
            ROOF_LENGTH / 2.0,
            0.0,
            ROOF_BOTTOM_Z + ROOF_THICK / 2.0,
        ),
        material="frame_dark",
        name="roof_strip",
    )
    frame.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH + REAR_BLOCK_LENGTH, BASE_WIDTH, BASE_THICK + SIDE_HEIGHT)),
        mass=4.8,
        origin=_origin_xyz(
            (GUIDE_LENGTH - REAR_BLOCK_LENGTH) / 2.0,
            0.0,
            (BASE_THICK + SIDE_HEIGHT) / 2.0,
        ),
    )

    runner = model.part("runner")
    runner.visual(
        Box((RUNNER_BAR_LENGTH, RUNNER_BAR_WIDTH, RUNNER_BAR_HEIGHT)),
        origin=_origin_xyz(RUNNER_BAR_LENGTH / 2.0, 0.0, 0.0),
        material="runner_steel",
        name="runner_bar",
    )
    runner.visual(
        Box((END_PLATE_THICK, END_PLATE_WIDTH, END_PLATE_HEIGHT)),
        origin=_origin_xyz(
            RUNNER_BAR_LENGTH - PLATE_OVERLAP + END_PLATE_THICK / 2.0,
            0.0,
            0.0,
        ),
        material="plate_steel",
        name="end_plate",
    )
    runner.inertial = Inertial.from_geometry(
        Box(
            (
                RUNNER_BAR_LENGTH + END_PLATE_THICK - PLATE_OVERLAP,
                END_PLATE_WIDTH,
                END_PLATE_HEIGHT,
            )
        ),
        mass=1.2,
        origin=_origin_xyz(
            (RUNNER_BAR_LENGTH + END_PLATE_THICK - PLATE_OVERLAP) / 2.0,
            0.0,
            0.0,
        ),
    )

    model.articulation(
        "frame_to_runner",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=runner,
        origin=_origin_xyz(0.0, 0.0, GUIDE_CENTER_Z),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TRAVEL,
            effort=80.0,
            velocity=0.25,
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

    frame = object_model.get_part("frame")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("frame_to_runner")

    ctx.expect_gap(
        runner,
        frame,
        axis="y",
        positive_elem="runner_bar",
        negative_elem="left_arm",
        min_gap=0.004,
        name="runner bar clears the left guide arm",
    )
    ctx.expect_gap(
        frame,
        runner,
        axis="y",
        positive_elem="right_arm",
        negative_elem="runner_bar",
        min_gap=0.004,
        name="runner bar clears the right guide arm",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="z",
        positive_elem="runner_bar",
        negative_elem="floor_strip",
        min_gap=0.004,
        name="runner bar clears the lower guide strip",
    )
    ctx.expect_gap(
        frame,
        runner,
        axis="z",
        positive_elem="roof_strip",
        negative_elem="runner_bar",
        min_gap=0.004,
        name="runner bar clears the upper keeper strip",
    )
    ctx.expect_overlap(
        runner,
        frame,
        axes="x",
        elem_a="runner_bar",
        elem_b="floor_strip",
        min_overlap=0.200,
        name="runner remains deeply inserted at rest",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="x",
        positive_elem="end_plate",
        negative_elem="left_arm",
        min_gap=0.010,
        name="plain end plate sits ahead of the fork tips",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: TRAVEL}):
        ctx.expect_gap(
            runner,
            frame,
            axis="y",
            positive_elem="runner_bar",
            negative_elem="left_arm",
            min_gap=0.004,
            name="runner bar keeps left clearance at full extension",
        )
        ctx.expect_gap(
            frame,
            runner,
            axis="y",
            positive_elem="right_arm",
            negative_elem="runner_bar",
            min_gap=0.004,
            name="runner bar keeps right clearance at full extension",
        )
        ctx.expect_overlap(
            runner,
            frame,
            axes="x",
            elem_a="runner_bar",
            elem_b="floor_strip",
            min_overlap=0.090,
            name="runner retains insertion at full extension",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "runner extends along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.100,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
