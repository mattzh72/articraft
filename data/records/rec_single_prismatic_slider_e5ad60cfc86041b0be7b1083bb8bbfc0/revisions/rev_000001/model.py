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


GUIDE_LENGTH = 0.42
GUIDE_WIDTH = 0.07
GUIDE_HEIGHT = 0.052
GUIDE_WALL = 0.0045

CARRIAGE_LENGTH = 0.18
CARRIAGE_WIDTH = 0.05
CARRIAGE_HEIGHT = GUIDE_HEIGHT - 2.0 * GUIDE_WALL

CARRIAGE_HOME_X = 0.34
CARRIAGE_TRAVEL = 0.09


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_body_runner")

    model.material("guide_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("carriage_alloy", rgba=(0.71, 0.74, 0.78, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        Box((GUIDE_LENGTH, GUIDE_WALL, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, -GUIDE_WIDTH / 2.0 + GUIDE_WALL / 2.0, 0.0)),
        material="guide_steel",
        name="back_wall",
    )
    guide_body.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_WALL)),
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, GUIDE_HEIGHT / 2.0 - GUIDE_WALL / 2.0)),
        material="guide_steel",
        name="top_lip",
    )
    guide_body.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_WALL)),
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, -GUIDE_HEIGHT / 2.0 + GUIDE_WALL / 2.0)),
        material="guide_steel",
        name="bottom_lip",
    )
    guide_body.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, 0.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        material="carriage_alloy",
        name="internal_shuttle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=0.38,
        origin=Origin(),
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=120.0,
            velocity=0.4,
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

    guide_body = object_model.get_part("guide_body")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guide_to_carriage")
    top_lip = guide_body.get_visual("top_lip")
    bottom_lip = guide_body.get_visual("bottom_lip")
    internal_shuttle = carriage.get_visual("internal_shuttle")

    ctx.check(
        "guide body exists",
        guide_body is not None,
        details="guide_body part should be present",
    )
    ctx.check(
        "carriage exists",
        carriage is not None,
        details="carriage part should be present",
    )

    ctx.expect_within(
        carriage,
        guide_body,
        axes="yz",
        inner_elem=internal_shuttle,
        margin=0.0,
        name="internal shuttle stays nested inside the guide section",
    )
    ctx.expect_gap(
        guide_body,
        carriage,
        axis="z",
        positive_elem=top_lip,
        negative_elem=internal_shuttle,
        min_gap=0.0,
        max_gap=0.0001,
        max_penetration=0.0,
        name="top lip supports the shuttle without penetration",
    )
    ctx.expect_gap(
        carriage,
        guide_body,
        axis="z",
        positive_elem=internal_shuttle,
        negative_elem=bottom_lip,
        min_gap=0.0,
        max_gap=0.0001,
        max_penetration=0.0,
        name="bottom lip supports the shuttle without penetration",
    )
    ctx.expect_overlap(
        carriage,
        guide_body,
        axes="x",
        elem_a=internal_shuttle,
        min_overlap=0.16,
        name="rest pose keeps substantial insertion inside the channel",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.expect_within(
            carriage,
            guide_body,
            axes="yz",
            inner_elem=internal_shuttle,
            margin=0.0,
            name="extended shuttle stays centered in the channel section",
        )
        ctx.expect_overlap(
            carriage,
            guide_body,
            axes="x",
            elem_a=internal_shuttle,
            min_overlap=0.075,
            name="extended carriage still remains inserted in the guide",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage extends along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
