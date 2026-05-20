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


FRAME_WIDTH = 0.30
FOOT_HEIGHT = 0.040
REAR_FOOT_LENGTH = 0.100
BRIDGE_THICKNESS = 0.085
RAIL_CENTER_Y = 0.095
RAIL_WIDTH = 0.055
RAIL_HEIGHT = 0.072
GUIDE_START_X = -0.215
GUIDE_END_X = 0.275
GUIDE_LENGTH = GUIDE_END_X - GUIDE_START_X
GUIDE_CENTER_X = (GUIDE_START_X + GUIDE_END_X) / 2.0
GUIDE_TOP_Z = FOOT_HEIGHT + RAIL_HEIGHT
FRONT_FOOT_LENGTH = 0.080
FRONT_FOOT_CENTER_X = GUIDE_END_X - (FRONT_FOOT_LENGTH / 2.0)
BRIDGE_CENTER_X = GUIDE_START_X - (BRIDGE_THICKNESS / 2.0)
GUIDE_PROXY_WIDTH = 0.048
GUIDE_PROXY_THICKNESS = 0.006
BRIDGE_POST_WIDTH = 0.060
BRIDGE_POST_HEIGHT = 0.120
BRIDGE_TOP_BEAM_HEIGHT = 0.040
BRIDGE_POST_CENTER_Y = (FRAME_WIDTH / 2.0) - (BRIDGE_POST_WIDTH / 2.0)

CARRIAGE_DECK_LENGTH = 0.420
CARRIAGE_DECK_WIDTH = 0.215
CARRIAGE_DECK_THICKNESS = 0.020
CARRIAGE_SHOE_LENGTH = 0.265
CARRIAGE_SHOE_WIDTH = 0.044
CARRIAGE_SHOE_HEIGHT = 0.028
CARRIAGE_SADDLE_LENGTH = 0.230
CARRIAGE_SADDLE_WIDTH = 0.116
CARRIAGE_RISER_LENGTH = 0.200
CARRIAGE_RISER_WIDTH = 0.036
CARRIAGE_RISER_CENTER_Y = 0.072
CARRIAGE_RISER_HEIGHT = 0.026
CARRIAGE_DECK_BOTTOM_Z = CARRIAGE_SHOE_HEIGHT + CARRIAGE_RISER_HEIGHT
CARRIAGE_TOP_Z = CARRIAGE_DECK_BOTTOM_Z + CARRIAGE_DECK_THICKNESS

SLIDE_LOWER = -0.060
SLIDE_UPPER = 0.160


def _aabb_length(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis] - aabb[0][axis]


def _overhang_metrics(
    ctx: TestContext,
    carriage,
    frame,
) -> tuple[float, float, float] | None:
    deck_aabb = ctx.part_element_world_aabb(carriage, elem="deck_proxy")
    support_aabb = ctx.part_element_world_aabb(frame, elem="guide_span")
    if deck_aabb is None or support_aabb is None:
        return None

    deck_length = _aabb_length(deck_aabb, 0)
    if deck_length is None:
        return None

    rear_overhang = max(0.0, support_aabb[0][0] - deck_aabb[0][0])
    front_overhang = max(0.0, deck_aabb[1][0] - support_aabb[1][0])
    return (deck_length, rear_overhang, front_overhang)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_transfer_slide")

    model.material("frame_blue", rgba=(0.24, 0.33, 0.46, 1.0))
    model.material("carriage_gray", rgba=(0.71, 0.74, 0.78, 1.0))
    model.material("guide_steel", rgba=(0.58, 0.61, 0.65, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((REAR_FOOT_LENGTH, FRAME_WIDTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(BRIDGE_CENTER_X, 0.0, FOOT_HEIGHT / 2.0)),
        material="frame_blue",
        name="rear_foot",
    )
    rear_frame.visual(
        Box((BRIDGE_THICKNESS, BRIDGE_POST_WIDTH, BRIDGE_POST_HEIGHT)),
        origin=Origin(
            xyz=(
                BRIDGE_CENTER_X,
                BRIDGE_POST_CENTER_Y,
                FOOT_HEIGHT + (BRIDGE_POST_HEIGHT / 2.0),
            )
        ),
        material="frame_blue",
        name="left_post",
    )
    rear_frame.visual(
        Box((BRIDGE_THICKNESS, BRIDGE_POST_WIDTH, BRIDGE_POST_HEIGHT)),
        origin=Origin(
            xyz=(
                BRIDGE_CENTER_X,
                -BRIDGE_POST_CENTER_Y,
                FOOT_HEIGHT + (BRIDGE_POST_HEIGHT / 2.0),
            )
        ),
        material="frame_blue",
        name="right_post",
    )
    rear_frame.visual(
        Box((BRIDGE_THICKNESS, FRAME_WIDTH, BRIDGE_TOP_BEAM_HEIGHT)),
        origin=Origin(
            xyz=(
                BRIDGE_CENTER_X,
                0.0,
                FOOT_HEIGHT + BRIDGE_POST_HEIGHT + (BRIDGE_TOP_BEAM_HEIGHT / 2.0),
            )
        ),
        material="frame_blue",
        name="top_beam",
    )
    rear_frame.visual(
        Box((GUIDE_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, RAIL_CENTER_Y, FOOT_HEIGHT + (RAIL_HEIGHT / 2.0))),
        material="frame_blue",
        name="left_rail",
    )
    rear_frame.visual(
        Box((GUIDE_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, -RAIL_CENTER_Y, FOOT_HEIGHT + (RAIL_HEIGHT / 2.0))),
        material="frame_blue",
        name="right_rail",
    )
    rear_frame.visual(
        Box((FRONT_FOOT_LENGTH, RAIL_WIDTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(FRONT_FOOT_CENTER_X, RAIL_CENTER_Y, FOOT_HEIGHT / 2.0)),
        material="frame_blue",
        name="left_front_foot",
    )
    rear_frame.visual(
        Box((FRONT_FOOT_LENGTH, RAIL_WIDTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(FRONT_FOOT_CENTER_X, -RAIL_CENTER_Y, FOOT_HEIGHT / 2.0)),
        material="frame_blue",
        name="right_front_foot",
    )
    rear_frame.visual(
        Box((GUIDE_LENGTH, GUIDE_PROXY_WIDTH, GUIDE_PROXY_THICKNESS)),
        origin=Origin(
            xyz=(
                GUIDE_CENTER_X,
                RAIL_CENTER_Y,
                GUIDE_TOP_Z - (GUIDE_PROXY_THICKNESS / 2.0),
            )
        ),
        material="guide_steel",
        name="left_guide",
    )
    rear_frame.visual(
        Box((GUIDE_LENGTH, GUIDE_PROXY_WIDTH, GUIDE_PROXY_THICKNESS)),
        origin=Origin(
            xyz=(
                GUIDE_CENTER_X,
                -RAIL_CENTER_Y,
                GUIDE_TOP_Z - (GUIDE_PROXY_THICKNESS / 2.0),
            )
        ),
        material="guide_steel",
        name="right_guide",
    )
    rear_frame.visual(
        Box((GUIDE_LENGTH, CARRIAGE_DECK_WIDTH, GUIDE_PROXY_THICKNESS)),
        origin=Origin(
            xyz=(GUIDE_CENTER_X, 0.0, GUIDE_TOP_Z - (GUIDE_PROXY_THICKNESS / 2.0))
        ),
        material="guide_steel",
        name="guide_span",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.620, FRAME_WIDTH, 0.200)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    front_carriage = model.part("front_carriage")
    front_carriage.visual(
        Box((CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_WIDTH, CARRIAGE_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, CARRIAGE_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="left_shoe",
    )
    front_carriage.visual(
        Box((CARRIAGE_SHOE_LENGTH, CARRIAGE_SHOE_WIDTH, CARRIAGE_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, -RAIL_CENTER_Y, CARRIAGE_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="right_shoe",
    )
    front_carriage.visual(
        Box((CARRIAGE_DECK_LENGTH, CARRIAGE_DECK_WIDTH, CARRIAGE_DECK_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_DECK_BOTTOM_Z + (CARRIAGE_DECK_THICKNESS / 2.0))
        ),
        material="carriage_gray",
        name="deck_proxy",
    )
    front_carriage.visual(
        Box((CARRIAGE_SADDLE_LENGTH, CARRIAGE_SADDLE_WIDTH, CARRIAGE_SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_SHOE_HEIGHT / 2.0)),
        material="carriage_gray",
        name="saddle",
    )
    front_carriage.visual(
        Box((CARRIAGE_RISER_LENGTH, CARRIAGE_RISER_WIDTH, CARRIAGE_RISER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CARRIAGE_RISER_CENTER_Y,
                CARRIAGE_SHOE_HEIGHT + (CARRIAGE_RISER_HEIGHT / 2.0),
            )
        ),
        material="carriage_gray",
        name="left_riser",
    )
    front_carriage.visual(
        Box((CARRIAGE_RISER_LENGTH, CARRIAGE_RISER_WIDTH, CARRIAGE_RISER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CARRIAGE_RISER_CENTER_Y,
                CARRIAGE_SHOE_HEIGHT + (CARRIAGE_RISER_HEIGHT / 2.0),
            )
        ),
        material="carriage_gray",
        name="right_riser",
    )
    front_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_DECK_LENGTH, CARRIAGE_DECK_WIDTH, CARRIAGE_TOP_Z)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_Z / 2.0)),
    )

    model.articulation(
        "rear_frame_to_front_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=front_carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=900.0,
            velocity=0.35,
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
    rear_frame = object_model.get_part("rear_frame")
    front_carriage = object_model.get_part("front_carriage")
    slide = object_model.get_articulation("rear_frame_to_front_carriage")

    ctx.expect_gap(
        front_carriage,
        rear_frame,
        axis="z",
        positive_elem="left_shoe",
        negative_elem="left_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="left shoe bears on the left guide",
    )
    ctx.expect_gap(
        front_carriage,
        rear_frame,
        axis="z",
        positive_elem="right_shoe",
        negative_elem="right_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="right shoe bears on the right guide",
    )
    ctx.expect_within(
        front_carriage,
        rear_frame,
        axes="y",
        inner_elem="left_shoe",
        outer_elem="left_guide",
        margin=0.0,
        name="left shoe stays laterally on the guide path",
    )
    ctx.expect_within(
        front_carriage,
        rear_frame,
        axes="y",
        inner_elem="right_shoe",
        outer_elem="right_guide",
        margin=0.0,
        name="right shoe stays laterally on the guide path",
    )

    lower = slide.motion_limits.lower if slide.motion_limits is not None else SLIDE_LOWER
    upper = slide.motion_limits.upper if slide.motion_limits is not None else SLIDE_UPPER

    rest_pos = ctx.part_world_position(front_carriage)
    with ctx.pose({slide: lower}):
        ctx.expect_overlap(
            front_carriage,
            rear_frame,
            axes="x",
            elem_a="left_shoe",
            elem_b="left_guide",
            min_overlap=0.240,
            name="rearward pose keeps the carriage retained on the guide",
        )
        lower_metrics = _overhang_metrics(ctx, front_carriage, rear_frame)
        ctx.check(
            "rearward pose keeps the carriage longer than its rear overhang",
            lower_metrics is not None and lower_metrics[1] > 0.0 and lower_metrics[0] > (3.0 * lower_metrics[1]),
            details=f"metrics={lower_metrics}",
        )

    with ctx.pose({slide: upper}):
        ctx.expect_overlap(
            front_carriage,
            rear_frame,
            axes="x",
            elem_a="left_shoe",
            elem_b="left_guide",
            min_overlap=0.240,
            name="forward pose keeps the carriage retained on the guide",
        )
        upper_metrics = _overhang_metrics(ctx, front_carriage, rear_frame)
        ctx.check(
            "forward pose keeps the carriage longer than its front overhang",
            upper_metrics is not None and upper_metrics[2] > 0.0 and upper_metrics[0] > (3.0 * upper_metrics[2]),
            details=f"metrics={upper_metrics}",
        )
        extended_pos = ctx.part_world_position(front_carriage)

    ctx.check(
        "front carriage advances along +X",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
