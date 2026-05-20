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


BODY_LENGTH = 0.34
BODY_WIDTH = 0.10
BASE_THICKNESS = 0.010
RAIL_LENGTH = 0.26
RAIL_WIDTH = 0.036
RAIL_HEIGHT = 0.018
WAY_LENGTH = 0.24
WAY_WIDTH = 0.014
WAY_HEIGHT = 0.002
WAY_Y = 0.028
STOP_WIDTH_X = 0.010
STOP_WIDTH_Y = 0.028
STOP_HEIGHT = 0.015
STOP_CENTER_Z = BASE_THICKNESS + (STOP_HEIGHT / 2.0) - 0.0005
STOP_CENTER_X = 0.150

CARRIAGE_LENGTH = 0.12
CARRIAGE_WIDTH = 0.082
TOP_PLATE_THICKNESS = 0.008
SKIRT_THICKNESS = 0.012
SKIRT_HEIGHT = 0.017
CARRIAGE_BOTTOM_Z = 0.012
TRAVEL_LIMIT = 0.085


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_carriage_slide")

    model.material("body_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("carriage_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("stop_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="body_gray",
        name="base_plate",
    )
    body.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (RAIL_HEIGHT / 2.0) - 0.0005)),
        material="body_gray",
        name="center_rail",
    )
    body.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(0.0, WAY_Y, BASE_THICKNESS + (WAY_HEIGHT / 2.0))),
        material="body_gray",
        name="left_way",
    )
    body.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(0.0, -WAY_Y, BASE_THICKNESS + (WAY_HEIGHT / 2.0))),
        material="body_gray",
        name="right_way",
    )
    body.visual(
        Box((STOP_WIDTH_X, STOP_WIDTH_Y, STOP_HEIGHT)),
        origin=Origin(xyz=(-STOP_CENTER_X, 0.0, STOP_CENTER_Z)),
        material="stop_black",
        name="left_stop",
    )
    body.visual(
        Box((STOP_WIDTH_X, STOP_WIDTH_Y, STOP_HEIGHT)),
        origin=Origin(xyz=(STOP_CENTER_X, 0.0, STOP_CENTER_Z)),
        material="stop_black",
        name="right_stop",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_THICKNESS + RAIL_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + RAIL_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, TOP_PLATE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, SKIRT_HEIGHT + (TOP_PLATE_THICKNESS / 2.0) - 0.00025)
        ),
        material="carriage_gray",
        name="top_plate",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, SKIRT_THICKNESS, SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CARRIAGE_WIDTH / 2.0) - (SKIRT_THICKNESS / 2.0),
                SKIRT_HEIGHT / 2.0,
            )
        ),
        material="carriage_gray",
        name="left_skirt",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, SKIRT_THICKNESS, SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CARRIAGE_WIDTH / 2.0) + (SKIRT_THICKNESS / 2.0),
                SKIRT_HEIGHT / 2.0,
            )
        ),
        material="carriage_gray",
        name="right_skirt",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, SKIRT_HEIGHT + TOP_PLATE_THICKNESS)),
        mass=1.1,
        origin=Origin(
            xyz=(0.0, 0.0, (SKIRT_HEIGHT + TOP_PLATE_THICKNESS) / 2.0)
        ),
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_LIMIT,
            upper=TRAVEL_LIMIT,
            effort=200.0,
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
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("body_to_carriage")
    limits = slide.motion_limits

    ctx.check("body exists", body is not None)
    ctx.check("carriage exists", carriage is not None)
    ctx.check(
        "slide axis is +x",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "slide has symmetric travel limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + TRAVEL_LIMIT) < 1e-9
        and abs(limits.upper - TRAVEL_LIMIT) < 1e-9,
        details=f"limits={limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            carriage,
            body,
            axes="y",
            margin=0.0,
            name="carriage stays laterally within body envelope at center",
        )
        ctx.expect_overlap(
            body,
            carriage,
            axes="x",
            min_overlap=0.11,
            name="body fully supports carriage at center",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: TRAVEL_LIMIT}):
        ctx.expect_gap(
            body,
            carriage,
            axis="x",
            positive_elem="right_stop",
            max_gap=0.001,
            max_penetration=1e-6,
            name="right stop limits positive travel",
        )
        ctx.expect_within(
            carriage,
            body,
            axes="y",
            margin=0.0,
            name="carriage stays laterally aligned at positive travel",
        )
        ctx.expect_overlap(
            body,
            carriage,
            axes="x",
            min_overlap=0.11,
            name="carriage remains supported at positive travel",
        )
        upper_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: -TRAVEL_LIMIT}):
        ctx.expect_gap(
            carriage,
            body,
            axis="x",
            negative_elem="left_stop",
            max_gap=0.001,
            max_penetration=1e-6,
            name="left stop limits negative travel",
        )
        ctx.expect_within(
            carriage,
            body,
            axes="y",
            margin=0.0,
            name="carriage stays laterally aligned at negative travel",
        )
        ctx.expect_overlap(
            body,
            carriage,
            axes="x",
            min_overlap=0.11,
            name="carriage remains supported at negative travel",
        )
        lower_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive prismatic motion moves carriage toward +x",
        rest_pos is not None
        and upper_pos is not None
        and upper_pos[0] > rest_pos[0] + 0.07,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )
    ctx.check(
        "negative prismatic motion moves carriage toward -x",
        rest_pos is not None
        and lower_pos is not None
        and lower_pos[0] < rest_pos[0] - 0.07,
        details=f"rest={rest_pos}, lower={lower_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
