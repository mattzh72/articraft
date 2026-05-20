from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_LENGTH = 1.20
RAIL_BASE_WIDTH = 0.16
RAIL_BASE_THICK = 0.024
RAIL_BEAM_LENGTH = 1.08
RAIL_BEAM_WIDTH = 0.10
RAIL_BEAM_HEIGHT = 0.094
RAIL_CAP_WIDTH = 0.14
RAIL_CAP_HEIGHT = 0.016
RAIL_TOP_Z = RAIL_BASE_THICK + RAIL_BEAM_HEIGHT + RAIL_CAP_HEIGHT

CARRIAGE_LENGTH = 0.22
CARRIAGE_WIDTH = 0.22
CARRIAGE_DECK_THICK = 0.028
CARRIAGE_INNER_GAP = 0.148
CARRIAGE_JAW_DEPTH = 0.030
CARRIAGE_FRAME_Z = RAIL_TOP_Z
CARRIAGE_TRAVEL = 0.36
DECK_TO_RAIL_CLEARANCE = 0.0

BEARING_BLOCK_X = 0.058
BEARING_BLOCK_Y = 0.034
BEARING_BLOCK_Z = 0.078
BEARING_BLOCK_CENTER_Y = (CARRIAGE_WIDTH / 2.0) - (BEARING_BLOCK_Y / 2.0)
BEARING_BLOCK_OUTER_FACE_Y = BEARING_BLOCK_CENTER_Y + (BEARING_BLOCK_Y / 2.0)
SUPPORT_PAD_X = 0.096
SUPPORT_PAD_Y = 0.050
SUPPORT_PAD_Z = 0.012
SUPPORT_PAD_CENTER_Y = BEARING_BLOCK_CENTER_Y - 0.008
SPINDLE_CENTER_Z = CARRIAGE_DECK_THICK + 0.038

SHAFT_RADIUS = 0.008
SHAFT_LENGTH = 0.076
SHAFT_CENTER_Y = SHAFT_LENGTH / 2.0
INNER_COLLAR_RADIUS = 0.013
INNER_COLLAR_LENGTH = 0.010
INNER_COLLAR_CENTER_Y = INNER_COLLAR_LENGTH / 2.0
ROLLER_RADIUS = 0.032
ROLLER_LENGTH = 0.036
ROLLER_CENTER_Y = 0.043
OUTER_WASHER_RADIUS = 0.013
OUTER_WASHER_LENGTH = 0.010
OUTER_WASHER_CENTER_Y = 0.069


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="crossbeam_carriage_roll_head")

    model.material("rail_paint", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("carriage_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("tool_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("roller_rubber", rgba=(0.16, 0.17, 0.18, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_THICK / 2.0)),
        material="rail_paint",
        name="rail_base",
    )
    rail.visual(
        Box((RAIL_BEAM_LENGTH, RAIL_BEAM_WIDTH, RAIL_BEAM_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, RAIL_BASE_THICK + (RAIL_BEAM_HEIGHT / 2.0))
        ),
        material="rail_paint",
        name="rail_web",
    )
    rail.visual(
        Box((RAIL_BEAM_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, RAIL_BASE_THICK + RAIL_BEAM_HEIGHT + (RAIL_CAP_HEIGHT / 2.0))
        ),
        material="rail_paint",
        name="rail_cap",
    )

    carriage = model.part("carriage")
    jaw_thickness = (CARRIAGE_WIDTH - CARRIAGE_INNER_GAP) / 2.0
    jaw_center_y = (CARRIAGE_INNER_GAP / 2.0) + (jaw_thickness / 2.0)
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_DECK_THICK)),
        origin=Origin(
            xyz=(0.0, 0.0, DECK_TO_RAIL_CLEARANCE + (CARRIAGE_DECK_THICK / 2.0))
        ),
        material="carriage_aluminum",
        name="carriage_deck",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, jaw_thickness, CARRIAGE_JAW_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                jaw_center_y,
                DECK_TO_RAIL_CLEARANCE - (CARRIAGE_JAW_DEPTH / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="left_guide",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, jaw_thickness, CARRIAGE_JAW_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                -jaw_center_y,
                DECK_TO_RAIL_CLEARANCE - (CARRIAGE_JAW_DEPTH / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="right_guide",
    )
    carriage.visual(
        Box((SUPPORT_PAD_X, SUPPORT_PAD_Y, SUPPORT_PAD_Z)),
        origin=Origin(
            xyz=(
                0.0,
                SUPPORT_PAD_CENTER_Y,
                DECK_TO_RAIL_CLEARANCE + CARRIAGE_DECK_THICK + (SUPPORT_PAD_Z / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="support_pad",
    )
    carriage.visual(
        Box((BEARING_BLOCK_X, BEARING_BLOCK_Y, BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(
                0.0,
                BEARING_BLOCK_CENTER_Y,
                DECK_TO_RAIL_CLEARANCE
                + CARRIAGE_DECK_THICK
                + SUPPORT_PAD_Z
                + (BEARING_BLOCK_Z / 2.0),
            )
        ),
        material="carriage_aluminum",
        name="bearing_block",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, SHAFT_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=INNER_COLLAR_RADIUS, length=INNER_COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, INNER_COLLAR_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
        name="inner_collar",
    )
    spindle.visual(
        Cylinder(radius=ROLLER_RADIUS, length=ROLLER_LENGTH),
        origin=Origin(
            xyz=(0.0, ROLLER_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="roller_rubber",
        name="roll_sleeve",
    )
    spindle.visual(
        Cylinder(radius=OUTER_WASHER_RADIUS, length=OUTER_WASHER_LENGTH),
        origin=Origin(
            xyz=(0.0, OUTER_WASHER_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
        name="outer_washer",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.35,
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, BEARING_BLOCK_OUTER_FACE_Y, SPINDLE_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=10.0,
            lower=-(2.0 * pi),
            upper=(2.0 * pi),
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

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    spin = object_model.get_articulation("carriage_to_spindle")

    ctx.check(
        "slide axis follows beam",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "spindle axis follows shaft",
        tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_origin_gap(
        carriage,
        rail,
        axis="z",
        min_gap=CARRIAGE_FRAME_Z - 0.001,
        max_gap=CARRIAGE_FRAME_Z + 0.001,
        name="carriage rides above the rail crown",
    )
    ctx.expect_origin_gap(
        spindle,
        carriage,
        axis="z",
        min_gap=SPINDLE_CENTER_Z - 0.001,
        max_gap=SPINDLE_CENTER_Z + 0.001,
        name="spindle sits above the carriage deck",
    )
    ctx.expect_origin_gap(
        spindle,
        carriage,
        axis="y",
        min_gap=BEARING_BLOCK_OUTER_FACE_Y - 0.001,
        max_gap=BEARING_BLOCK_OUTER_FACE_Y + 0.001,
        name="spindle is carried by the side bearing block",
    )
    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        positive_elem="carriage_deck",
        negative_elem="rail_cap",
        min_gap=DECK_TO_RAIL_CLEARANCE - 0.001,
        max_gap=DECK_TO_RAIL_CLEARANCE + 0.001,
        name="carriage deck rides just above the rail cap",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a="inner_collar",
        elem_b="bearing_block",
        name="spindle collar seats against the bearing block face",
    )

    with ctx.pose({slide: slide.motion_limits.lower}):
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            positive_elem="carriage_deck",
            negative_elem="rail_cap",
            min_gap=DECK_TO_RAIL_CLEARANCE - 0.001,
            max_gap=DECK_TO_RAIL_CLEARANCE + 0.001,
            name="carriage keeps its running clearance at lower travel",
        )
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            positive_elem="carriage_deck",
            negative_elem="rail_cap",
            min_gap=DECK_TO_RAIL_CLEARANCE - 0.001,
            max_gap=DECK_TO_RAIL_CLEARANCE + 0.001,
            name="carriage keeps its running clearance at upper travel",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        moved_pos = ctx.part_world_position(carriage)
    carriage_moves_cleanly = (
        rest_pos is not None
        and moved_pos is not None
        and moved_pos[0] > rest_pos[0] + 0.30
        and abs(moved_pos[1] - rest_pos[1]) < 1e-6
        and abs(moved_pos[2] - rest_pos[2]) < 1e-6
    )
    ctx.check(
        "carriage translates along the beam",
        carriage_moves_cleanly,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
