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


BASE_LENGTH = 0.46
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.016
FOOT_LENGTH = 0.085
FOOT_WIDTH = 0.048
FOOT_THICKNESS = 0.010
FOOT_X_OFFSET = 0.145
FOOT_Y_OFFSET = 0.046

RAIL_LENGTH = 0.36
RAIL_WIDTH = 0.042
RAIL_HEIGHT = 0.028
RAIL_CENTER_Z = 0.029

CARRIAGE_LENGTH = 0.135
CARRIAGE_WIDTH = 0.108
CARRIAGE_TOP_THICKNESS = 0.016
CARRIAGE_CHEEK_WIDTH = 0.028
CARRIAGE_CHEEK_HEIGHT = 0.036
CARRIAGE_CHEEK_Z = -0.017
CARRIAGE_CHEEK_Y = 0.040
CARRIAGE_FRAME_Z = 0.051
X_TRAVEL = 0.10

MAST_THICKNESS = 0.032
MAST_WIDTH = 0.090
MAST_HEIGHT = 0.210
MAST_CENTER_Z = 0.105

HEAD_HEIGHT = 0.088
HEAD_FRONT_THICKNESS = 0.020
HEAD_FRONT_WIDTH = 0.112
HEAD_FRONT_X = 0.028
HEAD_CHEEK_LENGTH = 0.048
HEAD_CHEEK_WIDTH = 0.010
HEAD_CHEEK_X = 0.014
HEAD_CHEEK_Y = 0.050
HEAD_ORIGIN_Z = 0.054
Z_TRAVEL = 0.11

TOOL_BLOCK_X = 0.050
TOOL_BLOCK_SIZE = (0.024, 0.056, 0.028)
TOOL_BLOCK_Z = -0.018
TOOL_NOSE_RADIUS = 0.008
TOOL_NOSE_LENGTH = 0.022
TOOL_NOSE_X = 0.072
TOOL_NOSE_Z = -0.022


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_xz_transfer_stage")

    model.material("base_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("machined_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("rail_steel", rgba=(0.57, 0.60, 0.64, 1.0))
    model.material("tool_dark", rgba=(0.26, 0.28, 0.31, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_black",
        name="base_plate",
    )
    base.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_CENTER_Z)),
        material="rail_steel",
        name="rail_beam",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            base.visual(
                Box((FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS)),
                origin=Origin(
                    xyz=(
                        sx * FOOT_X_OFFSET,
                        sy * FOOT_Y_OFFSET,
                        FOOT_THICKNESS / 2.0 - 0.001,
                    )
                ),
                material="base_black",
                name=f"foot_{'r' if sx > 0.0 else 'l'}_{'f' if sy > 0.0 else 'b'}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="machined_gray",
        name="carriage_plate",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_CHEEK_WIDTH, CARRIAGE_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, CARRIAGE_CHEEK_Y, CARRIAGE_CHEEK_Z)),
        material="machined_gray",
        name="carriage_left_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_CHEEK_WIDTH, CARRIAGE_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -CARRIAGE_CHEEK_Y, CARRIAGE_CHEEK_Z)),
        material="machined_gray",
        name="carriage_right_cheek",
    )
    carriage.visual(
        Box((MAST_THICKNESS, MAST_WIDTH, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_CENTER_Z)),
        material="machined_gray",
        name="mast",
    )

    head = model.part("head")
    head.visual(
        Box((HEAD_FRONT_THICKNESS, HEAD_FRONT_WIDTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(HEAD_FRONT_X, 0.0, 0.0)),
        material="machined_gray",
        name="head_front_plate",
    )
    head.visual(
        Box((HEAD_CHEEK_LENGTH, HEAD_CHEEK_WIDTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(HEAD_CHEEK_X, HEAD_CHEEK_Y, 0.0)),
        material="machined_gray",
        name="head_left_cheek",
    )
    head.visual(
        Box((HEAD_CHEEK_LENGTH, HEAD_CHEEK_WIDTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(HEAD_CHEEK_X, -HEAD_CHEEK_Y, 0.0)),
        material="machined_gray",
        name="head_right_cheek",
    )
    head.visual(
        Box(TOOL_BLOCK_SIZE),
        origin=Origin(xyz=(TOOL_BLOCK_X, 0.0, TOOL_BLOCK_Z)),
        material="tool_dark",
        name="tool_block",
    )
    head.visual(
        Cylinder(radius=TOOL_NOSE_RADIUS, length=TOOL_NOSE_LENGTH),
        origin=Origin(xyz=(TOOL_NOSE_X, 0.0, TOOL_NOSE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="rail_steel",
        name="tool_nose",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=180.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=120.0,
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
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")
    x_slide = object_model.get_articulation("base_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_head")

    ctx.check("base exists", base is not None)
    ctx.check("carriage exists", carriage is not None)
    ctx.check("head exists", head is not None)

    ctx.expect_contact(
        carriage,
        base,
        elem_a="carriage_plate",
        elem_b="rail_beam",
        name="carriage plate rides on the fixed rail beam",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="carriage_plate",
        elem_b="rail_beam",
        min_overlap=0.13,
        name="carriage remains engaged on the rail at center",
    )
    ctx.expect_contact(
        head,
        carriage,
        elem_a="head_left_cheek",
        elem_b="mast",
        name="left guide cheek bears on the mast",
    )
    ctx.expect_contact(
        head,
        carriage,
        elem_a="head_right_cheek",
        elem_b="mast",
        name="right guide cheek bears on the mast",
    )
    ctx.expect_gap(
        head,
        carriage,
        axis="x",
        positive_elem="head_front_plate",
        negative_elem="mast",
        min_gap=0.0015,
        max_gap=0.0035,
        name="head front plate clears the mast face",
    )
    ctx.expect_overlap(
        head,
        carriage,
        axes="yz",
        elem_a="head_front_plate",
        elem_b="mast",
        min_overlap=0.085,
        name="head stays aligned with the mast at low position",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({x_slide: X_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="carriage_plate",
            elem_b="rail_beam",
            min_overlap=0.07,
            name="carriage retains insertion at positive X travel",
        )
        carriage_high_x = ctx.part_world_position(carriage)
    with ctx.pose({x_slide: -X_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="carriage_plate",
            elem_b="rail_beam",
            min_overlap=0.07,
            name="carriage retains insertion at negative X travel",
        )
        carriage_low_x = ctx.part_world_position(carriage)

    ctx.check(
        "carriage moves along +X for positive travel",
        carriage_rest is not None
        and carriage_high_x is not None
        and carriage_high_x[0] > carriage_rest[0] + 0.05,
        details=f"rest={carriage_rest}, plus={carriage_high_x}",
    )
    ctx.check(
        "carriage moves along -X for negative travel",
        carriage_rest is not None
        and carriage_low_x is not None
        and carriage_low_x[0] < carriage_rest[0] - 0.05,
        details=f"rest={carriage_rest}, minus={carriage_low_x}",
    )

    head_rest = ctx.part_world_position(head)
    with ctx.pose({z_slide: Z_TRAVEL}):
        ctx.expect_contact(
            head,
            carriage,
            elem_a="head_left_cheek",
            elem_b="mast",
            name="left guide cheek stays engaged at full Z travel",
        )
        ctx.expect_contact(
            head,
            carriage,
            elem_a="head_right_cheek",
            elem_b="mast",
            name="right guide cheek stays engaged at full Z travel",
        )
        ctx.expect_overlap(
            head,
            carriage,
            axes="yz",
            elem_a="head_front_plate",
            elem_b="mast",
            min_overlap=0.085,
            name="head remains guided by the mast at full Z travel",
        )
        head_high = ctx.part_world_position(head)

    ctx.check(
        "head rises along +Z for positive travel",
        head_rest is not None and head_high is not None and head_high[2] > head_rest[2] + 0.08,
        details=f"rest={head_rest}, high={head_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
