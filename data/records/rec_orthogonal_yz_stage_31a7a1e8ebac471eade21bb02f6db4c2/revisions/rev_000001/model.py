from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.22
BASE_Y = 0.18
BASE_Z = 0.02

COLUMN_X = 0.06
COLUMN_Y = 0.10
COLUMN_Z = 0.68

GUIDE_RAIL_X = 0.012
GUIDE_RAIL_Y = 0.022
GUIDE_RAIL_Z = 0.54
GUIDE_RAIL_Y_OFFSET = 0.028
GUIDE_RAIL_BOTTOM_Z = BASE_Z + 0.08
GUIDE_RAIL_CENTER_Z = GUIDE_RAIL_BOTTOM_Z + GUIDE_RAIL_Z / 2.0
GUIDE_RAIL_CENTER_X = COLUMN_X / 2.0 + GUIDE_RAIL_X / 2.0

CARRIAGE_DEPTH = 0.052
CARRIAGE_WIDTH = 0.126
CARRIAGE_HEIGHT = 0.118
CARRIAGE_BEARING_X = 0.010
CARRIAGE_BEARING_Y = 0.026
CARRIAGE_BEARING_Z = 0.090
CARRIAGE_BEARING_Y_OFFSET = GUIDE_RAIL_Y_OFFSET
CARRIAGE_HOME_Z = 0.19
CARRIAGE_TRAVEL = 0.34

SIDE_RAIL_X = 0.018
SIDE_RAIL_Y = 0.140
SIDE_RAIL_Z = 0.014
SIDE_RAIL_CENTER_X = 0.026
SIDE_RAIL_CENTER_Y = 0.177
SIDE_RAIL_Z_OFFSET = 0.029

SLIDE_GUIDE_X = 0.014
SLIDE_GUIDE_Y = 0.085
SLIDE_GUIDE_Z = 0.018
SLIDE_TRAVEL = 0.055
SLIDE_HOME_X = SIDE_RAIL_CENTER_X + SIDE_RAIL_X / 2.0
SLIDE_HOME_Y = SIDE_RAIL_CENTER_Y - SIDE_RAIL_Y / 2.0


def _column_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_Z, centered=(True, True, False))
    mast = (
        cq.Workplane("XY")
        .box(COLUMN_X, COLUMN_Y, COLUMN_Z, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_Z))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.09, 0.06, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_Z + COLUMN_Z - 0.006))
    )
    left_gusset = (
        cq.Workplane("XZ")
        .moveTo(-COLUMN_X / 2.0, BASE_Z)
        .lineTo(-0.065, BASE_Z)
        .lineTo(-COLUMN_X / 2.0, BASE_Z + 0.13)
        .close()
        .extrude(0.012, both=True)
        .translate((0.0, COLUMN_Y / 2.0 - 0.006, 0.0))
    )
    right_gusset = left_gusset.mirror("YZ")

    return base.union(mast).union(top_cap).union(left_gusset).union(right_gusset)


def _carriage_body_shape() -> cq.Workplane:
    main_block = (
        cq.Workplane("XY")
        .box(0.040, 0.106, CARRIAGE_HEIGHT, centered=(False, True, True))
        .translate((0.030, 0.0, 0.0))
    )
    upper_rail_support = (
        cq.Workplane("XY")
        .box(0.024, 0.054, SIDE_RAIL_Z, centered=(False, True, True))
        .translate((0.020, 0.080, SIDE_RAIL_Z_OFFSET))
    )
    lower_rail_support = (
        cq.Workplane("XY")
        .box(0.024, 0.054, SIDE_RAIL_Z, centered=(False, True, True))
        .translate((0.020, 0.080, -SIDE_RAIL_Z_OFFSET))
    )
    front_pad = (
        cq.Workplane("XY")
        .box(0.014, 0.060, 0.074, centered=(False, True, True))
        .translate((0.043, 0.0, 0.0))
    )
    body = main_block.union(upper_rail_support).union(lower_rail_support).union(front_pad)
    body = body.cut(
        cq.Workplane("XY")
        .box(0.020, 0.056, 0.042, centered=(False, True, True))
        .translate((0.030, 0.0, 0.0))
    )
    return body


def _side_slide_body_shape() -> cq.Workplane:
    guide_bridge = (
        cq.Workplane("XY")
        .box(0.022, 0.110, 0.080, centered=(False, True, True))
        .translate((0.014, 0.085, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.026, 0.094, 0.046, centered=(False, True, True))
        .translate((0.020, 0.125, 0.0))
    )
    end_plate = (
        cq.Workplane("XY")
        .box(0.056, 0.008, 0.118, centered=(False, True, True))
        .translate((0.004, 0.173, 0.0))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.018, 0.052, 0.014, centered=(False, True, True))
        .translate((0.031, 0.120, 0.026))
    )
    bottom_cap = (
        cq.Workplane("XY")
        .box(0.018, 0.052, 0.014, centered=(False, True, True))
        .translate((0.031, 0.120, -0.026))
    )
    return guide_bridge.union(beam).union(end_plate).union(top_cap).union(bottom_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="column_carried_yz_slide")

    model.material("column_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_silver", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("guide_dark", rgba=(0.38, 0.40, 0.44, 1.0))
    model.material("slide_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))

    column = model.part("column")
    column.visual(
        mesh_from_cadquery(_column_body_shape(), "column_body"),
        material="column_gray",
        name="column_body",
    )
    for rail_name, y_pos in (
        ("guide_rail_left", -GUIDE_RAIL_Y_OFFSET),
        ("guide_rail_right", GUIDE_RAIL_Y_OFFSET),
    ):
        column.visual(
            Box((GUIDE_RAIL_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z)),
            origin=Origin(xyz=(GUIDE_RAIL_CENTER_X, y_pos, GUIDE_RAIL_CENTER_Z)),
            material="guide_dark",
            name=rail_name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="carriage_silver",
        name="carriage_body",
    )
    for pad_name, y_pos in (
        ("bearing_pad_left", -CARRIAGE_BEARING_Y_OFFSET),
        ("bearing_pad_right", CARRIAGE_BEARING_Y_OFFSET),
    ):
        carriage.visual(
            Box((CARRIAGE_BEARING_X, CARRIAGE_BEARING_Y, CARRIAGE_BEARING_Z)),
            origin=Origin(xyz=(CARRIAGE_BEARING_X / 2.0, y_pos, 0.0)),
            material="guide_dark",
            name=pad_name,
        )
        carriage.visual(
            Box((0.020, 0.014, CARRIAGE_BEARING_Z)),
            origin=Origin(xyz=(0.020, y_pos, 0.0)),
            material="guide_dark",
            name=f"{pad_name}_bridge",
        )
    for rail_name, z_pos in (
        ("side_rail_lower", -SIDE_RAIL_Z_OFFSET),
        ("side_rail_upper", SIDE_RAIL_Z_OFFSET),
    ):
        carriage.visual(
            Box((SIDE_RAIL_X, SIDE_RAIL_Y, SIDE_RAIL_Z)),
            origin=Origin(xyz=(SIDE_RAIL_CENTER_X, SIDE_RAIL_CENTER_Y, z_pos)),
            material="guide_dark",
            name=rail_name,
        )

    side_slide = model.part("side_slide")
    side_slide.visual(
        mesh_from_cadquery(_side_slide_body_shape(), "side_slide_body"),
        material="slide_aluminum",
        name="side_slide_body",
    )
    for block_name, z_pos in (
        ("guide_block_lower", -SIDE_RAIL_Z_OFFSET),
        ("guide_block_upper", SIDE_RAIL_Z_OFFSET),
    ):
        side_slide.visual(
            Box((SLIDE_GUIDE_X, SLIDE_GUIDE_Y, SLIDE_GUIDE_Z)),
            origin=Origin(
                xyz=(SLIDE_GUIDE_X / 2.0, SLIDE_GUIDE_Y / 2.0, z_pos),
            ),
            material="guide_dark",
            name=block_name,
        )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(GUIDE_RAIL_CENTER_X + GUIDE_RAIL_X / 2.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.25,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_side_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=side_slide,
        origin=Origin(xyz=(SLIDE_HOME_X, SLIDE_HOME_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=SLIDE_TRAVEL,
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

    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    side_slide = object_model.get_part("side_slide")
    lift = object_model.get_articulation("column_to_carriage")
    cross_slide = object_model.get_articulation("carriage_to_side_slide")
    guide_rail_left = column.get_visual("guide_rail_left")
    guide_rail_right = column.get_visual("guide_rail_right")
    bearing_pad_left = carriage.get_visual("bearing_pad_left")
    bearing_pad_right = carriage.get_visual("bearing_pad_right")
    side_rail_upper = carriage.get_visual("side_rail_upper")
    guide_block_upper = side_slide.get_visual("guide_block_upper")

    with ctx.pose({lift: 0.0, cross_slide: 0.0}):
        ctx.expect_gap(
            carriage,
            column,
            axis="x",
            positive_elem=bearing_pad_right,
            negative_elem=guide_rail_right,
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage bearing pad sits on column guide rail",
        )
        ctx.expect_overlap(
            carriage,
            column,
            axes="yz",
            elem_a=bearing_pad_right,
            elem_b=guide_rail_right,
            min_overlap=0.018,
            name="carriage bearing pad stays aligned with guide rail at rest",
        )
        ctx.expect_gap(
            side_slide,
            carriage,
            axis="x",
            positive_elem=guide_block_upper,
            negative_elem=side_rail_upper,
            max_gap=0.001,
            max_penetration=0.0,
            name="side slide guide block sits on carriage rail",
        )
        ctx.expect_overlap(
            side_slide,
            carriage,
            axes="y",
            elem_a=guide_block_upper,
            elem_b=side_rail_upper,
            min_overlap=0.080,
            name="retracted side slide keeps long rail engagement",
        )

    carriage_rest = ctx.part_world_position(carriage)
    side_slide_rest = ctx.part_world_position(side_slide)

    with ctx.pose({lift: CARRIAGE_TRAVEL, cross_slide: 0.0}):
        ctx.expect_gap(
            carriage,
            column,
            axis="x",
            positive_elem=bearing_pad_left,
            negative_elem=guide_rail_left,
            max_gap=0.001,
            max_penetration=0.0,
            name="raised carriage still bears against left guide rail",
        )
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            elem_a=bearing_pad_left,
            elem_b=guide_rail_left,
            min_overlap=0.085,
            name="raised carriage keeps retained vertical guide engagement",
        )
        carriage_high = ctx.part_world_position(carriage)

    with ctx.pose({lift: 0.0, cross_slide: SLIDE_TRAVEL}):
        ctx.expect_overlap(
            side_slide,
            carriage,
            axes="y",
            elem_a=guide_block_upper,
            elem_b=side_rail_upper,
            min_overlap=0.060,
            name="extended side slide still retains guide engagement",
        )
        side_slide_out = ctx.part_world_position(side_slide)

    ctx.check(
        "carriage extends upward along +Z",
        carriage_rest is not None
        and carriage_high is not None
        and carriage_high[2] > carriage_rest[2] + 0.25,
        details=f"rest={carriage_rest}, raised={carriage_high}",
    )
    ctx.check(
        "side slide extends outward along +Y",
        side_slide_rest is not None
        and side_slide_out is not None
        and side_slide_out[1] > side_slide_rest[1] + 0.045,
        details=f"rest={side_slide_rest}, extended={side_slide_out}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
