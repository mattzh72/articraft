from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LOWER_LENGTH = 0.190
LOWER_WIDTH = 0.110
LOWER_BASE_THICKNESS = 0.012
LOWER_RAIL_LENGTH = 0.170
LOWER_RAIL_WIDTH = 0.020
LOWER_RAIL_HEIGHT = 0.010
LOWER_RAIL_CENTER_Y = 0.028

CARRIAGE_LENGTH = 0.120
CARRIAGE_WIDTH = 0.108
CARRIAGE_BODY_HEIGHT = 0.018
CARRIAGE_UNDER_POCKET_LENGTH = 0.102
CARRIAGE_UNDER_POCKET_WIDTH = 0.034
CARRIAGE_UNDER_POCKET_DEPTH = 0.008
CARRIAGE_TOP_RAIL_WIDTH_X = 0.016
CARRIAGE_TOP_RAIL_LENGTH_Y = 0.090
CARRIAGE_TOP_RAIL_HEIGHT = 0.008
CARRIAGE_TOP_RAIL_CENTER_X = 0.020

UPPER_SLIDE_WIDTH_X = 0.082
UPPER_SLIDE_LENGTH_Y = 0.100
UPPER_SLIDE_BODY_HEIGHT = 0.016
UPPER_SLIDE_UNDER_POCKET_WIDTH_X = 0.024
UPPER_SLIDE_UNDER_POCKET_LENGTH_Y = 0.092
UPPER_SLIDE_UNDER_POCKET_DEPTH = 0.006
TOP_DECK_WIDTH_X = 0.060
TOP_DECK_LENGTH_Y = 0.060
TOP_DECK_HEIGHT = 0.010
LOWER_MOUNT_HOLE_RADIUS = 0.0032
TOP_DECK_HOLE_RADIUS = 0.0028

LOWER_TRAVEL = 0.030
UPPER_TRAVEL = 0.022


def _bottom_box(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False))


def _through_hole_cutter(
    points: list[tuple[float, float]],
    radius: float,
    height: float,
) -> cq.Workplane:
    return cq.Workplane("XY").pushPoints(points).circle(radius).extrude(height)


def _lower_guide_shape() -> cq.Workplane:
    base = _bottom_box(LOWER_LENGTH, LOWER_WIDTH, LOWER_BASE_THICKNESS)
    base = base.faces(">Z").edges().chamfer(0.0015)
    left_rail = _bottom_box(
        LOWER_RAIL_LENGTH,
        LOWER_RAIL_WIDTH,
        LOWER_RAIL_HEIGHT,
    ).edges("|Z").fillet(0.0010).translate(
        (0.0, -LOWER_RAIL_CENTER_Y, LOWER_BASE_THICKNESS)
    )
    right_rail = _bottom_box(
        LOWER_RAIL_LENGTH,
        LOWER_RAIL_WIDTH,
        LOWER_RAIL_HEIGHT,
    ).edges("|Z").fillet(0.0010).translate(
        (0.0, LOWER_RAIL_CENTER_Y, LOWER_BASE_THICKNESS)
    )
    mounting_holes = _through_hole_cutter(
        [
            (-0.065, -0.038),
            (-0.065, 0.038),
            (0.065, -0.038),
            (0.065, 0.038),
        ],
        LOWER_MOUNT_HOLE_RADIUS,
        LOWER_BASE_THICKNESS + LOWER_RAIL_HEIGHT + 0.002,
    )
    return base.union(left_rail).union(right_rail).cut(mounting_holes)


def _carriage_shape() -> cq.Workplane:
    body = _bottom_box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)
    body = body.faces(">Z").edges().chamfer(0.0015)
    under_pocket = _bottom_box(
        CARRIAGE_UNDER_POCKET_LENGTH,
        CARRIAGE_UNDER_POCKET_WIDTH,
        CARRIAGE_UNDER_POCKET_DEPTH,
    )
    left_top_rail = _bottom_box(
        CARRIAGE_TOP_RAIL_WIDTH_X,
        CARRIAGE_TOP_RAIL_LENGTH_Y,
        CARRIAGE_TOP_RAIL_HEIGHT,
    ).edges("|Z").fillet(0.0008).translate(
        (-CARRIAGE_TOP_RAIL_CENTER_X, 0.0, CARRIAGE_BODY_HEIGHT)
    )
    right_top_rail = _bottom_box(
        CARRIAGE_TOP_RAIL_WIDTH_X,
        CARRIAGE_TOP_RAIL_LENGTH_Y,
        CARRIAGE_TOP_RAIL_HEIGHT,
    ).edges("|Z").fillet(0.0008).translate(
        (CARRIAGE_TOP_RAIL_CENTER_X, 0.0, CARRIAGE_BODY_HEIGHT)
    )
    return body.cut(under_pocket).union(left_top_rail).union(right_top_rail)


def _upper_slide_shape() -> cq.Workplane:
    body = _bottom_box(
        UPPER_SLIDE_WIDTH_X,
        UPPER_SLIDE_LENGTH_Y,
        UPPER_SLIDE_BODY_HEIGHT,
    )
    body = body.faces(">Z").edges().chamfer(0.0012)
    under_pocket = _bottom_box(
        UPPER_SLIDE_UNDER_POCKET_WIDTH_X,
        UPPER_SLIDE_UNDER_POCKET_LENGTH_Y,
        UPPER_SLIDE_UNDER_POCKET_DEPTH,
    )
    top_deck = _bottom_box(
        TOP_DECK_WIDTH_X,
        TOP_DECK_LENGTH_Y,
        TOP_DECK_HEIGHT,
    ).edges("|Z").fillet(0.0010).translate((0.0, 0.0, UPPER_SLIDE_BODY_HEIGHT))
    deck_holes = _through_hole_cutter(
        [
            (-0.018, -0.018),
            (-0.018, 0.018),
            (0.018, -0.018),
            (0.018, 0.018),
        ],
        TOP_DECK_HOLE_RADIUS,
        UPPER_SLIDE_BODY_HEIGHT + TOP_DECK_HEIGHT + 0.001,
    )
    return body.cut(under_pocket).union(top_deck).cut(deck_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_fixture_xy_slide_block")

    model.material("dark_base", rgba=(0.27, 0.30, 0.33, 1.0))
    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))

    lower_guide = model.part("lower_guide")
    lower_guide.visual(
        mesh_from_cadquery(_lower_guide_shape(), "lower_guide_body"),
        material="dark_base",
        name="guide_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material="machined_aluminum",
        name="carriage_body",
    )

    upper_slide = model.part("upper_slide")
    upper_slide.visual(
        mesh_from_cadquery(_upper_slide_shape(), "upper_slide_body"),
        material="satin_aluminum",
        name="upper_slide_body",
    )

    model.articulation(
        "lower_guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=lower_guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_BASE_THICKNESS + LOWER_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
            effort=80.0,
            velocity=0.08,
        ),
    )
    model.articulation(
        "carriage_to_upper_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=upper_slide,
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_BODY_HEIGHT + CARRIAGE_TOP_RAIL_HEIGHT)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_TRAVEL,
            upper=UPPER_TRAVEL,
            effort=60.0,
            velocity=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_guide = object_model.get_part("lower_guide")
    carriage = object_model.get_part("carriage")
    upper_slide = object_model.get_part("upper_slide")
    lower_axis = object_model.get_articulation("lower_guide_to_carriage")
    upper_axis = object_model.get_articulation("carriage_to_upper_slide")

    ctx.check(
        "lower axis is prismatic along +X",
        lower_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(lower_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={lower_axis.articulation_type}, axis={lower_axis.axis}",
    )
    ctx.check(
        "upper axis is prismatic along +Y",
        upper_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(upper_axis.axis) == (0.0, 1.0, 0.0),
        details=f"type={upper_axis.articulation_type}, axis={upper_axis.axis}",
    )

    ctx.expect_contact(
        carriage,
        lower_guide,
        contact_tol=0.0005,
        name="carriage is seated on the lower guide",
    )
    ctx.expect_contact(
        upper_slide,
        carriage,
        contact_tol=0.0005,
        name="upper slide is seated on the carriage",
    )

    ctx.expect_within(
        carriage,
        lower_guide,
        axes="y",
        margin=0.002,
        name="carriage stays laterally captured by the lower guide",
    )
    ctx.expect_within(
        upper_slide,
        carriage,
        axes="x",
        margin=0.002,
        name="upper slide stays centered across the carriage width",
    )
    ctx.expect_overlap(
        carriage,
        lower_guide,
        axes="x",
        min_overlap=0.090,
        name="carriage has broad insertion on the lower guide at center",
    )
    ctx.expect_overlap(
        upper_slide,
        carriage,
        axes="y",
        min_overlap=0.070,
        name="upper slide has broad insertion on the carriage at center",
    )

    carriage_rest = ctx.part_world_position(carriage)
    upper_rest = ctx.part_world_position(upper_slide)
    with ctx.pose({lower_axis: LOWER_TRAVEL, upper_axis: UPPER_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            lower_guide,
            axes="x",
            min_overlap=0.090,
            name="carriage retains guide engagement at positive X travel",
        )
        ctx.expect_overlap(
            upper_slide,
            carriage,
            axes="y",
            min_overlap=0.056,
            name="upper slide retains guide engagement at positive Y travel",
        )
        ctx.expect_within(
            carriage,
            lower_guide,
            axes="y",
            margin=0.002,
            name="carriage remains laterally captured at positive X travel",
        )
        ctx.expect_within(
            upper_slide,
            carriage,
            axes="x",
            margin=0.002,
            name="upper slide remains laterally captured at positive Y travel",
        )
        carriage_extended = ctx.part_world_position(carriage)
        upper_extended = ctx.part_world_position(upper_slide)

    ctx.check(
        "carriage moves in +X at its upper limit",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.020,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )
    ctx.check(
        "upper slide moves in +Y at its upper limit",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] > upper_rest[1] + 0.015,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
