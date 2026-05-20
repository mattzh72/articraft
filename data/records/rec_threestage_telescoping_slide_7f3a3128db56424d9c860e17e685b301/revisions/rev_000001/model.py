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


SLIDE_AXIS = (1.0, 0.0, 0.0)

OUTER_LENGTH = 0.44
OUTER_WIDTH = 0.058
OUTER_HEIGHT = 0.036
OUTER_WALL = 0.0025
OUTER_SLOT_WIDTH = 0.032
OUTER_SLOT_END_MARGIN = 0.03

MIDDLE_HOME = 0.055
MIDDLE_TRAVEL = 0.18
MIDDLE_LENGTH = 0.39
MIDDLE_WIDTH = 0.038
MIDDLE_HEIGHT = 0.0275
MIDDLE_WALL = 0.0025
MIDDLE_BOTTOM_THICKNESS = 0.003
MIDDLE_SHOE_WIDTH = 0.005
MIDDLE_SHOE_THICKNESS = 0.0065
MIDDLE_SIDE_HEIGHT = 0.018

INNER_HOME = 0.09
INNER_TRAVEL = 0.15
INNER_LENGTH = 0.30
INNER_WIDTH = 0.020
INNER_HEIGHT = 0.008


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_telescoping_unit")

    model.material("outer_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("middle_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("inner_steel", rgba=(0.77, 0.79, 0.82, 1.0))

    outer = model.part("outer_sleeve")
    outer_inner_width = OUTER_WIDTH - (2.0 * OUTER_WALL)
    outer_lip_depth = max((outer_inner_width - OUTER_SLOT_WIDTH) / 2.0, 0.0)
    outer_slot_length = OUTER_LENGTH - (2.0 * OUTER_SLOT_END_MARGIN)
    _add_box(
        outer,
        name="outer_bottom",
        size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_WALL),
        xyz=(OUTER_LENGTH / 2.0, 0.0, -(OUTER_HEIGHT / 2.0) + (OUTER_WALL / 2.0)),
        material="outer_steel",
    )
    _add_box(
        outer,
        name="outer_left_wall",
        size=(OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT),
        xyz=(OUTER_LENGTH / 2.0, -(OUTER_WIDTH / 2.0) + (OUTER_WALL / 2.0), 0.0),
        material="outer_steel",
    )
    _add_box(
        outer,
        name="outer_right_wall",
        size=(OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT),
        xyz=(OUTER_LENGTH / 2.0, (OUTER_WIDTH / 2.0) - (OUTER_WALL / 2.0), 0.0),
        material="outer_steel",
    )
    if outer_lip_depth > 0.0:
        _add_box(
            outer,
            name="outer_left_lip",
            size=(outer_slot_length, outer_lip_depth, OUTER_WALL),
            xyz=(
                OUTER_LENGTH / 2.0,
                -(OUTER_WIDTH / 2.0) + OUTER_WALL + (outer_lip_depth / 2.0),
                (OUTER_HEIGHT / 2.0) - (OUTER_WALL / 2.0),
            ),
            material="outer_steel",
        )
        _add_box(
            outer,
            name="outer_right_lip",
            size=(outer_slot_length, outer_lip_depth, OUTER_WALL),
            xyz=(
                OUTER_LENGTH / 2.0,
                (OUTER_WIDTH / 2.0) - OUTER_WALL - (outer_lip_depth / 2.0),
                (OUTER_HEIGHT / 2.0) - (OUTER_WALL / 2.0),
            ),
            material="outer_steel",
        )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, 0.0)),
    )

    middle = model.part("middle_member")
    outer_bottom_top = -(OUTER_HEIGHT / 2.0) + OUTER_WALL
    middle_bottom_center_z = outer_bottom_top + (MIDDLE_SHOE_THICKNESS + MIDDLE_BOTTOM_THICKNESS) - (
        MIDDLE_BOTTOM_THICKNESS / 2.0
    )
    middle_bottom_top = middle_bottom_center_z + (MIDDLE_BOTTOM_THICKNESS / 2.0)
    middle_side_center_z = middle_bottom_top + (MIDDLE_SIDE_HEIGHT / 2.0)
    middle_shoe_center_z = outer_bottom_top + (MIDDLE_SHOE_THICKNESS / 2.0)
    middle_shoe_y = (MIDDLE_WIDTH / 2.0) - (MIDDLE_SHOE_WIDTH / 2.0)
    _add_box(
        middle,
        name="middle_bottom",
        size=(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_BOTTOM_THICKNESS),
        xyz=(MIDDLE_LENGTH / 2.0, 0.0, middle_bottom_center_z),
        material="middle_steel",
    )
    _add_box(
        middle,
        name="middle_left_wall",
        size=(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT),
        xyz=(MIDDLE_LENGTH / 2.0, -(MIDDLE_WIDTH / 2.0) + (MIDDLE_WALL / 2.0), middle_side_center_z),
        material="middle_steel",
    )
    _add_box(
        middle,
        name="middle_right_wall",
        size=(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT),
        xyz=(MIDDLE_LENGTH / 2.0, (MIDDLE_WIDTH / 2.0) - (MIDDLE_WALL / 2.0), middle_side_center_z),
        material="middle_steel",
    )
    _add_box(
        middle,
        name="middle_left_shoe",
        size=(MIDDLE_LENGTH, MIDDLE_SHOE_WIDTH, MIDDLE_SHOE_THICKNESS),
        xyz=(MIDDLE_LENGTH / 2.0, -middle_shoe_y, middle_shoe_center_z),
        material="middle_steel",
    )
    _add_box(
        middle,
        name="middle_right_shoe",
        size=(MIDDLE_LENGTH, MIDDLE_SHOE_WIDTH, MIDDLE_SHOE_THICKNESS),
        xyz=(MIDDLE_LENGTH / 2.0, middle_shoe_y, middle_shoe_center_z),
        material="middle_steel",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_member")
    inner_center_z = middle_bottom_top + (INNER_HEIGHT / 2.0)
    _add_box(
        inner,
        name="inner_member_body",
        size=(INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT),
        xyz=(INNER_LENGTH / 2.0, 0.0, inner_center_z),
        material="inner_steel",
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(MIDDLE_HOME, 0.0, 0.0)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(INNER_HOME, 0.0, 0.0)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            effort=110.0,
            velocity=0.5,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "both slide joints run along +X",
        outer_to_middle.axis == SLIDE_AXIS and middle_to_inner.axis == SLIDE_AXIS,
        details=f"outer_to_middle={outer_to_middle.axis}, middle_to_inner={middle_to_inner.axis}",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle member stays centered within outer sleeve at rest",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.20,
        name="middle member retains deep insertion in outer sleeve at rest",
    )
    ctx.expect_contact(
        middle,
        outer,
        name="middle member is physically supported by the outer sleeve at rest",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0,
        name="inner member stays centered within middle member at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.20,
        name="inner member retains insertion in middle member at rest",
    )
    ctx.expect_contact(
        inner,
        middle,
        name="inner member is physically supported by the middle member at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle member stays centered within outer sleeve when extended",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.20,
            name="middle member retains insertion in outer sleeve when extended",
        )
        ctx.expect_contact(
            middle,
            outer,
            name="middle member remains supported by the outer sleeve when extended",
        )
        middle_extended = ctx.part_world_position(middle)

    ctx.check(
        "middle member extends outward along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="inner member stays centered within middle member when extended",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.15,
            name="inner member retains insertion in middle member when extended",
        )
        ctx.expect_contact(
            inner,
            middle,
            name="inner member remains supported by the middle member when extended",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "inner member extends outward along +X",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.08,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle member stays aligned at full two-stage extension",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="inner member stays aligned at full two-stage extension",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.20,
            name="middle member keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.15,
            name="inner member keeps retained insertion at full extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
