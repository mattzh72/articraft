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


OUTER_LENGTH = 0.450
MIDDLE_LENGTH = 0.360
INNER_LENGTH = 0.300

OUTER_WIDTH = 0.018
MIDDLE_WIDTH = 0.0134
INNER_WIDTH = 0.0092

OUTER_HEIGHT = 0.040
MIDDLE_HEIGHT = 0.031
INNER_HEIGHT = 0.022

OUTER_WALL = 0.0012
MIDDLE_WALL = 0.0010
INNER_WALL = 0.0009

OUTER_TRAVEL = 0.210
INNER_TRAVEL = 0.170

OUTER_GUIDE_Y = 0.0011
MIDDLE_GUIDE_Y = -0.0011
OUTER_GUIDE_Z = 0.0020
MIDDLE_GUIDE_Z = 0.0015


def _return_lip_height(height: float, wall: float) -> float:
    return min(max(height * 0.22, wall * 4.0), (height - 2.0 * wall) * 0.34)


def _add_channel_visuals(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    open_side: str,
    material: str,
    prefix: str,
) -> None:
    lip_height = _return_lip_height(height, wall)

    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(length / 2.0, 0.0, wall / 2.0)),
        material=material,
        name=f"{prefix}_bottom_flange",
    )
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(length / 2.0, 0.0, height - wall / 2.0)),
        material=material,
        name=f"{prefix}_top_flange",
    )

    web_y = -width / 2.0 + wall / 2.0 if open_side == "+y" else width / 2.0 - wall / 2.0
    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(length / 2.0, web_y, height / 2.0)),
        material=material,
        name=f"{prefix}_web",
    )

    lip_y = width / 2.0 - wall / 2.0 if open_side == "+y" else -width / 2.0 + wall / 2.0
    part.visual(
        Box((length, wall, lip_height)),
        origin=Origin(xyz=(length / 2.0, lip_y, wall + lip_height / 2.0)),
        material=material,
        name=f"{prefix}_lower_return",
    )
    part.visual(
        Box((length, wall, lip_height)),
        origin=Origin(xyz=(length / 2.0, lip_y, height - wall - lip_height / 2.0)),
        material=material,
        name=f"{prefix}_upper_return",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_rail_telescoping_runner")

    model.material("outer_zinc", rgba=(0.65, 0.68, 0.73, 1.0))
    model.material("middle_steel", rgba=(0.50, 0.53, 0.58, 1.0))
    model.material("inner_zinc", rgba=(0.76, 0.79, 0.83, 1.0))

    outer = model.part("outer_rail")
    _add_channel_visuals(
        outer,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        open_side="+y",
        material="outer_zinc",
        prefix="outer",
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    middle = model.part("middle_runner")
    _add_channel_visuals(
        middle,
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        open_side="-y",
        material="middle_steel",
        prefix="middle",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.34,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )

    inner = model.part("inner_runner")
    _add_channel_visuals(
        inner,
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        open_side="+y",
        material="inner_zinc",
        prefix="inner",
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=0.21,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        # Real box-rail slides do not float concentrically; the nested runner is
        # carried against one stamped guide face. Bias the middle stage toward
        # the return lips so the formed guide surfaces stay in exact contact.
        origin=Origin(xyz=(0.0, OUTER_GUIDE_Y, OUTER_GUIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TRAVEL,
            effort=120.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        # The inner output runner is likewise carried on one side of the middle
        # runner rather than centered in free space.
        origin=Origin(xyz=(0.0, MIDDLE_GUIDE_Y, MIDDLE_GUIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=90.0,
            velocity=0.45,
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
    outer = object_model.get_part("outer_rail")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0005,
        name="middle runner stays centered inside outer rail at rest",
    )
    ctx.expect_contact(
        middle,
        outer,
        name="middle runner bears on outer guide faces at rest",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.30,
        name="middle runner has strong retained insertion at rest",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0005,
        name="inner runner stays centered inside middle runner at rest",
    )
    ctx.expect_contact(
        inner,
        middle,
        name="inner runner bears on middle guide faces at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.24,
        name="inner runner has strong retained insertion at rest",
    )

    middle_rest = ctx.part_world_position(middle)
    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({outer_to_middle: OUTER_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0005,
            name="middle runner stays guided inside outer rail at max travel",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.14,
            name="middle runner retains insertion at max outer travel",
        )
        middle_extended = ctx.part_world_position(middle)

    with ctx.pose({outer_to_middle: OUTER_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0005,
            name="inner runner stays guided inside middle runner at max travel",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.12,
            name="inner runner retains insertion at full extension",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "middle stage extends along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner stage extends along +X beyond middle motion",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.30,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
