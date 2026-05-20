from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _mat(name: str, color: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=color)


OUTER_LENGTH = 0.700
MIDDLE_LENGTH = 0.620
INNER_LENGTH = 0.540
SLIDE_TRAVEL = 0.300

OUTER_W = 0.120
OUTER_H = 0.080
OUTER_T = 0.008

MIDDLE_W = 0.086
MIDDLE_H = 0.052
MIDDLE_T = 0.006

INNER_W = 0.058
INNER_H = 0.030
INNER_T = 0.004


def _add_outer_channel(part, *, steel: Material, black: Material) -> None:
    """Grounded U-shaped outer channel with top return lips and mounting feet."""
    part.visual(
        Box((OUTER_LENGTH, OUTER_W, OUTER_T)),
        origin=Origin(xyz=(0.0, 0.0, -OUTER_H / 2.0 + OUTER_T / 2.0)),
        material=steel,
        name="outer_floor",
    )
    for sign, suffix in ((1.0, "a"), (-1.0, "b")):
        y = sign * (OUTER_W / 2.0 - OUTER_T / 2.0)
        part.visual(
            Box((OUTER_LENGTH, OUTER_T, OUTER_H)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel,
            name=f"outer_side_{suffix}",
        )
        part.visual(
            Box((OUTER_LENGTH, 0.026, OUTER_T)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * (OUTER_W / 2.0 - 0.026 / 2.0),
                    OUTER_H / 2.0 - OUTER_T / 2.0,
                )
            ),
            material=steel,
            name=f"outer_lip_{suffix}",
        )

    # Three bolted feet make the root read as the grounded fixed channel.
    for i, x in enumerate((-0.26, 0.0, 0.26)):
        part.visual(
            Box((0.140, 0.160, 0.008)),
            origin=Origin(xyz=(x, 0.0, -0.068)),
            material=steel,
            name=f"foot_plate_{i}",
        )
        part.visual(
            Box((0.100, 0.100, 0.024)),
            origin=Origin(xyz=(x, 0.0, -0.052)),
            material=steel,
            name=f"foot_riser_{i}",
        )
        for j, y in enumerate((-0.050, 0.050)):
            part.visual(
                Cylinder(radius=0.008, length=0.0065),
                origin=Origin(xyz=(x, y, -0.06125)),
                material=black,
                name=f"bolt_head_{i}_{j}",
            )


def _add_box_member(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    metal: Material,
    name_prefix: str,
) -> None:
    """Open-top rectangular box member, made from overlapping wall plates."""
    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, -height / 2.0 + wall / 2.0)),
        material=metal,
        name=f"{name_prefix}_floor",
    )
    for sign, suffix in ((1.0, "a"), (-1.0, "b")):
        part.visual(
            Box((length, wall, height)),
            origin=Origin(xyz=(0.0, sign * (width / 2.0 - wall / 2.0), 0.0)),
            material=metal,
            name=f"{name_prefix}_side_{suffix}",
        )
        lip_width = min(0.018, width * 0.32)
        part.visual(
            Box((length, lip_width, wall)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * (width / 2.0 - lip_width / 2.0),
                    height / 2.0 - wall / 2.0,
                )
            ),
            material=metal,
            name=f"{name_prefix}_lip_{suffix}",
        )


def _add_middle_glides(part, *, nylon: Material) -> None:
    """Nylon pads that fill the outer-channel clearance and visibly carry the slide."""
    # Side pads touch the outer channel's inside faces while slightly entering the
    # middle side wall, making the mounting path explicit without cross-part overlap.
    for sign, suffix in ((1.0, "a"), (-1.0, "b")):
        for i, x in enumerate((-0.220, 0.220)):
            part.visual(
                Box((0.090, 0.0095, 0.014)),
                origin=Origin(xyz=(x, sign * 0.04725, 0.0)),
                material=nylon,
                name=f"outer_side_glide_{suffix}_{i}",
            )
    for i, x in enumerate((-0.220, 0.220)):
        part.visual(
            Box((0.100, 0.026, 0.0065)),
            origin=Origin(xyz=(x, 0.0, -0.02875)),
            material=nylon,
            name=f"outer_bottom_glide_{i}",
        )


def _add_inner_glides(part, *, nylon: Material) -> None:
    """Smaller pads that support the inner member inside the middle box member."""
    for sign, suffix in ((1.0, "a"), (-1.0, "b")):
        for i, x in enumerate((-0.190, 0.190)):
            part.visual(
                Box((0.075, 0.0115, 0.010)),
                origin=Origin(xyz=(x, sign * 0.03475, 0.0)),
                material=nylon,
                name=f"inner_side_glide_{suffix}_{i}",
            )
    for i, x in enumerate((-0.190, 0.190)):
        part.visual(
            Box((0.085, 0.020, 0.0055)),
            origin=Origin(xyz=(x, 0.0, -0.01725)),
            material=nylon,
            name=f"inner_bottom_glide_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_box_telescoping_slide")

    galvanized = _mat("galvanized_steel", (0.64, 0.66, 0.63, 1.0))
    zinc = _mat("zinc_plated_steel", (0.80, 0.80, 0.76, 1.0))
    dark_steel = _mat("dark_inner_steel", (0.30, 0.31, 0.32, 1.0))
    nylon = _mat("black_nylon_glide", (0.02, 0.02, 0.018, 1.0))

    outer = model.part("outer_channel")
    _add_outer_channel(outer, steel=galvanized, black=nylon)

    middle = model.part("middle_member")
    _add_box_member(
        middle,
        length=MIDDLE_LENGTH,
        width=MIDDLE_W,
        height=MIDDLE_H,
        wall=MIDDLE_T,
        metal=zinc,
        name_prefix="middle",
    )
    _add_middle_glides(middle, nylon=nylon)

    inner = model.part("inner_member")
    _add_box_member(
        inner,
        length=INNER_LENGTH,
        width=INNER_W,
        height=INNER_H,
        wall=INNER_T,
        metal=dark_steel,
        name_prefix="inner",
    )
    _add_inner_glides(inner, nylon=nylon)

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=120.0, velocity=0.50),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=80.0, velocity=0.50),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "two serial prismatic slides",
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.articulation_type == ArticulationType.PRISMATIC
        and outer_slide.parent == "outer_channel"
        and outer_slide.child == "middle_member"
        and inner_slide.parent == "middle_member"
        and inner_slide.child == "inner_member",
        details=f"outer={outer_slide}, inner={inner_slide}",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.001,
        name="middle member nests in the grounded channel cross-section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.001,
        name="inner member nests in the middle member cross-section",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.55,
        name="collapsed middle member has long engagement in outer channel",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.48,
        name="collapsed inner member has long engagement in middle member",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: SLIDE_TRAVEL, inner_slide: SLIDE_TRAVEL}):
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.28,
            name="extended middle member remains retained in outer channel",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.20,
            name="extended inner member remains retained in middle member",
        )

    ctx.check(
        "serial extension moves members outward",
        rest_middle is not None
        and extended_middle is not None
        and rest_inner is not None
        and extended_inner is not None
        and extended_middle[0] > rest_middle[0] + 0.25
        and extended_inner[0] > rest_inner[0] + 0.55,
        details=(
            f"rest_middle={rest_middle}, extended_middle={extended_middle}, "
            f"rest_inner={rest_inner}, extended_inner={extended_inner}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
