from __future__ import annotations

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


SLIDE_LENGTH = 0.55
MIDDLE_TRAVEL = 0.27
INNER_TRAVEL = 0.27


def _add_u_channel(part, *, width: float, height: float, wall: float, length: float, z_offset: float, material) -> None:
    """Add a connected sheet-metal U channel made from web and side-wall plates."""

    part.visual(
        Box((length, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, z_offset - height / 2.0 + wall / 2.0)),
        material=material,
        name="web",
    )
    for i, y in enumerate((-(width / 2.0 - wall / 2.0), width / 2.0 - wall / 2.0)):
        part.visual(
            Box((length, wall, height)),
            origin=Origin(xyz=(0.0, y, z_offset)),
            material=material,
            name=f"side_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bright_steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    polymer = model.material("black_polymer", rgba=(0.015, 0.015, 0.018, 1.0))
    zinc = model.material("zinc_fastener", rgba=(0.38, 0.40, 0.42, 1.0))

    outer = model.part("outer_frame")
    _add_u_channel(
        outer,
        width=0.090,
        height=0.054,
        wall=0.006,
        length=SLIDE_LENGTH,
        z_offset=0.0,
        material=galvanized,
    )
    outer.visual(
        Box((0.620, 0.120, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_steel,
        name="mounting_flange",
    )
    for i, x in enumerate((-0.230, 0.230)):
        for j, y in enumerate((-0.052, 0.052)):
            outer.visual(
                Cylinder(radius=0.0065, length=0.0015),
                origin=Origin(xyz=(x, y, -0.02625)),
                material=zinc,
                name=f"base_screw_{i}_{j}",
            )

    middle = model.part("middle_channel")
    _add_u_channel(
        middle,
        width=0.066,
        height=0.036,
        wall=0.005,
        length=SLIDE_LENGTH,
        z_offset=-0.003,
        material=bright_steel,
    )
    for i, y in enumerate((-0.0357, 0.0357)):
        middle.visual(
            Box((0.220, 0.0054, 0.014)),
            origin=Origin(xyz=(-0.095, y, -0.001)),
            material=polymer,
            name=f"outer_glide_{i}",
        )
        middle.visual(
            Box((0.220, 0.0054, 0.014)),
            origin=Origin(xyz=(0.095, y, -0.001)),
            material=polymer,
            name=f"outer_glide_{i}_front",
        )

    inner = model.part("inner_channel")
    _add_u_channel(
        inner,
        width=0.044,
        height=0.024,
        wall=0.004,
        length=SLIDE_LENGTH,
        z_offset=-0.004,
        material=galvanized,
    )
    for i, y in enumerate((-0.0247, 0.0247)):
        inner.visual(
            Box((0.240, 0.0054, 0.010)),
            origin=Origin(xyz=(-0.080, y, -0.003)),
            material=polymer,
            name=f"inner_glide_{i}",
        )
        inner.visual(
            Box((0.240, 0.0054, 0.010)),
            origin=Origin(xyz=(0.105, y, -0.003)),
            material=polymer,
            name=f"inner_glide_{i}_front",
        )

    tray = model.part("tray_plate")
    tray.visual(
        Box((0.180, 0.160, 0.008)),
        origin=Origin(xyz=(0.090, 0.0, 0.034)),
        material=bright_steel,
        name="plate",
    )
    tray.visual(
        Box((0.040, 0.048, 0.022)),
        origin=Origin(xyz=(0.000, 0.0, 0.019)),
        material=dark_steel,
        name="front_saddle",
    )
    for i, x in enumerate((0.045, 0.135)):
        for j, y in enumerate((-0.050, 0.050)):
            tray.visual(
                Cylinder(radius=0.007, length=0.0015),
                origin=Origin(xyz=(x, y, 0.03875)),
                material=zinc,
                name=f"tray_screw_{i}_{j}",
            )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=MIDDLE_TRAVEL),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=INNER_TRAVEL),
    )
    model.articulation(
        "inner_to_tray",
        ArticulationType.FIXED,
        parent=inner,
        child=tray,
        origin=Origin(xyz=(SLIDE_LENGTH / 2.0, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_frame")
    middle = object_model.get_part("middle_channel")
    inner = object_model.get_part("inner_channel")
    tray = object_model.get_part("tray_plate")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.002,
        name="middle section nests inside the fixed outer channel",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.002,
        name="inner section nests inside the middle channel",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.50,
        name="collapsed middle section is deeply retained",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.50,
        name="collapsed inner section is deeply retained",
    )
    ctx.expect_gap(
        tray,
        inner,
        axis="z",
        positive_elem="front_saddle",
        max_gap=0.001,
        max_penetration=0.0,
        name="tray saddle sits on the inner rail top",
    )
    ctx.expect_overlap(
        tray,
        inner,
        axes="xy",
        elem_a="front_saddle",
        min_overlap=0.020,
        name="tray saddle spans the inner rail at the moving end",
    )

    rest_tray = ctx.part_world_position(tray)
    with ctx.pose({outer_slide: MIDDLE_TRAVEL, inner_slide: INNER_TRAVEL}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.25,
            name="extended middle section remains inserted in outer frame",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.25,
            name="extended inner section remains inserted in middle section",
        )
        ctx.expect_gap(
            tray,
            outer,
            axis="x",
            min_gap=0.25,
            name="tray projects beyond the fixed frame when fully extended",
        )
        extended_tray = ctx.part_world_position(tray)

    ctx.check(
        "tray translates along the slide axis",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[0] > rest_tray[0] + MIDDLE_TRAVEL + INNER_TRAVEL - 0.01,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()
