from __future__ import annotations

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


BASE_LENGTH = 3.70
FLY_LENGTH = 3.50
FLY_COLLAPSED_Z = 1.00
FLY_TRAVEL = 1.45

RAIL_CENTER_X = 0.180

BASE_RAIL_WIDTH = 0.068
BASE_RAIL_DEPTH = 0.036
BASE_RAIL_WALL = 0.0045

FLY_RAIL_WIDTH = 0.055
FLY_RAIL_DEPTH = 0.019
FLY_RAIL_WALL = 0.0035
FLY_Y_OFFSET = 0.031

RUNG_SPACING = 0.280
BASE_RUNG_DEPTH = 0.016
BASE_RUNG_HEIGHT = 0.028
BASE_RUNG_Y = -0.010

FLY_RUNG_DEPTH = 0.016
FLY_RUNG_HEIGHT = 0.026
FLY_RUNG_Y = 0.002

BASE_RUNG_LENGTH = 0.316
FLY_RUNG_LENGTH = 0.329


def _u_channel_rail(
    *,
    length: float,
    outer_width: float,
    outer_depth: float,
    wall: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        outer_width,
        outer_depth,
        length,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            outer_width - 2.0 * wall,
            outer_depth - wall,
            length + 0.010,
            centered=(True, True, False),
        )
        .translate((0.0, wall / 2.0, -0.005))
    )
    return outer.cut(inner)


def _box_tube_rail(
    *,
    length: float,
    outer_width: float,
    outer_depth: float,
    wall: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        outer_width,
        outer_depth,
        length,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            outer_width - 2.0 * wall,
            outer_depth - 2.0 * wall,
            length + 0.010,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.005))
    )
    return outer.cut(inner)


def _rung_bank(
    *,
    length: float,
    rung_length: float,
    rung_depth: float,
    rung_height: float,
    center_y: float,
    first_center_z: float,
    top_margin: float,
) -> cq.Workplane:
    rung = cq.Workplane("XY").box(rung_length, rung_depth, rung_height)
    rungs: cq.Workplane | None = None
    z = first_center_z
    while z <= length - top_margin:
        placed = rung.translate((0.0, center_y, z))
        rungs = placed if rungs is None else rungs.union(placed)
        z += RUNG_SPACING
    assert rungs is not None
    return rungs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_hardware = model.material("hardware", rgba=(0.22, 0.22, 0.24, 1.0))

    base_rail_mesh = mesh_from_cadquery(
        _u_channel_rail(
            length=BASE_LENGTH,
            outer_width=BASE_RAIL_WIDTH,
            outer_depth=BASE_RAIL_DEPTH,
            wall=BASE_RAIL_WALL,
        ),
        "base_rail",
    )
    fly_rail_mesh = mesh_from_cadquery(
        _box_tube_rail(
            length=FLY_LENGTH,
            outer_width=FLY_RAIL_WIDTH,
            outer_depth=FLY_RAIL_DEPTH,
            wall=FLY_RAIL_WALL,
        ),
        "fly_rail",
    )
    base_rungs_mesh = mesh_from_cadquery(
        _rung_bank(
            length=BASE_LENGTH,
            rung_length=BASE_RUNG_LENGTH,
            rung_depth=BASE_RUNG_DEPTH,
            rung_height=BASE_RUNG_HEIGHT,
            center_y=BASE_RUNG_Y,
            first_center_z=0.34,
            top_margin=0.28,
        ),
        "base_rungs",
    )
    fly_rungs_mesh = mesh_from_cadquery(
        _rung_bank(
            length=FLY_LENGTH,
            rung_length=FLY_RUNG_LENGTH,
            rung_depth=FLY_RUNG_DEPTH,
            rung_height=FLY_RUNG_HEIGHT,
            center_y=FLY_RUNG_Y,
            first_center_z=0.28,
            top_margin=0.24,
        ),
        "fly_rungs",
    )

    base = model.part("base")
    base.visual(
        base_rail_mesh,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="rail_0",
    )
    base.visual(
        base_rail_mesh,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="rail_1",
    )
    base.visual(base_rungs_mesh, material=aluminum, name="rungs")
    for index, x in enumerate((-RAIL_CENTER_X, RAIL_CENTER_X)):
        for guide_index, guide_z in enumerate((2.70, 3.45)):
            for side_index, guide_x in enumerate(
                (
                    x - (FLY_RAIL_WIDTH / 2.0 + 0.001),
                    x + (FLY_RAIL_WIDTH / 2.0 + 0.001),
                )
            ):
                base.visual(
                    Box((0.002, 0.0035, 0.120)),
                    origin=Origin(xyz=(guide_x, 0.01975, guide_z)),
                    material=dark_hardware,
                    name=f"guide_{index}_{guide_index}_{side_index}",
                )
        base.visual(
            Box((0.086, 0.050, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.029)),
            material=rubber,
            name=f"foot_{index}",
        )

    fly = model.part("fly")
    fly.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="rail_0",
    )
    fly.visual(
        fly_rail_mesh,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminum,
        name="rail_1",
    )
    fly.visual(fly_rungs_mesh, material=aluminum, name="rungs")
    for index, x in enumerate((-RAIL_CENTER_X, RAIL_CENTER_X)):
        fly.visual(
            Box((0.064, 0.026, 0.040)),
            origin=Origin(xyz=(x, 0.0, FLY_LENGTH - 0.020)),
            material=dark_hardware,
            name=f"cap_{index}",
        )
        for guide_index, guide_z in enumerate((1.60, 2.50)):
            fly.visual(
                Box((0.040, 0.006, 0.090)),
                origin=Origin(xyz=(x, -0.010, guide_z)),
                material=dark_hardware,
                name=f"guide_{index}_{guide_index}",
            )
        fly.visual(
            Box((0.018, 0.020, 0.110)),
            origin=Origin(xyz=(x * 1.06, 0.016, 0.62)),
            material=dark_hardware,
            name=f"lock_{index}",
        )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, FLY_Y_OFFSET, FLY_COLLAPSED_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.35,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    slide = object_model.get_articulation("base_to_fly")

    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem="rail_0",
        outer_elem="rail_0",
        margin=0.002,
        name="left fly rail stays laterally aligned with left base guide",
    )
    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem="rail_1",
        outer_elem="rail_1",
        margin=0.002,
        name="right fly rail stays laterally aligned with right base guide",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        positive_elem="rail_0",
        negative_elem="rail_0",
        min_gap=0.003,
        max_gap=0.006,
        name="left fly rail runs just ahead of left base channel",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        positive_elem="rail_1",
        negative_elem="rail_1",
        min_gap=0.003,
        max_gap=0.006,
        name="right fly rail runs just ahead of right base channel",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        elem_a="rail_0",
        elem_b="rail_0",
        min_overlap=2.60,
        name="collapsed fly retains deep left-rail insertion",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        elem_a="rail_1",
        elem_b="rail_1",
        min_overlap=2.60,
        name="collapsed fly retains deep right-rail insertion",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: FLY_TRAVEL}):
        ctx.expect_within(
            fly,
            base,
            axes="x",
            inner_elem="rail_0",
            outer_elem="rail_0",
            margin=0.002,
            name="extended left fly rail stays laterally aligned with left base guide",
        )
        ctx.expect_within(
            fly,
            base,
            axes="x",
            inner_elem="rail_1",
            outer_elem="rail_1",
            margin=0.002,
            name="extended right fly rail stays laterally aligned with right base guide",
        )
        ctx.expect_gap(
            fly,
            base,
            axis="y",
            positive_elem="rail_0",
            negative_elem="rail_0",
            min_gap=0.003,
            max_gap=0.006,
            name="extended left fly rail remains just ahead of left base channel",
        )
        ctx.expect_gap(
            fly,
            base,
            axis="y",
            positive_elem="rail_1",
            negative_elem="rail_1",
            min_gap=0.003,
            max_gap=0.006,
            name="extended right fly rail remains just ahead of right base channel",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a="rail_0",
            elem_b="rail_0",
            min_overlap=1.20,
            name="extended fly retains left-rail insertion",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a="rail_1",
            elem_b="rail_1",
            min_overlap=1.20,
            name="extended fly retains right-rail insertion",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section rises upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 1.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
