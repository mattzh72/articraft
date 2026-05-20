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


def make_guide_block() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.14, 0.14, 0.3)
        .faces(">Z")
        .workplane()
        .rect(0.105, 0.105)
        .cutThruAll()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lift_carriage")

    frame = model.part("frame")
    frame.visual(Box((1.2, 0.2, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)), name="base")
    frame.visual(Box((1.2, 0.2, 0.1)), origin=Origin(xyz=(0.0, 0.0, 1.95)), name="top")
    frame.visual(Box((0.1, 0.1, 1.8)), origin=Origin(xyz=(-0.5, 0.0, 1.0)), name="column_0")
    frame.visual(Box((0.1, 0.1, 1.8)), origin=Origin(xyz=(0.5, 0.0, 1.0)), name="column_1")

    carriage = model.part("carriage")
    carriage.visual(Box((0.88, 0.2, 0.3)), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="body")

    gb_mesh = mesh_from_cadquery(make_guide_block(), "guide_block")
    carriage.visual(gb_mesh, origin=Origin(xyz=(-0.5, 0.0, 0.0)), name="guide_block_0")
    carriage.visual(gb_mesh, origin=Origin(xyz=(0.5, 0.0, 0.0)), name="guide_block_1")

    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=1.0, lower=0.0, upper=1.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("lift")

    ctx.allow_isolated_part(
        carriage,
        reason="The carriage rides on the columns with a designed clearance and is supported by the prismatic joint.",
    )

    ctx.expect_within(
        frame,
        carriage,
        axes="xy",
        inner_elem="column_0",
        outer_elem="guide_block_0",
        margin=0.0,
    )
    ctx.expect_within(
        frame,
        carriage,
        axes="xy",
        inner_elem="column_1",
        outer_elem="guide_block_1",
        margin=0.0,
    )

    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="body",
        negative_elem="base",
        min_gap=0.005,
    )

    with ctx.pose({lift: 1.48}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top",
            negative_elem="body",
            min_gap=0.005,
        )

    return ctx.report()


object_model = build_object_model()