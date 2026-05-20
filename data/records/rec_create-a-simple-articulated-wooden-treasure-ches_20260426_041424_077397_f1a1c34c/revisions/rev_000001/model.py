from __future__ import annotations

import math

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


BODY_DEPTH = 0.46
BODY_WIDTH = 0.82
BODY_HEIGHT = 0.36
WALL_THICKNESS = 0.02
FLOOR_THICKNESS = 0.025

LID_DEPTH = BODY_DEPTH
LID_WIDTH = BODY_WIDTH + 0.06
LID_RADIUS = LID_DEPTH / 2.0
LID_THICKNESS = 0.018
LID_CLEARANCE = 0.004
LID_OPEN_ANGLE = math.radians(78.0)

CLASP_THICKNESS = 0.006


def _body_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        BODY_DEPTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        centered=(True, True, False),
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - 2.0 * WALL_THICKNESS,
            BODY_WIDTH - 2.0 * WALL_THICKNESS,
            BODY_HEIGHT - FLOOR_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, FLOOR_THICKNESS))
    )
    return outer.cut(cavity).edges("|Z").fillet(0.01)


def _lid_shape() -> cq.Workplane:
    outer_cylinder = (
        cq.Workplane("XZ")
        .center(LID_DEPTH / 2.0, 0.0)
        .circle(LID_RADIUS)
        .extrude(LID_WIDTH / 2.0, both=True)
    )
    outer_bounds = cq.Workplane("XY").box(
        LID_DEPTH,
        LID_WIDTH,
        LID_RADIUS + 0.03,
        centered=(False, True, False),
    )
    outer_shell = outer_cylinder.intersect(outer_bounds)

    inner_cylinder = (
        cq.Workplane("XZ")
        .center(LID_DEPTH / 2.0, 0.0)
        .circle(LID_RADIUS - LID_THICKNESS)
        .extrude((LID_WIDTH - 2.0 * LID_THICKNESS) / 2.0, both=True)
    )
    inner_bounds = (
        cq.Workplane("XY")
        .box(
            LID_DEPTH - 2.0 * LID_THICKNESS,
            LID_WIDTH - 2.0 * LID_THICKNESS,
            LID_RADIUS + 0.10,
            centered=(False, True, False),
        )
        .translate((LID_THICKNESS, 0.0, -0.05))
    )
    return outer_shell.cut(inner_cylinder.intersect(inner_bounds))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((low + high) * 0.5 for low, high in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_treasure_chest")

    wood = model.material("wood", rgba=(0.55, 0.34, 0.16, 1.0))
    brass = model.material("brass", rgba=(0.75, 0.62, 0.25, 1.0))

    chest = model.part("chest")
    chest.visual(
        mesh_from_cadquery(_body_shape(), "chest_body"),
        material=wood,
        name="body_shell",
    )
    chest.visual(
        Box((CLASP_THICKNESS, 0.10, 0.08)),
        origin=Origin(
            xyz=(
                BODY_DEPTH / 2.0 + CLASP_THICKNESS / 2.0,
                0.0,
                0.18,
            )
        ),
        material=brass,
        name="body_clasp",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "chest_lid"),
        material=wood,
        name="lid_shell",
    )
    lid.visual(
        Box((CLASP_THICKNESS, 0.08, 0.14)),
        origin=Origin(
            xyz=(
                LID_DEPTH + CLASP_THICKNESS / 2.0,
                0.0,
                -0.066,
            )
        ),
        material=brass,
        name="lid_clasp",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=chest,
        child=lid,
        origin=Origin(xyz=(-BODY_DEPTH / 2.0, 0.0, BODY_HEIGHT + LID_CLEARANCE)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=LID_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chest = object_model.get_part("chest")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        chest,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        min_gap=0.002,
        max_gap=0.012,
        name="closed lid sits just above the chest rim",
    )
    ctx.expect_overlap(
        lid,
        chest,
        axes="xy",
        min_overlap=0.35,
        name="closed lid covers the treasure chest opening",
    )
    ctx.expect_overlap(
        lid,
        chest,
        axes="y",
        elem_a="lid_clasp",
        elem_b="body_clasp",
        min_overlap=0.07,
        name="front clasp pieces stay centered",
    )
    ctx.expect_gap(
        lid,
        chest,
        axis="z",
        positive_elem="lid_clasp",
        negative_elem="body_clasp",
        min_gap=0.002,
        max_gap=0.012,
        name="closed clasp nearly meets the body latch",
    )

    closed_clasp_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_clasp"))

    with ctx.pose({hinge: LID_OPEN_ANGLE}):
        ctx.expect_gap(
            lid,
            chest,
            axis="z",
            positive_elem="lid_clasp",
            negative_elem="body_clasp",
            min_gap=0.25,
            name="opened lid lifts the clasp well clear of the chest body",
        )
        open_clasp_center = _aabb_center(ctx.part_element_world_aabb(lid, elem="lid_clasp"))

    moved_clearly = (
        closed_clasp_center is not None
        and open_clasp_center is not None
        and open_clasp_center[2] > closed_clasp_center[2] + 0.20
        and open_clasp_center[0] < closed_clasp_center[0] - 0.18
    )
    ctx.check(
        "lid swings upward and backward when opened",
        moved_clearly,
        details=f"closed_clasp_center={closed_clasp_center}, open_clasp_center={open_clasp_center}",
    )

    return ctx.report()


object_model = build_object_model()
