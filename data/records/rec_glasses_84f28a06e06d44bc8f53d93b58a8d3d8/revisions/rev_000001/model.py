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


LENS_OUTER_W = 0.056
LENS_OUTER_H = 0.038
LENS_INNER_W = 0.046
LENS_INNER_H = 0.028
FRAME_DEPTH = 0.004
BRIDGE_GAP = 0.014
RIM_CENTER_X = (LENS_OUTER_W + BRIDGE_GAP) / 2.0
HINGE_X = RIM_CENTER_X + (LENS_OUTER_W / 2.0) + 0.003
HINGE_Y = 0.0055
HINGE_Z = 0.004
TEMPLE_LENGTH = 0.126


def _rim(center_x: float) -> cq.Workplane:
    """One rounded rectangular eyeglass rim in the model's X/Z front plane."""
    rim = cq.Workplane("XY").box(LENS_OUTER_W, FRAME_DEPTH, LENS_OUTER_H)
    lens_opening = cq.Workplane("XY").box(
        LENS_INNER_W, FRAME_DEPTH * 3.0, LENS_INNER_H
    )
    rim = rim.cut(lens_opening)
    rim = rim.edges("|Y").fillet(0.0018)
    return rim.translate((center_x, 0.0, 0.0))


def _front_frame_geometry() -> cq.Workplane:
    """Unified front frame: two rims, a bridge, and compact hinge blocks."""
    left_rim = _rim(-RIM_CENTER_X)
    right_rim = _rim(RIM_CENTER_X)

    bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_GAP + 0.006, FRAME_DEPTH, 0.006)
        .edges("|Y")
        .fillet(0.0012)
        .translate((0.0, 0.0, 0.006))
    )

    upper_bridge = (
        cq.Workplane("XY")
        .box(BRIDGE_GAP + 0.010, FRAME_DEPTH * 0.95, 0.003)
        .edges("|Y")
        .fillet(0.0010)
        .translate((0.0, 0.0, 0.014))
    )

    frame = left_rim.union(right_rim).union(bridge).union(upper_bridge)

    for side in (-1.0, 1.0):
        hinge_block = (
            cq.Workplane("XY")
            .box(0.008, 0.008, 0.024)
            .edges("|Y")
            .fillet(0.0010)
            .translate((side * HINGE_X, 0.002, HINGE_Z))
        )
        frame = frame.union(hinge_block)

    return frame


def _temple_arm_geometry() -> cq.Workplane:
    """A slim temple arm modeled in a hinge-local frame extending along +Y."""
    hinge_leaf = (
        cq.Workplane("XY")
        .box(0.006, 0.010, 0.020)
        .edges("|Y")
        .fillet(0.0008)
        .translate((0.0, 0.0055, 0.0))
    )

    main_arm = (
        cq.Workplane("XY")
        .box(0.0040, 0.108, 0.0040)
        .edges("|Y")
        .fillet(0.0007)
        .translate((0.0, 0.062, 0.005))
    )

    ear_bend = (
        cq.Workplane("XY")
        .box(0.0043, 0.026, 0.0090)
        .edges("|Y")
        .fillet(0.0007)
        .translate((0.0, 0.107, 0.001))
    )

    ear_curve = (
        cq.Workplane("XY")
        .box(0.0044, 0.030, 0.0044)
        .edges("|Y")
        .fillet(0.0008)
        .translate((0.0, 0.115, -0.002))
    )

    return hinge_leaf.union(main_arm).union(ear_bend).union(ear_curve)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_eyeglasses")

    frame_material = model.material("satin_black_acetate", rgba=(0.01, 0.01, 0.012, 1.0))
    lens_material = model.material("faint_blue_lens", rgba=(0.72, 0.90, 1.0, 0.36))

    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_cadquery(_front_frame_geometry(), "front_frame_shell", tolerance=0.0005),
        material=frame_material,
        name="frame_shell",
    )
    for name, center_x in (("lens_0", -RIM_CENTER_X), ("lens_1", RIM_CENTER_X)):
        front_frame.visual(
            Box((LENS_INNER_W + 0.002, 0.0012, LENS_INNER_H + 0.002)),
            origin=Origin(xyz=(center_x, 0.0, 0.0)),
            material=lens_material,
            name=name,
        )

    temple_mesh = _temple_arm_geometry()
    temple_0 = model.part("temple_0")
    temple_0.visual(
        mesh_from_cadquery(temple_mesh, "temple_0_shell", tolerance=0.0005),
        material=frame_material,
        name="arm_shell",
    )
    temple_1 = model.part("temple_1")
    temple_1.visual(
        mesh_from_cadquery(temple_mesh, "temple_1_shell", tolerance=0.0005),
        material=frame_material,
        name="arm_shell",
    )

    model.articulation(
        "front_to_temple_0",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=temple_0,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "front_to_temple_1",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=temple_1,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")
    hinge_0 = object_model.get_articulation("front_to_temple_0")
    hinge_1 = object_model.get_articulation("front_to_temple_1")

    ctx.expect_overlap(
        front_frame,
        front_frame,
        axes="xz",
        elem_a="lens_0",
        elem_b="frame_shell",
        min_overlap=0.002,
        name="left lens is retained by its rim",
    )
    ctx.expect_overlap(
        front_frame,
        front_frame,
        axes="xz",
        elem_a="lens_1",
        elem_b="frame_shell",
        min_overlap=0.002,
        name="right lens is retained by its rim",
    )
    ctx.expect_gap(
        temple_0,
        front_frame,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="temple 0 hinge leaf sits against the frame block",
    )
    ctx.expect_gap(
        temple_1,
        front_frame,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="temple 1 hinge leaf sits against the frame block",
    )

    temple_0_open = ctx.part_world_aabb(temple_0)
    temple_1_open = ctx.part_world_aabb(temple_1)
    with ctx.pose({hinge_0: 1.2, hinge_1: 1.2}):
        temple_0_folded = ctx.part_world_aabb(temple_0)
        temple_1_folded = ctx.part_world_aabb(temple_1)

    ctx.check(
        "temple 0 folds inward",
        temple_0_open is not None
        and temple_0_folded is not None
        and temple_0_folded[1][0] > temple_0_open[1][0] + 0.025,
        details=f"open={temple_0_open}, folded={temple_0_folded}",
    )
    ctx.check(
        "temple 1 folds inward",
        temple_1_open is not None
        and temple_1_folded is not None
        and temple_1_folded[0][0] < temple_1_open[0][0] - 0.025,
        details=f"open={temple_1_open}, folded={temple_1_folded}",
    )

    return ctx.report()


object_model = build_object_model()
