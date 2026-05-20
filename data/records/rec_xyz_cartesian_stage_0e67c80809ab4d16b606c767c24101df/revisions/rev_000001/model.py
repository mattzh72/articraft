from __future__ import annotations

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


def make_rail(length: float, width: float, height: float, dir: str = "X") -> cq.Workplane:
    if dir == "X":
        rail = cq.Workplane("XY").box(length, width, height)
        rail = rail.edges(">Z").chamfer(0.002)
    elif dir == "Y":
        rail = cq.Workplane("XY").box(width, length, height)
        rail = rail.edges(">Z").chamfer(0.002)
    else:  # Z
        rail = cq.Workplane("XY").box(height, width, length)
        rail = rail.edges(">X").chamfer(0.002)
    return rail


def make_xy_carriage(size_x: float, size_y: float, size_z: float, slot_depth: float, slot_width: float, slot_dir: str = "X") -> cq.Workplane:
    block = cq.Workplane("XY").box(size_x, size_y, size_z)
    if slot_dir == "X":
        block = block.faces("<Z").workplane().rect(size_x + 0.01, slot_width).cutBlind(-slot_depth)
    else:
        block = block.faces("<Z").workplane().rect(slot_width, size_y + 0.01).cutBlind(-slot_depth)
    return block


def make_z_carriage(size_x: float, size_y: float, size_z: float, slot_depth: float, slot_width: float) -> cq.Workplane:
    block = cq.Workplane("XY").box(size_x, size_y, size_z)
    block = block.faces("<X").workplane().rect(size_y + 0.01, slot_width).cutBlind(-slot_depth)
    return block


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_stage")

    # Base
    base = model.part("base")
    base_plate = cq.Workplane("XY").box(0.70, 0.20, 0.02)
    base.visual(mesh_from_cadquery(base_plate, "base_plate"), origin=Origin(xyz=(0.0, 0.0, 0.01)), name="base_plate")

    x_rail = make_rail(0.60, 0.04, 0.02, dir="X")
    base.visual(mesh_from_cadquery(x_rail, "x_rail"), origin=Origin(xyz=(0.0, 0.0, 0.03)), name="x_rail")

    # X Carriage
    x_carriage = model.part("x_carriage")
    x_carriage_block = make_xy_carriage(0.12, 0.14, 0.03, 0.015, 0.042, slot_dir="X")
    x_carriage.visual(mesh_from_cadquery(x_carriage_block, "x_carriage_block"), origin=Origin(xyz=(0.0, 0.0, 0.04)), name="x_carriage_block")

    y_rail = make_rail(0.50, 0.04, 0.02, dir="Y")
    x_carriage.visual(mesh_from_cadquery(y_rail, "y_rail"), origin=Origin(xyz=(0.0, 0.0, 0.065)), name="y_rail")

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.20, upper=0.20),
    )

    # Y Carriage
    y_carriage = model.part("y_carriage")
    y_carriage_block = make_xy_carriage(0.14, 0.12, 0.03, 0.015, 0.042, slot_dir="Y")
    y_carriage.visual(mesh_from_cadquery(y_carriage_block, "y_carriage_block"), origin=Origin(xyz=(0.0, 0.0, 0.075)), name="y_carriage_block")

    z_column = cq.Workplane("XY").box(0.06, 0.12, 0.30)
    y_carriage.visual(mesh_from_cadquery(z_column, "z_column"), origin=Origin(xyz=(-0.04, 0.0, 0.240)), name="z_column")

    z_rail = make_rail(0.30, 0.04, 0.02, dir="Z")
    y_carriage.visual(mesh_from_cadquery(z_rail, "z_rail"), origin=Origin(xyz=(0.0, 0.0, 0.240)), name="z_rail")

    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    # Z Carriage
    z_carriage = model.part("z_carriage")
    z_carriage_block = make_z_carriage(0.03, 0.14, 0.10, 0.015, 0.042)
    z_carriage.visual(mesh_from_cadquery(z_carriage_block, "z_carriage_block"), origin=Origin(xyz=(0.01, 0.0, 0.240)), name="z_carriage_block")

    tool_plate = cq.Workplane("XY").box(0.12, 0.14, 0.02)
    z_carriage.visual(mesh_from_cadquery(tool_plate, "tool_plate"), origin=Origin(xyz=(0.085, 0.0, 0.240)), name="tool_plate")

    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.10, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")

    # Allow overlap for the nested proxy fit of carriage blocks sliding over rails
    ctx.allow_overlap(
        x_carriage, base, elem_a="x_carriage_block", elem_b="x_rail",
        reason="The carriage block is intentionally represented as sliding over the rail proxy."
    )
    ctx.allow_overlap(
        y_carriage, x_carriage, elem_a="y_carriage_block", elem_b="y_rail",
        reason="The carriage block is intentionally represented as sliding over the rail proxy."
    )
    ctx.allow_overlap(
        z_carriage, y_carriage, elem_a="z_carriage_block", elem_b="z_rail",
        reason="The carriage block is intentionally represented as sliding over the rail proxy."
    )

    # Check contact on the sliding surfaces
    ctx.expect_within(base, x_carriage, axes="yz", inner_elem="x_rail", outer_elem="x_carriage_block", margin=0.01)
    ctx.expect_within(x_carriage, y_carriage, axes="xz", inner_elem="y_rail", outer_elem="y_carriage_block", margin=0.01)
    ctx.expect_within(y_carriage, z_carriage, axes="xy", inner_elem="z_rail", outer_elem="z_carriage_block", margin=0.01)

    # Check overlaps to ensure carriages stay on rails
    ctx.expect_overlap(x_carriage, base, axes="y", min_overlap=0.03, elem_a="x_carriage_block", elem_b="x_rail")
    ctx.expect_overlap(y_carriage, x_carriage, axes="x", min_overlap=0.03, elem_a="y_carriage_block", elem_b="y_rail")
    ctx.expect_overlap(z_carriage, y_carriage, axes="y", min_overlap=0.03, elem_a="z_carriage_block", elem_b="z_rail")

    # Pose checks to verify movement
    with ctx.pose(x_axis=0.20):
        ctx.expect_overlap(x_carriage, base, axes="xy", min_overlap=0.03, elem_a="x_carriage_block", elem_b="x_rail")

    with ctx.pose(y_axis=0.15):
        ctx.expect_overlap(y_carriage, x_carriage, axes="xy", min_overlap=0.03, elem_a="y_carriage_block", elem_b="y_rail")

    with ctx.pose(z_axis=0.10):
        ctx.expect_overlap(z_carriage, y_carriage, axes="yz", min_overlap=0.03, elem_a="z_carriage_block", elem_b="z_rail")

    return ctx.report()


object_model = build_object_model()
