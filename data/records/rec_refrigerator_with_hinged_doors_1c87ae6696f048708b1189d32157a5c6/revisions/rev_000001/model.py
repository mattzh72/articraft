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

def build_cabinet_mesh() -> cq.Workplane:
    cabinet = cq.Workplane("XY").box(0.9, 0.8, 1.8, centered=(True, True, False))
    # fridge cut: Z from 0.65 to 1.75
    fridge_cut = cq.Workplane("XY").box(0.8, 0.75, 1.1, centered=(True, True, False)).translate((0, 0.025, 0.65))
    # freezer cut: Z from 0.05 to 0.60
    freezer_cut = cq.Workplane("XY").box(0.8, 0.75, 0.55, centered=(True, True, False)).translate((0, 0.025, 0.05))
    return cabinet.cut(fridge_cut).cut(freezer_cut)

def build_drawer_bin_mesh() -> cq.Workplane:
    # Build a hollow box centered at origin
    bin_outer = cq.Workplane("XY").box(0.76, 0.7, 0.45, centered=(True, True, True))
    bin_inner = cq.Workplane("XY").box(0.72, 0.66, 0.45, centered=(True, True, True)).translate((0, 0, 0.02))
    return bin_outer.cut(bin_inner)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_door_refrigerator")

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(build_cabinet_mesh(), "cabinet_shell"),
        name="shell"
    )

    # LEFT DOOR
    left_door = model.part("left_door")
    left_door.visual(
        Box((0.448, 0.05, 1.1)),
        origin=Origin(xyz=(0.224, 0.025, 0.0)),
        name="panel"
    )
    left_door.visual(
        Box((0.02, 0.03, 0.5)),
        origin=Origin(xyz=(0.38, 0.065, 0.0)),
        name="handle"
    )
    # Hinges bridging to cabinet
    left_door.visual(
        Box((0.02, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, -0.01, 0.5)),
        name="upper_hinge"
    )
    left_door.visual(
        Box((0.02, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, -0.01, -0.5)),
        name="lower_hinge"
    )

    # RIGHT DOOR
    right_door = model.part("right_door")
    right_door.visual(
        Box((0.448, 0.05, 1.1)),
        origin=Origin(xyz=(-0.224, 0.025, 0.0)),
        name="panel"
    )
    right_door.visual(
        Box((0.02, 0.03, 0.5)),
        origin=Origin(xyz=(-0.38, 0.065, 0.0)),
        name="handle"
    )
    # Hinges bridging to cabinet
    right_door.visual(
        Box((0.02, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, -0.01, 0.5)),
        name="upper_hinge"
    )
    right_door.visual(
        Box((0.02, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, -0.01, -0.5)),
        name="lower_hinge"
    )

    # FREEZER DRAWER
    freezer_drawer = model.part("freezer_drawer")
    freezer_drawer.visual(
        Box((0.898, 0.05, 0.55)),
        origin=Origin(xyz=(0, 0.025, 0.0)),
        name="front_panel"
    )
    freezer_drawer.visual(
        Box((0.6, 0.03, 0.02)),
        origin=Origin(xyz=(0, 0.065, 0.15)),
        name="handle"
    )
    freezer_drawer.visual(
        mesh_from_cadquery(build_drawer_bin_mesh(), "drawer_bin"),
        origin=Origin(xyz=(0, -0.35, 0.0)),
        name="bin"
    )
    # Rails bridging to cabinet walls
    freezer_drawer.visual(
        Box((0.02, 0.6, 0.05)),
        origin=Origin(xyz=(0.395, -0.3, 0.0)),
        name="right_rail"
    )
    freezer_drawer.visual(
        Box((0.02, 0.6, 0.05)),
        origin=Origin(xyz=(-0.395, -0.3, 0.0)),
        name="left_rail"
    )

    # Articulations
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-0.45, 0.401, 1.2)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.0),
    )

    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(0.45, 0.401, 1.2)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.0),
    )

    model.articulation(
        "freezer_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=freezer_drawer,
        origin=Origin(xyz=(0.0, 0.401, 0.325)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.5),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    drawer = object_model.get_part("freezer_drawer")
    
    # Allowances for hinges and rails
    ctx.allow_overlap(cabinet, left_door, elem_a="shell", elem_b="upper_hinge", reason="Hinge mounts to cabinet")
    ctx.allow_overlap(cabinet, left_door, elem_a="shell", elem_b="lower_hinge", reason="Hinge mounts to cabinet")
    ctx.allow_overlap(cabinet, right_door, elem_a="shell", elem_b="upper_hinge", reason="Hinge mounts to cabinet")
    ctx.allow_overlap(cabinet, right_door, elem_a="shell", elem_b="lower_hinge", reason="Hinge mounts to cabinet")
    ctx.allow_overlap(cabinet, drawer, elem_a="shell", elem_b="left_rail", reason="Rails mount inside cabinet")
    ctx.allow_overlap(cabinet, drawer, elem_a="shell", elem_b="right_rail", reason="Rails mount inside cabinet")
    
    # Rest pose checks
    ctx.expect_gap(left_door, cabinet, axis="y", positive_elem="panel", min_gap=0.001, max_gap=0.003, name="left door seal gap")
    ctx.expect_gap(right_door, cabinet, axis="y", positive_elem="panel", min_gap=0.001, max_gap=0.003, name="right door seal gap")
    ctx.expect_gap(drawer, cabinet, axis="y", positive_elem="front_panel", min_gap=0.001, max_gap=0.003, name="drawer seal gap")
    
    ctx.expect_gap(right_door, left_door, axis="x", positive_elem="panel", negative_elem="panel", min_gap=0.003, max_gap=0.005, name="gap between french doors")
    
    # Check that drawer bin is inside the cabinet at rest
    ctx.expect_within(drawer, cabinet, axes="x", inner_elem="bin", name="drawer bin fits in width")
    
    # Posed checks
    slide = object_model.get_articulation("freezer_drawer_slide")
    with ctx.pose({slide: 0.5}):
        ctx.expect_overlap(drawer, cabinet, axes="y", elem_a="bin", min_overlap=0.05, name="drawer bin remains supported when open")

    return ctx.report()

object_model = build_object_model()