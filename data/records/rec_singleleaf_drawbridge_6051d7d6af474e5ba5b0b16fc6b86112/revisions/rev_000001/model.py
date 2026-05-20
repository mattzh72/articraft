from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    weathered_concrete = Material("weathered_concrete", rgba=(0.48, 0.46, 0.41, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.10, 0.12, 0.13, 1.0))
    blue_steel = Material("blue_steel", rgba=(0.10, 0.20, 0.32, 1.0))
    asphalt = Material("asphalt", rgba=(0.035, 0.038, 0.04, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.76, 0.10, 1.0))
    worn_metal = Material("worn_metal", rgba=(0.55, 0.56, 0.54, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((1.40, 4.90, 0.25)),
        origin=Origin(xyz=(-0.40, 0.0, 0.125)),
        material=weathered_concrete,
        name="foundation_slab",
    )
    shore_frame.visual(
        Box((0.52, 4.40, 0.90)),
        origin=Origin(xyz=(-0.75, 0.0, 0.68)),
        material=weathered_concrete,
        name="abutment_wall",
    )
    shore_frame.visual(
        Box((1.25, 3.08, 0.12)),
        origin=Origin(xyz=(-0.82, 0.0, 1.08)),
        material=asphalt,
        name="shore_road",
    )
    shore_frame.visual(
        Box((0.34, 3.10, 0.24)),
        origin=Origin(xyz=(-0.34, 0.0, 0.98)),
        material=dark_steel,
        name="shore_sill",
    )

    shore_frame.visual(
        Box((0.82, 0.45, 0.45)),
        origin=Origin(xyz=(0.0, 2.00, 0.425)),
        material=weathered_concrete,
        name="bearing_pedestal_0",
    )
    shore_frame.visual(
        Cylinder(radius=0.42, length=0.34),
        origin=Origin(xyz=(0.0, 2.005, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_cap_0",
    )
    shore_frame.visual(
        Box((0.70, 0.38, 0.18)),
        origin=Origin(xyz=(0.0, 2.05, 0.66)),
        material=dark_steel,
        name="cap_seat_0",
    )
    shore_frame.visual(
        Box((0.82, 0.45, 0.45)),
        origin=Origin(xyz=(0.0, -2.00, 0.425)),
        material=weathered_concrete,
        name="bearing_pedestal_1",
    )
    shore_frame.visual(
        Cylinder(radius=0.42, length=0.34),
        origin=Origin(xyz=(0.0, -2.005, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_cap_1",
    )
    shore_frame.visual(
        Box((0.70, 0.38, 0.18)),
        origin=Origin(xyz=(0.0, -2.05, 0.66)),
        material=dark_steel,
        name="cap_seat_1",
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((7.20, 3.00, 0.28)),
        origin=Origin(xyz=(3.60, 0.0, -0.16)),
        material=blue_steel,
        name="main_shell",
    )
    bridge_leaf.visual(
        Box((0.62, 3.28, 0.46)),
        origin=Origin(xyz=(0.31, 0.0, -0.23)),
        material=dark_steel,
        name="hinge_band",
    )
    bridge_leaf.visual(
        Box((6.90, 2.55, 0.035)),
        origin=Origin(xyz=(3.72, 0.0, -0.002)),
        material=asphalt,
        name="road_surface",
    )
    bridge_leaf.visual(
        Box((6.00, 0.075, 0.018)),
        origin=Origin(xyz=(3.85, 0.0, 0.019)),
        material=safety_yellow,
        name="center_stripe",
    )
    for idx, y in enumerate((1.58, -1.58)):
        bridge_leaf.visual(
            Box((7.10, 0.18, 0.58)),
            origin=Origin(xyz=(3.62, y, -0.24)),
            material=dark_steel,
            name=f"side_girder_{idx}",
        )
    for idx, x in enumerate((1.25, 2.60, 3.95, 5.30, 6.65)):
        bridge_leaf.visual(
            Box((0.14, 2.82, 0.22)),
            origin=Origin(xyz=(x, 0.0, -0.398)),
            material=dark_steel,
            name=f"floor_beam_{idx}",
        )
    bridge_leaf.visual(
        Box((0.16, 3.08, 0.34)),
        origin=Origin(xyz=(7.24, 0.0, -0.22)),
        material=safety_yellow,
        name="free_end_plate",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.13, length=3.56),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_metal,
        name="hinge_shaft",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.33, length=0.20),
        origin=Origin(xyz=(0.0, 1.735, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_metal,
        name="trunnion_0",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.33, length=0.20),
        origin=Origin(xyz=(0.0, -1.735, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_metal,
        name="trunnion_1",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120000.0, velocity=0.18, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check(
        "single revolute leaf joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"articulations={object_model.articulations}, axis={hinge.axis}",
    )
    ctx.check(
        "leaf has separate shell and hinge band",
        leaf.get_visual("main_shell").name == "main_shell"
        and leaf.get_visual("hinge_band").name == "hinge_band",
        details="bridge leaf should be visibly split into a main shell plus hinge-side reinforcement band",
    )
    ctx.expect_gap(
        frame,
        leaf,
        axis="y",
        positive_elem="bearing_cap_0",
        negative_elem="trunnion_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="positive side bearing face contact",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="y",
        positive_elem="trunnion_1",
        negative_elem="bearing_cap_1",
        max_gap=0.001,
        max_penetration=0.00001,
        name="negative side bearing face contact",
    )

    rest_tip = ctx.part_element_world_aabb(leaf, elem="free_end_plate")
    with ctx.pose({hinge: 1.25}):
        raised_tip = ctx.part_element_world_aabb(leaf, elem="free_end_plate")
    ctx.check(
        "leaf raises upward at full open",
        rest_tip is not None and raised_tip is not None and raised_tip[0][2] > rest_tip[1][2] + 5.0,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
