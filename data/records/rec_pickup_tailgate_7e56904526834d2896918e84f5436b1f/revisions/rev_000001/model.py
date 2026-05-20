from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GATE_WIDTH = 1.46
GATE_HEIGHT = 0.76
GATE_THICKNESS = 0.052
HINGE_X = -0.72
HINGE_Y = -0.145
HINGE_Z = 0.94


def _gate_panel_with_spare_cutout() -> cq.Workplane:
    """Connected gate body with a circular spare-tire clearance cutout."""

    panel = (
        cq.Workplane("XZ")
        .center((GATE_WIDTH + 0.065) / 2.0, 0.0)
        .rect(GATE_WIDTH - 0.065, GATE_HEIGHT)
        .circle(0.235)
        .extrude(GATE_THICKNESS / 2.0, both=True)
    )

    features = [
        cq.Workplane("XY")
        .box(1.36, 0.018, 0.052)
        .translate((GATE_WIDTH / 2.0, -0.031, 0.330)),
        cq.Workplane("XY")
        .box(1.36, 0.018, 0.052)
        .translate((GATE_WIDTH / 2.0, -0.031, -0.330)),
        cq.Workplane("XY").box(0.075, 0.020, 0.69).translate((0.045, -0.032, 0.0)),
        cq.Workplane("XY").box(0.075, 0.020, 0.69).translate((GATE_WIDTH - 0.045, -0.032, 0.0)),
        cq.Workplane("XY").box(0.27, 0.018, 0.18).translate((0.175, -0.036, -0.22)),
        cq.Workplane("XY").box(0.27, 0.018, 0.18).translate((0.175, -0.036, 0.22)),
        cq.Workplane("XZ")
        .center(1.35, 0.020)
        .circle(0.055)
        .extrude(0.012, both=True)
        .translate((0.0, -0.037, 0.0)),
    ]
    body = panel
    for feature in features:
        body = body.union(feature)
    return body


def _cutout_rubber_lip() -> cq.Workplane:
    """Raised annular trim around the spare-tire opening."""

    lip = (
        cq.Workplane("XZ")
        .center(GATE_WIDTH / 2.0, 0.0)
        .circle(0.270)
        .circle(0.225)
        .extrude(0.014, both=True)
    )
    return lip.edges().fillet(0.003)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_swing_pickup_tailgate")

    painted_blue = model.material("weathered_blue_paint", rgba=(0.05, 0.16, 0.28, 1.0))
    bed_paint = model.material("dark_bed_paint", rgba=(0.035, 0.045, 0.055, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    hinge_steel = model.material("blackened_hinge_steel", rgba=(0.015, 0.015, 0.014, 1.0))
    latch_steel = model.material("brushed_latch_steel", rgba=(0.42, 0.43, 0.40, 1.0))

    bed = model.part("bed_corners")
    bed.visual(
        Box((1.78, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, -0.14, 0.39)),
        material=bed_paint,
        name="rear_bumper",
    )
    bed.visual(
        Box((1.72, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, -0.02, 0.48)),
        material=bed_paint,
        name="bed_sill",
    )
    bed.visual(
        Box((0.14, 0.24, 0.92)),
        origin=Origin(xyz=(-0.84, 0.0, 0.86)),
        material=bed_paint,
        name="hinge_post",
    )
    bed.visual(
        Box((0.14, 0.24, 0.92)),
        origin=Origin(xyz=(0.84, 0.0, 0.86)),
        material=bed_paint,
        name="latch_post",
    )
    bed.visual(
        Box((0.14, 0.66, 0.14)),
        origin=Origin(xyz=(-0.84, 0.25, 1.27)),
        material=bed_paint,
        name="hinge_side_rail",
    )
    bed.visual(
        Box((0.14, 0.66, 0.14)),
        origin=Origin(xyz=(0.84, 0.25, 1.27)),
        material=bed_paint,
        name="latch_side_rail",
    )

    # Exposed hinge hardware on the hinge-side bed corner.  The cylinders share
    # the same vertical line used by the tailgate revolute joint.
    for suffix, z in (("lower", HINGE_Z - 0.22), ("upper", HINGE_Z + 0.22)):
        bed.visual(
            Box((0.13, 0.020, 0.22)),
            origin=Origin(xyz=(-0.765, -0.125, z)),
            material=hinge_steel,
            name=f"{suffix}_hinge_leaf",
        )
        bed.visual(
            Cylinder(radius=0.030, length=0.22),
            origin=Origin(xyz=(HINGE_X, -0.145, z)),
            material=hinge_steel,
            name=f"{suffix}_hinge_barrel",
        )
    bed.visual(
        Cylinder(radius=0.012, length=0.74),
        origin=Origin(xyz=(HINGE_X, -0.145, HINGE_Z)),
        material=latch_steel,
        name="hinge_pin",
    )

    bed.visual(
        Box((0.040, 0.035, 0.18)),
        origin=Origin(xyz=(0.810, -0.118, HINGE_Z + 0.02)),
        material=hinge_steel,
        name="latch_keeper",
    )
    bed.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(xyz=(0.810, -0.125, HINGE_Z + 0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_steel,
        name="striker_pin",
    )

    gate = model.part("gate")
    gate.visual(
        mesh_from_cadquery(_gate_panel_with_spare_cutout(), "gate_panel"),
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
        material=painted_blue,
        name="gate_panel",
    )
    gate.visual(
        mesh_from_cadquery(_cutout_rubber_lip(), "cutout_rubber_lip"),
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
        material=black_rubber,
        name="cutout_rubber_lip",
    )
    gate.visual(
        Cylinder(radius=0.028, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="gate_hinge_knuckle",
    )
    gate.visual(
        Box((0.16, 0.030, 0.22)),
        origin=Origin(xyz=(0.080, 0.029, 0.0)),
        material=hinge_steel,
        name="hinge_strap",
    )
    lever = model.part("latch_lever")
    lever.visual(
        Cylinder(radius=0.042, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_steel,
        name="pivot_hub",
    )
    lever.visual(
        Box((0.34, 0.018, 0.040)),
        origin=Origin(xyz=(-0.17, -0.012, 0.0)),
        material=latch_steel,
        name="lever_arm",
    )
    lever.visual(
        Box((0.11, 0.024, 0.070)),
        origin=Origin(xyz=(-0.36, -0.014, 0.0)),
        material=latch_steel,
        name="grip_paddle",
    )
    lever.visual(
        Box((0.12, 0.018, 0.035)),
        origin=Origin(xyz=(0.060, -0.012, 0.0)),
        material=latch_steel,
        name="latch_tongue",
    )

    gate_hinge = model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=gate,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=gate,
        child=lever,
        origin=Origin(xyz=(1.35, 0.036, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.75, upper=0.75),
    )

    # Keep a semantic breadcrumb for tests/readers: positive gate motion swings
    # the latch edge rearward/outward rather than into the pickup bed.
    gate_hinge.meta["opens_outward"] = True
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed_corners")
    gate = object_model.get_part("gate")
    lever = object_model.get_part("latch_lever")
    gate_hinge = object_model.get_articulation("gate_hinge")
    lever_pivot = object_model.get_articulation("lever_pivot")

    ctx.allow_overlap(
        bed,
        gate,
        elem_a="hinge_pin",
        elem_b="gate_hinge_knuckle",
        reason="The hinge pin is intentionally captured through the gate-side hinge knuckle.",
    )
    ctx.expect_within(
        bed,
        gate,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="gate_hinge_knuckle",
        margin=0.001,
        name="hinge pin runs inside gate knuckle",
    )
    ctx.expect_overlap(
        bed,
        gate,
        axes="z",
        elem_a="hinge_pin",
        elem_b="gate_hinge_knuckle",
        min_overlap=0.15,
        name="gate knuckle is retained on vertical hinge pin",
    )

    ctx.expect_gap(
        bed,
        gate,
        axis="x",
        positive_elem="latch_post",
        negative_elem="gate_panel",
        min_gap=0.015,
        max_gap=0.055,
        name="gate closes close to latch-side post",
    )
    ctx.expect_gap(
        gate,
        bed,
        axis="z",
        positive_elem="gate_panel",
        negative_elem="bed_sill",
        min_gap=0.015,
        max_gap=0.055,
        name="gate clears rear bed sill",
    )
    ctx.expect_contact(
        lever,
        gate,
        elem_a="pivot_hub",
        elem_b="gate_panel",
        contact_tol=0.002,
        name="latch lever hub is seated on pivot collar",
    )

    closed_gate_aabb = ctx.part_element_world_aabb(gate, elem="gate_panel")
    closed_lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")
    with ctx.pose({gate_hinge: 1.20}):
        opened_gate_aabb = ctx.part_element_world_aabb(gate, elem="gate_panel")
    with ctx.pose({lever_pivot: 0.65}):
        rotated_lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_arm")

    ctx.check(
        "side hinge swings gate outward",
        closed_gate_aabb is not None
        and opened_gate_aabb is not None
        and opened_gate_aabb[0][1] < closed_gate_aabb[0][1] - 0.35,
        details=f"closed={closed_gate_aabb}, opened={opened_gate_aabb}",
    )
    ctx.check(
        "latch lever rotates on its pivot",
        closed_lever_aabb is not None
        and rotated_lever_aabb is not None
        and rotated_lever_aabb[1][2] > closed_lever_aabb[1][2] + 0.10,
        details=f"closed={closed_lever_aabb}, rotated={rotated_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
