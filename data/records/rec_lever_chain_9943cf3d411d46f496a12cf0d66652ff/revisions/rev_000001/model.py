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


LINK_WIDTH = 0.038
LINK_THICKNESS = 0.008
COLLAR_THICKNESS = 0.002
LOWER_LAYER_Z = 0.000
UPPER_LAYER_Z = 0.0115

LINK_LENGTHS = (0.180, 0.155, 0.130)


def _rounded_link_shape(
    length: float,
    *,
    tab_length: float = 0.0,
    tab_width: float = 0.022,
) -> cq.Workplane:
    """A flat lever bar with pivot eyes, through holes, collars, and optional end tab."""

    radius = LINK_WIDTH * 0.5
    pivot_hole_radius = 0.009
    collar_radius = 0.017

    body = (
        cq.Workplane("XY")
        .moveTo(0.0, -radius)
        .lineTo(length, -radius)
        .threePointArc((length + radius, 0.0), (length, radius))
        .lineTo(0.0, radius)
        .threePointArc((-radius, 0.0), (0.0, -radius))
        .close()
        .extrude(LINK_THICKNESS)
    )

    # Raised annular collars around the two bearing holes make each joint read as a bushing.
    for x in (0.0, length):
        top_collar = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(collar_radius)
            .extrude(COLLAR_THICKNESS)
            .translate((0.0, 0.0, LINK_THICKNESS))
        )
        bottom_collar = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(collar_radius)
            .extrude(COLLAR_THICKNESS)
            .translate((0.0, 0.0, -COLLAR_THICKNESS))
        )
        body = body.union(top_collar).union(bottom_collar)

    # A lightening slot leaves two rails between the pivot eyes without breaking the bar.
    if length > 0.115:
        slot_cutter = (
            cq.Workplane("XY")
            .center(length * 0.5, 0.0)
            .slot2D(length - 0.076, 0.010)
            .extrude(LINK_THICKNESS + 2.0 * COLLAR_THICKNESS + 0.008)
            .translate((0.0, 0.0, -COLLAR_THICKNESS - 0.004))
        )
        body = body.cut(slot_cutter)

    if tab_length > 0.0:
        tab_end_x = length + tab_length
        tab = (
            cq.Workplane("XY")
            .moveTo(length - 0.010, -tab_width * 0.5)
            .lineTo(tab_end_x, -tab_width * 0.5)
            .threePointArc((tab_end_x + tab_width * 0.5, 0.0), (tab_end_x, tab_width * 0.5))
            .lineTo(length - 0.010, tab_width * 0.5)
            .close()
            .extrude(LINK_THICKNESS)
        )
        body = body.union(tab)
        tab_hole = (
            cq.Workplane("XY")
            .center(tab_end_x, 0.0)
            .circle(0.004)
            .extrude(LINK_THICKNESS + 0.010)
            .translate((0.0, 0.0, -0.005))
        )
        body = body.cut(tab_hole)

    # Drill the pivot eyes through the collars and plate.
    for x in (0.0, length):
        hole = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(pivot_hole_radius)
            .extrude(LINK_THICKNESS + 2.0 * COLLAR_THICKNESS + 0.010)
            .translate((0.0, 0.0, -COLLAR_THICKNESS - 0.005))
        )
        body = body.cut(hole)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_link_lever_chain")

    base_mat = model.material("blackened_base", rgba=(0.08, 0.09, 0.10, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    link_mat = model.material("blue_steel_links", rgba=(0.12, 0.30, 0.62, 1.0))
    upper_link_mat = model.material("zinc_upper_link", rgba=(0.68, 0.70, 0.66, 1.0))
    tab_mat = model.material("safety_orange_tab", rgba=(0.95, 0.42, 0.08, 1.0))
    bolt_mat = model.material("dark_bolt_heads", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base_lug")
    base.visual(
        Box((0.160, 0.100, 0.012)),
        origin=Origin(xyz=(-0.045, 0.0, -0.045)),
        material=base_mat,
        name="mount_foot",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=base_mat,
        name="round_lug",
    )
    base.visual(
        Cylinder(radius=0.0093, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=steel_mat,
        name="base_pin",
    )
    base.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=steel_mat,
        name="pin_head",
    )
    for i, y in enumerate((-0.032, 0.032)):
        base.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(-0.095, y, -0.0375)),
            material=bolt_mat,
            name=f"foot_bolt_{i}",
        )

    lever_0 = model.part("lever_0")
    lever_0.visual(
        mesh_from_cadquery(_rounded_link_shape(LINK_LENGTHS[0]), "lever_0_plate"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_LAYER_Z - LINK_THICKNESS * 0.5)),
        material=link_mat,
        name="lever_0_plate",
    )

    lever_1 = model.part("lever_1")
    lever_1.visual(
        mesh_from_cadquery(_rounded_link_shape(LINK_LENGTHS[1]), "lever_1_plate"),
        origin=Origin(xyz=(0.0, 0.0, UPPER_LAYER_Z - LINK_THICKNESS * 0.5)),
        material=upper_link_mat,
        name="lever_1_plate",
    )

    lever_2 = model.part("lever_2")
    lever_2.visual(
        mesh_from_cadquery(
            _rounded_link_shape(LINK_LENGTHS[2], tab_length=0.046, tab_width=0.022),
            "lever_2_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_LAYER_Z - LINK_THICKNESS * 0.5)),
        material=link_mat,
        name="lever_2_plate",
    )
    lever_2.visual(
        Box((0.038, 0.018, 0.004)),
        origin=Origin(xyz=(LINK_LENGTHS[2] + 0.024, 0.0, LOWER_LAYER_Z + 0.006)),
        material=tab_mat,
        name="end_tab",
    )

    model.articulation(
        "base_to_lever_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "lever_0_to_lever_1",
        ArticulationType.REVOLUTE,
        parent=lever_0,
        child=lever_1,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0, lower=-2.10, upper=2.10),
    )
    model.articulation(
        "lever_1_to_lever_2",
        ArticulationType.REVOLUTE,
        parent=lever_1,
        child=lever_2,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-2.00, upper=2.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "exactly three revolute pivots",
        len(revolute_joints) == 3,
        details=f"found {len(revolute_joints)} revolute joints",
    )
    ctx.check(
        "all pivot axes are parallel",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0) for joint in revolute_joints),
        details=str([joint.axis for joint in revolute_joints]),
    )

    base = object_model.get_part("base_lug")
    lever_0 = object_model.get_part("lever_0")
    lever_1 = object_model.get_part("lever_1")
    lever_2 = object_model.get_part("lever_2")
    mid_joint = object_model.get_articulation("lever_0_to_lever_1")

    ctx.allow_overlap(
        base,
        lever_0,
        elem_a="base_pin",
        elem_b="lever_0_plate",
        reason="The first pivot pin is intentionally modeled with a tiny press-fit into the lever eye.",
    )
    ctx.allow_overlap(
        base,
        lever_0,
        elem_a="round_lug",
        elem_b="lever_0_plate",
        reason="The base lug slightly seats against the underside of the first lever collar.",
    )
    ctx.allow_overlap(
        lever_0,
        lever_1,
        elem_a="lever_0_plate",
        elem_b="lever_1_plate",
        reason="Stacked bearing collars at the first inter-link pivot have a tiny modeled preload.",
    )
    ctx.allow_overlap(
        lever_1,
        lever_2,
        elem_a="lever_1_plate",
        elem_b="lever_2_plate",
        reason="Stacked bearing collars at the second inter-link pivot have a tiny modeled preload.",
    )

    ctx.expect_within(
        base,
        lever_0,
        axes="xy",
        inner_elem="base_pin",
        outer_elem="lever_0_plate",
        margin=0.0,
        name="base pin passes through first pivot eye",
    )
    ctx.expect_overlap(
        base,
        lever_0,
        axes="z",
        elem_a="base_pin",
        elem_b="lever_0_plate",
        min_overlap=0.006,
        name="base pivot pin spans the first lever eye thickness",
    )
    ctx.expect_gap(
        lever_1,
        lever_0,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="lever_1_plate",
        negative_elem="lever_0_plate",
        name="adjacent first and second levers are vertically clearanced",
    )
    ctx.expect_gap(
        lever_1,
        lever_2,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="lever_1_plate",
        negative_elem="lever_2_plate",
        name="adjacent second and third levers are vertically clearanced",
    )
    ctx.expect_overlap(
        lever_0,
        lever_1,
        axes="xy",
        elem_a="lever_0_plate",
        elem_b="lever_1_plate",
        min_overlap=0.025,
        name="first inter-link pivot eyes stay coaxially overlapped",
    )
    ctx.expect_overlap(
        lever_1,
        lever_2,
        axes="xy",
        elem_a="lever_1_plate",
        elem_b="lever_2_plate",
        min_overlap=0.025,
        name="second inter-link pivot eyes stay coaxially overlapped",
    )

    rest_tip = ctx.part_element_world_aabb(lever_2, elem="end_tab")
    with ctx.pose({mid_joint: 0.70}):
        swung_tip = ctx.part_element_world_aabb(lever_2, elem="end_tab")

    rest_center_y = None if rest_tip is None else (rest_tip[0][1] + rest_tip[1][1]) * 0.5
    swung_center_y = None if swung_tip is None else (swung_tip[0][1] + swung_tip[1][1]) * 0.5
    ctx.check(
        "middle pivot swings the free tab in the linkage plane",
        rest_center_y is not None
        and swung_center_y is not None
        and swung_center_y > rest_center_y + 0.08,
        details=f"rest_y={rest_center_y}, swung_y={swung_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
