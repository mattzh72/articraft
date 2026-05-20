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


BASE_TOP_Z = 0.282
ARM_RAIL_TOP_Z = 0.083
SLIDE_HOME_X = 0.21
SLIDE_TRAVEL = 0.38


def _base_plate_mesh():
    """A low oval foot so the whole service arm reads as broad and grounded."""

    return (
        cq.Workplane("XY")
        .ellipse(0.36, 0.24)
        .extrude(0.045)
    )


def _arm_body_mesh():
    """Connected rotating hub and rounded boom for the single sweeping arm."""

    hub = cq.Workplane("XY").circle(0.095).extrude(0.055)
    beam = (
        cq.Workplane("XY")
        .box(0.68, 0.085, 0.055)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.39, 0.0, 0.0475))
    )
    nose = cq.Workplane("XY").circle(0.0425).extrude(0.055).translate((0.73, 0.0, 0.020))
    return hub.union(beam).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_radial_arm")

    model.material("powder_blue", rgba=(0.36, 0.52, 0.63, 1.0))
    model.material("dark_graphite", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("warm_gray", rgba=(0.46, 0.47, 0.45, 1.0))
    model.material("safety_orange", rgba=(0.90, 0.36, 0.12, 1.0))
    model.material("rubber_black", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_mesh(), "oval_base_plate", tolerance=0.0008),
        material="dark_graphite",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material="powder_blue",
        name="support_column",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.2525)),
        material="brushed_steel",
        name="bearing_drum",
    )
    base.visual(
        Cylinder(radius=0.104, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.276)),
        material="dark_graphite",
        name="bearing_cap",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_body_mesh(), "sweeping_arm_body", tolerance=0.0008),
        material="powder_blue",
        name="arm_body",
    )
    arm.visual(
        Box((0.58, 0.055, 0.008)),
        origin=Origin(xyz=(0.44, 0.0, 0.079)),
        material="brushed_steel",
        name="slide_rail",
    )
    arm.visual(
        Box((0.025, 0.075, 0.035)),
        origin=Origin(xyz=(0.755, 0.0, 0.0925)),
        material="rubber_black",
        name="end_stop",
    )

    head = model.part("head")
    head.visual(
        Box((0.112, 0.075, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="rubber_black",
        name="saddle_pad",
    )
    head.visual(
        Box((0.125, 0.130, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material="safety_orange",
        name="carriage_shell",
    )
    head.visual(
        Box((0.105, 0.016, 0.055)),
        origin=Origin(xyz=(0.0, 0.071, 0.0025)),
        material="warm_gray",
        name="side_cheek_0",
    )
    head.visual(
        Box((0.105, 0.016, 0.055)),
        origin=Origin(xyz=(0.0, -0.071, 0.0025)),
        material="warm_gray",
        name="side_cheek_1",
    )
    head.visual(
        Box((0.060, 0.035, 0.080)),
        origin=Origin(xyz=(0.0, -0.086, -0.015)),
        material="warm_gray",
        name="service_pod",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.025),
        origin=Origin(xyz=(0.0, -0.086, -0.066)),
        material="dark_graphite",
        name="tool_coupler",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.1, lower=-2.15, upper=2.15),
    )

    model.articulation(
        "arm_to_head",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=head,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, ARM_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.28, lower=0.0, upper=SLIDE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    sweep = object_model.get_articulation("base_to_arm")
    slide = object_model.get_articulation("arm_to_head")

    ctx.check(
        "one revolute sweep and one prismatic slide",
        sweep.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"sweep={sweep.articulation_type}, slide={slide.articulation_type}",
    )

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="arm_body",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.00001,
        name="rotating hub seats on bearing cap",
    )
    ctx.expect_gap(
        head,
        arm,
        axis="z",
        positive_elem="saddle_pad",
        negative_elem="slide_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="sliding saddle rides on rail",
    )
    ctx.expect_overlap(
        head,
        arm,
        axes="xy",
        elem_a="saddle_pad",
        elem_b="slide_rail",
        min_overlap=0.050,
        name="saddle has a stable rail footprint",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_gap(
            head,
            arm,
            axis="z",
            positive_elem="saddle_pad",
            negative_elem="slide_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended saddle stays on rail",
        )
        ctx.expect_overlap(
            head,
            arm,
            axes="xy",
            elem_a="saddle_pad",
            elem_b="slide_rail",
            min_overlap=0.050,
            name="extended head remains captured by rail",
        )
        extended_head_pos = ctx.part_world_position(head)

    ctx.check(
        "head slides outward along the arm",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[0] > rest_head_pos[0] + SLIDE_TRAVEL * 0.9,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    rest_arm_pos = ctx.part_world_position(arm)
    with ctx.pose({sweep: math.radians(55.0)}):
        swept_head_pos = ctx.part_world_position(head)
    ctx.check(
        "arm sweep carries the head around the vertical pivot",
        rest_arm_pos is not None
        and swept_head_pos is not None
        and abs(swept_head_pos[1]) > 0.12,
        details=f"arm_origin={rest_arm_pos}, swept_head={swept_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
