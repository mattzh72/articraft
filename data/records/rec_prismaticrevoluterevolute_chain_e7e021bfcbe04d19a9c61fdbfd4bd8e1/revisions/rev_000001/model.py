from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


SHOULDER_LENGTH = 0.42
OUTER_LENGTH = 0.27
SLIDE_TRAVEL = 0.42


def _carriage_saddle_shape() -> cq.Workplane:
    """Compact saddle block with clearanced linear-bearing bores."""
    body = cq.Workplane("XY").box(0.20, 0.32, 0.10).translate((0.0, 0.0, -0.13))
    for rail_y in (-0.11, 0.11):
        bore = (
            cq.Workplane("YZ")
            .center(rail_y, -0.13)
            .circle(0.028)
            .extrude(0.26, both=True)
        )
        body = body.cut(bore)
    return body


def _outer_link_shape(length: float) -> cq.Workplane:
    """Single central link plate with round end bosses and pin holes."""
    thickness = 0.012
    body = (
        cq.Workplane("XZ")
        .center(length / 2.0, 0.0)
        .rect(length, 0.038)
        .extrude(thickness, both=True)
    )
    for pivot_x in (0.0, length):
        boss = cq.Workplane("XZ").center(pivot_x, 0.0).circle(0.034).extrude(thickness, both=True)
        hole = cq.Workplane("XZ").center(pivot_x, 0.0).circle(0.012).extrude(0.060, both=True)
        body = body.union(boss).cut(hole)
    return body


def _shoulder_link_shape(length: float) -> cq.Workplane:
    """Shoulder link with a central carriage eye and a forked elbow clevis."""
    central = (
        cq.Workplane("XZ")
        .center((length - 0.075) / 2.0, 0.0)
        .rect(length - 0.075, 0.048)
        .extrude(0.040, both=True)
    )
    start_boss = cq.Workplane("XZ").center(0.0, 0.0).circle(0.040).extrude(0.040, both=True)
    body = central.union(start_boss)

    # Side bridges tie the central plate to both clevis ears while leaving the
    # middle slot clear for the outer link's tongue.
    for bridge_y in (-0.047, 0.047):
        side_bridge = (
            cq.Workplane("XY")
            .box(0.060, 0.018, 0.044)
            .translate((length - 0.070, bridge_y, 0.0))
        )
        body = body.union(side_bridge)

    for lug_y in (-0.044, 0.044):
        lug_bar = (
            cq.Workplane("XY")
            .box(0.110, 0.018, 0.048)
            .translate((length - 0.035, lug_y, 0.0))
        )
        lug_boss = (
            cq.Workplane("XZ")
            .center(length, 0.0)
            .circle(0.038)
            .extrude(0.018, both=True)
            .translate((0.0, lug_y, 0.0))
        )
        body = body.union(lug_bar).union(lug_boss)

    shoulder_hole = cq.Workplane("XZ").center(0.0, 0.0).circle(0.014).extrude(0.070, both=True)
    elbow_hole = cq.Workplane("XZ").center(length, 0.0).circle(0.014).extrude(0.140, both=True)
    return body.cut(shoulder_hole).cut(elbow_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_carriage_elbow")

    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.08, 0.095, 0.11, 1.0))
    rail_steel = model.material("brushed_rail_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    blue_carriage = model.material("blue_carriage_casting", rgba=(0.05, 0.18, 0.34, 1.0))
    yellow_link = model.material("safety_yellow_link", rgba=(0.95, 0.68, 0.08, 1.0))
    orange_link = model.material("orange_outer_link", rgba=(0.92, 0.36, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    stop_red = model.material("red_end_stop", rgba=(0.75, 0.05, 0.035, 1.0))

    guide = model.part("guide")
    guide.visual(
        Box((0.96, 0.36, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="base_plate",
    )
    for stop_x, stop_name in [(-0.43, "end_support_0"), (0.43, "end_support_1")]:
        guide.visual(
            Box((0.075, 0.32, 0.140)),
            origin=Origin(xyz=(stop_x, 0.0, 0.125)),
            material=dark_steel,
            name=stop_name,
        )
        guide.visual(
            Box((0.025, 0.24, 0.045)),
            origin=Origin(xyz=(stop_x * 0.97, 0.0, 0.2175)),
            material=stop_red,
            name=f"travel_bumper_{0 if stop_x < 0 else 1}",
        )
    for rail_y, rail_name in [(-0.11, "rail_0"), (0.11, "rail_1")]:
        guide.visual(
            Cylinder(radius=0.018, length=0.84),
            origin=Origin(xyz=(0.0, rail_y, 0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_steel,
            name=rail_name,
        )
    guide.visual(
        Box((0.82, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=rail_steel,
        name="center_way",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_saddle_shape(), "carriage_saddle"),
        material=blue_carriage,
        name="carriage_saddle",
    )
    carriage.visual(
        Box((0.22, 0.24, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.0675)),
        material=blue_carriage,
        name="top_deck",
    )
    carriage.visual(
        Box((0.16, 0.046, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, -0.207)),
        material=black_rubber,
        name="center_wear_shoe",
    )
    for cheek_y, cheek_name in [(-0.062, "pivot_cheek_0"), (0.062, "pivot_cheek_1")]:
        carriage.visual(
            Box((0.090, 0.024, 0.200)),
            origin=Origin(xyz=(0.0, cheek_y, -0.050)),
            material=blue_carriage,
            name=cheek_name,
        )
        carriage.visual(
            Cylinder(radius=0.048, length=0.020),
            origin=Origin(xyz=(0.0, cheek_y * 1.16, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rail_steel,
            name=f"shoulder_boss_{0 if cheek_y < 0 else 1}",
        )
    carriage.visual(
        Cylinder(radius=0.016, length=0.170),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="shoulder_pin",
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(_shoulder_link_shape(SHOULDER_LENGTH), "shoulder_link"),
        material=yellow_link,
        name="shoulder_bar",
    )
    shoulder_link.visual(
        Cylinder(radius=0.016, length=0.130),
        origin=Origin(xyz=(SHOULDER_LENGTH, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="elbow_pin",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_cadquery(_outer_link_shape(OUTER_LENGTH), "outer_link"),
        material=orange_link,
        name="outer_bar",
    )
    outer_link.visual(
        Box((0.074, 0.060, 0.048)),
        origin=Origin(xyz=(OUTER_LENGTH + 0.040, 0.0, 0.0)),
        material=orange_link,
        name="tool_mount",
    )
    outer_link.visual(
        Box((0.026, 0.070, 0.058)),
        origin=Origin(xyz=(OUTER_LENGTH + 0.090, 0.0, 0.0)),
        material=black_rubber,
        name="service_pad",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(-0.24, 0.0, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder_link,
        origin=Origin(rpy=(0.0, -0.34, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.65, upper=1.10),
    )
    model.articulation(
        "shoulder_to_outer",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=outer_link,
        origin=Origin(xyz=(SHOULDER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.25, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    shoulder_link = object_model.get_part("shoulder_link")
    outer_link = object_model.get_part("outer_link")
    slide = object_model.get_articulation("guide_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_shoulder")
    elbow = object_model.get_articulation("shoulder_to_outer")

    ctx.allow_overlap(
        carriage,
        shoulder_link,
        elem_a="shoulder_pin",
        elem_b="shoulder_bar",
        reason="The steel shoulder pin is intentionally captured through the shoulder link's pivot bushing.",
    )
    ctx.allow_overlap(
        shoulder_link,
        outer_link,
        elem_a="elbow_pin",
        elem_b="outer_bar",
        reason="The elbow pin is intentionally captured through the outer link's pivot bushing.",
    )

    ctx.expect_gap(
        carriage,
        guide,
        axis="z",
        positive_elem="carriage_saddle",
        negative_elem="base_plate",
        min_gap=0.060,
        max_gap=0.090,
        name="carriage saddle rides above base plate",
    )
    for rail_elem in ("rail_0", "rail_1"):
        ctx.expect_within(
            guide,
            carriage,
            axes="yz",
            inner_elem=rail_elem,
            outer_elem="carriage_saddle",
            margin=0.0,
            name=f"{rail_elem} passes through saddle envelope",
        )
        ctx.expect_overlap(
            guide,
            carriage,
            axes="x",
            elem_a=rail_elem,
            elem_b="carriage_saddle",
            min_overlap=0.16,
            name=f"{rail_elem} remains engaged in carriage saddle",
        )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            guide,
            carriage,
            axes="x",
            elem_a="rail_0",
            elem_b="carriage_saddle",
            min_overlap=0.16,
            name="extended carriage still retained on guide rail",
        )

    ctx.check(
        "prismatic carriage translates along guide",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + SLIDE_TRAVEL - 0.005
        and abs(extended_carriage_pos[1] - rest_carriage_pos[1]) < 1e-6
        and abs(extended_carriage_pos[2] - rest_carriage_pos[2]) < 1e-6,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    ctx.expect_origin_distance(
        shoulder_link,
        carriage,
        axes="xyz",
        max_dist=0.001,
        name="shoulder pivot sits on carriage pivot",
    )
    ctx.expect_overlap(
        carriage,
        shoulder_link,
        axes="yz",
        elem_a="shoulder_pin",
        elem_b="shoulder_bar",
        min_overlap=0.028,
        name="shoulder pin is captured through link bushing",
    )
    ctx.expect_origin_distance(
        outer_link,
        shoulder_link,
        axes="xz",
        min_dist=SHOULDER_LENGTH - 0.004,
        max_dist=SHOULDER_LENGTH + 0.004,
        name="elbow pivot sits at shoulder link end",
    )
    ctx.expect_overlap(
        shoulder_link,
        outer_link,
        axes="yz",
        elem_a="elbow_pin",
        elem_b="outer_bar",
        min_overlap=0.024,
        name="elbow pin is captured through outer link bushing",
    )
    ctx.check(
        "outer link is visibly shorter than shoulder link",
        OUTER_LENGTH < SHOULDER_LENGTH * 0.75,
        details=f"shoulder={SHOULDER_LENGTH}, outer={OUTER_LENGTH}",
    )

    shoulder_rest_aabb = ctx.part_element_world_aabb(shoulder_link, elem="shoulder_bar")
    pad_rest_aabb = ctx.part_element_world_aabb(outer_link, elem="service_pad")
    with ctx.pose({shoulder: 0.80}):
        shoulder_raised_aabb = ctx.part_element_world_aabb(shoulder_link, elem="shoulder_bar")
    with ctx.pose({elbow: 1.00}):
        pad_folded_aabb = ctx.part_element_world_aabb(outer_link, elem="service_pad")

    ctx.check(
        "shoulder revolute joint raises the shoulder link",
        shoulder_rest_aabb is not None
        and shoulder_raised_aabb is not None
        and shoulder_raised_aabb[1][2] > shoulder_rest_aabb[1][2] + 0.10,
        details=f"rest={shoulder_rest_aabb}, raised={shoulder_raised_aabb}",
    )
    ctx.check(
        "elbow revolute joint swings the outer service pad",
        pad_rest_aabb is not None
        and pad_folded_aabb is not None
        and pad_folded_aabb[1][2] > pad_rest_aabb[1][2] + 0.06,
        details=f"rest={pad_rest_aabb}, folded={pad_folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
