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


def _fork_frame_mesh():
    """One grounded fork frame with coaxial bearing bores through both arms."""

    base_size = (0.40, 0.18, 0.035)
    arm_size = (0.055, 0.12, 0.235)
    clear_span = 0.220
    arm_center_x = clear_span / 2.0 + arm_size[0] / 2.0
    bore_z = 0.170
    bore_radius = 0.024
    boss_radius = 0.046
    boss_length = 0.022

    base = cq.Workplane("XY").box(*base_size).translate((0.0, 0.0, base_size[2] / 2.0))
    left_arm = (
        cq.Workplane("XY")
        .box(*arm_size)
        .translate((-arm_center_x, 0.0, base_size[2] + arm_size[2] / 2.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(*arm_size)
        .translate((arm_center_x, 0.0, base_size[2] + arm_size[2] / 2.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.34, 0.026, 0.155)
        .translate((0.0, 0.047, base_size[2] + 0.155 / 2.0))
    )
    left_foot = (
        cq.Workplane("XY")
        .box(0.095, 0.155, 0.018)
        .translate((-arm_center_x, 0.0, base_size[2] + 0.009))
    )
    right_foot = (
        cq.Workplane("XY")
        .box(0.095, 0.155, 0.018)
        .translate((arm_center_x, 0.0, base_size[2] + 0.009))
    )

    positive_boss = (
        cq.Workplane("YZ")
        .center(0.0, bore_z)
        .circle(boss_radius)
        .extrude(boss_length)
        .translate((arm_center_x + arm_size[0] / 2.0, 0.0, 0.0))
    )
    negative_boss = (
        cq.Workplane("YZ")
        .center(0.0, bore_z)
        .circle(boss_radius)
        .extrude(-boss_length)
        .translate((-arm_center_x - arm_size[0] / 2.0, 0.0, 0.0))
    )

    bolt_heads = cq.Workplane("XY")
    for x in (-0.155, 0.155):
        for y in (-0.062, 0.062):
            bolt_heads = bolt_heads.union(
                cq.Workplane("XY")
                .circle(0.012)
                .extrude(0.007)
                .translate((x, y, base_size[2]))
            )

    frame = (
        base.union(left_arm)
        .union(right_arm)
        .union(rear_bridge)
        .union(left_foot)
        .union(right_foot)
        .union(positive_boss)
        .union(negative_boss)
        .union(bolt_heads)
    )
    bore_cutter = (
        cq.Workplane("YZ")
        .center(0.0, bore_z)
        .circle(bore_radius)
        .extrude(0.55, both=True)
    )
    return frame.cut(bore_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_rotary_shaft")

    painted_steel = model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    blue_output = model.material("blue_output_disk", rgba=(0.10, 0.22, 0.40, 1.0))

    fork_frame = model.part("fork_frame")
    fork_frame.visual(
        mesh_from_cadquery(_fork_frame_mesh(), "fork_frame"),
        material=painted_steel,
        name="frame_body",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.015, length=0.400),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="shaft_bar",
    )
    shaft.visual(
        Cylinder(radius=0.0244, length=0.076),
        origin=Origin(xyz=(-0.1485, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="plain_journal",
    )
    shaft.visual(
        Cylinder(radius=0.0244, length=0.076),
        origin=Origin(xyz=(0.1485, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="output_journal",
    )
    shaft.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.230, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue_output,
        name="output_disk",
    )
    shaft.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.207, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub_collar",
    )
    shaft.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.252, 0.030, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="drive_pin",
    )

    model.articulation(
        "shaft_axis",
        ArticulationType.CONTINUOUS,
        parent=fork_frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork_frame = object_model.get_part("fork_frame")
    shaft = object_model.get_part("shaft")
    joint = object_model.get_articulation("shaft_axis")

    ctx.allow_overlap(
        fork_frame,
        shaft,
        elem_a="frame_body",
        elem_b="plain_journal",
        reason=(
            "The plain-end bearing journal is intentionally seated in the fork bore "
            "with a tiny interference fit so the supported shaft is not floating."
        ),
    )
    ctx.allow_overlap(
        fork_frame,
        shaft,
        elem_a="frame_body",
        elem_b="output_journal",
        reason=(
            "The output-side bearing journal is intentionally seated in the fork bore "
            "with a tiny interference fit so the supported shaft is not floating."
        ),
    )

    ctx.check(
        "single continuous shaft joint",
        joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_within(
        shaft,
        fork_frame,
        axes="yz",
        inner_elem="shaft_bar",
        outer_elem="frame_body",
        margin=0.002,
        name="shaft centered inside fork opening",
    )
    ctx.expect_overlap(
        shaft,
        fork_frame,
        axes="x",
        elem_a="shaft_bar",
        elem_b="frame_body",
        min_overlap=0.30,
        name="shaft spans both fork arms",
    )
    ctx.expect_overlap(
        shaft,
        fork_frame,
        axes="x",
        elem_a="plain_journal",
        elem_b="frame_body",
        min_overlap=0.06,
        name="plain journal retained in support bore",
    )
    ctx.expect_overlap(
        shaft,
        fork_frame,
        axes="x",
        elem_a="output_journal",
        elem_b="frame_body",
        min_overlap=0.06,
        name="output journal retained in support bore",
    )
    ctx.expect_gap(
        shaft,
        fork_frame,
        axis="x",
        positive_elem="output_disk",
        negative_elem="frame_body",
        min_gap=0.010,
        max_gap=0.030,
        name="output disk clears fork frame",
    )

    def _element_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) / 2.0 for index in range(3))

    rest_pin = _element_center(shaft, "drive_pin")
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pin = _element_center(shaft, "drive_pin")
    ctx.check(
        "drive pin orbits with shaft",
        rest_pin is not None
        and turned_pin is not None
        and turned_pin[2] > rest_pin[2] + 0.020
        and turned_pin[1] < rest_pin[1] - 0.020,
        details=f"rest={rest_pin}, turned={turned_pin}",
    )

    return ctx.report()


object_model = build_object_model()
