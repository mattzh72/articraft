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


SHAFT_AXIS_Z = 0.105


def _annular_disc(inner_radius: float, outer_radius: float, thickness: float) -> cq.Workplane:
    """A centered annular disk whose local axis is +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )


def _frame_body() -> cq.Workplane:
    """Compact base and two bored bearing pedestals, all as one fixed body."""
    base = cq.Workplane("XY").box(0.42, 0.16, 0.024).translate((0.0, 0.0, 0.012))

    rail_y = 0.066
    for y in (-rail_y, rail_y):
        rail = cq.Workplane("XY").box(0.36, 0.014, 0.026).translate((0.0, y, 0.037))
        base = base.union(rail)

    support_x = 0.12
    for x in (-support_x, support_x):
        plate = (
            cq.Workplane("YZ", origin=(x - 0.016, 0.0, SHAFT_AXIS_Z))
            .rect(0.108, 0.165)
            .circle(0.030)
            .extrude(0.032)
        )
        base = base.union(plate)

        foot = cq.Workplane("XY").box(0.066, 0.132, 0.020).translate((x, 0.0, 0.034))
        base = base.union(foot)

    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_shaft_fixture")

    painted_frame = Material("painted_frame", rgba=(0.08, 0.13, 0.18, 1.0))
    bearing_steel = Material("bearing_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.12, 0.12, 0.13, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.02, 0.025, 0.03, 1.0))
    collar_steel = Material("collar_steel", rgba=(0.36, 0.38, 0.38, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_body(), "frame_body", tolerance=0.0008),
        material=painted_frame,
        name="frame_body",
    )

    outer_bearing_mesh = mesh_from_cadquery(
        _annular_disc(0.022, 0.048, 0.012),
        "bearing_outer_race",
        tolerance=0.0005,
    )
    for name, x in (("bearing_outer_0", -0.136), ("bearing_outer_1", 0.136)):
        frame.visual(
            outer_bearing_mesh,
            origin=Origin(xyz=(x, 0.0, SHAFT_AXIS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=name,
        )

    bolt_mesh = Cylinder(radius=0.0045, length=0.004)
    for i, x in enumerate((-0.142, 0.142)):
        for j, (y, z) in enumerate(
            ((-0.034, -0.034), (-0.034, 0.034), (0.034, -0.034), (0.034, 0.034))
        ):
            frame.visual(
                bolt_mesh,
                origin=Origin(
                    xyz=(x, y, SHAFT_AXIS_Z + z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=dark_steel,
                name=f"bearing_bolt_{i}_{j}",
            )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.014, length=0.350),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft_bar",
    )
    for name, x in (("bearing_sleeve_0", -0.120), ("bearing_sleeve_1", 0.120)):
        shaft.visual(
            Cylinder(radius=0.022, length=0.028),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=name,
        )
    shaft.visual(
        Cylinder(radius=0.027, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_steel,
        name="mid_collar",
    )
    shaft.visual(
        Box((0.018, 0.082, 0.082)),
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
        material=black_oxide,
        name="drive_plate",
    )
    for i, (y, z) in enumerate(
        ((-0.026, -0.026), (-0.026, 0.026), (0.026, -0.026), (0.026, 0.026))
    ):
        shaft.visual(
            Cylinder(radius=0.004, length=0.005),
            origin=Origin(xyz=(0.177, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"drive_bolt_{i}",
        )
    shaft.visual(
        Box((0.006, 0.018, 0.010)),
        origin=Origin(xyz=(0.178, 0.036, 0.0)),
        material=collar_steel,
        name="index_tab",
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("shaft_spin")

    ctx.check(
        "single continuous shaft joint",
        len(object_model.articulations) == 1
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_overlap(
        shaft,
        frame,
        axes="x",
        elem_a="shaft_bar",
        elem_b="frame_body",
        min_overlap=0.24,
        name="shaft spans both bearing supports",
    )
    for bearing in ("bearing_outer_0", "bearing_outer_1"):
        ctx.expect_within(
            shaft,
            frame,
            axes="yz",
            inner_elem="shaft_bar",
            outer_elem=bearing,
            margin=0.0,
            name=f"shaft is centered inside {bearing}",
        )

    before = ctx.part_element_world_aabb(shaft, elem="index_tab")
    with ctx.pose({spin: math.pi / 2.0}):
        after = ctx.part_element_world_aabb(shaft, elem="index_tab")

    def _center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    before_center = _center(before)
    after_center = _center(after)
    ctx.check(
        "index tab follows shaft rotation",
        before_center is not None
        and after_center is not None
        and before_center[1] > 0.030
        and after_center[2] > before_center[2] + 0.025
        and abs(after_center[1]) < 0.010,
        details=f"before={before_center}, after={after_center}",
    )

    return ctx.report()


object_model = build_object_model()
