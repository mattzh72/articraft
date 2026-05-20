from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _hole_points(count: int, radius: float, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + 2.0 * math.pi * index / count),
            radius * math.sin(phase + 2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]


def _annular_blank(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    hole_count: int,
    hole_radius: float,
    hole_diameter: float,
    phase: float = 0.0,
) -> cq.Workplane:
    body = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    body = body.faces(">Z").workplane().pushPoints(_hole_points(hole_count, hole_radius, phase)).hole(hole_diameter)
    return body


def _add_radial_tab(
    body: cq.Workplane,
    *,
    outer_radius: float,
    height: float,
    length: float,
    width: float,
    overlap: float = 0.018,
) -> cq.Workplane:
    tab = (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((outer_radius + 0.5 * length - overlap, 0.0, 0.0))
    )
    return body.union(tab)


def _build_base_turntable() -> cq.Workplane:
    height = 0.050
    body = _annular_blank(
        outer_radius=0.300,
        inner_radius=0.048,
        height=height,
        hole_count=24,
        hole_radius=0.258,
        hole_diameter=0.010,
    )
    outer_rim = cq.Workplane("XY").circle(0.300).circle(0.270).extrude(0.064)
    central_hub = cq.Workplane("XY").circle(0.118).circle(0.048).extrude(0.072)
    datum_tab = _add_radial_tab(cq.Workplane("XY"), outer_radius=0.300, height=0.050, length=0.070, width=0.060)
    body = body.union(outer_rim).union(central_hub).union(datum_tab)
    body = body.edges("|Z").fillet(0.003)
    return body


def _build_middle_ring() -> cq.Workplane:
    body = _annular_blank(
        outer_radius=0.230,
        inner_radius=0.060,
        height=0.042,
        hole_count=12,
        hole_radius=0.174,
        hole_diameter=0.014,
        phase=math.pi / 12.0,
    )
    inner_hub = cq.Workplane("XY").circle(0.126).circle(0.060).extrude(0.057)
    outer_land = cq.Workplane("XY").circle(0.230).circle(0.205).extrude(0.052)
    body = body.union(inner_hub).union(outer_land)
    body = _add_radial_tab(body, outer_radius=0.230, height=0.042, length=0.058, width=0.047)
    body = body.edges("|Z").fillet(0.0025)
    return body


def _build_top_flange() -> cq.Workplane:
    body = _annular_blank(
        outer_radius=0.158,
        inner_radius=0.046,
        height=0.038,
        hole_count=6,
        hole_radius=0.111,
        hole_diameter=0.012,
        phase=math.pi / 6.0,
    )
    tool_boss = cq.Workplane("XY").circle(0.092).circle(0.046).extrude(0.056)
    outer_lip = cq.Workplane("XY").circle(0.158).circle(0.140).extrude(0.047)
    body = body.union(tool_boss).union(outer_lip)
    body = _add_radial_tab(body, outer_radius=0.158, height=0.038, length=0.045, width=0.038)
    body = body.edges("|Z").fillet(0.002)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rotary_indexing_stack")

    dark_iron = model.material("dark_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    base_blue = model.material("blued_steel", rgba=(0.12, 0.20, 0.30, 1.0))
    middle_gray = model.material("ground_steel", rgba=(0.48, 0.51, 0.54, 1.0))
    top_gold = model.material("tooling_gold", rgba=(0.86, 0.61, 0.22, 1.0))
    index_red = model.material("index_red", rgba=(0.85, 0.06, 0.03, 1.0))

    stationary_core = model.part("stationary_core")
    stationary_core.visual(
        Cylinder(radius=0.340, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_iron,
        name="floor_plinth",
    )
    stationary_core.visual(
        Cylinder(radius=0.040, length=0.395),
        origin=Origin(xyz=(0.0, 0.0, 0.2525)),
        material=dark_iron,
        name="vertical_shaft",
    )
    stationary_core.visual(
        Cylinder(radius=0.180, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=bearing_steel,
        name="base_bearing_pad",
    )
    stationary_core.visual(
        Cylinder(radius=0.130, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=bearing_steel,
        name="middle_bearing_pad",
    )
    stationary_core.visual(
        Cylinder(radius=0.095, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.276)),
        material=bearing_steel,
        name="top_bearing_pad",
    )
    stationary_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.340, length=0.430),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )

    base_turntable = model.part("base_turntable")
    base_turntable.visual(
        mesh_from_cadquery(_build_base_turntable(), "base_turntable_body"),
        material=base_blue,
        name="base_turntable_body",
    )
    base_turntable.visual(
        Box((0.052, 0.018, 0.006)),
        origin=Origin(xyz=(0.335, 0.0, 0.052)),
        material=index_red,
        name="base_index_mark",
    )
    base_turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.300, length=0.064),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )

    middle_ring = model.part("middle_ring")
    middle_ring.visual(
        mesh_from_cadquery(_build_middle_ring(), "middle_ring_body"),
        material=middle_gray,
        name="middle_ring_body",
    )
    middle_ring.visual(
        Box((0.044, 0.014, 0.005)),
        origin=Origin(xyz=(0.260, 0.0, 0.044)),
        material=index_red,
        name="middle_index_mark",
    )
    middle_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.230, length=0.057),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
    )

    top_flange = model.part("top_flange")
    top_flange.visual(
        mesh_from_cadquery(_build_top_flange(), "top_flange_body"),
        material=top_gold,
        name="top_flange_body",
    )
    top_flange.visual(
        Box((0.034, 0.012, 0.005)),
        origin=Origin(xyz=(0.180, 0.0, 0.039)),
        material=index_red,
        name="top_index_mark",
    )
    top_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.158, length=0.056),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    model.articulation(
        "base_rotation",
        ArticulationType.REVOLUTE,
        parent=stationary_core,
        child=base_turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "middle_rotation",
        ArticulationType.REVOLUTE,
        parent=stationary_core,
        child=middle_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "top_rotation",
        ArticulationType.REVOLUTE,
        parent=stationary_core,
        child=top_flange,
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    core = object_model.get_part("stationary_core")
    base = object_model.get_part("base_turntable")
    middle = object_model.get_part("middle_ring")
    top = object_model.get_part("top_flange")
    joints = [
        object_model.get_articulation("base_rotation"),
        object_model.get_articulation("middle_rotation"),
        object_model.get_articulation("top_rotation"),
    ]

    ctx.check("three_revolute_stage_joints", all(joint is not None for joint in joints), "Expected base, middle, and top revolute joints.")
    for joint in joints:
        if joint is not None:
            ctx.check(
                f"{joint.name}_uses_shared_vertical_axis",
                tuple(round(float(value), 6) for value in joint.axis) == (0.0, 0.0, 1.0)
                and abs(float(joint.origin.xyz[0])) < 1e-6
                and abs(float(joint.origin.xyz[1])) < 1e-6,
                details=f"axis={joint.axis}, origin={joint.origin}",
            )

    ctx.expect_gap(
        base,
        core,
        axis="z",
        positive_elem="base_turntable_body",
        negative_elem="base_bearing_pad",
        max_gap=0.001,
        max_penetration=1e-6,
        name="base stage sits on its bearing pad",
    )
    ctx.expect_gap(
        middle,
        core,
        axis="z",
        positive_elem="middle_ring_body",
        negative_elem="middle_bearing_pad",
        max_gap=0.001,
        max_penetration=1e-6,
        name="middle stage sits on its bearing pad",
    )
    ctx.expect_gap(
        top,
        core,
        axis="z",
        positive_elem="top_flange_body",
        negative_elem="top_bearing_pad",
        max_gap=0.001,
        max_penetration=1e-6,
        name="top tooling flange sits on its bearing pad",
    )

    # The index marks are deliberately off-axis; a posed joint should sweep them
    # around the shared shaft while each part origin remains on the same axis.
    if joints[2] is not None:
        rest_aabb = ctx.part_element_world_aabb(top, elem="top_index_mark")
        with ctx.pose({joints[2]: 0.9}):
            moved_aabb = ctx.part_element_world_aabb(top, elem="top_index_mark")
        ctx.check(
            "top stage datum rotates about shaft",
            rest_aabb is not None
            and moved_aabb is not None
            and abs(float(rest_aabb[0][1]) - float(moved_aabb[0][1])) > 0.05,
            details=f"rest={rest_aabb}, moved={moved_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
