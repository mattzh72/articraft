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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_stage(
    *,
    outer_radius: float,
    rim_width: float,
    hub_inner_radius: float,
    hub_outer_radius: float,
    plate_thickness: float,
    hub_height: float,
    spoke_count: int,
    spoke_width: float,
    tab_length: float,
    tab_width: float,
) -> cq.Workplane:
    """One connected, hollow rotating stage centered on the Z axis."""

    inner_rim_radius = outer_radius - rim_width
    rim = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_rim_radius)
        .extrude(plate_thickness * 0.5, both=True)
    )
    hub = (
        cq.Workplane("XY")
        .circle(hub_outer_radius)
        .circle(hub_inner_radius)
        .extrude(hub_height * 0.5, both=True)
    )

    stage = rim.union(hub)
    spoke_length = inner_rim_radius - hub_outer_radius + 0.012
    spoke_center = hub_outer_radius + 0.5 * (spoke_length - 0.012)
    for i in range(spoke_count):
        angle = 360.0 * i / spoke_count
        spoke = (
            cq.Workplane("XY")
            .box(spoke_length, spoke_width, plate_thickness * 0.78)
            .translate((spoke_center, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        stage = stage.union(spoke)

    # A single raised index lug makes each stage's rotation visually legible.
    tab = (
        cq.Workplane("XY")
        .box(tab_length, tab_width, plate_thickness * 1.12)
        .translate((outer_radius + tab_length * 0.5 - 0.010, 0.0, plate_thickness * 0.08))
    )
    return stage.union(tab).edges("|Z").fillet(0.003)


def _annular_disc(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness * 0.5, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_coaxial_stage_stack")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.015, 0.015, 0.018, 1.0))
    stage_materials = [
        model.material("blue_anodized_stage", rgba=(0.05, 0.22, 0.72, 1.0)),
        model.material("bronze_stage", rgba=(0.76, 0.43, 0.16, 1.0)),
        model.material("green_stage", rgba=(0.05, 0.45, 0.28, 1.0)),
        model.material("red_stage", rgba=(0.70, 0.08, 0.07, 1.0)),
    ]

    support = model.part("top_support")
    support.visual(
        Box((0.92, 0.16, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="top_beam",
    )
    support.visual(
        Box((0.16, 0.52, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="cross_beam",
    )
    support.visual(
        Cylinder(radius=0.075, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=brushed_steel,
        name="upper_boss",
    )
    support.visual(
        Cylinder(radius=0.018, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        material=brushed_steel,
        name="center_shaft",
    )
    support.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.805)),
        material=bearing_black,
        name="shaft_end_nut",
    )

    stage_specs = [
        ("stage_0", -0.210, 0.400, 0.050, 5, 0.024, 0.070, 0.055),
        ("stage_1", -0.370, 0.320, 0.044, 4, 0.022, 0.060, 0.050),
        ("stage_2", -0.530, 0.240, 0.038, 3, 0.020, 0.050, 0.044),
        ("stage_3", -0.680, 0.165, 0.032, 3, 0.018, 0.042, 0.038),
    ]
    for index, (name, z, radius, rim_width, spokes, spoke_width, tab_len, tab_width) in enumerate(stage_specs):
        stage_shape = _annular_stage(
            outer_radius=radius,
            rim_width=rim_width,
            hub_inner_radius=0.030,
            hub_outer_radius=0.062,
            plate_thickness=0.030,
            hub_height=0.058,
            spoke_count=spokes,
            spoke_width=spoke_width,
            tab_length=tab_len,
            tab_width=tab_width,
        )
        stage = model.part(name)
        stage.visual(
            mesh_from_cadquery(stage_shape, f"{name}_structure", tolerance=0.0008, angular_tolerance=0.08),
            material=stage_materials[index],
            name="stage_structure",
        )
        # Thin black bearing races emphasize that the stage collar is captured around the fixed shaft.
        stage.visual(
            mesh_from_cadquery(_annular_disc(0.066, 0.030, 0.010), f"{name}_upper_bearing_race"),
            origin=Origin(xyz=(0.0, 0.0, 0.034)),
            material=bearing_black,
            name="upper_bearing_race",
        )
        stage.visual(
            mesh_from_cadquery(_annular_disc(0.066, 0.030, 0.010), f"{name}_lower_bearing_race"),
            origin=Origin(xyz=(0.0, 0.0, -0.034)),
            material=bearing_black,
            name="lower_bearing_race",
        )
        for ball_index in range(3):
            angle = (2.0 * math.pi * ball_index / 3.0) + index * 0.18
            stage.visual(
                Sphere(radius=0.0060),
                origin=Origin(xyz=(0.024 * math.cos(angle), 0.024 * math.sin(angle), 0.0)),
                material=brushed_steel,
                name=f"bearing_ball_{ball_index}",
            )
        model.articulation(
            f"support_to_{name}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=stage,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    stages = [object_model.get_part(f"stage_{i}") for i in range(4)]
    joints = [object_model.get_articulation(f"support_to_stage_{i}") for i in range(4)]

    for i, (stage, joint) in enumerate(zip(stages, joints)):
        ctx.check(
            f"stage {i} uses a revolute coaxial joint",
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_origin_distance(
            stage,
            support,
            axes="xy",
            max_dist=0.001,
            name=f"stage {i} is centered on the support shaft axis",
        )
        ctx.expect_overlap(
            stage,
            support,
            axes="z",
            elem_a="stage_structure",
            elem_b="center_shaft",
            min_overlap=0.020,
            name=f"center shaft passes through stage {i} height",
        )

    for i in range(3):
        ctx.expect_gap(
            stages[i],
            stages[i + 1],
            axis="z",
            min_gap=0.070,
            name=f"stage {i} hangs above stage {i + 1} with visible air gap",
        )

    rest_aabb = ctx.part_element_world_aabb(stages[0], elem="stage_structure")
    with ctx.pose({joints[0]: math.pi / 2.0, joints[1]: -math.pi / 3.0}):
        turned_aabb = ctx.part_element_world_aabb(stages[0], elem="stage_structure")
        still_aabb = ctx.part_element_world_aabb(stages[2], elem="stage_structure")
    ctx.check(
        "stage 0 index lug rotates independently about the shared axis",
        rest_aabb is not None
        and turned_aabb is not None
        and still_aabb is not None
        and turned_aabb[1][1] > rest_aabb[1][1] + 0.030,
        details=f"rest={rest_aabb}, turned={turned_aabb}, unaffected_stage_2={still_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
