from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = Material("satin_walnut", color=(0.38, 0.19, 0.08, 1.0))
    black = Material("matte_black", color=(0.005, 0.005, 0.004, 1.0))
    rubber = Material("ribbed_black_rubber", color=(0.015, 0.014, 0.012, 1.0))
    graphite = Material("dark_graphite_metal", color=(0.12, 0.12, 0.12, 1.0))
    brushed = Material("brushed_aluminum", color=(0.72, 0.70, 0.66, 1.0))
    chrome = Material("polished_chrome", color=(0.88, 0.86, 0.82, 1.0))
    stylus_red = Material("stylus_red", color=(0.55, 0.04, 0.035, 1.0))

    platter_xy = (-0.07, 0.0)
    platter_axis_z = 0.085
    pivot_xy = (0.205, 0.120)
    pivot_z = 0.140

    plinth = model.part("plinth")
    plinth_shell = (
        cq.Workplane("XY")
        .box(0.62, 0.46, 0.070)
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z")
        .fillet(0.006)
    )
    plinth.visual(
        mesh_from_cadquery(plinth_shell, "rounded_plinth", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=walnut,
        name="rounded_plinth",
    )

    # Thick fixed metal structure around the rotating stage: a broad bearing boss,
    # two side cheeks, and a low bridge all embedded into the wooden plinth.
    plinth.visual(
        Cylinder(radius=0.070, length=0.016),
        origin=Origin(xyz=(platter_xy[0], platter_xy[1], 0.077)),
        material=brushed,
        name="bearing_housing",
    )
    plinth.visual(
        Box((0.255, 0.036, 0.047)),
        origin=Origin(xyz=(platter_xy[0], -0.207, 0.0915)),
        material=graphite,
        name="front_support",
    )
    plinth.visual(
        Box((0.255, 0.036, 0.047)),
        origin=Origin(xyz=(platter_xy[0], 0.207, 0.0915)),
        material=graphite,
        name="rear_support",
    )
    plinth.visual(
        Box((0.205, 0.048, 0.014)),
        origin=Origin(xyz=(platter_xy[0], 0.0, 0.076)),
        material=graphite,
        name="bearing_bridge",
    )

    # Tonearm base off to one side, large enough to read as a real pivot bearing.
    plinth.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(pivot_xy[0], pivot_xy[1], 0.084)),
        material=brushed,
        name="arm_base",
    )
    plinth.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(pivot_xy[0], pivot_xy[1], 0.114)),
        material=chrome,
        name="pivot_tower",
    )
    plinth.visual(
        Box((0.040, 0.075, 0.014)),
        origin=Origin(xyz=(pivot_xy[0] - 0.025, pivot_xy[1] - 0.068, 0.077)),
        material=graphite,
        name="arm_rest",
    )

    # Small feet are slightly let into the underside so the root reads as one
    # physically supported plinth assembly.
    for index, x in enumerate((-0.245, 0.245)):
        for y in (-0.165, 0.165):
            plinth.visual(
                Cylinder(radius=0.028, length=0.018),
                origin=Origin(xyz=(x, y, 0.006)),
                material=black,
                name=f"foot_{index}_{0 if y < 0 else 1}",
            )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.178, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=brushed,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.165, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=rubber,
        name="record_mat",
    )
    for ring_index, (outer_r, inner_r) in enumerate(((0.150, 0.146), (0.121, 0.117), (0.088, 0.084))):
        ring = cq.Workplane("XY").circle(outer_r).circle(inner_r).extrude(0.0010)
        platter.visual(
            mesh_from_cadquery(ring, f"mat_groove_{ring_index}", tolerance=0.001),
            origin=Origin(xyz=(0.0, 0.0, 0.0365)),
            material=black,
            name=f"mat_groove_{ring_index}",
        )
    platter.visual(
        Cylinder(radius=0.008, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material=chrome,
        name="center_spindle",
    )

    model.articulation(
        "platter_bearing",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_xy[0], platter_xy[1], platter_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
        motion_properties=MotionProperties(damping=0.015, friction=0.01),
    )

    tonearm = model.part("tonearm")
    arm_yaw = -2.70
    direction = (math.cos(arm_yaw), math.sin(arm_yaw))
    reverse = (-direction[0], -direction[1])
    arm_length = 0.245

    tonearm.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=arm_length),
        origin=Origin(
            xyz=(direction[0] * arm_length * 0.5, direction[1] * arm_length * 0.5, 0.014),
            rpy=(0.0, math.pi / 2.0, arm_yaw),
        ),
        material=chrome,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.075),
        origin=Origin(
            xyz=(reverse[0] * 0.0375, reverse[1] * 0.0375, 0.012),
            rpy=(0.0, math.pi / 2.0, arm_yaw + math.pi),
        ),
        material=chrome,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.019, length=0.045),
        origin=Origin(
            xyz=(reverse[0] * 0.066, reverse[1] * 0.066, 0.012),
            rpy=(0.0, math.pi / 2.0, arm_yaw + math.pi),
        ),
        material=graphite,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.050, 0.027, 0.008)),
        origin=Origin(
            xyz=(direction[0] * 0.262, direction[1] * 0.262, 0.010),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material=black,
        name="headshell",
    )
    tonearm.visual(
        Cylinder(radius=0.0032, length=0.018),
        origin=Origin(xyz=(direction[0] * 0.276, direction[1] * 0.276, -0.0025)),
        material=stylus_red,
        name="stylus",
    )

    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(pivot_xy[0], pivot_xy[1], pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.0, lower=-0.65, upper=0.45),
        motion_properties=MotionProperties(damping=0.03, friction=0.015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_joint = object_model.get_articulation("platter_bearing")
    tonearm_joint = object_model.get_articulation("tonearm_pivot")

    ctx.check(
        "platter uses continuous vertical bearing",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_joint.articulation_type}, axis={platter_joint.axis}",
    )
    ctx.check(
        "tonearm uses limited vertical pivot",
        tonearm_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_joint.axis) == (0.0, 0.0, 1.0)
        and tonearm_joint.motion_limits is not None
        and tonearm_joint.motion_limits.lower < 0.0
        and tonearm_joint.motion_limits.upper > 0.0,
        details=f"type={tonearm_joint.articulation_type}, axis={tonearm_joint.axis}, limits={tonearm_joint.motion_limits}",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disc",
        negative_elem="bearing_housing",
        min_gap=-0.0001,
        max_gap=0.0005,
        name="platter rides just above bearing housing",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_disc",
        elem_b="bearing_housing",
        min_overlap=0.12,
        name="platter centered over bearing housing",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_collar",
        negative_elem="pivot_tower",
        max_gap=0.001,
        max_penetration=0.0005,
        name="tonearm collar seats on pivot tower",
    )

    with ctx.pose({tonearm_joint: tonearm_joint.motion_limits.upper}):
        raised_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_joint: tonearm_joint.motion_limits.lower}):
        lowered_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
    if raised_aabb is not None and lowered_aabb is not None:
        raised_center = (
            (raised_aabb[0][0] + raised_aabb[1][0]) * 0.5,
            (raised_aabb[0][1] + raised_aabb[1][1]) * 0.5,
        )
        lowered_center = (
            (lowered_aabb[0][0] + lowered_aabb[1][0]) * 0.5,
            (lowered_aabb[0][1] + lowered_aabb[1][1]) * 0.5,
        )
        swing = math.hypot(raised_center[0] - lowered_center[0], raised_center[1] - lowered_center[1])
    else:
        swing = 0.0
    ctx.check(
        "tonearm headshell sweeps across record area",
        swing > 0.18,
        details=f"headshell sweep={swing}",
    )

    return ctx.report()


object_model = build_object_model()
