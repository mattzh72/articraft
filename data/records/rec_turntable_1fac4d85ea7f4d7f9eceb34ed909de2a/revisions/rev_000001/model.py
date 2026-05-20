from __future__ import annotations

import math

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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_stage_record_turntable")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.45, 0.26, 0.12, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    record_black = model.material("record_black", rgba=(0.0, 0.0, 0.0, 1.0))
    label_red = model.material("label_red", rgba=(0.72, 0.08, 0.05, 1.0))
    stylus_dark = model.material("stylus_dark", rgba=(0.04, 0.035, 0.032, 1.0))

    platter_x = -0.070
    platter_y = 0.025
    plinth_height = 0.075
    support_top_z = 0.225

    plinth = model.part("plinth")
    plinth_shape = (
        cq.Workplane("XY")
        .box(0.56, 0.42, plinth_height)
        .edges("|Z")
        .fillet(0.035)
    )
    plinth.visual(
        mesh_from_cadquery(plinth_shape, "plinth_shell", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, plinth_height / 2.0)),
        material=warm_wood,
        name="plinth_shell",
    )
    plinth.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(platter_x, platter_y, plinth_height + 0.009)),
        material=satin_black,
        name="support_foot",
    )
    plinth.visual(
        Cylinder(radius=0.054, length=support_top_z - plinth_height),
        origin=Origin(xyz=(platter_x, platter_y, (support_top_z + plinth_height) / 2.0)),
        material=brushed_metal,
        name="center_support",
    )
    plinth.visual(
        Cylinder(radius=0.067, length=0.014),
        origin=Origin(xyz=(platter_x, platter_y, support_top_z - 0.007)),
        material=brushed_metal,
        name="support_cap",
    )

    # Low feet keep the broad plinth visually grounded.
    for index, (x, y) in enumerate(
        ((-0.215, -0.155), (0.215, -0.155), (-0.215, 0.155), (0.215, 0.155))
    ):
        plinth.visual(
            Cylinder(radius=0.028, length=0.018),
            origin=Origin(xyz=(x, y, -0.009)),
            material=dark_rubber,
            name=f"foot_{index}",
        )

    tonearm_pivot_x = 0.205
    tonearm_pivot_y = -0.128
    tonearm_bearing_z = 0.267
    plinth.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(tonearm_pivot_x, tonearm_pivot_y, plinth_height + 0.015)),
        material=satin_black,
        name="tonearm_base",
    )
    plinth.visual(
        Cylinder(radius=0.024, length=tonearm_bearing_z - (plinth_height + 0.030)),
        origin=Origin(
            xyz=(
                tonearm_pivot_x,
                tonearm_pivot_y,
                (tonearm_bearing_z + plinth_height + 0.030) / 2.0,
            )
        ),
        material=brushed_metal,
        name="tonearm_post",
    )
    plinth.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(xyz=(tonearm_pivot_x, tonearm_pivot_y, tonearm_bearing_z - 0.009)),
        material=satin_black,
        name="tonearm_bearing",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.167, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed_metal,
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.149, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=record_black,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.036, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0495)),
        material=label_red,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=brushed_metal,
        name="spindle",
    )
    platter.visual(
        Box((0.026, 0.010, 0.006)),
        origin=Origin(xyz=(0.132, 0.0, 0.045)),
        material=brushed_metal,
        name="strobe_mark",
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_x, platter_y, support_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.026, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="pivot_hub",
    )
    tonearm.visual(
        Cylinder(radius=0.0065, length=0.315),
        origin=Origin(xyz=(0.1575, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.020, length=0.062),
        origin=Origin(xyz=(-0.040, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.058, 0.030, 0.010)),
        origin=Origin(xyz=(0.320, 0.0, 0.025), rpy=(0.0, 0.0, -0.18)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Cylinder(radius=0.0024, length=0.021),
        origin=Origin(xyz=(0.342, -0.006, 0.0105)),
        material=stylus_dark,
        name="stylus",
    )

    # The child frame is rotated so local +X points from the base toward the record.
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(tonearm_pivot_x, tonearm_pivot_y, tonearm_bearing_z), rpy=(0.0, 0.0, 2.73)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.4, lower=-0.55, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    ctx.check(
        "platter has continuous vertical rotation",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_joint.articulation_type}, axis={platter_joint.axis}",
    )
    ctx.check(
        "tonearm has bounded vertical pivot",
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
        positive_elem="platter_disk",
        negative_elem="center_support",
        max_gap=0.0015,
        max_penetration=0.0,
        name="platter sits on the tall center support",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="plinth_shell",
        min_gap=0.140,
        name="rotating stage is elevated well above the plinth",
    )
    ctx.expect_origin_distance(
        tonearm,
        platter,
        axes="xy",
        min_dist=0.28,
        max_dist=0.38,
        name="tonearm pivot base is off to one side of the platter",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="headshell",
        negative_elem="record_disc",
        min_gap=0.006,
        name="headshell clears the record surface",
    )

    rest_marker = ctx.part_element_world_aabb(platter, elem="strobe_mark")
    with ctx.pose({platter_joint: math.pi / 2.0}):
        turned_marker = ctx.part_element_world_aabb(platter, elem="strobe_mark")
    if rest_marker is not None and turned_marker is not None:
        rest_center = tuple((rest_marker[0][i] + rest_marker[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_marker[0][i] + turned_marker[1][i]) / 2.0 for i in range(3))
        marker_motion = math.dist(rest_center[:2], turned_center[:2])
    else:
        marker_motion = 0.0
    ctx.check(
        "off-axis platter mark visibly rotates",
        marker_motion > 0.12,
        details=f"marker_motion={marker_motion}",
    )

    rest_head = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_joint: tonearm_joint.motion_limits.lower}):
        swung_head = ctx.part_element_world_aabb(tonearm, elem="headshell")
    if rest_head is not None and swung_head is not None:
        rest_center = tuple((rest_head[0][i] + rest_head[1][i]) / 2.0 for i in range(3))
        swung_center = tuple((swung_head[0][i] + swung_head[1][i]) / 2.0 for i in range(3))
        head_motion = math.dist(rest_center[:2], swung_center[:2])
    else:
        head_motion = 0.0
    ctx.check(
        "tonearm headshell sweeps across the record area",
        head_motion > 0.14,
        details=f"head_motion={head_motion}",
    )

    return ctx.report()


object_model = build_object_model()
