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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_manipulator")

    aluminum = model.material("brushed_aluminum", color=(0.72, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_bearing_steel", color=(0.08, 0.09, 0.10, 1.0))
    black = model.material("matte_black", color=(0.015, 0.015, 0.018, 1.0))
    blue = model.material("blue_slide_scale", color=(0.05, 0.18, 0.55, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.26, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="bench_plate",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=black,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_steel,
        name="base_bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=aluminum,
        name="bearing_race",
    )
    for i, (x, y) in enumerate(
        ((-0.115, -0.085), (-0.115, 0.085), (0.115, -0.085), (0.115, 0.085))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.028)),
            material=black,
            name=f"mount_bolt_{i}",
        )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.073, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aluminum,
        name="rotary_table",
    )
    shoulder.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=dark_steel,
        name="base_bearing_cap",
    )
    for y, name in ((-0.045, "link_plate_0"), (0.045, "link_plate_1")):
        shoulder.visual(
            Box((0.380, 0.026, 0.026)),
            origin=Origin(xyz=(0.205, y, 0.041)),
            material=aluminum,
            name=name,
        )
    for i, x in enumerate((0.120, 0.295)):
        shoulder.visual(
            Box((0.050, 0.112, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material=aluminum,
            name=f"plate_spacer_{i}",
        )
    shoulder.visual(
        Cylinder(radius=0.061, length=0.026),
        origin=Origin(xyz=(0.420, 0.0, 0.041)),
        material=aluminum,
        name="elbow_bearing_housing",
    )
    shoulder.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.420, 0.0, 0.057)),
        material=dark_steel,
        name="elbow_bearing_race",
    )

    elbow = model.part("elbow")
    elbow.visual(
        Cylinder(radius=0.057, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=aluminum,
        name="elbow_rotor",
    )
    elbow.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_steel,
        name="elbow_bearing_cap",
    )
    for y, name in ((-0.024, "slide_guide_0"), (0.024, "slide_guide_1")):
        elbow.visual(
            Box((0.320, 0.018, 0.034)),
            origin=Origin(xyz=(0.190, y, 0.049)),
            material=aluminum,
            name=name,
        )
        elbow.visual(
            Box((0.050, 0.020, 0.028)),
            origin=Origin(xyz=(0.045, y, 0.034)),
            material=aluminum,
            name=f"rear_cheek_{name[-1]}",
        )
        elbow.visual(
            Box((0.035, 0.020, 0.034)),
            origin=Origin(xyz=(0.345, y, 0.049)),
            material=aluminum,
            name=f"front_cheek_{name[-1]}",
        )
    elbow.visual(
        Box((0.320, 0.082, 0.012)),
        origin=Origin(xyz=(0.190, 0.0, 0.072)),
        material=aluminum,
        name="top_bridge",
    )
    elbow.visual(
        Box((0.240, 0.010, 0.004)),
        origin=Origin(xyz=(0.215, 0.0, 0.080)),
        material=blue,
        name="slide_scale",
    )

    end_slide = model.part("end_slide")
    end_slide.visual(
        Box((0.340, 0.030, 0.022)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=aluminum,
        name="inner_member",
    )
    end_slide.visual(
        Box((0.035, 0.042, 0.032)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=dark_steel,
        name="tool_block",
    )
    end_slide.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.291, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tool_flange",
    )
    end_slide.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.306, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="tool_socket",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(0.420, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-2.45, upper=2.45),
    )
    model.articulation(
        "end_slide",
        ArticulationType.PRISMATIC,
        parent=elbow,
        child=end_slide,
        origin=Origin(xyz=(0.170, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.140),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder")
    elbow = object_model.get_part("elbow")
    end_slide = object_model.get_part("end_slide")
    base_yaw = object_model.get_articulation("base_yaw")
    elbow_yaw = object_model.get_articulation("elbow_yaw")
    slide = object_model.get_articulation("end_slide")

    ctx.check(
        "two rotary joints plus telescoping slide",
        len(object_model.articulations) == 3
        and base_yaw.articulation_type == ArticulationType.REVOLUTE
        and elbow_yaw.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_contact(
        shoulder,
        base,
        elem_a="rotary_table",
        elem_b="bearing_race",
        name="base bearing race supports rotary table",
    )
    ctx.expect_contact(
        elbow,
        shoulder,
        elem_a="elbow_rotor",
        elem_b="elbow_bearing_race",
        name="elbow bearing race supports rotor",
    )
    ctx.expect_within(
        end_slide,
        elbow,
        axes="yz",
        inner_elem="inner_member",
        margin=0.001,
        name="slide member is captured between guide plates",
    )
    ctx.expect_overlap(
        end_slide,
        elbow,
        axes="x",
        elem_a="inner_member",
        min_overlap=0.180,
        name="collapsed slide retains insertion in sleeve",
    )

    rest_elbow_pos = ctx.part_world_position(elbow)
    with ctx.pose({base_yaw: 0.65}):
        yawed_elbow_pos = ctx.part_world_position(elbow)
    ctx.check(
        "base yaw sweeps first link around vertical bearing",
        rest_elbow_pos is not None
        and yawed_elbow_pos is not None
        and yawed_elbow_pos[1] > rest_elbow_pos[1] + 0.10,
        details=f"rest={rest_elbow_pos}, yawed={yawed_elbow_pos}",
    )

    rest_tool_pos = ctx.part_world_position(end_slide)
    with ctx.pose({elbow_yaw: 0.55}):
        bent_tool_pos = ctx.part_world_position(end_slide)
    ctx.check(
        "elbow yaw swings the telescoping stage",
        rest_tool_pos is not None
        and bent_tool_pos is not None
        and bent_tool_pos[1] > rest_tool_pos[1] + 0.075,
        details=f"rest={rest_tool_pos}, bent={bent_tool_pos}",
    )

    rest_slide_pos = ctx.part_world_position(end_slide)
    with ctx.pose({slide: 0.140}):
        ctx.expect_within(
            end_slide,
            elbow,
            axes="yz",
            inner_elem="inner_member",
            margin=0.001,
            name="extended slide remains in guide clearance",
        )
        ctx.expect_overlap(
            end_slide,
            elbow,
            axes="x",
            elem_a="inner_member",
            min_overlap=0.090,
            name="extended slide keeps retained insertion",
        )
        extended_slide_pos = ctx.part_world_position(end_slide)
    ctx.check(
        "end slide extends along the tool axis",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.120,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    return ctx.report()


object_model = build_object_model()
