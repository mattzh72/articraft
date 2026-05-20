from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    mesh_from_cadquery,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
import cadquery as cq


def _top_flange_mesh():
    """Small drilled flange with an index tab, authored in upper-stage local axes."""
    flange = cq.Workplane("XY").circle(0.082).extrude(0.016)
    tab = cq.Workplane("XY").box(0.055, 0.026, 0.016).translate((0.095, 0.0, 0.008))
    flange = flange.union(tab)
    for x, y in ((0.052, 0.0), (-0.052, 0.0), (0.0, 0.052), (0.0, -0.052)):
        cutter = cq.Workplane("XY").circle(0.006).extrude(0.022).translate((x, y, -0.003))
        flange = flange.cut(cutter)
    return flange


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_fixture_stack")

    cast_iron = model.material("satin_cast_iron", color=(0.24, 0.25, 0.25, 1.0))
    machined_steel = model.material("machined_steel", color=(0.72, 0.72, 0.68, 1.0))
    blue_anodized = model.material("blue_anodized", color=(0.12, 0.28, 0.55, 1.0))
    dark = model.material("black_oxide", color=(0.02, 0.022, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.285, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.125, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=machined_steel,
        name="lower_bearing",
    )
    for i, (x, y) in enumerate(((0.195, 0.120), (-0.195, 0.120), (0.195, -0.120), (-0.195, -0.120))):
        base.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, y, 0.037)),
            material=dark,
            name=f"base_bolt_{i}",
        )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.210, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=blue_anodized,
        name="lower_disk",
    )
    lower_stage.visual(
        Cylinder(radius=0.073, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=machined_steel,
        name="lower_hub",
    )
    lower_stage.visual(
        Box((0.082, 0.105, 0.140)),
        origin=Origin(xyz=(0.105, 0.0, 0.113)),
        material=blue_anodized,
        name="center_pedestal",
    )
    lower_stage.visual(
        Box((0.260, 0.090, 0.050)),
        origin=Origin(xyz=(0.225, 0.0, 0.203)),
        material=blue_anodized,
        name="side_arm",
    )
    for i, y in enumerate((-0.053, 0.053)):
        lower_stage.visual(
            Box((0.225, 0.014, 0.088)),
            origin=Origin(xyz=(0.235, y, 0.147)),
            material=blue_anodized,
            name=f"web_rib_{i}",
        )
    lower_stage.visual(
        Cylinder(radius=0.082, length=0.057),
        origin=Origin(xyz=(0.340, 0.0, 0.2065)),
        material=machined_steel,
        name="upper_bearing",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        Cylinder(radius=0.078, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=machined_steel,
        name="thrust_washer",
    )
    upper_stage.visual(
        Cylinder(radius=0.112, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=blue_anodized,
        name="upper_disk",
    )
    upper_stage.visual(
        Cylinder(radius=0.046, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=machined_steel,
        name="upper_boss",
    )
    upper_stage.visual(
        mesh_from_cadquery(_top_flange_mesh(), "top_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=machined_steel,
        name="top_flange",
    )

    model.articulation(
        "lower_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "upper_axis",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.340, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_axis = object_model.get_articulation("lower_axis")
    upper_axis = object_model.get_articulation("upper_axis")

    ctx.check(
        "two vertical revolute rotary stages",
        len(object_model.articulations) == 2
        and lower_axis.articulation_type == ArticulationType.REVOLUTE
        and upper_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_axis.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_axis.axis) == (0.0, 0.0, 1.0),
        details="Fixture should have exactly lower and upper revolute axes, both vertical and parallel.",
    )

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        positive_elem="lower_disk",
        negative_elem="lower_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower turntable is supported on base bearing",
    )
    ctx.expect_gap(
        upper_stage,
        lower_stage,
        axis="z",
        positive_elem="thrust_washer",
        negative_elem="upper_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper rotary stage is supported on raised bearing",
    )
    ctx.expect_origin_distance(
        upper_stage,
        lower_stage,
        axes="xy",
        min_dist=0.335,
        max_dist=0.345,
        name="upper axis is offset from lower axis",
    )

    rest_upper = ctx.part_world_position(upper_stage)
    with ctx.pose({lower_axis: math.pi / 2.0}):
        turned_upper = ctx.part_world_position(upper_stage)
        ctx.expect_origin_distance(
            upper_stage,
            lower_stage,
            axes="xy",
            min_dist=0.335,
            max_dist=0.345,
            name="offset axis rides with lower turntable",
        )
    ctx.check(
        "lower stage sweeps the raised side arm around the base axis",
        rest_upper is not None
        and turned_upper is not None
        and rest_upper[0] > 0.33
        and abs(rest_upper[1]) < 0.002
        and abs(turned_upper[0]) < 0.002
        and turned_upper[1] > 0.33,
        details=f"rest={rest_upper}, turned={turned_upper}",
    )

    with ctx.pose({upper_axis: math.pi / 2.0}):
        spun_upper = ctx.part_world_position(upper_stage)
        ctx.expect_gap(
            upper_stage,
            lower_stage,
            axis="z",
            positive_elem="thrust_washer",
            negative_elem="upper_bearing",
            max_gap=0.001,
            max_penetration=0.0,
            name="upper stage stays seated while rotating about its own axis",
        )
    ctx.check(
        "upper stage spins about its supported offset axis",
        rest_upper is not None
        and spun_upper is not None
        and abs(spun_upper[0] - rest_upper[0]) < 0.002
        and abs(spun_upper[1] - rest_upper[1]) < 0.002,
        details=f"rest={rest_upper}, spun={spun_upper}",
    )

    return ctx.report()


object_model = build_object_model()
