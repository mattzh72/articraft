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


def _linear_bushing_mesh(name: str):
    """A short hollow bearing sleeve with its bore running along local X."""
    sleeve = (
        cq.Workplane("YZ")
        .circle(0.029)
        # The bore is modeled with a very slight interference fit so the rail is
        # mechanically captured rather than numerically floating in the sleeve.
        .circle(0.0142)
        .extrude(0.105, both=True)
    )
    return mesh_from_cadquery(sleeve, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_slide_arm")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    guide_steel = model.material("guide_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    carriage_grey = model.material("carriage_grey", rgba=(0.38, 0.41, 0.44, 1.0))
    arm_blue = model.material("arm_blue", rgba=(0.14, 0.24, 0.55, 1.0))
    pivot_steel = model.material("pivot_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    bushing_mesh = _linear_bushing_mesh("linear_bushing")

    frame = model.part("frame")
    frame.visual(
        Box((0.18, 0.46, 0.150)),
        origin=Origin(xyz=(0.035, 0.0, 0.075)),
        material=frame_paint,
        name="rear_foot",
    )
    frame.visual(
        Box((0.060, 0.420, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=frame_paint,
        name="lower_bridge",
    )
    frame.visual(
        Box((0.060, 0.420, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        material=frame_paint,
        name="upper_bridge",
    )
    frame.visual(
        Box((0.060, 0.060, 0.540)),
        origin=Origin(xyz=(0.0, -0.180, 0.430)),
        material=frame_paint,
        name="upright_0",
    )
    frame.visual(
        Box((0.060, 0.060, 0.540)),
        origin=Origin(xyz=(0.0, 0.180, 0.430)),
        material=frame_paint,
        name="upright_1",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.560),
        origin=Origin(xyz=(0.300, -0.140, 0.480), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_steel,
        name="rail_0",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.560),
        origin=Origin(xyz=(0.300, 0.140, 0.480), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_steel,
        name="rail_1",
    )
    frame.visual(
        Box((0.050, 0.400, 0.090)),
        origin=Origin(xyz=(0.580, 0.0, 0.480)),
        material=frame_paint,
        name="front_bridge",
    )
    frame.visual(
        Box((0.040, 0.300, 0.040)),
        origin=Origin(xyz=(0.300, 0.0, 0.450)),
        material=frame_paint,
        name="lower_tie",
    )

    carriage = model.part("carriage")
    carriage.visual(
        bushing_mesh,
        origin=Origin(xyz=(0.0, -0.140, 0.0)),
        material=carriage_grey,
        name="bushing_0",
    )
    carriage.visual(
        bushing_mesh,
        origin=Origin(xyz=(0.0, 0.140, 0.0)),
        material=carriage_grey,
        name="bushing_1",
    )
    carriage.visual(
        Box((0.125, 0.330, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=carriage_grey,
        name="crosshead",
    )
    carriage.visual(
        Box((0.110, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, -0.140, -0.044)),
        material=carriage_grey,
        name="saddle_0",
    )
    carriage.visual(
        Box((0.110, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, 0.140, -0.044)),
        material=carriage_grey,
        name="saddle_1",
    )
    carriage.visual(
        Box((0.105, 0.090, 0.130)),
        origin=Origin(xyz=(0.040, 0.0, 0.041)),
        material=carriage_grey,
        name="arm_stanchion",
    )
    carriage.visual(
        Cylinder(radius=0.036, length=0.044),
        origin=Origin(xyz=(0.040, 0.0, 0.098)),
        material=pivot_steel,
        name="shoulder_cap",
    )

    link_0_length = 0.220
    link_0 = model.part("link_0")
    link_0.visual(
        Cylinder(radius=0.032, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=arm_blue,
        name="proximal_boss",
    )
    link_0.visual(
        Box((link_0_length, 0.050, 0.024)),
        origin=Origin(xyz=(link_0_length / 2.0, 0.0, 0.0175)),
        material=arm_blue,
        name="bar",
    )
    link_0.visual(
        Cylinder(radius=0.032, length=0.035),
        origin=Origin(xyz=(link_0_length, 0.0, 0.0175)),
        material=arm_blue,
        name="distal_boss",
    )

    link_1_length = 0.180
    link_1 = model.part("link_1")
    link_1.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=arm_blue,
        name="proximal_boss",
    )
    link_1.visual(
        Box((link_1_length, 0.046, 0.022)),
        origin=Origin(xyz=(link_1_length / 2.0, 0.0, 0.0525)),
        material=arm_blue,
        name="bar",
    )
    link_1.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(link_1_length, 0.0, 0.0525)),
        material=arm_blue,
        name="distal_boss",
    )

    pad = model.part("pad")
    pad.visual(
        Box((0.110, 0.085, 0.018)),
        origin=Origin(xyz=(0.050, 0.0, 0.009)),
        material=rubber,
        name="pad_block",
    )
    pad.visual(
        Box((0.040, 0.050, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, 0.023)),
        material=pivot_steel,
        name="mount_plate",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.140, 0.0, 0.480)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.280),
    )
    model.articulation(
        "shoulder_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=link_0,
        origin=Origin(xyz=(0.040, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link_0_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=link_1,
        child=pad,
        origin=Origin(xyz=(link_1_length, 0.0, 0.070)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    pad = object_model.get_part("pad")
    slide = object_model.get_articulation("carriage_slide")
    shoulder = object_model.get_articulation("shoulder_pivot")
    elbow = object_model.get_articulation("elbow_pivot")

    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="rail_0",
        elem_b="bushing_0",
        reason="The guide rail is intentionally captured in the linear bushing with a tiny bearing-seat interference.",
    )
    ctx.allow_overlap(
        frame,
        carriage,
        elem_a="rail_1",
        elem_b="bushing_1",
        reason="The guide rail is intentionally captured in the linear bushing with a tiny bearing-seat interference.",
    )

    ctx.expect_within(
        frame,
        carriage,
        axes="yz",
        inner_elem="rail_0",
        outer_elem="bushing_0",
        margin=0.005,
        name="rear rail centered through lower bushing",
    )
    ctx.expect_within(
        frame,
        carriage,
        axes="yz",
        inner_elem="rail_1",
        outer_elem="bushing_1",
        margin=0.005,
        name="front rail centered through upper bushing",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        elem_a="bushing_0",
        elem_b="rail_0",
        min_overlap=0.090,
        name="retracted carriage remains on rail",
    )
    ctx.expect_gap(
        link_0,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="proximal_boss",
        negative_elem="shoulder_cap",
        name="shoulder pivot stack is seated",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="proximal_boss",
        negative_elem="distal_boss",
        name="elbow pivot stack is seated",
    )
    ctx.expect_gap(
        pad,
        link_1,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pad_block",
        negative_elem="distal_boss",
        name="plain pad is seated on arm tip",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_pad = ctx.part_world_position(pad)
    with ctx.pose({slide: 0.280}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            elem_a="bushing_0",
            elem_b="rail_0",
            min_overlap=0.090,
            name="extended carriage remains on rail",
        )
        extended_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "carriage slides forward along rails",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.25,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    with ctx.pose({shoulder: 0.70, elbow: 0.0}):
        swept_pad = ctx.part_world_position(pad)

    with ctx.pose({shoulder: 0.0, elbow: 0.85}):
        folded_pad = ctx.part_world_position(pad)

    ctx.check(
        "shoulder pivot swings the arm laterally",
        rest_pad is not None and swept_pad is not None and swept_pad[1] > rest_pad[1] + 0.18,
        details=f"rest={rest_pad}, swept={swept_pad}",
    )
    ctx.check(
        "elbow pivot folds the second link",
        rest_pad is not None and folded_pad is not None and folded_pad[1] > rest_pad[1] + 0.10,
        details=f"rest={rest_pad}, folded={folded_pad}",
    )

    return ctx.report()


object_model = build_object_model()
