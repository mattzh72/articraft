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


def _handwheel_mesh(outer_radius: float, rim_radial: float, hub_radius: float, thickness: float):
    wheel = cq.Workplane("XZ").circle(outer_radius).extrude(thickness)
    window_offset = hub_radius + 0.020
    window_size = 2.0 * (outer_radius - rim_radial - window_offset)
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            wheel = wheel.cut(
                cq.Workplane("XZ")
                .center(sx * window_offset, sz * window_offset)
                .rect(window_size, window_size)
                .extrude(thickness)
            )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="irrigation_sluice_gate")

    frame_metal = model.material("frame_metal", rgba=(0.47, 0.50, 0.52, 1.0))
    gate_finish = model.material("gate_finish", rgba=(0.21, 0.38, 0.45, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.56, 0.58, 0.60, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.66, 0.12, 0.10, 1.0))
    pawl_finish = model.material("pawl_finish", rgba=(0.19, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.10, 0.12, 1.02)),
        origin=Origin(xyz=(-0.30, 0.0, 0.51)),
        material=frame_metal,
        name="left_post",
    )
    frame.visual(
        Box((0.10, 0.12, 1.02)),
        origin=Origin(xyz=(0.30, 0.0, 0.51)),
        material=frame_metal,
        name="right_post",
    )
    frame.visual(
        Box((0.70, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, 0.04, 0.06)),
        material=frame_metal,
        name="sill",
    )
    frame.visual(
        Box((0.70, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, 0.04, 0.78)),
        material=frame_metal,
        name="top_beam",
    )
    frame.visual(
        Box((0.035, 0.016, 0.82)),
        origin=Origin(xyz=(-0.216, 0.027, 0.53)),
        material=frame_metal,
        name="left_guide_front",
    )
    frame.visual(
        Box((0.035, 0.016, 0.82)),
        origin=Origin(xyz=(-0.216, -0.040, 0.53)),
        material=frame_metal,
        name="left_guide_rear",
    )
    frame.visual(
        Box((0.035, 0.016, 0.82)),
        origin=Origin(xyz=(0.216, 0.027, 0.53)),
        material=frame_metal,
        name="right_guide_front",
    )
    frame.visual(
        Box((0.035, 0.016, 0.82)),
        origin=Origin(xyz=(0.216, -0.040, 0.53)),
        material=frame_metal,
        name="right_guide_rear",
    )
    frame.visual(
        Box((0.050, 0.020, 0.08)),
        origin=Origin(xyz=(-0.242, -0.040, 0.90)),
        material=frame_metal,
        name="left_guide_head",
    )
    frame.visual(
        Box((0.050, 0.020, 0.08)),
        origin=Origin(xyz=(0.242, -0.040, 0.90)),
        material=frame_metal,
        name="right_guide_head",
    )

    gate = model.part("gate")
    gate.visual(
        Box((0.42, 0.020, 0.64)),
        origin=Origin(xyz=(0.0, -0.015, 0.32)),
        material=gate_finish,
        name="plate",
    )
    gate.visual(
        Box((0.46, 0.028, 0.05)),
        origin=Origin(xyz=(0.0, -0.015, 0.615)),
        material=gate_finish,
        name="top_rail",
    )
    gate.visual(
        Box((0.46, 0.026, 0.04)),
        origin=Origin(xyz=(0.0, -0.015, 0.020)),
        material=gate_finish,
        name="bottom_lip",
    )
    gate.visual(
        Box((0.024, 0.034, 0.68)),
        origin=Origin(xyz=(-0.218, -0.015, 0.34)),
        material=gate_finish,
        name="left_runner",
    )
    gate.visual(
        Box((0.024, 0.034, 0.68)),
        origin=Origin(xyz=(0.218, -0.015, 0.34)),
        material=gate_finish,
        name="right_runner",
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=0.0, upper=0.24),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.18, 0.12, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=housing_finish,
        name="mount_plate",
    )
    housing.visual(
        Box((0.10, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=housing_finish,
        name="pedestal",
    )
    housing.visual(
        Box((0.18, 0.10, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=housing_finish,
        name="body",
    )
    housing.visual(
        Box((0.16, 0.08, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=housing_finish,
        name="cap",
    )
    housing.visual(
        Box((0.036, 0.030, 0.040)),
        origin=Origin(xyz=(0.108, 0.040, 0.104)),
        material=housing_finish,
        name="pawl_bracket",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(-0.010, 0.064, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="front_boss",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.120, 0.062, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="pawl_boss",
    )

    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, 0.040, 0.84)),
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_mesh(0.080, 0.012, 0.016, 0.012), "handwheel"),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=wheel_finish,
        name="wheel",
    )
    model.articulation(
        "housing_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(xyz=(-0.010, 0.079, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )

    pawl = model.part("pawl")
    pawl.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pawl_finish,
        name="pivot_sleeve",
    )
    pawl.visual(
        Box((0.012, 0.012, 0.062)),
        origin=Origin(
            xyz=(-0.017, 0.007, 0.026),
            rpy=(0.0, -0.58, 0.0),
        ),
        material=pawl_finish,
        name="arm",
    )
    pawl.visual(
        Box((0.018, 0.010, 0.012)),
        origin=Origin(
            xyz=(-0.036, 0.007, 0.050),
            rpy=(0.0, -0.95, 0.0),
        ),
        material=pawl_finish,
        name="tooth",
    )
    model.articulation(
        "housing_to_pawl",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pawl,
        origin=Origin(xyz=(0.120, 0.069, 0.104)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.55, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate")
    housing = object_model.get_part("housing")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("pawl")
    gate_slide = object_model.get_articulation("frame_to_gate")
    wheel_joint = object_model.get_articulation("housing_to_handwheel")
    pawl_joint = object_model.get_articulation("housing_to_pawl")

    ctx.expect_contact(
        housing,
        frame,
        elem_a="mount_plate",
        elem_b="top_beam",
        name="operator housing is seated on the top beam",
    )
    ctx.expect_contact(
        handwheel,
        housing,
        elem_a="wheel",
        elem_b="front_boss",
        name="handwheel is mounted on the housing boss",
    )
    ctx.expect_contact(
        pawl,
        housing,
        elem_a="pivot_sleeve",
        elem_b="pawl_boss",
        name="pawl is mounted on its housing boss",
    )
    ctx.expect_origin_distance(
        handwheel,
        pawl,
        axes="x",
        min_dist=0.10,
        max_dist=0.16,
        name="pawl sits beside the handwheel",
    )

    wheel_limits = wheel_joint.motion_limits
    ctx.check(
        "handwheel joint is continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=f"type={wheel_joint.articulation_type!r}, limits={wheel_limits!r}",
    )

    slide_limits = gate_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({gate_slide: 0.0}):
            ctx.expect_gap(
                gate,
                frame,
                axis="z",
                positive_elem="plate",
                negative_elem="sill",
                min_gap=0.0,
                max_gap=0.005,
                name="gate rests close to the sill when closed",
            )
            ctx.expect_gap(
                frame,
                gate,
                axis="y",
                positive_elem="left_guide_front",
                negative_elem="left_runner",
                min_gap=0.010,
                max_gap=0.020,
                name="gate runner clears the front guide",
            )
            ctx.expect_gap(
                gate,
                frame,
                axis="y",
                positive_elem="left_runner",
                negative_elem="left_guide_rear",
                min_gap=0.0,
                max_gap=0.002,
                name="gate runner clears the rear guide",
            )
            closed_pos = ctx.part_world_position(gate)

        with ctx.pose({gate_slide: slide_limits.upper}):
            ctx.expect_gap(
                gate,
                frame,
                axis="z",
                positive_elem="plate",
                negative_elem="sill",
                min_gap=0.22,
                name="gate lifts clear of the sill when opened",
            )
            ctx.expect_gap(
                frame,
                gate,
                axis="y",
                positive_elem="left_guide_front",
                negative_elem="left_runner",
                min_gap=0.010,
                max_gap=0.020,
                name="front guide clearance remains at full lift",
            )
            ctx.expect_gap(
                gate,
                frame,
                axis="y",
                positive_elem="left_runner",
                negative_elem="left_guide_rear",
                min_gap=0.0,
                max_gap=0.002,
                name="rear guide clearance remains at full lift",
            )
            open_pos = ctx.part_world_position(gate)

        ctx.check(
            "gate rises upward",
            closed_pos is not None and open_pos is not None and open_pos[2] > closed_pos[2] + 0.20,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    pawl_limits = pawl_joint.motion_limits
    ctx.check(
        "pawl uses a bounded lock lever motion",
        pawl_joint.articulation_type == ArticulationType.REVOLUTE
        and pawl_limits is not None
        and pawl_limits.lower is not None
        and pawl_limits.upper is not None
        and pawl_limits.lower < pawl_limits.upper,
        details=f"type={pawl_joint.articulation_type!r}, limits={pawl_limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
