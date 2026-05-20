from __future__ import annotations

from math import pi

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


def _rounded_pad(length: float, width: float, thickness: float, name: str):
    pad = cq.Workplane("XY").box(length, width, thickness).edges().fillet(min(width, thickness) * 0.18)
    return mesh_from_cadquery(pad, name)


def _selector_rail(name: str):
    rail = cq.Workplane("XZ").rect(0.28, 0.065).extrude(0.006, both=True)
    holes = (
        cq.Workplane("XZ")
        .pushPoints([(-0.10, -0.012), (-0.05, 0.001), (0.0, 0.013), (0.05, 0.024), (0.10, 0.031)])
        .circle(0.008)
        .extrude(0.02, both=True)
    )
    return mesh_from_cadquery(rail.cut(holes), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_decline_bench")

    frame_paint = model.material("frame_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    upholstery = model.material("upholstery", rgba=(0.11, 0.11, 0.12, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    trim = model.material("trim", rgba=(0.23, 0.24, 0.25, 1.0))
    foam = model.material("foam", rgba=(0.16, 0.16, 0.17, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(Box((0.16, 0.56, 0.06)), origin=Origin(xyz=(-0.67, 0.0, 0.03)), material=frame_paint, name="rear_stabilizer")
    frame.visual(Box((0.08, 0.10, 0.28)), origin=Origin(xyz=(-0.58, 0.0, 0.17)), material=frame_paint, name="rear_upright")
    frame.visual(Box((1.16, 0.10, 0.08)), origin=Origin(xyz=(0.04, 0.0, 0.21)), material=frame_paint, name="main_spine")
    frame.visual(Box((0.10, 0.14, 0.14)), origin=Origin(xyz=(0.03, 0.0, 0.30)), material=frame_paint, name="back_hinge_support")
    frame.visual(Box((0.10, 0.16, 0.12)), origin=Origin(xyz=(0.35, 0.0, 0.29)), material=frame_paint, name="seat_hinge_support")
    frame.visual(Box((0.16, 0.50, 0.06)), origin=Origin(xyz=(0.65, 0.0, 0.03)), material=frame_paint, name="front_stabilizer")
    frame.visual(Box((0.08, 0.10, 0.32)), origin=Origin(xyz=(0.57, 0.0, 0.19)), material=frame_paint, name="front_upright")
    frame.visual(Box((0.24, 0.08, 0.08)), origin=Origin(xyz=(0.69, 0.0, 0.25)), material=frame_paint, name="front_bridge")
    frame.visual(Box((0.32, 0.08, 0.05)), origin=Origin(xyz=(0.22, 0.0, 0.29)), material=trim, name="seat_support_rail")
    frame.visual(Box((0.12, 0.10, 0.28)), origin=Origin(xyz=(0.81, 0.0, 0.18)), material=frame_paint, name="roller_post")
    frame.visual(Box((0.08, 0.10, 0.08)), origin=Origin(xyz=(0.87, 0.0, 0.28)), material=frame_paint, name="roller_head")
    frame.visual(Box((0.04, 0.08, 0.16)), origin=Origin(xyz=(-0.67, -0.24, 0.10)), material=frame_paint, name="wheel_bracket_0")
    frame.visual(Box((0.04, 0.08, 0.16)), origin=Origin(xyz=(-0.67, 0.24, 0.10)), material=frame_paint, name="wheel_bracket_1")
    frame.visual(
        _selector_rail("selector_rail"),
        origin=Origin(xyz=(-0.11, 0.0, 0.28)),
        material=hinge_steel,
        name="selector_rail",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        _rounded_pad(0.92, 0.31, 0.07, "backrest_pad"),
        origin=Origin(xyz=(-0.46, 0.0, 0.055)),
        material=upholstery,
        name="backrest_pad",
    )
    back_pad.visual(
        Box((0.40, 0.10, 0.04)),
        origin=Origin(xyz=(-0.23, 0.0, 0.00)),
        material=frame_paint,
        name="back_support_arm",
    )
    back_pad.visual(
        Cylinder(radius=0.016, length=0.14),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="back_hinge_barrel",
    )
    back_pad.visual(
        Box((0.03, 0.08, 0.03)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=hinge_steel,
        name="back_hinge_lug",
    )
    back_pad.visual(
        Box((0.05, 0.12, 0.02)),
        origin=Origin(xyz=(-0.89, 0.0, 0.01)),
        material=trim,
        name="back_tail",
    )
    back_pad.visual(
        Box((0.05, 0.028, 0.04)),
        origin=Origin(xyz=(-0.18, 0.041, 0.0)),
        material=hinge_steel,
        name="selector_collar",
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        _rounded_pad(0.30, 0.29, 0.06, "seat_pad"),
        origin=Origin(xyz=(-0.15, 0.0, 0.045)),
        material=upholstery,
        name="seat_pad",
    )
    seat_pad.visual(
        Box((0.17, 0.10, 0.035)),
        origin=Origin(xyz=(-0.085, 0.0, -0.0025)),
        material=frame_paint,
        name="seat_support_arm",
    )
    seat_pad.visual(
        Cylinder(radius=0.014, length=0.14),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="seat_hinge_barrel",
    )
    seat_pad.visual(
        Box((0.05, 0.12, 0.02)),
        origin=Origin(xyz=(-0.25, 0.0, 0.01)),
        material=trim,
        name="seat_tail",
    )

    leg_roller_0 = model.part("leg_roller_0")
    leg_roller_0.visual(
        Cylinder(radius=0.07, length=0.16),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=foam,
        name="roller_body",
    )
    leg_roller_0.visual(
        Cylinder(radius=0.025, length=0.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="roller_core",
    )

    leg_roller_1 = model.part("leg_roller_1")
    leg_roller_1.visual(
        Cylinder(radius=0.07, length=0.16),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=foam,
        name="roller_body",
    )
    leg_roller_1.visual(
        Cylinder(radius=0.025, length=0.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="roller_core",
    )

    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.06, length=0.05),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_tire",
    )
    wheel_0.visual(
        Cylinder(radius=0.032, length=0.056),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="wheel_hub",
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.06, length=0.05),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_tire",
    )
    wheel_1.visual(
        Cylinder(radius=0.032, length=0.056),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="wheel_hub",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.007, length=0.02),
        origin=Origin(xyz=(0.0, 0.01, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="knob_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.029, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="knob_head",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.03, 0.0, 0.39)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.5, lower=-0.35, upper=1.15),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(0.35, 0.0, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5, lower=0.0, upper=0.45),
    )
    model.articulation(
        "leg_roller_spin_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=leg_roller_0,
        origin=Origin(xyz=(0.87, -0.13, 0.28)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "leg_roller_spin_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=leg_roller_1,
        origin=Origin(xyz=(0.87, 0.13, 0.28)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )
    model.articulation(
        "wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.67, -0.305, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.67, 0.305, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "selector_pull",
        ArticulationType.PRISMATIC,
        parent=back_pad,
        child=selector_knob,
        origin=Origin(xyz=(-0.18, 0.055, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.12, lower=0.0, upper=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_pad = object_model.get_part("back_pad")
    seat_pad = object_model.get_part("seat_pad")
    frame = object_model.get_part("frame")
    leg_roller_0 = object_model.get_part("leg_roller_0")
    leg_roller_1 = object_model.get_part("leg_roller_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    selector_knob = object_model.get_part("selector_knob")
    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    selector_pull = object_model.get_articulation("selector_pull")

    with ctx.pose({back_hinge: 0.0, seat_hinge: 0.0}):
        ctx.expect_gap(
            seat_pad,
            back_pad,
            axis="x",
            positive_elem="seat_pad",
            negative_elem="backrest_pad",
            min_gap=0.0,
            max_gap=0.03,
            name="seat meets the backrest without overlap at rest",
        )
        ctx.expect_overlap(
            seat_pad,
            back_pad,
            axes="y",
            elem_a="seat_pad",
            elem_b="backrest_pad",
            min_overlap=0.24,
            name="seat and backrest stay aligned across the bench width",
        )

    rest_back = ctx.part_element_world_aabb(back_pad, elem="back_tail")
    with ctx.pose({back_hinge: 1.0}):
        raised_back = ctx.part_element_world_aabb(back_pad, elem="back_tail")
    ctx.check(
        "backrest raises upward",
        rest_back is not None and raised_back is not None and raised_back[1][2] > rest_back[1][2] + 0.18,
        details=f"rest={rest_back}, raised={raised_back}",
    )

    rest_seat = ctx.part_element_world_aabb(seat_pad, elem="seat_tail")
    with ctx.pose({seat_hinge: 0.42}):
        raised_seat = ctx.part_element_world_aabb(seat_pad, elem="seat_tail")
    ctx.check(
        "seat tilts upward from its front hinge",
        rest_seat is not None and raised_seat is not None and raised_seat[1][2] > rest_seat[1][2] + 0.05,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    ctx.expect_contact(
        leg_roller_0,
        frame,
        elem_a="roller_body",
        elem_b="roller_head",
        name="first leg roller stays seated on the front axle support",
    )
    ctx.expect_contact(
        leg_roller_1,
        frame,
        elem_a="roller_body",
        elem_b="roller_head",
        name="second leg roller stays seated on the front axle support",
    )
    ctx.expect_contact(
        wheel_0,
        frame,
        elem_a="wheel_tire",
        elem_b="wheel_bracket_0",
        name="first transport wheel stays seated on its axle bracket",
    )
    ctx.expect_contact(
        wheel_1,
        frame,
        elem_a="wheel_tire",
        elem_b="wheel_bracket_1",
        name="second transport wheel stays seated on its axle bracket",
    )

    rest_knob = ctx.part_world_position(selector_knob)
    with ctx.pose({selector_pull: 0.03}):
        pulled_knob = ctx.part_world_position(selector_knob)
    ctx.check(
        "pop pin knob pulls outward",
        rest_knob is not None and pulled_knob is not None and pulled_knob[1] > rest_knob[1] + 0.02,
        details=f"rest={rest_knob}, pulled={pulled_knob}",
    )

    return ctx.report()


object_model = build_object_model()
