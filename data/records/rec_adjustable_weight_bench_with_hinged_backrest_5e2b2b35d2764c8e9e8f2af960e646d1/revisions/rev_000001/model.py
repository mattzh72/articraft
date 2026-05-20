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


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name)


def _rounded_pad(width: float, length: float, thickness: float) -> cq.Workplane:
    pad = cq.Workplane("XY").box(
        width,
        length,
        thickness,
        centered=(True, True, False),
    )
    pad = pad.edges("|Z").fillet(min(0.025, width * 0.15, length * 0.06))
    pad = pad.faces(">Z").edges().fillet(min(0.010, thickness * 0.30))
    return pad


def _frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY")
    for size, xyz in (
        ((0.44, 0.08, 0.04), (0.0, -0.34, 0.0)),
        ((0.34, 0.08, 0.04), (0.0, 0.32, 0.0)),
        ((0.08, 0.72, 0.05), (0.0, -0.03, 0.135)),
        ((0.08, 0.15, 0.19), (0.0, -0.29, 0.04)),
        ((0.08, 0.14, 0.15), (0.0, 0.25, 0.04)),
        ((0.10, 0.16, 0.22), (0.0, 0.02, 0.18)),
        ((0.12, 0.16, 0.022), (0.0, -0.10, 0.376)),
        ((0.18, 0.12, 0.022), (0.0, 0.02, 0.376)),
        ((0.16, 0.08, 0.03), (0.0, 0.20, 0.175)),
        ((0.018, 0.04, 0.06), (-0.065, 0.20, 0.175)),
        ((0.018, 0.04, 0.06), (0.065, 0.20, 0.175)),
        ((0.022, 0.045, 0.04), (-0.095, 0.03, 0.355)),
        ((0.022, 0.045, 0.04), (0.095, 0.03, 0.355)),
    ):
        frame = frame.union(
            cq.Workplane("XY")
            .box(*size, centered=(True, True, False))
            .translate(xyz)
        )
    return frame


def _back_pad_shape() -> cq.Workplane:
    back = _rounded_pad(0.26, 0.72, 0.05).translate((0.0, 0.36, 0.002))
    ladder_rail = (
        cq.Workplane("XY")
        .box(0.034, 0.32, 0.055, centered=(True, True, False))
        .translate((0.0, 0.52, -0.053))
    )
    back = back.union(ladder_rail)
    for y_pos in (0.40, 0.48, 0.56, 0.64):
        back = back.union(
            cq.Workplane("XY")
            .box(0.056, 0.028, 0.024, centered=(True, True, False))
            .translate((0.0, y_pos, -0.077))
        )
    return back


def _seat_pad_shape() -> cq.Workplane:
    return _rounded_pad(0.26, 0.29, 0.05).translate((0.0, -0.145, 0.002))


def _rear_support_shape() -> cq.Workplane:
    support = cq.Workplane("XY")
    for x_pos in (-0.07, 0.07):
        support = support.union(
            cq.Workplane("XY")
            .box(0.018, 0.24, 0.028, centered=(True, False, True))
            .translate((x_pos, 0.12, 0.0))
        )
    support = support.union(
        cq.Workplane("XY")
        .box(0.158, 0.028, 0.028, centered=(True, True, True))
        .translate((0.0, 0.348, 0.0))
    )
    support = support.union(
        cq.Workplane("XY")
        .box(0.130, 0.052, 0.034, centered=(True, True, True))
        .translate((0.0, 0.288, 0.008))
    )
    support = support.union(
        cq.Workplane("YZ")
        .circle(0.016)
        .extrude(0.130, both=True)
        .translate((0.0, 0.348, 0.010))
    )
    return support


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apartment_gym_bench")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_rubber = model.material("knob_rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    frame = model.part("frame")
    for name, size, xyz in (
        ("front_foot", (0.44, 0.08, 0.04), (0.0, -0.34, 0.02)),
        ("rear_foot", (0.34, 0.08, 0.04), (0.0, 0.32, 0.02)),
        ("spine", (0.08, 0.72, 0.04), (0.0, -0.03, 0.145)),
        ("front_riser", (0.08, 0.15, 0.19), (0.0, -0.29, 0.135)),
        ("rear_riser", (0.08, 0.14, 0.15), (0.0, 0.285, 0.115)),
        ("hinge_tower", (0.10, 0.16, 0.22), (0.0, 0.02, 0.29)),
        ("tower_web", (0.08, 0.16, 0.035), (0.0, 0.02, 0.1775)),
        ("seat_bridge", (0.12, 0.16, 0.022), (0.0, -0.10, 0.387)),
        ("hinge_bridge", (0.18, 0.12, 0.022), (0.0, 0.02, 0.387)),
        ("pivot_bridge", (0.16, 0.08, 0.03), (0.0, 0.16, 0.19)),
        ("pivot_web", (0.08, 0.06, 0.035), (0.0, 0.15, 0.1625)),
        ("pivot_lug_0", (0.018, 0.04, 0.06), (-0.078, 0.18, 0.205)),
        ("pivot_lug_1", (0.018, 0.04, 0.06), (0.078, 0.18, 0.205)),
        ("hinge_lug_0", (0.022, 0.045, 0.04), (-0.095, 0.03, 0.375)),
        ("hinge_lug_1", (0.022, 0.045, 0.04), (0.095, 0.03, 0.375)),
        ("selector_mount", (0.05, 0.05, 0.05), (0.070, 0.10, 0.295)),
    ):
        frame.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_paint,
            name=name,
        )
    frame.visual(
        Box((0.282, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, -0.34, 0.080)),
        material=steel,
        name="front_axle",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.118),
        origin=Origin(xyz=(0.0, 0.20, 0.19), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_axle",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        _mesh(_rounded_pad(0.26, 0.72, 0.05), "bench_back_cushion"),
        origin=Origin(xyz=(0.0, 0.36, 0.002)),
        material=pad_vinyl,
        name="back_cushion",
    )
    for index, x_pos in enumerate((-0.095, 0.095)):
        back_pad.visual(
            Box((0.030, 0.030, 0.040)),
            origin=Origin(xyz=(x_pos, 0.015, 0.008)),
            material=steel,
            name=f"hinge_strap_{index}",
        )
    for index, x_pos in enumerate((-0.095, 0.095)):
        back_pad.visual(
            Box((0.024, 0.018, 0.030)),
            origin=Origin(xyz=(x_pos, 0.001, 0.015)),
            material=steel,
            name=f"hinge_tab_{index}",
        )
    back_pad.visual(
        Box((0.034, 0.32, 0.055)),
        origin=Origin(xyz=(0.0, 0.52, -0.0255)),
        material=steel,
        name="ladder_rail",
    )
    for index, y_pos in enumerate((0.40, 0.48, 0.56, 0.64)):
        back_pad.visual(
            Box((0.056, 0.028, 0.024)),
            origin=Origin(xyz=(0.0, y_pos, -0.065)),
            material=steel,
            name=f"ladder_tooth_{index}",
        )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        _mesh(_seat_pad_shape(), "bench_seat_pad"),
        material=pad_vinyl,
        name="seat_cushion",
    )

    rear_support = model.part("rear_support")
    for name, size, xyz in (
        ("support_arm_0", (0.018, 0.33, 0.028), (-0.070, 0.165, 0.0)),
        ("support_arm_1", (0.018, 0.33, 0.028), (0.070, 0.165, 0.0)),
        ("support_bridge", (0.158, 0.028, 0.028), (0.0, 0.340, 0.0)),
        ("support_head", (0.130, 0.052, 0.034), (0.0, 0.290, 0.008)),
    ):
        rear_support.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_paint,
            name=name,
        )
    rear_support.visual(
        Box((0.130, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.340, 0.010)),
        material=steel,
        name="support_roller",
    )
    rear_support.visual(
        Cylinder(radius=0.014, length=0.124),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="support_hub",
    )

    for index, x_pos in enumerate((-0.155, 0.155)):
        wheel = model.part(f"transport_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.036, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="hub",
        )
        model.articulation(
            f"frame_to_transport_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x_pos, -0.34, 0.080)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=20.0),
        )

    selector_body = model.part("selector_body")
    selector_body.visual(
        Box((0.010, 0.052, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.018)),
        material=frame_paint,
        name="selector_plate",
    )
    selector_body.visual(
        Box((0.010, 0.052, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, -0.018)),
        material=frame_paint,
        name="selector_plate_lower",
    )
    selector_body.visual(
        Box((0.032, 0.018, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, 0.014)),
        material=steel,
        name="selector_guide_top",
    )
    selector_body.visual(
        Box((0.032, 0.018, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, -0.014)),
        material=steel,
        name="selector_guide_bottom",
    )
    selector_body.visual(
        Box((0.012, 0.022, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=steel,
        name="selector_boss",
    )
    model.articulation(
        "frame_to_selector_body",
        ArticulationType.FIXED,
        parent=frame,
        child=selector_body,
        origin=Origin(xyz=(0.095, 0.10, 0.295)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="selector_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_rubber,
        name="selector_pull",
    )
    selector_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="selector_tip",
    )
    model.articulation(
        "selector_body_to_selector_knob",
        ArticulationType.PRISMATIC,
        parent=selector_body,
        child=selector_knob,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.10,
            lower=0.0,
            upper=0.018,
        ),
    )

    model.articulation(
        "frame_to_back_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.0, 0.08, 0.410)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "frame_to_seat_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(0.0, -0.05, 0.398)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=0.0,
            upper=0.34,
        ),
    )
    model.articulation(
        "frame_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_support,
        origin=Origin(xyz=(0.0, 0.20, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.4,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat_pad = object_model.get_part("seat_pad")
    rear_support = object_model.get_part("rear_support")
    selector_body = object_model.get_part("selector_body")
    selector_knob = object_model.get_part("selector_knob")
    back_joint = object_model.get_articulation("frame_to_back_pad")
    seat_joint = object_model.get_articulation("frame_to_seat_pad")
    support_joint = object_model.get_articulation("frame_to_rear_support")
    selector_joint = object_model.get_articulation("selector_body_to_selector_knob")

    ctx.allow_overlap(
        frame,
        rear_support,
        elem_a="pivot_axle",
        elem_b="support_hub",
        reason="The rear ladder support rotates around a shared pivot axle modeled inside the support hub.",
    )
    ctx.allow_overlap(
        frame,
        rear_support,
        elem_a="pivot_bridge",
        elem_b="support_hub",
        reason="The support hub nests into the lower pivot bridge around the rear support pivot.",
    )

    with ctx.pose({back_joint: 0.0, seat_joint: 0.0}):
        ctx.expect_gap(
            back_pad,
            seat_pad,
            axis="y",
            positive_elem="back_cushion",
            negative_elem="seat_cushion",
            min_gap=0.06,
            max_gap=0.14,
            name="seat and back pads keep a realistic split",
        )
        ctx.expect_gap(
            back_pad,
            frame,
            axis="z",
            positive_elem="back_cushion",
            negative_elem="hinge_bridge",
            min_gap=0.008,
            max_gap=0.025,
            name="back pad sits just above the frame",
        )
        ctx.expect_gap(
            seat_pad,
            frame,
            axis="z",
            positive_elem="seat_cushion",
            negative_elem="seat_bridge",
            min_gap=0.0,
            max_gap=0.010,
            name="seat pad sits just above the frame",
        )

    back_rest_aabb = ctx.part_world_aabb(back_pad)
    with ctx.pose({back_joint: 1.0}):
        back_raised_aabb = ctx.part_world_aabb(back_pad)
    ctx.check(
        "back pad raises upward",
        back_rest_aabb is not None
        and back_raised_aabb is not None
        and back_raised_aabb[1][2] > back_rest_aabb[1][2] + 0.18,
        details=f"rest={back_rest_aabb}, raised={back_raised_aabb}",
    )

    seat_rest_aabb = ctx.part_world_aabb(seat_pad)
    with ctx.pose({seat_joint: 0.30}):
        seat_raised_aabb = ctx.part_world_aabb(seat_pad)
    ctx.check(
        "seat pad tilts upward at the front",
        seat_rest_aabb is not None
        and seat_raised_aabb is not None
        and seat_raised_aabb[1][2] > seat_rest_aabb[1][2] + 0.03,
        details=f"rest={seat_rest_aabb}, raised={seat_raised_aabb}",
    )

    support_rest_aabb = ctx.part_world_aabb(rear_support)
    with ctx.pose({support_joint: 0.95}):
        support_raised_aabb = ctx.part_world_aabb(rear_support)
        ctx.expect_overlap(
            rear_support,
            back_pad,
            axes="xy",
            elem_a="support_head",
            elem_b="back_cushion",
            min_overlap=0.05,
            name="rear support stays under the back pad footprint",
        )
    ctx.check(
        "rear support swings upward",
        support_rest_aabb is not None
        and support_raised_aabb is not None
        and support_raised_aabb[1][2] > support_rest_aabb[1][2] + 0.20,
        details=f"rest={support_rest_aabb}, raised={support_raised_aabb}",
    )

    ctx.expect_origin_distance(
        selector_body,
        rear_support,
        axes="x",
        min_dist=0.09,
        name="selector stays laterally separate from the rear support",
    )

    selector_rest = ctx.part_world_position(selector_knob)
    with ctx.pose({selector_joint: 0.018}):
        selector_extended = ctx.part_world_position(selector_knob)
    ctx.check(
        "selector knob pulls outward",
        selector_rest is not None
        and selector_extended is not None
        and selector_extended[0] > selector_rest[0] + 0.012,
        details=f"rest={selector_rest}, extended={selector_extended}",
    )

    return ctx.report()


object_model = build_object_model()
