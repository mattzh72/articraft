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


def _pad_mesh(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    center_x: float,
    center_z: float,
    edge_radius: float,
):
    shape = (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges()
        .fillet(edge_radius)
        .translate((center_x, 0.0, center_z))
    )
    return mesh_from_cadquery(shape, name)


def _add_roller_visuals(part, *, foam, steel) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.055, length=0.320),
        origin=spin_origin,
        material=foam,
        name="foam",
    )
    part.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, 0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="end_cap_0",
    )
    part.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, -0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="end_cap_1",
    )


def _add_wheel_visuals(part, *, rubber, steel) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=spin_origin,
        material=steel,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap_0",
    )
    part.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decline_bench")

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.19, 0.20, 1.0))
    upholstery = model.material("upholstery", rgba=(0.10, 0.10, 0.11, 1.0))
    backer = model.material("backer", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    foam = model.material("foam", rgba=(0.13, 0.13, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.16, 0.52, 0.14)),
        origin=Origin(xyz=(-0.56, 0.0, 0.07)),
        material=powder_coat,
        name="rear_foot",
    )
    frame.visual(
        Box((0.10, 0.05, 0.12)),
        origin=Origin(xyz=(-0.50, 0.18, 0.12)),
        material=powder_coat,
        name="rear_riser_0",
    )
    frame.visual(
        Box((0.10, 0.05, 0.12)),
        origin=Origin(xyz=(-0.50, -0.18, 0.12)),
        material=powder_coat,
        name="rear_riser_1",
    )
    frame.visual(
        Box((1.02, 0.05, 0.05)),
        origin=Origin(xyz=(-0.05, 0.18, 0.16), rpy=(0.0, 0.12, 0.0)),
        material=powder_coat,
        name="rail_0",
    )
    frame.visual(
        Box((1.02, 0.05, 0.05)),
        origin=Origin(xyz=(-0.05, -0.18, 0.16), rpy=(0.0, 0.12, 0.0)),
        material=powder_coat,
        name="rail_1",
    )
    frame.visual(
        Box((0.22, 0.05, 0.05)),
        origin=Origin(xyz=(-0.43, 0.18, 0.155), rpy=(0.0, 0.12, 0.0)),
        material=powder_coat,
        name="rear_brace_0",
    )
    frame.visual(
        Box((0.22, 0.05, 0.05)),
        origin=Origin(xyz=(-0.43, -0.18, 0.155), rpy=(0.0, 0.12, 0.0)),
        material=powder_coat,
        name="rear_brace_1",
    )
    frame.visual(
        Box((0.12, 0.05, 0.10)),
        origin=Origin(xyz=(-0.40, 0.18, 0.17)),
        material=powder_coat,
        name="rear_gusset_0",
    )
    frame.visual(
        Box((0.12, 0.05, 0.10)),
        origin=Origin(xyz=(-0.40, -0.18, 0.17)),
        material=powder_coat,
        name="rear_gusset_1",
    )
    frame.visual(
        Box((0.16, 0.42, 0.06)),
        origin=Origin(xyz=(0.03, 0.0, 0.215)),
        material=powder_coat,
        name="back_bridge",
    )
    frame.visual(
        Box((0.08, 0.14, 0.12)),
        origin=Origin(xyz=(0.03, 0.0, 0.29)),
        material=powder_coat,
        name="back_support",
    )
    frame.visual(
        Box((0.14, 0.40, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.235)),
        material=powder_coat,
        name="seat_bridge",
    )
    frame.visual(
        Box((0.08, 0.14, 0.12)),
        origin=Origin(xyz=(0.18, 0.0, 0.32)),
        material=powder_coat,
        name="seat_support",
    )
    frame.visual(
        Box((0.18, 0.38, 0.06)),
        origin=Origin(xyz=(0.47, 0.0, 0.07)),
        material=powder_coat,
        name="front_base",
    )
    frame.visual(
        Box((0.08, 0.05, 0.44)),
        origin=Origin(xyz=(0.40, 0.18, 0.26)),
        material=powder_coat,
        name="upright_0",
    )
    frame.visual(
        Box((0.08, 0.05, 0.44)),
        origin=Origin(xyz=(0.40, -0.18, 0.26)),
        material=powder_coat,
        name="upright_1",
    )
    frame.visual(
        Box((0.10, 0.36, 0.06)),
        origin=Origin(xyz=(0.45, 0.0, 0.64)),
        material=powder_coat,
        name="roller_yoke",
    )
    frame.visual(
        Box((0.10, 0.02, 0.34)),
        origin=Origin(xyz=(0.49, 0.18, 0.49)),
        material=powder_coat,
        name="roller_cheek_0",
    )
    frame.visual(
        Box((0.10, 0.02, 0.34)),
        origin=Origin(xyz=(0.49, -0.18, 0.49)),
        material=powder_coat,
        name="roller_cheek_1",
    )
    frame.visual(
        Box((0.16, 0.03, 0.03)),
        origin=Origin(xyz=(0.60, 0.06, 0.115)),
        material=powder_coat,
        name="handle_stem_0",
    )
    frame.visual(
        Box((0.16, 0.03, 0.03)),
        origin=Origin(xyz=(0.60, -0.06, 0.115)),
        material=powder_coat,
        name="handle_stem_1",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.16),
        origin=Origin(xyz=(0.68, 0.0, 0.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="handle_grip",
    )
    frame.visual(
        Box((0.06, 0.03, 0.08)),
        origin=Origin(xyz=(0.50, 0.195, 0.09)),
        material=powder_coat,
        name="wheel_bracket_0",
    )
    frame.visual(
        Box((0.06, 0.03, 0.08)),
        origin=Origin(xyz=(0.50, -0.195, 0.09)),
        material=powder_coat,
        name="wheel_bracket_1",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        _pad_mesh(
            "back_pad_shell",
            length=0.98,
            width=0.30,
            thickness=0.065,
            center_x=-0.39,
            center_z=0.046,
            edge_radius=0.012,
        ),
        material=upholstery,
        name="pad_shell",
    )
    back_pad.visual(
        Box((0.78, 0.24, 0.012)),
        origin=Origin(xyz=(-0.43, 0.0, 0.0075)),
        material=backer,
        name="backer_plate",
    )
    back_pad.visual(
        Box((0.08, 0.12, 0.018)),
        origin=Origin(xyz=(-0.08, 0.0, -0.006)),
        material=backer,
        name="mount_block",
    )

    seat = model.part("seat")
    seat.visual(
        _pad_mesh(
            "seat_pad_shell",
            length=0.34,
            width=0.30,
            thickness=0.065,
            center_x=0.14,
            center_z=0.046,
            edge_radius=0.012,
        ),
        material=upholstery,
        name="pad_shell",
    )
    seat.visual(
        Box((0.12, 0.24, 0.012)),
        origin=Origin(xyz=(0.20, 0.0, 0.0075)),
        material=backer,
        name="backer_plate",
    )
    seat.visual(
        Box((0.09, 0.12, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=backer,
        name="mount_block",
    )

    leg_roller_0 = model.part("leg_roller_0")
    _add_roller_visuals(leg_roller_0, foam=foam, steel=steel)

    leg_roller_1 = model.part("leg_roller_1")
    _add_roller_visuals(leg_roller_1, foam=foam, steel=steel)

    front_wheel_0 = model.part("front_wheel_0")
    _add_wheel_visuals(front_wheel_0, rubber=rubber, steel=steel)

    front_wheel_1 = model.part("front_wheel_1")
    _add_wheel_visuals(front_wheel_1, rubber=rubber, steel=steel)

    model.articulation(
        "frame_to_back_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.03, 0.0, 0.35), rpy=(0.0, -math.radians(10.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-0.25,
            upper=0.85,
        ),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.18, 0.0, 0.375), rpy=(0.0, math.radians(9.0), 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-0.20,
            upper=0.45,
        ),
    )
    model.articulation(
        "frame_to_leg_roller_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=leg_roller_0,
        origin=Origin(xyz=(0.50, 0.0, 0.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_leg_roller_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=leg_roller_1,
        origin=Origin(xyz=(0.58, 0.0, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_front_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel_0,
        origin=Origin(xyz=(0.53, 0.23, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_front_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel_1,
        origin=Origin(xyz=(0.53, -0.23, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat = object_model.get_part("seat")
    leg_roller_0 = object_model.get_part("leg_roller_0")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")

    back_hinge = object_model.get_articulation("frame_to_back_pad")
    seat_hinge = object_model.get_articulation("frame_to_seat")

    ctx.allow_overlap(
        back_pad,
        frame,
        elem_a="mount_block",
        elem_b="back_support",
        reason="The hidden back hinge is simplified as a nested mount block seated inside the frame hinge cradle.",
    )
    ctx.allow_overlap(
        seat,
        frame,
        elem_a="mount_block",
        elem_b="seat_support",
        reason="The hidden seat hinge is simplified as a nested mount block seated inside the frame hinge cradle.",
    )

    ctx.expect_origin_gap(
        seat,
        back_pad,
        axis="x",
        min_gap=0.12,
        max_gap=0.22,
        name="seat hinge sits ahead of back hinge",
    )
    ctx.expect_origin_gap(
        leg_roller_0,
        seat,
        axis="x",
        min_gap=0.28,
        max_gap=0.40,
        name="leg roller assembly sits ahead of seat",
    )
    ctx.expect_origin_gap(
        leg_roller_0,
        front_wheel_0,
        axis="z",
        min_gap=0.40,
        max_gap=0.55,
        name="leg roller assembly stands above transport wheel height",
    )
    ctx.expect_overlap(
        back_pad,
        seat,
        axes="y",
        min_overlap=0.26,
        name="pads share a workout bench width",
    )

    back_rest_aabb = ctx.part_element_world_aabb(back_pad, elem="pad_shell")
    if back_hinge.motion_limits is not None and back_hinge.motion_limits.upper is not None:
        with ctx.pose({back_hinge: back_hinge.motion_limits.upper}):
            back_raised_aabb = ctx.part_element_world_aabb(back_pad, elem="pad_shell")
        ctx.check(
            "back pad raises at upper limit",
            back_rest_aabb is not None
            and back_raised_aabb is not None
            and back_raised_aabb[1][2] > back_rest_aabb[1][2] + 0.12,
            details=f"rest={back_rest_aabb}, raised={back_raised_aabb}",
        )

    seat_rest_aabb = ctx.part_element_world_aabb(seat, elem="pad_shell")
    if seat_hinge.motion_limits is not None and seat_hinge.motion_limits.upper is not None:
        with ctx.pose({seat_hinge: seat_hinge.motion_limits.upper}):
            seat_raised_aabb = ctx.part_element_world_aabb(seat, elem="pad_shell")
        ctx.check(
            "seat front lifts at upper limit",
            seat_rest_aabb is not None
            and seat_raised_aabb is not None
            and seat_raised_aabb[1][2] > seat_rest_aabb[1][2] + 0.05,
            details=f"rest={seat_rest_aabb}, raised={seat_raised_aabb}",
        )

    handle_aabb = ctx.part_element_world_aabb(frame, elem="handle_grip")
    wheel_0_aabb = ctx.part_world_aabb(front_wheel_0)
    wheel_1_aabb = ctx.part_world_aabb(front_wheel_1)
    handle_center_y = None if handle_aabb is None else (handle_aabb[0][1] + handle_aabb[1][1]) * 0.5
    wheel_front_x = None
    if wheel_0_aabb is not None and wheel_1_aabb is not None:
        wheel_front_x = max(wheel_0_aabb[1][0], wheel_1_aabb[1][0])
    ctx.check(
        "carry handle projects ahead of wheel set and stays centered",
        handle_aabb is not None
        and wheel_front_x is not None
        and handle_center_y is not None
        and abs(handle_center_y) < 0.01
        and handle_aabb[1][0] > wheel_front_x + 0.08,
        details=f"handle={handle_aabb}, wheel_front_x={wheel_front_x}, handle_center_y={handle_center_y}",
    )

    for joint_name in (
        "frame_to_leg_roller_0",
        "frame_to_leg_roller_1",
        "frame_to_front_wheel_0",
        "frame_to_front_wheel_1",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} spins around width axis",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
