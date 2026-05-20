from __future__ import annotations

from math import atan2, cos, hypot, pi, sin, sqrt

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


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _rot_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = cos(angle)
    s = sin(angle)
    return (x * c + z * s, y, -x * s + z * c)


def _rotated_origin(local_xyz: tuple[float, float, float], angle: float) -> Origin:
    return Origin(xyz=_rot_y(local_xyz, angle), rpy=(0.0, angle, 0.0))


def _add_frame_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    height: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, height, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _pad_mesh(length: float, width: float, thickness: float, center: tuple[float, float, float]):
    edge_radius = min(width * 0.10, thickness * 0.35)
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(edge_radius)
        .translate(center)
    )


def _support_arm_mesh():
    arm_profile = [
        (0.00, -0.020),
        (0.06, -0.020),
        (0.16, 0.040),
        (0.28, 0.180),
        (0.34, 0.245),
        (0.27, 0.245),
        (0.12, 0.070),
        (0.00, 0.030),
    ]
    arm_plate = (
        cq.Workplane("XZ")
        .polyline(arm_profile)
        .close()
        .extrude(0.170)
        .translate((0.0, -0.085, 0.0))
    )
    pivot_barrel = cq.Workplane("XZ").circle(0.032).extrude(0.170).translate((0.0, -0.085, 0.0))
    top_saddle = cq.Workplane("XY").box(0.085, 0.170, 0.045).translate((0.315, 0.0, 0.255))
    knob_boss = cq.Workplane("XZ").circle(0.018).extrude(0.030).translate((0.180, 0.085, 0.080))
    return arm_plate.union(pivot_barrel).union(top_saddle).union(knob_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_adjustable_bench")

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    upholstery = model.material("upholstery", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    selector_red = model.material("selector_red", rgba=(0.78, 0.12, 0.10, 1.0))
    selector_yellow = model.material("selector_yellow", rgba=(0.93, 0.75, 0.14, 1.0))

    backrest_rest = 0.34
    seat_rest = 0.06
    support_rest = -0.60

    frame = model.part("frame")

    _add_frame_member(
        frame,
        (0.44, -0.28, 0.035),
        (0.44, 0.28, 0.035),
        width=0.080,
        height=0.040,
        material=frame_paint,
        name="front_foot",
    )
    _add_frame_member(
        frame,
        (-0.64, -0.24, 0.035),
        (-0.64, 0.24, 0.035),
        width=0.080,
        height=0.040,
        material=frame_paint,
        name="rear_foot",
    )
    _add_frame_member(
        frame,
        (0.43, -0.16, 0.055),
        (-0.63, -0.15, 0.055),
        width=0.055,
        height=0.030,
        material=frame_paint,
        name="side_rail_0",
    )
    _add_frame_member(
        frame,
        (0.43, 0.16, 0.055),
        (-0.63, 0.15, 0.055),
        width=0.055,
        height=0.030,
        material=frame_paint,
        name="side_rail_1",
    )
    _add_frame_member(
        frame,
        (0.43, 0.0, 0.055),
        (-0.16, 0.0, 0.355),
        width=0.055,
        height=0.032,
        material=frame_paint,
        name="center_spine",
    )
    _add_frame_member(
        frame,
        (0.16, -0.15, 0.055),
        (0.22, -0.15, 0.370),
        width=0.050,
        height=0.028,
        material=frame_paint,
        name="seat_upright_0",
    )
    _add_frame_member(
        frame,
        (0.16, 0.15, 0.055),
        (0.22, 0.15, 0.370),
        width=0.050,
        height=0.028,
        material=frame_paint,
        name="seat_upright_1",
    )
    _add_frame_member(
        frame,
        (0.19, -0.16, 0.342),
        (0.19, 0.16, 0.342),
        width=0.055,
        height=0.030,
        material=frame_paint,
        name="seat_hinge_tube",
    )
    _add_frame_member(
        frame,
        (-0.18, -0.13, 0.055),
        (-0.16, -0.13, 0.410),
        width=0.050,
        height=0.028,
        material=frame_paint,
        name="back_upright_0",
    )
    _add_frame_member(
        frame,
        (-0.18, 0.13, 0.055),
        (-0.16, 0.13, 0.410),
        width=0.050,
        height=0.028,
        material=frame_paint,
        name="back_upright_1",
    )
    _add_frame_member(
        frame,
        (-0.20, -0.14, 0.355),
        (-0.20, 0.14, 0.355),
        width=0.055,
        height=0.030,
        material=frame_paint,
        name="back_bridge",
    )
    _add_frame_member(
        frame,
        (-0.44, -0.11, 0.130),
        (-0.44, 0.11, 0.130),
        width=0.050,
        height=0.028,
        material=frame_paint,
        name="support_pivot_tube",
    )
    _add_frame_member(
        frame,
        (-0.54, -0.15, 0.055),
        (-0.40, -0.11, 0.155),
        width=0.045,
        height=0.026,
        material=frame_paint,
        name="support_brace_0",
    )
    _add_frame_member(
        frame,
        (-0.54, 0.15, 0.055),
        (-0.40, 0.11, 0.155),
        width=0.045,
        height=0.026,
        material=frame_paint,
        name="support_brace_1",
    )
    _add_frame_member(
        frame,
        (-0.64, 0.22, 0.055),
        (-0.64, 0.283, 0.065),
        width=0.034,
        height=0.024,
        material=steel,
        name="wheel_stub_0",
    )
    _add_frame_member(
        frame,
        (-0.64, -0.22, 0.055),
        (-0.64, -0.283, 0.065),
        width=0.034,
        height=0.024,
        material=steel,
        name="wheel_stub_1",
    )
    frame.visual(
        Box((0.080, 0.110, 0.180)),
        origin=Origin(xyz=(-0.60, 0.0, 0.120)),
        material=frame_paint,
        name="rear_gusset",
    )
    for y_sign in (-1.0, 1.0):
        frame.visual(
            Cylinder(radius=0.014, length=0.060),
            origin=Origin(xyz=(0.22, y_sign * 0.120, 0.370), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"seat_pivot_pin_{int(y_sign > 0)}",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.060),
            origin=Origin(xyz=(-0.16, y_sign * 0.120, 0.410), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"back_pivot_pin_{int(y_sign > 0)}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(-0.40, y_sign * 0.085, 0.155), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"support_pivot_pin_{int(y_sign > 0)}",
        )
    for y_pos in (-0.26, 0.26):
        frame.visual(
            Box((0.070, 0.030, 0.012)),
            origin=Origin(xyz=(0.44, y_pos, 0.018)),
            material=frame_paint,
            name=f"front_floor_pad_{int(y_pos > 0)}",
        )
    for y_pos in (-0.22, 0.22):
        frame.visual(
            Box((0.070, 0.030, 0.012)),
            origin=Origin(xyz=(-0.64, y_pos, 0.018)),
            material=frame_paint,
            name=f"rear_floor_pad_{int(y_pos > 0)}",
        )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_pad_mesh(0.310, 0.320, 0.070, (-0.155, 0.0, 0.055)), "seat_pad"),
        origin=Origin(rpy=(0.0, seat_rest, 0.0)),
        material=upholstery,
        name="seat_pad",
    )
    seat.visual(
        Box((0.250, 0.240, 0.020)),
        origin=_rotated_origin((-0.145, 0.0, 0.018), seat_rest),
        material=steel,
        name="seat_pan",
    )
    seat.visual(
        Box((0.090, 0.190, 0.024)),
        origin=_rotated_origin((-0.245, 0.0, 0.012), seat_rest),
        material=steel,
        name="seat_rear_brace",
    )
    seat.visual(
        Cylinder(radius=0.018, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="seat_hinge_barrel",
    )

    backrest = model.part("backrest")
    backrest.visual(
        mesh_from_cadquery(_pad_mesh(0.860, 0.320, 0.065, (-0.430, 0.0, 0.055)), "backrest_pad"),
        origin=Origin(rpy=(0.0, backrest_rest, 0.0)),
        material=upholstery,
        name="backrest_pad",
    )
    backrest.visual(
        Box((0.680, 0.260, 0.020)),
        origin=_rotated_origin((-0.380, 0.0, 0.018), backrest_rest),
        material=steel,
        name="backrest_frame",
    )
    backrest.visual(
        Box((0.240, 0.110, 0.020)),
        origin=_rotated_origin((-0.175, 0.0, 0.000), backrest_rest),
        material=steel,
        name="index_plate",
    )
    backrest.visual(
        Cylinder(radius=0.018, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="backrest_hinge_barrel",
    )

    support_arm = model.part("support_arm")
    support_arm.visual(
        Box((0.235, 0.165, 0.030)),
        origin=_rotated_origin((0.110, 0.0, 0.030), support_rest),
        material=frame_paint,
        name="arm_body",
    )
    support_arm.visual(
        Box((0.100, 0.170, 0.038)),
        origin=_rotated_origin((0.205, 0.0, 0.070), support_rest),
        material=frame_paint,
        name="arm_head",
    )
    support_arm.visual(
        Cylinder(radius=0.022, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    support_arm.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=_rot_y((0.155, 0.085, 0.058), support_rest), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_boss",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=selector_yellow,
        name="knob_collar",
    )
    selector_knob.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_shaft",
    )
    selector_knob.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        material=selector_red,
        name="knob_cap",
    )

    for wheel_name, side_y in (("wheel_0", 0.295), ("wheel_1", -0.295)):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.034, length=0.018),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.0035, length=0.012),
            origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=selector_yellow,
            name="valve_stem",
        )
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.64, side_y, 0.065)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.220, 0.0, 0.370)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-0.08, upper=0.25),
    )
    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.160, 0.0, 0.410)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "frame_to_support_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.400, 0.0, 0.155)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.0, lower=-0.20, upper=0.45),
    )
    model.articulation(
        "support_arm_to_selector_knob",
        ArticulationType.PRISMATIC,
        parent=support_arm,
        child=selector_knob,
        origin=Origin(xyz=_rot_y((0.155, 0.098, 0.058), support_rest)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.12, lower=0.0, upper=0.025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    support_arm = object_model.get_part("support_arm")
    selector_knob = object_model.get_part("selector_knob")
    wheel = object_model.get_part("wheel_0")

    seat_hinge = object_model.get_articulation("frame_to_seat")
    backrest_hinge = object_model.get_articulation("frame_to_backrest")
    support_pivot = object_model.get_articulation("frame_to_support_arm")
    selector_slide = object_model.get_articulation("support_arm_to_selector_knob")
    wheel_spin = object_model.get_articulation("wheel_0_spin")

    for pin_name in ("seat_pivot_pin_0", "seat_pivot_pin_1"):
        ctx.allow_overlap(
            "frame",
            "seat",
            elem_a=pin_name,
            elem_b="seat_hinge_barrel",
            reason="The frame-side hinge pins are intentionally modeled as solid axle stubs passing into the seat hinge barrel.",
        )
    for pin_name in ("back_pivot_pin_0", "back_pivot_pin_1"):
        ctx.allow_overlap(
            "frame",
            "backrest",
            elem_a=pin_name,
            elem_b="backrest_hinge_barrel",
            reason="The backrest hinge is represented with solid pivot pins seated inside the barrel sleeve.",
        )
    for pin_name in ("support_pivot_pin_0", "support_pivot_pin_1"):
        ctx.allow_overlap(
            "frame",
            "support_arm",
            elem_a=pin_name,
            elem_b="pivot_barrel",
            reason="The support arm pivots on solid frame pins seated inside the arm barrel.",
        )

    def top_z(part, *, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else float(aabb[1][2])

    def center_of_elem(part, *, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return (
            float((aabb[0][0] + aabb[1][0]) * 0.5),
            float((aabb[0][1] + aabb[1][1]) * 0.5),
            float((aabb[0][2] + aabb[1][2]) * 0.5),
        )

    seat_limits = seat_hinge.motion_limits
    if seat_limits is not None and seat_limits.lower is not None and seat_limits.upper is not None:
        with ctx.pose({seat_hinge: seat_limits.lower}):
            seat_low = top_z(seat, elem="seat_pad")
        with ctx.pose({seat_hinge: seat_limits.upper}):
            seat_high = top_z(seat, elem="seat_pad")
        ctx.check(
            "seat rear rises through hinge range",
            seat_low is not None and seat_high is not None and seat_high > seat_low + 0.06,
            details=f"low_top_z={seat_low}, high_top_z={seat_high}",
        )

    backrest_limits = backrest_hinge.motion_limits
    if backrest_limits is not None and backrest_limits.lower is not None and backrest_limits.upper is not None:
        with ctx.pose({backrest_hinge: backrest_limits.lower}):
            backrest_low = top_z(backrest, elem="backrest_pad")
        with ctx.pose({backrest_hinge: backrest_limits.upper}):
            backrest_high = top_z(backrest, elem="backrest_pad")
        ctx.check(
            "backrest opens upward",
            backrest_low is not None and backrest_high is not None and backrest_high > backrest_low + 0.30,
            details=f"low_top_z={backrest_low}, high_top_z={backrest_high}",
        )

    support_limits = support_pivot.motion_limits
    if support_limits is not None and support_limits.lower is not None and support_limits.upper is not None:
        with ctx.pose({support_pivot: support_limits.lower}):
            support_low = top_z(support_arm, elem="arm_head")
        with ctx.pose({support_pivot: support_limits.upper}):
            support_high = top_z(support_arm, elem="arm_head")
        ctx.check(
            "support arm swings higher at upper limit",
            support_low is not None and support_high is not None and support_high > support_low + 0.05,
            details=f"low_top_z={support_low}, high_top_z={support_high}",
        )

    knob_limits = selector_slide.motion_limits
    if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
        with ctx.pose({selector_slide: knob_limits.lower}):
            knob_retracted = ctx.part_world_position(selector_knob)
        with ctx.pose({selector_slide: knob_limits.upper}):
            knob_extended = ctx.part_world_position(selector_knob)
        ctx.check(
            "selector knob pulls outward",
            knob_retracted is not None
            and knob_extended is not None
            and knob_extended[1] > knob_retracted[1] + 0.020,
            details=f"retracted={knob_retracted}, extended={knob_extended}",
        )

    with ctx.pose({wheel_spin: 0.0}):
        stem_rest = center_of_elem(wheel, elem="valve_stem")
    with ctx.pose({wheel_spin: pi / 2.0}):
        stem_turned = center_of_elem(wheel, elem="valve_stem")
    ctx.check(
        "transport wheel visibly spins",
        stem_rest is not None
        and stem_turned is not None
        and abs(stem_turned[2] - stem_rest[2]) > 0.035,
        details=f"rest={stem_rest}, turned={stem_turned}",
    )

    return ctx.report()


object_model = build_object_model()
