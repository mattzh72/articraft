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


def _rounded_pad(length: float, width: float, thickness: float, radius: float, name: str):
    """A single upholstered pad with rounded plan corners."""
    solid = cq.Workplane("XY").box(length, width, thickness).edges("|Z").fillet(radius)
    return mesh_from_cadquery(solid, name, tolerance=0.0015, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_incline_utility_bench")

    frame_mat = model.material("powder_coated_black", rgba=(0.02, 0.022, 0.024, 1.0))
    pin_mat = model.material("brushed_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    pad_mat = model.material("black_vinyl", rgba=(0.015, 0.014, 0.013, 1.0))
    seam_mat = model.material("slightly_gloss_seam", rgba=(0.035, 0.035, 0.034, 1.0))
    rubber_mat = model.material("matte_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    hinge_z = 0.405
    tube_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    base = model.part("base")

    # Lower rectangular transport frame.
    for y in (-0.24, 0.24):
        base.visual(
            Box((1.70, 0.045, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.10)),
            material=frame_mat,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    for x, name in ((0.80, "front_crossmember"), (-0.80, "rear_crossmember"), (0.06, "center_crossmember")):
        base.visual(
            Box((0.080, 0.585, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.10)),
            material=frame_mat,
            name=name,
        )

    # Lift handle: two projecting arms welded to the front crossmember and a grip between the wheels.
    for y in (-0.055, 0.055):
        base.visual(
            Box((0.205, 0.030, 0.035)),
            origin=Origin(xyz=(0.932, y, 0.120)),
            material=frame_mat,
            name=f"front_handle_arm_{0 if y < 0 else 1}",
        )
    base.visual(
        Box((0.045, 0.185, 0.038)),
        origin=Origin(xyz=(1.055, 0.0, 0.120)),
        material=frame_mat,
        name="front_handle_grip",
    )

    # Small transport wheels tied together by a visible axle.
    base.visual(
        Cylinder(radius=0.012, length=0.66),
        origin=Origin(xyz=(0.82, 0.0, 0.070), rpy=tube_y.rpy),
        material=pin_mat,
        name="front_axle",
    )
    for y in (-0.315, 0.315):
        side = 0 if y < 0 else 1
        base.visual(
            Cylinder(radius=0.068, length=0.048),
            origin=Origin(xyz=(0.82, y, 0.070), rpy=tube_y.rpy),
            material=rubber_mat,
            name=f"wheel_tire_{side}",
        )
        base.visual(
            Cylinder(radius=0.032, length=0.054),
            origin=Origin(xyz=(0.82, y, 0.070), rpy=tube_y.rpy),
            material=pin_mat,
            name=f"wheel_hub_{side}",
        )

    # Upper support rails and diagonal braces carrying the pad hinges.
    for y in (-0.18, 0.18):
        side = 0 if y < 0 else 1
        base.visual(
            Box((0.76, 0.035, 0.035)),
            origin=Origin(xyz=(0.20, y, 0.360)),
            material=frame_mat,
            name=f"upper_rail_{side}",
        )
        base.visual(
            Box((0.72, 0.032, 0.032)),
            origin=Origin(xyz=(0.325, y, 0.235), rpy=(0.0, 0.395, 0.0)),
            material=frame_mat,
            name=f"front_brace_{side}",
        )
        base.visual(
            Box((0.80, 0.032, 0.032)),
            origin=Origin(xyz=(-0.415, y, 0.235), rpy=(0.0, -0.345, 0.0)),
            material=frame_mat,
            name=f"rear_brace_{side}",
        )

    # Hinge clevis plates and pins at the two pad pivots.
    for x, prefix in ((0.55, "seat"), (0.03, "back")):
        for y in (-0.205, 0.205):
            side = 0 if y < 0 else 1
            base.visual(
                Box((0.075, 0.030, 0.125)),
                origin=Origin(xyz=(x, y, 0.365)),
                material=frame_mat,
                name=f"{prefix}_hinge_plate_{side}",
            )
        base.visual(
            Cylinder(radius=0.008, length=0.49),
            origin=Origin(xyz=(x, 0.0, hinge_z), rpy=tube_y.rpy),
            material=pin_mat,
            name=f"{prefix}_hinge_pin",
        )

    # Lower rear pivot for the adjustable ladder support.
    for y in (-0.226, 0.226):
        side = 0 if y < 0 else 1
        base.visual(
            Box((0.060, 0.032, 0.130)),
            origin=Origin(xyz=(-0.82, y, 0.130)),
            material=frame_mat,
            name=f"rear_pivot_plate_{side}",
        )
    for y in (-0.251, 0.251):
        side = 0 if y < 0 else 1
        base.visual(
            Cylinder(radius=0.012, length=0.040),
            origin=Origin(xyz=(-0.82, y, 0.150), rpy=tube_y.rpy),
            material=pin_mat,
            name=f"rear_pivot_cap_{side}",
        )

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_pad(0.96, 0.320, 0.075, 0.035, "backrest_pad_mesh"),
        origin=Origin(xyz=(-0.500, 0.0, 0.057)),
        material=pad_mat,
        name="back_pad",
    )
    backrest.visual(
        Box((0.008, 0.285, 0.004)),
        origin=Origin(xyz=(-0.500, 0.0, 0.096)),
        material=seam_mat,
        name="back_center_seam",
    )
    backrest.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(rpy=tube_y.rpy),
        material=pin_mat,
        name="back_hinge_sleeve",
    )
    backrest.visual(
        Box((0.060, 0.315, 0.020)),
        origin=Origin(xyz=(-0.030, 0.0, 0.018)),
        material=pin_mat,
        name="back_hinge_leaf",
    )
    for y in (-0.128, 0.128):
        side = 0 if y < 0 else 1
        backrest.visual(
            Box((0.620, 0.026, 0.024)),
            origin=Origin(xyz=(-0.555, y, 0.012)),
            material=frame_mat,
            name=f"adjust_bracket_rail_{side}",
        )
    for i, x in enumerate((-0.360, -0.500, -0.640, -0.780)):
        backrest.visual(
            Box((0.035, 0.300, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.004)),
            material=pin_mat,
            name=f"adjust_ladder_catch_{i}",
        )

    seat = model.part("seat")
    seat.visual(
        _rounded_pad(0.490, 0.330, 0.075, 0.035, "seat_pad_mesh"),
        origin=Origin(xyz=(-0.245, 0.0, 0.057)),
        material=pad_mat,
        name="seat_pad",
    )
    seat.visual(
        Box((0.006, 0.295, 0.004)),
        origin=Origin(xyz=(-0.245, 0.0, 0.096)),
        material=seam_mat,
        name="seat_center_seam",
    )
    seat.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(rpy=tube_y.rpy),
        material=pin_mat,
        name="seat_hinge_sleeve",
    )
    seat.visual(
        Box((0.060, 0.315, 0.020)),
        origin=Origin(xyz=(-0.030, 0.0, 0.018)),
        material=pin_mat,
        name="seat_hinge_leaf",
    )

    support = model.part("support_ladder")
    support.visual(
        Cylinder(radius=0.018, length=0.420),
        origin=Origin(rpy=tube_y.rpy),
        material=pin_mat,
        name="ladder_pivot_sleeve",
    )
    for y in (-0.195, 0.195):
        side = 0 if y < 0 else 1
        support.visual(
            Box((0.030, 0.030, 0.425)),
            origin=Origin(xyz=(-0.140, y, 0.160), rpy=(0.0, -0.720, 0.0)),
            material=frame_mat,
            name=f"ladder_side_rail_{side}",
        )
    for i, (x, z) in enumerate(((-0.070, 0.080), (-0.154, 0.176), (-0.238, 0.272))):
        support.visual(
            Box((0.038, 0.430, 0.026)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=pin_mat,
            name=f"ladder_rung_{i}",
        )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=Origin(xyz=(0.03, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=0.0, upper=1.22),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.55, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.36),
    )
    model.articulation(
        "support_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support,
        origin=Origin(xyz=(-0.82, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.1, lower=0.0, upper=0.78),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support = object_model.get_part("support_ladder")
    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    support_pivot = object_model.get_articulation("support_pivot")

    for link, sleeve, pin, label in (
        (backrest, "back_hinge_sleeve", "back_hinge_pin", "backrest hinge"),
        (seat, "seat_hinge_sleeve", "seat_hinge_pin", "seat hinge"),
    ):
        ctx.allow_overlap(
            base,
            link,
            elem_a=pin,
            elem_b=sleeve,
            reason=f"The {label} sleeve is intentionally captured around a steel pivot pin.",
        )
        ctx.expect_overlap(
            base,
            link,
            axes="xyz",
            elem_a=pin,
            elem_b=sleeve,
            min_overlap=0.010,
            name=f"{label} pin is captured inside sleeve",
        )

    sleeve_aabb = ctx.part_element_world_aabb(support, elem="ladder_pivot_sleeve")
    rear_plate_0 = ctx.part_element_world_aabb(base, elem="rear_pivot_plate_0")
    rear_plate_1 = ctx.part_element_world_aabb(base, elem="rear_pivot_plate_1")
    ctx.check(
        "rear ladder pivot sleeve sits between clevis plates",
        sleeve_aabb is not None
        and rear_plate_0 is not None
        and rear_plate_1 is not None
        and abs(rear_plate_0[1][1] - sleeve_aabb[0][1]) < 0.002
        and abs(rear_plate_1[0][1] - sleeve_aabb[1][1]) < 0.002,
        details=f"sleeve={sleeve_aabb}, plate_0={rear_plate_0}, plate_1={rear_plate_1}",
    )

    front_crossmember = ctx.part_element_world_aabb(base, elem="front_crossmember")
    handle_grip = ctx.part_element_world_aabb(base, elem="front_handle_grip")
    handle_arm = ctx.part_element_world_aabb(base, elem="front_handle_arm_0")
    ctx.check(
        "front carry handle projects from crossmember",
        front_crossmember is not None
        and handle_grip is not None
        and handle_arm is not None
        and handle_arm[0][0] <= front_crossmember[1][0] + 0.015
        and handle_grip[0][0] > front_crossmember[1][0] + 0.16,
        details=f"front_crossmember={front_crossmember}, handle_arm={handle_arm}, handle_grip={handle_grip}",
    )

    back_bracket = ctx.part_element_world_aabb(backrest, elem="adjust_bracket_rail_0")
    support_rest = ctx.part_world_aabb(support)
    ctx.check(
        "support ladder sits behind backrest bracket",
        back_bracket is not None and support_rest is not None and support_rest[0][0] < back_bracket[0][0] - 0.20,
        details=f"back_bracket={back_bracket}, support={support_rest}",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 1.05}):
        raised_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest rotates upward from seat junction",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.45,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.34}):
        raised_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat pivots on separate front hinge",
        rest_seat_aabb is not None
        and raised_seat_aabb is not None
        and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.11,
        details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
    )

    folded_support_aabb = ctx.part_world_aabb(support)
    with ctx.pose({support_pivot: 0.72}):
        raised_support_aabb = ctx.part_world_aabb(support)
    ctx.check(
        "rear ladder support rotates on lower pivot",
        folded_support_aabb is not None
        and raised_support_aabb is not None
        and raised_support_aabb[1][0] > folded_support_aabb[1][0] + 0.16,
        details=f"folded={folded_support_aabb}, raised={raised_support_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
