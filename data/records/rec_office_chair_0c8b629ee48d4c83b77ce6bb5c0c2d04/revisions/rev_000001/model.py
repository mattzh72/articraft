from __future__ import annotations

from math import cos, pi, sin

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


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return mesh_from_cadquery(shape, name)


def _rounded_pad(width: float, depth: float, thickness: float, radius: float, name: str):
    safe_radius = min(radius, 0.45 * min(width, depth))
    shape = (
        cq.Workplane("XY")
        .box(width, depth, thickness)
        .edges("|Z")
        .fillet(safe_radius)
    )
    return mesh_from_cadquery(shape, name)


def _tapered_beam(length: float, width_root: float, width_tip: float, thickness: float, name: str):
    half_length = length * 0.5
    half_root = width_root * 0.5
    half_tip = width_tip * 0.5
    shape = (
        cq.Workplane("XY")
        .polyline(
            [
                (-half_length, -half_root),
                (half_length, -half_tip),
                (half_length, half_tip),
                (-half_length, half_root),
            ]
        )
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness * 0.5))
    )
    return mesh_from_cadquery(shape, name)


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return radius * cos(angle), radius * sin(angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_chair")

    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.26, 0.27, 0.30, 1.0))
    silver = model.material("silver", rgba=(0.71, 0.74, 0.77, 1.0))
    fabric = model.material("fabric", rgba=(0.15, 0.16, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        _tube_mesh(0.070, 0.034, 0.050, "chair_hub_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=charcoal,
        name="hub_ring",
    )
    base.visual(
        _tube_mesh(0.040, 0.029, 0.230, "chair_lower_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=silver,
        name="lower_sleeve",
    )
    leg_beam = _tapered_beam(0.220, 0.070, 0.044, 0.028, "chair_leg_beam")

    leg_angles = [index * 2.0 * pi / 5.0 for index in range(5)]
    for angle in leg_angles:
        leg_x, leg_y = _polar(0.175, angle)
        tip_x, tip_y = _polar(0.322, angle)
        tangent_x, tangent_y = -sin(angle), cos(angle)
        yaw = angle

        base.visual(
            leg_beam,
            origin=Origin(xyz=(leg_x, leg_y, 0.049), rpy=(0.0, 0.0, yaw)),
            material=charcoal,
        )
        nose_x, nose_y = _polar(0.305, angle)
        base.visual(
            Box((0.055, 0.028, 0.008)),
            origin=Origin(xyz=(nose_x, nose_y, 0.066), rpy=(0.0, 0.0, yaw)),
            material=dark_grey,
        )
        base.visual(
            Box((0.090, 0.080, 0.018)),
            origin=Origin(xyz=(leg_x * 0.72, leg_y * 0.72, 0.062), rpy=(0.0, 0.0, yaw)),
            material=dark_grey,
        )
        base.visual(
            Cylinder(radius=0.011, length=0.018),
            origin=Origin(xyz=(tip_x, tip_y, 0.067)),
            material=charcoal,
        )
        base.visual(
            Box((0.020, 0.050, 0.008)),
            origin=Origin(xyz=(tip_x, tip_y, 0.060), rpy=(0.0, 0.0, yaw)),
            material=dark_grey,
        )
        for side in (-1.0, 1.0):
            base.visual(
                Box((0.018, 0.006, 0.038)),
                origin=Origin(
                    xyz=(
                        tip_x + tangent_x * 0.022 * side,
                        tip_y + tangent_y * 0.022 * side,
                        0.043,
                    ),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_grey,
            )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.026, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=silver,
        name="upper_column",
    )
    seat.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="column_cap",
    )
    seat.visual(
        Box((0.210, 0.145, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_grey,
        name="tilt_housing",
    )
    seat.visual(
        Box((0.235, 0.200, 0.016)),
        origin=Origin(xyz=(0.0, 0.000, 0.082)),
        material=charcoal,
        name="seat_plate",
    )
    seat.visual(
        Box((0.240, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.170, 0.099)),
        material=dark_grey,
        name="rear_bracket",
    )
    for side_x in (-0.095, 0.095):
        seat.visual(
            Box((0.020, 0.150, 0.030)),
            origin=Origin(xyz=(side_x, -0.082, 0.080)),
            material=dark_grey,
        )
        seat.visual(
            Box((0.020, 0.028, 0.090)),
            origin=Origin(xyz=(side_x, -0.170, 0.135)),
            material=dark_grey,
        )
    seat.visual(
        Box((0.430, 0.340, 0.034)),
        origin=Origin(xyz=(0.0, 0.045, 0.106)),
        material=black,
        name="seat_shell",
    )
    seat.visual(
        _rounded_pad(0.500, 0.420, 0.060, 0.030, "chair_seat_pad"),
        origin=Origin(xyz=(0.0, 0.070, 0.150)),
        material=fabric,
        name="seat_pad",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    for side_x in (-0.084, 0.084):
        backrest.visual(
            Box((0.028, 0.032, 0.278)),
            origin=Origin(xyz=(side_x, -0.055, 0.144), rpy=(0.14, 0.0, 0.0)),
            material=charcoal,
        )
        backrest.visual(
            Box((0.028, 0.030, 0.070)),
            origin=Origin(xyz=(side_x, -0.030, 0.036), rpy=(0.10, 0.0, 0.0)),
            material=charcoal,
        )
    backrest.visual(
        Box((0.280, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, -0.082, 0.190), rpy=(0.14, 0.0, 0.0)),
        material=charcoal,
    )
    backrest.visual(
        Box((0.390, 0.018, 0.380)),
        origin=Origin(xyz=(0.0, -0.088, 0.355), rpy=(0.12, 0.0, 0.0)),
        material=black,
        name="back_shell",
    )
    backrest.visual(
        _rounded_pad(0.450, 0.055, 0.520, 0.028, "chair_back_pad"),
        origin=Origin(xyz=(0.0, -0.108, 0.380), rpy=(0.12, 0.0, 0.0)),
        material=fabric,
        name="back_pad",
    )

    for index, angle in enumerate(leg_angles):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.028, length=0.012),
            origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="wheel_outer",
        )
        caster.visual(
            Cylinder(radius=0.028, length=0.012),
            origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="wheel_inner",
        )
        caster.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_grey,
            name="wheel_hub",
        )
        tip_x, tip_y = _polar(0.322, angle)
        model.articulation(
            f"caster_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(tip_x, tip_y, 0.028), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=25.0),
        )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.18, lower=0.0, upper=0.110),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.170, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=0.450),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    seat_height = object_model.get_articulation("seat_height")
    back_recline = object_model.get_articulation("back_recline")

    seat_limits = seat_height.motion_limits
    recline_limits = back_recline.motion_limits
    if seat_limits is not None and seat_limits.upper is not None:
        with ctx.pose({seat_height: 0.0}):
            ctx.expect_within(
                seat,
                base,
                axes="xy",
                inner_elem="upper_column",
                outer_elem="lower_sleeve",
                margin=0.003,
                name="upper gas column stays centered in the lower sleeve at rest",
            )
            ctx.expect_overlap(
                seat,
                base,
                axes="z",
                elem_a="upper_column",
                elem_b="lower_sleeve",
                min_overlap=0.140,
                name="seat column remains deeply inserted at rest",
            )

        rest_pos = ctx.part_world_position(seat)
        with ctx.pose({seat_height: seat_limits.upper}):
            ctx.expect_within(
                seat,
                base,
                axes="xy",
                inner_elem="upper_column",
                outer_elem="lower_sleeve",
                margin=0.003,
                name="upper gas column stays centered at full height",
            )
            ctx.expect_overlap(
                seat,
                base,
                axes="z",
                elem_a="upper_column",
                elem_b="lower_sleeve",
                min_overlap=0.030,
                name="seat column retains insertion at full height",
            )
            raised_pos = ctx.part_world_position(seat)

        ctx.check(
            "seat raises upward",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.08,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    ctx.expect_overlap(
        backrest,
        seat,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="rear_bracket",
        min_overlap=0.160,
        name="backrest hinge spans the seat bracket",
    )
    ctx.expect_gap(
        backrest,
        seat,
        axis="z",
        positive_elem="hinge_barrel",
        negative_elem="rear_bracket",
        min_gap=0.0005,
        max_gap=0.006,
        name="backrest hinge sits directly on the rear bracket",
    )

    if recline_limits is not None and recline_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
        with ctx.pose({back_recline: recline_limits.upper}):
            reclined_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
        ctx.check(
            "backrest reclines rearward",
            rest_aabb is not None
            and reclined_aabb is not None
            and reclined_aabb[0][1] < rest_aabb[0][1] - 0.06,
            details=f"rest={rest_aabb}, reclined={reclined_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
