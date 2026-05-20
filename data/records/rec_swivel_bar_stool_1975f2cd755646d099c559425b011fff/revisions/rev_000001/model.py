from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _perforated_seat_shell() -> cq.Workplane:
    """One welded, powder-coated metal seat: perforated pan, rolled rim, hub, ribs."""
    seat_radius = 0.225
    plate_thickness = 0.018

    disk = cq.Workplane("XY").circle(seat_radius).extrude(plate_thickness)

    cutters = cq.Workplane("XY")
    pitch = 0.045
    hole_radius = 0.0085
    max_hole_radius = 0.166
    for row, y_index in enumerate(range(-3, 4)):
        y = y_index * pitch
        x_offset = 0.5 * pitch if row % 2 else 0.0
        for x_index in range(-4, 5):
            x = x_index * pitch + x_offset
            if math.hypot(x, y) < max_hole_radius:
                cutters = cutters.union(
                    cq.Workplane("XY")
                    .center(x, y)
                    .circle(hole_radius)
                    .extrude(plate_thickness + 0.016)
                    .translate((0.0, 0.0, -0.008))
                )
    disk = disk.cut(cutters)

    rolled_rim = (
        cq.Workplane("XY")
        .circle(seat_radius + 0.010)
        .circle(seat_radius - 0.018)
        .extrude(0.030)
        .translate((0.0, 0.0, -0.006))
    )
    underside_hub = (
        cq.Workplane("XY")
        .circle(0.072)
        .extrude(0.034)
        .translate((0.0, 0.0, -0.030))
    )

    body = disk.union(rolled_rim).union(underside_hub)
    for angle in (0.0, 90.0, 180.0, 270.0):
        rib = (
            cq.Workplane("XY")
            .box(0.220, 0.020, 0.018)
            .translate((0.090, 0.0, -0.011))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.union(rib)

    return body


def _weatherproof_back_pad():
    """Rounded marine-vinyl back pad standing up from the hinge line."""
    pad = ExtrudeGeometry.centered(
        rounded_rect_profile(0.420, 0.245, 0.034, corner_segments=8),
        0.055,
    )
    return pad.rotate_x(math.pi / 2.0).translate(0.0, 0.022, 0.205)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_swivel_bar_stool")

    dark_powder = model.material("dark_gray_powder_coat", rgba=(0.08, 0.09, 0.09, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    cushion_vinyl = model.material("charcoal_marine_vinyl", rgba=(0.035, 0.045, 0.052, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.260, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_powder,
        name="weighted_base",
    )
    pedestal.visual(
        Cylinder(radius=0.253, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black_rubber,
        name="rubber_foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.043, length=0.670),
        origin=Origin(xyz=(0.0, 0.0, 0.391)),
        material=brushed_steel,
        name="center_column",
    )
    pedestal.visual(
        Cylinder(radius=0.092, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=brushed_steel,
        name="top_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.186, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=brushed_steel,
        name="footrest_ring",
    )
    pedestal.visual(
        Box((0.360, 0.026, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=brushed_steel,
        name="footrest_spoke_x",
    )
    pedestal.visual(
        Box((0.026, 0.360, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=brushed_steel,
        name="footrest_spoke_y",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_perforated_seat_shell(), "perforated_metal_seat", tolerance=0.0007),
        material=dark_powder,
        name="seat_panel",
    )
    for x in (-0.180, 0.180):
        seat.visual(
            Box((0.040, 0.130, 0.014)),
            origin=Origin(xyz=(x, 0.168, 0.006)),
            material=brushed_steel,
            name=f"rear_support_{0 if x < 0 else 1}",
        )
        seat.visual(
            Box((0.046, 0.026, 0.086)),
            origin=Origin(xyz=(x, 0.235, 0.036)),
            material=brushed_steel,
            name=f"hinge_stanchion_{0 if x < 0 else 1}",
        )
    seat.visual(
        Cylinder(radius=0.009, length=0.380),
        origin=Origin(xyz=(0.0, 0.235, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_pin",
    )

    backrest = model.part("backrest")
    backrest.visual(
        mesh_from_geometry(_weatherproof_back_pad(), "fold_down_backrest_pad"),
        material=cushion_vinyl,
        name="pad",
    )
    backrest.visual(
        Cylinder(radius=0.015, length=0.070),
        origin=Origin(xyz=(-0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_sleeve_0",
    )
    backrest.visual(
        Box((0.032, 0.018, 0.145)),
        origin=Origin(xyz=(-0.120, 0.018, 0.073)),
        material=brushed_steel,
        name="back_strap_0",
    )
    backrest.visual(
        Cylinder(radius=0.015, length=0.070),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hinge_sleeve_1",
    )
    backrest.visual(
        Box((0.032, 0.018, 0.145)),
        origin=Origin(xyz=(0.120, 0.018, 0.073)),
        material=brushed_steel,
        name="back_strap_1",
    )

    model.articulation(
        "pedestal_to_seat",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=6.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.235, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    swivel = object_model.get_articulation("pedestal_to_seat")
    fold_hinge = object_model.get_articulation("seat_to_backrest")

    ctx.allow_overlap(
        seat,
        backrest,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve_0",
        reason="The visible hinge sleeve is represented as captured around the stainless pin.",
    )
    ctx.allow_overlap(
        seat,
        backrest,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve_1",
        reason="The visible hinge sleeve is represented as captured around the stainless pin.",
    )

    ctx.check(
        "seat has continuous vertical swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "backrest hinge folds downward",
        fold_hinge.motion_limits is not None
        and fold_hinge.motion_limits.lower == 0.0
        and 1.35 <= (fold_hinge.motion_limits.upper or 0.0) <= 1.55
        and tuple(fold_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"limits={fold_hinge.motion_limits}, axis={fold_hinge.axis}",
    )

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="seat_panel",
        negative_elem="top_bearing",
        min_gap=0.0,
        max_gap=0.003,
        name="seat hub sits on the pedestal bearing",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="seat_panel",
        elem_b="top_bearing",
        min_overlap=0.08,
        name="seat hub is centered over the bearing",
    )
    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1"):
        ctx.expect_overlap(
            backrest,
            seat,
            axes="xyz",
            elem_a=sleeve,
            elem_b="hinge_pin",
            min_overlap=0.005,
            name=f"{sleeve} captures the hinge pin",
        )

    upright_pad = ctx.part_element_world_aabb(backrest, elem="pad")
    with ctx.pose({fold_hinge: 1.45}):
        folded_pad = ctx.part_element_world_aabb(backrest, elem="pad")
        ctx.expect_gap(
            backrest,
            seat,
            axis="z",
            positive_elem="pad",
            negative_elem="seat_panel",
            min_gap=0.030,
            name="folded back pad clears the perforated seat",
        )
    ctx.check(
        "folded pad moves forward and down",
        upright_pad is not None
        and folded_pad is not None
        and folded_pad[1][2] < upright_pad[1][2] - 0.12
        and folded_pad[0][1] < upright_pad[0][1] - 0.10,
        details=f"upright={upright_pad}, folded={folded_pad}",
    )

    return ctx.report()


object_model = build_object_model()
