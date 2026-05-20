from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


STEEL = Material("satin_black_steel", color=(0.02, 0.022, 0.022, 1.0))
VINYL = Material("black_vinyl_pad", color=(0.015, 0.014, 0.013, 1.0))
FOAM = Material("dense_black_foam", color=(0.025, 0.024, 0.022, 1.0))
RUBBER = Material("matte_rubber", color=(0.01, 0.01, 0.009, 1.0))
GREY = Material("dark_grey_plastic", color=(0.18, 0.18, 0.17, 1.0))
PIN = Material("brushed_pin_metal", color=(0.55, 0.55, 0.50, 1.0))


def _rounded_pad(length: float, width: float, thickness: float, center_x: float):
    """A simple molded cushion: rounded rectangular vinyl slab in local meters."""
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges()
        .fillet(0.018)
        .translate((center_x, 0.0, thickness * 0.5))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_decline_bench")

    frame = model.part("frame")

    # Main welded bench frame: low rectangular tube spine, crossmembers, posts,
    # hinge pins, roller axle, and a real U-shaped front carry handle tied into
    # the front crossmember between the transport wheels.
    frame.visual(
        Box((1.78, 0.060, 0.050)),
        origin=Origin(xyz=(-0.05, 0.0, 0.245)),
        material=STEEL,
        name="center_spine",
    )
    frame.visual(
        Box((1.16, 0.040, 0.040)),
        origin=Origin(xyz=(-0.22, 0.185, 0.370)),
        material=STEEL,
        name="side_rail_0",
    )
    frame.visual(
        Box((1.16, 0.040, 0.040)),
        origin=Origin(xyz=(-0.22, -0.185, 0.370)),
        material=STEEL,
        name="side_rail_1",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.70),
        origin=Origin(xyz=(-0.94, 0.0, 0.065), rpy=(pi / 2, 0.0, 0.0)),
        material=STEEL,
        name="rear_floor_bar",
    )
    frame.visual(
        Box((0.060, 0.070, 0.215)),
        origin=Origin(xyz=(-0.91, 0.0, 0.165)),
        material=STEEL,
        name="rear_support",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.54),
        origin=Origin(xyz=(0.78, 0.0, 0.105), rpy=(pi / 2, 0.0, 0.0)),
        material=STEEL,
        name="front_crossmember",
    )
    frame.visual(
        Box((0.070, 0.070, 0.220)),
        origin=Origin(xyz=(0.75, 0.0, 0.185)),
        material=STEEL,
        name="front_support",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.74),
        origin=Origin(xyz=(0.80, 0.0, 0.085), rpy=(pi / 2, 0.0, 0.0)),
        material=PIN,
        name="wheel_axle",
    )
    frame.visual(
        Box((0.060, 0.070, 0.230)),
        origin=Origin(xyz=(-0.04, 0.0, 0.352)),
        material=STEEL,
        name="back_hinge_stand",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.45),
        origin=Origin(xyz=(-0.04, 0.0, 0.370), rpy=(pi / 2, 0.0, 0.0)),
        material=STEEL,
        name="back_top_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.45),
        origin=Origin(xyz=(-0.04, 0.0, 0.485), rpy=(pi / 2, 0.0, 0.0)),
        material=PIN,
        name="back_hinge_pin",
    )
    frame.visual(
        Box((0.060, 0.070, 0.230)),
        origin=Origin(xyz=(0.04, 0.0, 0.352)),
        material=STEEL,
        name="seat_hinge_stand",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.45),
        origin=Origin(xyz=(0.25, 0.0, 0.370), rpy=(pi / 2, 0.0, 0.0)),
        material=STEEL,
        name="seat_top_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.39),
        origin=Origin(xyz=(0.04, 0.0, 0.485), rpy=(pi / 2, 0.0, 0.0)),
        material=PIN,
        name="seat_hinge_pin",
    )
    frame.visual(
        Box((0.052, 0.058, 0.018)),
        origin=Origin(xyz=(0.04, 0.0, 0.467)),
        material=STEEL,
        name="seat_pin_saddle",
    )
    frame.visual(
        Box((0.060, 0.070, 0.455)),
        origin=Origin(xyz=(0.58, 0.0, 0.405)),
        material=STEEL,
        name="front_upright",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.54),
        origin=Origin(xyz=(0.60, 0.0, 0.635), rpy=(pi / 2, 0.0, 0.0)),
        material=PIN,
        name="roller_axle",
    )
    frame.visual(
        Box((0.34, 0.040, 0.040)),
        origin=Origin(xyz=(0.43, 0.0, 0.265), rpy=(0.0, -0.55, 0.0)),
        material=STEEL,
        name="front_brace",
    )
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.765, -0.145, 0.120),
                (0.895, -0.130, 0.125),
                (1.020, -0.070, 0.150),
                (1.055, 0.000, 0.168),
                (1.020, 0.070, 0.150),
                (0.895, 0.130, 0.125),
                (0.765, 0.145, 0.120),
            ],
            radius=0.014,
            samples_per_segment=8,
            radial_segments=18,
            cap_ends=True,
        ),
        "front_carry_handle",
    )
    frame.visual(handle_mesh, material=STEEL, name="carry_handle")
    frame.visual(
        Box((0.045, 0.055, 0.038)),
        origin=Origin(xyz=(0.775, -0.145, 0.118)),
        material=STEEL,
        name="handle_mount_0",
    )
    frame.visual(
        Box((0.045, 0.055, 0.038)),
        origin=Origin(xyz=(0.775, 0.145, 0.118)),
        material=STEEL,
        name="handle_mount_1",
    )

    # Long back pad, hinged at its front edge so it can set a decline or incline.
    back_pad = model.part("back_pad")
    back_pad.visual(
        mesh_from_cadquery(_rounded_pad(0.95, 0.315, 0.075, -0.515), "back_cushion"),
        material=VINYL,
        name="back_cushion",
    )
    back_pad.visual(
        Box((0.90, 0.255, 0.020)),
        origin=Origin(xyz=(-0.515, 0.0, -0.006)),
        material=GREY,
        name="back_plate",
    )
    back_pad.visual(
        Cylinder(radius=0.012, length=0.25),
        origin=Origin(xyz=(-0.015, 0.0, 0.018), rpy=(pi / 2, 0.0, 0.0)),
        material=PIN,
        name="back_hinge_sleeve",
    )
    back_pad.visual(
        Box((0.060, 0.060, 0.026)),
        origin=Origin(xyz=(-0.055, 0.0, 0.006)),
        material=GREY,
        name="back_hinge_tab",
    )

    # Shorter seat pad on its own hinge ahead of the back hinge.
    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_rounded_pad(0.40, 0.315, 0.075, 0.245), "seat_cushion"),
        material=VINYL,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.36, 0.255, 0.020)),
        origin=Origin(xyz=(0.245, 0.0, -0.006)),
        material=GREY,
        name="seat_plate",
    )
    seat.visual(
        Cylinder(radius=0.011, length=0.22),
        origin=Origin(xyz=(0.015, 0.0, 0.018), rpy=(pi / 2, 0.0, 0.0)),
        material=PIN,
        name="seat_hinge_sleeve",
    )
    seat.visual(
        Box((0.076, 0.060, 0.026)),
        origin=Origin(xyz=(0.050, 0.0, 0.006)),
        material=GREY,
        name="seat_hinge_tab",
    )

    # Two independent foam leg rollers on the front support axle.
    roller_geom = TireGeometry(
        0.060,
        0.175,
        inner_radius=0.010,
        carcass=TireCarcass(belt_width_ratio=0.92, sidewall_bulge=0.025),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )
    for idx, y in enumerate((-0.155, 0.155)):
        roller = model.part(f"roller_{idx}")
        roller.visual(
            mesh_from_geometry(roller_geom, f"foam_roller_{idx}"),
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=FOAM,
            name="foam",
        )
        model.articulation(
            f"frame_to_roller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(0.60, y, 0.635)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )

    # Low transport wheels at the front end; the helper geometry includes
    # distinct rubber tire, rim, hub, bore and spokes rather than plain discs.
    wheel_geom = WheelGeometry(
        0.052,
        0.040,
        rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.018,
            width=0.032,
            cap_style="flat",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.018),
    )
    tire_geom = TireGeometry(
        0.075,
        0.046,
        inner_radius=0.053,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
        tread=TireTread(style="block", depth=0.003, count=16, land_ratio=0.60),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    for idx, y in enumerate((-0.330, 0.330)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            mesh_from_geometry(tire_geom, f"transport_tire_{idx}"),
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=RUBBER,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_geom, f"transport_rim_{idx}"),
            origin=Origin(rpy=(0.0, 0.0, pi / 2)),
            material=GREY,
            name="rim",
        )
        model.articulation(
            f"frame_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.80, y, 0.085)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=12.0),
        )

    model.articulation(
        "frame_to_back_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(-0.04, 0.0, 0.485)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.20, upper=0.85),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.04, 0.0, 0.485)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat = object_model.get_part("seat")
    back_hinge = object_model.get_articulation("frame_to_back_pad")
    seat_hinge = object_model.get_articulation("frame_to_seat")

    ctx.allow_overlap(
        back_pad,
        frame,
        elem_a="back_hinge_sleeve",
        elem_b="back_hinge_pin",
        reason="The back pad hinge sleeve is intentionally captured around the visible hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_pin",
        elem_b="seat_hinge_sleeve",
        reason="The seat hinge sleeve is intentionally captured around its hinge pin.",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            frame,
            f"roller_{idx}",
            elem_a="roller_axle",
            elem_b="foam",
            reason="The foam roller is shown retained on a support axle with a small captured bore overlap.",
        )
        ctx.allow_overlap(
            frame,
            f"wheel_{idx}",
            elem_a="wheel_axle",
            elem_b="rim",
            reason="The transport wheel rim is intentionally retained on the frame axle.",
        )

    def _coord(vec, index: int) -> float:
        try:
            return vec[index]
        except TypeError:
            return (vec.x, vec.y, vec.z)[index]

    def _zmax(aabb) -> float:
        return _coord(aabb[1], 2)

    continuous_names = {
        "frame_to_roller_0",
        "frame_to_roller_1",
        "frame_to_wheel_0",
        "frame_to_wheel_1",
    }
    ctx.check(
        "rollers and transport wheels are continuous",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in continuous_names
        ),
    )
    ctx.check(
        "pads use separate revolute hinges",
        back_hinge.articulation_type == ArticulationType.REVOLUTE
        and seat_hinge.articulation_type == ArticulationType.REVOLUTE
        and back_hinge is not seat_hinge,
    )
    ctx.expect_overlap(
        back_pad,
        frame,
        axes="y",
        elem_a="back_hinge_sleeve",
        elem_b="back_hinge_pin",
        min_overlap=0.20,
        name="back hinge sleeve spans pin",
    )
    ctx.expect_overlap(
        seat,
        frame,
        axes="y",
        elem_a="seat_hinge_sleeve",
        elem_b="seat_hinge_pin",
        min_overlap=0.18,
        name="seat hinge sleeve spans pin",
    )
    for idx in (0, 1):
        ctx.expect_overlap(
            f"roller_{idx}",
            frame,
            axes="y",
            elem_a="foam",
            elem_b="roller_axle",
            min_overlap=0.12,
            name=f"roller {idx} retained on axle",
        )
        ctx.expect_overlap(
            f"wheel_{idx}",
            frame,
            axes="y",
            elem_a="rim",
            elem_b="wheel_axle",
            min_overlap=0.03,
            name=f"wheel {idx} retained on axle",
        )

    back_rest = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    with ctx.pose({back_hinge: 0.55}):
        back_raised = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    ctx.check(
        "back pad raises on central hinge",
        back_rest is not None
        and back_raised is not None
        and _zmax(back_raised) > _zmax(back_rest) + 0.18,
        details=f"rest={back_rest}, raised={back_raised}",
    )

    seat_rest = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    with ctx.pose({seat_hinge: 0.45}):
        seat_raised = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "seat tips upward on its own hinge",
        seat_rest is not None
        and seat_raised is not None
        and _zmax(seat_raised) > _zmax(seat_rest) + 0.08,
        details=f"rest={seat_rest}, raised={seat_raised}",
    )

    handle_box = ctx.part_element_world_aabb(frame, elem="carry_handle")
    cross_box = ctx.part_element_world_aabb(frame, elem="front_crossmember")
    ctx.check(
        "front carry handle projects from crossmember",
        handle_box is not None
        and cross_box is not None
        and _coord(handle_box[1], 0) > _coord(cross_box[1], 0) + 0.20
        and _coord(handle_box[0], 0) < _coord(cross_box[1], 0) + 0.02,
        details=f"handle={handle_box}, crossmember={cross_box}",
    )

    return ctx.report()


object_model = build_object_model()
