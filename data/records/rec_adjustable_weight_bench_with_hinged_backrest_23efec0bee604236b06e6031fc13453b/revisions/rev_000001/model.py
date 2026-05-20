from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _box_between_xz(part, start, end, *, width: float, height: float, material, name: str) -> None:
    """Add a rectangular tube whose local X axis runs between two X/Z points."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = math.hypot(dx, dz)
    angle_y = math.atan2(-dz, dx)
    part.visual(
        Box((length, width, height)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_adjustable_weight_bench")

    steel = model.material("satin_black_powder_coat", rgba=(0.02, 0.022, 0.024, 1.0))
    edge_steel = model.material("rubbed_steel_edges", rgba=(0.18, 0.19, 0.19, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    vinyl = model.material("black_grained_vinyl", rgba=(0.015, 0.014, 0.013, 1.0))
    seam = model.material("dark_red_stitching", rgba=(0.55, 0.035, 0.025, 1.0))
    bracket = model.material("brushed_hinge_pin", rgba=(0.62, 0.64, 0.62, 1.0))

    # Static welded frame: one heavy center spine, a very wide rear stabilizer,
    # front wheel axle, and transverse hinge clevises for the pads and brace.
    frame = model.part("frame")
    frame.visual(
        Box((1.92, 0.085, 0.070)),
        origin=Origin(xyz=(0.05, 0.0, 0.135)),
        material=steel,
        name="center_spine",
    )
    frame.visual(
        Box((0.125, 1.16, 0.065)),
        origin=Origin(xyz=(-0.93, 0.0, 0.060)),
        material=steel,
        name="wide_rear_base",
    )
    frame.visual(
        Box((0.150, 0.080, 0.090)),
        origin=Origin(xyz=(-0.86, 0.0, 0.100)),
        material=steel,
        name="rear_spine_socket",
    )
    for y, name in ((-0.565, "rear_foot_0"), (0.565, "rear_foot_1")):
        frame.visual(
            Box((0.150, 0.060, 0.025)),
            origin=Origin(xyz=(-0.93, y, 0.025)),
            material=rubber,
            name=name,
        )
    _box_between_xz(
        frame,
        (-0.24, 0.0, 0.170),
        (0.00, 0.0, 0.480),
        width=0.065,
        height=0.055,
        material=steel,
        name="back_hinge_strut",
    )
    _box_between_xz(
        frame,
        (0.52, 0.0, 0.170),
        (0.24, 0.0, 0.480),
        width=0.065,
        height=0.055,
        material=steel,
        name="seat_hinge_strut",
    )
    frame.visual(
        Cylinder(radius=0.016, length=1.08),
        origin=Origin(xyz=(0.98, 0.0, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="front_axle",
    )
    frame.visual(
        Box((0.115, 0.105, 0.070)),
        origin=Origin(xyz=(0.95, 0.0, 0.125)),
        material=steel,
        name="front_axle_block",
    )
    for y, name in ((-0.255, "back_hinge_plate_0"), (0.255, "back_hinge_plate_1")):
        frame.visual(
            Box((0.090, 0.035, 0.150)),
            origin=Origin(xyz=(0.00, y, 0.490)),
            material=steel,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(0.00, 0.0, 0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="back_hinge_pin",
    )
    frame.visual(
        Box((0.080, 0.560, 0.030)),
        origin=Origin(xyz=(0.00, 0.0, 0.445)),
        material=steel,
        name="back_hinge_mast",
    )
    for y, name in ((-0.255, "seat_hinge_plate_0"), (0.255, "seat_hinge_plate_1")):
        frame.visual(
            Box((0.090, 0.035, 0.145)),
            origin=Origin(xyz=(0.24, y, 0.490)),
            material=steel,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(0.24, 0.0, 0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="seat_hinge_pin",
    )
    frame.visual(
        Box((0.080, 0.560, 0.030)),
        origin=Origin(xyz=(0.24, 0.0, 0.445)),
        material=steel,
        name="seat_hinge_mast",
    )
    for y, name in ((-0.190, "brace_pivot_plate_0"), (0.190, "brace_pivot_plate_1")):
        frame.visual(
            Box((0.090, 0.035, 0.120)),
            origin=Origin(xyz=(-0.78, y, 0.200)),
            material=steel,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.012, length=0.430),
        origin=Origin(xyz=(-0.78, 0.0, 0.200), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="brace_pivot_pin",
    )
    frame.visual(
        Box((0.090, 0.430, 0.030)),
        origin=Origin(xyz=(-0.78, 0.0, 0.155)),
        material=steel,
        name="brace_pivot_mast",
    )

    # Long backrest pad.  The child frame is exactly on the transverse hinge
    # line; the cushion extends rearward along local -X from that hinge.
    back_pad = model.part("back_pad")
    back_pad.visual(
        Box((1.16, 0.360, 0.078)),
        origin=Origin(xyz=(-0.585, 0.0, 0.057)),
        material=vinyl,
        name="long_back_cushion",
    )
    back_pad.visual(
        Box((1.08, 0.018, 0.010)),
        origin=Origin(xyz=(-0.590, -0.185, 0.094)),
        material=seam,
        name="back_side_piping_0",
    )
    back_pad.visual(
        Box((1.08, 0.018, 0.010)),
        origin=Origin(xyz=(-0.590, 0.185, 0.094)),
        material=seam,
        name="back_side_piping_1",
    )
    back_pad.visual(
        Box((1.05, 0.055, 0.034)),
        origin=Origin(xyz=(-0.560, 0.0, 0.018)),
        material=edge_steel,
        name="back_under_rail",
    )
    back_pad.visual(
        Cylinder(radius=0.027, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="back_hinge_sleeve",
    )
    back_pad.visual(
        Box((0.065, 0.310, 0.036)),
        origin=Origin(xyz=(-0.045, 0.0, 0.018)),
        material=edge_steel,
        name="back_hinge_tongue",
    )

    # Short independent seat pad, on a separate hinge ahead of the backrest
    # joint.
    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        Box((0.560, 0.360, 0.078)),
        origin=Origin(xyz=(0.245, 0.0, 0.057)),
        material=vinyl,
        name="short_seat_cushion",
    )
    seat_pad.visual(
        Box((0.500, 0.018, 0.010)),
        origin=Origin(xyz=(0.260, -0.185, 0.094)),
        material=seam,
        name="seat_side_piping_0",
    )
    seat_pad.visual(
        Box((0.500, 0.018, 0.010)),
        origin=Origin(xyz=(0.260, 0.185, 0.094)),
        material=seam,
        name="seat_side_piping_1",
    )
    seat_pad.visual(
        Box((0.485, 0.055, 0.034)),
        origin=Origin(xyz=(0.270, 0.0, 0.018)),
        material=edge_steel,
        name="seat_under_rail",
    )
    seat_pad.visual(
        Cylinder(radius=0.025, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="seat_hinge_sleeve",
    )
    seat_pad.visual(
        Box((0.065, 0.310, 0.036)),
        origin=Origin(xyz=(0.045, 0.0, 0.018)),
        material=edge_steel,
        name="seat_hinge_tongue",
    )

    # One articulated stepped rear brace.  It is a connected ladder-like member:
    # twin side rails plus transverse rungs that act as the multiple incline
    # support positions.
    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Cylinder(radius=0.023, length=0.315),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="lower_pivot_sleeve",
    )
    rail_start_x, rail_start_z = 0.015, 0.020
    rail_end_x, rail_end_z = 0.665, 0.260
    for y, name in ((-0.115, "brace_rail_0"), (0.115, "brace_rail_1")):
        _box_between_xz(
            rear_brace,
            (rail_start_x, y, rail_start_z),
            (rail_end_x, y, rail_end_z),
            width=0.030,
            height=0.030,
            material=steel,
            name=name,
        )
    for index, t in enumerate((0.18, 0.34, 0.50, 0.66, 0.82)):
        x = rail_start_x + (rail_end_x - rail_start_x) * t
        z = rail_start_z + (rail_end_z - rail_start_z) * t
        rear_brace.visual(
            Cylinder(radius=0.015, length=0.305),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bracket,
            name=f"incline_step_{index}",
        )
        rear_brace.visual(
            Box((0.030, 0.275, 0.020)),
            origin=Origin(xyz=(x + 0.027, 0.0, z + 0.012)),
            material=steel,
            name=f"step_lip_{index}",
        )
    rear_brace.visual(
        Cylinder(radius=0.019, length=0.345),
        origin=Origin(xyz=(0.670, 0.0, 0.260), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bracket,
        name="upper_catch_roller",
    )

    # Transport wheels.  Wheel/tire helper meshes are centered on local X, so
    # their visual frames are yawed 90 degrees to line up with the bench's
    # transverse axle and continuous spin axis.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.064,
            0.052,
            rim=WheelRim(inner_radius=0.044, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.026,
                width=0.038,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.032, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.030),
        ),
        "transport_wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.095,
            0.060,
            inner_radius=0.065,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.005, count=18, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "transport_tire",
    )
    for y, name in ((-0.520, "front_wheel_0"), (0.520, "front_wheel_1")):
        wheel = model.part(name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=bracket,
            name="hub",
        )
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.98, y, 0.120)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    model.articulation(
        "frame_to_back_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.00, 0.0, 0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "frame_to_seat_pad",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(0.24, 0.0, 0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=0.55),
    )
    model.articulation(
        "frame_to_rear_brace",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_brace,
        origin=Origin(xyz=(-0.78, 0.0, 0.200)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=0.72),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat_pad = object_model.get_part("seat_pad")
    rear_brace = object_model.get_part("rear_brace")
    wheel_0 = object_model.get_part("front_wheel_0")
    wheel_1 = object_model.get_part("front_wheel_1")

    back_joint = object_model.get_articulation("frame_to_back_pad")
    seat_joint = object_model.get_articulation("frame_to_seat_pad")
    brace_joint = object_model.get_articulation("frame_to_rear_brace")
    wheel_joint_0 = object_model.get_articulation("frame_to_front_wheel_0")
    wheel_joint_1 = object_model.get_articulation("frame_to_front_wheel_1")

    ctx.allow_overlap(
        frame,
        back_pad,
        elem_a="back_hinge_pin",
        elem_b="back_hinge_sleeve",
        reason="The visible hinge pin is intentionally captured inside the back pad sleeve.",
    )
    ctx.allow_overlap(
        frame,
        seat_pad,
        elem_a="seat_hinge_pin",
        elem_b="seat_hinge_sleeve",
        reason="The visible hinge pin is intentionally captured inside the seat pad sleeve.",
    )
    ctx.allow_overlap(
        frame,
        rear_brace,
        elem_a="brace_pivot_pin",
        elem_b="lower_pivot_sleeve",
        reason="The lower brace pivot pin is intentionally nested in the brace sleeve.",
    )
    ctx.allow_overlap(
        frame,
        wheel_0,
        elem_a="front_axle",
        elem_b="hub",
        reason="The transport wheel hub intentionally rotates around the fixed axle.",
    )
    ctx.allow_overlap(
        frame,
        wheel_1,
        elem_a="front_axle",
        elem_b="hub",
        reason="The transport wheel hub intentionally rotates around the fixed axle.",
    )

    ctx.expect_overlap(
        back_pad,
        frame,
        axes="xz",
        min_overlap=0.020,
        elem_a="back_hinge_sleeve",
        elem_b="back_hinge_pin",
        name="back sleeve is coaxial with its hinge pin",
    )
    ctx.expect_overlap(
        seat_pad,
        frame,
        axes="xz",
        min_overlap=0.020,
        elem_a="seat_hinge_sleeve",
        elem_b="seat_hinge_pin",
        name="seat sleeve is coaxial with its hinge pin",
    )
    ctx.expect_overlap(
        rear_brace,
        frame,
        axes="xz",
        min_overlap=0.020,
        elem_a="lower_pivot_sleeve",
        elem_b="brace_pivot_pin",
        name="rear brace lower sleeve is coaxial with the frame pivot",
    )
    ctx.expect_overlap(
        wheel_0,
        frame,
        axes="xz",
        min_overlap=0.025,
        elem_a="hub",
        elem_b="front_axle",
        name="first transport wheel is centered on the front axle",
    )
    ctx.expect_overlap(
        wheel_1,
        frame,
        axes="xz",
        min_overlap=0.025,
        elem_a="hub",
        elem_b="front_axle",
        name="second transport wheel is centered on the front axle",
    )

    ctx.check(
        "bench has requested moving mechanisms",
        all(
            joint is not None
            for joint in (back_joint, seat_joint, brace_joint, wheel_joint_0, wheel_joint_1)
        )
        and wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected back hinge, seat hinge, rear brace pivot, and two continuous wheel joints.",
    )

    rest_back = ctx.part_element_world_aabb(back_pad, elem="long_back_cushion")
    rest_seat = ctx.part_element_world_aabb(seat_pad, elem="short_seat_cushion")
    rest_brace = ctx.part_element_world_aabb(rear_brace, elem="upper_catch_roller")
    with ctx.pose({back_joint: 0.95, seat_joint: 0.35, brace_joint: 0.60}):
        raised_back = ctx.part_element_world_aabb(back_pad, elem="long_back_cushion")
        raised_seat = ctx.part_element_world_aabb(seat_pad, elem="short_seat_cushion")
        raised_brace = ctx.part_element_world_aabb(rear_brace, elem="upper_catch_roller")

    ctx.check(
        "pads and stepped brace rotate upward for incline positions",
        rest_back is not None
        and raised_back is not None
        and rest_seat is not None
        and raised_seat is not None
        and rest_brace is not None
        and raised_brace is not None
        and raised_back[1][2] > rest_back[1][2] + 0.45
        and raised_seat[1][2] > rest_seat[1][2] + 0.12
        and raised_brace[1][2] > rest_brace[1][2] + 0.20,
        details=f"rest_back={rest_back}, raised_back={raised_back}, rest_seat={rest_seat}, raised_seat={raised_seat}, rest_brace={rest_brace}, raised_brace={raised_brace}",
    )

    return ctx.report()


object_model = build_object_model()
