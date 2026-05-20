from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box(name: str, size: tuple[float, float, float], radius: float):
    shape = cq.Workplane("XY").box(*size).edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.004, angular_tolerance=0.18)


def _tube_between(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: float,
    material,
) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    horiz = math.sqrt(dx * dx + dy * dy)
    pitch = -math.atan2(dz, horiz)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_decline_bench")

    steel = model.material("powder_coated_steel", color=(0.07, 0.075, 0.08, 1.0))
    dark_steel = model.material("dark_steel", color=(0.015, 0.017, 0.018, 1.0))
    vinyl = model.material("black_textured_vinyl", color=(0.015, 0.018, 0.022, 1.0))
    seam = model.material("slightly_raised_seam", color=(0.055, 0.058, 0.064, 1.0))
    foam = model.material("dense_black_foam", color=(0.02, 0.021, 0.023, 1.0))
    rubber = model.material("matte_rubber", color=(0.006, 0.006, 0.006, 1.0))
    rim = model.material("satin_wheel_rim", color=(0.52, 0.54, 0.55, 1.0))

    frame = model.part("frame")
    # Low rectangular-tube chassis, with visible triangulation under the pads.
    frame.visual(
        Box((1.82, 0.060, 0.050)),
        origin=Origin(xyz=(-0.08, 0.0, 0.165)),
        material=steel,
        name="lower_spine",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.700),
        origin=Origin(xyz=(-1.00, 0.0, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_floor_foot",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.620),
        origin=Origin(xyz=(0.78, 0.0, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_floor_foot",
    )
    frame.visual(
        Box((0.060, 0.060, 0.260)),
        origin=Origin(xyz=(-1.00, 0.0, 0.160)),
        material=steel,
        name="rear_stanchion",
    )
    frame.visual(
        Box((0.060, 0.060, 0.500)),
        origin=Origin(xyz=(0.76, 0.0, 0.310)),
        material=steel,
        name="front_stanchion",
    )
    frame.visual(
        Box((0.060, 0.060, 0.300)),
        origin=Origin(xyz=(-0.10, 0.0, 0.325)),
        material=steel,
        name="middle_stanchion",
    )
    frame.visual(
        Box((0.050, 0.520, 0.040)),
        origin=Origin(xyz=(-0.10, 0.0, 0.455)),
        material=steel,
        name="middle_crossbar",
    )
    frame.visual(
        Box((0.050, 0.420, 0.035)),
        origin=Origin(xyz=(0.35, 0.0, 0.485)),
        material=steel,
        name="seat_crossbar",
    )

    for side, y in enumerate((-0.205, 0.205)):
        _tube_between(
            frame,
            name=f"back_side_rail_{side}",
            start=(-0.92, y, 0.265),
            end=(-0.10, y, 0.500),
            thickness=0.034,
            material=steel,
        )
        _tube_between(
            frame,
            name=f"seat_side_rail_{side}",
            start=(-0.10, y, 0.500),
            end=(0.36, y, 0.515),
            thickness=0.034,
            material=steel,
        )
    _tube_between(
        frame,
        name="front_diagonal",
        start=(0.30, 0.0, 0.185),
        end=(0.76, 0.0, 0.550),
        thickness=0.042,
        material=steel,
    )
    _tube_between(
        frame,
        name="rear_diagonal",
        start=(-0.78, 0.0, 0.185),
        end=(-0.12, 0.0, 0.465),
        thickness=0.042,
        material=steel,
    )
    for y in (-0.205, 0.205):
        frame.visual(
            Box((0.044, 0.036, 0.055)),
            origin=Origin(xyz=(-0.10, y, 0.493)),
            material=steel,
            name=f"back_rail_gusset_{0 if y < 0 else 1}",
        )

    # Hinge knuckles and pins for the two pad pivots and the folding brace.
    for y in (-0.165, 0.165):
        frame.visual(
            Cylinder(radius=0.024, length=0.090),
            origin=Origin(xyz=(-0.10, y, 0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"back_hinge_knuckle_{0 if y < 0 else 1}",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.080),
            origin=Origin(xyz=(0.35, y, 0.545), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"seat_hinge_knuckle_{0 if y < 0 else 1}",
        )
    frame.visual(
        Cylinder(radius=0.008, length=0.470),
        origin=Origin(xyz=(-0.10, 0.0, 0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="back_hinge_pin",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.430),
        origin=Origin(xyz=(0.35, 0.0, 0.545), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="seat_hinge_pin",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.240),
        origin=Origin(xyz=(0.02, 0.0, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="brace_hinge_pin",
    )
    frame.visual(
        Box((0.050, 0.075, 0.050)),
        origin=Origin(xyz=(0.02, 0.0, 0.185)),
        material=steel,
        name="brace_hinge_web",
    )
    for y in (-0.055, 0.055):
        frame.visual(
            Box((0.050, 0.035, 0.050)),
            origin=Origin(xyz=(0.02, y, 0.185)),
            material=steel,
            name=f"brace_hinge_sideweb_{0 if y < 0 else 1}",
        )
    for y in (-0.085, 0.085):
        frame.visual(
            Box((0.055, 0.035, 0.095)),
            origin=Origin(xyz=(0.02, y, 0.205)),
            material=steel,
            name=f"brace_hinge_lug_{0 if y < 0 else 1}",
        )

    # Transport wheel axle and fork ears at the low rear end.
    frame.visual(
        Cylinder(radius=0.010, length=0.700),
        origin=Origin(xyz=(-1.12, 0.0, 0.086), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="transport_axle",
    )
    for y in (-0.335, 0.335):
        frame.visual(
            Box((0.060, 0.030, 0.095)),
            origin=Origin(xyz=(-1.12, y, 0.095)),
            material=steel,
            name=f"wheel_fork_{0 if y < 0 else 1}",
        )
        _tube_between(
            frame,
            name=f"wheel_fork_strut_{0 if y < 0 else 1}",
            start=(-1.00, y, 0.080),
            end=(-1.12, y, 0.095),
            thickness=0.026,
            material=steel,
        )

    # Front leg-roller support yoke and support axle.
    frame.visual(
        Box((0.190, 0.055, 0.050)),
        origin=Origin(xyz=(0.845, 0.0, 0.560)),
        material=steel,
        name="roller_yoke_bridge",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.640),
        origin=Origin(xyz=(0.88, 0.0, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="roller_axle",
    )

    # Long declined back pad on a central horizontal hinge.
    back_pad = model.part("back_pad")
    back_pad.visual(
        Cylinder(radius=0.019, length=0.170),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    for y in (-0.055, 0.055):
        back_pad.visual(
            Box((0.105, 0.030, 0.024)),
            origin=Origin(xyz=(-0.050, y, 0.028)),
            material=dark_steel,
            name=f"hinge_tab_{0 if y < 0 else 1}",
        )
    back_pad.visual(
        _rounded_box("back_cushion_mesh", (1.160, 0.335, 0.078), 0.020),
        origin=Origin(xyz=(-0.585, 0.0, 0.074)),
        material=vinyl,
        name="back_cushion",
    )
    back_pad.visual(
        Box((1.020, 0.010, 0.004)),
        origin=Origin(xyz=(-0.620, 0.0, 0.111)),
        material=seam,
        name="center_seam",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.018, length=0.155),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    seat.visual(
        Box((0.095, 0.095, 0.035)),
        origin=Origin(xyz=(-0.045, 0.0, 0.026)),
        material=dark_steel,
        name="hinge_tab",
    )
    seat.visual(
        _rounded_box("seat_cushion_mesh", (0.420, 0.330, 0.078), 0.020),
        origin=Origin(xyz=(-0.215, 0.0, 0.074)),
        material=vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.310, 0.010, 0.004)),
        origin=Origin(xyz=(-0.230, 0.0, 0.111)),
        material=seam,
        name="center_seam",
    )

    # Distinct folding mid-frame brace, hinged from the frame below the seat.
    brace = model.part("mid_brace")
    brace.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    brace.visual(
        Box((0.416, 0.042, 0.034)),
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        material=steel,
        name="brace_tube",
    )
    brace.visual(
        Box((0.060, 0.150, 0.026)),
        origin=Origin(xyz=(0.440, 0.0, 0.000)),
        material=dark_steel,
        name="seat_stop_pad",
    )

    # Individually spinning, hollow foam leg rollers on the front support axle.
    roller_mesh = mesh_from_geometry(
        TireGeometry(
            0.078,
            0.160,
            inner_radius=0.023,
            carcass=TireCarcass(belt_width_ratio=0.86, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.004, count=16, land_ratio=0.65),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "leg_roller_foam",
    )
    leg_roller_0 = model.part("leg_roller_0")
    leg_roller_0.visual(
        roller_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=foam,
        name="foam_sleeve",
    )
    leg_roller_0.visual(
        Cylinder(radius=0.024, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_sleeve",
    )
    leg_roller_1 = model.part("leg_roller_1")
    leg_roller_1.visual(
        roller_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=foam,
        name="foam_sleeve",
    )
    leg_roller_1.visual(
        Cylinder(radius=0.024, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_sleeve",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.076,
            0.040,
            inner_radius=0.048,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.03),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.60),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "transport_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.049,
            0.036,
            rim=WheelRim(inner_radius=0.031, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(radius=0.016, width=0.026, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.026),
        ),
        "transport_wheel_rim",
    )
    transport_wheel_0 = model.part("transport_wheel_0")
    transport_wheel_0.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    transport_wheel_0.visual(
        rim_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim,
        name="rim",
    )
    transport_wheel_0.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_sleeve",
    )
    transport_wheel_1 = model.part("transport_wheel_1")
    transport_wheel_1.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    transport_wheel_1.visual(
        rim_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim,
        name="rim",
    )
    transport_wheel_1.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_sleeve",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(-0.10, 0.0, 0.520), rpy=(0.0, -0.18, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.5, lower=-0.22, upper=0.82),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.35, 0.0, 0.545), rpy=(0.0, -0.04, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5, lower=-0.18, upper=0.62),
    )
    model.articulation(
        "brace_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace,
        origin=Origin(xyz=(0.02, 0.0, 0.235), rpy=(0.0, -0.46, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.75, upper=0.25),
    )
    for name, child, y in (
        ("leg_roller_joint_0", leg_roller_0, -0.205),
        ("leg_roller_joint_1", leg_roller_1, 0.205),
    ):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=child,
            origin=Origin(xyz=(0.88, y, 0.585)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )
    for name, child, y in (
        ("transport_wheel_joint_0", transport_wheel_0, -0.285),
        ("transport_wheel_joint_1", transport_wheel_1, 0.285),
    ):
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=child,
            origin=Origin(xyz=(-1.12, y, 0.086)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat = object_model.get_part("seat")
    brace = object_model.get_part("mid_brace")
    roller_0 = object_model.get_part("leg_roller_0")
    roller_1 = object_model.get_part("leg_roller_1")
    wheel_0 = object_model.get_part("transport_wheel_0")
    wheel_1 = object_model.get_part("transport_wheel_1")

    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    brace_hinge = object_model.get_articulation("brace_hinge")

    # Captured shafts and hinge pins are intentionally modeled as nested solid
    # proxy cylinders so the support path is visible and mechanically explicit.
    captured_interfaces = (
        (back_pad, "back_hinge_pin", "hinge_sleeve", 0.13, "central back-pad hinge pin captured in the pad sleeve"),
        (seat, "seat_hinge_pin", "hinge_sleeve", 0.12, "seat hinge pin captured in the seat sleeve"),
        (brace, "brace_hinge_pin", "hinge_sleeve", 0.09, "folding brace hinge pin captured in the brace sleeve"),
        (roller_0, "roller_axle", "bearing_sleeve", 0.10, "front roller bearing sleeve rides on the fixed support axle"),
        (roller_1, "roller_axle", "bearing_sleeve", 0.10, "front roller bearing sleeve rides on the fixed support axle"),
        (wheel_0, "transport_axle", "hub_sleeve", 0.025, "transport wheel hub sleeve is captured on the fixed axle"),
        (wheel_1, "transport_axle", "hub_sleeve", 0.025, "transport wheel hub sleeve is captured on the fixed axle"),
    )
    for child, frame_elem, child_elem, y_overlap, reason in captured_interfaces:
        ctx.allow_overlap(frame, child, elem_a=frame_elem, elem_b=child_elem, reason=reason)
        ctx.expect_within(
            frame,
            child,
            axes="xz",
            inner_elem=frame_elem,
            outer_elem=child_elem,
            margin=0.002,
            name=f"{child.name} axle centered in sleeve",
        )
        ctx.expect_overlap(
            frame,
            child,
            axes="y",
            elem_a=frame_elem,
            elem_b=child_elem,
            min_overlap=y_overlap,
            name=f"{child.name} sleeve retains captured axle",
        )

    ctx.expect_gap(
        back_pad,
        frame,
        axis="z",
        positive_elem="back_cushion",
        negative_elem="lower_spine",
        min_gap=0.070,
        name="declined back pad clears the low frame spine",
    )
    ctx.expect_gap(
        seat,
        brace,
        axis="z",
        positive_elem="seat_cushion",
        negative_elem="seat_stop_pad",
        min_gap=0.025,
        name="seat cushion sits above the folding brace",
    )

    def _max_z(aabb):
        return None if aabb is None else aabb[1][2]

    back_rest = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    with ctx.pose({back_hinge: 0.58}):
        back_raised = ctx.part_element_world_aabb(back_pad, elem="back_cushion")
    ctx.check(
        "back pad hinge raises the long pad",
        back_rest is not None
        and back_raised is not None
        and _max_z(back_raised) is not None
        and _max_z(back_rest) is not None
        and _max_z(back_raised) > _max_z(back_rest) + 0.10,
        details=f"rest={back_rest}, raised={back_raised}",
    )

    seat_rest = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    with ctx.pose({seat_hinge: 0.45}):
        seat_raised = ctx.part_element_world_aabb(seat, elem="seat_cushion")
    ctx.check(
        "seat hinge tilts the seat pad upward",
        seat_rest is not None
        and seat_raised is not None
        and _max_z(seat_raised) is not None
        and _max_z(seat_rest) is not None
        and _max_z(seat_raised) > _max_z(seat_rest) + 0.035,
        details=f"rest={seat_rest}, raised={seat_raised}",
    )

    brace_rest = ctx.part_element_world_aabb(brace, elem="seat_stop_pad")
    with ctx.pose({brace_hinge: 0.25}):
        brace_tucked = ctx.part_element_world_aabb(brace, elem="seat_stop_pad")
    ctx.check(
        "brace hinge tucks the brace down under the bench",
        brace_rest is not None
        and brace_tucked is not None
        and _max_z(brace_tucked) is not None
        and _max_z(brace_rest) is not None
        and _max_z(brace_tucked) < _max_z(brace_rest) - 0.05,
        details=f"rest={brace_rest}, tucked={brace_tucked}",
    )

    return ctx.report()


object_model = build_object_model()
