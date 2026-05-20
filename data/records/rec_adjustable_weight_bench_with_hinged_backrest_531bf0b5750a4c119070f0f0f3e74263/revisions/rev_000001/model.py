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
    TireCarcass,
    TireGeometry,
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


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small bevel/fillet helper for gym-bench upholstery pads."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_adjustable_bench")

    steel = model.material("black_powder_coated_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_steel = model.material("dark_welded_steel", rgba=(0.006, 0.007, 0.008, 1.0))
    upholstery = model.material("black_vinyl_upholstery", rgba=(0.015, 0.016, 0.018, 1.0))
    plate_finish = model.material("matte_backing_plate", rgba=(0.03, 0.035, 0.04, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    foam = model.material("dense_foam_roller", rgba=(0.018, 0.018, 0.020, 1.0))

    pad_mesh_back = mesh_from_cadquery(_rounded_box((0.78, 0.37, 0.078), 0.024), "back_pad")
    pad_mesh_seat = mesh_from_cadquery(_rounded_box((0.40, 0.36, 0.075), 0.022), "seat_pad")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.058,
            0.038,
            inner_radius=0.040,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.003, count=16, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "transport_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.041,
            0.030,
            rim=WheelRim(inner_radius=0.027, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(radius=0.013, width=0.026, cap_style="flat"),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "transport_wheel",
    )

    frame = model.part("frame")

    # Heavy welded base: a long central spine, wide stabilizer feet, hinge towers,
    # and a rigid front yoke that carries the roller axle.
    frame.visual(Box((1.36, 0.075, 0.070)), origin=Origin(xyz=(0.00, 0.00, 0.180)), material=dark_steel, name="main_spine")
    frame.visual(Box((0.18, 0.82, 0.065)), origin=Origin(xyz=(-0.74, 0.00, 0.070)), material=dark_steel, name="rear_stabilizer")
    frame.visual(Box((0.17, 0.66, 0.060)), origin=Origin(xyz=(0.64, 0.00, 0.075)), material=dark_steel, name="front_stabilizer")
    frame.visual(Box((0.080, 0.090, 0.190)), origin=Origin(xyz=(-0.68, 0.00, 0.125)), material=dark_steel, name="rear_drop")
    frame.visual(Box((0.080, 0.100, 0.220)), origin=Origin(xyz=(0.59, 0.00, 0.175)), material=dark_steel, name="front_drop")

    # Broad top towers and side plates make the pad hinges visibly above the frame.
    frame.visual(Box((0.090, 0.500, 0.045)), origin=Origin(xyz=(-0.12, 0.00, 0.435)), material=steel, name="back_hinge_bridge")
    frame.visual(Box((0.075, 0.120, 0.270)), origin=Origin(xyz=(-0.12, 0.00, 0.320)), material=steel, name="back_hinge_pedestal")
    for y in (-0.235, 0.235):
        frame.visual(Box((0.060, 0.040, 0.155)), origin=Origin(xyz=(-0.12, y, 0.480)), material=steel, name=f"back_hinge_tab_{0 if y < 0 else 1}")
    frame.visual(
        Cylinder(radius=0.012, length=0.455),
        origin=Origin(xyz=(-0.12, 0.0, 0.500), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="back_hinge_pin",
    )

    frame.visual(Box((0.090, 0.460, 0.045)), origin=Origin(xyz=(0.30, 0.00, 0.425)), material=steel, name="seat_hinge_bridge")
    frame.visual(Box((0.075, 0.120, 0.265)), origin=Origin(xyz=(0.30, 0.00, 0.315)), material=steel, name="seat_hinge_pedestal")
    for y in (-0.215, 0.215):
        frame.visual(Box((0.056, 0.038, 0.145)), origin=Origin(xyz=(0.30, y, 0.470)), material=steel, name=f"seat_hinge_tab_{0 if y < 0 else 1}")
    frame.visual(
        Cylinder(radius=0.011, length=0.410),
        origin=Origin(xyz=(0.30, 0.0, 0.475), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="seat_hinge_pin",
    )

    # Lower rear support-arm pivot, with cheeks tied into the spine.
    frame.visual(Box((0.080, 0.410, 0.035)), origin=Origin(xyz=(-0.60, 0.00, 0.205)), material=steel, name="support_pivot_bridge")
    frame.visual(Box((0.070, 0.120, 0.090)), origin=Origin(xyz=(-0.60, 0.00, 0.175)), material=steel, name="support_pivot_post")
    for y in (-0.180, 0.180):
        frame.visual(Box((0.060, 0.035, 0.095)), origin=Origin(xyz=(-0.60, y, 0.255)), material=steel, name=f"support_pivot_tab_{0 if y < 0 else 1}")
    frame.visual(
        Cylinder(radius=0.012, length=0.390),
        origin=Origin(xyz=(-0.60, 0.0, 0.255), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="support_pivot_pin",
    )

    # Transport-wheel mounting tabs and visible axle stubs.
    for y, idx in ((-0.392, 0), (0.392, 1)):
        frame.visual(Box((0.105, 0.026, 0.095)), origin=Origin(xyz=(-0.82, y, 0.095)), material=steel, name=f"wheel_mount_{idx}")
        frame.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(xyz=(-0.82, -0.423 if y < 0 else 0.423, 0.081), rpy=(math.pi / 2, 0.0, 0.0)),
            material=brushed,
            name=f"wheel_axle_stub_{idx}",
        )

    # Rigid front yoke with a through axle for the two foam foot rollers.
    frame.visual(Box((0.100, 0.115, 0.300)), origin=Origin(xyz=(0.70, 0.00, 0.325)), material=steel, name="front_yoke_stem")
    frame.visual(Box((0.185, 0.690, 0.055)), origin=Origin(xyz=(0.78, 0.00, 0.340)), material=steel, name="front_yoke_bridge")
    for y in (-0.315, 0.315):
        frame.visual(Box((0.135, 0.045, 0.345)), origin=Origin(xyz=(0.82, y, 0.425)), material=steel, name=f"front_yoke_cheek_{0 if y < 0 else 1}")
    frame.visual(
        Cylinder(radius=0.017, length=0.610),
        origin=Origin(xyz=(0.82, 0.00, 0.500), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="front_axle",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.024, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="hinge_barrel",
    )
    backrest.visual(Box((0.780, 0.335, 0.026)), origin=Origin(xyz=(-0.380, 0.0, 0.022)), material=plate_finish, name="plate")
    backrest.visual(pad_mesh_back, origin=Origin(xyz=(-0.430, 0.0, 0.073)), material=upholstery, name="pad")
    # Underside stop blocks are welded to the backrest plate but leave the
    # separate support arm visibly independent.
    backrest.visual(Box((0.055, 0.275, 0.030)), origin=Origin(xyz=(-0.155, 0.0, -0.004)), material=plate_finish, name="support_stop")

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.022, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="hinge_barrel",
    )
    seat.visual(Box((0.090, 0.285, 0.036)), origin=Origin(xyz=(-0.060, 0.0, 0.018)), material=plate_finish, name="hinge_web")
    seat.visual(Box((0.380, 0.325, 0.025)), origin=Origin(xyz=(-0.160, 0.0, 0.020)), material=plate_finish, name="plate")
    seat.visual(pad_mesh_seat, origin=Origin(xyz=(-0.195, 0.0, 0.071)), material=upholstery, name="pad")

    support_arm = model.part("support_arm")
    for y in (-0.115, 0.115):
        support_arm.visual(Box((0.375, 0.045, 0.036)), origin=Origin(xyz=(0.2125, y, 0.0)), material=steel, name=f"side_rail_{0 if y < 0 else 1}")
    support_arm.visual(
        Cylinder(radius=0.026, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="lower_pivot_barrel",
    )
    support_arm.visual(
        Cylinder(radius=0.025, length=0.320),
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brushed,
        name="upper_crossbar",
    )
    support_arm.visual(Box((0.170, 0.260, 0.020)), origin=Origin(xyz=(0.195, 0.0, -0.018)), material=steel, name="wide_web")

    for y, idx in ((-0.455, 0), (0.455, 1)):
        wheel = model.part(f"transport_wheel_{idx}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(wheel_mesh, material=brushed, name="rim")
        model.articulation(
            f"frame_to_transport_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.82, y, 0.081), rpy=(0.0, 0.0, math.pi / 2)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=12.0),
        )

    for y, idx in ((-0.150, 0), (0.150, 1)):
        roller = model.part(f"front_roller_{idx}")
        roller.visual(
            Cylinder(radius=0.055, length=0.205),
            origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
            material=foam,
            name="foam",
        )
        for local_y, cap_idx in ((-0.108, 0), (0.108, 1)):
            roller.visual(
                Cylinder(radius=0.034, length=0.012),
                origin=Origin(xyz=(0.0, local_y, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
                material=brushed,
                name=f"end_cap_{cap_idx}",
            )
        model.articulation(
            f"frame_to_front_roller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(0.82, y, 0.500)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=10.0),
        )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.12, 0.0, 0.500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.1, lower=0.0, upper=1.12),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.30, 0.0, 0.475)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.9, lower=0.0, upper=0.38),
    )
    model.articulation(
        "frame_to_support_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.60, 0.0, 0.255), rpy=(0.0, -0.25, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.9, lower=-0.28, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_arm = object_model.get_part("support_arm")
    roller_0 = object_model.get_part("front_roller_0")
    roller_1 = object_model.get_part("front_roller_1")
    back_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    support_hinge = object_model.get_articulation("frame_to_support_arm")

    for child, pin_name, barrel_name, label in (
        (backrest, "back_hinge_pin", "hinge_barrel", "backrest"),
        (seat, "seat_hinge_pin", "hinge_barrel", "seat"),
        (support_arm, "support_pivot_pin", "lower_pivot_barrel", "support arm"),
    ):
        ctx.allow_overlap(
            frame,
            child,
            elem_a=pin_name,
            elem_b=barrel_name,
            reason=f"The {label} rotates around a captured transverse hinge pin.",
        )
        ctx.expect_within(
            frame,
            child,
            axes="xz",
            inner_elem=pin_name,
            outer_elem=barrel_name,
            margin=0.002,
            name=f"{label} hinge pin is centered in barrel",
        )
        ctx.expect_overlap(
            frame,
            child,
            axes="y",
            elem_a=pin_name,
            elem_b=barrel_name,
            min_overlap=0.25,
            name=f"{label} hinge barrel is retained on pin",
        )

    # The front foam rollers are intentionally captured around the steel yoke
    # axle; scope the interpenetration to the named axle/foam interface.
    for roller in (roller_0, roller_1):
        ctx.allow_overlap(
            frame,
            roller,
            elem_a="front_axle",
            elem_b="foam",
            reason="The front foam roller is modeled as a captured rotating sleeve around the shared steel axle.",
        )
        ctx.expect_within(
            frame,
            roller,
            axes="xz",
            inner_elem="front_axle",
            outer_elem="foam",
            margin=0.002,
            name=f"{roller.name} axle centered in foam",
        )
        ctx.expect_overlap(
            frame,
            roller,
            axes="y",
            elem_a="front_axle",
            elem_b="foam",
            min_overlap=0.18,
            name=f"{roller.name} remains sleeved on shared axle",
        )
        for cap_name in ("end_cap_0", "end_cap_1"):
            ctx.allow_overlap(
                frame,
                roller,
                elem_a="front_axle",
                elem_b=cap_name,
                reason="The steel axle passes through the roller end cap as a captured rotating sleeve.",
            )
            ctx.expect_overlap(
                frame,
                roller,
                axes="y",
                elem_a="front_axle",
                elem_b=cap_name,
                min_overlap=0.010,
                name=f"{roller.name} {cap_name} captured by axle",
            )

    for idx in (0, 1):
        wheel = object_model.get_part(f"transport_wheel_{idx}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=f"wheel_axle_stub_{idx}",
            elem_b="rim",
            reason="The small transport wheel rim is captured on the axle stub at its bore.",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem=f"wheel_axle_stub_{idx}",
            outer_elem="rim",
            margin=0.003,
            name=f"transport wheel {idx} axle stub is centered in rim",
        )

    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        positive_elem="pad",
        negative_elem="pad",
        min_gap=0.030,
        max_gap=0.090,
        name="visible split between seat and back pads",
    )
    ctx.expect_gap(
        backrest,
        support_arm,
        axis="z",
        positive_elem="plate",
        negative_elem="upper_crossbar",
        min_gap=0.010,
        max_gap=0.150,
        name="support arm is separate below backrest plate",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.75}):
        raised_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest hinge raises rear pad",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.20,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.32}):
        raised_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat forward hinge tips pad upward",
        rest_seat_aabb is not None
        and raised_seat_aabb is not None
        and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.05,
        details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
    )

    rest_arm_aabb = ctx.part_world_aabb(support_arm)
    with ctx.pose({support_hinge: 0.40}):
        swung_arm_aabb = ctx.part_world_aabb(support_arm)
    ctx.check(
        "wide rear support arm rotates on lower pivot",
        rest_arm_aabb is not None
        and swung_arm_aabb is not None
        and abs(swung_arm_aabb[1][2] - rest_arm_aabb[1][2]) > 0.08,
        details=f"rest={rest_arm_aabb}, swung={swung_arm_aabb}",
    )

    continuous_names = (
        "frame_to_transport_wheel_0",
        "frame_to_transport_wheel_1",
        "frame_to_front_roller_0",
        "frame_to_front_roller_1",
    )
    for joint_name in continuous_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            getattr(joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={getattr(joint, 'articulation_type', None)}",
        )

    return ctx.report()


object_model = build_object_model()
