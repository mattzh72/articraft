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
    TireGeometry,
    TireShoulder,
    TireSidewall,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float, name: str):
    """Mesh-backed rounded rectangular cushion/pad centered on its local origin."""
    shape = cq.Workplane("XY").box(*size)
    safe_radius = min(radius, 0.42 * min(size[0], size[1]))
    if safe_radius > 0:
        shape = shape.edges("|Z").fillet(safe_radius)
    return mesh_from_cadquery(shape, name, tolerance=0.002, angular_tolerance=0.15)


def _tube(length: float, outer_radius: float, inner_radius: float, name: str):
    """Vertical hollow guide tube centered on local origin."""
    shape = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0015, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ergonomic_reclining_chair")

    fabric = model.material("charcoal_woven_fabric", rgba=(0.06, 0.075, 0.085, 1.0))
    fabric_lift = model.material("soft_lumbar_fabric", rgba=(0.10, 0.12, 0.13, 1.0))
    plastic = model.material("satin_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    metal = model.material("brushed_dark_metal", rgba=(0.45, 0.46, 0.44, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    seat_cushion = _rounded_box((0.54, 0.52, 0.09), 0.055, "waterfall_seat_cushion")
    back_cushion = _rounded_box((0.060, 0.46, 0.58), 0.045, "tall_back_cushion")
    lumbar_pad = _rounded_box((0.038, 0.34, 0.13), 0.030, "raised_lumbar_pad")
    arm_pad = _rounded_box((0.38, 0.060, 0.045), 0.022, "soft_arm_pad")
    headrest_pad = _rounded_box((0.075, 0.34, 0.16), 0.040, "height_headrest_pad")
    guide_tube = _tube(0.16, 0.016, 0.0105, "headrest_guide_tube")
    caster_socket_tube = _tube(0.050, 0.032, 0.0145, "caster_socket_tube")
    caster_tire = mesh_from_geometry(
        TireGeometry(
            0.042,
            0.034,
            inner_radius=0.029,
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_rubber_tire",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.135, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=plastic,
        name="central_hub",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=metal,
        name="gas_lift",
    )

    caster_radius = 0.58
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0 + math.pi / 2.0
        radial = (math.cos(angle), math.sin(angle))
        arm_center = (0.30 * radial[0], 0.30 * radial[1], 0.118)
        base.visual(
            Box((0.510, 0.080, 0.040)),
            origin=Origin(xyz=arm_center, rpy=(0.0, 0.0, angle)),
            material=plastic,
            name=f"star_leg_{i}",
        )
        socket_center = (caster_radius * radial[0], caster_radius * radial[1], 0.115)
        base.visual(
            caster_socket_tube,
            origin=Origin(xyz=socket_center),
            material=plastic,
            name=f"caster_socket_{i}",
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.066, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal,
        name="lower_bearing",
    )
    seat.visual(
        Box((0.38, 0.36, 0.030)),
        origin=Origin(xyz=(0.04, 0.0, 0.032)),
        material=plastic,
        name="underseat_plate",
    )
    seat.visual(
        seat_cushion,
        origin=Origin(xyz=(0.07, 0.0, 0.082)),
        material=fabric,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.11, 0.48, 0.040)),
        origin=Origin(xyz=(-0.180, 0.0, 0.052)),
        material=plastic,
        name="rear_hinge_mount",
    )
    for side, arm_y in enumerate((-0.255, 0.255)):
        lug_y = -0.300 if side == 0 else 0.300
        seat.visual(
            Box((0.065, 0.042, 0.090)),
            origin=Origin(xyz=(-0.260, lug_y, 0.070)),
            material=metal,
            name=f"recline_lug_{side}",
        )
        seat.visual(
            Box((0.050, 0.080, 0.020)),
            origin=Origin(xyz=(-0.240, -0.260 if side == 0 else 0.260, 0.040)),
            material=metal,
            name=f"hinge_web_{side}",
        )
        for post_x in (-0.105, 0.205):
            seat.visual(
                Box((0.035, 0.038, 0.205)),
                origin=Origin(xyz=(post_x, arm_y * 1.05, 0.170)),
                material=plastic,
                name=f"arm_post_{side}_{0 if post_x < 0 else 1}",
            )
        seat.visual(
            arm_pad,
            origin=Origin(xyz=(0.055, arm_y * 1.05, 0.285)),
            material=plastic,
            name=f"arm_pad_{side}",
        )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0),
    )

    back = model.part("back")
    back.visual(
        Cylinder(radius=0.024, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="recline_barrel",
    )
    for side, y in enumerate((-0.235, 0.235)):
        back.visual(
            Cylinder(radius=0.018, length=0.72),
            origin=Origin(xyz=(0.0, y, 0.36)),
            material=plastic,
            name=f"side_rail_{side}",
        )
    back.visual(
        Cylinder(radius=0.017, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="top_crossbar",
    )
    back.visual(
        back_cushion,
        origin=Origin(xyz=(-0.035, 0.0, 0.405)),
        material=fabric,
        name="back_cushion",
    )
    back.visual(
        lumbar_pad,
        origin=Origin(xyz=(-0.073, 0.0, 0.305)),
        material=fabric_lift,
        name="lumbar_pad",
    )
    back.visual(
        Box((0.025, 0.24, 0.028)),
        origin=Origin(xyz=(0.070, 0.0, 0.745)),
        material=plastic,
        name="guide_bridge",
    )
    back.visual(
        Box((0.090, 0.035, 0.020)),
        origin=Origin(xyz=(0.040, 0.0, 0.745)),
        material=plastic,
        name="guide_neck",
    )
    back.visual(
        guide_tube,
        origin=Origin(xyz=(0.045, -0.080, 0.745)),
        material=metal,
        name="guide_tube_0",
    )
    back.visual(
        guide_tube,
        origin=Origin(xyz=(0.045, 0.080, 0.745)),
        material=metal,
        name="guide_tube_1",
    )

    recline_upper = 0.38
    model.articulation(
        "seat_to_back",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(-0.270, 0.0, 0.075), rpy=(0.0, -0.18, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=recline_upper),
    )

    headrest_support = model.part("headrest_support")
    headrest_support.visual(
        Cylinder(radius=0.0105, length=0.52),
        origin=Origin(xyz=(0.0, -0.080, 0.060)),
        material=metal,
        name="rod_0",
    )
    headrest_support.visual(
        Cylinder(radius=0.0105, length=0.52),
        origin=Origin(xyz=(0.0, 0.080, 0.060)),
        material=metal,
        name="rod_1",
    )
    headrest_support.visual(
        Cylinder(radius=0.012, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.302), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="support_yoke",
    )
    headrest_support.visual(
        Box((0.045, 0.045, 0.14)),
        origin=Origin(xyz=(-0.030, 0.0, 0.350)),
        material=plastic,
        name="headrest_stem",
    )
    headrest_support.visual(
        headrest_pad,
        origin=Origin(xyz=(-0.070, 0.0, 0.405)),
        material=fabric,
        name="headrest_pad",
    )
    model.articulation(
        "back_to_headrest_support",
        ArticulationType.PRISMATIC,
        parent=back,
        child=headrest_support,
        origin=Origin(xyz=(0.045, 0.0, 0.745)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=0.14),
    )

    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0 + math.pi / 2.0
        radial = (math.cos(angle), math.sin(angle))
        fork = model.part(f"caster_fork_{i}")
        fork.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=metal,
            name="swivel_bearing",
        )
        fork.visual(
            Cylinder(radius=0.012, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=metal,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.066, 0.048, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=plastic,
            name="fork_bridge",
        )
        for side, x in enumerate((-0.026, 0.026)):
            fork.visual(
                Box((0.006, 0.078, 0.098)),
                origin=Origin(xyz=(x, 0.0, -0.063)),
                material=plastic,
                name=f"fork_cheek_{side}",
            )
        fork.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="axle",
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(caster_tire, origin=Origin(), material=rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.030, length=0.034),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="hub",
        )

        model.articulation(
            f"base_to_caster_fork_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(
                xyz=(caster_radius * radial[0], caster_radius * radial[1], 0.090),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )
        model.articulation(
            f"caster_fork_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.065)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    headrest_support = object_model.get_part("headrest_support")
    swivel = object_model.get_articulation("base_to_seat")
    recline = object_model.get_articulation("seat_to_back")
    headrest_slide = object_model.get_articulation("back_to_headrest_support")

    ctx.check(
        "seat has a continuous vertical swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="lower_bearing",
        negative_elem="gas_lift",
        max_gap=0.002,
        max_penetration=0.001,
        name="seat bearing sits on gas lift column",
    )

    ctx.check(
        "back has bounded recline hinge",
        recline.articulation_type == ArticulationType.REVOLUTE
        and recline.motion_limits is not None
        and recline.motion_limits.upper is not None
        and recline.motion_limits.upper > 0.30,
        details=f"type={recline.articulation_type}, limits={recline.motion_limits}",
    )
    for lug_name in ("recline_lug_0", "recline_lug_1"):
        ctx.allow_overlap(
            back,
            seat,
            elem_a="recline_barrel",
            elem_b=lug_name,
            reason="The transverse recline barrel is intentionally captured by the seat hinge lug.",
        )
        ctx.expect_overlap(
            back,
            seat,
            axes="y",
            elem_a="recline_barrel",
            elem_b=lug_name,
            min_overlap=0.005,
            name=f"{lug_name} captures recline barrel end",
        )
    back_rest_aabb = ctx.part_world_aabb(back)
    with ctx.pose({recline: recline.motion_limits.upper}):
        back_reclined_aabb = ctx.part_world_aabb(back)
    ctx.check(
        "positive recline moves backrest rearward",
        back_rest_aabb is not None
        and back_reclined_aabb is not None
        and back_reclined_aabb[0][0] < back_rest_aabb[0][0] - 0.08,
        details=f"rest={back_rest_aabb}, reclined={back_reclined_aabb}",
    )

    ctx.check(
        "headrest support slides prismatically",
        headrest_slide.articulation_type == ArticulationType.PRISMATIC
        and headrest_slide.motion_limits is not None
        and headrest_slide.motion_limits.upper is not None
        and headrest_slide.motion_limits.upper >= 0.12,
        details=f"type={headrest_slide.articulation_type}, limits={headrest_slide.motion_limits}",
    )
    for rod_name, tube_name in (("rod_0", "guide_tube_0"), ("rod_1", "guide_tube_1")):
        ctx.allow_overlap(
            back,
            headrest_support,
            elem_a=tube_name,
            elem_b=rod_name,
            reason="The height-adjustment rod is intentionally captured inside the guide tube for a sliding fit.",
        )
    ctx.expect_within(
        headrest_support,
        back,
        axes="y",
        inner_elem="rod_0",
        outer_elem="guide_tube_0",
        margin=0.001,
        name="headrest rod is laterally centered in guide tube",
    )
    ctx.expect_overlap(
        headrest_support,
        back,
        axes="z",
        elem_a="rod_0",
        elem_b="guide_tube_0",
        min_overlap=0.10,
        name="headrest rod remains inserted at low setting",
    )
    ctx.expect_overlap(
        headrest_support,
        back,
        axes="z",
        elem_a="rod_1",
        elem_b="guide_tube_1",
        min_overlap=0.10,
        name="second headrest rod remains inserted at low setting",
    )
    headrest_rest_pos = ctx.part_world_position(headrest_support)
    with ctx.pose({headrest_slide: headrest_slide.motion_limits.upper}):
        ctx.expect_overlap(
            headrest_support,
            back,
            axes="z",
            elem_a="rod_0",
            elem_b="guide_tube_0",
            min_overlap=0.05,
            name="headrest rod remains inserted when raised",
        )
        ctx.expect_overlap(
            headrest_support,
            back,
            axes="z",
            elem_a="rod_1",
            elem_b="guide_tube_1",
            min_overlap=0.05,
            name="second headrest rod remains inserted when raised",
        )
        headrest_high_pos = ctx.part_world_position(headrest_support)
    ctx.check(
        "headrest upper setting rises along back frame",
        headrest_rest_pos is not None
        and headrest_high_pos is not None
        and headrest_high_pos[2] > headrest_rest_pos[2] + 0.11,
        details=f"low={headrest_rest_pos}, high={headrest_high_pos}",
    )

    for i in range(5):
        caster_swivel = object_model.get_articulation(f"base_to_caster_fork_{i}")
        wheel_spin = object_model.get_articulation(f"caster_fork_to_wheel_{i}")
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.check(
            f"caster {i} swivels and wheel spins",
            caster_swivel.articulation_type == ArticulationType.CONTINUOUS
            and wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
            details=f"swivel={caster_swivel.articulation_type}, spin={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="x",
            margin=0.003,
            name=f"caster {i} wheel fits between fork cheeks",
        )
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle",
            elem_b="hub",
            reason="The wheel hub rotates around a fixed axle represented as passing through the solid hub.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="x",
            elem_a="axle",
            elem_b="hub",
            min_overlap=0.030,
            name=f"caster {i} axle passes through wheel hub",
        )

    return ctx.report()


object_model = build_object_model()
