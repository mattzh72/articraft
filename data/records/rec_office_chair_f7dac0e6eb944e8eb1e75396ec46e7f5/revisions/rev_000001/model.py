from __future__ import annotations

import math

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


def _rounded_seat_cushion() -> cq.Workplane:
    """A broad rounded rectangle cushion, centered on its local origin."""
    return cq.Workplane("XY").box(0.54, 0.52, 0.09).edges("|Z").fillet(0.065)


def _rounded_back_pad() -> cq.Workplane:
    """A slim rounded back insert in the local YZ plane."""
    return cq.Workplane("XY").box(0.045, 0.44, 0.62).edges("|X").fillet(0.055)


def _rounded_headrest_pad() -> cq.Workplane:
    """A small padded headrest with rounded ends."""
    return cq.Workplane("XY").box(0.070, 0.34, 0.14).edges("|X").fillet(0.040)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_office_chair")

    black_plastic = model.material("satin_black_plastic", rgba=(0.015, 0.015, 0.018, 1.0))
    charcoal_fabric = model.material("charcoal_woven_fabric", rgba=(0.055, 0.060, 0.065, 1.0))
    dark_mesh = model.material("translucent_black_mesh", rgba=(0.025, 0.030, 0.035, 0.72))
    brushed_metal = model.material("brushed_grey_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    rim_material = model.material("dark_caster_rim", rgba=(0.11, 0.11, 0.12, 1.0))

    seat_cushion_mesh = mesh_from_cadquery(_rounded_seat_cushion(), "rounded_seat_cushion")
    back_pad_mesh = mesh_from_cadquery(_rounded_back_pad(), "rounded_back_pad")
    headrest_pad_mesh = mesh_from_cadquery(_rounded_headrest_pad(), "rounded_headrest_pad")

    caster_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.030,
            0.028,
            rim=WheelRim(inner_radius=0.021, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.014,
                width=0.020,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.018, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "caster_wheel_rim",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.043,
            0.034,
            inner_radius=0.031,
            carcass=TireCarcass(belt_width_ratio=0.76, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.68),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_rubber_tire",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.078, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=black_plastic,
        name="central_hub",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=brushed_metal,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.061, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=black_plastic,
        name="sleeve_collar",
    )

    leg_count = 5
    leg_center_radius = 0.250
    leg_length = 0.390
    caster_radius = 0.460
    caster_pivot_z = 0.105
    for i in range(leg_count):
        theta = 2.0 * math.pi * i / leg_count + math.pi / 2.0
        c = math.cos(theta)
        s = math.sin(theta)
        base.visual(
            Box((leg_length, 0.060, 0.036)),
            origin=Origin(
                xyz=(leg_center_radius * c, leg_center_radius * s, 0.070),
                rpy=(0.0, 0.0, theta),
            ),
            material=black_plastic,
            name=f"star_leg_{i}",
        )
        base.visual(
            Box((0.310, 0.030, 0.018)),
            origin=Origin(
                xyz=((leg_center_radius + 0.035) * c, (leg_center_radius + 0.035) * s, 0.094),
                rpy=(0.0, 0.0, theta),
            ),
            material=black_plastic,
            name=f"leg_rib_{i}",
        )
        base.visual(
            Cylinder(radius=0.026, length=0.026),
            origin=Origin(
                xyz=(caster_radius * c, caster_radius * s, 0.092),
                rpy=(0.0, 0.0, theta),
            ),
            material=black_plastic,
            name=f"caster_socket_{i}",
        )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.032, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=brushed_metal,
        name="lift_post",
    )
    lift_column.visual(
        Cylinder(radius=0.061, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black_plastic,
        name="top_bearing",
    )

    seat_carrier = model.part("seat_carrier")
    seat_carrier.visual(
        Box((0.340, 0.280, 0.035)),
        origin=Origin(xyz=(0.030, 0.0, 0.025)),
        material=black_plastic,
        name="swivel_plate",
    )
    seat_carrier.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.080, 0.0, 0.055)),
        material=charcoal_fabric,
        name="seat_cushion",
    )
    seat_carrier.visual(
        Box((0.110, 0.520, 0.045)),
        origin=Origin(xyz=(-0.215, 0.0, 0.035)),
        material=black_plastic,
        name="rear_mechanism",
    )
    for side, y in enumerate((-0.225, 0.225)):
        seat_carrier.visual(
            Box((0.060, 0.026, 0.115)),
            origin=Origin(xyz=(-0.235, y, 0.085)),
            material=black_plastic,
            name=f"hinge_clevis_{side}",
        )
        seat_carrier.visual(
            Box((0.045, 0.040, 0.180)),
            origin=Origin(xyz=(0.010, y * 1.24, 0.185)),
            material=black_plastic,
            name=f"arm_post_{side}",
        )
        seat_carrier.visual(
            Box((0.075, 0.082, 0.035)),
            origin=Origin(xyz=(0.010, y * 1.24, 0.098)),
            material=black_plastic,
            name=f"arm_base_{side}",
        )
        seat_carrier.visual(
            Box((0.330, 0.060, 0.035)),
            origin=Origin(xyz=(0.105, y * 1.24, 0.282)),
            material=black_plastic,
            name=f"arm_pad_{side}",
        )
    seat_carrier.visual(
        Box((0.030, 0.150, 0.018)),
        origin=Origin(xyz=(0.170, -0.305, 0.150), rpy=(0.0, 0.0, -0.35)),
        material=black_plastic,
        name="height_lever",
    )
    seat_carrier.visual(
        Box((0.070, 0.055, 0.055)),
        origin=Origin(xyz=(0.140, -0.265, 0.115)),
        material=black_plastic,
        name="lever_pivot",
    )

    back_frame = model.part("back_frame")
    back_frame.visual(
        Cylinder(radius=0.022, length=0.424),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="hinge_barrel",
    )
    for side, y in enumerate((-0.175, 0.175)):
        back_frame.visual(
            Box((0.048, 0.032, 0.145)),
            origin=Origin(xyz=(-0.025, y, 0.065)),
            material=black_plastic,
            name=f"lower_strut_{side}",
        )
    back_frame.visual(
        back_pad_mesh,
        origin=Origin(xyz=(-0.058, 0.0, 0.430)),
        material=dark_mesh,
        name="back_panel",
    )
    for side, y in enumerate((-0.255, 0.255)):
        back_frame.visual(
            Box((0.052, 0.038, 0.680)),
            origin=Origin(xyz=(-0.060, y, 0.430)),
            material=black_plastic,
            name=f"side_rail_{side}",
        )
    back_frame.visual(
        Box((0.055, 0.550, 0.040)),
        origin=Origin(xyz=(-0.060, 0.0, 0.125)),
        material=black_plastic,
        name="lower_crossbar",
    )
    back_frame.visual(
        Box((0.055, 0.520, 0.040)),
        origin=Origin(xyz=(-0.060, 0.0, 0.750)),
        material=black_plastic,
        name="top_crossbar",
    )

    headrest_bracket = model.part("headrest_bracket")
    headrest_bracket.visual(
        Box((0.044, 0.230, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_plastic,
        name="mount_foot",
    )
    for side, y in enumerate((-0.085, 0.085)):
        headrest_bracket.visual(
            Cylinder(radius=0.012, length=0.136),
            origin=Origin(xyz=(-0.010, y, 0.073)),
            material=brushed_metal,
            name=f"support_rod_{side}",
        )
        headrest_bracket.visual(
            Box((0.034, 0.030, 0.070)),
            origin=Origin(xyz=(-0.012, y * 2.05, 0.165)),
            material=black_plastic,
            name=f"tilt_lug_{side}",
        )
    headrest_bracket.visual(
        Box((0.034, 0.320, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, 0.140)),
        material=black_plastic,
        name="upper_bridge",
    )

    headrest = model.part("headrest")
    headrest.visual(
        Cylinder(radius=0.017, length=0.260),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="tilt_barrel",
    )
    headrest.visual(
        Box((0.035, 0.200, 0.070)),
        origin=Origin(xyz=(-0.025, 0.0, 0.045)),
        material=black_plastic,
        name="pad_stem",
    )
    headrest.visual(
        headrest_pad_mesh,
        origin=Origin(xyz=(-0.055, 0.0, 0.090)),
        material=charcoal_fabric,
        name="headrest_pad",
    )

    model.articulation(
        "column_height",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.20, lower=0.0, upper=0.120),
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=75.0, velocity=2.0),
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat_carrier,
        child=back_frame,
        origin=Origin(xyz=(-0.235, 0.0, 0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=0.45),
    )
    model.articulation(
        "headrest_mount",
        ArticulationType.FIXED,
        parent=back_frame,
        child=headrest_bracket,
        origin=Origin(xyz=(-0.060, 0.0, 0.770)),
    )
    model.articulation(
        "headrest_tilt",
        ArticulationType.REVOLUTE,
        parent=headrest_bracket,
        child=headrest,
        origin=Origin(xyz=(-0.012, 0.0, 0.165)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.35, upper=0.35),
    )

    for i in range(leg_count):
        theta = 2.0 * math.pi * i / leg_count + math.pi / 2.0
        c = math.cos(theta)
        s = math.sin(theta)
        fork = model.part(f"caster_fork_{i}")
        fork.visual(
            Cylinder(radius=0.015, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=brushed_metal,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.120, 0.070, 0.014)),
            origin=Origin(xyz=(0.070, 0.0, 0.012)),
            material=black_plastic,
            name="fork_bridge",
        )
        for side, y in enumerate((-0.029, 0.029)):
            fork.visual(
                Box((0.040, 0.008, 0.095)),
                origin=Origin(xyz=(0.085, y, -0.036)),
                material=black_plastic,
                name=f"fork_cheek_{side}",
            )
        fork.visual(
            Cylinder(radius=0.008, length=0.074),
            origin=Origin(xyz=(0.085, 0.0, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="axle_pin",
        )

        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            caster_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            caster_rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_material,
            name="rim",
        )

        model.articulation(
            f"caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(
                xyz=(caster_radius * c, caster_radius * s, caster_pivot_z),
                rpy=(0.0, 0.0, theta),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )
        model.articulation(
            f"wheel_axle_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.085, 0.0, -0.055)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat_carrier")
    back = object_model.get_part("back_frame")
    bracket = object_model.get_part("headrest_bracket")
    headrest = object_model.get_part("headrest")
    height = object_model.get_articulation("column_height")
    recline = object_model.get_articulation("back_recline")
    headrest_tilt = object_model.get_articulation("headrest_tilt")

    ctx.allow_overlap(
        base,
        lift_column,
        elem_a="outer_sleeve",
        elem_b="lift_post",
        reason="The chrome gas-lift post is intentionally retained inside the simplified outer sleeve.",
    )
    ctx.allow_overlap(
        base,
        lift_column,
        elem_a="sleeve_collar",
        elem_b="lift_post",
        reason="The molded collar is a simplified solid proxy for the ring that surrounds the gas-lift post.",
    )
    ctx.allow_overlap(
        base,
        lift_column,
        elem_a="central_hub",
        elem_b="lift_post",
        reason="The gas-lift post penetrates the central five-star hub as a captured pedestal socket.",
    )

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="lift_post",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="lift post stays centered in sleeve",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="lift_post",
        elem_b="outer_sleeve",
        min_overlap=0.130,
        name="lift post retained when lowered",
    )
    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="lift_post",
        outer_elem="central_hub",
        margin=0.002,
        name="lift post centered in central hub",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="lift_post",
        elem_b="central_hub",
        min_overlap=0.015,
        name="lift post seated in central hub",
    )

    rest_seat = ctx.part_world_position(seat)
    with ctx.pose({height: 0.120}):
        ctx.expect_within(
            lift_column,
            base,
            axes="xy",
            inner_elem="lift_post",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="raised lift post stays centered",
        )
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="lift_post",
            elem_b="outer_sleeve",
            min_overlap=0.080,
            name="lift post retained when raised",
        )
        raised_seat = ctx.part_world_position(seat)

    ctx.check(
        "height adjustment raises seat",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.10,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    ctx.expect_contact(
        bracket,
        back,
        elem_a="mount_foot",
        elem_b="top_crossbar",
        contact_tol=0.002,
        name="headrest bracket sits on back frame",
    )

    rest_back_aabb = ctx.part_element_world_aabb(back, elem="back_panel")
    with ctx.pose({recline: 0.35}):
        reclined_back_aabb = ctx.part_element_world_aabb(back, elem="back_panel")
    ctx.check(
        "positive recline moves back rearward",
        rest_back_aabb is not None
        and reclined_back_aabb is not None
        and (reclined_back_aabb[0][0] + reclined_back_aabb[1][0]) * 0.5
        < (rest_back_aabb[0][0] + rest_back_aabb[1][0]) * 0.5
        - 0.06,
        details=f"rest={rest_back_aabb}, reclined={reclined_back_aabb}",
    )

    rest_head_aabb = ctx.part_element_world_aabb(headrest, elem="headrest_pad")
    with ctx.pose({headrest_tilt: 0.30}):
        tilted_head_aabb = ctx.part_element_world_aabb(headrest, elem="headrest_pad")
    ctx.check(
        "positive headrest tilt nods pad forward",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and (tilted_head_aabb[0][0] + tilted_head_aabb[1][0]) * 0.5
        > (rest_head_aabb[0][0] + rest_head_aabb[1][0]) * 0.5
        + 0.015,
        details=f"rest={rest_head_aabb}, tilted={tilted_head_aabb}",
    )

    for i in range(5):
        fork = object_model.get_part(f"caster_fork_{i}")
        wheel = object_model.get_part(f"caster_wheel_{i}")
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a="axle_pin",
            elem_b="rim",
            reason="The axle pin is intentionally captured through the wheel bearing/rim proxy.",
        )
        ctx.expect_within(
            wheel,
            fork,
            axes="y",
            inner_elem="rim",
            outer_elem="axle_pin",
            margin=0.050,
            name=f"caster wheel {i} captured on axle",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="x",
            elem_a="axle_pin",
            elem_b="rim",
            min_overlap=0.006,
            name=f"caster axle {i} passes through wheel bore",
        )

    return ctx.report()


object_model = build_object_model()
