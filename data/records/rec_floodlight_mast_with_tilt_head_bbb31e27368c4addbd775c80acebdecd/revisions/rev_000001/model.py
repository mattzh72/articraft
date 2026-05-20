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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_tube(z_min: float, z_max: float, outer_radius: float, inner_radius: float):
    """CadQuery annular tube/collar authored in meters in the local frame."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_leg_work_light_tower")

    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_metal = model.material("dark_powder_coat", rgba=(0.04, 0.045, 0.05, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    warm_lens = model.material("warm_frosted_lens", rgba=(1.0, 0.92, 0.55, 0.72))
    led_white = model.material("led_emitters", rgba=(1.0, 0.96, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_hollow_tube(0.25, 0.92, 0.038, 0.024), "base_sleeve"),
        material=aluminum,
        name="base_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_hollow_tube(0.29, 0.36, 0.090, 0.027), "base_ring"),
        material=dark_metal,
        name="base_ring",
    )
    base.visual(
        mesh_from_cadquery(_hollow_tube(0.88, 0.98, 0.056, 0.024), "top_collar"),
        material=dark_metal,
        name="top_collar",
    )

    # Forked clevises on the base ring carry the two leg hinge pins.
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        hinge_x = sign * 0.125
        bracket_x = sign * 0.108
        for y in (-0.038, 0.038):
            base.visual(
                Box((0.050, 0.016, 0.075)),
                origin=Origin(xyz=(bracket_x, y, 0.320)),
                material=dark_metal,
                name=f"clevis_{suffix}_{'inner' if y < 0.0 else 'outer'}",
            )
        pin_name = "leg_pin_0" if suffix == "0" else "leg_pin_1"
        base.visual(
            Cylinder(radius=0.006, length=0.100),
            origin=Origin(xyz=(hinge_x, 0.0, 0.320), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=pin_name,
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.024, length=1.880),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=aluminum,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.038, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.185)),
        material=dark_metal,
        name="top_collar",
    )
    mast.visual(
        Box((0.240, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
        material=dark_metal,
        name="top_bridge",
    )
    for x in (-0.095, 0.095):
        side = "0" if x < 0.0 else "1"
        mast.visual(
            Box((0.018, 0.090, 0.028)),
            origin=Origin(xyz=(x, 0.0, 1.204)),
            material=dark_metal,
            name=f"top_yoke_{side}_bottom",
        )
        mast.visual(
            Box((0.018, 0.090, 0.028)),
            origin=Origin(xyz=(x, 0.0, 1.296)),
            material=dark_metal,
            name=f"top_yoke_{side}_top",
        )
        for y in (-0.037, 0.037):
            mast.visual(
                Box((0.018, 0.016, 0.110)),
                origin=Origin(xyz=(x, y, 1.250)),
                material=dark_metal,
                name=f"top_yoke_{side}_{'front' if y < 0.0 else 'rear'}",
            )
    mast.visual(
        Cylinder(radius=0.007, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 1.250), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="tilt_pin",
    )
    for x in (-0.111, 0.111):
        mast.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(x, 0.0, 1.250), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=f"tilt_washer_{'0' if x < 0.0 else '1'}",
        )

    for sign, leg_name in ((1.0, "leg_0"), (-1.0, "leg_1")):
        leg = model.part(leg_name)
        direction = (sign * 0.900, 0.0, -0.435)
        direction_mag = math.sqrt(sum(component * component for component in direction))
        unit_direction = tuple(component / direction_mag for component in direction)
        tube_start = 0.025
        leg_length = 0.695
        theta = math.atan2(direction[0], direction[2])
        leg.visual(
            Cylinder(radius=0.017, length=leg_length),
            origin=Origin(
                xyz=(
                    unit_direction[0] * (tube_start + leg_length / 2.0),
                    0.0,
                    unit_direction[2] * (tube_start + leg_length / 2.0),
                ),
                rpy=(0.0, theta, 0.0),
            ),
            material=dark_metal,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.055),
            origin=Origin(
                xyz=(
                    unit_direction[0] * 0.0455,
                    0.0,
                    unit_direction[2] * 0.0455,
                ),
                rpy=(0.0, theta, 0.0),
            ),
            material=dark_metal,
            name="hinge_neck",
        )
        leg.visual(
            Cylinder(radius=0.020, length=0.046),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_eye",
        )
        leg.visual(
            Box((0.220, 0.100, 0.034)),
            origin=Origin(xyz=(sign * 0.665, 0.0, -0.298)),
            material=rubber,
            name="foot_pad",
        )
        leg.visual(
            Box((0.075, 0.040, 0.030)),
            origin=Origin(xyz=(sign * 0.610, 0.0, -0.280)),
            material=dark_metal,
            name="foot_socket",
        )

        model.articulation(
            f"base_to_{leg_name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(xyz=(sign * 0.125, 0.0, 0.320)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.45, upper=0.45),
        )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.55),
    )

    lamp_bar = model.part("lamp_bar")
    lamp_bar.visual(
        Cylinder(radius=0.018, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="cross_tube",
    )
    lamp_bar.visual(
        Cylinder(radius=0.022, length=0.150),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tilt_hub",
    )
    lamp_bar.visual(
        Box((0.055, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="hub_riser",
    )

    for index, x in enumerate((-0.235, 0.235)):
        lamp_bar.visual(
            Box((0.080, 0.050, 0.070)),
            origin=Origin(xyz=(x, -0.015, 0.065)),
            material=dark_metal,
            name=f"head_mount_{index}",
        )
        lamp_bar.visual(
            Box((0.250, 0.080, 0.165)),
            origin=Origin(xyz=(x, -0.070, -0.005)),
            material=safety_yellow,
            name=f"head_shell_{index}",
        )
        lamp_bar.visual(
            Box((0.215, 0.010, 0.128)),
            origin=Origin(xyz=(x, -0.115, -0.005)),
            material=warm_lens,
            name=f"lens_{index}",
        )
        lamp_bar.visual(
            Box((0.040, 0.075, 0.040)),
            origin=Origin(xyz=(x, -0.035, 0.020)),
            material=dark_metal,
            name=f"head_yoke_{index}",
        )
        for led_i, led_x in enumerate((-0.055, 0.0, 0.055)):
            lamp_bar.visual(
                Sphere(radius=0.014),
                origin=Origin(xyz=(x + led_x, -0.122, -0.005)),
                material=led_white,
                name=f"led_{index}_{led_i}",
            )

    model.articulation(
        "mast_to_lamp_bar",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=lamp_bar,
        origin=Origin(xyz=(0.0, 0.0, 1.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    leg_0 = object_model.get_part("leg_0")
    leg_1 = object_model.get_part("leg_1")
    mast = object_model.get_part("mast")
    lamp_bar = object_model.get_part("lamp_bar")

    leg_0_hinge = object_model.get_articulation("base_to_leg_0")
    leg_1_hinge = object_model.get_articulation("base_to_leg_1")
    mast_slide = object_model.get_articulation("base_to_mast")
    tilt = object_model.get_articulation("mast_to_lamp_bar")

    ctx.allow_overlap(
        base,
        leg_0,
        elem_a="leg_pin_0",
        elem_b="hinge_eye",
        reason="The metal hinge pin intentionally passes through the leg hinge eye.",
    )
    ctx.allow_overlap(
        base,
        leg_1,
        elem_a="leg_pin_1",
        elem_b="hinge_eye",
        reason="The metal hinge pin intentionally passes through the leg hinge eye.",
    )
    ctx.allow_overlap(
        mast,
        lamp_bar,
        elem_a="tilt_pin",
        elem_b="tilt_hub",
        reason="The top tilt pin is intentionally captured inside the lamp-bar hinge hub.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="base_sleeve",
        elem_b="inner_tube",
        reason="The inner mast telescopes inside the base sleeve; the sleeve mesh is treated as a retained sliding proxy.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="top_collar",
        elem_b="inner_tube",
        reason="The clamp collar surrounds the sliding mast tube at the top of the base section.",
    )

    ctx.expect_overlap(
        base,
        leg_0,
        axes="xyz",
        elem_a="leg_pin_0",
        elem_b="hinge_eye",
        min_overlap=0.008,
        name="leg_0 hinge pin is captured",
    )
    ctx.expect_overlap(
        base,
        leg_1,
        axes="xyz",
        elem_a="leg_pin_1",
        elem_b="hinge_eye",
        min_overlap=0.008,
        name="leg_1 hinge pin is captured",
    )
    ctx.expect_overlap(
        mast,
        lamp_bar,
        axes="xyz",
        elem_a="tilt_pin",
        elem_b="tilt_hub",
        min_overlap=0.010,
        name="lamp tilt pin is captured",
    )

    ctx.expect_gap(
        leg_0,
        base,
        axis="x",
        positive_elem="foot_pad",
        negative_elem="base_ring",
        min_gap=0.45,
        name="leg_0 foot splays outboard of base ring",
    )
    ctx.expect_gap(
        base,
        leg_1,
        axis="x",
        positive_elem="base_ring",
        negative_elem="foot_pad",
        min_gap=0.45,
        name="leg_1 foot splays outboard of base ring",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="base_sleeve",
        margin=0.0,
        name="telescoping mast is centered in base sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="base_sleeve",
        min_overlap=0.60,
        name="collapsed mast remains deeply inserted",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="top_collar",
        min_overlap=0.08,
        name="mast passes through upper clamp collar",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.55}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="base_sleeve",
            margin=0.0,
            name="extended mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="base_sleeve",
            min_overlap=0.12,
            name="extended mast retains insertion",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward prismatically",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.50,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    def lens_center_z():
        aabb = ctx.part_element_world_aabb(lamp_bar, elem="lens_0")
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_lens_z = lens_center_z()
    with ctx.pose({tilt: 0.60}):
        down_lens_z = lens_center_z()
    with ctx.pose({tilt: -0.60}):
        up_lens_z = lens_center_z()
    ctx.check(
        "lamp bar tilts about the top hinge",
        rest_lens_z is not None
        and down_lens_z is not None
        and up_lens_z is not None
        and down_lens_z < rest_lens_z - 0.045
        and up_lens_z > rest_lens_z + 0.045,
        details=f"rest={rest_lens_z}, down={down_lens_z}, up={up_lens_z}",
    )

    with ctx.pose({leg_0_hinge: -0.25, leg_1_hinge: 0.25}):
        ctx.expect_overlap(
            base,
            leg_0,
            axes="xyz",
            elem_a="leg_pin_0",
            elem_b="hinge_eye",
            min_overlap=0.006,
            name="leg_0 remains on hinge while adjusted",
        )
        ctx.expect_overlap(
            base,
            leg_1,
            axes="xyz",
            elem_a="leg_pin_1",
            elem_b="hinge_eye",
            min_overlap=0.006,
            name="leg_1 remains on hinge while adjusted",
        )

    return ctx.report()


object_model = build_object_model()
