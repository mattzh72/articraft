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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _build_bottle_mesh():
    body = (
        cq.Workplane("XY")
        .circle(0.034)
        .workplane(offset=0.128)
        .circle(0.0305)
        .loft(combine=True)
    )
    body = body.faces(">Z").workplane(centerOption="CenterOfMass").circle(0.0265).extrude(0.016)
    body = body.faces(">Z").workplane(centerOption="CenterOfMass").circle(0.0215).extrude(0.026)
    shell = body.faces(">Z").shell(-0.0022)
    shell = shell.faces("<Z").workplane(centerOption="CenterOfMass").circle(0.0075).cutThruAll()
    return mesh_from_cadquery(shell, "portable_blender_bottle")


def _build_loop_mesh():
    loop_points = [
        (-0.021, 0.000, 0.000),
        (-0.021, 0.010, 0.0004),
        (-0.015, 0.024, 0.0007),
        (-0.007, 0.039, 0.0010),
        (0.000, 0.050, 0.0012),
        (0.007, 0.039, 0.0010),
        (0.015, 0.024, 0.0007),
        (0.021, 0.010, 0.0004),
        (0.021, 0.000, 0.000),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            loop_points,
            radius=0.0026,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "portable_blender_loop",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_blender")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))
    smoke = model.material("smoke", rgba=(0.26, 0.28, 0.31, 1.0))
    jar_clear = model.material("jar_clear", rgba=(0.88, 0.96, 1.00, 0.26))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    button_glow = model.material("button_glow", rgba=(0.40, 0.63, 0.64, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.040, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=body_white,
        name="housing",
    )
    base.visual(
        Cylinder(radius=0.031, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=charcoal,
        name="seat",
    )
    base.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=soft_black,
        name="foot_ring",
    )
    base.visual(
        Box((0.003, 0.004, 0.018)),
        origin=Origin(xyz=(-0.008, 0.0388, 0.022)),
        material=smoke,
        name="button_guide_left",
    )
    base.visual(
        Box((0.003, 0.004, 0.018)),
        origin=Origin(xyz=(0.008, 0.0388, 0.022)),
        material=smoke,
        name="button_guide_right",
    )
    base.visual(
        Box((0.013, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0388, 0.0305)),
        material=smoke,
        name="button_guide_top",
    )
    base.visual(
        Box((0.013, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0388, 0.0135)),
        material=smoke,
        name="button_guide_bottom",
    )

    bottle = model.part("bottle")
    bottle.visual(
        _build_bottle_mesh(),
        material=jar_clear,
        name="jar_shell",
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.0175, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=charcoal,
        name="plug",
    )
    lid.visual(
        Cylinder(radius=0.0285, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=charcoal,
        name="cap",
    )
    for side_sign in (-1.0, 1.0):
        x_pos = side_sign * 0.0235
        lid.visual(
            Box((0.006, 0.010, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, 0.013)),
            material=charcoal,
            name=f"pivot_bridge_{int(side_sign > 0)}",
        )
        lid.visual(
            Box((0.006, 0.002, 0.010)),
            origin=Origin(xyz=(x_pos, 0.0044, 0.0195)),
            material=charcoal,
            name=f"pivot_front_{int(side_sign > 0)}",
        )
        lid.visual(
            Box((0.006, 0.002, 0.010)),
            origin=Origin(xyz=(x_pos, -0.0044, 0.0195)),
            material=charcoal,
            name=f"pivot_rear_{int(side_sign > 0)}",
        )

    carry_loop = model.part("carry_loop")
    carry_loop.visual(
        _build_loop_mesh(),
        material=smoke,
        name="loop_rail",
    )
    carry_loop.visual(
        Cylinder(radius=0.0028, length=0.008),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke,
        name="pivot_pin_0",
    )
    carry_loop.visual(
        Cylinder(radius=0.0028, length=0.008),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke,
        name="pivot_pin_1",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.0085, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=button_glow,
        name="button_cap",
    )
    power_button.visual(
        Box((0.013, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.0008, 0.0)),
        material=button_glow,
        name="button_stem",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=stainless,
        name="shaft",
    )
    blade.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stainless,
        name="hub",
    )
    blade.visual(
        Box((0.022, 0.005, 0.0014)),
        origin=Origin(xyz=(0.009, 0.0, 0.008), rpy=(0.0, 0.32, 0.0)),
        material=stainless,
        name="blade_x_0",
    )
    blade.visual(
        Box((0.022, 0.005, 0.0014)),
        origin=Origin(xyz=(-0.009, 0.0, 0.008), rpy=(0.0, -0.32, 0.0)),
        material=stainless,
        name="blade_x_1",
    )
    blade.visual(
        Box((0.022, 0.005, 0.0014)),
        origin=Origin(xyz=(0.0, 0.009, 0.008), rpy=(-0.32, 0.0, math.pi / 2.0)),
        material=stainless,
        name="blade_y_0",
    )
    blade.visual(
        Box((0.022, 0.005, 0.0014)),
        origin=Origin(xyz=(0.0, -0.009, 0.008), rpy=(0.32, 0.0, math.pi / 2.0)),
        material=stainless,
        name="blade_y_1",
    )

    model.articulation(
        "base_to_bottle",
        ArticulationType.FIXED,
        parent=base,
        child=bottle,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )
    model.articulation(
        "bottle_to_lid",
        ArticulationType.FIXED,
        parent=bottle,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )
    model.articulation(
        "lid_to_carry_loop",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=carry_loop,
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_power_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=power_button,
        origin=Origin(xyz=(0.0, 0.0423, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0015,
        ),
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=35.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bottle = object_model.get_part("bottle")
    lid = object_model.get_part("lid")
    carry_loop = object_model.get_part("carry_loop")
    power_button = object_model.get_part("power_button")
    blade = object_model.get_part("blade")

    loop_joint = object_model.get_articulation("lid_to_carry_loop")
    button_joint = object_model.get_articulation("base_to_power_button")
    blade_joint = object_model.get_articulation("base_to_blade")

    ctx.expect_contact(
        bottle,
        base,
        contact_tol=0.001,
        name="bottle seats on the drive base",
    )
    ctx.expect_gap(
        power_button,
        base,
        axis="y",
        positive_elem="button_cap",
        min_gap=0.0011,
        max_gap=0.0015,
        name="power button rests proud of the front bezel",
    )
    ctx.expect_within(
        blade,
        bottle,
        axes="xy",
        margin=0.005,
        name="blade stays inside the bottle footprint",
    )
    ctx.expect_origin_gap(
        lid,
        blade,
        axis="z",
        min_gap=0.150,
        name="blade sits at the bottom well below the lid",
    )

    rest_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({button_joint: 0.0015}):
        ctx.expect_gap(
            power_button,
            base,
            axis="y",
            positive_elem="button_cap",
            max_penetration=0.0004,
            max_gap=0.0004,
            name="pressed power button nearly sits flush",
        )
        pressed_button_pos = ctx.part_world_position(power_button)

    ctx.check(
        "power button depresses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.0014,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    folded_loop_aabb = ctx.part_world_aabb(carry_loop)
    with ctx.pose({loop_joint: 1.35}):
        raised_loop_aabb = ctx.part_world_aabb(carry_loop)

    ctx.check(
        "carry loop raises above the lid",
        folded_loop_aabb is not None
        and raised_loop_aabb is not None
        and raised_loop_aabb[1][2] > folded_loop_aabb[1][2] + 0.035,
        details=f"folded={folded_loop_aabb}, raised={raised_loop_aabb}",
    )

    rest_blade_pos = ctx.part_world_position(blade)
    with ctx.pose({blade_joint: 1.2}):
        spun_blade_pos = ctx.part_world_position(blade)

    ctx.check(
        "blade spins about a fixed hub center",
        rest_blade_pos is not None
        and spun_blade_pos is not None
        and max(abs(a - b) for a, b in zip(rest_blade_pos, spun_blade_pos)) < 1e-6,
        details=f"rest={rest_blade_pos}, spun={spun_blade_pos}",
    )

    return ctx.report()


object_model = build_object_model()
