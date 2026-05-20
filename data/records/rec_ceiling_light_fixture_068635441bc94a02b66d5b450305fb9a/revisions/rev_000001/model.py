from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def annular_cylinder(height: float, outer_radius: float, inner_radius: float):
    """A centered hollow cylinder/ring with a real open bore."""
    outer = cq.Workplane("XY").cylinder(height, outer_radius)
    inner = cq.Workplane("XY").cylinder(height + 0.004, inner_radius)
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_pull_down_pendant_light")

    brass = model.material("brushed_brass", rgba=(0.86, 0.63, 0.30, 1.0))
    dark_brass = model.material("shadowed_brass", rgba=(0.42, 0.30, 0.14, 1.0))
    black = model.material("black_braided_cord", rgba=(0.015, 0.013, 0.012, 1.0))
    fabric = model.material("warm_linen_shade", rgba=(0.92, 0.82, 0.63, 0.82))
    glass = model.material("warm_frosted_glass", rgba=(1.0, 0.93, 0.72, 0.62))
    ceiling_white = model.material("matte_white_ceiling", rgba=(0.93, 0.92, 0.88, 1.0))
    button_mat = model.material("dark_lock_button", rgba=(0.05, 0.045, 0.04, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=0.18, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=ceiling_white,
        name="ceiling_plate",
    )
    canopy.visual(
        Cylinder(radius=0.132, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=brass,
        name="canopy_cup",
    )
    canopy.visual(
        mesh_from_cadquery(annular_cylinder(0.016, 0.152, 0.012), "canopy_rolled_lip"),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=brass,
        name="rolled_lip",
    )
    canopy.visual(
        mesh_from_cadquery(annular_cylinder(0.032, 0.027, 0.009), "cord_outlet_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=dark_brass,
        name="cord_outlet",
    )
    canopy.visual(
        Box((0.064, 0.032, 0.026)),
        origin=Origin(xyz=(0.126, 0.0, -0.020)),
        material=dark_brass,
        name="lock_pocket",
    )
    canopy.visual(
        Box((0.010, 0.040, 0.018)),
        origin=Origin(xyz=(0.101, 0.0, -0.020)),
        material=black,
        name="ratchet_slot",
    )

    lock_button = model.part("lock_button")
    lock_button.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_mat,
        name="button_plunger",
    )
    model.articulation(
        "canopy_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=lock_button,
        origin=Origin(xyz=(0.158, 0.0, -0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.012),
        motion_properties=MotionProperties(damping=0.2, friction=0.8),
    )

    cord = model.part("cord")
    cord.visual(
        Cylinder(radius=0.005, length=0.670),
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        material=black,
        name="moving_cord",
    )
    cord.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=black,
        name="cord_stop",
    )
    cord.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.635)),
        material=dark_brass,
        name="swivel_cap",
    )
    model.articulation(
        "canopy_to_cord",
        ArticulationType.PRISMATIC,
        parent=canopy,
        child=cord,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=-0.55, upper=0.0),
        motion_properties=MotionProperties(damping=0.6, friction=6.0),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_brass,
        name="swivel_cup",
    )
    shade.visual(
        Cylinder(radius=0.011, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=dark_brass,
        name="center_stem",
    )
    shade.visual(
        Cylinder(radius=0.038, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, -0.123)),
        material=brass,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.056),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=glass,
        name="globe_bulb",
    )
    shade.visual(
        mesh_from_cadquery(annular_cylinder(0.300, 0.180, 0.160), "linen_drum_shade"),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=fabric,
        name="drum_shell",
    )
    shade.visual(
        mesh_from_cadquery(annular_cylinder(0.010, 0.166, 0.148), "top_shade_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=brass,
        name="top_ring",
    )
    shade.visual(
        mesh_from_cadquery(annular_cylinder(0.012, 0.181, 0.158), "bottom_shade_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
        material=brass,
        name="bottom_ring",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        radius_mid = 0.079
        shade.visual(
            Box((0.158, 0.008, 0.006)),
            origin=Origin(
                xyz=(radius_mid * math.cos(angle), radius_mid * math.sin(angle), -0.074),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"spoke_{i}",
        )

    model.articulation(
        "cord_to_shade",
        ArticulationType.CONTINUOUS,
        parent=cord,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.657)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    lock_button = object_model.get_part("lock_button")
    cord = object_model.get_part("cord")
    shade = object_model.get_part("shade")
    pull_joint = object_model.get_articulation("canopy_to_cord")
    lock_joint = object_model.get_articulation("canopy_to_lock_button")
    spin_joint = object_model.get_articulation("cord_to_shade")

    ctx.expect_overlap(
        cord,
        canopy,
        axes="z",
        elem_a="moving_cord",
        elem_b="cord_outlet",
        min_overlap=0.015,
        name="cord passes through the canopy outlet",
    )
    ctx.expect_within(
        cord,
        canopy,
        axes="xy",
        inner_elem="moving_cord",
        outer_elem="cord_outlet",
        margin=0.001,
        name="cord is centered in the outlet guide",
    )
    ctx.expect_contact(
        cord,
        canopy,
        elem_a="cord_stop",
        elem_b="cord_outlet",
        contact_tol=0.002,
        name="cord stop bears against the lock outlet",
    )
    ctx.expect_contact(
        shade,
        cord,
        elem_a="swivel_cup",
        elem_b="swivel_cap",
        contact_tol=0.002,
        name="shade swivel cup is carried by the cord cap",
    )

    down_pos = ctx.part_world_position(shade)
    with ctx.pose({pull_joint: -0.55}):
        up_pos = ctx.part_world_position(shade)
        ctx.expect_gap(
            canopy,
            shade,
            axis="z",
            min_gap=0.10,
            positive_elem="rolled_lip",
            negative_elem="top_ring",
            name="retracted shade remains below the canopy lip",
        )
    ctx.check(
        "pull-down prismatic travel lowers the shade",
        down_pos is not None and up_pos is not None and down_pos[2] < up_pos[2] - 0.50,
        details=f"down={down_pos}, retracted={up_pos}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({spin_joint: math.pi / 2.0}):
        spun_shade_pos = ctx.part_world_position(shade)
        ctx.expect_overlap(
            shade,
            cord,
            axes="xy",
            elem_a="swivel_cup",
            elem_b="swivel_cap",
            min_overlap=0.018,
            name="spinning shade stays on the cord axis",
        )
    ctx.check(
        "continuous spin keeps the shade centered at the cord end",
        rest_shade_pos is not None
        and spun_shade_pos is not None
        and abs(rest_shade_pos[0] - spun_shade_pos[0]) < 1e-6
        and abs(rest_shade_pos[1] - spun_shade_pos[1]) < 1e-6,
        details=f"rest={rest_shade_pos}, spun={spun_shade_pos}",
    )

    button_out = ctx.part_world_position(lock_button)
    with ctx.pose({lock_joint: 0.012}):
        button_in = ctx.part_world_position(lock_button)
    ctx.check(
        "lock release button presses inward",
        button_out is not None and button_in is not None and button_in[0] < button_out[0] - 0.010,
        details=f"out={button_out}, in={button_in}",
    )

    return ctx.report()


object_model = build_object_model()
