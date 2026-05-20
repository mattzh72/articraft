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
)


def _canopy_shell() -> cq.Workplane:
    """A shallow hollow bell-shaped ceiling canopy with a visible lower mouth."""

    height = 0.115
    outer_bottom = 0.090
    outer_top = 0.165
    wall = 0.012

    outer = (
        cq.Workplane("XY")
        .circle(outer_bottom)
        .workplane(offset=height)
        .circle(outer_top)
        .loft(combine=True)
    )
    inner_void = (
        cq.Workplane("XY")
        .workplane(offset=-0.004)
        .circle(outer_bottom - wall)
        .workplane(offset=height - wall)
        .circle(outer_top - wall)
        .loft(combine=True)
    )
    return outer.cut(inner_void)


def _blade_center(angle: float, radius: float, tangential_offset: float = 0.0) -> tuple[float, float]:
    """Return a point in the fan disk using radial and tangential coordinates."""

    return (
        radius * math.cos(angle) - tangential_offset * math.sin(angle),
        radius * math.sin(angle) + tangential_offset * math.cos(angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_five_blade_ceiling_fan")

    dark_bronze = model.material("dark_bronze", rgba=(0.20, 0.13, 0.07, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.55, 0.38, 0.16, 1.0))
    shadow_metal = model.material("shadowed_metal", rgba=(0.06, 0.055, 0.045, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.62, 0.36, 0.16, 1.0))
    darker_wood = model.material("darker_wood_grain", rgba=(0.33, 0.18, 0.07, 1.0))

    mount = model.part("mount")
    mount.visual(
        mesh_from_cadquery(_canopy_shell(), "hollow_round_canopy"),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=dark_bronze,
        name="canopy_shell",
    )
    mount.visual(
        Cylinder(radius=0.172, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.622)),
        material=dark_bronze,
        name="ceiling_flange",
    )
    mount.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.492)),
        material=aged_brass,
        name="canopy_collar",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=dark_bronze,
        name="downrod",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.148, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=aged_brass,
        name="motor_shell",
    )
    rotor.visual(
        Cylinder(radius=0.158, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=dark_bronze,
        name="top_band",
    )
    rotor.visual(
        Cylinder(radius=0.158, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=dark_bronze,
        name="bottom_band",
    )
    rotor.visual(
        Cylinder(radius=0.072, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=dark_bronze,
        name="upper_bearing",
    )
    rotor.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
        material=dark_bronze,
        name="lower_finial",
    )

    blade_length = 0.560
    blade_width = 0.125
    blade_thickness = 0.018
    blade_inner_radius = 0.300
    blade_center_radius = blade_inner_radius + blade_length / 2.0

    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        yaw = angle

        bx, by = _blade_center(angle, blade_center_radius)
        rotor.visual(
            Box((blade_length, blade_width, blade_thickness)),
            origin=Origin(xyz=(bx, by, -0.076), rpy=(0.0, 0.0, yaw)),
            material=warm_wood,
            name=f"blade_{index}",
        )

        stripe_x, stripe_y = _blade_center(angle, blade_center_radius)
        rotor.visual(
            Box((blade_length * 0.86, 0.010, 0.004)),
            origin=Origin(xyz=(stripe_x, stripe_y, -0.064), rpy=(0.0, 0.0, yaw)),
            material=darker_wood,
            name=f"wood_grain_{index}",
        )

        for offset, suffix in ((-0.034, "a"), (0.034, "b")):
            ax, ay = _blade_center(angle, 0.225, offset)
            rotor.visual(
                Box((0.235, 0.015, 0.018)),
                origin=Origin(xyz=(ax, ay, -0.057), rpy=(0.0, 0.0, yaw)),
                material=dark_bronze,
                name=f"blade_iron_{index}_{suffix}",
            )

        px, py = _blade_center(angle, blade_inner_radius + 0.050)
        rotor.visual(
            Box((0.105, 0.112, 0.014)),
            origin=Origin(xyz=(px, py, -0.062), rpy=(0.0, 0.0, yaw)),
            material=aged_brass,
            name=f"root_plate_{index}",
        )

        for radial_offset in (-0.030, 0.030):
            for tangential_offset in (-0.030, 0.030):
                sx, sy = _blade_center(angle, blade_inner_radius + 0.050 + radial_offset, tangential_offset)
                rotor.visual(
                    Cylinder(radius=0.009, length=0.006),
                    origin=Origin(xyz=(sx, sy, -0.053)),
                    material=shadow_metal,
                    name=f"screw_{index}_{'in' if radial_offset < 0 else 'out'}_{'a' if tangential_offset < 0 else 'b'}",
                )

    model.articulation(
        "mount_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("mount_to_rotor")

    ctx.check(
        "motor assembly has continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "five wooden blades are modeled",
        all(rotor.get_visual(f"blade_{index}") is not None for index in range(5)),
        details="Expected blade_0 through blade_4 on the rotating assembly.",
    )

    ctx.expect_contact(
        mount,
        rotor,
        elem_a="downrod",
        elem_b="upper_bearing",
        contact_tol=0.001,
        name="fixed downrod visibly seats against rotating bearing",
    )

    blade_box = ctx.part_element_world_aabb(rotor, elem="blade_0")
    if blade_box is not None:
        blade_min, blade_max = blade_box
        blade_dims = (
            blade_max[0] - blade_min[0],
            blade_max[1] - blade_min[1],
            blade_max[2] - blade_min[2],
        )
        ctx.check(
            "blade_0 is a long flat rectangular paddle",
            blade_dims[0] > 0.50 and 0.10 < blade_dims[1] < 0.15 and blade_dims[2] < 0.025,
            details=f"dims={blade_dims}",
        )
    else:
        ctx.fail("blade_0 dimensions available", "No AABB for blade_0.")

    with ctx.pose({spin: math.pi / 2.0}):
        spun_box = ctx.part_element_world_aabb(rotor, elem="blade_0")
        if spun_box is not None:
            spun_min, spun_max = spun_box
            spun_center = (
                (spun_min[0] + spun_max[0]) / 2.0,
                (spun_min[1] + spun_max[1]) / 2.0,
            )
            ctx.check(
                "continuous joint rotates blades around vertical axis",
                abs(spun_center[0]) < 0.05 and spun_center[1] > 0.50,
                details=f"spun_center_xy={spun_center}",
            )
        else:
            ctx.fail("blade_0 spun pose available", "No AABB for blade_0 after spin pose.")

    return ctx.report()


object_model = build_object_model()
