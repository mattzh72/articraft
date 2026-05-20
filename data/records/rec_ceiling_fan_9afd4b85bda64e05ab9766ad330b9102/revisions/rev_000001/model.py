from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_three_blade_hvls_fan")

    brushed_aluminum = Material("brushed_aluminum", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_motor = Material("dark_motor", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.015, 0.015, 0.015, 1.0))
    safety_yellow = Material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))

    mount = model.part("ceiling_mount")
    mount.visual(
        Box((1.10, 1.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.64)),
        material=dark_motor,
        name="ceiling_plate",
    )
    mount.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        material=dark_motor,
        name="canopy",
    )
    mount.visual(
        Cylinder(radius=0.055, length=1.28),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=dark_motor,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.52, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=dark_motor,
        name="motor_housing",
    )
    mount.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=brushed_aluminum,
        name="bearing_face",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.20, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="bearing_plate",
    )
    rotor.visual(
        Cylinder(radius=0.45, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=dark_motor,
        name="round_hub",
    )
    rotor.visual(
        Cylinder(radius=0.30, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.2275)),
        material=brushed_aluminum,
        name="lower_hub_cap",
    )

    blade_profile = rounded_rect_profile(2.55, 0.48, 0.045, corner_segments=5)
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(blade_profile, 0.055, center=True),
        "wide_paddle_blade",
    )

    blade_pitch = math.radians(4.0)
    for index in range(3):
        theta = index * 2.0 * math.pi / 3.0
        c = math.cos(theta)
        s = math.sin(theta)

        # Two long fixed arm bars per blade visibly bridge the round hub to the
        # wide paddle root.  They slightly bury into the hub and blade skin so
        # the rotor reads as one bolted metal assembly.
        for side, lateral in enumerate((-0.155, 0.155)):
            rotor.visual(
                Box((0.94, 0.065, 0.060)),
                origin=Origin(
                    xyz=(0.685 * c - lateral * s, 0.685 * s + lateral * c, -0.10),
                    rpy=(0.0, 0.0, theta),
                ),
                material=brushed_aluminum,
                name=f"arm_{index}_{side}",
            )

        rotor.visual(
            Box((0.24, 0.56, 0.035)),
            origin=Origin(
                xyz=(1.11 * c, 1.11 * s, -0.070),
                rpy=(blade_pitch, 0.0, theta),
            ),
            material=brushed_aluminum,
            name=f"root_clamp_{index}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(
                xyz=(2.325 * c, 2.325 * s, -0.12),
                rpy=(blade_pitch, 0.0, theta),
            ),
            material=brushed_aluminum,
            name=f"blade_{index}",
        )
        rotor.visual(
            Box((0.08, 0.50, 0.060)),
            origin=Origin(
                xyz=(3.60 * c, 3.60 * s, -0.12),
                rpy=(blade_pitch, 0.0, theta),
            ),
            material=safety_yellow,
            name=f"tip_marker_{index}",
        )
        for bolt_side, lateral in enumerate((-0.18, 0.18)):
            rotor.visual(
                Cylinder(radius=0.035, length=0.045),
                origin=Origin(
                    xyz=(1.03 * c - lateral * s, 1.03 * s + lateral * c, -0.055),
                    rpy=(0.0, 0.0, theta),
                ),
                material=rubber_black,
                name=f"blade_bolt_{index}_{bolt_side}",
            )

    model.articulation(
        "spin",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=2.5),
        motion_properties=MotionProperties(damping=0.03, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("ceiling_mount")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("spin")

    ctx.check(
        "continuous vertical rotor joint",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        mount,
        rotor,
        elem_a="bearing_face",
        elem_b="bearing_plate",
        contact_tol=0.001,
        name="rotor bearing is seated under motor",
    )
    ctx.expect_overlap(
        mount,
        rotor,
        axes="xy",
        elem_a="bearing_face",
        elem_b="bearing_plate",
        min_overlap=0.15,
        name="bearing plate is centered under motor",
    )
    ctx.check(
        "three fixed blade-arm groups",
        all(rotor.get_visual(f"blade_{i}") is not None for i in range(3))
        and all(rotor.get_visual(f"arm_{i}_{side}") is not None for i in range(3) for side in range(2)),
        details="expected three blades and two fixed arm bars per blade",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 3.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    ctx.check(
        "blade assembly rotates about vertical axis",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(rest_aabb[0][0] - turned_aabb[0][0]) > 0.30
        and abs(rest_aabb[0][1] - turned_aabb[0][1]) > 0.30,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
