from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    armor_green = Material("matte_armor_green", rgba=(0.23, 0.28, 0.20, 1.0))
    dark_green = Material("dark_olive_panels", rgba=(0.13, 0.16, 0.12, 1.0))
    gunmetal = Material("gunmetal", rgba=(0.09, 0.10, 0.10, 1.0))
    black = Material("flat_black", rgba=(0.01, 0.01, 0.01, 1.0))
    glass = Material("blue_black_sensor_glass", rgba=(0.02, 0.08, 0.12, 1.0))
    rubber = Material("dark_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.70, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_green,
        name="ground_plinth",
    )
    base.visual(
        Cylinder(radius=0.42, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=gunmetal,
        name="lower_flange",
    )
    base.visual(
        Cylinder(radius=0.24, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=armor_green,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.30, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=gunmetal,
        name="top_bearing",
    )
    for i in range(8):
        a = i * math.tau / 8.0
        base.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(0.34 * math.cos(a), 0.34 * math.sin(a), 0.137)),
            material=black,
            name=f"flange_bolt_{i}",
        )

    upper = model.part("upper_structure")
    upper.visual(
        Cylinder(radius=0.31, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=gunmetal,
        name="azimuth_turntable",
    )
    upper.visual(
        Box((0.86, 0.48, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=armor_green,
        name="rotating_deck",
    )
    upper.visual(
        Box((0.08, 0.24, 0.56)),
        origin=Origin(xyz=(-0.345, 0.02, 0.45)),
        material=armor_green,
        name="missile_side_support",
    )
    upper.visual(
        Box((0.08, 0.24, 0.56)),
        origin=Origin(xyz=(0.345, 0.02, 0.45)),
        material=armor_green,
        name="optic_side_support",
    )
    upper.visual(
        Box((0.77, 0.09, 0.11)),
        origin=Origin(xyz=(0.0, -0.105, 0.225)),
        material=dark_green,
        name="rear_cross_bridge",
    )
    upper.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(-0.4025, 0.02, 0.48), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="missile_side_bearing",
    )
    upper.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.4025, 0.02, 0.48), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="optic_side_bearing",
    )
    upper.visual(
        Box((0.22, 0.16, 0.14)),
        origin=Origin(xyz=(0.0, -0.165, 0.31)),
        material=dark_green,
        name="rear_electronics_box",
    )
    upper.visual(
        Box((0.23, 0.08, 0.08)),
        origin=Origin(xyz=(0.49, 0.18, 0.21)),
        material=armor_green,
        name="optic_outrigger",
    )
    # Small optic pan/tilt bracket on the optic side.  It is rigidly attached
    # to the rotating structure; the optic pod itself is the tilting child.
    upper.visual(
        Box((0.07, 0.08, 0.28)),
        origin=Origin(xyz=(0.60, 0.18, 0.31)),
        material=armor_green,
        name="optic_post",
    )
    upper.visual(
        Box((0.24, 0.04, 0.08)),
        origin=Origin(xyz=(0.60, 0.20, 0.48)),
        material=armor_green,
        name="optic_bridge",
    )
    upper.visual(
        Box((0.035, 0.10, 0.16)),
        origin=Origin(xyz=(0.50, 0.25, 0.48)),
        material=armor_green,
        name="optic_cheek_0",
    )
    upper.visual(
        Box((0.035, 0.10, 0.16)),
        origin=Origin(xyz=(0.70, 0.25, 0.48)),
        material=armor_green,
        name="optic_cheek_1",
    )

    weapon = model.part("weapon_cradle")
    weapon.visual(
        Cylinder(radius=0.043, length=0.61),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    weapon.visual(
        Box((0.25, 0.48, 0.16)),
        origin=Origin(xyz=(0.0, 0.18, 0.0)),
        material=armor_green,
        name="cradle_body",
    )
    weapon.visual(
        Box((0.19, 0.18, 0.18)),
        origin=Origin(xyz=(0.0, -0.12, 0.0)),
        material=dark_green,
        name="rear_breech",
    )
    weapon.visual(
        Cylinder(radius=0.055, length=0.68),
        origin=Origin(xyz=(0.0, 0.39, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="main_barrel",
    )
    weapon.visual(
        Cylinder(radius=0.034, length=0.15),
        origin=Origin(xyz=(0.0, 0.79, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="muzzle_sleeve",
    )
    weapon.visual(
        Box((0.385, 0.055, 0.045)),
        origin=Origin(xyz=(-0.3175, 0.31, 0.080)),
        material=gunmetal,
        name="upper_pod_arm",
    )
    weapon.visual(
        Box((0.385, 0.055, 0.045)),
        origin=Origin(xyz=(-0.3175, 0.31, -0.080)),
        material=gunmetal,
        name="lower_pod_arm",
    )
    weapon.visual(
        Box((0.24, 0.50, 0.24)),
        origin=Origin(xyz=(-0.63, 0.28, 0.0)),
        material=dark_green,
        name="missile_pod_shell",
    )
    weapon.visual(
        Box((0.25, 0.018, 0.25)),
        origin=Origin(xyz=(-0.63, 0.539, 0.0)),
        material=armor_green,
        name="missile_pod_front_plate",
    )
    tube_x = (-0.695, -0.630, -0.565)
    tube_z = (-0.058, 0.058)
    tube_index = 0
    for z in tube_z:
        for x in tube_x:
            weapon.visual(
                Cylinder(radius=0.026, length=0.014),
                origin=Origin(xyz=(x, 0.555, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"launch_tube_{tube_index}",
            )
            tube_index += 1
    weapon.visual(
        Box((0.18, 0.018, 0.035)),
        origin=Origin(xyz=(-0.63, 0.552, -0.105)),
        material=rubber,
        name="pod_weather_lip",
    )

    optic = model.part("optic_pod")
    optic.visual(
        Cylinder(radius=0.025, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="tilt_pin",
    )
    optic.visual(
        Box((0.13, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=dark_green,
        name="sensor_housing",
    )
    optic.visual(
        Box((0.13, 0.035, 0.14)),
        origin=Origin(xyz=(0.0, 0.2325, 0.0)),
        material=armor_green,
        name="front_bezel",
    )
    optic.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(-0.033, 0.257, 0.025), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="wide_lens",
    )
    optic.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.036, 0.257, -0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="narrow_lens",
    )
    optic.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.036, 0.260, 0.042)),
        material=black,
        name="rangefinder_window",
    )

    model.articulation(
        "azimuth_joint",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=1.0),
    )
    model.articulation(
        "elevation_joint",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=weapon,
        origin=Origin(xyz=(0.0, 0.02, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=1.2, lower=-0.25, upper=0.70),
    )
    model.articulation(
        "optic_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=optic,
        origin=Origin(xyz=(0.60, 0.25, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_structure")
    weapon = object_model.get_part("weapon_cradle")
    optic = object_model.get_part("optic_pod")
    azimuth = object_model.get_articulation("azimuth_joint")
    elevation = object_model.get_articulation("elevation_joint")
    optic_tilt = object_model.get_articulation("optic_tilt_joint")

    ctx.expect_contact(
        upper,
        base,
        elem_a="azimuth_turntable",
        elem_b="top_bearing",
        name="rotating deck sits on pedestal bearing",
    )
    ctx.expect_contact(
        weapon,
        upper,
        elem_a="trunnion_shaft",
        elem_b="missile_side_support",
        contact_tol=0.001,
        name="weapon trunnion reaches missile side support",
    )
    ctx.expect_contact(
        weapon,
        upper,
        elem_a="trunnion_shaft",
        elem_b="optic_side_support",
        contact_tol=0.001,
        name="weapon trunnion reaches optic side support",
    )
    ctx.expect_contact(
        optic,
        upper,
        elem_a="tilt_pin",
        elem_b="optic_cheek_0",
        contact_tol=0.001,
        name="optic tilt pin clipped by first bracket cheek",
    )
    ctx.expect_contact(
        optic,
        upper,
        elem_a="tilt_pin",
        elem_b="optic_cheek_1",
        contact_tol=0.001,
        name="optic tilt pin clipped by second bracket cheek",
    )

    rest_barrel = ctx.part_element_world_aabb(weapon, elem="muzzle_sleeve")
    with ctx.pose({elevation: 0.55}):
        raised_barrel = ctx.part_element_world_aabb(weapon, elem="muzzle_sleeve")
    ctx.check(
        "weapon elevation raises muzzle",
        rest_barrel is not None
        and raised_barrel is not None
        and raised_barrel[1][2] > rest_barrel[1][2] + 0.18,
        details=f"rest={rest_barrel}, raised={raised_barrel}",
    )

    rest_lens = ctx.part_element_world_aabb(optic, elem="wide_lens")
    with ctx.pose({optic_tilt: 0.40}):
        tilted_lens = ctx.part_element_world_aabb(optic, elem="wide_lens")
        ctx.expect_contact(
            optic,
            upper,
            elem_a="tilt_pin",
            elem_b="optic_cheek_0",
            contact_tol=0.001,
            name="optic remains clipped while tilted",
        )
    ctx.check(
        "optic tilt raises sensor face",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[1][2] > rest_lens[1][2] + 0.04,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    rest_optic = ctx.part_element_world_aabb(optic, elem="sensor_housing")
    with ctx.pose({azimuth: 1.0}):
        turned_optic = ctx.part_element_world_aabb(optic, elem="sensor_housing")
    ctx.check(
        "azimuth rotates upper assembly",
        rest_optic is not None
        and turned_optic is not None
        and abs(turned_optic[0][0] - rest_optic[0][0]) > 0.10
        and abs(turned_optic[0][1] - rest_optic[0][1]) > 0.10,
        details=f"rest={rest_optic}, turned={turned_optic}",
    )

    return ctx.report()


object_model = build_object_model()
