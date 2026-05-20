from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opposed_twin_slide_gripper")

    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_steel = model.material("black_oxide_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    rail_mat = model.material("polished_hardened_rail", rgba=(0.82, 0.84, 0.82, 1.0))
    cover_mat = model.material("blued_access_cover", rgba=(0.10, 0.13, 0.16, 1.0))
    rubber = model.material("matte_rubber_pad", rgba=(0.015, 0.015, 0.014, 1.0))
    brass = model.material("oilite_bronze_bushing", rgba=(0.72, 0.50, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.78, 0.34, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=steel,
        name="ground_plate",
    )
    body.visual(
        Box((0.74, 0.045, 0.014)),
        origin=Origin(xyz=(0.0, 0.115, 0.042)),
        material=dark_steel,
        name="rail_plinth_0",
    )
    body.visual(
        Box((0.74, 0.045, 0.014)),
        origin=Origin(xyz=(0.0, -0.115, 0.042)),
        material=dark_steel,
        name="rail_plinth_1",
    )
    body.visual(
        Box((0.70, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, 0.115, 0.062)),
        material=rail_mat,
        name="rail_0",
    )
    body.visual(
        Box((0.70, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, -0.115, 0.062)),
        material=rail_mat,
        name="rail_1",
    )
    body.visual(
        Box((0.62, 0.015, 0.032)),
        origin=Origin(xyz=(0.0, 0.045, 0.051)),
        material=dark_steel,
        name="guide_wall_0",
    )
    body.visual(
        Box((0.62, 0.015, 0.032)),
        origin=Origin(xyz=(0.0, -0.045, 0.051)),
        material=dark_steel,
        name="guide_wall_1",
    )
    for sx in (-1.0, 1.0):
        body.visual(
            Box((0.035, 0.28, 0.060)),
            origin=Origin(xyz=(sx * 0.365, 0.0, 0.065)),
            material=dark_steel,
            name=f"end_bearing_{0 if sx < 0 else 1}",
        )
        body.visual(
            Box((0.020, 0.040, 0.048)),
            origin=Origin(xyz=(sx * 0.042, 0.156, 0.062)),
            material=dark_steel,
            name=f"hard_stop_{0 if sx < 0 else 1}",
        )
    body.visual(
        Box((0.28, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.165, 0.082)),
        material=dark_steel,
        name="sync_housing",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, -0.155, 0.108)),
        material=brass,
        name="pinion_bearing",
    )
    for sx in (-1.0, 1.0):
        body.visual(
            Box((0.020, 0.055, 0.032)),
            origin=Origin(xyz=(sx * 0.145, -0.165, 0.051)),
            material=dark_steel,
            name=f"sync_web_{0 if sx < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(sx * 0.145, -0.165, 0.107)),
            material=brass,
            name=f"sync_bushing_{0 if sx < 0 else 1}",
        )
    for i, x in enumerate((-0.285, -0.095, 0.095, 0.285)):
        for j, y in enumerate((0.115, -0.115)):
            body.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x, y, 0.077)),
                material=dark_steel,
                name=f"rail_screw_{i}_{j}",
            )

    def build_jaw(name: str, side: float) -> object:
        jaw = model.part(name)
        jaw.visual(
            Box((0.180, 0.220, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, 0.092)),
            material=steel,
            name="slide_carriage",
        )
        jaw.visual(
            Box((0.160, 0.045, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, 0.067)),
            material=dark_steel,
            name="guide_tongue",
        )
        for bx, by in ((-0.045, 0.115), (0.045, 0.115), (-0.045, -0.115), (0.045, -0.115)):
            suffix = 0 if by > 0 and bx < 0 else 1 if by > 0 else 2 if bx < 0 else 3
            jaw.visual(
                Box((0.065, 0.038, 0.028)),
                origin=Origin(xyz=(bx, by, 0.089)),
                material=dark_steel,
                name=f"bearing_block_{suffix}",
            )
        jaw.visual(
            Box((0.030, 0.180, 0.110)),
            origin=Origin(xyz=(side * 0.087, 0.0, 0.145)),
            material=steel,
            name="jaw_upright",
        )
        jaw.visual(
            Box((0.018, 0.150, 0.070)),
            origin=Origin(xyz=(side * 0.111, 0.0, 0.145)),
            material=rubber,
            name="jaw_pad",
        )
        for y in (-0.052, 0.052):
            jaw.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(side * 0.119, y, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"pad_screw_{0 if y < 0 else 1}",
            )
        for y in (-0.078, 0.078):
            jaw.visual(
                Box((0.095, 0.010, 0.016)),
                origin=Origin(xyz=(side * 0.044, y, 0.126), rpy=(0.0, -side * 0.55, 0.0)),
                material=steel,
                name=f"jaw_gusset_{0 if y < 0 else 1}",
            )
        jaw.visual(
            Box((0.025, 0.034, 0.034)),
            origin=Origin(xyz=(side * 0.087, 0.127, 0.088)),
            material=dark_steel,
            name="stop_lug",
        )
        jaw.visual(
            Box((0.060, 0.050, 0.014)),
            origin=Origin(xyz=(side * 0.030, -0.125, 0.112)),
            material=dark_steel,
            name="rack_bridge",
        )
        jaw.visual(
            Box((0.180, 0.012, 0.012)),
            origin=Origin(xyz=(side * 0.080, -0.154, 0.118)),
            material=dark_steel,
            name="rack_bar",
        )
        for k in range(6):
            jaw.visual(
                Box((0.014, 0.006, 0.014)),
                origin=Origin(xyz=(side * (0.015 + 0.020 * k), -0.145, 0.124)),
                material=dark_steel,
                name=f"rack_tooth_{k}",
            )
        return jaw

    jaw_0 = build_jaw("jaw_0", 1.0)
    jaw_1 = build_jaw("jaw_1", -1.0)

    cover_0 = model.part("cover_0")
    cover_0.visual(
        Box((0.110, 0.046, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=cover_mat,
        name="cover_plate",
    )
    cover_1 = model.part("cover_1")
    cover_1.visual(
        Box((0.110, 0.046, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=cover_mat,
        name="cover_plate",
    )
    for cover in (cover_0, cover_1):
        for x in (-0.038, 0.038):
            for y in (-0.014, 0.014):
                cover.visual(
                    Cylinder(radius=0.0045, length=0.003),
                    origin=Origin(xyz=(x, y, 0.0075)),
                    material=dark_steel,
                    name=f"cover_screw_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
                )

    pinion = model.part("pinion")
    pinion.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(),
        material=dark_steel,
        name="pinion_disk",
    )
    for k in range(12):
        angle = 2.0 * math.pi * k / 12.0
        pinion.visual(
            Box((0.010, 0.006, 0.020)),
            origin=Origin(
                xyz=(0.029 * math.cos(angle), 0.029 * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"pinion_tooth_{k}",
        )
    pinion.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(),
        material=brass,
        name="pinion_bushing",
    )

    travel = 0.075
    jaw_0_joint = model.articulation(
        "body_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_0,
        origin=Origin(xyz=(-0.230, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=travel),
        motion_properties=MotionProperties(damping=80.0, friction=18.0),
    )
    model.articulation(
        "body_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_1,
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=travel),
        motion_properties=MotionProperties(damping=80.0, friction=18.0),
        mimic=Mimic(joint=jaw_0_joint.name, multiplier=1.0),
    )
    model.articulation(
        "body_to_pinion",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pinion,
        origin=Origin(xyz=(0.0, -0.155, 0.123)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0, lower=-2.4, upper=0.0),
        motion_properties=MotionProperties(damping=0.8, friction=0.2),
    )
    model.articulation(
        "body_to_cover_0",
        ArticulationType.FIXED,
        parent=body,
        child=cover_0,
        origin=Origin(xyz=(-0.070, -0.165, 0.102)),
    )
    model.articulation(
        "body_to_cover_1",
        ArticulationType.FIXED,
        parent=body,
        child=cover_1,
        origin=Origin(xyz=(0.070, -0.165, 0.102)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw_0 = object_model.get_part("jaw_0")
    jaw_1 = object_model.get_part("jaw_1")
    slide = object_model.get_articulation("body_to_jaw_0")
    follower = object_model.get_articulation("body_to_jaw_1")

    ctx.check(
        "opposed jaws are synchronized",
        follower.mimic is not None and follower.mimic.joint == slide.name and abs(follower.mimic.multiplier - 1.0) < 1e-9,
        details=f"follower mimic={follower.mimic}",
    )
    ctx.expect_gap(
        jaw_1,
        jaw_0,
        axis="x",
        positive_elem="jaw_pad",
        negative_elem="jaw_pad",
        min_gap=0.200,
        max_gap=0.225,
        name="open pads leave central work opening",
    )
    for jaw_name, rail_name, block_name in (
        ("jaw_0", "rail_0", "bearing_block_1"),
        ("jaw_0", "rail_1", "bearing_block_3"),
        ("jaw_1", "rail_0", "bearing_block_0"),
        ("jaw_1", "rail_1", "bearing_block_2"),
    ):
        ctx.expect_gap(
            object_model.get_part(jaw_name),
            body,
            axis="z",
            positive_elem=block_name,
            negative_elem=rail_name,
            max_gap=0.002,
            max_penetration=0.0,
            name=f"{jaw_name} rides on {rail_name}",
        )
    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({slide: 0.075}):
        ctx.expect_gap(
            jaw_1,
            jaw_0,
            axis="x",
            positive_elem="jaw_pad",
            negative_elem="jaw_pad",
            min_gap=0.055,
            max_gap=0.075,
            name="closed pads stop before collision",
        )
        ctx.expect_gap(
            jaw_0,
            body,
            axis="z",
            positive_elem="guide_tongue",
            negative_elem="ground_plate",
            min_gap=0.010,
            name="slide tongue clears floor plate",
        )
        closed_0 = ctx.part_world_position(jaw_0)
        closed_1 = ctx.part_world_position(jaw_1)
    ctx.check(
        "both slides move toward center",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and closed_0[0] > rest_0[0] + 0.070
        and closed_1[0] < rest_1[0] - 0.070,
        details=f"rest=({rest_0}, {rest_1}), closed=({closed_0}, {closed_1})",
    )
    return ctx.report()


object_model = build_object_model()
