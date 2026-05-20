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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_radial_arm_fixture")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.64, 1.0))
    rail_steel = model.material("polished_guide_steel", rgba=(0.78, 0.80, 0.80, 1.0))
    screw_black = model.material("blackened_screws", rgba=(0.01, 0.01, 0.012, 1.0))
    carriage_blue = model.material("blue_carriage", rgba=(0.08, 0.22, 0.55, 1.0))

    back = model.part("back_support")
    back.visual(
        Box((0.040, 0.360, 0.780)),
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        material=painted_steel,
        name="wall_plate",
    )
    back.visual(
        Box((0.190, 0.170, 0.045)),
        origin=Origin(xyz=(0.095, 0.0, 1.0575)),
        material=painted_steel,
        name="upper_bearing_plate",
    )
    back.visual(
        Box((0.190, 0.170, 0.045)),
        origin=Origin(xyz=(0.095, 0.0, 0.7425)),
        material=painted_steel,
        name="lower_bearing_plate",
    )
    for y in (-0.076, 0.076):
        back.visual(
            Box((0.090, 0.018, 0.310)),
            origin=Origin(xyz=(0.150, y, 0.900)),
            material=painted_steel,
            name=f"yoke_cheek_{0 if y < 0 else 1}",
        )
    for y in (-0.120, 0.120):
        for z in (0.650, 1.150):
            back.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(0.024, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_black,
                name=f"mount_screw_{0 if y < 0 else 1}_{0 if z < 0.9 else 1}",
            )

    boss = model.part("rotary_boss")
    boss.visual(
        Cylinder(radius=0.052, length=0.270),
        origin=Origin(),
        material=brushed_metal,
        name="boss_collar",
    )
    boss.visual(
        Cylinder(radius=0.034, length=0.260),
        origin=Origin(),
        material=dark_steel,
        name="center_spindle",
    )
    boss.visual(
        Box((0.820, 0.064, 0.062)),
        origin=Origin(xyz=(0.455, 0.0, 0.025)),
        material=dark_steel,
        name="straight_arm",
    )
    boss.visual(
        Box((0.740, 0.090, 0.018)),
        origin=Origin(xyz=(0.480, 0.0, 0.064)),
        material=painted_steel,
        name="guide_deck",
    )
    for y in (-0.027, 0.027):
        boss.visual(
            Cylinder(radius=0.012, length=0.720),
            origin=Origin(xyz=(0.480, y, 0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_steel,
            name=f"guide_rod_{0 if y < 0 else 1}",
        )
    boss.visual(
        Box((0.034, 0.106, 0.106)),
        origin=Origin(xyz=(0.875, 0.0, 0.078)),
        material=painted_steel,
        name="front_stop",
    )
    boss.visual(
        Box((0.034, 0.106, 0.106)),
        origin=Origin(xyz=(0.105, 0.0, 0.078)),
        material=painted_steel,
        name="rear_stop",
    )
    boss.visual(
        Box((0.125, 0.090, 0.018)),
        origin=Origin(xyz=(0.115, 0.0, -0.010)),
        material=painted_steel,
        name="boss_arm_gusset",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.160, 0.112, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
        material=carriage_blue,
        name="carriage_bridge",
    )
    for y in (-0.058, 0.058):
        carriage.visual(
            Box((0.132, 0.018, 0.102)),
            origin=Origin(xyz=(0.0, y, 0.074)),
            material=carriage_blue,
            name=f"side_cheek_{0 if y < 0 else 1}",
        )
    for y in (-0.027, 0.027):
        carriage.visual(
            Box((0.112, 0.020, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.100)),
            material=brushed_metal,
            name=f"skid_pad_{0 if y < 0 else 1}",
        )
    carriage.visual(
        Box((0.090, 0.050, 0.035)),
        origin=Origin(xyz=(0.040, 0.0, 0.155)),
        material=carriage_blue,
        name="front_lug",
    )

    model.articulation(
        "back_to_boss",
        ArticulationType.REVOLUTE,
        parent=back,
        child=boss,
        origin=Origin(xyz=(0.150, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=boss,
        child=carriage,
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=-0.200, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back = object_model.get_part("back_support")
    boss = object_model.get_part("rotary_boss")
    carriage = object_model.get_part("carriage")
    boss_joint = object_model.get_articulation("back_to_boss")
    slide_joint = object_model.get_articulation("arm_to_carriage")

    ctx.expect_gap(
        back,
        boss,
        axis="z",
        positive_elem="upper_bearing_plate",
        negative_elem="boss_collar",
        max_gap=0.001,
        max_penetration=0.001,
        name="upper bearing plate seats on rotary boss",
    )
    ctx.expect_gap(
        boss,
        back,
        axis="z",
        positive_elem="boss_collar",
        negative_elem="lower_bearing_plate",
        max_gap=0.001,
        max_penetration=0.001,
        name="rotary boss seats on lower bearing plate",
    )
    ctx.expect_overlap(
        back,
        boss,
        axes="xy",
        elem_a="upper_bearing_plate",
        elem_b="boss_collar",
        min_overlap=0.060,
        name="bearing plates surround the vertical boss footprint",
    )

    for index in (0, 1):
        ctx.expect_gap(
            carriage,
            boss,
            axis="z",
            positive_elem=f"skid_pad_{index}",
            negative_elem=f"guide_rod_{index}",
            max_gap=0.002,
            max_penetration=0.001,
            name=f"carriage skid pad {index} rides on guide rod",
        )
    ctx.expect_within(
        carriage,
        boss,
        axes="x",
        inner_elem="carriage_bridge",
        outer_elem="guide_deck",
        margin=0.020,
        name="carriage starts within the guide travel",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: 0.220}):
        ctx.expect_within(
            carriage,
            boss,
            axes="x",
            inner_elem="carriage_bridge",
            outer_elem="guide_deck",
            margin=0.020,
            name="extended carriage remains on the guide",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic carriage translates forward along the arm",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.200,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_stop_aabb = ctx.part_element_world_aabb(boss, elem="front_stop")
    with ctx.pose({boss_joint: 1.200}):
        rotated_stop_aabb = ctx.part_element_world_aabb(boss, elem="front_stop")
    rest_stop_x = None if rest_stop_aabb is None else (rest_stop_aabb[0][0] + rest_stop_aabb[1][0]) / 2.0
    rotated_stop_y = None if rotated_stop_aabb is None else (rotated_stop_aabb[0][1] + rotated_stop_aabb[1][1]) / 2.0
    ctx.check(
        "revolute boss swings the arm radially from the wall bracket",
        rest_stop_x is not None
        and rotated_stop_y is not None
        and rest_stop_x > 0.900
        and rotated_stop_y > 0.650,
        details=f"front_stop_rest_x={rest_stop_x}, front_stop_rotated_y={rotated_stop_y}",
    )

    return ctx.report()


object_model = build_object_model()
