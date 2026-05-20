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
    model = ArticulatedObject(name="giant_binocular_parallelogram_mount")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    anodized = model.material("anodized_dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    glass = model.material("coated_glass", rgba=(0.12, 0.34, 0.44, 0.72))
    brass = model.material("engraved_brass", rgba=(0.78, 0.58, 0.22, 1.0))

    pillar = model.part("pillar")
    pillar.visual(
        Cylinder(radius=0.42, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=graphite,
        name="floor_plate",
    )
    pillar.visual(
        Cylinder(radius=0.105, length=1.15),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=anodized,
        name="main_column",
    )
    pillar.visual(
        Cylinder(radius=0.19, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 1.215)),
        material=brushed_steel,
        name="fixed_bearing_race",
    )
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        # Low triangular stance legs keep the tall mount believable at this scale.
        x = 0.33 * math.cos(yaw)
        y = 0.33 * math.sin(yaw)
        pillar.visual(
            Box((0.58, 0.095, 0.045)),
            origin=Origin(xyz=(x, y, 0.052), rpy=(0.0, 0.0, yaw)),
            material=graphite,
            name=f"outrigger_{i}",
        )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.17, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=brushed_steel,
        name="rotating_bearing_race",
    )
    fork.visual(
        Cylinder(radius=0.064, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=anodized,
        name="azimuth_post",
    )
    fork.visual(
        Cylinder(radius=0.028, length=0.58),
        origin=Origin(xyz=(-0.25, 0.0, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="counterweight_rod",
    )
    fork.visual(
        Cylinder(radius=0.105, length=0.13),
        origin=Origin(xyz=(-0.55, 0.0, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="counterweight",
    )
    # Four long members form the visible parallelogram arm set between the
    # azimuth post and the forked altitude head.
    for z in (0.10, 0.68):
        fork.visual(
            Box((0.15, 0.86, 0.075)),
            origin=Origin(xyz=(0.07, 0.0, z)),
            material=anodized,
            name=f"inner_crosshead_{int(z * 100)}",
        )
        fork.visual(
            Box((0.16, 1.04, 0.075)),
            origin=Origin(xyz=(0.69, 0.0, z)),
            material=anodized,
            name=f"outer_crosshead_{int(z * 100)}",
        )
        for side, y in enumerate((-0.32, 0.32)):
            fork.visual(
                Box((0.60, 0.075, 0.060)),
                origin=Origin(xyz=(0.38, y, z)),
                material=anodized,
                name=f"parallel_arm_{int(z * 100)}_{side}",
            )
    for x, tie_y_positions in ((0.07, (-0.32, 0.32)), (0.69, (-0.49, 0.49))):
        for side, y in enumerate(tie_y_positions):
            fork.visual(
                Box((0.10, 0.075, 0.62)),
                origin=Origin(xyz=(x, y, 0.39)),
                material=anodized,
                name=f"vertical_tie_{int(x * 100)}_{side}",
            )
    for side, y, cheek_name, boss_name, shadow_name in (
        (0, -0.485, "fork_cheek_0", "bearing_boss_0", "bearing_shadow_0"),
        (1, 0.485, "fork_cheek_1", "bearing_boss_1", "bearing_shadow_1"),
    ):
        fork.visual(
            Box((0.12, 0.09, 0.58)),
            origin=Origin(xyz=(0.78, y, 0.33)),
            material=anodized,
            name=cheek_name,
        )
        fork.visual(
            Cylinder(radius=0.12, length=0.09),
            origin=Origin(xyz=(0.78, y, 0.45), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=boss_name,
        )
        fork.visual(
            Cylinder(radius=0.065, length=0.092),
            origin=Origin(xyz=(0.78, y, 0.45), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name=shadow_name,
        )

    cradle = model.part("cradle")
    for side, y, lens_name in (
        (0, -0.22, "objective_lens_0"),
        (1, 0.22, "objective_lens_1"),
    ):
        cradle.visual(
            Cylinder(radius=0.16, length=0.95),
            origin=Origin(xyz=(0.095, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"objective_tube_{side}",
        )
        cradle.visual(
            Cylinder(radius=0.172, length=0.052),
            origin=Origin(xyz=(0.592, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"front_cell_{side}",
        )
        cradle.visual(
            Cylinder(radius=0.132, length=0.018),
            origin=Origin(xyz=(0.627, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name=lens_name,
        )
        cradle.visual(
            Cylinder(radius=0.058, length=0.22),
            origin=Origin(xyz=(-0.475, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"eyepiece_barrel_{side}",
        )
        cradle.visual(
            Cylinder(radius=0.073, length=0.035),
            origin=Origin(xyz=(-0.595, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name=f"eyecup_{side}",
        )
        cradle.visual(
            Box((0.30, 0.095, 0.040)),
            origin=Origin(xyz=(0.04, y, -0.158)),
            material=brass,
            name=f"saddle_pad_{side}",
        )
    cradle.visual(
        Box((0.52, 0.80, 0.085)),
        origin=Origin(xyz=(0.035, 0.0, -0.18)),
        material=anodized,
        name="wide_bridge",
    )
    cradle.visual(
        Box((0.28, 0.46, 0.060)),
        origin=Origin(xyz=(-0.13, 0.0, 0.155)),
        material=anodized,
        name="top_bridge",
    )
    cradle.visual(
        Cylinder(radius=0.052, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="trunnion_shaft",
    )
    for side, y in enumerate((-0.425, 0.425)):
        cradle.visual(
            Cylinder(radius=0.092, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"trunnion_flange_{side}",
        )

    model.articulation(
        "pillar_to_fork",
        ArticulationType.CONTINUOUS,
        parent=pillar,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.7),
    )
    model.articulation(
        "fork_to_cradle",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(0.78, 0.0, 0.45)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.5, lower=-0.25, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pillar = object_model.get_part("pillar")
    fork = object_model.get_part("fork")
    cradle = object_model.get_part("cradle")
    azimuth = object_model.get_articulation("pillar_to_fork")
    altitude = object_model.get_articulation("fork_to_cradle")

    ctx.expect_gap(
        fork,
        pillar,
        axis="z",
        positive_elem="rotating_bearing_race",
        negative_elem="fixed_bearing_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="azimuth bearing races are seated",
    )
    ctx.expect_gap(
        fork,
        cradle,
        axis="y",
        positive_elem="bearing_boss_1",
        negative_elem="trunnion_shaft",
        max_gap=0.001,
        max_penetration=0.0,
        name="positive fork boss captures trunnion",
    )
    ctx.expect_gap(
        cradle,
        fork,
        axis="y",
        positive_elem="trunnion_shaft",
        negative_elem="bearing_boss_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="negative fork boss captures trunnion",
    )

    rest_front = ctx.part_element_world_aabb(cradle, elem="objective_lens_1")
    with ctx.pose({altitude: 1.0}):
        raised_front = ctx.part_element_world_aabb(cradle, elem="objective_lens_1")
    rest_z = None if rest_front is None else (rest_front[0][2] + rest_front[1][2]) / 2.0
    raised_z = None if raised_front is None else (raised_front[0][2] + raised_front[1][2]) / 2.0
    ctx.check(
        "altitude joint raises objectives",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.20,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    rest_pos = ctx.part_world_position(cradle)
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(cradle)
    ctx.check(
        "azimuth bearing swings fork around pillar",
        rest_pos is not None
        and turned_pos is not None
        and turned_pos[0] < rest_pos[0] * 0.25
        and turned_pos[1] > rest_pos[0] * 0.75,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
