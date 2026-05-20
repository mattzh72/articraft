from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_canister_missile_launcher")

    olive = model.material("olive_drab", rgba=(0.28, 0.34, 0.20, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.16, 0.20, 0.12, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.08, 0.09, 0.09, 1.0))
    black = model.material("black_shadow", rgba=(0.005, 0.006, 0.005, 1.0))

    support_column = model.part("support_column")
    support_column.visual(
        Box((0.86, 0.86, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_olive,
        name="ground_plate",
    )
    support_column.visual(
        Cylinder(radius=0.18, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=olive,
        name="fixed_column",
    )
    support_column.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        material=gunmetal,
        name="top_bearing",
    )
    for index, (x, y) in enumerate(
        ((0.32, 0.32), (-0.32, 0.32), (-0.32, -0.32), (0.32, -0.32))
    ):
        support_column.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x, y, 0.109)),
            material=gunmetal,
            name=f"floor_bolt_{index}",
        )

    base_ring = model.part("base_ring")
    base_ring.visual(
        Cylinder(radius=0.36, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=gunmetal,
        name="turntable_disk",
    )
    base_ring.visual(
        Cylinder(radius=0.24, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=dark_olive,
        name="raised_hub",
    )
    base_ring.visual(
        Box((0.50, 0.38, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=olive,
        name="saddle_block",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        base_ring.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(
                xyz=(0.305 * math.cos(angle), 0.305 * math.sin(angle), 0.100)
            ),
            material=dark_olive,
            name=f"ring_bolt_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.48, 1.02, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=olive,
        name="yoke_crossbeam",
    )
    yoke.visual(
        Box((0.38, 0.32, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=dark_olive,
        name="center_riser",
    )
    for side_index, y in enumerate((-0.47, 0.47)):
        yoke.visual(
            Box((0.34, 0.08, 0.50)),
            origin=Origin(xyz=(0.0, y, 0.62)),
            material=olive,
            name=f"arm_{side_index}_lower",
        )
        yoke.visual(
            Box((0.34, 0.08, 0.20)),
            origin=Origin(xyz=(0.0, y, 1.56)),
            material=olive,
            name=f"arm_{side_index}_upper",
        )
        yoke.visual(
            Box((0.055, 0.08, 1.30)),
            origin=Origin(xyz=(-0.142, y, 1.02)),
            material=dark_olive,
            name=f"arm_{side_index}_rear_bar",
        )
        yoke.visual(
            Box((0.055, 0.08, 1.30)),
            origin=Origin(xyz=(0.142, y, 1.02)),
            material=dark_olive,
            name=f"arm_{side_index}_front_bar",
        )
    yoke.visual(
        Cylinder(radius=0.115, length=0.090),
        origin=Origin(xyz=(0.0, -0.47, 1.35), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="bearing_0",
    )
    yoke.visual(
        Cylinder(radius=0.115, length=0.090),
        origin=Origin(xyz=(0.0, 0.47, 1.35), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="bearing_1",
    )

    canister_pack = model.part("canister_pack")
    tube_length = 1.55
    cell = 0.32
    wall = 0.035
    pitch = 0.36
    centers = (-pitch / 2.0, pitch / 2.0)
    front_x = tube_length / 2.0
    rear_x = -tube_length / 2.0
    inner = cell - 2.0 * wall

    tube_id = 0
    for zc in centers:
        for yc in centers:
            canister_pack.visual(
                Box((tube_length, cell, wall)),
                origin=Origin(xyz=(0.0, yc, zc + cell / 2.0 - wall / 2.0)),
                material=olive,
                name=f"tube_{tube_id}_top",
            )
            canister_pack.visual(
                Box((tube_length, cell, wall)),
                origin=Origin(xyz=(0.0, yc, zc - cell / 2.0 + wall / 2.0)),
                material=olive,
                name=f"tube_{tube_id}_bottom",
            )
            canister_pack.visual(
                Box((tube_length, wall, cell)),
                origin=Origin(xyz=(0.0, yc - cell / 2.0 + wall / 2.0, zc)),
                material=olive,
                name=f"tube_{tube_id}_side_0",
            )
            canister_pack.visual(
                Box((tube_length, wall, cell)),
                origin=Origin(xyz=(0.0, yc + cell / 2.0 - wall / 2.0, zc)),
                material=olive,
                name=f"tube_{tube_id}_side_1",
            )
            canister_pack.visual(
                Box((0.016, inner + 0.010, inner + 0.010)),
                origin=Origin(xyz=(rear_x + 0.020, yc, zc)),
                material=black,
                name=f"bore_shadow_{tube_id}",
            )
            tube_id += 1

    canister_pack.visual(
        Box((tube_length, 0.055, 2.0 * cell + pitch - cell)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_olive,
        name="center_web_vertical",
    )
    canister_pack.visual(
        Box((tube_length, 2.0 * cell + pitch - cell, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_olive,
        name="center_web_horizontal",
    )
    canister_pack.visual(
        Box((0.060, 0.70, 0.045)),
        origin=Origin(xyz=(front_x - 0.030, 0.0, 0.345)),
        material=dark_olive,
        name="front_reference",
    )
    canister_pack.visual(
        Box((0.060, 0.70, 0.045)),
        origin=Origin(xyz=(rear_x + 0.030, 0.0, -0.345)),
        material=dark_olive,
        name="rear_tie_bar",
    )
    for side_index, y in enumerate((-0.3575, 0.3575)):
        canister_pack.visual(
            Box((0.42, 0.035, 0.52)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_olive,
            name=f"side_lug_{side_index}",
        )
        bolt_y = y - 0.023 if y < 0.0 else y + 0.023
        for bolt_index, (x, z) in enumerate(
            ((-0.15, -0.17), (0.15, -0.17), (-0.15, 0.17), (0.15, 0.17))
        ):
            canister_pack.visual(
                Cylinder(radius=0.018, length=0.018),
                origin=Origin(
                    xyz=(x, bolt_y, z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=gunmetal,
                name=f"lug_bolt_{side_index}_{bolt_index}",
            )

    canister_pack.visual(
        Cylinder(radius=0.075, length=1.07),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="pivot_pin",
    )
    for side_index, y in enumerate((-0.535, 0.535)):
        canister_pack.visual(
            Cylinder(radius=0.12, length=0.045),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"pivot_cap_{side_index}",
        )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=support_column,
        child=base_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.7),
    )
    model.articulation(
        "base_to_yoke",
        ArticulationType.FIXED,
        parent=base_ring,
        child=yoke,
        origin=Origin(),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=canister_pack,
        origin=Origin(xyz=(0.0, 0.0, 1.35)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.45, lower=-0.18, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_column = object_model.get_part("support_column")
    base_ring = object_model.get_part("base_ring")
    yoke = object_model.get_part("yoke")
    canister_pack = object_model.get_part("canister_pack")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.allow_overlap(
        canister_pack,
        yoke,
        elem_a="pivot_pin",
        elem_b="bearing_0",
        reason="The trunnion pin is intentionally captured inside the solid proxy for the first yoke bearing.",
    )
    ctx.allow_overlap(
        canister_pack,
        yoke,
        elem_a="pivot_pin",
        elem_b="bearing_1",
        reason="The trunnion pin is intentionally captured inside the solid proxy for the second yoke bearing.",
    )

    ctx.expect_contact(
        base_ring,
        support_column,
        elem_a="turntable_disk",
        elem_b="top_bearing",
        name="turntable rests on column bearing",
    )
    ctx.expect_contact(
        yoke,
        base_ring,
        elem_a="yoke_crossbeam",
        elem_b="saddle_block",
        name="yoke is seated on rotating saddle",
    )
    ctx.expect_within(
        canister_pack,
        yoke,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="bearing_0",
        margin=0.005,
        name="trunnion fits through first yoke arm window",
    )
    ctx.expect_within(
        canister_pack,
        yoke,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="bearing_1",
        margin=0.005,
        name="trunnion fits through second yoke arm window",
    )
    ctx.expect_overlap(
        canister_pack,
        yoke,
        axes="y",
        elem_a="pivot_pin",
        elem_b="bearing_0",
        min_overlap=0.03,
        name="trunnion spans first yoke arm",
    )
    ctx.expect_overlap(
        canister_pack,
        yoke,
        axes="y",
        elem_a="pivot_pin",
        elem_b="bearing_1",
        min_overlap=0.03,
        name="trunnion spans second yoke arm",
    )

    rest_front = ctx.part_element_world_aabb(canister_pack, elem="front_reference")
    rest_arm = ctx.part_element_world_aabb(yoke, elem="bearing_1")
    with ctx.pose({elevation: 0.85}):
        raised_front = ctx.part_element_world_aabb(canister_pack, elem="front_reference")
        ctx.expect_within(
            canister_pack,
            yoke,
            axes="xz",
            inner_elem="pivot_pin",
            outer_elem="bearing_0",
            margin=0.005,
            name="trunnion remains captured while elevated",
        )
    with ctx.pose({azimuth: math.pi / 2.0}):
        rotated_arm = ctx.part_element_world_aabb(yoke, elem="bearing_1")

    rest_front_z = (
        (rest_front[0][2] + rest_front[1][2]) / 2.0 if rest_front is not None else None
    )
    raised_front_z = (
        (raised_front[0][2] + raised_front[1][2]) / 2.0
        if raised_front is not None
        else None
    )
    ctx.check(
        "elevation joint raises canister muzzles",
        rest_front_z is not None
        and raised_front_z is not None
        and raised_front_z > rest_front_z + 0.25,
        details=f"rest_front_z={rest_front_z}, raised_front_z={raised_front_z}",
    )

    rest_arm_center = (
        (
            (rest_arm[0][0] + rest_arm[1][0]) / 2.0,
            (rest_arm[0][1] + rest_arm[1][1]) / 2.0,
        )
        if rest_arm is not None
        else None
    )
    rotated_arm_center = (
        (
            (rotated_arm[0][0] + rotated_arm[1][0]) / 2.0,
            (rotated_arm[0][1] + rotated_arm[1][1]) / 2.0,
        )
        if rotated_arm is not None
        else None
    )
    ctx.check(
        "azimuth joint rotates yoke around vertical column",
        rest_arm_center is not None
        and rotated_arm_center is not None
        and abs(rotated_arm_center[0] + rest_arm_center[1]) < 0.03
        and abs(rotated_arm_center[1] - rest_arm_center[0]) < 0.03,
        details=f"rest={rest_arm_center}, rotated={rotated_arm_center}",
    )

    return ctx.report()


object_model = build_object_model()
