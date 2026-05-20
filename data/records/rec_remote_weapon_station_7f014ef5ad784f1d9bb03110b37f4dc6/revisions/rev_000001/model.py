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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_remote_weapon_station")

    armor = model.material("matte_olive_armor", rgba=(0.26, 0.31, 0.22, 1.0))
    dark = model.material("flat_black", rgba=(0.02, 0.022, 0.024, 1.0))
    gunmetal = model.material("dark_gunmetal", rgba=(0.12, 0.13, 0.13, 1.0))
    steel = model.material("oiled_steel", rgba=(0.42, 0.43, 0.40, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.58, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark,
        name="ground_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.30, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=armor,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.46, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=steel,
        name="fixed_bearing",
    )

    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(_annular_cylinder(0.53, 0.31, 0.12), "lower_ring"),
        material=gunmetal,
        name="lower_ring",
    )
    turntable.visual(
        Box((0.78, 0.72, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=armor,
        name="deck_plate",
    )
    for arm_name, cap_name, foot_name, boss_name, y in (
        ("fork_arm_0", "fork_cap_0", "fork_foot_0", "fork_boss_0", 0.36),
        ("fork_arm_1", "fork_cap_1", "fork_foot_1", "fork_boss_1", -0.36),
    ):
        turntable.visual(
            Box((0.24, 0.12, 0.72)),
            origin=Origin(xyz=(0.0, y, 0.56)),
            material=armor,
            name=arm_name,
        )
        turntable.visual(
            Box((0.30, 0.14, 0.09)),
            origin=Origin(xyz=(0.0, y, 0.965)),
            material=armor,
            name=cap_name,
        )
        turntable.visual(
            Box((0.42, 0.14, 0.12)),
            origin=Origin(xyz=(-0.10, y, 0.26)),
            material=armor,
            name=foot_name,
        )
        turntable.visual(
            Cylinder(radius=0.14, length=0.09),
            origin=Origin(xyz=(0.0, 0.255 if y > 0.0 else -0.255, 0.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=boss_name,
        )

    weapon = model.part("weapon")
    weapon.visual(
        Box((0.85, 0.40, 0.32)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=armor,
        name="weapon_body",
    )
    weapon.visual(
        Box((0.25, 0.38, 0.28)),
        origin=Origin(xyz=(-0.37, 0.0, 0.0)),
        material=gunmetal,
        name="rear_counterweight",
    )
    weapon.visual(
        Cylinder(radius=0.092, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elevation_axle",
    )
    weapon.visual(
        Cylinder(radius=0.060, length=0.72),
        origin=Origin(xyz=(0.965, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel",
    )
    weapon.visual(
        Cylinder(radius=0.078, length=0.09),
        origin=Origin(xyz=(1.37, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="muzzle_brake",
    )
    weapon.visual(
        Box((0.16, 0.018, 0.10)),
        origin=Origin(xyz=(0.43, -0.209, 0.055)),
        material=dark,
        name="sight_window",
    )

    access_hatch = model.part("access_hatch")
    access_hatch.visual(
        Box((0.26, 0.24, 0.025)),
        origin=Origin(xyz=(0.13, 0.0, -0.0125)),
        material=armor,
        name="hatch_panel",
    )
    access_hatch.visual(
        Cylinder(radius=0.025, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hatch_barrel",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.5),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=weapon,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.7, lower=-0.25, upper=0.80),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=weapon,
        child=access_hatch,
        origin=Origin(xyz=(0.02, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    weapon = object_model.get_part("weapon")
    access_hatch = object_model.get_part("access_hatch")
    base_yaw = object_model.get_articulation("base_yaw")
    elevation_axis = object_model.get_articulation("elevation_axis")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.allow_overlap(
        turntable,
        weapon,
        elem_a="fork_boss_0",
        elem_b="elevation_axle",
        reason="The elevation axle is intentionally captured inside the fork-side trunnion boss.",
    )
    ctx.allow_overlap(
        turntable,
        weapon,
        elem_a="fork_boss_1",
        elem_b="elevation_axle",
        reason="The elevation axle is intentionally captured inside the fork-side trunnion boss.",
    )

    ctx.expect_gap(
        turntable,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_ring",
        negative_elem="fixed_bearing",
        name="rotating ring sits on fixed bearing",
    )
    for boss_name in ("fork_boss_0", "fork_boss_1"):
        ctx.expect_within(
            weapon,
            turntable,
            axes="xz",
            inner_elem="elevation_axle",
            outer_elem=boss_name,
            margin=0.001,
            name=f"axle centered in {boss_name}",
        )
        ctx.expect_overlap(
            turntable,
            weapon,
            axes="y",
            min_overlap=0.030,
            elem_a=boss_name,
            elem_b="elevation_axle",
            name=f"axle retained in {boss_name}",
        )
    ctx.expect_gap(
        access_hatch,
        weapon,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="hatch_panel",
        negative_elem="weapon_body",
        name="access hatch rests on weapon roof",
    )

    closed_deck = ctx.part_element_world_aabb(turntable, elem="deck_plate")
    closed_muzzle = ctx.part_element_world_aabb(weapon, elem="muzzle_brake")
    closed_hatch = ctx.part_element_world_aabb(access_hatch, elem="hatch_panel")
    with ctx.pose({base_yaw: math.pi / 2.0}):
        yawed_deck = ctx.part_element_world_aabb(turntable, elem="deck_plate")
    with ctx.pose({elevation_axis: 0.65}):
        raised_muzzle = ctx.part_element_world_aabb(weapon, elem="muzzle_brake")
    with ctx.pose({hatch_hinge: 1.0}):
        raised_hatch = ctx.part_element_world_aabb(access_hatch, elem="hatch_panel")

    ctx.check(
        "yaw joint rotates rectangular deck",
        closed_deck is not None
        and yawed_deck is not None
        and abs((closed_deck[1][0] - closed_deck[0][0]) - (yawed_deck[1][1] - yawed_deck[0][1])) < 0.01
        and abs((closed_deck[1][1] - closed_deck[0][1]) - (yawed_deck[1][0] - yawed_deck[0][0])) < 0.01,
        details=f"closed={closed_deck}, yawed={yawed_deck}",
    )
    ctx.check(
        "positive elevation raises muzzle",
        closed_muzzle is not None
        and raised_muzzle is not None
        and raised_muzzle[1][2] > closed_muzzle[1][2] + 0.15,
        details=f"closed={closed_muzzle}, raised={raised_muzzle}",
    )
    ctx.check(
        "positive hatch motion opens upward",
        closed_hatch is not None
        and raised_hatch is not None
        and raised_hatch[1][2] > closed_hatch[1][2] + 0.08,
        details=f"closed={closed_hatch}, raised={raised_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
