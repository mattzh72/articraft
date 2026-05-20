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
    model = ArticulatedObject(name="roof_weapon_station")

    armor = Material("matte_olive_armor", rgba=(0.22, 0.27, 0.16, 1.0))
    dark_steel = Material("dark_gunmetal", rgba=(0.04, 0.045, 0.045, 1.0))
    worn_steel = Material("worn_steel", rgba=(0.36, 0.36, 0.34, 1.0))
    roof_paint = Material("vehicle_roof_green", rgba=(0.16, 0.22, 0.12, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    roof = model.part("roof")
    roof.visual(
        Box((1.80, 1.25, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=roof_paint,
        name="roof_plate",
    )
    panel_detail = Material("subtle_roof_panel", rgba=(0.11, 0.16, 0.09, 1.0))
    roof.visual(
        Box((1.35, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.46, 0.126)),
        material=panel_detail,
        name="roof_panel_rail_0",
    )
    roof.visual(
        Box((1.35, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.46, 0.126)),
        material=panel_detail,
        name="roof_panel_rail_1",
    )
    roof.visual(
        Box((0.035, 0.86, 0.012)),
        origin=Origin(xyz=(-0.675, 0.0, 0.126)),
        material=panel_detail,
        name="roof_panel_rail_2",
    )
    roof.visual(
        Box((0.035, 0.86, 0.012)),
        origin=Origin(xyz=(0.675, 0.0, 0.126)),
        material=panel_detail,
        name="roof_panel_rail_3",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.39, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="rotating_ring",
    )
    turret.visual(
        Cylinder(radius=0.25, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=rubber,
        name="ring_recess",
    )
    turret.visual(
        Box((0.30, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=armor,
        name="pedestal",
    )
    turret.visual(
        Box((0.62, 0.64, 0.04)),
        origin=Origin(xyz=(0.04, 0.0, 0.16)),
        material=armor,
        name="cradle_floor",
    )
    turret.visual(
        Box((0.60, 0.04, 0.40)),
        origin=Origin(xyz=(0.04, -0.33, 0.38)),
        material=armor,
        name="side_wall_0",
    )
    turret.visual(
        Box((0.60, 0.04, 0.40)),
        origin=Origin(xyz=(0.04, 0.33, 0.38)),
        material=armor,
        name="side_wall_1",
    )
    turret.visual(
        Box((0.05, 0.22, 0.40)),
        origin=Origin(xyz=(0.36, -0.22, 0.38)),
        material=armor,
        name="front_cheek_0",
    )
    turret.visual(
        Box((0.05, 0.22, 0.40)),
        origin=Origin(xyz=(0.36, 0.22, 0.38)),
        material=armor,
        name="front_cheek_1",
    )
    turret.visual(
        Box((0.05, 0.66, 0.10)),
        origin=Origin(xyz=(0.36, 0.0, 0.23)),
        material=armor,
        name="front_sill",
    )
    turret.visual(
        Box((0.05, 0.66, 0.06)),
        origin=Origin(xyz=(0.36, 0.0, 0.55)),
        material=armor,
        name="front_brow",
    )
    turret.visual(
        Cylinder(radius=0.085, length=0.045),
        origin=Origin(xyz=(0.05, -0.3525, 0.35), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="pitch_bearing_0",
    )
    turret.visual(
        Cylinder(radius=0.085, length=0.045),
        origin=Origin(xyz=(0.05, 0.3525, 0.35), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="pitch_bearing_1",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.055, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion",
    )
    cradle.visual(
        Box((0.32, 0.16, 0.14)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=dark_steel,
        name="receiver",
    )
    cradle.visual(
        Cylinder(radius=0.036, length=0.74),
        origin=Origin(xyz=(0.50, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.052, length=0.08),
        origin=Origin(xyz=(0.91, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="muzzle_brake",
    )
    cradle.visual(
        Box((0.30, 0.18, 0.14)),
        origin=Origin(xyz=(-0.05, -0.20, 0.03)),
        material=armor,
        name="ammo_box",
    )
    cradle.visual(
        Box((0.18, 0.055, 0.075)),
        origin=Origin(xyz=(-0.04, -0.0975, 0.03)),
        material=worn_steel,
        name="ammo_mount",
    )
    cradle.visual(
        Box((0.18, 0.06, 0.05)),
        origin=Origin(xyz=(0.03, 0.11, -0.005)),
        material=worn_steel,
        name="feed_chute",
    )

    ammo_lid = model.part("ammo_lid")
    ammo_lid.visual(
        Box((0.30, 0.18, 0.02)),
        origin=Origin(xyz=(0.15, 0.0, 0.01)),
        material=armor,
        name="lid_panel",
    )
    ammo_lid.visual(
        Cylinder(radius=0.012, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="lid_hinge_barrel",
    )
    ammo_lid.visual(
        Box((0.055, 0.035, 0.018)),
        origin=Origin(xyz=(0.29, 0.0, 0.023)),
        material=dark_steel,
        name="lid_latch",
    )

    model.articulation(
        "yaw",
        ArticulationType.CONTINUOUS,
        parent=roof,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.05, 0.0, 0.35)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=-0.35, upper=0.45),
    )
    model.articulation(
        "ammo_hinge",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=ammo_lid,
        origin=Origin(xyz=(-0.20, -0.20, 0.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    ammo_lid = object_model.get_part("ammo_lid")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    ammo_hinge = object_model.get_articulation("ammo_hinge")

    ctx.expect_contact(
        roof,
        turret,
        elem_a="roof_plate",
        elem_b="rotating_ring",
        name="rotating ring sits on roof",
    )
    ctx.expect_within(
        cradle,
        turret,
        axes="yz",
        margin=0.0,
        name="pitching cradle is laterally captured inside shield",
    )
    ctx.expect_gap(
        ammo_lid,
        cradle,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="lid_panel",
        negative_elem="ammo_box",
        name="closed ammunition lid rests on box top",
    )
    ctx.expect_overlap(
        ammo_lid,
        cradle,
        axes="xy",
        min_overlap=0.12,
        elem_a="lid_panel",
        elem_b="ammo_box",
        name="ammunition lid covers the box footprint",
    )

    rest_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({pitch: 0.35}):
        raised_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    ctx.check(
        "positive pitch elevates gun",
        rest_muzzle is not None
        and raised_muzzle is not None
        and raised_muzzle[1][2] > rest_muzzle[1][2] + 0.08,
        details=f"rest={rest_muzzle}, raised={raised_muzzle}",
    )

    rest_lid = ctx.part_element_world_aabb(ammo_lid, elem="lid_panel")
    with ctx.pose({ammo_hinge: 1.0}):
        raised_lid = ctx.part_element_world_aabb(ammo_lid, elem="lid_panel")
    ctx.check(
        "ammunition lid hinges upward",
        rest_lid is not None
        and raised_lid is not None
        and raised_lid[1][2] > rest_lid[1][2] + 0.05,
        details=f"rest={rest_lid}, raised={raised_lid}",
    )

    rest_barrel = ctx.part_element_world_aabb(cradle, elem="barrel")
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_barrel = ctx.part_element_world_aabb(cradle, elem="barrel")
    if rest_barrel is not None and yawed_barrel is not None:
        rest_center_y = 0.5 * (rest_barrel[0][1] + rest_barrel[1][1])
        yawed_center_y = 0.5 * (yawed_barrel[0][1] + yawed_barrel[1][1])
        yaw_ok = yawed_center_y > rest_center_y + 0.25
    else:
        yaw_ok = False
    ctx.check(
        "yaw ring rotates weapon station",
        yaw_ok,
        details=f"rest={rest_barrel}, yawed={yawed_barrel}",
    )

    return ctx.report()


object_model = build_object_model()
