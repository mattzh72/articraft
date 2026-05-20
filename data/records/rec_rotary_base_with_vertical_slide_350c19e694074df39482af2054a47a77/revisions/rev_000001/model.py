from __future__ import annotations

from math import pi

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


RAM_TRAVEL = 0.22


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_welding_positioner")

    base_mat = model.material("charcoal_cast_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    bearing_mat = model.material("brushed_bearing_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    turret_mat = model.material("blue_rotary_turret", rgba=(0.05, 0.18, 0.38, 1.0))
    frame_mat = model.material("dark_frame", rgba=(0.11, 0.12, 0.13, 1.0))
    ram_mat = model.material("orange_z_ram", rgba=(0.95, 0.36, 0.08, 1.0))
    liner_mat = model.material("bright_slide_liner", rgba=(0.72, 0.74, 0.70, 1.0))
    index_mat = model.material("white_index_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.72, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_mat,
        name="floor_plate",
    )
    base.visual(
        Box((0.50, 0.50, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=base_mat,
        name="fixed_pedestal",
    )
    base.visual(
        Cylinder(radius=0.245, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=bearing_mat,
        name="bearing_ring",
    )

    turret = model.part("turret")
    turret.visual(
        Box((0.42, 0.42, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=turret_mat,
        name="turret_block",
    )
    turret.visual(
        Cylinder(radius=0.185, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=bearing_mat,
        name="rotary_cap",
    )
    turret.visual(
        Box((0.27, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.214, 0.125)),
        material=index_mat,
        name="index_mark",
    )

    frame_y = 0.045
    rail_x = 0.095
    turret.visual(
        Box((0.050, 0.140, 0.580)),
        origin=Origin(xyz=(-rail_x, frame_y, 0.445)),
        material=frame_mat,
        name="rail_0",
    )
    turret.visual(
        Box((0.050, 0.140, 0.580)),
        origin=Origin(xyz=(rail_x, frame_y, 0.445)),
        material=frame_mat,
        name="rail_1",
    )
    turret.visual(
        Box((0.250, 0.044, 0.540)),
        origin=Origin(xyz=(0.0, frame_y - 0.088, 0.430)),
        material=frame_mat,
        name="rear_web",
    )
    turret.visual(
        Box((0.250, 0.044, 0.080)),
        origin=Origin(xyz=(0.0, frame_y + 0.088, 0.735)),
        material=frame_mat,
        name="front_top_tie",
    )
    turret.visual(
        Box((0.250, 0.044, 0.080)),
        origin=Origin(xyz=(0.0, frame_y - 0.088, 0.735)),
        material=frame_mat,
        name="rear_top_tie",
    )
    turret.visual(
        Box((0.018, 0.100, 0.520)),
        origin=Origin(xyz=(-0.061, frame_y, 0.450)),
        material=liner_mat,
        name="liner_0",
    )
    turret.visual(
        Box((0.018, 0.100, 0.520)),
        origin=Origin(xyz=(0.061, frame_y, 0.450)),
        material=liner_mat,
        name="liner_1",
    )

    ram = model.part("ram")
    ram.visual(
        Box((0.060, 0.060, 0.640)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=ram_mat,
        name="ram_bar",
    )
    ram.visual(
        Box((0.104, 0.060, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=ram_mat,
        name="guide_carriage",
    )
    ram.visual(
        Box((0.125, 0.085, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=ram_mat,
        name="top_mount",
    )
    ram.visual(
        Box((0.090, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, 0.045, 0.420)),
        material=liner_mat,
        name="front_wear_plate",
    )

    model.articulation(
        "turret_turn",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.8, lower=-pi, upper=pi),
    )
    model.articulation(
        "ram_slide",
        ArticulationType.PRISMATIC,
        parent=turret,
        child=ram,
        origin=Origin(xyz=(0.0, frame_y, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.12, lower=0.0, upper=RAM_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    ram = object_model.get_part("ram")
    turret_turn = object_model.get_articulation("turret_turn")
    ram_slide = object_model.get_articulation("ram_slide")

    ctx.check(
        "two independent primary motions",
        len(object_model.articulations) == 2,
        details=f"found {len(object_model.articulations)} articulations",
    )
    ctx.expect_contact(
        turret,
        base,
        elem_a="turret_block",
        elem_b="bearing_ring",
        contact_tol=1e-5,
        name="square turret sits on rotary bearing",
    )
    ctx.expect_contact(
        ram,
        turret,
        elem_a="guide_carriage",
        elem_b="liner_0",
        contact_tol=1e-5,
        name="ram carriage is captured by upright rail",
    )

    rest_ram_aabb = ctx.part_world_aabb(ram)
    with ctx.pose({ram_slide: RAM_TRAVEL}):
        raised_ram_aabb = ctx.part_world_aabb(ram)
    ctx.check(
        "ram slide raises the Z ram",
        rest_ram_aabb is not None
        and raised_ram_aabb is not None
        and raised_ram_aabb[1][2] > rest_ram_aabb[1][2] + RAM_TRAVEL * 0.8,
        details=f"rest={rest_ram_aabb}, raised={raised_ram_aabb}",
    )

    rest_ram_pos = ctx.part_world_position(ram)
    with ctx.pose({turret_turn: 0.8}):
        turned_ram_pos = ctx.part_world_position(ram)
    ctx.check(
        "turret rotates the upright about the base centerline",
        rest_ram_pos is not None
        and turned_ram_pos is not None
        and ((turned_ram_pos[0] - rest_ram_pos[0]) ** 2 + (turned_ram_pos[1] - rest_ram_pos[1]) ** 2) ** 0.5
        > 0.02,
        details=f"rest={rest_ram_pos}, turned={turned_ram_pos}",
    )

    return ctx.report()


object_model = build_object_model()
