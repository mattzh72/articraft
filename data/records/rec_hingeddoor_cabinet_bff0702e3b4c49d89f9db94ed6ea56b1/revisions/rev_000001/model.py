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
    model = ArticulatedObject(name="small_wall_cabinet")

    width = 0.62
    depth = 0.20
    height = 0.55
    panel = 0.018
    front_y = -depth / 2.0

    wood = model.material("warm_birch", rgba=(0.72, 0.50, 0.30, 1.0))
    inner_wood = model.material("pale_interior_wood", rgba=(0.83, 0.68, 0.46, 1.0))
    shadow = model.material("recess_shadow", rgba=(0.42, 0.28, 0.16, 1.0))
    metal = model.material("brushed_steel", rgba=(0.62, 0.61, 0.57, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((panel, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + panel / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((panel, depth, height)),
        origin=Origin(xyz=(width / 2.0 - panel / 2.0, 0.0, height / 2.0)),
        material=wood,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, depth, panel)),
        origin=Origin(xyz=(0.0, 0.0, panel / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, panel)),
        origin=Origin(xyz=(0.0, 0.0, height - panel / 2.0)),
        material=wood,
        name="top_panel",
    )
    cabinet.visual(
        Box((width - 2.0 * panel, depth - 0.006, panel)),
        origin=Origin(xyz=(0.0, 0.003, height / 2.0)),
        material=inner_wood,
        name="central_shelf",
    )
    cabinet.visual(
        Box((width, panel, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - panel / 2.0, height / 2.0)),
        material=inner_wood,
        name="back_panel",
    )
    cabinet.visual(
        Box((width - 0.10, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.006, height - 0.075)),
        material=metal,
        name="wall_hanger_rail",
    )

    door_thickness = 0.018
    door_width = width / 2.0 - 0.003
    door_height = height - 2.0 * panel - 0.012
    door_y = front_y - 0.003 - door_thickness / 2.0
    stile = 0.036
    rail = 0.044
    trim_thickness = 0.006
    knob_radius = 0.012
    knob_length = 0.026

    def add_door(name: str, side: float):
        door = model.part(name)
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(side * door_width / 2.0, 0.0, 0.0)),
            material=wood,
            name="door_slab",
        )
        trim_y = -door_thickness / 2.0 - trim_thickness / 2.0 + 0.0005
        door.visual(
            Box((stile, trim_thickness, door_height)),
            origin=Origin(xyz=(side * stile / 2.0, trim_y, 0.0)),
            material=wood,
            name="hinge_stile",
        )
        door.visual(
            Box((stile, trim_thickness, door_height)),
            origin=Origin(xyz=(side * (door_width - stile / 2.0), trim_y, 0.0)),
            material=wood,
            name="center_stile",
        )
        door.visual(
            Box((door_width, trim_thickness, rail)),
            origin=Origin(xyz=(side * door_width / 2.0, trim_y, door_height / 2.0 - rail / 2.0)),
            material=wood,
            name="top_rail",
        )
        door.visual(
            Box((door_width, trim_thickness, rail)),
            origin=Origin(xyz=(side * door_width / 2.0, trim_y, -door_height / 2.0 + rail / 2.0)),
            material=wood,
            name="bottom_rail",
        )
        door.visual(
            Box((door_width - 2.0 * stile, 0.002, door_height - 2.0 * rail)),
            origin=Origin(xyz=(side * door_width / 2.0, -door_thickness / 2.0 - 0.001, 0.0)),
            material=shadow,
            name="recessed_panel",
        )
        door.visual(
            Cylinder(radius=knob_radius, length=knob_length),
            origin=Origin(
                xyz=(side * (door_width - 0.055), -door_thickness / 2.0 - knob_length / 2.0 + 0.001, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal,
            name="round_pull",
        )
        for index, z_offset in enumerate((-door_height * 0.28, door_height * 0.28)):
            door.visual(
                Box((panel, 0.004, 0.085)),
                origin=Origin(xyz=(side * panel / 2.0, 0.010, z_offset)),
                material=metal,
                name=f"hinge_leaf_{index}",
            )
            door.visual(
                Cylinder(radius=0.006, length=0.105),
                origin=Origin(xyz=(0.0, 0.006, z_offset)),
                material=metal,
                name=f"hinge_barrel_{index}",
            )
        return door

    door_0 = add_door("door_0", 1.0)
    door_1 = add_door("door_1", -1.0)

    open_angle = math.radians(110.0)
    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_0,
        origin=Origin(xyz=(-width / 2.0, door_y, height / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=open_angle),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door_1,
        origin=Origin(xyz=(width / 2.0, door_y, height / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=open_angle),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("cabinet_to_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_door_1")

    ctx.check(
        "both doors use side-hinged revolute joints",
        hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "door hinges open about one hundred ten degrees",
        hinge_0.motion_limits is not None
        and hinge_1.motion_limits is not None
        and abs(hinge_0.motion_limits.upper - math.radians(110.0)) < 1e-6
        and abs(hinge_1.motion_limits.upper - math.radians(110.0)) < 1e-6,
    )
    ctx.check(
        "hinge axes are vertical and mirrored",
        tuple(hinge_0.axis) == (0.0, 0.0, -1.0) and tuple(hinge_1.axis) == (0.0, 0.0, 1.0),
    )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        ctx.expect_gap(
            cabinet,
            door_0,
            axis="y",
            min_gap=0.001,
            max_gap=0.008,
            negative_elem="door_slab",
            name="closed door 0 sits just proud of cabinet front",
        )
        ctx.expect_gap(
            cabinet,
            door_1,
            axis="y",
            min_gap=0.001,
            max_gap=0.008,
            negative_elem="door_slab",
            name="closed door 1 sits just proud of cabinet front",
        )
        ctx.expect_overlap(door_0, cabinet, axes="xz", min_overlap=0.20, name="door 0 covers one front half")
        ctx.expect_overlap(door_1, cabinet, axes="xz", min_overlap=0.20, name="door 1 covers one front half")

    closed_0 = ctx.part_world_aabb(door_0)
    closed_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: math.radians(110.0), hinge_1: math.radians(110.0)}):
        open_0 = ctx.part_world_aabb(door_0)
        open_1 = ctx.part_world_aabb(door_1)
        ctx.check(
            "doors swing outward from the front when opened",
            closed_0 is not None
            and closed_1 is not None
            and open_0 is not None
            and open_1 is not None
            and open_0[0][1] < closed_0[0][1] - 0.08
            and open_1[0][1] < closed_1[0][1] - 0.08,
        )

    return ctx.report()


object_model = build_object_model()
