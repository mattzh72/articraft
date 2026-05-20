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
    model = ArticulatedObject(name="built_in_wall_oven")

    wood = model.material("warm_cabinet_wood", rgba=(0.62, 0.43, 0.25, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_enamel = model.material("black_enamel", rgba=(0.015, 0.014, 0.013, 1.0))
    smoked_glass = model.material("smoked_oven_glass", rgba=(0.04, 0.07, 0.09, 0.46))
    chrome = model.material("polished_chrome", rgba=(0.88, 0.90, 0.92, 1.0))
    rack_metal = model.material("rack_wire_steel", rgba=(0.78, 0.80, 0.80, 1.0))

    cabinet = model.part("cabinet")

    # A shallow cabinet surround whose front plane is flush with the oven trim.
    cabinet.visual(Box((0.12, 0.12, 1.55)), origin=Origin(xyz=(-0.465, 0.03, 0.775)), material=wood, name="side_surround_0")
    cabinet.visual(Box((0.12, 0.12, 1.55)), origin=Origin(xyz=(0.465, 0.03, 0.775)), material=wood, name="side_surround_1")
    cabinet.visual(Box((1.05, 0.12, 0.15)), origin=Origin(xyz=(0.0, 0.03, 1.475)), material=wood, name="top_surround")
    cabinet.visual(Box((1.05, 0.12, 0.15)), origin=Origin(xyz=(0.0, 0.03, 0.075)), material=wood, name="bottom_surround")

    # Stainless oven housing and front trim set into the wood opening.
    cabinet.visual(Box((0.05, 0.035, 1.16)), origin=Origin(xyz=(-0.380, -0.012, 0.775)), material=stainless, name="front_trim_0")
    cabinet.visual(Box((0.05, 0.035, 1.16)), origin=Origin(xyz=(0.380, -0.012, 0.775)), material=stainless, name="front_trim_1")
    cabinet.visual(Box((0.76, 0.035, 0.05)), origin=Origin(xyz=(0.0, -0.012, 1.330)), material=stainless, name="front_trim_top")
    cabinet.visual(Box((0.76, 0.035, 0.05)), origin=Origin(xyz=(0.0, -0.012, 0.220)), material=stainless, name="front_trim_bottom")
    cabinet.visual(Box((0.04, 0.56, 1.10)), origin=Origin(xyz=(-0.360, 0.280, 0.775)), material=dark_enamel, name="cavity_wall_0")
    cabinet.visual(Box((0.04, 0.56, 1.10)), origin=Origin(xyz=(0.360, 0.280, 0.775)), material=dark_enamel, name="cavity_wall_1")
    cabinet.visual(Box((0.72, 0.56, 0.04)), origin=Origin(xyz=(0.0, 0.280, 1.305)), material=dark_enamel, name="cavity_ceiling")
    cabinet.visual(Box((0.72, 0.56, 0.04)), origin=Origin(xyz=(0.0, 0.280, 0.245)), material=dark_enamel, name="cavity_floor")
    cabinet.visual(Box((0.72, 0.04, 1.10)), origin=Origin(xyz=(0.0, 0.560, 0.775)), material=dark_enamel, name="cavity_back")

    # Shelf slides fixed to the oven side walls. The pull-out rack rests on them.
    cabinet.visual(Box((0.045, 0.52, 0.025)), origin=Origin(xyz=(-0.318, 0.255, 0.615)), material=stainless, name="slide_bracket_0")
    cabinet.visual(Box((0.045, 0.52, 0.025)), origin=Origin(xyz=(0.318, 0.255, 0.615)), material=stainless, name="slide_bracket_1")
    cabinet.visual(Box((0.035, 0.52, 0.025)), origin=Origin(xyz=(-0.278, 0.255, 0.615)), material=rack_metal, name="slide_track_0")
    cabinet.visual(Box((0.035, 0.52, 0.025)), origin=Origin(xyz=(0.278, 0.255, 0.615)), material=rack_metal, name="slide_track_1")

    door = model.part("door")
    # Door frame is authored from the full-width bottom hinge line upward.
    door.visual(Box((0.045, 0.055, 1.02)), origin=Origin(xyz=(-0.295, -0.005, 0.550)), material=stainless, name="door_stile_0")
    door.visual(Box((0.045, 0.055, 1.02)), origin=Origin(xyz=(0.295, -0.005, 0.550)), material=stainless, name="door_stile_1")
    door.visual(Box((0.635, 0.055, 0.050)), origin=Origin(xyz=(0.0, -0.005, 1.035)), material=stainless, name="door_top_rail")
    door.visual(Box((0.635, 0.055, 0.050)), origin=Origin(xyz=(0.0, -0.005, 0.065)), material=stainless, name="door_bottom_rail")
    door.visual(Box((0.550, 0.012, 0.900)), origin=Origin(xyz=(0.0, -0.035, 0.550)), material=smoked_glass, name="glass_panel")
    door.visual(Box((0.635, 0.018, 0.040)), origin=Origin(xyz=(0.0, -0.005, 0.025)), material=stainless, name="hinge_leaf")
    door.visual(
        Cylinder(radius=0.025, length=0.670),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.470),
        origin=Origin(xyz=(0.0, -0.110, 0.860), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="bar_handle",
    )
    for index, x in enumerate((-0.200, 0.200)):
        door.visual(
            Cylinder(radius=0.010, length=0.080),
            origin=Origin(xyz=(x, -0.0725, 0.860), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"handle_post_{index}",
        )

    rack = model.part("rack")
    rack.visual(Box((0.025, 0.500, 0.025)), origin=Origin(xyz=(-0.270, 0.265, 0.000)), material=rack_metal, name="rack_side_0")
    rack.visual(Box((0.025, 0.500, 0.025)), origin=Origin(xyz=(0.270, 0.265, 0.000)), material=rack_metal, name="rack_side_1")
    for index, y in enumerate((0.035, 0.105, 0.175, 0.245, 0.315, 0.385, 0.455)):
        rack.visual(Box((0.565, 0.012, 0.012)), origin=Origin(xyz=(0.0, y, 0.004)), material=rack_metal, name=f"rack_cross_{index}")
    rack.visual(Box((0.565, 0.018, 0.055)), origin=Origin(xyz=(0.0, 0.018, 0.035)), material=rack_metal, name="front_lip")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -0.0545, 0.220)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=rack,
        origin=Origin(xyz=(0.0, -0.005, 0.640)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.320),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    door_hinge = object_model.get_articulation("door_hinge")
    rack_slide = object_model.get_articulation("rack_slide")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="front_trim_bottom",
        negative_elem="hinge_barrel",
        max_gap=0.002,
        max_penetration=0.00001,
        name="bottom hinge barrel sits on the oven face without penetrating",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="front_trim_0",
        negative_elem="door_stile_0",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door is flush proud of the stainless trim",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="glass_panel",
        elem_b="cavity_back",
        min_overlap=0.45,
        name="glass door covers the tall oven opening",
    )

    ctx.expect_gap(
        rack,
        cabinet,
        axis="z",
        positive_elem="rack_side_0",
        negative_elem="slide_track_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="rack side rail rests on slide track",
    )
    ctx.expect_overlap(
        rack,
        cabinet,
        axes="y",
        elem_a="rack_side_0",
        elem_b="slide_track_0",
        min_overlap=0.20,
        name="rack remains nested on the telescoping track at rest",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    rack_rest = ctx.part_world_position(rack)
    with ctx.pose({door_hinge: 1.20, rack_slide: 0.320}):
        opened_door_aabb = ctx.part_world_aabb(door)
        rack_extended = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            cabinet,
            axes="y",
            elem_a="rack_side_0",
            elem_b="slide_track_0",
            min_overlap=0.15,
            name="extended rack keeps retained insertion in the slide",
        )

    ctx.check(
        "door opens downward and outward on bottom hinge",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.25
        and opened_door_aabb[1][2] < closed_door_aabb[1][2] - 0.25,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )
    ctx.check(
        "rack slides out toward the oven face",
        rack_rest is not None and rack_extended is not None and rack_extended[1] < rack_rest[1] - 0.25,
        details=f"rest={rack_rest}, extended={rack_extended}",
    )

    return ctx.report()


object_model = build_object_model()
