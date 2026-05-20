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
    model = ArticulatedObject(name="shallow_office_refrigerator")

    cabinet_width = 0.54
    cabinet_depth = 0.42
    cabinet_height = 0.84
    wall = 0.035

    door_width = 0.50
    door_height = 0.78
    door_thickness = 0.052
    door_bottom = 0.030
    hinge_y = -0.032

    off_white = model.material("warm_white_powdercoat", rgba=(0.88, 0.90, 0.86, 1.0))
    door_white = model.material("slightly_glossy_door", rgba=(0.96, 0.97, 0.93, 1.0))
    liner = model.material("dark_recessed_liner", rgba=(0.08, 0.095, 0.10, 1.0))
    rubber = model.material("black_rubber_gasket", rgba=(0.01, 0.012, 0.012, 1.0))
    metal = model.material("brushed_hinge_metal", rgba=(0.60, 0.62, 0.60, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.025, 0.028, 0.030, 1.0))
    glass = model.material("faint_blue_glass_shelf", rgba=(0.55, 0.80, 0.88, 0.45))

    cabinet = model.part("cabinet")
    # An open-front shallow refrigerator body: side walls, top, bottom, and rear
    # panel are separate manufactured sheets but overlap slightly at edges so the
    # authored cabinet is one supported shell.
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2 + wall / 2, cabinet_depth / 2, cabinet_height / 2)),
        material=off_white,
        name="hinge_side_wall",
    )
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2 - wall / 2, cabinet_depth / 2, cabinet_height / 2)),
        material=off_white,
        name="latch_side_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, cabinet_depth / 2, cabinet_height - wall / 2)),
        material=off_white,
        name="top_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, wall)),
        origin=Origin(xyz=(0.0, cabinet_depth / 2, wall / 2)),
        material=off_white,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((cabinet_width, wall, cabinet_height)),
        origin=Origin(xyz=(0.0, cabinet_depth - wall / 2, cabinet_height / 2)),
        material=off_white,
        name="rear_wall",
    )
    cabinet.visual(
        Box((cabinet_width - 2 * wall, 0.006, cabinet_height - 2 * wall)),
        origin=Origin(xyz=(0.0, cabinet_depth - wall - 0.004, cabinet_height / 2)),
        material=liner,
        name="dark_back_liner",
    )
    cabinet.visual(
        Box((cabinet_width - 2 * wall, cabinet_depth - 0.095, 0.010)),
        origin=Origin(xyz=(0.0, 0.205, 0.43)),
        material=glass,
        name="single_shelf",
    )
    cabinet.visual(
        Box((cabinet_width - 0.08, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.006, 0.020)),
        material=charcoal,
        name="black_toe_kick",
    )

    # Exposed hinge knuckles are carried by small leaves that wrap onto the
    # cabinet front edge, making the hinge visually mounted rather than floating.
    hinge_x = -cabinet_width / 2 - 0.014
    for index, zc in enumerate((0.18, 0.42, 0.66)):
        cabinet.visual(
            Box((0.016, 0.038, 0.170)),
            origin=Origin(xyz=(-cabinet_width / 2 - 0.008, -0.014, zc)),
            material=metal,
            name=f"hinge_leaf_{index}",
        )
        cabinet.visual(
            Cylinder(radius=0.012, length=0.155),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=metal,
            name=f"hinge_knuckle_{index}",
        )

    cabinet.visual(
        Box((0.030, 0.028, 0.048)),
        origin=Origin(xyz=(cabinet_width / 2 - wall / 2, -0.010, cabinet_height - 0.090)),
        material=metal,
        name="latch_keeper",
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2, 0.0, door_height / 2)),
        material=door_white,
        name="door_slab",
    )
    door.visual(
        Box((door_width - 0.070, 0.007, door_height - 0.100)),
        origin=Origin(xyz=(door_width / 2, -door_thickness / 2 - 0.0035, door_height / 2)),
        material=door_white,
        name="raised_front_panel",
    )
    # Rear magnetic gasket strips just kiss the cabinet front when the door is
    # closed, leaving the door itself clear of the frame.
    gasket_y = door_thickness / 2 + 0.003
    door.visual(
        Box((door_width - 0.052, 0.006, 0.018)),
        origin=Origin(xyz=(door_width / 2, gasket_y, door_height - 0.030)),
        material=rubber,
        name="top_gasket",
    )
    door.visual(
        Box((door_width - 0.052, 0.006, 0.018)),
        origin=Origin(xyz=(door_width / 2, gasket_y, 0.030)),
        material=rubber,
        name="bottom_gasket",
    )
    door.visual(
        Box((0.018, 0.006, door_height - 0.052)),
        origin=Origin(xyz=(0.030, gasket_y, door_height / 2)),
        material=rubber,
        name="hinge_gasket",
    )
    door.visual(
        Box((0.018, 0.006, door_height - 0.052)),
        origin=Origin(xyz=(door_width - 0.030, gasket_y, door_height / 2)),
        material=rubber,
        name="latch_gasket",
    )
    door.visual(
        Box((0.020, 0.010, door_height - 0.080)),
        origin=Origin(xyz=(0.012, -door_thickness / 2 - 0.005, door_height / 2)),
        material=metal,
        name="moving_hinge_leaf",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-cabinet_width / 2, hinge_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    latch_lever = model.part("latch_lever")
    latch_lever.visual(
        Box((0.110, 0.012, 0.028)),
        origin=Origin(xyz=(-0.055, -0.006, 0.0)),
        material=charcoal,
        name="lever_paddle",
    )
    latch_lever.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=charcoal,
        name="pivot_cap",
    )
    latch_lever.visual(
        Cylinder(radius=0.007, length=0.011),
        origin=Origin(xyz=(-0.092, -0.006, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=metal,
        name="catch_roller",
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_lever,
        origin=Origin(xyz=(door_width - 0.060, -door_thickness / 2 - 0.007, door_height - 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    latch_lever = object_model.get_part("latch_lever")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.007,
        max_penetration=0.0,
        positive_elem="latch_side_wall",
        negative_elem="latch_gasket",
        name="closed door gasket sits just in front of cabinet frame",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.45,
        elem_a="door_slab",
        elem_b="rear_wall",
        name="single full-height door covers the cabinet opening",
    )
    ctx.expect_contact(
        latch_lever,
        door,
        elem_a="pivot_cap",
        elem_b="raised_front_panel",
        contact_tol=0.001,
        name="latch lever pivot cap is mounted on the door face",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({door_hinge: 1.20}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door hinge swings the free edge outward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.18,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    closed_lever_aabb = ctx.part_element_world_aabb(latch_lever, elem="lever_paddle")
    with ctx.pose({latch_pivot: 0.80}):
        rotated_lever_aabb = ctx.part_element_world_aabb(latch_lever, elem="lever_paddle")
    ctx.check(
        "reversible latch lever rotates on its own pivot",
        closed_lever_aabb is not None
        and rotated_lever_aabb is not None
        and (rotated_lever_aabb[1][2] - rotated_lever_aabb[0][2])
        > (closed_lever_aabb[1][2] - closed_lever_aabb[0][2]) + 0.045,
        details=f"closed={closed_lever_aabb}, rotated={rotated_lever_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
