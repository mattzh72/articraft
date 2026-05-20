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
    model = ArticulatedObject(name="wall_panel_distribution_board")

    paint = Material("warm_gray_powdercoat", rgba=(0.72, 0.72, 0.68, 1.0))
    dark = Material("black_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    shadow = Material("dark_cavity", rgba=(0.06, 0.065, 0.07, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    breaker = Material("breaker_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    label = Material("paper_label", rgba=(0.90, 0.88, 0.72, 1.0))
    copper = Material("copper_busbar", rgba=(0.80, 0.38, 0.12, 1.0))

    housing = model.part("housing")

    outer_w = 0.330
    height = 0.750
    depth = 0.115
    wall = 0.024
    hinge_x = -0.183
    hinge_y = 0.128
    door_w = 0.348
    door_h = 0.760

    # Shallow wall-mounted metal cabinet: back pan, folded side walls, and a
    # front return flange framing the opening.
    housing.visual(
        Box((outer_w, 0.018, height)),
        origin=Origin(xyz=(0.0, 0.009, height / 2.0)),
        material=paint,
        name="back_pan",
    )
    housing.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall / 2.0, depth / 2.0, height / 2.0)),
        material=paint,
        name="left_side_wall",
    )
    # Right side is split around the latch keep so the rotating cam has a real
    # relief pocket instead of intersecting a solid side wall.
    housing.visual(
        Box((wall, depth, 0.335)),
        origin=Origin(xyz=(outer_w / 2.0 - wall / 2.0, depth / 2.0, 0.1675)),
        material=paint,
        name="right_wall_lower",
    )
    housing.visual(
        Box((wall, depth, 0.325)),
        origin=Origin(xyz=(outer_w / 2.0 - wall / 2.0, depth / 2.0, 0.5875)),
        material=paint,
        name="right_wall_upper",
    )
    housing.visual(
        Box((outer_w, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, wall / 2.0)),
        material=paint,
        name="bottom_wall",
    )
    housing.visual(
        Box((outer_w, depth, wall)),
        origin=Origin(xyz=(0.0, depth / 2.0, height - wall / 2.0)),
        material=paint,
        name="top_wall",
    )
    flange_t = 0.008
    housing.visual(
        Box((outer_w, flange_t, 0.030)),
        origin=Origin(xyz=(0.0, depth - flange_t / 2.0, 0.035)),
        material=paint,
        name="bottom_flange",
    )
    housing.visual(
        Box((outer_w, flange_t, 0.030)),
        origin=Origin(xyz=(0.0, depth - flange_t / 2.0, height - 0.035)),
        material=paint,
        name="top_flange",
    )
    housing.visual(
        Box((0.018, flange_t, height - 0.035)),
        origin=Origin(xyz=(-outer_w / 2.0 + 0.020, depth - flange_t / 2.0, height / 2.0)),
        material=paint,
        name="left_flange",
    )
    housing.visual(
        Box((0.018, flange_t, 0.315)),
        origin=Origin(xyz=(outer_w / 2.0 - 0.020, depth - flange_t / 2.0, 0.1775)),
        material=paint,
        name="right_flange_lower",
    )
    housing.visual(
        Box((0.018, flange_t, 0.315)),
        origin=Origin(xyz=(outer_w / 2.0 - 0.020, depth - flange_t / 2.0, 0.5725)),
        material=paint,
        name="right_flange_upper",
    )
    housing.visual(
        Box((outer_w - 0.055, 0.006, height - 0.115)),
        origin=Origin(xyz=(0.0, 0.0205, height / 2.0)),
        material=galvanized,
        name="mounting_plate",
    )

    # Latch keep in the side-wall relief; the quarter-turn cam is intentionally
    # captured behind this folded keeper when the door is closed.
    housing.visual(
        Box((0.028, 0.014, 0.060)),
        origin=Origin(xyz=(outer_w / 2.0 - 0.005, 0.111, 0.380)),
        material=galvanized,
        name="latch_keeper",
    )
    housing.visual(
        Box((0.018, 0.090, 0.026)),
        origin=Origin(xyz=(outer_w / 2.0 - 0.006, 0.062, 0.380)),
        material=galvanized,
        name="keeper_bracket",
    )

    # Visible internal distribution gear so the open door reads as an electrical
    # board rather than an empty box.
    for row_z in (0.510, 0.275):
        housing.visual(
            Box((0.250, 0.012, 0.014)),
            origin=Origin(xyz=(0.0, 0.041, row_z)),
            material=galvanized,
            name=f"din_rail_{int(row_z * 1000)}",
        )
        for sx in (-0.105, 0.105):
            housing.visual(
                Box((0.014, 0.026, 0.032)),
                origin=Origin(xyz=(sx, 0.030, row_z)),
                material=galvanized,
                name=f"rail_standoff_{int(row_z * 1000)}_{'neg' if sx < 0 else 'pos'}",
            )
        for i, x in enumerate((-0.084, -0.050, -0.016, 0.018, 0.052, 0.086)):
            housing.visual(
                Box((0.032, 0.040, 0.090)),
                origin=Origin(xyz=(x, 0.064, row_z + 0.018)),
                material=breaker,
                name=f"breaker_{int(row_z * 1000)}_{i}",
            )
            housing.visual(
                Box((0.018, 0.014, 0.034)),
                origin=Origin(xyz=(x, 0.091, row_z + 0.023)),
                material=dark,
                name=f"toggle_{int(row_z * 1000)}_{i}",
            )

    housing.visual(
        Box((0.230, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.030, 0.130)),
        material=copper,
        name="earth_busbar",
    )
    for i, x in enumerate((-0.110, -0.055, 0.000, 0.055, 0.110)):
        housing.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, 0.039, 0.130), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"bus_screw_{i}",
        )

    # Fixed hinge leaves on the cabinet side; the rotating barrels are part of
    # the door and share the same vertical revolute axis.
    for name, zc in (("lower", 0.205), ("upper", 0.560)):
        housing.visual(
            Box((0.020, 0.006, 0.118)),
            origin=Origin(xyz=(-outer_w / 2.0 - 0.006, depth - 0.006, zc)),
            material=galvanized,
            name=f"{name}_hinge_leaf",
        )
        housing.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(
                xyz=(-outer_w / 2.0 - 0.006, depth - 0.001, zc + 0.035),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"{name}_hinge_screw_0",
        )
        housing.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(
                xyz=(-outer_w / 2.0 - 0.006, depth - 0.001, zc - 0.035),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"{name}_hinge_screw_1",
        )

    door = model.part("door")
    door.visual(
        Box((door_w, 0.014, door_h)),
        # The door frame is the vertical hinge line; the leaf extends in +X.
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h / 2.0)),
        material=paint,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.070, 0.004, door_h - 0.110)),
        origin=Origin(xyz=(door_w / 2.0 + 0.012, 0.009, door_h / 2.0)),
        material=Material("slightly_recessed_gray", rgba=(0.66, 0.67, 0.64, 1.0)),
        name="recessed_field",
    )
    door.visual(
        Box((0.085, 0.005, 0.035)),
        origin=Origin(xyz=(door_w * 0.56, 0.012, 0.635)),
        material=label,
        name="circuit_label",
    )
    door.visual(
        Box((0.016, 0.016, door_h)),
        origin=Origin(xyz=(door_w - 0.008, 0.0, door_h / 2.0)),
        material=paint,
        name="folded_stile",
    )
    for name, zc in (("lower", 0.205), ("upper", 0.560)):
        door.visual(
            Cylinder(radius=0.011, length=0.100),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=galvanized,
            name=f"{name}_hinge_knuckle",
        )
        door.visual(
            Box((0.055, 0.006, 0.075)),
            origin=Origin(xyz=(0.029, 0.006, zc)),
            material=galvanized,
            name=f"{name}_door_leaf",
        )
    for i, (x, z) in enumerate(((0.080, 0.070), (0.280, 0.070), (0.080, 0.690), (0.280, 0.690))):
        door.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(xyz=(x, 0.0105, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"door_screw_{i}",
        )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    handle_latch = model.part("handle_latch")
    handle_latch.visual(
        Cylinder(radius=0.026, length=0.009),
        origin=Origin(xyz=(0.0, 0.0045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_rose",
    )
    handle_latch.visual(
        Box((0.018, 0.018, 0.115)),
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=dark,
        name="turn_grip",
    )
    handle_latch.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="latch_stem",
    )
    handle_latch.visual(
        Box((0.024, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=galvanized,
        name="cam_hub",
    )
    handle_latch.visual(
        Box((0.090, 0.010, 0.018)),
        origin=Origin(xyz=(0.047, -0.023, 0.0)),
        material=galvanized,
        name="cam_tongue",
    )

    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle_latch,
        origin=Origin(xyz=(door_w - 0.055, 0.0075, 0.380)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    latch = object_model.get_part("handle_latch")
    door_joint = object_model.get_articulation("housing_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.allow_overlap(
        door,
        latch,
        elem_a="door_panel",
        elem_b="latch_stem",
        reason="The quarter-turn latch stem intentionally passes through the sheet-metal door.",
    )
    ctx.allow_overlap(
        housing,
        latch,
        elem_a="latch_keeper",
        elem_b="cam_tongue",
        reason="The cam tongue is intentionally captured behind the keeper in the latched pose.",
    )

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        negative_elem="top_flange",
        min_gap=0.0,
        max_gap=0.010,
        name="closed door stands just proud of cabinet frame",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="door_panel",
        elem_b="back_pan",
        min_overlap=0.25,
        name="single leaf covers the narrow cabinet opening",
    )
    ctx.expect_overlap(
        door,
        latch,
        axes="y",
        elem_a="door_panel",
        elem_b="latch_stem",
        min_overlap=0.010,
        name="latch stem crosses the door skin",
    )
    ctx.expect_overlap(
        housing,
        latch,
        axes="xyz",
        elem_a="latch_keeper",
        elem_b="cam_tongue",
        min_overlap=0.006,
        name="cam tongue is retained by keeper",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward on left vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.22,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_grip = ctx.part_element_world_aabb(latch, elem="turn_grip")
    with ctx.pose({latch_joint: math.pi / 2.0}):
        turned_grip = ctx.part_element_world_aabb(latch, elem="turn_grip")
    ctx.check(
        "quarter turn handle rotates ninety degrees",
        rest_grip is not None
        and turned_grip is not None
        and (rest_grip[1][2] - rest_grip[0][2]) > 0.09
        and (turned_grip[1][0] - turned_grip[0][0]) > 0.09,
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
