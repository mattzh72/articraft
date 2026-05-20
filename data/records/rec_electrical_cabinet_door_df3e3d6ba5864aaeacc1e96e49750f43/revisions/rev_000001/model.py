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
    model = ArticulatedObject(name="industrial_electrical_cabinet_door")

    model.material("powder_gray", rgba=(0.58, 0.61, 0.60, 1.0))
    model.material("panel_gray", rgba=(0.70, 0.73, 0.72, 1.0))
    model.material("dark_seal", rgba=(0.015, 0.018, 0.016, 1.0))
    model.material("zinc_plated", rgba=(0.66, 0.68, 0.66, 1.0))
    model.material("black_handle", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("label_white", rgba=(0.94, 0.94, 0.88, 1.0))
    model.material("warning_yellow", rgba=(1.0, 0.76, 0.05, 1.0))
    model.material("warning_black", rgba=(0.0, 0.0, 0.0, 1.0))
    model.material("copper", rgba=(0.78, 0.38, 0.16, 1.0))

    enclosure = model.part("enclosure")

    # Shallow welded enclosure: rear sheet, side walls, and a folded front flange.
    enclosure.visual(
        Box((0.90, 0.030, 1.25)),
        origin=Origin(xyz=(0.0, 0.205, 0.0)),
        material="powder_gray",
        name="rear_panel",
    )
    enclosure.visual(
        Box((0.030, 0.230, 1.25)),
        origin=Origin(xyz=(-0.435, 0.105, 0.0)),
        material="powder_gray",
        name="side_wall_0",
    )
    enclosure.visual(
        Box((0.030, 0.230, 1.25)),
        origin=Origin(xyz=(0.435, 0.105, 0.0)),
        material="powder_gray",
        name="side_wall_1",
    )
    enclosure.visual(
        Box((0.90, 0.230, 0.030)),
        origin=Origin(xyz=(0.0, 0.105, 0.610)),
        material="powder_gray",
        name="top_wall",
    )
    enclosure.visual(
        Box((0.90, 0.230, 0.030)),
        origin=Origin(xyz=(0.0, 0.105, -0.610)),
        material="powder_gray",
        name="bottom_wall",
    )
    enclosure.visual(
        Box((0.065, 0.025, 1.25)),
        origin=Origin(xyz=(-0.4075, -0.002, 0.0)),
        material="powder_gray",
        name="front_flange_0",
    )
    enclosure.visual(
        Box((0.065, 0.025, 1.25)),
        origin=Origin(xyz=(0.4075, -0.002, 0.0)),
        material="powder_gray",
        name="front_flange_1",
    )
    enclosure.visual(
        Box((0.88, 0.025, 0.065)),
        origin=Origin(xyz=(0.0, -0.002, 0.5925)),
        material="powder_gray",
        name="front_flange_2",
    )
    enclosure.visual(
        Box((0.88, 0.025, 0.065)),
        origin=Origin(xyz=(0.0, -0.002, -0.5925)),
        material="powder_gray",
        name="front_flange_3",
    )

    # Replaceable black compression gasket just proud of the front flange.
    enclosure.visual(
        Box((0.020, 0.008, 1.08)),
        origin=Origin(xyz=(-0.365, -0.014, 0.0)),
        material="dark_seal",
        name="seal_0",
    )
    enclosure.visual(
        Box((0.020, 0.008, 1.08)),
        origin=Origin(xyz=(0.365, -0.014, 0.0)),
        material="dark_seal",
        name="seal_1",
    )
    enclosure.visual(
        Box((0.75, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.014, 0.535)),
        material="dark_seal",
        name="seal_2",
    )
    enclosure.visual(
        Box((0.75, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.014, -0.535)),
        material="dark_seal",
        name="seal_3",
    )

    # Strike and utility details inside the service opening.
    enclosure.visual(
        Box((0.022, 0.028, 0.185)),
        origin=Origin(xyz=(0.386, 0.012, 0.045)),
        material="zinc_plated",
        name="strike_plate",
    )
    enclosure.visual(
        Box((0.075, 0.018, 0.026)),
        origin=Origin(xyz=(0.340, 0.020, 0.045)),
        material="zinc_plated",
        name="strike_tab",
    )
    enclosure.visual(
        Box((0.10, 0.008, 0.028)),
        origin=Origin(xyz=(-0.010, -0.018, -0.535)),
        material="copper",
        name="ground_bar",
    )
    for idx, x in enumerate((-0.19, 0.0, 0.19)):
        enclosure.visual(
            Cylinder(radius=0.030, length=0.004),
            origin=Origin(xyz=(x, 0.105, 0.6265), rpy=(0.0, 0.0, 0.0)),
            material="panel_gray",
            name=f"knockout_{idx}",
        )

    # Fixed side of a continuous-duty hinge, represented with stout alternating knuckles.
    hinge_x = -0.430
    hinge_y = -0.035
    for idx, (z, length) in enumerate(((-0.445, 0.180), (0.0, 0.180), (0.445, 0.180))):
        enclosure.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material="zinc_plated",
            name=f"fixed_knuckle_{idx}",
        )
        enclosure.visual(
            Box((0.018, 0.060, length - 0.020)),
            origin=Origin(xyz=(hinge_x, -0.016, z)),
            material="zinc_plated",
            name=f"fixed_leaf_{idx}",
        )
        for bolt_idx, bolt_z in enumerate((z - 0.050, z + 0.050)):
            enclosure.visual(
                Cylinder(radius=0.0065, length=0.005),
                origin=Origin(
                    xyz=(hinge_x, -0.049, bolt_z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material="zinc_plated",
                name=f"fixed_bolt_{idx}_{bolt_idx}",
            )

    door = model.part("door")

    door.visual(
        Box((0.830, 0.030, 1.200)),
        origin=Origin(xyz=(0.445, 0.000, 0.000)),
        material="panel_gray",
        name="main_panel",
    )
    # Folded front return and raised ribs make the panel look like pressed sheet metal.
    door.visual(
        Box((0.055, 0.014, 1.170)),
        origin=Origin(xyz=(0.0575, -0.021, 0.0)),
        material="powder_gray",
        name="front_stile_0",
    )
    door.visual(
        Box((0.055, 0.014, 1.170)),
        origin=Origin(xyz=(0.8325, -0.021, 0.0)),
        material="powder_gray",
        name="front_stile_1",
    )
    door.visual(
        Box((0.830, 0.014, 0.055)),
        origin=Origin(xyz=(0.445, -0.021, 0.5725)),
        material="powder_gray",
        name="front_rail_0",
    )
    door.visual(
        Box((0.830, 0.014, 0.055)),
        origin=Origin(xyz=(0.445, -0.021, -0.5725)),
        material="powder_gray",
        name="front_rail_1",
    )
    door.visual(
        Box((0.024, 0.010, 0.880)),
        origin=Origin(xyz=(0.430, 0.020, 0.000)),
        material="powder_gray",
        name="inner_stiffener",
    )
    door.visual(
        Box((0.620, 0.010, 0.024)),
        origin=Origin(xyz=(0.470, 0.020, 0.300)),
        material="powder_gray",
        name="inner_rail_0",
    )
    door.visual(
        Box((0.620, 0.010, 0.024)),
        origin=Origin(xyz=(0.470, 0.020, -0.300)),
        material="powder_gray",
        name="inner_rail_1",
    )

    # Moving leaf of the hinge; its knuckles alternate with the fixed knuckles above.
    for idx, (z, length) in enumerate(((-0.225, 0.230), (0.225, 0.230))):
        door.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="zinc_plated",
            name=f"door_knuckle_{idx}",
        )
        door.visual(
            Box((0.085, 0.012, length - 0.020)),
            origin=Origin(xyz=(0.043, -0.004, z)),
            material="zinc_plated",
            name=f"door_leaf_{idx}",
        )
        for bolt_idx, bolt_z in enumerate((z - 0.065, z + 0.065)):
            door.visual(
                Cylinder(radius=0.0065, length=0.005),
                origin=Origin(
                    xyz=(0.058, -0.025, bolt_z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material="zinc_plated",
                name=f"door_bolt_{idx}_{bolt_idx}",
            )

    # Static latch hardware and service markings on the door.
    door.visual(
        Cylinder(radius=0.037, length=0.012),
        origin=Origin(
            xyz=(0.680, -0.021, 0.050),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="zinc_plated",
        name="latch_bushing",
    )
    door.visual(
        Box((0.090, 0.010, 0.230)),
        origin=Origin(xyz=(0.680, -0.020, 0.015)),
        material="zinc_plated",
        name="latch_plate",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(
            xyz=(0.680, -0.024, -0.055),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="black_handle",
        name="key_core",
    )
    door.visual(
        Box((0.180, 0.006, 0.070)),
        origin=Origin(xyz=(0.270, -0.018, 0.360)),
        material="label_white",
        name="id_label",
    )
    door.visual(
        Box((0.090, 0.006, 0.070)),
        origin=Origin(xyz=(0.260, -0.018, 0.245)),
        material="warning_yellow",
        name="warning_label",
    )
    door.visual(
        Box((0.050, 0.007, 0.008)),
        origin=Origin(xyz=(0.260, -0.022, 0.245)),
        material="warning_black",
        name="warning_mark",
    )
    door.visual(
        Box((0.230, 0.009, 0.145)),
        origin=Origin(xyz=(0.300, -0.019, -0.360)),
        material="panel_gray",
        name="document_pocket",
    )

    for idx, (x, z) in enumerate(((0.095, 0.515), (0.765, 0.515), (0.095, -0.515), (0.765, -0.515))):
        door.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.017, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="zinc_plated",
            name=f"corner_screw_{idx}",
        )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.0, 0.038, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="zinc_plated",
        name="shaft",
    )
    latch_handle.visual(
        Box((0.040, 0.026, 0.300)),
        origin=Origin(xyz=(0.0, -0.010, 0.000)),
        material="black_handle",
        name="grip",
    )
    latch_handle.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, -0.023, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="black_handle",
        name="grip_end_0",
    )
    latch_handle.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, -0.023, -0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="black_handle",
        name="grip_end_1",
    )
    latch_handle.visual(
        Box((0.095, 0.012, 0.030)),
        origin=Origin(xyz=(0.047, 0.078, 0.0)),
        material="zinc_plated",
        name="cam_tongue",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_handle,
        origin=Origin(xyz=(0.680, -0.033, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        door,
        latch,
        elem_a="main_panel",
        elem_b="shaft",
        reason="The latch shaft intentionally passes through the sheet-metal door.",
    )
    ctx.allow_overlap(
        door,
        latch,
        elem_a="latch_bushing",
        elem_b="shaft",
        reason="The rotating shaft is captured inside the static latch bushing.",
    )
    ctx.allow_overlap(
        door,
        latch,
        elem_a="latch_plate",
        elem_b="shaft",
        reason="The latch escutcheon is represented as a solid plate around the shaft hole.",
    )

    with ctx.pose({door_hinge: 0.0, latch_turn: 0.0}):
        ctx.expect_gap(
            enclosure,
            door,
            axis="y",
            positive_elem="seal_1",
            negative_elem="main_panel",
            min_gap=0.001,
            max_gap=0.006,
            name="door closes against gasket clearance",
        )
        ctx.expect_overlap(
            door,
            enclosure,
            axes="xz",
            elem_a="main_panel",
            elem_b="seal_1",
            min_overlap=0.015,
            name="door covers the gasketed frame",
        )
        ctx.expect_overlap(
            latch,
            door,
            axes="y",
            elem_a="shaft",
            elem_b="main_panel",
            min_overlap=0.018,
            name="latch shaft penetrates the door sheet",
        )
        ctx.expect_overlap(
            latch,
            door,
            axes="xz",
            elem_a="shaft",
            elem_b="latch_bushing",
            min_overlap=0.010,
            name="shaft stays centered in bushing",
        )
        closed_panel = ctx.part_element_world_aabb(door, elem="main_panel")
        closed_grip = ctx.part_element_world_aabb(latch, elem="grip")

    with ctx.pose({door_hinge: 1.20, latch_turn: 0.0}):
        opened_panel = ctx.part_element_world_aabb(door, elem="main_panel")

    with ctx.pose({door_hinge: 0.0, latch_turn: math.pi / 2.0}):
        turned_grip = ctx.part_element_world_aabb(latch, elem="grip")

    ctx.check(
        "door swings outward from enclosure",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[0][1] < closed_panel[0][1] - 0.25,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )
    ctx.check(
        "quarter-turn handle rotates from vertical to horizontal",
        closed_grip is not None
        and turned_grip is not None
        and (closed_grip[1][2] - closed_grip[0][2]) > 0.24
        and (turned_grip[1][0] - turned_grip[0][0]) > 0.24,
        details=f"closed={closed_grip}, turned={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
