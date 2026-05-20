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
    model = ArticulatedObject(name="wall_medicine_cabinet")

    wall_paint = model.material("warm_wall_paint", rgba=(0.78, 0.76, 0.70, 1.0))
    enamel = model.material("white_enamel", rgba=(0.92, 0.93, 0.90, 1.0))
    interior = model.material("soft_plastic_interior", rgba=(0.82, 0.86, 0.86, 1.0))
    mirror = model.material("slightly_blue_mirror", rgba=(0.70, 0.88, 0.96, 1.0))
    mirror_edge = model.material("polished_mirror_edge", rgba=(0.55, 0.66, 0.72, 1.0))
    hardware = model.material("brushed_nickel", rgba=(0.58, 0.58, 0.55, 1.0))
    dark_shadow = model.material("dark_recess_shadow", rgba=(0.08, 0.09, 0.09, 1.0))

    wall = model.part("wall")
    wall.visual(
        Box((0.78, 0.035, 0.92)),
        origin=Origin(xyz=(0.0, 0.0175, 0.46)),
        material=wall_paint,
        name="wall_slab",
    )

    case = model.part("case")
    # Shallow, wall-fixed rectangular box: back, side walls, top/bottom and a front lip.
    case.visual(
        Box((0.44, 0.012, 0.60)),
        origin=Origin(xyz=(0.0, -0.006, 0.45)),
        material=interior,
        name="back_panel",
    )
    case.visual(
        Box((0.018, 0.120, 0.60)),
        origin=Origin(xyz=(-0.211, -0.060, 0.45)),
        material=enamel,
        name="hinge_side_wall",
    )
    case.visual(
        Box((0.018, 0.120, 0.60)),
        origin=Origin(xyz=(0.211, -0.060, 0.45)),
        material=enamel,
        name="catch_side_wall",
    )
    case.visual(
        Box((0.44, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, -0.060, 0.741)),
        material=enamel,
        name="top_wall",
    )
    case.visual(
        Box((0.44, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, -0.060, 0.159)),
        material=enamel,
        name="bottom_wall",
    )
    case.visual(
        Box((0.37, 0.095, 0.012)),
        origin=Origin(xyz=(0.0, -0.058, 0.45)),
        material=interior,
        name="middle_shelf",
    )
    case.visual(
        Box((0.44, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.114, 0.735)),
        material=enamel,
        name="front_top_rail",
    )
    case.visual(
        Box((0.44, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.114, 0.165)),
        material=enamel,
        name="front_bottom_rail",
    )
    case.visual(
        Box((0.030, 0.012, 0.60)),
        origin=Origin(xyz=(-0.205, -0.114, 0.45)),
        material=enamel,
        name="front_hinge_stile",
    )
    case.visual(
        Box((0.030, 0.012, 0.60)),
        origin=Origin(xyz=(0.205, -0.114, 0.45)),
        material=enamel,
        name="front_strike_stile",
    )
    case.visual(
        Box((0.010, 0.006, 0.070)),
        origin=Origin(xyz=(0.224, -0.123, 0.45)),
        material=hardware,
        name="strike_plate",
    )

    # Two compact exposed side hinges.  The case owns the fixed leaves, outer
    # knuckles, and pins; the door owns the center knuckles and straps.
    for label, zc in (("upper", 0.635), ("lower", 0.265)):
        case.visual(
            Box((0.008, 0.030, 0.082)),
            origin=Origin(xyz=(-0.216, -0.132, zc)),
            material=hardware,
            name=f"{label}_case_leaf",
        )
        for suffix, dz in (("lower", -0.026), ("upper", 0.026)):
            case.visual(
                Box((0.018, 0.018, 0.022)),
                origin=Origin(xyz=(-0.225, -0.140, zc + dz)),
                material=hardware,
                name=f"{label}_{suffix}_knuckle_bridge",
            )
            case.visual(
                Cylinder(radius=0.0075, length=0.022),
                origin=Origin(xyz=(-0.235, -0.140, zc + dz)),
                material=hardware,
                name=f"{label}_{suffix}_case_knuckle",
            )
        case.visual(
            Cylinder(radius=0.0028, length=0.082),
            origin=Origin(xyz=(-0.235, -0.140, zc)),
            material=dark_shadow,
            name=f"{label}_hinge_pin",
        )

    door = model.part("door")
    # The child frame is on the hinge axis; at q=0 the door spans local +X.
    door.visual(
        Box((0.420, 0.024, 0.595)),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material=enamel,
        name="door_panel",
    )
    door.visual(
        Box((0.328, 0.004, 0.485)),
        origin=Origin(xyz=(0.230, -0.014, 0.0)),
        material=mirror,
        name="mirror_pane",
    )
    door.visual(
        Box((0.420, 0.006, 0.040)),
        origin=Origin(xyz=(0.235, -0.014, 0.278)),
        material=mirror_edge,
        name="top_mirror_frame",
    )
    door.visual(
        Box((0.420, 0.006, 0.040)),
        origin=Origin(xyz=(0.235, -0.014, -0.278)),
        material=mirror_edge,
        name="bottom_mirror_frame",
    )
    door.visual(
        Box((0.040, 0.006, 0.555)),
        origin=Origin(xyz=(0.035, -0.014, 0.0)),
        material=mirror_edge,
        name="hinge_mirror_frame",
    )
    door.visual(
        Box((0.040, 0.006, 0.555)),
        origin=Origin(xyz=(0.435, -0.014, 0.0)),
        material=mirror_edge,
        name="catch_mirror_frame",
    )
    for label, zc in (("upper", 0.185), ("lower", -0.185)):
        door.visual(
            Box((0.052, 0.004, 0.080)),
            origin=Origin(xyz=(0.026, -0.016, zc)),
            material=hardware,
            name=f"{label}_door_leaf",
        )
        door.visual(
            Box((0.014, 0.014, 0.024)),
            origin=Origin(xyz=(0.004, -0.007, zc)),
            material=hardware,
            name=f"{label}_door_knuckle_bridge",
        )
        door.visual(
            Cylinder(radius=0.0075, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=hardware,
            name=f"{label}_door_knuckle",
        )

    catch_knob = model.part("catch_knob")
    catch_knob.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="knob_hub",
    )
    catch_knob.visual(
        Box((0.012, 0.008, 0.054)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=hardware,
        name="turn_bar",
    )
    catch_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_shadow,
        name="short_pivot",
    )

    model.articulation(
        "wall_to_case",
        ArticulationType.FIXED,
        parent=wall,
        child=case,
        origin=Origin(),
    )
    model.articulation(
        "case_to_door",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(-0.235, -0.140, 0.45)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_catch_knob",
        ArticulationType.REVOLUTE,
        parent=door,
        child=catch_knob,
        origin=Origin(xyz=(0.425, -0.017, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-1.5708, upper=1.5708),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall")
    case = object_model.get_part("case")
    door = object_model.get_part("door")
    catch_knob = object_model.get_part("catch_knob")
    door_hinge = object_model.get_articulation("case_to_door")
    knob_pivot = object_model.get_articulation("door_to_catch_knob")

    for label in ("upper", "lower"):
        ctx.allow_overlap(
            case,
            door,
            elem_a=f"{label}_hinge_pin",
            elem_b=f"{label}_door_knuckle",
            reason="The compact hinge pin intentionally passes through the rotating door knuckle.",
        )
        ctx.expect_within(
            case,
            door,
            axes="xy",
            inner_elem=f"{label}_hinge_pin",
            outer_elem=f"{label}_door_knuckle",
            margin=0.001,
            name=f"{label} hinge pin is centered in door knuckle",
        )
        ctx.expect_overlap(
            case,
            door,
            axes="z",
            elem_a=f"{label}_hinge_pin",
            elem_b=f"{label}_door_knuckle",
            min_overlap=0.020,
            name=f"{label} hinge pin runs through door knuckle",
        )

    ctx.expect_gap(
        wall,
        case,
        axis="y",
        positive_elem="wall_slab",
        negative_elem="back_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="cabinet back is flush to wall",
    )
    ctx.expect_gap(
        case,
        door,
        axis="y",
        positive_elem="front_strike_stile",
        negative_elem="door_panel",
        min_gap=0.003,
        max_gap=0.015,
        name="closed mirrored door stands just proud of case",
    )
    ctx.expect_contact(
        catch_knob,
        door,
        elem_a="knob_hub",
        elem_b="catch_mirror_frame",
        contact_tol=0.001,
        name="catch knob hub seats on the free edge frame",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door hinge opens outward from wall",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.12,
        details=f"closed={closed_aabb}, opened={open_aabb}",
    )

    rest_knob = ctx.part_world_aabb(catch_knob)
    with ctx.pose({knob_pivot: math.pi / 2.0}):
        turned_knob = ctx.part_world_aabb(catch_knob)
    if rest_knob is not None and turned_knob is not None:
        rest_dx = rest_knob[1][0] - rest_knob[0][0]
        rest_dz = rest_knob[1][2] - rest_knob[0][2]
        turned_dx = turned_knob[1][0] - turned_knob[0][0]
        turned_dz = turned_knob[1][2] - turned_knob[0][2]
        knob_turns = rest_dz > rest_dx * 1.4 and turned_dx > turned_dz * 1.4
    else:
        knob_turns = False
    ctx.check(
        "catch knob visibly rotates on short pivot",
        knob_turns,
        details=f"rest={rest_knob}, turned={turned_knob}",
    )

    return ctx.report()


object_model = build_object_model()
