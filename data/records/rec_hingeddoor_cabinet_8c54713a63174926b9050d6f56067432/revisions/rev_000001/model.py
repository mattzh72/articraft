from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_cabinet_flip_tray")

    walnut = model.material("warm_walnut", color=(0.45, 0.25, 0.11, 1.0))
    dark_walnut = model.material("dark_inner_walnut", color=(0.25, 0.13, 0.06, 1.0))
    brass = model.material("brushed_brass", color=(0.82, 0.62, 0.24, 1.0))

    case = model.part("case")

    # Realistic bedside scale: a roughly half-meter wide, hollow wooden case.
    case_w = 0.52
    case_d = 0.40
    case_h = 0.62
    wall = 0.035
    panel_h = case_h - 0.06
    center_z = 0.34

    case.visual(
        Box((wall, case_d, panel_h)),
        origin=Origin(xyz=(-case_w / 2 + wall / 2, 0.0, center_z)),
        material=walnut,
        name="side_0",
    )
    case.visual(
        Box((wall, case_d, panel_h)),
        origin=Origin(xyz=(case_w / 2 - wall / 2, 0.0, center_z)),
        material=walnut,
        name="side_1",
    )
    case.visual(
        Box((case_w, case_d, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=walnut,
        name="bottom_panel",
    )
    case.visual(
        Box((case_w, case_d, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, case_h - 0.020)),
        material=walnut,
        name="top_panel",
    )
    case.visual(
        Box((case_w - 0.015, 0.024, panel_h)),
        origin=Origin(xyz=(0.0, case_d / 2 - 0.012, center_z)),
        material=dark_walnut,
        name="back_panel",
    )

    # Projecting face frame around the open front.
    frame_y = -case_d / 2 - 0.0125
    frame_depth = 0.025
    frame_w = 0.045
    frame_z = 0.055
    case.visual(
        Box((frame_w, frame_depth, panel_h)),
        origin=Origin(xyz=(-case_w / 2 + frame_w / 2, frame_y, center_z)),
        material=walnut,
        name="front_stile_0",
    )
    case.visual(
        Box((frame_w, frame_depth, panel_h)),
        origin=Origin(xyz=(case_w / 2 - frame_w / 2, frame_y, center_z)),
        material=walnut,
        name="front_stile_1",
    )
    case.visual(
        Box((case_w, frame_depth, frame_z)),
        origin=Origin(xyz=(0.0, frame_y, case_h - frame_z / 2)),
        material=walnut,
        name="front_rail_top",
    )
    case.visual(
        Box((case_w, frame_depth, frame_z)),
        origin=Origin(xyz=(0.0, frame_y, 0.060 + frame_z / 2)),
        material=walnut,
        name="front_rail_bottom",
    )
    # Small plinth feet stay tied into the base slab while giving a cabinet-like stance.
    foot_size = (0.070, 0.070, 0.055)
    for ix, x in enumerate((-0.185, 0.185)):
        for iy, y in enumerate((-0.125, 0.125)):
            case.visual(
                Box(foot_size),
                origin=Origin(xyz=(x, y, 0.0275)),
                material=dark_walnut,
                name=f"foot_{ix}_{iy}",
            )

    # Lower hinge bearing blocks for the inner tray, fixed to the side stiles.
    tray_hinge_y = -0.145
    tray_hinge_z = 0.130
    for i, x in enumerate((-0.215, 0.215)):
        case.visual(
            Box((0.030, 0.050, 0.038)),
            origin=Origin(xyz=(x, tray_hinge_y, tray_hinge_z)),
            material=brass,
            name=f"tray_bearing_{i}",
        )

    door = model.part("door")
    door_w = 0.510
    door_h = 0.540
    door_t = 0.030
    door_rail = 0.055
    door_center_x = door_w / 2

    # The door part frame is the vertical hinge line; all door geometry extends
    # along local +X in the closed pose.
    door.visual(
        Box((door_rail, door_t, door_h)),
        origin=Origin(xyz=(door_rail / 2, 0.0, 0.0)),
        material=walnut,
        name="hinge_stile",
    )
    door.visual(
        Box((door_rail, door_t, door_h)),
        origin=Origin(xyz=(door_w - door_rail / 2, 0.0, 0.0)),
        material=walnut,
        name="pull_stile",
    )
    door.visual(
        Box((door_w, door_t, door_rail)),
        origin=Origin(xyz=(door_center_x, 0.0, door_h / 2 - door_rail / 2)),
        material=walnut,
        name="top_rail",
    )
    door.visual(
        Box((door_w, door_t, door_rail)),
        origin=Origin(xyz=(door_center_x, 0.0, -door_h / 2 + door_rail / 2)),
        material=walnut,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w - 2 * door_rail, 0.014, door_h - 2 * door_rail)),
        origin=Origin(xyz=(door_center_x, 0.006, 0.0)),
        material=dark_walnut,
        name="recessed_panel",
    )
    door.visual(
        Cylinder(radius=0.019, length=0.040),
        origin=Origin(
            xyz=(door_w - 0.060, -0.032, 0.005),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=brass,
        name="round_pull",
    )
    door.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(door_w - 0.060, -0.056, 0.005)),
        material=brass,
        name="pull_cap",
    )
    for i, z in enumerate((-0.175, 0.0, 0.175)):
        door.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=f"door_knuckle_{i}",
        )

    tray = model.part("tray")
    tray_w = 0.380
    tray_d = 0.220
    tray_base_t = 0.018
    tray_lip_h = 0.045
    tray_lip_t = 0.018

    # In the stowed pose the tray stands upright inside the cabinet.  The tray's
    # local +Z direction is its folded-up depth; a positive rotation around +X
    # flips that depth outward toward the user.
    tray.visual(
        Box((tray_w, tray_base_t, tray_d)),
        origin=Origin(xyz=(0.0, 0.0, tray_d / 2)),
        material=dark_walnut,
        name="tray_floor",
    )
    tray.visual(
        Box((tray_lip_t, tray_lip_h, tray_d)),
        origin=Origin(xyz=(-tray_w / 2 + tray_lip_t / 2, tray_lip_h / 2, tray_d / 2)),
        material=walnut,
        name="side_lip_0",
    )
    tray.visual(
        Box((tray_lip_t, tray_lip_h, tray_d)),
        origin=Origin(xyz=(tray_w / 2 - tray_lip_t / 2, tray_lip_h / 2, tray_d / 2)),
        material=walnut,
        name="side_lip_1",
    )
    tray.visual(
        Box((tray_w, tray_lip_h, tray_lip_t)),
        origin=Origin(xyz=(0.0, tray_lip_h / 2, tray_d - tray_lip_t / 2)),
        material=walnut,
        name="front_lip",
    )
    tray.visual(
        Box((tray_w, tray_lip_h * 0.70, tray_lip_t)),
        origin=Origin(xyz=(0.0, tray_lip_h * 0.35 / 2, tray_lip_t / 2)),
        material=walnut,
        name="hinge_lip",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=brass,
        name="tray_barrel",
    )

    door_hinge_x = -case_w / 2 + 0.005
    door_hinge_y = -0.240
    door_hinge_z = center_z

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=tray,
        origin=Origin(xyz=(0.0, tray_hinge_y, tray_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    door = object_model.get_part("door")
    tray = object_model.get_part("tray")
    door_hinge = object_model.get_articulation("door_hinge")
    tray_hinge = object_model.get_articulation("tray_hinge")

    ctx.check(
        "door hinge is vertical",
        tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "tray hinge is horizontal",
        tuple(round(v, 6) for v in tray_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tray_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0, tray_hinge: 0.0}):
        ctx.expect_gap(
            case,
            door,
            axis="y",
            max_gap=0.010,
            max_penetration=0.001,
            name="closed door sits just proud of the face frame",
        )
        ctx.expect_overlap(
            door,
            case,
            axes="xz",
            min_overlap=0.45,
            name="framed door covers the cabinet opening",
        )
        ctx.expect_within(
            tray,
            case,
            axes="xy",
            margin=0.002,
            name="stowed tray is inside the open case",
        )
        stowed_lip = ctx.part_element_world_aabb(tray, elem="front_lip")

    with ctx.pose({door_hinge: 1.60, tray_hinge: 1.35}):
        opened_lip = ctx.part_element_world_aabb(tray, elem="front_lip")
        open_pull = ctx.part_element_world_aabb(door, elem="pull_stile")
        ctx.expect_gap(
            tray,
            case,
            axis="z",
            negative_elem="front_rail_bottom",
            max_penetration=0.020,
            name="opened tray stays above the cabinet base",
        )

    ctx.check(
        "tray flips outward from its lower hinge",
        stowed_lip is not None
        and opened_lip is not None
        and opened_lip[0][1] < stowed_lip[0][1] - 0.12,
        details=f"stowed_lip={stowed_lip}, opened_lip={opened_lip}",
    )
    ctx.check(
        "door swings outward from the side hinge",
        open_pull is not None and open_pull[0][1] < -0.55,
        details=f"open_pull={open_pull}",
    )

    return ctx.report()


object_model = build_object_model()
