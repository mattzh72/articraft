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
    model = ArticulatedObject(name="relay_protection_panel")

    # Overall proportions: a floor-standing, tall/narrow steel relay cabinet.
    cabinet_w = 0.62
    cabinet_d = 0.30
    cabinet_h = 1.85
    wall = 0.025

    steel = model.material("painted_steel", rgba=(0.56, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_hinge_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black = model.material("black_gasket", rgba=(0.02, 0.022, 0.025, 1.0))
    glass = model.material("smoky_window_glass", rgba=(0.14, 0.22, 0.30, 0.46))
    panel_paint = model.material("warm_instrument_panel", rgba=(0.82, 0.80, 0.72, 1.0))
    relay_gray = model.material("relay_case_gray", rgba=(0.70, 0.72, 0.72, 1.0))
    label_white = model.material("engraved_white_labels", rgba=(0.92, 0.92, 0.86, 1.0))
    red = model.material("red_wire", rgba=(0.85, 0.05, 0.03, 1.0))
    blue = model.material("blue_wire", rgba=(0.05, 0.18, 0.82, 1.0))
    yellow = model.material("yellow_wire", rgba=(0.95, 0.70, 0.05, 1.0))
    copper = model.material("copper_terminals", rgba=(0.75, 0.42, 0.16, 1.0))

    cabinet = model.part("cabinet")

    # Hollow cabinet carcass: side walls, back, top/bottom and front returns leave
    # the front open when the door and swing-out instrument panel are articulated.
    cabinet.visual(
        Box((cabinet_w, wall, cabinet_h)),
        origin=Origin(xyz=(0.0, cabinet_d / 2.0 - wall / 2.0, cabinet_h / 2.0)),
        material=steel,
        name="back_wall",
    )
    cabinet.visual(
        Box((wall, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + wall / 2.0, 0.0, cabinet_h / 2.0)),
        material=steel,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((wall, cabinet_d, cabinet_h)),
        origin=Origin(xyz=(cabinet_w / 2.0 - wall / 2.0, 0.0, cabinet_h / 2.0)),
        material=steel,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((cabinet_w, cabinet_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - wall / 2.0)),
        material=steel,
        name="top_wall",
    )
    cabinet.visual(
        Box((cabinet_w, cabinet_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=steel,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((0.040, wall, cabinet_h)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + 0.020, -cabinet_d / 2.0 + wall / 2.0, cabinet_h / 2.0)),
        material=steel,
        name="front_return_0",
    )
    cabinet.visual(
        Box((0.040, wall, cabinet_h)),
        origin=Origin(xyz=(cabinet_w / 2.0 - 0.020, -cabinet_d / 2.0 + wall / 2.0, cabinet_h / 2.0)),
        material=steel,
        name="front_return_1",
    )
    cabinet.visual(
        Box((cabinet_w, wall, 0.055)),
        origin=Origin(xyz=(0.0, -cabinet_d / 2.0 + wall / 2.0, cabinet_h - 0.0275)),
        material=steel,
        name="front_header",
    )
    cabinet.visual(
        Box((cabinet_w, wall, 0.055)),
        origin=Origin(xyz=(0.0, -cabinet_d / 2.0 + wall / 2.0, 0.0275)),
        material=steel,
        name="front_sill",
    )

    # Stationary hinge leaves at the two external knuckles.
    for i, zc in enumerate((0.43, 1.45)):
        cabinet.visual(
            Box((0.055, 0.018, 0.32)),
            origin=Origin(xyz=(-cabinet_w / 2.0 - 0.010, -cabinet_d / 2.0 + 0.010, zc)),
            material=dark_steel,
            name=f"door_hinge_leaf_{i}",
        )

    # Interior hinge rail and wiring backplate behind the swing-out panel.
    cabinet.visual(
        Box((0.028, 0.028, 1.48)),
        origin=Origin(xyz=(-0.294, -0.105, 0.93)),
        material=dark_steel,
        name="inner_hinge_rail",
    )
    cabinet.visual(
        Box((0.50, 0.012, 1.28)),
        origin=Origin(xyz=(0.02, cabinet_d / 2.0 - wall - 0.006, 0.93)),
        material=panel_paint,
        name="wiring_backplate",
    )
    cabinet.visual(
        Box((0.055, 0.020, 1.05)),
        origin=Origin(xyz=(-0.19, cabinet_d / 2.0 - wall - 0.022, 0.88)),
        material=label_white,
        name="wire_duct_0",
    )
    cabinet.visual(
        Box((0.055, 0.020, 1.05)),
        origin=Origin(xyz=(0.23, cabinet_d / 2.0 - wall - 0.022, 0.88)),
        material=label_white,
        name="wire_duct_1",
    )
    for i, (x, mat) in enumerate(((-0.08, red), (0.00, yellow), (0.08, blue))):
        cabinet.visual(
            Box((0.015, 0.012, 0.84)),
            origin=Origin(xyz=(x, cabinet_d / 2.0 - wall - 0.018, 0.96)),
            material=mat,
            name=f"wire_run_{i}",
        )
    cabinet.visual(
        Box((0.42, 0.026, 0.055)),
        origin=Origin(xyz=(0.02, cabinet_d / 2.0 - wall - 0.025, 0.30)),
        material=copper,
        name="terminal_strip",
    )

    # Front door.  Its part frame is the lower point of the left hinge line;
    # closed geometry extends along +X from that frame.
    door_w = 0.64
    door_h = 1.78
    door_t = 0.025
    door_y = -cabinet_d / 2.0 - door_t / 2.0 - 0.006
    door_z0 = 0.035
    front_door = model.part("front_door")
    front_door.visual(
        Box((0.110, door_t, door_h)),
        origin=Origin(xyz=(0.055, 0.0, door_h / 2.0)),
        material=steel,
        name="door_left_stile",
    )
    front_door.visual(
        Box((0.110, door_t, door_h)),
        origin=Origin(xyz=(door_w - 0.055, 0.0, door_h / 2.0)),
        material=steel,
        name="door_right_stile",
    )
    front_door.visual(
        Box((door_w, door_t, 1.00)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, 0.50)),
        material=steel,
        name="door_lower_sheet",
    )
    front_door.visual(
        Box((door_w, door_t, 0.30)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h - 0.15)),
        material=steel,
        name="door_upper_rail",
    )
    front_door.visual(
        Box((0.38, 0.006, 0.44)),
        origin=Origin(xyz=(door_w / 2.0, -door_t / 2.0 - 0.003, 1.25)),
        material=glass,
        name="viewing_window",
    )
    front_door.visual(
        Box((0.43, 0.008, 0.030)),
        origin=Origin(xyz=(door_w / 2.0, -door_t / 2.0 - 0.004, 1.485)),
        material=black,
        name="window_gasket_top",
    )
    front_door.visual(
        Box((0.43, 0.008, 0.030)),
        origin=Origin(xyz=(door_w / 2.0, -door_t / 2.0 - 0.004, 1.015)),
        material=black,
        name="window_gasket_bottom",
    )
    front_door.visual(
        Box((0.030, 0.008, 0.49)),
        origin=Origin(xyz=(0.115, -door_t / 2.0 - 0.004, 1.25)),
        material=black,
        name="window_gasket_0",
    )
    front_door.visual(
        Box((0.030, 0.008, 0.49)),
        origin=Origin(xyz=(door_w - 0.115, -door_t / 2.0 - 0.004, 1.25)),
        material=black,
        name="window_gasket_1",
    )
    for i, zc in enumerate((0.43 - door_z0, 1.45 - door_z0)):
        front_door.visual(
            Cylinder(radius=0.016, length=0.24),
            origin=Origin(xyz=(-0.012, 0.0, zc)),
            material=dark_steel,
            name=f"door_hinge_barrel_{i}",
        )
        front_door.visual(
            Box((0.080, 0.008, 0.14)),
            origin=Origin(xyz=(0.030, door_t / 2.0 + 0.004, zc)),
            material=dark_steel,
            name=f"door_hinge_strap_{i}",
        )

    model.articulation(
        "cabinet_to_front_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=front_door,
        origin=Origin(xyz=(-cabinet_w / 2.0, door_y, door_z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=0.0, upper=1.9),
    )

    # A small latch handle rotates on the door skin.
    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_boss",
    )
    latch_handle.visual(
        Box((0.115, 0.014, 0.026)),
        origin=Origin(xyz=(0.030, -0.004, 0.0)),
        material=dark_steel,
        name="handle_grip",
    )
    model.articulation(
        "front_door_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=front_door,
        child=latch_handle,
        origin=Origin(xyz=(door_w - 0.105, -door_t / 2.0 - 0.017, 0.86)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    # Inner swing-out relay/instrument panel.  Its hinge line is also on the
    # left edge, giving access to wiring and terminal strips behind it.
    panel_w = 0.52
    panel_h = 1.36
    panel_t = 0.020
    panel_y = -0.105
    panel_z0 = 0.25
    instrument_panel = model.part("instrument_panel")
    instrument_panel.visual(
        Box((panel_w, panel_t, panel_h)),
        origin=Origin(xyz=(panel_w / 2.0, 0.0, panel_h / 2.0)),
        material=panel_paint,
        name="panel_plate",
    )
    instrument_panel.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(-0.008, 0.0, 0.33)),
        material=dark_steel,
        name="panel_hinge_barrel_0",
    )
    instrument_panel.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(-0.008, 0.0, 1.06)),
        material=dark_steel,
        name="panel_hinge_barrel_1",
    )
    for i, (x, z) in enumerate(((0.15, 0.86), (0.37, 0.86), (0.15, 0.58), (0.37, 0.58))):
        instrument_panel.visual(
            Box((0.135, 0.032, 0.18)),
            origin=Origin(xyz=(x, -panel_t / 2.0 - 0.016, z)),
            material=relay_gray,
            name=f"relay_module_{i}",
        )
        instrument_panel.visual(
            Box((0.100, 0.006, 0.035)),
            origin=Origin(xyz=(x, -panel_t / 2.0 - 0.034, z + 0.050)),
            material=label_white,
            name=f"relay_label_{i}",
        )
    for i, x in enumerate((0.18, 0.34)):
        instrument_panel.visual(
            Cylinder(radius=0.047, length=0.014),
            origin=Origin(xyz=(x, -panel_t / 2.0 - 0.007, 1.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"meter_bezel_{i}",
        )
        instrument_panel.visual(
            Cylinder(radius=0.037, length=0.016),
            origin=Origin(xyz=(x, -panel_t / 2.0 - 0.017, 1.15), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=label_white,
            name=f"meter_face_{i}",
        )
    instrument_panel.visual(
        Box((0.40, 0.030, 0.055)),
        origin=Origin(xyz=(panel_w / 2.0, -panel_t / 2.0 - 0.015, 0.25)),
        material=copper,
        name="panel_terminal_strip",
    )

    model.articulation(
        "cabinet_to_instrument_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=instrument_panel,
        origin=Origin(xyz=(-0.26, panel_y, panel_z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    front_door = object_model.get_part("front_door")
    instrument_panel = object_model.get_part("instrument_panel")
    latch_handle = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("cabinet_to_front_door")
    panel_hinge = object_model.get_articulation("cabinet_to_instrument_panel")
    handle_joint = object_model.get_articulation("front_door_to_latch_handle")

    ctx.check(
        "two nested swing-out access panels are articulated",
        door_hinge.motion_limits is not None
        and panel_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper >= 1.8
        and panel_hinge.motion_limits.upper >= 1.5,
        details="front door and inner relay panel should both swing wide for service access",
    )
    ctx.check(
        "latch handle has a quarter-turn joint",
        handle_joint.motion_limits is not None and handle_joint.motion_limits.upper >= math.pi / 2.0 - 1e-6,
        details="the visible cabinet latch should rotate like a real handle",
    )

    with ctx.pose({door_hinge: 0.0, panel_hinge: 0.0, handle_joint: 0.0}):
        ctx.expect_gap(
            cabinet,
            front_door,
            axis="y",
            min_gap=0.002,
            max_gap=0.012,
            negative_elem="door_lower_sheet",
            name="closed front door sits just proud of the cabinet opening",
        )
        ctx.expect_gap(
            instrument_panel,
            front_door,
            axis="y",
            min_gap=0.004,
            positive_elem="panel_plate",
            negative_elem="door_lower_sheet",
            name="inner instrument panel is behind the closed outer door",
        )
        ctx.expect_contact(
            cabinet,
            instrument_panel,
            elem_a="inner_hinge_rail",
            elem_b="panel_hinge_barrel_0",
            contact_tol=1e-5,
            name="inner panel hinge barrel is supported by the cabinet rail",
        )

    rest_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_right_stile")
    with ctx.pose({door_hinge: 1.2}):
        open_door_aabb = ctx.part_element_world_aabb(front_door, elem="door_right_stile")
    if rest_door_aabb is None or open_door_aabb is None:
        ctx.fail("front door open pose is measurable", "door_right_stile AABB was unavailable")
    else:
        rest_y = (rest_door_aabb[0][1] + rest_door_aabb[1][1]) / 2.0
        open_y = (open_door_aabb[0][1] + open_door_aabb[1][1]) / 2.0
        ctx.check(
            "front door swings outward from its left hinge",
            open_y < rest_y - 0.25,
            details=f"rest_y={rest_y:.3f}, open_y={open_y:.3f}",
        )

    rest_panel_aabb = ctx.part_element_world_aabb(instrument_panel, elem="panel_plate")
    with ctx.pose({door_hinge: 1.4, panel_hinge: 1.0}):
        open_panel_aabb = ctx.part_element_world_aabb(instrument_panel, elem="panel_plate")
    if rest_panel_aabb is None or open_panel_aabb is None:
        ctx.fail("inner panel open pose is measurable", "panel_plate AABB was unavailable")
    else:
        rest_panel_y = (rest_panel_aabb[0][1] + rest_panel_aabb[1][1]) / 2.0
        open_panel_y = (open_panel_aabb[0][1] + open_panel_aabb[1][1]) / 2.0
        ctx.check(
            "inner instrument panel swings outward for wiring access",
            open_panel_y < rest_panel_y - 0.18,
            details=f"rest_y={rest_panel_y:.3f}, open_y={open_panel_y:.3f}",
        )

    ctx.expect_contact(
        front_door,
        latch_handle,
        elem_a="door_right_stile",
        elem_b="handle_boss",
        contact_tol=1e-5,
        name="latch handle boss is seated on the door skin",
    )

    return ctx.report()


object_model = build_object_model()
