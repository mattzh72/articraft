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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_locking_mailbox")

    powder_black = Material("powder_coated_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_recess = Material("dark_slot_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    brushed_steel = Material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    brass = Material("warm_brass_lock", rgba=(0.95, 0.72, 0.28, 1.0))
    rubber = Material("black_rubber_gasket", rgba=(0.006, 0.006, 0.005, 1.0))

    box = model.part("box")

    # Fixed wall-mounted mailbox shell.  The front lower area is intentionally
    # left open; the articulated retrieval door covers that opening.
    box.visual(Box((0.40, 0.012, 0.46)), origin=Origin(xyz=(0.0, 0.086, 0.23)), material=powder_black, name="back_plate")
    box.visual(Box((0.40, 0.17, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=powder_black, name="bottom_wall")
    box.visual(Box((0.40, 0.17, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.440)), material=powder_black, name="top_wall")
    box.visual(Box((0.030, 0.17, 0.46)), origin=Origin(xyz=(-0.185, 0.0, 0.23)), material=powder_black, name="side_wall_0")
    box.visual(Box((0.030, 0.17, 0.46)), origin=Origin(xyz=(0.185, 0.0, 0.23)), material=powder_black, name="side_wall_1")

    # Front frame surrounding the lower retrieval opening and the fixed mail slot.
    box.visual(Box((0.40, 0.014, 0.035)), origin=Origin(xyz=(0.0, -0.086, 0.0475)), material=powder_black, name="lower_sill")
    box.visual(Box((0.035, 0.014, 0.290)), origin=Origin(xyz=(-0.1825, -0.086, 0.200)), material=powder_black, name="front_jamb_0")
    box.visual(Box((0.035, 0.014, 0.290)), origin=Origin(xyz=(0.1825, -0.086, 0.200)), material=powder_black, name="front_jamb_1")
    box.visual(Box((0.40, 0.014, 0.030)), origin=Origin(xyz=(0.0, -0.086, 0.335)), material=powder_black, name="door_header")

    # Mail slot above the retrieval door: a dark recessed aperture framed by the fixed box.
    box.visual(Box((0.040, 0.014, 0.075)), origin=Origin(xyz=(-0.180, -0.086, 0.382)), material=powder_black, name="slot_jamb_0")
    box.visual(Box((0.040, 0.014, 0.075)), origin=Origin(xyz=(0.180, -0.086, 0.382)), material=powder_black, name="slot_jamb_1")
    box.visual(Box((0.40, 0.014, 0.026)), origin=Origin(xyz=(0.0, -0.086, 0.431)), material=powder_black, name="slot_top_rail")
    box.visual(Box((0.40, 0.014, 0.020)), origin=Origin(xyz=(0.0, -0.086, 0.346)), material=powder_black, name="slot_bottom_rail")
    box.visual(Box((0.292, 0.006, 0.034)), origin=Origin(xyz=(0.0, -0.093, 0.385)), material=dark_recess, name="mail_slot_shadow")
    box.visual(Box((0.310, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.099, 0.405)), material=brushed_steel, name="slot_lip")

    # Wall-mount screw heads and a continuous hinge pin at the bottom.
    box.visual(Cylinder(radius=0.014, length=0.004), origin=Origin(xyz=(-0.125, -0.095, 0.415), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="mount_screw_0")
    box.visual(Cylinder(radius=0.014, length=0.004), origin=Origin(xyz=(0.125, -0.095, 0.415), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="mount_screw_1")
    box.visual(Box((0.36, 0.010, 0.026)), origin=Origin(xyz=(0.0, -0.088, 0.064)), material=powder_black, name="hinge_leaf")
    box.visual(Cylinder(radius=0.010, length=0.086), origin=Origin(xyz=(-0.124, -0.098, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)), material=powder_black, name="box_knuckle_0")
    box.visual(Cylinder(radius=0.010, length=0.086), origin=Origin(xyz=(0.124, -0.098, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)), material=powder_black, name="box_knuckle_1")
    box.visual(Cylinder(radius=0.004, length=0.360), origin=Origin(xyz=(0.0, -0.098, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed_steel, name="hinge_pin")

    door = model.part("door")
    # The door part frame is the lower horizontal hinge axis.  The panel extends
    # upward in local +Z in the closed pose; positive rotation around +X pulls it
    # down and outward.
    door.visual(Box((0.340, 0.018, 0.255)), origin=Origin(xyz=(0.0, -0.014, 0.1415)), material=powder_black, name="door_panel")
    door.visual(Box((0.318, 0.006, 0.215)), origin=Origin(xyz=(0.0, -0.0245, 0.145)), material=Material("subtle_door_inset", rgba=(0.035, 0.038, 0.040, 1.0)), name="recessed_face")
    door.visual(Box((0.320, 0.006, 0.010)), origin=Origin(xyz=(0.0, -0.026, 0.255)), material=rubber, name="upper_seal")
    door.visual(Box((0.150, 0.012, 0.026)), origin=Origin(xyz=(0.0, -0.005, 0.013)), material=powder_black, name="door_hinge_leaf")
    door.visual(Cylinder(radius=0.010, length=0.136), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=powder_black, name="door_knuckle")

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=box,
        child=door,
        origin=Origin(xyz=(0.0, -0.098, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=0.0, upper=1.75),
    )

    lock = model.part("lock_cylinder")
    # The lock rotates about the local door-normal axis.  A shaft passes through
    # the panel and carries the interior cam tab.
    lock.visual(Cylinder(radius=0.020, length=0.012), origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brass, name="lock_face")
    lock.visual(Cylinder(radius=0.009, length=0.048), origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="lock_shaft")
    lock.visual(Box((0.003, 0.014, 0.022)), origin=Origin(xyz=(0.0, -0.035, 0.0)), material=dark_recess, name="key_slot")
    lock.visual(Box((0.058, 0.008, 0.014)), origin=Origin(xyz=(0.028, 0.024, 0.0)), material=brushed_steel, name="cam_tab")

    model.articulation(
        "lock_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock,
        origin=Origin(xyz=(0.0, -0.014, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    # Keep a direct reference alive so linters do not treat the hinge setup as
    # unused when this file is inspected outside Articraft.
    door_hinge.meta["purpose"] = "pull_down_front_door"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    box = object_model.get_part("box")
    door = object_model.get_part("door")
    lock = object_model.get_part("lock_cylinder")
    door_hinge = object_model.get_articulation("door_hinge")
    lock_turn = object_model.get_articulation("lock_turn")

    ctx.allow_overlap(
        box,
        door,
        elem_a="hinge_pin",
        elem_b="door_knuckle",
        reason="The steel hinge pin is intentionally captured inside the lower door knuckle so the pull-down door remains clipped to the box.",
    )
    ctx.expect_within(
        box,
        door,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="door_knuckle",
        margin=0.001,
        name="hinge pin is centered inside door knuckle",
    )
    ctx.expect_overlap(
        box,
        door,
        axes="x",
        elem_a="hinge_pin",
        elem_b="door_knuckle",
        min_overlap=0.120,
        name="hinge pin retains the lower door knuckle",
    )

    ctx.allow_overlap(
        door,
        lock,
        elem_a="door_panel",
        elem_b="lock_shaft",
        reason="The lock shaft intentionally passes through the door panel as a keyed cam latch cylinder.",
    )
    ctx.expect_within(
        lock,
        door,
        axes="xz",
        inner_elem="lock_shaft",
        outer_elem="door_panel",
        margin=0.0,
        name="lock shaft is within the door panel face",
    )
    ctx.expect_overlap(
        lock,
        door,
        axes="y",
        elem_a="lock_shaft",
        elem_b="door_panel",
        min_overlap=0.010,
        name="lock shaft passes through the panel thickness",
    )
    ctx.allow_overlap(
        door,
        lock,
        elem_a="recessed_face",
        elem_b="lock_shaft",
        reason="The same lock shaft also passes through the thin decorative recessed face on the door.",
    )
    ctx.expect_overlap(
        lock,
        door,
        axes="y",
        elem_a="lock_shaft",
        elem_b="recessed_face",
        min_overlap=0.004,
        name="lock shaft passes through decorative face",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(box, door, axis="y", positive_elem="front_jamb_0", negative_elem="door_panel", min_gap=0.003, name="closed door sits proud of front frame")
        ctx.expect_overlap(door, box, axes="xz", elem_a="door_panel", elem_b="door_header", min_overlap=0.010, name="closed retrieval door reaches the header")

    rest_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.35}):
        open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_overlap(
            box,
            door,
            axes="x",
            elem_a="hinge_pin",
            elem_b="door_knuckle",
            min_overlap=0.120,
            name="opened door remains captured on hinge pin",
        )
    ctx.check(
        "door pulls down and outward",
        rest_panel is not None
        and open_panel is not None
        and open_panel[1][2] < rest_panel[1][2] - 0.035
        and open_panel[0][1] < rest_panel[0][1] - 0.060,
        details=f"closed_aabb={rest_panel}, opened_aabb={open_panel}",
    )

    with ctx.pose({lock_turn: math.pi / 2.0}):
        ctx.expect_overlap(lock, door, axes="y", elem_a="lock_shaft", elem_b="door_panel", min_overlap=0.010, name="turned lock remains seated in door")

    return ctx.report()


object_model = build_object_model()
