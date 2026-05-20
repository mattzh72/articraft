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
    model = ArticulatedObject(name="pedestal_parcel_mailbox")

    powder_coat = model.material("dark_olive_powdercoat", rgba=(0.10, 0.18, 0.14, 1.0))
    darker_metal = model.material("black_post_metal", rgba=(0.025, 0.028, 0.026, 1.0))
    hinge_metal = model.material("brushed_steel_hinge", rgba=(0.65, 0.66, 0.62, 1.0))
    lock_metal = model.material("satin_lock_metal", rgba=(0.74, 0.70, 0.58, 1.0))
    shadow = model.material("black_recess", rgba=(0.005, 0.005, 0.004, 1.0))

    # Fixed welded assembly: pedestal, post, and hollow rectangular parcel box.
    body = model.part("body")
    body.visual(Box((0.55, 0.42, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=darker_metal, name="ground_plate")
    body.visual(Box((0.16, 0.13, 0.86)), origin=Origin(xyz=(0.0, 0.0, 0.490)), material=darker_metal, name="post")
    body.visual(Box((0.32, 0.24, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.920)), material=darker_metal, name="post_cap")

    # Body outer envelope: 0.65 m wide, 0.42 m deep, and 0.55 m high.
    # It is built from panels so the front reads as an actual retrieval opening,
    # not a solid block hidden behind the door.
    body.visual(Box((0.035, 0.42, 0.55)), origin=Origin(xyz=(-0.3075, 0.0, 1.225)), material=powder_coat, name="side_wall_0")
    body.visual(Box((0.035, 0.42, 0.55)), origin=Origin(xyz=(0.3075, 0.0, 1.225)), material=powder_coat, name="side_wall_1")
    body.visual(Box((0.65, 0.42, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.9675)), material=powder_coat, name="bottom_panel")
    body.visual(Box((0.65, 0.42, 0.035)), origin=Origin(xyz=(0.0, 0.0, 1.4825)), material=powder_coat, name="top_panel")
    body.visual(Box((0.65, 0.035, 0.55)), origin=Origin(xyz=(0.0, 0.1925, 1.225)), material=powder_coat, name="back_panel")

    # Proud front return frame around the opening.
    body.visual(Box((0.65, 0.030, 0.040)), origin=Origin(xyz=(0.0, -0.225, 1.465)), material=powder_coat, name="front_frame_top")
    body.visual(Box((0.65, 0.030, 0.040)), origin=Origin(xyz=(0.0, -0.225, 0.985)), material=powder_coat, name="front_frame_bottom")
    body.visual(Box((0.040, 0.030, 0.51)), origin=Origin(xyz=(-0.305, -0.225, 1.225)), material=powder_coat, name="front_frame_side_0")
    body.visual(Box((0.040, 0.030, 0.51)), origin=Origin(xyz=(0.305, -0.225, 1.225)), material=powder_coat, name="front_frame_side_1")

    # The fixed middle hinge knuckle is outside the left front corner, tied back
    # to the box by a short leaf plate.
    hinge_x = -0.350
    hinge_y = -0.258
    hinge_z = 1.225
    body.visual(Box((0.036, 0.018, 0.110)), origin=Origin(xyz=(-0.337, -0.235, hinge_z)), material=hinge_metal, name="body_hinge_leaf")
    body.visual(Cylinder(radius=0.022, length=0.080), origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)), material=hinge_metal, name="body_hinge_barrel")

    # Retrieval door part.  Its frame origin lies on the vertical hinge axis so
    # the broad panel extends in +X and opens outward toward -Y.
    door = model.part("retrieval_door")
    door.visual(Box((0.630, 0.028, 0.475)), origin=Origin(xyz=(0.350, -0.002, 0.0)), material=powder_coat, name="door_panel")
    door.visual(Box((0.585, 0.010, 0.390)), origin=Origin(xyz=(0.350, -0.020, 0.0)), material=shadow, name="recessed_panel")
    door.visual(Box((0.610, 0.014, 0.030)), origin=Origin(xyz=(0.350, -0.030, 0.220)), material=powder_coat, name="door_raised_top")
    door.visual(Box((0.610, 0.014, 0.030)), origin=Origin(xyz=(0.350, -0.030, -0.220)), material=powder_coat, name="door_raised_bottom")
    door.visual(Box((0.030, 0.014, 0.435)), origin=Origin(xyz=(0.055, -0.030, 0.0)), material=powder_coat, name="door_raised_hinge_side")
    door.visual(Box((0.030, 0.014, 0.435)), origin=Origin(xyz=(0.645, -0.030, 0.0)), material=powder_coat, name="door_raised_latch_side")

    # Alternating hinge knuckles carried by the door.  The leaves only exist at
    # the door knuckles, leaving clearance for the fixed middle knuckle.
    for suffix, zc in (("lower", -0.145), ("upper", 0.145)):
        door.visual(Box((0.058, 0.014, 0.190)), origin=Origin(xyz=(0.018, -0.017, zc)), material=hinge_metal, name=f"door_hinge_leaf_{suffix}")
        door.visual(Cylinder(radius=0.022, length=0.190), origin=Origin(xyz=(0.0, 0.0, zc)), material=hinge_metal, name=f"door_hinge_barrel_{suffix}")

    # Raised lock housing near the latch edge.  It is fixed to the door; the
    # separate key cylinder below rotates inside its local axis.
    lock_x = 0.560
    lock_z = 0.055
    door.visual(Box((0.105, 0.020, 0.080)), origin=Origin(xyz=(lock_x, -0.026, lock_z)), material=lock_metal, name="lock_housing")
    door.visual(Box((0.135, 0.012, 0.045)), origin=Origin(xyz=(0.630, -0.022, -0.115)), material=powder_coat, name="pull_lip")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    key = model.part("key_cylinder")
    key.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lock_metal,
        name="lock_core",
    )
    key.visual(Box((0.032, 0.002, 0.006)), origin=Origin(xyz=(0.0, -0.027, 0.0)), material=shadow, name="key_slot")

    model.articulation(
        "key_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=key,
        origin=Origin(xyz=(lock_x, -0.036, lock_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("retrieval_door")
    key = object_model.get_part("key_cylinder")
    door_hinge = object_model.get_articulation("door_hinge")
    key_turn = object_model.get_articulation("key_turn")

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.40,
        elem_a="door_panel",
        name="retrieval door spans the front opening width",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_frame_top",
        negative_elem="door_panel",
        min_gap=0.002,
        max_gap=0.012,
        name="closed retrieval door sits just proud of the front frame",
    )
    ctx.expect_gap(
        door,
        key,
        axis="y",
        positive_elem="lock_housing",
        negative_elem="lock_core",
        max_gap=0.001,
        max_penetration=0.000001,
        name="key cylinder is seated against the lock housing",
    )

    rest_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door hinge opens outward from the box front",
        rest_panel is not None
        and open_panel is not None
        and open_panel[0][1] < rest_panel[0][1] - 0.20,
        details=f"closed={rest_panel}, open={open_panel}",
    )

    rest_slot = ctx.part_element_world_aabb(key, elem="key_slot")
    with ctx.pose({key_turn: math.pi / 2.0}):
        turned_slot = ctx.part_element_world_aabb(key, elem="key_slot")
    if rest_slot is not None and turned_slot is not None:
        rest_dx = rest_slot[1][0] - rest_slot[0][0]
        rest_dz = rest_slot[1][2] - rest_slot[0][2]
        turned_dx = turned_slot[1][0] - turned_slot[0][0]
        turned_dz = turned_slot[1][2] - turned_slot[0][2]
        slot_rotates = rest_dx > rest_dz * 2.0 and turned_dz > turned_dx * 2.0
    else:
        slot_rotates = False
    ctx.check(
        "key cylinder rotates the visible key slot",
        slot_rotates,
        details=f"rest={rest_slot}, turned={turned_slot}",
    )

    return ctx.report()


object_model = build_object_model()
