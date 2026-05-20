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
    model = ArticulatedObject(name="security_mailbox")

    body_mat = Material("powder_coated_dark_gray", rgba=(0.09, 0.10, 0.11, 1.0))
    door_mat = Material("slightly_lighter_door", rgba=(0.13, 0.14, 0.15, 1.0))
    post_mat = Material("black_post", rgba=(0.03, 0.03, 0.035, 1.0))
    steel_mat = Material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    shadow_mat = Material("black_shadow", rgba=(0.005, 0.005, 0.006, 1.0))

    # Intrinsic frame: +Z is up, the mailbox front faces -X, and Y spans the width.
    front_x = -0.225
    rear_x = 0.175
    body_z = 1.10
    body_h = 0.64
    body_w = 0.46
    body_d = rear_x - front_x
    wall = 0.020

    root = model.part("post_body")

    # Ground post and mounting saddle: the mailbox reads as a fixed post-mounted unit.
    root.visual(
        Box((0.080, 0.080, 0.76)),
        origin=Origin(xyz=(0.075, 0.0, 0.38)),
        material=post_mat,
        name="square_post",
    )
    root.visual(
        Box((0.28, 0.30, 0.030)),
        origin=Origin(xyz=(0.055, 0.0, 0.775)),
        material=post_mat,
        name="mounting_saddle",
    )
    root.visual(
        Box((0.24, 0.050, 0.18)),
        origin=Origin(xyz=(0.050, 0.0, 0.86)),
        material=post_mat,
        name="rear_support_web",
    )

    # Hollow rectangular mailbox shell, built from connected walls and front frame rails.
    root.visual(
        Box((body_d, wall, body_h)),
        origin=Origin(xyz=((front_x + rear_x) * 0.5, -body_w * 0.5 + wall * 0.5, body_z)),
        material=body_mat,
        name="side_wall_0",
    )
    root.visual(
        Box((body_d, wall, body_h)),
        origin=Origin(xyz=((front_x + rear_x) * 0.5, body_w * 0.5 - wall * 0.5, body_z)),
        material=body_mat,
        name="side_wall_1",
    )
    root.visual(
        Box((body_d, body_w, wall)),
        origin=Origin(xyz=((front_x + rear_x) * 0.5, 0.0, body_z - body_h * 0.5 + wall * 0.5)),
        material=body_mat,
        name="bottom_wall",
    )
    root.visual(
        Box((body_d, body_w, wall)),
        origin=Origin(xyz=((front_x + rear_x) * 0.5, 0.0, body_z + body_h * 0.5 - wall * 0.5)),
        material=body_mat,
        name="top_wall",
    )
    root.visual(
        Box((wall, body_w, body_h)),
        origin=Origin(xyz=(rear_x - wall * 0.5, 0.0, body_z)),
        material=body_mat,
        name="rear_wall",
    )

    frame_x = front_x - 0.002
    root.visual(
        Box((0.024, 0.018, 0.60)),
        origin=Origin(xyz=(frame_x, -body_w * 0.5 + 0.009, body_z)),
        material=body_mat,
        name="front_side_rail_0",
    )
    root.visual(
        Box((0.024, 0.018, 0.60)),
        origin=Origin(xyz=(frame_x, body_w * 0.5 - 0.009, body_z)),
        material=body_mat,
        name="front_side_rail_1",
    )
    root.visual(
        Box((0.024, 0.405, 0.030)),
        origin=Origin(xyz=(frame_x, 0.0, 0.795)),
        material=body_mat,
        name="front_sill",
    )
    root.visual(
        Box((0.024, 0.405, 0.030)),
        origin=Origin(xyz=(frame_x, 0.0, 1.135)),
        material=body_mat,
        name="door_slot_rail",
    )
    root.visual(
        Box((0.024, 0.405, 0.042)),
        origin=Origin(xyz=(frame_x, 0.0, 1.306)),
        material=body_mat,
        name="upper_front_rail",
    )
    root.visual(
        Box((0.010, 0.420, 0.128)),
        origin=Origin(xyz=(-0.150, 0.0, 1.220)),
        material=shadow_mat,
        name="mail_slot_shadow",
    )

    # Exposed vertical hinge pin for the retrieval door, supported at the top and bottom.
    door_hinge_x = front_x - 0.022
    door_hinge_y = -0.195
    door_center_z = 0.965
    root.visual(
        Cylinder(radius=0.006, length=0.360),
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        material=steel_mat,
        name="door_hinge_pin",
    )
    for i, z in enumerate((0.785, 1.145)):
        root.visual(
            Box((0.030, 0.046, 0.012)),
            origin=Origin(xyz=(door_hinge_x + 0.010, door_hinge_y - 0.010, z)),
            material=steel_mat,
            name=f"door_hinge_saddle_{i}",
        )

    # Upper horizontal hinge pin for the anti-fishing slot cover. End brackets keep it clipped on.
    slot_hinge_z = 1.285
    root.visual(
        Cylinder(radius=0.006, length=0.405),
        origin=Origin(xyz=(door_hinge_x, 0.0, slot_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="slot_hinge_pin",
    )
    for i, y in enumerate((-0.195, 0.195)):
        root.visual(
            Box((0.030, 0.030, 0.034)),
            origin=Origin(xyz=(door_hinge_x + 0.010, y, slot_hinge_z)),
            material=steel_mat,
            name=f"slot_hinge_clip_{i}",
        )

    # Lower retrieval door. Its frame is the hinge line, so the panel extends in +Y.
    door = model.part("retrieval_door")
    door.visual(
        Box((0.018, 0.360, 0.310)),
        origin=Origin(xyz=(0.0, 0.190, 0.0)),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_mat,
        name="door_hinge_knuckle",
    )
    door.visual(
        Box((0.006, 0.330, 0.018)),
        origin=Origin(xyz=(-0.012, 0.190, 0.135)),
        material=body_mat,
        name="door_top_rib",
    )
    door.visual(
        Box((0.006, 0.330, 0.018)),
        origin=Origin(xyz=(-0.012, 0.190, -0.135)),
        material=body_mat,
        name="door_bottom_rib",
    )
    door.visual(
        Box((0.006, 0.018, 0.280)),
        origin=Origin(xyz=(-0.012, 0.350, 0.0)),
        material=body_mat,
        name="door_free_rib",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=root,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    # Central cam lock on its own axis normal to the retrieval door.
    lock = model.part("cam_lock")
    lock.visual(
        Cylinder(radius=0.017, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="lock_plug",
    )
    lock.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="lock_cap",
    )
    lock.visual(
        Box((0.003, 0.032, 0.006)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        material=shadow_mat,
        name="key_slot",
    )
    lock.visual(
        Box((0.008, 0.075, 0.018)),
        origin=Origin(xyz=(0.022, 0.038, 0.0)),
        material=steel_mat,
        name="cam_tail",
    )
    model.articulation(
        "door_to_lock",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lock,
        origin=Origin(xyz=(-0.001, 0.190, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )

    # Small anti-fishing slot cover. Its frame is the top hinge line, so the cover hangs downward.
    cover = model.part("slot_cover")
    cover.visual(
        Box((0.018, 0.340, 0.092)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=door_mat,
        name="cover_panel",
    )
    cover.visual(
        Cylinder(radius=0.012, length=0.345),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="cover_hinge_knuckle",
    )
    for i, y in enumerate((-0.120, 0.120)):
        cover.visual(
            Box((0.012, 0.026, 0.060)),
            origin=Origin(xyz=(-0.004, y, -0.039)),
            material=steel_mat,
            name=f"cover_clip_strap_{i}",
        )
    cover.visual(
        Box((0.030, 0.315, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, -0.096)),
        material=door_mat,
        name="anti_fishing_lip",
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=root,
        child=cover,
        origin=Origin(xyz=(door_hinge_x, 0.0, slot_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("post_body")
    door = object_model.get_part("retrieval_door")
    lock = object_model.get_part("cam_lock")
    cover = object_model.get_part("slot_cover")
    door_hinge = object_model.get_articulation("body_to_door")
    lock_turn = object_model.get_articulation("door_to_lock")
    cover_hinge = object_model.get_articulation("body_to_cover")

    ctx.allow_overlap(
        body,
        door,
        elem_a="door_hinge_pin",
        elem_b="door_hinge_knuckle",
        reason="The retrieval door knuckle is intentionally captured around the fixed vertical hinge pin.",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="z",
        elem_a="door_hinge_pin",
        elem_b="door_hinge_knuckle",
        min_overlap=0.29,
        name="retrieval door hinge remains pinned",
    )

    ctx.allow_overlap(
        body,
        cover,
        elem_a="slot_hinge_pin",
        elem_b="cover_hinge_knuckle",
        reason="The slot cover knuckle clips around the upper hinge pin so the small lid stays attached.",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="y",
        elem_a="slot_hinge_pin",
        elem_b="cover_hinge_knuckle",
        min_overlap=0.33,
        name="slot cover clip spans the upper hinge pin",
    )

    ctx.allow_overlap(
        lock,
        door,
        elem_a="lock_plug",
        elem_b="door_panel",
        reason="The cam lock plug passes through the lock hole in the retrieval door.",
    )
    ctx.expect_within(
        lock,
        door,
        axes="yz",
        inner_elem="lock_plug",
        outer_elem="door_panel",
        margin=0.0,
        name="cam lock plug is centered in the door panel",
    )
    ctx.expect_overlap(
        lock,
        door,
        axes="x",
        elem_a="lock_plug",
        elem_b="door_panel",
        min_overlap=0.016,
        name="cam lock plug passes through the door thickness",
    )

    rest_free_edge = ctx.part_element_world_aabb(door, elem="door_free_rib")
    with ctx.pose({door_hinge: 1.20}):
        open_free_edge = ctx.part_element_world_aabb(door, elem="door_free_rib")
    ctx.check(
        "retrieval door swings outward on side hinge",
        rest_free_edge is not None
        and open_free_edge is not None
        and open_free_edge[0][0] < rest_free_edge[0][0] - 0.12,
        details=f"rest={rest_free_edge}, open={open_free_edge}",
    )

    rest_lip = ctx.part_element_world_aabb(cover, elem="anti_fishing_lip")
    with ctx.pose({cover_hinge: 1.0}):
        open_lip = ctx.part_element_world_aabb(cover, elem="anti_fishing_lip")
        ctx.expect_overlap(
            body,
            cover,
            axes="y",
            elem_a="slot_hinge_pin",
            elem_b="cover_hinge_knuckle",
            min_overlap=0.33,
            name="slot cover remains clipped while open",
        )
    ctx.check(
        "slot cover lifts upward and outward",
        rest_lip is not None
        and open_lip is not None
        and open_lip[0][0] < rest_lip[0][0] - 0.04
        and open_lip[0][2] > rest_lip[0][2] + 0.02,
        details=f"rest={rest_lip}, open={open_lip}",
    )

    with ctx.pose({lock_turn: math.pi / 2.0}):
        turned_tail = ctx.part_element_world_aabb(lock, elem="cam_tail")
    ctx.check(
        "cam lock rotates its latch tail",
        turned_tail is not None and (turned_tail[1][2] - turned_tail[0][2]) > 0.055,
        details=f"turned_tail={turned_tail}",
    )

    return ctx.report()


object_model = build_object_model()
