from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="european_slot_mailbox")

    powder_green = model.material("powder_coated_deep_green", rgba=(0.05, 0.18, 0.13, 1.0))
    dark_green = model.material("shadowed_green_edges", rgba=(0.025, 0.08, 0.06, 1.0))
    stainless = model.material("brushed_stainless_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    black = model.material("black_slot_shadow", rgba=(0.005, 0.005, 0.004, 1.0))
    brass = model.material("brass_lock_face", rgba=(0.82, 0.62, 0.25, 1.0))

    body_w = 0.400
    body_h = 0.280
    body_d = 0.120
    wall = 0.010
    front_y = -body_d / 2.0
    rear_y = body_d / 2.0
    hinge_x = -0.194
    hinge_y = front_y - 0.007
    center_z = body_h / 2.0

    body = model.part("body")
    body.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, rear_y - wall / 2.0, center_z)),
        material=dark_green,
        name="rear_panel",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall / 2.0, 0.0, center_z)),
        material=powder_green,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall / 2.0, 0.0, center_z)),
        material=powder_green,
        name="side_wall_1",
    )
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall / 2.0)),
        material=powder_green,
        name="top_wall",
    )
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=powder_green,
        name="bottom_wall",
    )
    body.visual(
        Box((0.340, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, front_y + 0.005, 0.010)),
        material=black,
        name="door_stop",
    )

    hinge_r = 0.006
    body_hinge_leaf_x = hinge_x - 0.010
    body.visual(
        Cylinder(radius=hinge_r, length=0.060),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.055)),
        material=stainless,
        name="side_hinge_barrel_0",
    )
    body.visual(
        Box((0.014, 0.006, 0.064)),
        origin=Origin(xyz=(body_hinge_leaf_x, front_y - 0.002, 0.055)),
        material=stainless,
        name="side_hinge_leaf_0",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=0.060),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.225)),
        material=stainless,
        name="side_hinge_barrel_1",
    )
    body.visual(
        Box((0.014, 0.006, 0.064)),
        origin=Origin(xyz=(body_hinge_leaf_x, front_y - 0.002, 0.225)),
        material=stainless,
        name="side_hinge_leaf_1",
    )

    door_w = 0.380
    door_h = 0.260
    door_t = 0.008
    door_left_clearance = 0.008
    slot_w = 0.245
    slot_h = 0.032
    slot_z = 0.050
    door_outer = rounded_rect_profile(door_w, door_h, 0.012, corner_segments=7)
    slot_hole = _shift_profile(
        rounded_rect_profile(slot_w, slot_h, 0.004, corner_segments=4),
        dy=slot_z,
    )
    door_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(door_outer, [slot_hole], door_t, center=True),
        "access_door_with_posting_slot",
    )

    access_door = model.part("access_door")
    access_door.visual(
        door_mesh,
        origin=Origin(
            xyz=(door_left_clearance + door_w / 2.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=powder_green,
        name="door_panel",
    )
    access_door.visual(
        Cylinder(radius=hinge_r, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="side_hinge_barrel",
    )
    access_door.visual(
        Box((0.022, 0.004, 0.102)),
        origin=Origin(xyz=(0.010, 0.000, 0.0)),
        material=stainless,
        name="side_hinge_leaf",
    )

    # The door carries the fixed clips for the slot lid hinge.  They sit just
    # outside the lid's width so the moving lid remains captured but uncollided.
    lid_w = 0.275
    lid_h = 0.060
    slot_lid_hinge_z = slot_z + lid_h / 2.0
    slot_lid_hinge_x = door_left_clearance + door_w / 2.0
    slot_lid_hinge_y = -0.012
    slot_clip_x_0 = slot_lid_hinge_x - lid_w / 2.0 - 0.018
    slot_clip_x_1 = slot_lid_hinge_x + lid_w / 2.0 + 0.018
    access_door.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(slot_clip_x_0, slot_lid_hinge_y, slot_lid_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="slot_hinge_clip_0",
    )
    access_door.visual(
        Box((0.028, 0.004, 0.018)),
        origin=Origin(xyz=(slot_clip_x_0, -0.006, slot_lid_hinge_z - 0.010)),
        material=stainless,
        name="slot_hinge_leaf_0",
    )
    access_door.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(slot_clip_x_1, slot_lid_hinge_y, slot_lid_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="slot_hinge_clip_1",
    )
    access_door.visual(
        Box((0.028, 0.004, 0.018)),
        origin=Origin(xyz=(slot_clip_x_1, -0.006, slot_lid_hinge_z - 0.010)),
        material=stainless,
        name="slot_hinge_leaf_1",
    )

    access_door.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.310, -0.007, -0.070), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="round_lock",
    )
    access_door.visual(
        Box((0.004, 0.002, 0.018)),
        origin=Origin(xyz=(0.310, -0.0105, -0.070)),
        material=black,
        name="key_slot",
    )

    slot_lid = model.part("slot_lid")
    slot_lid.visual(
        Box((lid_w, 0.005, lid_h)),
        origin=Origin(xyz=(0.0, 0.0, -lid_h / 2.0)),
        material=powder_green,
        name="lid_panel",
    )
    slot_lid.visual(
        Cylinder(radius=0.005, length=lid_w),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="lid_hinge_barrel",
    )
    slot_lid.visual(
        Box((0.220, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.000, -0.007)),
        material=stainless,
        name="lid_hinge_leaf",
    )

    model.articulation(
        "body_to_access_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_door,
        origin=Origin(xyz=(hinge_x, hinge_y, center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "access_door_to_slot_lid",
        ArticulationType.REVOLUTE,
        parent=access_door,
        child=slot_lid,
        origin=Origin(xyz=(slot_lid_hinge_x, slot_lid_hinge_y, slot_lid_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    access_door = object_model.get_part("access_door")
    slot_lid = object_model.get_part("slot_lid")
    door_joint = object_model.get_articulation("body_to_access_door")
    lid_joint = object_model.get_articulation("access_door_to_slot_lid")

    ctx.expect_gap(
        body,
        access_door,
        axis="y",
        positive_elem="side_wall_1",
        negative_elem="door_panel",
        min_gap=0.001,
        max_gap=0.008,
        name="closed access door stands proud of the shallow body",
    )
    ctx.expect_overlap(
        access_door,
        body,
        axes="xz",
        min_overlap=0.20,
        name="access door covers the mailbox front opening",
    )
    ctx.expect_gap(
        access_door,
        slot_lid,
        axis="y",
        positive_elem="door_panel",
        negative_elem="lid_panel",
        min_gap=0.001,
        max_gap=0.012,
        name="slot lid is clipped in front of the door without rubbing",
    )
    ctx.expect_overlap(
        slot_lid,
        access_door,
        axes="xz",
        elem_a="lid_panel",
        elem_b="door_panel",
        min_overlap=0.055,
        name="slot lid covers the posting slot area",
    )
    ctx.expect_contact(
        access_door,
        body,
        elem_a="side_hinge_barrel",
        elem_b="side_hinge_barrel_0",
        name="main door lower knuckle is retained on the side hinge",
    )
    ctx.expect_contact(
        access_door,
        body,
        elem_a="side_hinge_barrel",
        elem_b="side_hinge_barrel_1",
        name="main door upper knuckle is retained on the side hinge",
    )
    ctx.expect_contact(
        slot_lid,
        access_door,
        elem_a="lid_hinge_barrel",
        elem_b="slot_hinge_clip_0",
        name="slot lid barrel is clipped at one hinge end",
    )
    ctx.expect_contact(
        slot_lid,
        access_door,
        elem_a="lid_hinge_barrel",
        elem_b="slot_hinge_clip_1",
        name="slot lid barrel is clipped at the other hinge end",
    )

    closed_door_aabb = ctx.part_world_aabb(access_door)
    with ctx.pose({door_joint: 1.25}):
        open_door_aabb = ctx.part_world_aabb(access_door)
    ctx.check(
        "main access door opens outward on a vertical side hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.16,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_lid_aabb = ctx.part_world_aabb(slot_lid)
    with ctx.pose({lid_joint: 1.05}):
        raised_lid_aabb = ctx.part_world_aabb(slot_lid)
    ctx.check(
        "narrow slot lid lifts upward around its top hinge",
        closed_lid_aabb is not None
        and raised_lid_aabb is not None
        and raised_lid_aabb[0][2] > closed_lid_aabb[0][2] + 0.020
        and raised_lid_aabb[0][1] < closed_lid_aabb[0][1] - 0.020,
        details=f"closed={closed_lid_aabb}, raised={raised_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
