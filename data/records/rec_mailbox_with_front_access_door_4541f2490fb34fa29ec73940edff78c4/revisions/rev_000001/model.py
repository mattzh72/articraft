from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def make_post() -> cq.Workplane:
    # Vertical post 4x4 inch (0.1m x 0.1m)
    vert = cq.Workplane("XY").box(0.1, 0.1, 1.2, centered=(True, True, False))
    # Horizontal mounting board
    horiz = cq.Workplane("XY").workplane(offset=1.2).box(0.14, 0.45, 0.05, centered=(True, True, False))
    return vert.union(horiz)

def make_mailbox_body(width=0.16, side_height=0.14, depth=0.48, thickness=0.002) -> cq.Workplane:
    radius = width / 2
    body = (
        cq.Workplane("XZ")
        .moveTo(-radius, 0)
        .lineTo(radius, 0)
        .lineTo(radius, side_height)
        .threePointArc((0, side_height + radius), (-radius, side_height))
        .close()
        .extrude(-depth) # Extrude in +Y direction from Y=0 to Y=0.48
    )
    # Center along Y
    body = body.translate((0, -depth / 2, 0))
    # Shell to make it hollow, opening the front face (<Y)
    body = body.faces("<Y").shell(-thickness)
    return body

def make_mailbox_door(width=0.16, side_height=0.14, thickness=0.002) -> cq.Workplane:
    radius = width / 2
    # The door panel
    door = (
        cq.Workplane("XZ")
        .moveTo(-radius, 0)
        .lineTo(radius, 0)
        .lineTo(radius, side_height)
        .threePointArc((0, side_height + radius), (-radius, side_height))
        .close()
        .extrude(thickness) # Extrude in -Y direction from Y=0 to Y=-0.002
    )
    # The handle at the top center
    handle = (
        cq.Workplane("XZ")
        .workplane(offset=0)
        .center(0, side_height + radius / 2)
        .rect(0.04, 0.01)
        .extrude(0.02) # Extrude in -Y direction from Y=0 to Y=-0.02
    )
    return door.union(handle)

def make_flag() -> cq.Workplane:
    # A simple indicator flag
    rod = cq.Workplane("XY").circle(0.003).extrude(0.15)
    plate = (
        cq.Workplane("XZ")
        .workplane(offset=-0.001)
        .center(0.03, 0.13) # Shifted so it extends outwards (+X)
        .rect(0.06, 0.04)
        .extrude(0.002)
    )
    return rod.union(plate)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mailbox")

    # Dimensions
    mb_width = 0.16
    mb_side_height = 0.14
    mb_depth = 0.48
    mb_thickness = 0.002

    # Post
    post = model.part("post")
    post.visual(
        mesh_from_cadquery(make_post(), "post"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="post_mesh",
        color=(0.4, 0.3, 0.2, 1.0),
    )

    # Mailbox Body
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_mailbox_body(mb_width, mb_side_height, mb_depth, mb_thickness), "body"),
        origin=Origin(xyz=(0.0, 0.0, 1.25)), # Placed on top of the post horizontal board (Z=1.25)
        name="body_mesh",
        color=(0.2, 0.2, 0.25, 1.0),
    )
    
    # The body is rigidly mounted to the post
    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=Origin(),
    )

    # Door
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(make_mailbox_door(mb_width, mb_side_height, mb_thickness), "door"),
        # The door mesh sits exactly on the front plane of the body and extrudes outward (-Y)
        origin=Origin(),
        name="door_mesh",
        color=(0.2, 0.2, 0.25, 1.0),
    )
    
    # Hinge for the door
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -mb_depth / 2, 1.25)),
        axis=(1.0, 0.0, 0.0), # Rotate around X axis
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    # Flag
    flag = model.part("flag")
    flag.visual(
        mesh_from_cadquery(make_flag(), "flag"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)), # Placed via articulation origin below
        name="flag_mesh",
        color=(0.8, 0.1, 0.1, 1.0),
    )
    
    # Hinge for the flag
    # Placed on the right side of the body (+X).
    # X is set to 0.082 so the rod (radius 0.003) slightly overlaps the body side (0.080).
    model.articulation(
        "flag_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(mb_width / 2 + 0.002, -0.1, 1.25 + 0.05)),
        axis=(1.0, 0.0, 0.0), # Rotate around X axis
        motion_limits=MotionLimits(effort=0.5, velocity=1.0, lower=-1.57, upper=0.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "body",
        "flag",
        reason="The flag rod intentionally overlaps the mailbox body slightly to represent a mounted connection.",
    )
    
    door = object_model.get_part("door")
    body = object_model.get_part("body")
    flag = object_model.get_part("flag")
    door_hinge = object_model.get_articulation("door_hinge")
    flag_hinge = object_model.get_articulation("flag_hinge")
    
    # At rest, the door is closed against the body
    ctx.expect_gap(body, door, axis="y", max_penetration=0.001, name="door_closed_against_body")
    
    # Check door opening kinematics
    door_closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.57}):
        door_open_aabb = ctx.part_world_aabb(door)
        if door_closed_aabb and door_open_aabb:
            ctx.check(
                "door_opens_outward_and_downward",
                door_open_aabb[1][2] < door_closed_aabb[1][2] and door_open_aabb[0][1] < door_closed_aabb[0][1],
                "The door should rotate downward and extend further forward (-Y) when opened.",
            )
            
    # Check flag kinematics
    flag_up_aabb = ctx.part_world_aabb(flag)
    with ctx.pose({flag_hinge: -1.57}):
        flag_down_aabb = ctx.part_world_aabb(flag)
        if flag_up_aabb and flag_down_aabb:
            ctx.check(
                "flag_lowers_backward",
                flag_down_aabb[1][2] < flag_up_aabb[1][2] and flag_down_aabb[1][1] > flag_up_aabb[1][1],
                "The flag should rotate downward and point backward (+Y) when lowered.",
            )

    return ctx.report()

object_model = build_object_model()
