from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_mailbox")

    width = 0.2
    height_straight = 0.15
    radius = width / 2
    length = 0.5
    thickness = 0.005

    # --- Main Body ---
    outer = (
        cq.Workplane("XZ")
        .moveTo(-width / 2, 0)
        .lineTo(width / 2, 0)
        .lineTo(width / 2, height_straight)
        .threePointArc((0, height_straight + radius), (-width / 2, height_straight))
        .close()
        .extrude(length)
    )
    inner = (
        cq.Workplane("XZ", origin=(0, 0.01, 0))
        .moveTo(-width / 2 + thickness, thickness)
        .lineTo(width / 2 - thickness, thickness)
        .lineTo(width / 2 - thickness, height_straight)
        .threePointArc(
            (0, height_straight + radius - thickness),
            (-width / 2 + thickness, height_straight),
        )
        .close()
        .extrude(length - thickness + 0.01)
    )
    body_cq = outer.cut(inner)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_cq, "body_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="body_shell",
    )

    # --- Post ---
    post_height = 0.6
    post_width = 0.08
    post_depth = 0.08

    post = model.part("post")
    post.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(0, 0, post_height / 2)),
        name="post_geom",
    )

    # Mount body on post, centering the body's length over the post.
    # Body's local Y goes from -0.5 to 0. Center is at -0.25.
    # So we place the body at Y = +0.25 relative to the post.
    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=Origin(xyz=(0, length / 2, post_height)),
    )

    # --- Door ---
    door_thickness = 0.01
    door_cq = (
        cq.Workplane("XZ")
        .moveTo(-width / 2, 0)
        .lineTo(width / 2, 0)
        .lineTo(width / 2, height_straight)
        .threePointArc((0, height_straight + radius), (-width / 2, height_straight))
        .close()
        # Extrude in +Y so it sits outside the body
        .extrude(-door_thickness)
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(door_cq, "door_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="door_shell",
    )

    # --- Handle ---
    handle_width = 0.06
    handle_height = 0.015
    handle_depth = 0.02

    door.visual(
        Box((handle_width, handle_depth, handle_height)),
        origin=Origin(xyz=(0, door_thickness + handle_depth / 2, height_straight)),
        name="handle",
    )

    # --- Hinge ---
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    post = object_model.get_part("post")
    hinge = object_model.get_articulation("body_to_door")

    # At rest, door is closed and sits just in front of the body
    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_shell",
        negative_elem="body_shell",
        max_penetration=0.001,
        name="door sits in front of body",
    )

    # Post is under body
    ctx.expect_gap(
        body,
        post,
        axis="z",
        max_penetration=0.001,
        name="body sits on post",
    )

    # Check open pose
    with ctx.pose({hinge: 1.57}):
        # When open, the door top rotates forward (+Y) and down, so it should be well clear of the body
        door_aabb = ctx.part_world_aabb(door)
        body_aabb = ctx.part_world_aabb(body)
        if door_aabb is not None and body_aabb is not None:
            ctx.check(
                "door opens outward",
                door_aabb[1][1] > body_aabb[1][1] + 0.1,
                "Door max Y should move significantly outward (+Y) when open.",
            )

    return ctx.report()


object_model = build_object_model()
