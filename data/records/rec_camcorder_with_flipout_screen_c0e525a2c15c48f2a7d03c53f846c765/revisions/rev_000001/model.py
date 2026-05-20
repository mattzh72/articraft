from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.086
BODY_WIDTH = 0.046
BODY_HEIGHT = 0.054
BODY_HALF_LENGTH = BODY_LENGTH * 0.5
BODY_HALF_WIDTH = BODY_WIDTH * 0.5

SCREEN_LENGTH = 0.062
SCREEN_HEIGHT = 0.040
SCREEN_THICKNESS = 0.0048

HINGE_RADIUS = 0.0014
HINGE_X = -0.030
HINGE_Y = BODY_HALF_WIDTH + HINGE_RADIUS

BUTTON_WIDTH = 0.0064
BUTTON_DEPTH = 0.0024
BUTTON_HEIGHT = 0.0058
BUTTON_TRAVEL = 0.0020


def _body_shell_mesh():
    body = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
    body = body.edges("|Z").fillet(0.0055)

    rear_bulge = cq.Workplane("XY").box(0.018, 0.038, 0.038).translate((-0.034, 0.000, 0.001))
    top_hump = cq.Workplane("XY").box(0.030, 0.020, 0.008).translate((-0.008, 0.000, 0.031))
    nose_cheek = cq.Workplane("XY").box(0.012, 0.032, 0.034).translate((0.040, 0.000, -0.003))
    eyecup = cq.Workplane("XY").box(0.010, 0.022, 0.018).translate((-0.046, 0.000, 0.012))

    body = body.union(rear_bulge).union(top_hump).union(nose_cheek).union(eyecup)

    screen_recess = cq.Workplane("XY").box(0.060, 0.0034, 0.040).translate((0.004, 0.0213, 0.004))
    body = body.cut(screen_recess)

    for x_pos in (-0.004, 0.008, 0.020):
        button_pocket = (
            cq.Workplane("XY")
            .box(0.0074, 0.0052, 0.0068)
            .translate((x_pos, BODY_HALF_WIDTH - 0.0026, -0.015))
        )
        body = body.cut(button_pocket)

    return mesh_from_cadquery(body, "camcorder_body_shell")


def _lens_ring_mesh():
    ring = cq.Workplane("YZ").circle(0.0178).circle(0.0152).extrude(0.011)
    for x_pos in (0.0015, 0.0048, 0.0081):
        rib = (
            cq.Workplane("YZ")
            .workplane(offset=x_pos)
            .circle(0.0184)
            .circle(0.0178)
            .extrude(0.0008)
        )
        ring = ring.union(rib)
    return mesh_from_cadquery(ring, "camcorder_lens_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_video_camcorder")

    body_mat = model.material("body_mat", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.25, 0.26, 0.28, 1.0))
    glass_mat = model.material("glass_mat", rgba=(0.16, 0.24, 0.30, 0.48))
    lens_mat = model.material("lens_mat", rgba=(0.08, 0.08, 0.09, 1.0))
    button_mat = model.material("button_mat", rgba=(0.58, 0.60, 0.63, 1.0))
    strap_mat = model.material("strap_mat", rgba=(0.18, 0.18, 0.19, 1.0))
    strap_pad_mat = model.material("strap_pad_mat", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_mat, name="body_shell")
    body.visual(
        Cylinder(radius=0.0146, length=0.0260),
        origin=Origin(xyz=(0.056, 0.000, 0.000), rpy=(0.000, 1.57079632679, 0.000)),
        material=trim_mat,
        name="lens_housing",
    )
    body.visual(
        Cylinder(radius=0.0118, length=0.0220),
        origin=Origin(xyz=(0.058, 0.000, 0.000), rpy=(0.000, 1.57079632679, 0.000)),
        material=lens_mat,
        name="lens_core",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.0018),
        origin=Origin(xyz=(0.0699, 0.000, 0.000), rpy=(0.000, 1.57079632679, 0.000)),
        material=glass_mat,
        name="front_glass",
    )
    body.visual(
        Cylinder(radius=0.0178, length=0.0010),
        origin=Origin(xyz=(0.0525, 0.000, 0.000), rpy=(0.000, 1.57079632679, 0.000)),
        material=trim_mat,
        name="lens_mount_face",
    )
    body.visual(
        Box((0.012, 0.010, 0.006)),
        origin=Origin(xyz=(0.006, 0.000, 0.030)),
        material=trim_mat,
        name="top_mic",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.011),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, -0.0135)),
        material=trim_mat,
        name="hinge_barrel_lower",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.011),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0135)),
        material=trim_mat,
        name="hinge_barrel_upper",
    )
    body.visual(
        Box((0.0045, HINGE_RADIUS * 2.0, 0.011)),
        origin=Origin(xyz=(HINGE_X + 0.0022, BODY_HALF_WIDTH + HINGE_RADIUS, -0.0135)),
        material=trim_mat,
        name="hinge_leaf_lower",
    )
    body.visual(
        Box((0.0045, HINGE_RADIUS * 2.0, 0.011)),
        origin=Origin(xyz=(HINGE_X + 0.0022, BODY_HALF_WIDTH + HINGE_RADIUS, 0.0135)),
        material=trim_mat,
        name="hinge_leaf_upper",
    )
    for index, x_pos in enumerate((-0.004, 0.008, 0.020)):
        body.visual(
            Box((0.0048, 0.0040, 0.0040)),
            origin=Origin(xyz=(x_pos, BODY_HALF_WIDTH - 0.0034, -0.015)),
            material=trim_mat,
            name=f"button_guide_{index}",
        )

    hand_strap = model.part("hand_strap")
    hand_strap.visual(
        Box((0.010, 0.002, 0.022)),
        origin=Origin(xyz=(-0.025, -0.001, 0.000)),
        material=strap_pad_mat,
        name="front_anchor",
    )
    hand_strap.visual(
        Box((0.010, 0.002, 0.022)),
        origin=Origin(xyz=(0.022, -0.001, 0.000)),
        material=strap_pad_mat,
        name="rear_anchor",
    )
    hand_strap.visual(
        Box((0.052, 0.002, 0.020)),
        origin=Origin(xyz=(-0.0015, -0.003, 0.000)),
        material=strap_mat,
        name="strap_band",
    )
    model.articulation(
        "body_to_hand_strap",
        ArticulationType.FIXED,
        parent=body,
        child=hand_strap,
        origin=Origin(xyz=(0.000, -BODY_HALF_WIDTH, 0.000)),
    )

    screen = model.part("screen")
    screen.visual(
        Cylinder(radius=0.0012, length=0.014),
        origin=Origin(),
        material=trim_mat,
        name="hinge_barrel",
    )
    screen.visual(
        Box((0.005, 0.0014, 0.030)),
        origin=Origin(xyz=(0.0025, 0.0007, 0.000)),
        material=trim_mat,
        name="hinge_leaf",
    )
    screen.visual(
        Box((SCREEN_LENGTH, SCREEN_THICKNESS, SCREEN_HEIGHT)),
        origin=Origin(xyz=(0.036, 0.0038, 0.000)),
        material=trim_mat,
        name="screen_panel",
    )
    screen.visual(
        Box((0.053, 0.0008, 0.032)),
        origin=Origin(xyz=(0.036, 0.0066, 0.000)),
        material=glass_mat,
        name="display",
    )
    model.articulation(
        "body_to_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=2.35,
        ),
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(_lens_ring_mesh(), material=lens_mat, name="ring_shell")
    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.053, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=8.0,
        ),
    )

    for index, x_pos in enumerate((-0.004, 0.008, 0.020)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.000, BUTTON_DEPTH * 0.5, 0.000)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.0042, 0.0030, 0.0036)),
            origin=Origin(xyz=(0.000, -0.0013, 0.000)),
            material=button_mat,
            name="button_stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_HALF_WIDTH, -0.015)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    screen = object_model.get_part("screen")
    hand_strap = object_model.get_part("hand_strap")
    lens_ring = object_model.get_part("lens_ring")

    screen_hinge = object_model.get_articulation("body_to_screen")
    lens_joint = object_model.get_articulation("body_to_lens_ring")

    ctx.expect_overlap(
        screen,
        body,
        axes="xz",
        elem_a="screen_panel",
        elem_b="body_shell",
        min_overlap=0.030,
        name="screen covers the side opening when closed",
    )
    ctx.expect_gap(
        screen,
        body,
        axis="y",
        positive_elem="screen_panel",
        negative_elem="body_shell",
        min_gap=0.001,
        max_gap=0.010,
        name="screen sits just outside the body sidewall",
    )

    closed_screen_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
    screen_limits = screen_hinge.motion_limits
    if screen_limits is not None and screen_limits.upper is not None:
        with ctx.pose({screen_hinge: screen_limits.upper}):
            open_screen_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
            ctx.check(
                "screen swings outward away from the body",
                closed_screen_aabb is not None
                and open_screen_aabb is not None
                and open_screen_aabb[1][1] > closed_screen_aabb[1][1] + 0.025,
                details=f"closed={closed_screen_aabb}, open={open_screen_aabb}",
            )

    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="ring_shell",
        elem_b="lens_housing",
        min_overlap=0.009,
        name="lens ring remains seated on the front lens housing",
    )
    ctx.expect_origin_distance(
        lens_ring,
        body,
        axes="yz",
        max_dist=0.0005,
        name="lens ring shares the body optical axis",
    )
    with ctx.pose({lens_joint: 1.7}):
        ctx.expect_overlap(
            lens_ring,
            body,
            axes="x",
            elem_a="ring_shell",
            elem_b="lens_housing",
            min_overlap=0.009,
            name="lens ring stays seated after rotation",
        )

    ctx.expect_contact(
        hand_strap,
        body,
        elem_a="front_anchor",
        elem_b="body_shell",
        contact_tol=0.0005,
        name="front strap anchor mounts to the body",
    )
    ctx.expect_contact(
        hand_strap,
        body,
        elem_a="rear_anchor",
        elem_b="body_shell",
        contact_tol=0.0005,
        name="rear strap anchor mounts to the body",
    )

    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]
    for index, button in enumerate(button_parts):
        ctx.allow_overlap(
            body,
            button,
            elem_a=f"button_guide_{index}",
            elem_b="button_stem",
            reason="The button stem is intentionally represented as sliding inside a simplified guide block in the sidewall.",
        )
    rest_positions = {part.name: ctx.part_world_position(part) for part in button_parts}

    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} depresses inward",
                rest_positions[button.name] is not None
                and pressed_position is not None
                and pressed_position[1] < rest_positions[button.name][1] - 0.0015,
                details=f"rest={rest_positions[button.name]}, pressed={pressed_position}",
            )
            for other in button_parts:
                if other.name == button.name:
                    continue
                other_pressed_position = ctx.part_world_position(other)
                ctx.check(
                    f"{button.name} moves independently of {other.name}",
                    rest_positions[other.name] is not None
                    and other_pressed_position is not None
                    and abs(other_pressed_position[1] - rest_positions[other.name][1]) < 1e-7,
                    details=f"rest={rest_positions[other.name]}, posed={other_pressed_position}",
                )

    return ctx.report()


object_model = build_object_model()
