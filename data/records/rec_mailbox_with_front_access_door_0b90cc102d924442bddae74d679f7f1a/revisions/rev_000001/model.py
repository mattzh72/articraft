from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="narrow_postbox")

    post_oak = model.material("post_oak", rgba=(0.44, 0.30, 0.18, 1.0))
    box_red = model.material("box_red", rgba=(0.55, 0.08, 0.08, 1.0))
    box_red_dark = model.material("box_red_dark", rgba=(0.38, 0.05, 0.05, 1.0))
    aged_black = model.material("aged_black", rgba=(0.15, 0.14, 0.14, 1.0))
    brass = model.material("brass", rgba=(0.67, 0.55, 0.28, 1.0))

    post_height = 1.20
    post_size = 0.09

    body_width = 0.24
    body_depth = 0.18
    body_height = 0.56
    wall = 0.008
    front_lip_depth = 0.012
    cap_thickness = 0.012

    body_center_z = 0.92
    body_center_y = 0.135

    inner_width = body_width - (2.0 * wall)

    slot_sill_height = 0.018
    slot_sill_center_z = 0.125
    top_band_height = 0.110
    top_band_center_z = (body_height / 2.0) - wall - (top_band_height / 2.0)

    door_width = 0.218
    door_height = 0.386
    door_thickness = 0.014
    door_center_z = -0.075

    hood_width = 0.204

    post = model.part("post")
    post.visual(
        Box((post_size, post_size, post_height)),
        origin=Origin(xyz=(0.0, 0.0, post_height / 2.0)),
        material=post_oak,
        name="post_shaft",
    )

    body = model.part("body")
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-(body_width / 2.0) + (wall / 2.0), 0.0, 0.0)),
        material=box_red,
        name="left_wall",
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=((body_width / 2.0) - (wall / 2.0), 0.0, 0.0)),
        material=box_red,
        name="right_wall",
    )
    body.visual(
        Box((body_width - (2.0 * wall), wall, body_height - (2.0 * wall))),
        origin=Origin(xyz=(0.0, -(body_depth / 2.0) + (wall / 2.0), 0.0)),
        material=box_red,
        name="body_back_wall",
    )
    body.visual(
        Box((body_width - (2.0 * wall), body_depth - wall, wall)),
        origin=Origin(
            xyz=(0.0, wall / 2.0, (body_height / 2.0) - (wall / 2.0))
        ),
        material=box_red,
        name="top_wall",
    )
    body.visual(
        Box((body_width - (2.0 * wall), body_depth - wall, wall)),
        origin=Origin(
            xyz=(0.0, wall / 2.0, -(body_height / 2.0) + (wall / 2.0))
        ),
        material=box_red,
        name="bottom_wall",
    )
    body.visual(
        Box((inner_width, front_lip_depth, top_band_height)),
        origin=Origin(
            xyz=(
                0.0,
                (body_depth / 2.0) - (front_lip_depth / 2.0),
                top_band_center_z,
            )
        ),
        material=box_red_dark,
        name="front_header",
    )
    body.visual(
        Box((inner_width, front_lip_depth, slot_sill_height)),
        origin=Origin(
            xyz=(
                0.0,
                (body_depth / 2.0) - (front_lip_depth / 2.0),
                slot_sill_center_z,
            )
        ),
        material=box_red_dark,
        name="slot_sill",
    )
    body.visual(
        Box((body_width + 0.012, body_depth + 0.012, cap_thickness)),
        origin=Origin(
            xyz=(0.0, 0.0, (body_height / 2.0) + (cap_thickness / 2.0))
        ),
        material=aged_black,
        name="top_cap",
    )

    door = model.part("front_door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, door_thickness / 2.0, 0.0)),
        material=box_red_dark,
        name="door_panel",
    )
    door.visual(
        Box((door_width - 0.032, 0.005, door_height - 0.070)),
        origin=Origin(
            xyz=(door_width / 2.0, door_thickness + 0.0025, 0.0)
        ),
        material=box_red,
        name="door_outer_frame",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(
            xyz=(door_width - 0.030, door_thickness + 0.008, -0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="door_knob",
    )

    hood = model.part("slot_hood")
    hood.visual(
        Cylinder(radius=0.005, length=hood_width),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=aged_black,
        name="hood_barrel",
    )
    hood.visual(
        Box((hood_width, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.012, -0.004)),
        material=box_red_dark,
        name="hood_cap",
    )
    hood.visual(
        Box((hood_width, 0.046, 0.006)),
        origin=Origin(
            xyz=(0.0, 0.033, -0.015),
            rpy=(-0.55, 0.0, 0.0),
        ),
        material=box_red_dark,
        name="hood_flap",
    )

    model.articulation(
        "post_to_body",
        ArticulationType.FIXED,
        parent=post,
        child=body,
        origin=Origin(xyz=(0.0, body_center_y, body_center_z)),
    )
    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(
                -(door_width / 2.0),
                body_depth / 2.0,
                door_center_z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.90,
        ),
    )
    model.articulation(
        "body_to_slot_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(
            xyz=(0.0, (body_depth / 2.0) + 0.005, 0.169)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    post = object_model.get_part("post")
    body = object_model.get_part("body")
    door = object_model.get_part("front_door")
    hood = object_model.get_part("slot_hood")

    door_hinge = object_model.get_articulation("body_to_front_door")
    hood_hinge = object_model.get_articulation("body_to_slot_hood")

    ctx.check(
        "door hinge uses a vertical axis",
        tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "hood hinge uses a horizontal axis",
        tuple(round(v, 3) for v in hood_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hood_hinge.axis}",
    )

    ctx.expect_contact(
        post,
        body,
        elem_a="post_shaft",
        elem_b="body_back_wall",
        name="box body is mounted to the post",
    )

    with ctx.pose({door_hinge: 0.0, hood_hinge: 0.0}):
        ctx.expect_gap(
            hood,
            body,
            axis="y",
            positive_elem="hood_flap",
            max_gap=0.055,
            min_gap=0.0,
            name="closed hood projects slightly in front of the slot",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_panel",
            min_overlap=0.20,
            name="door covers the main front opening",
        )
        ctx.expect_contact(
            door,
            body,
            elem_a="door_panel",
            name="closed door seats against the front frame",
        )
        ctx.expect_origin_gap(
            hood,
            door,
            axis="z",
            min_gap=0.12,
            name="slot hood sits above the front door",
        )

        closed_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_hood_flap = ctx.part_element_world_aabb(hood, elem="hood_flap")

    with ctx.pose({door_hinge: 1.15}):
        open_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")

    with ctx.pose({hood_hinge: 0.90}):
        open_hood_flap = ctx.part_element_world_aabb(hood, elem="hood_flap")

    door_swings_outward = (
        closed_door_panel is not None
        and open_door_panel is not None
        and open_door_panel[1][1] > closed_door_panel[1][1] + 0.08
    )
    ctx.check(
        "front door swings outward on its vertical hinge",
        door_swings_outward,
        details=f"closed={closed_door_panel}, open={open_door_panel}",
    )

    hood_lifts_upward = (
        closed_hood_flap is not None
        and open_hood_flap is not None
        and open_hood_flap[1][2] > closed_hood_flap[1][2] + 0.03
    )
    ctx.check(
        "weather hood lifts upward when opened",
        hood_lifts_upward,
        details=f"closed={closed_hood_flap}, open={open_hood_flap}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
