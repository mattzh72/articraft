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
    model = ArticulatedObject(name="telescoping_cctv_column")

    steel = model.material("powder_coated_steel", rgba=(0.17, 0.18, 0.18, 1.0))
    edge_worn = model.material("worn_dark_steel", rgba=(0.28, 0.29, 0.28, 1.0))
    black = model.material("matte_black_camera", rgba=(0.015, 0.016, 0.017, 1.0))
    glass = model.material("smoked_lens_glass", rgba=(0.03, 0.08, 0.12, 0.78))
    bolt = model.material("black_oxide_bolts", rgba=(0.05, 0.05, 0.045, 1.0))

    base_tube = model.part("base_tube")
    base_tube.visual(
        Box((0.30, 0.30, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=edge_worn,
        name="base_plate",
    )
    # Square tube sleeve: four separate walls leave a true open bore for the
    # sliding round post instead of representing the lower column as a solid.
    base_tube.visual(
        Box((0.012, 0.140, 1.100)),
        origin=Origin(xyz=(0.064, 0.0, 0.575)),
        material=steel,
        name="tube_wall_x_pos",
    )
    base_tube.visual(
        Box((0.012, 0.140, 1.100)),
        origin=Origin(xyz=(-0.064, 0.0, 0.575)),
        material=steel,
        name="tube_wall_x_neg",
    )
    base_tube.visual(
        Box((0.116, 0.012, 1.100)),
        origin=Origin(xyz=(0.0, 0.064, 0.575)),
        material=steel,
        name="tube_wall_y_pos",
    )
    base_tube.visual(
        Box((0.116, 0.012, 1.100)),
        origin=Origin(xyz=(0.0, -0.064, 0.575)),
        material=steel,
        name="tube_wall_y_neg",
    )
    for name, x, y, size in (
        ("top_collar_x_pos", 0.079, 0.0, (0.018, 0.180, 0.050)),
        ("top_collar_x_neg", -0.079, 0.0, (0.018, 0.180, 0.050)),
        ("top_collar_y_pos", 0.0, 0.079, (0.140, 0.018, 0.050)),
        ("top_collar_y_neg", 0.0, -0.079, (0.140, 0.018, 0.050)),
    ):
        base_tube.visual(
            Box(size),
            origin=Origin(xyz=(x, y, 1.125)),
            material=steel,
            name=name,
        )
    base_tube.visual(
        Box((0.017, 0.035, 0.080)),
        origin=Origin(xyz=(0.0495, 0.0, 1.030)),
        material=bolt,
        name="guide_pad_x_pos",
    )
    base_tube.visual(
        Box((0.017, 0.035, 0.080)),
        origin=Origin(xyz=(-0.0495, 0.0, 1.030)),
        material=bolt,
        name="guide_pad_x_neg",
    )
    base_tube.visual(
        Box((0.035, 0.017, 0.080)),
        origin=Origin(xyz=(0.0, 0.0495, 1.030)),
        material=bolt,
        name="guide_pad_y_pos",
    )
    base_tube.visual(
        Box((0.035, 0.017, 0.080)),
        origin=Origin(xyz=(0.0, -0.0495, 1.030)),
        material=bolt,
        name="guide_pad_y_neg",
    )
    for i, (x, y) in enumerate(
        ((0.105, 0.105), (-0.105, 0.105), (-0.105, -0.105), (0.105, -0.105))
    ):
        base_tube.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(x, y, 0.031)),
            material=bolt,
            name=f"anchor_bolt_{i}",
        )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.041, length=1.550),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=edge_worn,
        name="round_post",
    )
    inner_post.visual(
        Cylinder(radius=0.049, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        material=steel,
        name="top_clamp_band",
    )
    inner_post.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.8625)),
        material=steel,
        name="post_cap",
    )
    inner_post.visual(
        Box((0.105, 0.070, 0.075)),
        origin=Origin(xyz=(0.055, 0.0, 0.800)),
        material=steel,
        name="clamp_block",
    )
    inner_post.visual(
        Box((0.410, 0.048, 0.050)),
        origin=Origin(xyz=(0.245, 0.0, 0.800)),
        material=steel,
        name="side_arm",
    )
    inner_post.visual(
        Box((0.090, 0.072, 0.012)),
        origin=Origin(xyz=(0.450, 0.0, 0.781)),
        material=steel,
        name="pan_mount_plate",
    )
    # Thin reference bands on the telescoping post make the hidden insertion
    # length legible without adding extra movable controls.
    for i, z in enumerate((0.22, 0.42, 0.62)):
        inner_post.visual(
            Cylinder(radius=0.0425, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bolt,
            name=f"height_band_{i}",
        )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.044, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=edge_worn,
        name="pan_bearing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=steel,
        name="pan_neck",
    )
    pan_yoke.visual(
        Box((0.155, 0.126, 0.030)),
        origin=Origin(xyz=(0.070, 0.0, -0.040)),
        material=steel,
        name="yoke_bridge",
    )
    pan_yoke.visual(
        Box((0.035, 0.012, 0.160)),
        origin=Origin(xyz=(0.150, 0.068, -0.130)),
        material=steel,
        name="yoke_cheek_0",
    )
    pan_yoke.visual(
        Box((0.035, 0.012, 0.160)),
        origin=Origin(xyz=(0.150, -0.068, -0.130)),
        material=steel,
        name="yoke_cheek_1",
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.045, length=0.220),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="bullet_body",
    )
    camera_head.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(-0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_cap",
    )
    camera_head.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(0.229, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="front_bezel",
    )
    camera_head.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.241, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    camera_head.visual(
        Cylinder(radius=0.014, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_worn,
        name="tilt_pin",
    )
    camera_head.visual(
        Box((0.225, 0.120, 0.010)),
        origin=Origin(xyz=(0.123, 0.0, 0.050)),
        material=black,
        name="sunshade_roof",
    )
    for name, y in (("sunshade_side_0", 0.051), ("sunshade_side_1", -0.051)):
        camera_head.visual(
            Box((0.205, 0.008, 0.046)),
            origin=Origin(xyz=(0.128, y, 0.027)),
            material=black,
            name=name,
        )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=base_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=0.45),
    )
    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=pan_yoke,
        origin=Origin(xyz=(0.450, 0.0, 0.745)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-2.80, upper=2.80),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_head,
        origin=Origin(xyz=(0.150, 0.0, -0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8, lower=-0.60, upper=0.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_tube = object_model.get_part("base_tube")
    inner_post = object_model.get_part("inner_post")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_head = object_model.get_part("camera_head")
    slide = object_model.get_articulation("post_slide")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.expect_origin_distance(
        base_tube,
        inner_post,
        axes="xy",
        max_dist=0.001,
        name="round post stays centered in square sleeve",
    )
    ctx.expect_overlap(
        inner_post,
        base_tube,
        axes="z",
        elem_a="round_post",
        elem_b="tube_wall_x_pos",
        min_overlap=0.35,
        name="collapsed post remains deeply inserted",
    )
    ctx.expect_contact(
        base_tube,
        inner_post,
        elem_a="guide_pad_x_pos",
        elem_b="round_post",
        contact_tol=1e-4,
        name="guide pad supports sliding post",
    )
    ctx.expect_contact(
        pan_yoke,
        inner_post,
        elem_a="pan_bearing",
        elem_b="pan_mount_plate",
        contact_tol=1e-4,
        name="pan bearing seats under arm bracket",
    )
    ctx.expect_contact(
        camera_head,
        pan_yoke,
        elem_a="tilt_pin",
        elem_b="yoke_cheek_0",
        contact_tol=1e-4,
        name="tilt pin reaches first yoke cheek",
    )
    ctx.expect_contact(
        camera_head,
        pan_yoke,
        elem_a="tilt_pin",
        elem_b="yoke_cheek_1",
        contact_tol=1e-4,
        name="tilt pin reaches second yoke cheek",
    )

    rest_post = ctx.part_world_position(inner_post)
    with ctx.pose({slide: 0.45}):
        raised_post = ctx.part_world_position(inner_post)
        ctx.expect_overlap(
            inner_post,
            base_tube,
            axes="z",
            elem_a="round_post",
            elem_b="tube_wall_x_pos",
            min_overlap=0.20,
            name="extended post retains insertion",
        )
        ctx.expect_contact(
            base_tube,
            inner_post,
            elem_a="guide_pad_x_pos",
            elem_b="round_post",
            contact_tol=1e-4,
            name="extended post remains in guide contact",
        )
    ctx.check(
        "post slide raises the inner post",
        rest_post is not None
        and raised_post is not None
        and raised_post[2] > rest_post[2] + 0.40,
        details=f"rest={rest_post}, raised={raised_post}",
    )

    rest_camera = ctx.part_world_position(camera_head)
    with ctx.pose({pan: 0.80}):
        panned_camera = ctx.part_world_position(camera_head)
    ctx.check(
        "pan joint swings camera sideways about vertical axis",
        rest_camera is not None
        and panned_camera is not None
        and abs(panned_camera[1] - rest_camera[1]) > 0.08,
        details=f"rest={rest_camera}, panned={panned_camera}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(camera_head, elem="front_lens")
    with ctx.pose({tilt: 0.45}):
        tilted_lens_aabb = ctx.part_element_world_aabb(camera_head, elem="front_lens")
    if rest_lens_aabb is not None and tilted_lens_aabb is not None:
        rest_lens_z = (rest_lens_aabb[0][2] + rest_lens_aabb[1][2]) / 2.0
        tilted_lens_z = (tilted_lens_aabb[0][2] + tilted_lens_aabb[1][2]) / 2.0
    else:
        rest_lens_z = tilted_lens_z = None
    ctx.check(
        "tilt joint pitches the bullet camera downward",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z < rest_lens_z - 0.05,
        details=f"rest_lens_z={rest_lens_z}, tilted_lens_z={tilted_lens_z}",
    )

    return ctx.report()


object_model = build_object_model()
