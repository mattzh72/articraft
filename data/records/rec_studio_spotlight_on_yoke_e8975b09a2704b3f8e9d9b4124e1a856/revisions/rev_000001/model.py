from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_along_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Centered hollow cylinder whose axis is local X."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.036, 0.038, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.46, 0.47, 0.48, 1.0))
    amber_glass = model.material("warm_lens_glass", rgba=(0.95, 0.72, 0.34, 0.42))
    pale_glass = model.material("pale_glass", rgba=(0.80, 0.91, 1.00, 0.34))

    # Root: a compact square floor/desk base with a fixed vertical support post.
    base_post = model.part("base_post")
    base_post.visual(
        Box((0.240, 0.240, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=matte_black,
        name="square_base",
    )
    base_post.visual(
        Box((0.185, 0.185, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=satin_black,
        name="raised_plinth",
    )
    base_post.visual(
        Cylinder(radius=0.017, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=dark_steel,
        name="support_post",
    )
    base_post.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.347)),
        material=brushed_steel,
        name="pan_bearing",
    )

    # Child of the pan joint: the U-shaped yoke and turntable collar.
    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.037, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="pan_collar",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_steel,
        name="turntable_stem",
    )
    yoke.visual(
        Box((0.240, 0.284, 0.026)),
        origin=Origin(xyz=(0.090, 0.0, 0.062)),
        material=satin_black,
        name="top_bridge",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        yoke.visual(
            Box((0.024, 0.020, 0.175)),
            origin=Origin(xyz=(0.180, y_sign * 0.123, -0.030)),
            material=satin_black,
            name=f"yoke_arm_{index}",
        )
        yoke.visual(
            Cylinder(radius=0.024, length=0.016),
            origin=Origin(
                xyz=(0.180, y_sign * 0.141, -0.110),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"tilt_cap_{index}",
        )
        yoke.visual(
            Box((0.030, 0.018, 0.030)),
            origin=Origin(xyz=(0.180, y_sign * 0.123, -0.110)),
            material=dark_steel,
            name=f"pivot_block_{index}",
        )

    # The lamp can frame is at the horizontal tilt/trunnion axis.
    lamp = model.part("lamp_can")
    lamp.visual(
        mesh_from_cadquery(_tube_along_x(0.090, 0.075, 0.300), "lamp_can_shell"),
        material=satin_black,
        name="can_shell",
    )
    lamp.visual(
        mesh_from_cadquery(_tube_along_x(0.098, 0.073, 0.024), "front_bezel_mesh"),
        origin=Origin(xyz=(0.144, 0.0, 0.0)),
        material=matte_black,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.083, length=0.010),
        origin=Origin(xyz=(-0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="bulb_mount",
    )
    lamp.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=amber_glass,
        name="lamp_bulb",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        lamp.visual(
            Cylinder(radius=0.019, length=0.024),
            origin=Origin(
                xyz=(0.0, y_sign * 0.101, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"trunnion_{index}",
        )
    # Fixed half of the small side hinge for the front lens cover.
    for index, z_sign in enumerate((-1.0, 1.0)):
        lamp.visual(
            Box((0.022, 0.014, 0.035)),
            origin=Origin(xyz=(0.154, 0.094, z_sign * 0.045)),
            material=dark_steel,
            name=f"cover_hinge_leaf_{index}",
        )
    lamp.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.166, 0.098, 0.047)),
        material=brushed_steel,
        name="cover_hinge_knuckle_0",
    )
    lamp.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.166, 0.098, -0.047)),
        material=brushed_steel,
        name="cover_hinge_knuckle_1",
    )

    # The cover's child frame is the small side hinge axis.  At q=0 the cover sits
    # in front of the can; positive rotation swings it out to the side.
    lens_cover = model.part("lens_cover")
    lens_cover.visual(
        mesh_from_cadquery(_tube_along_x(0.087, 0.070, 0.008), "cover_ring_mesh"),
        origin=Origin(xyz=(0.014, -0.098, 0.0)),
        material=satin_black,
        name="cover_ring",
    )
    lens_cover.visual(
        Cylinder(radius=0.073, length=0.006),
        origin=Origin(xyz=(0.014, -0.098, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_glass,
        name="lens_glass",
    )
    lens_cover.visual(
        Box((0.016, 0.095, 0.024)),
        origin=Origin(xyz=(0.010, -0.050, 0.0)),
        material=satin_black,
        name="cover_hinge_leaf",
    )
    lens_cover.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="cover_hinge_barrel",
    )

    pan_joint = model.articulation(
        "post_pan",
        ArticulationType.REVOLUTE,
        parent=base_post,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )
    tilt_joint = model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.180, 0.0, -0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.95, upper=0.95),
    )
    cover_joint = model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=lamp,
        child=lens_cover,
        origin=Origin(xyz=(0.166, 0.098, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.4, lower=0.0, upper=1.75),
    )

    # Keep the joint variables referenced by name in the tests, while also making
    # the intended primary mechanisms clear at build time.
    _ = (pan_joint, tilt_joint, cover_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_post")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp_can")
    cover = object_model.get_part("lens_cover")

    pan = object_model.get_articulation("post_pan")
    tilt = object_model.get_articulation("yoke_tilt")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.expect_contact(
        base,
        yoke,
        elem_a="pan_bearing",
        elem_b="pan_collar",
        contact_tol=0.001,
        name="yoke sits on the post pan bearing",
    )
    ctx.expect_gap(
        cover,
        lamp,
        axis="x",
        positive_elem="cover_ring",
        negative_elem="front_bezel",
        min_gap=0.006,
        max_gap=0.035,
        name="closed lens cover sits just in front of the can bezel",
    )
    ctx.expect_overlap(
        cover,
        lamp,
        axes="yz",
        elem_a="cover_ring",
        elem_b="front_bezel",
        min_overlap=0.120,
        name="closed lens cover is centered over the front opening",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_ring")
    with ctx.pose({cover_hinge: 1.30}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_ring")
    ctx.check(
        "side hinge swings cover forward and outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and (open_cover_aabb[0][0] + open_cover_aabb[1][0]) * 0.5
        > (closed_cover_aabb[0][0] + closed_cover_aabb[1][0]) * 0.5 + 0.045
        and (open_cover_aabb[0][1] + open_cover_aabb[1][1]) * 0.5
        > (closed_cover_aabb[0][1] + closed_cover_aabb[1][1]) * 0.5 + 0.035,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    closed_front_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({tilt: 0.70}):
        tilted_front_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    ctx.check(
        "lamp can tilts on the yoke trunnions",
        closed_front_aabb is not None
        and tilted_front_aabb is not None
        and (tilted_front_aabb[0][2] + tilted_front_aabb[1][2]) * 0.5
        < (closed_front_aabb[0][2] + closed_front_aabb[1][2]) * 0.5 - 0.045,
        details=f"closed={closed_front_aabb}, tilted={tilted_front_aabb}",
    )

    ctx.check(
        "pan joint has stage-light turntable travel",
        pan.motion_limits is not None
        and pan.motion_limits.lower <= -3.0
        and pan.motion_limits.upper >= 3.0,
    )

    return ctx.report()


object_model = build_object_model()
