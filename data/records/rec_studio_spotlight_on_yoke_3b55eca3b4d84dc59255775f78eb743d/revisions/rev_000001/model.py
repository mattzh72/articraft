from __future__ import annotations

import cadquery as cq
from math import pi

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


def _vertical_tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Open annular tube with its lower lip at local z=0."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate(
        (0.0, 0.0, -0.002)
    )
    return outer.cut(cutter)


def _horizontal_tube(
    outer_radius: float, inner_radius: float, length: float, *, center_x: float = 0.0
) -> cq.Workplane:
    """Open annular tube aligned with local X and centered at center_x."""
    outer = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )
    cutter = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(length + 0.006)
        .translate((center_x - (length + 0.006) / 2.0, 0.0, 0.0))
    )
    return outer.cut(cutter)


def _lamp_can_mesh() -> cq.Workplane:
    """Stage-style open lamp can with a proud accessory/lens ring at the front."""
    body = _horizontal_tube(0.140, 0.120, 0.420)
    front_ring = _horizontal_tube(0.158, 0.104, 0.040, center_x=0.220)
    rear_bezel = _horizontal_tube(0.145, 0.116, 0.020, center_x=-0.210)
    return body.union(front_ring).union(rear_bezel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_yoke_spotlight")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.034, 0.031, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.10, 0.095, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.68, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    glass = model.material("pale_fresnel_glass", rgba=(0.75, 0.90, 1.00, 0.45))

    # Floor stand: heavy plate plus a genuinely hollow outer sleeve so the mast
    # can slide through it without a collision proxy.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.300, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=matte_black,
        name="base_plate",
    )
    stand.visual(
        mesh_from_cadquery(_vertical_tube(0.045, 0.032, 0.720), "stand_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="sleeve_tube",
    )
    stand.visual(
        mesh_from_cadquery(_vertical_tube(0.055, 0.033, 0.055), "lower_sleeve_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=matte_black,
        name="lower_collar",
    )
    stand.visual(
        mesh_from_cadquery(_vertical_tube(0.056, 0.033, 0.060), "upper_sleeve_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        material=matte_black,
        name="upper_collar",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.065),
        origin=Origin(xyz=(0.062, 0.0, 0.742), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="clamp_screw",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.103, 0.0, 0.742), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="clamp_knob",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.026, length=1.050),
        # Hidden length below the part frame keeps the mast retained inside the
        # sleeve even at full extension.
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=brushed_steel,
        name="inner_post",
    )
    post.visual(
        Cylinder(radius=0.043, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        material=dark_metal,
        name="pan_spigot",
    )
    post.visual(
        Cylinder(radius=0.044, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_metal,
        name="height_stop_collar",
    )

    model.articulation(
        "stand_to_post",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.770)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.22, lower=0.0, upper=0.350),
    )

    # Panning yoke: a stage-fixture U bracket sitting on the post's pan spigot.
    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=matte_black,
        name="pan_bearing",
    )
    yoke.visual(
        Box((0.095, 0.405, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=matte_black,
        name="lower_bridge",
    )
    for side, y in (("near", -0.180), ("far", 0.180)):
        yoke.visual(
            Box((0.085, 0.030, 0.345)),
            origin=Origin(xyz=(0.0, y, 0.220)),
            material=matte_black,
            name=f"{side}_cheek",
        )
        yoke.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(0.0, y * 1.075, 0.280), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{side}_tilt_boss",
        )

    model.articulation(
        "post_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2),
    )

    # Lamp can: a horizontal cylindrical shell with lens, rear plate, side
    # trunnion shaft, and fixed hinge lands for the front accessory clips.
    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_lamp_can_mesh(), "lamp_can_shell"),
        material=satin_black,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.124, length=0.010),
        origin=Origin(xyz=(0.222, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    can.visual(
        Cylinder(radius=0.118, length=0.016),
        origin=Origin(xyz=(-0.222, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=matte_black,
        name="rear_cap",
    )
    can.visual(
        Cylinder(radius=0.021, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="tilt_axle",
    )
    can.visual(
        Box((0.036, 0.086, 0.020)),
        origin=Origin(xyz=(0.230, 0.0, 0.160)),
        material=matte_black,
        name="top_clip_lug",
    )
    can.visual(
        Box((0.036, 0.086, 0.020)),
        origin=Origin(xyz=(0.230, 0.0, -0.160)),
        material=matte_black,
        name="bottom_clip_lug",
    )

    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=-0.75, upper=0.75),
    )

    top_clip = model.part("top_clip")
    top_clip.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    top_clip.visual(
        Box((0.030, 0.060, 0.060)),
        origin=Origin(xyz=(0.015, 0.0, -0.030)),
        material=matte_black,
        name="clip_tab",
    )
    top_clip.visual(
        Box((0.030, 0.060, 0.010)),
        origin=Origin(xyz=(0.025, 0.0, -0.064)),
        material=matte_black,
        name="retaining_lip",
    )

    bottom_clip = model.part("bottom_clip")
    bottom_clip.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    bottom_clip.visual(
        Box((0.030, 0.060, 0.060)),
        origin=Origin(xyz=(0.015, 0.0, 0.030)),
        material=matte_black,
        name="clip_tab",
    )
    bottom_clip.visual(
        Box((0.030, 0.060, 0.010)),
        origin=Origin(xyz=(0.025, 0.0, 0.064)),
        material=matte_black,
        name="retaining_lip",
    )

    model.articulation(
        "can_to_top_clip",
        ArticulationType.REVOLUTE,
        parent=can,
        child=top_clip,
        origin=Origin(xyz=(0.245, 0.0, 0.176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "can_to_bottom_clip",
        ArticulationType.REVOLUTE,
        parent=can,
        child=bottom_clip,
        origin=Origin(xyz=(0.245, 0.0, -0.176)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    post = object_model.get_part("post")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    top_clip = object_model.get_part("top_clip")
    bottom_clip = object_model.get_part("bottom_clip")
    slide = object_model.get_articulation("stand_to_post")
    pan = object_model.get_articulation("post_to_yoke")
    tilt = object_model.get_articulation("yoke_to_can")
    top_hinge = object_model.get_articulation("can_to_top_clip")
    bottom_hinge = object_model.get_articulation("can_to_bottom_clip")

    ctx.check(
        "primary mechanisms are articulated",
        len(object_model.articulations) == 5
        and slide.articulation_type == ArticulationType.PRISMATIC
        and pan.articulation_type == ArticulationType.CONTINUOUS
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and top_hinge.articulation_type == ArticulationType.REVOLUTE
        and bottom_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )

    ctx.expect_within(
        post,
        stand,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="sleeve_tube",
        margin=0.003,
        name="post stays centered in sleeve",
    )
    ctx.expect_overlap(
        post,
        stand,
        axes="z",
        elem_a="inner_post",
        elem_b="sleeve_tube",
        min_overlap=0.35,
        name="collapsed post has deep sleeve insertion",
    )
    rest_post = ctx.part_world_position(post)
    with ctx.pose({slide: 0.350}):
        ctx.expect_within(
            post,
            stand,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="sleeve_tube",
            margin=0.003,
            name="extended post remains centered",
        )
        ctx.expect_overlap(
            post,
            stand,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_tube",
            min_overlap=0.060,
            name="extended post remains retained",
        )
        extended_post = ctx.part_world_position(post)
    ctx.check(
        "post slides upward at extension",
        rest_post is not None and extended_post is not None and extended_post[2] > rest_post[2] + 0.30,
        details=f"rest={rest_post}, extended={extended_post}",
    )

    def aabb_center_z(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return 0.5 * (lo[2] + hi[2])

    lens_rest_z = aabb_center_z(can, "front_lens")
    with ctx.pose({tilt: 0.55}):
        lens_up_z = aabb_center_z(can, "front_lens")
    ctx.check(
        "lamp can tilts upward on yoke",
        lens_rest_z is not None and lens_up_z is not None and lens_up_z > lens_rest_z + 0.08,
        details=f"rest_z={lens_rest_z}, tilted_z={lens_up_z}",
    )

    def aabb_center_x(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return 0.5 * (lo[0] + hi[0])

    top_lip_rest_x = aabb_center_x(top_clip, "retaining_lip")
    bottom_lip_rest_x = aabb_center_x(bottom_clip, "retaining_lip")
    with ctx.pose({top_hinge: 0.95, bottom_hinge: 0.95}):
        top_lip_open_x = aabb_center_x(top_clip, "retaining_lip")
        bottom_lip_open_x = aabb_center_x(bottom_clip, "retaining_lip")
    ctx.check(
        "front accessory clips swing away from the ring",
        top_lip_rest_x is not None
        and bottom_lip_rest_x is not None
        and top_lip_open_x is not None
        and bottom_lip_open_x is not None
        and top_lip_open_x > top_lip_rest_x + 0.025
        and bottom_lip_open_x > bottom_lip_rest_x + 0.025,
        details=(
            f"top {top_lip_rest_x}->{top_lip_open_x}, "
            f"bottom {bottom_lip_rest_x}->{bottom_lip_open_x}"
        ),
    )

    ctx.expect_overlap(yoke, can, axes="z", min_overlap=0.15, name="yoke surrounds can height")
    ctx.expect_overlap(yoke, can, axes="y", min_overlap=0.25, name="yoke spans across can sides")

    return ctx.report()


object_model = build_object_model()
