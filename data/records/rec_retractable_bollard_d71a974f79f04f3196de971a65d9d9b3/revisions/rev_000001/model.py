from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _tube(outer_radius: float, inner_radius: float, height: float, z_min: float):
    """A vertical hollow cylinder in meters, with its lower face at z_min."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _ground_slab(size: float, thickness: float, opening_radius: float):
    slab = cq.Workplane("XY").box(size, size, thickness).translate((0.0, 0.0, -thickness / 2.0))
    cutter = (
        cq.Workplane("XY")
        .circle(opening_radius)
        .extrude(thickness + 0.04)
        .translate((0.0, 0.0, -thickness - 0.02))
    )
    return slab.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_security_bollard")

    asphalt = Material("dark_asphalt", color=(0.045, 0.043, 0.040, 1.0))
    galvanized = Material("galvanized_steel", color=(0.54, 0.56, 0.55, 1.0))
    dark_steel = Material("dark_powder_coated_steel", color=(0.06, 0.07, 0.075, 1.0))
    safety_yellow = Material("reflective_yellow", color=(1.0, 0.78, 0.05, 1.0))
    amber = Material("transparent_amber_lens", color=(1.0, 0.48, 0.03, 0.62))
    black_rubber = Material("black_rubber_shadow", color=(0.004, 0.004, 0.004, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_ground_slab(0.78, 0.055, 0.098), "pavement_with_opening"),
        material=asphalt,
        name="pavement",
    )
    sleeve.visual(
        mesh_from_cadquery(_tube(0.130, 0.095, 0.56, -0.48), "guide_sleeve"),
        material=galvanized,
        name="guide_sleeve",
    )
    sleeve.visual(
        mesh_from_cadquery(_tube(0.225, 0.098, 0.040, -0.004), "flange_collar"),
        material=galvanized,
        name="flange_collar",
    )
    sleeve.visual(
        mesh_from_cadquery(_tube(0.096, 0.076, 0.006, 0.036), "annular_opening_shadow"),
        material=black_rubber,
        name="opening_shadow",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        sleeve.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.175 * math.cos(angle), 0.175 * math.sin(angle), 0.041)),
            material=dark_steel,
            name=f"collar_bolt_{index}",
        )
    sleeve.visual(
        Box((0.023, 0.026, 0.075)),
        origin=Origin(xyz=(0.0845, 0.0, 0.0325)),
        material=black_rubber,
        name="guide_liner_x_pos",
    )
    sleeve.visual(
        Box((0.023, 0.026, 0.075)),
        origin=Origin(xyz=(-0.0845, 0.0, 0.0325)),
        material=black_rubber,
        name="guide_liner_x_neg",
    )
    sleeve.visual(
        Box((0.026, 0.023, 0.075)),
        origin=Origin(xyz=(0.0, 0.0845, 0.0325)),
        material=black_rubber,
        name="guide_liner_y_pos",
    )
    sleeve.visual(
        Box((0.026, 0.023, 0.075)),
        origin=Origin(xyz=(0.0, -0.0845, 0.0325)),
        material=black_rubber,
        name="guide_liner_y_neg",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.073, length=1.18),
        # The post frame sits at the collar opening; the tube extends downward
        # into the guide sleeve so it remains visibly captured at full travel.
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_steel,
        name="post_shell",
    )
    for index, z in enumerate((0.19, 0.50)):
        post.visual(
            Cylinder(radius=0.076, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=safety_yellow,
            name=f"reflective_band_{index}",
        )
    post.visual(
        Cylinder(radius=0.083, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=dark_steel,
        name="crown_cap",
    )
    post.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.813)),
        material=amber,
        name="warning_light",
    )
    # Side hinge lugs and small pedestal blocks keep the cover visibly mounted to
    # the crown instead of appearing to float above it.
    post.visual(
        Box((0.030, 0.032, 0.046)),
        origin=Origin(xyz=(-0.076, -0.0525, 0.823)),
        material=dark_steel,
        name="hinge_pedestal_0",
    )
    post.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(-0.085, -0.0525, 0.843), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_lug_0",
    )
    post.visual(
        Box((0.030, 0.032, 0.046)),
        origin=Origin(xyz=(-0.076, 0.0525, 0.823)),
        material=dark_steel,
        name="hinge_pedestal_1",
    )
    post.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(-0.085, 0.0525, 0.843), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_lug_1",
    )

    cover = model.part("light_cover")
    cover.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cover_barrel",
    )
    cover.visual(
        Box((0.150, 0.140, 0.026)),
        origin=Origin(xyz=(0.096, 0.0, 0.021)),
        material=amber,
        name="cover_lens",
    )
    cover.visual(
        Box((0.030, 0.060, 0.010)),
        origin=Origin(xyz=(0.025, 0.0, 0.010)),
        material=dark_steel,
        name="cover_hinge_leaf",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.35),
    )
    model.articulation(
        "post_to_light_cover",
        ArticulationType.REVOLUTE,
        parent=post,
        child=cover,
        origin=Origin(xyz=(-0.085, 0.0, 0.843)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cover = object_model.get_part("light_cover")
    slide = object_model.get_articulation("sleeve_to_post")
    hinge = object_model.get_articulation("post_to_light_cover")

    ctx.expect_origin_distance(
        sleeve,
        post,
        axes="xy",
        max_dist=0.001,
        name="post remains concentric with sleeve guide",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_shell",
        elem_b="guide_sleeve",
        min_overlap=0.40,
        name="resting post is deeply inserted in sleeve",
    )
    ctx.expect_contact(
        post,
        sleeve,
        elem_a="post_shell",
        elem_b="guide_liner_x_pos",
        contact_tol=0.001,
        name="post bears against sleeve guide liner",
    )
    ctx.expect_contact(
        cover,
        post,
        elem_a="cover_barrel",
        elem_b="hinge_lug_1",
        contact_tol=0.0015,
        name="cover barrel touches crown hinge lug",
    )

    rest_pos = ctx.part_world_position(post)
    with ctx.pose({slide: 0.35}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_shell",
            elem_b="guide_sleeve",
            min_overlap=0.09,
            name="extended post still has retained sleeve insertion",
        )
        extended_pos = ctx.part_world_position(post)
    ctx.check(
        "post extends upward on prismatic joint",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "warning cover rotates upward from crown",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.06,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
