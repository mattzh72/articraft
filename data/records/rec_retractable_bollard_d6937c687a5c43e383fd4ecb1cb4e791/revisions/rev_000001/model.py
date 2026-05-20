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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_retractable_bollard")

    stainless = Material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    concrete = Material("pavement_concrete", rgba=(0.43, 0.42, 0.39, 1.0))
    amber = Material("amber_warning_lens", rgba=(1.0, 0.56, 0.05, 0.75))
    clear_smoke = Material("clear_smoke_cover", rgba=(0.62, 0.78, 0.95, 0.38))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    reflective = Material("yellow_reflective_band", rgba=(1.0, 0.82, 0.12, 1.0))

    sleeve = model.part("sleeve")

    pavement_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .box(0.92, 0.92, 0.10)
        .translate((0.0, 0.0, -0.05))
        .faces(">Z")
        .workplane()
        .circle(0.185)
        .cutThruAll(),
        "pavement_with_opening",
        tolerance=0.002,
    )
    sleeve.visual(pavement_mesh, material=concrete, name="pavement_opening")

    collar_mesh = mesh_from_cadquery(
        cq.Workplane("XY").circle(0.245).circle(0.112).extrude(0.045),
        "annular_sleeve_collar",
        tolerance=0.0015,
    )
    sleeve.visual(collar_mesh, material=stainless, name="collar_ring")

    sleeve_tube_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .circle(0.128)
        .circle(0.098)
        .extrude(0.42)
        .translate((0.0, 0.0, -0.40)),
        "below_grade_sleeve_tube",
        tolerance=0.0015,
    )
    sleeve.visual(sleeve_tube_mesh, material=dark_steel, name="sleeve_tube")

    # Low stainless ledges under the free end of the flap make the closed cover
    # read as a supported bridge instead of a floating plate.
    sleeve.visual(
        Box((0.12, 0.070, 0.012)),
        origin=Origin(xyz=(0.165, -0.135, 0.051)),
        material=stainless,
        name="flap_ledge_rear",
    )
    sleeve.visual(
        Box((0.12, 0.070, 0.012)),
        origin=Origin(xyz=(0.165, 0.135, 0.051)),
        material=stainless,
        name="flap_ledge_front",
    )

    for y in (-0.155, 0.155):
        sleeve.visual(
            Box((0.040, 0.070, 0.052)),
            origin=Origin(xyz=(-0.290, y, 0.026)),
            material=stainless,
            name=f"flap_hinge_stand_{'rear' if y < 0 else 'front'}",
        )
        sleeve.visual(
            Cylinder(radius=0.014, length=0.070),
            origin=Origin(xyz=(-0.290, y, 0.066), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"flap_hinge_knuckle_{'rear' if y < 0 else 'front'}",
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.085, length=1.30),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=stainless,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=0.098, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=dark_steel,
        name="lower_guide_ring",
    )
    post.visual(
        Cylinder(radius=0.093, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.0325)),
        material=stainless,
        name="crown_cap",
    )
    for z in (0.30, 0.68):
        post.visual(
            Cylinder(radius=0.087, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=reflective,
            name=f"reflective_band_{int(z * 100)}",
        )
    post.visual(
        Box((0.006, 0.006, 0.86)),
        origin=Origin(xyz=(0.087, 0.0, 0.55)),
        material=rubber,
        name="post_seam",
    )
    post.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.065)),
        material=dark_steel,
        name="light_base",
    )
    post.visual(
        Sphere(radius=0.034),
        origin=Origin(xyz=(0.0, 0.0, 1.100)),
        material=amber,
        name="warning_light",
    )
    post.visual(
        Box((0.052, 0.028, 0.060)),
        origin=Origin(xyz=(-0.077, -0.050, 1.085)),
        material=stainless,
        name="light_hinge_stand_0",
    )
    post.visual(
        Box((0.052, 0.028, 0.060)),
        origin=Origin(xyz=(-0.077, 0.050, 1.085)),
        material=stainless,
        name="light_hinge_stand_1",
    )
    post.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.075, -0.050, 1.103), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="light_hinge_knuckle_0",
    )
    post.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(-0.075, 0.050, 1.103), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="light_hinge_knuckle_1",
    )

    sleeve_flap = model.part("sleeve_flap")
    sleeve_flap.visual(
        Cylinder(radius=0.012, length=0.178),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="flap_hinge_barrel",
    )
    sleeve_flap.visual(
        Box((0.075, 0.210, 0.014)),
        origin=Origin(xyz=(0.040, 0.0, -0.002)),
        material=stainless,
        name="flap_hinge_leaf",
    )
    sleeve_flap.visual(
        Box((0.435, 0.060, 0.018)),
        origin=Origin(xyz=(0.2625, -0.135, 0.0)),
        material=stainless,
        name="flap_arm_rear",
    )
    sleeve_flap.visual(
        Box((0.045, 0.050, 0.006)),
        origin=Origin(xyz=(0.415, -0.135, -0.006)),
        material=rubber,
        name="flap_bumper_rear",
    )
    sleeve_flap.visual(
        Box((0.435, 0.060, 0.018)),
        origin=Origin(xyz=(0.2625, 0.135, 0.0)),
        material=stainless,
        name="flap_arm_front",
    )
    sleeve_flap.visual(
        Box((0.045, 0.050, 0.006)),
        origin=Origin(xyz=(0.415, 0.135, -0.006)),
        material=rubber,
        name="flap_bumper_front",
    )
    sleeve_flap.visual(
        Box((0.070, 0.330, 0.018)),
        origin=Origin(xyz=(0.445, 0.0, 0.0)),
        material=stainless,
        name="flap_nose_bridge",
    )
    light_cover = model.part("light_cover")
    light_cover.visual(
        Cylinder(radius=0.007, length=0.074),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cover_hinge_barrel",
    )
    light_cover.visual(
        Box((0.020, 0.040, 0.086)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=stainless,
        name="cover_hinge_tab",
    )
    light_cover.visual(
        Box((0.095, 0.016, 0.018)),
        origin=Origin(xyz=(0.050, -0.028, 0.050)),
        material=stainless,
        name="cover_side_strut_0",
    )
    light_cover.visual(
        Box((0.095, 0.016, 0.018)),
        origin=Origin(xyz=(0.050, 0.028, 0.050)),
        material=stainless,
        name="cover_side_strut_1",
    )
    light_cover.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.075, 0.0, 0.045)),
        material=clear_smoke,
        name="clear_light_lid",
    )
    light_cover.visual(
        Cylinder(radius=0.047, length=0.004),
        origin=Origin(xyz=(0.075, 0.0, 0.056)),
        material=amber,
        name="amber_warning_window",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.35, lower=-0.50, upper=0.0),
    )
    model.articulation(
        "sleeve_to_flap",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=sleeve_flap,
        origin=Origin(xyz=(-0.290, 0.0, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "post_to_light_cover",
        ArticulationType.REVOLUTE,
        parent=post,
        child=light_cover,
        origin=Origin(xyz=(-0.075, 0.0, 1.103)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    sleeve_flap = object_model.get_part("sleeve_flap")
    light_cover = object_model.get_part("light_cover")
    post_slide = object_model.get_articulation("sleeve_to_post")
    flap_hinge = object_model.get_articulation("sleeve_to_flap")
    light_hinge = object_model.get_articulation("post_to_light_cover")

    ctx.allow_overlap(
        post,
        sleeve,
        elem_a="lower_guide_ring",
        elem_b="sleeve_tube",
        reason="The lower guide ring is intentionally captured inside the simplified vertical sleeve tube so the bollard post reads as a retained sliding member.",
    )

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_body",
        outer_elem="collar_ring",
        margin=0.0,
        name="post centered in sleeve collar",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_tube",
        min_overlap=0.08,
        name="post retained in vertical sleeve",
    )
    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="lower_guide_ring",
        outer_elem="sleeve_tube",
        margin=0.0,
        name="guide ring centered inside sleeve tube",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="lower_guide_ring",
        elem_b="sleeve_tube",
        min_overlap=0.04,
        name="guide ring remains inside sleeve tube",
    )
    ctx.expect_gap(
        sleeve_flap,
        sleeve,
        axis="z",
        positive_elem="flap_arm_front",
        negative_elem="flap_ledge_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="flap rests on front ledge",
    )
    ctx.expect_gap(
        sleeve_flap,
        sleeve,
        axis="z",
        positive_elem="flap_arm_rear",
        negative_elem="flap_ledge_rear",
        max_gap=0.001,
        max_penetration=0.0,
        name="flap rests on rear ledge",
    )
    ctx.expect_overlap(
        sleeve_flap,
        sleeve,
        axes="xy",
        elem_a="flap_nose_bridge",
        elem_b="pavement_opening",
        min_overlap=0.05,
        name="sleeve flap bridges opening footprint",
    )
    ctx.expect_overlap(
        light_cover,
        post,
        axes="xy",
        elem_a="clear_light_lid",
        elem_b="warning_light",
        min_overlap=0.035,
        name="warning cover remains over crown light",
    )
    ctx.expect_gap(
        light_cover,
        post,
        axis="z",
        positive_elem="clear_light_lid",
        negative_elem="warning_light",
        min_gap=0.001,
        max_gap=0.025,
        name="warning cover clears crown light",
    )

    rest_post = ctx.part_world_position(post)
    with ctx.pose({post_slide: -0.45}):
        retracted_post = ctx.part_world_position(post)
        ctx.expect_within(
            post,
            sleeve,
            axes="xy",
            inner_elem="post_body",
            outer_elem="collar_ring",
            margin=0.0,
            name="post stays coaxial while retracted",
        )
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_tube",
            min_overlap=0.20,
            name="retracted post remains captured in sleeve",
        )
    ctx.check(
        "post slides down on sleeve axis",
        rest_post is not None
        and retracted_post is not None
        and retracted_post[2] < rest_post[2] - 0.40,
        details=f"rest={rest_post}, retracted={retracted_post}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(sleeve_flap, elem="flap_nose_bridge")
    with ctx.pose({flap_hinge: 1.20}):
        raised_flap_aabb = ctx.part_element_world_aabb(sleeve_flap, elem="flap_nose_bridge")
    ctx.check(
        "sleeve flap rotates upward from hinge",
        closed_flap_aabb is not None
        and raised_flap_aabb is not None
        and raised_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.05,
        details=f"closed={closed_flap_aabb}, raised={raised_flap_aabb}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(light_cover, elem="clear_light_lid")
    with ctx.pose({light_hinge: 1.0}):
        raised_cover_aabb = ctx.part_element_world_aabb(light_cover, elem="clear_light_lid")
    ctx.check(
        "warning light cover opens upward",
        closed_cover_aabb is not None
        and raised_cover_aabb is not None
        and raised_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.02,
        details=f"closed={closed_cover_aabb}, raised={raised_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
