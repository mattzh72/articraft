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
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="streetside_retractable_bollard")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    black_finish = model.material("black_powder_coat", rgba=(0.005, 0.006, 0.007, 1.0))
    rubber_black = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    concrete = model.material("pale_concrete", rgba=(0.48, 0.46, 0.42, 1.0))
    warning_red = model.material("protected_red_actuator", rgba=(0.75, 0.05, 0.03, 1.0))

    # Static streetside sleeve, below-grade socket mouth, base collar, and the
    # recessed actuator pocket.  The central bore is modeled as a real annulus so
    # the sliding post fits through the opening rather than intersecting a solid.
    base_sleeve = model.part("base_sleeve")

    sidewalk_shape = cq.Workplane("XY").box(0.70, 0.56, 0.030).translate((0.0, 0.0, -0.015))
    base_sleeve.visual(
        mesh_from_cadquery(sidewalk_shape, "sidewalk_pad", tolerance=0.002),
        name="sidewalk_pad",
        material=concrete,
    )

    flange_shape = cq.Workplane("XY").circle(0.245).circle(0.095).extrude(0.042)
    base_sleeve.visual(
        mesh_from_cadquery(flange_shape, "black_ground_flange", tolerance=0.0015),
        name="ground_flange",
        material=black_finish,
    )

    collar_shape = (
        cq.Workplane("XY")
        .circle(0.165)
        .circle(0.087)
        .extrude(0.215)
        .translate((0.0, 0.0, 0.040))
    )
    base_sleeve.visual(
        mesh_from_cadquery(collar_shape, "black_base_collar", tolerance=0.0015),
        name="base_collar",
        material=black_finish,
    )

    upper_lip_shape = (
        cq.Workplane("XY")
        .circle(0.112)
        .circle(0.078)
        .extrude(0.028)
        .translate((0.0, 0.0, 0.255))
    )
    base_sleeve.visual(
        mesh_from_cadquery(upper_lip_shape, "steel_sleeve_lip", tolerance=0.001),
        name="sleeve_lip",
        material=dark_steel,
    )

    guide_bushing_shape = (
        cq.Workplane("XY")
        .circle(0.080)
        .circle(0.0648)
        .extrude(0.030)
        .translate((0.0, 0.0, 0.252))
    )
    base_sleeve.visual(
        mesh_from_cadquery(guide_bushing_shape, "sliding_guide_bushing", tolerance=0.001),
        name="guide_bushing",
        material=dark_steel,
    )

    actuator_frame_shape = (
        cq.Workplane("XZ")
        .rect(0.165, 0.135)
        .rect(0.112, 0.082)
        .extrude(0.036)
        .translate((0.0, -0.196, 0.145))
    )
    base_sleeve.visual(
        mesh_from_cadquery(actuator_frame_shape, "actuator_cavity_frame", tolerance=0.001),
        name="actuator_cavity_frame",
        material=black_finish,
    )
    base_sleeve.visual(
        Box((0.126, 0.070, 0.105)),
        origin=Origin(xyz=(0.0, -0.168, 0.145)),
        name="actuator_mount_block",
        material=black_finish,
    )
    base_sleeve.visual(
        Box((0.102, 0.005, 0.074)),
        origin=Origin(xyz=(0.0, -0.176, 0.145)),
        name="actuator_recess_back",
        material=dark_steel,
    )
    base_sleeve.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, -0.207, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="recessed_actuator",
        material=warning_red,
    )

    # Alternating fixed hinge knuckles on the top edge of the actuator pocket.
    for x in (-0.066, 0.066):
        base_sleeve.visual(
            Cylinder(radius=0.010, length=0.038),
            origin=Origin(xyz=(x, -0.250, 0.218), rpy=(0.0, math.pi / 2.0, 0.0)),
            name=f"fixed_hinge_knuckle_{0 if x < 0 else 1}",
            material=black_finish,
        )
        base_sleeve.visual(
            Box((0.026, 0.020, 0.035)),
            origin=Origin(xyz=(x, -0.235, 0.201)),
            name=f"hinge_bracket_{0 if x < 0 else 1}",
            material=black_finish,
        )
    base_sleeve.visual(
        Cylinder(radius=0.0055, length=0.172),
        origin=Origin(xyz=(0.0, -0.250, 0.218), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="hinge_pin",
        material=dark_steel,
    )

    # Sliding brushed-metal post.  Its local frame is the sleeve-mouth plane;
    # the long hidden lower tail keeps realistic retained insertion at full height.
    post = model.part("post")
    post.visual(
        Cylinder(radius=0.065, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        name="brushed_post_tube",
        material=brushed_steel,
    )
    post.visual(
        Cylinder(radius=0.073, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        name="crown_cap",
        material=brushed_steel,
    )
    for z in (0.050, 0.205, 0.360, 0.515, 0.670):
        post.visual(
            Cylinder(radius=0.066, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, z)),
            name=f"brushed_band_{int(z * 1000)}",
            material=dark_steel,
        )

    post_slide = model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=base_sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=-0.680, upper=0.0),
    )

    # Pivoting security cover for the recessed actuator.  The child frame is on
    # the hinge line; at q=0 it hangs closed in front of the pocket, and positive
    # rotation flips it outward on the horizontal hinge.
    cover = model.part("cover_plate")
    cover.visual(
        Box((0.137, 0.008, 0.122)),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        name="cover_plate_panel",
        material=black_finish,
    )
    cover.visual(
        Cylinder(radius=0.011, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="cover_hinge_knuckle",
        material=black_finish,
    )
    cover.visual(
        Box((0.058, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        name="hinge_tab",
        material=black_finish,
    )

    model.articulation(
        "sleeve_to_cover",
        ArticulationType.REVOLUTE,
        parent=base_sleeve,
        child=cover,
        origin=Origin(xyz=(0.0, -0.250, 0.218)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    # Crown release knob: a small black knob on a short vertical shaft, separate
    # from the sliding post so it can rotate independently on the bollard crown.
    crown_knob = model.part("crown_knob")
    crown_knob.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        name="knob_shaft",
        material=rubber_black,
    )
    crown_knob.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        name="knob_cap",
        material=rubber_black,
    )
    crown_knob.visual(
        Box((0.088, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        name="knob_grip_bar",
        material=rubber_black,
    )

    model.articulation(
        "post_to_knob",
        ArticulationType.REVOLUTE,
        parent=post,
        child=crown_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.750)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.5708, upper=1.5708),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_sleeve = object_model.get_part("base_sleeve")
    post = object_model.get_part("post")
    cover = object_model.get_part("cover_plate")
    knob = object_model.get_part("crown_knob")
    post_slide = object_model.get_articulation("sleeve_to_post")
    cover_hinge = object_model.get_articulation("sleeve_to_cover")
    knob_joint = object_model.get_articulation("post_to_knob")

    ctx.allow_overlap(
        base_sleeve,
        post,
        elem_a="guide_bushing",
        elem_b="brushed_post_tube",
        reason=(
            "The post is intentionally captured in a close sliding guide bushing; "
            "the bushing mesh is a sleeve proxy around the hidden sliding tube."
        ),
    )
    ctx.allow_overlap(
        base_sleeve,
        cover,
        elem_a="hinge_pin",
        elem_b="cover_hinge_knuckle",
        reason="The cover barrel is intentionally captured around the fixed hinge pin.",
    )

    ctx.expect_within(
        post,
        base_sleeve,
        axes="xy",
        inner_elem="brushed_post_tube",
        outer_elem="base_collar",
        margin=0.003,
        name="post is centered inside sleeve bore",
    )
    ctx.expect_overlap(
        post,
        base_sleeve,
        axes="z",
        elem_a="brushed_post_tube",
        elem_b="base_collar",
        min_overlap=0.150,
        name="raised post keeps hidden insertion in sleeve",
    )
    ctx.expect_gap(
        base_sleeve,
        cover,
        axis="y",
        positive_elem="actuator_cavity_frame",
        negative_elem="cover_plate_panel",
        min_gap=0.003,
        max_gap=0.020,
        name="closed cover sits just in front of recessed frame",
    )
    ctx.expect_overlap(
        cover,
        base_sleeve,
        axes="x",
        elem_a="cover_hinge_knuckle",
        elem_b="hinge_pin",
        min_overlap=0.050,
        name="cover hinge barrel is carried by fixed pin",
    )
    ctx.expect_contact(
        knob,
        post,
        elem_a="knob_shaft",
        elem_b="crown_cap",
        contact_tol=0.002,
        name="knob shaft seats on the crown cap",
    )

    raised_top = ctx.part_element_world_aabb(post, elem="crown_cap")
    with ctx.pose({post_slide: -0.680}):
        retracted_top = ctx.part_element_world_aabb(post, elem="crown_cap")
        ctx.expect_within(
            post,
            base_sleeve,
            axes="xy",
            inner_elem="brushed_post_tube",
            outer_elem="base_collar",
            margin=0.003,
            name="retracted post remains coaxial in sleeve",
        )
    ctx.check(
        "post retracts downward along sleeve axis",
        raised_top is not None
        and retracted_top is not None
        and retracted_top[1][2] < raised_top[1][2] - 0.60,
        details=f"raised={raised_top}, retracted={retracted_top}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate_panel")
    with ctx.pose({cover_hinge: 1.35}):
        opened_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate_panel")
    ctx.check(
        "cover pivots outward from the actuator cavity",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[0][1] < closed_cover_aabb[0][1] - 0.055,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    with ctx.pose({knob_joint: 1.0}):
        ctx.expect_contact(
            knob,
            post,
            elem_a="knob_shaft",
            elem_b="crown_cap",
            contact_tol=0.002,
            name="rotated knob remains seated on crown",
        )

    return ctx.report()


object_model = build_object_model()
