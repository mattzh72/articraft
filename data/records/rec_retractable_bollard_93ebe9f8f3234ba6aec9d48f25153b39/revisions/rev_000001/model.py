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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").cylinder(height, outer_radius)
    inner = cq.Workplane("XY").cylinder(height + 0.004, inner_radius)
    return outer.cut(inner)


def _paving_slab_with_round_sleeve_opening() -> cq.Workplane:
    slab = cq.Workplane("XY").box(0.82, 0.82, 0.08).translate((0.0, 0.0, -0.04))
    sleeve_clearance = cq.Workplane("XY").cylinder(0.12, 0.082).translate((0.0, 0.0, -0.04))
    return slab.cut(sleeve_clearance)


def _post_body() -> cq.Workplane:
    radius = 0.035
    bottom_z = -0.260
    cylinder_top_z = 0.560
    tube = cq.Workplane("XY").cylinder(cylinder_top_z - bottom_z, radius).translate(
        (0.0, 0.0, 0.5 * (bottom_z + cylinder_top_z))
    )
    dome = cq.Workplane("XY").sphere(radius).translate((0.0, 0.0, cylinder_top_z))
    return tube.union(dome)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rising_parking_bollard")

    paving_concrete = model.material("paving_concrete", rgba=(0.48, 0.48, 0.45, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    worn_black = model.material("worn_black", rgba=(0.02, 0.025, 0.025, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.76, 0.05, 1.0))
    reflective_white = model.material("reflective_white", rgba=(0.92, 0.95, 0.90, 1.0))

    paving = model.part("paving")
    paving.visual(
        mesh_from_cadquery(_paving_slab_with_round_sleeve_opening(), "paving_slab"),
        material=paving_concrete,
        name="paving_slab",
    )
    paving.visual(
        mesh_from_cadquery(_annular_cylinder(0.125, 0.044, 0.012), "flush_sleeve_flange"),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=brushed_steel,
        name="flush_sleeve_flange",
    )
    paving.visual(
        mesh_from_cadquery(_annular_cylinder(0.060, 0.041, 0.700), "sleeve_wall"),
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        material=dark_steel,
        name="sleeve_wall",
    )
    paving.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.056, tube=0.006, radial_segments=12, tubular_segments=72),
            "rounded_lip_bead",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brushed_steel,
        name="rounded_lip_bead",
    )
    paving.visual(
        Box((0.155, 0.012, 0.135)),
        origin=Origin(xyz=(0.0, -0.195, 0.066)),
        material=dark_steel,
        name="inspection_recess",
    )
    paving.visual(
        Box((0.170, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.178, 0.010)),
        material=brushed_steel,
        name="hatch_threshold",
    )
    paving.visual(
        Cylinder(radius=0.0065, length=0.128),
        origin=Origin(xyz=(-0.060, -0.188, 0.064)),
        material=brushed_steel,
        name="fixed_hinge_leaf",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_post_body(), "post_body", tolerance=0.0008),
        material=worn_black,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=0.0362, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=safety_yellow,
        name="lower_reflective_band",
    )
    post.visual(
        Cylinder(radius=0.0362, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=reflective_white,
        name="upper_reflective_band",
    )
    post.visual(
        Box((0.008, 0.014, 0.070)),
        origin=Origin(xyz=(0.038, 0.0, -0.080)),
        material=dark_steel,
        name="guide_pad_0",
    )
    post.visual(
        Box((0.008, 0.014, 0.070)),
        origin=Origin(xyz=(-0.038, 0.0, -0.080)),
        material=dark_steel,
        name="guide_pad_1",
    )
    post.visual(
        Box((0.014, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, 0.038, -0.080)),
        material=dark_steel,
        name="guide_pad_2",
    )
    post.visual(
        Box((0.014, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, -0.038, -0.080)),
        material=dark_steel,
        name="guide_pad_3",
    )

    latch_ring = model.part("latch_ring")
    latch_ring.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.034, tube=0.0045, radial_segments=14, tubular_segments=72),
            "latch_grab_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=brushed_steel,
        name="latch_grab_ring",
    )
    latch_ring.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=brushed_steel,
        name="latch_top_cap",
    )
    latch_ring.visual(
        Box((0.006, 0.005, 0.038)),
        origin=Origin(xyz=(0.030, 0.0, 0.110)),
        material=brushed_steel,
        name="latch_upright_0",
    )
    latch_ring.visual(
        Box((0.006, 0.005, 0.038)),
        origin=Origin(xyz=(-0.030, 0.0, 0.110)),
        material=brushed_steel,
        name="latch_upright_1",
    )

    hatch = model.part("inspection_hatch")
    hatch.visual(
        Box((0.104, 0.006, 0.102)),
        origin=Origin(xyz=(0.052, -0.006, 0.056)),
        material=brushed_steel,
        name="hatch_plate",
    )
    hatch.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.0, -0.003, 0.056)),
        material=dark_steel,
        name="hatch_hinge_barrel",
    )
    hatch.visual(
        Box((0.026, 0.004, 0.010)),
        origin=Origin(xyz=(0.077, -0.011, 0.057)),
        material=worn_black,
        name="quarter_turn_slot",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=paving,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=-0.50, upper=0.0),
    )
    model.articulation(
        "latch_spin",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=latch_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=paving,
        child=hatch,
        origin=Origin(xyz=(-0.060, -0.202, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    paving = object_model.get_part("paving")
    post = object_model.get_part("post")
    latch = object_model.get_part("latch_ring")
    hatch = object_model.get_part("inspection_hatch")
    post_slide = object_model.get_articulation("post_slide")
    latch_spin = object_model.get_articulation("latch_spin")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    for index in range(4):
        pad_name = f"guide_pad_{index}"
        ctx.allow_overlap(
            paving,
            post,
            elem_a="sleeve_wall",
            elem_b=pad_name,
            reason="Hidden low-friction guide shoes are intentionally seated into the sleeve liner to keep the rising post supported.",
        )
        ctx.expect_overlap(
            post,
            paving,
            axes="z",
            elem_a=pad_name,
            elem_b="sleeve_wall",
            min_overlap=0.060,
            name=f"{pad_name} remains engaged with the sleeve liner",
        )
    ctx.allow_overlap(
        latch,
        post,
        elem_a="latch_top_cap",
        elem_b="post_body",
        reason="The rotating latch cap is seated into the rounded dome as a compact bearing interface.",
    )
    ctx.expect_gap(
        latch,
        post,
        axis="z",
        positive_elem="latch_top_cap",
        negative_elem="post_body",
        max_gap=0.002,
        max_penetration=0.004,
        name="latch cap is seated on the dome crown",
    )

    ctx.expect_overlap(
        post,
        paving,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_wall",
        min_overlap=0.20,
        name="raised post remains captured in the sleeve",
    )
    ctx.expect_within(
        post,
        paving,
        axes="xy",
        elem_a="post_body",
        elem_b="sleeve_wall",
        margin=0.0,
        name="post centerline stays inside circular sleeve footprint",
    )
    ctx.expect_origin_distance(
        latch,
        post,
        axes="xy",
        max_dist=0.001,
        name="latch ring spins around post centerline",
    )
    ctx.check(
        "latch joint is continuous",
        str(latch_spin.articulation_type).lower().endswith("continuous"),
        details=f"latch type={latch_spin.articulation_type}",
    )

    raised_aabb = ctx.part_world_aabb(post)
    with ctx.pose({post_slide: -0.50}):
        lowered_aabb = ctx.part_world_aabb(post)
        ctx.expect_overlap(
            post,
            paving,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_wall",
            min_overlap=0.55,
            name="lowered post is deeply retained by the sleeve",
        )
    ctx.check(
        "post translates downward in the sleeve",
        raised_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[1][2] < raised_aabb[1][2] - 0.45,
        details=f"raised={raised_aabb}, lowered={lowered_aabb}",
    )

    closed_hatch = ctx.part_world_aabb(hatch)
    with ctx.pose({hatch_hinge: 1.25}):
        open_hatch = ctx.part_world_aabb(hatch)
    ctx.check(
        "inspection hatch swings on a vertical hinge",
        closed_hatch is not None
        and open_hatch is not None
        and open_hatch[1][1] > closed_hatch[1][1] + 0.040,
        details=f"closed={closed_hatch}, open={open_hatch}",
    )
    ctx.check(
        "hatch hinge has vertical axis",
        tuple(round(v, 3) for v in hatch_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={hatch_hinge.axis}",
    )

    return ctx.report()


object_model = build_object_model()
