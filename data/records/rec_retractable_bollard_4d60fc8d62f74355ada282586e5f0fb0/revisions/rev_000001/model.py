from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


POST_RADIUS = 0.054
POST_VISIBLE_HEIGHT = 0.740
POST_RETAINED_INSERTION = 0.180
POST_TRAVEL = 0.660
SLEEVE_INNER_RADIUS = 0.0635
SLEEVE_OUTER_RADIUS = 0.080
SLEEVE_DEPTH = 0.860


def _shell_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 64,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(outer_radius, z0), (outer_radius, z1)],
            inner_profile=[(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_access_bollard")

    paving = model.material("paving", rgba=(0.54, 0.54, 0.52, 1.0))
    trim = model.material("trim", rgba=(0.25, 0.27, 0.29, 1.0))
    sleeve_metal = model.material("sleeve_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    anodized = model.material("anodized", rgba=(0.46, 0.47, 0.49, 1.0))
    latch = model.material("latch", rgba=(0.88, 0.65, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        _shell_mesh(
            name="bollard_paving_ring",
            outer_radius=0.240,
            inner_radius=0.075,
            z0=-0.050,
            z1=0.000,
        ),
        material=paving,
        name="paving",
    )
    base.visual(
        _shell_mesh(
            name="bollard_sleeve_body",
            outer_radius=SLEEVE_OUTER_RADIUS,
            inner_radius=SLEEVE_INNER_RADIUS,
            z0=-SLEEVE_DEPTH,
            z1=0.006,
        ),
        material=sleeve_metal,
        name="sleeve_body",
    )
    base.visual(
        _shell_mesh(
            name="bollard_sleeve_trim",
            outer_radius=0.105,
            inner_radius=SLEEVE_INNER_RADIUS,
            z0=-0.004,
            z1=0.008,
        ),
        material=trim,
        name="sleeve_trim",
    )
    base.visual(
        Box((0.110, 0.076, 0.064)),
        origin=Origin(xyz=(0.182, 0.0, 0.032)),
        material=trim,
        name="service_box",
    )
    base.visual(
        Box((0.088, 0.008, 0.056)),
        origin=Origin(xyz=(0.182, 0.036, 0.032)),
        material=sleeve_metal,
        name="service_frame",
    )

    post = model.part("post")
    post_length = POST_VISIBLE_HEIGHT + POST_RETAINED_INSERTION
    post.visual(
        Cylinder(radius=POST_RADIUS, length=post_length),
        origin=Origin(xyz=(0.0, 0.0, (POST_VISIBLE_HEIGHT - POST_RETAINED_INSERTION) * 0.5)),
        material=steel,
        name="shaft",
    )
    post.visual(
        _shell_mesh(
            name="bollard_post_seat",
            outer_radius=0.078,
            inner_radius=POST_RADIUS,
            z0=0.008,
            z1=0.020,
        ),
        material=anodized,
        name="seat",
    )
    post.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        material=anodized,
        name="track",
    )
    post.visual(
        mesh_from_geometry(DomeGeometry(radius=0.055, radial_segments=40, height_segments=18), "bollard_dome"),
        origin=Origin(xyz=(0.0, 0.0, POST_VISIBLE_HEIGHT)),
        material=steel,
        name="dome",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.25,
            lower=-POST_TRAVEL,
            upper=0.0,
        ),
    )

    latch_ring = model.part("latch_ring")
    latch_ring.visual(
        _shell_mesh(
            name="bollard_latch_ring",
            outer_radius=0.0685,
            inner_radius=0.0585,
            z0=0.006,
            z1=0.030,
        ),
        material=latch,
        name="band",
    )
    latch_ring.visual(
        _shell_mesh(
            name="bollard_latch_seat",
            outer_radius=0.0605,
            inner_radius=0.0545,
            z0=0.000,
            z1=0.006,
        ),
        material=anodized,
        name="seat",
    )
    latch_ring.visual(
        Box((0.018, 0.014, 0.016)),
        origin=Origin(xyz=(0.077, 0.0, 0.018)),
        material=latch,
        name="grip",
    )

    model.articulation(
        "ring_spin",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=latch_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0),
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.072, 0.004, 0.048)),
        origin=Origin(xyz=(0.036, 0.002, 0.024)),
        material=anodized,
        name="panel",
    )
    hatch.visual(
        Box((0.012, 0.008, 0.010)),
        origin=Origin(xyz=(0.054, 0.008, 0.022)),
        material=steel,
        name="pull",
    )

    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hatch,
        origin=Origin(xyz=(0.146, 0.0385, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    latch_ring = object_model.get_part("latch_ring")
    hatch = object_model.get_part("hatch")
    post_slide = object_model.get_articulation("post_slide")
    ring_spin = object_model.get_articulation("ring_spin")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.expect_within(
        post,
        base,
        axes="xy",
        inner_elem="shaft",
        outer_elem="sleeve_body",
        margin=0.020,
        name="post stays centered over the sleeve",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="shaft",
        elem_b="sleeve_body",
        min_overlap=POST_RETAINED_INSERTION - 0.01,
        name="raised post keeps retained insertion in the sleeve",
    )
    ctx.expect_contact(
        post,
        base,
        elem_a="seat",
        elem_b="sleeve_trim",
        contact_tol=0.001,
        name="raised post seats on the sleeve trim",
    )
    ctx.expect_origin_distance(
        latch_ring,
        post,
        axes="xy",
        max_dist=0.001,
        name="latch ring stays coaxial with the post",
    )
    ctx.expect_contact(
        latch_ring,
        post,
        elem_a="seat",
        elem_b="track",
        contact_tol=0.001,
        name="latch ring rides on the post track",
    )
    ctx.expect_gap(
        hatch,
        base,
        axis="y",
        positive_elem="panel",
        negative_elem="service_box",
        min_gap=0.0003,
        max_gap=0.012,
        name="closed hatch sits just outside the service box face",
    )

    raised_pos = ctx.part_world_position(post)
    with ctx.pose({post_slide: post_slide.motion_limits.lower}):
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="shaft",
            elem_b="sleeve_body",
            min_overlap=0.500,
            name="lowered post remains guided deep inside the sleeve",
        )
        lowered_pos = ctx.part_world_position(post)

    ctx.check(
        "post retracts downward into the sleeve",
        raised_pos is not None
        and lowered_pos is not None
        and lowered_pos[2] < raised_pos[2] - 0.50,
        details=f"raised={raised_pos}, lowered={lowered_pos}",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="panel")
    with ctx.pose({hatch_hinge: 1.10}):
        open_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="panel")

    ctx.check(
        "hatch opens outward on a vertical hinge",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][1] > closed_hatch_aabb[1][1] + 0.020,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )
    ctx.check(
        "latch ring uses continuous centerline rotation",
        ring_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(ring_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={ring_spin.articulation_type}, axis={ring_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
