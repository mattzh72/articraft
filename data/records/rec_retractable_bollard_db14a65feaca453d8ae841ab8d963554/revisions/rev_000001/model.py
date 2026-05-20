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


SLEEVE_OUTER_RADIUS = 0.075
SLEEVE_INNER_RADIUS = 0.060
SLEEVE_BOTTOM_Z = -0.360
TOP_FACE_Z = 0.018
POST_RADIUS = 0.050
POST_LENGTH = 0.400
POST_LOCAL_CENTER_Z = -0.130
POST_TRAVEL = 0.280

LOCK_CENTER = (0.0, -0.082)
LOCK_HOUSING_TOP_Z = 0.041
LOCK_COVER_HINGE = (0.0, -0.106, 0.058)


def _sleeve_shape() -> cq.Workplane:
    """CadQuery shell for the fixed ground sleeve, flange, drain slots, and lock boss."""

    tube_height = TOP_FACE_Z - SLEEVE_BOTTOM_Z
    tube_outer = (
        cq.Workplane("XY")
        .circle(SLEEVE_OUTER_RADIUS)
        .extrude(tube_height)
        .translate((0.0, 0.0, SLEEVE_BOTTOM_Z))
    )
    tube_bore = (
        cq.Workplane("XY")
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(tube_height + 0.030)
        .translate((0.0, 0.0, SLEEVE_BOTTOM_Z - 0.015))
    )
    tube = tube_outer.cut(tube_bore)

    top_flange = cq.Workplane("XY").circle(0.110).extrude(TOP_FACE_Z)
    top_opening = (
        cq.Workplane("XY")
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(TOP_FACE_Z + 0.010)
        .translate((0.0, 0.0, -0.005))
    )
    flange = top_flange.cut(top_opening)

    lock_housing = (
        cq.Workplane("XY")
        .box(0.074, 0.038, 0.024)
        .edges("|Z")
        .fillet(0.006)
        .translate((LOCK_CENTER[0], LOCK_CENTER[1], 0.029))
    )
    core_bore = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.040)
        .translate((LOCK_CENTER[0], LOCK_CENTER[1], 0.030))
    )
    lock_housing = lock_housing.cut(core_bore)

    body = tube.union(flange).union(lock_housing)

    # Rectangular through-slots around the lower sleeve wall; they read as drain ports
    # while preserving a continuous tube.
    for angle_deg in range(0, 360, 45):
        slot = (
            cq.Workplane("XY")
            .box(0.060, 0.020, 0.062)
            .translate((SLEEVE_OUTER_RADIUS - 0.010, 0.0, -0.205))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        body = body.cut(slot)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_retractable_bollard")

    stainless = model.material("brushed_stainless", rgba=(0.62, 0.65, 0.63, 1.0))
    dark = model.material("blackened_recess", rgba=(0.015, 0.016, 0.014, 1.0))
    yellow = model.material("reflective_yellow", rgba=(1.0, 0.74, 0.10, 1.0))
    rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.028, 1.0))
    brass = model.material("brass_lock", rgba=(0.88, 0.62, 0.23, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shape(), "sleeve_shell", tolerance=0.0007),
        material=stainless,
        name="sleeve_shell",
    )
    # Small fixed hinge ears are mounted into the raised lock housing and flank the
    # moving cover's central knuckle.
    for suffix, x in (("0", -0.032), ("1", 0.032)):
        sleeve.visual(
            Box((0.016, 0.007, 0.014)),
            origin=Origin(xyz=(x, -0.104, 0.047)),
            material=stainless,
            name=f"hinge_ear_{suffix}",
        )
        sleeve.visual(
            Cylinder(radius=0.0052, length=0.014),
            origin=Origin(xyz=(x, -0.106, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"hinge_barrel_{suffix}",
        )
    for name, xyz, size in (
        ("guide_pad_x_pos", (0.0545, 0.0, -0.020), (0.011, 0.020, 0.055)),
        ("guide_pad_x_neg", (-0.0545, 0.0, -0.020), (0.011, 0.020, 0.055)),
        ("guide_pad_y_pos", (0.0, 0.0545, -0.020), (0.020, 0.011, 0.055)),
        ("guide_pad_y_neg", (0.0, -0.0545, -0.020), (0.020, 0.011, 0.055)),
    ):
        sleeve.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=dark,
            name=name,
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, POST_LOCAL_CENTER_Z)),
        material=stainless,
        name="post_body",
    )
    post.visual(
        Cylinder(radius=0.054, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=rubber,
        name="post_cap",
    )
    for suffix, z in (("lower", 0.026), ("upper", 0.055)):
        post.visual(
            Cylinder(radius=0.0515, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=yellow,
            name=f"{suffix}_band",
        )

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        Box((0.072, 0.052, 0.006)),
        # At q=0 the cover is swung outward beside the lock boss, exposing the core.
        # Positive rotation folds it back over the lock housing.
        origin=Origin(xyz=(0.0, -0.030, 0.004)),
        material=stainless,
        name="cover_plate",
    )
    lock_cover.visual(
        Cylinder(radius=0.0050, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="cover_knuckle",
    )
    lock_cover.visual(
        Box((0.030, 0.0025, 0.0015)),
        origin=Origin(xyz=(0.0, -0.041, 0.0073)),
        material=dark,
        name="cover_lip",
    )

    lock_core = model.part("lock_core")
    lock_core.visual(
        Cylinder(radius=0.011, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=brass,
        name="core_stem",
    )
    lock_core.visual(
        Cylinder(radius=0.016, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=brass,
        name="core_face",
    )
    lock_core.visual(
        Box((0.023, 0.003, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=dark,
        name="key_slot",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, TOP_FACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=POST_TRAVEL),
    )
    model.articulation(
        "sleeve_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=lock_cover,
        origin=Origin(xyz=LOCK_COVER_HINGE),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=2.65),
    )
    model.articulation(
        "sleeve_to_lock_core",
        ArticulationType.CONTINUOUS,
        parent=sleeve,
        child=lock_core,
        origin=Origin(xyz=(LOCK_CENTER[0], LOCK_CENTER[1], 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    lock_cover = object_model.get_part("lock_cover")
    lock_core = object_model.get_part("lock_core")
    post_slide = object_model.get_articulation("sleeve_to_post")
    cover_hinge = object_model.get_articulation("sleeve_to_lock_cover")
    core_turn = object_model.get_articulation("sleeve_to_lock_core")

    for pad_name, positive_link, negative_link, axis in (
        ("guide_pad_x_pos", sleeve, post, "x"),
        ("guide_pad_x_neg", post, sleeve, "x"),
        ("guide_pad_y_pos", sleeve, post, "y"),
        ("guide_pad_y_neg", post, sleeve, "y"),
    ):
        ctx.allow_overlap(
            sleeve,
            post,
            elem_a=pad_name,
            elem_b="post_body",
            reason="Low-friction guide pad is intentionally modeled with slight compression against the sliding post.",
        )
        if pad_name.endswith("_neg"):
            ctx.expect_gap(
                positive_link,
                negative_link,
                axis=axis,
                max_penetration=0.002,
                positive_elem="post_body",
                negative_elem=pad_name,
                name=f"{pad_name} has only slight guide compression",
            )
        else:
            ctx.expect_gap(
                positive_link,
                negative_link,
                axis=axis,
                max_penetration=0.002,
                positive_elem=pad_name,
                negative_elem="post_body",
                name=f"{pad_name} has only slight guide compression",
            )

    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        min_overlap=0.27,
        elem_a="post_body",
        elem_b="sleeve_shell",
        name="retracted post remains captured in sleeve",
    )
    retracted_aabb = ctx.part_world_aabb(post)
    with ctx.pose({post_slide: POST_TRAVEL}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            min_overlap=0.035,
            elem_a="post_body",
            elem_b="sleeve_shell",
            name="extended post retains insertion in sleeve",
        )
        extended_aabb = ctx.part_world_aabb(post)
    ctx.check(
        "post extends upward",
        retracted_aabb is not None
        and extended_aabb is not None
        and extended_aabb[1][2] > retracted_aabb[1][2] + 0.20,
        details=f"retracted={retracted_aabb}, extended={extended_aabb}",
    )

    rest_cover_aabb = ctx.part_world_aabb(lock_cover)
    with ctx.pose({cover_hinge: 1.4}):
        swung_cover_aabb = ctx.part_world_aabb(lock_cover)
    ctx.check(
        "lock cover rotates on side hinge",
        rest_cover_aabb is not None
        and swung_cover_aabb is not None
        and swung_cover_aabb[0][1] > rest_cover_aabb[0][1] + 0.025,
        details=f"rest={rest_cover_aabb}, swung={swung_cover_aabb}",
    )

    slot_rest = ctx.part_element_world_aabb(lock_core, elem="key_slot")
    with ctx.pose({core_turn: math.pi / 2.0}):
        slot_turned = ctx.part_element_world_aabb(lock_core, elem="key_slot")
    ctx.check(
        "lock core turns continuously",
        slot_rest is not None
        and slot_turned is not None
        and (slot_rest[1][0] - slot_rest[0][0]) > (slot_rest[1][1] - slot_rest[0][1])
        and (slot_turned[1][1] - slot_turned[0][1]) > (slot_turned[1][0] - slot_turned[0][0]),
        details=f"rest={slot_rest}, turned={slot_turned}",
    )

    return ctx.report()


object_model = build_object_model()
