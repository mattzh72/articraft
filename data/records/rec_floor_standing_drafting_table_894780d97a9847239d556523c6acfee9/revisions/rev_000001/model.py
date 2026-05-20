from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _square_tube(outer_x: float, outer_y: float, height: float, wall: float):
    outer = cq.Workplane("XY").box(outer_x, outer_y, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(outer_x - 2.0 * wall, outer_y - 2.0 * wall, height + 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, -0.003))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_stand")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    board_offwhite = model.material("board_offwhite", rgba=(0.86, 0.87, 0.84, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.175, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=powder_black,
        name="pedestal_plate",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=graphite,
        name="pedestal_cap",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="floor_pad",
    )
    base.visual(
        mesh_from_cadquery(_square_tube(0.066, 0.052, 0.360, 0.005), "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=satin_gray,
        name="outer_sleeve",
    )
    base.visual(
        Box((0.086, 0.072, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=graphite,
        name="sleeve_collar",
    )
    base.visual(
        mesh_from_cadquery(_square_tube(0.080, 0.066, 0.022, 0.010), "top_socket"),
        origin=Origin(xyz=(0.0, 0.0, 0.382)),
        material=graphite,
        name="top_socket",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Box((0.048, 0.034, 0.660)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=satin_gray,
        name="post_tube",
    )
    inner_post.visual(
        Box((0.072, 0.058, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="slide_carriage",
    )
    inner_post.visual(
        Box((0.054, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.432)),
        material=graphite,
        name="top_cap",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.404)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.160,
        ),
    )

    head = model.part("head")
    head.visual(
        Box((0.060, 0.046, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="receiver_block",
    )
    head.visual(
        Box((0.032, 0.100, 0.054)),
        origin=Origin(xyz=(0.0, 0.050, 0.047)),
        material=graphite,
        name="support_neck",
    )
    head.visual(
        Box((0.180, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.085, 0.030)),
        material=graphite,
        name="lower_crosshead",
    )
    head.visual(
        Box((0.014, 0.036, 0.090)),
        origin=Origin(xyz=(-0.083, 0.085, 0.065)),
        material=graphite,
        name="left_cheek",
    )
    head.visual(
        Box((0.014, 0.036, 0.090)),
        origin=Origin(xyz=(0.083, 0.085, 0.065)),
        material=graphite,
        name="right_cheek",
    )
    model.articulation(
        "post_to_head",
        ArticulationType.FIXED,
        parent=inner_post,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.444)),
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.011, length=0.152),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=powder_black,
        name="hinge_barrel",
    )
    board.visual(
        Box((0.145, 0.050, 0.034)),
        origin=Origin(xyz=(0.0, 0.008, 0.017)),
        material=graphite,
        name="hinge_block",
    )
    board.visual(
        Box((0.560, 0.014, 0.390)),
        origin=Origin(xyz=(0.0, 0.032, 0.225)),
        material=board_offwhite,
        name="panel",
    )
    board.visual(
        Box((0.500, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.054, 0.040)),
        material=satin_gray,
        name="lower_lip",
    )
    board.visual(
        Box((0.500, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.072, 0.047)),
        material=satin_gray,
        name="lip_fence",
    )

    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=board,
        origin=Origin(xyz=(0.0, 0.085, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=powder_black,
        name="thread_shaft",
    )
    clamp_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="mushroom",
                top_diameter=0.038,
                base_diameter=0.026,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=6, depth=0.0022),
                center=False,
            ),
            "clamp_knob_body",
        ),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    clamp_knob.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=rubber,
        name="knob_washer",
    )

    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=clamp_knob,
        origin=Origin(xyz=(0.090, 0.085, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
        ),
    )

    return model


def _aabb_text(bounds) -> str:
    if bounds is None:
        return "None"
    return f"min={tuple(round(v, 4) for v in bounds[0])}, max={tuple(round(v, 4) for v in bounds[1])}"


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_post = object_model.get_part("inner_post")
    board = object_model.get_part("board")
    clamp_knob = object_model.get_part("clamp_knob")

    post_slide = object_model.get_articulation("post_slide")
    board_tilt = object_model.get_articulation("board_tilt")
    knob_spin = object_model.get_articulation("knob_spin")

    post_limits = post_slide.motion_limits
    board_limits = board_tilt.motion_limits

    rest_post_pos = None
    raised_post_pos = None
    board_rest = None
    board_tilted = None
    knob_pos_a = None
    knob_pos_b = None

    if post_limits is not None and post_limits.upper is not None:
        with ctx.pose({post_slide: 0.0}):
            ctx.expect_within(
                inner_post,
                base,
                axes="xy",
                inner_elem="post_tube",
                outer_elem="outer_sleeve",
                margin=0.002,
                name="inner post stays centered in the outer sleeve at rest",
            )
            ctx.expect_overlap(
                inner_post,
                base,
                axes="z",
                elem_a="post_tube",
                elem_b="outer_sleeve",
                min_overlap=0.220,
                name="rest pose keeps substantial post insertion",
            )
            rest_post_pos = ctx.part_world_position(inner_post)

        with ctx.pose({post_slide: post_limits.upper}):
            ctx.expect_within(
                inner_post,
                base,
                axes="xy",
                inner_elem="post_tube",
                outer_elem="outer_sleeve",
                margin=0.002,
                name="inner post stays centered in the outer sleeve when raised",
            )
            ctx.expect_overlap(
                inner_post,
                base,
                axes="z",
                elem_a="post_tube",
                elem_b="outer_sleeve",
                min_overlap=0.060,
                name="raised pose still retains insertion in the sleeve",
            )
            raised_post_pos = ctx.part_world_position(inner_post)

        ctx.check(
            "post extends upward",
            rest_post_pos is not None
            and raised_post_pos is not None
            and raised_post_pos[2] > rest_post_pos[2] + 0.12,
            details=f"rest={rest_post_pos}, raised={raised_post_pos}",
        )

    if board_limits is not None and board_limits.upper is not None:
        with ctx.pose({board_tilt: 0.0}):
            board_rest = ctx.part_element_world_aabb(board, elem="panel")
        with ctx.pose({board_tilt: board_limits.upper}):
            board_tilted = ctx.part_element_world_aabb(board, elem="panel")

        ctx.check(
            "board tilts backward at the upper limit",
            board_rest is not None
            and board_tilted is not None
            and board_tilted[0][1] < board_rest[0][1] - 0.20
            and board_tilted[1][2] < board_rest[1][2] - 0.10,
            details=f"rest={_aabb_text(board_rest)}, tilted={_aabb_text(board_tilted)}",
        )

    with ctx.pose({knob_spin: 0.0}):
        knob_pos_a = ctx.part_world_position(clamp_knob)
    with ctx.pose({knob_spin: 2.3}):
        knob_pos_b = ctx.part_world_position(clamp_knob)

    ctx.check(
        "clamp knob spins in place",
        knob_pos_a is not None
        and knob_pos_b is not None
        and max(abs(a - b) for a, b in zip(knob_pos_a, knob_pos_b)) < 1e-6,
        details=f"pose_a={knob_pos_a}, pose_b={knob_pos_b}",
    )

    return ctx.report()


object_model = build_object_model()
