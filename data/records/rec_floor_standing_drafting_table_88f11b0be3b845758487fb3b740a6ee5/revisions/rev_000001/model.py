from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobRelief,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drafting_table")

    dark_steel = Material("dark_powder_coated_steel", color=(0.06, 0.07, 0.075, 1.0))
    chrome = Material("brushed_chrome", color=(0.72, 0.74, 0.72, 1.0))
    warm_board = Material("warm_drafting_board", color=(0.86, 0.82, 0.68, 1.0))
    pale_surface = Material("pale_green_drawing_surface", color=(0.70, 0.82, 0.72, 1.0))
    black_plastic = Material("black_bakelite", color=(0.015, 0.015, 0.014, 1.0))
    rubber = Material("matte_rubber", color=(0.025, 0.025, 0.025, 1.0))

    base = model.part("base")

    # Floor-standing H base with long studio runners and two hollow outer posts.
    for i, x in enumerate((-0.36, 0.36)):
        base.visual(
            Box((0.16, 0.90, 0.07)),
            origin=Origin(xyz=(x, 0.0, 0.035)),
            material=dark_steel,
            name=f"floor_runner_{i}",
        )
        base.visual(
            Box((0.13, 0.10, 0.026)),
            origin=Origin(xyz=(x, -0.37, 0.018)),
            material=rubber,
            name=f"front_foot_{i}",
        )
        base.visual(
            Box((0.13, 0.10, 0.026)),
            origin=Origin(xyz=(x, 0.37, 0.018)),
            material=rubber,
            name=f"rear_foot_{i}",
        )

        # Rectangular tube walls, deliberately leaving a clear inner cavity.
        for side, dx in (("left", -0.0405), ("right", 0.0405)):
            base.visual(
                Box((0.014, 0.085, 0.94)),
                origin=Origin(xyz=(x + dx, 0.0, 0.535)),
                material=dark_steel,
                name=f"outer_post_{i}_{side}_wall",
            )
        for side, y in (("front", 0.0355), ("rear", -0.0355)):
            base.visual(
                Box((0.095, 0.014, 0.94)),
                origin=Origin(xyz=(x, y, 0.535)),
                material=dark_steel,
                name=f"outer_post_{i}_{side}_wall",
            )

    base.visual(
        Box((0.86, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_steel,
        name="lower_crossbar",
    )
    base.visual(
        Box((0.86, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, -0.058, 0.980)),
        material=dark_steel,
        name="rear_top_tie",
    )

    inner = model.part("inner_stages")
    for i, x in enumerate((-0.36, 0.36)):
        inner.visual(
            Box((0.046, 0.036, 0.84)),
            origin=Origin(xyz=(x, 0.0, -0.10)),
            material=chrome,
            name=f"inner_post_{i}",
        )
        inner.visual(
            Box((0.030, 0.011, 0.070)),
            origin=Origin(xyz=(x, 0.023, -0.250)),
            material=chrome,
            name=f"front_glide_pad_{i}",
        )

    inner.visual(
        Box((0.86, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=chrome,
        name="upper_bridge",
    )
    inner.visual(
        Box((0.10, 0.09, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=chrome,
        name="central_support",
    )
    inner.visual(
        Box((0.34, 0.14, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=dark_steel,
        name="head_saddle",
    )
    for i, x in enumerate((-0.18, 0.18)):
        inner.visual(
            Box((0.052, 0.050, 0.120)),
            origin=Origin(xyz=(x, -0.085, 0.585)),
            material=dark_steel,
            name=f"head_cheek_{i}",
        )
    inner.visual(
        Box((0.54, 0.035, 0.040)),
        origin=Origin(xyz=(0.37, -0.085, 0.565)),
        material=dark_steel,
        name="clamp_arm",
    )
    inner.visual(
        Cylinder(radius=0.024, length=0.10),
        origin=Origin(xyz=(0.69, -0.085, 0.565), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="threaded_boss",
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.026, length=1.25),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    board.visual(
        Box((1.40, 0.90, 0.038)),
        origin=Origin(xyz=(0.0, 0.420, 0.035)),
        material=warm_board,
        name="drawing_panel",
    )
    board.visual(
        Box((1.34, 0.82, 0.006)),
        origin=Origin(xyz=(0.0, 0.445, 0.057)),
        material=pale_surface,
        name="drawing_surface",
    )
    board.visual(
        Box((1.45, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, 0.885, 0.040)),
        material=warm_board,
        name="front_pencil_ledge",
    )
    for i, x in enumerate((-0.725, 0.725)):
        board.visual(
            Box((0.035, 0.92, 0.052)),
            origin=Origin(xyz=(x, 0.430, 0.040)),
            material=warm_board,
            name=f"side_edge_{i}",
        )
    board.visual(
        Box((1.35, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.020, 0.030)),
        material=dark_steel,
        name="rear_hinge_leaf",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.105,
            0.055,
            body_style="lobed",
            base_diameter=0.066,
            top_diameter=0.095,
            crown_radius=0.003,
            bore=KnobBore(style="round", diameter=0.014),
            body_reliefs=(KnobRelief(style="top_recess", width=0.036, depth=0.003),),
            center=False,
        ),
        "lobed_clamp_knob",
    )
    knob = model.part("clamp_knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.120),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="threaded_stem",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.27),
    )
    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=inner,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-0.12, upper=1.05),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=inner,
        child=knob,
        origin=Origin(xyz=(0.740, -0.085, 0.565)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    inner = object_model.get_part("inner_stages")
    board = object_model.get_part("board")
    knob = object_model.get_part("clamp_knob")
    height = object_model.get_articulation("height_slide")
    tilt = object_model.get_articulation("board_tilt")

    ctx.allow_overlap(
        inner,
        knob,
        elem_a="threaded_boss",
        elem_b="threaded_stem",
        reason="The clamp knob's threaded stem is intentionally engaged inside the side boss.",
    )
    ctx.expect_overlap(
        knob,
        inner,
        axes="x",
        elem_a="threaded_stem",
        elem_b="threaded_boss",
        min_overlap=0.045,
        name="clamp stem remains threaded into boss",
    )
    ctx.expect_within(
        knob,
        inner,
        axes="yz",
        inner_elem="threaded_stem",
        outer_elem="threaded_boss",
        margin=0.002,
        name="clamp stem is coaxial with boss",
    )

    for i in (0, 1):
        ctx.expect_overlap(
            inner,
            base,
            axes="z",
            elem_a=f"inner_post_{i}",
            elem_b=f"outer_post_{i}_front_wall",
            min_overlap=0.35,
            name=f"inner post {i} retained in outer sleeve",
        )

    with ctx.pose({height: 0.27}):
        for i in (0, 1):
            ctx.expect_overlap(
                inner,
                base,
                axes="z",
                elem_a=f"inner_post_{i}",
                elem_b=f"outer_post_{i}_front_wall",
                min_overlap=0.20,
                name=f"extended inner post {i} still retained",
            )

    ctx.expect_contact(
        board,
        inner,
        elem_a="hinge_barrel",
        elem_b="head_saddle",
        contact_tol=0.002,
        name="board hinge barrel sits on support head",
    )

    rest_panel = ctx.part_element_world_aabb(board, elem="drawing_panel")
    with ctx.pose({tilt: 0.70}):
        tilted_panel = ctx.part_element_world_aabb(board, elem="drawing_panel")
    ctx.check(
        "tilt joint raises the drawing surface",
        rest_panel is not None
        and tilted_panel is not None
        and tilted_panel[1][2] > rest_panel[1][2] + 0.25,
        details=f"rest_panel={rest_panel}, tilted_panel={tilted_panel}",
    )

    return ctx.report()


object_model = build_object_model()
