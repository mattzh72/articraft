from __future__ import annotations

import math

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
)


WIDTH = 1.42
DEPTH = 0.58
HEIGHT = 1.10
BOARD = 0.045
BACK_THICKNESS = 0.025
FRONT_RAIL_DEPTH = 0.045

DRAWER_COUNT = 5
DRAWER_FACE_WIDTH = 1.24
DRAWER_FACE_HEIGHT = 0.170
DRAWER_GAP = 0.025
DRAWER_BOTTOM_MARGIN = 0.075
DRAWER_FRONT_THICKNESS = 0.035
DRAWER_BODY_DEPTH = 0.490
DRAWER_BODY_WIDTH = 1.208
DRAWER_BODY_HEIGHT = 0.112
DRAWER_TRAVEL = 0.320


def _drawer_center_z(index: int) -> float:
    """Bottom-to-top drawer center height."""
    return (
        DRAWER_BOTTOM_MARGIN
        + DRAWER_FACE_HEIGHT / 2.0
        + index * (DRAWER_FACE_HEIGHT + DRAWER_GAP)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_drawer_wooden_dresser")

    walnut = model.material("warm_walnut", color=(0.55, 0.31, 0.13, 1.0))
    walnut_dark = model.material("dark_walnut_endgrain", color=(0.33, 0.18, 0.075, 1.0))
    walnut_light = model.material("drawer_walnut", color=(0.63, 0.38, 0.18, 1.0))
    brass = model.material("aged_brass", color=(0.86, 0.62, 0.24, 1.0))
    rail_metal = model.material("dark_slide_steel", color=(0.18, 0.18, 0.17, 1.0))

    half_w = WIDTH / 2.0
    half_d = DEPTH / 2.0

    carcass = model.part("carcass")
    # Main rectangular dresser case: two sides, top, bottom and a thin back.
    carcass.visual(
        Box((DEPTH, BOARD, HEIGHT)),
        origin=Origin(xyz=(0.0, half_w - BOARD / 2.0, HEIGHT / 2.0)),
        material=walnut,
        name="side_panel_0",
    )
    carcass.visual(
        Box((DEPTH, BOARD, HEIGHT)),
        origin=Origin(xyz=(0.0, -half_w + BOARD / 2.0, HEIGHT / 2.0)),
        material=walnut,
        name="side_panel_1",
    )
    carcass.visual(
        Box((DEPTH, WIDTH, BOARD)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - BOARD / 2.0)),
        material=walnut_dark,
        name="top_board",
    )
    carcass.visual(
        Box((DEPTH, WIDTH, BOARD)),
        origin=Origin(xyz=(0.0, 0.0, BOARD / 2.0)),
        material=walnut_dark,
        name="bottom_board",
    )
    carcass.visual(
        Box((BACK_THICKNESS, WIDTH, HEIGHT)),
        origin=Origin(xyz=(-half_d + BACK_THICKNESS / 2.0, 0.0, HEIGHT / 2.0)),
        material=walnut_dark,
        name="back_panel",
    )

    # A proud face frame around the stack of drawer fronts.
    stile_width = 0.080
    front_x = half_d - FRONT_RAIL_DEPTH / 2.0
    carcass.visual(
        Box((FRONT_RAIL_DEPTH, stile_width, HEIGHT)),
        origin=Origin(xyz=(front_x, half_w - stile_width / 2.0, HEIGHT / 2.0)),
        material=walnut_dark,
        name="front_stile_0",
    )
    carcass.visual(
        Box((FRONT_RAIL_DEPTH, stile_width, HEIGHT)),
        origin=Origin(xyz=(front_x, -half_w + stile_width / 2.0, HEIGHT / 2.0)),
        material=walnut_dark,
        name="front_stile_1",
    )
    for rail_index in range(DRAWER_COUNT + 1):
        if rail_index == 0:
            rail_z = DRAWER_BOTTOM_MARGIN / 2.0
            rail_h = DRAWER_BOTTOM_MARGIN
        elif rail_index == DRAWER_COUNT:
            rail_z = HEIGHT - DRAWER_BOTTOM_MARGIN / 2.0
            rail_h = DRAWER_BOTTOM_MARGIN
        else:
            lower_top = _drawer_center_z(rail_index - 1) + DRAWER_FACE_HEIGHT / 2.0
            upper_bottom = _drawer_center_z(rail_index) - DRAWER_FACE_HEIGHT / 2.0
            rail_z = (lower_top + upper_bottom) / 2.0
            rail_h = upper_bottom - lower_top
        carcass.visual(
            Box((FRONT_RAIL_DEPTH, WIDTH, rail_h)),
            origin=Origin(xyz=(front_x, 0.0, rail_z)),
            material=walnut_dark,
            name=f"front_rail_{rail_index}",
        )

    # Fixed metal guide rails mounted to both side panels for every drawer bay.
    fixed_rail_length = 0.500
    fixed_rail_y = half_w - BOARD - 0.0175
    fixed_rail_z_offset = -0.052
    for drawer_index in range(DRAWER_COUNT):
        z = _drawer_center_z(drawer_index) + fixed_rail_z_offset
        for side_index, sign in enumerate((1.0, -1.0)):
            carcass.visual(
                Box((fixed_rail_length, 0.035, 0.024)),
                origin=Origin(xyz=(-0.040, sign * fixed_rail_y, z)),
                material=rail_metal,
                name=f"guide_rail_{drawer_index}_{side_index}",
            )

    drawer_origin_x = half_d + 0.004 + DRAWER_FRONT_THICKNESS / 2.0
    body_center_x = -DRAWER_FRONT_THICKNESS / 2.0 - DRAWER_BODY_DEPTH / 2.0
    runner_y = fixed_rail_y - 0.0175 - 0.013
    runner_z = -0.052
    for drawer_index in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{drawer_index}")
        z = _drawer_center_z(drawer_index)

        drawer.visual(
            Box((DRAWER_FRONT_THICKNESS, DRAWER_FACE_WIDTH, DRAWER_FACE_HEIGHT)),
            origin=Origin(),
            material=walnut_light,
            name="front_panel",
        )
        # Raised rails on the drawer face give each front a manufactured wooden panel.
        trim_depth = 0.009
        trim_x = DRAWER_FRONT_THICKNESS / 2.0 + trim_depth / 2.0 - 0.001
        trim_margin_y = 0.055
        trim_margin_z = 0.025
        drawer.visual(
            Box((trim_depth, DRAWER_FACE_WIDTH - 2.0 * trim_margin_y, 0.016)),
            origin=Origin(xyz=(trim_x, 0.0, DRAWER_FACE_HEIGHT / 2.0 - trim_margin_z)),
            material=walnut_dark,
            name="top_trim",
        )
        drawer.visual(
            Box((trim_depth, DRAWER_FACE_WIDTH - 2.0 * trim_margin_y, 0.016)),
            origin=Origin(xyz=(trim_x, 0.0, -DRAWER_FACE_HEIGHT / 2.0 + trim_margin_z)),
            material=walnut_dark,
            name="bottom_trim",
        )
        drawer.visual(
            Box((trim_depth, 0.016, DRAWER_FACE_HEIGHT - 2.0 * trim_margin_z)),
            origin=Origin(xyz=(trim_x, DRAWER_FACE_WIDTH / 2.0 - trim_margin_y, 0.0)),
            material=walnut_dark,
            name="side_trim_0",
        )
        drawer.visual(
            Box((trim_depth, 0.016, DRAWER_FACE_HEIGHT - 2.0 * trim_margin_z)),
            origin=Origin(xyz=(trim_x, -DRAWER_FACE_WIDTH / 2.0 + trim_margin_y, 0.0)),
            material=walnut_dark,
            name="side_trim_1",
        )

        # Real drawer box behind the front, with sides, bottom, and back.
        drawer.visual(
            Box((DRAWER_BODY_DEPTH, 0.020, DRAWER_BODY_HEIGHT)),
            origin=Origin(
                xyz=(body_center_x, DRAWER_BODY_WIDTH / 2.0 - 0.010, -0.006)
            ),
            material=walnut,
            name="box_side_0",
        )
        drawer.visual(
            Box((DRAWER_BODY_DEPTH, 0.020, DRAWER_BODY_HEIGHT)),
            origin=Origin(
                xyz=(body_center_x, -DRAWER_BODY_WIDTH / 2.0 + 0.010, -0.006)
            ),
            material=walnut,
            name="box_side_1",
        )
        drawer.visual(
            Box((DRAWER_BODY_DEPTH, DRAWER_BODY_WIDTH, 0.018)),
            origin=Origin(xyz=(body_center_x, 0.0, -DRAWER_BODY_HEIGHT / 2.0)),
            material=walnut_dark,
            name="box_bottom",
        )
        drawer.visual(
            Box((0.020, DRAWER_BODY_WIDTH, DRAWER_BODY_HEIGHT)),
            origin=Origin(
                xyz=(-DRAWER_FRONT_THICKNESS / 2.0 - DRAWER_BODY_DEPTH + 0.010, 0.0, -0.006)
            ),
            material=walnut,
            name="box_back",
        )
        # Mating slide runners attach to the drawer sides and touch the fixed guides.
        drawer.visual(
            Box((DRAWER_BODY_DEPTH, 0.026, 0.022)),
            origin=Origin(xyz=(body_center_x, runner_y, runner_z)),
            material=rail_metal,
            name="runner_0",
        )
        drawer.visual(
            Box((DRAWER_BODY_DEPTH, 0.026, 0.022)),
            origin=Origin(xyz=(body_center_x, -runner_y, runner_z)),
            material=rail_metal,
            name="runner_1",
        )

        # Round knob: a short stem plus a spherical wooden/brass pull.
        drawer.visual(
            Cylinder(radius=0.014, length=0.036),
            origin=Origin(
                xyz=(DRAWER_FRONT_THICKNESS / 2.0 + 0.018, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name="knob_stem",
        )
        drawer.visual(
            Sphere(radius=0.037),
            origin=Origin(xyz=(DRAWER_FRONT_THICKNESS / 2.0 + 0.055, 0.0, 0.0)),
            material=brass,
            name="knob_round",
        )

        model.articulation(
            f"slide_{drawer_index}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(drawer_origin_x, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    slide_joints = [object_model.get_articulation(f"slide_{i}") for i in range(DRAWER_COUNT)]
    ctx.check(
        "five prismatic drawer slides",
        len(slide_joints) == DRAWER_COUNT
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in slide_joints),
        details=f"joints={[j.name for j in slide_joints]}",
    )

    for i, joint in enumerate(slide_joints):
        drawer = object_model.get_part(f"drawer_{i}")
        ctx.expect_gap(
            drawer,
            carcass,
            axis="x",
            positive_elem="front_panel",
            max_gap=0.006,
            max_penetration=0.0,
            name=f"drawer_{i} front sits just proud of carcass",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="x",
            elem_a="runner_0",
            elem_b=f"guide_rail_{i}_0",
            min_overlap=0.35,
            name=f"drawer_{i} closed runner is carried by guide rail",
        )
        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({joint: DRAWER_TRAVEL}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="x",
                elem_a="runner_0",
                elem_b=f"guide_rail_{i}_0",
                min_overlap=0.070,
                name=f"drawer_{i} extended runner remains retained",
            )
            extended_position = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{i} slides outward",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.25,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


object_model = build_object_model()
