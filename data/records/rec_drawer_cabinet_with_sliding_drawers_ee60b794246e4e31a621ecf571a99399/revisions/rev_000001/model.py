from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CASE_WIDTH = 1.20
CASE_DEPTH = 0.64
CASE_HEIGHT = 0.86
COUNTER_WIDTH = 1.28
COUNTER_DEPTH = 0.74
COUNTER_THICKNESS = 0.04

PANEL_THICKNESS = 0.020
WALL_THICKNESS = 0.018
BACK_THICKNESS = 0.006
DIVIDER_THICKNESS = 0.018
BASE_PLINTH_HEIGHT = 0.10
BASE_PLINTH_WIDTH = 1.10
BASE_PLINTH_DEPTH = 0.48
BASE_PLINTH_Y = -0.04

SIDE_CADDY_WIDTH = 0.150

LEFT_INNER_X = -CASE_WIDTH / 2.0 + WALL_THICKNESS
DIVIDER_X = CASE_WIDTH / 2.0 - SIDE_CADDY_WIDTH - DIVIDER_THICKNESS / 2.0
DIVIDER_LEFT_X = DIVIDER_X - DIVIDER_THICKNESS / 2.0
DIVIDER_RIGHT_X = DIVIDER_X + DIVIDER_THICKNESS / 2.0
DRAWER_BAY_CENTER_X = (LEFT_INNER_X + DIVIDER_LEFT_X) / 2.0
DRAWER_OPENING_WIDTH = DIVIDER_LEFT_X - LEFT_INNER_X

DRAWER_FRONT_HEIGHT = 0.212
DRAWER_FACE_GAP = 0.018
DRAWER_FRONT_PROUD = 0.012
DRAWER_FRONT_WIDTH = DRAWER_OPENING_WIDTH - 0.008
DRAWER_BOX_WIDTH = 0.970
DRAWER_BOX_DEPTH = 0.520
DRAWER_BOX_HEIGHT = 0.155
DRAWER_BOTTOM_THICKNESS = 0.010
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_HANDLE_WIDTH = 0.44
DRAWER_HANDLE_HEIGHT = 0.016
DRAWER_HANDLE_PROJECTION = 0.012
DRAWER_RUNNER_LENGTH = 0.460
DRAWER_RUNNER_THICKNESS = 0.008
DRAWER_RUNNER_HEIGHT = 0.012
DRAWER_RUNNER_LOCAL_X = DRAWER_OPENING_WIDTH / 2.0 - DRAWER_RUNNER_THICKNESS / 2.0
DRAWER_RUNNER_LOCAL_Y = -0.250
DRAWER_RUNNER_LOCAL_Z = -0.046
DRAWER_TRAVEL = 0.38

DRAWER_Z = {
    "bottom": 0.249,
    "middle": 0.479,
    "top": 0.709,
}

GUIDE_LENGTH = 0.520
GUIDE_THICKNESS = 0.010
GUIDE_HEIGHT = 0.018
GUIDE_CENTER_Y = 0.070
GUIDE_LEFT_X = LEFT_INNER_X + GUIDE_THICKNESS / 2.0
GUIDE_RIGHT_X = DIVIDER_LEFT_X - GUIDE_THICKNESS / 2.0

SHELF_CENTER_Z = 0.48
SHELF_FACE_HEIGHT = 0.56
SHELF_FACE_LENGTH = 0.44
SHELF_FACE_PROUD = 0.012
SHELF_BODY_WIDTH = 0.156
SHELF_WALL_THICKNESS = 0.012
SHELF_BOTTOM_THICKNESS = 0.012
SHELF_RUNNER_LENGTH = 0.130
SHELF_RUNNER_THICKNESS = 0.010
SHELF_RUNNER_HEIGHT = 0.018
SHELF_RUNNER_LOCAL_X = -0.067
SHELF_RUNNER_LOCAL_Y = 0.15
SHELF_TRAVEL = 0.075

SHELF_GUIDE_LENGTH = 0.130
SHELF_GUIDE_THICKNESS = 0.014
SHELF_GUIDE_HEIGHT = 0.022
SHELF_GUIDE_CENTER_X = 0.535
SHELF_GUIDE_CENTER_Y = 0.162


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _build_drawer(model: ArticulatedObject, part_name: str, front_finish, box_finish, rail_finish):
    drawer = model.part(part_name)

    _add_box(
        drawer,
        (DRAWER_FRONT_WIDTH, PANEL_THICKNESS, DRAWER_FRONT_HEIGHT),
        (0.0, PANEL_THICKNESS / 2.0 + DRAWER_FRONT_PROUD, 0.0),
        front_finish,
        "front_panel",
    )
    _add_box(
        drawer,
        (DRAWER_HANDLE_WIDTH, DRAWER_HANDLE_PROJECTION, DRAWER_HANDLE_HEIGHT),
        (
            0.0,
            PANEL_THICKNESS + DRAWER_FRONT_PROUD + DRAWER_HANDLE_PROJECTION / 2.0 - 0.002,
            DRAWER_FRONT_HEIGHT * 0.16,
        ),
        rail_finish,
        "handle",
    )
    _add_box(
        drawer,
        (0.016, 0.010, 0.050),
        (
            -DRAWER_HANDLE_WIDTH * 0.32,
            PANEL_THICKNESS + DRAWER_FRONT_PROUD + 0.003,
            DRAWER_FRONT_HEIGHT * 0.16,
        ),
        rail_finish,
        "handle_post_0",
    )
    _add_box(
        drawer,
        (0.016, 0.010, 0.050),
        (
            DRAWER_HANDLE_WIDTH * 0.32,
            PANEL_THICKNESS + DRAWER_FRONT_PROUD + 0.003,
            DRAWER_FRONT_HEIGHT * 0.16,
        ),
        rail_finish,
        "handle_post_1",
    )

    box_center_y = DRAWER_FRONT_PROUD - DRAWER_BOX_DEPTH / 2.0
    box_center_z = -0.020

    _add_box(
        drawer,
        (DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS),
        (
            0.0,
            box_center_y,
            box_center_z - DRAWER_BOX_HEIGHT / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0,
        ),
        box_finish,
        "bottom_panel",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT),
        (-DRAWER_BOX_WIDTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0, box_center_y, box_center_z),
        box_finish,
        "left_wall",
    )
    _add_box(
        drawer,
        (DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT),
        (DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0, box_center_y, box_center_z),
        box_finish,
        "right_wall",
    )
    _add_box(
        drawer,
        (DRAWER_BOX_WIDTH, DRAWER_SIDE_THICKNESS, DRAWER_BOX_HEIGHT),
        (
            0.0,
            box_center_y - DRAWER_BOX_DEPTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0,
            box_center_z,
        ),
        box_finish,
        "rear_wall",
    )
    _add_box(
        drawer,
        (0.020, 0.100, 0.024),
        (
            DRAWER_BOX_WIDTH / 2.0 + 0.005,
            box_center_y - DRAWER_BOX_DEPTH / 2.0 + 0.070,
            DRAWER_RUNNER_LOCAL_Z + 0.018,
        ),
        box_finish,
        "runner_mount_right",
    )
    _add_box(
        drawer,
        (0.020, 0.100, 0.024),
        (
            -(DRAWER_BOX_WIDTH / 2.0 + 0.005),
            box_center_y - DRAWER_BOX_DEPTH / 2.0 + 0.070,
            DRAWER_RUNNER_LOCAL_Z + 0.018,
        ),
        box_finish,
        "runner_mount_left",
    )

    _add_box(
        drawer,
        (DRAWER_RUNNER_THICKNESS, DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_HEIGHT),
        (-DRAWER_RUNNER_LOCAL_X, DRAWER_RUNNER_LOCAL_Y, DRAWER_RUNNER_LOCAL_Z),
        rail_finish,
        "runner_left",
    )
    _add_box(
        drawer,
        (DRAWER_RUNNER_THICKNESS, DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_HEIGHT),
        (DRAWER_RUNNER_LOCAL_X, DRAWER_RUNNER_LOCAL_Y, DRAWER_RUNNER_LOCAL_Z),
        rail_finish,
        "runner_right",
    )

    return drawer


def _build_utensil_shelf(
    model: ArticulatedObject,
    front_finish,
    box_finish,
    rail_finish,
):
    shelf = model.part("utensil_shelf")

    _add_box(
        shelf,
        (PANEL_THICKNESS, SHELF_FACE_LENGTH, SHELF_FACE_HEIGHT),
        (PANEL_THICKNESS / 2.0 + SHELF_FACE_PROUD, 0.0, 0.0),
        front_finish,
        "side_panel",
    )
    _add_box(
        shelf,
        (SHELF_BODY_WIDTH, SHELF_FACE_LENGTH - 0.02, SHELF_BOTTOM_THICKNESS),
        (
            SHELF_FACE_PROUD - SHELF_BODY_WIDTH / 2.0,
            0.0,
            -SHELF_FACE_HEIGHT / 2.0 + SHELF_BOTTOM_THICKNESS / 2.0,
        ),
        box_finish,
        "bottom_panel",
    )
    _add_box(
        shelf,
        (SHELF_WALL_THICKNESS, 0.120, SHELF_FACE_HEIGHT - 0.02),
        (
            -SHELF_BODY_WIDTH + SHELF_FACE_PROUD + SHELF_WALL_THICKNESS / 2.0,
            0.0,
            -0.01,
        ),
        box_finish,
        "inner_wall",
    )
    _add_box(
        shelf,
        (SHELF_BODY_WIDTH, SHELF_WALL_THICKNESS, SHELF_FACE_HEIGHT - 0.02),
        (
            SHELF_FACE_PROUD - SHELF_BODY_WIDTH / 2.0,
            SHELF_FACE_LENGTH / 2.0 - SHELF_WALL_THICKNESS / 2.0 - 0.01,
            -0.01,
        ),
        box_finish,
        "front_rail",
    )
    _add_box(
        shelf,
        (SHELF_BODY_WIDTH, SHELF_WALL_THICKNESS, SHELF_FACE_HEIGHT - 0.02),
        (
            SHELF_FACE_PROUD - SHELF_BODY_WIDTH / 2.0,
            -SHELF_FACE_LENGTH / 2.0 + SHELF_WALL_THICKNESS / 2.0 + 0.01,
            -0.01,
        ),
        box_finish,
        "rear_rail",
    )
    _add_box(
        shelf,
        (SHELF_RUNNER_LENGTH, SHELF_RUNNER_THICKNESS, SHELF_RUNNER_HEIGHT),
        (SHELF_RUNNER_LOCAL_X, SHELF_RUNNER_LOCAL_Y, 0.0),
        rail_finish,
        "runner_front",
    )
    _add_box(
        shelf,
        (0.012, 0.085, 0.050),
        (-SHELF_BODY_WIDTH + SHELF_FACE_PROUD + 0.018, 0.1025, 0.0),
        box_finish,
        "runner_front_support",
    )
    _add_box(
        shelf,
        (SHELF_RUNNER_LENGTH, SHELF_RUNNER_THICKNESS, SHELF_RUNNER_HEIGHT),
        (SHELF_RUNNER_LOCAL_X, -SHELF_RUNNER_LOCAL_Y, 0.0),
        rail_finish,
        "runner_rear",
    )
    _add_box(
        shelf,
        (0.012, 0.085, 0.050),
        (-SHELF_BODY_WIDTH + SHELF_FACE_PROUD + 0.018, -0.1025, 0.0),
        box_finish,
        "runner_rear_support",
    )

    return shelf


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_island_drawer_cabinet")

    painted_case = model.material("painted_case", rgba=(0.93, 0.94, 0.92, 1.0))
    drawer_front = model.material("drawer_front", rgba=(0.96, 0.96, 0.95, 1.0))
    drawer_box = model.material("drawer_box", rgba=(0.86, 0.83, 0.77, 1.0))
    runner_metal = model.material("runner_metal", rgba=(0.70, 0.73, 0.76, 1.0))
    plinth_finish = model.material("plinth_finish", rgba=(0.80, 0.81, 0.79, 1.0))
    countertop = model.material("countertop", rgba=(0.74, 0.76, 0.78, 1.0))

    carcass = model.part("carcass")
    _add_box(
        carcass,
        (BASE_PLINTH_WIDTH, BASE_PLINTH_DEPTH, BASE_PLINTH_HEIGHT),
        (0.0, BASE_PLINTH_Y, BASE_PLINTH_HEIGHT / 2.0),
        plinth_finish,
        "plinth",
    )
    _add_box(
        carcass,
        (CASE_WIDTH, CASE_DEPTH, WALL_THICKNESS),
        (0.0, 0.0, BASE_PLINTH_HEIGHT + WALL_THICKNESS / 2.0),
        painted_case,
        "bottom_panel",
    )
    _add_box(
        carcass,
        (CASE_WIDTH, CASE_DEPTH, WALL_THICKNESS),
        (0.0, 0.0, CASE_HEIGHT - WALL_THICKNESS / 2.0),
        painted_case,
        "top_panel",
    )
    _add_box(
        carcass,
        (WALL_THICKNESS, CASE_DEPTH, CASE_HEIGHT),
        (-CASE_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, CASE_HEIGHT / 2.0),
        painted_case,
        "left_side",
    )
    _add_box(
        carcass,
        (DIVIDER_THICKNESS, CASE_DEPTH, CASE_HEIGHT),
        (DIVIDER_X, 0.0, CASE_HEIGHT / 2.0),
        painted_case,
        "divider",
    )
    _add_box(
        carcass,
        (CASE_WIDTH, BACK_THICKNESS, CASE_HEIGHT),
        (0.0, -CASE_DEPTH / 2.0 + BACK_THICKNESS / 2.0, CASE_HEIGHT / 2.0),
        painted_case,
        "back_panel",
    )
    _add_box(
        carcass,
        (WALL_THICKNESS, 0.095, CASE_HEIGHT),
        (CASE_WIDTH / 2.0 - WALL_THICKNESS / 2.0, CASE_DEPTH / 2.0 - 0.095 / 2.0, CASE_HEIGHT / 2.0),
        painted_case,
        "side_front_stile",
    )
    _add_box(
        carcass,
        (WALL_THICKNESS, 0.095, CASE_HEIGHT),
        (CASE_WIDTH / 2.0 - WALL_THICKNESS / 2.0, -CASE_DEPTH / 2.0 + 0.095 / 2.0, CASE_HEIGHT / 2.0),
        painted_case,
        "side_rear_stile",
    )
    _add_box(
        carcass,
        (DRAWER_OPENING_WIDTH, WALL_THICKNESS, WALL_THICKNESS),
        (DRAWER_BAY_CENTER_X, CASE_DEPTH / 2.0 - 0.029, (DRAWER_Z["bottom"] + DRAWER_Z["middle"]) / 2.0),
        painted_case,
        "lower_separator",
    )
    _add_box(
        carcass,
        (DRAWER_OPENING_WIDTH, WALL_THICKNESS, WALL_THICKNESS),
        (DRAWER_BAY_CENTER_X, CASE_DEPTH / 2.0 - 0.029, (DRAWER_Z["middle"] + DRAWER_Z["top"]) / 2.0),
        painted_case,
        "upper_separator",
    )
    _add_box(
        carcass,
        (COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS),
        (0.0, 0.0, CASE_HEIGHT + COUNTER_THICKNESS / 2.0),
        countertop,
        "countertop",
    )

    for label, z_center in DRAWER_Z.items():
        _add_box(
            carcass,
            (GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT),
            (GUIDE_LEFT_X, GUIDE_CENTER_Y, z_center + DRAWER_RUNNER_LOCAL_Z),
            runner_metal,
            f"{label}_guide_left",
        )
        _add_box(
            carcass,
            (GUIDE_THICKNESS, GUIDE_LENGTH, GUIDE_HEIGHT),
            (GUIDE_RIGHT_X, GUIDE_CENTER_Y, z_center + DRAWER_RUNNER_LOCAL_Z),
            runner_metal,
            f"{label}_guide_right",
        )

    _add_box(
        carcass,
        (SHELF_GUIDE_LENGTH, SHELF_GUIDE_THICKNESS, SHELF_GUIDE_HEIGHT),
        (SHELF_GUIDE_CENTER_X, SHELF_GUIDE_CENTER_Y, SHELF_CENTER_Z),
        runner_metal,
        "shelf_guide_front",
    )
    _add_box(
        carcass,
        (SHELF_GUIDE_LENGTH, SHELF_GUIDE_THICKNESS, SHELF_GUIDE_HEIGHT),
        (SHELF_GUIDE_CENTER_X, -SHELF_GUIDE_CENTER_Y, SHELF_CENTER_Z),
        runner_metal,
        "shelf_guide_rear",
    )
    _add_box(
        carcass,
        (0.020, 0.030, 0.030),
        (DIVIDER_RIGHT_X + 0.010, SHELF_GUIDE_CENTER_Y, SHELF_CENTER_Z),
        painted_case,
        "shelf_guide_front_bracket",
    )
    _add_box(
        carcass,
        (0.020, 0.030, 0.030),
        (DIVIDER_RIGHT_X + 0.010, -SHELF_GUIDE_CENTER_Y, SHELF_CENTER_Z),
        painted_case,
        "shelf_guide_rear_bracket",
    )

    for label in ("bottom", "middle", "top"):
        drawer = _build_drawer(model, f"{label}_drawer", drawer_front, drawer_box, runner_metal)
        model.articulation(
            f"carcass_to_{label}_drawer",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(DRAWER_BAY_CENTER_X, CASE_DEPTH / 2.0, DRAWER_Z[label])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    utensil_shelf = _build_utensil_shelf(model, drawer_front, drawer_box, runner_metal)
    model.articulation(
        "carcass_to_utensil_shelf",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=utensil_shelf,
        origin=Origin(xyz=(CASE_WIDTH / 2.0, 0.0, SHELF_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.25,
            lower=0.0,
            upper=SHELF_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    for label in ("bottom", "middle", "top"):
        drawer = object_model.get_part(f"{label}_drawer")
        slide = object_model.get_articulation(f"carcass_to_{label}_drawer")

        ctx.allow_overlap(
            drawer,
            carcass,
            elem_a="runner_left",
            elem_b=f"{label}_guide_left",
            reason="The drawer slide is represented as a runner nested inside a guide rail proxy.",
        )
        ctx.allow_overlap(
            drawer,
            carcass,
            elem_a="runner_right",
            elem_b=f"{label}_guide_right",
            reason="The drawer slide is represented as a runner nested inside a guide rail proxy.",
        )

        ctx.expect_overlap(
            drawer,
            carcass,
            axes="y",
            elem_a="runner_left",
            elem_b=f"{label}_guide_left",
            min_overlap=0.42,
            name=f"{label} drawer left runner stays engaged at rest",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="y",
            elem_a="runner_right",
            elem_b=f"{label}_guide_right",
            min_overlap=0.42,
            name=f"{label} drawer right runner stays engaged at rest",
        )

        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({slide: DRAWER_TRAVEL}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="runner_left",
                elem_b=f"{label}_guide_left",
                min_overlap=0.09,
                name=f"{label} drawer left runner retains insertion when open",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="runner_right",
                elem_b=f"{label}_guide_right",
                min_overlap=0.09,
                name=f"{label} drawer right runner retains insertion when open",
            )
            extended_position = ctx.part_world_position(drawer)

        ctx.check(
            f"{label} drawer extends toward the front",
            rest_position is not None
            and extended_position is not None
            and extended_position[1] > rest_position[1] + 0.15,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    shelf = object_model.get_part("utensil_shelf")
    shelf_slide = object_model.get_articulation("carcass_to_utensil_shelf")

    ctx.expect_overlap(
        shelf,
        carcass,
        axes="x",
        elem_a="runner_front",
        elem_b="shelf_guide_front",
        min_overlap=0.12,
        name="utensil shelf front runner stays engaged at rest",
    )
    ctx.expect_overlap(
        shelf,
        carcass,
        axes="x",
        elem_a="runner_rear",
        elem_b="shelf_guide_rear",
        min_overlap=0.12,
        name="utensil shelf rear runner stays engaged at rest",
    )

    shelf_rest = ctx.part_world_position(shelf)
    with ctx.pose({shelf_slide: SHELF_TRAVEL}):
        ctx.expect_overlap(
            shelf,
            carcass,
            axes="x",
            elem_a="runner_front",
            elem_b="shelf_guide_front",
            min_overlap=0.05,
            name="utensil shelf front runner retains insertion when open",
        )
        ctx.expect_overlap(
            shelf,
            carcass,
            axes="x",
            elem_a="runner_rear",
            elem_b="shelf_guide_rear",
            min_overlap=0.05,
            name="utensil shelf rear runner retains insertion when open",
        )
        shelf_extended = ctx.part_world_position(shelf)

    ctx.check(
        "utensil shelf extends toward the cabinet side",
        shelf_rest is not None
        and shelf_extended is not None
        and shelf_extended[0] > shelf_rest[0] + 0.06,
        details=f"rest={shelf_rest}, extended={shelf_extended}",
    )

    return ctx.report()


object_model = build_object_model()
