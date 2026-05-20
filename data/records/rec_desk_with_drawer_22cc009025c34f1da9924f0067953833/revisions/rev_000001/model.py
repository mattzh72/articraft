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
)


DESK_WIDTH = 1.08
DESK_DEPTH = 0.52
TOP_THICKNESS = 0.028
DESK_HEIGHT = 0.76
LEG_SIZE = 0.045
TOP_UNDERSIDE_Z = DESK_HEIGHT - TOP_THICKNESS

FRAME_FRONT_FACE_Y = 0.245
FRAME_DEPTH = 0.022
FRAME_BACK_FACE_Y = FRAME_FRONT_FACE_Y - FRAME_DEPTH
APRON_BOTTOM_Z = 0.620
APRON_TOP_RAIL_HEIGHT = 0.022
APRON_BOTTOM_RAIL_HEIGHT = 0.020
DRAWER_OPENING_BOTTOM_Z = APRON_BOTTOM_Z + APRON_BOTTOM_RAIL_HEIGHT
DRAWER_OPENING_TOP_Z = TOP_UNDERSIDE_Z - APRON_TOP_RAIL_HEIGHT
DRAWER_OPENING_HEIGHT = DRAWER_OPENING_TOP_Z - DRAWER_OPENING_BOTTOM_Z
PARTITION_DEPTH = 0.330
PARTITION_BACK_FACE_Y = FRAME_BACK_FACE_Y - PARTITION_DEPTH
BACK_SUPPORT_DEPTH = 0.018
BACK_SUPPORT_FRONT_FACE_Y = PARTITION_BACK_FACE_Y
BACK_SUPPORT_CENTER_Y = BACK_SUPPORT_FRONT_FACE_Y - BACK_SUPPORT_DEPTH / 2.0
RUNNER_DEPTH = FRAME_BACK_FACE_Y - PARTITION_BACK_FACE_Y
RUNNER_CENTER_Y = (FRAME_BACK_FACE_Y + PARTITION_BACK_FACE_Y) / 2.0

LEG_CENTER_X = 0.4675
LEG_CENTER_Y = 0.2225
SIDE_APRON_HEIGHT = 0.090
SIDE_APRON_CENTER_Z = TOP_UNDERSIDE_Z - SIDE_APRON_HEIGHT / 2.0

LEFT_DRAWER_X = -0.317
RIGHT_DRAWER_X = 0.317
CENTER_DRAWER_X = 0.0

SIDE_FRONT_WIDTH = 0.192
SIDE_BODY_WIDTH = 0.182
SIDE_BODY_OFFSET_X = 0.008
CENTER_FRONT_WIDTH = 0.386
CENTER_BODY_WIDTH = 0.378

DRAWER_FRONT_THICKNESS = 0.020
DRAWER_BODY_DEPTH = 0.320
DRAWER_HEIGHT = 0.056
DRAWER_FRONT_HEIGHT = 0.066
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.008
DRAWER_BACK_THICKNESS = 0.010
DRAWER_TRAVEL = 0.180


def add_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    joint_name: str,
    joint_x: float,
    front_width: float,
    body_width: float,
    body_offset_x: float,
    handle_span: float,
    material: str,
    pull_material: str,
) -> None:
    drawer = model.part(name)

    drawer.visual(
        Box((front_width, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -DRAWER_FRONT_THICKNESS / 2.0, DRAWER_FRONT_HEIGHT / 2.0)
        ),
        material=material,
        name="front",
    )

    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_HEIGHT)),
        origin=Origin(
            xyz=(
                body_offset_x - body_width / 2.0 + DRAWER_SIDE_THICKNESS / 2.0,
                -(DRAWER_FRONT_THICKNESS + DRAWER_BODY_DEPTH / 2.0),
                DRAWER_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="side_0",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BODY_DEPTH, DRAWER_HEIGHT)),
        origin=Origin(
            xyz=(
                body_offset_x + body_width / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
                -(DRAWER_FRONT_THICKNESS + DRAWER_BODY_DEPTH / 2.0),
                DRAWER_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="side_1",
    )
    drawer.visual(
        Box(
            (
                body_width - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                body_offset_x,
                -(DRAWER_FRONT_THICKNESS + (DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS) / 2.0),
                DRAWER_BOTTOM_THICKNESS / 2.0,
            )
        ),
        material=material,
        name="bottom",
    )
    drawer.visual(
        Box((body_width, DRAWER_BACK_THICKNESS, DRAWER_HEIGHT - DRAWER_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                body_offset_x,
                -(DRAWER_FRONT_THICKNESS + DRAWER_BODY_DEPTH - DRAWER_BACK_THICKNESS / 2.0),
                DRAWER_BOTTOM_THICKNESS
                + (DRAWER_HEIGHT - DRAWER_BOTTOM_THICKNESS) / 2.0,
            )
        ),
        material=material,
        name="back",
    )

    post_offset = handle_span / 2.0
    for index, x in enumerate((-post_offset, post_offset)):
        drawer.visual(
            Box((0.010, 0.020, 0.010)),
            origin=Origin(xyz=(x, 0.010, 0.036)),
            material=pull_material,
            name=f"pull_post_{index}",
        )
    drawer.visual(
        Cylinder(radius=0.004, length=handle_span),
        origin=Origin(
            xyz=(0.0, 0.022, 0.036),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pull_material,
        name="pull_bar",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent="desk",
        child=drawer,
        origin=Origin(xyz=(joint_x, FRAME_FRONT_FACE_Y, DRAWER_OPENING_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="writing_desk")

    walnut = model.material("walnut", rgba=(0.49, 0.32, 0.20, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.38, 0.24, 0.15, 1.0))
    runner_wood = model.material("runner_wood", rgba=(0.32, 0.21, 0.13, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.64, 0.39, 1.0))

    desk = model.part("desk")
    desk.visual(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0)),
        material=walnut,
        name="top",
    )

    for x_index, x_sign in enumerate((-1.0, 1.0)):
        for y_index, y_sign in enumerate((-1.0, 1.0)):
            desk.visual(
                Box((LEG_SIZE, LEG_SIZE, TOP_UNDERSIDE_Z)),
                origin=Origin(
                    xyz=(
                        x_sign * LEG_CENTER_X,
                        y_sign * LEG_CENTER_Y,
                        TOP_UNDERSIDE_Z / 2.0,
                    )
                ),
                material=dark_walnut,
                name=f"leg_{x_index}_{y_index}",
            )

    desk.visual(
        Box((0.890, FRAME_DEPTH, APRON_TOP_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_FRONT_FACE_Y - FRAME_DEPTH / 2.0,
                TOP_UNDERSIDE_Z - APRON_TOP_RAIL_HEIGHT / 2.0,
            )
        ),
        material=dark_walnut,
        name="front_top_rail",
    )
    desk.visual(
        Box((0.890, FRAME_DEPTH, APRON_BOTTOM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRAME_FRONT_FACE_Y - FRAME_DEPTH / 2.0,
                APRON_BOTTOM_Z + APRON_BOTTOM_RAIL_HEIGHT / 2.0,
            )
        ),
        material=dark_walnut,
        name="front_bottom_rail",
    )

    front_frame_members = (
        ("side_stile_0", -0.431, 0.028),
        ("divider_0", -0.207, 0.020),
        ("divider_1", 0.207, 0.020),
        ("side_stile_1", 0.431, 0.028),
    )
    for name, x_center, width in front_frame_members:
        desk.visual(
            Box((width, FRAME_DEPTH, DRAWER_OPENING_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_center,
                    FRAME_FRONT_FACE_Y - FRAME_DEPTH / 2.0,
                    DRAWER_OPENING_BOTTOM_Z + DRAWER_OPENING_HEIGHT / 2.0,
                )
            ),
            material=dark_walnut,
            name=name,
        )

    for name, x_center, thickness in (
        ("guide_0", -0.411, 0.012),
        ("partition_0", -0.207, 0.016),
        ("partition_1", 0.207, 0.016),
        ("guide_1", 0.411, 0.012),
    ):
        desk.visual(
            Box((thickness, PARTITION_DEPTH, TOP_UNDERSIDE_Z - APRON_BOTTOM_Z)),
            origin=Origin(
                xyz=(
                    x_center,
                    FRAME_BACK_FACE_Y - PARTITION_DEPTH / 2.0,
                    APRON_BOTTOM_Z + (TOP_UNDERSIDE_Z - APRON_BOTTOM_Z) / 2.0,
                )
            ),
            material=dark_walnut,
            name=name,
        )

    desk.visual(
        Box((0.834, BACK_SUPPORT_DEPTH, 0.028)),
        origin=Origin(
            xyz=(0.0, BACK_SUPPORT_CENTER_Y, TOP_UNDERSIDE_Z - 0.014),
        ),
        material=dark_walnut,
        name="back_support",
    )

    desk.visual(
        Box((0.018, 0.400, SIDE_APRON_HEIGHT)),
        origin=Origin(xyz=(-0.436, 0.0, SIDE_APRON_CENTER_Z)),
        material=dark_walnut,
        name="side_apron_0",
    )
    desk.visual(
        Box((0.018, 0.400, SIDE_APRON_HEIGHT)),
        origin=Origin(xyz=(0.436, 0.0, SIDE_APRON_CENTER_Z)),
        material=dark_walnut,
        name="side_apron_1",
    )
    desk.visual(
        Box((0.872, 0.018, SIDE_APRON_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.236, SIDE_APRON_CENTER_Z)),
        material=dark_walnut,
        name="back_apron",
    )

    runner_specs = (
        ("side_0_runner_0", -0.365),
        ("side_0_runner_1", -0.253),
        ("center_runner_0", -0.132),
        ("center_runner_1", 0.132),
        ("side_1_runner_0", 0.253),
        ("side_1_runner_1", 0.365),
    )
    for name, x_center in runner_specs:
        desk.visual(
            Box((0.012, RUNNER_DEPTH, 0.010)),
            origin=Origin(xyz=(x_center, RUNNER_CENTER_Y, DRAWER_OPENING_BOTTOM_Z - 0.006)),
            material=runner_wood,
            name=name,
        )

    add_drawer(
        model,
        name="side_drawer_0",
        joint_name="desk_to_side_drawer_0",
        joint_x=LEFT_DRAWER_X,
        front_width=SIDE_FRONT_WIDTH,
        body_width=SIDE_BODY_WIDTH,
        body_offset_x=SIDE_BODY_OFFSET_X,
        handle_span=0.082,
        material=walnut.name,
        pull_material=brass.name,
    )
    add_drawer(
        model,
        name="center_drawer",
        joint_name="desk_to_center_drawer",
        joint_x=CENTER_DRAWER_X,
        front_width=CENTER_FRONT_WIDTH,
        body_width=CENTER_BODY_WIDTH,
        body_offset_x=0.0,
        handle_span=0.140,
        material=walnut.name,
        pull_material=brass.name,
    )
    add_drawer(
        model,
        name="side_drawer_1",
        joint_name="desk_to_side_drawer_1",
        joint_x=RIGHT_DRAWER_X,
        front_width=SIDE_FRONT_WIDTH,
        body_width=SIDE_BODY_WIDTH,
        body_offset_x=-SIDE_BODY_OFFSET_X,
        handle_span=0.082,
        material=walnut.name,
        pull_material=brass.name,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drawer_specs = (
        (
            object_model.get_part("side_drawer_0"),
            object_model.get_articulation("desk_to_side_drawer_0"),
            "side_0_runner_0",
        ),
        (
            object_model.get_part("center_drawer"),
            object_model.get_articulation("desk_to_center_drawer"),
            "center_runner_0",
        ),
        (
            object_model.get_part("side_drawer_1"),
            object_model.get_articulation("desk_to_side_drawer_1"),
            "side_1_runner_0",
        ),
    )

    for drawer, joint, runner in drawer_specs:
        rest_position = ctx.part_world_position(drawer)
        ctx.expect_gap(
            drawer,
            "desk",
            axis="z",
            positive_elem="bottom",
            negative_elem=runner,
            min_gap=0.0005,
            max_gap=0.0015,
            name=f"{drawer.name} rides just above its runner",
        )
        ctx.expect_overlap(
            drawer,
            "desk",
            axes="y",
            elem_a="bottom",
            elem_b=runner,
            min_overlap=0.300,
            name=f"{drawer.name} is deeply supported at rest",
        )

        with ctx.pose({joint: DRAWER_TRAVEL}):
            extended_position = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                "desk",
                axes="y",
                elem_a="bottom",
                elem_b=runner,
                min_overlap=0.120,
                name=f"{drawer.name} keeps retained insertion when extended",
            )
            ctx.check(
                f"{drawer.name} extends forward",
                rest_position is not None
                and extended_position is not None
                and extended_position[1] > rest_position[1] + 0.15,
                details=f"rest={rest_position}, extended={extended_position}",
            )

    return ctx.report()


object_model = build_object_model()
