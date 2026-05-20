from __future__ import annotations

from math import pi

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


BOARD_WIDTH = 0.280
BOARD_HEIGHT = 0.220
BOARD_THICKNESS = 0.008

ARM_WIDTH = 0.286
ARM_HEIGHT = 0.050
ARM_THICKNESS = 0.010

HINGE_WIDTH = 0.264
HINGE_RADIUS = 0.0048
HINGE_SEGMENT_GAP = 0.003
HINGE_SEGMENT_LENGTH = (HINGE_WIDTH - 4.0 * HINGE_SEGMENT_GAP) / 5.0
HINGE_AXIS_Y = 0.0082
HINGE_AXIS_Z = BOARD_HEIGHT
HALF_PI = pi / 2.0


def _hinge_segment_centers() -> list[float]:
    left = -HINGE_WIDTH / 2.0
    pitch = HINGE_SEGMENT_LENGTH + HINGE_SEGMENT_GAP
    return [left + HINGE_SEGMENT_LENGTH / 2.0 + i * pitch for i in range(5)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="film_clapperboard")
    hinge_segment_centers = _hinge_segment_centers()

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    slate_white = model.material("slate_white", rgba=(0.95, 0.95, 0.93, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.72, 0.74, 0.78, 1.0))
    red = model.material("red", rgba=(0.84, 0.22, 0.18, 1.0))
    yellow = model.material("yellow", rgba=(0.96, 0.82, 0.18, 1.0))
    blue = model.material("blue", rgba=(0.20, 0.46, 0.84, 1.0))
    green = model.material("green", rgba=(0.22, 0.68, 0.32, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_HEIGHT / 2.0)),
        material=matte_black,
        name="slate_body",
    )
    board.visual(
        Box((0.246, 0.0012, 0.164)),
        origin=Origin(xyz=(0.0, BOARD_THICKNESS / 2.0 + 0.0006, 0.096)),
        material=slate_white,
        name="writing_panel",
    )

    for index, z_pos in enumerate((0.145, 0.105, 0.065)):
        board.visual(
            Box((0.222, 0.0008, 0.003)),
            origin=Origin(xyz=(0.006, BOARD_THICKNESS / 2.0 + 0.0010, z_pos)),
            material=matte_black,
            name=f"rule_{index}",
        )
    for index, x_pos in enumerate((-0.060, 0.015, 0.088)):
        board.visual(
            Box((0.003, 0.0008, 0.120)),
            origin=Origin(xyz=(x_pos, BOARD_THICKNESS / 2.0 + 0.0010, 0.085)),
            material=matte_black,
            name=f"divider_{index}",
        )

    board.visual(
        Box((HINGE_WIDTH, 0.0018, 0.016)),
        origin=Origin(xyz=(0.0, BOARD_THICKNESS / 2.0 + 0.0009, BOARD_HEIGHT - 0.008)),
        material=hinge_metal,
        name="hinge_leaf",
    )

    for index in (0, 2, 4):
        board.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_SEGMENT_LENGTH),
            origin=Origin(
                xyz=(hinge_segment_centers[index], HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, HALF_PI, 0.0),
            ),
            material=hinge_metal,
            name=f"board_knuckle_{index}",
        )

    clapper = model.part("clapper")
    clapper.visual(
        Box((HINGE_WIDTH, 0.0036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0012, -0.007)),
        material=hinge_metal,
        name="arm_leaf",
    )
    clapper.visual(
        Box((ARM_WIDTH, ARM_THICKNESS, ARM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0037, -ARM_HEIGHT / 2.0)),
        material=matte_black,
        name="arm_body",
    )

    stripe_centers = (-0.100, -0.050, 0.000, 0.050, 0.100)
    for index, x_pos in enumerate(stripe_centers):
        clapper.visual(
            Box((0.058, 0.0016, 0.012)),
            origin=Origin(
                xyz=(x_pos, 0.0079, -0.025),
                rpy=(0.0, 0.78, 0.0),
            ),
            material=slate_white,
            name=f"stripe_{index}",
        )

    for index in (1, 3):
        clapper.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_SEGMENT_LENGTH),
            origin=Origin(
                xyz=(hinge_segment_centers[index], 0.0, 0.0),
                rpy=(0.0, HALF_PI, 0.0),
            ),
            material=hinge_metal,
            name=f"arm_knuckle_{index}",
        )

    model.articulation(
        "clapper_hinge",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapper,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=3.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapper = object_model.get_part("clapper")
    hinge = object_model.get_articulation("clapper_hinge")
    limits = hinge.motion_limits

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            clapper,
            board,
            axes="x",
            elem_a="arm_body",
            elem_b="slate_body",
            min_overlap=0.250,
            name="clapper spans the slate width",
        )
        ctx.expect_overlap(
            clapper,
            board,
            axes="z",
            elem_a="arm_body",
            elem_b="slate_body",
            min_overlap=0.040,
            name="closed clapper covers the top writing band",
        )
        ctx.expect_gap(
            clapper,
            board,
            axis="y",
            positive_elem="arm_body",
            negative_elem="hinge_leaf",
            min_gap=0.0005,
            max_gap=0.0040,
            name="closed clapper sits just proud of the hinge bracket",
        )
        closed_arm_aabb = ctx.part_element_world_aabb(clapper, elem="arm_body")

    open_arm_aabb = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.upper}):
            open_arm_aabb = ctx.part_element_world_aabb(clapper, elem="arm_body")

    ctx.check(
        "clapper swings forward",
        closed_arm_aabb is not None
        and open_arm_aabb is not None
        and open_arm_aabb[1][1] > closed_arm_aabb[1][1] + 0.030,
        details=f"closed={closed_arm_aabb}, open={open_arm_aabb}",
    )
    ctx.check(
        "clapper lifts its free edge",
        closed_arm_aabb is not None
        and open_arm_aabb is not None
        and open_arm_aabb[0][2] > closed_arm_aabb[0][2] + 0.020,
        details=f"closed={closed_arm_aabb}, open={open_arm_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
