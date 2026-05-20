from __future__ import annotations

from math import pi

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
    mesh_from_cadquery,
)


BOARD_W = 0.305
BOARD_H = 0.225
BOARD_T = 0.006

CONTROL_W = 0.262
CONTROL_H = 0.028
CONTROL_D = 0.016
CONTROL_Z = -0.040

CLAP_W = 0.323
CLAP_H = 0.032
CLAP_D = 0.014
CLAP_HINGE_Y = 0.026
CLAP_HINGE_Z = 0.010

BUTTON_R = 0.0044
BUTTON_LEN = 0.006
BUTTON_TRAVEL = 0.003
BUTTON_XS = (0.068, 0.086, 0.104)


def _control_strip_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CONTROL_W, CONTROL_D, CONTROL_H)
        .edges("|Y")
        .fillet(0.004)
        .faces(">Y")
        .workplane(centerOption="CenterOfBoundBox")
        .center(-0.055, 0.0)
        .rect(0.106, 0.014)
        .cutBlind(-0.003)
        .faces(">Y")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(x, 0.0) for x in BUTTON_XS])
        .circle(BUTTON_R)
        .cutBlind(-0.0065)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="music_video_clapperboard")

    acrylic = model.material("acrylic_black", rgba=(0.06, 0.08, 0.10, 0.82))
    trim_white = model.material("trim_white", rgba=(0.96, 0.96, 0.95, 1.0))
    strip_black = model.material("strip_black", rgba=(0.14, 0.14, 0.16, 1.0))
    screen_blue = model.material("screen_blue", rgba=(0.18, 0.32, 0.54, 0.95))
    clap_red = model.material("clap_red", rgba=(0.84, 0.20, 0.18, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.26, 0.26, 0.28, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.16, 0.16, 1.0))

    board = model.part("board")
    board.visual(
        Box((BOARD_W, BOARD_T, BOARD_H)),
        origin=Origin(xyz=(0.0, 0.0, -BOARD_H / 2.0)),
        material=acrylic,
        name="board_panel",
    )

    line_y = BOARD_T / 2.0 - 0.0003
    line_t = 0.0010
    board.visual(
        Box((0.0035, line_t, 0.148)),
        origin=Origin(xyz=(-0.141, line_y, -0.136)),
        material=trim_white,
        name="left_frame",
    )
    board.visual(
        Box((0.0035, line_t, 0.148)),
        origin=Origin(xyz=(0.141, line_y, -0.136)),
        material=trim_white,
        name="right_frame",
    )
    board.visual(
        Box((0.276, line_t, 0.0035)),
        origin=Origin(xyz=(0.0, line_y, -0.062)),
        material=trim_white,
        name="top_frame",
    )
    board.visual(
        Box((0.276, line_t, 0.0035)),
        origin=Origin(xyz=(0.0, line_y, -0.210)),
        material=trim_white,
        name="bottom_frame",
    )
    board.visual(
        Box((0.276, line_t, 0.0030)),
        origin=Origin(xyz=(0.0, line_y, -0.120)),
        material=trim_white,
        name="mid_rule_0",
    )
    board.visual(
        Box((0.276, line_t, 0.0030)),
        origin=Origin(xyz=(0.0, line_y, -0.164)),
        material=trim_white,
        name="mid_rule_1",
    )
    board.visual(
        Box((0.0030, line_t, 0.090)),
        origin=Origin(xyz=(-0.054, line_y, -0.165)),
        material=trim_white,
        name="column_rule",
    )
    for index, x_pos in enumerate((-0.102, 0.102)):
        board.visual(
            Box((0.024, CLAP_HINGE_Y - BOARD_T / 2.0, CLAP_HINGE_Z)),
            origin=Origin(
                xyz=(
                    x_pos,
                    BOARD_T / 2.0 + (CLAP_HINGE_Y - BOARD_T / 2.0) / 2.0,
                    CLAP_HINGE_Z / 2.0,
                )
            ),
            material=hinge_dark,
            name=f"hinge_block_{index}",
        )

    control_strip = model.part("control_strip")
    control_strip.visual(
        mesh_from_cadquery(_control_strip_shape(), "control_strip"),
        material=strip_black,
        name="housing_shell",
    )
    control_strip.visual(
        Box((0.102, 0.0012, 0.013)),
        origin=Origin(xyz=(-0.055, CONTROL_D / 2.0 - 0.0044, 0.0)),
        material=screen_blue,
        name="display",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((CLAP_W, CLAP_D, CLAP_H)),
        origin=Origin(xyz=(0.0, CLAP_D / 2.0, -CLAP_H / 2.0)),
        material=strip_black,
        name="bar",
    )
    clapstick.visual(
        Cylinder(radius=0.0028, length=CLAP_W - 0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_dark,
        name="hinge_rod",
    )

    stripe_width = 0.050
    stripe_centers = (-0.108, -0.054, 0.0, 0.054, 0.108)
    stripe_materials = (trim_white, clap_red, trim_white, clap_red, trim_white)
    for index, (x_pos, stripe_material) in enumerate(zip(stripe_centers, stripe_materials)):
        clapstick.visual(
            Box((stripe_width, 0.0014, CLAP_H - 0.004)),
            origin=Origin(xyz=(x_pos, CLAP_D - 0.0007, -CLAP_H / 2.0)),
            material=stripe_material,
            name=f"stripe_{index}",
        )

    button_parts = []
    for index in range(3):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_R, length=BUTTON_LEN),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=button_red,
            name="button_cap",
        )
        button_parts.append(button)

    model.articulation(
        "board_to_control_strip",
        ArticulationType.FIXED,
        parent=board,
        child=control_strip,
        origin=Origin(xyz=(0.0, BOARD_T / 2.0 + CONTROL_D / 2.0, CONTROL_Z)),
    )
    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, CLAP_HINGE_Y, CLAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=1.15),
    )

    for index, (x_pos, button) in enumerate(zip(BUTTON_XS, button_parts)):
        model.articulation(
            f"control_strip_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_D / 2.0, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    control_strip = object_model.get_part("control_strip")
    clapstick = object_model.get_part("clapstick")
    clap_hinge = object_model.get_articulation("board_to_clapstick")

    ctx.expect_contact(
        control_strip,
        board,
        name="control strip housing mounts directly to the board face",
    )
    ctx.expect_overlap(
        control_strip,
        board,
        axes="x",
        min_overlap=0.20,
        name="control strip width stays within the board footprint",
    )
    ctx.expect_overlap(
        control_strip,
        board,
        axes="z",
        min_overlap=0.02,
        name="control strip occupies a real top band on the board",
    )
    ctx.expect_gap(
        clapstick,
        control_strip,
        axis="y",
        min_gap=0.002,
        max_gap=0.010,
        name="closed clapstick clears the electronics strip",
    )

    closed_aabb = ctx.part_world_aabb(clapstick)
    limits = clap_hinge.motion_limits
    if closed_aabb is not None and limits is not None and limits.upper is not None:
        with ctx.pose({clap_hinge: limits.upper}):
            open_aabb = ctx.part_world_aabb(clapstick)
        if open_aabb is not None:
            ctx.check(
                "clapstick opens upward and outward",
                open_aabb[1][1] > closed_aabb[1][1] + 0.018
                and open_aabb[0][2] > closed_aabb[0][2] + 0.010,
                details=f"closed={closed_aabb}, open={open_aabb}",
            )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"control_strip_to_button_{index}")
        button_limits = button_joint.motion_limits
        if button_limits is None or button_limits.upper is None:
            continue

        rest_pos = ctx.part_world_position(button)
        neighbor_names = [f"button_{other}" for other in range(3) if other != index]
        rest_neighbor_positions = {
            name: ctx.part_world_position(object_model.get_part(name)) for name in neighbor_names
        }

        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
            pressed_neighbor_positions = {
                name: ctx.part_world_position(object_model.get_part(name)) for name in neighbor_names
            }
            ctx.expect_within(
                button,
                control_strip,
                axes="xz",
                margin=0.004,
                name=f"button_{index} stays aligned in the control strip when pressed",
            )

        moved_inward = (
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0025
        )
        neighbors_static = True
        for name in neighbor_names:
            rest_neighbor = rest_neighbor_positions[name]
            pressed_neighbor = pressed_neighbor_positions[name]
            if (
                rest_neighbor is None
                or pressed_neighbor is None
                or abs(pressed_neighbor[1] - rest_neighbor[1]) > 1e-6
            ):
                neighbors_static = False
                break

        ctx.check(
            f"button_{index} depresses independently",
            moved_inward and neighbors_static,
            details=(
                f"rest={rest_pos}, pressed={pressed_pos}, "
                f"rest_neighbors={rest_neighbor_positions}, pressed_neighbors={pressed_neighbor_positions}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
