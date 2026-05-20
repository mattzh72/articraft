from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.292
BODY_DEPTH = 0.286
BODY_HEIGHT = 0.084
LID_HEIGHT = 0.070
HINGE_Z = 0.090

PLATE_SIZE = 0.236
PLATE_BASE_Z = 0.060
PLATE_THICKNESS = 0.010
PLATE_RIB_WIDTH = 0.018
PLATE_RIB_HEIGHT = 0.006

BUTTON_CAP_SIZE = (0.018, 0.014, 0.005)
BUTTON_Y = BODY_DEPTH * 0.5 - 0.021
BUTTON_XS = (0.074, 0.104)
BUTTON_CENTER_Z = 0.088


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .edges()
        .fillet(0.007)
    )

    plate_opening = (
        cq.Workplane("XY")
        .box(0.232, 0.232, 0.026, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_HEIGHT - 0.024))
    )
    shell = shell.cut(plate_opening)

    well_size = (0.022, 0.018, 0.015)
    for x in BUTTON_XS:
        well = (
            cq.Workplane("XY")
            .box(*well_size, centered=(True, True, False))
            .translate((x, BUTTON_Y, BODY_HEIGHT - well_size[2]))
        )
        shell = shell.cut(well)

    foot_size = (0.038, 0.028, 0.004)
    foot_x = BODY_WIDTH * 0.5 - 0.046
    foot_y = BODY_DEPTH * 0.5 - 0.042
    for x in (-foot_x, foot_x):
        for y in (-foot_y, foot_y):
            foot = (
                cq.Workplane("XY")
                .box(*foot_size, centered=(True, True, False))
                .translate((x, y, 0.0))
            )
            shell = shell.union(foot)

    return shell


def _lower_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_BASE_Z))
    )
    cross_x = (
        cq.Workplane("XY")
        .box(PLATE_RIB_WIDTH, PLATE_SIZE, PLATE_RIB_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_BASE_Z + PLATE_THICKNESS))
    )
    cross_y = (
        cq.Workplane("XY")
        .box(PLATE_SIZE, PLATE_RIB_WIDTH, PLATE_RIB_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, PLATE_BASE_Z + PLATE_THICKNESS))
    )
    return plate.union(cross_x).union(cross_y)


def _lid_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH + 0.006, BODY_DEPTH, LID_HEIGHT, centered=(True, False, False))
        .translate((0.0, 0.0, -0.006))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .edges()
        .fillet(0.009)
    )

    cavity = (
        cq.Workplane("XY")
        .box(0.226, 0.220, 0.052, centered=(True, False, False))
        .translate((0.0, 0.030, -0.006))
    )
    return shell.cut(cavity)


def _upper_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.228, 0.222, 0.008, centered=(True, False, False))
        .translate((0.0, 0.030, -0.002))
    )
    rib_x = (
        cq.Workplane("XY")
        .box(PLATE_RIB_WIDTH, 0.212, 0.008, centered=(True, False, False))
        .translate((0.0, 0.035, -0.010))
    )
    rib_y = (
        cq.Workplane("XY")
        .box(0.212, PLATE_RIB_WIDTH, 0.008, centered=(True, False, False))
        .translate((0.0, 0.101, -0.010))
    )
    return plate.union(rib_x).union(rib_y)


def _handle_shape() -> cq.Workplane:
    handle = (
        cq.Workplane("XY")
        .box(0.122, 0.034, 0.022, centered=(True, True, False))
        .translate((0.0, BODY_DEPTH - 0.004, 0.004))
        .edges("|Z")
        .fillet(0.007)
    )
    finger_relief = (
        cq.Workplane("YZ")
        .center(0.015, BODY_DEPTH + 0.005)
        .circle(0.010)
        .extrude(0.108, both=True)
        .translate((0.0, 0.0, 0.0))
    )
    return handle.cut(finger_relief)


def _button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(*BUTTON_CAP_SIZE, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0022)
        .faces(">Z")
        .edges()
        .fillet(0.0012)
    )
    stem = (
        cq.Workplane("XY")
        .box(0.011, 0.007, 0.003, centered=(True, True, False))
        .translate((0.0, 0.0, -0.0005))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waffle_maker")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    lid_metal = model.material("lid_metal", rgba=(0.73, 0.74, 0.75, 1.0))
    plate_black = model.material("plate_black", rgba=(0.10, 0.10, 0.11, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=body_dark,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_lower_plate_shape(), "lower_plate"),
        material=plate_black,
        name="lower_plate",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "lid_shell"),
        material=lid_metal,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_upper_plate_shape(), "upper_plate"),
        material=plate_black,
        name="upper_plate",
    )
    lid.visual(
        mesh_from_cadquery(_handle_shape(), "handle"),
        material=control_black,
        name="handle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + 0.006, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )

    button_mesh = mesh_from_cadquery(_button_shape(), "program_button")
    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=control_black,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, BUTTON_Y, BUTTON_CENTER_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    button_0_slide = object_model.get_articulation("body_to_button_0")
    button_1_slide = object_model.get_articulation("body_to_button_1")

    lid_limits = lid_hinge.motion_limits
    button_0_limits = button_0_slide.motion_limits
    button_1_limits = button_1_slide.motion_limits

    with ctx.pose({lid_hinge: 0.0, button_0_slide: 0.0, button_1_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_penetration=1e-4,
            max_gap=0.005,
            name="closed shell seam stays tight without penetration",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.22,
            name="lid covers the lower body footprint",
        )
        ctx.expect_gap(
            button_1,
            button_0,
            axis="x",
            min_gap=0.010,
            name="program buttons remain visibly discrete",
        )

    closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle")
    closed_button_0 = ctx.part_world_position(button_0)
    closed_button_1 = ctx.part_world_position(button_1)

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_handle_aabb = ctx.part_element_world_aabb(lid, elem="handle")
        closed_handle_center = _aabb_center(closed_handle_aabb)
        open_handle_center = _aabb_center(open_handle_aabb)
        ctx.check(
            "lid handle rises when opened",
            closed_handle_center is not None
            and open_handle_center is not None
            and open_handle_center[2] > closed_handle_center[2] + 0.10,
            details=f"closed={closed_handle_center}, open={open_handle_center}",
        )

    if button_0_limits is not None and button_0_limits.upper is not None:
        with ctx.pose({button_0_slide: button_0_limits.upper}):
            pressed_button_0 = ctx.part_world_position(button_0)
            unpressed_button_1 = ctx.part_world_position(button_1)
        ctx.check(
            "button_0 presses downward independently",
            closed_button_0 is not None
            and pressed_button_0 is not None
            and pressed_button_0[2] < closed_button_0[2] - 0.001
            and closed_button_1 is not None
            and unpressed_button_1 is not None
            and abs(unpressed_button_1[2] - closed_button_1[2]) < 1e-6,
            details=(
                f"button_0 rest={closed_button_0}, pressed={pressed_button_0}, "
                f"button_1 rest={closed_button_1}, during_button_0_press={unpressed_button_1}"
            ),
        )

    if button_1_limits is not None and button_1_limits.upper is not None:
        with ctx.pose({button_1_slide: button_1_limits.upper}):
            pressed_button_1 = ctx.part_world_position(button_1)
            unpressed_button_0 = ctx.part_world_position(button_0)
        ctx.check(
            "button_1 presses downward independently",
            closed_button_1 is not None
            and pressed_button_1 is not None
            and pressed_button_1[2] < closed_button_1[2] - 0.001
            and closed_button_0 is not None
            and unpressed_button_0 is not None
            and abs(unpressed_button_0[2] - closed_button_0[2]) < 1e-6,
            details=(
                f"button_1 rest={closed_button_1}, pressed={pressed_button_1}, "
                f"button_0 rest={closed_button_0}, during_button_1_press={unpressed_button_0}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
