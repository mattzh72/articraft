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


BODY_LENGTH = 0.272
BODY_WIDTH = 0.232
BODY_BASE_HEIGHT = 0.030
BODY_DECK_HEIGHT = 0.010
BODY_DECK_OFFSET_X = 0.008

LID_LENGTH = 0.245
LID_WIDTH = 0.220
LID_HEIGHT = 0.060

HINGE_X = -0.110
HINGE_Z = 0.046

BUTTON_TRAVEL = 0.0015


def _build_body_shell() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.008)
    )
    upper_deck = (
        cq.Workplane("XY")
        .box(0.246, 0.206, BODY_DECK_HEIGHT, centered=(True, True, False))
        .translate((BODY_DECK_OFFSET_X, 0.0, BODY_BASE_HEIGHT))
        .edges("|Z")
        .fillet(0.006)
    )
    return base.union(upper_deck)


def _build_lid_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(LID_LENGTH, LID_WIDTH, LID_HEIGHT, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.016)
        .faces("<Z")
        .shell(-0.0035)
    )


def _build_front_handle() -> cq.Workplane:
    grip = (
        cq.Workplane("YZ")
        .slot2D(0.104, 0.014)
        .extrude(0.030)
        .translate((0.242, 0.000, 0.026))
    )
    foot_0 = (
        cq.Workplane("XY")
        .box(0.018, 0.018, 0.022, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.233, -0.041, 0.000))
    )
    foot_1 = (
        cq.Workplane("XY")
        .box(0.018, 0.018, 0.022, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((0.233, 0.041, 0.000))
    )
    return grip.union(foot_0).union(foot_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sandwich_press")

    housing = model.material("housing", rgba=(0.14, 0.15, 0.16, 1.0))
    panel = model.material("panel", rgba=(0.20, 0.21, 0.22, 1.0))
    plate = model.material("plate", rgba=(0.48, 0.49, 0.50, 1.0))
    handle = model.material("handle", rgba=(0.08, 0.08, 0.09, 1.0))
    button = model.material("button", rgba=(0.77, 0.79, 0.80, 1.0))
    accent = model.material("accent", rgba=(0.24, 0.26, 0.28, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "sandwich_press_body_shell"),
        material=housing,
        name="body_shell",
    )
    body.visual(
        Box((0.194, 0.152, 0.010)),
        origin=Origin(xyz=(0.012, 0.000, 0.038)),
        material=plate,
        name="lower_plate",
    )
    body.visual(
        Box((0.060, 0.120, 0.004)),
        origin=Origin(xyz=(0.012, 0.000, 0.043)),
        material=accent,
        name="lower_plate_ridge",
    )
    body.visual(
        Box((0.060, 0.074, 0.058)),
        origin=Origin(xyz=(-0.145, 0.000, 0.029)),
        material=panel,
        name="hinge_block",
    )
    body.visual(
        Box((0.058, 0.044, 0.008)),
        origin=Origin(xyz=(0.084, 0.070, 0.044)),
        material=panel,
        name="button_podium",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "sandwich_press_lid_shell"),
        material=housing,
        name="lid_shell",
    )
    lid.visual(
        Box((0.186, 0.146, 0.008)),
        origin=Origin(xyz=(0.124, 0.000, 0.006)),
        material=plate,
        name="upper_plate",
    )
    lid.visual(
        Box((0.090, 0.112, 0.060)),
        origin=Origin(xyz=(0.040, 0.000, 0.030)),
        material=accent,
        name="upper_mount",
    )
    lid.visual(
        mesh_from_cadquery(_build_front_handle(), "sandwich_press_front_handle"),
        material=handle,
        name="front_handle",
    )

    button_0 = model.part("program_button_0")
    button_0.visual(
        Box((0.017, 0.013, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=accent,
        name="button_stem",
    )
    button_0.visual(
        Box((0.020, 0.016, 0.005)),
        origin=Origin(xyz=(0.000, 0.000, 0.0055)),
        material=button,
        name="button_cap",
    )

    button_1 = model.part("program_button_1")
    button_1.visual(
        Box((0.017, 0.013, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=accent,
        name="button_stem",
    )
    button_1.visual(
        Box((0.020, 0.016, 0.005)),
        origin=Origin(xyz=(0.000, 0.000, 0.0055)),
        material=button,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.000, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    model.articulation(
        "body_to_program_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(0.072, 0.060, 0.0495)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.03,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_program_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.096, 0.078, 0.0495)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.03,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    button_joint_0 = object_model.get_articulation("body_to_program_button_0")
    button_joint_1 = object_model.get_articulation("body_to_program_button_1")

    ctx.expect_overlap(
        lid,
        "body",
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.140,
        name="cooking plates align in the closed pose",
    )
    ctx.expect_gap(
        lid,
        "body",
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.002,
        max_gap=0.008,
        name="closed cooking plates remain nearly shut",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_upper_plate_aabb = ctx.part_element_world_aabb(lid, elem="upper_plate")
    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
            open_upper_plate_aabb = ctx.part_element_world_aabb(lid, elem="upper_plate")
        ctx.check(
            "lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.090,
            details=f"closed_aabb={closed_lid_aabb}, open_aabb={open_lid_aabb}",
        )
        ctx.check(
            "upper plate rises clear when opened",
            closed_upper_plate_aabb is not None
            and open_upper_plate_aabb is not None
            and open_upper_plate_aabb[1][2] > closed_upper_plate_aabb[1][2] + 0.140,
            details=(
                f"closed_upper_plate_aabb={closed_upper_plate_aabb}, "
                f"open_upper_plate_aabb={open_upper_plate_aabb}"
            ),
        )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_steady = ctx.part_world_position(button_1)
    ctx.check(
        "program button 0 presses independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.001
        and button_1_rest is not None
        and button_1_steady is not None
        and abs(button_1_steady[2] - button_1_rest[2]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_steady={button_1_steady}"
        ),
    )

    with ctx.pose({button_joint_1: BUTTON_TRAVEL}):
        button_1_pressed = ctx.part_world_position(button_1)
        button_0_steady = ctx.part_world_position(button_0)
    ctx.check(
        "program button 1 presses independently",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.001
        and button_0_rest is not None
        and button_0_steady is not None
        and abs(button_0_steady[2] - button_0_rest[2]) < 1e-6,
        details=(
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}, "
            f"button_0_rest={button_0_rest}, button_0_steady={button_0_steady}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
