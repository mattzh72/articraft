from __future__ import annotations

import math

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

BASE_W = 0.320
BASE_D = 0.270
BASE_H = 0.072
BASE_FLOOR = 0.010

PLATE_W = 0.270
PLATE_D = 0.205
PLATE_T = 0.012
BASE_CAVITY_Y = -0.006

LID_W = 0.314
LID_D = 0.254
LID_H = 0.052
LID_TOP = 0.009
LID_CAVITY_W = 0.266
LID_CAVITY_D = 0.201
LID_CAVITY_Y = -0.004

SEAM_GAP = 0.0025
HINGE_Y = -(BASE_D / 2.0) - 0.004
HINGE_Z = BASE_H - 0.004
LID_CENTER_Y = (LID_D / 2.0) + 0.007
LID_BOTTOM_Z = BASE_H + SEAM_GAP - HINGE_Z

BUTTON_HOLE_W = 0.019
BUTTON_HOLE_D = 0.016
BUTTON_CAP_W = 0.017
BUTTON_CAP_D = 0.014
BUTTON_CAP_H = 0.006
BUTTON_STEM_W = 0.010
BUTTON_STEM_D = 0.010
BUTTON_STEM_H = 0.014
BUTTON_TRAVEL = 0.003
BUTTON_POSITIONS = ((0.070, 0.108), (0.101, 0.108))

LATCH_Y = (BASE_D / 2.0) + 0.001
LATCH_Z = BASE_H


def _make_base_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(0.018)
    outer = outer.edges(">Z").fillet(0.009)

    cavity = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_D, BASE_H - BASE_FLOOR, centered=(True, True, False))
        .translate((0.0, BASE_CAVITY_Y, BASE_FLOOR))
    )
    shell = outer.cut(cavity)

    for x_pos, y_pos in BUTTON_POSITIONS:
        shell = shell.cut(
            cq.Workplane("XY")
            .box(BUTTON_HOLE_W, BUTTON_HOLE_D, 0.020, centered=(True, True, False))
            .translate((x_pos, y_pos, BASE_H - 0.020))
        )

    return shell


def _make_lid_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(LID_W, LID_D, LID_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(0.016)
    outer = outer.edges(">Z").fillet(0.010)

    cavity = (
        cq.Workplane("XY")
        .box(LID_CAVITY_W, LID_CAVITY_D, LID_H - LID_TOP, centered=(True, True, False))
        .translate((0.0, LID_CAVITY_Y, 0.0))
    )
    return outer.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_sandwich_clamshell_press")

    model.material("cast_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("satin_black", rgba=(0.17, 0.17, 0.18, 1.0))
    model.material("plate_dark", rgba=(0.23, 0.24, 0.24, 1.0))
    model.material("steel", rgba=(0.60, 0.61, 0.63, 1.0))
    model.material("button_cream", rgba=(0.88, 0.85, 0.77, 1.0))
    model.material("button_red", rgba=(0.78, 0.20, 0.18, 1.0))

    base = model.part("base")
    lid = model.part("lid")
    latch_strap = model.part("latch_strap")
    button_0 = model.part("program_button_0")
    button_1 = model.part("program_button_1")

    base.visual(
        mesh_from_cadquery(_make_base_shell(), "base_shell"),
        material="cast_black",
        name="base_shell",
    )
    base.visual(
        Box((PLATE_W + 0.001, PLATE_D + 0.001, PLATE_T)),
        origin=Origin(xyz=(0.0, BASE_CAVITY_Y, BASE_H - 0.017)),
        material="plate_dark",
        name="lower_plate",
    )
    for i, x_pos in enumerate((-0.117, 0.117)):
        base.visual(
            Box((0.094, 0.016, 0.018)),
            origin=Origin(xyz=(x_pos, -(BASE_D / 2.0) + 0.004, BASE_H - 0.008)),
            material="cast_black",
            name=f"rear_hinge_mount_{i}",
        )
        base.visual(
            Cylinder(radius=0.007, length=0.092),
            origin=Origin(xyz=(x_pos, HINGE_Y, HINGE_Z, ), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="steel",
            name=f"rear_hinge_barrel_{i}",
        )

    for i, x_pos in enumerate((-0.016, 0.016)):
        base.visual(
            Box((0.014, 0.012, 0.014)),
            origin=Origin(xyz=(x_pos, LATCH_Y - 0.004, LATCH_Z)),
            material="cast_black",
            name=f"latch_mount_{i}",
        )
        base.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(xyz=(x_pos, LATCH_Y, LATCH_Z, ), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="steel",
            name=f"latch_barrel_{i}",
        )

    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "lid_shell"),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, LID_BOTTOM_Z)),
        material="satin_black",
        name="lid_shell",
    )
    lid.visual(
        Box((LID_CAVITY_W + 0.001, LID_CAVITY_D + 0.001, 0.010)),
        origin=Origin(xyz=(0.0, LID_CENTER_Y + LID_CAVITY_Y, LID_BOTTOM_Z + 0.006)),
        material="plate_dark",
        name="upper_plate",
    )
    lid.visual(
        Box((0.126, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.013, 0.012)),
        material="satin_black",
        name="hinge_mount",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.110),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.060, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, LID_CENTER_Y + (LID_D / 2.0) - 0.004, LID_BOTTOM_Z + 0.004)),
        material="satin_black",
        name="front_catch",
    )

    latch_strap.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="steel",
        name="barrel",
    )
    latch_strap.visual(
        Box((0.016, 0.010, 0.044)),
        origin=Origin(
            xyz=(0.0, -0.005, 0.022),
        ),
        material="cast_black",
        name="strap",
    )
    latch_strap.visual(
        Box((0.016, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.005, 0.012)),
        material="cast_black",
        name="hook",
    )

    for part_obj, button_material in ((button_0, "button_cream"), (button_1, "button_red")):
        part_obj.visual(
            Box((BUTTON_CAP_W, BUTTON_CAP_D, BUTTON_CAP_H)),
            material=button_material,
            name="cap",
        )
        part_obj.visual(
            Box((BUTTON_STEM_W, BUTTON_STEM_D, BUTTON_STEM_H)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=button_material,
            name="stem",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=18.0, velocity=1.6),
    )
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch_strap,
        origin=Origin(xyz=(0.0, LATCH_Y, LATCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.20, upper=0.0, effort=3.0, velocity=2.4),
    )

    for idx, (button_part, (x_pos, y_pos)) in enumerate(zip((button_0, button_1), BUTTON_POSITIONS)):
        model.articulation(
            f"program_button_{idx}_press",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button_part,
            origin=Origin(xyz=(x_pos, y_pos, BASE_H)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=1.0, velocity=0.08),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch_strap = object_model.get_part("latch_strap")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")

    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_hinge = object_model.get_articulation("latch_hinge")
    button_0_press = object_model.get_articulation("program_button_0_press")
    button_1_press = object_model.get_articulation("program_button_1_press")

    with ctx.pose({lid_hinge: 0.0, latch_hinge: 0.0, button_0_press: 0.0, button_1_press: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="base_shell",
            min_gap=0.001,
            max_gap=0.010,
            name="closed lid keeps a tight seam above the base casting",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_shell",
            elem_b="base_shell",
            min_overlap=0.200,
            name="closed lid covers the cooking deck footprint",
        )
        button_0_cap = ctx.part_element_world_aabb(button_0, elem="cap")
        button_1_cap = ctx.part_element_world_aabb(button_1, elem="cap")
        ctx.check(
            "front cream button stands proud of the lower housing",
            button_0_cap is not None and button_0_cap[1][2] > BASE_H + 0.002,
            details=f"button_0_cap={button_0_cap}",
        )
        ctx.check(
            "front red button stands proud of the lower housing",
            button_1_cap is not None and button_1_cap[1][2] > BASE_H + 0.002,
            details=f"button_1_cap={button_1_cap}",
        )

    open_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if open_limit is not None:
        with ctx.pose({lid_hinge: open_limit}):
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="front_catch",
                negative_elem="base_shell",
                min_gap=0.060,
                name="opened lid lifts its front edge well above the base",
            )

    release_limit = latch_hinge.motion_limits.lower if latch_hinge.motion_limits is not None else None
    if release_limit is not None:
        hook_rest = ctx.part_element_world_aabb(latch_strap, elem="hook")
        with ctx.pose({latch_hinge: release_limit}):
            hook_released = ctx.part_element_world_aabb(latch_strap, elem="hook")
            ctx.check(
                "latch strap rotates down and away from the lid catch",
                hook_rest is not None
                and hook_released is not None
                and hook_released[0][1] > hook_rest[0][1] + 0.015
                and hook_released[1][2] < hook_rest[1][2] - 0.010,
                details=f"rest={hook_rest}, released={hook_released}",
            )

    button_0_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_0_press: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(button_0)
        ctx.check(
            "cream program button depresses downward",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_0_pressed[2] < button_0_rest[2] - 0.002,
            details=f"rest={button_0_rest}, pressed={button_0_pressed}",
        )

    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_1_press: BUTTON_TRAVEL}):
        button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "red program button depresses downward",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_1_pressed[2] < button_1_rest[2] - 0.002,
            details=f"rest={button_1_rest}, pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
