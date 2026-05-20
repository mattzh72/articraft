from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_L = 0.155
BASE_W = 0.044
BASE_T = 0.010

ARM_L = 0.148
ARM_W = 0.039
HINGE_X = -0.067
HINGE_Z = 0.034

TRAY_TRAVEL = 0.055
ANVIL_X = 0.050


def _base_shape() -> cq.Workplane:
    base_slab = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T)
        .translate((0.0, 0.0, BASE_T / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )
    rear_block = (
        cq.Workplane("XY")
        .box(0.024, 0.028, 0.004)
        .translate((-0.064, 0.0, BASE_T + 0.002))
        .edges("|Z")
        .fillet(0.002)
    )
    front_pad = (
        cq.Workplane("XY")
        .box(0.028, 0.024, 0.004)
        .translate((0.051, 0.0, BASE_T + 0.002))
        .edges("|Z")
        .fillet(0.0015)
    )
    anvil_recess = (
        cq.Workplane("XY")
        .box(0.026, 0.018, 0.0044)
        .translate((ANVIL_X, 0.0, BASE_T + 0.0021))
    )
    pivot_hole = (
        cq.Workplane("XY")
        .cylinder(0.0065, 0.0025)
        .translate((ANVIL_X, 0.0, BASE_T - 0.00325))
    )
    throat_relief = (
        cq.Workplane("XY")
        .box(0.024, 0.018, 0.003)
        .translate((0.066, 0.0, BASE_T - 0.0015))
    )
    return base_slab.union(rear_block).union(front_pad).cut(anvil_recess).cut(pivot_hole).cut(throat_relief)


def _arm_shape() -> cq.Workplane:
    outer_profile = [
        (0.000, -0.0085),
        (0.000, 0.004),
        (0.012, 0.0065),
        (0.084, 0.0060),
        (0.126, 0.0020),
        (ARM_L, -0.008),
        (ARM_L, -0.020),
        (0.120, -0.020),
        (0.020, -0.018),
        (0.000, -0.0145),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(ARM_W)
        .translate((0.0, ARM_W / 2.0, 0.0))
    )
    tray_cavity = (
        cq.Workplane("XY")
        .box(0.120, 0.031, 0.017)
        .translate((0.073, 0.0, -0.0095))
    )
    rear_opening = (
        cq.Workplane("XY")
        .box(0.024, 0.033, 0.018)
        .translate((0.006, 0.0, -0.0090))
    )
    staple_slot = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.006)
        .translate((0.137, 0.0, -0.012))
    )
    return shell.cut(tray_cavity).cut(rear_opening).cut(staple_slot)


def _tray_shape() -> cq.Workplane:
    floor = cq.Workplane("XY").box(0.124, 0.022, 0.0032).translate((0.056, 0.0, -0.0138))
    left_wall = cq.Workplane("XY").box(0.110, 0.0022, 0.011).translate((0.055, 0.0121, -0.0086))
    right_wall = cq.Workplane("XY").box(0.110, 0.0022, 0.011).translate((0.055, -0.0121, -0.0086))
    front_block = cq.Workplane("XY").box(0.014, 0.022, 0.007).translate((0.117, 0.0, -0.0086))
    rear_pull = (
        cq.Workplane("XY")
        .box(0.020, 0.028, 0.008)
        .translate((-0.009, 0.0, -0.0100))
        .edges("|Z")
        .fillet(0.0012)
    )
    return floor.union(left_wall).union(right_wall).union(front_block).union(rear_pull)


def _anvil_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.002)
        .edges("|Z")
        .fillet(0.0012)
    )
    selector_tab = cq.Workplane("XY").box(0.004, 0.004, 0.002).translate((0.005, 0.006, 0.0))
    return plate.union(selector_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")
    model.material("die_cast", rgba=(0.53, 0.55, 0.58, 1.0))
    model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("polished_steel", rgba=(0.71, 0.73, 0.76, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_body"),
        material="dark_steel",
        name="base_body",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shape(), "arm_shell"),
        material="die_cast",
        name="arm_shell",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shape(), "tray_body"),
        material="polished_steel",
        name="tray_body",
    )

    anvil = model.part("anvil")
    anvil.visual(
        mesh_from_cadquery(_anvil_shape(), "anvil_plate"),
        material="polished_steel",
        name="anvil_plate",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=25.0, velocity=2.0),
    )
    model.articulation(
        "arm_to_tray",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tray,
        origin=Origin(xyz=(0.004, 0.0, 0.0021)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=30.0, velocity=0.30),
    )
    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(ANVIL_X, 0.0, BASE_T + 0.0009)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=pi / 2.0, effort=2.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    tray = object_model.get_part("tray")
    anvil = object_model.get_part("anvil")
    arm_hinge = object_model.get_articulation("base_to_arm")
    tray_slide = object_model.get_articulation("arm_to_tray")
    anvil_pivot = object_model.get_articulation("base_to_anvil")

    ctx.expect_contact(
        arm,
        base,
        elem_a="arm_shell",
        elem_b="base_body",
        contact_tol=0.0025,
        name="closed arm seats close to the base",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        elem_a="arm_shell",
        elem_b="base_body",
        min_overlap=0.030,
        name="arm footprint stays over the base",
    )
    ctx.expect_within(
        tray,
        arm,
        axes="yz",
        inner_elem="tray_body",
        outer_elem="arm_shell",
        margin=0.002,
        name="tray stays side-guided inside the arm body",
    )
    ctx.expect_overlap(
        tray,
        arm,
        axes="x",
        elem_a="tray_body",
        elem_b="arm_shell",
        min_overlap=0.090,
        name="tray remains deeply inserted when closed",
    )
    ctx.expect_contact(
        anvil,
        base,
        elem_a="anvil_plate",
        elem_b="base_body",
        contact_tol=0.001,
        name="anvil plate sits on the base deck",
    )

    arm_limits = arm_hinge.motion_limits
    if arm_limits is not None and arm_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(arm)
        with ctx.pose({arm_hinge: arm_limits.upper}):
            open_aabb = ctx.part_world_aabb(arm)
        ctx.check(
            "arm opens upward from the rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.055,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    tray_limits = tray_slide.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        tray_rest = ctx.part_world_position(tray)
        with ctx.pose({tray_slide: tray_limits.upper}):
            tray_extended = ctx.part_world_position(tray)
            ctx.expect_within(
                tray,
                arm,
                axes="yz",
                inner_elem="tray_body",
                outer_elem="arm_shell",
                margin=0.002,
                name="extended tray stays captured by the side guides",
            )
            ctx.expect_overlap(
                tray,
                arm,
                axes="x",
                elem_a="tray_body",
                elem_b="arm_shell",
                min_overlap=0.035,
                name="extended tray retains insertion in the upper body",
            )
        ctx.check(
            "tray pulls rearward out of the stapler body",
            tray_rest is not None
            and tray_extended is not None
            and tray_extended[0] < tray_rest[0] - 0.040,
            details=f"rest={tray_rest}, extended={tray_extended}",
        )

    anvil_limits = anvil_pivot.motion_limits
    if anvil_limits is not None and anvil_limits.upper is not None:
        rest_box = ctx.part_element_world_aabb(anvil, elem="anvil_plate")
        with ctx.pose({anvil_pivot: anvil_limits.upper}):
            turned_box = ctx.part_element_world_aabb(anvil, elem="anvil_plate")
        rest_dx = None if rest_box is None else rest_box[1][0] - rest_box[0][0]
        rest_dy = None if rest_box is None else rest_box[1][1] - rest_box[0][1]
        turned_dx = None if turned_box is None else turned_box[1][0] - turned_box[0][0]
        turned_dy = None if turned_box is None else turned_box[1][1] - turned_box[0][1]
        ctx.check(
            "anvil visibly reorients around its pivot",
            None not in (rest_dx, rest_dy, turned_dx, turned_dy)
            and turned_dy > rest_dy + 0.006
            and turned_dx < rest_dx - 0.006,
            details=(
                f"rest_dx={rest_dx}, rest_dy={rest_dy}, "
                f"turned_dx={turned_dx}, turned_dy={turned_dy}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
