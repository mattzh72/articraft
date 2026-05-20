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


WIDTH = 0.300
DEPTH = 0.255
FOOT_HEIGHT = 0.006
BASE_BODY_HEIGHT = 0.048
BASE_HEIGHT = FOOT_HEIGHT + BASE_BODY_HEIGHT

HINGE_X = -DEPTH / 2.0 + 0.020
HINGE_Z = 0.067

BASE_PLATE_SIZE = (0.236, 0.280, 0.008)

LID_DEPTH = 0.232
LID_WIDTH = 0.292
LID_SHELL_HEIGHT = 0.032
LID_PLATE_SIZE = (0.218, 0.276, 0.008)


def _base_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(DEPTH, WIDTH, BASE_BODY_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT + BASE_BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.017)
        .edges(">Z")
        .fillet(0.010)
    )

    cavity = (
        cq.Workplane("XY")
        .box(DEPTH - 0.022, WIDTH - 0.022, BASE_BODY_HEIGHT)
        .translate((0.0, 0.0, FOOT_HEIGHT + 0.007 + BASE_BODY_HEIGHT / 2.0))
    )
    shell = outer.cut(cavity)

    for x_pos in (-0.090, 0.090):
        for y_pos in (-0.115, 0.115):
            foot = (
                cq.Workplane("XY")
                .circle(0.012)
                .extrude(FOOT_HEIGHT)
                .translate((x_pos, y_pos, 0.0))
            )
            shell = shell.union(foot)

    return shell


def _lid_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, LID_SHELL_HEIGHT)
        .translate((LID_DEPTH / 2.0, 0.0, 0.013))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.010)
    )

    cavity = (
        cq.Workplane("XY")
        .box(LID_DEPTH - 0.020, LID_WIDTH - 0.020, LID_SHELL_HEIGHT + 0.004)
        .translate((LID_DEPTH / 2.0, 0.0, 0.005))
    )
    lid = outer.cut(cavity)

    crown = (
        cq.Workplane("XY")
        .box(0.150, 0.220, 0.007)
        .translate((0.120, 0.0, 0.026))
        .edges(">Z")
        .fillet(0.003)
    )
    return lid.union(crown)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sandwich_press")

    body_finish = model.material("body_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.34, 0.35, 0.36, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.78, 0.79, 0.80, 1.0))
    control_finish = model.material("control_finish", rgba=(0.67, 0.68, 0.70, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.18, 0.62, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell_shape(), "base_shell"),
        material=body_finish,
        name="base_shell",
    )
    base.visual(
        Box(BASE_PLATE_SIZE),
        origin=Origin(xyz=(0.008, 0.0, 0.048)),
        material=plate_finish,
        name="base_plate",
    )

    lid = model.part("top_platen")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "top_platen_shell"),
        material=lid_finish,
        name="lid_shell",
    )
    lid.visual(
        Box(LID_PLATE_SIZE),
        origin=Origin(xyz=(0.116, 0.0, -0.010)),
        material=plate_finish,
        name="lid_plate",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.108),
        origin=Origin(xyz=(0.214, 0.0, -0.003), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="handle_grip",
    )
    for index, y_pos in enumerate((-0.043, 0.043)):
        lid.visual(
            Box((0.016, 0.014, 0.021)),
            origin=Origin(xyz=(0.214, y_pos, 0.007)),
            material=trim_finish,
            name=f"handle_post_{index}",
        )

    model.articulation(
        "base_to_top_platen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=18.0, velocity=1.6),
    )

    lever = model.part("height_stop_lever")
    lever.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="lever_hub",
    )
    lever.visual(
        Box((0.052, 0.008, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, -0.012)),
        material=control_finish,
        name="lever_arm",
    )
    lever.visual(
        Box((0.012, 0.010, 0.020)),
        origin=Origin(xyz=(0.062, 0.0, -0.015)),
        material=accent_finish,
        name="lever_tip",
    )

    model.articulation(
        "base_to_height_stop_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(-0.080, WIDTH / 2.0 + 0.005, 0.046)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=4.0, velocity=2.5),
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="selector_dial",
    )
    knob.visual(
        Box((0.010, 0.006, 0.008)),
        origin=Origin(xyz=(0.010, 0.004, 0.014)),
        material=accent_finish,
        name="selector_pointer",
    )

    model.articulation(
        "base_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.078, WIDTH / 2.0 + 0.008, 0.033)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("top_platen")
    hinge = object_model.get_articulation("base_to_top_platen")
    lever = object_model.get_part("height_stop_lever")
    lever_joint = object_model.get_articulation("base_to_height_stop_lever")
    knob_joint = object_model.get_articulation("base_to_selector_knob")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="base_plate",
            min_gap=0.001,
            max_gap=0.006,
            name="closed platens stay nearly touching",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_plate",
            elem_b="base_plate",
            min_overlap=0.19,
            name="cooking plates align in plan",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.0}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "top platen opens upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    lever_tip_closed = ctx.part_element_world_aabb(lever, elem="lever_tip")
    with ctx.pose({lever_joint: 0.9}):
        lever_tip_raised = ctx.part_element_world_aabb(lever, elem="lever_tip")

    ctx.check(
        "height stop lever raises above the sidewall",
        lever_tip_closed is not None
        and lever_tip_raised is not None
        and lever_tip_raised[1][2] > lever_tip_closed[1][2] + 0.03,
        details=f"closed={lever_tip_closed}, raised={lever_tip_raised}",
    )

    pointer_aabb_0 = ctx.part_element_world_aabb("selector_knob", elem="selector_pointer")
    with ctx.pose({knob_joint: 1.2}):
        pointer_aabb_1 = ctx.part_element_world_aabb("selector_knob", elem="selector_pointer")

    pointer_center_0 = None
    if pointer_aabb_0 is not None:
        pointer_center_0 = tuple((lo + hi) / 2.0 for lo, hi in zip(pointer_aabb_0[0], pointer_aabb_0[1]))
    pointer_center_1 = None
    if pointer_aabb_1 is not None:
        pointer_center_1 = tuple((lo + hi) / 2.0 for lo, hi in zip(pointer_aabb_1[0], pointer_aabb_1[1]))

    ctx.check(
        "selector knob rotates a visible pointer",
        pointer_center_0 is not None
        and pointer_center_1 is not None
        and abs(pointer_center_1[0] - pointer_center_0[0]) > 0.005
        and abs(pointer_center_1[2] - pointer_center_0[2]) > 0.005,
        details=f"rest={pointer_center_0}, turned={pointer_center_1}",
    )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "selector knob is continuous",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        details=f"limits={knob_limits}",
    )

    return ctx.report()


object_model = build_object_model()
