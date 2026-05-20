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


BASE_LENGTH = 0.292
BASE_WIDTH = 0.236
BODY_HEIGHT = 0.064
FOOT_HEIGHT = 0.006
BASE_TOP_Z = FOOT_HEIGHT + BODY_HEIGHT
BASE_WALL = 0.009
BASE_FLOOR = 0.010

LID_LENGTH = 0.286
LID_WIDTH = 0.232
LID_HEIGHT = 0.060
LID_BOTTOM_Z = -0.004
LID_REAR_OFFSET = 0.007

HINGE_X = -0.128
HINGE_Z = 0.078

STRAP_HINGE_X = 0.126
STRAP_HINGE_Z = 0.032
STRAP_WIDTH = 0.080

KNOB_X = 0.095
KNOB_Y = 0.149
CONTROL_POD_Y = 0.143
CONTROL_POD_TOP_Z = 0.086


def _make_base_body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, FOOT_HEIGHT))
        .edges("|Z")
        .fillet(0.020)
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            BASE_LENGTH - 2.0 * BASE_WALL,
            BASE_WIDTH - 2.0 * BASE_WALL,
            BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, FOOT_HEIGHT + BASE_FLOOR))
    )
    return outer.cut(cavity)


def _make_lid_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(LID_LENGTH, LID_WIDTH, LID_HEIGHT, centered=(False, True, False))
        .translate((LID_REAR_OFFSET, 0.0, LID_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.016)
    )
    crown = (
        cq.Workplane("XY")
        .box(0.150, 0.165, 0.010, centered=(False, True, False))
        .translate((LID_REAR_OFFSET + 0.068, 0.0, LID_BOTTOM_Z + LID_HEIGHT - 0.001))
        .edges("|Z")
        .fillet(0.006)
    )
    nose = (
        cq.Workplane("XY")
        .box(0.060, 0.175, 0.012, centered=(False, True, False))
        .translate((LID_REAR_OFFSET + LID_LENGTH - 0.066, 0.0, LID_BOTTOM_Z + 0.022))
        .edges("|Z")
        .fillet(0.005)
    )
    recess = (
        cq.Workplane("XY")
        .box(
            LID_LENGTH - 0.030,
            LID_WIDTH - 0.028,
            0.034,
            centered=(False, True, False),
        )
        .translate((LID_REAR_OFFSET + 0.016, 0.0, LID_BOTTOM_Z))
    )
    return shell.union(crown).union(nose).cut(recess)


def _make_knob_body_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(0.010)
        .faces(">Z")
        .workplane()
        .circle(0.014)
        .extrude(0.010)
        .edges(">Z")
        .fillet(0.0025)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_sandwich_clamshell_press")

    base_finish = model.material("base_finish", rgba=(0.19, 0.20, 0.21, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.80, 0.81, 0.83, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    foot_finish = model.material("foot_finish", rgba=(0.05, 0.05, 0.05, 1.0))
    knob_mark_finish = model.material("knob_mark_finish", rgba=(0.88, 0.62, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_body_shape(), "base_body"),
        material=base_finish,
        name="body",
    )
    base.visual(
        Box((0.060, 0.052, 0.016)),
        origin=Origin(xyz=(0.098, CONTROL_POD_Y, BASE_TOP_Z + 0.008)),
        material=base_finish,
        name="control_pod",
    )
    base.visual(
        Box((0.060, 0.092, 0.010)),
        origin=Origin(xyz=(0.110, 0.0, 0.022)),
        material=base_finish,
        name="latch_mount",
    )
    for index, x_sign in enumerate((-1.0, 1.0)):
        base.visual(
            Box((0.035, 0.035, FOOT_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_sign * (BASE_LENGTH / 2.0 - 0.044),
                    BASE_WIDTH / 2.0 - 0.040,
                    FOOT_HEIGHT / 2.0,
                )
            ),
            material=foot_finish,
            name=f"foot_{index * 2}",
        )
        base.visual(
            Box((0.035, 0.035, FOOT_HEIGHT)),
            origin=Origin(
                xyz=(
                    x_sign * (BASE_LENGTH / 2.0 - 0.044),
                    -BASE_WIDTH / 2.0 + 0.040,
                    FOOT_HEIGHT / 2.0,
                )
            ),
            material=foot_finish,
            name=f"foot_{index * 2 + 1}",
        )
    for index, y_center in enumerate((-0.071, 0.071)):
        base.visual(
            Box((0.022, 0.052, 0.016)),
            origin=Origin(xyz=(HINGE_X - 0.003, y_center, 0.070)),
            material=base_finish,
            name=f"hinge_mount_{index}",
        )
        base.visual(
            Cylinder(radius=0.007, length=0.052),
            origin=Origin(
                xyz=(HINGE_X, y_center, HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_finish,
            name=f"hinge_barrel_{index}",
        )
    base.visual(
        Box((0.220, 0.180, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + BASE_FLOOR + 0.002)),
        material=plate_finish,
        name="lower_plate",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell_shape(), "lid_shell"),
        material=lid_finish,
        name="shell",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.084),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_finish,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.260, 0.206, 0.004)),
        origin=Origin(xyz=(LID_REAR_OFFSET + 0.143, 0.0, 0.000)),
        material=plate_finish,
        name="upper_plate",
    )
    lid.visual(
        Box((0.028, 0.082, 0.006)),
        origin=Origin(xyz=(LID_REAR_OFFSET + LID_LENGTH - 0.014, 0.0, -0.002)),
        material=trim_finish,
        name="catch",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.005, length=STRAP_WIDTH),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="barrel",
    )
    latch.visual(
        Box((0.006, STRAP_WIDTH - 0.010, 0.040)),
        origin=Origin(xyz=(0.006, 0.0, 0.020)),
        material=trim_finish,
        name="strap",
    )
    latch.visual(
        Box((0.026, STRAP_WIDTH - 0.008, 0.007)),
        origin=Origin(xyz=(0.019, 0.0, 0.037)),
        material=trim_finish,
        name="hook",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_cadquery(_make_knob_body_shape(), "browning_knob"),
        material=trim_finish,
        name="body",
    )
    knob.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_finish,
        name="stem",
    )
    knob.visual(
        Box((0.016, 0.004, 0.003)),
        origin=Origin(xyz=(0.010, 0.0, 0.019)),
        material=knob_mark_finish,
        name="pointer",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "base_to_latch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch,
        origin=Origin(xyz=(STRAP_HINGE_X, 0.0, STRAP_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=2.0, velocity=3.0),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, CONTROL_POD_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    knob = object_model.get_part("knob")

    lid_joint = object_model.get_articulation("base_to_lid")
    latch_joint = object_model.get_articulation("base_to_latch")
    knob_joint = object_model.get_articulation("base_to_knob")

    with ctx.pose({lid_joint: 0.0, latch_joint: 0.0, knob_joint: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="shell",
            negative_elem="body",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid sits just above the lower frame",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="shell",
            elem_b="body",
            min_overlap=0.180,
            name="closed lid covers the lower cooking footprint",
        )
        ctx.expect_overlap(
            latch,
            lid,
            axes="xy",
            elem_a="hook",
            elem_b="catch",
            min_overlap=0.020,
            name="closed latch hook lines up with the lid catch",
        )
        ctx.expect_gap(
            lid,
            latch,
            axis="z",
            positive_elem="catch",
            negative_elem="hook",
            min_gap=0.0003,
            max_gap=0.006,
            name="closed latch hook tucks just under the lid catch",
        )
        ctx.expect_contact(
            knob,
            base,
            elem_a="stem",
            elem_b="control_pod",
            contact_tol=0.001,
            name="browning knob stem stays mounted on the control pod",
        )
        closed_catch = ctx.part_element_world_aabb(lid, elem="catch")
        closed_hook = ctx.part_element_world_aabb(latch, elem="hook")
        rest_pointer = ctx.part_element_world_aabb(knob, elem="pointer")

    with ctx.pose({lid_joint: 1.18, latch_joint: 1.0}):
        open_catch = ctx.part_element_world_aabb(lid, elem="catch")

    ctx.check(
        "lid front rises clear when opened",
        closed_catch is not None
        and open_catch is not None
        and open_catch[1][2] > closed_catch[1][2] + 0.100,
        details=f"closed_catch={closed_catch}, open_catch={open_catch}",
    )

    with ctx.pose({latch_joint: 1.0}):
        open_hook = ctx.part_element_world_aabb(latch, elem="hook")

    ctx.check(
        "latch strap swings forward to unlatch",
        closed_hook is not None
        and open_hook is not None
        and open_hook[1][0] > closed_hook[1][0] + 0.014
        and open_hook[1][2] < closed_hook[1][2] - 0.010,
        details=f"closed_hook={closed_hook}, open_hook={open_hook}",
    )

    with ctx.pose({knob_joint: math.pi / 2.0}):
        quarter_turn_pointer = ctx.part_element_world_aabb(knob, elem="pointer")

    rest_pointer_center = _aabb_center(rest_pointer)
    quarter_turn_center = _aabb_center(quarter_turn_pointer)
    ctx.check(
        "browning knob pointer rotates around the dial axis",
        rest_pointer_center is not None
        and quarter_turn_center is not None
        and quarter_turn_center[1] > rest_pointer_center[1] + 0.007
        and quarter_turn_center[0] < rest_pointer_center[0] - 0.007,
        details=(
            f"rest_pointer_center={rest_pointer_center}, "
            f"quarter_turn_center={quarter_turn_center}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
