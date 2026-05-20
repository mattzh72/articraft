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


BODY_WIDTH = 0.48
BODY_DEPTH = 0.38
BODY_HEIGHT = 0.20

LID_WIDTH = 0.47
LID_DEPTH = 0.36
LID_HEIGHT = 0.055

HINGE_Y = 0.175
HINGE_Z = BODY_HEIGHT

PANEL_Y = -BODY_DEPTH / 2.0 - 0.007
PANEL_Z = 0.083
PANEL_THICKNESS = 0.014
PANEL_HEIGHT = 0.135

DIAL_CENTER = (0.0, -BODY_DEPTH / 2.0 - PANEL_THICKNESS, 0.070)
DIAL_RADIUS = 0.043
DIAL_THICKNESS = 0.024


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A small utility for appliance panels and molded plastic caps."""
    safe_radius = min(radius, size[0] * 0.42, size[2] * 0.42)
    return (
        cq.Workplane("XY")
        .box(*size)
        .edges("|Y")
        .fillet(safe_radius)
    )


def _cooker_body_shell() -> cq.Workplane:
    """Low, broad rounded rectangular cooker body with softened top shoulders."""
    return (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.065)
        .edges(">Z")
        .fillet(0.026)
        .edges("<Z")
        .fillet(0.012)
    )


def _lid_cover() -> cq.Workplane:
    """Shallow rounded lid whose local origin lies on the rear hinge line."""
    return (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_HEIGHT, centered=(True, True, False))
        .translate((0.0, -LID_DEPTH / 2.0, 0.0))
        .edges("|Z")
        .fillet(0.055)
        .edges(">Z")
        .fillet(0.022)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_size_rice_cooker")

    shell_mat = model.material("warm_white_plastic", rgba=(0.92, 0.90, 0.82, 1.0))
    lid_mat = model.material("slightly_glossier_lid", rgba=(0.96, 0.95, 0.88, 1.0))
    panel_mat = model.material("charcoal_control_panel", rgba=(0.055, 0.060, 0.065, 1.0))
    trim_mat = model.material("satin_gray_trim", rgba=(0.45, 0.47, 0.47, 1.0))
    dark_mat = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    red_mat = model.material("red_cook_dot", rgba=(0.95, 0.08, 0.035, 1.0))
    green_mat = model.material("green_warm_dot", rgba=(0.05, 0.75, 0.18, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_cooker_body_shell(), "body_shell", tolerance=0.0015),
        material=shell_mat,
        name="body_shell",
    )
    shell.visual(
        mesh_from_cadquery(_rounded_box((0.185, PANEL_THICKNESS, PANEL_HEIGHT), 0.018), "control_panel"),
        origin=Origin(xyz=(0.0, PANEL_Y, PANEL_Z)),
        material=panel_mat,
        name="control_panel",
    )
    shell.visual(
        Cylinder(radius=0.010, length=0.44),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0, BODY_HEIGHT - 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="rear_hinge_band",
    )
    shell.visual(
        Box((0.110, 0.009, 0.018)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 - 0.010, 0.137)),
        material=trim_mat,
        name="control_latch_bezel",
    )
    for x, z, mat, name in (
        (-0.056, 0.122, green_mat, "warm_indicator"),
        (0.056, 0.122, red_mat, "cook_indicator"),
    ):
        shell.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(xyz=(x, -BODY_DEPTH / 2.0 - 0.016, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=name,
        )
    for x, y, name in (
        (-0.160, -0.122, "foot_0"),
        (0.160, -0.122, "foot_1"),
        (-0.160, 0.122, "foot_2"),
        (0.160, 0.122, "foot_3"),
    ):
        shell.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(x, y, -0.004)),
            material=dark_mat,
            name=name,
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cover(), "lid_cover", tolerance=0.0015),
        material=lid_mat,
        name="lid_cover",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_box((0.150, 0.075, 0.023), 0.018), "top_handle"),
        origin=Origin(xyz=(0.0, -0.205, LID_HEIGHT + 0.011)),
        material=trim_mat,
        name="top_handle",
    )
    lid.visual(
        Box((0.105, 0.020, 0.035)),
        origin=Origin(xyz=(0.0, -LID_DEPTH - 0.007, 0.022)),
        material=trim_mat,
        name="front_latch",
    )
    lid.visual(
        Box((0.385, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, 0.012)),
        material=trim_mat,
        name="lid_hinge_leaf",
    )

    dial = model.part("selector_dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_THICKNESS),
        # The cylinder axis is the front-facing axle.  It touches the panel at local y=0.
        origin=Origin(xyz=(0.0, -DIAL_THICKNESS / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="dial_cap",
    )
    dial.visual(
        Box((0.010, 0.004, 0.037)),
        origin=Origin(xyz=(0.0, -DIAL_THICKNESS - 0.0015, 0.017)),
        material=panel_mat,
        name="pointer_mark",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "selector_axle",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=dial,
        origin=Origin(xyz=DIAL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("selector_dial")
    lid_hinge = object_model.get_articulation("lid_hinge")
    selector_axle = object_model.get_articulation("selector_axle")

    ctx.check(
        "low broad cooker proportions",
        BODY_WIDTH > BODY_HEIGHT * 2.0 and BODY_DEPTH > BODY_HEIGHT * 1.6,
        details=f"width={BODY_WIDTH}, depth={BODY_DEPTH}, height={BODY_HEIGHT}",
    )
    ctx.check(
        "rear hinge axis is horizontal",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0) and lid_hinge.origin.xyz[1] > 0.15,
        details=f"axis={lid_hinge.axis}, origin={lid_hinge.origin.xyz}",
    )
    ctx.check(
        "selector dial uses continuous axle",
        selector_axle.articulation_type == ArticulationType.CONTINUOUS and tuple(selector_axle.axis) == (0.0, 1.0, 0.0),
        details=f"type={selector_axle.articulation_type}, axis={selector_axle.axis}",
    )

    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_cover",
        negative_elem="body_shell",
        min_gap=0.0,
        max_gap=0.003,
        name="closed lid rests on body rim",
    )
    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_cover",
        elem_b="body_shell",
        min_overlap=0.28,
        name="lid covers broad cooker opening",
    )
    ctx.expect_gap(
        shell,
        dial,
        axis="y",
        positive_elem="control_panel",
        negative_elem="dial_cap",
        min_gap=0.0,
        max_gap=0.003,
        name="selector dial is seated on control panel",
    )
    ctx.expect_gap(
        lid,
        dial,
        axis="z",
        positive_elem="front_latch",
        negative_elem="dial_cap",
        min_gap=0.075,
        name="dial sits below the latch",
    )

    rest_latch_aabb = ctx.part_element_world_aabb(lid, elem="front_latch")
    with ctx.pose({lid_hinge: 1.10}):
        open_latch_aabb = ctx.part_element_world_aabb(lid, elem="front_latch")
    ctx.check(
        "lid opens upward about rear hinge",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[0][2] > rest_latch_aabb[0][2] + 0.12,
        details=f"closed={rest_latch_aabb}, open={open_latch_aabb}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(dial, elem="pointer_mark")
    with ctx.pose({selector_axle: math.pi / 2.0}):
        turned_pointer_aabb = ctx.part_element_world_aabb(dial, elem="pointer_mark")
    ctx.check(
        "selector dial visibly rotates",
        rest_pointer_aabb is not None
        and turned_pointer_aabb is not None
        and turned_pointer_aabb[1][0] > rest_pointer_aabb[1][0] + 0.012,
        details=f"rest={rest_pointer_aabb}, turned={turned_pointer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
