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


BODY_DEPTH = 0.240
BODY_WIDTH = 0.300
BODY_HEIGHT = 0.120
WALL = 0.014

HINGE_AXIS_X = -BODY_DEPTH / 2.0 - 0.026
HINGE_AXIS_Z = BODY_HEIGHT + 0.010


def _body_shell() -> cq.Workplane:
    """Rounded, open-topped wooden presentation-box body."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.008)
    )
    cavity = cq.Workplane("XY").box(
        BODY_DEPTH - 2.0 * WALL,
        BODY_WIDTH - 2.0 * WALL,
        BODY_HEIGHT,
    ).translate((0.0, 0.0, WALL + BODY_HEIGHT / 2.0))
    return outer.cut(cavity)


def _lid_frame() -> cq.Workplane:
    """Wooden lid frame in the lid joint frame; rear hinge axis is at x=0."""
    lid_depth = 0.260
    lid_width = 0.310
    lid_thickness = 0.022
    rear_offset = 0.016
    x_center = rear_offset + lid_depth / 2.0
    frame_z_center = 0.001

    outer = (
        cq.Workplane("XY")
        .box(lid_depth, lid_width, lid_thickness)
        .translate((x_center, 0.0, frame_z_center))
        .edges("|Z")
        .fillet(0.006)
    )
    window_cut = cq.Workplane("XY").box(
        0.166,
        0.218,
        lid_thickness * 3.0,
    ).translate((x_center + 0.010, 0.0, frame_z_center))
    return outer.cut(window_cut)


def _cradle_pillow() -> cq.Workplane:
    """Soft rounded cushion carried by the rotating cradle."""
    return (
        cq.Workplane("XY")
        .box(0.032, 0.072, 0.046)
        .translate((0.034, 0.0, 0.0))
        .edges()
        .fillet(0.008)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    dark_walnut = model.material("dark_walnut", rgba=(0.23, 0.10, 0.035, 1.0))
    black_velvet = model.material("black_velvet", rgba=(0.010, 0.009, 0.012, 1.0))
    cream_suede = model.material("cream_suede", rgba=(0.77, 0.66, 0.50, 1.0))
    brass = model.material("brushed_brass", rgba=(0.95, 0.70, 0.30, 1.0))
    dark_metal = model.material("dark_motor_boss", rgba=(0.035, 0.035, 0.040, 1.0))
    glass = model.material("smoky_glass", rgba=(0.50, 0.66, 0.76, 0.34))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell", tolerance=0.0007),
        material=dark_walnut,
        name="body_shell",
    )
    body.visual(
        Box((BODY_DEPTH - 2.0 * WALL - 0.006, BODY_WIDTH - 2.0 * WALL - 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.002)),
        material=black_velvet,
        name="floor_liner",
    )
    body.visual(
        Box((0.006, BODY_WIDTH - 2.0 * WALL - 0.014, 0.080)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - WALL - 0.003, 0.0, 0.060)),
        material=black_velvet,
        name="front_liner",
    )
    body.visual(
        Box((0.006, BODY_WIDTH - 2.0 * WALL - 0.014, 0.080)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 + WALL + 0.003, 0.0, 0.060)),
        material=black_velvet,
        name="rear_liner",
    )
    for y, name in ((BODY_WIDTH / 2.0 - WALL - 0.003, "side_liner_0"), (-(BODY_WIDTH / 2.0 - WALL - 0.003), "side_liner_1")):
        body.visual(
            Box((BODY_DEPTH - 2.0 * WALL - 0.020, 0.006, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=black_velvet,
            name=name,
        )

    # Fixed motor stand and spindle: the cradle joint origin is on the spindle nose.
    body.visual(
        Box((0.052, 0.082, 0.012)),
        origin=Origin(xyz=(-0.040, 0.0, WALL + 0.006)),
        material=dark_metal,
        name="motor_plinth",
    )
    body.visual(
        Box((0.018, 0.048, 0.046)),
        origin=Origin(xyz=(-0.038, 0.0, 0.039)),
        material=dark_metal,
        name="spindle_post",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(-0.023, 0.0, 0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="spindle",
    )

    # Overslung hinge hardware is deliberately pulled behind the case so the
    # horizontal rear hinge axis is visually obvious.
    outer_knuckles = ((-0.106, "hinge_knuckle_0"), (0.106, "hinge_knuckle_1"))
    for y, name in outer_knuckles:
        body.visual(
            Cylinder(radius=0.007, length=0.056),
            origin=Origin(xyz=(HINGE_AXIS_X, y, HINGE_AXIS_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=name,
        )
        body.visual(
            Box((0.014, 0.056, 0.004)),
            origin=Origin(xyz=(HINGE_AXIS_X + 0.007, y, HINGE_AXIS_Z)),
            material=brass,
            name=f"hinge_leaf_{name[-1]}",
        )
        body.visual(
            Box((0.008, 0.056, 0.028)),
            origin=Origin(xyz=(HINGE_AXIS_X + 0.007, y, BODY_HEIGHT - 0.004)),
            material=brass,
            name=f"hinge_standoff_{name[-1]}",
        )
        body.visual(
            Box((0.028, 0.056, 0.004)),
            origin=Origin(xyz=(-BODY_DEPTH / 2.0 - 0.014, y, BODY_HEIGHT - 0.014)),
            material=brass,
            name=f"hinge_foot_{name[-1]}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame(), "lid_frame", tolerance=0.0007),
        material=dark_walnut,
        name="lid_frame",
    )
    lid.visual(
        Box((0.176, 0.228, 0.003)),
        origin=Origin(xyz=(0.156, 0.0, 0.002)),
        material=glass,
        name="glass_pane",
    )
    lid.visual(
        Box((0.170, 0.218, 0.005)),
        origin=Origin(xyz=(0.156, 0.0, -0.0125)),
        material=black_velvet,
        name="underside_pad",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="moving_knuckle",
    )
    lid.visual(
        Box((0.038, 0.112, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.0005)),
        material=brass,
        name="moving_leaf",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hub",
    )
    cradle.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_velvet,
        name="cradle_disk",
    )
    cradle.visual(
        mesh_from_cadquery(_cradle_pillow(), "cradle_pillow", tolerance=0.0007),
        material=cream_suede,
        name="pillow",
    )
    cradle.visual(
        Box((0.010, 0.018, 0.006)),
        origin=Origin(xyz=(0.023, 0.0, 0.046)),
        material=brass,
        name="top_marker",
    )
    cradle.visual(
        Box((0.012, 0.086, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, -0.044)),
        material=brass,
        name="lower_keeper",
    )
    cradle.visual(
        Box((0.012, 0.006, 0.078)),
        origin=Origin(xyz=(0.028, 0.043, 0.0)),
        material=brass,
        name="side_keeper_0",
    )
    cradle.visual(
        Box((0.012, 0.006, 0.078)),
        origin=Origin(xyz=(0.028, -0.043, 0.0)),
        material=brass,
        name="side_keeper_1",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(-0.010, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")

    ctx.check(
        "lid joint is horizontal rear revolute",
        lid_joint.articulation_type == ArticulationType.REVOLUTE and tuple(lid_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}",
    )
    ctx.check(
        "cradle has continuous spindle rotation",
        cradle_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(cradle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={cradle_joint.articulation_type}, axis={cradle_joint.axis}",
    )

    with ctx.pose({lid_joint: 0.0, cradle_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_frame",
            negative_elem="body_shell",
            max_gap=0.0015,
            max_penetration=0.0,
            name="closed lid frame sits on body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_frame",
            elem_b="body_shell",
            min_overlap=0.20,
            name="lid covers the box body",
        )
        ctx.expect_contact(
            cradle,
            body,
            elem_a="hub",
            elem_b="spindle",
            contact_tol=0.001,
            name="cradle hub bears on spindle nose",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.35}):
        opened_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "positive lid angle opens upward",
        closed_aabb is not None and opened_aabb is not None and opened_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    marker_rest = ctx.part_element_world_aabb(cradle, elem="top_marker")
    with ctx.pose({cradle_joint: math.pi / 2.0}):
        marker_quarter = ctx.part_element_world_aabb(cradle, elem="top_marker")
    ctx.check(
        "cradle marker follows continuous rotation",
        marker_rest is not None
        and marker_quarter is not None
        and marker_quarter[0][1] < marker_rest[0][1] - 0.030
        and marker_quarter[1][2] < marker_rest[1][2] - 0.020,
        details=f"rest={marker_rest}, quarter_turn={marker_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
