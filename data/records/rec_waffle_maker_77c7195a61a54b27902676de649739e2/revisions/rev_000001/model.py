from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.285
BODY_DEPTH = 0.275
BODY_HEIGHT = 0.082
LID_HEIGHT = 0.050
CORNER_RADIUS = 0.022
WALL_THICKNESS = 0.006
BOTTOM_THICKNESS = 0.011
TOP_THICKNESS = 0.009

HINGE_AXIS_Y = -0.150
HINGE_AXIS_Z = 0.083
HINGE_BLOCK_WIDTH = 0.180
HINGE_BLOCK_DEPTH = 0.032
HINGE_BLOCK_HEIGHT = 0.027
HINGE_BLOCK_Z = 0.044
HINGE_BARREL_RADIUS = 0.012

PLATE_WIDTH = 0.216
PLATE_DEPTH = 0.214
LOWER_PLATE_THICKNESS = 0.010
LOWER_PLATE_Z = 0.060
UPPER_PLATE_THICKNESS = 0.008
UPPER_PLATE_Z = 0.004


def _x_axis_cylinder(length: float, radius: float, *, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((-length / 2.0, y, z))
    )


def _waffle_plate(
    width: float,
    depth: float,
    thickness: float,
    *,
    support_height: float,
    support_direction: str = "down",
) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, thickness / 2.0))
        .box(width, depth, thickness)
        .edges("|Z")
        .fillet(0.004)
    )

    groove_depth = thickness * 0.38
    groove_width = 0.007
    usable_width = width - 0.040
    usable_depth = depth - 0.040
    x_positions = [-usable_width / 4.0, 0.0, usable_width / 4.0]
    y_positions = [-usable_depth / 4.0, 0.0, usable_depth / 4.0]

    for x_pos in x_positions:
        cutter = (
            cq.Workplane("XY")
            .transformed(offset=(x_pos, 0.0, thickness - groove_depth / 2.0))
            .box(groove_width, depth - 0.026, groove_depth + 0.002)
        )
        plate = plate.cut(cutter)

    for y_pos in y_positions:
        cutter = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, y_pos, thickness - groove_depth / 2.0))
            .box(width - 0.026, groove_width, groove_depth + 0.002)
        )
        plate = plate.cut(cutter)

    if support_direction == "up":
        support_center_z = thickness + support_height / 2.0
    else:
        support_center_z = -support_height / 2.0

    support = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, support_center_z))
        .box(width * 0.56, depth * 0.56, support_height)
        .edges("|Z")
        .fillet(0.004)
    )
    return plate.union(support)


def _body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BODY_HEIGHT / 2.0))
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )

    inner_height = BODY_HEIGHT - BOTTOM_THICKNESS + 0.004
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BOTTOM_THICKNESS + inner_height / 2.0))
        .box(
            BODY_WIDTH - 2.0 * WALL_THICKNESS,
            BODY_DEPTH - 2.0 * WALL_THICKNESS,
            inner_height,
        )
        .edges("|Z")
        .fillet(max(CORNER_RADIUS - WALL_THICKNESS, 0.006))
    )

    shell = outer.cut(inner)

    lower_plate = _waffle_plate(
        PLATE_WIDTH,
        PLATE_DEPTH,
        LOWER_PLATE_THICKNESS,
        support_height=LOWER_PLATE_Z - BOTTOM_THICKNESS + 0.001,
    ).translate((0.0, 0.0, LOWER_PLATE_Z))

    return shell.union(lower_plate)


def _lid_shape() -> cq.Workplane:
    lid_center_y = BODY_DEPTH / 2.0 - 0.006
    lid_center_z = 0.026

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, lid_center_y, lid_center_z))
        .box(BODY_WIDTH, BODY_DEPTH, LID_HEIGHT)
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )

    inner_height = LID_HEIGHT - TOP_THICKNESS + 0.004
    lid_bottom_z = lid_center_z - LID_HEIGHT / 2.0
    inner = (
        cq.Workplane("XY")
        .transformed(
            offset=(
                0.0,
                lid_center_y,
                lid_bottom_z - 0.002 + inner_height / 2.0,
            )
        )
        .box(
            BODY_WIDTH - 2.0 * WALL_THICKNESS,
            BODY_DEPTH - 2.0 * WALL_THICKNESS,
            inner_height,
        )
        .edges("|Z")
        .fillet(max(CORNER_RADIUS - WALL_THICKNESS, 0.006))
    )

    shell = outer.cut(inner)

    upper_plate = _waffle_plate(
        PLATE_WIDTH,
        PLATE_DEPTH,
        UPPER_PLATE_THICKNESS,
        support_height=(lid_center_z + LID_HEIGHT / 2.0 - TOP_THICKNESS)
        - (UPPER_PLATE_Z + UPPER_PLATE_THICKNESS)
        + 0.006,
        support_direction="up",
    ).translate((0.0, lid_center_y, UPPER_PLATE_Z))

    rear_bridge = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.008, 0.012))
        .box(0.190, 0.024, 0.024)
        .edges("|X")
        .fillet(0.004)
    )

    return shell.union(upper_plate).union(rear_bridge)


def _handle_shape() -> cq.Workplane:
    grip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.022, 0.0))
        .box(0.098, 0.022, 0.018)
        .edges("|Y")
        .fillet(0.004)
    )

    foot_a = (
        cq.Workplane("XY")
        .transformed(offset=(-0.034, 0.009, 0.0))
        .box(0.020, 0.018, 0.018)
        .edges("|Y")
        .fillet(0.003)
    )
    foot_b = (
        cq.Workplane("XY")
        .transformed(offset=(0.034, 0.009, 0.0))
        .box(0.020, 0.018, 0.018)
        .edges("|Y")
        .fillet(0.003)
    )

    return grip.union(foot_a).union(foot_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_waffle_maker")

    housing_finish = model.material("housing_finish", rgba=(0.76, 0.77, 0.79, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.25, 0.26, 0.28, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.15, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=housing_finish,
        name="body_shell",
    )
    body.visual(
        Box((HINGE_BLOCK_WIDTH, 0.036, 0.028)),
        origin=Origin(
            xyz=(0.0, HINGE_AXIS_Y - 0.004, 0.058),
        ),
        material=trim_finish,
        name="hinge_block",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(
            xyz=(BODY_WIDTH / 2.0 - 0.010, 0.056, 0.040),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_finish,
        name="dial_socket",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lid_shell"),
        material=housing_finish,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.011, length=BODY_WIDTH - 0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )

    handle = model.part("front_handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "front_handle"),
        material=handle_finish,
        name="handle_body",
    )

    dial = model.part("thermostat_dial")
    dial.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.018,
                body_style="tapered",
                top_diameter=0.031,
                base_diameter=0.036,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "thermostat_dial",
        ),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_finish,
        name="dial_knob",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=10.0, velocity=1.8),
    )

    model.articulation(
        "lid_to_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, BODY_DEPTH - 0.006, 0.018)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_WIDTH / 2.0, 0.056, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("front_handle")
    dial = object_model.get_part("thermostat_dial")

    lid_hinge = object_model.get_articulation("body_to_lid")
    dial_joint = object_model.get_articulation("body_to_dial")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="hinge_barrel",
            elem_b="hinge_block",
            contact_tol=0.0008,
            name="rear hinge keeps the lid supported at rest",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.18,
            name="lid covers the lower housing footprint when closed",
        )
        ctx.expect_gap(
            dial,
            body,
            axis="x",
            positive_elem="dial_shaft",
            negative_elem="body_shell",
            max_gap=0.0015,
            max_penetration=0.0005,
            name="thermostat dial seats on the body side wall",
        )

    closed_handle_pos = ctx.part_world_position(handle)
    upper_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if upper_limit is not None:
        with ctx.pose({lid_hinge: upper_limit}):
            opened_handle_pos = ctx.part_world_position(handle)
        ctx.check(
            "lid lifts upward from the rear hinge",
            closed_handle_pos is not None
            and opened_handle_pos is not None
            and opened_handle_pos[2] > closed_handle_pos[2] + 0.12,
            details=f"closed={closed_handle_pos}, opened={opened_handle_pos}",
        )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.7}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "thermostat dial rotates in place",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_rest[0] - dial_turned[0]) < 1e-6
        and abs(dial_rest[1] - dial_turned[1]) < 1e-6
        and abs(dial_rest[2] - dial_turned[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    return ctx.report()


object_model = build_object_model()
