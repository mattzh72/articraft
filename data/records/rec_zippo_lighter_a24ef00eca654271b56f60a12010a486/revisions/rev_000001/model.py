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


CASE_W = 0.0385
CASE_D = 0.0135
BODY_H = 0.0385
LID_H = 0.0180

CASE_WALL = 0.00045
CASE_BOTTOM = 0.00075
LID_WALL = 0.00040
LID_CLEAR = 0.00015

LID_OUTER_W = CASE_W + 2.0 * (LID_WALL + LID_CLEAR)
LID_OUTER_D = CASE_D + 2.0 * (LID_WALL + LID_CLEAR)
LID_LEFT_FROM_AXIS = 0.00035
LID_BOTTOM_FROM_AXIS = -0.00100

HINGE_RADIUS = 0.00105
SHELL_KNUCKLE_LEN = 0.00440
KNUCKLE_GAP = 0.00040
LID_KNUCKLE_LEN = (LID_OUTER_D - SHELL_KNUCKLE_LEN - 2.0 * KNUCKLE_GAP) / 2.0
HINGE_AXIS_X = -(LID_LEFT_FROM_AXIS + (LID_OUTER_W * 0.5))
HINGE_AXIS_Z = 0.03830

INSERT_W = 0.0305
INSERT_D = 0.0108
INSERT_BODY_H = 0.0315
CHIMNEY_W = 0.0195
CHIMNEY_D = 0.0102
CHIMNEY_H = 0.0130
CHIMNEY_WALL = 0.00065
CHIMNEY_Z0 = INSERT_BODY_H - 0.0010
WHEEL_CENTER_Z = 0.03870
WHEEL_RADIUS = 0.00435
WHEEL_WIDTH = 0.00520
AXLE_RADIUS = 0.00068
AXLE_LEN = CHIMNEY_D - 2.0 * CHIMNEY_WALL - 0.00020


def _y_cylinder(radius: float, length: float, *, x: float, y: float, z: float):
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((x, y - (length * 0.5), z))
    )


def _z_cylinder(radius: float, length: float, *, x: float, y: float, z0: float):
    return cq.Workplane("XY").circle(radius).extrude(length).translate((x, y, z0))


def _make_insert_shape():
    body = (
        cq.Workplane("XY")
        .box(INSERT_W, INSERT_D, INSERT_BODY_H)
        .translate((0.0, 0.0, INSERT_BODY_H * 0.5))
        .edges("|Z")
        .fillet(0.00080)
    )

    chimney_outer = (
        cq.Workplane("XY")
        .box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H)
        .translate((0.0, 0.0, CHIMNEY_Z0 + (CHIMNEY_H * 0.5)))
        .edges("|Z")
        .fillet(0.00055)
    )
    chimney_inner = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_W - 2.0 * CHIMNEY_WALL,
            CHIMNEY_D - 2.0 * CHIMNEY_WALL,
            CHIMNEY_H - CHIMNEY_WALL + 0.0010,
        )
        .translate(
            (
                0.0,
                0.0,
                CHIMNEY_Z0 + ((CHIMNEY_H - CHIMNEY_WALL + 0.0010) * 0.5),
            )
        )
    )
    insert = body.union(chimney_outer.cut(chimney_inner))

    flint_tube = _z_cylinder(radius=0.00180, length=0.00950, x=0.0, y=0.0, z0=0.02860)
    insert = insert.union(flint_tube)

    hole_cutter = None
    x_positions = (-0.0052, 0.0, 0.0052)
    z_positions = (0.0346, 0.0382)
    for x in x_positions:
        for z in z_positions:
            hole = _y_cylinder(
                0.00105,
                CHIMNEY_D + 0.0030,
                x=x,
                y=0.0,
                z=z,
            )
            hole_cutter = hole if hole_cutter is None else hole_cutter.union(hole)

    return insert.cut(hole_cutter)
def _make_wheel_shape():
    wheel = (
        cq.Workplane("XZ")
        .polygon(20, WHEEL_RADIUS * 1.92)
        .extrude(WHEEL_WIDTH)
        .translate((0.0, -(WHEEL_WIDTH * 0.5), 0.0))
    )
    axle = _y_cylinder(AXLE_RADIUS, AXLE_LEN, x=0.0, y=0.0, z=0.0)
    return wheel.union(axle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pocket_lighter")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    shell = model.part("shell")
    shell_wall_h = BODY_H - CASE_BOTTOM
    shell.visual(
        Box((CASE_W, CASE_D, CASE_BOTTOM)),
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM * 0.5)),
        material=brushed_steel,
        name="shell_bottom",
    )
    shell.visual(
        Box((CASE_WALL, CASE_D, shell_wall_h)),
        origin=Origin(
            xyz=(-(CASE_W * 0.5) + (CASE_WALL * 0.5), 0.0, CASE_BOTTOM + (shell_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="shell_hinge_side",
    )
    shell.visual(
        Box((CASE_WALL, CASE_D, shell_wall_h)),
        origin=Origin(
            xyz=((CASE_W * 0.5) - (CASE_WALL * 0.5), 0.0, CASE_BOTTOM + (shell_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="shell_free_side",
    )
    shell.visual(
        Box((CASE_W - 2.0 * CASE_WALL, CASE_WALL, shell_wall_h)),
        origin=Origin(
            xyz=(0.0, (CASE_D * 0.5) - (CASE_WALL * 0.5), CASE_BOTTOM + (shell_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="shell_front",
    )
    shell.visual(
        Box((CASE_W - 2.0 * CASE_WALL, CASE_WALL, shell_wall_h)),
        origin=Origin(
            xyz=(0.0, -((CASE_D * 0.5) - (CASE_WALL * 0.5)), CASE_BOTTOM + (shell_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="shell_rear",
    )
    shell.visual(
        Cylinder(radius=HINGE_RADIUS, length=SHELL_KNUCKLE_LEN),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z), rpy=(pi * 0.5, 0.0, 0.0)),
        material=brushed_steel,
        name="shell_knuckle",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_make_insert_shape(), "lighter_insert"),
        material=satin_steel,
        name="insert_body",
    )

    lid = model.part("lid")
    lid_wall_h = LID_H - LID_WALL
    lid_center_x = LID_LEFT_FROM_AXIS + (LID_OUTER_W * 0.5)
    lid.visual(
        Box((LID_OUTER_W, LID_OUTER_D, LID_WALL)),
        origin=Origin(xyz=(lid_center_x, 0.0, LID_BOTTOM_FROM_AXIS + LID_H - (LID_WALL * 0.5))),
        material=brushed_steel,
        name="lid_top",
    )
    lid.visual(
        Box((LID_WALL, LID_OUTER_D, lid_wall_h)),
        origin=Origin(
            xyz=(LID_LEFT_FROM_AXIS + (LID_WALL * 0.5), 0.0, LID_BOTTOM_FROM_AXIS + (lid_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="lid_hinge_side",
    )
    lid.visual(
        Box((LID_WALL, LID_OUTER_D, lid_wall_h)),
        origin=Origin(
            xyz=(
                LID_LEFT_FROM_AXIS + LID_OUTER_W - (LID_WALL * 0.5),
                0.0,
                LID_BOTTOM_FROM_AXIS + (lid_wall_h * 0.5),
            )
        ),
        material=brushed_steel,
        name="lid_free_side",
    )
    lid.visual(
        Box((LID_OUTER_W - 2.0 * LID_WALL, LID_WALL, lid_wall_h)),
        origin=Origin(
            xyz=(lid_center_x, (LID_OUTER_D * 0.5) - (LID_WALL * 0.5), LID_BOTTOM_FROM_AXIS + (lid_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="lid_front",
    )
    lid.visual(
        Box((LID_OUTER_W - 2.0 * LID_WALL, LID_WALL, lid_wall_h)),
        origin=Origin(
            xyz=(lid_center_x, -((LID_OUTER_D * 0.5) - (LID_WALL * 0.5)), LID_BOTTOM_FROM_AXIS + (lid_wall_h * 0.5))
        ),
        material=brushed_steel,
        name="lid_rear",
    )
    front_knuckle_y = (SHELL_KNUCKLE_LEN * 0.5) + KNUCKLE_GAP + (LID_KNUCKLE_LEN * 0.5)
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=LID_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, front_knuckle_y, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=brushed_steel,
        name="lid_knuckle_0",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=LID_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, -front_knuckle_y, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=brushed_steel,
        name="lid_knuckle_1",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_make_wheel_shape(), "lighter_wheel"),
        material=dark_steel,
        name="spark_wheel",
    )

    model.articulation(
        "shell_to_insert",
        ArticulationType.FIXED,
        parent=shell,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM)),
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=8.0, lower=0.0, upper=2.05),
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("wheel")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    wheel_spin = object_model.get_articulation("insert_to_wheel")

    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        min_overlap=0.010,
        name="closed lid covers the shell footprint",
    )
    ctx.expect_origin_gap(
        wheel,
        insert,
        axis="z",
        min_gap=0.037,
        max_gap=0.0405,
        name="spark wheel sits above the insert body",
    )
    ctx.expect_origin_distance(
        wheel,
        insert,
        axes="x",
        max_dist=0.001,
        name="spark wheel stays centered on the insert",
    )
    ctx.expect_contact(
        wheel,
        insert,
        name="spark wheel axle bears against the chimney support faces",
    )

    closed_lid = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid swings upward from the shell",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.010
        and open_lid[0][0] < closed_lid[0][0] - 0.006,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: pi * 0.5}):
        spun_wheel_pos = ctx.part_world_position(wheel)
        ctx.expect_contact(
            wheel,
            insert,
            name="wheel remains mounted while spun",
        )
    ctx.check(
        "wheel spins about a fixed axle center",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and abs(rest_wheel_pos[0] - spun_wheel_pos[0]) < 1e-6
        and abs(rest_wheel_pos[1] - spun_wheel_pos[1]) < 1e-6
        and abs(rest_wheel_pos[2] - spun_wheel_pos[2]) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
