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
    mesh_from_geometry,
    wire_from_points,
)


BODY_LENGTH = 2.10
BODY_DEPTH = 0.64
BODY_HEIGHT = 0.82
WALL = 0.040
RIM_HEIGHT = 0.035

LID_LENGTH = 2.18
LID_DEPTH = 0.74
LID_THICKNESS = 0.075

LID_PIVOT_Y = BODY_DEPTH / 2.0 + 0.050
LID_PIVOT_Z = BODY_HEIGHT + RIM_HEIGHT - 0.001 + LID_THICKNESS / 2.0
LID_CENTER_Y = -0.396

HANDLE_PIVOT_Y = LID_CENTER_Y - LID_DEPTH / 2.0 - 0.034
HANDLE_PIVOT_Z = -0.005


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_chest_body_bag_freezer")

    enamel_white = model.material("enamel_white", color=(0.94, 0.96, 0.96, 1.0))
    liner_gray = model.material("liner_gray", color=(0.55, 0.58, 0.60, 1.0))
    gasket_black = model.material("gasket_black", color=(0.015, 0.017, 0.018, 1.0))
    stainless = model.material("stainless", color=(0.73, 0.74, 0.72, 1.0))

    body_shell = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_DEPTH, BODY_HEIGHT)
        .faces(">Z")
        .shell(-WALL)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
    )
    rim_outer_x = BODY_LENGTH + 0.060
    rim_outer_y = BODY_DEPTH + 0.060
    rim_z = BODY_HEIGHT + RIM_HEIGHT / 2.0 - 0.001
    top_rim = (
        cq.Workplane("XY")
        .box(rim_outer_x, rim_outer_y, RIM_HEIGHT)
        .cut(cq.Workplane("XY").box(BODY_LENGTH - 2.0 * WALL, BODY_DEPTH - 2.0 * WALL, RIM_HEIGHT + 0.006))
        .translate((0.0, 0.0, rim_z))
    )
    body_cabinet = body_shell.union(top_rim)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_cabinet, "body_shell", tolerance=0.002, angular_tolerance=0.15),
        material=enamel_white,
        name="body_shell",
    )
    body.visual(
        Box((BODY_LENGTH - 2.0 * WALL - 0.020, BODY_DEPTH - 2.0 * WALL - 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, WALL + 0.005)),
        material=liner_gray,
        name="inner_floor",
    )
    for i, hinge_x in enumerate((-0.68, 0.68)):
        body.visual(
            Cylinder(radius=0.018, length=0.240),
            origin=Origin(
                xyz=(hinge_x, LID_PIVOT_Y, LID_PIVOT_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stainless,
            name=f"rear_barrel_{i}",
        )
        body.visual(
            Box((0.260, 0.052, 0.014)),
            origin=Origin(xyz=(hinge_x, BODY_DEPTH / 2.0 + 0.026, BODY_HEIGHT + 0.016)),
            material=stainless,
            name=f"rear_leaf_{i}",
        )
        body.visual(
            Box((0.032, 0.026, 0.090)),
            origin=Origin(xyz=(hinge_x, LID_PIVOT_Y, LID_PIVOT_Z - 0.045)),
            material=stainless,
            name=f"rear_knuckle_support_{i}",
        )

    lid_shape = cq.Workplane("XY").box(LID_LENGTH, LID_DEPTH, LID_THICKNESS).edges().fillet(0.012)
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid_panel", tolerance=0.0015, angular_tolerance=0.12),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, 0.0)),
        material=enamel_white,
        name="lid_panel",
    )
    lid.visual(
        Box((LID_LENGTH - 0.160, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, LID_CENTER_Y - LID_DEPTH / 2.0 + 0.020, -LID_THICKNESS / 2.0 - 0.006)),
        material=gasket_black,
        name="front_gasket",
    )
    for i, bracket_x in enumerate((-0.86, 0.86)):
        lid.visual(
            Box((0.120, 0.016, 0.074)),
            origin=Origin(xyz=(bracket_x, HANDLE_PIVOT_Y + 0.026, HANDLE_PIVOT_Z)),
            material=stainless,
            name=f"handle_bracket_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_PIVOT_Y, LID_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.30),
    )

    handle = model.part("front_handle")
    handle.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (-0.86, 0.0, 0.0),
                    (-0.86, -0.050, -0.080),
                    (0.86, -0.050, -0.080),
                    (0.86, 0.0, 0.0),
                ],
                radius=0.012,
                radial_segments=20,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=12,
            ),
            "front_handle_tube",
        ),
        material=stainless,
        name="handle_tube",
    )
    for i, pivot_x in enumerate((-0.86, 0.86)):
        handle.visual(
            Cylinder(radius=0.018, length=0.090),
            origin=Origin(xyz=(pivot_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"handle_pivot_{i}",
        )

    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("front_handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_hinge = object_model.get_articulation("lid_to_handle")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        min_gap=0.0,
        max_gap=0.0015,
        name="closed lid rests on the chest rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.50,
        name="full width lid covers the long chest opening",
    )
    ctx.expect_gap(
        lid,
        handle,
        axis="y",
        positive_elem="lid_panel",
        negative_elem="handle_tube",
        min_gap=0.010,
        name="stowed handle clears the lid front face",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear barrels",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.35,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_hinge: 1.00}):
        lifted_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "grab handle swings outward and upward",
        closed_handle_aabb is not None
        and lifted_handle_aabb is not None
        and float(lifted_handle_aabb[0][1]) < float(closed_handle_aabb[0][1]) - 0.035
        and float(lifted_handle_aabb[1][2]) > float(closed_handle_aabb[1][2]) + 0.030,
        details=f"closed={closed_handle_aabb}, lifted={lifted_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
