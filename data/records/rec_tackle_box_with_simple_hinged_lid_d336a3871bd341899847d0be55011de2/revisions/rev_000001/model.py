from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    """Small CadQuery box helper; dimensions are in meters."""
    return cq.Workplane("XY").box(*size).translate(center)


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]) -> cq.Workplane:
    shape = _cq_box(*boxes[0])
    for size, center in boxes[1:]:
        shape = shape.union(_cq_box(size, center))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    shell_mat = Material("light_sand_plastic", color=(0.78, 0.73, 0.62, 1.0))
    tray_mat = Material("warm_off_white_tray", color=(0.86, 0.84, 0.76, 1.0))
    hinge_mat = Material("dark_hinge_pin", color=(0.18, 0.18, 0.17, 1.0))

    width = 0.56
    depth = 0.32
    body_h = 0.16
    wall = 0.018
    floor = 0.012
    rear_y = depth / 2.0
    front_y = -depth / 2.0
    hinge_y = rear_y + 0.030
    hinge_z = body_h + 0.018

    body = model.part("body")

    body_shell = _union_boxes(
        [
            ((width, depth, floor), (0.0, 0.0, floor / 2.0)),
            ((width, wall, body_h), (0.0, rear_y - wall / 2.0, body_h / 2.0)),
            ((width, wall, body_h), (0.0, front_y + wall / 2.0, body_h / 2.0)),
            ((wall, depth, body_h), (-width / 2.0 + wall / 2.0, 0.0, body_h / 2.0)),
            ((wall, depth, body_h), (width / 2.0 - wall / 2.0, 0.0, body_h / 2.0)),
            # A narrow rim reads as a molded shell lip without filling the box.
            ((width, 0.010, 0.010), (0.0, rear_y - wall - 0.005, body_h + 0.005)),
            ((width, 0.010, 0.010), (0.0, front_y + wall + 0.005, body_h + 0.005)),
            ((0.010, depth - 2.0 * wall, 0.010), (-width / 2.0 + wall + 0.005, 0.0, body_h + 0.005)),
            ((0.010, depth - 2.0 * wall, 0.010), (width / 2.0 - wall - 0.005, 0.0, body_h + 0.005)),
        ]
    )
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell", tolerance=0.001),
        material=shell_mat,
        name="body_shell",
    )

    # Fixed internal tackle tray details: low dividers molded into the body.
    tray_h = 0.052
    tray_z = floor + tray_h / 2.0
    tray_boxes = [
        ((width - 2.0 * wall - 0.030, 0.008, tray_h), (0.0, -0.015, tray_z)),
        ((0.008, depth - 2.0 * wall - 0.038, tray_h), (-0.090, -0.008, tray_z)),
        ((0.008, depth - 2.0 * wall - 0.038, tray_h), (0.095, -0.008, tray_z)),
        ((0.135, 0.008, tray_h), (-0.185, 0.065, tray_z)),
        ((0.135, 0.008, tray_h), (0.185, 0.065, tray_z)),
    ]
    tray_insert = _union_boxes(tray_boxes)
    body.visual(
        mesh_from_cadquery(tray_insert, "fixed_tray_details", tolerance=0.001),
        material=tray_mat,
        name="fixed_tray",
    )

    # Rear hinge support knuckles are fixed to the body, leaving a center gap for the lid knuckle.
    body.visual(
        Cylinder(radius=0.012, length=0.130),
        origin=Origin(xyz=(-0.205, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.130),
        origin=Origin(xyz=(0.205, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_knuckle_1",
    )
    body.visual(
        Box((0.130, 0.018, 0.028)),
        origin=Origin(xyz=(-0.205, rear_y + 0.009, body_h + 0.006)),
        material=shell_mat,
        name="hinge_leaf_0",
    )
    body.visual(
        Box((0.130, 0.018, 0.028)),
        origin=Origin(xyz=(0.205, rear_y + 0.009, body_h + 0.006)),
        material=shell_mat,
        name="hinge_leaf_1",
    )

    lid = model.part("lid")
    lid_panel = _union_boxes(
        [
            # The lid's part frame is exactly on the hinge axis.  The closed panel
            # extends along local -Y from that rear hinge line.
            ((width, 0.305, 0.018), (0.0, -0.1975, 0.001)),
            # Simple raised outer frame.
            ((width, 0.012, 0.010), (0.0, -0.050, 0.013)),
            ((width, 0.012, 0.010), (0.0, -0.337, 0.013)),
            ((0.012, 0.287, 0.010), (-width / 2.0 + 0.018, -0.1935, 0.013)),
            ((0.012, 0.287, 0.010), (width / 2.0 - 0.018, -0.1935, 0.013)),
            # Only two shallow molded ribs to keep the lid plain and light.
            ((0.010, 0.215, 0.008), (-0.095, -0.195, 0.012)),
            ((0.010, 0.215, 0.008), (0.095, -0.195, 0.012)),
            # Center hinge leaf ties the lid panel to the rotating barrel.
            ((0.240, 0.042, 0.012), (0.0, -0.023, 0.002)),
        ]
    )
    lid.visual(
        mesh_from_cadquery(lid_panel, "lid_panel", tolerance=0.001),
        material=shell_mat,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            min_gap=-0.0001,
            max_gap=0.002,
            name="closed lid sits on body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.25,
            name="lid covers rectangular box footprint",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="fixed_tray",
            min_gap=0.020,
            name="opened lid clears fixed internal tray",
        )

    ctx.check(
        "single rear hinge lifts the lid",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
