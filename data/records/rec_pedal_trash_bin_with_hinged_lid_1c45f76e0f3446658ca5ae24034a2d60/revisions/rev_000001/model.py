from __future__ import annotations

from math import pi

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
import cadquery as cq


def _open_rectangular_shell(depth: float, width: float, height: float, wall: float) -> cq.Workplane:
    """Tall open-top rectangular bin shell with a real bottom and hollow cavity."""
    outer = (
        cq.Workplane("XY")
        .box(depth, width, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall, width - 2.0 * wall, height + 0.04)
        .translate((0.0, 0.0, wall + (height + 0.04) / 2.0))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_pedal_trash_bin")

    # Object frame: +X is the front with the pedal, +Y spans the width, +Z is up.
    body_depth = 0.30
    body_width = 0.36
    body_height = 0.65
    wall = 0.018

    metal = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            _open_rectangular_shell(body_depth, body_width, body_height, wall),
            "open_body_shell",
            tolerance=0.001,
            angular_tolerance=0.08,
        ),
        material=metal,
        name="shell",
    )

    # Dark rolled rim trim and the small front clevis brackets that carry the pedal pivot.
    rim_h = 0.018
    body.visual(
        Box((0.020, body_width + 0.020, rim_h)),
        origin=Origin(xyz=(body_depth / 2.0 + 0.002, 0.0, body_height - rim_h / 2.0)),
        material=dark_plastic,
        name="front_rim",
    )
    body.visual(
        Box((0.020, body_width + 0.020, rim_h)),
        origin=Origin(xyz=(-body_depth / 2.0 - 0.002, 0.0, body_height - rim_h / 2.0)),
        material=dark_plastic,
        name="rear_rim",
    )
    body.visual(
        Box((body_depth + 0.020, 0.018, rim_h)),
        origin=Origin(xyz=(0.0, body_width / 2.0 + 0.002, body_height - rim_h / 2.0)),
        material=dark_plastic,
        name="side_rim_0",
    )
    body.visual(
        Box((body_depth + 0.020, 0.018, rim_h)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 - 0.002, body_height - rim_h / 2.0)),
        material=dark_plastic,
        name="side_rim_1",
    )

    pedal_pivot_z = 0.090
    for y, name in ((0.158, "pedal_bracket_0"), (-0.158, "pedal_bracket_1")):
        body.visual(
            Box((0.018, 0.030, 0.040)),
            origin=Origin(xyz=(body_depth / 2.0 + 0.006, y, pedal_pivot_z)),
            material=dark_plastic,
            name=name,
        )

    lid = model.part("lid")
    lid_depth = body_depth + 0.035
    lid_width = body_width + 0.045
    lid_thickness = 0.025
    hinge_radius = 0.012
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        # The lid part frame sits on the rear hinge axis; the panel spans forward.
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, -hinge_radius + lid_thickness / 2.0)),
        material=dark_plastic,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=lid_width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="rear_hinge_barrel",
    )
    lid.visual(
        Box((0.110, 0.018, 0.010)),
        origin=Origin(xyz=(lid_depth - 0.035, 0.0, -hinge_radius + lid_thickness + 0.005)),
        material=rubber,
        name="front_lip",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-body_depth / 2.0, 0.0, body_height + hinge_radius)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Box((0.140, 0.250, 0.024)),
        origin=Origin(xyz=(0.070, 0.0, -0.022)),
        material=metal,
        name="pedal_plate",
    )
    pedal.visual(
        Box((0.095, 0.185, 0.006)),
        origin=Origin(xyz=(0.087, 0.0, -0.007)),
        material=rubber,
        name="tread_pad",
    )
    pedal.visual(
        Cylinder(radius=0.011, length=0.270),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pedal_axle",
    )

    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(body_depth / 2.0 + 0.011, 0.0, pedal_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=0.0, upper=0.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_joint = object_model.get_articulation("body_to_lid")
    pedal_joint = object_model.get_articulation("body_to_pedal")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        name="closed lid sits just above the top rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.24,
        name="lid spans the rectangular top opening",
    )
    ctx.expect_gap(
        pedal,
        body,
        axis="x",
        positive_elem="pedal_plate",
        negative_elem="shell",
        min_gap=0.006,
        max_gap=0.025,
        name="foot pedal projects from the front face",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.05}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear-hinged lid rotates upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.14,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_world_aabb(pedal)
    with ctx.pose({pedal_joint: 0.38}):
        pressed_pedal_aabb = ctx.part_world_aabb(pedal)
    ctx.check(
        "pedal pivots downward when pressed",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.025,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
