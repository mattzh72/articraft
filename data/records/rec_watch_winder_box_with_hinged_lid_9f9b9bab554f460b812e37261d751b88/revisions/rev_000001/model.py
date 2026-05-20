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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _flat_frame(width: float, depth: float, thickness: float, rail: float) -> cq.Workplane:
    """A shallow rectangular lid frame with a centered panel opening."""
    outer = cq.Workplane("XY").box(width, depth, thickness)
    inner = cq.Workplane("XY").box(width - 2.0 * rail, depth - 2.0 * rail, thickness * 1.8)
    return outer.cut(inner)


def _rounded_cushion(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """Soft watch pillow geometry with rounded edges."""
    return cq.Workplane("XY").box(width, depth, height).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("satin_walnut", rgba=(0.38, 0.18, 0.075, 1.0))
    dark_walnut = model.material("dark_endgrain", rgba=(0.22, 0.10, 0.045, 1.0))
    velvet = model.material("black_velvet", rgba=(0.012, 0.011, 0.013, 1.0))
    glass = model.material("smoked_glass", rgba=(0.55, 0.70, 0.82, 0.34))
    brass = model.material("brushed_brass", rgba=(0.88, 0.64, 0.28, 1.0))
    steel = model.material("dark_hinge_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    cushion = model.material("cream_pillow", rgba=(0.80, 0.73, 0.62, 1.0))

    width = 0.34
    depth = 0.28
    wall = 0.018
    base_thickness = 0.024
    body_height = 0.160
    wall_height = body_height - base_thickness
    wall_center_z = base_thickness + wall_height / 2.0

    body = model.part("body")
    body.visual(
        Box((width, depth, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=dark_walnut,
        name="floor",
    )
    body.visual(
        Box((wall, depth, wall_height)),
        origin=Origin(xyz=(-(width - wall) / 2.0, 0.0, wall_center_z)),
        material=walnut,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, wall_height)),
        origin=Origin(xyz=((width - wall) / 2.0, 0.0, wall_center_z)),
        material=walnut,
        name="side_wall_1",
    )
    body.visual(
        Box((width, wall, wall_height)),
        origin=Origin(xyz=(0.0, -(depth - wall) / 2.0, wall_center_z)),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((width, wall, wall_height)),
        origin=Origin(xyz=(0.0, (depth - wall) / 2.0, wall_center_z)),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((width - 2.0 * wall - 0.018, depth - 2.0 * wall - 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.003)),
        material=velvet,
        name="velvet_liner",
    )
    body.visual(
        Box((0.270, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.0025, body_height - 0.011)),
        material=steel,
        name="rear_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.096),
        origin=Origin(
            xyz=(0.0, -0.076, 0.091),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="spindle",
    )

    hinge_z = body_height + 0.008
    hinge_y = -depth / 2.0

    lid = model.part("lid")
    lid_frame = _flat_frame(width=width, depth=depth, thickness=0.016, rail=0.030)
    lid.visual(
        mesh_from_cadquery(lid_frame, "lid_centered_frame", tolerance=0.0007),
        origin=Origin(xyz=(0.0, depth / 2.0, 0.0)),
        material=walnut,
        name="lid_frame",
    )
    lid.visual(
        Box((width - 2.0 * 0.030 + 0.008, depth - 2.0 * 0.030 + 0.008, 0.004)),
        origin=Origin(xyz=(0.0, depth / 2.0, -0.001)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.285),
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.285, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.003, -0.004)),
        material=steel,
        name="hinge_leaf",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(
            xyz=(0.0, 0.002, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="hub",
    )
    cradle.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.025, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="round_plate",
    )
    cradle.visual(
        mesh_from_cadquery(_rounded_cushion(0.092, 0.035, 0.052, 0.010), "watch_pillow", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
        material=cushion,
        name="pillow",
    )
    cradle.visual(
        Box((0.098, 0.004, 0.007)),
        origin=Origin(xyz=(0.0, 0.064, 0.021)),
        material=velvet,
        name="strap_0",
    )
    cradle.visual(
        Box((0.098, 0.004, 0.007)),
        origin=Origin(xyz=(0.0, 0.064, -0.021)),
        material=velvet,
        name="strap_1",
    )
    cradle.visual(
        Sphere(radius=0.005),
        origin=Origin(xyz=(0.0, 0.034, 0.042)),
        material=brass,
        name="index_dot",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.040, 0.091)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")

    ctx.allow_overlap(
        body,
        cradle,
        elem_a="spindle",
        elem_b="hub",
        reason="The brass hub is intentionally modeled as a rotating sleeve captured over the fixed spindle.",
    )

    ctx.expect_within(
        body,
        cradle,
        axes="xz",
        inner_elem="spindle",
        outer_elem="hub",
        margin=0.001,
        name="spindle centered inside hub",
    )
    ctx.expect_overlap(
        body,
        cradle,
        axes="y",
        elem_a="spindle",
        elem_b="hub",
        min_overlap=0.020,
        name="hub remains captured on spindle",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid rests on front wall",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_frame",
        elem_b="floor",
        min_overlap=0.20,
        name="lid covers box footprint",
    )

    frame_aabb = ctx.part_element_world_aabb(lid, elem="lid_frame")
    glass_aabb = ctx.part_element_world_aabb(lid, elem="glass_panel")
    if frame_aabb is not None and glass_aabb is not None:
        frame_min, frame_max = frame_aabb
        glass_min, glass_max = glass_aabb
        frame_cx = (frame_min[0] + frame_max[0]) / 2.0
        frame_cy = (frame_min[1] + frame_max[1]) / 2.0
        glass_cx = (glass_min[0] + glass_max[0]) / 2.0
        glass_cy = (glass_min[1] + glass_max[1]) / 2.0
        ctx.check(
            "glass panel centered in frame",
            abs(frame_cx - glass_cx) < 0.001 and abs(frame_cy - glass_cy) < 0.001,
            details=f"frame center=({frame_cx:.4f}, {frame_cy:.4f}), glass center=({glass_cx:.4f}, {glass_cy:.4f})",
        )
    else:
        ctx.fail("glass panel centered in frame", "Could not read lid frame or glass panel AABB.")

    with ctx.pose({lid_joint: 1.20}):
        closed_pos = ctx.part_world_position(lid)
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward about rear hinge",
            open_aabb is not None and open_aabb[1][2] > 0.24 and closed_pos is not None,
            details=f"open_aabb={open_aabb}, lid_pos={closed_pos}",
        )

    rest_dot = ctx.part_element_world_aabb(cradle, elem="index_dot")
    with ctx.pose({cradle_joint: math.pi / 2.0}):
        turned_dot = ctx.part_element_world_aabb(cradle, elem="index_dot")
        rest_pos = ctx.part_world_position(cradle)
        turned_pos = ctx.part_world_position(cradle)
    if rest_dot is not None and turned_dot is not None and rest_pos is not None and turned_pos is not None:
        rest_center = tuple((rest_dot[0][i] + rest_dot[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_dot[0][i] + turned_dot[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "cradle rotates continuously about spindle",
            turned_center[0] > rest_center[0] + 0.030
            and abs(turned_pos[1] - rest_pos[1]) < 0.001
            and abs(turned_pos[2] - rest_pos[2]) < 0.001,
            details=f"rest_dot={rest_center}, turned_dot={turned_center}, rest_pos={rest_pos}, turned_pos={turned_pos}",
        )
    else:
        ctx.fail("cradle rotates continuously about spindle", "Could not read cradle marker positions.")

    return ctx.report()


object_model = build_object_model()
