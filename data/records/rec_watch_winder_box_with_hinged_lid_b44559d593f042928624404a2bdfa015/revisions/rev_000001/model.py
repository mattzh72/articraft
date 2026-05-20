from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("dark_walnut", rgba=(0.17, 0.075, 0.030, 1.0))
    black_velvet = model.material("black_velvet", rgba=(0.010, 0.012, 0.015, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.08, 0.105, 0.12, 0.42))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.60, 0.55, 1.0))
    warm_pad = model.material("warm_watch_pillow", rgba=(0.46, 0.34, 0.23, 1.0))

    width = 0.320
    depth = 0.240
    tray_height = 0.120
    wall = 0.018
    bottom = 0.016

    # Lower case: a real open-top tray, not a solid block.  The uninterrupted
    # side and front faces keep the presentation-box silhouette quiet.
    outer = (
        cq.Workplane("XY")
        .box(width, depth, tray_height)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, tray_height / 2.0))
    )
    cavity = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        tray_height - bottom + 0.020,
    ).translate((0.0, 0.0, bottom + (tray_height - bottom + 0.020) / 2.0))
    body_shell = outer.cut(cavity)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shell, "hollow_body"),
        material=walnut,
        name="hollow_body",
    )
    body.visual(
        Box((width - 2.0 * wall - 0.010, depth - 2.0 * wall - 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, bottom + 0.002)),
        material=black_velvet,
        name="velvet_floor",
    )

    spindle_z = 0.065
    spindle_y = 0.015
    rear_inner_y = depth / 2.0 - wall
    body.visual(
        Cylinder(radius=0.027, length=0.014),
        origin=Origin(
            xyz=(0.0, rear_inner_y + 0.001, spindle_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_metal,
        name="spindle_boss",
    )
    body.visual(
        Cylinder(radius=0.006, length=rear_inner_y - spindle_y),
        origin=Origin(
            xyz=(0.0, (rear_inner_y + spindle_y) / 2.0, spindle_z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_metal,
        name="spindle_shaft",
    )

    hinge_y = depth / 2.0 + 0.006
    hinge_z = tray_height + 0.001
    for x in (-0.100, 0.100):
        body.visual(
            Box((0.066, 0.008, 0.020)),
            origin=Origin(xyz=(x, depth / 2.0 + 0.002, tray_height - 0.006)),
            material=satin_metal,
            name=f"hinge_leaf_{0 if x < 0.0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.064),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"hinge_barrel_{0 if x < 0.0 else 1}",
        )

    # The lid is intentionally a single large smoked panel with only a slim rear
    # hinge rail, matching the requested large uninterrupted presentation face.
    lid_width = width + 0.006
    lid_depth = depth - 0.002
    lid_thickness = 0.012
    lid = model.part("lid")
    lid_panel = (
        cq.Workplane("XY")
        .box(lid_width, lid_depth, lid_thickness)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, -(lid_depth / 2.0 + 0.008), 0.007))
    )
    lid.visual(
        mesh_from_cadquery(lid_panel, "smoked_lid_panel"),
        material=smoked_glass,
        name="smoked_panel",
    )
    lid.visual(
        Box((0.086, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.004)),
        material=satin_metal,
        name="rear_rail",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="center_barrel",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_velvet,
        name="padded_disk",
    )
    cradle.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hub",
    )
    cradle.visual(
        Box((0.078, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, -0.053, 0.0)),
        material=warm_pad,
        name="watch_pillow",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    model.articulation(
        "spindle_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, spindle_y, spindle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=5.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("spindle_to_cradle")

    ctx.check(
        "lid uses rear revolute hinge",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}",
    )
    ctx.check(
        "cradle uses continuous spindle",
        cradle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cradle_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={cradle_joint.articulation_type}, axis={cradle_joint.axis}",
    )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="smoked_panel",
        negative_elem="hollow_body",
        min_gap=0.001,
        max_gap=0.006,
        name="single lid panel sits just proud of the tray",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="smoked_panel",
        elem_b="hollow_body",
        min_overlap=0.20,
        name="large lid panel covers the presentation box",
    )
    ctx.expect_gap(
        body,
        cradle,
        axis="y",
        positive_elem="spindle_shaft",
        negative_elem="hub",
        min_gap=0.0,
        max_gap=0.001,
        name="cradle hub seats on the spindle nose",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="xz",
        inner_elem="padded_disk",
        outer_elem="hollow_body",
        margin=0.002,
        name="rotating cradle fits inside the open tray",
    )

    closed_panel = ctx.part_element_world_aabb(lid, elem="smoked_panel")
    with ctx.pose({lid_joint: 1.25}):
        open_panel = ctx.part_element_world_aabb(lid, elem="smoked_panel")
    ctx.check(
        "positive lid motion opens upward",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.10,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    closed_pillow = ctx.part_element_world_aabb(cradle, elem="watch_pillow")
    with ctx.pose({cradle_joint: pi / 2.0}):
        turned_pillow = ctx.part_element_world_aabb(cradle, elem="watch_pillow")
    if closed_pillow is not None and turned_pillow is not None:
        closed_x = closed_pillow[1][0] - closed_pillow[0][0]
        closed_z = closed_pillow[1][2] - closed_pillow[0][2]
        turned_x = turned_pillow[1][0] - turned_pillow[0][0]
        turned_z = turned_pillow[1][2] - turned_pillow[0][2]
        turns = turned_x < closed_x - 0.02 and turned_z > closed_z + 0.02
    else:
        turns = False
        closed_x = closed_z = turned_x = turned_z = None
    ctx.check(
        "continuous spindle visibly rotates the watch pillow",
        turns,
        details=(
            f"closed_x={closed_x}, closed_z={closed_z}, "
            f"turned_x={turned_x}, turned_z={turned_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
