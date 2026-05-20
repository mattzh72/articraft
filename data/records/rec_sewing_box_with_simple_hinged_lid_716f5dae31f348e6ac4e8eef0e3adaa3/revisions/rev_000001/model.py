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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    # Small tabletop sewing box proportions, in meters.
    depth = 0.24
    width = 0.36
    height = 0.135
    wall = 0.010
    bottom = 0.012
    lid_thickness = 0.014

    wood = model.material("warm_light_wood", color=(0.86, 0.70, 0.48, 1.0))
    frame = model.material("slightly_darker_frame", color=(0.72, 0.52, 0.32, 1.0))
    brass = model.material("brushed_brass", color=(0.85, 0.63, 0.25, 1.0))

    body_shell = (
        cq.Workplane("XY")
        .box(depth, width, height, centered=(True, True, False))
        .cut(
            cq.Workplane("XY")
            .box(depth - 2 * wall, width - 2 * wall, height, centered=(True, True, False))
            .translate((0.0, 0.0, bottom))
        )
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell", tolerance=0.0008),
        material=wood,
        name="open_shell",
    )

    # A sparse exterior frame: one upper band around the box and a simple front
    # lower rail.  It reads as a light framed shell without a dense rib pattern.
    body.visual(
        Box((0.008, width + 0.008, 0.022)),
        origin=Origin(xyz=(depth / 2 + 0.002, 0.0, height - 0.018)),
        material=frame,
        name="front_upper_frame",
    )
    body.visual(
        Box((0.008, width + 0.008, 0.018)),
        origin=Origin(xyz=(depth / 2 + 0.002, 0.0, 0.030)),
        material=frame,
        name="front_lower_frame",
    )
    for sign, name in ((-1.0, "side_frame_0"), (1.0, "side_frame_1")):
        body.visual(
            Box((depth - 0.010, 0.008, 0.020)),
            origin=Origin(xyz=(0.006, sign * (width / 2 + 0.002), height - 0.018)),
            material=frame,
            name=name,
        )

    hinge_x = -depth / 2 - 0.008
    hinge_z = height + 0.006
    hinge_radius = 0.005
    for y, name in ((-0.115, "rear_knuckle_0"), (0.115, "rear_knuckle_1")):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.078),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-pi / 2, 0.0, 0.0)),
            material=brass,
            name=name,
        )
        body.visual(
            Box((0.006, 0.078, 0.044)),
            origin=Origin(xyz=(-depth / 2 - 0.002, y, height - 0.020)),
            material=brass,
            name=f"rear_leaf_{0 if y < 0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((depth, width, lid_thickness)),
        # The child frame is on the rear hinge axis.  The closed plain panel
        # spans forward from that line and its underside rests on the box rim.
        origin=Origin(xyz=(depth / 2 + 0.010, 0.0, lid_thickness / 2 - 0.006)),
        material=wood,
        name="plain_panel",
    )
    lid.visual(
        Cylinder(radius=hinge_radius * 0.92, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=brass,
        name="center_knuckle",
    )
    lid.visual(
        Box((0.050, 0.116, 0.003)),
        origin=Origin(xyz=(0.025, 0.0, -0.0045)),
        material=brass,
        name="lid_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.75),
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
            positive_elem="plain_panel",
            negative_elem="open_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid rests on shell rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="plain_panel",
            elem_b="open_shell",
            min_overlap=0.20,
            name="plain lid covers box opening",
        )

    closed_panel = ctx.part_element_world_aabb(lid, elem="plain_panel")
    with ctx.pose({hinge: 1.45}):
        raised_panel = ctx.part_element_world_aabb(lid, elem="plain_panel")
        ctx.check(
            "rear hinge raises the lid panel",
            closed_panel is not None
            and raised_panel is not None
            and raised_panel[1][2] > closed_panel[1][2] + 0.10,
            details=f"closed={closed_panel}, raised={raised_panel}",
        )

    return ctx.report()


object_model = build_object_model()
