from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    painted_wood = Material("warm_painted_wood", rgba=(0.64, 0.43, 0.30, 1.0))
    interior_wood = Material("dark_inner_wood", rgba=(0.34, 0.22, 0.15, 1.0))
    lid_wood = Material("plain_lid_wood", rgba=(0.73, 0.55, 0.36, 1.0))
    hinge_metal = Material("brushed_hinge_metal", rgba=(0.72, 0.68, 0.58, 1.0))

    width = 0.220
    depth = 0.150
    height = 0.180
    wall = 0.012
    bottom = 0.014
    hinge_y = 0.0925
    hinge_z = 0.187

    body = model.part("body")
    body.visual(
        Box((width, depth, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=interior_wood,
        name="bottom_panel",
    )
    body.visual(
        Box((width, wall, height - bottom + 0.001)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, (height + bottom) / 2.0 - 0.0005)),
        material=painted_wood,
        name="front_wall",
    )
    body.visual(
        Box((wall, depth, height - bottom + 0.001)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, (height + bottom) / 2.0 - 0.0005)),
        material=painted_wood,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, height - bottom + 0.001)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, (height + bottom) / 2.0 - 0.0005)),
        material=painted_wood,
        name="side_wall_1",
    )
    body.visual(
        Box((width, 0.026, height - 0.006)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.013, height / 2.0)),
        material=painted_wood,
        name="rear_deep_wall",
    )
    body.visual(
        Box((width + 0.020, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.005, height - 0.015)),
        material=painted_wood,
        name="rear_hinge_frame",
    )

    for x in (-0.070, 0.070):
        body.visual(
            Box((0.050, 0.004, 0.020)),
            origin=Origin(xyz=(x, hinge_y + 0.0015, hinge_z - 0.011)),
            material=hinge_metal,
            name=f"hinge_leaf_{0 if x < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.004, length=0.050),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_knuckle_{0 if x < 0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((width + 0.012, depth + 0.015, 0.014)),
        origin=Origin(xyz=(0.0, -0.089, 0.0)),
        material=lid_wood,
        name="lid_panel",
    )
    lid.visual(
        Box((width - 0.042, depth - 0.048, 0.008)),
        origin=Origin(xyz=(0.0, -0.1015, -0.011)),
        material=interior_wood,
        name="underside_lip",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="center_knuckle",
    )
    lid.visual(
        Box((0.060, 0.009, 0.004)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=hinge_metal,
        name="center_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single rear revolute lid",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (-1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            min_gap=0.0,
            max_gap=0.001,
            name="plain lid rests on the front rim",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="rear_hinge_frame",
            min_gap=0.0,
            max_gap=0.001,
            name="plain lid rests on the deep hinge frame",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="bottom_panel",
            min_overlap=0.145,
            name="lid covers the rectangular body footprint",
        )

    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward from the rear hinge",
        open_aabb is not None and open_aabb[1][2] > 0.28,
        details=f"open_lid_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
