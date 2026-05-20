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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    wood = model.material("warm_varnished_wood", rgba=(0.62, 0.34, 0.16, 1.0))
    darker_wood = model.material("plain_darker_lid", rgba=(0.48, 0.25, 0.11, 1.0))
    lining = model.material("cream_fabric_lining", rgba=(0.86, 0.77, 0.58, 1.0))
    brass = model.material("aged_brass_hinge", rgba=(0.78, 0.59, 0.25, 1.0))

    depth = 0.24
    width = 0.34
    height = 0.13
    wall = 0.014
    bottom = 0.014
    lip_depth = 0.020
    lip_thick = 0.008
    lid_thick = 0.014
    lid_front_overhang = 0.010
    lid_side_overhang = 0.008
    hinge_offset = 0.014
    hinge_radius = 0.007
    hinge_axis_z = height + lid_thick / 2.0
    hinge_axis_x = -depth / 2.0 - hinge_offset

    body = model.part("box_body")
    body.visual(
        Box((depth, width, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=wood,
        name="solid_bottom",
    )
    body.visual(
        Box((wall, width, height - bottom)),
        origin=Origin(xyz=(depth / 2.0 - wall / 2.0, 0.0, bottom + (height - bottom) / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((wall, width, height - bottom)),
        origin=Origin(xyz=(-depth / 2.0 + wall / 2.0, 0.0, bottom + (height - bottom) / 2.0)),
        material=wood,
        name="rear_wall",
    )
    body.visual(
        Box((depth - 2.0 * wall, wall, height - bottom)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, bottom + (height - bottom) / 2.0)),
        material=wood,
        name="side_wall_0",
    )
    body.visual(
        Box((depth - 2.0 * wall, wall, height - bottom)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, bottom + (height - bottom) / 2.0)),
        material=wood,
        name="side_wall_1",
    )

    # A continuous fixed frame lip just below the lid gives the open box a stop ledge.
    inner_width = width - 2.0 * wall
    inner_depth = depth - 2.0 * wall
    lip_z = height - lip_thick / 2.0
    body.visual(
        Box((lip_depth, inner_width, lip_thick)),
        origin=Origin(xyz=(depth / 2.0 - wall - lip_depth / 2.0, 0.0, lip_z)),
        material=wood,
        name="front_stop_lip",
    )
    body.visual(
        Box((lip_depth, inner_width, lip_thick)),
        origin=Origin(xyz=(-depth / 2.0 + wall + lip_depth / 2.0, 0.0, lip_z)),
        material=wood,
        name="rear_stop_lip",
    )
    body.visual(
        Box((inner_depth - 2.0 * lip_depth, lip_depth, lip_thick)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall - lip_depth / 2.0, lip_z)),
        material=wood,
        name="side_stop_lip_0",
    )
    body.visual(
        Box((inner_depth - 2.0 * lip_depth, lip_depth, lip_thick)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall + lip_depth / 2.0, lip_z)),
        material=wood,
        name="side_stop_lip_1",
    )
    body.visual(
        Box((inner_depth - 0.010, inner_width - 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, bottom + 0.0015)),
        material=lining,
        name="fabric_lined_floor",
    )

    hinge_plate_x = hinge_offset - hinge_radius + 0.004
    hinge_plate_length = 0.070
    for idx, y in enumerate((-0.112, 0.112)):
        body.visual(
            Box((hinge_plate_x, hinge_plate_length, 0.026)),
            origin=Origin(
                xyz=(
                    -depth / 2.0 - hinge_plate_x / 2.0 + 0.001,
                    y,
                    hinge_axis_z,
                )
            ),
            material=brass,
            name=f"hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=hinge_plate_length),
            origin=Origin(
                xyz=(hinge_axis_x, y, hinge_axis_z),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=brass,
            name=f"hinge_barrel_{idx}",
        )

    lid = model.part("lid_panel")
    lid.visual(
        Box((depth + lid_front_overhang, width + 2.0 * lid_side_overhang, lid_thick)),
        origin=Origin(
            xyz=(
                hinge_offset + (depth + lid_front_overhang) / 2.0,
                0.0,
                0.0,
            )
        ),
        material=darker_wood,
        name="plain_top",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((hinge_offset, 0.140, 0.006)),
        origin=Origin(xyz=(hinge_offset / 2.0, 0.0, -lid_thick / 2.0 - 0.003)),
        material=brass,
        name="center_hinge_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("lid_panel")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "single rear lid hinge",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (0.0, -1.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="plain_top",
        negative_elem="front_stop_lip",
        name="closed lid rests on stop lip",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.18,
        elem_a="plain_top",
        elem_b="solid_bottom",
        name="lid covers rectangular box footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="plain_top")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="plain_top")
    ctx.check(
        "hinge opens the plain lid upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
