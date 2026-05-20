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
    model = ArticulatedObject(name="twin_door_chest_freezer")

    freezer_white = Material("powder_coated_white", rgba=(0.92, 0.94, 0.93, 1.0))
    liner_blue = Material("pale_liner", rgba=(0.78, 0.88, 0.90, 1.0))
    gasket_black = Material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    hinge_metal = Material("brushed_hinge_metal", rgba=(0.62, 0.64, 0.62, 1.0))
    shadow_black = Material("vent_black", rgba=(0.02, 0.02, 0.018, 1.0))

    width = 1.60
    depth = 0.78
    body_height = 0.80
    wall = 0.050
    bottom = 0.080
    lid_width = 0.78
    lid_depth = 0.74
    lid_thick = 0.060
    seam_gap = 0.020
    hinge_radius = 0.024
    hinge_axis_y = -depth / 2.0 - 0.005
    hinge_axis_z = body_height + 0.060

    body = model.part("body")
    body.visual(
        Box((width, depth, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=freezer_white,
        name="bottom_pan",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_height / 2.0)),
        material=freezer_white,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_height / 2.0)),
        material=freezer_white,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=freezer_white,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth - 2.0 * wall, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=freezer_white,
        name="side_wall_1",
    )

    # Pale liner panels make the top-opening chest read hollow when a lid is raised.
    inner_width = width - 2.0 * wall
    inner_depth = depth - 2.0 * wall
    liner_h = body_height - bottom - 0.035
    body.visual(
        Box((inner_width - 0.025, inner_depth - 0.025, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, bottom + 0.009)),
        material=liner_blue,
        name="liner_floor",
    )
    body.visual(
        Box((inner_width, 0.010, liner_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall - 0.005, bottom + liner_h / 2.0)),
        material=liner_blue,
        name="front_liner",
    )
    body.visual(
        Box((inner_width, 0.010, liner_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + 0.005, bottom + liner_h / 2.0)),
        material=liner_blue,
        name="rear_liner",
    )
    body.visual(
        Box((0.010, inner_depth, liner_h)),
        origin=Origin(xyz=(-width / 2.0 + wall + 0.005, 0.0, bottom + liner_h / 2.0)),
        material=liner_blue,
        name="side_liner_0",
    )
    body.visual(
        Box((0.010, inner_depth, liner_h)),
        origin=Origin(xyz=(width / 2.0 - wall - 0.005, 0.0, bottom + liner_h / 2.0)),
        material=liner_blue,
        name="side_liner_1",
    )

    # Low black seating rim around the opening, just below the closed lid gaskets.
    rim_z = body_height + 0.009
    body.visual(
        Box((width - 0.030, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.032, rim_z)),
        material=gasket_black,
        name="front_rim",
    )
    body.visual(
        Box((width - 0.030, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.032, rim_z)),
        material=gasket_black,
        name="rear_rim",
    )
    body.visual(
        Box((0.040, depth - 0.060, 0.018)),
        origin=Origin(xyz=(-width / 2.0 + 0.032, 0.0, rim_z)),
        material=gasket_black,
        name="side_rim_0",
    )
    body.visual(
        Box((0.040, depth - 0.060, 0.018)),
        origin=Origin(xyz=(width / 2.0 - 0.032, 0.0, rim_z)),
        material=gasket_black,
        name="side_rim_1",
    )

    # A small front service grille is flush-mounted into the lower body wall.
    for i in range(6):
        body.visual(
            Box((0.26, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, depth / 2.0 - 0.001, 0.145 + i * 0.025)),
            material=shadow_black,
            name=f"front_vent_bar_{i}",
        )

    hinge_rpy = (0.0, math.pi / 2.0, 0.0)
    lid_centers = (
        -lid_width / 2.0 - seam_gap / 2.0,
        lid_width / 2.0 + seam_gap / 2.0,
    )

    for idx, x_center in enumerate(lid_centers):
        # Alternating hinge knuckles: two fixed outer barrels on the body and a
        # rotating central barrel carried by the lid.
        for suffix, x_off in (("outer", -0.255), ("inner", 0.255)):
            body.visual(
                Cylinder(radius=hinge_radius, length=0.150),
                origin=Origin(
                    xyz=(x_center + x_off, hinge_axis_y, hinge_axis_z),
                    rpy=hinge_rpy,
                ),
                material=hinge_metal,
                name=f"hinge_{idx}_{suffix}_barrel",
            )
            body.visual(
                Box((0.170, 0.030, 0.100)),
                origin=Origin(
                    xyz=(x_center + x_off, hinge_axis_y + 0.003, body_height + 0.030)
                ),
                material=hinge_metal,
                name=f"hinge_{idx}_{suffix}_leaf",
            )

        lid = model.part(f"lid_{idx}")
        lid.visual(
            Box((lid_width, lid_depth, lid_thick)),
            origin=Origin(xyz=(0.0, 0.410, -0.005)),
            material=freezer_white,
            name="lid_panel",
        )
        lid.visual(
            Box((lid_width - 0.035, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.760, -0.036)),
            material=gasket_black,
            name="front_gasket",
        )
        lid.visual(
            Box((lid_width - 0.035, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.075, -0.036)),
            material=gasket_black,
            name="rear_gasket",
        )
        lid.visual(
            Box((0.018, lid_depth - 0.060, 0.012)),
            origin=Origin(xyz=(-lid_width / 2.0 + 0.030, 0.415, -0.036)),
            material=gasket_black,
            name="side_gasket_0",
        )
        lid.visual(
            Box((0.018, lid_depth - 0.060, 0.012)),
            origin=Origin(xyz=(lid_width / 2.0 - 0.030, 0.415, -0.036)),
            material=gasket_black,
            name="side_gasket_1",
        )
        lid.visual(
            Box((lid_width * 0.48, 0.036, 0.035)),
            origin=Origin(xyz=(0.0, 0.792, -0.012)),
            material=freezer_white,
            name="front_handle",
        )
        lid.visual(
            Cylinder(radius=hinge_radius * 0.92, length=0.300),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=hinge_rpy),
            material=hinge_metal,
            name="center_barrel",
        )
        lid.visual(
            Box((0.280, 0.090, 0.018)),
            origin=Origin(xyz=(0.0, 0.060, 0.018)),
            material=hinge_metal,
            name="hinge_leaf",
        )

        model.articulation(
            f"body_to_lid_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=lid,
            origin=Origin(xyz=(x_center, hinge_axis_y, hinge_axis_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    hinge_0 = object_model.get_articulation("body_to_lid_0")
    hinge_1 = object_model.get_articulation("body_to_lid_1")

    ctx.expect_gap(
        lid_0,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_rim",
        min_gap=0.002,
        max_gap=0.020,
        name="lid 0 sits just above the body rim",
    )
    ctx.expect_gap(
        lid_1,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_rim",
        min_gap=0.002,
        max_gap=0.020,
        name="lid 1 sits just above the body rim",
    )
    ctx.expect_overlap(
        lid_0,
        body,
        axes="y",
        elem_a="lid_panel",
        elem_b="front_rim",
        min_overlap=0.035,
        name="lid 0 spans to the front rim",
    )
    ctx.expect_overlap(
        lid_1,
        body,
        axes="y",
        elem_a="lid_panel",
        elem_b="front_rim",
        min_overlap=0.035,
        name="lid 1 spans to the front rim",
    )

    lid0_aabb = ctx.part_element_world_aabb(lid_0, elem="lid_panel")
    lid1_aabb = ctx.part_element_world_aabb(lid_1, elem="lid_panel")
    if lid0_aabb is not None and lid1_aabb is not None:
        lid0_width = lid0_aabb[1][0] - lid0_aabb[0][0]
        lid1_width = lid1_aabb[1][0] - lid1_aabb[0][0]
        seam_gap = lid1_aabb[0][0] - lid0_aabb[1][0]
        closed_lid0_top = lid0_aabb[1][2]
        closed_lid1_top = lid1_aabb[1][2]
    else:
        lid0_width = lid1_width = seam_gap = 0.0
        closed_lid0_top = 0.0
        closed_lid1_top = 0.0
    ctx.check(
        "two equal-width lid panels with a center seam",
        abs(lid0_width - lid1_width) < 0.005 and 0.010 <= seam_gap <= 0.035,
        details=f"lid0_width={lid0_width}, lid1_width={lid1_width}, seam_gap={seam_gap}",
    )

    with ctx.pose({hinge_0: 1.10}):
        raised_lid0 = ctx.part_element_world_aabb(lid_0, elem="lid_panel")
        still_lid1 = ctx.part_element_world_aabb(lid_1, elem="lid_panel")
        raised_top = raised_lid0[1][2] if raised_lid0 is not None else 0.0
        still_top = still_lid1[1][2] if still_lid1 is not None else 0.0
    ctx.check(
        "lid 0 opens upward independently",
        raised_top > closed_lid1_top + 0.45 and abs(still_top - closed_lid1_top) < 0.01,
        details=f"raised_lid0_top={raised_top}, closed_lid1_top={closed_lid1_top}, still_lid1_top={still_top}",
    )

    with ctx.pose({hinge_1: 1.10}):
        raised_lid1 = ctx.part_element_world_aabb(lid_1, elem="lid_panel")
        still_lid0 = ctx.part_element_world_aabb(lid_0, elem="lid_panel")
        raised_top = raised_lid1[1][2] if raised_lid1 is not None else 0.0
        still_top = still_lid0[1][2] if still_lid0 is not None else 0.0
    ctx.check(
        "lid 1 opens upward independently",
        raised_top > closed_lid0_top + 0.45 and abs(still_top - closed_lid0_top) < 0.01,
        details=f"raised_lid1_top={raised_top}, closed_lid0_top={closed_lid0_top}, still_lid0_top={still_top}",
    )

    return ctx.report()


object_model = build_object_model()
