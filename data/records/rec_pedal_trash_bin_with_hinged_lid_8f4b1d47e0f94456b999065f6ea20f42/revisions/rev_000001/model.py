from __future__ import annotations

import math

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
    model = ArticulatedObject(name="square_commercial_trash_bin")

    stainless = model.material("brushed_stainless", rgba=(0.63, 0.65, 0.63, 1.0))
    dark_stainless = model.material("dark_inner_cavity", rgba=(0.08, 0.085, 0.08, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.016, 0.015, 1.0))
    satin_black = model.material("satin_black", rgba=(0.03, 0.032, 0.034, 1.0))
    pedal_metal = model.material("pedal_brushed_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))

    body = model.part("body")

    # Tall square hollow bin body: four stainless walls on a weighted base, with
    # an open dark interior visible below the broad lid.
    body.visual(
        Box((0.54, 0.54, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black_plastic,
        name="base_plinth",
    )
    body.visual(
        Box((0.034, 0.500, 0.780)),
        origin=Origin(xyz=(0.250, 0.0, 0.470)),
        material=stainless,
        name="front_wall",
    )
    body.visual(
        Box((0.034, 0.500, 0.780)),
        origin=Origin(xyz=(-0.250, 0.0, 0.470)),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((0.500, 0.034, 0.780)),
        origin=Origin(xyz=(0.0, 0.250, 0.470)),
        material=stainless,
        name="side_wall_0",
    )
    body.visual(
        Box((0.500, 0.034, 0.780)),
        origin=Origin(xyz=(0.0, -0.250, 0.470)),
        material=stainless,
        name="side_wall_1",
    )
    body.visual(
        Box((0.420, 0.420, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=dark_stainless,
        name="inner_floor",
    )
    body.visual(
        Box((0.050, 0.560, 0.030)),
        origin=Origin(xyz=(0.250, 0.0, 0.875)),
        material=black_plastic,
        name="top_rim_front",
    )
    body.visual(
        Box((0.050, 0.560, 0.030)),
        origin=Origin(xyz=(-0.250, 0.0, 0.875)),
        material=black_plastic,
        name="top_rim_rear",
    )
    body.visual(
        Box((0.560, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.250, 0.875)),
        material=black_plastic,
        name="top_rim_side_0",
    )
    body.visual(
        Box((0.560, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, -0.250, 0.875)),
        material=black_plastic,
        name="top_rim_side_1",
    )
    body.visual(
        Box((0.006, 0.380, 0.560)),
        origin=Origin(xyz=(0.270, 0.0, 0.455)),
        material=stainless,
        name="front_inset_panel",
    )
    body.visual(
        Box((0.010, 0.018, 0.640)),
        origin=Origin(xyz=(0.270, 0.215, 0.440)),
        material=dark_stainless,
        name="front_seam_0",
    )
    body.visual(
        Box((0.010, 0.018, 0.640)),
        origin=Origin(xyz=(0.270, -0.215, 0.440)),
        material=dark_stainless,
        name="front_seam_1",
    )

    # Rear hinge support leaves and split knuckles are fixed to the body.  The
    # central lid knuckle is on the moving lid part, leaving visible hinge gaps.
    for idx, y in enumerate((-0.180, 0.180)):
        body.visual(
            Box((0.055, 0.125, 0.030)),
            origin=Origin(xyz=(-0.276, y, 0.895)),
            material=black_plastic,
            name=f"rear_hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=0.018, length=0.125),
            origin=Origin(xyz=(-0.295, y, 0.915), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_plastic,
            name=f"body_hinge_knuckle_{idx}",
        )

    # Lower front clevis brackets visually carry the pedal's transverse pivot.
    for idx, y in enumerate((-0.150, 0.150)):
        body.visual(
            Box((0.070, 0.030, 0.100)),
            origin=Origin(xyz=(0.298, y, 0.115)),
            material=satin_black,
            name=f"pedal_bracket_{idx}",
        )
    body.visual(
        Box((0.060, 0.340, 0.032)),
        origin=Origin(xyz=(0.285, 0.0, 0.064)),
        material=satin_black,
        name="pedal_mount_sill",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.565, 0.560, 0.045)),
        origin=Origin(xyz=(0.3125, 0.0, 0.020)),
        material=black_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.490, 0.460, 0.018)),
        origin=Origin(xyz=(0.315, 0.0, 0.0505)),
        material=satin_black,
        name="lid_crown",
    )
    lid.visual(
        Box((0.075, 0.200, 0.018)),
        origin=Origin(xyz=(0.035, 0.0, 0.010)),
        material=black_plastic,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.018, 0.560, 0.055)),
        origin=Origin(xyz=(0.595, 0.0, -0.015)),
        material=black_plastic,
        name="front_lid_skirt",
    )
    lid.visual(
        Box((0.540, 0.018, 0.055)),
        origin=Origin(xyz=(0.310, 0.289, -0.015)),
        material=black_plastic,
        name="side_lid_skirt_0",
    )
    lid.visual(
        Box((0.540, 0.018, 0.055)),
        origin=Origin(xyz=(0.310, -0.289, -0.015)),
        material=black_plastic,
        name="side_lid_skirt_1",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="lid_hinge_knuckle",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.018, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="pedal_pivot_barrel",
    )
    pedal.visual(
        Box((0.180, 0.240, 0.026)),
        origin=Origin(xyz=(0.100, 0.0, -0.025)),
        material=pedal_metal,
        name="pedal_plate",
    )
    for idx, y in enumerate((-0.060, 0.0, 0.060)):
        pedal.visual(
            Box((0.138, 0.014, 0.008)),
            origin=Origin(xyz=(0.110, y, -0.008)),
            material=rubber,
            name=f"pedal_tread_{idx}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.295, 0.0, 0.915)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.323, 0.0, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_rim_front",
            min_gap=0.005,
            max_gap=0.035,
            name="closed lid sits just above front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="top_rim_front",
            min_overlap=0.045,
            name="lid broadly covers square top",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="x",
            positive_elem="pedal_plate",
            negative_elem="front_wall",
            min_gap=0.030,
            max_gap=0.120,
            name="pedal projects from lower front",
        )
        ctx.expect_overlap(
            pedal,
            body,
            axes="y",
            elem_a="pedal_plate",
            elem_b="front_wall",
            min_overlap=0.200,
            name="pedal is centered across the front",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "rear hinge raises the broad lid",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    with ctx.pose({pedal_hinge: 0.35}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    ctx.check(
        "transverse pedal pivot lowers the front plate",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.035,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
