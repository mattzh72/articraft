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
    model = ArticulatedObject(name="deep_freeze_chest")

    enamel = Material("warm_white_enamel", color=(0.92, 0.94, 0.92, 1.0))
    liner = Material("cool_gray_liner", color=(0.64, 0.70, 0.74, 1.0))
    shadow = Material("dark_cavity_shadow", color=(0.05, 0.07, 0.08, 1.0))
    rubber = Material("black_rubber_gasket", color=(0.02, 0.02, 0.018, 1.0))
    metal = Material("brushed_hinge_metal", color=(0.62, 0.64, 0.62, 1.0))
    handle_mat = Material("soft_gray_handle", color=(0.18, 0.20, 0.21, 1.0))

    width = 1.25
    depth = 0.72
    body_height = 0.78
    wall = 0.060
    bottom = 0.075

    lid_width = 1.32
    lid_depth = 0.77
    lid_thickness = 0.10
    gasket_height = 0.028
    gasket_width = 0.055

    hinge_y = -depth / 2.0 - 0.030
    hinge_z = body_height + 0.040
    hinge_radius = 0.020

    body = model.part("body")
    body.visual(
        Box((width, depth, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=enamel,
        name="bottom_pan",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_height / 2.0)),
        material=enamel,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, body_height / 2.0)),
        material=enamel,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, body_height / 2.0)),
        material=enamel,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, body_height / 2.0)),
        material=enamel,
        name="side_wall_1",
    )

    # A dark bottom plane and light inner liner make the top read as a real,
    # insulated open chest rather than a solid block.
    body.visual(
        Box((width - 2.0 * wall, depth - 2.0 * wall, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, bottom + 0.009)),
        material=shadow,
        name="cavity_floor",
    )
    body.visual(
        Box((width - 2.0 * wall, 0.014, body_height - bottom)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall - 0.007, bottom + (body_height - bottom) / 2.0)),
        material=liner,
        name="inner_front_liner",
    )
    body.visual(
        Box((width - 2.0 * wall, 0.014, body_height - bottom)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + 0.007, bottom + (body_height - bottom) / 2.0)),
        material=liner,
        name="inner_rear_liner",
    )
    body.visual(
        Box((0.014, depth - 2.0 * wall, body_height - bottom)),
        origin=Origin(xyz=(-width / 2.0 + wall + 0.007, 0.0, bottom + (body_height - bottom) / 2.0)),
        material=liner,
        name="inner_side_liner_0",
    )
    body.visual(
        Box((0.014, depth - 2.0 * wall, body_height - bottom)),
        origin=Origin(xyz=(width / 2.0 - wall - 0.007, 0.0, bottom + (body_height - bottom) / 2.0)),
        material=liner,
        name="inner_side_liner_1",
    )

    body.visual(
        Box((width + 0.030, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.010, body_height - 0.020)),
        material=metal,
        name="rear_hinge_backer",
    )

    # Alternating full-width hinge knuckles around the same revolute axis.
    parent_knuckles = [(-0.45, 0.18), (0.0, 0.18), (0.45, 0.18)]
    child_knuckles = [(-0.225, 0.18), (0.225, 0.18)]
    for idx, (x, length) in enumerate(parent_knuckles):
        body.visual(
            Box((length, 0.044, 0.052)),
            origin=Origin(xyz=(x, hinge_y + 0.022, hinge_z - 0.046)),
            material=metal,
            name=f"fixed_hinge_leaf_{idx}",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"fixed_hinge_knuckle_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, lid_depth / 2.0 + 0.035, 0.035)),
        material=enamel,
        name="lid_shell",
    )
    lid.visual(
        Box((0.46, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, lid_depth + 0.015, 0.004)),
        material=handle_mat,
        name="front_handle",
    )

    # The rubber strips project below the lid and meet the top rim at q=0.
    lid.visual(
        Box((lid_width - 2.0 * gasket_width, gasket_width, gasket_height)),
        origin=Origin(xyz=(0.0, lid_depth - gasket_width / 2.0 - 0.022, -0.026)),
        material=rubber,
        name="front_gasket",
    )
    lid.visual(
        Box((lid_width - 2.0 * gasket_width, gasket_width, gasket_height)),
        origin=Origin(xyz=(0.0, gasket_width / 2.0 + 0.062, -0.026)),
        material=rubber,
        name="rear_gasket",
    )
    lid.visual(
        Box((gasket_width, lid_depth - 2.0 * gasket_width - 0.044, gasket_height)),
        origin=Origin(xyz=(-lid_width / 2.0 + gasket_width / 2.0 + 0.022, lid_depth / 2.0, -0.026)),
        material=rubber,
        name="side_gasket_0",
    )
    lid.visual(
        Box((gasket_width, lid_depth - 2.0 * gasket_width - 0.044, gasket_height)),
        origin=Origin(xyz=(lid_width / 2.0 - gasket_width / 2.0 - 0.022, lid_depth / 2.0, -0.026)),
        material=rubber,
        name="side_gasket_1",
    )

    for idx, (x, length) in enumerate(child_knuckles):
        lid.visual(
            Box((length, 0.044, 0.056)),
            origin=Origin(xyz=(x, 0.022, -0.010)),
            material=metal,
            name=f"moving_hinge_leaf_{idx}",
        )
        lid.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"moving_hinge_knuckle_{idx}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.check(
        "lid uses rear revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (1.0, 0.0, 0.0)
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper >= 1.2,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_gasket",
            negative_elem="front_wall",
            max_gap=0.002,
            max_penetration=0.001,
            name="front gasket seats on body rim",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="rear_gasket",
            negative_elem="rear_wall",
            max_gap=0.002,
            max_penetration=0.001,
            name="rear gasket seats on body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="front_wall",
            min_overlap=0.04,
            name="closed lid overhangs the front rim",
        )

    rest_front = ctx.part_element_world_aabb(lid, elem="front_gasket")
    with ctx.pose({hinge: 1.25}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_gasket")
    ctx.check(
        "lid swings upward from rear hinge",
        rest_front is not None
        and opened_front is not None
        and opened_front[0][2] > rest_front[0][2] + 0.35,
        details=f"rest={rest_front}, opened={opened_front}",
    )

    return ctx.report()


object_model = build_object_model()
