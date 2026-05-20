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
    model = ArticulatedObject(name="watch_winder_presentation_box")

    lacquer = Material("black_lacquer", rgba=(0.015, 0.012, 0.010, 1.0))
    satin_black = Material("satin_black", rgba=(0.035, 0.035, 0.038, 1.0))
    walnut = Material("dark_walnut", rgba=(0.20, 0.095, 0.035, 1.0))
    velvet = Material("burgundy_velvet", rgba=(0.30, 0.025, 0.055, 1.0))
    cream = Material("cream_suede", rgba=(0.78, 0.66, 0.48, 1.0))
    metal = Material("brushed_steel", rgba=(0.62, 0.60, 0.56, 1.0))
    glass = Material("smoked_glass", rgba=(0.25, 0.38, 0.45, 0.38))

    body_w = 0.440
    body_d = 0.280
    floor_h = 0.025
    wall_t = 0.020
    wall_h = 0.095
    top_z = floor_h + wall_h

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, floor_h)),
        origin=Origin(xyz=(0.0, 0.0, floor_h / 2.0)),
        material=lacquer,
        name="floor",
    )
    body.visual(
        Box((wall_t, body_d, wall_h)),
        origin=Origin(xyz=(-(body_w - wall_t) / 2.0, 0.0, floor_h + wall_h / 2.0)),
        material=walnut,
        name="side_wall_0",
    )
    body.visual(
        Box((wall_t, body_d, wall_h)),
        origin=Origin(xyz=((body_w - wall_t) / 2.0, 0.0, floor_h + wall_h / 2.0)),
        material=walnut,
        name="side_wall_1",
    )
    body.visual(
        Box((body_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -(body_d - wall_t) / 2.0, floor_h + wall_h / 2.0)),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((body_w, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, (body_d - wall_t) / 2.0, floor_h + wall_h / 2.0)),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, body_d - 2.0 * wall_t, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, floor_h + 0.003)),
        material=velvet,
        name="velvet_liner",
    )

    # A low internal motor plinth and boss carry the rotating spindle.
    body.visual(
        Box((0.160, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, 0.072, floor_h + 0.045)),
        material=satin_black,
        name="motor_panel",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(xyz=(0.0, 0.052, floor_h + 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="bearing_boss",
    )

    # Exposed hinge knuckles are mounted just behind the rear wall; the moving
    # lid carries the center knuckle.
    hinge_y = body_d / 2.0 + 0.013
    hinge_z = top_z + 0.012
    for i, x in enumerate((-0.150, 0.150)):
        body.visual(
            Box((0.110, 0.014, 0.026)),
            origin=Origin(xyz=(x, body_d / 2.0 + 0.007, top_z + 0.003)),
            material=metal,
            name=f"hinge_bracket_{i}",
        )
        body.visual(
            Cylinder(radius=0.007, length=0.110),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_knuckle_{i}",
        )

    # Four small feet keep the presentation box low but visibly supported.
    for ix, x in enumerate((-0.165, 0.165)):
        for iy, y in enumerate((-0.095, 0.095)):
            body.visual(
                Box((0.055, 0.035, 0.010)),
                origin=Origin(xyz=(x, y, -0.005)),
                material=satin_black,
                name=f"foot_{ix}_{iy}",
            )

    lid = model.part("lid")
    lid_w = 0.405
    lid_len = 0.270
    rail = 0.026
    lid_t = 0.014
    lid_y_center = -0.159
    frame_z = -0.005
    side_x = (lid_w - rail) / 2.0
    for i, x in enumerate((-side_x, side_x)):
        lid.visual(
            Box((rail, lid_len, lid_t)),
            origin=Origin(xyz=(x, lid_y_center, frame_z)),
            material=walnut,
            name=f"lid_side_{i}",
        )
    lid.visual(
        Box((lid_w, rail, lid_t)),
        origin=Origin(xyz=(0.0, -0.281, frame_z)),
        material=walnut,
        name="lid_front",
    )
    lid.visual(
        Box((lid_w, rail, lid_t)),
        origin=Origin(xyz=(0.0, -0.037, frame_z)),
        material=walnut,
        name="lid_rear",
    )
    lid.visual(
        Box((0.357, 0.226, 0.004)),
        origin=Origin(xyz=(0.0, lid_y_center, -0.002)),
        material=glass,
        name="glass_pane",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.160),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.160, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, -0.016, -0.006)),
        material=metal,
        name="lid_hinge_leaf",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    cradle = model.part("cradle")
    spindle_z = floor_h + 0.055
    spindle_y = 0.030
    cradle.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="spindle_pin",
    )
    cradle.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="cradle_disc",
    )
    cradle.visual(
        Box((0.080, 0.026, 0.052)),
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
        material=cream,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.014, 0.030, 0.072)),
        origin=Origin(xyz=(0.0, -0.047, 0.0)),
        material=velvet,
        name="retaining_strap",
    )
    cradle.visual(
        Box((0.030, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.064, 0.040)),
        material=metal,
        name="strap_buckle",
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, spindle_y, spindle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    spindle_joint = object_model.get_articulation("body_to_cradle")

    ctx.allow_overlap(
        body,
        cradle,
        elem_a="bearing_boss",
        elem_b="spindle_pin",
        reason="The rotating spindle pin is intentionally captured inside the fixed bearing boss.",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="xz",
        inner_elem="spindle_pin",
        outer_elem="bearing_boss",
        margin=0.001,
        name="spindle is centered in bearing",
    )
    ctx.expect_overlap(
        cradle,
        body,
        axes="y",
        elem_a="spindle_pin",
        elem_b="bearing_boss",
        min_overlap=0.008,
        name="spindle remains inserted in bearing",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front",
            negative_elem="front_wall",
            max_gap=0.002,
            max_penetration=0.0001,
            name="closed lid sits on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="glass_pane",
            elem_b="floor",
            min_overlap=0.20,
            name="wide lid panel spans the presentation box",
        )
        closed_front = ctx.part_element_world_aabb(lid, elem="lid_front")

    with ctx.pose({lid_hinge: 1.15}):
        opened_front = ctx.part_element_world_aabb(lid, elem="lid_front")

    closed_top = None if closed_front is None else closed_front[1][2]
    opened_top = None if opened_front is None else opened_front[1][2]
    ctx.check(
        "lid opens upward about rear hinge",
        closed_top is not None and opened_top is not None and opened_top > closed_top + 0.12,
        details=f"closed_top={closed_top}, opened_top={opened_top}",
    )

    with ctx.pose({spindle_joint: 0.0}):
        buckle_top_pose = ctx.part_element_world_aabb(cradle, elem="strap_buckle")
    with ctx.pose({spindle_joint: math.pi}):
        buckle_bottom_pose = ctx.part_element_world_aabb(cradle, elem="strap_buckle")

    top_center_z = None if buckle_top_pose is None else (buckle_top_pose[0][2] + buckle_top_pose[1][2]) / 2.0
    bottom_center_z = None if buckle_bottom_pose is None else (buckle_bottom_pose[0][2] + buckle_bottom_pose[1][2]) / 2.0
    ctx.check(
        "cradle rotates continuously around the spindle",
        top_center_z is not None and bottom_center_z is not None and top_center_z > bottom_center_z + 0.06,
        details=f"top_pose_z={top_center_z}, half_turn_z={bottom_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
