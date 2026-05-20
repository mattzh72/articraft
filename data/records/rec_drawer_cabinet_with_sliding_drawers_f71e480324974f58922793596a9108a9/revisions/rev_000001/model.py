from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

RUNNER_CLEARANCE = 0.002


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_base_cabinet")

    paint = model.material("paint", rgba=(0.95, 0.95, 0.93, 1.0))
    birch = model.material("birch", rgba=(0.80, 0.73, 0.62, 1.0))
    stone = model.material("stone", rgba=(0.86, 0.86, 0.84, 1.0))
    rail = model.material("rail", rgba=(0.63, 0.65, 0.69, 1.0))
    pull = model.material("pull", rgba=(0.18, 0.18, 0.20, 1.0))

    cabinet_width = 0.80
    cabinet_depth = 0.56
    cabinet_height = 0.72
    side_thickness = 0.018
    back_thickness = 0.006
    deck_thickness = 0.018
    toe_kick_height = 0.10
    toe_kick_recess = 0.08
    toe_kick_thickness = 0.018
    rail_thickness = 0.018
    rail_depth = 0.09

    countertop_width = 0.83
    countertop_depth = 0.62
    countertop_thickness = 0.03

    drawer_front_thickness = 0.019
    drawer_side_thickness = 0.012
    drawer_bottom_thickness = 0.012
    drawer_reveal = 0.004
    drawer_front_width = cabinet_width - 2.0 * drawer_reveal
    drawer_box_width = 0.712
    runner_thickness = 0.012
    runner_height = 0.03
    runner_front_setback = 0.035
    retained_runner_overlap = 0.09

    drawer_front_heights = [0.104, 0.136, 0.168, 0.192]
    drawer_depths = [0.32, 0.38, 0.44, 0.50]

    drawer_centers_z: list[float] = []
    running_top = cabinet_height - drawer_reveal
    for front_height in drawer_front_heights:
        drawer_centers_z.append(running_top - 0.5 * front_height)
        running_top -= front_height + drawer_reveal

    carcass = model.part("carcass")
    carcass.visual(
        Box((cabinet_depth, side_thickness, cabinet_height)),
        origin=Origin(
            xyz=(
                -0.5 * cabinet_depth,
                0.5 * cabinet_width - 0.5 * side_thickness,
                0.5 * cabinet_height,
            )
        ),
        material=paint,
        name="left_side",
    )
    carcass.visual(
        Box((cabinet_depth, side_thickness, cabinet_height)),
        origin=Origin(
            xyz=(
                -0.5 * cabinet_depth,
                -0.5 * cabinet_width + 0.5 * side_thickness,
                0.5 * cabinet_height,
            )
        ),
        material=paint,
        name="right_side",
    )
    carcass.visual(
        Box((back_thickness, cabinet_width - 2.0 * side_thickness, cabinet_height)),
        origin=Origin(
            xyz=(
                -cabinet_depth + 0.5 * back_thickness,
                0.0,
                0.5 * cabinet_height,
            )
        ),
        material=paint,
        name="back",
    )
    carcass.visual(
        Box((cabinet_depth - back_thickness, cabinet_width - 2.0 * side_thickness, deck_thickness)),
        origin=Origin(
            xyz=(
                -0.5 * (cabinet_depth - back_thickness),
                0.0,
                toe_kick_height + 0.5 * deck_thickness,
            )
        ),
        material=paint,
        name="bottom_deck",
    )
    carcass.visual(
        Box((toe_kick_thickness, cabinet_width - 2.0 * side_thickness, toe_kick_height)),
        origin=Origin(
            xyz=(
                -toe_kick_recess - 0.5 * toe_kick_thickness,
                0.0,
                0.5 * toe_kick_height,
            )
        ),
        material=paint,
        name="toe_front",
    )
    carcass.visual(
        Box((toe_kick_recess, side_thickness, toe_kick_height)),
        origin=Origin(
            xyz=(
                -0.5 * toe_kick_recess,
                0.5 * cabinet_width - 0.5 * side_thickness,
                0.5 * toe_kick_height,
            )
        ),
        material=paint,
        name="toe_left",
    )
    carcass.visual(
        Box((toe_kick_recess, side_thickness, toe_kick_height)),
        origin=Origin(
            xyz=(
                -0.5 * toe_kick_recess,
                -0.5 * cabinet_width + 0.5 * side_thickness,
                0.5 * toe_kick_height,
            )
        ),
        material=paint,
        name="toe_right",
    )
    carcass.visual(
        Box((rail_depth, cabinet_width - 2.0 * side_thickness, rail_thickness)),
        origin=Origin(
            xyz=(-0.5 * rail_depth, 0.0, cabinet_height - 0.5 * rail_thickness)
        ),
        material=birch,
        name="front_rail",
    )
    carcass.visual(
        Box((rail_depth, cabinet_width - 2.0 * side_thickness, rail_thickness)),
        origin=Origin(
            xyz=(
                -cabinet_depth + 0.5 * rail_depth,
                0.0,
                cabinet_height - 0.5 * rail_thickness,
            )
        ),
        material=birch,
        name="rear_rail",
    )

    for index, (front_height, depth, center_z) in enumerate(
        zip(drawer_front_heights, drawer_depths, drawer_centers_z)
    ):
        body_height = front_height - 0.036
        body_bottom_z = -0.5 * front_height + 0.012
        body_center_z = body_bottom_z + 0.5 * body_height
        back_thickness_drawer = drawer_side_thickness
        body_depth = depth
        runner_length = depth - 0.07
        drawer = model.part(f"drawer_{index}")

        drawer.visual(
            Box((drawer_front_thickness, drawer_front_width, front_height)),
            origin=Origin(xyz=(0.5 * drawer_front_thickness, 0.0, 0.0)),
            material=paint,
            name="front",
        )
        drawer.visual(
            Box((body_depth, drawer_side_thickness, body_height)),
            origin=Origin(
                xyz=(
                    -0.5 * body_depth + 0.001,
                    0.5 * drawer_box_width - 0.5 * drawer_side_thickness,
                    body_center_z,
                )
            ),
            material=birch,
            name="left_side",
        )
        drawer.visual(
            Box((body_depth, drawer_side_thickness, body_height)),
            origin=Origin(
                xyz=(
                    -0.5 * body_depth + 0.001,
                    -0.5 * drawer_box_width + 0.5 * drawer_side_thickness,
                    body_center_z,
                )
            ),
            material=birch,
            name="right_side",
        )
        drawer.visual(
            Box((back_thickness_drawer, drawer_box_width, body_height)),
            origin=Origin(
                xyz=(-body_depth + 0.5 * back_thickness_drawer, 0.0, body_center_z)
            ),
            material=birch,
            name="back",
        )
        drawer.visual(
            Box((body_depth - back_thickness_drawer, drawer_box_width, drawer_bottom_thickness)),
            origin=Origin(
                xyz=(
                    -0.5 * (body_depth - back_thickness_drawer) + 0.001,
                    0.0,
                    body_bottom_z + 0.5 * drawer_bottom_thickness,
                )
            ),
            material=birch,
            name="bottom",
        )

        runner_z = body_bottom_z + 0.028
        drawer_runner_y = 0.5 * drawer_box_width + 0.5 * runner_thickness
        cabinet_runner_y = 0.5 * cabinet_width - side_thickness - 0.5 * runner_thickness
        runner_center_x = -runner_front_setback - 0.5 * runner_length

        drawer.visual(
            Box((runner_length, runner_thickness, runner_height)),
            origin=Origin(xyz=(runner_center_x, drawer_runner_y, runner_z)),
            material=rail,
            name="left_runner",
        )
        drawer.visual(
            Box((runner_length, runner_thickness, runner_height)),
            origin=Origin(xyz=(runner_center_x, -drawer_runner_y, runner_z)),
            material=rail,
            name="right_runner",
        )

        pull_post_y = 0.12
        pull_post_x = drawer_front_thickness + 0.008
        pull_bar_x = drawer_front_thickness + 0.022
        drawer.visual(
            Box((0.016, 0.012, 0.016)),
            origin=Origin(xyz=(pull_post_x, pull_post_y, 0.0)),
            material=pull,
            name="pull_post_a",
        )
        drawer.visual(
            Box((0.016, 0.012, 0.016)),
            origin=Origin(xyz=(pull_post_x, -pull_post_y, 0.0)),
            material=pull,
            name="pull_post_b",
        )
        drawer.visual(
            Box((0.012, 0.24, 0.012)),
            origin=Origin(xyz=(pull_bar_x, 0.0, 0.0)),
            material=pull,
            name="pull_bar",
        )

        carcass.visual(
            Box((runner_length, runner_thickness, runner_height)),
            origin=Origin(xyz=(runner_center_x, cabinet_runner_y, center_z + runner_z)),
            material=rail,
            name=f"runner_{index}_left",
        )
        carcass.visual(
            Box((runner_length, runner_thickness, runner_height)),
            origin=Origin(xyz=(runner_center_x, -cabinet_runner_y, center_z + runner_z)),
            material=rail,
            name=f"runner_{index}_right",
        )

        model.articulation(
            f"carcass_to_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.45,
                lower=0.0,
                upper=runner_length - retained_runner_overlap,
            ),
        )

    countertop = model.part("countertop")
    countertop.visual(
        Box((countertop_depth, countertop_width, countertop_thickness)),
        origin=Origin(
            xyz=(
                -0.5 * cabinet_depth,
                0.0,
                cabinet_height + 0.5 * countertop_thickness,
            )
        ),
        material=stone,
        name="slab",
    )
    model.articulation(
        "carcass_to_countertop",
        ArticulationType.FIXED,
        parent=carcass,
        child=countertop,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    countertop = object_model.get_part("countertop")

    drawers = [object_model.get_part(f"drawer_{index}") for index in range(4)]
    joints = [
        object_model.get_articulation(f"carcass_to_drawer_{index}") for index in range(4)
    ]

    ctx.expect_gap(
        countertop,
        carcass,
        axis="z",
        positive_elem="slab",
        negative_elem="left_side",
        min_gap=0.0,
        max_gap=0.001,
        name="countertop sits directly on the carcass",
    )

    cab_aabb = ctx.part_world_aabb(carcass)
    top_aabb = ctx.part_world_aabb(countertop)
    countertop_overhangs = False
    if cab_aabb is not None and top_aabb is not None:
        countertop_overhangs = (
            top_aabb[0][0] < cab_aabb[0][0]
            and top_aabb[1][0] > cab_aabb[1][0]
            and top_aabb[0][1] < cab_aabb[0][1]
            and top_aabb[1][1] > cab_aabb[1][1]
        )
    ctx.check(
        "countertop overhangs the cabinet footprint",
        countertop_overhangs,
        details=f"countertop={top_aabb}, carcass={cab_aabb}",
    )

    bottom_depths: list[float] = []
    for index, (drawer, joint) in enumerate(zip(drawers, joints)):
        ctx.expect_gap(
            drawer,
            carcass,
            axis="x",
            positive_elem="front",
            negative_elem="left_side",
            min_gap=0.0,
            max_gap=0.001,
            name=f"drawer_{index} front sits flush with the cabinet face",
        )

        drawer_bottom_aabb = ctx.part_element_world_aabb(drawer, elem="bottom")
        if drawer_bottom_aabb is not None:
            bottom_depths.append(drawer_bottom_aabb[1][0] - drawer_bottom_aabb[0][0])
        else:
            bottom_depths.append(0.0)

        rest_position = ctx.part_world_position(drawer)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        extended_position = None
        if upper is not None:
            with ctx.pose({joint: upper}):
                ctx.expect_gap(
                    carcass,
                    drawer,
                    axis="y",
                    positive_elem=f"runner_{index}_left",
                    negative_elem="left_runner",
                    min_gap=0.001,
                    max_gap=RUNNER_CLEARANCE + 0.001,
                    name=f"drawer_{index} left runner keeps side clearance in the cabinet track",
                )
                ctx.expect_gap(
                    drawer,
                    carcass,
                    axis="y",
                    positive_elem="right_runner",
                    negative_elem=f"runner_{index}_right",
                    min_gap=0.001,
                    max_gap=RUNNER_CLEARANCE + 0.001,
                    name=f"drawer_{index} right runner keeps side clearance in the cabinet track",
                )
                ctx.expect_overlap(
                    drawer,
                    carcass,
                    axes="z",
                    elem_a="left_runner",
                    elem_b=f"runner_{index}_left",
                    min_overlap=0.025,
                    name=f"drawer_{index} left runner stays level with the cabinet track",
                )
                ctx.expect_overlap(
                    drawer,
                    carcass,
                    axes="z",
                    elem_a="right_runner",
                    elem_b=f"runner_{index}_right",
                    min_overlap=0.025,
                    name=f"drawer_{index} right runner stays level with the cabinet track",
                )
                ctx.expect_overlap(
                    drawer,
                    carcass,
                    axes="x",
                    elem_a="left_runner",
                    elem_b=f"runner_{index}_left",
                    min_overlap=0.085,
                    name=f"drawer_{index} left runner retains insertion at full extension",
                )
                ctx.expect_overlap(
                    drawer,
                    carcass,
                    axes="x",
                    elem_a="right_runner",
                    elem_b=f"runner_{index}_right",
                    min_overlap=0.085,
                    name=f"drawer_{index} right runner retains insertion at full extension",
                )
                extended_position = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{index} extends outward",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.05,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    ctx.check(
        "drawer depths increase from top to bottom",
        all(
            later > earlier + 0.04
            for earlier, later in zip(bottom_depths[:-1], bottom_depths[1:])
        ),
        details=f"depths={bottom_depths}",
    )

    return ctx.report()


object_model = build_object_model()
