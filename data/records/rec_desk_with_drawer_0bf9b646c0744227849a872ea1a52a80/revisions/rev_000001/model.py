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


def _add_drawer_shell(
    part,
    *,
    width: float,
    depth: float,
    height: float,
    front_material,
    shell_material,
    pull_material,
    pull_width: float,
    pull_z: float,
) -> None:
    wall_t = 0.012
    front_t = 0.018
    back_t = 0.012
    bottom_t = 0.010

    shell_depth = depth - front_t - back_t + 0.004
    shell_y = (back_t - front_t) / 2.0
    shell_height = height - bottom_t
    shell_z = -height / 2.0 + bottom_t + shell_height / 2.0

    part.visual(
        Box((width, front_t, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - front_t / 2.0, 0.0)),
        material=front_material,
        name="front_panel",
    )
    part.visual(
        Box((width - 2.0 * wall_t + 0.002, back_t, shell_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + back_t / 2.0, shell_z)),
        material=shell_material,
        name="back_panel",
    )
    part.visual(
        Box((width - 2.0 * wall_t + 0.002, shell_depth, bottom_t)),
        origin=Origin(xyz=(0.0, shell_y, -height / 2.0 + bottom_t / 2.0)),
        material=shell_material,
        name="bottom_panel",
    )
    part.visual(
        Box((wall_t, shell_depth, shell_height)),
        origin=Origin(xyz=(-width / 2.0 + wall_t / 2.0, shell_y, shell_z)),
        material=shell_material,
        name="left_wall",
    )
    part.visual(
        Box((wall_t, shell_depth, shell_height)),
        origin=Origin(xyz=(width / 2.0 - wall_t / 2.0, shell_y, shell_z)),
        material=shell_material,
        name="right_wall",
    )
    part.visual(
        Box((pull_width, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.008, pull_z)),
        material=pull_material,
        name="pull",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trestle_studio_desk")

    walnut = model.material("walnut", rgba=(0.49, 0.34, 0.22, 1.0))
    dark_laminate = model.material("dark_laminate", rgba=(0.20, 0.17, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.11, 0.12, 0.13, 1.0))

    desk = model.part("desk")
    desk.visual(
        Box((1.60, 0.75, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.732)),
        material=walnut,
        name="top_panel",
    )

    for prefix, sign in (("left", -1.0), ("right", 1.0)):
        foot_x = sign * 0.72
        upper_x = sign * 0.68
        leg_x = sign * 0.70
        leg_angle = -sign * 0.15

        desk.visual(
            Box((0.08, 0.58, 0.035)),
            origin=Origin(xyz=(foot_x, 0.0, 0.0175)),
            material=blackened_steel,
            name=f"{prefix}_foot",
        )
        desk.visual(
            Box((0.08, 0.34, 0.050)),
            origin=Origin(xyz=(upper_x, 0.0, 0.689)),
            material=blackened_steel,
            name=f"{prefix}_head",
        )
        desk.visual(
            Box((0.055, 0.055, 0.705)),
            origin=Origin(xyz=(leg_x, 0.145, 0.353), rpy=(0.0, leg_angle, 0.0)),
            material=blackened_steel,
            name=f"{prefix}_front_leg",
        )
        desk.visual(
            Box((0.055, 0.055, 0.705)),
            origin=Origin(xyz=(leg_x, -0.145, 0.353), rpy=(0.0, leg_angle, 0.0)),
            material=blackened_steel,
            name=f"{prefix}_rear_leg",
        )

    desk.visual(
        Box((1.38, 0.25, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=blackened_steel,
        name="stretcher",
    )
    desk.visual(
        Box((0.08, 0.32, 0.070)),
        origin=Origin(xyz=(-0.66, 0.0, 0.190)),
        material=blackened_steel,
        name="left_stretcher_block",
    )
    desk.visual(
        Box((0.08, 0.32, 0.070)),
        origin=Origin(xyz=(0.66, 0.0, 0.190)),
        material=blackened_steel,
        name="right_stretcher_block",
    )
    desk.visual(
        Box((1.28, 0.024, 0.060)),
        origin=Origin(xyz=(0.0, -0.313, 0.684)),
        material=graphite,
        name="rear_apron",
    )
    desk.visual(
        Box((0.22, 0.024, 0.050)),
        origin=Origin(xyz=(-0.63, 0.323, 0.689)),
        material=graphite,
        name="front_apron_left",
    )
    desk.visual(
        Box((0.022, 0.024, 0.050)),
        origin=Origin(xyz=(0.312, 0.323, 0.689)),
        material=graphite,
        name="front_apron_divider",
    )
    desk.visual(
        Box((0.10, 0.024, 0.050)),
        origin=Origin(xyz=(0.745, 0.323, 0.689)),
        material=graphite,
        name="front_apron_right",
    )

    desk.visual(
        Box((0.016, 0.40, 0.022)),
        origin=Origin(xyz=(-0.298, 0.070, 0.703)),
        material=graphite,
        name="center_runner_0",
    )
    desk.visual(
        Box((0.016, 0.40, 0.022)),
        origin=Origin(xyz=(0.298, 0.070, 0.703)),
        material=graphite,
        name="center_runner_1",
    )
    desk.visual(
        Box((0.62, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, -0.170, 0.700)),
        material=graphite,
        name="center_back_cleat",
    )

    desk.visual(
        Box((0.012, 0.44, 0.022)),
        origin=Origin(xyz=(0.324, 0.060, 0.703)),
        material=graphite,
        name="utility_runner_0",
    )
    desk.visual(
        Box((0.012, 0.44, 0.022)),
        origin=Origin(xyz=(0.606, 0.060, 0.703)),
        material=graphite,
        name="utility_runner_1",
    )
    desk.visual(
        Box((0.31, 0.050, 0.028)),
        origin=Origin(xyz=(0.465, -0.205, 0.700)),
        material=graphite,
        name="utility_back_cleat",
    )

    center_drawer = model.part("center_drawer")
    _add_drawer_shell(
        center_drawer,
        width=0.56,
        depth=0.40,
        height=0.074,
        front_material=walnut,
        shell_material=dark_laminate,
        pull_material=graphite,
        pull_width=0.180,
        pull_z=0.0,
    )
    center_drawer.visual(
        Box((0.010, 0.374, 0.020)),
        origin=Origin(xyz=(-0.285, -0.003, 0.027)),
        material=graphite,
        name="runner_bar_0",
    )
    center_drawer.visual(
        Box((0.010, 0.374, 0.020)),
        origin=Origin(xyz=(0.285, -0.003, 0.027)),
        material=graphite,
        name="runner_bar_1",
    )

    utility_drawer = model.part("utility_drawer")
    _add_drawer_shell(
        utility_drawer,
        width=0.25,
        depth=0.44,
        height=0.108,
        front_material=walnut,
        shell_material=dark_laminate,
        pull_material=graphite,
        pull_width=0.105,
        pull_z=0.010,
    )
    utility_drawer.visual(
        Box((0.010, 0.414, 0.020)),
        origin=Origin(xyz=(-0.130, -0.003, 0.044)),
        material=graphite,
        name="runner_bar_0",
    )
    utility_drawer.visual(
        Box((0.010, 0.414, 0.020)),
        origin=Origin(xyz=(0.130, -0.003, 0.044)),
        material=graphite,
        name="runner_bar_1",
    )

    model.articulation(
        "center_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=center_drawer,
        origin=Origin(xyz=(0.0, 0.135, 0.675)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=0.22,
        ),
    )
    model.articulation(
        "utility_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=utility_drawer,
        origin=Origin(xyz=(0.465, 0.115, 0.657)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk")
    center_drawer = object_model.get_part("center_drawer")
    utility_drawer = object_model.get_part("utility_drawer")
    center_slide = object_model.get_articulation("center_slide")
    utility_slide = object_model.get_articulation("utility_slide")

    center_upper = center_slide.motion_limits.upper or 0.22
    utility_upper = utility_slide.motion_limits.upper or 0.26

    ctx.expect_gap(
        desk,
        center_drawer,
        axis="z",
        positive_elem="top_panel",
        negative_elem="front_panel",
        min_gap=0.001,
        max_gap=0.015,
        name="center drawer nests tightly below the top",
    )
    ctx.expect_gap(
        desk,
        utility_drawer,
        axis="z",
        positive_elem="top_panel",
        negative_elem="front_panel",
        min_gap=0.001,
        max_gap=0.010,
        name="utility drawer nests tightly below the top",
    )
    ctx.expect_gap(
        utility_drawer,
        center_drawer,
        axis="x",
        min_gap=0.03,
        name="drawer modules stay laterally separated",
    )
    ctx.expect_origin_gap(
        utility_drawer,
        center_drawer,
        axis="x",
        min_gap=0.40,
        max_gap=0.60,
        name="utility drawer sits to the right of the pencil drawer",
    )
    ctx.expect_overlap(
        center_drawer,
        desk,
        axes="y",
        elem_a="bottom_panel",
        elem_b="center_runner_0",
        min_overlap=0.24,
        name="center drawer stays engaged on its runner at rest",
    )
    ctx.expect_overlap(
        utility_drawer,
        desk,
        axes="y",
        elem_a="bottom_panel",
        elem_b="utility_runner_0",
        min_overlap=0.28,
        name="utility drawer stays engaged on its runner at rest",
    )
    ctx.expect_contact(
        center_drawer,
        desk,
        elem_a="runner_bar_0",
        elem_b="center_runner_0",
        name="center drawer is carried by its side runner",
    )
    ctx.expect_contact(
        utility_drawer,
        desk,
        elem_a="runner_bar_0",
        elem_b="utility_runner_0",
        name="utility drawer is carried by its side runner",
    )

    center_rest = ctx.part_world_position(center_drawer)
    with ctx.pose({center_slide: center_upper}):
        ctx.expect_contact(
            center_drawer,
            desk,
            elem_a="runner_bar_0",
            elem_b="center_runner_0",
            name="center drawer stays on the runner when opened",
        )
        ctx.expect_overlap(
            center_drawer,
            desk,
            axes="y",
            elem_a="bottom_panel",
            elem_b="center_runner_0",
            min_overlap=0.09,
            name="center drawer retains insertion at full extension",
        )
        center_open = ctx.part_world_position(center_drawer)

    utility_rest = ctx.part_world_position(utility_drawer)
    with ctx.pose({utility_slide: utility_upper}):
        ctx.expect_contact(
            utility_drawer,
            desk,
            elem_a="runner_bar_0",
            elem_b="utility_runner_0",
            name="utility drawer stays on the runner when opened",
        )
        ctx.expect_overlap(
            utility_drawer,
            desk,
            axes="y",
            elem_a="bottom_panel",
            elem_b="utility_runner_0",
            min_overlap=0.10,
            name="utility drawer retains insertion at full extension",
        )
        utility_open = ctx.part_world_position(utility_drawer)

    ctx.check(
        "center drawer extends forward",
        center_rest is not None
        and center_open is not None
        and center_open[1] > center_rest[1] + 0.18,
        details=f"rest={center_rest}, open={center_open}",
    )
    ctx.check(
        "utility drawer extends forward",
        utility_rest is not None
        and utility_open is not None
        and utility_open[1] > utility_rest[1] + 0.22,
        details=f"rest={utility_rest}, open={utility_open}",
    )

    return ctx.report()


object_model = build_object_model()
