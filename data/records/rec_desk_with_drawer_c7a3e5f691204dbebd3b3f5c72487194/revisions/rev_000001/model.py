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
    model = ArticulatedObject(name="writing_desk")

    wood = model.material("wood", rgba=(0.63, 0.48, 0.30, 1.0))
    drawer_box_finish = model.material("drawer_box_finish", rgba=(0.50, 0.35, 0.21, 1.0))
    runner_finish = model.material("runner_finish", rgba=(0.25, 0.25, 0.27, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.67, 0.38, 1.0))

    desk_width = 1.20
    desk_depth = 0.60
    desk_height = 0.75
    top_thickness = 0.03
    top_underside = desk_height - top_thickness

    leg_size = 0.05
    leg_height = top_underside
    leg_x = desk_width / 2.0 - leg_size / 2.0
    leg_y = desk_depth / 2.0 - leg_size / 2.0

    apron_height = 0.10
    apron_thickness = 0.02
    apron_center_z = top_underside - apron_height / 2.0
    side_apron_length = desk_depth - 2.0 * leg_size
    rear_apron_length = desk_width - 2.0 * leg_size
    drawer_opening_width = 0.46
    front_rail_length = (rear_apron_length - drawer_opening_width) / 2.0
    front_rail_x = drawer_opening_width / 2.0 + front_rail_length / 2.0

    drawer_front_width = 0.44
    drawer_front_height = 0.09
    drawer_front_thickness = 0.018
    drawer_reveal = 0.006
    drawer_center_z = top_underside - drawer_reveal - drawer_front_height / 2.0

    drawer_body_width = 0.40
    drawer_body_depth = 0.43
    drawer_side_thickness = 0.012
    drawer_back_thickness = 0.012
    drawer_bottom_thickness = 0.008
    drawer_box_height = 0.074
    drawer_box_top_local = 0.030
    drawer_box_bottom_local = drawer_box_top_local - drawer_box_height

    glide_thickness = 0.006
    glide_height = 0.010
    glide_length = 0.38
    glide_center_z = 0.025
    glide_center_y = -0.235

    runner_width = 0.012
    runner_height = 0.014
    runner_length = 0.42
    runner_center_x = drawer_body_width / 2.0 + glide_thickness + runner_width / 2.0
    runner_center_y = 0.06
    runner_center_z = drawer_center_z + glide_center_z + glide_height / 2.0 + runner_height / 2.0
    runner_mount_height = top_underside - (runner_center_z + runner_height / 2.0)

    frame = model.part("frame")
    frame.visual(
        Box((desk_width, desk_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=wood,
        name="top_panel",
    )

    for index, (x_sign, y_sign) in enumerate(
        ((-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0), (1.0, -1.0))
    ):
        frame.visual(
            Box((leg_size, leg_size, leg_height)),
            origin=Origin(xyz=(x_sign * leg_x, y_sign * leg_y, leg_height / 2.0)),
            material=wood,
            name=f"leg_{index}",
        )

    for index, x_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((apron_thickness, side_apron_length, apron_height)),
            origin=Origin(xyz=(x_sign * (desk_width / 2.0 - leg_size - apron_thickness / 2.0), 0.0, apron_center_z)),
            material=wood,
            name=f"side_apron_{index}",
        )

    frame.visual(
        Box((rear_apron_length, apron_thickness, apron_height)),
        origin=Origin(
            xyz=(0.0, -desk_depth / 2.0 + leg_size + apron_thickness / 2.0, apron_center_z)
        ),
        material=wood,
        name="rear_apron",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((front_rail_length, apron_thickness, drawer_front_height)),
            origin=Origin(
                xyz=(
                    x_sign * front_rail_x,
                    desk_depth / 2.0 - leg_size - apron_thickness / 2.0,
                    drawer_center_z,
                )
            ),
            material=wood,
            name=f"front_rail_{index}",
        )

    for index, x_sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((runner_width, runner_length, runner_height)),
            origin=Origin(
                xyz=(x_sign * runner_center_x, runner_center_y, runner_center_z)
            ),
            material=runner_finish,
            name=f"runner_{index}",
        )
        for mount_index, mount_y in enumerate((-0.08, 0.19)):
            frame.visual(
                Box((runner_width, 0.03, runner_mount_height)),
                origin=Origin(
                    xyz=(
                        x_sign * runner_center_x,
                        mount_y,
                        runner_center_z + runner_height / 2.0 + runner_mount_height / 2.0,
                    )
                ),
                material=wood,
                name=f"runner_mount_{index}_{mount_index}",
            )

    drawer = model.part("drawer")
    drawer.visual(
        Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
        origin=Origin(xyz=(0.0, -drawer_front_thickness / 2.0, 0.0)),
        material=wood,
        name="drawer_front",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        drawer.visual(
            Box((drawer_side_thickness, drawer_body_depth - drawer_front_thickness, drawer_box_height)),
            origin=Origin(
                xyz=(
                    x_sign * (drawer_body_width / 2.0 - drawer_side_thickness / 2.0),
                    -(drawer_front_thickness + (drawer_body_depth - drawer_front_thickness) / 2.0),
                    drawer_box_top_local - drawer_box_height / 2.0,
                )
            ),
            material=drawer_box_finish,
            name=f"drawer_side_{index}",
        )

    drawer.visual(
        Box(
            (
                drawer_body_width - 2.0 * drawer_side_thickness,
                drawer_body_depth - drawer_front_thickness - drawer_back_thickness,
                drawer_bottom_thickness,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -(drawer_front_thickness + (drawer_body_depth - drawer_front_thickness - drawer_back_thickness) / 2.0),
                drawer_box_bottom_local + drawer_bottom_thickness / 2.0,
            )
        ),
        material=drawer_box_finish,
        name="drawer_bottom",
    )
    drawer.visual(
        Box(
            (
                drawer_body_width - 2.0 * drawer_side_thickness,
                drawer_back_thickness,
                0.062,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -(drawer_body_depth - drawer_back_thickness / 2.0),
                drawer_box_bottom_local + 0.031,
            )
        ),
        material=drawer_box_finish,
        name="drawer_back",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        drawer.visual(
            Box((glide_thickness, glide_length, glide_height)),
            origin=Origin(
                xyz=(
                    x_sign * (drawer_body_width / 2.0 + glide_thickness / 2.0),
                    glide_center_y,
                    glide_center_z,
                )
            ),
            material=runner_finish,
            name=f"glide_{index}",
        )

    drawer.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pull_stem",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pull_knob",
    )

    model.articulation(
        "frame_to_drawer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(0.0, desk_depth / 2.0, drawer_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("frame_to_drawer")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    ctx.expect_gap(
        frame,
        drawer,
        axis="z",
        positive_elem="top_panel",
        negative_elem="drawer_front",
        min_gap=0.005,
        max_gap=0.008,
        name="drawer front sits just below the desktop",
    )
    ctx.expect_overlap(
        frame,
        drawer,
        axes="x",
        elem_a="top_panel",
        elem_b="drawer_front",
        min_overlap=0.40,
        name="drawer remains centered under the top",
    )
    ctx.expect_overlap(
        frame,
        drawer,
        axes="y",
        elem_a="runner_0",
        elem_b="glide_0",
        min_overlap=0.28,
        name="left glide is fully engaged at rest",
    )
    ctx.expect_overlap(
        frame,
        drawer,
        axes="y",
        elem_a="runner_1",
        elem_b="glide_1",
        min_overlap=0.28,
        name="right glide is fully engaged at rest",
    )

    rest_pos = ctx.part_world_position(drawer)
    if upper is not None:
        with ctx.pose({slide: upper}):
            ctx.expect_overlap(
                frame,
                drawer,
                axes="y",
                elem_a="runner_0",
                elem_b="glide_0",
                min_overlap=0.16,
                name="left glide stays retained when the drawer is open",
            )
            ctx.expect_overlap(
                frame,
                drawer,
                axes="y",
                elem_a="runner_1",
                elem_b="glide_1",
                min_overlap=0.16,
                name="right glide stays retained when the drawer is open",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            "drawer extends forward",
            rest_pos is not None
            and open_pos is not None
            and open_pos[1] > rest_pos[1] + 0.18,
            details=f"rest={rest_pos}, open={open_pos}, travel={upper}",
        )

    return ctx.report()


object_model = build_object_model()
