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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_slate")

    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.15, 1.0))
    white_paint = model.material("white_paint", rgba=(0.96, 0.96, 0.95, 1.0))
    display_tint = model.material("display_tint", rgba=(0.09, 0.14, 0.10, 1.0))

    board_w = 0.280
    board_h = 0.180
    board_t = 0.008

    housing_w = board_w
    housing_h = 0.040
    housing_d = 0.030
    wall_t = 0.003

    door_w = 0.116
    door_h = 0.028
    door_t = 0.003

    stick_w = 0.294
    stick_d = 0.018
    stick_h = 0.020

    board_top = board_h
    housing_bottom = board_top
    housing_top = housing_bottom + housing_h
    housing_center_z = housing_bottom + housing_h / 2.0

    slate = model.part("slate")
    slate.visual(
        Box((board_w, board_t, board_h)),
        origin=Origin(xyz=(0.0, 0.0, board_h / 2.0)),
        material=matte_black,
        name="board_panel",
    )

    slate.visual(
        Box((housing_w, wall_t, housing_h)),
        origin=Origin(xyz=(0.0, housing_d / 2.0 - wall_t / 2.0, housing_center_z)),
        material=charcoal,
        name="housing_front",
    )
    slate.visual(
        Box((housing_w, housing_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, housing_top - wall_t / 2.0)),
        material=charcoal,
        name="housing_top",
    )
    slate.visual(
        Box((housing_w, housing_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, housing_bottom + wall_t / 2.0)),
        material=charcoal,
        name="housing_bottom",
    )
    side_h = housing_h - 2.0 * wall_t
    slate.visual(
        Box((wall_t, housing_d, side_h)),
        origin=Origin(xyz=(-housing_w / 2.0 + wall_t / 2.0, 0.0, housing_center_z)),
        material=charcoal,
        name="housing_side_0",
    )
    slate.visual(
        Box((wall_t, housing_d, side_h)),
        origin=Origin(xyz=(housing_w / 2.0 - wall_t / 2.0, 0.0, housing_center_z)),
        material=charcoal,
        name="housing_side_1",
    )

    rear_strip_w = (housing_w - door_w) / 2.0
    rear_frame_h = housing_h - 2.0 * wall_t
    slate.visual(
        Box((rear_strip_w, wall_t, rear_frame_h)),
        origin=Origin(
            xyz=(-door_w / 2.0 - rear_strip_w / 2.0, -housing_d / 2.0 + wall_t / 2.0, housing_center_z)
        ),
        material=charcoal,
        name="rear_frame_0",
    )
    slate.visual(
        Box((rear_strip_w, wall_t, rear_frame_h)),
        origin=Origin(
            xyz=(door_w / 2.0 + rear_strip_w / 2.0, -housing_d / 2.0 + wall_t / 2.0, housing_center_z)
        ),
        material=charcoal,
        name="rear_frame_1",
    )
    rear_strip_h = wall_t
    slate.visual(
        Box((door_w, wall_t, rear_strip_h)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_d / 2.0 + wall_t / 2.0,
                housing_bottom + wall_t + rear_strip_h / 2.0,
            )
        ),
        material=charcoal,
        name="rear_frame_bottom",
    )
    slate.visual(
        Box((door_w, wall_t, rear_strip_h)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_d / 2.0 + wall_t / 2.0,
                housing_top - wall_t - rear_strip_h / 2.0,
            )
        ),
        material=charcoal,
        name="rear_frame_top",
    )

    slate.visual(
        Box((0.176, 0.002, 0.020)),
        origin=Origin(xyz=(0.0, housing_d / 2.0 + 0.001, housing_center_z + 0.002)),
        material=display_tint,
        name="timecode_window",
    )
    slate.visual(
        Box((0.060, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, housing_d / 2.0 + 0.001, housing_bottom + 0.009)),
        material=white_paint,
        name="label_strip",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((stick_w, stick_d, stick_h)),
        origin=Origin(xyz=(0.0, stick_d / 2.0, stick_h / 2.0)),
        material=matte_black,
        name="stick_body",
    )
    stripe_x = (-0.117, -0.070, -0.023, 0.024, 0.071, 0.118)
    for index, x_pos in enumerate(stripe_x):
        clapstick.visual(
            Box((0.044, 0.004, 0.028)),
            origin=Origin(
                xyz=(x_pos, stick_d - 0.002, stick_h / 2.0),
                rpy=(0.0, 0.88, 0.0),
            ),
            material=white_paint,
            name=f"stripe_{index}",
        )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.0, door_t / 2.0, -door_h / 2.0)),
        material=charcoal,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.046, 0.002, 0.006)),
        origin=Origin(xyz=(0.0, door_t, -door_h + 0.004)),
        material=white_paint,
        name="door_tab",
    )

    model.articulation(
        "clapstick_hinge",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.0, housing_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "battery_hinge",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=battery_door,
        origin=Origin(xyz=(0.0, -housing_d / 2.0, housing_top - 2.0 * wall_t)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    slate = object_model.get_part("slate")
    clapstick = object_model.get_part("clapstick")
    battery_door = object_model.get_part("battery_door")

    clapstick_hinge = object_model.get_articulation("clapstick_hinge")
    battery_hinge = object_model.get_articulation("battery_hinge")

    with ctx.pose({clapstick_hinge: 0.0, battery_hinge: 0.0}):
        ctx.expect_gap(
            clapstick,
            slate,
            axis="z",
            positive_elem="stick_body",
            negative_elem="housing_top",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed clapstick seats on the top module",
        )
        ctx.expect_overlap(
            clapstick,
            slate,
            axes="x",
            elem_a="stick_body",
            elem_b="housing_top",
            min_overlap=0.270,
            name="clapstick spans the slate width",
        )

        closed_clap_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
        closed_door_aabb = ctx.part_element_world_aabb(battery_door, elem="door_panel")
        closed_slate_aabb = ctx.part_world_aabb(slate)

    clap_upper = clapstick_hinge.motion_limits.upper if clapstick_hinge.motion_limits is not None else None
    if clap_upper is not None:
        with ctx.pose({clapstick_hinge: clap_upper}):
            open_clap_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
        ctx.check(
            "clapstick opens upward",
            closed_clap_aabb is not None
            and open_clap_aabb is not None
            and open_clap_aabb[1][2] > closed_clap_aabb[1][2] + 0.002
            and open_clap_aabb[0][1] < closed_clap_aabb[0][1] - 0.010,
            details=f"closed={closed_clap_aabb}, open={open_clap_aabb}",
        )

    door_upper = battery_hinge.motion_limits.upper if battery_hinge.motion_limits is not None else None
    if door_upper is not None:
        with ctx.pose({battery_hinge: door_upper}):
            open_door_aabb = ctx.part_element_world_aabb(battery_door, elem="door_panel")
        ctx.check(
            "battery door sits flush with the rear housing when closed",
            closed_door_aabb is not None
            and closed_slate_aabb is not None
            and abs(closed_door_aabb[0][1] - closed_slate_aabb[0][1]) <= 0.0015,
            details=f"door={closed_door_aabb}, slate={closed_slate_aabb}",
        )
        ctx.check(
            "battery door swings rearward",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.015,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
