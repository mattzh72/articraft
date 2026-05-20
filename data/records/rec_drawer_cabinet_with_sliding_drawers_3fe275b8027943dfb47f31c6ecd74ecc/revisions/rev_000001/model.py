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

BODY_W = 1.08
BODY_D = 0.58
BODY_H = 0.80
SIDE_WALL = 0.022
TOP_WALL = 0.020
BOTTOM_WALL = 0.022
BACK_WALL = 0.020
FRAME_DEPTH = 0.024
FACE_JAMB = 0.045
TOP_FASCIA = 0.050
BOTTOM_RAIL = 0.060
SLOT_GAP = 0.012
SLOT_MARGIN = 0.006

SHALLOW_H = 0.082
DEEP_H = 0.145
SLOT_HEIGHTS = (SHALLOW_H, SHALLOW_H, SHALLOW_H, SHALLOW_H, DEEP_H, DEEP_H)

OPEN_W = BODY_W - 2.0 * FACE_JAMB
INNER_W = BODY_W - 2.0 * SIDE_WALL
DRAWER_FRONT_T = 0.020
DRAWER_FRONT_W = OPEN_W - 0.008
DRAWER_BOX_W = 0.950
DRAWER_DEPTH = 0.540
DRAWER_SHELL_T = 0.012
DRAWER_RUNNER_LEN = 0.460
DRAWER_RUNNER_T = 0.014
DRAWER_RUNNER_H = 0.020
CABINET_RUNNER_LEN = 0.480
CABINET_RUNNER_T = 0.029
CABINET_RUNNER_H = 0.022
DRAWER_TRAVEL = 0.330

WHEEL_R = 0.060
WHEEL_W = 0.040
WHEEL_X = -0.220
WHEEL_Y = 0.420
WHEEL_Z = -0.462
FORK_T = 0.008
FORK_GAP = 0.004
WHEEL_HUB_W = WHEEL_W + 2.0 * FORK_GAP
FORK_H = 0.110
FORK_PLATE_X = 0.016
FORK_TOP_X = 0.140
FORK_TOP_Y = 0.092
FORK_TOP_T = 0.012

FOOT_PAD_X = 0.055
FOOT_PAD_Y = 0.055
FOOT_PAD_T = 0.018
FOOT_STRUT_X = 0.030
FOOT_STRUT_Y = 0.030
FOOT_STRUT_H = 0.080
FOOT_X = 0.200
FOOT_Y = 0.400


def slot_centers() -> tuple[float, ...]:
    lower_open = -BODY_H / 2.0 + BOTTOM_RAIL + SLOT_MARGIN
    centers: list[float] = []
    cursor = lower_open
    for height in SLOT_HEIGHTS:
        cursor += height / 2.0
        centers.append(cursor)
        cursor += height / 2.0 + SLOT_GAP
    return tuple(centers)


SLOT_CENTERS = slot_centers()
CABINET_RUNNER_Y = INNER_W / 2.0 - CABINET_RUNNER_T / 2.0
DRAWER_RUNNER_Y = DRAWER_BOX_W / 2.0 + DRAWER_RUNNER_T / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tool_drawer_chest")

    model.material("body_red", rgba=(0.77, 0.11, 0.08, 1.0))
    model.material("steel_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("runner_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("wheel_hub", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("foot_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((BODY_D, BODY_W, TOP_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0 - TOP_WALL / 2.0)),
        material="body_red",
        name="top",
    )
    cabinet.visual(
        Box((BODY_D, BODY_W, BOTTOM_WALL)),
        origin=Origin(xyz=(0.0, 0.0, -BODY_H / 2.0 + BOTTOM_WALL / 2.0)),
        material="body_red",
        name="bottom",
    )
    cabinet.visual(
        Box((BODY_D, SIDE_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - SIDE_WALL / 2.0, 0.0)),
        material="body_red",
        name="side_0",
    )
    cabinet.visual(
        Box((BODY_D, SIDE_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + SIDE_WALL / 2.0, 0.0)),
        material="body_red",
        name="side_1",
    )
    cabinet.visual(
        Box((BACK_WALL, INNER_W, BODY_H - TOP_WALL - BOTTOM_WALL)),
        origin=Origin(xyz=(-BODY_D / 2.0 + BACK_WALL / 2.0, 0.0, 0.0)),
        material="body_red",
        name="back",
    )

    open_stack_h = BODY_H - TOP_FASCIA - BOTTOM_RAIL
    front_x = BODY_D / 2.0 - FRAME_DEPTH / 2.0

    cabinet.visual(
        Box((FRAME_DEPTH, BODY_W, TOP_FASCIA)),
        origin=Origin(xyz=(front_x, 0.0, BODY_H / 2.0 - TOP_FASCIA / 2.0)),
        material="body_red",
        name="front_top",
    )
    cabinet.visual(
        Box((FRAME_DEPTH, BODY_W, BOTTOM_RAIL)),
        origin=Origin(xyz=(front_x, 0.0, -BODY_H / 2.0 + BOTTOM_RAIL / 2.0)),
        material="body_red",
        name="front_bottom",
    )
    cabinet.visual(
        Box((FRAME_DEPTH, FACE_JAMB, open_stack_h)),
        origin=Origin(xyz=(front_x, OPEN_W / 2.0 + FACE_JAMB / 2.0, (TOP_FASCIA - BOTTOM_RAIL) / 2.0)),
        material="body_red",
        name="jamb_0",
    )
    cabinet.visual(
        Box((FRAME_DEPTH, FACE_JAMB, open_stack_h)),
        origin=Origin(xyz=(front_x, -(OPEN_W / 2.0 + FACE_JAMB / 2.0), (TOP_FASCIA - BOTTOM_RAIL) / 2.0)),
        material="body_red",
        name="jamb_1",
    )

    separator_cursor = -BODY_H / 2.0 + BOTTOM_RAIL + SLOT_MARGIN
    for index, height in enumerate(SLOT_HEIGHTS[:-1]):
        separator_cursor += height
        cabinet.visual(
            Box((FRAME_DEPTH, OPEN_W, SLOT_GAP)),
            origin=Origin(xyz=(front_x, 0.0, separator_cursor + SLOT_GAP / 2.0)),
            material="body_red",
            name=f"separator_{index}",
        )
        separator_cursor += SLOT_GAP

    runner_x = BODY_D / 2.0 - DRAWER_FRONT_T - CABINET_RUNNER_LEN / 2.0 - 0.020
    for index, center_z in enumerate(SLOT_CENTERS):
        cabinet.visual(
            Box((CABINET_RUNNER_LEN, CABINET_RUNNER_T, CABINET_RUNNER_H)),
            origin=Origin(xyz=(runner_x, CABINET_RUNNER_Y, center_z)),
            material="runner_steel",
            name=f"runner_{index}_0",
        )
        cabinet.visual(
            Box((CABINET_RUNNER_LEN, CABINET_RUNNER_T, CABINET_RUNNER_H)),
            origin=Origin(xyz=(runner_x, -CABINET_RUNNER_Y, center_z)),
            material="runner_steel",
            name=f"runner_{index}_1",
        )

    fork_top_z = -BODY_H / 2.0 + FORK_TOP_T / 2.0
    fork_plate_y_offset = WHEEL_W / 2.0 + FORK_GAP + FORK_T / 2.0
    fork_plate_z = (fork_top_z - FORK_TOP_T / 2.0 + WHEEL_Z) / 2.0
    for side_index, wheel_y in enumerate((WHEEL_Y, -WHEEL_Y)):
        cabinet.visual(
            Box((FORK_TOP_X, FORK_TOP_Y, FORK_TOP_T)),
            origin=Origin(xyz=(WHEEL_X, wheel_y, fork_top_z)),
            material="steel_dark",
            name=f"fork_top_{side_index}",
        )
        cabinet.visual(
            Box((FORK_PLATE_X, FORK_T, FORK_H)),
            origin=Origin(xyz=(WHEEL_X, wheel_y + fork_plate_y_offset, fork_plate_z)),
            material="steel_dark",
            name=f"fork_plate_{side_index}_0",
        )
        cabinet.visual(
            Box((FORK_PLATE_X, FORK_T, FORK_H)),
            origin=Origin(xyz=(WHEEL_X, wheel_y - fork_plate_y_offset, fork_plate_z)),
            material="steel_dark",
            name=f"fork_plate_{side_index}_1",
        )

    foot_z = WHEEL_Z + WHEEL_R - FOOT_PAD_T / 2.0
    foot_strut_z = (-BODY_H / 2.0 + foot_z) / 2.0
    for side_index, foot_y in enumerate((FOOT_Y, -FOOT_Y)):
        cabinet.visual(
            Box((FOOT_STRUT_X, FOOT_STRUT_Y, FOOT_STRUT_H)),
            origin=Origin(xyz=(FOOT_X, foot_y, foot_strut_z)),
            material="steel_dark",
            name=f"foot_strut_{side_index}",
        )
        cabinet.visual(
            Box((FOOT_PAD_X, FOOT_PAD_Y, FOOT_PAD_T)),
            origin=Origin(xyz=(FOOT_X, foot_y, foot_z)),
            material="foot_rubber",
            name=f"foot_pad_{side_index}",
        )

    for index, (height, center_z) in enumerate(zip(SLOT_HEIGHTS, SLOT_CENTERS)):
        drawer = model.part(f"drawer_{index}")

        front_height = height - 0.008
        box_height = height - 0.022
        shell_len = DRAWER_DEPTH - DRAWER_FRONT_T
        shell_center_x = -(DRAWER_DEPTH + DRAWER_FRONT_T) / 2.0
        back_x = -DRAWER_DEPTH + DRAWER_SHELL_T / 2.0
        drawer_runner_x = -DRAWER_FRONT_T - DRAWER_RUNNER_LEN / 2.0

        drawer.visual(
            Box((DRAWER_FRONT_T, DRAWER_FRONT_W, front_height)),
            origin=Origin(xyz=(-DRAWER_FRONT_T / 2.0, 0.0, 0.0)),
            material="body_red",
            name="front",
        )
        drawer.visual(
            Box((shell_len, DRAWER_BOX_W, DRAWER_SHELL_T)),
            origin=Origin(xyz=(shell_center_x, 0.0, -box_height / 2.0 + DRAWER_SHELL_T / 2.0)),
            material="steel_dark",
            name="tray_bottom",
        )
        drawer.visual(
            Box((shell_len, DRAWER_SHELL_T, box_height)),
            origin=Origin(xyz=(shell_center_x, DRAWER_BOX_W / 2.0 - DRAWER_SHELL_T / 2.0, 0.0)),
            material="steel_dark",
            name="tray_side_0",
        )
        drawer.visual(
            Box((shell_len, DRAWER_SHELL_T, box_height)),
            origin=Origin(xyz=(shell_center_x, -DRAWER_BOX_W / 2.0 + DRAWER_SHELL_T / 2.0, 0.0)),
            material="steel_dark",
            name="tray_side_1",
        )
        drawer.visual(
            Box((DRAWER_SHELL_T, DRAWER_BOX_W, box_height)),
            origin=Origin(xyz=(back_x, 0.0, 0.0)),
            material="steel_dark",
            name="tray_back",
        )
        drawer.visual(
            Box((DRAWER_RUNNER_LEN, DRAWER_RUNNER_T, DRAWER_RUNNER_H)),
            origin=Origin(xyz=(drawer_runner_x, DRAWER_RUNNER_Y, 0.0)),
            material="runner_steel",
            name="runner_0",
        )
        drawer.visual(
            Box((DRAWER_RUNNER_LEN, DRAWER_RUNNER_T, DRAWER_RUNNER_H)),
            origin=Origin(xyz=(drawer_runner_x, -DRAWER_RUNNER_Y, 0.0)),
            material="runner_steel",
            name="runner_1",
        )

        post_y = 0.240 if index < 4 else 0.280
        handle_w = 0.540 if index < 4 else 0.620
        drawer.visual(
            Box((0.032, 0.020, 0.028)),
            origin=Origin(xyz=(0.016, post_y, 0.0)),
            material="steel_dark",
            name="pull_post_0",
        )
        drawer.visual(
            Box((0.032, 0.020, 0.028)),
            origin=Origin(xyz=(0.016, -post_y, 0.0)),
            material="steel_dark",
            name="pull_post_1",
        )
        drawer.visual(
            Box((0.024, handle_w, 0.020)),
            origin=Origin(xyz=(0.044, 0.0, 0.0)),
            material="steel_dark",
            name="pull_bar",
        )

        model.articulation(
            f"cabinet_to_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(BODY_D / 2.0, 0.0, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=250.0,
                velocity=0.45,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    for side_index, wheel_y in enumerate((WHEEL_Y, -WHEEL_Y)):
        wheel = model.part(f"wheel_{side_index}")
        wheel.visual(
            Cylinder(radius=WHEEL_R, length=WHEEL_W),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material="wheel_rubber",
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=WHEEL_R * 0.58, length=WHEEL_HUB_W),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material="wheel_hub",
            name="hub",
        )
        model.articulation(
            f"cabinet_to_wheel_{side_index}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=wheel,
            origin=Origin(xyz=(WHEEL_X, wheel_y, WHEEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    for index in range(len(SLOT_HEIGHTS)):
        drawer = object_model.get_part(f"drawer_{index}")
        slide = object_model.get_articulation(f"cabinet_to_drawer_{index}")
        upper = slide.motion_limits.upper if slide.motion_limits is not None else DRAWER_TRAVEL
        closed_pos = ctx.part_world_position(drawer)

        ctx.expect_within(
            drawer,
            cabinet,
            axes="yz",
            elem_a="tray_bottom",
            margin=0.0,
            name=f"drawer_{index} stays centered in cabinet width and height",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="x",
            elem_a="runner_0",
            elem_b=f"runner_{index}_0",
            min_overlap=0.42,
            name=f"drawer_{index} runner remains engaged when closed",
        )

        with ctx.pose({slide: upper}):
            ctx.expect_within(
                drawer,
                cabinet,
                axes="yz",
                elem_a="tray_bottom",
                margin=0.0,
                name=f"drawer_{index} stays aligned when extended",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="x",
                elem_a="runner_0",
                elem_b=f"runner_{index}_0",
                min_overlap=0.10,
                name=f"drawer_{index} runner keeps retained insertion",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer_{index} extends outward",
            closed_pos is not None and open_pos is not None and open_pos[0] > closed_pos[0] + 0.28,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    wheel_joint_0 = object_model.get_articulation("cabinet_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("cabinet_to_wheel_1")
    wheel_0_rest = ctx.part_world_position(wheel_0)

    with ctx.pose({wheel_joint_0: 1.7, wheel_joint_1: -2.2}):
        wheel_0_spun = ctx.part_world_position(wheel_0)

    ctx.check(
        "rear wheel spin keeps axle center fixed",
        wheel_0_rest is not None and wheel_0_spun is not None and max(abs(a - b) for a, b in zip(wheel_0_rest, wheel_0_spun)) < 1e-6,
        details=f"rest={wheel_0_rest}, spun={wheel_0_spun}",
    )
    ctx.check(
        "rear wheels sit behind the cabinet centerline",
        wheel_0_rest is not None and ctx.part_world_position(wheel_1) is not None and wheel_0_rest[0] < -0.10 and ctx.part_world_position(wheel_1)[0] < -0.10,
        details=f"wheel_0={wheel_0_rest}, wheel_1={ctx.part_world_position(wheel_1)}",
    )

    return ctx.report()


object_model = build_object_model()
