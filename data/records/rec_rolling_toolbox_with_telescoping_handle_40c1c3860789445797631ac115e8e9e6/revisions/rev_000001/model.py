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


BODY_DEPTH = 0.370
BODY_WIDTH = 0.580
BODY_HEIGHT = 0.590
BODY_WALL = 0.010

LID_DEPTH = 0.392
LID_WIDTH = 0.604
LID_HEIGHT = 0.062
LID_WALL = 0.008

DRAWER_OPENING_WIDTH = 0.470
DRAWER_OPENING_HEIGHT = 0.102
DRAWER_OPENING_BOTTOM = 0.434
DRAWER_PANEL_THICK = 0.012
DRAWER_PANEL_PROUD = 0.0015
DRAWER_BOX_DEPTH = 0.220
DRAWER_BOX_WIDTH = 0.446
DRAWER_BOX_HEIGHT = 0.080
DRAWER_WALL = 0.006
DRAWER_TRAVEL = 0.155

DRAWER_RUNNER_LENGTH = 0.195
DRAWER_RUNNER_WIDTH = 0.040
DRAWER_RUNNER_HEIGHT = 0.012
DRAWER_RUNNER_Y = 0.260
DRAWER_RUNNER_Z = DRAWER_OPENING_BOTTOM + 0.029

GUIDE_CENTER_X = -BODY_DEPTH / 2.0 - 0.010
HANDLE_RAIL_Y = 0.214
GUIDE_OUTER_Y = 0.040
GUIDE_HEIGHT = 0.230
GUIDE_BASE_Z = 0.320

HANDLE_ROD_X = 0.024
HANDLE_ROD_Y = 0.018
HANDLE_ROD_LENGTH = 0.540
HANDLE_INSERTION = 0.190
HANDLE_TRAVEL = 0.150
HANDLE_GRIP_Z = 0.330
HANDLE_BRACE_Z = 0.160
HANDLE_GRIP_DEPTH = 0.032
HANDLE_GRIP_HEIGHT = 0.028
HANDLE_GRIP_WIDTH = HANDLE_RAIL_Y * 2.0 + 0.090
HANDLE_BRACE_WIDTH = HANDLE_RAIL_Y * 2.0 + 0.030

WHEEL_CENTER_X = -0.108
WHEEL_RADIUS = 0.060
WHEEL_WIDTH = 0.036
TIRE_RADIUS = 0.082
TIRE_WIDTH = 0.044
WHEEL_CENTER_Y = BODY_WIDTH / 2.0 + TIRE_WIDTH / 2.0
WHEEL_CENTER_Z = 0.082

LATCH_X = 0.070
LATCH_PIVOT_Z = 0.508
LATCH_OFFSET_Y = BODY_WIDTH / 2.0 + 0.018
LATCH_OPEN_ANGLE = 1.10

def _add_body_visuals(body) -> None:
    wall_height = BODY_HEIGHT - BODY_WALL
    front_upper_height = BODY_HEIGHT - (DRAWER_OPENING_BOTTOM + DRAWER_OPENING_HEIGHT)
    front_lower_height = DRAWER_OPENING_BOTTOM - BODY_WALL

    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, BODY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_WALL / 2.0)),
        material="body_red",
        name="floor",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WALL, wall_height)),
        origin=Origin(xyz=(0.0, BODY_WIDTH / 2.0 - BODY_WALL / 2.0, BODY_WALL + wall_height / 2.0)),
        material="body_red",
        name="right_wall",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WALL, wall_height)),
        origin=Origin(xyz=(0.0, -BODY_WIDTH / 2.0 + BODY_WALL / 2.0, BODY_WALL + wall_height / 2.0)),
        material="body_red",
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, wall_height)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 + BODY_WALL / 2.0, 0.0, BODY_WALL + wall_height / 2.0)),
        material="body_red",
        name="rear_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, front_lower_height)),
        origin=Origin(
            xyz=(
                BODY_DEPTH / 2.0 - BODY_WALL / 2.0,
                0.0,
                BODY_WALL + front_lower_height / 2.0,
            )
        ),
        material="body_red",
        name="front_lower_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, front_upper_height)),
        origin=Origin(
            xyz=(
                BODY_DEPTH / 2.0 - BODY_WALL / 2.0,
                0.0,
                DRAWER_OPENING_BOTTOM + DRAWER_OPENING_HEIGHT + front_upper_height / 2.0,
            )
        ),
        material="body_red",
        name="front_upper_wall",
    )

    for y_sign, label in ((-1.0, "left"), (1.0, "right")):
        body.visual(
            Box((DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_WIDTH, DRAWER_RUNNER_HEIGHT)),
            origin=Origin(
                xyz=(
                    BODY_DEPTH / 2.0 - BODY_WALL - DRAWER_RUNNER_LENGTH / 2.0 - 0.006,
                    y_sign * DRAWER_RUNNER_Y,
                    DRAWER_RUNNER_Z,
                )
            ),
            material="dark_metal",
            name=f"{label}_runner",
        )
        body.visual(
            Box((0.050, 0.060, 0.022)),
            origin=Origin(
                xyz=(
                    BODY_DEPTH / 2.0 - 0.038,
                    y_sign * 0.180,
                    0.011,
                )
            ),
            material="black_plastic",
            name=f"{label}_foot",
        )
        body.visual(
            Box((0.060, 0.016, 0.088)),
            origin=Origin(
                xyz=(
                    WHEEL_CENTER_X,
                    y_sign * (BODY_WIDTH / 2.0 - 0.008),
                    0.044,
                )
            ),
            material="black_plastic",
            name=f"{label}_axle_boss",
        )
        body.visual(
            Box((0.030, 0.026, 0.022)),
            origin=Origin(
                xyz=(
                    LATCH_X,
                    y_sign * (BODY_WIDTH / 2.0),
                    LATCH_PIVOT_Z,
                )
            ),
            material="black_plastic",
            name=f"{label}_latch_mount",
        )

        cheek_y = 0.006
        cheek_x = 0.034
        back_plate_x = 0.006
        body.visual(
            Box((cheek_x, cheek_y, GUIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    GUIDE_CENTER_X - 0.007,
                    y_sign * (HANDLE_RAIL_Y - (GUIDE_OUTER_Y / 2.0 - cheek_y / 2.0)),
                    GUIDE_BASE_Z + GUIDE_HEIGHT / 2.0,
                )
            ),
            material="black_plastic",
            name=f"{label}_guide_inner_cheek",
        )
        body.visual(
            Box((cheek_x, cheek_y, GUIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    GUIDE_CENTER_X - 0.007,
                    y_sign * (HANDLE_RAIL_Y + (GUIDE_OUTER_Y / 2.0 - cheek_y / 2.0)),
                    GUIDE_BASE_Z + GUIDE_HEIGHT / 2.0,
                )
            ),
            material="black_plastic",
            name=f"{label}_guide_outer_cheek",
        )
        body.visual(
            Box((back_plate_x, GUIDE_OUTER_Y, GUIDE_HEIGHT)),
            origin=Origin(
                xyz=(
                    GUIDE_CENTER_X - cheek_x / 2.0,
                    y_sign * HANDLE_RAIL_Y,
                    GUIDE_BASE_Z + GUIDE_HEIGHT / 2.0,
                )
            ),
            material="black_plastic",
            name=f"{label}_guide_back",
        )

    body.visual(
        Box((0.044, HANDLE_RAIL_Y * 2.0 + 0.060, 0.020)),
        origin=Origin(xyz=(GUIDE_CENTER_X + 0.006, 0.0, GUIDE_BASE_Z - 0.010)),
        material="black_plastic",
        name="rear_bridge",
    )


def _add_lid_visuals(lid) -> None:
    skirt_height = LID_HEIGHT - LID_WALL
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_WALL)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, LID_HEIGHT - LID_WALL / 2.0)),
        material="black_plastic",
        name="top_panel",
    )
    lid.visual(
        Box((LID_DEPTH, LID_WALL, skirt_height)),
        origin=Origin(
            xyz=(LID_DEPTH / 2.0, LID_WIDTH / 2.0 - LID_WALL / 2.0, skirt_height / 2.0)
        ),
        material="black_plastic",
        name="right_skirt",
    )
    lid.visual(
        Box((LID_DEPTH, LID_WALL, skirt_height)),
        origin=Origin(
            xyz=(LID_DEPTH / 2.0, -LID_WIDTH / 2.0 + LID_WALL / 2.0, skirt_height / 2.0)
        ),
        material="black_plastic",
        name="left_skirt",
    )
    lid.visual(
        Box((LID_WALL, LID_WIDTH - 2.0 * LID_WALL, skirt_height)),
        origin=Origin(
            xyz=(LID_DEPTH - LID_WALL / 2.0, 0.0, skirt_height / 2.0)
        ),
        material="black_plastic",
        name="front_skirt",
    )
    lid.visual(
        Box((0.018, LID_WIDTH - 2.0 * LID_WALL, 0.024)),
        origin=Origin(xyz=(0.009, 0.0, 0.012)),
        material="black_plastic",
        name="rear_band",
    )
    latch_local_x = LATCH_X + BODY_DEPTH / 2.0
    for y_sign, label in ((-1.0, "left"), (1.0, "right")):
        lid.visual(
            Box((0.022, 0.014, 0.018)),
            origin=Origin(
                xyz=(
                    latch_local_x,
                    y_sign * (LID_WIDTH / 2.0 - 0.014),
                    0.014,
                )
            ),
            material="black_plastic",
            name=f"{label}_catch",
        )


def _add_drawer_visuals(drawer) -> None:
    wall_height = DRAWER_BOX_HEIGHT - DRAWER_WALL
    drawer.visual(
        Box((DRAWER_BOX_DEPTH, DRAWER_BOX_WIDTH, DRAWER_WALL)),
        origin=Origin(xyz=(-DRAWER_BOX_DEPTH / 2.0, 0.0, DRAWER_WALL / 2.0)),
        material="dark_metal",
        name="tray_floor",
    )
    drawer.visual(
        Box((DRAWER_BOX_DEPTH, DRAWER_WALL, wall_height)),
        origin=Origin(
            xyz=(-DRAWER_BOX_DEPTH / 2.0, DRAWER_BOX_WIDTH / 2.0 - DRAWER_WALL / 2.0, DRAWER_WALL + wall_height / 2.0)
        ),
        material="dark_metal",
        name="tray_right_wall",
    )
    drawer.visual(
        Box((DRAWER_BOX_DEPTH, DRAWER_WALL, wall_height)),
        origin=Origin(
            xyz=(-DRAWER_BOX_DEPTH / 2.0, -DRAWER_BOX_WIDTH / 2.0 + DRAWER_WALL / 2.0, DRAWER_WALL + wall_height / 2.0)
        ),
        material="dark_metal",
        name="tray_left_wall",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_BOX_WIDTH - 2.0 * DRAWER_WALL, wall_height)),
        origin=Origin(
            xyz=(-(DRAWER_BOX_DEPTH - DRAWER_WALL / 2.0), 0.0, DRAWER_WALL + wall_height / 2.0)
        ),
        material="dark_metal",
        name="tray_rear_wall",
    )
    drawer.visual(
        Box((DRAWER_PANEL_THICK, DRAWER_OPENING_WIDTH, DRAWER_OPENING_HEIGHT)),
        origin=Origin(xyz=(DRAWER_PANEL_THICK / 2.0, 0.0, DRAWER_OPENING_HEIGHT / 2.0)),
        material="dark_metal",
        name="front_panel",
    )
    drawer.visual(
        Box((0.018, 0.200, 0.014)),
        origin=Origin(xyz=(DRAWER_PANEL_THICK + 0.008, 0.0, DRAWER_OPENING_HEIGHT * 0.48)),
        material="black_plastic",
        name="pull_bar",
    )

    for y_sign, label in ((-1.0, "left"), (1.0, "right")):
        drawer.visual(
            Box((DRAWER_RUNNER_LENGTH - 0.020, 0.020, 0.012)),
            origin=Origin(
                xyz=(
                    -DRAWER_BOX_DEPTH + (DRAWER_RUNNER_LENGTH - 0.020) / 2.0 + 0.020,
                    y_sign * (DRAWER_BOX_WIDTH / 2.0 + 0.007),
                    DRAWER_BOX_HEIGHT * 0.42,
                )
            ),
            material="dark_metal",
            name=f"{label}_slide",
        )


def _add_handle_visuals(handle) -> None:
    rail_center_z = HANDLE_ROD_LENGTH / 2.0 - HANDLE_INSERTION
    for y_sign, label in ((-1.0, "left"), (1.0, "right")):
        handle.visual(
            Box((HANDLE_ROD_X, HANDLE_ROD_Y, HANDLE_ROD_LENGTH)),
            origin=Origin(xyz=(0.0, y_sign * HANDLE_RAIL_Y, rail_center_z)),
            material="aluminum",
            name=f"{label}_rail",
        )
    handle.visual(
        Box((HANDLE_GRIP_DEPTH, HANDLE_GRIP_WIDTH, HANDLE_GRIP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, HANDLE_GRIP_Z)),
        material="black_plastic",
        name="grip",
    )
    handle.visual(
        Box((HANDLE_ROD_X, HANDLE_BRACE_WIDTH, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, HANDLE_BRACE_Z)),
        material="aluminum",
        name="brace",
    )


def _add_wheel_visuals(wheel) -> None:
    wheel.visual(
        Cylinder(radius=TIRE_RADIUS, length=TIRE_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tire_rubber",
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="wheel_steel",
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="wheel_steel",
        name="hub",
    )


def _add_latch_visuals(latch) -> None:
    latch.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_metal",
        name="pivot",
    )
    latch.visual(
        Box((0.012, 0.006, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="dark_metal",
        name="lever",
    )
    latch.visual(
        Box((0.012, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material="dark_metal",
        name="hook",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    model.material("body_red", rgba=(0.76, 0.13, 0.10, 1.0))
    model.material("black_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("dark_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("wheel_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("tire_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    _add_body_visuals(body)

    lid = model.part("lid")
    _add_lid_visuals(lid)

    drawer = model.part("drawer")
    _add_drawer_visuals(drawer)

    handle = model.part("rear_handle")
    _add_handle_visuals(handle)

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(left_wheel)

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(right_wheel)

    left_latch = model.part("left_latch")
    _add_latch_visuals(left_latch)

    right_latch = model.part("right_latch")
    _add_latch_visuals(right_latch)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-BODY_DEPTH / 2.0, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4, lower=0.0, upper=1.55),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(
            xyz=(
                BODY_DEPTH / 2.0 + DRAWER_PANEL_PROUD,
                0.0,
                DRAWER_OPENING_BOTTOM,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_BASE_Z + GUIDE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, -WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    model.articulation(
        "left_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(LATCH_X, -LATCH_OFFSET_Y, LATCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=LATCH_OPEN_ANGLE),
    )
    model.articulation(
        "right_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(LATCH_X, LATCH_OFFSET_Y, LATCH_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=LATCH_OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    handle = object_model.get_part("rear_handle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    handle_slide = object_model.get_articulation("handle_slide")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    left_latch_pivot = object_model.get_articulation("left_latch_pivot")
    right_latch_pivot = object_model.get_articulation("right_latch_pivot")

    with ctx.pose({lid_hinge: 0.0, drawer_slide: 0.0, handle_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.010,
            max_penetration=0.0,
            name="lid closes onto the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.320,
            name="lid covers the body footprint",
        )

        closed_body_aabb = ctx.part_world_aabb(body)
        closed_drawer_aabb = ctx.part_world_aabb(drawer)
        closed_handle_aabb = ctx.part_world_aabb(handle)
        drawer_front_gap = None
        handle_inserted = None
        closed_front_panel_aabb = ctx.part_element_world_aabb(drawer, elem="front_panel")
        if closed_body_aabb is not None and closed_front_panel_aabb is not None:
            drawer_front_gap = closed_front_panel_aabb[1][0] - closed_body_aabb[1][0]
        if closed_handle_aabb is not None:
            handle_inserted = closed_handle_aabb[0][2] < GUIDE_BASE_Z + GUIDE_HEIGHT - 0.12

        ctx.check(
            "drawer front sits proud of shell",
            drawer_front_gap is not None and 0.004 <= drawer_front_gap <= 0.020,
            details=f"front_gap={drawer_front_gap}",
        )
        ctx.check(
            "collapsed handle remains captured in the guides",
            bool(handle_inserted),
            details=f"handle_aabb={closed_handle_aabb}",
        )

        left_wheel_aabb = ctx.part_world_aabb(left_wheel)
        right_wheel_aabb = ctx.part_world_aabb(right_wheel)
        ctx.check(
            "transport wheels sit outside the shell sides",
            left_wheel_aabb is not None
            and right_wheel_aabb is not None
            and left_wheel_aabb[1][1] <= -BODY_WIDTH / 2.0
            and right_wheel_aabb[0][1] >= BODY_WIDTH / 2.0,
            details=f"left={left_wheel_aabb}, right={right_wheel_aabb}",
        )
        ctx.check(
            "wheel joints are continuous rotors",
            left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
            details=f"left={left_wheel_spin.articulation_type}, right={right_wheel_spin.articulation_type}",
        )

        left_latch_aabb = ctx.part_world_aabb(left_latch)
        right_latch_aabb = ctx.part_world_aabb(right_latch)
        ctx.check(
            "side latches bridge the lid seam when closed",
            left_latch_aabb is not None
            and right_latch_aabb is not None
            and left_latch_aabb[0][2] < BODY_HEIGHT - 0.02
            and left_latch_aabb[1][2] > BODY_HEIGHT + 0.02
            and right_latch_aabb[0][2] < BODY_HEIGHT - 0.02
            and right_latch_aabb[1][2] > BODY_HEIGHT + 0.02,
            details=f"left={left_latch_aabb}, right={right_latch_aabb}",
        )

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    handle_upper = handle_slide.motion_limits.upper if handle_slide.motion_limits is not None else None

    if lid_upper is not None:
        closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    if drawer_upper is not None:
        closed_drawer_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: drawer_upper}):
            open_drawer_pos = ctx.part_world_position(drawer)
            open_drawer_aabb = ctx.part_world_aabb(drawer)
        ctx.check(
            "drawer slides out the front",
            closed_drawer_pos is not None
            and open_drawer_pos is not None
            and open_drawer_pos[0] > closed_drawer_pos[0] + 0.10,
            details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
        )
        ctx.check(
            "drawer remains retained on its runners",
            open_drawer_aabb is not None and open_drawer_aabb[0][0] < BODY_DEPTH / 2.0 - 0.020,
            details=f"drawer_aabb={open_drawer_aabb}",
        )

    if handle_upper is not None:
        closed_handle_pos = ctx.part_world_position(handle)
        with ctx.pose({handle_slide: handle_upper}):
            extended_handle_pos = ctx.part_world_position(handle)
            extended_handle_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "rear handle telescopes upward",
            closed_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > closed_handle_pos[2] + 0.10,
            details=f"closed={closed_handle_pos}, extended={extended_handle_pos}",
        )
        ctx.check(
            "extended handle still keeps insertion in the guide sleeves",
            extended_handle_aabb is not None and extended_handle_aabb[0][2] <= GUIDE_BASE_Z + GUIDE_HEIGHT - 0.040 + 1e-6,
            details=f"handle_aabb={extended_handle_aabb}",
        )

    left_latch_upper = left_latch_pivot.motion_limits.upper if left_latch_pivot.motion_limits is not None else None
    right_latch_upper = right_latch_pivot.motion_limits.upper if right_latch_pivot.motion_limits is not None else None
    if left_latch_upper is not None and right_latch_upper is not None:
        closed_left_latch = ctx.part_world_aabb(left_latch)
        closed_right_latch = ctx.part_world_aabb(right_latch)
        with ctx.pose({left_latch_pivot: left_latch_upper, right_latch_pivot: right_latch_upper}):
            open_left_latch = ctx.part_world_aabb(left_latch)
            open_right_latch = ctx.part_world_aabb(right_latch)
        ctx.check(
            "latches pivot outward from the shell sides",
            closed_left_latch is not None
            and closed_right_latch is not None
            and open_left_latch is not None
            and open_right_latch is not None
            and open_left_latch[0][1] < closed_left_latch[0][1] - 0.01
            and open_right_latch[1][1] > closed_right_latch[1][1] + 0.01,
            details=(
                f"closed_left={closed_left_latch}, open_left={open_left_latch}, "
                f"closed_right={closed_right_latch}, open_right={open_right_latch}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
