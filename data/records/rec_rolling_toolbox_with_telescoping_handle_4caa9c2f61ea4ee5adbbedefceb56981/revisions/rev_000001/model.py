from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.40
BODY_D = 0.63
BODY_H = 0.34
BODY_WALL = 0.005
BODY_FLOOR = 0.008

LID_W = 0.406
LID_D = 0.610
LID_H = 0.078
LID_WALL = 0.0045
LID_TOP = 0.005
LID_Y_OFFSET = 0.008
LID_Z_OFFSET = -0.010

HINGE_Y = -BODY_D / 2.0 + 0.006
HINGE_Z = BODY_H / 2.0 + 0.00866

SLEEVE_X = 0.118
SLEEVE_Y = -BODY_D / 2.0 - 0.012
SLEEVE_BOTTOM = -0.050
SLEEVE_TOP = 0.290
SLEEVE_OUTER_R = 0.013
SLEEVE_INNER_R = 0.009

HANDLE_TRAVEL = 0.300
HANDLE_ROD_R = 0.008
HANDLE_ROD_BOTTOM = -0.340
HANDLE_ROD_TOP = 0.260

WHEEL_R = 0.085
WHEEL_W = 0.046
WHEEL_X = BODY_W / 2.0 + WHEEL_W / 2.0 + 0.004
WHEEL_Y = -BODY_D / 2.0 + 0.072
WHEEL_Z = -BODY_H / 2.0 + 0.060

LATCH_Y = 0.055
LATCH_PIVOT_Z = BODY_H / 2.0 - 0.055
LATCH_PIVOT_X = BODY_W / 2.0 + 0.008
LATCH_OPEN = 0.95


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name)


def _ring_box(width: float, depth: float, height: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height)
    inner = cq.Workplane("XY").box(width - 2.0 * wall, depth - 2.0 * wall, height + 0.002)
    return outer.cut(inner)


def _tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _body_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    outer = outer.edges("|Z").fillet(0.028)
    outer = outer.faces(">Z").edges().fillet(0.010)

    cavity = cq.Workplane("XY").box(
        BODY_W - 2.0 * BODY_WALL,
        BODY_D - 2.0 * BODY_WALL,
        BODY_H - BODY_FLOOR + 0.030,
    ).translate((0.0, 0.0, BODY_FLOOR / 2.0 + 0.015))
    shell = outer.cut(cavity)

    lower_band = _ring_box(BODY_W + 0.012, BODY_D + 0.012, 0.052, 0.010).translate((0.0, 0.0, -0.080))
    upper_band = _ring_box(BODY_W + 0.008, BODY_D + 0.008, 0.040, 0.009).translate((0.0, 0.0, 0.072))
    rear_beam = cq.Workplane("XY").box(0.220, 0.022, 0.110).translate((0.0, -BODY_D / 2.0 + 0.011, -0.032))
    rear_pad = cq.Workplane("XY").box(0.150, 0.028, 0.070).translate((0.0, -BODY_D / 2.0 + 0.014, 0.060))
    front_foot_right = cq.Workplane("XY").box(0.050, 0.080, 0.020).translate((0.112, BODY_D / 2.0 - 0.040, -BODY_H / 2.0 - 0.010))
    front_foot_left = cq.Workplane("XY").box(0.050, 0.080, 0.020).translate((-0.112, BODY_D / 2.0 - 0.040, -BODY_H / 2.0 - 0.010))
    latch_mount_right = cq.Workplane("XY").box(0.020, 0.055, 0.055).translate((BODY_W / 2.0 - 0.004, LATCH_Y, LATCH_PIVOT_Z))
    latch_mount_left = cq.Workplane("XY").box(0.020, 0.055, 0.055).translate((-BODY_W / 2.0 + 0.004, LATCH_Y, LATCH_PIVOT_Z))

    return (
        shell.union(lower_band)
        .union(upper_band)
        .union(rear_beam)
        .union(rear_pad)
        .union(front_foot_right)
        .union(front_foot_left)
        .union(latch_mount_right)
        .union(latch_mount_left)
    )


def _lid_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(LID_W, LID_D, LID_H).translate(
        (0.0, LID_Y_OFFSET + LID_D / 2.0, LID_Z_OFFSET + LID_H / 2.0)
    )
    outer = outer.edges("|Z").fillet(0.020)
    outer = outer.faces(">Z").edges().fillet(0.010)
    crown = cq.Workplane("XY").box(LID_W - 0.090, LID_D - 0.130, 0.014).translate(
        (0.0, LID_Y_OFFSET + LID_D / 2.0 + 0.018, LID_Z_OFFSET + LID_H - 0.010)
    )
    return outer.union(crown).union(_lid_keeper_shape(-1.0)).union(_lid_keeper_shape(1.0))


def _lid_keeper_shape(sign: float) -> cq.Workplane:
    return cq.Workplane("XY").box(0.016, 0.030, 0.010).translate(
        (sign * (BODY_W / 2.0 - 0.010), LATCH_Y - HINGE_Y, 0.005)
    )


def _sleeve_shape(x_pos: float) -> cq.Workplane:
    return _tube(SLEEVE_OUTER_R, SLEEVE_INNER_R, SLEEVE_TOP - SLEEVE_BOTTOM).translate((x_pos, SLEEVE_Y, SLEEVE_BOTTOM))


def _handle_rod_shape(x_pos: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(HANDLE_ROD_R).extrude(HANDLE_ROD_TOP - HANDLE_ROD_BOTTOM).translate(
        (x_pos, 0.0, HANDLE_ROD_BOTTOM)
    )


def _handle_crossbar(radius: float, length: float, z_pos: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((-length / 2.0, 0.0, z_pos))


def _handle_stop_shape(x_pos: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(0.014).extrude(0.006).translate((x_pos, 0.0, 0.0))


def _wheel_tire_shape() -> cq.Workplane:
    tire = cq.Workplane("YZ").circle(WHEEL_R).extrude(WHEEL_W).translate((-WHEEL_W / 2.0, 0.0, 0.0))
    right_relief = cq.Workplane("YZ").circle(WHEEL_R * 0.68).extrude(WHEEL_W * 0.18).translate(
        (WHEEL_W / 2.0 - WHEEL_W * 0.18, 0.0, 0.0)
    )
    left_relief = cq.Workplane("YZ").circle(WHEEL_R * 0.68).extrude(WHEEL_W * 0.18).translate(
        (-WHEEL_W / 2.0, 0.0, 0.0)
    )
    return tire.cut(right_relief).cut(left_relief)


def _wheel_hub_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(WHEEL_R * 0.48).extrude(WHEEL_W * 0.62).translate((-WHEEL_W * 0.31, 0.0, 0.0))
    cap = cq.Workplane("YZ").circle(WHEEL_R * 0.18).extrude(WHEEL_W * 0.74).translate((-WHEEL_W * 0.37, 0.0, 0.0))
    return hub.union(cap)


def _latch_lever_shape(sign: float) -> cq.Workplane:
    points = [
        (0.0, -0.010),
        (sign * 0.012, -0.010),
        (sign * 0.017, 0.015),
        (sign * 0.019, 0.050),
        (sign * 0.023, 0.082),
        (sign * 0.006, 0.086),
        (sign * -0.002, 0.070),
        (sign * 0.001, 0.018),
    ]
    lever = cq.Workplane("XZ").polyline(points).close().extrude(0.022).translate((0.0, -0.011, 0.0))
    pivot_nub = cq.Workplane("XY").box(0.010, 0.022, 0.018).translate((sign * -0.001, 0.0, 0.002))
    return lever.union(pivot_nub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_rolling_toolbox_base")

    shell_polymer = model.material("shell_polymer", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_polymer = model.material("trim_polymer", rgba=(0.11, 0.12, 0.13, 1.0))
    latch_polymer = model.material("latch_polymer", rgba=(0.74, 0.21, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(_mesh(_body_shell_shape(), "toolbox_body_shell"), material=shell_polymer, name="body_shell")
    body.visual(_mesh(_sleeve_shape(-SLEEVE_X), "toolbox_left_sleeve"), material=dark_steel, name="left_sleeve")
    body.visual(_mesh(_sleeve_shape(SLEEVE_X), "toolbox_right_sleeve"), material=dark_steel, name="right_sleeve")

    lid = model.part("lid")
    lid.visual(_mesh(_lid_shell_shape(), "toolbox_lid_shell"), material=trim_polymer, name="lid_shell")

    handle = model.part("handle")
    handle.visual(_mesh(_handle_rod_shape(-SLEEVE_X), "toolbox_handle_left_rod"), material=steel, name="left_rod")
    handle.visual(_mesh(_handle_rod_shape(SLEEVE_X), "toolbox_handle_right_rod"), material=steel, name="right_rod")
    handle.visual(_mesh(_handle_stop_shape(-SLEEVE_X), "toolbox_handle_left_stop"), material=steel, name="left_stop")
    handle.visual(_mesh(_handle_stop_shape(SLEEVE_X), "toolbox_handle_right_stop"), material=steel, name="right_stop")
    handle.visual(_mesh(_handle_crossbar(HANDLE_ROD_R, 2.0 * SLEEVE_X, 0.252), "toolbox_handle_crossbar"), material=steel, name="crossbar")
    handle.visual(_mesh(_handle_crossbar(0.015, 0.180, 0.252), "toolbox_handle_grip"), material=trim_polymer, name="grip")

    left_latch = model.part("left_latch")
    left_latch.visual(_mesh(_latch_lever_shape(-1.0), "toolbox_left_latch"), material=latch_polymer, name="lever")

    right_latch = model.part("right_latch")
    right_latch.visual(_mesh(_latch_lever_shape(1.0), "toolbox_right_latch"), material=latch_polymer, name="lever")

    left_wheel = model.part("left_wheel")
    left_wheel.visual(_mesh(_wheel_tire_shape(), "toolbox_left_wheel_tire"), material=rubber, name="tire")
    left_wheel.visual(_mesh(_wheel_hub_shape(), "toolbox_left_wheel_hub"), material=dark_steel, name="hub")

    right_wheel = model.part("right_wheel")
    right_wheel.visual(_mesh(_wheel_tire_shape(), "toolbox_right_wheel_tire"), material=rubber, name="tire")
    right_wheel.visual(_mesh(_wheel_hub_shape(), "toolbox_right_wheel_hub"), material=dark_steel, name="hub")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, SLEEVE_Y, SLEEVE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "body_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(-LATCH_PIVOT_X, LATCH_Y, LATCH_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=LATCH_OPEN),
    )
    model.articulation(
        "body_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(LATCH_PIVOT_X, LATCH_Y, LATCH_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=LATCH_OPEN),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=25.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    lid_joint = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_handle")
    left_latch_joint = object_model.get_articulation("body_to_left_latch")
    right_latch_joint = object_model.get_articulation("body_to_right_latch")

    with ctx.pose({lid_joint: 0.0, handle_joint: 0.0, left_latch_joint: 0.0, right_latch_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="lid closes cleanly above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.30,
            name="lid covers the body opening",
        )

        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="left_rod",
            outer_elem="left_sleeve",
            margin=0.0015,
            name="left handle rod stays centered in the left sleeve at rest",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="right_rod",
            outer_elem="right_sleeve",
            margin=0.0015,
            name="right handle rod stays centered in the right sleeve at rest",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="left_rod",
            elem_b="left_sleeve",
            min_overlap=0.32,
            name="left handle rod remains deeply inserted when collapsed",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="right_rod",
            elem_b="right_sleeve",
            min_overlap=0.32,
            name="right handle rod remains deeply inserted when collapsed",
        )

        left_lever_closed = ctx.part_element_world_aabb(left_latch, elem="lever")
        right_lever_closed = ctx.part_element_world_aabb(right_latch, elem="lever")
        ctx.check(
            "left latch bridges the lid seam",
            left_lever_closed is not None
            and left_lever_closed[0][2] < BODY_H / 2.0 - 0.010
            and left_lever_closed[1][2] > BODY_H / 2.0 + 0.015,
            details=f"left latch aabb={left_lever_closed}",
        )
        ctx.check(
            "right latch bridges the lid seam",
            right_lever_closed is not None
            and right_lever_closed[0][2] < BODY_H / 2.0 - 0.010
            and right_lever_closed[1][2] > BODY_H / 2.0 + 0.015,
            details=f"right latch aabb={right_lever_closed}",
        )

        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        closed_handle_pos = ctx.part_world_position(handle)

    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.22,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="left_rod",
            outer_elem="left_sleeve",
            margin=0.0015,
            name="left handle rod stays centered when extended",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="right_rod",
            outer_elem="right_sleeve",
            margin=0.0015,
            name="right handle rod stays centered when extended",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="left_rod",
            elem_b="left_sleeve",
            min_overlap=0.035,
            name="left handle rod retains insertion when extended",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="right_rod",
            elem_b="right_sleeve",
            min_overlap=0.035,
            name="right handle rod retains insertion when extended",
        )
        extended_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "telescoping handle rises upward",
        closed_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > closed_handle_pos[2] + 0.20,
        details=f"closed={closed_handle_pos}, extended={extended_handle_pos}",
    )

    with ctx.pose({left_latch_joint: 0.0, right_latch_joint: 0.0}):
        left_latch_closed = ctx.part_element_world_aabb(left_latch, elem="lever")
        right_latch_closed = ctx.part_element_world_aabb(right_latch, elem="lever")
    with ctx.pose({left_latch_joint: left_latch_joint.motion_limits.upper, right_latch_joint: right_latch_joint.motion_limits.upper}):
        left_latch_open = ctx.part_element_world_aabb(left_latch, elem="lever")
        right_latch_open = ctx.part_element_world_aabb(right_latch, elem="lever")
    ctx.check(
        "left latch swings outward from the shell side",
        left_latch_closed is not None
        and left_latch_open is not None
        and left_latch_open[0][0] < left_latch_closed[0][0] - 0.020,
        details=f"closed={left_latch_closed}, open={left_latch_open}",
    )
    ctx.check(
        "right latch swings outward from the shell side",
        right_latch_closed is not None
        and right_latch_open is not None
        and right_latch_open[1][0] > right_latch_closed[1][0] + 0.020,
        details=f"closed={right_latch_closed}, open={right_latch_open}",
    )

    return ctx.report()


object_model = build_object_model()
