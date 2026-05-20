from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.70
BODY_D = 0.41
BASE_Z = 0.055
BASE_H = 0.31
BODY_TOP = BASE_Z + BASE_H
WALL = 0.005
CORNER_R = 0.024

LID_W = 0.718
LID_D = 0.428
LID_H = 0.105

CHANNEL_X = 0.235
CHANNEL_Y = -(BODY_D * 0.5 + 0.022)
CHANNEL_BOTTOM = 0.125
CHANNEL_H = 0.235
CHANNEL_OUTER_X = 0.040
CHANNEL_OUTER_Y = 0.044
CHANNEL_WALL = 0.003

STAGE_TUBE_OUTER_X = 0.030
STAGE_TUBE_OUTER_Y = 0.022
STAGE_TUBE_WALL = 0.003
STAGE_TUBE_H = 0.34
STAGE_TUBE_BOTTOM = -0.28
STAGE_BRIDGE_Z = 0.054
STAGE0_TRAVEL = 0.22

INNER_BAR_X = 0.018
INNER_BAR_Y = 0.012
INNER_BAR_H = 0.42
INNER_BAR_CENTER_Z = -0.12
INNER_GRIP_Z = 0.098
STAGE1_TRAVEL = 0.16

WHEEL_RADIUS = 0.11
WHEEL_WIDTH = 0.050
WHEEL_X = BODY_W * 0.5 + WHEEL_WIDTH * 0.5 + 0.006
WHEEL_Y = -0.155
WHEEL_Z = WHEEL_RADIUS

LATCH_X = 0.215
LATCH_Y = LID_D - 0.020
LATCH_Z = 0.072


def _rect_tube_mesh(
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    name: str,
):
    outer = cq.Workplane("XY").box(outer_x, outer_y, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(outer_x - 2.0 * wall, outer_y - 2.0 * wall, height + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -0.002))
    )
    return mesh_from_cadquery(outer.cut(inner), name)


def _build_body_shell():
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BASE_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_Z))
        .edges("|Z")
        .fillet(CORNER_R)
        .faces(">Z")
        .shell(-WALL)
    )

    front_skid = (
        cq.Workplane("XY")
        .box(0.48, 0.032, BASE_Z + 0.006, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5 - 0.038, 0.0))
    )
    left_foot = (
        cq.Workplane("XY")
        .box(0.10, 0.070, BASE_Z + 0.006, centered=(True, True, False))
        .translate((-0.22, BODY_D * 0.5 - 0.070, 0.0))
    )
    right_foot = (
        cq.Workplane("XY")
        .box(0.10, 0.070, BASE_Z + 0.006, centered=(True, True, False))
        .translate((0.22, BODY_D * 0.5 - 0.070, 0.0))
    )
    upper_rail = (
        cq.Workplane("XY")
        .box(BODY_W - 0.12, 0.020, 0.020, centered=(True, True, False))
        .translate((0.0, BODY_D * 0.5 - 0.010, BODY_TOP - 0.020))
    )
    left_boss = (
        cq.Workplane("XY")
        .box(0.022, 0.080, 0.090, centered=(True, True, False))
        .translate((-(BODY_W * 0.5 - 0.005), WHEEL_Y, 0.048))
    )
    right_boss = (
        cq.Workplane("XY")
        .box(0.022, 0.080, 0.090, centered=(True, True, False))
        .translate(((BODY_W * 0.5 - 0.005), WHEEL_Y, 0.048))
    )
    return shell.union(front_skid).union(left_foot).union(right_foot).union(upper_rail).union(left_boss).union(right_boss)


def _build_lid_shell():
    lid = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(CORNER_R * 0.85)
        .faces("<Z")
        .shell(-WALL)
    )
    lid = lid.faces(">Z").workplane().rect(LID_W - 0.19, LID_D - 0.16).cutBlind(-0.014)
    lid = lid.faces(">Z").workplane().rect(LID_W - 0.35, LID_D - 0.28).cutBlind(-0.008)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_ready_toolbox")

    shell_black = model.material("shell_black", rgba=(0.13, 0.13, 0.14, 1.0))
    lid_yellow = model.material("lid_yellow", rgba=(0.90, 0.74, 0.12, 1.0))
    handle_aluminum = model.material("handle_aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    latch_black = model.material("latch_black", rgba=(0.11, 0.11, 0.12, 1.0))
    catch_steel = model.material("catch_steel", rgba=(0.42, 0.43, 0.45, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "toolbox_body_shell"),
        material=shell_black,
        name="case_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "toolbox_lid_shell"),
        material=lid_yellow,
        name="lid_shell",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        channel = model.part(f"rear_channel_{index}")
        channel.visual(
            _rect_tube_mesh(
                CHANNEL_OUTER_X,
                CHANNEL_OUTER_Y,
                CHANNEL_H,
                CHANNEL_WALL,
                f"toolbox_rear_channel_{index}",
            ),
            material=shell_black,
            name="channel_sleeve",
        )
        model.articulation(
            f"body_to_rear_channel_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=channel,
            origin=Origin(xyz=(x_sign * CHANNEL_X, CHANNEL_Y, CHANNEL_BOTTOM)),
        )

    handle_stage_0 = model.part("handle_stage_0")
    stage_tube_mesh = _rect_tube_mesh(
        STAGE_TUBE_OUTER_X,
        STAGE_TUBE_OUTER_Y,
        STAGE_TUBE_H,
        STAGE_TUBE_WALL,
        "toolbox_handle_stage_0_tube",
    )
    handle_stage_0.visual(
        stage_tube_mesh,
        origin=Origin(xyz=(-CHANNEL_X, 0.0, STAGE_TUBE_BOTTOM)),
        material=handle_aluminum,
        name="left_tube",
    )
    handle_stage_0.visual(
        stage_tube_mesh,
        origin=Origin(xyz=(CHANNEL_X, 0.0, STAGE_TUBE_BOTTOM)),
        material=handle_aluminum,
        name="right_tube",
    )
    handle_stage_0.visual(
        Box((CHANNEL_X * 2.0 + 0.040, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, STAGE_BRIDGE_Z)),
        material=handle_aluminum,
        name="bridge",
    )

    handle_stage_1 = model.part("handle_stage_1")
    handle_stage_1.visual(
        Box((INNER_BAR_X, INNER_BAR_Y, INNER_BAR_H)),
        origin=Origin(xyz=(-CHANNEL_X, 0.0, INNER_BAR_CENTER_Z)),
        material=handle_aluminum,
        name="left_bar",
    )
    handle_stage_1.visual(
        Box((INNER_BAR_X, INNER_BAR_Y, INNER_BAR_H)),
        origin=Origin(xyz=(CHANNEL_X, 0.0, INNER_BAR_CENTER_Z)),
        material=handle_aluminum,
        name="right_bar",
    )
    handle_stage_1.visual(
        Box((CHANNEL_X * 2.0 + 0.090, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, INNER_GRIP_Z)),
        material=handle_aluminum,
        name="grip_bar",
    )
    handle_stage_1.visual(
        Box((0.40, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, INNER_GRIP_Z + 0.001)),
        material=dark_rubber,
        name="grip_wrap",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.078, length=WHEEL_WIDTH * 0.70),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_dark,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.042, length=WHEEL_WIDTH * 0.92),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=catch_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=WHEEL_WIDTH * 1.04),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_dark,
            name="axle_cap",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x_sign * WHEEL_X, WHEEL_Y, WHEEL_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=20.0),
        )

    for index, x_sign in enumerate((-1.0, 1.0)):
        catch = model.part(f"catch_{index}")
        catch.visual(
            Box((0.060, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, 0.005, 0.008)),
            material=catch_steel,
            name="catch_block",
        )
        catch.visual(
            Box((0.040, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.008, 0.021)),
            material=catch_steel,
            name="catch_top",
        )
        model.articulation(
            f"body_to_catch_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=catch,
            origin=Origin(xyz=(x_sign * LATCH_X, BODY_D * 0.5, BODY_TOP - 0.025)),
        )

    for index, x_sign in enumerate((-1.0, 1.0)):
        latch = model.part(f"latch_{index}")
        latch.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=Origin(xyz=(0.0, 0.026, -0.002), rpy=(0.0, pi / 2.0, 0.0)),
            material=latch_black,
            name="barrel",
        )
        latch.visual(
            Box((0.056, 0.010, 0.058)),
            origin=Origin(xyz=(0.0, 0.024, -0.030)),
            material=latch_black,
            name="tab_plate",
        )
        latch.visual(
            Box((0.052, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.028, -0.054)),
            material=latch_black,
            name="finger_lip",
        )
        model.articulation(
            f"lid_to_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x_sign * LATCH_X, LATCH_Y, LATCH_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.15),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -(LID_D * 0.5), BODY_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.22),
    )
    model.articulation(
        "body_to_handle_stage_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle_stage_0,
        origin=Origin(xyz=(0.0, CHANNEL_Y, CHANNEL_BOTTOM + CHANNEL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=STAGE0_TRAVEL),
    )
    model.articulation(
        "handle_stage_0_to_handle_stage_1",
        ArticulationType.PRISMATIC,
        parent=handle_stage_0,
        child=handle_stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=STAGE1_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle_stage_0 = object_model.get_part("handle_stage_0")
    handle_stage_1 = object_model.get_part("handle_stage_1")
    rear_channel_0 = object_model.get_part("rear_channel_0")
    catch_0 = object_model.get_part("catch_0")
    latch_0 = object_model.get_part("latch_0")
    wheel_0 = object_model.get_part("wheel_0")

    lid_joint = object_model.get_articulation("body_to_lid")
    stage_0_joint = object_model.get_articulation("body_to_handle_stage_0")
    stage_1_joint = object_model.get_articulation("handle_stage_0_to_handle_stage_1")
    latch_0_joint = object_model.get_articulation("lid_to_latch_0")
    latch_1_joint = object_model.get_articulation("lid_to_latch_1")
    wheel_0_joint = object_model.get_articulation("body_to_wheel_0")
    wheel_1_joint = object_model.get_articulation("body_to_wheel_1")

    ctx.allow_isolated_part(
        handle_stage_1,
        reason="The inner telescoping handle stage is intentionally modeled with running clearance inside the outer sleeve stage.",
    )

    ctx.check(
        "primary mechanisms use expected articulation types",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and stage_0_joint.articulation_type == ArticulationType.PRISMATIC
        and stage_1_joint.articulation_type == ArticulationType.PRISMATIC
        and latch_0_joint.articulation_type == ArticulationType.REVOLUTE
        and latch_1_joint.articulation_type == ArticulationType.REVOLUTE
        and wheel_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_1_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"lid={lid_joint.articulation_type}, "
            f"stage0={stage_0_joint.articulation_type}, "
            f"stage1={stage_1_joint.articulation_type}, "
            f"latch0={latch_0_joint.articulation_type}, "
            f"wheel0={wheel_0_joint.articulation_type}"
        ),
    )

    with ctx.pose({lid_joint: 0.0, latch_0_joint: 0.0, latch_1_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="case_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid seats cleanly on the case rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="case_shell",
            min_overlap=0.30,
            name="lid covers the case opening",
        )
        ctx.expect_overlap(
            latch_0,
            catch_0,
            axes="x",
            elem_a="tab_plate",
            elem_b="catch_block",
            min_overlap=0.030,
            name="front latch lines up with the catch",
        )

    with ctx.pose({stage_0_joint: 0.0, stage_1_joint: 0.0}):
        ctx.expect_within(
            handle_stage_0,
            rear_channel_0,
            axes="xy",
            inner_elem="left_tube",
            outer_elem="channel_sleeve",
            margin=0.006,
            name="outer handle tube stays centered in the guide channel",
        )
        ctx.expect_overlap(
            handle_stage_0,
            rear_channel_0,
            axes="z",
            elem_a="left_tube",
            elem_b="channel_sleeve",
            min_overlap=0.12,
            name="collapsed outer handle stage remains inserted",
        )
        ctx.expect_within(
            handle_stage_1,
            handle_stage_0,
            axes="xy",
            inner_elem="left_bar",
            outer_elem="left_tube",
            margin=0.006,
            name="inner handle bar stays nested in the outer stage",
        )
        ctx.expect_overlap(
            handle_stage_1,
            handle_stage_0,
            axes="z",
            elem_a="left_bar",
            elem_b="left_tube",
            min_overlap=0.18,
            name="collapsed inner handle stage remains inserted",
        )
        collapsed_handle_pos = ctx.part_world_position(handle_stage_1)

    with ctx.pose({stage_0_joint: STAGE0_TRAVEL, stage_1_joint: STAGE1_TRAVEL}):
        ctx.expect_within(
            handle_stage_0,
            rear_channel_0,
            axes="xy",
            inner_elem="left_tube",
            outer_elem="channel_sleeve",
            margin=0.006,
            name="extended outer handle tube stays centered in the guide channel",
        )
        ctx.expect_overlap(
            handle_stage_0,
            rear_channel_0,
            axes="z",
            elem_a="left_tube",
            elem_b="channel_sleeve",
            min_overlap=0.05,
            name="extended outer handle stage retains insertion",
        )
        ctx.expect_within(
            handle_stage_1,
            handle_stage_0,
            axes="xy",
            inner_elem="left_bar",
            outer_elem="left_tube",
            margin=0.006,
            name="extended inner handle bar stays centered in the outer stage",
        )
        ctx.expect_overlap(
            handle_stage_1,
            handle_stage_0,
            axes="z",
            elem_a="left_bar",
            elem_b="left_tube",
            min_overlap=0.10,
            name="extended inner handle stage retains insertion",
        )
        extended_handle_pos = ctx.part_world_position(handle_stage_1)

    ctx.check(
        "pull handle extends upward",
        collapsed_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > collapsed_handle_pos[2] + 0.30,
        details=f"collapsed={collapsed_handle_pos}, extended={extended_handle_pos}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({latch_0_joint: 0.0, latch_1_joint: 0.0, lid_joint: 0.0}):
        closed_latch_aabb = ctx.part_element_world_aabb(latch_0, elem="tab_plate")
    with ctx.pose({latch_0_joint: 1.0, latch_1_joint: 1.0, lid_joint: 0.0}):
        released_latch_aabb = ctx.part_element_world_aabb(latch_0, elem="tab_plate")
    with ctx.pose({latch_0_joint: 1.0, latch_1_joint: 1.0, lid_joint: 1.05}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            (lower[0] + upper[0]) * 0.5,
            (lower[1] + upper[1]) * 0.5,
            (lower[2] + upper[2]) * 0.5,
        )

    closed_lid_center = _aabb_center(closed_lid_aabb)
    opened_lid_center = _aabb_center(opened_lid_aabb)
    closed_latch_center = _aabb_center(closed_latch_aabb)
    released_latch_center = _aabb_center(released_latch_aabb)

    ctx.check(
        "lid opens upward when released",
        closed_lid_center is not None
        and opened_lid_center is not None
        and opened_lid_center[2] > closed_lid_center[2] + 0.09,
        details=f"closed={closed_lid_center}, opened={opened_lid_center}",
    )
    ctx.check(
        "latch tab swings forward to release",
        closed_latch_center is not None
        and released_latch_center is not None
        and released_latch_center[1] > closed_latch_center[1] + 0.010,
        details=f"closed={closed_latch_center}, released={released_latch_center}",
    )

    wheel_rest = ctx.part_world_position(wheel_0)
    with ctx.pose({wheel_0_joint: 1.7, wheel_1_joint: -1.2}):
        wheel_spun = ctx.part_world_position(wheel_0)
    ctx.check(
        "rear wheels spin on fixed hub centers",
        wheel_rest is not None
        and wheel_spun is not None
        and abs(wheel_spun[0] - wheel_rest[0]) < 1e-6
        and abs(wheel_spun[1] - wheel_rest[1]) < 1e-6
        and abs(wheel_spun[2] - wheel_rest[2]) < 1e-6,
        details=f"rest={wheel_rest}, spun={wheel_spun}",
    )

    return ctx.report()


object_model = build_object_model()
