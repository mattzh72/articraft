from __future__ import annotations

import math

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


PAN_HEIGHT = 0.0
COLLAR_Z = -0.82
HINGE_RADIUS = 0.085
FRAME_Y = 0.153
FRAME_FRONT_Y = 0.162
LEAF_HINGE_Y = 0.174
FRAME_OUTER_HALF = 0.135
FRAME_INNER_HALF = 0.105


def _can_body_mesh():
    body_length = 0.26
    outer_radius = 0.11
    inner_radius = 0.101
    rear_cap = 0.02
    front_collar_length = 0.024

    outer = cq.Workplane("XZ").circle(outer_radius).extrude(body_length / 2.0, both=True)
    cavity = (
        cq.Workplane("XZ")
        .workplane(offset=-body_length / 2.0 + rear_cap)
        .circle(inner_radius)
        .extrude(body_length - rear_cap + 0.001)
    )
    shell = outer.cut(cavity)

    front_collar = (
        cq.Workplane("XZ")
        .workplane(offset=body_length / 2.0)
        .circle(0.123)
        .extrude(front_collar_length)
        .cut(
            cq.Workplane("XZ")
            .workplane(offset=body_length / 2.0 - 0.001)
            .circle(0.094)
            .extrude(front_collar_length + 0.002)
        )
    )

    rear_housing = (
        cq.Workplane("XZ")
        .workplane(offset=-body_length / 2.0 - 0.02)
        .circle(0.058)
        .extrude(0.024)
    )

    top_cap = (
        cq.Workplane("XY")
        .box(0.084, 0.058, 0.024)
        .translate((0.0, -0.012, 0.082))
    )

    return shell.union(front_collar).union(rear_housing).union(top_cap)


def _add_tripod_leg(
    model: ArticulatedObject,
    stand,
    *,
    index: int,
    azimuth: float,
    metal,
    rubber,
):
    leg = model.part(f"leg_{index}")

    leg_length = 0.84
    leg_angle = math.radians(31.0)
    radial_run = leg_length * math.sin(leg_angle)
    vertical_drop = leg_length * math.cos(leg_angle)
    tube_pitch = math.pi - math.atan2(radial_run, vertical_drop)

    leg.visual(
        Cylinder(radius=0.014, length=leg_length),
        origin=Origin(
            xyz=(radial_run / 2.0, 0.0, -vertical_drop / 2.0),
            rpy=(0.0, tube_pitch, 0.0),
        ),
        material=metal,
        name="tube",
    )
    leg.visual(
        Box((0.03, 0.026, 0.018)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    leg.visual(
        Box((0.09, 0.034, 0.018)),
        origin=Origin(xyz=(radial_run + 0.02, 0.0, -vertical_drop - 0.009)),
        material=rubber,
        name="shoe",
    )

    model.articulation(
        f"stand_to_leg_{index}",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=leg,
        origin=Origin(
            xyz=(HINGE_RADIUS * math.cos(azimuth), HINGE_RADIUS * math.sin(azimuth), COLLAR_Z),
            rpy=(0.0, 0.0, azimuth),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.6,
            lower=-0.48,
            upper=0.28,
        ),
    )


def _add_leaf(
    model: ArticulatedObject,
    can,
    *,
    name: str,
    joint_name: str,
    origin_xyz: tuple[float, float, float],
    axis: tuple[float, float, float],
    panel_size: tuple[float, float, float],
    panel_offset: tuple[float, float, float],
    barrel_origin: Origin,
    finish,
):
    leaf = model.part(name)
    leaf.visual(
        Box(panel_size),
        origin=Origin(xyz=panel_offset),
        material=finish,
        name="panel",
    )
    leaf.visual(
        Cylinder(radius=0.005, length=0.15 if panel_size[0] > panel_size[2] else 0.19),
        origin=barrel_origin,
        material=finish,
        name="barrel",
    )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=can,
        child=leaf,
        origin=Origin(xyz=origin_xyz),
        axis=axis,
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=2.1,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_tripod")

    stand_gray = model.material("stand_gray", rgba=(0.26, 0.27, 0.29, 1.0))
    head_gray = model.material("head_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    can_black = model.material("can_black", rgba=(0.06, 0.06, 0.07, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))
    indicator = model.material("indicator", rgba=(0.88, 0.88, 0.86, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.05, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=head_gray,
        name="pan_head",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
        material=stand_gray,
        name="center_column",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_Z)),
        material=head_gray,
        name="leg_collar",
    )
    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        stand.visual(
            Box((0.016, 0.03, 0.024)),
            origin=Origin(
                xyz=(0.06 * math.cos(azimuth), 0.06 * math.sin(azimuth), COLLAR_Z),
                rpy=(0.0, 0.0, azimuth),
            ),
            material=head_gray,
            name=f"leg_lug_{index}",
        )

    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        _add_tripod_leg(
            model,
            stand,
            index=index,
            azimuth=azimuth,
            metal=stand_gray,
            rubber=rubber,
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.099)),
        material=head_gray,
        name="pan_collar",
    )
    yoke.visual(
        Box((0.084, 0.05, 0.034)),
        origin=Origin(xyz=(0.0, -0.055, -0.125)),
        material=head_gray,
        name="pan_block",
    )
    yoke.visual(
        Box((0.31, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.055, -0.155)),
        material=head_gray,
        name="bridge",
    )
    yoke.visual(
        Box((0.03, 0.11, 0.25)),
        origin=Origin(xyz=(-0.152, -0.02, -0.295)),
        material=head_gray,
        name="arm_0",
    )
    yoke.visual(
        Box((0.03, 0.11, 0.25)),
        origin=Origin(xyz=(0.152, -0.02, -0.295)),
        material=head_gray,
        name="arm_1",
    )
    yoke.visual(
        Box((0.304, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.055, -0.405)),
        material=head_gray,
        name="rear_tie",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-1.6,
            upper=1.6,
        ),
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_can_body_mesh(), "spotlight_can"),
        material=can_black,
        name="body_shell",
    )
    can.visual(
        Box((FRAME_OUTER_HALF * 2.0, 0.018, FRAME_OUTER_HALF - FRAME_INNER_HALF)),
        origin=Origin(xyz=(0.0, FRAME_Y, (FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0)),
        material=can_black,
        name="frame_top",
    )
    can.visual(
        Box((FRAME_OUTER_HALF * 2.0, 0.018, FRAME_OUTER_HALF - FRAME_INNER_HALF)),
        origin=Origin(xyz=(0.0, FRAME_Y, -(FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0)),
        material=can_black,
        name="frame_bottom",
    )
    can.visual(
        Box((FRAME_OUTER_HALF - FRAME_INNER_HALF, 0.018, FRAME_INNER_HALF * 2.0)),
        origin=Origin(xyz=((FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0, FRAME_Y, 0.0)),
        material=can_black,
        name="frame_right",
    )
    can.visual(
        Box((FRAME_OUTER_HALF - FRAME_INNER_HALF, 0.018, FRAME_INNER_HALF * 2.0)),
        origin=Origin(xyz=(-(FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0, FRAME_Y, 0.0)),
        material=can_black,
        name="frame_left",
    )
    can.visual(
        Cylinder(radius=0.024, length=0.02),
        origin=Origin(xyz=(0.119, -0.01, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    can.visual(
        Cylinder(radius=0.024, length=0.02),
        origin=Origin(xyz=(-0.119, -0.01, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    can.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.116, -0.07, -0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_gray,
        name="dimmer_boss",
    )
    can.visual(
        Box((0.05, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.142, (FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0 - 0.003)),
        material=can_black,
        name="top_frame_bridge",
    )
    can.visual(
        Box((0.05, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.142, -((FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0 - 0.003))),
        material=can_black,
        name="bottom_frame_bridge",
    )
    can.visual(
        Box((0.018, 0.026, 0.05)),
        origin=Origin(xyz=((FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0 - 0.003, 0.142, 0.0)),
        material=can_black,
        name="right_frame_bridge",
    )
    can.visual(
        Box((0.018, 0.026, 0.05)),
        origin=Origin(xyz=(-((FRAME_OUTER_HALF + FRAME_INNER_HALF) / 2.0 - 0.003), 0.142, 0.0)),
        material=can_black,
        name="left_frame_bridge",
    )
    can.visual(
        Box((0.18, 0.01, 0.012)),
        origin=Origin(xyz=(0.0, 0.1665, FRAME_OUTER_HALF)),
        material=can_black,
        name="top_hinge_mount",
    )
    can.visual(
        Box((0.18, 0.01, 0.012)),
        origin=Origin(xyz=(0.0, 0.1665, -FRAME_OUTER_HALF)),
        material=can_black,
        name="bottom_hinge_mount",
    )
    can.visual(
        Box((0.012, 0.01, 0.18)),
        origin=Origin(xyz=(-FRAME_OUTER_HALF, 0.1665, 0.0)),
        material=can_black,
        name="left_hinge_mount",
    )
    can.visual(
        Box((0.012, 0.01, 0.18)),
        origin=Origin(xyz=(FRAME_OUTER_HALF, 0.1665, 0.0)),
        material=can_black,
        name="right_hinge_mount",
    )

    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.176, -0.27)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-1.0,
            upper=1.0,
        ),
    )

    dial = model.part("dimmer_dial")
    dial.visual(
        Cylinder(radius=0.033, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob",
    )
    dial.visual(
        Box((0.008, 0.01, 0.008)),
        origin=Origin(xyz=(0.018, 0.0, 0.028)),
        material=indicator,
        name="indicator",
    )

    model.articulation(
        "can_to_dimmer_dial",
        ArticulationType.CONTINUOUS,
        parent=can,
        child=dial,
        origin=Origin(xyz=(0.124, -0.07, -0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    leaf_finish = can_black
    _add_leaf(
        model,
        can,
        name="top_leaf",
        joint_name="can_to_top_leaf",
        origin_xyz=(0.0, LEAF_HINGE_Y, FRAME_OUTER_HALF),
        axis=(-1.0, 0.0, 0.0),
        panel_size=(0.215, 0.004, 0.105),
        panel_offset=(0.0, 0.0, 0.0525),
        barrel_origin=Origin(
            xyz=(0.0, 0.0, 0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        finish=leaf_finish,
    )
    _add_leaf(
        model,
        can,
        name="bottom_leaf",
        joint_name="can_to_bottom_leaf",
        origin_xyz=(0.0, LEAF_HINGE_Y, -FRAME_OUTER_HALF),
        axis=(1.0, 0.0, 0.0),
        panel_size=(0.215, 0.004, 0.105),
        panel_offset=(0.0, 0.0, -0.0525),
        barrel_origin=Origin(
            xyz=(0.0, 0.0, -0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        finish=leaf_finish,
    )
    _add_leaf(
        model,
        can,
        name="left_leaf",
        joint_name="can_to_left_leaf",
        origin_xyz=(-FRAME_OUTER_HALF, LEAF_HINGE_Y, 0.0),
        axis=(0.0, 0.0, -1.0),
        panel_size=(0.105, 0.004, 0.215),
        panel_offset=(-0.0525, 0.0, 0.0),
        barrel_origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        finish=leaf_finish,
    )
    _add_leaf(
        model,
        can,
        name="right_leaf",
        joint_name="can_to_right_leaf",
        origin_xyz=(FRAME_OUTER_HALF, LEAF_HINGE_Y, 0.0),
        axis=(0.0, 0.0, 1.0),
        panel_size=(0.105, 0.004, 0.215),
        panel_offset=(0.0525, 0.0, 0.0),
        barrel_origin=Origin(xyz=(0.005, 0.0, 0.0)),
        finish=leaf_finish,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    leg_0 = object_model.get_part("leg_0")
    can = object_model.get_part("can")
    top_leaf = object_model.get_part("top_leaf")
    right_leaf = object_model.get_part("right_leaf")

    leg_joint = object_model.get_articulation("stand_to_leg_0")
    pan_joint = object_model.get_articulation("stand_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_can")
    top_leaf_joint = object_model.get_articulation("can_to_top_leaf")
    right_leaf_joint = object_model.get_articulation("can_to_right_leaf")
    dial_joint = object_model.get_articulation("can_to_dimmer_dial")

    ctx.expect_gap(
        top_leaf,
        can,
        axis="y",
        positive_elem="panel",
        negative_elem="frame_top",
        min_gap=0.0005,
        max_gap=0.012,
        name="top leaf rests just ahead of the frame",
    )
    ctx.expect_gap(
        right_leaf,
        can,
        axis="y",
        positive_elem="panel",
        negative_elem="frame_right",
        min_gap=0.0005,
        max_gap=0.012,
        name="right leaf rests just ahead of the frame",
    )
    ctx.allow_overlap(
        "stand",
        "yoke",
        elem_a="center_column",
        elem_b="pan_collar",
        reason="The pan bearing is represented as a solid collar wrapped around the stand spigot rather than as a hollow sleeve.",
    )
    ctx.expect_overlap(
        top_leaf,
        can,
        axes="x",
        elem_a="panel",
        elem_b="frame_top",
        min_overlap=0.18,
        name="top leaf spans the frame width",
    )

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    rest_leg_center = center_from_aabb(ctx.part_element_world_aabb(leg_0, elem="shoe"))
    with ctx.pose({leg_joint: 0.25}):
        swept_leg_center = center_from_aabb(ctx.part_element_world_aabb(leg_0, elem="shoe"))
    ctx.check(
        "tripod leg swings on its crown hinge",
        rest_leg_center is not None
        and swept_leg_center is not None
        and swept_leg_center[0] > rest_leg_center[0] + 0.12
        and swept_leg_center[2] > rest_leg_center[2] + 0.10,
        details=f"rest={rest_leg_center}, swept={swept_leg_center}",
    )

    rest_front = center_from_aabb(ctx.part_element_world_aabb(can, elem="frame_top"))
    with ctx.pose({pan_joint: 1.0}):
        panned_front = center_from_aabb(ctx.part_element_world_aabb(can, elem="frame_top"))
    ctx.check(
        "yoke pan swings the can sideways",
        rest_front is not None
        and panned_front is not None
        and abs(panned_front[0] - rest_front[0]) > 0.10,
        details=f"rest={rest_front}, panned={panned_front}",
    )

    with ctx.pose({tilt_joint: 0.55}):
        tilted_front = center_from_aabb(ctx.part_element_world_aabb(can, elem="frame_top"))
    ctx.check(
        "can tilt lifts the front frame",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.05,
        details=f"rest={rest_front}, tilted={tilted_front}",
    )

    rest_top_leaf = center_from_aabb(ctx.part_element_world_aabb(top_leaf, elem="panel"))
    with ctx.pose({top_leaf_joint: 1.1}):
        opened_top_leaf = center_from_aabb(ctx.part_element_world_aabb(top_leaf, elem="panel"))
    ctx.check(
        "top barndoor leaf opens forward",
        rest_top_leaf is not None
        and opened_top_leaf is not None
        and opened_top_leaf[1] > rest_top_leaf[1] + 0.04,
        details=f"rest={rest_top_leaf}, opened={opened_top_leaf}",
    )

    rest_right_leaf = center_from_aabb(ctx.part_element_world_aabb(right_leaf, elem="panel"))
    with ctx.pose({right_leaf_joint: 1.0}):
        opened_right_leaf = center_from_aabb(ctx.part_element_world_aabb(right_leaf, elem="panel"))
    ctx.check(
        "side barndoor leaf opens forward",
        rest_right_leaf is not None
        and opened_right_leaf is not None
        and opened_right_leaf[1] > rest_right_leaf[1] + 0.04,
        details=f"rest={rest_right_leaf}, opened={opened_right_leaf}",
    )

    rest_indicator = center_from_aabb(ctx.part_element_world_aabb("dimmer_dial", elem="indicator"))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        rotated_indicator = center_from_aabb(ctx.part_element_world_aabb("dimmer_dial", elem="indicator"))
    ctx.check(
        "dimmer dial indicator rotates around its side shaft",
        rest_indicator is not None
        and rotated_indicator is not None
        and abs(rotated_indicator[1] - rest_indicator[1]) > 0.02
        and abs(rotated_indicator[2] - rest_indicator[2]) > 0.02,
        details=f"rest={rest_indicator}, rotated={rotated_indicator}",
    )

    return ctx.report()


object_model = build_object_model()
