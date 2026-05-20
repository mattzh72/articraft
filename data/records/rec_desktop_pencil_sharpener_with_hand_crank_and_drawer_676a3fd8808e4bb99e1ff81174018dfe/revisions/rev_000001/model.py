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


BASE_LOWER_H = 0.006
BASE_UPPER_H = 0.006
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H

BODY_X = 0.105
BODY_Y = 0.080
BODY_Z = 0.078

DRAWER_BOTTOM_Z = 0.020
DRAWER_TRAVEL = 0.032

ENTRY_Z = 0.067
ENTRY_BORE_R = 0.0065
ENTRY_RING_R = 0.014
ENTRY_RING_L = 0.008
ENTRY_BORE_L = 0.036

SHAFT_X = -0.004
SHAFT_Z = 0.056
SHAFT_BOSS_R = 0.014
SHAFT_BOSS_L = 0.012


def _body_shell_shape():
    shell = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_Z)
        .translate((0.0, 0.0, BASE_TOP_Z + BODY_Z * 0.5))
        .edges("|Z")
        .fillet(0.008)
    )

    drawer_cut_x = 0.071
    drawer_cut_y = 0.056
    drawer_cut_z = 0.032
    drawer_cut = (
        cq.Workplane("XY")
        .box(drawer_cut_x, drawer_cut_y, drawer_cut_z)
        .translate(
            (
                BODY_X * 0.5 - drawer_cut_x * 0.5,
                0.0,
                DRAWER_BOTTOM_Z + drawer_cut_z * 0.5,
            )
        )
    )

    cutter_chamber = (
        cq.Workplane("XY")
        .box(0.052, 0.034, 0.026)
        .translate((0.010, 0.0, 0.066))
    )
    chute_cut = (
        cq.Workplane("XY")
        .box(0.032, 0.024, 0.024)
        .translate((0.012, 0.0, 0.052))
    )
    entry_bore = (
        cq.Workplane("YZ")
        .circle(ENTRY_BORE_R)
        .extrude(ENTRY_BORE_L)
        .translate((BODY_X * 0.5 - ENTRY_BORE_L, 0.0, ENTRY_Z))
    )

    return shell.cut(drawer_cut).cut(cutter_chamber).cut(chute_cut).cut(entry_bore)


def _entry_ring_shape():
    return (
        cq.Workplane("YZ")
        .circle(ENTRY_RING_R)
        .extrude(ENTRY_RING_L)
        .faces(">X")
        .workplane()
        .circle(ENTRY_BORE_R)
        .cutBlind(-ENTRY_RING_L)
        .translate((BODY_X * 0.5, 0.0, ENTRY_Z))
    )


def _side_boss_shape():
    return (
        cq.Workplane("XZ")
        .circle(SHAFT_BOSS_R)
        .extrude(SHAFT_BOSS_L)
        .translate((SHAFT_X, BODY_Y * 0.5 + SHAFT_BOSS_L, SHAFT_Z))
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="suction_base_pencil_sharpener")

    body_red = model.material("body_red", rgba=(0.74, 0.16, 0.14, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    smoke = model.material("smoke", rgba=(0.26, 0.28, 0.31, 0.78))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "sharpener_shell"),
        material=body_red,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_entry_ring_shape(), "sharpener_entry_ring"),
        material=steel,
        name="entry_ring",
    )
    body.visual(
        mesh_from_cadquery(_side_boss_shape(), "sharpener_side_boss"),
        material=steel,
        name="side_boss",
    )
    body.visual(
        Cylinder(radius=0.033, length=BASE_UPPER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H * 0.5)),
        material=rubber,
        name="base_pedestal",
    )
    body.visual(
        Cylinder(radius=0.041, length=BASE_LOWER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H * 0.5)),
        material=rubber,
        name="suction_cup",
    )
    for rail_index, rail_y in enumerate((-0.028, 0.028)):
        body.visual(
            Box((0.020, 0.004, 0.028)),
            origin=Origin(xyz=(BODY_X * 0.5 + 0.010, rail_y, 0.033)),
            material=dark_plastic,
            name=f"drawer_rail_{rail_index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.048, 0.048, 0.026)),
        origin=Origin(xyz=(0.029, 0.0, 0.013)),
        material=smoke,
        name="tray",
    )
    drawer.visual(
        Box((0.005, 0.060, 0.034)),
        origin=Origin(xyz=(0.0025, 0.0, 0.017)),
        material=smoke,
        name="front_panel",
    )
    drawer.visual(
        Box((0.011, 0.036, 0.010)),
        origin=Origin(xyz=(0.0105, 0.0, 0.017)),
        material=dark_plastic,
        name="pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    crank.visual(
        Box((0.040, 0.010, 0.014)),
        origin=Origin(xyz=(0.020, 0.009, -0.012)),
        material=steel,
        name="arm",
    )
    crank.visual(
        Box((0.014, 0.010, 0.026)),
        origin=Origin(xyz=(0.043, 0.006, -0.024)),
        material=steel,
        name="drop",
    )
    crank.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.047, 0.004, -0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_mount",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="collar",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="grip",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="end_cap",
    )

    drawer_joint = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_X * 0.5, 0.0, DRAWER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    crank_joint = model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(SHAFT_X, BODY_Y * 0.5 + SHAFT_BOSS_L, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=12.0),
    )

    model.articulation(
        "crank_to_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=knob,
        origin=Origin(xyz=(0.047, 0.008, -0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    knob = object_model.get_part("knob")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")

    drawer_limits = drawer_joint.motion_limits
    drawer_upper = drawer_limits.upper if drawer_limits is not None else None

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_gap(
            drawer,
            body,
            axis="x",
            positive_elem="front_panel",
            negative_elem="shell",
            max_gap=0.0005,
            max_penetration=0.0,
            name="drawer front closes flush with the body shell",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="tray",
            outer_elem="shell",
            margin=0.0,
            name="closed drawer stays aligned with the body opening",
        )
        ctx.expect_gap(
            drawer,
            body,
            axis="x",
            positive_elem="tray",
            negative_elem="shell",
            min_gap=0.004,
            max_gap=0.0065,
            name="closed drawer bin begins just ahead of the body shell",
        )
        ctx.expect_gap(
            crank,
            body,
            axis="y",
            positive_elem="hub",
            negative_elem="side_boss",
            max_gap=0.0005,
            max_penetration=0.0,
            name="crank hub seats against the side boss",
        )
        ctx.expect_gap(
            knob,
            crank,
            axis="y",
            positive_elem="collar",
            negative_elem="knob_mount",
            max_gap=0.0005,
            max_penetration=1e-5,
            name="handle knob is mounted directly on the crank arm",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    rest_tip = ctx.part_element_world_aabb(crank, elem="knob_mount")
    rest_tip_center = _aabb_center(rest_tip)

    if drawer_upper is not None:
        with ctx.pose({drawer_joint: drawer_upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="tray",
                outer_elem="shell",
                margin=0.0,
                name="extended drawer stays aligned in front of the body",
            )
            ctx.expect_gap(
                drawer,
                body,
                axis="x",
                positive_elem="tray",
                negative_elem="shell",
                min_gap=0.034,
                name="extended drawer moves clearly forward of the body",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)

        ctx.check(
            "drawer slides forward out of the body",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.02,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    with ctx.pose({crank_joint: math.pi * 0.5}):
        quarter_tip = ctx.part_element_world_aabb(crank, elem="knob_mount")
        quarter_tip_center = _aabb_center(quarter_tip)

    crank_sweeps = (
        rest_tip_center is not None
        and quarter_tip_center is not None
        and abs(quarter_tip_center[0] - rest_tip_center[0]) > 0.012
        and abs(quarter_tip_center[2] - rest_tip_center[2]) > 0.012
    )
    ctx.check(
        "crank tip sweeps around the side shaft",
        crank_sweeps,
        details=f"rest={rest_tip_center}, quarter_turn={quarter_tip_center}",
    )

    return ctx.report()


object_model = build_object_model()
