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


METER_W = 0.245
METER_D = 0.205
METER_H = 0.118

HOPPER_H = 0.235
HOPPER_TOP_W = 0.228
HOPPER_TOP_D = 0.182
HOPPER_BOTTOM_W = 0.184
HOPPER_BOTTOM_D = 0.142
HOPPER_WALL = 0.0045

THROAT_W = 0.162
THROAT_D = 0.120
THROAT_H = 0.060

CHUTE_RECESS_W = 0.108
CHUTE_RECESS_D = 0.052
CHUTE_RECESS_H = 0.050
CHUTE_BOTTOM_Z = 0.022

WHEEL_R = 0.062
WHEEL_RING_THICKNESS = 0.012
WHEEL_T = 0.024
WHEEL_HUB_R = 0.022
WHEEL_CENTER_Z = 0.074
WHEEL_CENTER_Y = (METER_D / 2.0) + (WHEEL_T / 2.0)

LID_W = HOPPER_TOP_W + 0.010
LID_D = HOPPER_TOP_D + 0.012
LID_T = 0.005
LID_FRONT_LIP_H = 0.016
LID_SIDE_LIP_H = 0.014
LID_HINGE_R = 0.004
LID_HINGE_Y = -(HOPPER_TOP_D / 2.0) + 0.004
LID_HINGE_Z = METER_H + HOPPER_H + 0.006

FLAP_W = 0.092
FLAP_H = 0.042
FLAP_T = 0.004
FLAP_HINGE_Y = (METER_D / 2.0) - CHUTE_RECESS_D + 0.022
FLAP_HINGE_Z = CHUTE_BOTTOM_Z + 0.0022


def _build_hopper_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(HOPPER_BOTTOM_W, HOPPER_BOTTOM_D)
        .workplane(offset=HOPPER_H)
        .rect(HOPPER_TOP_W, HOPPER_TOP_D)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .rect(HOPPER_BOTTOM_W - (2.0 * HOPPER_WALL), HOPPER_BOTTOM_D - (2.0 * HOPPER_WALL))
        .workplane(offset=HOPPER_H + 0.002)
        .rect(HOPPER_TOP_W - (2.0 * HOPPER_WALL), HOPPER_TOP_D - (2.0 * HOPPER_WALL))
        .loft(combine=True)
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner).translate((0.0, 0.0, METER_H))


def _build_meter_box() -> cq.Workplane:
    box = cq.Workplane("XY").box(METER_W, METER_D, METER_H, centered=(True, True, False))

    feed_throat = (
        cq.Workplane("XY")
        .box(THROAT_W, THROAT_D, THROAT_H, centered=(True, True, False))
        .translate((0.0, 0.0, METER_H - THROAT_H))
    )

    chute_recess = (
        cq.Workplane("XY")
        .box(CHUTE_RECESS_W, CHUTE_RECESS_D + 0.002, CHUTE_RECESS_H, centered=(True, True, False))
        .translate(
            (
                0.0,
                (METER_D / 2.0) - ((CHUTE_RECESS_D + 0.002) / 2.0) + 0.001,
                CHUTE_BOTTOM_Z - 0.001,
            )
        )
    )

    wheel_boss = (
        cq.Workplane("XZ")
        .center(0.0, WHEEL_CENTER_Z)
        .circle(0.030)
        .extrude(0.007)
        .translate((0.0, (METER_D / 2.0) - 0.002, 0.0))
    )

    return box.cut(feed_throat).cut(chute_recess).union(wheel_boss)


def _build_lid() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T)
        .translate((0.0, (LID_D / 2.0) + 0.004, 0.0015))
    )
    front_pull = (
        cq.Workplane("XY")
        .box(0.090, 0.010, 0.006)
        .translate((0.0, LID_D + 0.001, 0.0035))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(LID_W - 0.028, 0.012, 0.006)
        .translate((0.0, 0.004, 0.0005))
    )
    left_tab = (
        cq.Workplane("XY")
        .box(0.014, 0.010, 0.008)
        .translate((0.090, -0.004, -0.002))
    )
    right_tab = (
        cq.Workplane("XY")
        .box(0.014, 0.010, 0.008)
        .translate((-0.090, -0.004, -0.002))
    )
    hinge_barrel = cq.Workplane("YZ").circle(LID_HINGE_R).extrude((LID_W - 0.024) / 2.0, both=True)

    return panel.union(front_pull).union(rear_bridge).union(left_tab).union(right_tab).union(hinge_barrel)


def _build_wheel() -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .circle(WHEEL_R)
        .circle(WHEEL_R - WHEEL_RING_THICKNESS)
        .extrude(WHEEL_T / 2.0, both=True)
    )
    hub = cq.Workplane("XZ").circle(WHEEL_HUB_R).extrude(WHEEL_T / 2.0, both=True)

    wheel = ring.union(hub)
    spoke_len = (WHEEL_R - WHEEL_RING_THICKNESS) - WHEEL_HUB_R
    spoke_center_x = WHEEL_HUB_R + (spoke_len / 2.0)
    for angle in range(0, 360, 72):
        spoke = (
            cq.Workplane("XZ")
            .center(spoke_center_x, 0.0)
            .rect(spoke_len, 0.012)
            .extrude((WHEEL_T * 0.75) / 2.0, both=True)
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
        )
        wheel = wheel.union(spoke)

    return wheel


def _build_wheel_cap() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.004, both=True)
        .translate((0.0, (WHEEL_T / 2.0) - 0.002, 0.0))
    )


def _build_flap() -> cq.Workplane:
    panel = cq.Workplane("XY").box(FLAP_W, FLAP_T, FLAP_H, centered=(True, True, False))
    hinge_bead = cq.Workplane("YZ").circle(0.0032).extrude((FLAP_W - 0.008) / 2.0, both=True)
    pull_lip = (
        cq.Workplane("XY")
        .box(FLAP_W - 0.020, 0.005, 0.004)
        .translate((0.0, 0.0025, FLAP_H - 0.002))
    )
    return panel.union(hinge_bead).union(pull_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bulk_candy_vendor")

    model.material("clear_acrylic", rgba=(0.84, 0.94, 0.98, 0.34))
    model.material("smoked_acrylic", rgba=(0.18, 0.19, 0.21, 0.92))
    model.material("meter_box_finish", rgba=(0.61, 0.63, 0.66, 1.0))
    model.material("satin_metal", rgba=(0.77, 0.79, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_meter_box(), "meter_box"),
        material="meter_box_finish",
        name="meter_box",
    )
    body.visual(
        mesh_from_cadquery(_build_hopper_shell(), "hopper_shell"),
        material="clear_acrylic",
        name="hopper_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid(), "lid_panel"),
        material="clear_acrylic",
        name="lid_panel",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_build_wheel(), "dispense_wheel"),
        material="smoked_acrylic",
        name="wheel_rim",
    )
    wheel.visual(
        mesh_from_cadquery(_build_wheel_cap(), "wheel_cap"),
        material="satin_metal",
        name="wheel_cap",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap(), "chute_flap"),
        material="smoked_acrylic",
        name="flap_panel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=4.0, velocity=1.6),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=7.0),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=1.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("wheel")
    flap = object_model.get_part("flap")

    lid_hinge = object_model.get_articulation("lid_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")

    lid_limits = lid_hinge.motion_limits
    flap_limits = flap_hinge.motion_limits

    ctx.expect_gap(
        wheel,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        name="dispense wheel sits just proud of the metering box face",
    )
    ctx.expect_overlap(
        wheel,
        body,
        axes="xz",
        min_overlap=0.090,
        name="dispense wheel stays centered on the front of the box",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            min_overlap=0.200,
            name="lid covers the hopper width in the closed pose",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    open_lid_aabb = None
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid lifts upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10
        and open_lid_aabb[1][1] < closed_lid_aabb[1][1] - 0.05,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            body,
            axes="x",
            min_overlap=0.070,
            name="chute flap spans the chute opening width",
        )
        closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    open_flap_aabb = None
    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({flap_hinge: flap_limits.upper}):
            open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "chute flap swings forward and downward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.020
        and open_flap_aabb[1][2] < closed_flap_aabb[1][2] - 0.015,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
