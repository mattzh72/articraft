from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_X = 0.235
BASE_Y = 0.175
BASE_H = 0.090

PEDESTAL_R = 0.067
PEDESTAL_H = 0.018
DRIVE_HUB_R = 0.017
DRIVE_HUB_H = 0.022

BOWL_JOINT_Z = BASE_H + PEDESTAL_H
BOWL_H = 0.138
TUBE_X = 0.032
TUBE_Y = 0.045
TUBE_BASE_Z = 0.110
TUBE_H = 0.120
PUSHER_TRAVEL = 0.085


def _rotated_box(size: tuple[float, float, float], offset: tuple[float, float, float], angle_deg: float):
    return (
        cq.Workplane("XY")
        .box(*size)
        .translate(offset)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    )


def make_base_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )

    front_recess = (
        cq.Workplane("XY")
        .box(0.012, 0.118, 0.048, centered=(True, True, False))
        .translate((BASE_X / 2.0 - 0.004, 0.0, 0.014))
    )
    return shell.cut(front_recess)


def make_bowl_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.090).extrude(BOWL_H)
    inner = cq.Workplane("XY").circle(0.085).extrude(BOWL_H - 0.004).translate((0.0, 0.0, 0.004))
    throat = (
        cq.Workplane("XY")
        .box(0.060, 0.080, 0.070, centered=(True, True, False))
        .translate((TUBE_X, TUBE_Y, 0.080))
    )
    spindle_socket = (
        cq.Workplane("XY")
        .circle(0.021)
        .extrude(0.032)
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner).cut(throat).cut(spindle_socket)


def make_locking_ring() -> cq.Workplane:
    tab_a = _rotated_box((0.016, 0.012, 0.008), (0.082, 0.0, 0.000), 30.0)
    tab_b = _rotated_box((0.016, 0.012, 0.008), (0.082, 0.0, 0.000), 210.0)
    return tab_a.union(tab_b)


def make_feed_tube_shell() -> cq.Workplane:
    front = cq.Workplane("XY").box(0.060, 0.004, TUBE_H, centered=(True, True, False)).translate((0.0, 0.040, 0.0))
    rear = cq.Workplane("XY").box(0.060, 0.004, TUBE_H, centered=(True, True, False)).translate((0.0, -0.040, 0.0))
    left = cq.Workplane("XY").box(0.004, 0.076, TUBE_H, centered=(True, True, False)).translate((-0.028, 0.0, 0.0))
    right = cq.Workplane("XY").box(0.004, 0.076, TUBE_H, centered=(True, True, False)).translate((0.028, 0.0, 0.0))
    rim_front = cq.Workplane("XY").box(0.060, 0.008, 0.008, centered=(True, True, False)).translate((0.0, 0.038, TUBE_H - 0.008))
    rim_rear = cq.Workplane("XY").box(0.060, 0.008, 0.008, centered=(True, True, False)).translate((0.0, -0.038, TUBE_H - 0.008))
    return front.union(rear).union(left).union(right).union(rim_front).union(rim_rear)


def make_basket() -> cq.Workplane:
    base_disk = cq.Workplane("XY").circle(0.066).extrude(0.004)
    wall = cq.Workplane("XY").circle(0.068).circle(0.064).extrude(0.050).translate((0.0, 0.0, 0.004))
    hub = cq.Workplane("XY").circle(0.013).extrude(0.060)
    basket = base_disk.union(wall).union(hub)

    cutters = None
    for i in range(16):
        cutter = _rotated_box((0.012, 0.005, 0.026), (0.066, 0.0, 0.026), i * 22.5)
        cutters = cutter if cutters is None else cutters.union(cutter)
    return basket.cut(cutters)


def make_pusher() -> cq.Workplane:
    stem = (
        cq.Workplane("XY")
        .box(0.052, 0.076, 0.180, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, 0.0, -0.120))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.040, 0.020, 0.022, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, 0.008))
    )
    return stem.union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_food_processor")

    housing_white = model.material("housing_white", rgba=(0.90, 0.90, 0.88, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.73, 0.76, 0.79, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.78, 1.0))
    clear_bowl = model.material("clear_bowl", rgba=(0.78, 0.88, 0.94, 0.34))
    smoke_clear = model.material("smoke_clear", rgba=(0.60, 0.70, 0.76, 0.44))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shell(), "base_shell"),
        material=housing_white,
        name="housing_shell",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + PEDESTAL_H / 2.0)),
        material=panel_gray,
        name="coupler_deck",
    )
    base.visual(
        Cylinder(radius=DRIVE_HUB_R, length=DRIVE_HUB_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + DRIVE_HUB_H / 2.0)),
        material=charcoal,
        name="drive_hub",
    )
    base.visual(
        Box((0.004, 0.114, 0.050)),
        origin=Origin(xyz=(BASE_X / 2.0 - 0.001, 0.0, 0.038)),
        material=panel_gray,
        name="front_panel",
    )
    for idx, (x, y) in enumerate(
        (
            (0.080, 0.055),
            (0.080, -0.055),
            (-0.080, 0.055),
            (-0.080, -0.055),
        )
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"foot_{idx}",
        )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(make_bowl_shell(), "bowl_shell"),
        material=clear_bowl,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(make_locking_ring(), "locking_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=charcoal,
        name="locking_ring",
    )
    tube_wall_z = TUBE_BASE_Z + (TUBE_H + 0.004) / 2.0 - 0.004
    bowl.visual(
        Box((0.066, 0.006, TUBE_H + 0.004)),
        origin=Origin(xyz=(TUBE_X, TUBE_Y + 0.042, tube_wall_z)),
        material=clear_bowl,
        name="feed_tube_front",
    )
    bowl.visual(
        Box((0.066, 0.006, TUBE_H + 0.004)),
        origin=Origin(xyz=(TUBE_X, TUBE_Y - 0.042, tube_wall_z)),
        material=clear_bowl,
        name="feed_tube_rear",
    )
    bowl.visual(
        Box((0.006, 0.084, TUBE_H + 0.004)),
        origin=Origin(xyz=(TUBE_X - 0.030, TUBE_Y, tube_wall_z)),
        material=clear_bowl,
        name="feed_tube_left",
    )
    bowl.visual(
        Box((0.006, 0.084, TUBE_H + 0.004)),
        origin=Origin(xyz=(TUBE_X + 0.030, TUBE_Y, tube_wall_z)),
        material=clear_bowl,
        name="feed_tube_right",
    )
    bowl.visual(
        Box((0.020, 0.030, 0.108)),
        origin=Origin(xyz=(0.0, -0.112, 0.068)),
        material=charcoal,
        name="handle_grip",
    )
    bowl.visual(
        Box((0.028, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, -0.096, 0.036)),
        material=charcoal,
        name="handle_lower_mount",
    )
    bowl.visual(
        Box((0.030, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, -0.098, 0.114)),
        material=charcoal,
        name="handle_upper_mount",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(make_basket(), "grating_basket"),
        material=stainless,
        name="basket_shell",
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(make_pusher(), "pusher"),
        material=smoke_clear,
        name="pusher_body",
    )
    pusher.visual(
        Box((0.044, 0.001, 0.110)),
        origin=Origin(xyz=(0.0, 0.0385, -0.055)),
        material=smoke_clear,
        name="guide_front",
    )
    pusher.visual(
        Box((0.044, 0.001, 0.110)),
        origin=Origin(xyz=(0.0, -0.0385, -0.055)),
        material=smoke_clear,
        name="guide_rear",
    )
    pusher.visual(
        Box((0.001, 0.070, 0.110)),
        origin=Origin(xyz=(0.0265, 0.0, -0.055)),
        material=smoke_clear,
        name="guide_right",
    )
    pusher.visual(
        Box((0.001, 0.070, 0.110)),
        origin=Origin(xyz=(-0.0265, 0.0, -0.055)),
        material=smoke_clear,
        name="guide_left",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.026,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.050, 0.006, flare=0.10),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "timer_dial",
        ),
        material=charcoal,
        name="dial_knob",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.0035, length=0.032),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rocker_pivot",
    )
    power_rocker.visual(
        Box((0.028, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=charcoal,
        name="rocker_cap",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, BOWL_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )
    model.articulation(
        "bowl_to_basket",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=16.0),
    )
    model.articulation(
        "bowl_to_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=pusher,
        origin=Origin(xyz=(TUBE_X, TUBE_Y, TUBE_BASE_Z + TUBE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=PUSHER_TRAVEL),
    )
    model.articulation(
        "base_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_dial,
        origin=Origin(xyz=(BASE_X / 2.0 + 0.001, -0.018, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )
    model.articulation(
        "base_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=base,
        child=power_rocker,
        origin=Origin(xyz=(0.064, BASE_Y / 2.0, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=-math.radians(12.0),
            upper=math.radians(12.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    basket = object_model.get_part("basket")
    pusher = object_model.get_part("pusher")
    rocker = object_model.get_part("power_rocker")

    bowl_joint = object_model.get_articulation("base_to_bowl")
    basket_joint = object_model.get_articulation("bowl_to_basket")
    pusher_joint = object_model.get_articulation("bowl_to_pusher")
    rocker_joint = object_model.get_articulation("base_to_power_rocker")
    tube_half_x = 0.027
    tube_half_y = 0.039
    tube_bottom_world = BOWL_JOINT_Z + TUBE_BASE_Z
    tube_top_world = BOWL_JOINT_Z + TUBE_BASE_Z + TUBE_H

    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        min_overlap=0.10,
        elem_a="locking_ring",
        elem_b="coupler_deck",
        name="bowl locking ring stays centered over the base coupling",
    )
    ctx.expect_within(
        basket,
        bowl,
        axes="xy",
        margin=0.010,
        inner_elem="basket_shell",
        outer_elem="bowl_shell",
        name="grating basket remains within the bowl plan view",
    )
    ctx.expect_gap(
        bowl,
        basket,
        axis="z",
        positive_elem="feed_tube_front",
        negative_elem="basket_shell",
        min_gap=0.030,
        name="feed chute stays open above the spinning basket",
    )
    pusher_rest_aabb = ctx.part_world_aabb(pusher)
    pusher_rest_xy_ok = False
    pusher_rest_inserted = -1.0
    if pusher_rest_aabb is not None:
        (rest_min_x, rest_min_y, rest_min_z), (rest_max_x, rest_max_y, rest_max_z) = pusher_rest_aabb
        pusher_rest_xy_ok = (
            rest_min_x >= TUBE_X - tube_half_x - 0.001
            and rest_max_x <= TUBE_X + tube_half_x + 0.001
            and rest_min_y >= TUBE_Y - tube_half_y - 0.001
            and rest_max_y <= TUBE_Y + tube_half_y + 0.001
        )
        pusher_rest_inserted = min(rest_max_z, tube_top_world) - tube_bottom_world
    ctx.check(
        "pusher stays inside the feed tube footprint at rest",
        pusher_rest_xy_ok,
        details=f"aabb={pusher_rest_aabb}",
    )
    ctx.check(
        "resting pusher remains deeply inserted in the tube",
        pusher_rest_inserted >= 0.110,
        details=f"inserted_length={pusher_rest_inserted:.4f}",
    )

    with ctx.pose({bowl_joint: 0.30, basket_joint: 0.85}):
        ctx.expect_overlap(
            bowl,
            base,
            axes="xy",
            min_overlap=0.10,
            elem_a="locking_ring",
            elem_b="coupler_deck",
            name="bowl remains seated while twist-locking",
        )
        ctx.expect_within(
            basket,
            bowl,
            axes="xy",
            margin=0.010,
            inner_elem="basket_shell",
            outer_elem="bowl_shell",
            name="basket stays centered during bowl rotation",
        )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: PUSHER_TRAVEL}):
        pusher_extended_aabb = ctx.part_world_aabb(pusher)
        pusher_extended_xy_ok = False
        pusher_extended_inserted = -1.0
        if pusher_extended_aabb is not None:
            (ext_min_x, ext_min_y, ext_min_z), (ext_max_x, ext_max_y, ext_max_z) = pusher_extended_aabb
            pusher_extended_xy_ok = (
                ext_min_x >= TUBE_X - tube_half_x - 0.001
                and ext_max_x <= TUBE_X + tube_half_x + 0.001
                and ext_min_y >= TUBE_Y - tube_half_y - 0.001
                and ext_max_y <= TUBE_Y + tube_half_y + 0.001
            )
            pusher_extended_inserted = min(ext_max_z, tube_top_world + PUSHER_TRAVEL) - (tube_bottom_world + PUSHER_TRAVEL)
        ctx.check(
            "extended pusher remains guided by the tube",
            pusher_extended_xy_ok,
            details=f"aabb={pusher_extended_aabb}",
        )
        ctx.check(
            "extended pusher still retains insertion in the tube",
            pusher_extended_inserted >= 0.026,
            details=f"inserted_length={pusher_extended_inserted:.4f}",
        )
        pusher_extended = ctx.part_world_position(pusher)
    ctx.check(
        "pusher retracts upward",
        pusher_rest is not None
        and pusher_extended is not None
        and pusher_extended[2] > pusher_rest[2] + 0.06,
        details=f"rest={pusher_rest}, extended={pusher_extended}",
    )

    rocker_rest = ctx.part_world_aabb(rocker)
    with ctx.pose({rocker_joint: math.radians(12.0)}):
        rocker_on = ctx.part_world_aabb(rocker)
    ctx.check(
        "rocker pivots independently",
        rocker_rest is not None
        and rocker_on is not None
        and rocker_on[1][1] > rocker_rest[1][1] + 0.001,
        details=f"rest={rocker_rest}, posed={rocker_on}",
    )

    return ctx.report()


object_model = build_object_model()
