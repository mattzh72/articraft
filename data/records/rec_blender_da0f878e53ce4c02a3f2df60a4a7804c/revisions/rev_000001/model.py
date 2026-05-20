from __future__ import annotations

import math

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


BASE_WIDTH = 0.135
BASE_DEPTH = 0.110
BASE_HEIGHT = 0.082

BOTTLE_BODY_RADIUS = 0.038
BOTTLE_COLLAR_RADIUS = 0.043
BOTTLE_HEIGHT = 0.222


def _centered_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _base_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    deck_recess = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(0.033)
        .extrude(-0.010)
    )
    housing = housing.cut(deck_recess)

    slot = _centered_box(
        (0.015, 0.006, 0.004),
        (0.0, 0.039, BASE_HEIGHT - 0.002),
    )
    for angle_deg in (0.0, 120.0, 240.0):
        housing = housing.cut(slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    switch_pocket = _centered_box(
        (0.036, 0.020, 0.026),
        (0.0, BASE_DEPTH / 2.0 - 0.008, 0.030),
    )
    housing = housing.cut(switch_pocket)

    return housing


def _bottle_shape() -> cq.Workplane:
    lower_body = cq.Workplane("XY").circle(BOTTLE_BODY_RADIUS).extrude(0.176)
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=0.176)
        .circle(BOTTLE_BODY_RADIUS)
        .workplane(offset=0.022)
        .circle(0.031)
        .loft(combine=True)
    )
    neck = cq.Workplane("XY").workplane(offset=0.198).circle(0.031).extrude(0.012)
    lid = cq.Workplane("XY").workplane(offset=0.210).circle(0.033).extrude(0.006)
    mount_collar = cq.Workplane("XY").circle(BOTTLE_COLLAR_RADIUS).extrude(0.011)

    bottle = lower_body.union(shoulder).union(neck).union(lid).union(mount_collar)

    tab = _centered_box((0.014, 0.005, 0.003), (0.0, 0.0385, 0.006))
    for angle_deg in (0.0, 120.0, 240.0):
        bottle = bottle.union(tab.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    spout_collar = (
        cq.Workplane("XY")
        .workplane(offset=0.210)
        .center(0.0, 0.018)
        .circle(0.011)
        .extrude(0.012)
    )
    hinge_pedestal = _centered_box((0.026, 0.010, 0.006), (0.0, 0.006, 0.219))
    bottle = bottle.union(spout_collar).union(hinge_pedestal)

    inner_body = cq.Workplane("XY").workplane(offset=0.004).circle(0.034).extrude(0.168)
    inner_shoulder = (
        cq.Workplane("XY")
        .workplane(offset=0.172)
        .circle(0.034)
        .workplane(offset=0.022)
        .circle(0.026)
        .loft(combine=True)
    )
    inner_neck = cq.Workplane("XY").workplane(offset=0.194).circle(0.026).extrude(0.016)
    spout_hole = (
        cq.Workplane("XY")
        .workplane(offset=0.204)
        .center(0.0, 0.018)
        .circle(0.006)
        .extrude(0.020)
    )
    drive_bore = cq.Workplane("XY").circle(0.006).extrude(0.014)

    bottle = bottle.cut(inner_body).cut(inner_shoulder).cut(inner_neck).cut(spout_hole).cut(drive_bore)
    return bottle


def _switch_shape() -> cq.Workplane:
    rocker = _centered_box((0.032, 0.014, 0.022), (0.0, 0.003, 0.0))
    mounting_flange = _centered_box((0.040, 0.002, 0.026), (0.0, -0.002, 0.0))
    return rocker.edges("|X").fillet(0.003).union(mounting_flange)


def _spout_cap_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("YZ")
        .circle(0.003)
        .extrude(0.024)
        .translate((-0.012, 0.0, 0.0))
    )
    cover = _centered_box((0.030, 0.026, 0.008), (0.0, 0.015, 0.001))
    plug = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .center(0.0, 0.012)
        .circle(0.0045)
        .extrude(0.004)
    )
    return barrel.union(cover.edges("|X").fillet(0.002)).union(plug)


def _blade_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").workplane(offset=-0.004).circle(0.007).extrude(0.008)
    shaft = cq.Workplane("XY").workplane(offset=-0.024).circle(0.0045).extrude(0.026)
    coupling = cq.Workplane("XY").workplane(offset=-0.024).circle(0.0065).extrude(0.006)

    blade_x_pos = (
        _centered_box((0.030, 0.007, 0.0018), (0.017, 0.0, 0.001))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
    )
    blade_x_neg = (
        _centered_box((0.030, 0.007, 0.0018), (-0.017, 0.0, 0.001))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
    )
    blade_y_pos = (
        _centered_box((0.007, 0.030, 0.0018), (0.0, 0.017, 0.001))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -18.0)
    )
    blade_y_neg = (
        _centered_box((0.007, 0.030, 0.0018), (0.0, -0.017, 0.001))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 18.0)
    )

    return (
        hub.union(shaft)
        .union(coupling)
        .union(blade_x_pos)
        .union(blade_x_neg)
        .union(blade_y_pos)
        .union(blade_y_neg)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_single_serve_blender")

    body_mat = model.material("body_mat", rgba=(0.92, 0.93, 0.94, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.16, 0.17, 0.19, 1.0))
    bottle_mat = model.material("bottle_mat", rgba=(0.80, 0.90, 0.98, 0.35))
    cap_mat = model.material("cap_mat", rgba=(0.20, 0.22, 0.25, 1.0))
    switch_mat = model.material("switch_mat", rgba=(0.70, 0.10, 0.10, 1.0))
    blade_mat = model.material("blade_mat", rgba=(0.78, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_housing"),
        material=body_mat,
        name="housing",
    )

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_shape(), "bottle_shell"),
        material=bottle_mat,
        name="shell",
    )

    front_switch = model.part("front_switch")
    front_switch.visual(
        mesh_from_cadquery(_switch_shape(), "front_switch"),
        material=switch_mat,
        name="rocker",
    )

    spout_cap = model.part("spout_cap")
    spout_cap.visual(
        mesh_from_cadquery(_spout_cap_shape(), "spout_cap"),
        material=cap_mat,
        name="cover",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "blade"),
        material=blade_mat,
        name="rotor",
    )

    model.articulation(
        "base_to_bottle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bottle,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    model.articulation(
        "base_to_front_switch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=front_switch,
        origin=Origin(xyz=(0.0, BASE_DEPTH / 2.0 + 0.001, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    model.articulation(
        "bottle_to_spout_cap",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=spout_cap,
        origin=Origin(xyz=(0.0, 0.006, BOTTLE_HEIGHT + 0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=5.0,
            lower=0.0,
            upper=2.05,
        ),
    )

    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=45.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bottle = object_model.get_part("bottle")
    front_switch = object_model.get_part("front_switch")
    spout_cap = object_model.get_part("spout_cap")
    blade = object_model.get_part("blade")

    bottle_joint = object_model.get_articulation("base_to_bottle")
    switch_joint = object_model.get_articulation("base_to_front_switch")
    cap_joint = object_model.get_articulation("bottle_to_spout_cap")
    blade_joint = object_model.get_articulation("base_to_blade")

    ctx.expect_gap(
        bottle,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="bottle seats on blender deck",
    )
    ctx.expect_overlap(
        bottle,
        base,
        axes="xy",
        min_overlap=0.075,
        name="bottle stays centered over the compact base",
    )
    ctx.expect_overlap(
        blade,
        bottle,
        axes="xy",
        min_overlap=0.012,
        name="blade stays inside the bottle footprint",
    )
    ctx.expect_overlap(
        spout_cap,
        bottle,
        axes="xy",
        min_overlap=0.018,
        name="spout cap covers the drink opening region when closed",
    )
    ctx.expect_origin_gap(
        front_switch,
        base,
        axis="y",
        min_gap=0.050,
        max_gap=0.060,
        name="rocker switch is mounted on the front face",
    )

    bottle_limits = bottle_joint.motion_limits
    ctx.check(
        "bottle twist-lock has quarter-turn style travel",
        bottle_limits is not None
        and bottle_limits.lower == 0.0
        and bottle_limits.upper is not None
        and 0.75 <= bottle_limits.upper <= 1.20,
        details=f"limits={bottle_limits}",
    )
    ctx.check(
        "blade uses continuous spin articulation",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type}",
    )

    blade_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade spindle drops into the recessed drive socket",
        blade_aabb is not None and (BASE_HEIGHT - 0.012) <= blade_aabb[0][2] <= (BASE_HEIGHT - 0.008),
        details=f"blade_aabb={blade_aabb}",
    )

    bottle_rest = ctx.part_world_position(bottle)
    with ctx.pose({bottle_joint: bottle_limits.upper if bottle_limits is not None else 0.0}):
        bottle_twisted = ctx.part_world_position(bottle)
    ctx.check(
        "bottle twists in place on its mount",
        bottle_rest is not None
        and bottle_twisted is not None
        and max(abs(a - b) for a, b in zip(bottle_rest, bottle_twisted)) <= 1e-6,
        details=f"rest={bottle_rest}, twisted={bottle_twisted}",
    )

    with ctx.pose({cap_joint: 0.0}):
        cap_closed_aabb = ctx.part_world_aabb(spout_cap)
    with ctx.pose({cap_joint: 1.9}):
        cap_open_aabb = ctx.part_world_aabb(spout_cap)
    ctx.check(
        "spout cap flips upward",
        cap_closed_aabb is not None
        and cap_open_aabb is not None
        and cap_open_aabb[1][2] > cap_closed_aabb[1][2] + 0.018,
        details=f"closed={cap_closed_aabb}, open={cap_open_aabb}",
    )

    with ctx.pose({switch_joint: -0.25}):
        switch_low_aabb = ctx.part_world_aabb(front_switch)
    with ctx.pose({switch_joint: 0.25}):
        switch_high_aabb = ctx.part_world_aabb(front_switch)
    ctx.check(
        "front rocker visibly pivots",
        switch_low_aabb is not None
        and switch_high_aabb is not None
        and abs(switch_high_aabb[1][2] - switch_low_aabb[1][2]) > 0.001,
        details=f"low={switch_low_aabb}, high={switch_high_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
