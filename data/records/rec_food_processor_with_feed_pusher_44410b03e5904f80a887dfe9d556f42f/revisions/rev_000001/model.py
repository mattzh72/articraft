from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.190
BASE_DEPTH = 0.130
BASE_HEIGHT = 0.055
BASE_FRONT_Y = -BASE_DEPTH * 0.5

BOWL_CENTER_Y = 0.016
BOWL_BASE_Z = 0.068

FEED_TUBE_X = 0.022
FEED_TUBE_Y = -0.014
FEED_TUBE_TOP_Z = 0.148


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    rear_hump = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.048, BASE_HEIGHT - 0.001))
        .box(0.110, 0.045, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
    )
    front_skirt = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -0.048, 0.0))
        .box(0.164, 0.022, 0.010, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )
    return body.union(rear_hump).union(front_skirt)


def _bowl_shell_shape() -> cq.Workplane:
    outer_body = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.010)).circle(0.064).extrude(0.074)
    lid = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.084)).circle(0.068).extrude(0.006)
    lock_ring = cq.Workplane("XY").circle(0.036).circle(0.0305).extrude(0.010)

    shell = outer_body.union(lid).union(lock_ring)
    shell = shell.cut(cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.014)).circle(0.058).extrude(0.070))
    shell = shell.cut(cq.Workplane("XY").circle(0.010).extrude(0.022))
    shell = shell.cut(
        cq.Workplane("XY")
        .transformed(offset=(FEED_TUBE_X, FEED_TUBE_Y, 0.082))
        .circle(0.0182)
        .extrude(0.010)
    )
    return shell


def _feed_tube_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .transformed(offset=(FEED_TUBE_X, FEED_TUBE_Y, 0.084))
        .circle(0.026)
        .circle(0.018)
        .extrude(0.006)
    )
    tube = cq.Workplane("XY").transformed(offset=(FEED_TUBE_X, FEED_TUBE_Y, 0.090)).circle(0.022).extrude(0.058)
    shell = flange.union(tube)
    shell = shell.cut(
        cq.Workplane("XY")
        .transformed(offset=(FEED_TUBE_X, FEED_TUBE_Y, 0.084))
        .circle(0.0172)
        .extrude(0.064)
    )
    return shell


def _handle_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").transformed(offset=(0.074, 0.0, 0.052)).box(0.024, 0.022, 0.072)
    inner = cq.Workplane("XY").transformed(offset=(0.077, 0.0, 0.052)).box(0.010, 0.028, 0.050)
    return outer.cut(inner)


def _blade_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.012).circle(0.0050).extrude(0.010).translate((0.0, 0.0, -0.005))
    lower_blade = (
        cq.Workplane("XY")
        .box(0.072, 0.010, 0.002)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 12.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 18.0)
    )
    upper_blade = (
        cq.Workplane("XY")
        .box(0.060, 0.009, 0.002)
        .translate((0.0, 0.0, 0.004))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -10.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 198.0)
    )
    blade = hub.union(lower_blade).union(upper_blade)
    return blade.cut(cq.Workplane("XY").circle(0.0047).extrude(0.024).translate((0.0, 0.0, -0.012)))


def _pusher_shape() -> cq.Workplane:
    slug = cq.Workplane("XY").transformed(offset=(0.0, 0.0, -0.064)).circle(0.0157).extrude(0.064)
    grip = cq.Workplane("XY").circle(0.0166).extrude(0.012)
    rib_x_pos = cq.Workplane("XY").transformed(offset=(0.0162, 0.0, -0.046)).box(0.0020, 0.0032, 0.060)
    rib_x_neg = cq.Workplane("XY").transformed(offset=(-0.0162, 0.0, -0.046)).box(0.0020, 0.0032, 0.060)
    rib_y_pos = cq.Workplane("XY").transformed(offset=(0.0, 0.0162, -0.046)).box(0.0032, 0.0020, 0.060)
    rib_y_neg = cq.Workplane("XY").transformed(offset=(0.0, -0.0162, -0.046)).box(0.0032, 0.0020, 0.060)
    return slug.union(grip).union(rib_x_pos).union(rib_x_neg).union(rib_y_pos).union(rib_y_neg)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_food_processor")

    plastic_white = model.material("plastic_white", rgba=(0.93, 0.93, 0.92, 1.0))
    plastic_grey = model.material("plastic_grey", rgba=(0.72, 0.72, 0.70, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.11, 0.12, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.78, 0.83, 0.88, 0.34))
    steel = model.material("steel", rgba=(0.80, 0.81, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_shell"),
        material=plastic_white,
        name="base_shell",
    )
    base.visual(
        Box((0.094, 0.003, 0.052)),
        origin=Origin(xyz=(0.0, BASE_FRONT_Y - 0.0015, 0.028)),
        material=plastic_grey,
        name="front_panel",
    )
    base.visual(
        Box((0.026, 0.004, 0.018)),
        origin=Origin(xyz=(-0.020, BASE_FRONT_Y - 0.0020, 0.015)),
        material=plastic_grey,
        name="rocker_bezel_0",
    )
    base.visual(
        Box((0.026, 0.004, 0.018)),
        origin=Origin(xyz=(0.020, BASE_FRONT_Y - 0.0020, 0.015)),
        material=plastic_grey,
        name="rocker_bezel_1",
    )
    base.visual(
        Cylinder(radius=0.029, length=0.008),
        origin=Origin(xyz=(0.0, BOWL_CENTER_Y, BOWL_BASE_Z + 0.004)),
        material=plastic_grey,
        name="lock_collar",
    )
    base.visual(
        Cylinder(radius=0.0047, length=0.026),
        origin=Origin(xyz=(0.0, BOWL_CENTER_Y, BOWL_BASE_Z + 0.013)),
        material=steel,
        name="spindle",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shell_shape(), "bowl_shell"),
        material=clear_smoke,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(_feed_tube_shape(), "feed_tube_shell"),
        material=clear_smoke,
        name="feed_tube_shell",
    )
    bowl.visual(
        mesh_from_cadquery(_handle_shape(), "handle_shell"),
        material=plastic_dark,
        name="handle_shell",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, BOWL_CENTER_Y, BOWL_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0),
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "blade_shell"),
        material=steel,
        name="blade_shell",
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade,
        origin=Origin(xyz=(0.0, BOWL_CENTER_Y, BOWL_BASE_Z + 0.031)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_pusher_shape(), "pusher_shell"),
        material=plastic_white,
        name="pusher_shell",
    )
    model.articulation(
        "bowl_to_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=pusher,
        origin=Origin(xyz=(FEED_TUBE_X, FEED_TUBE_Y, FEED_TUBE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.040),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.018,
                body_style="skirted",
                top_diameter=0.022,
                center=False,
            ),
            "selector_knob_shell",
        ),
        material=plastic_black,
        name="knob_shell",
    )
    selector_knob.visual(
        Box((0.003, 0.008, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0095, 0.0180)),
        material=plastic_white,
        name="pointer",
    )
    model.articulation(
        "base_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_knob,
        origin=Origin(xyz=(0.0, BASE_FRONT_Y - 0.0010, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    for index, x_pos in enumerate((-0.020, 0.020)):
        rocker = model.part(f"rocker_{index}")
        rocker.visual(
            Box((0.018, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=plastic_dark,
            name="rocker_body",
        )
        model.articulation(
            f"base_to_rocker_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=rocker,
            origin=Origin(xyz=(x_pos, BASE_FRONT_Y - 0.0015, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=3.0,
                lower=-0.26,
                upper=0.26,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    blade = object_model.get_part("blade")
    pusher = object_model.get_part("pusher")
    selector_knob = object_model.get_part("selector_knob")
    rocker_0 = object_model.get_part("rocker_0")
    rocker_1 = object_model.get_part("rocker_1")

    bowl_lock = object_model.get_articulation("base_to_bowl")
    blade_spin = object_model.get_articulation("base_to_blade")
    pusher_slide = object_model.get_articulation("bowl_to_pusher")
    knob_spin = object_model.get_articulation("base_to_selector_knob")
    rocker_joint_0 = object_model.get_articulation("base_to_rocker_0")
    rocker_joint_1 = object_model.get_articulation("base_to_rocker_1")

    ctx.allow_overlap(
        bowl,
        pusher,
        elem_a="feed_tube_shell",
        elem_b="pusher_shell",
        reason="The pusher is intentionally represented as sliding inside the feed-tube sleeve proxy.",
    )
    ctx.allow_isolated_part(
        blade,
        reason="The blade is intentionally modeled with a slight running clearance around the center spindle so it can rotate freely.",
    )

    ctx.expect_within(
        pusher,
        bowl,
        axes="xy",
        inner_elem="pusher_shell",
        outer_elem="feed_tube_shell",
        margin=0.0012,
        name="pusher stays centered in feed tube at rest",
    )
    ctx.expect_overlap(
        pusher,
        bowl,
        axes="z",
        elem_a="pusher_shell",
        elem_b="feed_tube_shell",
        min_overlap=0.045,
        name="pusher remains inserted at rest",
    )
    blade_origin = ctx.part_world_position(blade)
    bowl_origin = ctx.part_world_position(bowl)
    ctx.check(
        "blade sits above the bowl floor region",
        blade_origin is not None and bowl_origin is not None and blade_origin[2] > bowl_origin[2] + 0.018,
        details=f"blade_origin={blade_origin}, bowl_origin={bowl_origin}",
    )
    ctx.expect_origin_distance(
        blade,
        bowl,
        axes="xy",
        max_dist=0.0005,
        name="blade stays centered on the spindle axis",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.040}):
        ctx.expect_within(
            pusher,
            bowl,
            axes="xy",
            inner_elem="pusher_shell",
            outer_elem="feed_tube_shell",
            margin=0.0012,
            name="pusher stays centered when pressed",
        )
        ctx.expect_overlap(
            pusher,
            bowl,
            axes="z",
            elem_a="pusher_shell",
            elem_b="feed_tube_shell",
            min_overlap=0.020,
            name="pusher keeps retained insertion at full stroke",
        )
        pusher_pressed = ctx.part_world_position(pusher)
    ctx.check(
        "pusher moves downward",
        pusher_rest is not None and pusher_pressed is not None and pusher_pressed[2] < pusher_rest[2] - 0.020,
        details=f"rest={pusher_rest}, pressed={pusher_pressed}",
    )

    handle_rest = _aabb_center(ctx.part_element_world_aabb(bowl, elem="handle_shell"))
    with ctx.pose({bowl_lock: 0.45}):
        handle_twisted = _aabb_center(ctx.part_element_world_aabb(bowl, elem="handle_shell"))
    ctx.check(
        "bowl twists on lock collar",
        handle_rest is not None
        and handle_twisted is not None
        and abs(handle_twisted[0] - handle_rest[0]) > 0.006
        and abs(handle_twisted[1] - handle_rest[1]) > 0.020,
        details=f"rest={handle_rest}, twisted={handle_twisted}",
    )

    blade_rest_aabb = ctx.part_element_world_aabb(blade, elem="blade_shell")
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_turned_aabb = ctx.part_element_world_aabb(blade, elem="blade_shell")
    ctx.check(
        "blade rotates about the spindle",
        blade_rest_aabb is not None
        and blade_turned_aabb is not None
        and abs(float(blade_turned_aabb[1][0]) - float(blade_rest_aabb[1][0])) > 0.003,
        details=f"rest={blade_rest_aabb}, turned={blade_turned_aabb}",
    )

    pointer_rest = _aabb_center(ctx.part_element_world_aabb(selector_knob, elem="pointer"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        pointer_turned = _aabb_center(ctx.part_element_world_aabb(selector_knob, elem="pointer"))
    ctx.check(
        "selector knob rotates on the front face",
        pointer_rest is not None
        and pointer_turned is not None
        and abs(pointer_turned[0] - pointer_rest[0]) > 0.006
        and abs(pointer_turned[2] - pointer_rest[2]) > 0.006,
        details=f"rest={pointer_rest}, turned={pointer_turned}",
    )

    rocker_0_rest = _aabb_center(ctx.part_element_world_aabb(rocker_0, elem="rocker_body"))
    rocker_1_rest = _aabb_center(ctx.part_element_world_aabb(rocker_1, elem="rocker_body"))
    with ctx.pose({rocker_joint_0: 0.22}):
        rocker_0_tilted = _aabb_center(ctx.part_element_world_aabb(rocker_0, elem="rocker_body"))
        rocker_1_static = _aabb_center(ctx.part_element_world_aabb(rocker_1, elem="rocker_body"))
    with ctx.pose({rocker_joint_1: -0.22}):
        rocker_1_tilted = _aabb_center(ctx.part_element_world_aabb(rocker_1, elem="rocker_body"))
    ctx.check(
        "rocker switch 0 pivots independently",
        rocker_0_rest is not None
        and rocker_0_tilted is not None
        and abs(rocker_0_tilted[2] - rocker_0_rest[2]) > 0.0005
        and rocker_1_rest is not None
        and rocker_1_static is not None
        and abs(rocker_1_static[2] - rocker_1_rest[2]) < 1e-6,
        details=f"rest0={rocker_0_rest}, tilted0={rocker_0_tilted}, rest1={rocker_1_rest}, static1={rocker_1_static}",
    )
    ctx.check(
        "rocker switch 1 pivots independently",
        rocker_1_rest is not None
        and rocker_1_tilted is not None
        and abs(rocker_1_tilted[2] - rocker_1_rest[2]) > 0.0005,
        details=f"rest1={rocker_1_rest}, tilted1={rocker_1_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
