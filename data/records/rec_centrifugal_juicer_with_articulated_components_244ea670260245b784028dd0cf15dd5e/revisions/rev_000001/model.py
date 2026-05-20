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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.24
BASE_HEIGHT = 0.138
HINGE_X = -0.142
HINGE_Z = 0.150
BASKET_X = 0.012
DRAWER_Y = 0.060
DRAWER_Z = 0.010
CHAMBER_X = 0.158
CHUTE_X = 0.170
CHUTE_TOP_Z = 0.214


def _base_shell_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .ellipse(BASE_LENGTH * 0.50, BASE_WIDTH * 0.50)
        .workplane(offset=0.094)
        .ellipse(0.158, 0.112)
        .workplane(offset=0.044)
        .ellipse(0.136, 0.094)
        .loft(combine=True)
    )

    drawer_pocket = cq.Workplane("XY").box(0.160, 0.104, 0.042).translate((0.140, DRAWER_Y, 0.024))

    return body.cut(drawer_pocket)


def _lid_chamber_shape() -> cq.Workplane:
    wall_outer = (
        cq.Workplane("XY")
        .ellipse(0.155, 0.110)
        .workplane(offset=0.062)
        .ellipse(0.149, 0.105)
        .workplane(offset=0.040)
        .ellipse(0.116, 0.084)
        .loft(combine=True)
        .translate((CHAMBER_X, 0.0, 0.0))
    )
    wall_inner = (
        cq.Workplane("XY")
        .ellipse(0.151, 0.106)
        .workplane(offset=0.068)
        .ellipse(0.145, 0.101)
        .workplane(offset=0.044)
        .ellipse(0.108, 0.078)
        .loft(combine=True)
        .translate((CHAMBER_X, 0.0, -0.010))
    )
    top_ring = (
        cq.Workplane("XY")
        .ellipse(0.124, 0.088)
        .ellipse(0.054, 0.036)
        .extrude(0.008)
        .translate((CHAMBER_X + 0.006, 0.0, 0.100))
    )
    return wall_outer.cut(wall_inner).union(top_ring)


def _chute_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.092, 0.070, 0.138).translate((CHUTE_X, 0.0, 0.139))
    inner = cq.Workplane("XY").box(0.074, 0.052, 0.156).translate((CHUTE_X, 0.0, 0.139))
    return outer.cut(inner)


def _drawer_shape() -> cq.Workplane:
    tray = cq.Workplane("XY").box(0.102, 0.092, 0.026).translate((-0.051, 0.0, 0.013))
    cavity = cq.Workplane("XY").box(0.088, 0.076, 0.016).translate((-0.056, 0.0, 0.019))
    finger_scoop = cq.Workplane("YZ").circle(0.012).extrude(0.014).translate((0.006, 0.0, 0.016))
    return tray.cut(cavity).cut(finger_scoop)


def _basket_shape() -> cq.Workplane:
    filter_wall = cq.Workplane("XY").circle(0.060).circle(0.050).extrude(0.058)
    conical_floor = (
        cq.Workplane("XY")
        .circle(0.050)
        .workplane(offset=0.028)
        .circle(0.018)
        .loft(combine=True)
    )
    hub = cq.Workplane("XY").circle(0.015).extrude(0.060)
    rim = cq.Workplane("XY").circle(0.066).circle(0.058).extrude(0.006).translate((0.0, 0.0, 0.052))
    spindle = cq.Workplane("XY").circle(0.004).extrude(0.082)
    return filter_wall.union(conical_floor).union(hub).union(rim).union(spindle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_silver = model.material("body_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.70, 0.78, 0.84, 0.34))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_drawer = model.material("dark_drawer", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shell_shape(), "base_shell"),
        material=body_silver,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(BASKET_X, 0.0, BASE_HEIGHT + 0.008), rpy=(0.0, 0.0, 0.0)),
        material=body_silver,
        name="spindle_mount",
    )
    for index, y_pos in enumerate((-0.059, 0.059)):
        base.visual(
            Box((0.020, 0.012, 0.040)),
            origin=Origin(xyz=(HINGE_X + 0.006, y_pos, BASE_HEIGHT - 0.006)),
            material=body_silver,
            name=f"hinge_block_{index}",
        )
    base.visual(
        Box((0.062, 0.044, 0.022)),
        origin=Origin(xyz=(0.178, DRAWER_Y, 0.084)),
        material=charcoal,
        name="juice_outlet",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_chamber_shape(), "chamber_shell"),
        material=smoke_clear,
        name="chamber_shell",
    )
    lid.visual(
        mesh_from_cadquery(_chute_shape(), "chute_shell"),
        material=smoke_clear,
        name="chute_shell",
    )
    for index, y_pos in enumerate((-0.038, 0.038)):
        lid.visual(
            Cylinder(radius=0.009, length=0.032),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"hinge_barrel_{index}",
        )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.070, 0.047, 0.112)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=black_plastic,
        name="pusher_body",
    )
    pusher.visual(
        Box((0.082, 0.058, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=black_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.052, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=black_plastic,
        name="pusher_grip",
    )

    selector_lever = model.part("selector_lever")
    selector_lever.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="lever_pivot",
    )
    selector_lever.visual(
        Box((0.032, 0.010, 0.012)),
        origin=Origin(xyz=(0.028, 0.0, 0.009)),
        material=charcoal,
        name="lever_arm",
    )
    selector_lever.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.044, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="lever_tip",
    )

    drip_drawer = model.part("drip_drawer")
    drip_drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "drip_drawer"),
        material=dark_drawer,
        name="drawer_body",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "basket_shell"),
        material=steel,
        name="basket_shell",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    pusher_slide = model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, 0.0, CHUTE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "selector_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=selector_lever,
        origin=Origin(xyz=(0.1446, -0.058, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.38,
            upper=0.38,
        ),
    )
    drawer_slide = model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drip_drawer,
        origin=Origin(xyz=(0.160, DRAWER_Y, DRAWER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(BASKET_X, 0.0, BASE_HEIGHT + 0.017)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=40.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    selector_lever = object_model.get_part("selector_lever")
    drip_drawer = object_model.get_part("drip_drawer")

    lid_hinge = object_model.get_articulation("lid_hinge")
    pusher_slide = object_model.get_articulation("pusher_slide")
    selector_pivot = object_model.get_articulation("selector_pivot")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.allow_overlap(
        base,
        drip_drawer,
        elem_a="base_shell",
        elem_b="drawer_body",
        reason="The recessed drip drawer rides in hidden internal guides while the lower body remains a simplified solid proxy around the recess.",
    )
    ctx.allow_overlap(
        base,
        selector_lever,
        elem_a="base_shell",
        elem_b="lever_pivot",
        reason="The selector lever rotates on a short pivot axle that is intentionally partially housed inside the front control boss.",
    )

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.18,
            name="lid covers the upper base in the closed pose",
        )
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="chute_shell",
            margin=0.002,
            name="pusher body stays guided inside the chute footprint",
        )
        rest_pusher_pos = ctx.part_world_position(pusher)
        closed_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.085}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="chute_shell",
            margin=0.002,
            name="pusher remains centered in the chute at full travel",
        )
        pressed_pusher_pos = ctx.part_world_position(pusher)

    ctx.check(
        "pusher travels downward into the feed chute",
        rest_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < rest_pusher_pos[2] - 0.04,
        details=f"rest={rest_pusher_pos}, pressed={pressed_pusher_pos}",
    )

    with ctx.pose({lid_hinge: 1.05}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({drawer_slide: 0.0}):
        drawer_rest_pos = ctx.part_world_position(drip_drawer)
        drawer_rest_aabb = ctx.part_world_aabb(drip_drawer)
        outlet_aabb = ctx.part_element_world_aabb(base, elem="juice_outlet")

    with ctx.pose({drawer_slide: 0.055}):
        drawer_open_pos = ctx.part_world_position(drip_drawer)

    ctx.check(
        "drip drawer extends forward from the lower body",
        drawer_rest_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[0] > drawer_rest_pos[0] + 0.04,
        details=f"rest={drawer_rest_pos}, open={drawer_open_pos}",
    )
    ctx.check(
        "drawer sits below the juice outlet",
        drawer_rest_aabb is not None
        and outlet_aabb is not None
        and drawer_rest_aabb[1][2] < outlet_aabb[0][2] + 0.005
        and abs(
            ((drawer_rest_aabb[0][1] + drawer_rest_aabb[1][1]) * 0.5)
            - ((outlet_aabb[0][1] + outlet_aabb[1][1]) * 0.5)
        )
        < 0.020,
        details=f"drawer={drawer_rest_aabb}, outlet={outlet_aabb}",
    )

    with ctx.pose({selector_pivot: -0.30}):
        low_tip_aabb = ctx.part_element_world_aabb(selector_lever, elem="lever_tip")
    with ctx.pose({selector_pivot: 0.30}):
        high_tip_aabb = ctx.part_element_world_aabb(selector_lever, elem="lever_tip")

    ctx.check(
        "selector lever swings upward across its short front pivot",
        low_tip_aabb is not None
        and high_tip_aabb is not None
        and high_tip_aabb[1][2] > low_tip_aabb[1][2] + 0.010,
        details=f"low={low_tip_aabb}, high={high_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
