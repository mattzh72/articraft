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


BASE_LENGTH = 0.245
BASE_WIDTH = 0.195
BASE_HEIGHT = 0.125
CHAMBER_OUTER_RADIUS = 0.078
CHAMBER_INNER_RADIUS = 0.061
CHAMBER_RING_HEIGHT = 0.058
CHAMBER_BASE_Z = BASE_HEIGHT
LID_HINGE_X = -0.083
LID_HINGE_Z = CHAMBER_BASE_Z + CHAMBER_RING_HEIGHT
LID_OPEN_LIMIT = math.radians(70.0)
PUSHER_TRAVEL = 0.045
LATCH_OPEN_LIMIT = math.radians(58.0)
FLAP_OPEN_LIMIT = math.radians(82.0)


def _build_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.028)
        .faces(">Z")
        .edges()
        .fillet(0.012)
    )

def _build_chamber_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(CHAMBER_OUTER_RADIUS)
        .extrude(CHAMBER_RING_HEIGHT)
        .cut(
            cq.Workplane("XY")
            .circle(CHAMBER_INNER_RADIUS)
            .extrude(CHAMBER_RING_HEIGHT)
        )
        .translate((0.0, 0.0, CHAMBER_BASE_Z))
    )

def _build_spout_shape() -> cq.Workplane:
    spout_outer = (
        cq.Workplane("YZ", origin=(0.060, 0.0, 0.146))
        .rect(0.060, 0.038)
        .workplane(offset=0.076)
        .rect(0.046, 0.026)
        .loft(combine=True)
    )
    spout_inner = (
        cq.Workplane("YZ", origin=(0.064, 0.0, 0.146))
        .rect(0.044, 0.024)
        .workplane(offset=0.070)
        .rect(0.032, 0.016)
        .loft(combine=True)
    )

    return spout_outer.cut(spout_inner)


def _build_lid_shape() -> cq.Workplane:
    dome_outer = (
        cq.Workplane("XY")
        .ellipse(0.080, 0.072)
        .workplane(offset=0.044)
        .ellipse(0.066, 0.058)
        .loft(combine=True)
        .translate((0.083, 0.0, 0.0))
    )
    dome_inner = (
        cq.Workplane("XY")
        .ellipse(0.075, 0.067)
        .workplane(offset=0.040)
        .ellipse(0.061, 0.053)
        .loft(combine=True)
        .translate((0.083, 0.0, 0.004))
    )

    chute_outer = (
        cq.Workplane("XY")
        .box(0.086, 0.066, 0.098)
        .translate((0.118, 0.0, 0.082))
        .edges("|Z")
        .fillet(0.008)
    )
    chute_inner = (
        cq.Workplane("XY")
        .box(0.060, 0.040, 0.124)
        .translate((0.118, 0.0, 0.078))
    )

    return dome_outer.union(chute_outer).cut(dome_inner).cut(chute_inner)


def _build_basket_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(0.056)
        .extrude(0.055)
        .cut(cq.Workplane("XY").circle(0.051).extrude(0.052).translate((0.0, 0.0, 0.003)))
    )
    hub = cq.Workplane("XY").circle(0.015).extrude(0.020)
    floor = cq.Workplane("XY").circle(0.035).extrude(0.004)
    return shell.union(hub).union(floor)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.96, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.78, 0.84, 0.88, 0.42))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.81, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "juicer_base"),
        material=body_white,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_build_chamber_ring_shape(), "juicer_chamber_ring"),
        material=body_white,
        name="chamber_rim",
    )
    base.visual(
        mesh_from_cadquery(_build_spout_shape(), "juicer_spout"),
        material=body_white,
        name="spout_shell",
    )
    for index, y_pos in enumerate((-0.015, 0.015)):
        base.visual(
            Box((0.004, 0.008, 0.012)),
            origin=Origin(xyz=(0.132, y_pos, 0.156)),
            material=body_white,
            name=f"flap_tab_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "juicer_lid"),
        material=smoke_clear,
        name="lid_shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket_shape(), "juicer_basket"),
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Box((0.018, 0.006, 0.016)),
        origin=Origin(xyz=(0.049, 0.0, 0.026)),
        material=steel,
        name="blade_fin",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.058, 0.038, 0.083)),
        origin=Origin(xyz=(0.0, 0.0, -0.0285)),
        material=charcoal,
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.078, 0.056, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0130)),
        material=charcoal,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0295), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pusher_grip",
    )

    side_latch = model.part("side_latch")
    side_latch.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="latch_barrel",
    )
    side_latch.visual(
        Box((0.016, 0.014, 0.145)),
        origin=Origin(xyz=(0.0, 0.014, 0.0725)),
        material=dark_trim,
        name="latch_arm",
    )
    side_latch.visual(
        Box((0.030, 0.016, 0.018)),
        origin=Origin(xyz=(0.018, 0.004, 0.136)),
        material=dark_trim,
        name="latch_hook",
    )

    drip_flap = model.part("drip_flap")
    drip_flap.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="flap_barrel",
    )
    drip_flap.visual(
        Box((0.030, 0.004, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, -0.010)),
        material=dark_trim,
        name="flap_panel",
    )
    drip_flap.visual(
        Box((0.010, 0.008, 0.008)),
        origin=Origin(xyz=(0.026, 0.0, -0.018)),
        material=dark_trim,
        name="flap_tip",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LID_OPEN_LIMIT,
            effort=8.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.118, 0.0, 0.127)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PUSHER_TRAVEL,
            effort=30.0,
            velocity=0.15,
        ),
    )
    model.articulation(
        "base_to_side_latch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_latch,
        origin=Origin(xyz=(-0.005, 0.103, 0.060)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LATCH_OPEN_LIMIT,
            effort=2.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "base_to_drip_flap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=drip_flap,
        origin=Origin(xyz=(0.1395, 0.0, 0.156)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FLAP_OPEN_LIMIT,
            effort=1.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    pusher = object_model.get_part("pusher")
    side_latch = object_model.get_part("side_latch")
    drip_flap = object_model.get_part("drip_flap")
    lid_joint = object_model.get_articulation("base_to_lid")
    basket_joint = object_model.get_articulation("base_to_basket")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    latch_joint = object_model.get_articulation("base_to_side_latch")
    flap_joint = object_model.get_articulation("base_to_drip_flap")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.12,
        name="closed lid covers the juicing chamber",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        name="closed lid sits just above the chamber rim",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: LID_OPEN_LIMIT}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.06,
        details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
    )

    raised_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: PUSHER_TRAVEL}):
        pressed_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides downward through the chute",
        raised_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < raised_pusher_pos[2] - 0.04,
        details=f"raised={raised_pusher_pos!r}, pressed={pressed_pusher_pos!r}",
    )

    closed_latch_aabb = ctx.part_world_aabb(side_latch)
    with ctx.pose({latch_joint: LATCH_OPEN_LIMIT}):
        open_latch_aabb = ctx.part_world_aabb(side_latch)
    ctx.check(
        "side latch swings outward from the body",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][1] > closed_latch_aabb[1][1] + 0.03,
        details=f"closed={closed_latch_aabb!r}, open={open_latch_aabb!r}",
    )

    resting_flap_aabb = ctx.part_element_world_aabb(drip_flap, elem="flap_panel")
    with ctx.pose({flap_joint: FLAP_OPEN_LIMIT}):
        raised_flap_aabb = ctx.part_element_world_aabb(drip_flap, elem="flap_panel")
    ctx.check(
        "anti drip flap flips upward at the spout",
        resting_flap_aabb is not None
        and raised_flap_aabb is not None
        and raised_flap_aabb[1][2] > resting_flap_aabb[1][2] + 0.012,
        details=f"rest={resting_flap_aabb!r}, raised={raised_flap_aabb!r}",
    )

    basket_blade_rest = aabb_center(ctx.part_element_world_aabb(basket, elem="blade_fin"))
    with ctx.pose({basket_joint: math.pi / 2.0}):
        basket_blade_rotated = aabb_center(ctx.part_element_world_aabb(basket, elem="blade_fin"))
    ctx.check(
        "basket spins about the vertical axis",
        basket_blade_rest is not None
        and basket_blade_rotated is not None
        and abs(basket_blade_rotated[1] - basket_blade_rest[1]) > 0.03,
        details=f"rest={basket_blade_rest!r}, rotated={basket_blade_rotated!r}",
    )

    return ctx.report()


object_model = build_object_model()
