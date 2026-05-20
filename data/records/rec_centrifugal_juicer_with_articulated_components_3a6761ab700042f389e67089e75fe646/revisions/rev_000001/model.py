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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_LENGTH = 0.36
BASE_WIDTH = 0.24
BASE_HEIGHT = 0.118
PEDESTAL_RADIUS = 0.108
PEDESTAL_HEIGHT = 0.032
DRIVE_X = 0.025

HINGE_X = -0.087
HINGE_Z = 0.184
BASKET_Z = BASE_HEIGHT + PEDESTAL_HEIGHT

LID_OPEN = math.radians(68.0)
PUSHER_TRAVEL = 0.052
DRAWER_TRAVEL = 0.14
DIAL_SWEEP = math.radians(110.0)


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.028)
        .edges(">Z")
        .fillet(0.012)
    )
    pedestal = (
        cq.Workplane("XY")
        .center(DRIVE_X, 0.0)
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.192, 0.158, 0.078, centered=(True, True, False))
        .translate((-0.096, 0.0, 0.018))
    )
    return body.union(pedestal).cut(drawer_cavity)


def _lid_shape() -> cq.Workplane:
    chamber_outer = (
        cq.Workplane("XY").center(0.115, 0.0).circle(0.112).extrude(0.050)
    )
    chamber_inner = (
        cq.Workplane("XY")
        .center(0.115, 0.0)
        .circle(0.098)
        .extrude(0.052)
        .translate((0.0, 0.0, -0.001))
    )
    chamber = chamber_outer.cut(chamber_inner).translate((0.0, 0.0, -0.034))

    cover = (
        cq.Workplane("XY").center(0.115, 0.0).circle(0.112).extrude(0.010).translate((0.0, 0.0, 0.008))
    )
    cover_opening = (
        cq.Workplane("XY").center(0.125, 0.0).circle(0.036).extrude(0.014).translate((0.0, 0.0, 0.007))
    )
    cover = cover.cut(cover_opening)

    chute_outer = (
        cq.Workplane("XY").center(0.125, 0.0).circle(0.045).extrude(0.147).translate((0.0, 0.0, 0.018))
    )
    chute_inner = (
        cq.Workplane("XY").center(0.125, 0.0).circle(0.036).extrude(0.149).translate((0.0, 0.0, 0.017))
    )
    chute = chute_outer.cut(chute_inner)

    hinge_arm_left = (
        cq.Workplane("XY")
        .box(0.076, 0.026, 0.018, centered=(False, True, False))
        .translate((0.0, -0.082, 0.010))
    )
    hinge_arm_right = (
        cq.Workplane("XY")
        .box(0.076, 0.026, 0.018, centered=(False, True, False))
        .translate((0.0, 0.082, 0.010))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(0.018, 0.108, 0.016, centered=(True, True, False))
        .translate((0.217, 0.0, -0.027))
    )
    return hinge_arm_left.union(hinge_arm_right).union(chamber).union(cover).union(chute).union(front_lip)


def _drawer_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.170, 0.146, 0.066, centered=(False, True, False))
    inner = (
        cq.Workplane("XY")
        .box(0.154, 0.130, 0.058, centered=(False, True, False))
        .translate((0.008, 0.0, 0.008))
    )
    pull = (
        cq.Workplane("XY")
        .box(0.024, 0.118, 0.040, centered=(True, True, False))
        .translate((-0.010, 0.0, 0.013))
    )
    return outer.cut(inner).union(pull)


def _basket_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.080).extrude(0.034)
    inner = cq.Workplane("XY").circle(0.068).extrude(0.030).translate((0.0, 0.0, 0.004))
    shell = outer.cut(inner)

    disc = cq.Workplane("XY").circle(0.072).extrude(0.004).translate((0.0, 0.0, 0.034))
    disc_hole = (
        cq.Workplane("XY").circle(0.018).extrude(0.006).translate((0.0, 0.0, 0.033))
    )
    hub = cq.Workplane("XY").circle(0.018).extrude(0.016).translate((0.0, 0.0, 0.018))
    return shell.union(disc.cut(disc_hole)).union(hub)


def _dial_mesh(name: str):
    dial_geom = KnobGeometry(
        0.034,
        0.018,
        body_style="tapered",
        top_diameter=0.028,
        grip=KnobGrip(style="fluted", count=12, depth=0.0008),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    return mesh_from_geometry(dial_geom, name)


def _rounded_axis(axis) -> tuple[float, float, float]:
    return tuple(round(float(v), 3) for v in axis)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_juicer")

    body = model.material("body", rgba=(0.16, 0.18, 0.20, 1.0))
    trim = model.material("trim", rgba=(0.23, 0.25, 0.28, 1.0))
    clear = model.material("clear", rgba=(0.74, 0.84, 0.90, 0.35))
    stainless = model.material("stainless", rgba=(0.79, 0.81, 0.83, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    pusher_dark = model.material("pusher_dark", rgba=(0.22, 0.23, 0.24, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "juicer_base"), material=body, name="base_shell")
    base.visual(
        Box((0.008, 0.126, 0.050)),
        origin=Origin(xyz=(0.182, 0.0, 0.060)),
        material=trim,
        name="control_panel",
    )
    base.visual(
        Box((0.044, 0.052, 0.014)),
        origin=Origin(xyz=(0.148, 0.0, 0.136), rpy=(0.0, -0.22, 0.0)),
        material=stainless,
        name="spout",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shape(), "juicer_lid"), material=clear, name="lid_shell")

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "juicer_basket"),
        material=stainless,
        name="basket_shell",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.036, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pusher_dark,
        name="pusher_body",
    )
    pusher.visual(
        Cylinder(radius=0.041, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=pusher_dark,
        name="pusher_cap",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "pulp_drawer"),
        material=trim,
        name="drawer_shell",
    )

    dial_mesh_0 = _dial_mesh("dial_0_cap")
    dial_mesh_1 = _dial_mesh("dial_1_cap")
    dial_pitch = math.pi / 2.0

    dial_0 = model.part("dial_0")
    dial_0.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, dial_pitch, 0.0)),
        material=trim,
        name="dial_shaft",
    )
    dial_0.visual(
        dial_mesh_0,
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, dial_pitch, 0.0)),
        material=knob_dark,
        name="dial_cap",
    )

    dial_1 = model.part("dial_1")
    dial_1.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, dial_pitch, 0.0)),
        material=trim,
        name="dial_shaft",
    )
    dial_1.visual(
        dial_mesh_1,
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, dial_pitch, 0.0)),
        material=knob_dark,
        name="dial_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=LID_OPEN,
        ),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(DRIVE_X, 0.0, BASKET_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.125, 0.0, 0.165)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=PUSHER_TRAVEL,
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(-BASE_LENGTH * 0.5, 0.0, 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "dial_0_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial_0,
        origin=Origin(xyz=(0.186, -0.038, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=-DIAL_SWEEP,
            upper=DIAL_SWEEP,
        ),
    )
    model.articulation(
        "dial_1_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial_1,
        origin=Origin(xyz=(0.186, 0.038, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=-DIAL_SWEEP,
            upper=DIAL_SWEEP,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    pusher = object_model.get_part("pusher")
    drawer = object_model.get_part("drawer")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")

    lid_hinge = object_model.get_articulation("lid_hinge")
    basket_spin = object_model.get_articulation("basket_spin")
    pusher_slide = object_model.get_articulation("pusher_slide")
    drawer_slide = object_model.get_articulation("drawer_slide")
    dial_0_spin = object_model.get_articulation("dial_0_spin")
    dial_1_spin = object_model.get_articulation("dial_1_spin")

    ctx.allow_overlap(
        base,
        drawer,
        elem_a="base_shell",
        elem_b="drawer_shell",
        reason="The pulp drawer is intentionally represented as nesting inside the rear base housing cavity.",
    )
    ctx.allow_overlap(
        lid,
        pusher,
        elem_a="lid_shell",
        elem_b="pusher_body",
        reason="The feed pusher is intentionally represented as sliding inside the chute shell.",
    )

    ctx.expect_within(
        basket,
        lid,
        axes="xy",
        margin=0.02,
        name="basket stays inside the clear chamber footprint",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="xy",
        min_overlap=0.05,
        name="pusher stays aligned with the feed chute",
    )
    ctx.expect_origin_distance(
        dial_0,
        dial_1,
        axes="y",
        min_dist=0.06,
        max_dist=0.09,
        name="front dials sit side by side",
    )
    ctx.expect_overlap(
        dial_0,
        base,
        axes="yz",
        min_overlap=0.02,
        name="lower dial remains on the front panel footprint",
    )
    ctx.expect_overlap(
        dial_1,
        base,
        axes="yz",
        min_overlap=0.02,
        name="upper dial remains on the front panel footprint",
    )
    ctx.expect_within(
        drawer,
        base,
        axes="yz",
        margin=0.01,
        name="drawer stays centered in the rear cavity opening",
    )

    ctx.check(
        "articulation axes match the juicer mechanisms",
        _rounded_axis(lid_hinge.axis) == (0.0, -1.0, 0.0)
        and _rounded_axis(basket_spin.axis) == (0.0, 0.0, 1.0)
        and _rounded_axis(pusher_slide.axis) == (0.0, 0.0, -1.0)
        and _rounded_axis(drawer_slide.axis) == (-1.0, 0.0, 0.0)
        and _rounded_axis(dial_0_spin.axis) == (1.0, 0.0, 0.0)
        and _rounded_axis(dial_1_spin.axis) == (1.0, 0.0, 0.0),
        details=(
            f"lid={lid_hinge.axis}, basket={basket_spin.axis}, pusher={pusher_slide.axis}, "
            f"drawer={drawer_slide.axis}, dial_0={dial_0_spin.axis}, dial_1={dial_1_spin.axis}"
        ),
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: LID_OPEN}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid swings upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: PUSHER_TRAVEL}):
        lowered_pusher_pos = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="x",
            min_overlap=0.05,
            name="lowered pusher remains engaged in the chute assembly",
        )
    ctx.check(
        "pusher slides down the chute axis",
        rest_pusher_pos is not None
        and lowered_pusher_pos is not None
        and lowered_pusher_pos[2] < rest_pusher_pos[2] - 0.04,
        details=f"rest={rest_pusher_pos}, lowered={lowered_pusher_pos}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        open_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            min_overlap=0.025,
            name="drawer remains retained when extended",
        )
    ctx.check(
        "drawer slides out from the rear",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[0] < closed_drawer_pos[0] - 0.10,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
