import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="juicer")

    base = model.part("base")
    
    # Base geometry
    base_body = (
        cq.Workplane("XY")
        .cylinder(0.15, 0.10)
        .translate((0, 0, 0.075))
    )
    chamber_cut = (
        cq.Workplane("XY")
        .cylinder(0.08, 0.09)
        .translate((0, 0, 0.11))
    )
    base_hinge_left = (
        cq.Workplane("XY")
        .box(0.01, 0.035, 0.02)
        .translate((-0.015, 0.1125, 0.14))
    )
    base_hinge_right = (
        cq.Workplane("XY")
        .box(0.01, 0.035, 0.02)
        .translate((0.015, 0.1125, 0.14))
    )
    motor_shaft = (
        cq.Workplane("XY")
        .cylinder(0.02, 0.01)
        .translate((0, 0, 0.07))
    )
    base_shape = (
        base_body
        .cut(chamber_cut)
        .union(base_hinge_left)
        .union(base_hinge_right)
        .union(motor_shaft)
    )
    base.visual(
        mesh_from_cadquery(base_shape, "base_mesh"),
        name="base_visual"
    )

    # Dials
    dial_0 = model.part("dial_0")
    dial_0.visual(
        Cylinder(radius=0.015, height=0.01),
        origin=Origin(xyz=(0, -0.005, 0), rpy=(math.pi/2, 0, 0)),
        name="dial_0_visual"
    )
    model.articulation(
        "base_to_dial_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial_0,
        origin=Origin(xyz=(-0.03, -0.095, 0.05)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.5, upper=1.5)
    )

    dial_1 = model.part("dial_1")
    dial_1.visual(
        Cylinder(radius=0.015, height=0.01),
        origin=Origin(xyz=(0, -0.005, 0), rpy=(math.pi/2, 0, 0)),
        name="dial_1_visual"
    )
    model.articulation(
        "base_to_dial_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial_1,
        origin=Origin(xyz=(0.03, -0.095, 0.05)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.5, upper=1.5)
    )

    # Basket
    basket = model.part("basket")
    basket_outer = (
        cq.Workplane("XY")
        .circle(0.04).workplane(offset=0.06).circle(0.08).loft()
    )
    basket_inner = (
        cq.Workplane("XY")
        .circle(0.038).workplane(offset=0.06).circle(0.078).loft()
        .translate((0, 0, 0.002))
    )
    basket_shape = basket_outer.cut(basket_inner)
    basket.visual(
        mesh_from_cadquery(basket_shape, "basket_mesh"),
        name="basket_visual"
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0, 0, 0.08)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0)
    )

    # Lid
    lid = model.part("lid")
    lid_outer = (
        cq.Workplane("XY")
        .cylinder(0.05, 0.10)
        .translate((0, -0.1125, 0.035))
    )
    lid_inner = (
        cq.Workplane("XY")
        .cylinder(0.048, 0.098)
        .translate((0, -0.1125, 0.033))
    )
    lid_body = lid_outer.cut(lid_inner)
    
    chute = (
        cq.Workplane("XY")
        .cylinder(0.10, 0.03)
        .translate((0, -0.1125, 0.11))
    )
    chute_hole = (
        cq.Workplane("XY")
        .cylinder(0.20, 0.028)
        .translate((0, -0.1125, 0.11))
    )
    lid_hinge = (
        cq.Workplane("YZ")
        .cylinder(0.02, 0.01)
    )
    lid_hinge_arm = (
        cq.Workplane("XY")
        .box(0.018, 0.02, 0.02)
        .translate((0, -0.01, 0))
    )
    lid_shape = lid_body.union(chute).union(lid_hinge).union(lid_hinge_arm).cut(chute_hole)
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid_mesh"),
        name="lid_visual"
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0, 0.1125, 0.14)),
        axis=(-1, 0, 0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.5)
    )

    # Pusher
    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.026, height=0.12),
        origin=Origin(xyz=(0, 0, -0.06)),
        name="pusher_visual"
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0, -0.1125, 0.16)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.035, upper=0.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    pusher = object_model.get_part("pusher")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")

    ctx.allow_overlap(dial_0, base, reason="Dial slightly embedded in base surface for flush fit.")
    ctx.allow_overlap(dial_1, base, reason="Dial slightly embedded in base surface for flush fit.")
    ctx.allow_overlap(basket, base, reason="Basket sits exactly on the motor shaft.")
    ctx.allow_overlap(lid, base, reason="Lid rests exactly on base rim.")
    
    ctx.allow_isolated_part(pusher, reason="Pusher slides inside the chute with a clearance gap.")

    ctx.expect_contact(basket, base, name="Basket sits on motor shaft")
    ctx.expect_contact(lid, base, name="Lid rests on base rim")
    ctx.expect_within(pusher, lid, axes="xy", margin=0.005, name="Pusher guided inside chute")

    lid_joint = object_model.get_articulation("base_to_lid")
    rest_pos = ctx.part_world_position(pusher)
    with ctx.pose({lid_joint: 1.5}):
        open_pos = ctx.part_world_position(pusher)
        ctx.check(
            "pusher swings backward when lid opens",
            open_pos is not None and rest_pos is not None and open_pos[1] > rest_pos[1] + 0.10,
            details="The lid opening should swing the front chute and pusher backwards."
        )

    pusher_joint = object_model.get_articulation("lid_to_pusher")
    with ctx.pose({pusher_joint: -0.035}):
        down_pos = ctx.part_world_position(pusher)
        ctx.check(
            "pusher slides down",
            down_pos is not None and rest_pos is not None and down_pos[2] < rest_pos[2] - 0.03,
            details="The pusher should slide downwards into the chute."
        )

    return ctx.report()

object_model = build_object_model()