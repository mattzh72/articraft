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

def build_bowl_mesh():
    outer = cq.Workplane("XY").cylinder(0.14, 0.085, centered=(True, True, False))
    inner = cq.Workplane("XY").transformed(offset=(0, 0, 0.002)).cylinder(0.14, 0.083, centered=(True, True, False))
    shaft_hole = cq.Workplane("XY").transformed(offset=(0, 0, -0.001)).cylinder(0.005, 0.015, centered=(True, True, False))
    return outer.cut(inner).cut(shaft_hole)

def build_lid_mesh():
    # outer base from Z=-0.01 to 0.02
    outer = cq.Workplane("XY").transformed(offset=(0, 0, -0.01)).cylinder(0.03, 0.088, centered=(True, True, False))
    outer_tube = cq.Workplane("XY").transformed(offset=(0, 0.04, 0.02)).cylinder(0.10, 0.03, centered=(True, True, False))
    outer = outer.union(outer_tube)
    
    # inner base from Z=-0.011 to 0.001
    inner = cq.Workplane("XY").transformed(offset=(0, 0, -0.011)).cylinder(0.012, 0.085, centered=(True, True, False))
    inner_tube = cq.Workplane("XY").transformed(offset=(0, 0.04, -0.011)).cylinder(0.132, 0.028, centered=(True, True, False))
    inner = inner.union(inner_tube)
    
    return outer.cut(inner)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_processor")

    base = model.part("base")
    base.visual(Box((0.18, 0.22, 0.16)), origin=Origin(xyz=(0, 0, 0.08)), name="base_body")
    base.visual(Cylinder(0.01, 0.02), origin=Origin(xyz=(0, 0, 0.165)), name="motor_shaft")

    timer_dial = model.part("timer_dial")
    timer_dial.visual(Cylinder(0.01, 0.015), origin=Origin(xyz=(0, 0, 0)), name="dial_body")
    model.articulation(
        "base_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=timer_dial,
        origin=Origin(xyz=(-0.04, -0.11, 0.06), rpy=(-math.pi/2, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0)
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(Box((0.015, 0.005, 0.03)), origin=Origin(xyz=(0, 0, 0)), name="rocker_body")
    model.articulation(
        "base_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=base,
        child=power_rocker,
        origin=Origin(xyz=(0.04, -0.11, 0.06)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.2, upper=0.2)
    )

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(build_bowl_mesh(), "bowl_mesh"), name="bowl_shell")
    model.articulation(
        "base_to_bowl",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0, 0, 0.16)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.5)
    )

    basket = model.part("basket")
    basket.visual(Cylinder(0.012, 0.08), origin=Origin(xyz=(0, 0, 0.04)), name="basket_stem")
    basket.visual(Cylinder(0.075, 0.01), origin=Origin(xyz=(0, 0, 0.085)), name="basket_disk")
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0, 0, 0.16)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0)
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(build_lid_mesh(), "lid_mesh"), name="lid_shell")
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0, 0, 0.15)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.3)
    )

    pusher = model.part("pusher")
    pusher.visual(Cylinder(0.027, 0.12), origin=Origin(xyz=(0, 0, 0.06)), name="pusher_body")
    pusher.visual(Cylinder(0.032, 0.01), origin=Origin(xyz=(0, 0, 0.125)), name="pusher_cap")
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0, 0.04, 0.00)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.12)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    timer_dial = object_model.get_part("timer_dial")
    power_rocker = object_model.get_part("power_rocker")
    bowl = object_model.get_part("bowl")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    
    ctx.allow_overlap(base, timer_dial, reason="Timer dial is embedded in the front panel.")
    ctx.allow_overlap(base, power_rocker, reason="Power rocker switch is embedded in the front panel.")
    ctx.allow_overlap(base, basket, elem_a="motor_shaft", elem_b="basket_stem", reason="Motor shaft is captured inside the basket stem.")
    
    ctx.expect_within(pusher, lid, axes="xy", inner_elem="pusher_body", outer_elem="lid_shell", margin=0.002)
    ctx.expect_overlap(pusher, lid, axes="z", elem_a="pusher_body", elem_b="lid_shell", min_overlap=0.05)
    
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    with ctx.pose({pusher_joint: 0.10}):
        ctx.expect_within(pusher, lid, axes="xy", inner_elem="pusher_body", outer_elem="lid_shell", margin=0.002)
        
    return ctx.report()

object_model = build_object_model()