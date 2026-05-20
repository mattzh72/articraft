from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_wheel():
    radius = 1.5
    width = 0.8
    rim_thickness = 0.05
    rim_depth = 0.2
    hub_radius = 0.2
    hub_width = 0.9
    axle_radius = 0.05
    axle_width = 1.8
    
    # Hub and axle
    wheel = cq.Workplane("XY").circle(hub_radius).extrude(hub_width).translate((0, 0, -hub_width/2))
    axle = cq.Workplane("XY").circle(axle_radius).extrude(axle_width).translate((0, 0, -axle_width/2))
    wheel = wheel.union(axle)
    
    # Rims
    rim_solid = cq.Workplane("XY").circle(radius).circle(radius - rim_depth).extrude(rim_thickness)
    rim1 = rim_solid.translate((0, 0, width/2 - rim_thickness))
    rim2 = rim_solid.translate((0, 0, -width/2))
    wheel = wheel.union(rim1).union(rim2)
    
    # Spokes
    spoke_thickness = 0.05
    spoke_width = 0.1
    spoke_base = cq.Workplane("XY").box(radius * 2, spoke_width, spoke_thickness)
    spokes = cq.Workplane("XY")
    for i in range(4):
        s = spoke_base.rotate((0,0,0), (0,0,1), i * 45)
        s1 = s.translate((0, 0, width/2 - rim_thickness/2))
        s2 = s.translate((0, 0, -width/2 + rim_thickness/2))
        spokes = spokes.add(s1.vals()).add(s2.vals())
    spokes = spokes.combine()
    wheel = wheel.union(spokes)
    
    # Buckets
    bucket_count = 16
    bucket_base = cq.Workplane("XY").box(rim_depth, spoke_thickness, width + 0.02)
    bucket_base = bucket_base.translate((radius - rim_depth/2, 0, 0))
    bucket_base = bucket_base.rotate((radius - rim_depth/2, 0, 0), (0, 0, 1), 45)
    
    buckets = cq.Workplane("XY")
    for i in range(bucket_count):
        b = bucket_base.rotate((0,0,0), (0,0,1), i * 360 / bucket_count)
        buckets = buckets.add(b.vals())
    buckets = buckets.combine()
    wheel = wheel.union(buckets)
    
    # Rotate to have axle along Y axis
    wheel = wheel.rotate((0,0,0), (1,0,0), 90)
    return wheel

def make_chute():
    chute = cq.Workplane("XY").box(1.7, 0.6, 0.05).translate((0, 0, 0))
    side1 = cq.Workplane("XY").box(1.7, 0.05, 0.2).translate((0, 0.275, 0.125))
    side2 = cq.Workplane("XY").box(1.7, 0.05, 0.2).translate((0, -0.275, 0.125))
    return chute.union(side1).union(side2)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waterwheel")
    
    base = model.part("base")
    # Base plate
    base.visual(Box((4.0, 2.0, 0.1)), origin=Origin(xyz=(0, 0, 0.05)), name="base_plate")
    # Wheel pillars
    base.visual(Box((0.4, 0.2, 2.1)), origin=Origin(xyz=(0, 0.7, 1.05)), name="pillar_1")
    base.visual(Box((0.4, 0.2, 2.1)), origin=Origin(xyz=(0, -0.7, 1.05)), name="pillar_2")
    # Chute support
    base.visual(Box((0.2, 0.6, 3.8)), origin=Origin(xyz=(-1.7, 0, 1.9)), name="chute_support")
    
    chute = model.part("chute")
    chute_geom = make_chute()
    chute.visual(
        mesh_from_cadquery(chute_geom, "chute_mesh"),
        origin=Origin(xyz=(-1.1, 0, 3.7), rpy=(0, 0.15, 0)),
        name="chute_visual"
    )
    
    # Fix chute to base
    model.articulation(
        "base_to_chute",
        ArticulationType.FIXED,
        parent=base,
        child=chute,
        origin=Origin()
    )
    
    wheel = model.part("wheel")
    wheel_geom = make_wheel()
    wheel.visual(
        mesh_from_cadquery(wheel_geom, "wheel_mesh"),
        origin=Origin(),
        name="wheel_visual"
    )
    
    model.articulation(
        "axle_joint",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0, 0, 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    wheel = object_model.get_part("wheel")
    chute = object_model.get_part("chute")
    
    # The wheel axle intersects the pillars intentionally
    ctx.allow_overlap(
        wheel, base,
        elem_a="wheel_visual", elem_b="pillar_1",
        reason="Axle is captured by the support pillar."
    )
    ctx.allow_overlap(
        wheel, base,
        elem_a="wheel_visual", elem_b="pillar_2",
        reason="Axle is captured by the support pillar."
    )
    ctx.allow_overlap(
        chute, base,
        elem_a="chute_visual", elem_b="chute_support",
        reason="Chute rests on the support pillar."
    )
    
    ctx.expect_within(wheel, base, axes="y", name="Wheel is constrained between pillars")
    
    return ctx.report()

object_model = build_object_model()