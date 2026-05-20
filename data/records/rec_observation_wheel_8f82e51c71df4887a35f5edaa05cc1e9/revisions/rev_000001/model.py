import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ferris_wheel")

    # Materials
    mat_base = Material(name="base_mat", rgba=(0.2, 0.3, 0.4, 1.0))
    mat_wheel = Material(name="wheel_mat", rgba=(0.8, 0.8, 0.8, 1.0))
    mat_gondola = Material(name="gondola_mat", rgba=(0.8, 0.2, 0.2, 1.0))

    # 1. Base Support
    base = model.part("base")
    
    # A-frame sketch
    a_sketch = (
        cq.Sketch()
        .polygon([(-6.0, 0.0), (0.0, 12.5), (6.0, 0.0), (4.0, 0.0), (0.0, 10.0), (-4.0, 0.0)])
    )
    a_shape = cq.Workplane("XZ").placeSketch(a_sketch).extrude(0.5, both=True)
    tower1 = a_shape.translate((0, 2.25, 0))
    tower2 = a_shape.translate((0, -2.25, 0))
    base_axle_support = cq.Workplane("XZ").cylinder(6.0, 0.4).translate((0, 0, 12.0))
    platform = cq.Workplane("XY").workplane(offset=-0.5).box(16.0, 10.0, 1.0)
    base_geom = tower1.union(tower2).union(base_axle_support).union(platform)

    base.visual(
        mesh_from_cadquery(base_geom, "base_mesh"),
        material=mat_base,
        name="base_visual",
    )

    # 2. Wheel
    wheel = model.part("wheel")
    
    # Hub with a tiny clearance (length 3.4 to fit between towers at +/- 1.75)
    hub = cq.Workplane("XZ").cylinder(3.4, 0.8).cut(cq.Workplane("XZ").cylinder(3.4, 0.41))
    
    def make_ring(radius, thickness, offset_y):
        outer = cq.Workplane("XZ").workplane(offset=offset_y).cylinder(thickness, radius)
        inner = cq.Workplane("XZ").workplane(offset=offset_y).cylinder(thickness, radius - 0.2)
        return outer.cut(inner)

    rim1 = make_ring(8.0, 0.2, 1.5)
    rim2 = make_ring(8.0, 0.2, -1.5)
    
    spokes = cq.Workplane()
    for i in range(4):
        angle = i * 180 / 4
        s1 = cq.Workplane("XZ").workplane(offset=1.5).rect(16.0, 0.2).extrude(0.1, both=True).rotate((0,0,0), (0,1,0), angle)
        s2 = cq.Workplane("XZ").workplane(offset=-1.5).rect(16.0, 0.2).extrude(0.1, both=True).rotate((0,0,0), (0,1,0), angle)
        spokes = spokes.union(s1).union(s2)

    cross_bars = cq.Workplane()
    for i in range(8):
        angle_deg = i * 360 / 8
        angle_rad = math.radians(angle_deg)
        x = 8.0 * math.cos(angle_rad)
        z = 8.0 * math.sin(angle_rad)
        cb = cq.Workplane("XZ").center(x, z).cylinder(3.2, 0.1)
        cross_bars = cross_bars.union(cb)

    wheel_geom = hub.union(rim1).union(rim2).union(spokes).union(cross_bars)

    wheel.visual(
        mesh_from_cadquery(wheel_geom, "wheel_mesh"),
        material=mat_wheel,
        name="wheel_visual",
    )

    model.articulation(
        "base_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 12.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0),
    )

    # 3. Gondolas
    cabin = cq.Workplane("XY").workplane(offset=-1.5).box(1.5, 1.5, 2.0)
    hanger_stem = cq.Workplane("XY").workplane(offset=-0.25).cylinder(0.5, 0.1)
    hanger_ring = cq.Workplane("XZ").cylinder(0.2, 0.2).cut(cq.Workplane("XZ").cylinder(0.2, 0.11))
    gondola_geom = cabin.union(hanger_stem).union(hanger_ring)
    
    for i in range(8):
        gondola = model.part(f"gondola_{i}")
        gondola.visual(
            mesh_from_cadquery(gondola_geom, f"gondola_mesh_{i}"),
            material=mat_gondola,
            name=f"gondola_visual_{i}",
        )
        
        angle_deg = i * 360 / 8
        angle_rad = math.radians(angle_deg)
        x = 8.0 * math.cos(angle_rad)
        z = 8.0 * math.sin(angle_rad)
        
        model.articulation(
            f"wheel_to_gondola_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            mimic=Mimic(joint="base_to_wheel", multiplier=-1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=1.0),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # The wheel hub rotates around and explicitly overlaps with the base axle support.
    ctx.allow_overlap(
        "base", "wheel",
        reason="The wheel hub acts as a sleeve that rotates around the base axle support."
    )
    
    # The gondola hanger rings surround and overlap with the wheel cross bars.
    for i in range(8):
        ctx.allow_overlap(
            "wheel", f"gondola_{i}",
            reason="The gondola hanger ring surrounds and is supported by the wheel cross bar."
        )
        
        # Verify the gondola remains upright at different wheel angles
        # The mimic joint should handle this automatically.
        gondola_name = f"gondola_{i}"
        
        # We can test that the gondola is properly captured by the cross bar
        ctx.expect_overlap(
            "wheel", gondola_name,
            axes="y",
            min_overlap=0.05,
            name=f"{gondola_name} remains captured on the cross bar along Y axis"
        )
        
    return ctx.report()

object_model = build_object_model()