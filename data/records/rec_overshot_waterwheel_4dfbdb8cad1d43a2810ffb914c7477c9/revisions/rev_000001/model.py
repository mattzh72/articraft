from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def create_support_frame():
    frame = cq.Workplane("XY").box(5.0, 3.0, 0.2).translate((0, 0, 0.1))
    
    for y_offset in [-0.8, 0.8]:
        post = cq.Workplane("XY").box(0.4, 0.4, 4.5).translate((0, y_offset, 2.35))
        frame = frame.union(post)
        
        bearing = cq.Workplane("XY").box(0.8, 0.4, 0.4).translate((0, y_offset, 2.4))
        groove = cq.Workplane("XZ").cylinder(1.0, 0.16).translate((0, y_offset, 2.5))
        bearing = bearing.cut(groove)
        frame = frame.union(bearing)
        
        brace1 = cq.Workplane("XZ").rect(0.2, 3.0).extrude(0.1, both=True).rotate((0,0,0), (0,1,0), 30).translate((0, y_offset, 1.2))
        brace2 = cq.Workplane("XZ").rect(0.2, 3.0).extrude(0.1, both=True).rotate((0,0,0), (0,1,0), -30).translate((0, y_offset, 1.2))
        frame = frame.union(brace1).union(brace2)
        
    cut_box = cq.Workplane("XY").box(10.0, 10.0, 2.0).translate((0, 0, -0.8))
    frame = frame.cut(cut_box)
    
    # Flume
    flume_bottom = cq.Workplane("XY").box(2.8, 0.8, 0.1).translate((-1.6, 0, 4.65))
    flume_side1 = cq.Workplane("XY").box(2.8, 0.1, 0.4).translate((-1.6, -0.35, 4.9))
    flume_side2 = cq.Workplane("XY").box(2.8, 0.1, 0.4).translate((-1.6, 0.35, 4.9))
    flume_back = cq.Workplane("XY").box(0.1, 0.8, 0.4).translate((-2.95, 0, 4.9))
    
    frame = frame.union(flume_bottom).union(flume_side1).union(flume_side2).union(flume_back)
    
    # Flume supports
    f_post1 = cq.Workplane("XY").box(0.3, 0.3, 4.5).translate((-2.5, -0.8, 2.25))
    f_post2 = cq.Workplane("XY").box(0.3, 0.3, 4.5).translate((-2.5, 0.8, 2.25))
    
    c_beam1 = cq.Workplane("XY").box(2.8, 0.2, 0.3).translate((-1.25, -0.8, 4.45))
    c_beam2 = cq.Workplane("XY").box(2.8, 0.2, 0.3).translate((-1.25, 0.8, 4.45))
    
    cross1 = cq.Workplane("XY").box(0.2, 1.8, 0.3).translate((-2.5, 0, 4.55))
    cross2 = cq.Workplane("XY").box(0.2, 1.8, 0.3).translate((-1.5, 0, 4.55))
    cross3 = cq.Workplane("XY").box(0.2, 1.8, 0.3).translate((-0.5, 0, 4.55))
    
    frame = frame.union(f_post1).union(f_post2).union(c_beam1).union(c_beam2).union(cross1).union(cross2).union(cross3)
    
    return frame

def create_wheel():
    radius = 2.0
    width = 1.2
    rim_depth = 0.3
    hub_radius = 0.3
    shaft_radius = 0.15
    num_spokes = 8
    num_buckets = 24
    spoke_width = 0.15
    spoke_thickness = 0.1
    rim_thickness = 0.1
    
    axle = cq.Workplane("XZ").cylinder(width + 0.6, shaft_radius)
    hub = cq.Workplane("XZ").cylinder(width, hub_radius)
    wheel = axle.union(hub)
    
    left_rim = (
        cq.Workplane("XZ")
        .workplane(offset=-width/2)
        .circle(radius)
        .circle(radius - rim_depth)
        .extrude(rim_thickness)
    )
    right_rim = (
        cq.Workplane("XZ")
        .workplane(offset=width/2 - rim_thickness)
        .circle(radius)
        .circle(radius - rim_depth)
        .extrude(rim_thickness)
    )
    wheel = wheel.union(left_rim).union(right_rim)
    
    spokes = []
    spoke_l_base = (
        cq.Workplane("XZ")
        .workplane(offset=-width/2)
        .rect(spoke_width, radius * 2 - rim_depth)
        .extrude(spoke_thickness)
    )
    spoke_r_base = (
        cq.Workplane("XZ")
        .workplane(offset=width/2 - spoke_thickness)
        .rect(spoke_width, radius * 2 - rim_depth)
        .extrude(spoke_thickness)
    )
    
    for i in range(num_spokes // 2):
        angle = i * (180.0 / (num_spokes // 2))
        spokes.append(spoke_l_base.rotate((0,0,0), (0,1,0), angle).val())
        spokes.append(spoke_r_base.rotate((0,0,0), (0,1,0), angle).val())
        
    wheel = wheel.union(cq.Workplane("XZ").add(spokes).combine())
    
    bucket_length = 0.5
    bucket_thickness = 0.05
    bucket_half_width = (width - rim_thickness * 2) / 2.0
    
    bucket = (
        cq.Workplane("XZ")
        .rect(bucket_length, bucket_thickness)
        .extrude(bucket_half_width, both=True)
    )
    bucket = bucket.translate((-1.85, 0, 0))
    bucket = bucket.rotate((-1.85, 0, 0), (-1.85, 1, 0), -127)
    
    buckets = []
    for i in range(num_buckets):
        angle = i * (360.0 / num_buckets)
        buckets.append(bucket.rotate((0,0,0), (0,1,0), angle).val())
        
    wheel = wheel.union(cq.Workplane("XZ").add(buckets).combine())
    
    return wheel

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    # Support Frame
    frame = model.part("support_frame")
    frame.visual(
        mesh_from_cadquery(create_support_frame(), "frame_mesh"),
        name="frame_visual",
        color=(0.35, 0.25, 0.15) # Wood color
    )

    # Wheel
    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(create_wheel(), "wheel_mesh"),
        name="wheel_visual",
        color=(0.4, 0.3, 0.2) # Slightly different wood color
    )

    # Articulation
    model.articulation(
        name="wheel_rotation",
        articulation_type=ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 2.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=2.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # We have an intentional overlap: the axle sits exactly in the groove of the bearing block.
    # The axle is part of the wheel, and the bearing block is part of the frame.
    ctx.allow_overlap(
        "wheel", "support_frame",
        reason="The wheel axle sits in the bearing block groove."
    )
    
    # Check that the wheel is centered in the frame
    ctx.expect_within(
        "wheel", "support_frame",
        axes="y",
        margin=0.1,
        name="wheel fits between the supports"
    )
    
    # Check that the wheel is above the ground
    ctx.check("wheel clears ground", ctx.part_world_aabb("wheel")[0][2] > 0.1)

    return ctx.report()

object_model = build_object_model()
