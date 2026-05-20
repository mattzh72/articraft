from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate")

    painted_metal = Material(name="painted_metal", color=(0.2, 0.3, 0.6))
    black_plastic = Material(name="black_plastic", color=(0.1, 0.1, 0.1))

    # Bed Frame Part
    bed = model.part("bed_frame")
    
    bed_floor = cq.Workplane("XY").rect(1.6, 0.8).extrude(0.05).translate((0, 0.4, 0))
    left_wall = cq.Workplane("XY").rect(0.05, 0.9).extrude(0.6).translate((-0.825, 0.35, 0))
    right_wall = cq.Workplane("XY").rect(0.05, 0.9).extrude(0.6).translate((0.825, 0.35, 0))
    front_wall = cq.Workplane("XY").rect(1.7, 0.05).extrude(0.6).translate((0, 0.775, 0))
    bed_cq = bed_floor.union(left_wall).union(right_wall).union(front_wall)
    
    wheel_well_left = cq.Workplane("XZ").cylinder(0.6, 0.2).translate((-0.8, 0.4, 0.05))
    wheel_well_right = cq.Workplane("XZ").cylinder(0.6, 0.2).translate((0.8, 0.4, 0.05))
    bed_cq = bed_cq.union(wheel_well_left).union(wheel_well_right)
    
    left_hole = cq.Workplane("YZ").cylinder(0.06, 0.016).translate((-0.825, 0, 0.05))
    right_hole = cq.Workplane("YZ").cylinder(0.06, 0.016).translate((0.825, 0, 0.05))
    bed_cq = bed_cq.cut(left_hole).cut(right_hole)

    bed.visual(
        mesh_from_cadquery(bed_cq, "bed_frame_mesh"),
        material=painted_metal,
        name="bed_shell"
    )

    # Tailgate Part
    tailgate = model.part("tailgate")
    
    main_body = (
        cq.Workplane("XY")
        .box(1.58, 0.08, 0.53)
        .translate((0, -0.04, 0.265))
    )
    stamp = (
        cq.Workplane("XY")
        .box(1.4, 0.02, 0.3)
        .translate((0, -0.08, 0.265))
    )
    main_body = main_body.cut(stamp)
    
    rib1 = cq.Workplane("XY").box(1.4, 0.01, 0.05).translate((0, 0.005, 0.15))
    rib2 = cq.Workplane("XY").box(1.4, 0.01, 0.05).translate((0, 0.005, 0.35))
    main_body = main_body.union(rib1).union(rib2)
    
    left_pin = cq.Workplane("YZ").cylinder(0.02, 0.015).translate((-0.8, 0, 0))
    right_pin = cq.Workplane("YZ").cylinder(0.02, 0.015).translate((0.8, 0, 0))
    main_body = main_body.union(left_pin).union(right_pin)

    tailgate.visual(
        mesh_from_cadquery(main_body, "tailgate_body_mesh"),
        material=painted_metal,
        name="tailgate_shell"
    )
    
    top_cap_cq = (
        cq.Workplane("XY")
        .box(1.59, 0.09, 0.02)
        .translate((0, -0.045, 0.54))
    )
    tailgate.visual(
        mesh_from_cadquery(top_cap_cq, "tailgate_cap_mesh"),
        material=black_plastic,
        name="top_cap"
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708) 
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    bed = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    hinge = object_model.get_articulation("tailgate_hinge")

    ctx.allow_overlap(
        bed, tailgate,
        elem_a="bed_shell",
        elem_b="tailgate_shell",
        reason="Hinge pins are captured inside the side wall holes."
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(tailgate, bed, axes="xy", margin=0.02)

    with ctx.pose({hinge: 1.57}):
        ctx.expect_overlap(bed, tailgate, axes="z", min_overlap=0.01)
        
    return ctx.report()

object_model = build_object_model()