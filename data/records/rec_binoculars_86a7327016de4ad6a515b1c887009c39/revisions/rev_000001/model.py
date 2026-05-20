import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilized_binoculars")

    black_rubber = model.material("black_rubber", rgba=(0.15, 0.15, 0.15, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.25, 0.25, 0.25, 1.0))
    glass = model.material("glass", rgba=(0.1, 0.1, 0.3, 0.8))

    # Dimensions
    hinge_length = 0.08
    hinge_radius = 0.008
    prism_box_width = 0.04
    prism_box_length = 0.07
    prism_box_height = 0.035
    objective_length = 0.06
    objective_radius = 0.022
    eyepiece_length = 0.04
    eyepiece_radius = 0.016
    barrel_offset_x = 0.035
    barrel_offset_z = -0.01

    def make_half(is_left: bool):
        sign = -1 if is_left else 1
        
        # Hinge barrels (interleaved)
        hinge = cq.Workplane("XZ").workplane(offset=-hinge_length/2)
        if is_left:
            # Left gets front and back hinge segments
            hinge = hinge.center(0, 0).circle(hinge_radius).extrude(hinge_length * 0.3)
            hinge = hinge.faces(">Y").workplane().center(0, 0).circle(hinge_radius).extrude(hinge_length * 0.4).cut(hinge) # wait, just add another cylinder
            # Let's do it simpler
            h1 = cq.Workplane("XZ").workplane(offset=hinge_length/2 - hinge_length*0.3).circle(hinge_radius).extrude(hinge_length*0.3)
            h2 = cq.Workplane("XZ").workplane(offset=-hinge_length/2).circle(hinge_radius).extrude(hinge_length*0.3)
            hinge_parts = h1.union(h2)
        else:
            # Right gets middle hinge segment
            hinge_parts = cq.Workplane("XZ").workplane(offset=-hinge_length/2 + hinge_length*0.3).circle(hinge_radius).extrude(hinge_length*0.4)

        # Prism block (the "stabilizing prism block" half)
        # We want it to connect from the hinge to the barrel
        prism_center_x = sign * (prism_box_width / 2 + 0.005)
        prism = cq.Workplane("XY").workplane(offset=barrel_offset_z).center(prism_center_x, 0).box(prism_box_width, prism_box_length, prism_box_height)
        
        # Bridge arm connecting hinge to prism block
        arm = cq.Workplane("XY").workplane(offset=0).center(sign * 0.015, 0).box(0.03, prism_box_length * 0.8, hinge_radius * 2)
        
        # Objective barrel
        obj_barrel = cq.Workplane("XZ").workplane(offset=prism_box_length/2).center(sign * barrel_offset_x, barrel_offset_z).circle(objective_radius).extrude(objective_length)
        
        # Eyepiece
        eyepiece = cq.Workplane("XZ").workplane(offset=-prism_box_length/2 - eyepiece_length).center(sign * barrel_offset_x, barrel_offset_z).circle(eyepiece_radius).extrude(eyepiece_length)
        
        # Combine
        body = hinge_parts.union(prism).union(arm).union(obj_barrel).union(eyepiece)
        
        # Fillet some edges to make it look organic
        # body = body.edges("|Y").fillet(0.005) # might fail, skip for now
        
        # Lenses
        obj_lens = cq.Workplane("XZ").workplane(offset=prism_box_length/2 + objective_length - 0.002).center(sign * barrel_offset_x, barrel_offset_z).circle(objective_radius - 0.002).extrude(0.002)
        eye_lens = cq.Workplane("XZ").workplane(offset=-prism_box_length/2 - eyepiece_length).center(sign * barrel_offset_x, barrel_offset_z).circle(eyepiece_radius - 0.002).extrude(0.002)
        
        return body, obj_lens, eye_lens

    left_body, left_obj, left_eye = make_half(True)
    right_body, right_obj, right_eye = make_half(False)

    # Focus wheel with a shaft extending into the hinge barrel
    wheel_radius = 0.012
    wheel_length = 0.015
    shaft_radius = 0.004
    shaft_length = 0.01
    
    wheel_body = cq.Workplane("XZ").workplane(offset=-hinge_length/2 - wheel_length).circle(wheel_radius).extrude(wheel_length)
    shaft = cq.Workplane("XZ").workplane(offset=-hinge_length/2).circle(shaft_radius).extrude(shaft_length)
    wheel = wheel_body.union(shaft)

    left_side = model.part("left_side")
    left_side.visual(mesh_from_cadquery(left_body, "left_body"), material=black_rubber)
    left_side.visual(mesh_from_cadquery(left_obj, "left_obj"), material=glass)
    left_side.visual(mesh_from_cadquery(left_eye, "left_eye"), material=glass)

    right_side = model.part("right_side")
    right_side.visual(mesh_from_cadquery(right_body, "right_body"), material=black_rubber)
    right_side.visual(mesh_from_cadquery(right_obj, "right_obj"), material=glass)
    right_side.visual(mesh_from_cadquery(right_eye, "right_eye"), material=glass)

    focus_knob = model.part("focus_wheel")
    focus_knob.visual(mesh_from_cadquery(wheel, "wheel"), material=dark_gray)

    # Articulations
    # The bridge hinge adjusts IPD. 
    # Left side is root. Right side hinges around Y axis (X=0, Z=0).
    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=left_side,
        child=right_side,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.2, upper=0.4, effort=5.0, velocity=1.0)
    )

    # Focus wheel rotates continuously
    model.articulation(
        "focus_joint",
        ArticulationType.CONTINUOUS,
        parent=left_side,
        child=focus_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # We allow overlap between the interleaved hinge barrels
    ctx.allow_overlap("left_side", "right_side", reason="Hinge barrels are interleaved")
    
    # Allow overlap for the focus wheel shaft inserted into the left side hinge barrel
    ctx.allow_overlap("focus_wheel", "left_side", reason="Focus wheel shaft is inserted into the hinge barrel")
    
    # Exact assertions
    # The right side should overlap the left side on the Y axis due to the interleaved hinge
    ctx.expect_overlap("right_side", "left_side", axes="y", min_overlap=0.01)
    
    # The focus wheel should overlap the left side on the Y axis due to the inserted shaft
    ctx.expect_overlap("focus_wheel", "left_side", axes="y", min_overlap=0.005)
    
    # Hinge pose check: rotating the bridge hinge should decrease the distance between the two halves (IPD adjustment)
    left_aabb_rest = ctx.part_world_aabb("left_side")
    right_aabb_rest = ctx.part_world_aabb("right_side")
    if left_aabb_rest and right_aabb_rest:
        left_center_rest = [(left_aabb_rest[0][i] + left_aabb_rest[1][i]) / 2 for i in range(3)]
        right_center_rest = [(right_aabb_rest[0][i] + right_aabb_rest[1][i]) / 2 for i in range(3)]
        dist_rest = sum((left_center_rest[i] - right_center_rest[i])**2 for i in range(3))**0.5
        
        with ctx.pose(bridge_hinge=0.4):
            left_aabb_posed = ctx.part_world_aabb("left_side")
            right_aabb_posed = ctx.part_world_aabb("right_side")
            if left_aabb_posed and right_aabb_posed:
                left_center_posed = [(left_aabb_posed[0][i] + left_aabb_posed[1][i]) / 2 for i in range(3)]
                right_center_posed = [(right_aabb_posed[0][i] + right_aabb_posed[1][i]) / 2 for i in range(3)]
                dist_posed = sum((left_center_posed[i] - right_center_posed[i])**2 for i in range(3))**0.5
                
                ctx.check("ipd_adjustment", dist_posed < dist_rest - 0.002, "Bridge hinge should decrease distance between halves")

    
    return ctx.report()

object_model = build_object_model()
