import cadquery as cq
import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_ladder")
    
    H = 1.4
    theta_f = math.radians(10)
    dy_f = H * math.tan(theta_f)
    
    bot_x = 0.25
    top_x = 0.15
    rail_w = 0.025
    rail_d = 0.08
    
    # Front rails
    front_right = (
        cq.Workplane("XY")
        .workplane(offset=0).center(bot_x, 0).rect(rail_w, rail_d)
        .workplane(offset=H).center(top_x - bot_x, dy_f).rect(rail_w, rail_d)
        .loft()
    )
    front_left = (
        cq.Workplane("XY")
        .workplane(offset=0).center(-bot_x, 0).rect(rail_w, rail_d)
        .workplane(offset=H).center(-top_x + bot_x, dy_f).rect(rail_w, rail_d)
        .loft()
    )
    front_rails = front_right.union(front_left)
    
    # Steps
    steps = None
    for i in range(1, 5):
        z = 0.28 * i
        t = z / H
        step_y = t * dy_f
        step_bot_x = bot_x - t * (bot_x - top_x)
        step_w = 2 * step_bot_x + rail_w + 0.002
        step = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .center(0, step_y)
            .rect(step_w, rail_d)
            .extrude(0.025)
        )
        if steps is None:
            steps = step
        else:
            steps = steps.union(step)
            
    # Hinge cylinder
    hinge_z = H - 0.05
    hinge_y = dy_f + 0.06
    top_x_r = 0.14
    hinge_cyl = (
        cq.Workplane("YZ")
        .workplane(offset=-top_x_r - 0.02)
        .center(hinge_y, hinge_z)
        .circle(0.015)
        .extrude(2 * top_x_r + 0.04)
    )
    
    hinge_bracket = (
        cq.Workplane("XY")
        .workplane(offset=hinge_z)
        .center(0, hinge_y - 0.02)
        .rect(2 * top_x_r + 0.04, 0.08)
        .extrude(H - hinge_z)
    )
    
    front_metal = front_rails.union(steps).union(hinge_cyl).union(hinge_bracket)
    
    # Top Cap
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=H)
        .center(0, dy_f + 0.02)
        .rect(0.36, 0.16)
        .extrude(0.04)
    )
    
    # Front feet
    front_feet = (
        cq.Workplane("XY")
        .box(rail_w + 0.01, rail_d + 0.01, 0.05).translate((bot_x, 0, 0.025))
        .union(cq.Workplane("XY").box(rail_w + 0.01, rail_d + 0.01, 0.05).translate((-bot_x, 0, 0.025)))
    )
    
    front_part = model.part("front_frame")
    mat_metal = Material(name="metal", color=(0.8, 0.8, 0.82))
    mat_plastic_black = Material(name="plastic_black", color=(0.1, 0.1, 0.1))
    mat_plastic_blue = Material(name="plastic_blue", color=(0.1, 0.3, 0.7))
    
    front_part.visual(mesh_from_cadquery(front_metal, "front_metal"), name="metal", material=mat_metal)
    front_part.visual(mesh_from_cadquery(top_cap, "top_cap"), name="top_cap", material=mat_plastic_blue)
    front_part.visual(mesh_from_cadquery(front_feet, "front_feet"), name="feet", material=mat_plastic_black)
    
    # Rear Frame
    theta_open = math.radians(15)
    L_rear = hinge_z / math.cos(theta_open)
    
    bot_x_r = 0.24
    rail_w_r = 0.025
    rail_d_r = 0.04
    
    rear_right = (
        cq.Workplane("XY")
        .workplane(offset=0.015).center(top_x_r, 0).rect(rail_w_r, rail_d_r)
        .workplane(offset=-L_rear - 0.015).center(bot_x_r - top_x_r, 0).rect(rail_w_r, rail_d_r)
        .loft()
    )
    rear_left = (
        cq.Workplane("XY")
        .workplane(offset=0.015).center(-top_x_r, 0).rect(rail_w_r, rail_d_r)
        .workplane(offset=-L_rear - 0.015).center(-bot_x_r + top_x_r, 0).rect(rail_w_r, rail_d_r)
        .loft()
    )
    rear_rails = rear_right.union(rear_left)
    
    braces = None
    for i in range(1, 4):
        z = -L_rear * (i / 4.0)
        t = abs(z) / L_rear
        brace_x = top_x_r + t * (bot_x_r - top_x_r)
        brace_w = 2 * brace_x + rail_w_r + 0.002
        brace = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .center(0, 0)
            .rect(brace_w, rail_d_r)
            .extrude(0.025)
        )
        if braces is None:
            braces = brace
        else:
            braces = braces.union(brace)
            
    rear_metal = rear_rails.union(braces)
    
    rear_feet = (
        cq.Workplane("XY")
        .box(rail_w_r + 0.01, rail_d_r + 0.01, 0.05).translate((bot_x_r, 0, -L_rear + 0.025))
        .union(cq.Workplane("XY").box(rail_w_r + 0.01, rail_d_r + 0.01, 0.05).translate((-bot_x_r, 0, -L_rear + 0.025)))
    )
    
    rear_part = model.part("rear_frame")
    rear_part.visual(mesh_from_cadquery(rear_metal, "rear_metal"), name="metal", material=mat_metal)
    rear_part.visual(mesh_from_cadquery(rear_feet, "rear_feet"), name="feet", material=mat_plastic_black)
    
    # Articulation
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=front_part,
        child=rear_part,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=theta_open)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("fold_hinge")
    
    ctx.allow_overlap(rear, front, elem_a="metal", elem_b="metal", reason="Rear rails wrap around the hinge cylinder on the front frame.")
    
    ctx.expect_gap(rear, front, axis="y", min_gap=0.15, positive_elem="feet", negative_elem="feet")
    
    with ctx.pose({hinge: math.radians(15)}):
        aabb = ctx.part_world_aabb(rear)
        ctx.check("rear_frame_swings_back", aabb is not None and aabb[1][1] > 0.6, "Rear frame should swing back significantly")
        
    return ctx.report()

object_model = build_object_model()