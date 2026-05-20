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
    model = ArticulatedObject(name="washing_machine")
    
    # --- Body ---
    body = model.part("body")
    
    body_box = cq.Workplane("XY").box(0.70, 0.80, 0.95).translate((0, 0, 0.525))
    
    feet = (
        cq.Workplane("XY").workplane(offset=0.025)
        .rect(0.60, 0.70, forConstruction=True)
        .vertices()
        .cylinder(0.05, 0.03)
    )
    
    door_hole_cutter = cq.Workplane("XZ").cylinder(0.32, 0.26).translate((0, -0.26, 0.45))
    drawer_hole_cutter = cq.Workplane("XY").box(0.22, 0.28, 0.12).translate((-0.20, -0.28, 0.88))
    
    body_cq = body_box.union(feet).cut(door_hole_cutter).cut(drawer_hole_cutter)
    
    dial = cq.Workplane("XZ").cylinder(0.02, 0.04).translate((0.15, -0.41, 0.88))
    body_cq = body_cq.union(dial)
    
    body.visual(
        mesh_from_cadquery(body_cq, "body_mesh"),
        name="body_visual"
    )
    
    # --- Door ---
    door = model.part("door")
    
    door_ring = cq.Workplane("XZ").cylinder(0.04, 0.28).translate((0, -0.02, 0))
    inner_cutter = cq.Workplane("XZ").cylinder(0.06, 0.20).translate((0, -0.02, 0))
    door_ring = door_ring.cut(inner_cutter)
    
    door_dome = cq.Workplane("XZ").cylinder(0.15, 0.20).translate((0, 0.075, 0))
    door_cq = door_ring.union(door_dome)
    
    door.visual(
        mesh_from_cadquery(door_cq, "door_mesh"),
        origin=Origin(xyz=(0.28, 0, 0)),
        name="door_visual"
    )
    
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.28, -0.40, 0.45)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=2.0),
    )
    
    # --- Drawer ---
    drawer = model.part("drawer")
    
    drawer_front_cq = cq.Workplane("XY").box(0.23, 0.02, 0.13).translate((0, -0.01, 0))
    drawer_body_cq = cq.Workplane("XY").box(0.21, 0.24, 0.11).translate((0, 0.12, 0))
    handle_cq = cq.Workplane("XY").box(0.10, 0.02, 0.04).translate((0, -0.03, 0))
    
    drawer_cq = drawer_front_cq.union(drawer_body_cq).union(handle_cq)
    
    drawer.visual(
        mesh_from_cadquery(drawer_cq, "drawer_mesh"),
        name="drawer_visual"
    )
    
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.20, -0.40, 0.88)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.20),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")
    
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    
    ctx.allow_overlap(
        body, door,
        elem_a="body_visual",
        elem_b="door_visual",
        reason="Door dome is intentionally nested inside the body cavity."
    )
    ctx.allow_overlap(
        body, drawer,
        elem_a="body_visual",
        elem_b="drawer_visual",
        reason="Drawer is intentionally nested inside the body cavity."
    )
    
    # Drawer tests
    ctx.expect_within(drawer, body, axes="xz", margin=0.01, name="drawer_centered_in_hole")
    ctx.expect_overlap(drawer, body, axes="y", min_overlap=0.10, name="drawer_inserted_at_rest")
    
    with ctx.pose({drawer_joint: 0.20}):
        ctx.expect_overlap(drawer, body, axes="y", min_overlap=0.02, name="drawer_retained_at_extension")
        
    # Door tests
    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.5}):
        open_aabb = ctx.part_world_aabb(door)
        if rest_aabb is not None and open_aabb is not None:
            ctx.check("door_opens_outwards", open_aabb[0][1] < rest_aabb[0][1] - 0.10, "Door should move outwards (-Y)")
            
    return ctx.report()

object_model = build_object_model()
