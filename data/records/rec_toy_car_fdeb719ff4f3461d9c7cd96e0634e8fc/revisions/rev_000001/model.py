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
    model = ArticulatedObject(name="toy_coupe")

    # --- Geometry Generation ---
    
    # Car body
    # Base: 0.30 x 0.12 x 0.04 (Z: 0.01 to 0.05)
    body_cq = cq.Workplane("XY").box(0.30, 0.12, 0.04).translate((0, 0, 0.03))
    
    # Cabin: 0.12 x 0.08 x 0.04 (Z: 0.05 to 0.09)
    cabin = cq.Workplane("XY").box(0.12, 0.08, 0.04).translate((-0.02, 0, 0.07))
    body_cq = body_cq.union(cabin)
    
    # Interior cutout
    interior = cq.Workplane("XY").box(0.10, 0.06, 0.06).translate((-0.02, 0, 0.06))
    body_cq = body_cq.cut(interior)
    
    # Windshield and rear window cuts
    windshield_cut = cq.Workplane("YZ").box(0.06, 0.03, 0.04).translate((0.03, 0, 0.065))
    body_cq = body_cq.cut(windshield_cut)
    
    rear_window_cut = cq.Workplane("YZ").box(0.06, 0.03, 0.04).translate((-0.07, 0, 0.065))
    body_cq = body_cq.cut(rear_window_cut)
    
    # Door cuts
    door_cut = cq.Workplane("XY").box(0.08, 0.14, 0.06).translate((-0.02, 0, 0.05))
    body_cq = body_cq.cut(door_cut)
    
    # Wheel arches
    arch_cut = cq.Workplane("XZ").cylinder(0.04, 0.04)
    body_cq = body_cq.cut(arch_cut.translate((0.10, 0.06, 0.01)))
    body_cq = body_cq.cut(arch_cut.translate((0.10, -0.06, 0.01)))
    body_cq = body_cq.cut(arch_cut.translate((-0.10, 0.06, 0.01)))
    body_cq = body_cq.cut(arch_cut.translate((-0.10, -0.06, 0.01)))
    
    # Axles
    axle = cq.Workplane("XZ").cylinder(0.14, 0.005)
    body_cq = body_cq.union(axle.translate((0.10, 0, 0.01)))
    body_cq = body_cq.union(axle.translate((-0.10, 0, 0.01)))

    # Door geometry
    # 0.08 long, 0.03 thick, 0.06 high
    door_cq = cq.Workplane("XY").box(0.08, 0.03, 0.06)
    hinge_barrel = cq.Workplane("XY").cylinder(0.06, 0.005).translate((0.04, 0, 0))
    door_cq = door_cq.union(hinge_barrel)

    # Wheel geometry
    # Radius 0.035, width 0.02
    wheel_cq = cq.Workplane("XZ").cylinder(0.02, 0.035)

    # --- Parts and Articulations ---
    
    body = model.part("body")
    body.visual(mesh_from_cadquery(body_cq, "body_mesh"), name="shell")

    # Wheels
    wheel_positions = {
        "front_left_wheel": (0.10, 0.06, 0.01),
        "front_right_wheel": (0.10, -0.06, 0.01),
        "rear_left_wheel": (-0.10, 0.06, 0.01),
        "rear_right_wheel": (-0.10, -0.06, 0.01),
    }
    
    for name, pos in wheel_positions.items():
        wheel = model.part(name)
        wheel.visual(mesh_from_cadquery(wheel_cq, f"{name}_mesh"), name="tire")
        model.articulation(
            name=f"body_to_{name}",
            articulation_type=ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=pos),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0),
        )

    # Left door
    left_door = model.part("left_door")
    left_door.visual(
        mesh_from_cadquery(door_cq, "left_door_mesh"),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        name="panel"
    )
    model.articulation(
        name="body_to_left_door",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(0.02, 0.045, 0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.5),
    )

    # Right door
    right_door = model.part("right_door")
    right_door.visual(
        mesh_from_cadquery(door_cq, "right_door_mesh"),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        name="panel"
    )
    model.articulation(
        name="body_to_right_door",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(0.02, -0.045, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    
    for name in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
        wheel = object_model.get_part(name)
        ctx.allow_overlap(body, wheel, reason="Axle is captured inside the wheel hub.")
        ctx.expect_within(wheel, body, axes="x", margin=0.05)
        
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    
    ctx.allow_overlap(body, left_door, reason="Door hinge barrel is embedded in the A-pillar.")
    ctx.allow_overlap(body, right_door, reason="Door hinge barrel is embedded in the A-pillar.")
    
    ctx.expect_within(left_door, body, axes="xz", margin=0.01)
    ctx.expect_within(right_door, body, axes="xz", margin=0.01)
    
    with ctx.pose(body_to_left_door=1.5):
        aabb = ctx.link_world_aabb(left_door)
        if aabb:
            ctx.check("left_door_opens_outward", aabb[1][1] > 0.08, f"Left door max Y: {aabb[1][1]}")
            
    with ctx.pose(body_to_right_door=1.5):
        aabb = ctx.link_world_aabb(right_door)
        if aabb:
            ctx.check("right_door_opens_outward", aabb[0][1] < -0.08, f"Right door min Y: {aabb[0][1]}")

    return ctx.report()


object_model = build_object_model()