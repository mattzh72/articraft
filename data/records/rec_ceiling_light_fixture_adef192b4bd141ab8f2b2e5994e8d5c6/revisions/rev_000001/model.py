import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_light")
    
    # --- Base Part ---
    base = model.part("base")
    
    black_metal = Material(name="black_metal", color=(0.15, 0.15, 0.15))
    
    # Canopy
    base.visual(
        Cylinder(radius=0.06, height=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        name="canopy",
        color=black_metal,
    )
    # Stem
    base.visual(
        Cylinder(radius=0.012, height=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        name="stem",
        color=black_metal,
    )
    # Housing Top
    base.visual(
        Cylinder(radius=0.20, height=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0675)),
        name="housing_top",
        color=black_metal,
    )
    # Housing Shell
    # Extrudes layered rings from Z=0 to Z=0.12. We place it at Z=-0.19, so it covers [-0.19, -0.07]
    shell_cq = (
        cq.Workplane("XY")
        .circle(0.20).circle(0.195).extrude(0.03)
        .faces(">Z").workplane(offset=0.01)
        .circle(0.20).circle(0.195).extrude(0.03)
        .faces(">Z").workplane(offset=0.01)
        .circle(0.20).circle(0.195).extrude(0.04)
    )
    struts = (
        cq.Workplane("XY")
        .polarArray(0.1975, 0, 360, 4)
        .rect(0.005, 0.005)
        .extrude(0.12)
    )
    shell_cq = shell_cq.union(struts)
    
    base.visual(
        mesh_from_cadquery(shell_cq, "housing_shell"),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        name="housing_shell",
        color=black_metal,
    )
    # Socket
    base.visual(
        Cylinder(radius=0.015, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        name="socket",
        color=(0.8, 0.8, 0.8),
    )
    # Bulb
    base.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        name="bulb",
        color=(1.0, 0.95, 0.8),
    )
    
    # Latch receiver on base
    base.visual(
        Box((0.01, 0.02, 0.01)),
        origin=Origin(xyz=(-0.204, 0.0, -0.185)),
        name="latch_receiver",
        color=black_metal,
    )
    
    # Hinge knuckles on base
    base.visual(
        Cylinder(radius=0.005, height=0.01),
        origin=Origin(xyz=(0.204, 0.015, -0.1905), rpy=(1.5708, 0.0, 0.0)),
        name="hinge_knuckle_base_1",
        color=black_metal,
    )
    base.visual(
        Cylinder(radius=0.005, height=0.01),
        origin=Origin(xyz=(0.204, -0.015, -0.1905), rpy=(1.5708, 0.0, 0.0)),
        name="hinge_knuckle_base_2",
        color=black_metal,
    )
    
    # --- Diffuser Door Part ---
    door = model.part("diffuser_door")
    
    # Door Frame
    # Extrudes 0.009 from Z=0 to Z=0.009.
    # Local origin = (0, 0, -0.200) - (0.204, 0, -0.1905) = (-0.204, 0, -0.0095)
    frame_cq = cq.Workplane("XY").circle(0.20).circle(0.18).extrude(0.009)
    door.visual(
        mesh_from_cadquery(frame_cq, "door_frame"),
        origin=Origin(xyz=(-0.204, 0.0, -0.0095)),
        name="door_frame",
        color=black_metal,
    )
    # Diffuser Panel
    # Local origin = (0, 0, -0.1955) - (0.204, 0, -0.1905) = (-0.204, 0, -0.005)
    door.visual(
        Cylinder(radius=0.182, height=0.005),
        origin=Origin(xyz=(-0.204, 0.0, -0.005)),
        name="diffuser_panel",
        color=(0.9, 0.9, 0.9, 0.8),
    )
    
    # Latch tab on door
    # Local origin = (-0.204, 0, -0.1955) - (0.204, 0, -0.1905) = (-0.408, 0, -0.005)
    door.visual(
        Box((0.01, 0.02, 0.009)),
        origin=Origin(xyz=(-0.408, 0.0, -0.005)),
        name="latch_tab",
        color=black_metal,
    )
    
    # Hinge knuckle on door
    # Local origin = (0.204, 0, -0.1905) - (0.204, 0, -0.1905) = (0, 0, 0)
    door.visual(
        Cylinder(radius=0.005, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5708, 0.0, 0.0)),
        name="hinge_knuckle_door",
        color=black_metal,
    )
    
    # --- Articulation ---
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=door,
        origin=Origin(xyz=(0.204, 0.0, -0.1905)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=1.0, velocity=1.0),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    door = object_model.get_part("diffuser_door")
    hinge = object_model.get_articulation("door_hinge")
    
    # At rest, the door should be closed with a realistic tiny gap.
    ctx.expect_gap(
        base, door,
        axis="z",
        positive_elem="housing_shell",
        negative_elem="door_frame",
        max_penetration=0.0,
        max_gap=0.002,
        name="Door frame is flush with housing shell at rest"
    )
    
    ctx.expect_gap(
        base, door,
        axis="z",
        positive_elem="latch_receiver",
        negative_elem="latch_tab",
        max_penetration=0.0,
        max_gap=0.002,
        name="Latch tab aligns with latch receiver"
    )
    
    # When opened, the door should swing down cleanly.
    with ctx.pose({hinge: 1.5}):
        door_aabb = ctx.part_world_aabb(door)
        if door_aabb:
            ctx.check(
                "Door swings down when opened",
                door_aabb[0][2] < -0.3,
                details=f"Door min Z is {door_aabb[0][2]}"
            )
            
    return ctx.report()

object_model = build_object_model()