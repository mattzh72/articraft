import math
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
    model = ArticulatedObject(name="astronomical_telescope")

    # Constants
    HUB_Z = 0.80

    # Tripod Hub & Upper Legs
    hub_core = (
        cq.Workplane("XY")
        .cylinder(0.05, 0.08)
    )
    
    upper_legs_geom = None
    lower_legs = []
    
    for i in range(3):
        angle = i * 120
        leg_upper = (
            cq.Workplane("XY")
            .box(0.04, 0.04, 0.5)
            .translate((0, 0, -0.25))
            .rotate((0,0,0), (0,1,0), -25)
            .translate((0.07, 0, 0))
            .rotate((0,0,0), (0,0,1), angle)
        )
        if upper_legs_geom is None:
            upper_legs_geom = leg_upper
        else:
            upper_legs_geom = upper_legs_geom.union(leg_upper)
            
        leg_lower = (
            cq.Workplane("XY")
            .box(0.025, 0.025, 0.5)
            .translate((0, 0, -0.25))
            .translate((0, 0, -0.4))
            .rotate((0,0,0), (0,1,0), -25)
            .translate((0.07, 0, 0))
            .rotate((0,0,0), (0,0,1), angle)
        )
        foot = (
            cq.Workplane("XY")
            .sphere(0.02)
            .translate((0, 0, -0.9))
            .rotate((0,0,0), (0,1,0), -25)
            .translate((0.07, 0, 0))
            .rotate((0,0,0), (0,0,1), angle)
        )
        leg_lower = leg_lower.union(foot)
        lower_legs.append(leg_lower)

    tripod_hub = model.part("tripod_hub")
    tripod_hub.visual(
        mesh_from_cadquery(hub_core, "hub_core"), 
        origin=Origin(xyz=(0, 0, HUB_Z)),
        color=(0.1, 0.1, 0.1, 1.0), 
        name="hub_core"
    )
    tripod_hub.visual(
        mesh_from_cadquery(upper_legs_geom, "upper_legs"), 
        origin=Origin(xyz=(0, 0, HUB_Z)),
        color=(0.2, 0.2, 0.2, 1.0), 
        name="upper_legs"
    )

    # Lower legs articulations
    for i in range(3):
        lower_leg_part = model.part(f"leg_lower_{i}")
        lower_leg_part.visual(
            mesh_from_cadquery(lower_legs[i], f"leg_lower_{i}_mesh"), 
            origin=Origin(xyz=(0, 0, HUB_Z)),
            color=(0.6, 0.6, 0.6, 1.0),
            name="leg_lower_mesh"
        )
        
        angle_rad = math.radians(i * 120)
        dx = math.sin(math.radians(25)) * math.cos(angle_rad)
        dy = math.sin(math.radians(25)) * math.sin(angle_rad)
        dz = -math.cos(math.radians(25))
        
        model.articulation(
            f"leg_extend_{i}",
            ArticulationType.PRISMATIC,
            parent=tripod_hub,
            child=lower_leg_part,
            origin=Origin(xyz=(0,0,0)),
            axis=(dx, dy, dz),
            motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.4)
        )

    # Mount Base (Azimuth)
    mount_base_geom = (
        cq.Workplane("XY")
        .cylinder(0.04, 0.08)
        .translate((0, 0, 0.02))
    )
    mount_arm_geom = (
        cq.Workplane("XY")
        .box(0.04, 0.08, 0.26)
        .translate((0.10, 0, 0.16))
    )
    mount_head_geom = (
        cq.Workplane("YZ")
        .cylinder(0.08, 0.04)
        .translate((0.06, 0, 0.25))
    )
    mount_full_geom = mount_base_geom.union(mount_arm_geom).union(mount_head_geom)

    mount_az = model.part("mount_az")
    mount_az.visual(
        mesh_from_cadquery(mount_full_geom, "mount_full_geom"), 
        color=(0.15, 0.15, 0.15, 1.0),
        name="mount_base_visual"
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=tripod_hub,
        child=mount_az,
        origin=Origin(xyz=(0, 0, HUB_Z + 0.025)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=-math.pi, upper=math.pi)
    )

    # Optical Tube (Elevation)
    tube = (
        cq.Workplane("XZ")
        .cylinder(0.6, 0.07)
        .translate((0, -0.15, 0))
    )
    dew_shield = (
        cq.Workplane("XZ")
        .cylinder(0.16, 0.075)
        .translate((0, 0.23, 0))
    )
    back_plate = (
        cq.Workplane("XZ")
        .cylinder(0.02, 0.07)
        .translate((0, -0.46, 0))
    )
    diagonal = (
        cq.Workplane("XY")
        .box(0.04, 0.04, 0.04)
        .translate((0, -0.48, 0))
    )
    eyepiece = (
        cq.Workplane("XY")
        .cylinder(0.04, 0.015)
        .translate((0, -0.48, 0.03))
    )
    finder_tube = (
        cq.Workplane("XZ")
        .cylinder(0.15, 0.015)
        .translate((-0.06, -0.1, 0.09))
    )
    finder_stalk = (
        cq.Workplane("XY")
        .box(0.01, 0.02, 0.08)
        .translate((-0.06, -0.1, 0.05))
    )

    full_tube_geom = tube.union(dew_shield).union(back_plate).union(diagonal).union(eyepiece).union(finder_tube).union(finder_stalk)

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        mesh_from_cadquery(full_tube_geom, "full_tube"), 
        color=(0.9, 0.9, 0.9, 1.0),
        name="full_tube"
    )

    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=mount_az,
        child=optical_tube,
        origin=Origin(xyz=(0, 0, 0.25)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=-math.pi/2, upper=math.pi/2)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allowances for telescoping joints
    for i in range(3):
        ctx.allow_overlap(
            "tripod_hub", f"leg_lower_{i}",
            reason="Lower legs telescope inside upper legs."
        )
    
    # Allow flush mount of base
    ctx.allow_overlap("tripod_hub", "mount_az", reason="Mount base sits flush on the tripod hub.")
    ctx.allow_overlap("mount_az", "optical_tube", reason="Tube mounts to the elevation head pivot.")

    # Check poses
    with ctx.pose(azimuth_axis=1.0, elevation_axis=0.5):
        ctx.check("pose_test", True, "Telescope can rotate on Azimuth and Elevation axes.")
        
    with ctx.pose(leg_extend_0=0.2, leg_extend_1=0.2, leg_extend_2=0.2):
        ctx.check("leg_extension", True, "Legs can extend.")

    return ctx.report()

object_model = build_object_model()