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
    model = ArticulatedObject(name="tripod")

    # --- HUB ---
    hub = model.part("hub")
    
    # Hub geometry: central cylinder + 3 clevis mounts
    hub_cq = cq.Workplane("XY").cylinder(0.06, 0.035) # Z from -0.03 to 0.03
    for i in range(3):
        angle = i * 120
        mount_side1 = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, angle))
            .transformed(offset=cq.Vector(0.045, 0.022, 0))
            .box(0.03, 0.01, 0.04) # Z from -0.02 to 0.02
        )
        mount_side2 = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, angle))
            .transformed(offset=cq.Vector(0.045, -0.022, 0))
            .box(0.03, 0.01, 0.04)
        )
        hub_cq = hub_cq.union(mount_side1).union(mount_side2)
    # Cut central hole for column
    hub_cq = hub_cq.faces(">Z").hole(0.04) # diameter 0.04 -> radius 0.02
    
    hub.visual(
        mesh_from_cadquery(hub_cq, "hub_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="hub_visual",
        color=(0.2, 0.2, 0.2, 1.0)
    )

    # --- LEGS ---
    # Leg geometry
    leg_cq = (
        cq.Workplane("XY")
        .workplane(offset=-1.0)
        .cylinder(1.0, 0.015, centered=(True, True, False)) # Z from -1.0 to 0
    )
    # Add a rounded foot
    foot = cq.Workplane("XY").workplane(offset=-1.0).sphere(0.02)
    leg_cq = leg_cq.union(foot)
    # Hinge barrel at the top
    hinge_barrel = (
        cq.Workplane("XZ") 
        .cylinder(0.034, 0.015) # along Y, length 0.034, radius 0.015
    )
    leg_cq = leg_cq.union(hinge_barrel)

    leg_angles = [0, 120, 240]
    
    for i, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{i}")
        leg.visual(
            mesh_from_cadquery(leg_cq, f"leg_mesh_{i}"),
            name=f"leg_visual_{i}",
            color=(0.1, 0.1, 0.1, 1.0)
        )
        
        rad = math.radians(angle)
        hx = 0.055 * math.cos(rad)
        hy = 0.055 * math.sin(rad)
        
        model.articulation(
            f"hub_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=leg,
            origin=Origin(xyz=(hx, hy, 0.0), rpy=(0.0, 0.0, rad)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=1.0)
        )

    # --- COLUMN ---
    column = model.part("column")
    col_cq = (
        cq.Workplane("XY")
        .cylinder(0.7, 0.02) # Centered at 0, goes from -0.35 to 0.35
    )
    stopper = (
        cq.Workplane("XY")
        .workplane(offset=-0.35)
        .cylinder(0.02, 0.025, centered=(True, True, False)) # from -0.35 to -0.33
    )
    col_cq = col_cq.union(stopper)
    
    column.visual(
        mesh_from_cadquery(col_cq, "column_mesh"),
        origin=Origin(xyz=(0, 0, 0.05)), # shifts visual to -0.3 to 0.4
        name="column_visual",
        color=(0.7, 0.7, 0.7, 1.0)
    )
    
    model.articulation(
        "hub_to_column",
        ArticulationType.PRISMATIC,
        parent=hub,
        child=column,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=-0.35, upper=0.24)
    )

    # --- PAN BASE ---
    pan_base = model.part("pan_base")
    pan_cq = (
        cq.Workplane("XY")
        .cylinder(0.02, 0.03, centered=(True, True, False)) # z=0 to 0.02
    )
    sides = (
        cq.Workplane("XY")
        .workplane(offset=0.02)
        .pushPoints([(0, 0.025), (0, -0.025)])
        .box(0.04, 0.01, 0.07, centered=(True, True, False)) # z=0.02 to 0.09
    )
    pan_cq = pan_cq.union(sides)
    
    pan_base.visual(
        mesh_from_cadquery(pan_cq, "pan_base_mesh"),
        name="pan_base_visual",
        color=(0.15, 0.15, 0.15, 1.0)
    )
    
    model.articulation(
        "column_to_pan",
        ArticulationType.REVOLUTE,
        parent=column,
        child=pan_base,
        origin=Origin(xyz=(0, 0, 0.4)), 
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-3.14, upper=3.14)
    )

    # --- CAMERA ---
    camera = model.part("camera")
    cam_cq = (
        cq.Workplane("XY")
        .box(0.12, 0.04, 0.06, centered=(True, True, True))
    )
    lens = (
        cq.Workplane("YZ", origin=(0.06, 0, 0))
        .workplane(offset=0)
        .cylinder(0.08, 0.025, centered=(True, True, False)) # extends in +X
    )
    cam_cq = cam_cq.union(lens)
    
    camera.visual(
        mesh_from_cadquery(cam_cq, "camera_mesh"),
        name="camera_visual",
        color=(0.05, 0.05, 0.05, 1.0)
    )
    
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_base,
        child=camera,
        origin=Origin(xyz=(0, 0, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.785, upper=0.785)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    hub = object_model.get_part("hub")
    column = object_model.get_part("column")
    pan_base = object_model.get_part("pan_base")
    camera = object_model.get_part("camera")
    
    ctx.allow_overlap(
        column, hub,
        reason="Column slides through the central hole in the hub."
    )
    
    ctx.expect_within(
        column, hub,
        axes="xy",
        margin=0.001,
        name="column stays centered in the hub hole"
    )
    
    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            leg, hub,
            reason="Leg hinge pin sits inside the hub clevis."
        )
        ctx.expect_contact(leg, hub, contact_tol=0.005, name=f"leg_{i} is mounted to hub")
        
    ctx.allow_overlap(
        camera, pan_base,
        reason="Camera is mounted inside the U-bracket of the pan base."
    )
    ctx.expect_contact(pan_base, column, contact_tol=0.001, name="pan base is mounted to column")
    ctx.expect_contact(camera, pan_base, contact_tol=0.001, name="camera is mounted to pan base")
    
    return ctx.report()

object_model = build_object_model()