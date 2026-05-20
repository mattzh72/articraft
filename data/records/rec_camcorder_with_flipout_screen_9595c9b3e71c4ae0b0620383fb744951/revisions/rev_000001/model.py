from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camcorder")

    # --- Geometry Generation ---
    # Main body: X: -0.06 to 0.06. Y: 0 to 0.04. Z: 0 to 0.06.
    body_shape = (
        cq.Workplane("XY")
        .box(0.12, 0.04, 0.06)
        .translate((0, 0.02, 0.03))
    )
    
    # Grip: X: -0.06 to 0.04. Y: -0.03 to 0. Z: 0 to 0.05.
    grip_shape = (
        cq.Workplane("XY")
        .box(0.10, 0.03, 0.05)
        .edges("|Z")
        .fillet(0.01)
        .translate((-0.01, -0.015, 0.025))
    )
    
    body_shape = body_shape.edges("|Z").fillet(0.005)
    body_shape = body_shape.union(grip_shape)
    
    # Recess for monitor
    # Monitor: 0.07 x 0.008 x 0.035.
    # Flush at Y=0.040. So Y bounds: 0.032 to 0.040.
    # Cut Y bounds: 0.032 to 0.048. Center Y = 0.040, width = 0.016.
    # X center = -0.015. Z center = 0.0375.
    recess = (
        cq.Workplane("XY")
        .box(0.072, 0.016, 0.037)
        .translate((-0.015, 0.04, 0.0375))
    )
    body_shape = body_shape.cut(recess)
    
    # Recess for port cover
    # Cover: 0.02 x 0.004 x 0.016.
    # Flush at Y=0.040. Y bounds: 0.036 to 0.040.
    # Cut Y bounds: 0.036 to 0.044. Center Y = 0.040, width = 0.008.
    # X center = 0.02. Z center = 0.010.
    port_recess = (
        cq.Workplane("XY")
        .box(0.022, 0.008, 0.018)
        .translate((0.02, 0.04, 0.010))
    )
    body_shape = body_shape.cut(port_recess)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shape, "body_mesh"),
        name="body_shell"
    )

    # --- Lens Barrel ---
    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(0.07, 0.02, 0.03), rpy=(0, 1.5708, 0)),
        name="barrel_shell"
    )
    
    model.articulation(
        "body_to_barrel",
        ArticulationType.FIXED,
        parent=body,
        child=lens_barrel,
        origin=Origin()
    )

    # --- Lens Ring ---
    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        Cylinder(radius=0.022, length=0.015),
        origin=Origin(xyz=(0.0075, 0, 0), rpy=(0, 1.5708, 0)),
        name="ring_shell"
    )
    
    model.articulation(
        "barrel_to_ring",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=lens_ring,
        origin=Origin(xyz=(0.08, 0.02, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )

    # --- Side Monitor ---
    side_monitor = model.part("side_monitor")
    side_monitor.visual(
        Box((0.07, 0.008, 0.035)),
        origin=Origin(xyz=(-0.035, -0.004, 0.0)),
        name="monitor_shell"
    )
    
    model.articulation(
        "body_to_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_monitor,
        origin=Origin(xyz=(0.02, 0.04, 0.0375)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=2.5)
    )

    # --- Port Cover ---
    port_cover = model.part("port_cover")
    port_cover.visual(
        Box((0.02, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.002, 0.008)),
        name="cover_shell"
    )
    
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=port_cover,
        origin=Origin(xyz=(0.02, 0.04, 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.57)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    monitor = object_model.get_part("side_monitor")
    cover = object_model.get_part("port_cover")
    
    monitor_joint = object_model.get_articulation("body_to_monitor")
    cover_joint = object_model.get_articulation("body_to_cover")
    
    ctx.allow_overlap(
        body, monitor,
        elem_a="body_shell", elem_b="monitor_shell",
        reason="The monitor sits inside a recess in the body, which may be filled by the convex collision proxy."
    )
    ctx.allow_overlap(
        body, cover,
        elem_a="body_shell", elem_b="cover_shell",
        reason="The port cover sits inside a recess in the body, which may be filled by the convex collision proxy."
    )
    
    ctx.expect_contact(monitor, body, name="monitor sits flush in recess")
    ctx.expect_contact(cover, body, name="cover sits flush in recess")
    
    with ctx.pose({monitor_joint: 1.57}):
        ctx.expect_gap(monitor, body, axis="y", max_penetration=0.001, name="monitor opens outward")
        
    with ctx.pose({cover_joint: 1.57}):
        ctx.expect_gap(cover, body, axis="y", max_penetration=0.001, name="cover opens outward")
        
    return ctx.report()

object_model = build_object_model()
