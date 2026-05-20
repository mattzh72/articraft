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

def build_lower_link_geom() -> cq.Workplane:
    link = (
        cq.Workplane("XY")
        .cylinder(0.05, 0.03)
        .union(cq.Workplane("XY").center(0.2, 0).cylinder(0.05, 0.025))
        .union(cq.Workplane("XY").center(0.1, 0).box(0.2, 0.04, 0.05))
    )
    post_hole = cq.Workplane("XY").cylinder(0.1, 0.021)
    pin_hole = cq.Workplane("XY").center(0.2, 0).cylinder(0.1, 0.011)
    return link.cut(post_hole).cut(pin_hole)

def build_outer_link_geom() -> cq.Workplane:
    link = (
        cq.Workplane("XY")
        .cylinder(0.05, 0.025)
        .union(cq.Workplane("XY").center(0.2, 0).cylinder(0.05, 0.025))
        .union(cq.Workplane("XY").center(0.1, 0).box(0.2, 0.035, 0.05))
    )
    pin = cq.Workplane("XY").workplane(offset=-0.05).cylinder(0.05, 0.01)
    pin_hole = cq.Workplane("XY").center(0.2, 0).cylinder(0.1, 0.011)
    return link.union(pin).cut(pin_hole)

def build_pan_support_geom() -> cq.Workplane:
    base = cq.Workplane("XY").cylinder(0.02, 0.025)
    pin = cq.Workplane("XY").workplane(offset=-0.035).cylinder(0.05, 0.01)
    # Ensure solid unions with slight overlaps
    body = cq.Workplane("XY").workplane(offset=0.029).center(0.014, 0).box(0.032, 0.05, 0.042)
    ear1 = cq.Workplane("XY").workplane(offset=0.03).center(0.039, -0.02).box(0.022, 0.01, 0.04)
    ear2 = cq.Workplane("XY").workplane(offset=0.03).center(0.039, 0.02).box(0.022, 0.01, 0.04)
    tilt_hole = cq.Workplane("XZ").center(0.04, 0.03).cylinder(0.1, 0.006)
    return base.union(pin).union(body).union(ear1).union(ear2).cut(tilt_hole)

def build_head_frame_geom() -> cq.Workplane:
    pin = cq.Workplane("XZ").cylinder(0.05, 0.005)
    # Ensure solid unions
    standoff = cq.Workplane("XY").center(0.01, 0).box(0.022, 0.028, 0.02)
    vesa = cq.Workplane("XY").center(0.024, 0).box(0.01, 0.12, 0.12)
    return pin.union(standoff).union(vesa)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_mount")

    # Post base
    post_base = model.part("post_base")
    post_base.visual(Box((0.1, 0.1, 0.05)), origin=Origin(xyz=(0, 0, 0.025)), name="clamp")
    post_base.visual(Cylinder(radius=0.02, height=0.4), origin=Origin(xyz=(0, 0, 0.25)), name="post")

    # Lower link
    lower_link = model.part("lower_link")
    lower_link.visual(mesh_from_cadquery(build_lower_link_geom(), "lower_link_geom"), name="geom")

    model.articulation(
        "post_to_lower",
        ArticulationType.REVOLUTE,
        parent=post_base,
        child=lower_link,
        origin=Origin(xyz=(0, 0, 0.25)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14)
    )

    # Outer link
    outer_link = model.part("outer_link")
    outer_link.visual(mesh_from_cadquery(build_outer_link_geom(), "outer_link_geom"), name="geom")

    model.articulation(
        "lower_to_outer",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=outer_link,
        origin=Origin(xyz=(0.2, 0, 0.051)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-2.5, upper=2.5)
    )

    # Head pan support
    head_pan_support = model.part("head_pan_support")
    head_pan_support.visual(mesh_from_cadquery(build_pan_support_geom(), "pan_support_geom"), name="geom")

    model.articulation(
        "outer_to_pan",
        ArticulationType.REVOLUTE,
        parent=outer_link,
        child=head_pan_support,
        origin=Origin(xyz=(0.2, 0, 0.036)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-1.5, upper=1.5)
    )

    # Head frame
    head_frame = model.part("head_frame")
    head_frame.visual(mesh_from_cadquery(build_head_frame_geom(), "head_frame_geom"), name="geom")

    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=head_pan_support,
        child=head_frame,
        origin=Origin(xyz=(0.04, 0, 0.03)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.5, upper=0.5)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # We rely on the baseline QC to ensure there are no real 3D overlaps.
    # The pin-in-hole joints are designed with 0.001m gap clearances.
    ctx.allow_isolated_part("lower_link", reason="Slides around post with clearance")
    ctx.allow_isolated_part("outer_link", reason="Hinge pin has clearance")
    ctx.allow_isolated_part("head_pan_support", reason="Hinge pin has clearance")
    ctx.allow_isolated_part("head_frame", reason="Tilt pin has clearance")
    
    return ctx.report()

object_model = build_object_model()
