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
    model = ArticulatedObject(name="pencil_sharpener")

    # Body
    body_cq = (
        cq.Workplane("XY")
        .box(0.070, 0.100, 0.100)
        .translate((0, 0, 0.050))
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .edges()
        .fillet(0.010)
    )
    
    # Drawer cavity
    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.056, 0.052, 0.046)
        .translate((0, 0.025, 0.025))
    )
    body_cq = body_cq.cut(drawer_cavity)
    
    # Pencil cavity (cone)
    pencil_cavity = (
        cq.Workplane("XZ")
        .center(0, 0.065)
        .workplane(offset=-0.050) # Y = 0.050 (front)
        .circle(0.015)
        .workplane(offset=0.040) # Y = 0.010
        .circle(0.002)
        .loft(combine=True)
    )
    body_cq = body_cq.cut(pencil_cavity)

    # Selector dial recess
    recess = (
        cq.Workplane("XZ")
        .workplane(offset=-0.051) # Y = 0.051
        .center(0, 0.084)
        .circle(0.011)
        .extrude(0.002) # to Y = 0.049
    )
    body_cq = body_cq.cut(recess)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_cq, "body_mesh"),
        origin=Origin(),
        name="body_vis",
        color=(0.4, 0.4, 0.45),
    )

    # Drawer
    drawer_cq = (
        cq.Workplane("XY")
        .box(0.054, 0.050, 0.043)
        .translate((0, 0.025, 0.0245))
    )
    drawer_inner = (
        cq.Workplane("XY")
        .box(0.050, 0.046, 0.043)
        .translate((0, 0.025, 0.0265))
    )
    drawer_cq = drawer_cq.cut(drawer_inner)
    
    drawer_handle = (
        cq.Workplane("XY")
        .box(0.020, 0.005, 0.010)
        .translate((0, 0.0525, 0.024))
    )
    drawer_cq = drawer_cq.union(drawer_handle)

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(drawer_cq, "drawer_mesh"),
        origin=Origin(),
        name="drawer_vis",
        color=(0.2, 0.2, 0.2),
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.060),
    )

    # Selector dial
    selector_cq = (
        cq.Workplane("XZ")
        .circle(0.010)
        .extrude(-0.005) # to Y = 0.005
    )
    holes = (
        cq.Workplane("XZ")
        .polarArray(0.007, 0, 360, 6)
        .circle(0.0015)
        .extrude(-0.005)
    )
    selector_cq = selector_cq.cut(holes)
    
    pointer = (
        cq.Workplane("XZ")
        .workplane(offset=-0.005) # Y = 0.005
        .center(0, 0.007)
        .circle(0.0015)
        .extrude(-0.002) # to Y = 0.007
    )
    selector_cq = selector_cq.union(pointer)

    selector = model.part("selector")
    selector.visual(
        mesh_from_cadquery(selector_cq, "selector_mesh"),
        origin=Origin(), # Local to part frame
        name="selector_vis",
        color=(0.8, 0.8, 0.8),
    )

    model.articulation(
        "selector_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, 0.049, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    # Crank
    crank_hub = (
        cq.Workplane("YZ")
        .workplane(offset=-0.002)
        .circle(0.008)
        .extrude(0.007)
    )
    crank_arm = (
        cq.Workplane("YZ")
        .workplane(offset=0.005)
        .center(0, 0.015)
        .rect(0.012, 0.040)
        .extrude(0.005)
    )
    crank_cq = crank_hub.union(crank_arm)

    crank = model.part("crank")
    crank.visual(
        mesh_from_cadquery(crank_cq, "crank_mesh"),
        origin=Origin(), # Local to part frame
        name="crank_vis",
        color=(0.7, 0.7, 0.7),
    )

    model.articulation(
        "crank_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.035, 0.0, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    # Spinner knob
    spinner_cq = (
        cq.Workplane("YZ")
        .workplane(offset=-0.002)
        .circle(0.006)
        .extrude(0.017)
        .edges(">X")
        .fillet(0.002)
    )
    spinner = model.part("spinner")
    spinner.visual(
        mesh_from_cadquery(spinner_cq, "spinner_mesh"),
        origin=Origin(), # Local to part frame
        name="spinner_vis",
        color=(0.1, 0.1, 0.1),
    )

    model.articulation(
        "spinner_turn",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=spinner,
        origin=Origin(xyz=(0.010, 0.0, 0.030)), # Relative to crank
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap("crank", "body", reason="Crank axle embeds into the body for mounting")
    ctx.allow_overlap("spinner", "crank", reason="Spinner axle embeds into the crank arm")
    ctx.allow_overlap("selector", "body", reason="Selector dial embeds into the front recess")
    ctx.allow_isolated_part("drawer", reason="Drawer slides loosely in the cavity")

    ctx.expect_within("drawer", "body", axes="xz", name="Drawer remains centered in the cavity")
    
    with ctx.pose(drawer_slide=0.060):
        ctx.expect_gap("drawer", "body", axis="y", min_gap=0.005, name="Drawer pulls out from the front")

    return ctx.report()

object_model = build_object_model()
