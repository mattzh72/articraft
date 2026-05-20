import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Box,
    Cylinder,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="washing_machine")
    
    # Dimensions
    cab_w = 0.68
    cab_d = 0.72
    cab_h = 0.90
    
    console_w = 0.68
    console_d = 0.12
    console_h = 0.20
    console_y = 0.30  # Center of console in Y
    
    tub_d = 0.54
    tub_h = 0.80
    tub_y = -0.04
    
    # 1. Cabinet
    # Base shell
    cab_shell = (
        cq.Workplane("XY")
        .box(cab_w, cab_d, cab_h, centered=(True, True, False))
        .shell(-0.01)
    )
    # Hole for tub in the top deck
    cab_deck_hole = (
        cq.Workplane("XY")
        .workplane(offset=cab_h)
        .center(0, tub_y)
        .circle(tub_d / 2 + 0.01)
        .extrude(-0.03)
    )
    
    cab_with_holes = cab_shell.cut(cab_deck_hole)
    
    # Tub support (motor housing)
    tub_support = (
        cq.Workplane("XY")
        .center(0, tub_y)
        .cylinder(cab_h - tub_h - 0.01, 0.1, centered=(True, True, False))
    )
    
    # Console
    console = (
        cq.Workplane("XY")
        .workplane(offset=cab_h)
        .center(0, console_y)
        .box(console_w, console_d, console_h, centered=(True, True, False))
    )
    
    cabinet_shape = cab_with_holes.union(console).union(tub_support)
    
    cabinet = model.part("cabinet")
    cabinet.visual(mesh_from_cadquery(cabinet_shape, "cabinet_mesh"), name="cabinet_body")
    
    # 2. Tub
    tub_shape = (
        cq.Workplane("XY")
        .cylinder(tub_h, tub_d / 2, centered=(True, True, False))
        .faces(">Z")
        .shell(-0.005)
    )
    tub = model.part("tub")
    tub.visual(mesh_from_cadquery(tub_shape, "tub_mesh"), name="tub_body")
    
    model.articulation(
        "tub_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0, tub_y, cab_h - tub_h - 0.01)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0)
    )
    
    # 3. Lid
    lid_w = 0.64
    lid_d = 0.58
    lid_h = 0.02
    hinge_y = 0.24
    
    lid_shape = (
        cq.Workplane("XY")
        .box(lid_w, lid_d, lid_h, centered=(True, True, False))
    )
    lid = model.part("lid")
    # Offset visual so the hinge is at the rear edge (+Y edge of the lid) and top edge (+Z)
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid_mesh"),
        origin=Origin(xyz=(0, -lid_d / 2, -lid_h)),
        name="lid_body"
    )
    
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0, hinge_y, cab_h + lid_h)),
        axis=(-1, 0, 0), # Positive q opens lid upward
        motion_limits=MotionLimits(lower=0.0, upper=2.0)
    )
    
    # 4. Cycle Dial
    dial_r = 0.03
    dial_h = 0.02
    dial = model.part("cycle_dial")
    dial.visual(
        Cylinder(radius=dial_r, height=dial_h),
        origin=Origin(xyz=(0, 0, 0), rpy=(1.5708, 0, 0)),
        name="dial_body"
    )
    
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.15, 0.24 + dial_h / 2 - 0.005, cab_h + console_h / 2)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0)
    )
    
    # 5. Buttons
    btn_r = 0.01
    btn_h = 0.015
    for i in range(4):
        btn = model.part(f"button_{i}")
        btn.visual(
            Cylinder(radius=btn_r, height=btn_h),
            origin=Origin(xyz=(0, 0, 0), rpy=(1.5708, 0, 0)),
            name=f"button_body_{i}"
        )
        
        btn_x = 0.05 + i * 0.06
        model.articulation(
            f"button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=btn,
            origin=Origin(xyz=(btn_x, 0.24 + btn_h / 2 - 0.005, cab_h + console_h / 2)),
            axis=(0, 1, 0), # Push into the console
            motion_limits=MotionLimits(lower=0.0, upper=0.005)
        )
        
    # 6. Lid-lock Plunger
    plunger_r = 0.005
    plunger_h = 0.015
    plunger = model.part("lid_lock_plunger")
    plunger.visual(
        Cylinder(radius=plunger_r, height=plunger_h),
        origin=Origin(xyz=(0, 0, plunger_h / 2)),
        name="plunger_body"
    )
    
    model.articulation(
        "plunger_press",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=plunger,
        origin=Origin(xyz=(0, -0.32, cab_h)),
        axis=(0, 0, -1), # Push down into the deck
        motion_limits=MotionLimits(lower=0.0, upper=0.01)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("cabinet", "lid", reason="Lid rests flush on the cabinet deck.")
    ctx.allow_overlap("cabinet", "tub", reason="Tub is housed inside the cabinet cavity.")
    ctx.allow_overlap("cabinet", "lid_lock_plunger", reason="Plunger is mounted in the deck.")
    ctx.allow_overlap("cabinet", "cycle_dial", reason="Dial is mounted into the console.")
    for i in range(4):
        ctx.allow_overlap("cabinet", f"button_{i}", reason="Button is mounted into the console.")
    
    ctx.allow_overlap("lid", "lid_lock_plunger", reason="The lid lock plunger is pressed by the lid when closed.")
    
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    
    # Exact assertions
    ctx.expect_gap(lid, tub, axis="z", min_gap=0.005, name="Lid clears the tub")
    
    with ctx.pose(lid_hinge=1.5):
        lid_aabb = ctx.part_world_aabb(lid)
        if lid_aabb:
            ctx.check("Lid opens upward", lid_aabb[1][2] > 1.2, details="Lid max Z should be high when open")
            
    return ctx.report()

object_model = build_object_model()