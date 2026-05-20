from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Cylinder,
)
import cadquery as cq

steel = Material(name="steel", color=(0.7, 0.7, 0.7))
cast_iron = Material(name="cast_iron", color=(0.3, 0.3, 0.3))
black_plastic = Material(name="black_plastic", color=(0.1, 0.1, 0.1))
silver = Material(name="silver", color=(0.9, 0.9, 0.9))


def make_table_bracket():
    # Sleeve
    sleeve = cq.Workplane("XY").cylinder(0.12, 0.045).faces(">Z").hole(0.07)
    
    # Arm extending to the front (-Y)
    arm = cq.Workplane("XY").box(0.04, 0.105, 0.07).translate((0, -0.0975, 0))
    
    # Trunnion clamp at the front
    trunnion = cq.Workplane("YZ").cylinder(0.14, 0.035).faces(">X").hole(0.021).translate((0, -0.15, 0))
    
    # Crank boss on the right side of the sleeve
    crank_boss = cq.Workplane("YZ").cylinder(0.02, 0.015).translate((0.055, 0, -0.03))
    
    # Column lock boss at the back (+Y), across the split
    col_lock_boss = cq.Workplane("YZ").cylinder(0.06, 0.015).translate((0, 0.055, 0))
    
    # Tilt lock boss at the top of the trunnion, across the split
    tilt_boss = cq.Workplane("YZ").cylinder(0.14, 0.012).translate((0, -0.15, 0.025))
    
    bracket = sleeve.union(arm).union(trunnion).union(crank_boss).union(col_lock_boss).union(tilt_boss)
    
    # Split for column clamp (slit along Y axis, at the back)
    col_split = cq.Workplane("XY").box(0.005, 0.06, 0.14).translate((0, 0.045, 0))
    bracket = bracket.cut(col_split)
    
    # Split for trunnion clamp (slit along Z axis, at the top)
    tilt_split = cq.Workplane("XY").box(0.15, 0.005, 0.04).translate((0, -0.15, 0.035))
    bracket = bracket.cut(tilt_split)
    
    return bracket

def make_table():
    # Main surface
    top = cq.Workplane("XY").box(0.25, 0.25, 0.02).translate((0, -0.02, 0.05))
    # Trunnion pin
    pin = cq.Workplane("YZ").cylinder(0.24, 0.01)
    # Ribs connecting pin to top
    rib1 = cq.Workplane("XY").box(0.015, 0.10, 0.05).translate((0.115, -0.02, 0.025))
    rib2 = cq.Workplane("XY").box(0.015, 0.10, 0.05).translate((-0.115, -0.02, 0.025))
    
    table = top.union(pin).union(rib1).union(rib2)
    
    # Cut slots
    slot = cq.Workplane("XY").slot2D(0.16, 0.016).extrude(0.05).translate((0, -0.02, 0.04))
    slot1 = slot.rotate((0,-0.02,0), (0,0,1), 45)
    slot2 = slot.rotate((0,-0.02,0), (0,0,1), -45)
    table = table.cut(slot1).cut(slot2)
    return table

def make_handle(length=0.08, arm_dir="-z"):
    # Hub along X
    hub = cq.Workplane("YZ").cylinder(0.015, 0.012).translate((0.0075, 0, 0))
    if arm_dir == "-z":
        arm = cq.Workplane("XY").box(0.01, 0.01, length).translate((0.0075, 0, -length/2))
        knob = cq.Workplane("YZ").cylinder(0.025, 0.008).translate((0.02, 0, -length))
    elif arm_dir == "-y":
        arm = cq.Workplane("XY").box(0.01, length, 0.01).translate((0.0075, -length/2, 0))
        knob = cq.Workplane("YZ").cylinder(0.025, 0.008).translate((0.02, -length, 0))
    return hub.union(arm).union(knob)

def make_crank():
    # Hub along X
    hub = cq.Workplane("YZ").cylinder(0.015, 0.015).translate((0.0075, 0, 0))
    # Arm along Z
    arm = cq.Workplane("XY").box(0.015, 0.01, 0.06).translate((0.0075, 0, 0.03))
    # Handle along X
    handle = cq.Workplane("YZ").cylinder(0.04, 0.006).translate((0.0275, 0, 0.06))
    return hub.union(arm).union(handle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_tilt_table")
    
    column = model.part("column")
    column.visual(Cylinder(radius=0.035, height=0.8), origin=Origin(xyz=(0, 0, 0.4)), name="col_vis", material=steel)
    
    carriage_slider = model.part("carriage_slider")
    carriage_slider.visual(Cylinder(radius=0.045, height=0.01), origin=Origin(xyz=(0,0,0)), name="slider_vis", material=steel)
    
    model.articulation(
        "bracket_elevation",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage_slider,
        origin=Origin(xyz=(0, 0, 0.4)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-0.2, upper=0.2, effort=100, velocity=1)
    )
    
    table_bracket = model.part("table_bracket")
    table_bracket.visual(mesh_from_cadquery(make_table_bracket(), "table_bracket"), material=cast_iron)
    
    model.articulation(
        "bracket_swing",
        ArticulationType.CONTINUOUS,
        parent=carriage_slider,
        child=table_bracket,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=100, velocity=1)
    )
    
    table = model.part("table")
    table.visual(mesh_from_cadquery(make_table(), "table"), material=cast_iron)
    
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(xyz=(0, -0.15, 0)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=-1.5708, upper=1.5708, effort=100, velocity=1)
    )
    
    col_lock = model.part("bracket_lock_handle")
    col_lock.visual(mesh_from_cadquery(make_handle(0.06, arm_dir="-z"), "col_lock"), material=black_plastic)
    model.articulation(
        "bracket_lock",
        ArticulationType.CONTINUOUS,
        parent=table_bracket,
        child=col_lock,
        origin=Origin(xyz=(0.03, 0.055, 0)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=10, velocity=1)
    )
    
    tilt_lock = model.part("tilt_lock_handle")
    tilt_lock.visual(mesh_from_cadquery(make_handle(0.06, arm_dir="-y"), "tilt_lock"), material=black_plastic)
    model.articulation(
        "tilt_lock",
        ArticulationType.CONTINUOUS,
        parent=table_bracket,
        child=tilt_lock,
        origin=Origin(xyz=(0.07, -0.15, 0.025)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=10, velocity=1)
    )
    
    crank = model.part("elevation_crank")
    crank.visual(mesh_from_cadquery(make_crank(), "crank"), material=silver)
    model.articulation(
        "crank_turn",
        ArticulationType.CONTINUOUS,
        parent=table_bracket,
        child=crank,
        origin=Origin(xyz=(0.065, 0, -0.03)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=10, velocity=1)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("column", "carriage_slider", reason="slider is an invisible proxy for elevation")
    ctx.allow_overlap("table_bracket", "carriage_slider", reason="slider is inside the sleeve")
    ctx.allow_overlap("column", "table_bracket", reason="sleeve wraps column, mesh tessellation causes minor overlap")
    ctx.allow_overlap("table_bracket", "table", reason="trunnion pin is captured in the bracket clamp")
    ctx.allow_overlap("table_bracket", "bracket_lock_handle", reason="handle mounts to boss")
    ctx.allow_overlap("table_bracket", "tilt_lock_handle", reason="handle mounts to boss")
    ctx.allow_overlap("table_bracket", "elevation_crank", reason="crank mounts to boss")
    
    return ctx.report()

object_model = build_object_model()