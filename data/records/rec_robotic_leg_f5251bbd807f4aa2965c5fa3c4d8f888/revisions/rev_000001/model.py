import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)

def build_hip_mount():
    base = cq.Workplane("XY").box(0.14, 0.12, 0.02).translate((0, 0, 0.01))
    yoke = cq.Workplane("XY").box(0.14, 0.08, 0.05).translate((0, 0, -0.025))
    cutout = cq.Workplane("YZ").center(0, -0.04).cylinder(0.15, 0.04)
    yoke = yoke.cut(cutout)
    return base.union(yoke)

def build_hip_roll_link():
    roll_cyl = cq.Workplane("YZ").cylinder(0.14, 0.04)
    drop = cq.Workplane("XY").box(0.06, 0.06, 0.08).translate((0, 0, -0.04))
    cutout = cq.Workplane("XZ").center(0, -0.08).cylinder(0.07, 0.04)
    drop = drop.cut(cutout)
    return roll_cyl.union(drop)

def build_thigh_core():
    pitch_cyl = cq.Workplane("XZ").cylinder(0.06, 0.04)
    strut = cq.Workplane("XY").box(0.04, 0.04, 0.40).translate((0, 0, -0.20))
    knee_cyl = cq.Workplane("XZ").center(0, -0.40).cylinder(0.06, 0.045)
    axle = cq.Workplane("XZ").center(0, -0.40).cylinder(0.08, 0.015)
    return pitch_cyl.union(strut).union(knee_cyl).union(axle)

def build_thigh_armor():
    armor = cq.Workplane("XY").workplane(offset=-0.20).box(0.11, 0.13, 0.26)
    armor = armor.edges("|Z").fillet(0.03)
    armor = armor.faces(">Z").chamfer(0.015)
    armor = armor.faces("<Z").chamfer(0.015)
    core_cut = cq.Workplane("XY").workplane(offset=-0.20).box(0.04, 0.04, 0.30)
    armor = armor.cut(core_cut)
    return armor

def build_thigh_knee_caps():
    cap1 = cq.Workplane("XZ").workplane(offset=0.045).center(0, -0.40).cylinder(0.01, 0.046)
    cap2 = cq.Workplane("XZ").workplane(offset=-0.045).center(0, -0.40).cylinder(0.01, 0.046)
    return cap1.union(cap2)

def build_calf_core():
    top_bar = cq.Workplane("XY").box(0.03, 0.08, 0.02).translate((0, 0, 0))
    fork1 = cq.Workplane("XZ").workplane(offset=0.035).cylinder(0.01, 0.045)
    fork2 = cq.Workplane("XZ").workplane(offset=-0.035).cylinder(0.01, 0.045)
    
    hole = cq.Workplane("XZ").cylinder(0.10, 0.016)
    fork1 = fork1.cut(hole)
    fork2 = fork2.cut(hole)
    
    strut = cq.Workplane("XY").box(0.03, 0.04, 0.40).translate((0, 0, -0.20))
    
    bot_bar = cq.Workplane("XY").box(0.03, 0.08, 0.02).translate((0, 0, -0.40))
    fork3 = cq.Workplane("XZ").workplane(offset=0.035).center(0, -0.40).cylinder(0.01, 0.035)
    fork4 = cq.Workplane("XZ").workplane(offset=-0.035).center(0, -0.40).cylinder(0.01, 0.035)
    
    return top_bar.union(fork1).union(fork2).union(strut).union(bot_bar).union(fork3).union(fork4)

def build_calf_armor():
    armor = cq.Workplane("XY").workplane(offset=-0.20).box(0.09, 0.11, 0.28)
    armor = armor.edges("|Z").fillet(0.025)
    armor = armor.faces(">Z").chamfer(0.015)
    armor = armor.faces("<Z").chamfer(0.015)
    core_cut = cq.Workplane("XY").workplane(offset=-0.20).box(0.03, 0.04, 0.30)
    armor = armor.cut(core_cut)
    
    shin = cq.Workplane("XY").box(0.04, 0.02, 0.20).translate((0, 0.06, -0.20))
    shin = shin.edges("|Z").fillet(0.008)
    return armor.union(shin)

def build_calf_ankle_caps():
    cap1 = cq.Workplane("XZ").workplane(offset=0.045).center(0, -0.40).cylinder(0.01, 0.036)
    cap2 = cq.Workplane("XZ").workplane(offset=-0.045).center(0, -0.40).cylinder(0.01, 0.036)
    return cap1.union(cap2)

def build_ankle_pitch_link():
    pitch_cyl = cq.Workplane("XZ").cylinder(0.06, 0.035)
    drop = cq.Workplane("XY").box(0.04, 0.04, 0.04).translate((0, 0, -0.02))
    roll_cyl = cq.Workplane("YZ").center(0, -0.04).cylinder(0.06, 0.03)
    return pitch_cyl.union(drop).union(roll_cyl)

def build_foot():
    fork1 = cq.Workplane("YZ").workplane(offset=0.035).cylinder(0.01, 0.03)
    fork2 = cq.Workplane("YZ").workplane(offset=-0.035).cylinder(0.01, 0.03)
    body = cq.Workplane("XY").box(0.10, 0.22, 0.04).translate((0, 0.04, -0.04))
    body = body.edges("|Z").fillet(0.02)
    body = body.faces(">Z").chamfer(0.01)
    strut = cq.Workplane("XY").box(0.06, 0.06, 0.04).translate((0, 0, -0.02))
    return fork1.union(fork2).union(body).union(strut)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="humanoid_leg")
    
    mat_dark_metal = Material(name="dark_metal", color=(0.2, 0.2, 0.22, 1.0))
    mat_light_armor = Material(name="light_armor", color=(0.85, 0.85, 0.88, 1.0))
    mat_accent = Material(name="accent", color=(0.1, 0.5, 0.8, 1.0))
    
    hip_mount = model.part("hip_mount")
    hip_mount.visual(mesh_from_cadquery(build_hip_mount(), "hip_mount_mesh"), material=mat_dark_metal)
    
    hip_roll_link = model.part("hip_roll_link")
    hip_roll_link.visual(mesh_from_cadquery(build_hip_roll_link(), "hip_roll_mesh"), material=mat_dark_metal)
    
    thigh = model.part("thigh")
    thigh.visual(mesh_from_cadquery(build_thigh_core(), "thigh_core_mesh"), material=mat_dark_metal, name="core")
    thigh.visual(mesh_from_cadquery(build_thigh_armor(), "thigh_armor_mesh"), material=mat_light_armor, name="armor")
    thigh.visual(mesh_from_cadquery(build_thigh_knee_caps(), "thigh_knee_caps_mesh"), material=mat_accent, name="knee_caps")
    
    calf = model.part("calf")
    calf.visual(mesh_from_cadquery(build_calf_core(), "calf_core_mesh"), material=mat_dark_metal, name="core")
    calf.visual(mesh_from_cadquery(build_calf_armor(), "calf_armor_mesh"), material=mat_light_armor, name="armor")
    calf.visual(mesh_from_cadquery(build_calf_ankle_caps(), "calf_ankle_caps_mesh"), material=mat_accent, name="ankle_caps")
    
    ankle_pitch_link = model.part("ankle_pitch_link")
    ankle_pitch_link.visual(mesh_from_cadquery(build_ankle_pitch_link(), "ankle_pitch_mesh"), material=mat_dark_metal)
    
    foot = model.part("foot")
    foot.visual(mesh_from_cadquery(build_foot(), "foot_mesh"), material=mat_dark_metal)
    
    model.articulation(
        name="hip_roll",
        articulation_type=ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=hip_roll_link,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.5, upper=0.5, effort=100.0, velocity=5.0)
    )
    
    model.articulation(
        name="hip_pitch",
        articulation_type=ArticulationType.REVOLUTE,
        parent=hip_roll_link,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.5, upper=1.5, effort=100.0, velocity=5.0)
    )
    
    model.articulation(
        name="knee_pitch",
        articulation_type=ArticulationType.REVOLUTE,
        parent=thigh,
        child=calf,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.0, upper=0.0, effort=100.0, velocity=5.0)
    )
    
    model.articulation(
        name="ankle_pitch",
        articulation_type=ArticulationType.REVOLUTE,
        parent=calf,
        child=ankle_pitch_link,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.8, upper=0.8, effort=100.0, velocity=5.0)
    )
    
    model.articulation(
        name="ankle_roll",
        articulation_type=ArticulationType.REVOLUTE,
        parent=ankle_pitch_link,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.5, upper=0.5, effort=100.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allow intentional nesting overlaps
    ctx.allow_overlap("hip_mount", "hip_roll_link", reason="Roll cylinder nests in yoke")
    ctx.allow_overlap("hip_roll_link", "thigh", reason="Pitch cylinder nests in drop bracket")
    ctx.allow_overlap("thigh", "calf", reason="Knee forks nest around knee cylinder")
    ctx.allow_overlap("calf", "ankle_pitch_link", reason="Ankle forks nest around ankle cylinder")
    ctx.allow_overlap("ankle_pitch_link", "foot", reason="Foot forks nest around roll cylinder")
    
    knee_joint = object_model.get_articulation("knee_pitch")
    
    with ctx.pose({knee_joint: -1.0}):
        ctx.expect_overlap("thigh", "calf", axes="x", min_overlap=0.01)
            
    return ctx.report()

object_model = build_object_model()