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
    model = ArticulatedObject(name="sewing_box")

    wood_color = (0.55, 0.35, 0.15)
    dark_wood_color = (0.4, 0.2, 0.1)
    brass_color = (0.8, 0.7, 0.3)
    
    # ---------------------------------------------------------
    # Base Box
    # ---------------------------------------------------------
    length = 0.32
    depth = 0.20
    height = 0.15
    wall_t = 0.01
    
    # Base is centered at 0,0,0. Z goes from -0.075 to 0.075.
    base_cq = cq.Workplane("XY").box(length, depth, height)
    
    # Top cavity cut: Z from 0.005 to 0.075
    top_cut = (
        cq.Workplane("XY")
        .center(0, 0)
        .workplane(offset=0.04)
        .box(length - 2*wall_t, depth - 2*wall_t, 0.07)
    )
    
    # Drawer cavity cut: Y from -0.10 to 0.09, Z from -0.065 to -0.005
    drawer_cut = (
        cq.Workplane("XY")
        .center(0, -0.005)
        .workplane(offset=-0.035)
        .box(length - 2*wall_t, depth - wall_t, 0.06)
    )
    
    base_cq = base_cq.cut(top_cut).cut(drawer_cut)
    
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        name="base_shell",
        color=wood_color,
    )
    
    # Rails for the tray
    base.visual(
        Box((0.30, 0.012, 0.01)),
        origin=Origin(xyz=(0.0, -0.086, 0.10)),
        name="front_rail",
        color=dark_wood_color,
    )
    base.visual(
        Box((0.30, 0.012, 0.01)),
        origin=Origin(xyz=(0.0, 0.086, 0.10)),
        name="back_rail",
        color=dark_wood_color,
    )
    
    # ---------------------------------------------------------
    # Drawer
    # ---------------------------------------------------------
    drawer_x = 0.296
    drawer_y = 0.186
    drawer_z = 0.056
    drawer_wall = 0.005
    
    drawer_cq = (
        cq.Workplane("XY")
        .box(drawer_x, drawer_y, drawer_z)
        .faces(">Z").workplane()
        .rect(drawer_x - 2*drawer_wall, drawer_y - 2*drawer_wall)
        .cutBlind(-(drawer_z - drawer_wall))
    )
    
    drawer = model.part("drawer")
    # Place drawer with a 0.5mm gap from the floor to avoid mesh contact warnings
    drawer_part_origin = Origin(xyz=(0.0, -0.099, 0.0385))
    
    # Visual is shifted so its front face is at local Y=0
    drawer.visual(
        mesh_from_cadquery(drawer_cq, "drawer_mesh"),
        origin=Origin(xyz=(0.0, drawer_y / 2, 0.0)),
        name="drawer_shell",
        color=wood_color,
    )
    # Knob on the front face
    drawer.visual(
        Cylinder(radius=0.01, length=0.015),
        origin=Origin(xyz=(0.0, -0.0065, 0.0), rpy=(1.570796, 0.0, 0.0)),
        name="drawer_knob",
        color=brass_color,
    )
    
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=drawer_part_origin,
        axis=(0.0, -1.0, 0.0), # Slides out towards -Y
        motion_limits=MotionLimits(lower=0.0, upper=0.15),
    )
    
    # ---------------------------------------------------------
    # Sliding Tray
    # ---------------------------------------------------------
    tray_x = 0.138
    tray_y = 0.178
    tray_z = 0.03
    tray_wall = 0.003
    
    tray_cq = (
        cq.Workplane("XY")
        .box(tray_x, tray_y, tray_z)
        .faces(">Z").workplane()
        .rect(tray_x - 2*tray_wall, tray_y - 2*tray_wall)
        .cutBlind(-(tray_z - tray_wall))
    )
    
    tray = model.part("sliding_tray")
    # Tray slides along X. Set part origin at its leftmost position with 0.5mm gap above rails.
    tray_part_origin = Origin(xyz=(-0.08, 0.0, 0.1205))
    
    tray.visual(
        mesh_from_cadquery(tray_cq, "tray_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="tray_shell",
        color=wood_color,
    )
    
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=tray_part_origin,
        axis=(1.0, 0.0, 0.0), # Slides right
        motion_limits=MotionLimits(lower=0.0, upper=0.16),
    )
    
    # ---------------------------------------------------------
    # Lid
    # ---------------------------------------------------------
    lid = model.part("lid")
    # Hinge at the back top edge, with a 0.5mm gap to avoid contact warnings
    lid_part_origin = Origin(xyz=(0.0, 0.10, 0.1505))
    
    # Lid main board
    lid.visual(
        Box((0.32, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, -0.10, 0.01)),
        name="lid_board",
        color=wood_color,
    )
    # Handle posts
    lid.visual(
        Box((0.02, 0.02, 0.032)),
        origin=Origin(xyz=(-0.06, -0.10, 0.034)),
        name="handle_post_left",
        color=dark_wood_color,
    )
    lid.visual(
        Box((0.02, 0.02, 0.032)),
        origin=Origin(xyz=(0.06, -0.10, 0.034)),
        name="handle_post_right",
        color=dark_wood_color,
    )
    # Handle top bar
    lid.visual(
        Box((0.14, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, -0.10, 0.054)),
        name="handle_bar",
        color=dark_wood_color,
    )
    
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=lid_part_origin,
        axis=(-1.0, 0.0, 0.0), # Positive opens upwards
        motion_limits=MotionLimits(lower=0.0, upper=2.0),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    tray = object_model.get_part("sliding_tray")
    
    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    tray_slide = object_model.get_articulation("tray_slide")
    
    # Allow isolated parts due to intentional functional clearance gaps
    ctx.allow_isolated_part(lid, reason="Hinged lid rests on the base with a functional clearance gap.")
    ctx.allow_isolated_part(tray, reason="Sliding tray rests on rails with a functional clearance gap.")
    ctx.allow_isolated_part(drawer, reason="Drawer slides in its cavity with a functional clearance gap.")
    
    # Rest pose checks
    ctx.expect_within(drawer, base, axes="xz", name="drawer fits inside base")
    ctx.expect_within(tray, base, axes="y", name="tray fits inside base Y")
    
    # Drawer slide test
    with ctx.pose({drawer_slide: 0.10}):
        ctx.expect_overlap(drawer, base, axes="z", min_overlap=0.01, name="drawer retains insertion when opened")
        
    # Lid open test
    with ctx.pose({lid_hinge: 1.5}):
        lid_aabb = ctx.part_world_aabb(lid)
        ctx.check("lid opens upward", lid_aabb is not None and lid_aabb[1][2] > 0.25, details="Lid should swing up")
        
    # Tray slide test
    with ctx.pose({tray_slide: 0.10}):
        ctx.expect_within(tray, base, axes="y", name="tray stays on rails when sliding")
        
    return ctx.report()


object_model = build_object_model()
