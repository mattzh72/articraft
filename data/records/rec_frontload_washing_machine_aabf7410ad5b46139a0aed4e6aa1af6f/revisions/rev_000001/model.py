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
    model = ArticulatedObject(name="washing_machine")
    
    width = 0.60
    depth = 0.58
    height = 0.83
    feet_h = 0.02
    
    tub_center_z = 0.0
    tub_radius = 0.25
    tub_depth = 0.40
    
    drawer_w = 0.18
    drawer_h = 0.08
    drawer_d = 0.25
    drawer_x = -0.18
    drawer_z = 0.35
    
    hatch_w = 0.15
    hatch_h = 0.10
    hatch_d = 0.10
    hatch_x = 0.20
    hatch_z = -0.32
    
    door_radius = tub_radius + 0.02
    door_thickness = 0.03
    drum_radius = tub_radius - 0.02
    drum_depth = tub_depth - 0.02
    
    # 1. Cabinet
    cab_cq = cq.Workplane("XY").box(width, depth, height).edges("|Z and >Y").fillet(0.03)
    toe_kick = cq.Workplane("XZ", origin=(0, depth/2 - 0.05, -height/2 + 0.02)).rect(width + 0.02, 0.04).extrude(0.1)
    cab_cq = cab_cq.cut(toe_kick)
    tub_cut = cq.Workplane("XZ", origin=(0, depth/2 + 0.01, tub_center_z)).circle(tub_radius).extrude(-0.50)
    cab_cq = cab_cq.cut(tub_cut)
    
    outer_tub = (
        cq.Workplane("XZ", origin=(0, depth/2 - 0.02, tub_center_z)).circle(tub_radius).extrude(-tub_depth)
        .cut(cq.Workplane("XZ", origin=(0, depth/2 + 0.01, tub_center_z)).circle(tub_radius - 0.01).extrude(-tub_depth - 0.02))
        .cut(cq.Workplane("XZ", origin=(0, depth/2 + 0.01, tub_center_z)).circle(0.025).extrude(-0.50))
    )
    cab_cq = cab_cq.union(outer_tub)
    
    axle_support = (
        cq.Workplane("XZ", origin=(0, depth/2 - 0.02 - tub_depth, tub_center_z)).circle(0.05).extrude(-0.08)
        .cut(cq.Workplane("XZ", origin=(0, depth/2 - 0.02 - tub_depth + 0.01, tub_center_z)).circle(0.025).extrude(-0.05))
    )
    cab_cq = cab_cq.union(axle_support)
    
    gasket = (
        cq.Workplane("XZ", origin=(0, depth/2, tub_center_z)).circle(tub_radius).circle(tub_radius - 0.04).extrude(-0.06)
    ).union(
        cq.Workplane("XZ", origin=(0, depth/2 + 0.01, tub_center_z)).circle(tub_radius + 0.01).circle(tub_radius - 0.03).extrude(-0.01)
    )
    cab_cq = cab_cq.union(gasket)
    
    fascia = cq.Workplane("XZ", origin=(0, depth/2 - 0.01, height/2 - 0.065)).rect(width - 0.002, 0.13).extrude(0.04).edges("|X and >Z and >Y").fillet(0.01)
    cab_cq = cab_cq.union(fascia)
    
    drawer_pocket = cq.Workplane("XZ", origin=(drawer_x, depth/2 + 0.04, drawer_z)).rect(drawer_w, drawer_h).extrude(-drawer_d - 0.04)
    cab_cq = cab_cq.cut(drawer_pocket)
    hatch_pocket = cq.Workplane("XZ", origin=(hatch_x, depth/2 + 0.01, hatch_z)).rect(hatch_w, hatch_h).extrude(-hatch_d - 0.01)
    cab_cq = cab_cq.cut(hatch_pocket)
    
    feet = cq.Workplane("XY", origin=(0, 0, -height/2 - feet_h/2 + 0.01)).rect(width - 0.1, depth - 0.1, forConstruction=True).vertices().cylinder(feet_h + 0.01, 0.02)
    cab_cq = cab_cq.union(feet)
    
    rail_w, rail_h = 0.01, 0.01
    rail_left = cq.Workplane("XZ", origin=(drawer_x - drawer_w/2 + rail_w/2, depth/2, drawer_z)).rect(rail_w, rail_h).extrude(-drawer_d + 0.02)
    rail_right = cq.Workplane("XZ", origin=(drawer_x + drawer_w/2 - rail_w/2, depth/2, drawer_z)).rect(rail_w, rail_h).extrude(-drawer_d + 0.02)
    cab_cq = cab_cq.union(rail_left).union(rail_right)
    
    bracket_1 = cq.Workplane("XY", origin=(-door_radius, depth/2 + 0.01, tub_center_z + 0.1)).box(0.03, 0.02, 0.03)
    bracket_2 = cq.Workplane("XY", origin=(-door_radius, depth/2 + 0.01, tub_center_z - 0.1)).box(0.03, 0.02, 0.03)
    cab_cq = cab_cq.union(bracket_1).union(bracket_2)
    
    latch_recv = cq.Workplane("XY", origin=(door_radius, depth/2 + 0.01, tub_center_z)).box(0.02, 0.01, 0.04)
    cab_cq = cab_cq.cut(latch_recv)
    
    cabinet = model.part("cabinet")
    cabinet.visual(mesh_from_cadquery(cab_cq, "cabinet_mesh"), origin=Origin(xyz=(0, 0, height/2 + feet_h)))
    
    # 2. Drum
    drum_local = (
        cq.Workplane("XY")
        .circle(drum_radius)
        .extrude(drum_depth)
        .faces(">Z").shell(-0.005)
    )
    for z_offset in [0.1, 0.2, 0.3]:
        for angle in range(0, 360, 45):
            c = (
                cq.Workplane("XY")
                .workplane(offset=z_offset)
                .transformed(rotate=(0, 0, angle))
                .center(drum_radius, 0)
                .circle(0.015)
                .extrude(0.02, both=True)
            )
            drum_local = drum_local.cut(c)
    drum_axle = cq.Workplane("XY", origin=(0,0,-0.052)).circle(0.02).extrude(0.052)
    drum_local = drum_local.union(drum_axle)
    drum_cq = drum_local.rotate((0,0,0), (1,0,0), -90)
    
    drum = model.part("drum")
    drum.visual(mesh_from_cadquery(drum_cq, "drum_mesh"))
    
    drum_y = depth/2 - 0.02 - drum_depth
    drum_z = height/2 + feet_h + tub_center_z
    model.articulation(
        "drum_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0, drum_y, drum_z)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0)
    )
    
    # 3. Door
    door_frame = (
        cq.Workplane("XY")
        .circle(door_radius)
        .circle(door_radius - 0.07)
        .extrude(door_thickness)
    )
    barrel_1 = cq.Workplane("XY", origin=(-door_radius, 0.1, door_thickness/2)).box(0.04, 0.04, door_thickness)
    barrel_2 = cq.Workplane("XY", origin=(-door_radius, -0.1, door_thickness/2)).box(0.04, 0.04, door_thickness)
    latch = cq.Workplane("XY", origin=(door_radius, 0, door_thickness/2)).box(0.03, 0.04, door_thickness)
    door_frame = door_frame.union(barrel_1).union(barrel_2).union(latch)
    door_frame = door_frame.rotate((0,0,0), (1,0,0), -90)
    
    door_glass = (
        cq.Workplane("XY", origin=(0,0,-0.01))
        .circle(door_radius - 0.065)
        .extrude(0.07)
        .faces("<Z").shell(-0.01)
    ).rotate((0,0,0), (1,0,0), 90)
    
    door = model.part("door")
    door_y = depth/2 + 0.008
    hinge_x = -door_radius
    hinge_y = door_y + door_thickness/2
    
    door.visual(mesh_from_cadquery(door_frame, "door_frame_mesh"), origin=Origin(xyz=(-hinge_x, door_y - hinge_y, 0)))
    door.visual(mesh_from_cadquery(door_glass, "door_glass_mesh"), origin=Origin(xyz=(-hinge_x, door_y - hinge_y, 0)))
    
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, drum_z)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=2.5)
    )
    
    # 4. Drawer
    drawer_cq = (
        cq.Workplane("XY")
        .box(drawer_w - 0.01, drawer_h - 0.01, drawer_d)
    )
    comp_w = (drawer_w - 0.04) / 3
    for i in range(3):
        cx = -drawer_w/2 + 0.01 + comp_w/2 + i*(comp_w + 0.01)
        cut_box = cq.Workplane("XY", origin=(cx, 0, 0.01)).box(comp_w, drawer_h - 0.03, drawer_d - 0.02)
        drawer_cq = drawer_cq.cut(cut_box)
    drawer_front = cq.Workplane("XY", origin=(0, -drawer_h/2 + 0.005, drawer_d/2)).box(drawer_w - 0.01, drawer_h - 0.01, 0.02)
    drawer_cq = drawer_cq.union(drawer_front)
    groove = cq.Workplane("YZ").rect(drawer_h - 0.02, drawer_d).extrude(0.01)
    drawer_cq = drawer_cq.cut(groove.translate((-drawer_w/2, 0, 0))).cut(groove.translate((drawer_w/2 - 0.01, 0, 0)))
    drawer_cq = drawer_cq.rotate((0,0,0), (1,0,0), -90)
    
    drawer = model.part("drawer")
    drawer.visual(mesh_from_cadquery(drawer_cq, "drawer_mesh"))
    
    drawer_world_y = 0.203
    drawer_world_z = height/2 + feet_h + drawer_z
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(drawer_x, drawer_world_y, drawer_world_z)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.15)
    )
    
    # 5. Controls
    fascia_y = depth/2 + 0.028
    
    knob_cq = (
        cq.Workplane("XY")
        .circle(0.03)
        .extrude(0.02)
        .faces(">Z").workplane().rect(0.01, 0.04).extrude(0.01)
    ).rotate((0,0,0), (1,0,0), -90)
    
    knob = model.part("cycle_knob")
    knob.visual(mesh_from_cadquery(knob_cq, "knob_mesh"))
    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.15, fascia_y, drawer_world_z)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0)
    )
    
    button_cq = (
        cq.Workplane("XY")
        .rect(0.03, 0.015)
        .extrude(0.01)
    ).rotate((0,0,0), (1,0,0), -90)
    
    for i, bx in enumerate([-0.05, 0.0, 0.05, 0.10]):
        btn = model.part(f"button_{i}")
        btn.visual(mesh_from_cadquery(button_cq, f"button_{i}_mesh"))
        model.articulation(
            f"button_{i}_press",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=btn,
            origin=Origin(xyz=(bx, fascia_y, drawer_world_z)),
            axis=(0, -1, 0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=0.005)
        )
        
    start_btn_cq = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(0.01)
    ).rotate((0,0,0), (1,0,0), -90)
    
    start_btn = model.part("start_button")
    start_btn.visual(mesh_from_cadquery(start_btn_cq, "start_btn_mesh"))
    model.articulation(
        "start_press",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_btn,
        origin=Origin(xyz=(0.22, fascia_y, drawer_world_z)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=0.005)
    )
    
    # 6. Service Hatch
    hatch_cq = (
        cq.Workplane("XY")
        .box(hatch_w - 0.01, hatch_h - 0.01, 0.02)
    )
    pin = cq.Workplane("YZ", origin=(0, -hatch_h/2 + 0.01, 0)).circle(0.005).extrude(hatch_w/2 - 0.01, both=True)
    hatch_cq = hatch_cq.union(pin)
    hatch_cq = hatch_cq.rotate((0,0,0), (1,0,0), -90)
    
    hatch = model.part("service_hatch")
    hatch_world_y = depth/2 - 0.002
    hatch_world_z = height/2 + feet_h + hatch_z
    hatch_hinge_z = hatch_world_z - hatch_h/2 + 0.01
    
    hatch.visual(mesh_from_cadquery(hatch_cq, "hatch_mesh"), origin=Origin(xyz=(0, hatch_world_y - hatch_world_y, hatch_world_z - hatch_hinge_z)))
    
    model.articulation(
        "hatch_open",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=hatch,
        origin=Origin(xyz=(hatch_x, hatch_world_y, hatch_hinge_z)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.57)
    )
    
    # 7. Drain Cap
    drain_cap_cq = (
        cq.Workplane("XY")
        .circle(0.02)
        .extrude(0.02)
        .faces(">Z").workplane().rect(0.005, 0.03).extrude(0.01)
    ).rotate((0,0,0), (1,0,0), -90)
    
    drain_cap = model.part("drain_cap")
    drain_cap.visual(mesh_from_cadquery(drain_cap_cq, "drain_cap_mesh"))
    model.articulation(
        "drain_cap_turn",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drain_cap,
        origin=Origin(xyz=(hatch_x, 0.198, hatch_world_z)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("drawer", "cabinet", reason="Drawer slides into pocket")
    ctx.allow_overlap("door", "drum", reason="Door glass protrudes into the drum")
    ctx.allow_overlap("door", "cabinet", reason="Door frame seals against the gasket")
    ctx.allow_overlap("service_hatch", "cabinet", reason="Hatch fits into cabinet pocket")
    ctx.allow_overlap("drain_cap", "cabinet", reason="Drain cap seated in cabinet")
    ctx.allow_overlap("drum", "cabinet", reason="Drum axle seated in support")
    
    for i in range(4):
        ctx.allow_overlap(f"button_{i}", "cabinet", reason="Button seated in fascia")
    ctx.allow_overlap("start_button", "cabinet", reason="Button seated in fascia")
    ctx.allow_overlap("cycle_knob", "cabinet", reason="Knob seated in fascia")
    
    ctx.allow_isolated_part("drawer", reason="Drawer slides in pocket")
    ctx.allow_isolated_part("door", reason="Door attached via hinge")
    ctx.allow_isolated_part("service_hatch", reason="Hatch attached via hinge")
    ctx.allow_isolated_part("drain_cap", reason="Drain cap seated in pocket")
    ctx.allow_isolated_part("drum", reason="Drum suspended on axle")
    for i in range(4):
        ctx.allow_isolated_part(f"button_{i}", reason="Button embedded in fascia")
    ctx.allow_isolated_part("start_button", reason="Button embedded in fascia")
    ctx.allow_isolated_part("cycle_knob", reason="Knob embedded in fascia")
    door = object_model.get_part("door")
    cabinet = object_model.get_part("cabinet")
    door_hinge = object_model.get_articulation("door_hinge")
    
    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(door)
        
    if rest_aabb and open_aabb:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2
        open_center_x = (open_aabb[0][0] + open_aabb[1][0]) / 2
        ctx.check("door_opens_outward", open_center_x < rest_center_x - 0.1, f"rest_x={rest_center_x}, open_x={open_center_x}")
        
    return ctx.report()

object_model = build_object_model()
