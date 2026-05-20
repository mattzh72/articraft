import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    SpurGear,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="crankset_assembly")

    # Materials
    bb_mat = Material(color=(0.15, 0.15, 0.15), name="bb_mat")
    crank_mat = Material(color=(0.8, 0.8, 0.85), name="crank_mat")
    chainring_mat = Material(color=(0.2, 0.2, 0.2), name="chainring_mat")
    pedal_mat = Material(color=(0.05, 0.05, 0.05), name="pedal_mat")

    # ---------------------------------------------------------
    # 1. Bottom Bracket (Base)
    # ---------------------------------------------------------
    bb_part = model.part("bottom_bracket")
    
    # Shell: Y-axis cylinder
    bb_shell = cq.Workplane("XZ").cylinder(0.068, 0.02)
    
    # Frame stubs
    seat_tube = cq.Workplane("XY").center(0, 0).circle(0.016).extrude(0.15).rotate((0,0,0), (0,1,0), -15)
    down_tube = cq.Workplane("XY").center(0, 0).circle(0.018).extrude(0.15).rotate((0,0,0), (0,1,0), 60)
    
    # Chainstays (using YZ plane, extruding along -X)
    chainstay_l = (cq.Workplane("YZ").center(0.025, 0)
                   .circle(0.01)
                   .workplane(offset=-0.15)
                   .center(0.03, 0)
                   .circle(0.008)
                   .loft())
    chainstay_r = (cq.Workplane("YZ").center(-0.025, 0)
                   .circle(0.01)
                   .workplane(offset=-0.15)
                   .center(-0.03, 0)
                   .circle(0.008)
                   .loft())
    
    bb_body = bb_shell.union(seat_tube).union(down_tube).union(chainstay_l).union(chainstay_r)
    bb_part.visual(mesh_from_cadquery(bb_body, "bb_mesh"), name="bb_frame", material=bb_mat)

    # ---------------------------------------------------------
    # 2. Crankset (Rotating)
    # ---------------------------------------------------------
    crank_part = model.part("crankset")
    
    # Spindle
    spindle = cq.Workplane("XZ").cylinder(0.14, 0.012)
    
    # Right Crank Arm (at Y = -0.06, extending to Z = 0.17)
    r_crank = (cq.Workplane("XZ")
               .workplane(offset=0.06)
               .moveTo(0, 0).circle(0.02)
               .extrude(0.015)
               .union(
                   cq.Workplane("XZ").workplane(offset=0.06)
                   .moveTo(0, 0.17).circle(0.015)
                   .extrude(0.015)
               )
               .union(
                   cq.Workplane("XZ").workplane(offset=0.06)
                   .polyline([(-0.02, 0), (0.02, 0), (0.015, 0.17), (-0.015, 0.17)])
                   .close()
                   .extrude(0.015)
               ))
               
    # Left Crank Arm (at Y = 0.06, extending to Z = -0.17)
    l_crank = (cq.Workplane("XZ")
               .workplane(offset=-0.06)
               .moveTo(0, 0).circle(0.02)
               .extrude(-0.015)
               .union(
                   cq.Workplane("XZ").workplane(offset=-0.06)
                   .moveTo(0, -0.17).circle(0.015)
                   .extrude(-0.015)
               )
               .union(
                   cq.Workplane("XZ").workplane(offset=-0.06)
                   .polyline([(-0.02, 0), (0.02, 0), (0.015, -0.17), (-0.015, -0.17)])
                   .close()
                   .extrude(-0.015)
               ))
               
    # Spider (4 arms)
    spider = (cq.Workplane("XZ").workplane(offset=0.05)
              .polarArray(0.045, 45, 360, 4)
              .rect(0.09, 0.015)
              .extrude(0.005))
              
    # Pedal Axles
    r_axle = (cq.Workplane("XZ").workplane(offset=0.07)
              .center(0, 0.17)
              .circle(0.006)
              .extrude(0.09)) # reaches Y = -0.16
    l_axle = (cq.Workplane("XZ").workplane(offset=-0.07)
              .center(0, -0.17)
              .circle(0.006)
              .extrude(-0.09)) # reaches Y = 0.16
              
    crank_core = spindle.union(r_crank).union(l_crank).union(spider).union(r_axle).union(l_axle)
    crank_part.visual(mesh_from_cadquery(crank_core, "crank_core"), name="core", material=crank_mat)
    
    # Chainring
    chainring_gear = SpurGear(module=0.004, teeth_number=45, width=0.004, bore_d=0.13)
    chainring_body = cq.Workplane("XZ").workplane(offset=0.052).gear(chainring_gear).val()
    crank_part.visual(mesh_from_cadquery(chainring_body, "chainring"), name="chainring", material=chainring_mat)

    # ---------------------------------------------------------
    # 3. Pedals
    # ---------------------------------------------------------
    # Base pedal body centered at origin
    pedal_body = (cq.Workplane("XY")
                  .box(0.10, 0.08, 0.02)
                  .faces(">Y").workplane().circle(0.007).cutThruAll())
    
    pins = (cq.Workplane("XY").workplane(offset=0.01)
            .rect(0.08, 0.06, forConstruction=True)
            .vertices()
            .circle(0.002).extrude(0.002))
    pins_bottom = (cq.Workplane("XY").workplane(offset=-0.01)
                   .rect(0.08, 0.06, forConstruction=True)
                   .vertices()
                   .circle(0.002).extrude(-0.002))
    
    pedal_mesh = pedal_body.union(pins).union(pins_bottom)
    
    r_pedal = model.part("right_pedal")
    r_pedal.visual(mesh_from_cadquery(pedal_mesh, "r_pedal_mesh"), name="body", material=pedal_mat)
    
    l_pedal = model.part("left_pedal")
    l_pedal.visual(mesh_from_cadquery(pedal_mesh, "l_pedal_mesh"), name="body", material=pedal_mat)

    # ---------------------------------------------------------
    # Articulations
    # ---------------------------------------------------------
    # Crankset to Bottom Bracket
    model.articulation(
        "bb_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=bb_part,
        child=crank_part,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )
    
    # Right Pedal to Crankset
    model.articulation(
        "crankset_to_r_pedal",
        ArticulationType.CONTINUOUS,
        parent=crank_part,
        child=r_pedal,
        origin=Origin(xyz=(0, -0.12, 0.17)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )
    
    # Left Pedal to Crankset
    model.articulation(
        "crankset_to_l_pedal",
        ArticulationType.CONTINUOUS,
        parent=crank_part,
        child=l_pedal,
        origin=Origin(xyz=(0, 0.12, -0.17)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allowances for axles running through bores
    ctx.allow_overlap(
        "bottom_bracket", "crankset",
        elem_a="bb_frame", elem_b="core",
        reason="The spindle runs through the bottom bracket shell."
    )
    ctx.allow_overlap(
        "crankset", "right_pedal",
        elem_a="core", elem_b="body",
        reason="The right pedal axle runs through the right pedal body."
    )
    ctx.allow_overlap(
        "crankset", "left_pedal",
        elem_a="core", elem_b="body",
        reason="The left pedal axle runs through the left pedal body."
    )
    
    ctx.allow_isolated_part("right_pedal", reason="Supported by axle with modeled clearance.")
    ctx.allow_isolated_part("left_pedal", reason="Supported by axle with modeled clearance.")

    
    # Check pedal mounts
    ctx.expect_within("right_pedal", "crankset", axes="y", margin=0.04, inner_elem="body", outer_elem="core", name="right pedal stays on axle")
    ctx.expect_within("left_pedal", "crankset", axes="y", margin=0.04, inner_elem="body", outer_elem="core", name="left pedal stays on axle")

    # Verify crank rotation
    crank_joint = object_model.get_articulation("bb_to_crankset")
    pos_0 = ctx.part_world_position("right_pedal")
    
    with ctx.pose({crank_joint: 1.57}):
        pos_90 = ctx.part_world_position("right_pedal")
        ctx.expect_within("right_pedal", "crankset", axes="y", margin=0.04, inner_elem="body", outer_elem="core", name="right pedal stays on axle when crank rotates")
        
    if pos_0 and pos_90:
        ctx.check("pedal_moves", abs(pos_0[2] - pos_90[2]) > 0.1, "Pedal should move vertically when crank rotates")

    return ctx.report()

object_model = build_object_model()
