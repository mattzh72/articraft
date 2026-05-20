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
    model = ArticulatedObject(name="control_cabinet")
    
    w, d, h = 0.6, 0.4, 1.8
    t = 0.02
    
    # 1. Carcass
    carcass_shape = (
        cq.Workplane("XY")
        .box(w, d, h, centered=(True, True, False))
        .faces(">Y")
        .shell(-t)
    )
    
    # Outer knuckles on carcass
    outer_hinge_x = -w/2 - 0.025  # -0.325
    outer_hinge_y = d/2 + t/2 + 0.002  # 0.212
    knuckle_h = 0.04
    outer_knuckle_zs = [0.3, 0.9, 1.5]
    
    carcass_outer_knuckles = cq.Workplane("XY")
    for z in outer_knuckle_zs:
        # Bracket from left wall (-0.30) to hinge (-0.325)
        carcass_outer_knuckles = carcass_outer_knuckles.add(
            cq.Workplane("XY", origin=(-0.3125, 0.181, z))
            .box(0.025, 0.062, knuckle_h, centered=(True, True, False))
        ).add(
            cq.Workplane("XY", origin=(outer_hinge_x, outer_hinge_y, z))
            .cylinder(knuckle_h, 0.01, centered=(True, True, False))
        )
    # Add a continuous hinge pin
    carcass_outer_knuckles = carcass_outer_knuckles.add(
        cq.Workplane("XY", origin=(outer_hinge_x, outer_hinge_y, 0.3))
        .cylinder(1.282, 0.008, centered=(True, True, False))
    )
    carcass_shape = carcass_shape.union(carcass_outer_knuckles)
    
    # Inner knuckles on carcass
    inner_hinge_x = -0.25
    inner_hinge_y = 0.15
    inner_knuckle_h = 0.03
    inner_knuckle_zs = [0.5, 1.1]
    
    carcass_inner_knuckles = cq.Workplane("XY")
    for z in inner_knuckle_zs:
        # Bracket from left wall (-0.28) to hinge (-0.25)
        carcass_inner_knuckles = carcass_inner_knuckles.add(
            cq.Workplane("XY", origin=(-0.270, inner_hinge_y, z))
            .box(0.04, 0.02, inner_knuckle_h, centered=(True, True, False))
        ).add(
            cq.Workplane("XY", origin=(inner_hinge_x, inner_hinge_y, z))
            .cylinder(inner_knuckle_h, 0.01, centered=(True, True, False))
        )
    # Add a continuous hinge pin
    carcass_inner_knuckles = carcass_inner_knuckles.add(
        cq.Workplane("XY", origin=(inner_hinge_x, inner_hinge_y, 0.5))
        .cylinder(0.662, 0.008, centered=(True, True, False))
    )
    carcass_shape = carcass_shape.union(carcass_inner_knuckles)
    
    carcass = model.part("carcass")
    carcass.visual(
        mesh_from_cadquery(carcass_shape, "carcass_mesh"),
        name="carcass_visual"
    )
    
    # 2. Outer Door
    outer_door_shape = (
        cq.Workplane("XY", origin=(0.027, 0, 0.002))
        .box(0.596, t, 1.796, centered=(False, True, False))
    )
    door_knuckles = cq.Workplane("XY")
    for z in outer_knuckle_zs:
        door_knuckles = door_knuckles.add(
            cq.Workplane("XY", origin=(0, 0, z + knuckle_h + 0.002))
            .cylinder(knuckle_h, 0.01, centered=(True, True, False))
        ).add(
            cq.Workplane("XY", origin=(0.0135, 0, z + knuckle_h + 0.002))
            .box(0.027, 0.02, knuckle_h, centered=(True, True, False))
        )
    outer_door_shape = outer_door_shape.union(door_knuckles)
    
    outer_door = model.part("outer_door")
    outer_door.visual(
        mesh_from_cadquery(outer_door_shape, "outer_door_mesh"),
        name="outer_door_visual"
    )
    
    model.articulation(
        "carcass_to_outer_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=outer_door,
        origin=Origin(xyz=(outer_hinge_x, outer_hinge_y, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=2.0)
    )
    
    # 3. Inner Door
    w_in = 0.50
    t_in = 0.015
    h_in = 1.70
    z_in_start = 0.05
    
    inner_door_shape = (
        cq.Workplane("XY", origin=(0.015, 0, z_in_start))
        .box(w_in, t_in, h_in, centered=(False, True, False))
    )
    
    inner_door_knuckles = cq.Workplane("XY")
    for z in inner_knuckle_zs:
        inner_door_knuckles = inner_door_knuckles.add(
            cq.Workplane("XY", origin=(0, 0, z + inner_knuckle_h + 0.002))
            .cylinder(inner_knuckle_h, 0.01, centered=(True, True, False))
        ).add(
            cq.Workplane("XY", origin=(0.0075, 0, z + inner_knuckle_h + 0.002))
            .box(0.015, 0.015, inner_knuckle_h, centered=(True, True, False))
        )
    inner_door_shape = inner_door_shape.union(inner_door_knuckles)
    
    inner_door = model.part("inner_door")
    inner_door.visual(
        mesh_from_cadquery(inner_door_shape, "inner_door_mesh"),
        name="inner_door_visual"
    )
    
    model.articulation(
        "carcass_to_inner_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=inner_door,
        origin=Origin(xyz=(inner_hinge_x, inner_hinge_y, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.5)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    carcass = object_model.get_part("carcass")
    outer_door = object_model.get_part("outer_door")
    inner_door = object_model.get_part("inner_door")
    
    ctx.allow_overlap(outer_door, carcass, reason="Hinge pin is captured inside door knuckle.")
    ctx.allow_overlap(inner_door, carcass, reason="Hinge pin is captured inside door knuckle.")
    
    # Check that doors are correctly placed relative to carcass at rest
    ctx.expect_gap(outer_door, carcass, axis="y", min_gap=-0.02, max_gap=0.005, name="outer door is flush with carcass")
    ctx.expect_within(inner_door, carcass, axes="x", name="inner door fits inside carcass width")
    
    outer_joint = object_model.get_articulation("carcass_to_outer_door")
    inner_joint = object_model.get_articulation("carcass_to_inner_door")
    
    # Check that doors open outward
    rest_outer = ctx.part_world_aabb(outer_door)
    with ctx.pose({outer_joint: 1.5}):
        open_outer = ctx.part_world_aabb(outer_door)
        if rest_outer and open_outer:
            ctx.check("outer_door_opens_outward", open_outer[1][1] > rest_outer[1][1] + 0.1)
            
    rest_inner = ctx.part_world_aabb(inner_door)
    with ctx.pose({inner_joint: 1.0}):
        open_inner = ctx.part_world_aabb(inner_door)
        if rest_inner and open_inner:
            ctx.check("inner_door_opens_outward", open_inner[1][1] > rest_inner[1][1] + 0.05)

    return ctx.report()

object_model = build_object_model()