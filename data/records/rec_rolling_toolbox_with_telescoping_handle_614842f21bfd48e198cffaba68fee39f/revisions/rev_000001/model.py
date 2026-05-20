from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")
    
    red_mat = Material("red", color=(0.8, 0.1, 0.1, 1.0))
    black_mat = Material("black", color=(0.1, 0.1, 0.1, 1.0))
    silver_mat = Material("silver", color=(0.7, 0.7, 0.7, 1.0))
    clear_mat = Material("clear", color=(0.9, 0.9, 0.9, 0.5))
    
    w, d, h = 0.60, 0.40, 0.45
    
    # --- Base ---
    base = model.part("base")
    
    base_geom = cq.Workplane("XY").box(w, d, h)
    
    # Wheel wells (back bottom corners)
    wheel_well_w = 0.06
    wheel_well_d = 0.16
    wheel_well_h = 0.16
    base_geom = base_geom.cut(cq.Workplane("XY").center(-0.27, 0.12).box(wheel_well_w, wheel_well_d, wheel_well_h).translate((0, 0, -h/2 + wheel_well_h/2)))
    base_geom = base_geom.cut(cq.Workplane("XY").center(0.27, 0.12).box(wheel_well_w, wheel_well_d, wheel_well_h).translate((0, 0, -h/2 + wheel_well_h/2)))
    
    # Shell to hollow out
    base_geom = base_geom.faces("+Z").shell(-0.005)
    
    # Add back strip to close the top rear
    back_strip = cq.Workplane("XY").center(0, 0.17).box(w, 0.06, 0.035).translate((0, 0, h/2 - 0.0175))
    base_geom = base_geom.union(back_strip)
    
    # Grip recess in back strip
    base_geom = base_geom.cut(cq.Workplane("XY").center(0, 0.17).box(0.35, 0.05, 0.03).translate((0, 0, h/2 - 0.015)))
    
    # Handle tube holes
    base_geom = base_geom.cut(cq.Workplane("XY").center(-0.15, 0.17).cylinder(0.1, 0.015).translate((0, 0, h/2 - 0.03)))
    base_geom = base_geom.cut(cq.Workplane("XY").center(0.15, 0.17).cylinder(0.1, 0.015).translate((0, 0, h/2 - 0.03)))
    
    # Front feet
    foot = cq.Workplane("XY").box(0.04, 0.04, 0.015)
    base_geom = base_geom.union(foot.translate((-0.25, -0.15, -h/2 - 0.0075)))
    base_geom = base_geom.union(foot.translate((0.25, -0.15, -h/2 - 0.0075)))
    
    base.visual(
        mesh_from_cadquery(base_geom, "base_mesh"),
        origin=Origin(xyz=(0, 0, h/2)),
        material=black_mat,
        name="base_shell"
    )
    
    # Axles for wheels
    base.visual(
        Cylinder(radius=0.01, length=0.06),
        origin=Origin(xyz=(-0.27, 0.14, 0.06), rpy=(0, 1.5708, 0)),
        material=silver_mat,
        name="axle_left"
    )
    base.visual(
        Cylinder(radius=0.01, length=0.06),
        origin=Origin(xyz=(0.27, 0.14, 0.06), rpy=(0, 1.5708, 0)),
        material=silver_mat,
        name="axle_right"
    )
    
    # --- Lid ---
    lid = model.part("lid")
    lid_d = 0.34
    lid_h = 0.05
    
    lid_geom = cq.Workplane("XY").box(w, lid_d, lid_h)
    
    # Organizer recesses
    org_w, org_d, org_h = 0.22, 0.14, 0.02
    lid_geom = lid_geom.cut(cq.Workplane("XY").center(-0.15, 0).box(org_w, org_d, org_h).translate((0, 0, lid_h/2 - org_h/2)))
    lid_geom = lid_geom.cut(cq.Workplane("XY").center(0.15, 0).box(org_w, org_d, org_h).translate((0, 0, lid_h/2 - org_h/2)))
    
    lid_geom = lid_geom.faces("-Z").shell(-0.005)
    
    lid.visual(
        mesh_from_cadquery(lid_geom, "lid_mesh"),
        origin=Origin(xyz=(0, -0.17, 0.025)),
        material=red_mat,
        name="lid_shell"
    )
    
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0, 0.14, 0.45)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=1.57)
    )
    
    # --- Organizer Covers ---
    cover_0 = model.part("organizer_cover_0")
    cover_0.visual(
        Box((0.21, 0.13, 0.02)),
        origin=Origin(xyz=(0, -0.065, -0.01)),
        material=clear_mat,
        name="cover_box_0"
    )
    model.articulation(
        "lid_to_cover_0",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover_0,
        origin=Origin(xyz=(-0.15, -0.10, 0.05)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=2.0)
    )

    cover_1 = model.part("organizer_cover_1")
    cover_1.visual(
        Box((0.21, 0.13, 0.02)),
        origin=Origin(xyz=(0, -0.065, -0.01)),
        material=clear_mat,
        name="cover_box_1"
    )
    model.articulation(
        "lid_to_cover_1",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cover_1,
        origin=Origin(xyz=(0.15, -0.10, 0.05)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=0.0, upper=2.0)
    )
        
    # --- Handle ---
    handle = model.part("handle")
    handle.visual(
        Box((0.324, 0.03, 0.03)),
        origin=Origin(xyz=(0, 0, 0.015)),
        material=red_mat,
        name="grip"
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.4),
        origin=Origin(xyz=(-0.15, 0, -0.2)),
        material=silver_mat,
        name="tube_left"
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.4),
        origin=Origin(xyz=(0.15, 0, -0.2)),
        material=silver_mat,
        name="tube_right"
    )
    
    model.articulation(
        "base_to_handle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0, 0.17, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=0.35)
    )
    
    # --- Wheels ---
    wheel_0 = model.part("wheel_0")
    wheel_0.visual(
        Cylinder(radius=0.075, length=0.04),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.5708, 0)),
        material=black_mat,
        name="wheel_cyl_0"
    )
    model.articulation(
        "base_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel_0,
        origin=Origin(xyz=(-0.27, 0.14, 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0)
    )

    wheel_1 = model.part("wheel_1")
    wheel_1.visual(
        Cylinder(radius=0.075, length=0.04),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.5708, 0)),
        material=black_mat,
        name="wheel_cyl_1"
    )
    model.articulation(
        "base_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel_1,
        origin=Origin(xyz=(0.27, 0.14, 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0)
    )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    cover_0 = object_model.get_part("organizer_cover_0")
    cover_1 = object_model.get_part("organizer_cover_1")
    
    # Allowances
    ctx.allow_coplanar_surfaces(lid, base, reason="Lid rests flush on base.")
    ctx.allow_coplanar_surfaces(cover_0, lid, reason="Cover rests flush in lid recess.")
    ctx.allow_coplanar_surfaces(cover_1, lid, reason="Cover rests flush in lid recess.")
    
    ctx.allow_overlap(base, wheel_0, elem_a="axle_left", elem_b="wheel_cyl_0", reason="Wheel mounts on base axle.")
    ctx.allow_overlap(base, wheel_1, elem_a="axle_right", elem_b="wheel_cyl_1", reason="Wheel mounts on base axle.")
    
    ctx.allow_overlap(base, handle, elem_a="base_shell", elem_b="grip", reason="Handle grip nests in base recess.")
    ctx.allow_overlap(base, handle, elem_a="base_shell", elem_b="tube_left", reason="Handle tube slides inside base.")
    ctx.allow_overlap(base, handle, elem_a="base_shell", elem_b="tube_right", reason="Handle tube slides inside base.")
    ctx.allow_overlap(lid, cover_0, elem_a="lid_shell", elem_b="cover_box_0", reason="Cover rests in lid recess.")
    ctx.allow_overlap(lid, cover_1, elem_a="lid_shell", elem_b="cover_box_1", reason="Cover rests in lid recess.")
    
    # Checks
    ctx.expect_gap(lid, base, axis="z", max_gap=0.001, max_penetration=0.001, name="lid rests on base")
    
    ctx.expect_within(cover_0, lid, axes="z", margin=0.001, name="cover 0 sits within lid height")
    ctx.expect_within(cover_1, lid, axes="z", margin=0.001, name="cover 1 sits within lid height")
    ctx.expect_within(cover_0, lid, axes="xy", margin=0.001, name="cover 0 within lid footprint")
    
    ctx.expect_within(handle, base, axes="xy", inner_elem="tube_left", outer_elem="base_shell", margin=0.001, name="left tube stays centered in base")
    ctx.expect_within(handle, base, axes="xy", inner_elem="tube_right", outer_elem="base_shell", margin=0.001, name="right tube stays centered in base")
    ctx.expect_overlap(handle, base, axes="z", elem_a="tube_left", elem_b="base_shell", min_overlap=0.1, name="left tube retains insertion at rest")
    
    # Articulation checks
    handle_joint = object_model.get_articulation("base_to_handle")
    with ctx.pose({handle_joint: 0.35}):
        ctx.expect_overlap(handle, base, axes="z", elem_a="tube_left", elem_b="base_shell", min_overlap=0.02, name="extended handle retains insertion in base")
        
    lid_joint = object_model.get_articulation("base_to_lid")
    with ctx.pose({lid_joint: 1.5}):
        lid_aabb = ctx.part_world_aabb(lid)
        ctx.check("lid_opens_up", lid_aabb is not None and lid_aabb[1][2] > 0.60, "Lid should move up when opened")
        
    return ctx.report()

object_model = build_object_model()