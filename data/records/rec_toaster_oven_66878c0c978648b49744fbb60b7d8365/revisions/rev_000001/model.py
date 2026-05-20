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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toaster_oven")
    
    # Colors & Materials
    body_mat = Material(name="body_mat", rgba=(0.2, 0.2, 0.22, 1.0))
    glass_mat = Material(name="glass_mat", rgba=(0.1, 0.1, 0.1, 0.5))
    metal_mat = Material(name="metal_mat", rgba=(0.8, 0.8, 0.8, 1.0))
    plastic_mat = Material(name="plastic_mat", rgba=(0.05, 0.05, 0.05, 1.0))
    heater_mat = Material(name="heater_mat", rgba=(0.3, 0.3, 0.3, 1.0))

    # Dimensions
    body_w = 0.45
    body_d = 0.32
    body_h = 0.28
    
    cavity_w = 0.32
    cavity_d = 0.30
    cavity_h = 0.20
    
    # Body
    body_outer = cq.Workplane("XY").box(body_w, body_d, body_h).edges().fillet(0.02)
    # Cavity offset: X center = -0.05, Y center = -0.01 (to cut through front)
    cavity = cq.Workplane("XY").box(cavity_w, cavity_d + 0.02, cavity_h).translate((-0.05, -0.01, 0))
    body_shape = body_outer.cut(cavity)
    
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shape, "body_shape"),
        origin=Origin(xyz=(0, 0, 0.16)),
        material=body_mat,
        name="body_shell"
    )

    # Feet
    foot_shape = cq.Workplane("XY").cylinder(0.02, 0.015)
    for i, (fx, fy) in enumerate([
        (-0.20, -0.13),
        (0.20, -0.13),
        (-0.20, 0.13),
        (0.20, 0.13)
    ]):
        body.visual(
            mesh_from_cadquery(foot_shape, f"foot_{i}"),
            origin=Origin(xyz=(fx, fy, 0.01)),
            material=plastic_mat,
            name=f"foot_{i}"
        )
        
    # Heating elements
    heater_shape = cq.Workplane("YZ").cylinder(0.34, 0.004)
    for i, (hy, hz) in enumerate([
        (-0.06, 0.24), (0.06, 0.24), # top
        (-0.06, 0.08), (0.06, 0.08)  # bottom
    ]):
        body.visual(
            mesh_from_cadquery(heater_shape, f"heater_{i}"),
            origin=Origin(xyz=(-0.05, hy, hz)),
            material=heater_mat,
            name=f"heater_{i}"
        )
        
    # Rack
    rack_shape = cq.Workplane("XY").box(0.33, 0.27, 0.004)
    rack_cut = cq.Workplane("XY").box(0.31, 0.25, 0.01)
    rack_shape = rack_shape.cut(rack_cut)
    for wx in [-0.1, 0, 0.1]:
        rack_shape = rack_shape.union(cq.Workplane("XY").box(0.004, 0.27, 0.004).translate((wx, 0, 0)))
        
    body.visual(
        mesh_from_cadquery(rack_shape, "rack_shape"),
        origin=Origin(xyz=(-0.05, 0.0, 0.15)),
        material=metal_mat,
        name="rack"
    )
    
    # Crumb Tray
    tray_shape = cq.Workplane("XY").box(0.33, 0.27, 0.006).edges("|Z").fillet(0.002)
    body.visual(
        mesh_from_cadquery(tray_shape, "tray_shape"),
        origin=Origin(xyz=(-0.05, 0.0, 0.065)),
        material=metal_mat,
        name="crumb_tray"
    )

    # Door
    door = model.part("door")
    
    door_frame_cq = cq.Workplane("XY").box(0.33, 0.02, 0.23)
    door_glass_cut = cq.Workplane("XY").box(0.28, 0.03, 0.17)
    door_frame_cq = door_frame_cq.cut(door_glass_cut)
    
    door_glass_cq = cq.Workplane("XY").box(0.28, 0.005, 0.17)
    
    handle_cq = (
        cq.Workplane("XY").box(0.20, 0.015, 0.015).translate((0, -0.03, 0.09))
        .union(cq.Workplane("XY").box(0.015, 0.03, 0.015).translate((-0.09, -0.015, 0.09)))
        .union(cq.Workplane("XY").box(0.015, 0.03, 0.015).translate((0.09, -0.015, 0.09)))
    )
    
    door.visual(
        mesh_from_cadquery(door_frame_cq, "door_frame"),
        origin=Origin(xyz=(0, -0.01, 0.115)),
        material=metal_mat,
        name="door_frame"
    )
    door.visual(
        mesh_from_cadquery(door_glass_cq, "door_glass"),
        origin=Origin(xyz=(0, -0.01, 0.115)),
        material=glass_mat,
        name="door_glass"
    )
    door.visual(
        mesh_from_cadquery(handle_cq, "door_handle"),
        origin=Origin(xyz=(0, -0.01, 0.115)),
        material=metal_mat,
        name="door_handle"
    )
    
    model.articulation(
        name="door_hinge",
        articulation_type=ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.05, -0.16, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57, effort=5.0, velocity=1.0)
    )
    
    # Knobs
    knob_shape_y = (
        cq.Workplane("XZ").cylinder(0.015, 0.015)
        .union(cq.Workplane("XZ").box(0.005, 0.03, 0.016))
    )
    
    knob_z_positions = [0.23, 0.16, 0.09]
    knob_names = ["knob_temp", "knob_func", "knob_timer"]
    
    for kz, kname in zip(knob_z_positions, knob_names):
        knob = model.part(kname)
        knob.visual(
            mesh_from_cadquery(knob_shape_y, f"{kname}_shape"),
            origin=Origin(xyz=(0, -0.0075, 0)),
            material=plastic_mat,
            name=f"{kname}_visual"
        )
        model.articulation(
            name=f"{kname}_joint",
            articulation_type=ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.1675, -0.16, kz)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=3.14, effort=1.0, velocity=5.0)
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    
    ctx.expect_gap(body, door, axis="y", max_gap=0.002, max_penetration=0.002, name="door_closes_against_body")
    
    for kname in ["knob_temp", "knob_func", "knob_timer"]:
        knob = object_model.get_part(kname)
        ctx.expect_gap(body, knob, axis="y", max_gap=0.002, max_penetration=0.002, name=f"{kname}_mounted_on_body")
        
    return ctx.report()


object_model = build_object_model()
