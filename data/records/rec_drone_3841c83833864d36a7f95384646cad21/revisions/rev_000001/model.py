import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
    mesh_from_geometry,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quadcopter")

    # Materials
    mat_body = Material(name="body_mat", rgba=(0.85, 0.85, 0.85, 1.0))
    mat_dark = Material(name="dark_mat", rgba=(0.15, 0.15, 0.15, 1.0))
    mat_arm = Material(name="arm_mat", rgba=(0.2, 0.2, 0.2, 1.0))
    mat_prop = Material(name="prop_mat", rgba=(0.1, 0.1, 0.1, 1.0))
    mat_accent = Material(name="accent_mat", rgba=(0.9, 0.1, 0.1, 1.0))

    # 1. Body
    body = model.part("body")
    
    # Central hub
    body_cq = (
        cq.Workplane("XY")
        .box(0.10, 0.10, 0.03)
        .edges("|Z").fillet(0.02)
        .edges("|X or |Y").fillet(0.005)
    )
    
    # Top battery cover
    top_cover = (
        cq.Workplane("XY")
        .workplane(offset=0.015)
        .rect(0.06, 0.04)
        .extrude(0.01)
        .edges(">Z").fillet(0.002)
    )
    body_cq = body_cq.union(top_cover)
    body.visual(mesh_from_cadquery(body_cq, "body_core"), material=mat_body, name="body_core")
    
    # 2. Arms
    arm_cq = (
        cq.Workplane("YZ")
        .rect(0.016, 0.012)
        .extrude(0.12)
        .edges("|X").fillet(0.005)
    )
    motor_mount = (
        cq.Workplane("XY")
        .center(0.12, 0)
        .cylinder(0.02, 0.015)
    )
    arm_cq = arm_cq.union(motor_mount)
    arm_mesh = mesh_from_cadquery(arm_cq, "arm")
    
    # FL: +X, +Y
    body.visual(arm_mesh, origin=Origin(xyz=(0.04, 0.04, 0), rpy=(0, 0, math.radians(45))), material=mat_arm, name="arm_fl")
    # FR: +X, -Y
    body.visual(arm_mesh, origin=Origin(xyz=(0.04, -0.04, 0), rpy=(0, 0, math.radians(-45))), material=mat_arm, name="arm_fr")
    # RL: -X, +Y
    body.visual(arm_mesh, origin=Origin(xyz=(-0.04, 0.04, 0), rpy=(0, 0, math.radians(135))), material=mat_arm, name="arm_rl")
    # RR: -X, -Y
    body.visual(arm_mesh, origin=Origin(xyz=(-0.04, -0.04, 0), rpy=(0, 0, math.radians(-135))), material=mat_arm, name="arm_rr")
    
    # 3. Landing Skids
    skid_leg1 = cq.Workplane("XY").center(0.03, 0).cylinder(0.06, 0.003).translate((0, 0, -0.03))
    skid_leg2 = cq.Workplane("XY").center(-0.03, 0).cylinder(0.06, 0.003).translate((0, 0, -0.03))
    skid_tube = cq.Workplane("YZ").cylinder(0.12, 0.004).translate((0, 0, -0.06))
    skid_cq = skid_leg1.union(skid_leg2).union(skid_tube)
    skid_mesh = mesh_from_cadquery(skid_cq, "skid")
    
    body.visual(skid_mesh, origin=Origin(xyz=(0, 0.04, 0)), material=mat_dark, name="skid_left")
    body.visual(skid_mesh, origin=Origin(xyz=(0, -0.04, 0)), material=mat_dark, name="skid_right")
    
    # 4. Propellers
    prop_geom = FanRotorGeometry(
        outer_radius=0.08,
        hub_radius=0.012,
        blade_count=2,
        thickness=0.004,
        blade_pitch_deg=15.0,
        blade_sweep_deg=5.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.05),
        hub=FanRotorHub(style="spinner", bore_diameter=0.004),
        center=False,
    )
    prop_mesh = mesh_from_geometry(prop_geom, "propeller")
    
    def add_prop(name, x_sign, y_sign, spin_dir):
        prop = model.part(name)
        ox, oy = 0.04 * x_sign, 0.04 * y_sign
        if x_sign > 0 and y_sign > 0: angle = 45
        elif x_sign > 0 and y_sign < 0: angle = -45
        elif x_sign < 0 and y_sign > 0: angle = 135
        else: angle = -135
        
        hx = ox + 0.12 * math.cos(math.radians(angle))
        hy = oy + 0.12 * math.sin(math.radians(angle))
        hz = 0.01
        
        prop.visual(prop_mesh, origin=Origin(), material=mat_prop, name=f"{name}_visual")
        
        model.articulation(
            f"{name}_joint",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=prop,
            origin=Origin(xyz=(hx, hy, hz)),
            axis=(0, 0, spin_dir),
            motion_limits=MotionLimits(effort=1.0, velocity=100.0),
        )
        return prop
        
    prop_fl = add_prop("prop_fl", 1, 1, 1)
    prop_fr = add_prop("prop_fr", 1, -1, -1)
    prop_rl = add_prop("prop_rl", -1, 1, -1)
    prop_rr = add_prop("prop_rr", -1, -1, 1)
    
    # 5. Gimbal and Camera
    gimbal_yaw = model.part("gimbal_yaw")
    gimbal_yaw_cq = (
        cq.Workplane("XY")
        .cylinder(0.005, 0.015)
        .translate((0, 0, -0.0025))
    )
    arm1 = cq.Workplane("XY").box(0.01, 0.004, 0.025).translate((0, 0.0125, -0.0125))
    arm2 = cq.Workplane("XY").box(0.01, 0.004, 0.025).translate((0, -0.0125, -0.0125))
    gimbal_yaw_cq = gimbal_yaw_cq.union(arm1).union(arm2)
    gimbal_yaw.visual(mesh_from_cadquery(gimbal_yaw_cq, "yaw_bracket"), material=mat_dark, name="yaw_bracket")
    
    model.articulation(
        "gimbal_yaw_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=gimbal_yaw,
        origin=Origin(xyz=(0.04, 0, -0.015)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-math.pi/2, upper=math.pi/2),
    )
    
    camera = model.part("camera")
    camera_cq = (
        cq.Workplane("XY")
        .box(0.02, 0.02, 0.02)
        .edges().fillet(0.002)
    )
    lens = (
        cq.Workplane("YZ")
        .cylinder(0.008, 0.008)
        .translate((0.014, 0, 0))
    )
    axle = (
        cq.Workplane("XZ")
        .cylinder(0.026, 0.002)
    )
    camera_cq = camera_cq.union(lens).union(axle)
    camera.visual(mesh_from_cadquery(camera_cq, "camera_body"), material=mat_dark, name="camera_body")
    
    lens_accent = (
        cq.Workplane("YZ")
        .cylinder(0.002, 0.006)
        .translate((0.019, 0, 0))
    )
    camera.visual(mesh_from_cadquery(lens_accent, "lens_accent"), material=mat_accent, name="lens_accent")
    
    model.articulation(
        "gimbal_pitch_joint",
        ArticulationType.REVOLUTE,
        parent=gimbal_yaw,
        child=camera,
        origin=Origin(xyz=(0, 0, -0.02)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-math.pi/4, upper=math.pi/4),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    
    for name in ["prop_fl", "prop_fr", "prop_rl", "prop_rr"]:
        prop = object_model.get_part(name)
        ctx.expect_contact(prop, body, name=f"{name} contacts body")
        
    yaw = object_model.get_part("gimbal_yaw")
    cam = object_model.get_part("camera")
    ctx.expect_contact(yaw, body, name="yaw bracket contacts body")
    
    ctx.allow_overlap(
        cam, yaw,
        reason="Camera pitch axle is intentionally captured inside the gimbal yaw bracket arms."
    )
    ctx.expect_within(cam, yaw, axes="y", name="camera within yaw bracket")
    
    return ctx.report()

object_model = build_object_model()