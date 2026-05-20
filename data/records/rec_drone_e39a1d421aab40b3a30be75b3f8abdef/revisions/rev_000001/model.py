import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
    mesh_from_geometry,
    Material,
    Cylinder,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drone")

    mat_body = Material("body_mat", rgba=(0.2, 0.2, 0.2, 1.0))
    mat_arm = Material("arm_mat", rgba=(0.8, 0.8, 0.8, 1.0))
    mat_rotor = Material("rotor_mat", rgba=(0.1, 0.1, 0.1, 1.0))
    mat_skid = Material("skid_mat", rgba=(0.15, 0.15, 0.15, 1.0))
    mat_cam = Material("cam_mat", rgba=(0.1, 0.1, 0.1, 1.0))
    mat_lens = Material("lens_mat", rgba=(0.1, 0.3, 0.8, 1.0))

    hx, hy = 0.06, 0.035

    # 1. Body
    body = model.part("body")
    body_base = (
        cq.Workplane("XY")
        .box(0.16, 0.04, 0.03)
        .edges("|Z")
        .fillet(0.01)
    )
    bosses = (
        cq.Workplane("XY")
        .pushPoints([(hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)])
        .cylinder(0.03, 0.015)
    )
    slots = (
        cq.Workplane("XY")
        .pushPoints([(hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)])
        .cylinder(0.012, 0.016)
    )
    body_cq = body_base.union(bosses).cut(slots)
    body.visual(mesh_from_cadquery(body_cq, "body_shell"), material=mat_body, name="shell")

    # Skids
    skid_cq = (
        cq.Workplane("XZ")
        .moveTo(-0.06, -0.015)
        .lineTo(0.06, -0.015)
        .lineTo(0.07, -0.04)
        .lineTo(0.06, -0.04)
        .lineTo(0.05, -0.025)
        .lineTo(-0.05, -0.025)
        .lineTo(-0.06, -0.04)
        .lineTo(-0.07, -0.04)
        .close()
        .extrude(0.005, both=True)
    )
    body.visual(mesh_from_cadquery(skid_cq, "skid_left"), origin=Origin(xyz=(0, 0.02, 0)), material=mat_skid, name="skid_left")
    body.visual(mesh_from_cadquery(skid_cq, "skid_right"), origin=Origin(xyz=(0, -0.02, 0)), material=mat_skid, name="skid_right")

    # Camera
    camera = model.part("camera")
    cam_cq = (
        cq.Workplane("XY")
        .box(0.02, 0.02, 0.02)
        .edges()
        .fillet(0.005)
    )
    camera.visual(mesh_from_cadquery(cam_cq, "camera_shell"), origin=Origin(xyz=(0.01, 0, 0)), material=mat_cam, name="shell")
    camera.visual(Cylinder(0.008, 0.005), origin=Origin(xyz=(0.0225, 0, 0), rpy=(0, math.pi/2, 0)), material=mat_lens, name="lens")

    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera,
        origin=Origin(xyz=(0.08, 0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=0.5)
    )

    # 2. Arms
    arm_len = 0.08
    arm_base_cq = (
        cq.Workplane("XY")
        .cylinder(0.012, 0.015)
    )
    arm_cq = (
        cq.Workplane("XY")
        .center(arm_len/2, 0)
        .box(arm_len, 0.015, 0.01)
    )
    motor_pod_cq = (
        cq.Workplane("XY")
        .center(arm_len, 0)
        .cylinder(0.015, 0.012)
    )
    arm_solid = arm_base_cq.union(arm_cq).union(motor_pod_cq)
    arm_mesh = mesh_from_cadquery(arm_solid, "arm")

    rotor_geom = FanRotorGeometry(
        outer_radius=0.06,
        hub_radius=0.01,
        blade_count=2,
        thickness=0.005,
        blade_pitch_deg=15.0,
        blade=FanRotorBlade(shape="straight"),
        hub=FanRotorHub(style="spinner", bore_diameter=0.002)
    )
    rotor_mesh = mesh_from_geometry(rotor_geom, "rotor")

    arm_configs = [
        ("front_left", hx, hy, math.pi/4, 0.0, 2.36),
        ("front_right", hx, -hy, -math.pi/4, -2.36, 0.0),
        ("rear_left", -hx, hy, 3*math.pi/4, -2.36, 0.0),
        ("rear_right", -hx, -hy, -3*math.pi/4, 0.0, 2.36)
    ]

    for name, x, y, angle, lower, upper in arm_configs:
        arm = model.part(f"arm_{name}")
        arm.visual(arm_mesh, material=mat_arm, name="shell")

        model.articulation(
            f"hinge_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(x, y, 0.0), rpy=(0, 0, angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=lower, upper=upper)
        )

        rotor = model.part(f"rotor_{name}")
        rotor.visual(rotor_mesh, material=mat_rotor, name="prop")

        model.articulation(
            f"spin_{name}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=rotor,
            origin=Origin(xyz=(arm_len, 0, 0.0075)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=10.0, velocity=100.0)
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "camera",
        "body",
        reason="The camera base is intentionally modeled as partially recessed into the nose of the body during tilt.",
    )

    return ctx.report()


object_model = build_object_model()