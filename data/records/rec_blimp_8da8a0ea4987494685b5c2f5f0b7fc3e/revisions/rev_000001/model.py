import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Sphere,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
)
from sdk import (
    LatheGeometry,
    CapsuleGeometry,
    mesh_from_geometry,
)

def make_envelope():
    profile = []
    steps = 32
    for i in range(steps + 1):
        t = math.pi * (1.0 - i / steps)
        z = 10.0 * math.cos(t)
        r = 2.5 * math.sin(t)
        profile.append((r, z))
    geom = LatheGeometry(profile, segments=32)
    geom.rotate_y(math.pi / 2)
    return mesh_from_geometry(geom, "envelope")

def make_pod():
    geom = CapsuleGeometry(radius=0.3, length=0.8)
    geom.rotate_y(math.pi / 2)
    return mesh_from_geometry(geom, "pod")

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blimp")

    # Materials
    mat_env = Material("envelope_mat", color=(0.9, 0.9, 0.9))
    mat_gon = Material("gondola_mat", color=(0.2, 0.4, 0.8))
    mat_fin = Material("fin_mat", color=(0.8, 0.2, 0.2))
    mat_prop = Material("prop_mat", color=(0.1, 0.1, 0.1))

    # Envelope
    envelope = model.part("envelope")
    envelope.visual(make_envelope(), material=mat_env, name="envelope_shell")

    # Gondola
    gondola = model.part("gondola")
    gondola.visual(Box((4.0, 1.5, 1.5)), material=mat_gon, name="gondola_shell")
    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        "envelope",
        "gondola",
        origin=Origin(xyz=(0.0, 0.0, -3.05))
    )

    # Pods
    left_pod = model.part("left_pod")
    left_pod.visual(make_pod(), material=mat_gon, name="left_pod_shell")
    model.articulation(
        "gondola_to_left_pod",
        ArticulationType.FIXED,
        "gondola",
        "left_pod",
        origin=Origin(xyz=(0.0, 1.0, 0.0))
    )

    right_pod = model.part("right_pod")
    right_pod.visual(make_pod(), material=mat_gon, name="right_pod_shell")
    model.articulation(
        "gondola_to_right_pod",
        ArticulationType.FIXED,
        "gondola",
        "right_pod",
        origin=Origin(xyz=(0.0, -1.0, 0.0))
    )

    # Propellers
    def add_propeller(part_name, parent_name, joint_name):
        prop = model.part(part_name)
        prop.visual(Sphere(radius=0.15), material=mat_prop, name=f"{part_name}_spinner")
        prop.visual(Box((0.05, 0.4, 0.1)), material=mat_prop, name=f"{part_name}_blade_1")
        prop.visual(Box((0.05, 0.1, 0.4)), material=mat_prop, name=f"{part_name}_blade_2")
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent_name,
            part_name,
            origin=Origin(xyz=(0.72, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=10.0)
        )

    add_propeller("left_propeller", "left_pod", "left_pod_to_propeller")
    add_propeller("right_propeller", "right_pod", "right_pod_to_propeller")

    # Fins and Rudders/Elevators
    def add_fin_and_panel(fin_name, panel_name, fin_pos, fin_size, panel_size, axis):
        fin = model.part(fin_name)
        fin.visual(Box(fin_size), material=mat_fin, name=f"{fin_name}_shell")
        model.articulation(
            f"envelope_to_{fin_name}",
            ArticulationType.FIXED,
            "envelope",
            fin_name,
            origin=Origin(xyz=fin_pos)
        )

        panel = model.part(panel_name)
        panel.visual(Box(panel_size), material=mat_fin, origin=Origin(xyz=(-0.5, 0.0, 0.0)), name=f"{panel_name}_shell")
        model.articulation(
            f"{fin_name}_to_{panel_name}",
            ArticulationType.REVOLUTE,
            fin_name,
            panel_name,
            origin=Origin(xyz=(-1.02, 0.0, 0.0)),
            axis=axis,
            motion_limits=MotionLimits(lower=-0.5, upper=0.5)
        )

    # Top Fin (Vertical)
    add_fin_and_panel(
        "top_fin", "top_rudder",
        fin_pos=(-8.0, 0.0, 2.0),
        fin_size=(2.0, 0.1, 2.0),
        panel_size=(1.0, 0.1, 2.0),
        axis=(0.0, 0.0, 1.0)
    )

    # Bottom Fin (Vertical)
    add_fin_and_panel(
        "bottom_fin", "bottom_rudder",
        fin_pos=(-8.0, 0.0, -2.0),
        fin_size=(2.0, 0.1, 2.0),
        panel_size=(1.0, 0.1, 2.0),
        axis=(0.0, 0.0, 1.0)
    )

    # Left Fin (Horizontal)
    add_fin_and_panel(
        "left_fin", "left_elevator",
        fin_pos=(-8.0, 2.0, 0.0),
        fin_size=(2.0, 2.0, 0.1),
        panel_size=(1.0, 2.0, 0.1),
        axis=(0.0, 1.0, 0.0)
    )

    # Right Fin (Horizontal)
    add_fin_and_panel(
        "right_fin", "right_elevator",
        fin_pos=(-8.0, -2.0, 0.0),
        fin_size=(2.0, 2.0, 0.1),
        panel_size=(1.0, 2.0, 0.1),
        axis=(0.0, 1.0, 0.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap("gondola", "envelope", reason="Gondola is embedded in the bottom of the envelope")
    ctx.allow_overlap("left_pod", "gondola", reason="Pod is mounted into the side of the gondola")
    ctx.allow_overlap("right_pod", "gondola", reason="Pod is mounted into the side of the gondola")
    ctx.allow_overlap("left_propeller", "left_pod", reason="Propeller spinner overlaps the pod nose")
    ctx.allow_overlap("right_propeller", "right_pod", reason="Propeller spinner overlaps the pod nose")
    
    for fin in ["top_fin", "bottom_fin", "left_fin", "right_fin"]:
        ctx.allow_overlap(fin, "envelope", reason="Fin is embedded in the envelope surface")
        
    for panel in ["top_rudder", "bottom_rudder", "left_elevator", "right_elevator"]:
        ctx.allow_overlap(panel, "envelope", reason="Control panel clips the tapering envelope near the hinge")

    ctx.expect_gap("left_pod", "gondola", axis="y", max_penetration=0.06)
    ctx.expect_gap("gondola", "right_pod", axis="y", max_penetration=0.06)

    ctx.expect_gap("left_propeller", "left_pod", axis="x", max_penetration=0.15)
    ctx.expect_gap("right_propeller", "right_pod", axis="x", max_penetration=0.15)

    # Note: rudder is at -9.02, fin is at -8.0.
    # So fin is on the positive side of rudder along X.
    ctx.expect_gap("top_fin", "top_rudder", axis="x", min_gap=0.01)
    ctx.expect_gap("bottom_fin", "bottom_rudder", axis="x", min_gap=0.01)
    ctx.expect_gap("left_fin", "left_elevator", axis="x", min_gap=0.01)
    ctx.expect_gap("right_fin", "right_elevator", axis="x", min_gap=0.01)

    return ctx.report()


object_model = build_object_model()
