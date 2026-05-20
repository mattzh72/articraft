from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    wire_from_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _envelope_mesh():
    sections = [
        (-15.0, -0.20, 0.20, 0.35),
        (-14.0, -1.40, 1.55, 2.25),
        (-12.2, -2.85, 3.00, 5.25),
        (-9.0, -3.70, 3.85, 7.45),
        (-4.5, -4.10, 4.15, 8.35),
        (0.0, -4.25, 4.25, 8.70),
        (4.8, -4.05, 4.10, 8.30),
        (9.4, -3.25, 3.45, 6.75),
        (12.4, -1.90, 2.10, 3.90),
        (14.3, -0.62, 0.72, 1.15),
        (15.0, -0.14, 0.14, 0.30),
    ]
    geom = superellipse_side_loft(sections, exponents=2.25, segments=72)
    # The side-loft helper runs along local +Y; rotate it so the airship's
    # longitudinal axis is world +X and the nose points forward.
    geom.rotate_z(-math.pi / 2.0)
    return geom


def _gondola_mesh():
    sections = [
        (-2.70, -0.38, 0.32, 0.90),
        (-2.25, -0.72, 0.62, 1.85),
        (-1.25, -0.82, 0.72, 2.30),
        (0.85, -0.82, 0.72, 2.38),
        (2.10, -0.68, 0.58, 1.90),
        (2.65, -0.32, 0.28, 0.80),
    ]
    geom = superellipse_side_loft(sections, exponents=3.2, segments=36)
    geom.rotate_z(-math.pi / 2.0)
    return geom


def _support_struts_mesh():
    geom = MeshGeometry()
    struts = [
        ((-1.95, -0.90, -3.72), (-1.18, -0.82, -4.56)),
        ((-1.95, 0.90, -3.72), (-1.18, 0.82, -4.56)),
        ((1.95, -0.90, -3.72), (1.18, -0.82, -4.56)),
        ((1.95, 0.90, -3.72), (1.18, 0.82, -4.56)),
        ((-1.40, 0.0, -3.88), (1.40, 0.0, -4.50)),
    ]
    for a, b in struts:
        geom.merge(
            wire_from_points(
                [a, b],
                radius=0.075,
                radial_segments=14,
                cap_ends=True,
            )
        )
    return geom


def _rotor_mesh(name: str):
    rotor = FanRotorGeometry(
        outer_radius=0.58,
        hub_radius=0.15,
        blade_count=5,
        thickness=0.085,
        blade_pitch_deg=30.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.18),
        hub=FanRotorHub(style="spinner", bore_diameter=0.055),
    )
    return mesh_from_geometry(rotor, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    fabric = _mat(model, "warm_gray_fabric", (0.76, 0.77, 0.72, 1.0))
    seam = _mat(model, "darker_fabric_seams", (0.38, 0.40, 0.38, 1.0))
    frame = _mat(model, "dark_anodized_frame", (0.08, 0.09, 0.10, 1.0))
    cabin = _mat(model, "matte_utility_blue", (0.12, 0.20, 0.27, 1.0))
    glass = _mat(model, "smoked_glass", (0.03, 0.07, 0.09, 0.72))
    tail_mat = _mat(model, "tail_fabric_panel", (0.58, 0.61, 0.56, 1.0))
    pod_mat = _mat(model, "engine_pod_gray", (0.28, 0.30, 0.31, 1.0))
    prop_mat = _mat(model, "black_composite", (0.015, 0.015, 0.014, 1.0))

    hull = model.part("hull")
    hull.visual(
        mesh_from_geometry(_envelope_mesh(), "thick_envelope"),
        material=fabric,
        name="envelope",
    )
    hull.visual(
        Cylinder(radius=0.035, length=22.0),
        origin=Origin(xyz=(0.0, -4.38, -0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seam,
        name="port_belly_seam",
    )
    hull.visual(
        Cylinder(radius=0.035, length=22.0),
        origin=Origin(xyz=(0.0, 4.38, -0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seam,
        name="starboard_belly_seam",
    )
    hull.visual(
        mesh_from_geometry(_support_struts_mesh(), "gondola_support_struts"),
        material=frame,
        name="gondola_struts",
    )
    # Boxy cruciform stabilizers are fixed to the rear of the envelope.
    hull.visual(
        Box((3.2, 0.36, 2.65)),
        origin=Origin(xyz=(-13.18, 0.0, 3.95)),
        material=tail_mat,
        name="upper_fin",
    )
    hull.visual(
        Box((2.7, 0.34, 2.05)),
        origin=Origin(xyz=(-13.05, 0.0, -3.10)),
        material=tail_mat,
        name="lower_fin",
    )
    hull.visual(
        Box((3.2, 3.15, 0.28)),
        origin=Origin(xyz=(-13.18, 3.65, 0.22)),
        material=tail_mat,
        name="port_tailplane",
    )
    hull.visual(
        Box((3.2, 3.15, 0.28)),
        origin=Origin(xyz=(-13.18, -3.65, 0.22)),
        material=tail_mat,
        name="starboard_tailplane",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_geometry(_gondola_mesh(), "compact_gondola_shell"),
        material=cabin,
        name="gondola_shell",
    )
    gondola.visual(
        Box((0.95, 0.035, 0.42)),
        origin=Origin(xyz=(1.30, 1.155, 0.20)),
        material=glass,
        name="port_window_0",
    )
    gondola.visual(
        Box((0.95, 0.035, 0.42)),
        origin=Origin(xyz=(-0.05, 1.155, 0.22)),
        material=glass,
        name="port_window_1",
    )
    gondola.visual(
        Box((0.95, 0.035, 0.42)),
        origin=Origin(xyz=(1.30, -1.155, 0.20)),
        material=glass,
        name="starboard_window_0",
    )
    gondola.visual(
        Box((0.95, 0.035, 0.42)),
        origin=Origin(xyz=(-0.05, -1.155, 0.22)),
        material=glass,
        name="starboard_window_1",
    )
    gondola.visual(
        Box((1.05, 0.045, 0.50)),
        origin=Origin(xyz=(2.18, 0.0, 0.17), rpy=(0.0, 0.0, 0.0)),
        material=glass,
        name="front_windscreen",
    )
    gondola.visual(
        Box((2.85, 1.88, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=frame,
        name="roof_hardpoints",
    )
    gondola.visual(
        Box((5.10, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, 1.26, -0.48)),
        material=frame,
        name="port_skid_rail",
    )
    gondola.visual(
        Box((5.10, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -1.26, -0.48)),
        material=frame,
        name="starboard_skid_rail",
    )
    for bracket_x in (-1.85, 1.85):
        gondola.visual(
            Box((0.18, 0.34, 0.55)),
            origin=Origin(xyz=(bracket_x, 1.16, -0.25)),
            material=frame,
            name=f"port_skid_bracket_{bracket_x:+.0f}".replace("+", "p").replace("-", "n"),
        )
        gondola.visual(
            Box((0.18, 0.34, 0.55)),
            origin=Origin(xyz=(bracket_x, -1.16, -0.25)),
            material=frame,
            name=f"starboard_skid_bracket_{bracket_x:+.0f}".replace("+", "p").replace("-", "n"),
        )
    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -5.34)),
    )

    for side_name, side in (("port", 1.0), ("starboard", -1.0)):
        pylon = model.part(f"{side_name}_pylon")
        pylon.visual(
            Box((0.52, 0.16, 0.46)),
            origin=Origin(xyz=(0.0, 0.0, 0.25)),
            material=frame,
            name="side_mount",
        )
        pylon.visual(
            Cylinder(radius=0.105, length=1.32),
            origin=Origin(xyz=(0.0, side * 0.66, 0.02), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=frame,
            name="pylon_tube",
        )
        pylon.visual(
            Box((0.34, 0.22, 0.24)),
            origin=Origin(xyz=(0.0, side * 1.34, 0.17)),
            material=frame,
            name="upper_yoke_lug",
        )
        pylon.visual(
            Box((0.34, 0.22, 0.24)),
            origin=Origin(xyz=(0.0, side * 1.34, -0.17)),
            material=frame,
            name="lower_yoke_lug",
        )
        model.articulation(
            f"gondola_to_{side_name}_pylon",
            ArticulationType.FIXED,
            parent=gondola,
            child=pylon,
            origin=Origin(xyz=(-1.40, side * 1.232, -0.20)),
        )

        pod = model.part(f"{side_name}_pod")
        pod.visual(
            mesh_from_geometry(CapsuleGeometry(0.43, 1.20, radial_segments=32), f"{side_name}_pod_nacelle"),
            origin=Origin(xyz=(0.0, side * 0.56, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pod_mat,
            name="nacelle",
        )
        pod.visual(
            Cylinder(radius=0.18, length=0.34),
            origin=Origin(xyz=(0.0, side * 0.04, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=frame,
            name="pivot_socket",
        )
        pod.visual(
            Cylinder(radius=0.075, length=0.18),
            origin=Origin(xyz=(1.03, side * 0.56, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=frame,
            name="shaft_boss",
        )
        model.articulation(
            f"{side_name}_pylon_to_pod",
            ArticulationType.REVOLUTE,
            parent=pylon,
            child=pod,
            origin=Origin(xyz=(0.0, side * 1.34, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=280.0, velocity=0.65, lower=-0.55, upper=0.55),
        )

        prop = model.part(f"{side_name}_propeller")
        prop.visual(
            _rotor_mesh(f"{side_name}_propeller_rotor"),
            origin=Origin(xyz=(0.15, side * 0.56, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_mat,
            name="rotor_blades",
        )
        prop.visual(
            Cylinder(radius=0.055, length=0.25),
            origin=Origin(xyz=(0.06, side * 0.56, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=frame,
            name="propeller_shaft",
        )
        model.articulation(
            f"{side_name}_pod_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=prop,
            origin=Origin(xyz=(1.12, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=85.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        Box((1.08, 0.24, 2.18)),
        origin=Origin(xyz=(-0.54, 0.0, 0.0)),
        material=tail_mat,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.075, length=2.28),
        origin=Origin(xyz=(-0.12, 0.0, 0.0)),
        material=frame,
        name="rudder_hinge_barrel",
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-14.78, 0.0, 3.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.2, lower=-0.58, upper=0.58),
    )

    for side_name, side in (("port", 1.0), ("starboard", -1.0)):
        elevator = model.part(f"{side_name}_elevator")
        elevator.visual(
            Box((1.02, 2.92, 0.22)),
            origin=Origin(xyz=(-0.51, 0.0, 0.0)),
            material=tail_mat,
            name="elevator_panel",
        )
        elevator.visual(
            Cylinder(radius=0.065, length=2.98),
            origin=Origin(xyz=(-0.10, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=frame,
            name="elevator_hinge_barrel",
        )
        model.articulation(
            f"hull_to_{side_name}_elevator",
            ArticulationType.REVOLUTE,
            parent=hull,
            child=elevator,
            origin=Origin(xyz=(-14.78, side * 3.65, 0.22)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.45, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")

    ctx.allow_overlap(
        gondola,
        hull,
        elem_a="roof_hardpoints",
        elem_b="gondola_struts",
        reason="The fixed support struts are intentionally seated into the gondola roof sockets.",
    )
    ctx.expect_gap(
        hull,
        gondola,
        axis="z",
        positive_elem="envelope",
        negative_elem="gondola_shell",
        min_gap=0.22,
        max_gap=0.72,
        name="gondola hangs below the envelope",
    )
    ctx.expect_overlap(
        hull,
        gondola,
        axes="xy",
        elem_a="gondola_struts",
        elem_b="roof_hardpoints",
        min_overlap=0.55,
        name="struts land over the gondola roof hardpoints",
    )

    for side_name in ("port", "starboard"):
        pylon = object_model.get_part(f"{side_name}_pylon")
        pod = object_model.get_part(f"{side_name}_pod")
        prop = object_model.get_part(f"{side_name}_propeller")
        pod_pivot = object_model.get_articulation(f"{side_name}_pylon_to_pod")
        prop_joint = object_model.get_articulation(f"{side_name}_pod_to_propeller")

        ctx.allow_overlap(
            gondola,
            pylon,
            elem_a="gondola_shell",
            elem_b="side_mount",
            reason="The pylon side plate is intentionally seated into the gondola side structure.",
        )
        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="upper_yoke_lug",
            elem_b="pivot_socket",
            reason="The pylon yoke lugs intentionally wrap the pod trunnion socket.",
        )
        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="lower_yoke_lug",
            elem_b="pivot_socket",
            reason="The pylon yoke lugs intentionally wrap the pod trunnion socket.",
        )
        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="pylon_tube",
            elem_b="pivot_socket",
            reason="The main pylon tube is intentionally seated into the pod trunnion socket.",
        )
        ctx.allow_overlap(
            pod,
            prop,
            elem_a="shaft_boss",
            elem_b="propeller_shaft",
            reason="The propeller shaft is intentionally captured inside the nacelle nose boss.",
        )
        ctx.check(
            f"{side_name} propeller has a continuous shaft joint",
            prop_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(prop_joint.articulation_type),
        )
        ctx.expect_contact(
            gondola,
            pylon,
            elem_a="gondola_shell",
            elem_b="side_mount",
            contact_tol=0.08,
            name=f"{side_name} pylon is mounted on the gondola side",
        )
        ctx.expect_contact(
            pylon,
            pod,
            elem_a="upper_yoke_lug",
            elem_b="pivot_socket",
            contact_tol=0.08,
            name=f"{side_name} pod is carried at the pylon yoke",
        )
        ctx.expect_contact(
            pylon,
            pod,
            elem_a="lower_yoke_lug",
            elem_b="pivot_socket",
            contact_tol=0.08,
            name=f"{side_name} lower yoke also captures the pod socket",
        )
        ctx.expect_overlap(
            pylon,
            pod,
            axes="yz",
            elem_a="pylon_tube",
            elem_b="pivot_socket",
            min_overlap=0.06,
            name=f"{side_name} trunnion socket is retained on the pylon tube",
        )
        ctx.expect_overlap(
            pod,
            prop,
            axes="yz",
            elem_a="shaft_boss",
            elem_b="propeller_shaft",
            min_overlap=0.045,
            name=f"{side_name} propeller shaft stays coaxial with pod boss",
        )
        ctx.expect_overlap(
            pod,
            prop,
            axes="x",
            elem_a="shaft_boss",
            elem_b="propeller_shaft",
            min_overlap=0.035,
            name=f"{side_name} propeller shaft remains inserted in the nose boss",
        )

        rest_aabb = ctx.part_element_world_aabb(pod, elem="nacelle")
        with ctx.pose({pod_pivot: 0.45}):
            tilted_aabb = ctx.part_element_world_aabb(pod, elem="nacelle")
        ctx.check(
            f"{side_name} engine pod visibly vectors",
            rest_aabb is not None
            and tilted_aabb is not None
            and (tilted_aabb[1][2] - rest_aabb[1][2] > 0.08 or rest_aabb[0][2] - tilted_aabb[0][2] > 0.08),
            details=f"rest={rest_aabb}, tilted={tilted_aabb}",
        )

    rudder = object_model.get_part("rudder")
    rudder_joint = object_model.get_articulation("hull_to_rudder")
    rest_rudder = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_joint: 0.42}):
        turned_rudder = ctx.part_world_aabb(rudder)
    ctx.check(
        "rudder swings about the vertical rear-fin hinge",
        rest_rudder is not None
        and turned_rudder is not None
        and (turned_rudder[1][1] - turned_rudder[0][1]) > (rest_rudder[1][1] - rest_rudder[0][1]) + 0.28,
        details=f"rest={rest_rudder}, turned={turned_rudder}",
    )

    for side_name in ("port", "starboard"):
        elevator = object_model.get_part(f"{side_name}_elevator")
        elevator_joint = object_model.get_articulation(f"hull_to_{side_name}_elevator")
        rest_elevator = ctx.part_world_aabb(elevator)
        with ctx.pose({elevator_joint: 0.35}):
            deflected_elevator = ctx.part_world_aabb(elevator)
        ctx.check(
            f"{side_name} elevator deflects on a horizontal hinge",
            rest_elevator is not None
            and deflected_elevator is not None
            and (deflected_elevator[1][2] - deflected_elevator[0][2])
            > (rest_elevator[1][2] - rest_elevator[0][2]) + 0.15,
            details=f"rest={rest_elevator}, deflected={deflected_elevator}",
        )

    return ctx.report()


object_model = build_object_model()
