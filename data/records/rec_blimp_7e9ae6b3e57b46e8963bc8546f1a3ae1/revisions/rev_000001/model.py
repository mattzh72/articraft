from __future__ import annotations

from math import cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    FanRotorShroud,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _airship_section(x: float, ry: float, rz: float, *, segments: int = 64):
    """Closed loop in the YZ plane used for the blimp envelope."""
    return [
        (x, ry * cos(2.0 * pi * i / segments), rz * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _rounded_envelope_mesh():
    # Event-airship scale: roughly 18 m long and 5 m across, with a fuller
    # advertising belly and a slightly tapered aft cone for the cruciform tail.
    sections = []
    for x, scale in (
        (-9.00, 0.045),
        (-8.55, 0.18),
        (-7.45, 0.55),
        (-5.40, 0.88),
        (-2.40, 1.00),
        (2.20, 1.00),
        (5.50, 0.82),
        (7.55, 0.48),
        (8.55, 0.17),
        (9.00, 0.045),
    ):
        sections.append(_airship_section(x, 2.55 * scale, 2.20 * scale))
    return section_loft(sections)


def _gondola_mesh():
    sections = []
    for x, scale in (
        (-1.65, 0.12),
        (-1.30, 0.62),
        (-0.55, 1.00),
        (0.55, 1.00),
        (1.30, 0.62),
        (1.65, 0.12),
    ):
        loop = []
        for i in range(40):
            t = 2.0 * pi * i / 40
            # Locally centered below the hull mount.  Slightly flattened top
            # keeps the gondola readable as a short passenger/sign gondola.
            z = -0.88 + 0.42 * scale * sin(t)
            if z > -0.58:
                z = -0.58 + (z + 0.58) * 0.55
            loop.append((x, 0.62 * scale * cos(t), z))
        sections.append(loop)
    return section_loft(sections)


def _propeller_mesh(name: str):
    return mesh_from_geometry(
        FanRotorGeometry(
            0.58,
            0.12,
            5,
            thickness=0.075,
            blade_pitch_deg=33.0,
            blade_sweep_deg=28.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.14),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.055, bore_diameter=0.045),
            shroud=FanRotorShroud(thickness=0.018, depth=0.065, clearance=0.004, lip_depth=0.01),
        ),
        name,
    )


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_sign_blimp")

    hull_fabric = _mat(model, "midnight_blue_fabric", (0.035, 0.055, 0.13, 1.0))
    tail_fabric = _mat(model, "navy_tail_fabric", (0.05, 0.07, 0.16, 1.0))
    sign_black = _mat(model, "black_sign_face", (0.004, 0.006, 0.010, 1.0))
    led_amber = _mat(model, "warm_amber_led", (1.0, 0.63, 0.08, 1.0))
    led_cyan = _mat(model, "cyan_led", (0.05, 0.75, 1.0, 1.0))
    gondola_white = _mat(model, "warm_white_composite", (0.86, 0.85, 0.78, 1.0))
    window_blue = _mat(model, "smoked_blue_glass", (0.05, 0.16, 0.28, 0.88))
    metal = _mat(model, "brushed_dark_metal", (0.28, 0.29, 0.30, 1.0))
    light_metal = _mat(model, "satin_aluminum", (0.58, 0.60, 0.62, 1.0))
    prop_black = _mat(model, "matte_black_propeller", (0.01, 0.01, 0.012, 1.0))
    rubber = _mat(model, "dark_rubber", (0.018, 0.018, 0.016, 1.0))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_geometry(_rounded_envelope_mesh(), "rounded_envelope"),
        material=hull_fabric,
        name="envelope_shell",
    )
    # Flat LED sign boxes are deliberately thick enough to seat into the curved
    # fabric envelope, while their outer faces sit proud like event signage.
    for idx, sign in enumerate((1.0, -1.0)):
        envelope.visual(
            Box((5.2, 0.12, 0.92)),
            origin=Origin(xyz=(0.45, sign * 2.50, 0.18)),
            material=sign_black,
            name=f"side_sign_{idx}",
        )
        for col in range(9):
            x = -1.80 + col * 0.48
            mat = led_amber if col % 2 == 0 else led_cyan
            envelope.visual(
                Box((0.22, 0.045, 0.15)),
                origin=Origin(xyz=(x, sign * 2.575, 0.42)),
                material=mat,
                name=f"upper_led_{idx}_{col}",
            )
            envelope.visual(
                Box((0.22, 0.045, 0.15)),
                origin=Origin(xyz=(x + 0.10, sign * 2.575, -0.08)),
                material=led_amber,
                name=f"lower_led_{idx}_{col}",
            )
        # Metal hardpoints protrude from the fabric envelope to give the side
        # nacelle pads a real contact face instead of appearing suspended.
        envelope.visual(
            Box((0.76, 0.11, 0.62)),
            origin=Origin(xyz=(1.65, sign * 2.45, 0.95)),
            material=metal,
            name=f"nacelle_hardpoint_{idx}",
        )

    # Fixed cruciform tail stubs rooted into the aft envelope.  The four moving
    # panels below form the actual rudders and elevators.
    envelope.visual(
        Box((1.45, 0.13, 1.20)),
        origin=Origin(xyz=(-7.65, 0.0, 1.45)),
        material=tail_fabric,
        name="upper_tail_stub",
    )
    envelope.visual(
        Box((1.45, 0.13, 1.20)),
        origin=Origin(xyz=(-7.65, 0.0, -1.45)),
        material=tail_fabric,
        name="lower_tail_stub",
    )
    for idx, sign in enumerate((1.0, -1.0)):
        envelope.visual(
            Box((1.45, 1.05, 0.13)),
            origin=Origin(xyz=(-7.65, sign * 1.48, 0.0)),
            material=tail_fabric,
            name=f"side_tail_stub_{idx}",
        )
    # Exposed hinge barrels at the trailing edge of the fixed cruciform tail.
    envelope.visual(
        Cylinder(radius=0.055, length=1.16),
        origin=Origin(xyz=(-8.38, 0.0, 1.45)),
        material=light_metal,
        name="upper_rudder_hinge",
    )
    envelope.visual(
        Cylinder(radius=0.055, length=1.16),
        origin=Origin(xyz=(-8.38, 0.0, -1.45)),
        material=light_metal,
        name="lower_rudder_hinge",
    )
    for idx, sign in enumerate((1.0, -1.0)):
        envelope.visual(
            Cylinder(radius=0.055, length=1.08),
            origin=Origin(xyz=(-8.38, sign * 1.48, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=light_metal,
            name=f"elevator_hinge_{idx}",
        )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_geometry(_gondola_mesh(), "short_gondola"),
        material=gondola_white,
        name="gondola_shell",
    )
    gondola.visual(
        Box((2.65, 0.84, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.54)),
        material=metal,
        name="roof_saddle",
    )
    for idx, (x, y) in enumerate(((-1.05, -0.34), (-1.05, 0.34), (1.05, -0.34), (1.05, 0.34))):
        gondola.visual(
            Box((0.13, 0.13, 0.52)),
            origin=Origin(xyz=(x, y, -0.26)),
            material=metal,
            name=f"strut_{idx}",
        )
    for idx, x in enumerate((-0.95, -0.35, 0.35, 0.95)):
        gondola.visual(
            Box((0.36, 0.080, 0.20)),
            origin=Origin(xyz=(x, 0.58, -0.80)),
            material=window_blue,
            name=f"window_{idx}",
        )
        gondola.visual(
            Box((0.36, 0.080, 0.20)),
            origin=Origin(xyz=(x, -0.58, -0.80)),
            material=window_blue,
            name=f"window_{idx + 4}",
        )
    for idx, sign in enumerate((1.0, -1.0)):
        gondola.visual(
            Box((0.42, 0.12, 0.30)),
            origin=Origin(xyz=(1.35, sign * 0.52, -0.79)),
            material=metal,
            name=f"pylon_hardpoint_{idx}",
        )
    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -2.16)),
    )

    # Side nacelles with visible bracket arms and bolted pads seated to the hull.
    ring_mesh = mesh_from_geometry(TorusGeometry(0.42, 0.035, radial_segments=24, tubular_segments=48), "side_nacelle_ring")
    prop_meshes = [_propeller_mesh("side_propeller_0"), _propeller_mesh("side_propeller_1")]
    for idx, sign in enumerate((1.0, -1.0)):
        nacelle = model.part(f"nacelle_{idx}")
        nacelle.visual(
            Box((0.72, 0.10, 0.58)),
            origin=Origin(xyz=(0.0, sign * 0.035, 0.0)),
            material=metal,
            name="hull_pad",
        )
        nacelle.visual(
            Cylinder(radius=0.32, length=0.78),
            origin=Origin(xyz=(0.0, sign * 0.72, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=light_metal,
            name="nacelle_shell",
        )
        nacelle.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, sign * 0.72, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="duct_ring",
        )
        for arm, (x, z) in enumerate(((-0.25, 0.22), (0.25, 0.22), (-0.25, -0.22), (0.25, -0.22))):
            nacelle.visual(
                Box((0.10, 0.62, 0.09)),
                origin=Origin(xyz=(x, sign * 0.385, z)),
                material=metal,
                name=f"bracket_arm_{arm}",
            )
        for bolt, (x, z) in enumerate(((-0.25, 0.20), (0.25, 0.20), (-0.25, -0.20), (0.25, -0.20))):
            nacelle.visual(
                Cylinder(radius=0.035, length=0.028),
                origin=Origin(xyz=(x, sign * 0.105, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name=f"bolt_{bolt}",
            )
        model.articulation(
            f"envelope_to_nacelle_{idx}",
            ArticulationType.FIXED,
            parent=envelope,
            child=nacelle,
            origin=Origin(xyz=(1.65, sign * 2.52, 0.95)),
        )

        propeller = model.part(f"propeller_{idx}")
        propeller.visual(prop_meshes[idx], material=prop_black, name="prop_rotor")
        propeller.visual(
            Cylinder(radius=0.055, length=0.28),
            origin=Origin(xyz=(0.0, 0.0, -0.08)),
            material=metal,
            name="prop_shaft",
        )
        model.articulation(
            f"nacelle_to_propeller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=propeller,
            origin=Origin(
                xyz=(0.0, sign * 1.19, 0.0),
                rpy=(-sign * pi / 2.0, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=30.0),
        )

    # Vectoring engine pylons beside the gondola.  Each pylon is a fixed carried
    # member with a visible pivot socket; the pod rotates on that socket.
    engine_ring_mesh = mesh_from_geometry(TorusGeometry(0.265, 0.040, radial_segments=20, tubular_segments=40), "engine_pod_ring")
    for idx, sign in enumerate((1.0, -1.0)):
        pylon = model.part(f"pylon_{idx}")
        pylon.visual(
            Box((0.42, 0.12, 0.30)),
            origin=Origin(xyz=(0.0, sign * 0.04, 0.0)),
            material=metal,
            name="mount_foot",
        )
        pylon.visual(
            Box((0.18, 0.68, 0.14)),
            origin=Origin(xyz=(0.0, sign * 0.44, 0.0)),
            material=metal,
            name="pylon_beam",
        )
        pylon.visual(
            Cylinder(radius=0.105, length=0.18),
            origin=Origin(xyz=(0.0, sign * 0.86, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=light_metal,
            name="pivot_socket",
        )
        model.articulation(
            f"gondola_to_pylon_{idx}",
            ArticulationType.FIXED,
            parent=gondola,
            child=pylon,
            origin=Origin(xyz=(1.35, sign * 0.60, -0.79)),
        )

        pod = model.part(f"engine_pod_{idx}")
        pod.visual(
            Cylinder(radius=0.24, length=0.78),
            origin=Origin(xyz=(0.0, sign * 0.38, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=light_metal,
            name="pod_shell",
        )
        pod.visual(
            engine_ring_mesh,
            origin=Origin(xyz=(0.36, sign * 0.38, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="nozzle_ring",
        )
        pod.visual(
            Cylinder(radius=0.105, length=0.28),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_pin",
        )
        model.articulation(
            f"pylon_to_engine_pod_{idx}",
            ArticulationType.REVOLUTE,
            parent=pylon,
            child=pod,
            origin=Origin(xyz=(0.0, sign * 0.86, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=1.4, lower=-0.65, upper=0.65),
        )

    # Hinged rudder and elevator panels.
    upper_rudder = model.part("upper_rudder")
    upper_rudder.visual(
        Box((0.92, 0.10, 1.05)),
        origin=Origin(xyz=(-0.49, 0.0, 0.0)),
        material=tail_fabric,
        name="rudder_panel",
    )
    model.articulation(
        "envelope_to_upper_rudder",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=upper_rudder,
        origin=Origin(xyz=(-8.40, 0.0, 1.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.3, lower=-0.45, upper=0.45),
    )

    lower_rudder = model.part("lower_rudder")
    lower_rudder.visual(
        Box((0.92, 0.10, 1.05)),
        origin=Origin(xyz=(-0.49, 0.0, 0.0)),
        material=tail_fabric,
        name="rudder_panel",
    )
    model.articulation(
        "envelope_to_lower_rudder",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=lower_rudder,
        origin=Origin(xyz=(-8.40, 0.0, -1.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.3, lower=-0.45, upper=0.45),
    )

    for idx, sign in enumerate((1.0, -1.0)):
        elevator = model.part(f"elevator_{idx}")
        elevator.visual(
            Box((0.92, 0.96, 0.10)),
            origin=Origin(xyz=(-0.49, 0.0, 0.0)),
            material=tail_fabric,
            name="elevator_panel",
        )
        model.articulation(
            f"envelope_to_elevator_{idx}",
            ArticulationType.REVOLUTE,
            parent=envelope,
            child=elevator,
            origin=Origin(xyz=(-8.40, sign * 1.48, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=1.3, lower=-0.40, upper=0.40),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")

    # Seated brackets and captured rotating shafts are intentionally embedded
    # locally; the checks below prove the overlaps are localized, mechanical
    # interfaces rather than hidden broad collisions.
    for strut_idx in range(4):
        ctx.allow_overlap(
            envelope,
            gondola,
            elem_a="envelope_shell",
            elem_b=f"strut_{strut_idx}",
            reason="The gondola suspension strut terminates in a small socket under the fabric envelope.",
        )
        ctx.expect_gap(
            envelope,
            gondola,
            axis="z",
            positive_elem="envelope_shell",
            negative_elem=f"strut_{strut_idx}",
            max_penetration=0.10,
            name=f"gondola strut_{strut_idx} has shallow hull insertion",
        )
        ctx.expect_overlap(
            gondola,
            envelope,
            axes="xy",
            elem_a=f"strut_{strut_idx}",
            elem_b="envelope_shell",
            min_overlap=0.08,
            name=f"gondola strut_{strut_idx} lands under hull",
        )

    for idx, sign in enumerate((1.0, -1.0)):
        nacelle = object_model.get_part(f"nacelle_{idx}")
        propeller = object_model.get_part(f"propeller_{idx}")
        pylon = object_model.get_part(f"pylon_{idx}")
        pod = object_model.get_part(f"engine_pod_{idx}")

        ctx.allow_overlap(
            envelope,
            nacelle,
            elem_a="envelope_shell",
            elem_b="hull_pad",
            reason="The nacelle mounting pad is seated slightly into the curved fabric hull.",
        )
        if sign > 0:
            ctx.expect_gap(
                nacelle,
                envelope,
                axis="y",
                positive_elem="hull_pad",
                negative_elem="envelope_shell",
                max_penetration=0.12,
                name=f"nacelle_{idx} pad has only local hull seating",
            )
        else:
            ctx.expect_gap(
                envelope,
                nacelle,
                axis="y",
                positive_elem="envelope_shell",
                negative_elem="hull_pad",
                max_penetration=0.12,
                name=f"nacelle_{idx} pad has only local hull seating",
            )
        ctx.expect_overlap(
            nacelle,
            envelope,
            axes="xz",
            elem_a="hull_pad",
            elem_b="envelope_shell",
            min_overlap=0.20,
            name=f"nacelle_{idx} pad lands on hull side",
        )

        ctx.allow_overlap(
            nacelle,
            propeller,
            elem_a="nacelle_shell",
            elem_b="prop_shaft",
            reason="The propeller shaft is captured inside the nacelle nose bearing.",
        )
        ctx.expect_overlap(
            propeller,
            nacelle,
            axes="y",
            elem_a="prop_shaft",
            elem_b="nacelle_shell",
            min_overlap=0.03,
            name=f"propeller_{idx} shaft remains inserted in nacelle",
        )

        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="pivot_socket",
            elem_b="pivot_pin",
            reason="The engine pod pivot pin is intentionally captured by the pylon socket.",
        )
        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="pylon_beam",
            elem_b="pivot_pin",
            reason="The pivot pin passes through the reinforced end of the pylon beam.",
        )
        ctx.expect_overlap(
            pod,
            pylon,
            axes="y",
            elem_a="pivot_pin",
            elem_b="pivot_socket",
            min_overlap=0.06,
            name=f"engine_pod_{idx} pivot pin sits in pylon socket",
        )
        ctx.expect_overlap(
            pod,
            pylon,
            axes="xz",
            elem_a="pivot_pin",
            elem_b="pivot_socket",
            min_overlap=0.08,
            name=f"engine_pod_{idx} pivot is coaxial with pylon socket",
        )
        ctx.expect_overlap(
            pod,
            pylon,
            axes="y",
            elem_a="pivot_pin",
            elem_b="pylon_beam",
            min_overlap=0.02,
            name=f"engine_pod_{idx} pivot pin passes through pylon end",
        )

        ctx.allow_overlap(
            gondola,
            pylon,
            elem_a="gondola_shell",
            elem_b="mount_foot",
            reason="The vectoring-engine pylon mounting foot is seated into the gondola side shell.",
        )
        if sign > 0:
            ctx.expect_gap(
                pylon,
                gondola,
                axis="y",
                positive_elem="mount_foot",
                negative_elem="gondola_shell",
                max_penetration=0.08,
                name=f"pylon_{idx} foot has shallow gondola seating",
            )
        else:
            ctx.expect_gap(
                gondola,
                pylon,
                axis="y",
                positive_elem="gondola_shell",
                negative_elem="mount_foot",
                max_penetration=0.08,
                name=f"pylon_{idx} foot has shallow gondola seating",
            )
        ctx.expect_overlap(
            pylon,
            gondola,
            axes="xz",
            elem_a="mount_foot",
            elem_b="gondola_shell",
            min_overlap=0.18,
            name=f"pylon_{idx} foot is carried by gondola",
        )

    for idx in (0, 1):
        joint = object_model.get_articulation(f"nacelle_to_propeller_{idx}")
        ctx.check(
            f"propeller_{idx} uses continuous rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    for idx in (0, 1):
        joint = object_model.get_articulation(f"pylon_to_engine_pod_{idx}")
        pod = object_model.get_part(f"engine_pod_{idx}")
        before = ctx.part_element_world_aabb(pod, elem="pod_shell")
        with ctx.pose({joint: 0.55}):
            after = ctx.part_element_world_aabb(pod, elem="pod_shell")
        ctx.check(
            f"engine_pod_{idx} visibly vectors on pylon",
            before is not None
            and after is not None
            and abs((after[1][2] - after[0][2]) - (before[1][2] - before[0][2])) > 0.03,
            details=f"before={before}, after={after}",
        )

    upper_rudder_joint = object_model.get_articulation("envelope_to_upper_rudder")
    upper_rudder = object_model.get_part("upper_rudder")
    ctx.allow_overlap(
        envelope,
        upper_rudder,
        elem_a="upper_rudder_hinge",
        elem_b="rudder_panel",
        reason="The rudder panel is captured on the exposed hinge barrel.",
    )
    ctx.expect_gap(
        envelope,
        upper_rudder,
        axis="x",
        positive_elem="upper_rudder_hinge",
        negative_elem="rudder_panel",
        max_penetration=0.015,
        name="upper rudder is only locally captured by hinge",
    )
    rudder_before = ctx.part_element_world_aabb(upper_rudder, elem="rudder_panel")
    with ctx.pose({upper_rudder_joint: 0.35}):
        rudder_after = ctx.part_element_world_aabb(upper_rudder, elem="rudder_panel")
    ctx.check(
        "rudder pivots about a vertical hinge",
        rudder_before is not None
        and rudder_after is not None
        and abs((rudder_after[1][1] - rudder_after[0][1]) - (rudder_before[1][1] - rudder_before[0][1])) > 0.20,
        details=f"before={rudder_before}, after={rudder_after}",
    )

    lower_rudder = object_model.get_part("lower_rudder")
    ctx.allow_overlap(
        envelope,
        lower_rudder,
        elem_a="lower_rudder_hinge",
        elem_b="rudder_panel",
        reason="The lower rudder panel is captured on the exposed hinge barrel.",
    )
    ctx.expect_gap(
        envelope,
        lower_rudder,
        axis="x",
        positive_elem="lower_rudder_hinge",
        negative_elem="rudder_panel",
        max_penetration=0.015,
        name="lower rudder is only locally captured by hinge",
    )

    elevator_joint = object_model.get_articulation("envelope_to_elevator_0")
    elevator = object_model.get_part("elevator_0")
    for idx in (0, 1):
        elevator_part = object_model.get_part(f"elevator_{idx}")
        ctx.allow_overlap(
            envelope,
            elevator_part,
            elem_a=f"elevator_hinge_{idx}",
            elem_b="elevator_panel",
            reason="The elevator panel is captured on the exposed horizontal hinge barrel.",
        )
        ctx.expect_gap(
            envelope,
            elevator_part,
            axis="x",
            positive_elem=f"elevator_hinge_{idx}",
            negative_elem="elevator_panel",
            max_penetration=0.015,
            name=f"elevator_{idx} is only locally captured by hinge",
        )
    elevator_before = ctx.part_element_world_aabb(elevator, elem="elevator_panel")
    with ctx.pose({elevator_joint: 0.32}):
        elevator_after = ctx.part_element_world_aabb(elevator, elem="elevator_panel")
    ctx.check(
        "elevator pivots about a horizontal hinge",
        elevator_before is not None
        and elevator_after is not None
        and abs((elevator_after[1][2] - elevator_after[0][2]) - (elevator_before[1][2] - elevator_before[0][2])) > 0.20,
        details=f"before={elevator_before}, after={elevator_after}",
    )

    ctx.expect_overlap(
        gondola,
        envelope,
        axes="xy",
        elem_a="roof_saddle",
        elem_b="envelope_shell",
        min_overlap=0.50,
        name="short gondola is tucked beneath the envelope",
    )

    return ctx.report()


object_model = build_object_model()
