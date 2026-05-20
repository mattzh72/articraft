from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Orient a default local-Z cylinder between two local-space points."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_cylinder_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _envelope_mesh():
    """A sixty-meter prolate airship envelope with fuller shoulders and tapered caps."""
    length = 60.0
    radius = 6.8
    profile: list[tuple[float, float]] = []
    samples = 64
    for i in range(samples + 1):
        x = -length * 0.5 + length * i / samples
        u = x / (length * 0.5)
        # A softened super-ellipse of revolution gives a cigar-like blimp form
        # instead of a pure sphere/capsule silhouette.
        r = radius * max(0.0, 1.0 - abs(u) ** 2.15) ** 0.43
        if i in (0, samples):
            r = 0.0
        profile.append((r, x))
    geom = LatheGeometry(profile, segments=72)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(0.0, 0.0, 8.0)
    return geom


def _nacelle_mesh(sign: float):
    body = CapsuleGeometry(0.65, 1.35, radial_segments=32, height_segments=10)
    body.rotate_y(math.pi / 2.0)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="advertising_blimp")

    envelope_fabric = model.material("warm_white_fabric", rgba=(0.92, 0.88, 0.72, 1.0))
    blue = model.material("advertising_blue", rgba=(0.08, 0.22, 0.62, 1.0))
    yellow = model.material("accent_yellow", rgba=(0.95, 0.68, 0.16, 1.0))
    gondola_skin = model.material("gondola_white", rgba=(0.86, 0.88, 0.88, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.25, 0.55, 0.76, 0.65))
    metal = model.material("dull_aluminum", rgba=(0.55, 0.57, 0.58, 1.0))
    dark = model.material("rubber_black", rgba=(0.03, 0.035, 0.04, 1.0))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_geometry(_envelope_mesh(), "cigar_envelope"),
        material=envelope_fabric,
        name="envelope_shell",
    )
    # Advertising side panels are very shallow blocks embedded in the fabric
    # surface; the bright bands make the object read as a commercial blimp.
    for y, suffix in ((6.74, "port"), (-6.74, "starboard")):
        envelope.visual(
            Box((17.0, 0.08, 2.05)),
            origin=Origin(xyz=(4.0, y, 8.15)),
            material=blue,
            name=f"{suffix}_ad_panel",
        )
        envelope.visual(
            Box((4.0, 0.095, 1.28)),
            origin=Origin(xyz=(0.0, y + (0.02 if y > 0 else -0.02), 8.15)),
            material=yellow,
            name=f"{suffix}_logo_block",
        )

    # Flat mounting pads/keel rail give real surfaces for the gondola and
    # side-nacelle struts to land on without merging the cabins into the bag.
    envelope.visual(
        Box((7.2, 0.42, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 1.22)),
        material=metal,
        name="keel_rail",
    )
    envelope.visual(
        Box((1.30, 0.80, 0.90)),
        origin=Origin(xyz=(4.0, 6.55, 5.25)),
        material=metal,
        name="port_engine_pad",
    )
    envelope.visual(
        Box((1.30, 0.80, 0.90)),
        origin=Origin(xyz=(4.0, -6.55, 5.25)),
        material=metal,
        name="starboard_engine_pad",
    )

    # Cruciform fixed tail planes. The moving rudder/elevators are separate
    # parts behind the hinge line.
    envelope.visual(
        Box((7.0, 0.18, 4.2)),
        origin=Origin(xyz=(-26.5, 0.0, 13.6)),
        material=blue,
        name="dorsal_fin",
    )
    envelope.visual(
        Box((7.0, 0.18, 3.7)),
        origin=Origin(xyz=(-26.5, 0.0, 2.95)),
        material=blue,
        name="ventral_fin",
    )
    for y, suffix in ((4.15, "port"), (-4.15, "starboard")):
        envelope.visual(
            Box((7.0, 4.4, 0.18)),
            origin=Origin(xyz=(-26.5, y, 8.0)),
            material=blue,
            name=f"{suffix}_tailplane",
        )

    gondola = model.part("gondola")
    gondola.visual(
        Box((7.8, 2.15, 1.65)),
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
        material=gondola_skin,
        name="cabin_shell",
    )
    gondola.visual(
        Box((1.5, 1.92, 0.68)),
        origin=Origin(xyz=(3.45, 0.0, -0.05)),
        material=glass,
        name="front_windshield",
    )
    for y, suffix in ((1.095, "port"), (-1.095, "starboard")):
        for i, x in enumerate((-2.1, -0.7, 0.7, 2.1)):
            gondola.visual(
                Box((0.72, 0.05, 0.55)),
                origin=Origin(xyz=(x, y, -0.20)),
                material=glass,
                name=f"{suffix}_window_{i}",
            )
    for x in (-2.25, 2.25):
        gondola.visual(
            Box((0.30, 0.36, 0.70)),
            origin=Origin(xyz=(x, 0.0, 0.75)),
            material=metal,
            name=f"cabin_strut_{'rear' if x < 0 else 'front'}",
        )
    # Rear mooring-wheel fork and axle.
    for y, suffix in ((0.34, "port"), (-0.34, "starboard")):
        gondola.visual(
            Box((0.22, 0.10, 1.18)),
            origin=Origin(xyz=(-3.15, y, -1.78)),
            material=metal,
            name=f"{suffix}_fork_prong",
        )
    gondola.visual(
        Cylinder(radius=0.063, length=0.76),
        origin=Origin(xyz=(-3.15, 0.0, -2.02), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_axle",
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(),
    )

    nacelle_body_mesh = mesh_from_geometry(_nacelle_mesh(1.0), "nacelle_shell")
    for sign, side in ((1.0, "port"), (-1.0, "starboard")):
        nacelle = model.part(f"{side}_nacelle")
        nacelle.visual(nacelle_body_mesh, material=metal, name="nacelle_shell")
        pylon_start = (0.0, 0.0, 0.50)
        pylon_end = (0.0, -sign * 1.32, 1.98)
        nacelle.visual(
            Cylinder(radius=0.13, length=_distance(pylon_start, pylon_end)),
            origin=Origin(
                xyz=_midpoint(pylon_start, pylon_end),
                rpy=_rpy_for_cylinder(pylon_start, pylon_end),
            ),
            material=metal,
            name="support_pylon",
        )
        nacelle.visual(
            Cylinder(radius=0.16, length=0.32),
            origin=Origin(xyz=(1.43, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="prop_shaft",
        )
        model.articulation(
            f"envelope_to_{side}_nacelle",
            ArticulationType.FIXED,
            parent=envelope,
            child=nacelle,
            origin=Origin(xyz=(4.0, sign * 8.27, 3.27)),
        )

        propeller = model.part(f"{side}_propeller")
        propeller.visual(
            mesh_from_geometry(
                FanRotorGeometry(
                    1.18,
                    0.22,
                    4,
                    thickness=0.16,
                    blade_pitch_deg=32.0,
                    blade_sweep_deg=18.0,
                    blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.14),
                    hub=FanRotorHub(style="spinner", bore_diameter=0.11),
                ),
                f"{side}_propeller_rotor",
            ),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="propeller_rotor",
        )
        model.articulation(
            f"{side}_propeller_spin",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=propeller,
            origin=Origin(xyz=(1.56, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=45.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        Box((2.55, 0.16, 6.2)),
        origin=Origin(xyz=(-1.275, 0.0, 0.0)),
        material=blue,
        name="rudder_panel",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=rudder,
        origin=Origin(xyz=(-30.0, 0.0, 8.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.48, upper=0.48),
    )

    for y, side in ((3.70, "port"), (-3.70, "starboard")):
        elevator = model.part(f"{side}_elevator")
        elevator.visual(
            Box((2.45, 3.85, 0.16)),
            origin=Origin(xyz=(-1.225, 0.0, 0.0)),
            material=blue,
            name="elevator_panel",
        )
        model.articulation(
            f"{side}_elevator_hinge",
            ArticulationType.REVOLUTE,
            parent=envelope,
            child=elevator,
            origin=Origin(xyz=(-30.0, y, 8.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-0.42, upper=0.42),
        )

    wheel = model.part("mooring_wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.46,
                0.22,
                inner_radius=0.31,
                carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.06),
                tread=TireTread(style="block", depth=0.035, count=18, land_ratio=0.56),
                grooves=(TireGroove(center_offset=0.0, width=0.035, depth=0.010),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
                shoulder=TireShoulder(width=0.018, radius=0.012),
            ),
            "mooring_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.30,
                0.17,
                rim=WheelRim(inner_radius=0.18, flange_height=0.018, flange_thickness=0.008),
                hub=WheelHub(
                    radius=0.07,
                    width=0.14,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.095, hole_diameter=0.014),
                ),
                face=WheelFace(dish_depth=0.012, front_inset=0.005, rear_inset=0.005),
                spokes=WheelSpokes(style="split_y", count=5, thickness=0.012, window_radius=0.035),
                bore=WheelBore(style="round", diameter=0.125),
            ),
            "mooring_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal,
        name="rim",
    )
    model.articulation(
        "mooring_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=wheel,
        origin=Origin(xyz=(-3.15, 0.0, -2.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    port_nacelle = object_model.get_part("port_nacelle")
    starboard_nacelle = object_model.get_part("starboard_nacelle")
    port_propeller = object_model.get_part("port_propeller")
    starboard_propeller = object_model.get_part("starboard_propeller")
    wheel = object_model.get_part("mooring_wheel")

    ctx.allow_overlap(
        envelope,
        port_nacelle,
        elem_a="port_engine_pad",
        elem_b="support_pylon",
        reason="The side-engine pylon is intentionally seated into the envelope hardpoint pad.",
    )
    ctx.allow_overlap(
        envelope,
        starboard_nacelle,
        elem_a="starboard_engine_pad",
        elem_b="support_pylon",
        reason="The side-engine pylon is intentionally seated into the envelope hardpoint pad.",
    )
    ctx.allow_overlap(
        gondola,
        wheel,
        elem_a="wheel_axle",
        elem_b="rim",
        reason="The small mooring-wheel axle is intentionally captured through the wheel hub bore.",
    )
    ctx.allow_overlap(
        port_nacelle,
        port_propeller,
        elem_a="prop_shaft",
        elem_b="propeller_rotor",
        reason="The propeller hub is intentionally captured on the nacelle shaft.",
    )
    ctx.allow_overlap(
        starboard_nacelle,
        starboard_propeller,
        elem_a="prop_shaft",
        elem_b="propeller_rotor",
        reason="The propeller hub is intentionally captured on the nacelle shaft.",
    )

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="keel_rail",
        negative_elem="cabin_shell",
        min_gap=0.55,
        max_gap=0.80,
        name="gondola cabin hangs below envelope keel",
    )
    ctx.expect_origin_gap(
        port_nacelle,
        envelope,
        axis="y",
        min_gap=7.5,
        name="port nacelle is mounted beside envelope",
    )
    ctx.expect_origin_gap(
        envelope,
        starboard_nacelle,
        axis="y",
        min_gap=7.5,
        name="starboard nacelle is mounted beside envelope",
    )
    ctx.expect_gap(
        gondola,
        wheel,
        axis="z",
        positive_elem="cabin_shell",
        negative_elem="tire",
        min_gap=0.20,
        max_gap=0.55,
        name="mooring wheel sits under cabin tail",
    )
    ctx.expect_gap(
        port_nacelle,
        envelope,
        axis="y",
        positive_elem="support_pylon",
        negative_elem="port_engine_pad",
        max_penetration=0.11,
        name="port pylon seats locally in engine pad",
    )
    ctx.expect_gap(
        envelope,
        starboard_nacelle,
        axis="y",
        positive_elem="starboard_engine_pad",
        negative_elem="support_pylon",
        max_penetration=0.11,
        name="starboard pylon seats locally in engine pad",
    )
    ctx.expect_overlap(
        gondola,
        wheel,
        axes="y",
        elem_a="wheel_axle",
        elem_b="rim",
        min_overlap=0.12,
        name="mooring wheel axle passes through hub",
    )
    ctx.expect_overlap(
        port_nacelle,
        port_propeller,
        axes="x",
        elem_a="prop_shaft",
        elem_b="propeller_rotor",
        min_overlap=0.05,
        name="port propeller hub is retained on shaft",
    )
    ctx.expect_overlap(
        starboard_nacelle,
        starboard_propeller,
        axes="x",
        elem_a="prop_shaft",
        elem_b="propeller_rotor",
        min_overlap=0.05,
        name="starboard propeller hub is retained on shaft",
    )

    continuous_joints = (
        "port_propeller_spin",
        "starboard_propeller_spin",
        "mooring_wheel_spin",
    )
    for joint_name in continuous_joints:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={joint.articulation_type}",
        )

    rudder_hinge = object_model.get_articulation("rudder_hinge")
    port_elevator_hinge = object_model.get_articulation("port_elevator_hinge")
    ctx.check(
        "rudder uses vertical hinge axis",
        rudder_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={rudder_hinge.axis}",
    )
    ctx.check(
        "elevators use horizontal hinge axis",
        port_elevator_hinge.axis == (0.0, 1.0, 0.0),
        details=f"axis={port_elevator_hinge.axis}",
    )

    rudder = object_model.get_part("rudder")
    with ctx.pose({rudder_hinge: 0.35}):
        ctx.expect_gap(
            envelope,
            rudder,
            axis="x",
            max_penetration=0.04,
            name="deflected rudder stays aft of envelope",
        )

    return ctx.report()


object_model = build_object_model()
