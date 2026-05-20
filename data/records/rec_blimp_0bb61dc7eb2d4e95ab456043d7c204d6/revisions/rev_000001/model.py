from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    CapsuleGeometry,
    mesh_from_geometry,
    section_loft,
)


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a cylinder whose local +Z axis runs from p0 to p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _ellipse_section(x, y_radius, z_radius, samples=48):
    return tuple(
        (
            x,
            y_radius * math.cos(2.0 * math.pi * i / samples),
            z_radius * math.sin(2.0 * math.pi * i / samples),
        )
        for i in range(samples)
    )


def _build_envelope_mesh():
    length = 62.0
    radius = 7.15
    profile = []
    for i in range(49):
        t = i / 48.0
        x = -0.5 * length + length * t
        # Inflated blimp envelope: very rounded middle with sharply tapered ends.
        r = radius * (math.sin(math.pi * t) ** 0.56)
        profile.append((max(r, 0.02), x))
    geom = LatheGeometry(profile, segments=96, closed=True)
    # Lathe axis starts on local Z; rotate the long axis to local X and make the
    # vertical radius slightly shallower than the side radius.
    geom.rotate_y(math.pi / 2.0)
    geom.scale(1.0, 1.0, 0.92)
    return geom


def _build_gondola_mesh():
    sections = [
        _ellipse_section(-4.30, 0.10, 0.08),
        _ellipse_section(-3.65, 1.05, 0.72),
        _ellipse_section(-2.10, 1.43, 1.05),
        _ellipse_section(0.60, 1.50, 1.14),
        _ellipse_section(2.75, 1.24, 0.96),
        _ellipse_section(4.18, 0.18, 0.16),
    ]
    return section_loft(sections)


def _add_propeller(part, mesh_name, material):
    rotor = FanRotorGeometry(
        1.18,
        0.22,
        4,
        thickness=0.14,
        blade_pitch_deg=34.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=16.0, camber=0.10),
        hub=FanRotorHub(style="spinner", bore_diameter=0.10),
        center=True,
    )
    rotor_mesh = mesh_from_geometry(rotor, mesh_name)
    part.visual(
        rotor_mesh,
        # FanRotorGeometry spins around local Z; this places that axis along the
        # blimp's local fore-aft shaft axis.
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="rotor",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="advertising_blimp")

    envelope_mat = model.material("pearl_white_fabric", rgba=(0.88, 0.91, 0.88, 1.0))
    blue_mat = model.material("deep_blue_trim", rgba=(0.03, 0.13, 0.42, 1.0))
    red_mat = model.material("advertising_red", rgba=(0.90, 0.08, 0.05, 1.0))
    cabin_mat = model.material("brushed_silver", rgba=(0.64, 0.67, 0.68, 1.0))
    dark_mat = model.material("dark_glass", rgba=(0.02, 0.035, 0.055, 1.0))
    strut_mat = model.material("strut_aluminum", rgba=(0.40, 0.42, 0.43, 1.0))
    prop_mat = model.material("satin_black_propellers", rgba=(0.015, 0.015, 0.014, 1.0))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_geometry(_build_envelope_mesh(), "cigar_envelope"),
        material=envelope_mat,
        name="envelope_body",
    )
    # Painted advertising band and side panels, slightly embedded in the fabric
    # so they read as paint/decals rather than floating placards.
    envelope.visual(
        Box((30.0, 0.08, 2.4)),
        origin=Origin(xyz=(2.5, 7.12, 0.10)),
        material=red_mat,
        name="port_ad_panel",
    )
    envelope.visual(
        Box((30.0, 0.08, 2.4)),
        origin=Origin(xyz=(2.5, -7.12, 0.10)),
        material=red_mat,
        name="starboard_ad_panel",
    )
    envelope.visual(
        Box((20.0, 14.15, 0.10)),
        origin=Origin(xyz=(7.0, 0.0, -0.08)),
        material=blue_mat,
        name="equator_stripe",
    )

    # Fixed cruciform tail surfaces are rooted into the aft cone of the envelope.
    envelope.visual(
        Box((3.25, 0.26, 3.75)),
        origin=Origin(xyz=(-27.05, 0.0, 4.48)),
        material=blue_mat,
        name="dorsal_fin",
    )
    envelope.visual(
        Box((3.25, 0.26, 3.55)),
        origin=Origin(xyz=(-27.05, 0.0, -4.36)),
        material=blue_mat,
        name="ventral_fin",
    )
    envelope.visual(
        Box((3.25, 5.15, 0.24)),
        origin=Origin(xyz=(-27.05, 5.18, 0.0)),
        material=blue_mat,
        name="port_stabilizer",
    )
    envelope.visual(
        Box((3.25, 5.15, 0.24)),
        origin=Origin(xyz=(-27.05, -5.18, 0.0)),
        material=blue_mat,
        name="starboard_stabilizer",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_geometry(_build_gondola_mesh(), "underslung_gondola"),
        material=cabin_mat,
        name="cabin_shell",
    )
    gondola.visual(
        Box((3.55, 0.075, 0.55)),
        origin=Origin(xyz=(0.35, 1.49, 0.30)),
        material=dark_mat,
        name="port_windows",
    )
    gondola.visual(
        Box((3.55, 0.075, 0.55)),
        origin=Origin(xyz=(0.35, -1.49, 0.30)),
        material=dark_mat,
        name="starboard_windows",
    )
    gondola.visual(
        Box((0.08, 1.18, 0.50)),
        origin=Origin(xyz=(4.06, 0.0, 0.27)),
        material=dark_mat,
        name="windshield",
    )
    for y, name in ((0.95, "port_skid"), (-0.95, "starboard_skid")):
        gondola.visual(
            Cylinder(radius=0.075, length=7.25),
            origin=Origin(xyz=(0.0, y, -1.33), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=strut_mat,
            name=name,
        )
        for x in (-2.6, 2.45):
            _cylinder_between(
                gondola,
                f"{name}_brace_{'front' if x > 0 else 'rear'}",
                (x, y, -1.28),
                (x * 0.90, y * 0.82, -0.70),
                0.045,
                strut_mat,
            )

    # Four suspension struts and saddle pads visibly separate the gondola from
    # the envelope while carrying it from the lower fabric hull.
    saddle_points = [
        (-2.90, 1.45, 3.58),
        (-2.90, -1.45, 3.58),
        (2.55, 1.35, 3.58),
        (2.55, -1.35, 3.58),
    ]
    roof_points = [(-2.50, 1.05, 0.92), (-2.50, -1.05, 0.92), (2.05, 0.98, 0.88), (2.05, -0.98, 0.88)]
    for i, (roof, saddle) in enumerate(zip(roof_points, saddle_points)):
        gondola.visual(
            Box((0.82, 0.48, 0.28)),
            origin=Origin(xyz=(roof[0], roof[1], roof[2] - 0.05)),
            material=strut_mat,
            name=f"roof_hardpoint_{i}",
        )
        _cylinder_between(gondola, f"suspension_strut_{i}", roof, saddle, 0.070, strut_mat)
        gondola.visual(
            Box((1.00, 0.52, 0.24)),
            origin=Origin(xyz=(saddle[0], saddle[1], saddle[2] + 0.04)),
            material=strut_mat,
            name=f"top_saddle_{i}",
        )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(-4.0, 0.0, -10.20)),
    )

    # Vectoring engine pylons.  Each pylon's child frame is the pod pivot axis.
    for side, sign in (("port", 1.0), ("starboard", -1.0)):
        pylon = model.part(f"{side}_pylon")
        # Pylon arm reaches back from the pivot to the gondola side.
        _cylinder_between(
            pylon,
            "pylon_arm",
            (0.0, -sign * 0.82, 0.0),
            (0.0, -sign * 1.10, 0.0),
            0.115,
            strut_mat,
        )
        pylon.visual(
            Cylinder(radius=0.16, length=1.70),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=strut_mat,
            name="pivot_pin",
        )
        pylon.visual(
            Box((0.78, 0.22, 0.48)),
            origin=Origin(xyz=(0.0, -sign * 1.18, 0.0)),
            material=strut_mat,
            name="cabin_mount",
        )
        model.articulation(
            f"gondola_to_{side}_pylon",
            ArticulationType.FIXED,
            parent=gondola,
            child=pylon,
            origin=Origin(xyz=(0.35, sign * 2.78, -0.35)),
        )

        pod = model.part(f"{side}_pod")
        pod_body = CapsuleGeometry(0.72, 2.45, radial_segments=36, height_segments=10)
        pod_body.rotate_y(math.pi / 2.0)
        pod.visual(
            mesh_from_geometry(pod_body, f"{side}_nacelle_body"),
            material=envelope_mat,
            name="nacelle_body",
        )
        cowling = TorusGeometry(0.63, 0.055, radial_segments=16, tubular_segments=48)
        cowling.rotate_y(math.pi / 2.0)
        cowling.translate(1.50, 0.0, 0.0)
        pod.visual(
            mesh_from_geometry(cowling, f"{side}_front_cowling"),
            material=blue_mat,
            name="front_cowling",
        )
        pod.visual(
            Cylinder(radius=0.22, length=1.60),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=strut_mat,
            name="pivot_collar",
        )
        pod.visual(
            Cylinder(radius=0.082, length=0.28),
            origin=Origin(xyz=(1.94, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=strut_mat,
            name="prop_shaft",
        )
        model.articulation(
            f"{side}_pylon_to_pod",
            ArticulationType.REVOLUTE,
            parent=pylon,
            child=pod,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=250.0, velocity=0.8, lower=-0.45, upper=0.45),
        )

        prop = model.part(f"{side}_propeller")
        _add_propeller(prop, f"{side}_propeller_rotor", prop_mat)
        model.articulation(
            f"{side}_pod_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=prop,
            origin=Origin(xyz=(2.06, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=90.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        Box((2.35, 0.20, 3.15)),
        origin=Origin(xyz=(-1.18, 0.0, 1.58)),
        material=blue_mat,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.085, length=3.25),
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        material=strut_mat,
        name="hinge_barrel",
    )
    model.articulation(
        "envelope_to_rudder",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=rudder,
        origin=Origin(xyz=(-28.68, 0.0, 4.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-0.55, upper=0.55),
    )

    for side, sign in (("port", 1.0), ("starboard", -1.0)):
        elevator = model.part(f"{side}_elevator")
        elevator.visual(
            Box((2.25, 4.55, 0.18)),
            origin=Origin(xyz=(-1.15, 0.0, 0.0)),
            material=blue_mat,
            name="elevator_panel",
        )
        elevator.visual(
            Cylinder(radius=0.08, length=4.60),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=strut_mat,
            name="hinge_barrel",
        )
        model.articulation(
            f"envelope_to_{side}_elevator",
            ArticulationType.REVOLUTE,
            parent=envelope,
            child=elevator,
            origin=Origin(xyz=(-28.68, sign * 5.18, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=1.3, lower=-0.48, upper=0.48),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")

    for i in range(4):
        pad = f"top_saddle_{i}"
        ctx.allow_overlap(
            envelope,
            gondola,
            elem_a="envelope_body",
            elem_b=pad,
            reason="The suspension saddle pad is intentionally embedded a few centimeters into the fabric hull as a mounted hardpoint.",
        )
        ctx.expect_overlap(
            envelope,
            gondola,
            axes="xy",
            elem_a="envelope_body",
            elem_b=pad,
            min_overlap=0.15,
            name=f"{pad} lies under the envelope footprint",
        )
        ctx.expect_gap(
            envelope,
            gondola,
            axis="z",
            positive_elem="envelope_body",
            negative_elem=pad,
            max_penetration=0.60,
            name=f"{pad} has only local saddle embed",
        )

    # Pylons and hinges intentionally use local captured pins/barrels.  The
    # allowances are scoped to those named mechanical interfaces.
    for side in ("port", "starboard"):
        pylon = object_model.get_part(f"{side}_pylon")
        pod = object_model.get_part(f"{side}_pod")
        prop = object_model.get_part(f"{side}_propeller")

        ctx.allow_overlap(
            gondola,
            pylon,
            elem_a="cabin_shell",
            elem_b="cabin_mount",
            reason="The pylon mounting foot is seated into the gondola side skin.",
        )
        ctx.expect_overlap(
            gondola,
            pylon,
            axes="xz",
            elem_a="cabin_shell",
            elem_b="cabin_mount",
            min_overlap=0.20,
            name=f"{side} pylon foot is carried by the gondola side",
        )

        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="pivot_pin",
            elem_b="pivot_collar",
            reason="The vectoring engine pivot pin is intentionally captured in the pod collar.",
        )
        ctx.expect_overlap(
            pylon,
            pod,
            axes="y",
            elem_a="pivot_pin",
            elem_b="pivot_collar",
            min_overlap=0.45,
            name=f"{side} pod collar is retained on the pylon pin",
        )
        ctx.expect_within(
            pod,
            pylon,
            axes="xz",
            inner_elem="pivot_collar",
            outer_elem="pivot_pin",
            margin=0.09,
            name=f"{side} pod pivot is centered on the pylon pin",
        )
        ctx.allow_overlap(
            pylon,
            pod,
            elem_a="pivot_pin",
            elem_b="nacelle_body",
            reason="The same short pivot pin passes through the nacelle wall at the vectoring trunnion.",
        )
        ctx.expect_overlap(
            pylon,
            pod,
            axes="y",
            elem_a="pivot_pin",
            elem_b="nacelle_body",
            min_overlap=0.70,
            name=f"{side} pivot pin crosses the nacelle trunnion",
        )

        ctx.allow_overlap(
            pod,
            prop,
            elem_a="prop_shaft",
            elem_b="rotor",
            reason="The propeller hub is seated over the short visible shaft stub.",
        )
        ctx.expect_overlap(
            pod,
            prop,
            axes="x",
            elem_a="prop_shaft",
            elem_b="rotor",
            min_overlap=0.03,
            name=f"{side} propeller hub overlaps the shaft stub",
        )

        ctx.expect_contact(
            prop,
            pod,
            elem_a="rotor",
            elem_b="prop_shaft",
            contact_tol=0.04,
            name=f"{side} propeller hub sits on its local shaft",
        )

    rudder = object_model.get_part("rudder")
    ctx.allow_overlap(
        envelope,
        rudder,
        elem_a="dorsal_fin",
        elem_b="hinge_barrel",
        reason="The rudder hinge barrel is captured in the fixed dorsal tail hinge.",
    )
    ctx.expect_overlap(
        envelope,
        rudder,
        axes="z",
        elem_a="dorsal_fin",
        elem_b="hinge_barrel",
        min_overlap=2.0,
        name="rudder hinge spans the dorsal fin",
    )

    for side in ("port", "starboard"):
        elevator = object_model.get_part(f"{side}_elevator")
        ctx.allow_overlap(
            envelope,
            elevator,
            elem_a=f"{side}_stabilizer",
            elem_b="hinge_barrel",
            reason="The elevator hinge barrel is captured along the fixed horizontal stabilizer trailing edge.",
        )
        ctx.expect_overlap(
            envelope,
            elevator,
            axes="y",
            elem_a=f"{side}_stabilizer",
            elem_b="hinge_barrel",
            min_overlap=3.8,
            name=f"{side} elevator hinge spans the stabilizer",
        )

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="envelope_body",
        negative_elem="cabin_shell",
        min_gap=1.30,
        name="gondola cabin hangs below the envelope with visible daylight",
    )
    ctx.expect_gap(
        object_model.get_part("port_pod"),
        gondola,
        axis="y",
        positive_elem="nacelle_body",
        negative_elem="cabin_shell",
        min_gap=0.35,
        name="port nacelle is beside, not merged into, the gondola",
    )
    ctx.expect_gap(
        gondola,
        object_model.get_part("starboard_pod"),
        axis="y",
        positive_elem="cabin_shell",
        negative_elem="nacelle_body",
        min_gap=0.35,
        name="starboard nacelle is beside, not merged into, the gondola",
    )

    for side in ("port", "starboard"):
        prop_joint = object_model.get_articulation(f"{side}_pod_to_propeller")
        ctx.check(
            f"{side} propeller joint is continuous",
            prop_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(prop_joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={prop_joint.articulation_type}, axis={prop_joint.axis}",
        )

    port_prop = object_model.get_part("port_propeller")
    port_vector = object_model.get_articulation("port_pylon_to_pod")
    rest_prop = ctx.part_world_position(port_prop)
    with ctx.pose({port_vector: 0.35}):
        vectored_prop = ctx.part_world_position(port_prop)
    ctx.check(
        "port pod vectors its thrust line",
        rest_prop is not None
        and vectored_prop is not None
        and vectored_prop[2] < rest_prop[2] - 0.35,
        details=f"rest={rest_prop}, vectored={vectored_prop}",
    )

    rudder_joint = object_model.get_articulation("envelope_to_rudder")
    rest_rudder = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_joint: 0.42}):
        turned_rudder = ctx.part_world_aabb(rudder)
    if rest_rudder and turned_rudder:
        rest_center_y = (rest_rudder[0][1] + rest_rudder[1][1]) * 0.5
        turned_center_y = (turned_rudder[0][1] + turned_rudder[1][1]) * 0.5
        rudder_moves = abs(turned_center_y - rest_center_y) > 0.35
    else:
        rudder_moves = False
    ctx.check(
        "rudder swings about a vertical tail hinge",
        rudder_moves,
        details=f"rest={rest_rudder}, turned={turned_rudder}",
    )

    elevator_joint = object_model.get_articulation("envelope_to_port_elevator")
    port_elevator = object_model.get_part("port_elevator")
    rest_elevator = ctx.part_world_aabb(port_elevator)
    with ctx.pose({elevator_joint: 0.36}):
        raised_elevator = ctx.part_world_aabb(port_elevator)
    if rest_elevator and raised_elevator:
        rest_center_z = (rest_elevator[0][2] + rest_elevator[1][2]) * 0.5
        raised_center_z = (raised_elevator[0][2] + raised_elevator[1][2]) * 0.5
        elevator_moves = abs(raised_center_z - rest_center_z) > 0.25
    else:
        elevator_moves = False
    ctx.check(
        "tail elevator pitches about a horizontal hinge",
        elevator_moves,
        details=f"rest={rest_elevator}, raised={raised_elevator}",
    )

    return ctx.report()


object_model = build_object_model()
