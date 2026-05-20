from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
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
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _cylinder_between(part, start, end, radius, *, material, name):
    """Add a cylinder visual whose local +Z runs between two model-space points."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder requested for {name}")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    fabric = model.material("doped_fabric", rgba=(0.70, 0.78, 0.83, 1.0))
    seam = model.material("reinforced_seams", rgba=(0.48, 0.56, 0.61, 1.0))
    frame = model.material("dark_frame", rgba=(0.08, 0.09, 0.10, 1.0))
    gondola_skin = model.material("gondola_skin", rgba=(0.74, 0.72, 0.66, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.12, 0.15, 0.95))
    pod_skin = model.material("engine_pod_skin", rgba=(0.30, 0.32, 0.33, 1.0))
    tail_skin = model.material("tail_fabric", rgba=(0.62, 0.66, 0.68, 1.0))
    control_skin = model.material("control_surfaces", rgba=(0.83, 0.36, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    metal = model.material("brushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    blade_mat = model.material("propeller_blades", rgba=(0.10, 0.10, 0.11, 1.0))

    airframe = model.part("airframe")

    # A full workhorse-scale pressure envelope: about 32 m long and 8.7 m thick.
    envelope_profile = [
        (0.00, -16.0),
        (0.75, -15.3),
        (2.30, -13.5),
        (3.65, -10.2),
        (4.25, -5.0),
        (4.35, 1.5),
        (4.05, 8.5),
        (2.55, 13.2),
        (0.80, 15.35),
        (0.00, 16.0),
    ]
    envelope = LatheGeometry(envelope_profile, segments=72, closed=True)
    envelope.rotate_y(math.pi / 2.0)
    airframe.visual(
        mesh_from_geometry(envelope, "thick_envelope"),
        material=fabric,
        name="envelope",
    )

    # Raised circumferential reinforcing bands read as fabric seams/ballonet straps.
    for i, (x, r) in enumerate(
        [(-11.5, 3.12), (-7.0, 4.02), (-2.5, 4.29), (2.5, 4.30), (7.0, 4.11), (11.5, 3.10)]
    ):
        belt = TorusGeometry(r, 0.090, radial_segments=16, tubular_segments=96)
        belt.rotate_y(math.pi / 2.0)
        belt.translate(x, 0.0, 0.0)
        airframe.visual(
            mesh_from_geometry(belt, f"envelope_belt_{i}"),
            material=seam,
            name=f"envelope_belt_{i}",
        )

    # Compact suspended gondola below the centerline.
    airframe.visual(
        Box((5.6, 1.75, 1.55)),
        origin=Origin(xyz=(0.0, 0.0, -5.25)),
        material=gondola_skin,
        name="gondola_body",
    )
    airframe.visual(
        Box((5.15, 1.35, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -4.34)),
        material=frame,
        name="gondola_roof_rail",
    )
    airframe.visual(
        Box((1.05, 1.79, 0.48)),
        origin=Origin(xyz=(2.38, 0.0, -5.03)),
        material=glass,
        name="front_windscreen",
    )
    for i, x in enumerate((-1.65, -0.45, 0.75)):
        airframe.visual(
            Box((0.62, 0.050, 0.45)),
            origin=Origin(xyz=(x, 0.885, -5.05)),
            material=glass,
            name=f"port_window_{i}",
        )
        airframe.visual(
            Box((0.62, 0.050, 0.45)),
            origin=Origin(xyz=(x, -0.885, -5.05)),
            material=glass,
            name=f"starboard_window_{i}",
        )

    # Visible hangar-style support struts from the envelope belly to the gondola roof.
    strut_pairs = [
        ((-2.25, 1.25, -3.55), (-2.25, 0.62, -4.24)),
        ((-2.25, -1.25, -3.55), (-2.25, -0.62, -4.24)),
        ((0.00, 1.35, -3.70), (0.00, 0.64, -4.24)),
        ((0.00, -1.35, -3.70), (0.00, -0.64, -4.24)),
        ((2.25, 1.25, -3.55), (2.25, 0.62, -4.24)),
        ((2.25, -1.25, -3.55), (2.25, -0.62, -4.24)),
    ]
    for i, (a, b) in enumerate(strut_pairs):
        _cylinder_between(airframe, a, b, 0.055, material=frame, name=f"gondola_strut_{i}")
    _cylinder_between(airframe, (-2.7, 0.0, -4.21), (2.7, 0.0, -4.21), 0.045, material=frame, name="gondola_keel_truss")

    # Twin side engine pods with hard points tied back to the envelope.
    for side_name, y, sign, pod_name, shaft_name, upper_name, lower_name, drag_name in (
        (
            "port",
            5.35,
            1.0,
            "port_engine_pod",
            "port_propeller_shaft",
            "port_pod_upper_strut",
            "port_pod_lower_strut",
            "port_pod_drag_strut",
        ),
        (
            "starboard",
            -5.35,
            -1.0,
            "starboard_engine_pod",
            "starboard_propeller_shaft",
            "starboard_pod_upper_strut",
            "starboard_pod_lower_strut",
            "starboard_pod_drag_strut",
        ),
    ):
        airframe.visual(
            Cylinder(radius=0.55, length=1.85),
            origin=Origin(xyz=(1.0, y, -1.15), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pod_skin,
            name=pod_name,
        )
        airframe.visual(
            Cylinder(radius=0.066, length=0.72),
            origin=Origin(xyz=(2.22, y, -1.15), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=shaft_name,
        )
        _cylinder_between(
            airframe,
            (0.45, sign * 3.85, -0.85),
            (0.45, sign * 4.95, -1.15),
            0.065,
            material=frame,
            name=upper_name,
        )
        _cylinder_between(
            airframe,
            (1.55, sign * 3.70, -1.75),
            (1.55, sign * 4.95, -1.15),
            0.060,
            material=frame,
            name=lower_name,
        )
        _cylinder_between(
            airframe,
            (-0.05, sign * 3.95, -1.25),
            (1.95, sign * 4.95, -1.15),
            0.050,
            material=frame,
            name=drag_name,
        )

    # Boxy cruciform fixed tail surfaces, joined into the rear envelope.
    airframe.visual(
        Box((3.2, 0.18, 3.75)),
        origin=Origin(xyz=(-14.45, 0.0, 0.0)),
        material=tail_skin,
        name="vertical_fin",
    )
    airframe.visual(
        Box((3.2, 6.7, 0.18)),
        origin=Origin(xyz=(-14.45, 0.0, 0.0)),
        material=tail_skin,
        name="tailplane",
    )
    airframe.visual(
        Box((0.18, 0.28, 3.85)),
        origin=Origin(xyz=(-16.05, 0.0, 0.0)),
        material=metal,
        name="rudder_hinge_post",
    )
    airframe.visual(
        Box((0.18, 6.85, 0.28)),
        origin=Origin(xyz=(-16.05, 0.0, 0.0)),
        material=metal,
        name="elevator_hinge_tube",
    )

    # Real undercarriage: V struts, a transverse axle, and fork lugs below the gondola.
    _cylinder_between(airframe, (-1.25, 0.62, -6.02), (-0.72, 1.51, -6.25), 0.050, material=frame, name="port_gear_front_strut")
    _cylinder_between(airframe, (0.35, 0.62, -6.02), (-0.72, 1.51, -6.25), 0.050, material=frame, name="port_gear_rear_strut")
    _cylinder_between(airframe, (-1.25, -0.62, -6.02), (-0.72, -1.51, -6.25), 0.050, material=frame, name="starboard_gear_front_strut")
    _cylinder_between(airframe, (0.35, -0.62, -6.02), (-0.72, -1.51, -6.25), 0.050, material=frame, name="starboard_gear_rear_strut")
    airframe.visual(
        Cylinder(radius=0.048, length=3.20),
        origin=Origin(xyz=(-0.72, 0.0, -6.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="landing_axle",
    )
    airframe.visual(
        Box((0.18, 0.08, 0.50)),
        origin=Origin(xyz=(-0.72, 1.51, -6.43)),
        material=metal,
        name="port_wheel_fork",
    )
    airframe.visual(
        Box((0.18, 0.08, 0.50)),
        origin=Origin(xyz=(-0.72, -1.51, -6.43)),
        material=metal,
        name="starboard_wheel_fork",
    )

    # Continuously rotating propellers, authored as separate articulated parts.
    prop_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.86,
            0.16,
            5,
            thickness=0.13,
            blade_pitch_deg=32.0,
            blade_sweep_deg=26.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
            hub=FanRotorHub(style="spinner", bore_diameter=0.13, rear_collar_height=0.05),
        ),
        "propeller_rotor",
    )
    for side_name, y in (("port", 5.35), ("starboard", -5.35)):
        prop = model.part(f"{side_name}_propeller")
        prop.visual(prop_mesh, material=blade_mat, name="rotor")
        model.articulation(
            f"{side_name}_propeller_spin",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=prop,
            origin=Origin(xyz=(2.58, y, -1.15), rpy=(0.0, math.pi / 2.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=60.0, velocity=90.0),
        )

    # Rudder and independent elevator halves with realistic hinge axes and travel.
    rudder = model.part("rudder")
    rudder.visual(
        Box((0.78, 0.13, 2.85)),
        origin=Origin(xyz=(-0.39, 0.0, 0.0)),
        material=control_skin,
        name="rudder_panel",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-16.14, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850.0, velocity=1.2, lower=-0.52, upper=0.52),
    )

    for side_name, y in (("port", 2.25), ("starboard", -2.25)):
        elevator = model.part(f"{side_name}_elevator")
        elevator.visual(
            Box((0.78, 2.25, 0.13)),
            origin=Origin(xyz=(-0.39, 0.0, 0.0)),
            material=control_skin,
            name="elevator_panel",
        )
        model.articulation(
            f"{side_name}_elevator_hinge",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=elevator,
            origin=Origin(xyz=(-16.14, y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=520.0, velocity=1.4, lower=-0.44, upper=0.44),
        )

    # Landing wheels roll continuously on the transverse axle.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.38,
            0.18,
            inner_radius=0.255,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.030, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.025, depth=0.010),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.020, radius=0.006),
        ),
        "landing_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.275,
            0.16,
            rim=WheelRim(inner_radius=0.19, flange_height=0.018, flange_thickness=0.008, bead_seat_depth=0.006),
            hub=WheelHub(
                radius=0.080,
                width=0.11,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.095, hole_diameter=0.014),
            ),
            face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.012, window_radius=0.045),
            bore=WheelBore(style="round", diameter=0.095),
        ),
        "landing_wheel_rim",
    )
    for side_name, y in (("port", 1.35), ("starboard", -1.35)):
        wheel = model.part(f"{side_name}_wheel")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(rim_mesh, material=metal, name="rim")
        model.articulation(
            f"{side_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=wheel,
            origin=Origin(xyz=(-0.72, y, -6.70), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=180.0, velocity=35.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")

    moving_joint_names = [
        "port_propeller_spin",
        "starboard_propeller_spin",
        "port_wheel_spin",
        "starboard_wheel_spin",
    ]
    ctx.check(
        "propellers and wheels use continuous spin joints",
        all(object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS for name in moving_joint_names),
    )

    for name in ("rudder_hinge", "port_elevator_hinge", "starboard_elevator_hinge"):
        joint = object_model.get_articulation(name)
        limits = joint.motion_limits
        ctx.check(
            f"{name} has bounded aerodynamic travel",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
        )

    envelope_box = ctx.part_element_world_aabb(airframe, elem="envelope")
    gondola_box = ctx.part_element_world_aabb(airframe, elem="gondola_body")
    axle_box = ctx.part_element_world_aabb(airframe, elem="landing_axle")
    ctx.check(
        "gondola hangs below the thick envelope",
        envelope_box is not None
        and gondola_box is not None
        and gondola_box[1][2] < (envelope_box[0][2] + envelope_box[1][2]) * 0.5,
        details=f"envelope={envelope_box}, gondola={gondola_box}",
    )
    ctx.check(
        "landing axle is supported below gondola structure",
        gondola_box is not None and axle_box is not None and axle_box[1][2] < gondola_box[0][2],
        details=f"gondola={gondola_box}, axle={axle_box}",
    )

    port_wheel = object_model.get_part("port_wheel")
    starboard_wheel = object_model.get_part("starboard_wheel")
    port_propeller = object_model.get_part("port_propeller")
    starboard_propeller = object_model.get_part("starboard_propeller")

    ctx.allow_overlap(
        airframe,
        port_propeller,
        elem_a="port_propeller_shaft",
        elem_b="rotor",
        reason="The fixed shaft is intentionally captured through the propeller hub/bearing so the rotor is physically supported while spinning.",
    )
    ctx.allow_overlap(
        airframe,
        starboard_propeller,
        elem_a="starboard_propeller_shaft",
        elem_b="rotor",
        reason="The fixed shaft is intentionally captured through the propeller hub/bearing so the rotor is physically supported while spinning.",
    )
    ctx.expect_overlap(
        port_propeller,
        airframe,
        axes="x",
        elem_a="rotor",
        elem_b="port_propeller_shaft",
        min_overlap=0.04,
        name="port propeller hub is retained on shaft",
    )
    ctx.expect_overlap(
        starboard_propeller,
        airframe,
        axes="x",
        elem_a="rotor",
        elem_b="starboard_propeller_shaft",
        min_overlap=0.04,
        name="starboard propeller hub is retained on shaft",
    )
    ctx.allow_overlap(
        airframe,
        port_wheel,
        elem_a="landing_axle",
        elem_b="rim",
        reason="The wheel rim/hub is intentionally captured around the fixed landing axle so the wheel is supported while rolling.",
    )
    ctx.allow_overlap(
        airframe,
        starboard_wheel,
        elem_a="landing_axle",
        elem_b="rim",
        reason="The wheel rim/hub is intentionally captured around the fixed landing axle so the wheel is supported while rolling.",
    )
    ctx.expect_overlap(
        port_wheel,
        airframe,
        axes="z",
        elem_a="rim",
        elem_b="landing_axle",
        min_overlap=0.02,
        name="port wheel rim surrounds axle height",
    )
    ctx.expect_overlap(
        starboard_wheel,
        airframe,
        axes="z",
        elem_a="rim",
        elem_b="landing_axle",
        min_overlap=0.02,
        name="starboard wheel rim surrounds axle height",
    )

    return ctx.report()


object_model = build_object_model()
