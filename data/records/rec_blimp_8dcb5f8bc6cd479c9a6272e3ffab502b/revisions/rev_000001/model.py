from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    Material,
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


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    segments: int = 16,
):
    """Closed cylinder mesh from start to end in the current part frame."""
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    geom = CylinderGeometry(radius, length, radial_segments=segments, closed=True)
    if length > 1e-9:
        ux, uy, uz = vx / length, vy / length, vz / length
        dot = max(-1.0, min(1.0, uz))
        angle = math.acos(dot)
        axis = (-uy, ux, 0.0)
        axis_len = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1])
        if axis_len > 1e-9:
            geom.rotate(axis, angle)
        elif uz < 0.0:
            geom.rotate((1.0, 0.0, 0.0), math.pi)
    geom.translate(
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return geom


def _envelope_geometry():
    length = 60.0
    radius = 7.5
    profile: list[tuple[float, float]] = []
    for i in range(81):
        u = i / 80.0
        z = -length * 0.5 + length * u
        # A cigar envelope: broad midbody with long, smooth tapered ends.
        r = radius * (max(0.0, math.sin(math.pi * u)) ** 0.43)
        profile.append((r, z))
    geom = LatheGeometry(profile, segments=72, closed=True)
    # Lathe builds along local Z; rotate the envelope to the aircraft X axis.
    geom.rotate_y(math.pi / 2.0)
    return geom


def _pod_geometry():
    geom = CapsuleGeometry(0.72, 1.65, radial_segments=32, height_segments=8)
    geom.rotate_y(math.pi / 2.0)
    return geom


def _make_propeller_mesh(name: str):
    return mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=0.95,
            hub_radius=0.18,
            blade_count=4,
            thickness=0.16,
            blade_pitch_deg=31.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.075),
        ),
        name,
    )


def _make_tire_mesh(name: str):
    return mesh_from_geometry(
        TireGeometry(
            outer_radius=0.34,
            width=0.32,
            inner_radius=0.235,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.018, count=18, land_ratio=0.60),
            grooves=(TireGroove(center_offset=0.0, width=0.020, depth=0.006),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.018, radius=0.008),
        ),
        name,
    )


def _make_rim_mesh(name: str):
    return mesh_from_geometry(
        WheelGeometry(
            radius=0.235,
            width=0.34,
            rim=WheelRim(
                inner_radius=0.145,
                flange_height=0.018,
                flange_thickness=0.010,
                bead_seat_depth=0.006,
            ),
            hub=WheelHub(
                radius=0.082,
                width=0.28,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.105,
                    hole_diameter=0.013,
                ),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.012, window_radius=0.030),
            bore=WheelBore(style="round", diameter=0.080),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="advertising_blimp")

    envelope_mat = model.material("pearl_envelope", rgba=(0.93, 0.94, 0.90, 1.0))
    stripe_mat = model.material("deep_blue_advertising", rgba=(0.02, 0.12, 0.34, 1.0))
    banner_mat = model.material("warm_yellow_banner", rgba=(1.0, 0.80, 0.16, 1.0))
    gondola_mat = model.material("white_gondola", rgba=(0.92, 0.95, 0.98, 1.0))
    glass_mat = model.material("dark_glass", rgba=(0.02, 0.04, 0.07, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    red_mat = model.material("tail_red", rgba=(0.78, 0.03, 0.04, 1.0))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_geometry(_envelope_geometry(), "cigar_envelope"),
        material=envelope_mat,
        name="cigar_envelope",
    )
    # A shallow external keel gives the underslung gondola a real hard point.
    envelope.visual(
        Box((10.4, 0.48, 0.30)),
        origin=Origin(xyz=(6.0, 0.0, -7.55)),
        material=metal_mat,
        name="keel_rail",
    )
    # Flat side advertising panels sit proud of the broad envelope sides.
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        envelope.visual(
            Box((18.5, 0.10, 2.35)),
            origin=Origin(xyz=(3.0, sign * 7.43, 0.35)),
            material=banner_mat,
            name=f"{side_name}_banner",
        )
        # Bold block marks suggest painted advertising copy without text assets.
        for i, x in enumerate((-2.8, 0.0, 2.8)):
            envelope.visual(
                Box((1.35, 0.07, 1.55)),
                origin=Origin(xyz=(x, sign * 7.48, 0.35)),
                material=stripe_mat,
                name=f"{side_name}_ad_block_{i}",
            )
    # Side hardpoints for the two engine nacelles.
    envelope.visual(
        Box((2.25, 0.50, 0.70)),
        origin=Origin(xyz=(4.0, 7.35, -2.40)),
        material=metal_mat,
        name="left_nacelle_pad",
    )
    envelope.visual(
        Box((2.25, 0.50, 0.70)),
        origin=Origin(xyz=(4.0, -7.35, -2.40)),
        material=metal_mat,
        name="right_nacelle_pad",
    )
    # Cruciform fixed tail surfaces; moving rudder/elevators hinge behind them.
    envelope.visual(
        Box((3.30, 0.22, 4.70)),
        origin=Origin(xyz=(-28.85, 0.0, 4.05)),
        material=stripe_mat,
        name="upper_fin",
    )
    envelope.visual(
        Box((3.30, 0.22, 4.70)),
        origin=Origin(xyz=(-28.85, 0.0, -4.05)),
        material=stripe_mat,
        name="lower_fin",
    )
    envelope.visual(
        Box((3.30, 4.60, 0.22)),
        origin=Origin(xyz=(-28.85, 4.00, 0.0)),
        material=stripe_mat,
        name="left_stabilizer",
    )
    envelope.visual(
        Box((3.30, 4.60, 0.22)),
        origin=Origin(xyz=(-28.85, -4.00, 0.0)),
        material=stripe_mat,
        name="right_stabilizer",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((8.0, 2.45, 2.20)),
        origin=Origin(),
        material=gondola_mat,
        name="cabin_shell",
    )
    gondola.visual(
        Box((7.1, 2.55, 0.28)),
        origin=Origin(xyz=(-0.2, 0.0, -0.95)),
        material=stripe_mat,
        name="blue_belly",
    )
    # Windshield and cabin side windows, slightly proud of the shell.
    gondola.visual(
        Box((0.06, 1.55, 0.72)),
        origin=Origin(xyz=(4.03, 0.0, 0.28)),
        material=glass_mat,
        name="windshield",
    )
    for sign in (1.0, -1.0):
        for i, x in enumerate((2.5, 1.35, 0.20, -0.95, -2.10)):
            gondola.visual(
                Box((0.72, 0.06, 0.52)),
                origin=Origin(xyz=(x, sign * 1.255, 0.28)),
                material=glass_mat,
                name=f"side_window_{'left' if sign > 0 else 'right'}_{i}",
            )
    # Four roof struts and pads touch the envelope keel rail, keeping the cabin
    # visibly suspended below the envelope rather than merged into it.
    for i, x in enumerate((-3.35, -1.15, 1.15, 3.35)):
        y = 0.74 if i % 2 == 0 else -0.74
        gondola.visual(
            Box((0.64, 0.34, 0.08)),
            origin=Origin(xyz=(x, 0.0, 1.86)),
            material=metal_mat,
            name=f"roof_pad_{i}",
        )
        gondola.visual(
            mesh_from_geometry(
                _cylinder_between((x * 0.92, y, 1.10), (x, 0.14 if y > 0 else -0.14, 1.82), 0.055),
                f"gondola_strut_{i}",
            ),
            material=metal_mat,
            name=f"roof_strut_{i}",
        )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(6.0, 0.0, -9.60)),
    )

    nacelles = []
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        nacelle = model.part(f"{side_name}_nacelle")
        nacelles.append((side_name, sign, nacelle))
        nacelle.visual(
            mesh_from_geometry(_pod_geometry(), f"{side_name}_engine_pod"),
            material=metal_mat,
            name="engine_pod",
        )
        nacelle.visual(
            Cylinder(radius=0.78, length=0.16),
            origin=Origin(xyz=(1.50, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stripe_mat,
            name="cowl_ring",
        )
        nacelle.visual(
            Cylinder(radius=0.060, length=0.58),
            origin=Origin(xyz=(1.52, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="shaft",
        )
        nacelle.visual(
            Box((1.75, 0.08, 0.42)),
            origin=Origin(xyz=(0.0, -sign * 1.11, 0.30)),
            material=metal_mat,
            name="mount_pad",
        )
        for i, x in enumerate((-0.70, 0.70)):
            nacelle.visual(
                mesh_from_geometry(
                    _cylinder_between(
                        (x, -sign * 1.11, 0.30),
                        (x * 0.75, -sign * 0.55, 0.50),
                        0.065,
                    ),
                    f"{side_name}_nacelle_strut_{i}",
                ),
                material=metal_mat,
                name=f"mount_strut_{i}",
            )
        model.articulation(
            f"envelope_to_{side_name}_nacelle",
            ArticulationType.FIXED,
            parent=envelope,
            child=nacelle,
            origin=Origin(xyz=(4.0, sign * 8.75, -2.70)),
        )

        propeller = model.part(f"{side_name}_propeller")
        propeller.visual(
            _make_propeller_mesh(f"{side_name}_propeller_rotor"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="rotor",
        )
        model.articulation(
            f"{side_name}_propeller_spin",
            ArticulationType.CONTINUOUS,
            parent=nacelle,
            child=propeller,
            origin=Origin(xyz=(1.82, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=95.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        Cylinder(radius=0.080, length=7.80),
        origin=Origin(),
        material=metal_mat,
        name="hinge_barrel",
    )
    rudder.visual(
        Box((2.00, 0.20, 7.40)),
        origin=Origin(xyz=(-1.00, 0.0, 0.0)),
        material=red_mat,
        name="rudder_panel",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=rudder,
        origin=Origin(xyz=(-30.58, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        elevator = model.part(f"{side_name}_elevator")
        elevator.visual(
            Cylinder(radius=0.080, length=3.80),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name="hinge_barrel",
        )
        elevator.visual(
            Box((2.00, 3.80, 0.20)),
            origin=Origin(xyz=(-1.00, 0.0, 0.0)),
            material=red_mat,
            name="elevator_panel",
        )
        model.articulation(
            f"{side_name}_elevator_hinge",
            ArticulationType.REVOLUTE,
            parent=envelope,
            child=elevator,
            origin=Origin(xyz=(-30.58, sign * 4.00, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.45, upper=0.45),
        )

    gear = model.part("gear")
    gear.visual(
        Box((6.6, 1.55, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=metal_mat,
        name="top_plate",
    )
    gear.visual(
        Cylinder(radius=0.052, length=3.10),
        origin=Origin(xyz=(-2.35, 0.0, -0.72), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="main_axle",
    )
    gear.visual(
        Cylinder(radius=0.050, length=0.90),
        origin=Origin(xyz=(2.70, 0.0, -0.70), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="nose_axle",
    )
    for i, y in enumerate((-0.62, 0.62)):
        gear.visual(
            mesh_from_geometry(
                _cylinder_between((-2.95, y, -0.10), (-2.35, y * 1.48, -0.72), 0.040),
                f"main_gear_strut_{i}",
            ),
            material=metal_mat,
            name=f"main_strut_{i}",
        )
        gear.visual(
            mesh_from_geometry(
                _cylinder_between((-2.95, y, -0.10), (2.70, 0.0, -0.10), 0.035),
                f"gear_longeron_{i}",
            ),
            material=metal_mat,
            name=f"longeron_{i}",
        )
    gear.visual(
        mesh_from_geometry(
            _cylinder_between((2.70, 0.0, -0.10), (2.70, 0.0, -0.28), 0.055),
            "nose_gear_strut",
        ),
        material=metal_mat,
        name="nose_strut",
    )
    gear.visual(
        Cylinder(radius=0.045, length=0.70),
        origin=Origin(xyz=(2.70, 0.0, -0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="nose_fork_bridge",
    )
    for i, y in enumerate((-0.36, 0.36)):
        gear.visual(
            mesh_from_geometry(
                _cylinder_between((2.70, y, -0.28), (2.70, y, -0.70), 0.045),
                f"nose_fork_{i}",
            ),
            material=metal_mat,
            name=f"nose_fork_{i}",
        )
    model.articulation(
        "gondola_to_gear",
        ArticulationType.FIXED,
        parent=gondola,
        child=gear,
        origin=Origin(xyz=(0.0, 0.0, -1.10)),
    )

    wheel_specs = (
        ("left_wheel", (-2.35, 1.45, -0.72), "main_axle"),
        ("right_wheel", (-2.35, -1.45, -0.72), "main_axle"),
        ("nose_wheel", (2.70, 0.0, -0.70), "nose_axle"),
    )
    for wheel_name, xyz, _axle in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.visual(
            _make_tire_mesh(f"{wheel_name}_tire"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber_mat,
            name="tire",
        )
        wheel.visual(
            _make_rim_mesh(f"{wheel_name}_rim"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=metal_mat,
            name="rim",
        )
        model.articulation(
            f"{wheel_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=gear,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    gear = object_model.get_part("gear")

    ctx.expect_contact(
        gondola,
        envelope,
        elem_a="roof_pad_0",
        elem_b="keel_rail",
        contact_tol=0.002,
        name="gondola roof pads touch the envelope keel",
    )
    ctx.expect_contact(
        gear,
        gondola,
        elem_a="top_plate",
        elem_b="cabin_shell",
        contact_tol=0.002,
        name="landing gear top plate is mounted to the gondola",
    )
    for side_name in ("left", "right"):
        nacelle = object_model.get_part(f"{side_name}_nacelle")
        propeller = object_model.get_part(f"{side_name}_propeller")
        pad_name = f"{side_name}_nacelle_pad"
        ctx.expect_contact(
            nacelle,
            envelope,
            elem_a="mount_pad",
            elem_b=pad_name,
            contact_tol=0.002,
            name=f"{side_name} nacelle pad touches envelope hardpoint",
        )
        spin = object_model.get_articulation(f"{side_name}_propeller_spin")
        ctx.check(
            f"{side_name} propeller is continuous on shaft axis",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )
        ctx.allow_overlap(
            nacelle,
            propeller,
            elem_a="shaft",
            elem_b="rotor",
            reason="The propeller hub is intentionally captured on the local engine shaft.",
        )
        ctx.expect_gap(
            propeller,
            nacelle,
            axis="x",
            positive_elem="rotor",
            negative_elem="shaft",
            min_gap=-0.075,
            max_gap=0.006,
            name=f"{side_name} propeller hub is seated on the shaft end",
        )

    rudder_joint = object_model.get_articulation("rudder_hinge")
    ctx.check(
        "rudder hinges about vertical tail axis",
        rudder_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(rudder_joint.axis) == (0.0, 0.0, 1.0)
        and rudder_joint.motion_limits is not None
        and rudder_joint.motion_limits.lower < 0.0
        and rudder_joint.motion_limits.upper > 0.0,
        details=f"axis={rudder_joint.axis}, limits={rudder_joint.motion_limits}",
    )
    for side_name in ("left", "right"):
        elevator_joint = object_model.get_articulation(f"{side_name}_elevator_hinge")
        ctx.check(
            f"{side_name} elevator hinges about horizontal tail axis",
            elevator_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(elevator_joint.axis) == (0.0, 1.0, 0.0)
            and elevator_joint.motion_limits is not None
            and elevator_joint.motion_limits.lower < 0.0
            and elevator_joint.motion_limits.upper > 0.0,
            details=f"axis={elevator_joint.axis}, limits={elevator_joint.motion_limits}",
        )

    for wheel_name, axle_name in (
        ("left_wheel", "main_axle"),
        ("right_wheel", "main_axle"),
        ("nose_wheel", "nose_axle"),
    ):
        wheel = object_model.get_part(wheel_name)
        spin = object_model.get_articulation(f"{wheel_name}_spin")
        ctx.allow_overlap(
            gear,
            wheel,
            elem_a=axle_name,
            elem_b="rim",
            reason="The landing wheel rim is intentionally captured on the metal axle.",
        )
        ctx.expect_within(
            gear,
            wheel,
            axes="xz",
            inner_elem=axle_name,
            outer_elem="rim",
            margin=0.010,
            name=f"{wheel_name} axle runs through rim center",
        )
        ctx.expect_overlap(
            gear,
            wheel,
            axes="y",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.04,
            name=f"{wheel_name} rim remains on its axle",
        )
        ctx.check(
            f"{wheel_name} spins continuously on axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

    return ctx.report()


object_model = build_object_model()
