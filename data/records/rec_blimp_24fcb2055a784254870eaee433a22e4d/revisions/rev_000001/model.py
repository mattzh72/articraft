from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube(name: str, points, radius: float, *, samples: int = 10, radial_segments: int = 16, cap_ends: bool = True):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=radial_segments,
            cap_ends=cap_ends,
        ),
    )


def _make_envelope_mesh():
    return LatheGeometry(
        [
            (0.00, -18.80),
            (0.28, -18.45),
            (0.92, -17.70),
            (1.85, -16.20),
            (2.95, -13.20),
            (3.78, -8.20),
            (4.06, -2.20),
            (4.12, 2.80),
            (3.82, 8.80),
            (3.05, 13.90),
            (1.95, 17.20),
            (0.82, 19.00),
            (0.16, 19.45),
            (0.00, 19.60),
        ],
        segments=88,
    ).rotate_y(math.pi / 2.0)


def _make_ad_band_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (3.90, -6.20),
            (4.02, -3.60),
            (4.03, 4.20),
            (3.92, 6.90),
        ],
        [
            (3.82, -6.20),
            (3.94, -3.60),
            (3.95, 4.20),
            (3.84, 6.90),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _make_nacelle_mesh():
    return LatheGeometry(
        [
            (0.00, -1.55),
            (0.17, -1.42),
            (0.32, -1.12),
            (0.45, -0.45),
            (0.48, 0.20),
            (0.45, 0.92),
            (0.36, 1.42),
            (0.22, 1.70),
            (0.08, 1.86),
            (0.00, 1.94),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)


def _make_propeller_mesh():
    return FanRotorGeometry(
        1.00,
        0.14,
        3,
        thickness=0.12,
        blade_pitch_deg=28.0,
        blade_sweep_deg=30.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.18),
        hub=FanRotorHub(
            style="spinner",
            rear_collar_height=0.08,
            rear_collar_radius=0.10,
            bore_diameter=0.05,
        ),
    ).rotate_y(math.pi / 2.0)


def _make_gondola_shell():
    return CapsuleGeometry(radius=0.62, length=4.55, radial_segments=36, height_segments=12).rotate_y(
        math.pi / 2.0
    ).scale(1.0, 1.08, 1.25)


def _horizontal_fixed_mesh(name: str, span_sign: float):
    profile = [
        (0.0, 0.0),
        (1.18, 0.08 * span_sign),
        (0.78, 2.55 * span_sign),
        (0.0, 2.85 * span_sign),
    ]
    return _mesh(name, ExtrudeGeometry(profile, 0.10, center=True))


def _elevator_mesh(name: str, span_sign: float):
    profile = [
        (-0.78, 0.0),
        (0.0, 0.0),
        (0.0, 2.45 * span_sign),
        (-0.58, 2.18 * span_sign),
    ]
    return _mesh(name, ExtrudeGeometry(profile, 0.09, center=True))


def _vertical_fixed_mesh(name: str, top: bool):
    if top:
        profile = [
            (0.0, 0.0),
            (1.10, 0.16),
            (0.76, 2.18),
            (0.0, 2.50),
        ]
    else:
        profile = [
            (0.0, 0.0),
            (0.92, -0.18),
            (0.52, -1.82),
            (0.0, -1.60),
        ]
    return _mesh(name, ExtrudeGeometry(profile, 0.10, center=True).rotate_x(math.pi / 2.0))


def _rudder_mesh():
    profile = [
        (-0.88, 0.0),
        (0.0, 0.0),
        (0.0, 2.10),
        (-0.72, 1.88),
    ]
    return _mesh("rudder_surface", ExtrudeGeometry(profile, 0.09, center=True).rotate_x(math.pi / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="advertising_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.92, 0.94, 0.95, 1.0))
    ad_red = model.material("ad_red", rgba=(0.77, 0.13, 0.12, 1.0))
    gondola_gray = model.material("gondola_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.28, 0.31, 0.35, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.12, 0.15, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.61, 0.65, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    envelope = model.part("envelope")
    envelope.visual(_mesh("envelope_skin", _make_envelope_mesh()), material=envelope_white, name="envelope_skin")
    envelope.visual(_mesh("advertising_band", _make_ad_band_mesh()), material=ad_red, name="advertising_band")

    keel = model.part("keel")
    keel.visual(
        Box((10.80, 0.66, 0.40)),
        origin=Origin(xyz=(0.30, 0.0, -4.55)),
        material=trim_gray,
        name="keel_beam",
    )
    keel.visual(
        Box((3.20, 0.50, 0.26)),
        origin=Origin(xyz=(0.30, 0.0, -4.23)),
        material=trim_gray,
        name="saddle_pad",
    )
    keel.visual(
        _tube(
            "keel_anchor_front",
            [(-4.00, 0.20, -4.36), (-4.20, 0.40, -4.12), (-4.38, 0.72, -4.02)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="anchor_front_port",
    )
    keel.visual(
        _tube(
            "keel_anchor_front_mirror",
            [(-4.00, -0.20, -4.36), (-4.20, -0.40, -4.12), (-4.38, -0.72, -4.02)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="anchor_front_starboard",
    )
    keel.visual(
        _tube(
            "keel_anchor_rear",
            [(4.20, 0.20, -4.36), (4.42, 0.56, -4.20), (4.64, 1.02, -4.12)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="anchor_rear_port",
    )
    keel.visual(
        _tube(
            "keel_anchor_rear_mirror",
            [(4.20, -0.20, -4.36), (4.42, -0.56, -4.20), (4.64, -1.02, -4.12)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="anchor_rear_starboard",
    )
    keel.visual(
        _tube(
            "gondola_hanger_front_port",
            [(0.95, 0.18, -4.74), (0.98, 0.24, -5.06), (1.02, 0.29, -5.46)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="hanger_front_port",
    )
    keel.visual(
        _tube(
            "gondola_hanger_front_starboard",
            [(0.95, -0.18, -4.74), (0.98, -0.24, -5.06), (1.02, -0.29, -5.46)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="hanger_front_starboard",
    )
    keel.visual(
        _tube(
            "gondola_hanger_rear_port",
            [(-0.64, 0.18, -4.74), (-0.68, 0.24, -5.06), (-0.74, 0.29, -5.46)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="hanger_rear_port",
    )
    keel.visual(
        _tube(
            "gondola_hanger_rear_starboard",
            [(-0.64, -0.18, -4.74), (-0.68, -0.24, -5.06), (-0.74, -0.29, -5.46)],
            radius=0.08,
            cap_ends=False,
        ),
        material=trim_gray,
        name="hanger_rear_starboard",
    )
    keel.visual(
        _tube(
            "port_mount_brace",
            [(0.52, 0.30, -4.42), (0.74, 1.36, -4.48), (0.92, 2.48, -4.50)],
            radius=0.09,
            cap_ends=False,
        ),
        material=trim_gray,
        name="port_mount_brace",
    )
    keel.visual(
        _tube(
            "starboard_mount_brace",
            [(0.52, -0.30, -4.42), (0.74, -1.36, -4.48), (0.92, -2.48, -4.50)],
            radius=0.09,
            cap_ends=False,
        ),
        material=trim_gray,
        name="starboard_mount_brace",
    )

    gondola = model.part("gondola")
    gondola.visual(_mesh("gondola_shell", _make_gondola_shell()), material=gondola_gray, name="cabin_shell")
    gondola.visual(
        Box((4.90, 1.46, 0.60)),
        origin=Origin(xyz=(-0.10, 0.0, -0.54)),
        material=gondola_gray,
        name="cabin_floor",
    )
    gondola.visual(
        Box((3.20, 1.42, 0.42)),
        origin=Origin(xyz=(0.52, 0.0, 0.18)),
        material=dark_glass,
        name="window_band",
    )
    gondola.visual(
        Box((0.62, 0.74, 0.12)),
        origin=Origin(xyz=(0.27, 0.0, 0.76)),
        material=trim_gray,
        name="roof_pad_front",
    )
    gondola.visual(
        Box((0.62, 0.74, 0.12)),
        origin=Origin(xyz=(-1.49, 0.0, 0.74)),
        material=trim_gray,
        name="roof_pad_rear",
    )
    gondola.visual(
        Box((0.20, 1.20, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -1.25)),
        material=steel,
        name="gear_crossbeam",
    )
    gondola.visual(
        _tube(
            "gear_front_port",
            [(0.96, 0.30, -0.64), (0.48, 0.42, -0.96), (0.08, 0.48, -1.25)],
            radius=0.06,
            cap_ends=False,
        ),
        material=steel,
        name="gear_front_port",
    )
    gondola.visual(
        _tube(
            "gear_front_starboard",
            [(0.96, -0.30, -0.64), (0.48, -0.42, -0.96), (0.08, -0.48, -1.25)],
            radius=0.06,
            cap_ends=False,
        ),
        material=steel,
        name="gear_front_starboard",
    )
    gondola.visual(
        _tube(
            "gear_rear_port",
            [(-0.96, 0.30, -0.62), (-0.48, 0.42, -0.96), (-0.08, 0.48, -1.25)],
            radius=0.06,
            cap_ends=False,
        ),
        material=steel,
        name="gear_rear_port",
    )
    gondola.visual(
        _tube(
            "gear_rear_starboard",
            [(-0.96, -0.30, -0.62), (-0.48, -0.42, -0.96), (-0.08, -0.48, -1.25)],
            radius=0.06,
            cap_ends=False,
        ),
        material=steel,
        name="gear_rear_starboard",
    )
    gondola.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(0.0, 0.60, -1.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_stub_port",
    )
    gondola.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(0.0, -0.60, -1.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_stub_starboard",
    )

    port_nacelle = model.part("port_nacelle")
    port_nacelle.visual(_mesh("port_nacelle_shell", _make_nacelle_mesh()), material=gondola_gray, name="nacelle_shell")
    port_nacelle.visual(
        _tube(
            "port_pylon",
            [(0.0, -0.38, 0.12), (0.10, -1.02, 0.56), (0.0, -2.34, 0.42)],
            radius=0.09,
            cap_ends=False,
        ),
        material=trim_gray,
        name="mount_pylon",
    )
    port_nacelle.visual(
        Cylinder(radius=0.14, length=0.36),
        origin=Origin(xyz=(1.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_collar",
    )
    port_nacelle.visual(
        Cylinder(radius=0.16, length=0.28),
        origin=Origin(xyz=(-1.56, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="exhaust_ring",
    )

    starboard_nacelle = model.part("starboard_nacelle")
    starboard_nacelle.visual(
        _mesh("starboard_nacelle_shell", _make_nacelle_mesh()),
        material=gondola_gray,
        name="nacelle_shell",
    )
    starboard_nacelle.visual(
        _tube(
            "starboard_pylon",
            [(0.0, 0.38, 0.12), (0.10, 1.02, 0.56), (0.0, 2.34, 0.42)],
            radius=0.09,
            cap_ends=False,
        ),
        material=trim_gray,
        name="mount_pylon",
    )
    starboard_nacelle.visual(
        Cylinder(radius=0.14, length=0.36),
        origin=Origin(xyz=(1.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_collar",
    )
    starboard_nacelle.visual(
        Cylinder(radius=0.16, length=0.28),
        origin=Origin(xyz=(-1.56, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="exhaust_ring",
    )

    port_propeller = model.part("port_propeller")
    port_propeller.visual(_mesh("port_propeller_rotor", _make_propeller_mesh()), material=ad_red, name="rotor")
    port_propeller.visual(
        Cylinder(radius=0.05, length=0.10),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_stub",
    )

    starboard_propeller = model.part("starboard_propeller")
    starboard_propeller.visual(
        _mesh("starboard_propeller_rotor", _make_propeller_mesh()),
        material=ad_red,
        name="rotor",
    )
    starboard_propeller.visual(
        Cylinder(radius=0.05, length=0.10),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft_stub",
    )

    tailplane = model.part("tailplane")
    tailplane.visual(
        Box((0.36, 0.58, 0.58)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=trim_gray,
        name="tail_hub",
    )
    tailplane.visual(
        Box((0.20, 0.20, 0.20)),
        origin=Origin(xyz=(1.16, 0.0, 0.0)),
        material=trim_gray,
        name="tail_root_pad",
    )
    tailplane.visual(
        Box((0.44, 0.34, 0.14)),
        origin=Origin(xyz=(0.26, 0.56, 0.0)),
        material=trim_gray,
        name="port_root_fairing",
    )
    tailplane.visual(
        Box((0.44, 0.34, 0.14)),
        origin=Origin(xyz=(0.26, -0.56, 0.0)),
        material=trim_gray,
        name="starboard_root_fairing",
    )
    tailplane.visual(
        Box((0.34, 0.12, 0.56)),
        origin=Origin(xyz=(0.22, 0.0, 0.76)),
        material=trim_gray,
        name="dorsal_root_fairing",
    )
    tailplane.visual(
        Box((0.34, 0.12, 0.50)),
        origin=Origin(xyz=(0.22, 0.0, -0.66)),
        material=trim_gray,
        name="ventral_root_fairing",
    )
    tailplane.visual(_horizontal_fixed_mesh("port_fixed_stab", 1.0), material=envelope_white, name="port_stabilizer")
    tailplane.visual(
        _horizontal_fixed_mesh("starboard_fixed_stab", -1.0),
        material=envelope_white,
        name="starboard_stabilizer",
    )
    tailplane.visual(_vertical_fixed_mesh("dorsal_fin", True), material=envelope_white, name="dorsal_fin")
    tailplane.visual(_vertical_fixed_mesh("ventral_fin", False), material=envelope_white, name="ventral_fin")

    rudder = model.part("rudder")
    rudder.visual(_rudder_mesh(), material=ad_red, name="rudder_surface")

    port_elevator = model.part("port_elevator")
    port_elevator.visual(_elevator_mesh("port_elevator_surface", 1.0), material=ad_red, name="elevator_surface")

    starboard_elevator = model.part("starboard_elevator")
    starboard_elevator.visual(
        _elevator_mesh("starboard_elevator_surface", -1.0),
        material=ad_red,
        name="elevator_surface",
    )

    port_wheel = model.part("port_wheel")
    port_wheel.visual(
        _mesh(
            "port_wheel_mesh",
            WheelGeometry(
                0.24,
                0.14,
                rim=WheelRim(inner_radius=0.18, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.004),
                hub=WheelHub(radius=0.05, width=0.11, cap_style="domed"),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="solid_slot", count=5, thickness=0.005, window_radius=0.020),
                bore=WheelBore(style="round", diameter=0.020),
            ),
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=steel,
        name="wheel_hub",
    )
    port_wheel.visual(
        _mesh("port_tire_mesh", TireGeometry(0.33, 0.16, inner_radius=0.24)),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    port_wheel.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(0.0, -0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_boss",
    )

    starboard_wheel = model.part("starboard_wheel")
    starboard_wheel.visual(
        _mesh(
            "starboard_wheel_mesh",
            WheelGeometry(
                0.24,
                0.14,
                rim=WheelRim(inner_radius=0.18, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.004),
                hub=WheelHub(radius=0.05, width=0.11, cap_style="domed"),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="solid_slot", count=5, thickness=0.005, window_radius=0.020),
                bore=WheelBore(style="round", diameter=0.020),
            ),
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=steel,
        name="wheel_hub",
    )
    starboard_wheel.visual(
        _mesh("starboard_tire_mesh", TireGeometry(0.33, 0.16, inner_radius=0.24)),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    starboard_wheel.visual(
        Cylinder(radius=0.06, length=0.07),
        origin=Origin(xyz=(0.0, 0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_boss",
    )

    model.articulation(
        "envelope_to_keel",
        ArticulationType.FIXED,
        parent=envelope,
        child=keel,
        origin=Origin(),
    )
    model.articulation(
        "keel_to_gondola",
        ArticulationType.FIXED,
        parent=keel,
        child=gondola,
        origin=Origin(xyz=(0.75, 0.0, -6.25)),
    )
    model.articulation(
        "keel_to_port_nacelle",
        ArticulationType.FIXED,
        parent=keel,
        child=port_nacelle,
        origin=Origin(xyz=(0.95, 4.82, -4.96)),
    )
    model.articulation(
        "keel_to_starboard_nacelle",
        ArticulationType.FIXED,
        parent=keel,
        child=starboard_nacelle,
        origin=Origin(xyz=(0.95, -4.82, -4.96)),
    )
    model.articulation(
        "port_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=port_nacelle,
        child=port_propeller,
        origin=Origin(xyz=(2.02, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=45.0),
    )
    model.articulation(
        "starboard_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=starboard_nacelle,
        child=starboard_propeller,
        origin=Origin(xyz=(2.02, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=45.0),
    )
    model.articulation(
        "envelope_to_tailplane",
        ArticulationType.FIXED,
        parent=envelope,
        child=tailplane,
        origin=Origin(xyz=(-19.93, 0.0, 0.0)),
    )
    model.articulation(
        "tailplane_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tailplane,
        child=rudder,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.52, upper=0.52),
    )
    model.articulation(
        "tailplane_to_port_elevator",
        ArticulationType.REVOLUTE,
        parent=tailplane,
        child=port_elevator,
        origin=Origin(xyz=(0.0, 0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "tailplane_to_starboard_elevator",
        ArticulationType.REVOLUTE,
        parent=tailplane,
        child=starboard_elevator,
        origin=Origin(xyz=(0.0, -0.72, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "gondola_to_port_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=port_wheel,
        origin=Origin(xyz=(0.0, 0.75, -1.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )
    model.articulation(
        "gondola_to_starboard_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=starboard_wheel,
        origin=Origin(xyz=(0.0, -0.75, -1.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def elem_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    port_nacelle = object_model.get_part("port_nacelle")
    starboard_nacelle = object_model.get_part("starboard_nacelle")
    port_wheel = object_model.get_part("port_wheel")
    starboard_wheel = object_model.get_part("starboard_wheel")
    rudder = object_model.get_part("rudder")
    port_elevator = object_model.get_part("port_elevator")
    starboard_elevator = object_model.get_part("starboard_elevator")

    ctx.allow_overlap(
        "keel",
        "port_nacelle",
        elem_a="port_mount_brace",
        elem_b="mount_pylon",
        reason="The nacelle pylon meets the keel brace at a compact shared mounting node.",
    )
    ctx.allow_overlap(
        "keel",
        "starboard_nacelle",
        elem_a="starboard_mount_brace",
        elem_b="mount_pylon",
        reason="The nacelle pylon meets the keel brace at a compact shared mounting node.",
    )
    ctx.allow_overlap(
        "port_nacelle",
        "port_propeller",
        elem_a="nacelle_shell",
        elem_b="shaft_stub",
        reason="The propeller shaft stub is intentionally simplified as entering the nacelle nose opening proxy.",
    )
    ctx.allow_overlap(
        "starboard_nacelle",
        "starboard_propeller",
        elem_a="nacelle_shell",
        elem_b="shaft_stub",
        reason="The propeller shaft stub is intentionally simplified as entering the nacelle nose opening proxy.",
    )
    ctx.allow_overlap(
        "envelope",
        "keel",
        elem_a="envelope_skin",
        elem_b="saddle_pad",
        reason="The keel saddle pad lightly embeds into the envelope skin as a simplified suspension saddle.",
    )
    ctx.allow_overlap(
        "envelope",
        "tailplane",
        elem_a="envelope_skin",
        elem_b="tail_root_pad",
        reason="The tail root pad is simplified as a compact attachment lug at the tail cone.",
    )
    ctx.allow_overlap(
        "keel",
        "gondola",
        elem_a="hanger_front_port",
        elem_b="roof_pad_front",
        reason="The front hanger is simplified as socketing into the gondola roof pad.",
    )
    ctx.allow_overlap(
        "keel",
        "gondola",
        elem_a="hanger_front_starboard",
        elem_b="roof_pad_front",
        reason="The front hanger is simplified as socketing into the gondola roof pad.",
    )
    ctx.allow_overlap(
        "keel",
        "gondola",
        elem_a="hanger_rear_port",
        elem_b="roof_pad_rear",
        reason="The rear hanger is simplified as socketing into the gondola roof pad.",
    )
    ctx.allow_overlap(
        "keel",
        "gondola",
        elem_a="hanger_rear_starboard",
        elem_b="roof_pad_rear",
        reason="The rear hanger is simplified as socketing into the gondola roof pad.",
    )
    ctx.allow_overlap(
        "gondola",
        "starboard_wheel",
        elem_a="axle_stub_starboard",
        elem_b="hub_boss",
        reason="The starboard wheel hub is modeled as a close bearing fit on the axle stub.",
    )

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="envelope_skin",
        negative_elem="cabin_shell",
        min_gap=0.60,
        max_gap=1.50,
        name="gondola cabin hangs distinctly below the envelope",
    )
    ctx.expect_gap(
        port_nacelle,
        envelope,
        axis="y",
        positive_elem="nacelle_shell",
        negative_elem="envelope_skin",
        min_gap=0.20,
        max_gap=1.10,
        name="port nacelle stays clearly outboard of the envelope",
    )
    ctx.expect_gap(
        envelope,
        starboard_nacelle,
        axis="y",
        positive_elem="envelope_skin",
        negative_elem="nacelle_shell",
        min_gap=0.20,
        max_gap=1.10,
        name="starboard nacelle stays clearly outboard of the envelope",
    )
    ctx.expect_contact(
        gondola,
        port_wheel,
        elem_a="axle_stub_port",
        elem_b="hub_boss",
        name="port wheel rides on a supported axle stub",
    )
    ctx.expect_contact(
        gondola,
        starboard_wheel,
        elem_a="axle_stub_starboard",
        elem_b="hub_boss",
        name="starboard wheel rides on a supported axle stub",
    )

    rudder_joint = object_model.get_articulation("tailplane_to_rudder")
    port_elevator_joint = object_model.get_articulation("tailplane_to_port_elevator")
    starboard_elevator_joint = object_model.get_articulation("tailplane_to_starboard_elevator")

    rudder_rest = elem_center("rudder", "rudder_surface")
    rudder_limits = rudder_joint.motion_limits
    if rudder_rest is not None and rudder_limits is not None and rudder_limits.upper is not None:
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            rudder_deflected = elem_center("rudder", "rudder_surface")
        ctx.check(
            "positive rudder input swings the blade sideways",
            rudder_deflected is not None and rudder_deflected[1] < rudder_rest[1] - 0.04,
            details=f"rest={rudder_rest}, deflected={rudder_deflected}",
        )

    port_rest = elem_center("port_elevator", "elevator_surface")
    starboard_rest = elem_center("starboard_elevator", "elevator_surface")
    port_limits = port_elevator_joint.motion_limits
    if (
        port_rest is not None
        and starboard_rest is not None
        and port_limits is not None
        and port_limits.upper is not None
    ):
        with ctx.pose({port_elevator_joint: port_limits.upper, starboard_elevator_joint: port_limits.upper}):
            port_up = elem_center("port_elevator", "elevator_surface")
            starboard_up = elem_center("starboard_elevator", "elevator_surface")
        ctx.check(
            "positive elevator input raises both elevator surfaces",
            port_up is not None
            and starboard_up is not None
            and port_up[2] > port_rest[2] + 0.03
            and starboard_up[2] > starboard_rest[2] + 0.03,
            details=(
                f"port_rest={port_rest}, port_up={port_up}, "
                f"starboard_rest={starboard_rest}, starboard_up={starboard_up}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
