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
    MotionLimits,
    Origin,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    section_loft,
    TestContext,
    TestReport,
)
import cadquery as cq


def _ellipse_section(
    x: float,
    radius_y: float,
    radius_z: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius_y * math.cos(2.0 * math.pi * i / segments),
            radius_z * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _panel_mesh(
    name: str,
    profile: list[tuple[float, float]],
    thickness: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True),
        name,
    )


def _mirror_profile_y(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for x, y in reversed(profile)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sightseeing_blimp")

    envelope_mat = model.material("pearl_envelope", rgba=(0.92, 0.90, 0.80, 1.0))
    cabin_mat = model.material("warm_white_cabin", rgba=(0.86, 0.88, 0.86, 1.0))
    window_mat = model.material("blue_tinted_glass", rgba=(0.08, 0.22, 0.36, 0.82))
    fin_mat = model.material("navy_tail_fins", rgba=(0.02, 0.08, 0.18, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.58, 0.61, 0.64, 1.0))
    dark_mat = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    prop_mat = model.material("black_composite", rgba=(0.025, 0.025, 0.030, 1.0))

    airship = model.part("airship")

    hull_sections = [
        _ellipse_section(-21.0, 0.18, 0.16),
        _ellipse_section(-19.0, 1.25, 1.10),
        _ellipse_section(-16.0, 3.10, 2.75),
        _ellipse_section(-9.0, 5.25, 4.75),
        _ellipse_section(3.0, 5.65, 5.05),
        _ellipse_section(13.5, 4.30, 3.95),
        _ellipse_section(19.0, 1.45, 1.25),
        _ellipse_section(21.0, 0.16, 0.14),
    ]
    airship.visual(
        mesh_from_geometry(section_loft(hull_sections), "streamlined_envelope"),
        material=envelope_mat,
        name="envelope",
    )

    # Long passenger gondola tucked into the lower hull.
    airship.visual(
        mesh_from_geometry(CapsuleGeometry(1.32, 12.4, radial_segments=32), "passenger_cabin"),
        origin=Origin(xyz=(-1.0, 0.0, -6.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cabin_mat,
        name="cabin_shell",
    )
    airship.visual(
        Box((15.0, 2.15, 0.30)),
        origin=Origin(xyz=(-1.0, 0.0, -6.98)),
        material=metal_mat,
        name="cabin_keel",
    )

    for side in (-1.0, 1.0):
        for i, x in enumerate([-6.3, -5.1, -3.9, -2.7, -1.5, -0.3, 0.9, 2.1, 3.3]):
            airship.visual(
                Box((0.72, 0.035, 0.42)),
                origin=Origin(xyz=(x, side * 1.335, -5.86)),
                material=window_mat,
                name=f"window_{'a' if side < 0 else 'b'}_{i}",
            )
    airship.visual(
        Box((0.12, 0.92, 0.42)),
        origin=Origin(xyz=(6.34, 0.0, -5.88)),
        material=window_mat,
        name="front_windshield",
    )

    # Side-mounted propulsion pods and their struts.
    for idx, side in enumerate((-1.0, 1.0)):
        pod_y = side * 2.62
        pod_z = -5.70
        airship.visual(
            mesh_from_geometry(CapsuleGeometry(0.56, 1.35, radial_segments=28), f"prop_pod_{idx}"),
            origin=Origin(xyz=(0.0, pod_y, pod_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"prop_pod_{idx}",
        )
        for x in (-0.62, 0.62):
            airship.visual(
                Box((0.18, 1.45, 0.16)),
                origin=Origin(xyz=(x, side * 1.90, pod_z + 0.12)),
                material=metal_mat,
                name=f"pod_strut_{idx}_{'front' if x > 0 else 'rear'}",
            )
        airship.visual(
            Cylinder(radius=0.075, length=0.52),
            origin=Origin(xyz=(1.19, pod_y, pod_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"prop_shaft_{idx}",
        )

    # Cruciform tail: fixed vertical and horizontal fin roots at the aft hull.
    top_fin = [(0.0, 0.0), (-2.55, 0.20), (-2.18, 2.50), (-0.20, 1.72)]
    airship.visual(
        _panel_mesh("top_fin", top_fin, 0.18),
        origin=Origin(xyz=(-16.55, 0.0, 2.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fin_mat,
        name="top_fin",
    )
    airship.visual(
        _panel_mesh("bottom_fin", _mirror_profile_y(top_fin), 0.18),
        origin=Origin(xyz=(-16.55, 0.0, -2.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fin_mat,
        name="bottom_fin",
    )
    side_fin = [(0.0, 0.0), (-2.65, 0.12), (-2.05, 2.25), (-0.18, 1.58)]
    for idx, side in enumerate((-1.0, 1.0)):
        profile = _mirror_profile_y(side_fin) if side < 0 else side_fin
        airship.visual(
            _panel_mesh(f"side_fin_{idx}", profile, 0.16),
            origin=Origin(xyz=(-16.65, side * 2.65, 0.0)),
            material=fin_mat,
            name=f"side_fin_{idx}",
        )

    # Rear mooring wheel fork under the tail of the gondola.
    wheel_x = -8.35
    wheel_z = -7.62
    airship.visual(
        Box((0.52, 0.72, 0.16)),
        origin=Origin(xyz=(wheel_x, 0.0, -7.12)),
        material=metal_mat,
        name="wheel_fork_bridge",
    )
    for side in (-1.0, 1.0):
        airship.visual(
            Box((0.18, 0.10, 0.92)),
            origin=Origin(xyz=(wheel_x, side * 0.35, -7.46)),
            material=metal_mat,
            name=f"wheel_fork_leg_{'a' if side < 0 else 'b'}",
        )
    airship.visual(
        Cylinder(radius=0.053, length=0.82),
        origin=Origin(xyz=(wheel_x, 0.0, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="wheel_axle",
    )

    # Continuous rotors in the side pods.
    rotor_meshes = []
    for idx in range(2):
        rotor_meshes.append(
            mesh_from_geometry(
                FanRotorGeometry(
                    0.74,
                    0.18,
                    5,
                    thickness=0.15,
                    blade_pitch_deg=33.0,
                    blade_sweep_deg=28.0,
                    blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18),
                    hub=FanRotorHub(style="spinner", bore_diameter=0.05),
                ),
                f"propeller_rotor_{idx}",
            )
        )

    for idx, side in enumerate((-1.0, 1.0)):
        prop = model.part(f"propeller_{idx}")
        prop.visual(
            rotor_meshes[idx],
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_mat,
            name="rotor",
        )
        model.articulation(
            f"propeller_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=airship,
            child=prop,
            origin=Origin(xyz=(1.48, side * 2.62, -5.70)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=45.0),
        )

    # Hinged rudder and all-moving elevator panels.
    rudder = model.part("rudder")
    rudder_profile = [(0.0, -1.15), (-1.85, -0.95), (-1.72, 1.05), (0.0, 1.20)]
    rudder.visual(
        _panel_mesh("rudder_panel", rudder_profile, 0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fin_mat,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.055, length=2.55),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal_mat,
        name="rudder_hinge_pin",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=airship,
        child=rudder,
        origin=Origin(xyz=(-19.12, 0.0, 3.54)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.45, effort=12.0, velocity=1.4),
    )

    elevator_profile = [(0.0, 0.0), (-2.05, 0.10), (-1.72, 1.62), (0.0, 1.25)]
    for idx, side in enumerate((-1.0, 1.0)):
        elevator = model.part(f"elevator_{idx}")
        profile = _mirror_profile_y(elevator_profile) if side < 0 else elevator_profile
        elevator.visual(
            _panel_mesh(f"elevator_panel_{idx}", profile, 0.15),
            material=fin_mat,
            name="elevator_panel",
        )
        model.articulation(
            f"elevator_{idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=airship,
            child=elevator,
            origin=Origin(xyz=(-19.30, side * 2.76, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=10.0, velocity=1.3),
        )

    # Small steerable/rolling mooring wheel captured by the fork.
    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.36,
                0.18,
                inner_radius=0.245,
                carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
                tread=TireTread(style="ribbed", depth=0.018, count=22, land_ratio=0.60),
                shoulder=TireShoulder(width=0.018, radius=0.007),
            ),
            "mooring_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_mat,
        name="tire",
    )
    mooring_wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.245,
                0.16,
                rim=WheelRim(inner_radius=0.13, flange_height=0.012, flange_thickness=0.006),
                hub=WheelHub(radius=0.085, width=0.13, cap_style="domed"),
                face=WheelFace(dish_depth=0.018, front_inset=0.006, rear_inset=0.004),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.012, window_radius=0.035),
                bore=WheelBore(style="round", diameter=0.105),
            ),
            "mooring_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal_mat,
        name="rim",
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=airship,
        child=mooring_wheel,
        origin=Origin(xyz=(wheel_x, 0.0, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airship = object_model.get_part("airship")
    mooring_wheel = object_model.get_part("mooring_wheel")
    rudder = object_model.get_part("rudder")

    for idx in (0, 1):
        propeller = object_model.get_part(f"propeller_{idx}")
        ctx.allow_overlap(
            airship,
            propeller,
            elem_a=f"prop_shaft_{idx}",
            elem_b="rotor",
            reason="The propeller hub is intentionally captured on the fixed motor shaft.",
        )
        ctx.expect_overlap(
            airship,
            propeller,
            axes="x",
            elem_a=f"prop_shaft_{idx}",
            elem_b="rotor",
            min_overlap=0.020,
            name=f"propeller {idx} is retained on its shaft",
        )

    ctx.allow_overlap(
        airship,
        rudder,
        elem_a="top_fin",
        elem_b="rudder_hinge_pin",
        reason="The rudder hinge pin is seated in the trailing hinge knuckle of the vertical fin.",
    )
    ctx.expect_overlap(
        airship,
        rudder,
        axes="z",
        elem_a="top_fin",
        elem_b="rudder_hinge_pin",
        min_overlap=1.50,
        name="rudder hinge pin spans the top fin knuckle",
    )

    ctx.allow_overlap(
        airship,
        mooring_wheel,
        elem_a="wheel_axle",
        elem_b="rim",
        reason="The mooring wheel rim is captured on the small fixed axle through the support fork.",
    )
    ctx.expect_overlap(
        airship,
        mooring_wheel,
        axes="y",
        elem_a="wheel_axle",
        elem_b="rim",
        min_overlap=0.10,
        name="mooring wheel is retained across the fork axle",
    )

    for name in ("propeller_0_spin", "propeller_1_spin", "wheel_spin"):
        joint = object_model.get_articulation(name)
        ctx.check(
            f"{name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{name} type={joint.articulation_type}",
        )

    for name in ("rudder_hinge", "elevator_0_hinge", "elevator_1_hinge"):
        joint = object_model.get_articulation(name)
        limits = joint.motion_limits
        ctx.check(
            f"{name} has bounded travel",
            limits is not None and limits.lower is not None and limits.upper is not None,
            details=f"{name} limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
