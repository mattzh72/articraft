from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireGeometry,
    WheelGeometry,
    mesh_from_geometry,
)


def _envelope_mesh(name: str):
    profile = [
        (0.0, -13.9),
        (0.45, -13.3),
        (1.15, -12.6),
        (2.25, -11.3),
        (3.15, -8.8),
        (3.85, -4.8),
        (4.10, -0.4),
        (4.06, 3.6),
        (3.55, 8.0),
        (2.60, 11.4),
        (1.20, 13.2),
        (0.0, 14.0),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=80), name)


def _nacelle_pod_mesh(name: str):
    profile = [
        (0.0, -0.92),
        (0.10, -0.84),
        (0.20, -0.70),
        (0.30, -0.28),
        (0.33, 0.10),
        (0.31, 0.50),
        (0.22, 0.76),
        (0.10, 0.88),
        (0.0, 0.96),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=40), name)


def _vertical_fixed_surface_mesh(
    name: str,
    forward_chord: float,
    tip_forward: float,
    height: float,
    thickness: float,
):
    profile = [
        (0.0, 0.0),
        (forward_chord, 0.0),
        (tip_forward, height * 0.78),
        (0.18, height),
        (0.0, height * 0.92),
    ]
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _vertical_control_surface_mesh(
    name: str,
    aft_chord: float,
    tip_aft: float,
    height: float,
    thickness: float,
):
    profile = [
        (0.0, 0.0),
        (-aft_chord, 0.0),
        (-tip_aft, height * 0.78),
        (-0.22, height),
        (0.0, height * 0.92),
    ]
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _horizontal_fixed_surface_mesh(
    name: str,
    span_sign: float,
    forward_chord: float,
    tip_forward: float,
    span: float,
    thickness: float,
):
    tip_y = span_sign * span
    profile = [
        (0.0, 0.0),
        (forward_chord, 0.0),
        (tip_forward, tip_y * 0.72),
        (0.18, tip_y),
        (0.0, tip_y * 0.94),
    ]
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _horizontal_control_surface_mesh(
    name: str,
    span_sign: float,
    aft_chord: float,
    tip_aft: float,
    span: float,
    thickness: float,
):
    tip_y = span_sign * span
    profile = [
        (0.0, 0.0),
        (-aft_chord, 0.0),
        (-tip_aft, tip_y * 0.72),
        (-0.22, tip_y),
        (0.0, tip_y * 0.94),
    ]
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _register_materials(model: ArticulatedObject) -> None:
    model.material("envelope_white", rgba=(0.93, 0.94, 0.96, 1.0))
    model.material("sign_frame", rgba=(0.10, 0.11, 0.14, 1.0))
    model.material("sign_glow", rgba=(0.98, 0.84, 0.28, 1.0))
    model.material("gondola_gray", rgba=(0.43, 0.45, 0.48, 1.0))
    model.material("tail_red", rgba=(0.73, 0.10, 0.11, 1.0))
    model.material("metal_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("wheel_silver", rgba=(0.72, 0.73, 0.76, 1.0))
    model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("window_dark", rgba=(0.14, 0.17, 0.22, 1.0))


def _add_nacelle(
    model: ArticulatedObject,
    envelope,
    *,
    side_name: str,
    side_sign: float,
) -> None:
    nacelle = model.part(f"{side_name}_nacelle")
    nacelle.visual(
        Box((0.82, 0.18, 0.58)),
        origin=Origin(xyz=(0.0, side_sign * 0.20, 0.0)),
        material="metal_dark",
        name=f"{side_name}_mount_pad",
    )
    nacelle.visual(
        Box((0.28, 0.56, 0.14)),
        origin=Origin(xyz=(0.36, side_sign * 0.68, -0.16)),
        material="metal_dark",
        name=f"{side_name}_front_arm",
    )
    nacelle.visual(
        Box((0.28, 0.56, 0.14)),
        origin=Origin(xyz=(-0.36, side_sign * 0.68, -0.16)),
        material="metal_dark",
        name=f"{side_name}_rear_arm",
    )
    nacelle.visual(
        Box((0.18, 0.26, 0.12)),
        origin=Origin(xyz=(0.36, side_sign * 0.40, -0.12)),
        material="metal_dark",
        name=f"{side_name}_front_link",
    )
    nacelle.visual(
        Box((0.18, 0.26, 0.12)),
        origin=Origin(xyz=(-0.36, side_sign * 0.40, -0.12)),
        material="metal_dark",
        name=f"{side_name}_rear_link",
    )
    nacelle.visual(
        Box((1.08, 0.20, 0.22)),
        origin=Origin(xyz=(0.0, side_sign * 1.00, -0.22)),
        material="metal_dark",
        name=f"{side_name}_brace",
    )
    nacelle.visual(
        _nacelle_pod_mesh(f"{side_name}_pod_shell"),
        origin=Origin(xyz=(0.10, side_sign * 1.36, -0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="gondola_gray",
        name="pod_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.07, length=0.30),
        origin=Origin(xyz=(0.98, side_sign * 1.36, -0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="metal_dark",
        name="prop_shaft",
    )

    model.articulation(
        f"envelope_to_{side_name}_nacelle",
        ArticulationType.FIXED,
        parent=envelope,
        child=nacelle,
        origin=Origin(xyz=(1.10, side_sign * 3.41, -2.05)),
    )

    propeller = model.part(f"{side_name}_propeller")
    propeller.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.68,
                0.15,
                4,
                thickness=0.09,
                blade=FanRotorBlade(shape="scimitar"),
                hub=FanRotorHub(style="spinner", bore_diameter=0.05),
            ),
            f"{side_name}_propeller_rotor",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="prop_black",
        name="rotor",
    )

    model.articulation(
        f"{side_name}_nacelle_to_{side_name}_propeller",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=propeller,
        origin=Origin(xyz=(1.11, side_sign * 1.36, -0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=50.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_sign_blimp")
    _register_materials(model)

    envelope = model.part("envelope")
    envelope.visual(
        _envelope_mesh("envelope_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="envelope_white",
        name="envelope_shell",
    )

    for side_name, side_sign in (("port", 1.0), ("starboard", -1.0)):
        envelope.visual(
            Box((8.90, 0.34, 2.45)),
            origin=Origin(xyz=(0.85, side_sign * 3.96, 0.22)),
            material="sign_frame",
            name=f"{side_name}_sign_frame",
        )
        envelope.visual(
            Box((8.30, 0.16, 1.92)),
            origin=Origin(xyz=(0.95, side_sign * 4.06, 0.22)),
            material="sign_glow",
            name=f"{side_name}_sign_face",
        )
        envelope.visual(
            Box((0.88, 0.40, 0.74)),
            origin=Origin(xyz=(1.10, side_sign * 3.32, -2.05)),
            material="metal_dark",
            name=f"{side_name}_mount_boss",
        )

    envelope.visual(
        Box((1.24, 0.98, 0.32)),
        origin=Origin(xyz=(-1.20, 0.0, -3.89)),
        material="metal_dark",
        name="gondola_boss",
    )
    envelope.visual(
        Box((0.56, 0.38, 0.90)),
        origin=Origin(xyz=(-11.90, 0.0, 3.40)),
        material="metal_dark",
        name="top_tail_boss",
    )
    envelope.visual(
        Box((0.56, 0.38, 0.90)),
        origin=Origin(xyz=(-11.90, 0.0, -3.40)),
        material="metal_dark",
        name="bottom_tail_boss",
    )
    envelope.visual(
        Box((0.64, 0.90, 0.38)),
        origin=Origin(xyz=(-11.70, 3.25, 0.0)),
        material="metal_dark",
        name="port_tail_boss",
    )
    envelope.visual(
        Box((0.64, 0.90, 0.38)),
        origin=Origin(xyz=(-11.70, -3.25, 0.0)),
        material="metal_dark",
        name="starboard_tail_boss",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((1.10, 0.84, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
        material="metal_dark",
        name="roof_mount",
    )
    gondola.visual(
        Box((0.56, 0.52, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, -0.53)),
        material="metal_dark",
        name="center_pylon",
    )
    gondola.visual(
        Box((4.20, 1.58, 1.24)),
        origin=Origin(xyz=(0.20, 0.0, -1.18)),
        material="gondola_gray",
        name="cabin_shell",
    )
    gondola.visual(
        Sphere(radius=0.66),
        origin=Origin(xyz=(2.02, 0.0, -1.38)),
        material="gondola_gray",
        name="nose_fairing",
    )
    gondola.visual(
        Box((2.85, 1.12, 0.20)),
        origin=Origin(xyz=(0.02, 0.0, -0.80)),
        material="gondola_gray",
        name="roof_blister",
    )
    gondola.visual(
        Box((2.72, 1.62, 0.38)),
        origin=Origin(xyz=(0.60, 0.0, -1.03)),
        material="window_dark",
        name="window_band",
    )
    gondola.visual(
        Box((0.82, 0.46, 0.18)),
        origin=Origin(xyz=(0.20, 0.0, -1.86)),
        material="metal_dark",
        name="gear_boss",
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(-1.20, 0.0, -4.05)),
    )

    _add_nacelle(model, envelope, side_name="port", side_sign=1.0)
    _add_nacelle(model, envelope, side_name="starboard", side_sign=-1.0)

    landing_gear = model.part("landing_gear")
    landing_gear.visual(
        Box((0.78, 0.42, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material="metal_dark",
        name="gear_mount",
    )
    landing_gear.visual(
        Box((0.18, 0.22, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, -0.38)),
        material="metal_dark",
        name="center_strut",
    )
    landing_gear.visual(
        Cylinder(radius=0.038, length=1.18),
        origin=Origin(xyz=(0.0, 0.0, -0.60), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="metal_dark",
        name="axle",
    )

    model.articulation(
        "gondola_to_landing_gear",
        ArticulationType.FIXED,
        parent=gondola,
        child=landing_gear,
        origin=Origin(xyz=(0.20, 0.0, -1.95)),
    )

    for side_name, side_sign in (("port", 1.0), ("starboard", -1.0)):
        wheel = model.part(f"{side_name}_wheel")
        wheel.visual(
            mesh_from_geometry(TireGeometry(0.24, 0.08, inner_radius=0.18), f"{side_name}_tire"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material="tire_black",
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(WheelGeometry(0.18, 0.06), f"{side_name}_wheel_rim"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material="wheel_silver",
            name="rim",
        )
        model.articulation(
            f"landing_gear_to_{side_name}_wheel",
            ArticulationType.CONTINUOUS,
            parent=landing_gear,
            child=wheel,
            origin=Origin(xyz=(0.0, side_sign * 0.59, -0.60)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=30.0),
        )

    top_fin = model.part("top_fin")
    top_fin.visual(
        Box((0.60, 0.28, 0.84)),
        origin=Origin(xyz=(0.82, 0.0, 0.42)),
        material="metal_dark",
        name="mount_pad",
    )
    top_fin.visual(
        _vertical_fixed_surface_mesh(
            "top_fin_panel",
            forward_chord=1.00,
            tip_forward=0.38,
            height=4.65,
            thickness=0.16,
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tail_red",
        name="fin_panel",
    )
    model.articulation(
        "envelope_to_top_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=top_fin,
        origin=Origin(xyz=(-12.74, 0.0, 3.85)),
    )

    top_rudder = model.part("top_rudder")
    top_rudder.visual(
        _vertical_control_surface_mesh(
            "top_rudder_panel",
            aft_chord=1.08,
            tip_aft=0.92,
            height=3.95,
            thickness=0.10,
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tail_red",
        name="surface",
    )
    model.articulation(
        "top_fin_to_top_rudder",
        ArticulationType.REVOLUTE,
        parent=top_fin,
        child=top_rudder,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=20.0, velocity=1.6),
    )

    bottom_fin = model.part("bottom_fin")
    bottom_fin.visual(
        Box((0.60, 0.28, 0.84)),
        origin=Origin(xyz=(0.82, 0.0, -0.42)),
        material="metal_dark",
        name="mount_pad",
    )
    bottom_fin.visual(
        _vertical_fixed_surface_mesh(
            "bottom_fin_panel",
            forward_chord=0.96,
            tip_forward=0.34,
            height=4.20,
            thickness=0.16,
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="tail_red",
        name="fin_panel",
    )
    model.articulation(
        "envelope_to_bottom_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=bottom_fin,
        origin=Origin(xyz=(-12.68, 0.0, -3.85)),
    )

    bottom_rudder = model.part("bottom_rudder")
    bottom_rudder.visual(
        _vertical_control_surface_mesh(
            "bottom_rudder_panel",
            aft_chord=1.00,
            tip_aft=0.84,
            height=3.45,
            thickness=0.10,
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="tail_red",
        name="surface",
    )
    model.articulation(
        "bottom_fin_to_bottom_rudder",
        ArticulationType.REVOLUTE,
        parent=bottom_fin,
        child=bottom_rudder,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=20.0, velocity=1.6),
    )

    port_stabilizer = model.part("port_stabilizer")
    port_stabilizer.visual(
        Box((0.60, 0.78, 0.22)),
        origin=Origin(xyz=(0.86, 0.39, 0.0)),
        material="metal_dark",
        name="mount_pad",
    )
    port_stabilizer.visual(
        _horizontal_fixed_surface_mesh(
            "port_stabilizer_panel",
            span_sign=1.0,
            forward_chord=0.90,
            tip_forward=0.26,
            span=4.15,
            thickness=0.15,
        ),
        material="tail_red",
        name="stabilizer_panel",
    )
    port_stabilizer.visual(
        _vertical_fixed_surface_mesh(
            "port_end_fin",
            forward_chord=0.78,
            tip_forward=0.20,
            height=2.10,
            thickness=0.12,
        ),
        origin=Origin(xyz=(-0.62, 4.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tail_red",
        name="end_fin",
    )
    model.articulation(
        "envelope_to_port_stabilizer",
        ArticulationType.FIXED,
        parent=envelope,
        child=port_stabilizer,
        origin=Origin(xyz=(-12.56, 3.70, 0.0)),
    )

    port_elevator = model.part("port_elevator")
    port_elevator.visual(
        _horizontal_control_surface_mesh(
            "port_elevator_panel",
            span_sign=1.0,
            aft_chord=1.02,
            tip_aft=0.78,
            span=3.72,
            thickness=0.10,
        ),
        material="tail_red",
        name="surface",
    )
    model.articulation(
        "port_stabilizer_to_port_elevator",
        ArticulationType.REVOLUTE,
        parent=port_stabilizer,
        child=port_elevator,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.45, effort=16.0, velocity=1.5),
    )

    starboard_stabilizer = model.part("starboard_stabilizer")
    starboard_stabilizer.visual(
        Box((0.60, 0.78, 0.22)),
        origin=Origin(xyz=(0.86, -0.39, 0.0)),
        material="metal_dark",
        name="mount_pad",
    )
    starboard_stabilizer.visual(
        _horizontal_fixed_surface_mesh(
            "starboard_stabilizer_panel",
            span_sign=-1.0,
            forward_chord=0.90,
            tip_forward=0.26,
            span=4.15,
            thickness=0.15,
        ),
        material="tail_red",
        name="stabilizer_panel",
    )
    starboard_stabilizer.visual(
        _vertical_fixed_surface_mesh(
            "starboard_end_fin",
            forward_chord=0.78,
            tip_forward=0.20,
            height=2.10,
            thickness=0.12,
        ),
        origin=Origin(xyz=(-0.62, -4.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="tail_red",
        name="end_fin",
    )
    model.articulation(
        "envelope_to_starboard_stabilizer",
        ArticulationType.FIXED,
        parent=envelope,
        child=starboard_stabilizer,
        origin=Origin(xyz=(-12.56, -3.70, 0.0)),
    )

    starboard_elevator = model.part("starboard_elevator")
    starboard_elevator.visual(
        _horizontal_control_surface_mesh(
            "starboard_elevator_panel",
            span_sign=-1.0,
            aft_chord=1.02,
            tip_aft=0.78,
            span=3.72,
            thickness=0.10,
        ),
        material="tail_red",
        name="surface",
    )
    model.articulation(
        "starboard_stabilizer_to_starboard_elevator",
        ArticulationType.REVOLUTE,
        parent=starboard_stabilizer,
        child=starboard_elevator,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=0.45, effort=16.0, velocity=1.5),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    port_nacelle = object_model.get_part("port_nacelle")
    starboard_nacelle = object_model.get_part("starboard_nacelle")
    landing_gear = object_model.get_part("landing_gear")
    port_wheel = object_model.get_part("port_wheel")
    starboard_wheel = object_model.get_part("starboard_wheel")
    top_rudder = object_model.get_part("top_rudder")
    port_elevator = object_model.get_part("port_elevator")

    top_rudder_joint = object_model.get_articulation("top_fin_to_top_rudder")
    port_elevator_joint = object_model.get_articulation("port_stabilizer_to_port_elevator")

    ctx.allow_overlap(
        "port_nacelle",
        "port_propeller",
        elem_a="prop_shaft",
        elem_b="rotor",
        reason="The spinning propeller hub is intentionally centered on the nacelle shaft.",
    )
    ctx.allow_overlap(
        "starboard_nacelle",
        "starboard_propeller",
        elem_a="prop_shaft",
        elem_b="rotor",
        reason="The spinning propeller hub is intentionally centered on the nacelle shaft.",
    )
    ctx.allow_overlap(
        "landing_gear",
        "port_wheel",
        elem_a="axle",
        elem_b="rim",
        reason="The landing wheel rotates around the fixed axle through its hub.",
    )
    ctx.allow_overlap(
        "landing_gear",
        "starboard_wheel",
        elem_a="axle",
        elem_b="rim",
        reason="The landing wheel rotates around the fixed axle through its hub.",
    )
    ctx.allow_overlap(
        "envelope",
        "port_nacelle",
        elem_a="envelope_shell",
        elem_b="port_mount_pad",
        reason="The nacelle mount pad is simplified as a flat saddle bracket bearing against the curved hull shell.",
    )
    ctx.allow_overlap(
        "envelope",
        "starboard_nacelle",
        elem_a="envelope_shell",
        elem_b="starboard_mount_pad",
        reason="The nacelle mount pad is simplified as a flat saddle bracket bearing against the curved hull shell.",
    )
    ctx.allow_overlap(
        "envelope",
        "gondola",
        elem_a="envelope_shell",
        elem_b="roof_mount",
        reason="The gondola roof saddle is simplified as a flat mount plate bearing against the curved hull shell.",
    )

    ctx.expect_contact(
        gondola,
        envelope,
        elem_a="roof_mount",
        elem_b="gondola_boss",
        contact_tol=0.003,
        name="gondola roof mount meets hull boss",
    )
    ctx.expect_contact(
        port_nacelle,
        envelope,
        elem_a="port_mount_pad",
        elem_b="port_mount_boss",
        contact_tol=0.003,
        name="port nacelle bracket lands on hull boss",
    )
    ctx.expect_contact(
        starboard_nacelle,
        envelope,
        elem_a="starboard_mount_pad",
        elem_b="starboard_mount_boss",
        contact_tol=0.003,
        name="starboard nacelle bracket lands on hull boss",
    )
    ctx.expect_contact(
        landing_gear,
        gondola,
        elem_a="gear_mount",
        elem_b="gear_boss",
        contact_tol=0.003,
        name="landing gear is bolted to gondola boss",
    )

    ctx.expect_origin_gap(
        envelope,
        gondola,
        axis="z",
        min_gap=2.4,
        name="gondola hangs well below the hull centerline",
    )
    ctx.expect_overlap(
        port_nacelle,
        envelope,
        axes="xz",
        min_overlap=0.45,
        name="port nacelle stays aligned to the hull side",
    )
    ctx.expect_overlap(
        starboard_nacelle,
        envelope,
        axes="xz",
        min_overlap=0.45,
        name="starboard nacelle stays aligned to the hull side",
    )
    ctx.expect_origin_gap(
        gondola,
        port_wheel,
        axis="z",
        min_gap=1.6,
        name="port wheel sits below the gondola",
    )
    ctx.expect_origin_gap(
        gondola,
        starboard_wheel,
        axis="z",
        min_gap=1.6,
        name="starboard wheel sits below the gondola",
    )

    rest_rudder_center = _aabb_center(ctx.part_world_aabb(top_rudder))
    with ctx.pose({top_rudder_joint: 0.40}):
        turned_rudder_center = _aabb_center(ctx.part_world_aabb(top_rudder))
    ctx.check(
        "top rudder swings sideways",
        rest_rudder_center is not None
        and turned_rudder_center is not None
        and turned_rudder_center[1] > rest_rudder_center[1] + 0.12,
        details=f"rest={rest_rudder_center}, turned={turned_rudder_center}",
    )

    rest_elevator_center = _aabb_center(ctx.part_world_aabb(port_elevator))
    with ctx.pose({port_elevator_joint: 0.35}):
        raised_elevator_center = _aabb_center(ctx.part_world_aabb(port_elevator))
    ctx.check(
        "port elevator raises on positive deflection",
        rest_elevator_center is not None
        and raised_elevator_center is not None
        and raised_elevator_center[2] > rest_elevator_center[2] + 0.10,
        details=f"rest={rest_elevator_center}, raised={raised_elevator_center}",
    )

    return ctx.report()


object_model = build_object_model()
