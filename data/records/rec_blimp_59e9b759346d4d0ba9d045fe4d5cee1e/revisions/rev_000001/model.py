from __future__ import annotations

from math import pi

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
    TestContext,
    TestReport,
    TireGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _envelope_geometry(length: float, radius: float):
    profile = [
        (0.00, -0.50 * length),
        (0.22 * radius, -0.47 * length),
        (0.54 * radius, -0.42 * length),
        (0.82 * radius, -0.28 * length),
        (0.98 * radius, -0.10 * length),
        (1.00 * radius, 0.08 * length),
        (0.95 * radius, 0.28 * length),
        (0.76 * radius, 0.41 * length),
        (0.40 * radius, 0.48 * length),
        (0.00, 0.50 * length),
    ]
    return LatheGeometry(profile, segments=96).rotate_y(pi / 2.0)


def _pod_shell_geometry(length: float, radius: float):
    profile = [
        (0.00, -0.50 * length),
        (0.24 * radius, -0.46 * length),
        (0.86 * radius, -0.22 * length),
        (1.00 * radius, 0.00),
        (0.82 * radius, 0.22 * length),
        (0.32 * radius, 0.44 * length),
        (0.00, 0.50 * length),
    ]
    return LatheGeometry(profile, segments=64).rotate_y(pi / 2.0)


def _surface_profile(forward_chord: float, tip_forward_chord: float, span: float) -> list[tuple[float, float]]:
    if span >= 0.0:
        return [(0.0, 0.0), (forward_chord, 0.0), (tip_forward_chord, span), (0.0, span)]
    return [(0.0, 0.0), (0.0, span), (tip_forward_chord, span), (forward_chord, 0.0)]


def _control_profile(aft_chord: float, tip_aft_chord: float, span: float) -> list[tuple[float, float]]:
    if span >= 0.0:
        return [(0.0, 0.0), (0.0, span), (-tip_aft_chord, span), (-aft_chord, 0.0)]
    return [(0.0, 0.0), (-aft_chord, 0.0), (-tip_aft_chord, span), (0.0, span)]


def _horizontal_surface_mesh(forward_chord: float, tip_forward_chord: float, span: float, thickness: float):
    return ExtrudeGeometry(_surface_profile(forward_chord, tip_forward_chord, span), thickness, center=True)


def _horizontal_control_mesh(aft_chord: float, tip_aft_chord: float, span: float, thickness: float):
    return ExtrudeGeometry(_control_profile(aft_chord, tip_aft_chord, span), thickness, center=True)


def _vertical_surface_mesh(forward_chord: float, tip_forward_chord: float, span: float, thickness: float):
    return (
        ExtrudeGeometry(_surface_profile(forward_chord, tip_forward_chord, span), thickness, center=True)
        .rotate_x(pi / 2.0)
    )


def _vertical_control_mesh(aft_chord: float, tip_aft_chord: float, span: float, thickness: float):
    return (
        ExtrudeGeometry(_control_profile(aft_chord, tip_aft_chord, span), thickness, center=True)
        .rotate_x(pi / 2.0)
    )


def _strut_mesh(start: tuple[float, float, float], end: tuple[float, float, float], radius: float):
    return tube_from_spline_points(
        [start, end],
        radius=radius,
        samples_per_segment=2,
        radial_segments=16,
        cap_ends=True,
    )


def _add_wheel_visuals(part, tire_mesh, *, wheel_material, tire_material) -> None:
    spin_origin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(tire_mesh, origin=spin_origin, material=tire_material, name="tire")
    part.visual(
        Cylinder(radius=0.19, length=0.095),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.055, length=0.118),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communications_blimp")

    envelope_skin = model.material("envelope_skin", rgba=(0.84, 0.86, 0.88, 1.0))
    gondola_paint = model.material("gondola_paint", rgba=(0.80, 0.82, 0.84, 1.0))
    structure_gray = model.material("structure_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    prop_black = model.material("prop_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.64, 0.66, 0.70, 1.0))

    wheel_tire_mesh = _save_mesh("blimp_wheel_tire", TireGeometry(0.29, 0.12, inner_radius=0.20))
    propeller_mesh = _save_mesh(
        "blimp_propeller",
        FanRotorGeometry(
            1.05,
            0.18,
            3,
            thickness=0.12,
            blade_pitch_deg=24.0,
            blade_sweep_deg=14.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.10),
            hub=FanRotorHub(style="spinner"),
        ).rotate_y(pi / 2.0),
    )

    hull = model.part("hull")
    hull.visual(
        _save_mesh("blimp_envelope", _envelope_geometry(32.0, 4.0)),
        material=envelope_skin,
        name="envelope",
    )
    hull.visual(
        Box((6.4, 0.72, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, -4.25)),
        material=structure_gray,
        name="keel_beam",
    )
    hull.visual(
        Box((0.95, 0.34, 0.24)),
        origin=Origin(xyz=(1.55, 0.0, -4.92)),
        material=structure_gray,
        name="front_mount_pad",
    )
    hull.visual(
        Box((0.95, 0.34, 0.24)),
        origin=Origin(xyz=(-1.55, 0.0, -4.92)),
        material=structure_gray,
        name="rear_mount_pad",
    )
    hull.visual(
        Box((0.26, 0.26, 0.30)),
        origin=Origin(xyz=(1.55, 0.0, -4.65)),
        material=structure_gray,
        name="front_mount_post",
    )
    hull.visual(
        Box((0.26, 0.26, 0.30)),
        origin=Origin(xyz=(-1.55, 0.0, -4.65)),
        material=structure_gray,
        name="rear_mount_post",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((6.20, 2.20, 1.80)),
        origin=Origin(xyz=(0.0, 0.0, -0.98)),
        material=gondola_paint,
        name="equipment_body",
    )
    gondola.visual(
        Box((6.40, 2.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=structure_gray,
        name="roof_deck",
    )
    gondola.visual(
        Box((4.60, 0.22, 0.22)),
        origin=Origin(xyz=(0.0, 1.11, -0.72)),
        material=dark_trim,
        name="window_band",
    )
    gondola.visual(
        Box((0.90, 0.24, 0.08)),
        origin=Origin(xyz=(1.55, 0.0, 0.52)),
        material=structure_gray,
        name="front_mount_cap",
    )
    gondola.visual(
        Box((0.90, 0.24, 0.08)),
        origin=Origin(xyz=(-1.55, 0.0, 0.52)),
        material=structure_gray,
        name="rear_mount_cap",
    )
    gondola.visual(
        _save_mesh(
            "gondola_front_left_strut",
            _strut_mesh((1.05, 0.72, 0.0), (1.55, 0.08, 0.48), radius=0.05),
        ),
        material=structure_gray,
        name="front_left_strut",
    )
    gondola.visual(
        _save_mesh(
            "gondola_front_right_strut",
            _strut_mesh((1.05, -0.72, 0.0), (1.55, -0.08, 0.48), radius=0.05),
        ),
        material=structure_gray,
        name="front_right_strut",
    )
    gondola.visual(
        _save_mesh(
            "gondola_rear_left_strut",
            _strut_mesh((-1.05, 0.72, 0.0), (-1.55, 0.08, 0.48), radius=0.05),
        ),
        material=structure_gray,
        name="rear_left_strut",
    )
    gondola.visual(
        _save_mesh(
            "gondola_rear_right_strut",
            _strut_mesh((-1.05, -0.72, 0.0), (-1.55, -0.08, 0.48), radius=0.05),
        ),
        material=structure_gray,
        name="rear_right_strut",
    )
    gondola.visual(
        Box((0.92, 0.36, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -1.86)),
        material=structure_gray,
        name="gear_socket",
    )

    gear = model.part("gear_frame")
    gear.visual(
        Box((0.86, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=structure_gray,
        name="gear_mount_plate",
    )
    gear.visual(
        Cylinder(radius=0.045, length=1.711),
        origin=Origin(xyz=(0.0, 0.0, -0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=structure_gray,
        name="main_axle_bar",
    )
    for name, start, end in [
        ("front_left_gear_strut", (0.32, 0.12, -0.08), (0.0, 0.80, -0.72)),
        ("rear_left_gear_strut", (-0.32, 0.12, -0.08), (0.0, 0.80, -0.72)),
        ("front_right_gear_strut", (0.32, -0.12, -0.08), (0.0, -0.80, -0.72)),
        ("rear_right_gear_strut", (-0.32, -0.12, -0.08), (0.0, -0.80, -0.72)),
    ]:
        gear.visual(
            _save_mesh(name, _strut_mesh(start, end, radius=0.04)),
            material=structure_gray,
            name=name,
        )

    main_wheel_0 = model.part("main_wheel_0")
    _add_wheel_visuals(
        main_wheel_0,
        wheel_tire_mesh,
        wheel_material=wheel_metal,
        tire_material=rubber,
    )

    main_wheel_1 = model.part("main_wheel_1")
    _add_wheel_visuals(
        main_wheel_1,
        wheel_tire_mesh,
        wheel_material=wheel_metal,
        tire_material=rubber,
    )

    left_engine_pod = model.part("left_engine_pod")
    left_engine_pod.visual(
        _save_mesh("left_engine_pod_shell", _pod_shell_geometry(2.50, 0.34)),
        origin=Origin(xyz=(-0.95, 0.0, 0.0)),
        material=dark_trim,
        name="pod_shell",
    )
    left_engine_pod.visual(
        Box((0.72, 1.20, 0.16)),
        origin=Origin(xyz=(-0.88, -0.55, 0.02)),
        material=structure_gray,
        name="pylon_root",
    )
    left_engine_pod.visual(
        _save_mesh(
            "left_engine_pod_brace",
            _strut_mesh((-0.35, -1.05, 0.12), (-0.92, -0.18, -0.08), radius=0.04),
        ),
        material=structure_gray,
        name="pylon_brace",
    )
    left_engine_pod.visual(
        Cylinder(radius=0.04, length=0.18),
        origin=Origin(xyz=(0.39, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=structure_gray,
        name="prop_shaft",
    )

    right_engine_pod = model.part("right_engine_pod")
    right_engine_pod.visual(
        _save_mesh("right_engine_pod_shell", _pod_shell_geometry(2.50, 0.34)),
        origin=Origin(xyz=(-0.95, 0.0, 0.0)),
        material=dark_trim,
        name="pod_shell",
    )
    right_engine_pod.visual(
        Box((0.72, 1.20, 0.16)),
        origin=Origin(xyz=(-0.88, 0.55, 0.02)),
        material=structure_gray,
        name="pylon_root",
    )
    right_engine_pod.visual(
        _save_mesh(
            "right_engine_pod_brace",
            _strut_mesh((-0.35, 1.05, 0.12), (-0.92, 0.18, -0.08), radius=0.04),
        ),
        material=structure_gray,
        name="pylon_brace",
    )
    right_engine_pod.visual(
        Cylinder(radius=0.04, length=0.18),
        origin=Origin(xyz=(0.39, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=structure_gray,
        name="prop_shaft",
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(propeller_mesh, material=prop_black, name="propeller")

    right_propeller = model.part("right_propeller")
    right_propeller.visual(propeller_mesh, material=prop_black, name="propeller")

    hull.visual(
        _save_mesh("top_fin_fixed", _vertical_surface_mesh(2.20, 1.25, 3.10, 0.12)),
        origin=Origin(xyz=(-13.55, 0.0, 2.28)),
        material=envelope_skin,
        name="top_fin",
    )

    rudder = model.part("rudder")
    rudder.visual(
        _save_mesh("rudder_surface", _vertical_control_mesh(0.95, 0.55, 2.55, 0.10)),
        material=envelope_skin,
        name="rudder",
    )

    hull.visual(
        _save_mesh("bottom_fin_surface", _vertical_surface_mesh(1.85, 0.95, -2.45, 0.10)),
        origin=Origin(xyz=(-13.35, 0.0, -2.22)),
        material=envelope_skin,
        name="bottom_fin",
    )

    hull.visual(
        _save_mesh("left_tailplane_surface", _horizontal_surface_mesh(2.05, 1.05, 3.45, 0.10)),
        origin=Origin(xyz=(-13.75, 2.26, 0.02)),
        material=envelope_skin,
        name="left_tailplane",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        _save_mesh("left_elevator_surface", _horizontal_control_mesh(0.90, 0.52, 3.05, 0.09)),
        material=envelope_skin,
        name="elevator",
    )

    hull.visual(
        _save_mesh("right_tailplane_surface", _horizontal_surface_mesh(2.05, 1.05, -3.45, 0.10)),
        origin=Origin(xyz=(-13.75, -2.26, 0.02)),
        material=envelope_skin,
        name="right_tailplane",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        _save_mesh("right_elevator_surface", _horizontal_control_mesh(0.90, 0.52, -3.05, 0.09)),
        material=envelope_skin,
        name="elevator",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -5.60)),
    )
    model.articulation(
        "gondola_to_gear",
        ArticulationType.FIXED,
        parent=gondola,
        child=gear,
        origin=Origin(xyz=(0.0, 0.0, -1.90)),
    )
    model.articulation(
        "gondola_to_left_engine_pod",
        ArticulationType.FIXED,
        parent=gondola,
        child=left_engine_pod,
        origin=Origin(xyz=(1.55, 2.25, -0.55)),
    )
    model.articulation(
        "gondola_to_right_engine_pod",
        ArticulationType.FIXED,
        parent=gondola,
        child=right_engine_pod,
        origin=Origin(xyz=(1.55, -2.25, -0.55)),
    )
    model.articulation(
        "left_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=left_engine_pod,
        child=left_propeller,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=60.0),
    )
    model.articulation(
        "right_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=right_engine_pod,
        child=right_propeller,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=60.0),
    )
    model.articulation(
        "main_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=gear,
        child=main_wheel_0,
        origin=Origin(xyz=(0.0, 0.905, -0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "main_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=gear,
        child=main_wheel_1,
        origin=Origin(xyz=(0.0, -0.905, -0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-13.55, 0.0, 2.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "hull_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(-13.75, 2.26, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-0.50, upper=0.50),
    )
    model.articulation(
        "hull_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-13.75, -2.26, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-0.50, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _aabb_center(part):
        box = ctx.part_world_aabb(part)
        if box is None:
            return None
        mins, maxs = box
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    gear = object_model.get_part("gear_frame")
    wheel_0 = object_model.get_part("main_wheel_0")
    wheel_1 = object_model.get_part("main_wheel_1")
    left_pod = object_model.get_part("left_engine_pod")
    right_pod = object_model.get_part("right_engine_pod")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    rudder_joint = object_model.get_articulation("hull_to_rudder")
    left_elevator_joint = object_model.get_articulation("hull_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("hull_to_right_elevator")

    ctx.expect_gap(
        hull,
        gondola,
        axis="z",
        positive_elem="envelope",
        negative_elem="equipment_body",
        min_gap=0.75,
        name="gondola hangs clearly below the hull envelope",
    )
    ctx.expect_contact(
        hull,
        gondola,
        elem_a="front_mount_pad",
        elem_b="front_mount_cap",
        name="front gondola mount contacts the hull support pad",
    )
    ctx.expect_contact(
        hull,
        gondola,
        elem_a="rear_mount_pad",
        elem_b="rear_mount_cap",
        name="rear gondola mount contacts the hull support pad",
    )
    ctx.expect_contact(
        gondola,
        gear,
        elem_a="gear_socket",
        elem_b="gear_mount_plate",
        name="landing gear is attached to the gondola structure",
    )
    ctx.expect_contact(
        gondola,
        left_pod,
        elem_a="equipment_body",
        elem_b="pylon_root",
        name="left engine pod is mounted to the gondola side",
    )
    ctx.expect_contact(
        gondola,
        right_pod,
        elem_a="equipment_body",
        elem_b="pylon_root",
        name="right engine pod is mounted to the gondola side",
    )
    ctx.allow_overlap(
        gear,
        wheel_0,
        elem_a="main_axle_bar",
        elem_b="hub",
        reason="The left landing wheel rotates on the supported axle bar that passes into its hub.",
    )
    ctx.allow_overlap(
        gear,
        wheel_1,
        elem_a="main_axle_bar",
        elem_b="hub",
        reason="The right landing wheel rotates on the supported axle bar that passes into its hub.",
    )
    ctx.allow_overlap(
        left_pod,
        left_propeller,
        elem_a="prop_shaft",
        elem_b="propeller",
        reason="The left propeller is intentionally modeled spinning on the shaft that enters its hub region.",
    )
    ctx.allow_overlap(
        right_pod,
        right_propeller,
        elem_a="prop_shaft",
        elem_b="propeller",
        reason="The right propeller is intentionally modeled spinning on the shaft that enters its hub region.",
    )
    ctx.expect_gap(
        gondola,
        wheel_0,
        axis="z",
        positive_elem="equipment_body",
        negative_elem="tire",
        min_gap=0.20,
        name="left wheel sits below the gondola body",
    )
    ctx.expect_gap(
        gondola,
        wheel_1,
        axis="z",
        positive_elem="equipment_body",
        negative_elem="tire",
        min_gap=0.20,
        name="right wheel sits below the gondola body",
    )

    rudder_rest = _aabb_center(rudder)
    with ctx.pose({rudder_joint: 0.45}):
        rudder_deflected = _aabb_center(rudder)
    ctx.check(
        "rudder deflects laterally off the fin centerline",
        rudder_rest is not None
        and rudder_deflected is not None
        and abs(rudder_deflected[1] - rudder_rest[1]) > 0.10,
        details=f"rest={rudder_rest}, deflected={rudder_deflected}",
    )

    left_elevator_rest = _aabb_center(left_elevator)
    with ctx.pose({left_elevator_joint: 0.35}):
        left_elevator_up = _aabb_center(left_elevator)
    ctx.check(
        "left elevator raises at positive deflection",
        left_elevator_rest is not None
        and left_elevator_up is not None
        and left_elevator_up[2] > left_elevator_rest[2] + 0.06,
        details=f"rest={left_elevator_rest}, up={left_elevator_up}",
    )

    right_elevator_rest = _aabb_center(right_elevator)
    with ctx.pose({right_elevator_joint: 0.35}):
        right_elevator_up = _aabb_center(right_elevator)
    ctx.check(
        "right elevator raises at positive deflection",
        right_elevator_rest is not None
        and right_elevator_up is not None
        and right_elevator_up[2] > right_elevator_rest[2] + 0.06,
        details=f"rest={right_elevator_rest}, up={right_elevator_up}",
    )

    return ctx.report()


object_model = build_object_model()
