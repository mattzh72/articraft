from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    FanRotorGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _superellipse_section(
    x: float,
    width: float,
    height: float,
    *,
    exponent: float = 2.4,
    samples: int = 28,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float, float]]:
    half_w = max(width * 0.5, 0.01)
    half_h = max(height * 0.5, 0.01)
    cy, cz = center
    points: list[tuple[float, float, float]] = []
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        c = math.cos(angle)
        s = math.sin(angle)
        y = cy + half_w * math.copysign(abs(c) ** (2.0 / exponent), c)
        z = cz + half_h * math.copysign(abs(s) ** (2.0 / exponent), s)
        points.append((x, y, z))
    return points


def _loft_plate_xz(
    outline: list[tuple[float, float]],
    *,
    thickness: float,
) :
    half_t = thickness * 0.5
    return section_loft(
        [
            [(x, -half_t, z) for x, z in outline],
            [(x, half_t, z) for x, z in outline],
        ]
    )


def _loft_plate_xy(
    outline: list[tuple[float, float]],
    *,
    thickness: float,
):
    half_t = thickness * 0.5
    return section_loft(
        [
            [(x, y, -half_t) for x, y in outline],
            [(x, y, half_t) for x, y in outline],
        ]
    )


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.90, 0.92, 0.94, 1.0))
    hull_gray = model.material("hull_gray", rgba=(0.69, 0.72, 0.76, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.47, 0.50, 0.54, 1.0))
    pod_gray = model.material("pod_gray", rgba=(0.58, 0.61, 0.66, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.12, 0.17, 0.22, 1.0))
    prop_dark = model.material("prop_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    hull = model.part("hull")

    envelope_sections = [
        _superellipse_section(-18.0, 0.25, 0.25, exponent=2.0),
        _superellipse_section(-16.7, 2.4, 2.8, exponent=2.1),
        _superellipse_section(-13.4, 5.6, 6.4, exponent=2.2),
        _superellipse_section(-8.0, 8.0, 9.2, exponent=2.35),
        _superellipse_section(-1.0, 8.9, 10.1, exponent=2.5),
        _superellipse_section(6.0, 8.5, 9.8, exponent=2.45),
        _superellipse_section(12.2, 6.2, 7.1, exponent=2.25),
        _superellipse_section(16.0, 2.8, 3.2, exponent=2.1),
        _superellipse_section(18.1, 0.35, 0.35, exponent=2.0),
    ]
    hull.visual(
        _save_mesh("blimp_envelope", section_loft(envelope_sections)),
        material=envelope_white,
        name="envelope",
    )

    hull.visual(
        Box((10.8, 2.5, 1.65)),
        origin=Origin(xyz=(0.45, 0.0, -6.7)),
        material=hull_gray,
        name="cabin_body",
    )
    hull.visual(
        Cylinder(radius=0.82, length=9.4),
        origin=Origin(xyz=(0.55, 0.0, -6.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_gray,
        name="cabin_shell",
    )
    hull.visual(
        Box((2.5, 2.05, 1.5)),
        origin=Origin(xyz=(6.9, 0.0, -6.45)),
        material=hull_gray,
        name="cabin_nose",
    )
    hull.visual(
        Box((1.9, 1.9, 1.4)),
        origin=Origin(xyz=(-5.9, 0.0, -6.55)),
        material=hull_gray,
        name="cabin_tail",
    )
    hull.visual(
        Box((7.9, 0.14, 0.56)),
        origin=Origin(xyz=(0.7, 1.18, -5.95)),
        material=glass_dark,
        name="left_window_band",
    )
    hull.visual(
        Box((7.9, 0.14, 0.56)),
        origin=Origin(xyz=(0.7, -1.18, -5.95)),
        material=glass_dark,
        name="right_window_band",
    )
    hull.visual(
        Box((0.18, 1.45, 0.68)),
        origin=Origin(xyz=(7.15, 0.0, -5.9)),
        material=glass_dark,
        name="front_window",
    )
    hull.visual(
        Box((5.8, 0.34, 0.28)),
        origin=Origin(xyz=(0.5, 0.0, -7.57)),
        material=trim_gray,
        name="keel_strip",
    )

    for x_pos in (-3.8, -0.8, 2.2, 5.0):
        for y_pos in (-0.56, 0.56):
            hull.visual(
                Cylinder(radius=0.13, length=1.8),
                origin=Origin(xyz=(x_pos, y_pos, -4.55)),
                material=trim_gray,
                name=f"cabin_strut_{int((x_pos + 4.0) * 10):02d}_{0 if y_pos < 0 else 1}",
            )
    hull.visual(
        Cylinder(radius=0.17, length=0.8),
        origin=Origin(xyz=(1.2, 3.9, -3.2), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="left_pod_stub",
    )
    hull.visual(
        Cylinder(radius=0.17, length=0.8),
        origin=Origin(xyz=(1.2, -3.9, -3.2), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="right_pod_stub",
    )

    top_fin_outline = [
        (-15.9, 1.08),
        (-17.15, 1.08),
        (-17.15, 4.35),
        (-16.2, 3.9),
    ]
    top_rudder_outline = [
        (0.0, 1.23),
        (-0.95, 1.33),
        (-0.72, 4.02),
        (0.0, 4.28),
    ]
    horizontal_tail_outline = [
        (-15.95, 1.22),
        (-17.15, 1.22),
        (-17.15, 4.72),
        (-16.05, 3.95),
    ]
    elevator_outline = [
        (0.0, 1.25),
        (-0.92, 1.36),
        (-0.66, 4.42),
        (0.0, 4.62),
    ]

    top_fin_mesh = _save_mesh("blimp_top_fin", _loft_plate_xz(top_fin_outline, thickness=0.34))
    top_rudder_mesh = _save_mesh("blimp_top_rudder", _loft_plate_xz(top_rudder_outline, thickness=0.22))
    tailplane_mesh = _save_mesh("blimp_tailplane", _loft_plate_xy(horizontal_tail_outline, thickness=0.28))
    elevator_mesh = _save_mesh("blimp_elevator", _loft_plate_xy(elevator_outline, thickness=0.18))

    hull.visual(top_fin_mesh, material=envelope_white, name="top_fin")
    hull.visual(
        top_fin_mesh,
        origin=Origin(rpy=(math.pi, 0.0, 0.0)),
        material=envelope_white,
        name="bottom_fin",
    )
    hull.visual(tailplane_mesh, material=envelope_white, name="left_tailplane")
    hull.visual(
        tailplane_mesh,
        origin=Origin(rpy=(math.pi, 0.0, 0.0)),
        material=envelope_white,
        name="right_tailplane",
    )

    def _build_pod(part_name: str, side_sign: float):
        pod = model.part(part_name)
        nacelle_sections = [
            _superellipse_section(0.12, 0.42, 0.46, exponent=2.2, samples=22, center=(1.55 * side_sign, 0.0)),
            _superellipse_section(-0.55, 0.88, 0.94, exponent=2.3, samples=22, center=(1.55 * side_sign, 0.0)),
            _superellipse_section(-2.0, 1.06, 1.12, exponent=2.4, samples=22, center=(1.55 * side_sign, 0.0)),
            _superellipse_section(-3.5, 0.82, 0.88, exponent=2.25, samples=22, center=(1.55 * side_sign, 0.0)),
            _superellipse_section(-4.35, 0.32, 0.36, exponent=2.1, samples=22, center=(1.55 * side_sign, 0.0)),
        ]
        pod.visual(
            _save_mesh(f"{part_name}_nacelle", section_loft(nacelle_sections)),
            material=pod_gray,
            name="nacelle_shell",
        )
        pod.visual(
            Cylinder(radius=0.15, length=1.32),
            origin=Origin(
                xyz=(0.0, 0.66 * side_sign, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_gray,
            name="pylon",
        )
        return pod

    left_pod = _build_pod("left_pod", 1.0)
    right_pod = _build_pod("right_pod", -1.0)

    propeller_rotor = _save_mesh(
        "blimp_propeller_rotor",
        FanRotorGeometry(
            outer_radius=0.95,
            hub_radius=0.20,
            blade_count=4,
            thickness=0.10,
            blade_pitch_deg=24.0,
            blade_sweep_deg=22.0,
            blade_root_chord=0.34,
            blade_tip_chord=0.16,
        ).rotate_y(math.pi / 2.0),
    )
    spinner_mesh = _save_mesh(
        "blimp_propeller_spinner",
        ConeGeometry(radius=0.19, height=0.44, radial_segments=28).rotate_y(math.pi / 2.0).translate(0.28, 0.0, 0.0),
    )

    for part_name in ("left_propeller", "right_propeller"):
        prop = model.part(part_name)
        prop.visual(propeller_rotor, material=prop_dark, name="rotor")
        prop.visual(
            Cylinder(radius=0.14, length=0.16),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_gray,
            name="hub",
        )
        prop.visual(
            Cylinder(radius=0.055, length=0.38),
            origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_gray,
            name="shaft",
        )
        prop.visual(spinner_mesh, material=trim_gray, name="spinner")

    top_rudder = model.part("top_rudder")
    top_rudder.visual(top_rudder_mesh, material=envelope_white, name="rudder_surface")

    bottom_rudder = model.part("bottom_rudder")
    bottom_rudder.visual(
        top_rudder_mesh,
        origin=Origin(rpy=(math.pi, 0.0, 0.0)),
        material=envelope_white,
        name="rudder_surface",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(elevator_mesh, material=envelope_white, name="elevator_surface")

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        elevator_mesh,
        origin=Origin(rpy=(math.pi, 0.0, 0.0)),
        material=envelope_white,
        name="elevator_surface",
    )

    model.articulation(
        "left_pod_mount",
        ArticulationType.FIXED,
        parent=hull,
        child=left_pod,
        origin=Origin(xyz=(1.2, 4.3, -3.2)),
    )
    model.articulation(
        "right_pod_mount",
        ArticulationType.FIXED,
        parent=hull,
        child=right_pod,
        origin=Origin(xyz=(1.2, -4.3, -3.2)),
    )
    model.articulation(
        "left_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=left_pod,
        child="left_propeller",
        origin=Origin(xyz=(0.45, 1.55, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3000.0, velocity=40.0),
    )
    model.articulation(
        "right_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=right_pod,
        child="right_propeller",
        origin=Origin(xyz=(0.45, -1.55, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3000.0, velocity=40.0),
    )
    model.articulation(
        "top_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=top_rudder,
        origin=Origin(xyz=(-17.15, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "bottom_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=bottom_rudder,
        origin=Origin(xyz=(-17.15, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(-17.15, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1000.0,
            velocity=1.1,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-17.15, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1000.0,
            velocity=1.1,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    left_pod = object_model.get_part("left_pod")
    right_pod = object_model.get_part("right_pod")
    top_rudder = object_model.get_part("top_rudder")
    bottom_rudder = object_model.get_part("bottom_rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    left_spin = object_model.get_articulation("left_propeller_spin")
    right_spin = object_model.get_articulation("right_propeller_spin")
    top_rudder_hinge = object_model.get_articulation("top_rudder_hinge")
    bottom_rudder_hinge = object_model.get_articulation("bottom_rudder_hinge")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")

    ctx.allow_overlap(
        left_pod,
        "left_propeller",
        elem_a="nacelle_shell",
        elem_b="shaft",
        reason="The propeller shaft is intentionally seated into the simplified nacelle nose instead of a modeled hollow gearbox cavity.",
    )
    ctx.allow_overlap(
        right_pod,
        "right_propeller",
        elem_a="nacelle_shell",
        elem_b="shaft",
        reason="The propeller shaft is intentionally seated into the simplified nacelle nose instead of a modeled hollow gearbox cavity.",
    )

    ctx.expect_origin_gap(
        left_pod,
        hull,
        axis="y",
        min_gap=4.0,
        name="left pod sits outboard of the hull",
    )
    ctx.expect_origin_gap(
        hull,
        right_pod,
        axis="y",
        min_gap=4.0,
        name="right pod sits outboard of the hull",
    )

    ctx.expect_gap(
        hull,
        top_rudder,
        axis="x",
        positive_elem="top_fin",
        negative_elem="rudder_surface",
        max_gap=0.03,
        max_penetration=0.0,
        name="top rudder is hinged at the top fin trailing edge",
    )
    ctx.expect_gap(
        hull,
        bottom_rudder,
        axis="x",
        positive_elem="bottom_fin",
        negative_elem="rudder_surface",
        max_gap=0.03,
        max_penetration=0.0,
        name="bottom rudder is hinged at the lower fin trailing edge",
    )
    ctx.expect_gap(
        hull,
        left_elevator,
        axis="x",
        positive_elem="left_tailplane",
        negative_elem="elevator_surface",
        max_gap=0.03,
        max_penetration=0.0,
        name="left elevator is hinged at the horizontal tail trailing edge",
    )
    ctx.expect_gap(
        hull,
        right_elevator,
        axis="x",
        positive_elem="right_tailplane",
        negative_elem="elevator_surface",
        max_gap=0.03,
        max_penetration=0.0,
        name="right elevator is hinged at the horizontal tail trailing edge",
    )

    ctx.check(
        "propellers use continuous shaft joints",
        (
            left_spin.articulation_type == ArticulationType.CONTINUOUS
            and right_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(left_spin.axis) == (1.0, 0.0, 0.0)
            and tuple(right_spin.axis) == (1.0, 0.0, 0.0)
        ),
        details=(
            f"left type={left_spin.articulation_type}, left axis={left_spin.axis}, "
            f"right type={right_spin.articulation_type}, right axis={right_spin.axis}"
        ),
    )

    rudder_limits = top_rudder_hinge.motion_limits
    if rudder_limits is not None and rudder_limits.lower is not None and rudder_limits.upper is not None:
        with ctx.pose({top_rudder_hinge: rudder_limits.lower, bottom_rudder_hinge: rudder_limits.lower}):
            low_top = _center_from_aabb(ctx.part_element_world_aabb(top_rudder, elem="rudder_surface"))
            low_bottom = _center_from_aabb(ctx.part_element_world_aabb(bottom_rudder, elem="rudder_surface"))
        with ctx.pose({top_rudder_hinge: rudder_limits.upper, bottom_rudder_hinge: rudder_limits.upper}):
            high_top = _center_from_aabb(ctx.part_element_world_aabb(top_rudder, elem="rudder_surface"))
            high_bottom = _center_from_aabb(ctx.part_element_world_aabb(bottom_rudder, elem="rudder_surface"))
        ctx.check(
            "vertical tail surfaces swing laterally",
            (
                low_top is not None
                and high_top is not None
                and low_bottom is not None
                and high_bottom is not None
                and abs(high_top[1] - low_top[1]) > 0.35
                and abs(high_bottom[1] - low_bottom[1]) > 0.35
            ),
            details=(
                f"top lower={low_top}, top upper={high_top}, "
                f"bottom lower={low_bottom}, bottom upper={high_bottom}"
            ),
        )

    elevator_limits = left_elevator_hinge.motion_limits
    if elevator_limits is not None and elevator_limits.lower is not None and elevator_limits.upper is not None:
        with ctx.pose(left_elevator_hinge=elevator_limits.lower, right_elevator_hinge=elevator_limits.lower):
            low_left = _center_from_aabb(ctx.part_element_world_aabb(left_elevator, elem="elevator_surface"))
            low_right = _center_from_aabb(ctx.part_element_world_aabb(right_elevator, elem="elevator_surface"))
        with ctx.pose(left_elevator_hinge=elevator_limits.upper, right_elevator_hinge=elevator_limits.upper):
            high_left = _center_from_aabb(ctx.part_element_world_aabb(left_elevator, elem="elevator_surface"))
            high_right = _center_from_aabb(ctx.part_element_world_aabb(right_elevator, elem="elevator_surface"))
        ctx.check(
            "horizontal tail surfaces pitch through their hinge range",
            (
                low_left is not None
                and high_left is not None
                and low_right is not None
                and high_right is not None
                and (high_left[2] - low_left[2]) > 0.25
                and (high_right[2] - low_right[2]) > 0.25
            ),
            details=(
                f"left lower={low_left}, left upper={high_left}, "
                f"right lower={low_right}, right upper={high_right}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
