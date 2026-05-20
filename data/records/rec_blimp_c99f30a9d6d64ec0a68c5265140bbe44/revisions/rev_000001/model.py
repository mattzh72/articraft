from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

TAIL_HINGE_X = -12.96
CABIN_ORIGIN = (1.10, 0.0, -4.315)
CABIN_HALF_WIDTH = 1.30
PYLON_PIVOT_OFFSET = 1.32


def _superellipse_section(
    x_pos: float,
    half_width: float,
    half_height: float,
    *,
    exponent: float = 2.4,
    samples: int = 40,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    power = 2.0 / exponent
    for idx in range(samples):
        angle = math.tau * idx / samples
        c = math.cos(angle)
        s = math.sin(angle)
        y = math.copysign(abs(c) ** power, c) * half_width
        z = math.copysign(abs(s) ** power, s) * half_height
        points.append((x_pos, y, z))
    return points


def _mesh_from_sections(
    sections: list[list[tuple[float, float, float]]],
    name: str,
):
    return mesh_from_geometry(repair_loft(section_loft(sections), repair="mesh"), name)


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _vertical_surface(points: list[tuple[float, float]], thickness: float):
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness, both=True)


def _horizontal_surface(points: list[tuple[float, float]], thickness: float):
    return cq.Workplane("XY").polyline(points).close().extrude(thickness, both=True)


def _build_hull_mesh():
    sections = [
        _superellipse_section(-12.86, 0.05, 0.05, exponent=2.0),
        _superellipse_section(-12.20, 0.24, 0.26, exponent=2.0),
        _superellipse_section(-10.90, 0.78, 0.82, exponent=2.1),
        _superellipse_section(-8.00, 1.55, 1.62, exponent=2.2),
        _superellipse_section(-3.20, 2.30, 2.38, exponent=2.25),
        _superellipse_section(1.50, 2.58, 2.66, exponent=2.25),
        _superellipse_section(6.00, 2.45, 2.54, exponent=2.25),
        _superellipse_section(10.00, 2.02, 2.10, exponent=2.2),
        _superellipse_section(12.20, 1.28, 1.34, exponent=2.15),
        _superellipse_section(13.50, 0.54, 0.58, exponent=2.05),
        _superellipse_section(14.25, 0.10, 0.11, exponent=2.0),
    ]
    return _mesh_from_sections(sections, "hull_envelope")


def _build_cabin_mesh():
    sections = [
        _superellipse_section(-4.10, 0.10, 0.14, exponent=2.5),
        _superellipse_section(-3.50, 0.55, 0.60, exponent=2.8),
        _superellipse_section(-2.40, 1.05, 0.85, exponent=3.2),
        _superellipse_section(-0.60, 1.28, 0.99, exponent=3.3),
        _superellipse_section(2.20, 1.30, 1.02, exponent=3.3),
        _superellipse_section(3.30, 1.15, 0.90, exponent=3.0),
        _superellipse_section(4.10, 0.36, 0.40, exponent=2.6),
    ]
    return _mesh_from_sections(sections, "cabin_shell")


def _build_pod_mesh():
    sections = [
        _superellipse_section(-0.72, 0.04, 0.04, exponent=2.0),
        _superellipse_section(-0.48, 0.16, 0.16, exponent=2.0),
        _superellipse_section(-0.12, 0.23, 0.24, exponent=2.15),
        _superellipse_section(0.40, 0.23, 0.24, exponent=2.15),
        _superellipse_section(0.82, 0.16, 0.18, exponent=2.1),
        _superellipse_section(1.06, 0.05, 0.05, exponent=2.0),
    ]
    return _mesh_from_sections(sections, "engine_pod_shell")


def _build_rudder_mesh():
    return mesh_from_cadquery(
        _vertical_surface(
            [
                (0.0, 0.60),
                (-0.10, 0.82),
                (-0.78, 1.26),
                (-1.22, 1.46),
                (-1.22, -1.46),
                (-0.78, -1.26),
                (-0.10, -0.82),
                (0.0, -0.60),
            ],
            0.12,
        ),
        "rudder_surface",
    )


def _build_elevator_mesh(side_sign: float):
    span = 1.34 * side_sign
    profile = [
        (0.0, 0.0),
        (-0.10, 0.16 * side_sign),
        (-0.94, 0.98 * side_sign),
        (-0.74, span),
        (0.0, 1.26 * side_sign),
    ]
    return mesh_from_cadquery(
        _horizontal_surface(profile, 0.10),
        "elevator_surface_pos" if side_sign > 0.0 else "elevator_surface_neg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sightseeing_blimp")

    envelope_paint = model.material("envelope_paint", rgba=(0.92, 0.93, 0.95, 1.0))
    cabin_paint = model.material("cabin_paint", rgba=(0.83, 0.85, 0.88, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    pod_gray = model.material("pod_gray", rgba=(0.56, 0.59, 0.63, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.46, 0.60, 0.68, 0.48))

    hull = model.part("hull")
    hull.visual(_build_hull_mesh(), material=envelope_paint, name="envelope")
    hull.visual(
        mesh_from_cadquery(
            _vertical_surface(
                [
                    (-10.10, 0.24),
                    (-9.55, 0.92),
                    (-10.08, 1.76),
                    (TAIL_HINGE_X, 1.08),
                    (TAIL_HINGE_X, 0.56),
                ],
                0.22,
            ),
            "upper_fin_root",
        ),
        material=envelope_paint,
        name="upper_fin_root",
    )
    hull.visual(
        mesh_from_cadquery(
            _vertical_surface(
                [
                    (-10.10, -0.24),
                    (TAIL_HINGE_X, -0.56),
                    (TAIL_HINGE_X, -1.08),
                    (-10.08, -1.76),
                    (-9.55, -0.92),
                ],
                0.22,
            ),
            "lower_fin_root",
        ),
        material=envelope_paint,
        name="lower_fin_root",
    )
    hull.visual(
        mesh_from_cadquery(
            _horizontal_surface(
                [
                    (-9.95, 0.18),
                    (-9.35, 0.96),
                    (-10.12, 1.66),
                    (TAIL_HINGE_X, 1.72),
                    (TAIL_HINGE_X, 0.55),
                ],
                0.16,
            ),
            "left_tailplane_root",
        ),
        material=envelope_paint,
        name="left_tailplane_root",
    )
    hull.visual(
        mesh_from_cadquery(
            _horizontal_surface(
                [
                    (-9.95, -0.18),
                    (TAIL_HINGE_X, -0.55),
                    (TAIL_HINGE_X, -1.72),
                    (-10.12, -1.66),
                    (-9.35, -0.96),
                ],
                0.16,
            ),
            "right_tailplane_root",
        ),
        material=envelope_paint,
        name="right_tailplane_root",
    )
    for idx, (top, bottom) in enumerate(
        [
            ((-1.65, -0.55, -2.40), (-1.20, -0.82, -3.28)),
            ((-1.65, 0.55, -2.40), (-1.20, 0.82, -3.28)),
            ((2.10, -0.48, -2.46), (1.75, -0.76, -3.24)),
            ((2.10, 0.48, -2.46), (1.75, 0.76, -3.24)),
        ]
    ):
        _add_member(
            hull,
            top,
            bottom,
            radius=0.09,
            material=trim_gray,
            name=f"cabin_strut_{idx}",
        )

    cabin = model.part("cabin")
    cabin.visual(_build_cabin_mesh(), material=cabin_paint, name="cabin_shell")
    cabin.visual(
        Box((6.05, 0.03, 0.58)),
        origin=Origin(xyz=(0.25, 1.255, 0.20)),
        material=glass_tint,
        name="window_band_0",
    )
    cabin.visual(
        Box((6.05, 0.03, 0.58)),
        origin=Origin(xyz=(0.25, -1.255, 0.20)),
        material=glass_tint,
        name="window_band_1",
    )
    cabin.visual(
        Box((0.42, 1.18, 0.58)),
        origin=Origin(xyz=(3.76, 0.0, 0.24)),
        material=glass_tint,
        name="windscreen",
    )
    cabin.visual(
        Box((6.60, 2.05, 0.14)),
        origin=Origin(xyz=(0.10, 0.0, -0.96)),
        material=trim_gray,
        name="cabin_floor",
    )
    for idx, (x_pos, y_pos, z_pos) in enumerate(
        [
            (-2.30, -0.82, 0.88),
            (-2.30, 0.82, 0.88),
            (0.65, -0.76, 0.94),
            (0.65, 0.76, 0.94),
        ]
    ):
        cabin.visual(
            Cylinder(radius=0.14, length=0.22),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=trim_gray,
            name=f"roof_pad_{idx}",
        )

    left_pylon = model.part("left_pylon")
    left_pylon.visual(
        Box((0.22, 0.93, 0.20)),
        origin=Origin(xyz=(0.0, 0.495, 0.00)),
        material=trim_gray,
        name="beam",
    )
    left_pylon.visual(
        Box((0.32, 0.08, 0.30)),
        origin=Origin(xyz=(0.0, 0.028, -0.06)),
        material=trim_gray,
        name="root_fairing",
    )
    left_pylon.visual(
        Box((0.34, 0.04, 0.68)),
        origin=Origin(xyz=(0.0, 0.95, 0.00)),
        material=trim_gray,
        name="inner_fork",
    )
    left_pylon.visual(
        Box((0.34, 0.04, 0.68)),
        origin=Origin(xyz=(0.0, 1.69, 0.00)),
        material=trim_gray,
        name="outer_fork",
    )
    left_pylon.visual(
        Box((0.38, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, 1.32, 0.34)),
        material=trim_gray,
        name="fork_bridge",
    )
    left_pylon.visual(
        Cylinder(radius=0.03, length=0.78),
        origin=Origin(xyz=(0.0, 1.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_pin",
    )
    _add_member(
        left_pylon,
        (0.0, 0.18, -0.18),
        (0.0, 0.94, -0.10),
        radius=0.05,
        material=trim_gray,
        name="brace",
    )

    right_pylon = model.part("right_pylon")
    right_pylon.visual(
        Box((0.22, 0.93, 0.20)),
        origin=Origin(xyz=(0.0, -0.495, 0.00)),
        material=trim_gray,
        name="beam",
    )
    right_pylon.visual(
        Box((0.32, 0.08, 0.30)),
        origin=Origin(xyz=(0.0, -0.028, -0.06)),
        material=trim_gray,
        name="root_fairing",
    )
    right_pylon.visual(
        Box((0.34, 0.04, 0.68)),
        origin=Origin(xyz=(0.0, -0.95, 0.00)),
        material=trim_gray,
        name="inner_fork",
    )
    right_pylon.visual(
        Box((0.34, 0.04, 0.68)),
        origin=Origin(xyz=(0.0, -1.69, 0.00)),
        material=trim_gray,
        name="outer_fork",
    )
    right_pylon.visual(
        Box((0.38, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, -1.32, 0.34)),
        material=trim_gray,
        name="fork_bridge",
    )
    right_pylon.visual(
        Cylinder(radius=0.03, length=0.78),
        origin=Origin(xyz=(0.0, -1.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_pin",
    )
    _add_member(
        right_pylon,
        (0.0, -0.18, -0.18),
        (0.0, -0.94, -0.10),
        radius=0.05,
        material=trim_gray,
        name="brace",
    )

    left_pod = model.part("left_pod")
    left_pod.visual(_build_pod_mesh(), material=pod_gray, name="pod_shell")
    left_pod.visual(
        Cylinder(radius=0.055, length=0.70),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_sleeve",
    )
    left_pod.visual(
        Cylinder(radius=0.09, length=0.36),
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shaft_collar",
    )
    left_pod.visual(
        Cylinder(radius=0.09, length=0.36),
        origin=Origin(xyz=(-0.36, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_exhaust",
    )

    right_pod = model.part("right_pod")
    right_pod.visual(_build_pod_mesh(), material=pod_gray, name="pod_shell")
    right_pod.visual(
        Cylinder(radius=0.055, length=0.70),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_sleeve",
    )
    right_pod.visual(
        Cylinder(radius=0.09, length=0.36),
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shaft_collar",
    )
    right_pod.visual(
        Cylinder(radius=0.09, length=0.36),
        origin=Origin(xyz=(-0.36, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_exhaust",
    )

    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.72,
            0.14,
            3,
            thickness=0.16,
            blade_pitch_deg=30.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="broad", camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.04),
        ),
        "propeller_rotor",
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    left_propeller.visual(
        Cylinder(radius=0.09, length=0.26),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_barrel",
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    right_propeller.visual(
        Cylinder(radius=0.09, length=0.26),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub_barrel",
    )

    rudder = model.part("rudder")
    rudder.visual(_build_rudder_mesh(), material=envelope_paint, name="rudder_surface")

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        _build_elevator_mesh(1.0),
        material=envelope_paint,
        name="elevator_surface",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        _build_elevator_mesh(-1.0),
        material=envelope_paint,
        name="elevator_surface",
    )

    model.articulation(
        "hull_to_cabin",
        ArticulationType.FIXED,
        parent=hull,
        child=cabin,
        origin=Origin(xyz=CABIN_ORIGIN),
    )
    model.articulation(
        "cabin_to_left_pylon",
        ArticulationType.FIXED,
        parent=cabin,
        child=left_pylon,
        origin=Origin(xyz=(0.85, CABIN_HALF_WIDTH + 0.012, 0.00)),
    )
    model.articulation(
        "cabin_to_right_pylon",
        ArticulationType.FIXED,
        parent=cabin,
        child=right_pylon,
        origin=Origin(xyz=(0.85, -(CABIN_HALF_WIDTH + 0.012), 0.00)),
    )
    model.articulation(
        "left_pylon_to_left_pod",
        ArticulationType.REVOLUTE,
        parent=left_pylon,
        child=left_pod,
        origin=Origin(xyz=(0.0, PYLON_PIVOT_OFFSET, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "right_pylon_to_right_pod",
        ArticulationType.REVOLUTE,
        parent=right_pylon,
        child=right_pod,
        origin=Origin(xyz=(0.0, -PYLON_PIVOT_OFFSET, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "left_pod_to_left_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_pod,
        child=left_propeller,
        origin=Origin(xyz=(1.16, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=40.0),
    )
    model.articulation(
        "right_pod_to_right_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_pod,
        child=right_propeller,
        origin=Origin(xyz=(1.16, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=40.0),
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(TAIL_HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=math.radians(-28.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "hull_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(TAIL_HINGE_X, 0.55, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.3,
            lower=math.radians(-18.0),
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "hull_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(TAIL_HINGE_X, -0.55, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.3,
            lower=math.radians(-18.0),
            upper=math.radians(24.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    cabin = object_model.get_part("cabin")
    left_pod = object_model.get_part("left_pod")
    right_pod = object_model.get_part("right_pod")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    left_pod_joint = object_model.get_articulation("left_pylon_to_left_pod")
    right_pod_joint = object_model.get_articulation("right_pylon_to_right_pod")
    left_prop_joint = object_model.get_articulation("left_pod_to_left_propeller")
    right_prop_joint = object_model.get_articulation("right_pod_to_right_propeller")
    rudder_joint = object_model.get_articulation("hull_to_rudder")
    left_elevator_joint = object_model.get_articulation("hull_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("hull_to_right_elevator")

    ctx.expect_gap(
        hull,
        cabin,
        axis="z",
        positive_elem="envelope",
        negative_elem="cabin_shell",
        min_gap=0.45,
        max_gap=1.20,
        name="cabin hangs visibly below the envelope",
    )
    ctx.expect_overlap(
        hull,
        cabin,
        axes="x",
        elem_a="envelope",
        elem_b="cabin_shell",
        min_overlap=6.0,
        name="cabin sits beneath the main hull span",
    )
    ctx.allow_overlap(
        object_model.get_part("left_pylon"),
        left_pod,
        elem_a="pivot_pin",
        elem_b="pivot_sleeve",
        reason="The left engine pod rotates on a real trunnion pin captured by the pylon fork.",
    )
    ctx.allow_overlap(
        object_model.get_part("left_pylon"),
        left_pod,
        elem_a="pivot_pin",
        elem_b="pod_shell",
        reason="The left pod shell is simplified around the trunnion pin carried by the pylon fork.",
    )
    ctx.allow_overlap(
        object_model.get_part("right_pylon"),
        right_pod,
        elem_a="pivot_pin",
        elem_b="pivot_sleeve",
        reason="The right engine pod rotates on a real trunnion pin captured by the pylon fork.",
    )
    ctx.allow_overlap(
        object_model.get_part("right_pylon"),
        right_pod,
        elem_a="pivot_pin",
        elem_b="pod_shell",
        reason="The right pod shell is simplified around the trunnion pin carried by the pylon fork.",
    )
    ctx.allow_overlap(
        cabin,
        object_model.get_part("left_pylon"),
        elem_a="cabin_shell",
        elem_b="root_fairing",
        reason="The left pylon root fairing is intentionally blended into the gondola side shell.",
    )
    ctx.allow_overlap(
        cabin,
        object_model.get_part("right_pylon"),
        elem_a="cabin_shell",
        elem_b="root_fairing",
        reason="The right pylon root fairing is intentionally blended into the gondola side shell.",
    )
    ctx.allow_overlap(
        left_pod,
        left_propeller,
        elem_a="pod_shell",
        elem_b="hub_barrel",
        reason="The left propeller hub barrel intentionally nests into the pod nose fairing.",
    )
    ctx.allow_overlap(
        right_pod,
        right_propeller,
        elem_a="pod_shell",
        elem_b="hub_barrel",
        reason="The right propeller hub barrel intentionally nests into the pod nose fairing.",
    )
    for idx in range(4):
        ctx.allow_overlap(
            hull,
            cabin,
            elem_a=f"cabin_strut_{idx}",
            elem_b=f"roof_pad_{idx}",
            reason="The simplified strut ends intentionally seat into gondola roof hardpoints.",
        )
    ctx.expect_origin_gap(
        left_pod,
        right_pod,
        axis="y",
        min_gap=4.8,
        name="engine pods stay on opposite sides of the gondola",
    )

    ctx.check(
        "propellers use continuous shaft joints",
        left_prop_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_prop_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_prop_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(right_prop_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"left={left_prop_joint.articulation_type}, axis={left_prop_joint.axis}; "
            f"right={right_prop_joint.articulation_type}, axis={right_prop_joint.axis}"
        ),
    )

    left_pod_upper = left_pod_joint.motion_limits.upper if left_pod_joint.motion_limits else None
    if left_pod_upper is not None:
        with ctx.pose({left_pod_joint: 0.0}):
            rest_prop_aabb = ctx.part_world_aabb(left_propeller)
        with ctx.pose({left_pod_joint: left_pod_upper}):
            vectored_prop_aabb = ctx.part_world_aabb(left_propeller)
        ctx.check(
            "left engine pod vectors the propeller upward",
            rest_prop_aabb is not None
            and vectored_prop_aabb is not None
            and vectored_prop_aabb[1][2] > rest_prop_aabb[1][2] + 0.25,
            details=f"rest={rest_prop_aabb}, vectored={vectored_prop_aabb}",
        )

    rudder_upper = rudder_joint.motion_limits.upper if rudder_joint.motion_limits else None
    if rudder_upper is not None:
        with ctx.pose({rudder_joint: 0.0}):
            rest_rudder_aabb = ctx.part_world_aabb(rudder)
        with ctx.pose({rudder_joint: rudder_upper}):
            turned_rudder_aabb = ctx.part_world_aabb(rudder)
        ctx.check(
            "rudder swings laterally about the vertical hinge",
            rest_rudder_aabb is not None
            and turned_rudder_aabb is not None
            and turned_rudder_aabb[1][1] > rest_rudder_aabb[1][1] + 0.20,
            details=f"rest={rest_rudder_aabb}, turned={turned_rudder_aabb}",
        )

    elevator_upper = left_elevator_joint.motion_limits.upper if left_elevator_joint.motion_limits else None
    if elevator_upper is not None:
        with ctx.pose({left_elevator_joint: 0.0, right_elevator_joint: 0.0}):
            rest_left_aabb = ctx.part_world_aabb(left_elevator)
            rest_right_aabb = ctx.part_world_aabb(right_elevator)
        with ctx.pose({left_elevator_joint: elevator_upper, right_elevator_joint: elevator_upper}):
            raised_left_aabb = ctx.part_world_aabb(left_elevator)
            raised_right_aabb = ctx.part_world_aabb(right_elevator)
        ctx.check(
            "elevators pitch their trailing edges upward",
            rest_left_aabb is not None
            and rest_right_aabb is not None
            and raised_left_aabb is not None
            and raised_right_aabb is not None
            and raised_left_aabb[1][2] > rest_left_aabb[1][2] + 0.12
            and raised_right_aabb[1][2] > rest_right_aabb[1][2] + 0.12,
            details=(
                f"left_rest={rest_left_aabb}, left_raised={raised_left_aabb}, "
                f"right_rest={rest_right_aabb}, right_raised={raised_right_aabb}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
