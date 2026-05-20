from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ellipse_loop_x(
    x: float,
    half_width: float,
    half_height: float,
    *,
    samples: int = 28,
) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            half_width * math.cos((2.0 * math.pi * i) / samples),
            half_height * math.sin((2.0 * math.pi * i) / samples),
        )
        for i in range(samples)
    ]


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _profile_loft_y(
    profile_xz: list[tuple[float, float]], *, thickness: float
):
    half_t = thickness * 0.5
    return section_loft(
        [
            [(x, -half_t, z) for x, z in profile_xz],
            [(x, half_t, z) for x, z in profile_xz],
        ]
    )


def _profile_loft_z(
    profile_xy: list[tuple[float, float]], *, thickness: float
):
    half_t = thickness * 0.5
    return section_loft(
        [
            [(x, y, -half_t) for x, y in profile_xy],
            [(x, y, half_t) for x, y in profile_xy],
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_blimp")

    envelope_skin = model.material("envelope_skin", rgba=(0.86, 0.88, 0.90, 1.0))
    support_gray = model.material("support_gray", rgba=(0.51, 0.54, 0.58, 1.0))
    nacelle_gray = model.material("nacelle_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    carbon_black = model.material("carbon_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.42, 0.50, 0.55))
    tire_black = model.material("tire_black", rgba=(0.06, 0.06, 0.06, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.65, 0.67, 0.70, 1.0))

    envelope_sections = [
        (-8.55, 0.08, 0.08),
        (-7.25, 0.70, 0.68),
        (-5.40, 1.26, 1.22),
        (-2.50, 1.63, 1.50),
        (0.00, 1.74, 1.58),
        (2.65, 1.60, 1.45),
        (5.35, 1.16, 1.06),
        (7.40, 0.54, 0.48),
        (8.75, 0.07, 0.07),
    ]
    envelope_mesh = _save_mesh(
        "envelope_shell",
        section_loft([_ellipse_loop_x(x, w, h, samples=30) for x, w, h in envelope_sections]),
    )

    gondola_body_geom = CapsuleGeometry(0.36, 1.80).rotate_y(math.pi / 2.0).scale(1.0, 0.92, 0.82)
    gondola_body_mesh = _save_mesh("gondola_body", gondola_body_geom)

    nacelle_geom = CapsuleGeometry(0.22, 0.72).rotate_y(math.pi / 2.0).scale(1.0, 0.95, 1.0)
    nacelle_mesh = _save_mesh("engine_nacelle", nacelle_geom)

    rotor_mesh = _save_mesh(
        "engine_rotor",
        FanRotorGeometry(
            0.42,
            0.08,
            3,
            thickness=0.05,
            blade_pitch_deg=18.0,
            blade_sweep_deg=12.0,
            blade=FanRotorBlade(shape="narrow", camber=0.08),
            hub=FanRotorHub(style="spinner", bore_diameter=0.022),
        ),
    )
    fixed_fin_mesh = _save_mesh(
        "fixed_fin",
        _profile_loft_y(
            [
                (0.00, 0.00),
                (0.38, 0.04),
                (0.50, 0.92),
                (0.18, 1.15),
                (0.02, 0.40),
            ],
            thickness=0.10,
        ),
    )
    tailplane_mesh = _save_mesh(
        "tailplane",
        _profile_loft_z(
            [
                (0.00, -0.22),
                (0.10, -1.02),
                (0.42, -1.02),
                (0.50, -0.30),
                (0.50, 0.30),
                (0.42, 1.02),
                (0.10, 1.02),
                (0.00, 0.22),
            ],
            thickness=0.10,
        ),
    )
    rudder_mesh = _save_mesh(
        "rudder_panel",
        _profile_loft_y(
            [
                (0.00, -0.55),
                (0.52, -0.45),
                (0.62, 0.45),
                (0.00, 0.55),
            ],
            thickness=0.04,
        ),
    )
    elevator_mesh = _save_mesh(
        "elevator_panel",
        _profile_loft_z(
            [
                (0.00, -0.48),
                (0.48, -0.42),
                (0.60, 0.42),
                (0.00, 0.48),
            ],
            thickness=0.06,
        ),
    )

    hull = model.part("hull")
    hull.visual(envelope_mesh, material=envelope_skin, name="envelope_shell")
    hull.visual(
        Box((4.30, 0.42, 0.34)),
        origin=Origin(xyz=(-0.50, 0.0, -1.56)),
        material=support_gray,
        name="keel_fairing",
    )
    hull.visual(
        Box((1.10, 0.24, 0.18)),
        origin=Origin(xyz=(-2.55, 0.0, -1.48)),
        material=support_gray,
        name="forward_keel",
    )
    _add_member(
        hull,
        (0.55, 0.72, -0.95),
        (1.02, 1.58, -1.07),
        radius=0.055,
        material=support_gray,
        name="left_engine_strut",
    )
    _add_member(
        hull,
        (0.55, -0.72, -0.95),
        (1.02, -1.58, -1.07),
        radius=0.055,
        material=support_gray,
        name="right_engine_strut",
    )
    hull.visual(
        Box((0.30, 0.18, 0.08)),
        origin=Origin(xyz=(1.10, 1.58, -1.11)),
        material=support_gray,
        name="left_engine_mount",
    )
    hull.visual(
        Box((0.30, 0.18, 0.08)),
        origin=Origin(xyz=(1.10, -1.58, -1.11)),
        material=support_gray,
        name="right_engine_mount",
    )
    _add_member(
        hull,
        (7.00, 0.0, -0.02),
        (8.48, 0.0, 0.00),
        radius=0.120,
        material=support_gray,
        name="tail_boom",
    )
    _add_member(
        hull,
        (7.20, 0.0, 0.24),
        (8.48, 0.0, 0.08),
        radius=0.060,
        material=support_gray,
        name="tail_upper_brace",
    )
    _add_member(
        hull,
        (7.20, 0.0, -0.24),
        (8.48, 0.0, -0.08),
        radius=0.060,
        material=support_gray,
        name="tail_lower_brace",
    )
    hull.visual(
        Box((0.18, 0.36, 0.32)),
        origin=Origin(xyz=(8.71, 0.0, 0.0)),
        material=support_gray,
        name="tail_mount",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((1.20, 0.28, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=support_gray,
        name="mount_plate",
    )
    gondola.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(-0.42, 0.0, -0.11)),
        material=support_gray,
        name="forward_pylon",
    )
    gondola.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(0.40, 0.0, -0.11)),
        material=support_gray,
        name="rear_pylon",
    )
    gondola.visual(
        gondola_body_mesh,
        origin=Origin(xyz=(0.05, 0.0, -0.44)),
        material=nacelle_gray,
        name="body_shell",
    )
    gondola.visual(
        Box((0.95, 0.56, 0.18)),
        origin=Origin(xyz=(-0.42, 0.0, -0.20)),
        material=glass,
        name="canopy",
    )
    gondola.visual(
        Box((0.46, 0.22, 0.16)),
        origin=Origin(xyz=(-0.70, 0.0, -0.56)),
        material=support_gray,
        name="sensor_bay",
    )
    gondola.visual(
        Box((0.62, 0.18, 0.10)),
        origin=Origin(xyz=(0.60, 0.0, -0.72)),
        material=support_gray,
        name="rear_skid",
    )
    gondola.visual(
        Box((0.28, 0.14, 0.16)),
        origin=Origin(xyz=(0.58, 0.0, -0.61)),
        material=support_gray,
        name="skid_post",
    )
    gondola.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(-0.72, 0.0, -0.58)),
        material=support_gray,
        name="sensor_stem",
    )
    gondola.visual(
        Sphere(radius=0.13),
        origin=Origin(xyz=(-0.78, 0.0, -0.64)),
        material=glass,
        name="sensor_ball",
    )

    left_engine = model.part("left_engine")
    left_engine.visual(
        Box((0.26, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=support_gray,
        name="mount_pad",
    )
    left_engine.visual(
        Box((0.18, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=support_gray,
        name="pylon_stub",
    )
    left_engine.visual(
        nacelle_mesh,
        origin=Origin(xyz=(0.08, 0.0, -0.30)),
        material=nacelle_gray,
        name="nacelle_shell",
    )
    left_engine.visual(
        Cylinder(radius=0.09, length=0.03),
        origin=Origin(xyz=(0.67, 0.0, -0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=support_gray,
        name="drive_plate",
    )

    right_engine = model.part("right_engine")
    right_engine.visual(
        Box((0.26, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=support_gray,
        name="mount_pad",
    )
    right_engine.visual(
        Box((0.18, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=support_gray,
        name="pylon_stub",
    )
    right_engine.visual(
        nacelle_mesh,
        origin=Origin(xyz=(0.08, 0.0, -0.30)),
        material=nacelle_gray,
        name="nacelle_shell",
    )
    right_engine.visual(
        Cylinder(radius=0.09, length=0.03),
        origin=Origin(xyz=(0.67, 0.0, -0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=support_gray,
        name="drive_plate",
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        Cylinder(radius=0.08, length=0.05),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="hub",
    )
    left_propeller.visual(
        rotor_mesh,
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon_black,
        name="rotor",
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        Cylinder(radius=0.08, length=0.05),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="hub",
    )
    right_propeller.visual(
        rotor_mesh,
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon_black,
        name="rotor",
    )

    tail_frame = model.part("tail_frame")
    tail_frame.visual(
        Box((0.14, 0.26, 0.26)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=support_gray,
        name="root_block",
    )
    tail_frame.visual(
        Cylinder(radius=0.12, length=0.22),
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=support_gray,
        name="tail_collar",
    )
    tail_frame.visual(
        fixed_fin_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=envelope_skin,
        name="fixed_fin",
    )
    tail_frame.visual(
        tailplane_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=envelope_skin,
        name="tailplane",
    )
    rudder = model.part("rudder")
    rudder.visual(
        rudder_mesh,
        material=envelope_skin,
        name="rudder_panel",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        elevator_mesh,
        material=envelope_skin,
        name="elevator_panel",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        elevator_mesh,
        material=envelope_skin,
        name="elevator_panel",
    )

    wheel_fork = model.part("wheel_fork")
    wheel_fork.visual(
        Box((0.18, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=support_gray,
        name="fork_top",
    )
    wheel_fork.visual(
        Box((0.03, 0.015, 0.16)),
        origin=Origin(xyz=(0.0, 0.0425, -0.10)),
        material=support_gray,
        name="left_arm",
    )
    wheel_fork.visual(
        Box((0.03, 0.015, 0.16)),
        origin=Origin(xyz=(0.0, -0.0425, -0.10)),
        material=support_gray,
        name="right_arm",
    )
    wheel_fork.visual(
        Box((0.04, 0.085, 0.025)),
        origin=Origin(xyz=(-0.012, 0.0, -0.06)),
        material=support_gray,
        name="fork_brace",
    )

    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        Cylinder(radius=0.11, length=0.05),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="tire",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.06, length=0.01),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="left_hub",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.06, length=0.01),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="right_hub",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(-1.00, 0.0, -1.73)),
    )
    model.articulation(
        "hull_to_left_engine",
        ArticulationType.FIXED,
        parent=hull,
        child=left_engine,
        origin=Origin(xyz=(1.10, 1.58, -1.15)),
    )
    model.articulation(
        "hull_to_right_engine",
        ArticulationType.FIXED,
        parent=hull,
        child=right_engine,
        origin=Origin(xyz=(1.10, -1.58, -1.15)),
    )
    model.articulation(
        "left_engine_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_engine,
        child=left_propeller,
        origin=Origin(xyz=(0.685, 0.0, -0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=55.0),
    )
    model.articulation(
        "right_engine_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_engine,
        child=right_propeller,
        origin=Origin(xyz=(0.685, 0.0, -0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=55.0),
    )
    model.articulation(
        "hull_to_tail_frame",
        ArticulationType.FIXED,
        parent=hull,
        child=tail_frame,
        origin=Origin(xyz=(8.80, 0.0, 0.0)),
    )
    model.articulation(
        "tail_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=rudder,
        origin=Origin(xyz=(0.50, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "tail_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=left_elevator,
        origin=Origin(xyz=(0.50, 0.58, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "tail_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=right_elevator,
        origin=Origin(xyz=(0.50, -0.58, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "gondola_to_wheel_fork",
        ArticulationType.FIXED,
        parent=gondola,
        child=wheel_fork,
        origin=Origin(xyz=(0.60, 0.0, -0.77)),
    )
    model.articulation(
        "fork_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=wheel_fork,
        child=mooring_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    left_engine = object_model.get_part("left_engine")
    right_engine = object_model.get_part("right_engine")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    tail_frame = object_model.get_part("tail_frame")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    wheel_fork = object_model.get_part("wheel_fork")
    mooring_wheel = object_model.get_part("mooring_wheel")

    rudder_joint = object_model.get_articulation("tail_to_rudder")
    left_elevator_joint = object_model.get_articulation("tail_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("tail_to_right_elevator")

    ctx.expect_contact(
        gondola,
        hull,
        elem_a="mount_plate",
        elem_b="keel_fairing",
        name="gondola is mounted to the hull keel",
    )
    ctx.expect_contact(
        left_engine,
        hull,
        elem_a="mount_pad",
        elem_b="left_engine_mount",
        name="left nacelle is mounted to the hull",
    )
    ctx.expect_contact(
        right_engine,
        hull,
        elem_a="mount_pad",
        elem_b="right_engine_mount",
        name="right nacelle is mounted to the hull",
    )
    ctx.expect_contact(
        left_propeller,
        left_engine,
        elem_a="hub",
        elem_b="drive_plate",
        name="left propeller sits on the nacelle drive plate",
    )
    ctx.expect_contact(
        right_propeller,
        right_engine,
        elem_a="hub",
        elem_b="drive_plate",
        name="right propeller sits on the nacelle drive plate",
    )
    ctx.expect_contact(
        tail_frame,
        hull,
        elem_a="root_block",
        elem_b="tail_mount",
        name="tail frame is mounted to the rear boom",
    )
    ctx.expect_contact(
        wheel_fork,
        gondola,
        elem_a="fork_top",
        elem_b="rear_skid",
        name="wheel fork is mounted below the gondola tail",
    )
    ctx.expect_contact(
        mooring_wheel,
        wheel_fork,
        elem_a="left_hub",
        elem_b="left_arm",
        name="wheel is supported by the left fork arm",
    )
    ctx.expect_contact(
        mooring_wheel,
        wheel_fork,
        elem_a="right_hub",
        elem_b="right_arm",
        name="wheel is supported by the right fork arm",
    )

    ctx.expect_origin_gap(
        left_engine,
        gondola,
        axis="x",
        min_gap=1.6,
        max_gap=2.4,
        name="left nacelle sits aft of the gondola",
    )
    ctx.expect_origin_gap(
        right_engine,
        gondola,
        axis="x",
        min_gap=1.6,
        max_gap=2.4,
        name="right nacelle sits aft of the gondola",
    )
    ctx.expect_origin_distance(
        left_engine,
        right_engine,
        axes="y",
        min_dist=3.0,
        max_dist=3.4,
        name="engine nacelles stay side mounted",
    )
    ctx.expect_origin_gap(
        gondola,
        mooring_wheel,
        axis="z",
        min_gap=0.45,
        name="mooring wheel hangs beneath the gondola",
    )

    ctx.expect_gap(
        rudder,
        tail_frame,
        axis="x",
        positive_elem="rudder_panel",
        negative_elem="fixed_fin",
        max_gap=0.001,
        max_penetration=0.0,
        name="rudder closes cleanly against the fixed fin",
    )
    ctx.expect_gap(
        left_elevator,
        tail_frame,
        axis="x",
        positive_elem="elevator_panel",
        negative_elem="tailplane",
        max_gap=0.001,
        max_penetration=0.0,
        name="left elevator closes cleanly against the tailplane",
    )
    ctx.expect_gap(
        right_elevator,
        tail_frame,
        axis="x",
        positive_elem="elevator_panel",
        negative_elem="tailplane",
        max_gap=0.001,
        max_penetration=0.0,
        name="right elevator closes cleanly against the tailplane",
    )

    rudder_rest = ctx.part_element_world_aabb(rudder, elem="rudder_panel")
    with ctx.pose({rudder_joint: rudder_joint.motion_limits.upper}):
        rudder_turned = ctx.part_element_world_aabb(rudder, elem="rudder_panel")
    ctx.check(
        "rudder positive deflection swings toward positive y",
        rudder_rest is not None
        and rudder_turned is not None
        and rudder_turned[1][1] > rudder_rest[1][1] + 0.22,
        details=f"rest={rudder_rest}, turned={rudder_turned}",
    )

    left_elevator_rest = ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
    right_elevator_rest = ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
    with ctx.pose(
        {
            left_elevator_joint: left_elevator_joint.motion_limits.upper,
            right_elevator_joint: right_elevator_joint.motion_limits.upper,
        }
    ):
        left_elevator_up = ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
        right_elevator_up = ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
    ctx.check(
        "positive elevator command raises both trailing edges",
        left_elevator_rest is not None
        and right_elevator_rest is not None
        and left_elevator_up is not None
        and right_elevator_up is not None
        and left_elevator_up[1][2] > left_elevator_rest[1][2] + 0.09
        and right_elevator_up[1][2] > right_elevator_rest[1][2] + 0.09,
        details=(
            f"left_rest={left_elevator_rest}, left_up={left_elevator_up}, "
            f"right_rest={right_elevator_rest}, right_up={right_elevator_up}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
