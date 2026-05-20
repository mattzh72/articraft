from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ellipse_section(x: float, radius_y: float, radius_z: float, *, segments: int = 40) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius_y * math.cos((2.0 * math.pi * index) / segments),
            radius_z * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _add_propeller(model: ArticulatedObject, name: str, *, hub_material, blade_material):
    propeller = model.part(name)
    propeller.visual(
        Cylinder(radius=0.13, length=0.18),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    propeller.visual(
        Box((0.045, 0.18, 1.32)),
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        material=blade_material,
        name="blade_vertical",
    )
    propeller.visual(
        Box((0.045, 1.32, 0.18)),
        origin=Origin(xyz=(0.26, 0.0, 0.0)),
        material=blade_material,
        name="blade_horizontal",
    )
    propeller.visual(
        Cylinder(radius=0.06, length=0.10),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="spinner",
    )
    return propeller


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    envelope_orange = model.material("envelope_orange", rgba=(0.86, 0.43, 0.12, 1.0))
    hull_gray = model.material("hull_gray", rgba=(0.52, 0.55, 0.58, 1.0))
    cabin_gray = model.material("cabin_gray", rgba=(0.65, 0.67, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.15, 0.22, 0.28, 1.0))
    prop_black = model.material("prop_black", rgba=(0.06, 0.06, 0.07, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    envelope = model.part("envelope")
    envelope_sections = [
        _ellipse_section(-13.0, 0.05, 0.05),
        _ellipse_section(-11.8, 0.85, 0.95),
        _ellipse_section(-8.4, 2.45, 2.75),
        _ellipse_section(-4.2, 2.95, 3.05),
        _ellipse_section(0.0, 3.10, 3.15),
        _ellipse_section(4.5, 3.02, 3.08),
        _ellipse_section(9.0, 2.15, 2.35),
        _ellipse_section(12.0, 0.62, 0.72),
        _ellipse_section(13.0, 0.04, 0.04),
    ]
    envelope.visual(
        _mesh("utility_blimp_envelope", section_loft(envelope_sections)),
        material=envelope_orange,
        name="hull",
    )
    envelope.visual(
        Box((4.8, 0.58, 0.38)),
        origin=Origin(xyz=(0.0, 0.0, -3.05)),
        material=hull_gray,
        name="keel",
    )
    envelope.visual(
        Box((1.10, 0.44, 0.30)),
        origin=Origin(xyz=(-0.82, 2.98, -1.11)),
        material=hull_gray,
        name="left_pod_mount",
    )
    envelope.visual(
        Box((1.10, 0.44, 0.30)),
        origin=Origin(xyz=(-0.82, -2.98, -1.11)),
        material=hull_gray,
        name="right_pod_mount",
    )
    envelope.visual(
        Box((0.58, 0.48, 0.48)),
        origin=Origin(xyz=(-13.04, 0.0, 0.0)),
        material=hull_gray,
        name="tail_mount",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((2.40, 0.64, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=hull_gray,
        name="top_saddle",
    )
    gondola.visual(
        Box((0.92, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=hull_gray,
        name="mount_boss",
    )
    gondola.visual(
        Box((4.70, 1.62, 0.28)),
        origin=Origin(xyz=(0.10, 0.0, -0.27)),
        material=cabin_gray,
        name="roof",
    )
    gondola.visual(
        Box((5.40, 1.85, 1.35)),
        origin=Origin(xyz=(0.08, 0.0, -0.95)),
        material=cabin_gray,
        name="cabin",
    )
    gondola.visual(
        Box((1.10, 1.45, 1.02)),
        origin=Origin(xyz=(2.35, 0.0, -0.82)),
        material=cabin_gray,
        name="nose",
    )
    gondola.visual(
        Box((0.68, 1.32, 0.42)),
        origin=Origin(xyz=(2.58, 0.0, -0.48)),
        material=glass_dark,
        name="windshield",
    )
    gondola.visual(
        Box((3.10, 1.42, 0.30)),
        origin=Origin(xyz=(-0.18, 0.0, -1.77)),
        material=dark_metal,
        name="belly_fairing",
    )

    strut_paths = [
        [(0.95, 0.24, -0.12), (1.18, 0.38, -0.22), (1.48, 0.56, -0.40)],
        [(0.95, -0.24, -0.12), (1.18, -0.38, -0.22), (1.48, -0.56, -0.40)],
        [(-0.95, 0.24, -0.12), (-1.18, 0.38, -0.22), (-1.48, 0.56, -0.40)],
        [(-0.95, -0.24, -0.12), (-1.18, -0.38, -0.22), (-1.48, -0.56, -0.40)],
    ]
    for index, path in enumerate(strut_paths):
        gondola.visual(
            _mesh(f"utility_blimp_gondola_strut_{index}", tube_from_spline_points(path, radius=0.06, samples_per_segment=10, radial_segments=16)),
            material=hull_gray,
            name=f"strut_{index}",
        )

    gondola.visual(
        Box((0.20, 0.34, 0.52)),
        origin=Origin(xyz=(-1.62, 0.0, -1.88)),
        material=dark_metal,
        name="wheel_post",
    )
    gondola.visual(
        Box((0.72, 0.10, 0.18)),
        origin=Origin(xyz=(-1.94, 0.10, -2.20)),
        material=dark_metal,
        name="fork_arm_upper",
    )
    gondola.visual(
        Box((0.72, 0.10, 0.18)),
        origin=Origin(xyz=(-1.94, -0.10, -2.20)),
        material=dark_metal,
        name="fork_arm_lower",
    )

    left_pod = model.part("left_pod")
    left_pod.visual(
        Cylinder(radius=0.10, length=0.22),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_gray,
        name="shaft_collar",
    )
    left_pod.visual(
        Cylinder(radius=0.30, length=1.70),
        origin=Origin(xyz=(-0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="nacelle",
    )
    left_pod.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(-1.84, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_cone",
    )
    left_pod.visual(
        Box((0.56, 0.56, 0.56)),
        origin=Origin(xyz=(-0.42, -0.24, 0.43)),
        material=hull_gray,
        name="pylon",
    )

    right_pod = model.part("right_pod")
    right_pod.visual(
        Cylinder(radius=0.10, length=0.22),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hull_gray,
        name="shaft_collar",
    )
    right_pod.visual(
        Cylinder(radius=0.30, length=1.70),
        origin=Origin(xyz=(-0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="nacelle",
    )
    right_pod.visual(
        Cylinder(radius=0.18, length=0.48),
        origin=Origin(xyz=(-1.84, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_cone",
    )
    right_pod.visual(
        Box((0.56, 0.56, 0.56)),
        origin=Origin(xyz=(-0.42, 0.24, 0.43)),
        material=hull_gray,
        name="pylon",
    )

    left_propeller = _add_propeller(model, "left_propeller", hub_material=hull_gray, blade_material=prop_black)
    right_propeller = _add_propeller(model, "right_propeller", hub_material=hull_gray, blade_material=prop_black)

    tail_frame = model.part("tail_frame")
    tail_frame.visual(
        Box((1.00, 0.58, 0.58)),
        origin=Origin(xyz=(-0.50, 0.0, 0.0)),
        material=hull_gray,
        name="tail_boom",
    )
    tail_frame.visual(
        Box((2.60, 0.26, 3.20)),
        origin=Origin(xyz=(-1.30, 0.0, 0.35)),
        material=hull_gray,
        name="rear_fin",
    )
    tail_frame.visual(
        Box((2.10, 3.80, 0.26)),
        origin=Origin(xyz=(-1.05, 0.0, 0.0)),
        material=hull_gray,
        name="tailplane",
    )

    rudder = model.part("rudder")
    rudder.visual(
        Cylinder(radius=0.06, length=3.00),
        origin=Origin(xyz=(-0.06, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    rudder.visual(
        Box((0.95, 0.18, 3.00)),
        origin=Origin(xyz=(-0.475, 0.0, 0.0)),
        material=cabin_gray,
        name="rudder_surface",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Cylinder(radius=0.05, length=1.22),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    left_elevator.visual(
        Box((0.82, 1.22, 0.16)),
        origin=Origin(xyz=(-0.41, 0.0, 0.0)),
        material=cabin_gray,
        name="elevator_surface",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Cylinder(radius=0.05, length=1.22),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    right_elevator.visual(
        Box((0.82, 1.22, 0.16)),
        origin=Origin(xyz=(-0.41, 0.0, 0.0)),
        material=cabin_gray,
        name="elevator_surface",
    )

    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hull_gray,
        name="hub",
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -3.24)),
    )
    model.articulation(
        "envelope_to_left_pod",
        ArticulationType.FIXED,
        parent=envelope,
        child=left_pod,
        origin=Origin(xyz=(-0.75, 3.56, -1.97)),
    )
    model.articulation(
        "envelope_to_right_pod",
        ArticulationType.FIXED,
        parent=envelope,
        child=right_pod,
        origin=Origin(xyz=(-0.75, -3.56, -1.97)),
    )
    model.articulation(
        "envelope_to_tail_frame",
        ArticulationType.FIXED,
        parent=envelope,
        child=tail_frame,
        origin=Origin(xyz=(-13.33, 0.0, 0.0)),
    )
    model.articulation(
        "left_pod_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_pod,
        child=left_propeller,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "right_pod_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_pod,
        child=right_propeller,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "tail_frame_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=rudder,
        origin=Origin(xyz=(-2.60, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.50, upper=0.50),
    )
    model.articulation(
        "tail_frame_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=left_elevator,
        origin=Origin(xyz=(-2.10, 1.28, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=-0.44, upper=0.44),
    )
    model.articulation(
        "tail_frame_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=right_elevator,
        origin=Origin(xyz=(-2.10, -1.28, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=-0.44, upper=0.44),
    )
    model.articulation(
        "gondola_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=mooring_wheel,
        origin=Origin(xyz=(-2.10, 0.0, -2.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    left_pod = object_model.get_part("left_pod")
    right_pod = object_model.get_part("right_pod")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    mooring_wheel = object_model.get_part("mooring_wheel")

    rudder_joint = object_model.get_articulation("tail_frame_to_rudder")
    left_elevator_joint = object_model.get_articulation("tail_frame_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("tail_frame_to_right_elevator")
    left_propeller_joint = object_model.get_articulation("left_pod_to_propeller")
    right_propeller_joint = object_model.get_articulation("right_pod_to_propeller")
    wheel_joint = object_model.get_articulation("gondola_to_mooring_wheel")

    ctx.expect_origin_gap(
        envelope,
        gondola,
        axis="z",
        min_gap=3.0,
        max_gap=3.4,
        name="gondola hangs well below the hull centerline",
    )
    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        max_gap=0.02,
        max_penetration=0.001,
        name="gondola saddle seats cleanly against the hull keel",
    )
    ctx.expect_origin_distance(
        left_pod,
        right_pod,
        axes="y",
        min_dist=6.8,
        max_dist=7.4,
        name="engine pods are mounted wide on both sides of the hull",
    )
    ctx.expect_origin_gap(
        gondola,
        mooring_wheel,
        axis="z",
        min_gap=2.0,
        name="mooring wheel sits below the gondola cabin",
    )
    ctx.expect_origin_gap(
        gondola,
        mooring_wheel,
        axis="x",
        min_gap=1.4,
        name="mooring wheel sits under the rear half of the gondola",
    )

    for joint_name, joint in (
        ("left propeller spins continuously", left_propeller_joint),
        ("right propeller spins continuously", right_propeller_joint),
        ("mooring wheel spins continuously", wheel_joint),
    ):
        limits = joint.motion_limits
        ctx.check(
            joint_name,
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"motion_limits={limits}",
        )

    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        rudder_rest = ctx.part_world_aabb(rudder)
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            rudder_turned = ctx.part_world_aabb(rudder)
        ctx.check(
            "rudder deflects laterally at positive limit",
            rudder_rest is not None
            and rudder_turned is not None
            and rudder_turned[0][1] < rudder_rest[0][1] - 0.18,
            details=f"rest={rudder_rest}, turned={rudder_turned}",
        )

    left_limits = left_elevator_joint.motion_limits
    right_limits = right_elevator_joint.motion_limits
    if (
        left_limits is not None
        and left_limits.upper is not None
        and right_limits is not None
        and right_limits.upper is not None
    ):
        left_rest = ctx.part_world_aabb(left_elevator)
        right_rest = ctx.part_world_aabb(right_elevator)
        with ctx.pose(
            {
                left_elevator_joint: left_limits.upper,
                right_elevator_joint: right_limits.upper,
            }
        ):
            left_up = ctx.part_world_aabb(left_elevator)
            right_up = ctx.part_world_aabb(right_elevator)
        ctx.check(
            "elevators pitch upward together",
            left_rest is not None
            and right_rest is not None
            and left_up is not None
            and right_up is not None
            and left_up[1][2] > left_rest[1][2] + 0.12
            and right_up[1][2] > right_rest[1][2] + 0.12,
            details=f"left_rest={left_rest}, left_up={left_up}, right_rest={right_rest}, right_up={right_up}",
        )

    return ctx.report()


object_model = build_object_model()
