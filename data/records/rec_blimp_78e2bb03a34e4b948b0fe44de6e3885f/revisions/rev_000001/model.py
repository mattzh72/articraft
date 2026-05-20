from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _hull_envelope() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(-15.95, 0.0)
        .spline(
            [
                (-15.2, 0.28),
                (-13.6, 1.10),
                (-9.0, 2.95),
                (-1.5, 3.95),
                (7.2, 3.55),
                (13.4, 1.85),
                (15.35, 0.42),
            ]
        )
        .lineTo(15.95, 0.0)
        .close()
        .revolve(360, (0, 0), (1, 0))
    )


def _tail_frame() -> cq.Workplane:
    dorsal_root = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-14.45, 0.0),
                (-14.90, 1.15),
                (-15.20, 2.75),
                (-15.48, 3.05),
                (-15.18, 0.48),
            ]
        )
        .close()
        .extrude(0.13, both=True)
    )
    ventral_root = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-14.40, 0.0),
                (-15.05, -0.72),
                (-15.34, -1.90),
                (-15.56, -2.18),
                (-15.12, -0.42),
            ]
        )
        .close()
        .extrude(0.11, both=True)
    )
    right_root = (
        cq.Workplane("XY")
        .polyline(
            [
                (-14.40, 0.00),
                (-14.76, 0.20),
                (-15.00, 0.44),
                (-15.14, 0.58),
                (-14.98, 0.08),
            ]
        )
        .close()
        .extrude(0.09, both=True)
    )
    right_pad = cq.Workplane("XY").box(0.16, 0.22, 0.20).translate((-14.78, 0.61, 0.0))
    left_root = (
        cq.Workplane("XY")
        .polyline(
            [
                (-14.40, 0.00),
                (-14.76, -0.20),
                (-15.00, -0.44),
                (-15.14, -0.58),
                (-14.98, -0.08),
            ]
        )
        .close()
        .extrude(0.09, both=True)
    )
    left_pad = cq.Workplane("XY").box(0.16, 0.22, 0.20).translate((-14.78, -0.61, 0.0))
    rudder_spine = cq.Workplane("XY").box(0.10, 0.18, 5.10).translate((-15.51, 0.0, 0.25))
    return (
        dorsal_root.union(ventral_root)
        .union(right_root)
        .union(left_root)
        .union(right_pad)
        .union(left_pad)
        .union(rudder_spine)
    )


def _mount_frame() -> cq.Workplane:
    frame = cq.Workplane("XY")
    for x_pos in (-1.8, 2.0):
        for y_pos in (-0.90, 0.90):
            frame = frame.union(
                cq.Workplane("XY")
                .box(0.18, 0.18, 1.55)
                .translate((x_pos, y_pos, -4.15))
            )

    for side in (-1.0, 1.0):
        frame = frame.union(
            cq.Workplane("XY")
            .box(0.50, 1.10, 0.28)
            .translate((-1.30, side * 4.15, -1.05))
        )
        frame = frame.union(
            cq.Workplane("XY")
            .box(0.30, 0.30, 0.55)
            .translate((-1.35, side * 3.72, -0.82))
        )
    return frame


def _gondola_body() -> cq.Workplane:
    cabin = (
        cq.Workplane("XY")
        .box(7.6, 2.35, 1.9)
        .edges("|X")
        .fillet(0.12)
        .edges("|Z")
        .fillet(0.18)
    )
    cockpit = (
        cq.Workplane("XY")
        .box(1.85, 2.05, 1.25)
        .translate((2.55, 0.0, 0.18))
        .edges("|Y")
        .fillet(0.12)
        .edges("|Z")
        .fillet(0.10)
    )
    tail_cap = (
        cq.Workplane("XY")
        .box(1.25, 1.85, 1.05)
        .translate((-3.05, 0.0, -0.04))
        .edges("|Y")
        .fillet(0.10)
        .edges("|Z")
        .fillet(0.08)
    )
    support_stem = cq.Workplane("XY").box(0.26, 0.18, 0.28).translate((-2.80, 0.0, -1.08))
    fork_bridge = cq.Workplane("XY").box(0.42, 0.34, 0.12).translate((-2.80, 0.0, -1.18))
    fork_cheek_r = cq.Workplane("XY").box(0.16, 0.05, 0.48).translate((-2.80, 0.12, -1.42))
    fork_cheek_l = cq.Workplane("XY").box(0.16, 0.05, 0.48).translate((-2.80, -0.12, -1.42))
    axle = (
        cq.Workplane("XZ")
        .circle(0.020)
        .extrude(0.12, both=True)
        .translate((-2.80, 0.0, -1.56))
    )
    return (
        cabin.union(cockpit)
        .union(tail_cap)
        .union(support_stem)
        .union(fork_bridge)
        .union(fork_cheek_r)
        .union(fork_cheek_l)
        .union(axle)
    )


def _pod_body(side: float) -> cq.Workplane:
    nacelle = (
        cq.Workplane("YZ")
        .workplane(offset=-2.25)
        .circle(0.05)
        .workplane(offset=0.65)
        .circle(0.24)
        .workplane(offset=0.80)
        .circle(0.39)
        .workplane(offset=0.60)
        .circle(0.34)
        .workplane(offset=0.18)
        .circle(0.17)
        .loft(combine=True)
    )
    shaft = cq.Workplane("YZ").circle(0.06).extrude(0.20).translate((-0.12, 0.0, 0.0))
    mount_boss = cq.Workplane("XY").box(0.54, 0.22, 0.28).translate((-0.92, -side * 0.29, 0.0))
    return nacelle.union(shaft).union(mount_boss)


def _rudder_body() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -2.35),
                (-1.50, -1.70),
                (-1.88, 0.60),
                (-1.55, 2.55),
                (0.0, 2.95),
            ]
        )
        .close()
        .extrude(0.08, both=True)
    )


def _elevator_body(side: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(
            [
                (0.0, 0.0),
                (-1.80, side * 0.30),
                (-2.15, side * 2.50),
                (-1.35, side * 3.30),
                (0.0, side * 3.00),
            ]
        )
        .close()
        .extrude(0.08, both=True)
    )


def _mooring_wheel_body() -> cq.Workplane:
    tire = (
        cq.Workplane("YZ")
        .circle(0.135)
        .extrude(0.0325, both=True)
        .cut(cq.Workplane("YZ").circle(0.093).extrude(0.0325, both=True))
    )
    hub = (
        cq.Workplane("YZ")
        .circle(0.106)
        .extrude(0.025, both=True)
        .cut(cq.Workplane("YZ").circle(0.020).extrude(0.040, both=True))
    )
    return tire.union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rescue_service_blimp")

    hull_red = model.material("hull_red", rgba=(0.84, 0.10, 0.12, 1.0))
    hull_white = model.material("hull_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.52, 0.56, 0.60, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    metallic = model.material("metallic", rgba=(0.63, 0.66, 0.70, 1.0))

    hull = model.part("hull")
    hull.visual(
        mesh_from_cadquery(_hull_envelope(), "envelope"),
        material=hull_red,
        name="envelope",
    )
    hull.visual(
        mesh_from_cadquery(_tail_frame(), "tail_frame"),
        material=hull_red,
        name="tail_frame",
    )
    hull.visual(
        mesh_from_cadquery(_mount_frame(), "mount_frame"),
        material=trim_gray,
        name="mount_frame",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_cadquery(_gondola_body(), "gondola"),
        material=hull_white,
        name="cabin",
    )
    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(1.00, 0.0, -5.875)),
    )

    for index, side in enumerate((1.0, -1.0)):
        pod = model.part(f"pod_{index}")
        pod.visual(
            mesh_from_cadquery(_pod_body(side), f"pod_{index}"),
            material=trim_gray,
            name="nacelle",
        )
        model.articulation(
            f"hull_to_pod_{index}",
            ArticulationType.FIXED,
            parent=hull,
            child=pod,
            origin=Origin(xyz=(-1.0, side * 5.07, -1.05)),
        )

        prop = model.part(f"propeller_{index}")
        prop.visual(
            mesh_from_geometry(
                FanRotorGeometry(
                    0.95,
                    0.22,
                    3,
                    thickness=0.22,
                    blade_pitch_deg=31.0,
                    blade_sweep_deg=17.0,
                    blade=FanRotorBlade(shape="scimitar", camber=0.14, tip_pitch_deg=14.0),
                    hub=FanRotorHub(
                        style="spinner",
                        rear_collar_height=0.05,
                        rear_collar_radius=0.15,
                        bore_diameter=0.05,
                    ),
                    center=False,
                ),
                f"propeller_{index}",
            ),
            material=black,
            name="rotor",
        )
        model.articulation(
            f"pod_{index}_to_propeller_{index}",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=prop,
            origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=25.0, velocity=35.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_cadquery(_rudder_body(), "rudder"),
        material=hull_red,
        name="rudder_surface",
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-15.56, 0.0, 0.25)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.70, effort=40.0, velocity=1.2),
    )

    for name, side in (("elevator_0", 1.0), ("elevator_1", -1.0)):
        elevator = model.part(name)
        elevator.visual(
            mesh_from_cadquery(_elevator_body(side), name),
            material=hull_red,
            name="elevator_surface",
        )
        model.articulation(
            f"hull_to_{name}",
            ArticulationType.REVOLUTE,
            parent=hull,
            child=elevator,
            origin=Origin(xyz=(-14.70, side * 0.72, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=-0.48, upper=0.48, effort=40.0, velocity=1.0),
        )

    wheel = model.part("mooring_wheel")
    wheel.visual(
        mesh_from_cadquery(_mooring_wheel_body(), "mooring_wheel"),
        material=dark_gray,
        name="wheel",
    )
    model.articulation(
        "gondola_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=wheel,
        origin=Origin(xyz=(-2.80, 0.0, -1.56), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def _aabb_extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (high[0] - low[0], high[1] - low[1], high[2] - low[2])


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    pod_0 = object_model.get_part("pod_0")
    pod_1 = object_model.get_part("pod_1")
    propeller_0 = object_model.get_part("propeller_0")
    propeller_1 = object_model.get_part("propeller_1")
    rudder = object_model.get_part("rudder")
    elevator_0 = object_model.get_part("elevator_0")
    wheel = object_model.get_part("mooring_wheel")

    rudder_joint = object_model.get_articulation("hull_to_rudder")
    elevator_joint = object_model.get_articulation("hull_to_elevator_0")

    hull_aabb = ctx.part_world_aabb(hull)
    hull_size = _aabb_extent(hull_aabb)
    ctx.check(
        "airship uses emergency-service scale",
        hull_size is not None and hull_size[0] > 31.0 and hull_size[1] > 8.5 and hull_size[2] > 7.0,
        details=f"hull_size={hull_size}",
    )

    ctx.allow_overlap(
        gondola,
        wheel,
        elem_a="cabin",
        elem_b="wheel",
        reason="The rear mooring wheel is intentionally modeled around its gondola-mounted axle inside the short tail fork.",
    )
    ctx.allow_overlap(
        hull,
        pod_0,
        elem_a="mount_frame",
        elem_b="nacelle",
        reason="The port nacelle mount boss is intentionally bedded into the faired hull pylon.",
    )
    ctx.allow_overlap(
        hull,
        pod_1,
        elem_a="mount_frame",
        elem_b="nacelle",
        reason="The starboard nacelle mount boss is intentionally bedded into the faired hull pylon.",
    )
    ctx.allow_overlap(
        pod_0,
        propeller_0,
        elem_a="nacelle",
        elem_b="rotor",
        reason="The propeller hub is intentionally mounted around the pod shaft collar.",
    )
    ctx.allow_overlap(
        pod_1,
        propeller_1,
        elem_a="nacelle",
        elem_b="rotor",
        reason="The propeller hub is intentionally mounted around the pod shaft collar.",
    )

    ctx.expect_gap(
        hull,
        gondola,
        axis="z",
        positive_elem="envelope",
        negative_elem="cabin",
        min_gap=0.60,
        max_gap=1.40,
        name="gondola hangs clearly below the envelope",
    )

    pod_0_pos = ctx.part_world_position(pod_0)
    pod_1_pos = ctx.part_world_position(pod_1)
    ctx.check(
        "twin propulsion pods flank the hull symmetrically",
        pod_0_pos is not None
        and pod_1_pos is not None
        and pod_0_pos[1] > 4.0
        and pod_1_pos[1] < -4.0
        and abs(pod_0_pos[0] - pod_1_pos[0]) < 0.05
        and abs(pod_0_pos[2] - pod_1_pos[2]) < 0.05
        and abs(pod_0_pos[1] + pod_1_pos[1]) < 0.05,
        details=f"pod_0={pod_0_pos}, pod_1={pod_1_pos}",
    )

    gondola_pos = ctx.part_world_position(gondola)
    wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "mooring wheel sits aft and below the gondola",
        gondola_pos is not None
        and wheel_pos is not None
        and wheel_pos[0] < gondola_pos[0] - 2.0
        and wheel_pos[2] < gondola_pos[2] - 1.0,
        details=f"gondola={gondola_pos}, wheel={wheel_pos}",
    )

    rudder_rest = ctx.part_element_world_aabb(rudder, elem="rudder_surface")
    rudder_upper = None
    if rudder_joint.motion_limits is not None and rudder_joint.motion_limits.upper is not None:
        with ctx.pose({rudder_joint: rudder_joint.motion_limits.upper}):
            rudder_upper = ctx.part_element_world_aabb(rudder, elem="rudder_surface")
    rudder_rest_center = _aabb_center(rudder_rest)
    rudder_upper_center = _aabb_center(rudder_upper)
    ctx.check(
        "rudder swings laterally at its hinge",
        rudder_rest_center is not None
        and rudder_upper_center is not None
        and abs(rudder_upper_center[1] - rudder_rest_center[1]) > 0.35,
        details=f"rest={rudder_rest_center}, upper={rudder_upper_center}",
    )

    elevator_rest = ctx.part_element_world_aabb(elevator_0, elem="elevator_surface")
    elevator_upper = None
    if elevator_joint.motion_limits is not None and elevator_joint.motion_limits.upper is not None:
        with ctx.pose({elevator_joint: elevator_joint.motion_limits.upper}):
            elevator_upper = ctx.part_element_world_aabb(elevator_0, elem="elevator_surface")
    ctx.check(
        "elevator panel pitches upward at positive deflection",
        elevator_rest is not None
        and elevator_upper is not None
        and elevator_upper[1][2] > elevator_rest[1][2] + 0.25,
        details=f"rest={elevator_rest}, upper={elevator_upper}",
    )

    return ctx.report()


object_model = build_object_model()
