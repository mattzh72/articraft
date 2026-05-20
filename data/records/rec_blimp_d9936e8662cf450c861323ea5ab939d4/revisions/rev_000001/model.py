from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HULL_LENGTH = 14.0
HULL_RADIUS = 1.6


def _envelope_shape() -> cq.Workplane:
    sections = [
        (0.0, 0.03),
        (0.7, 0.30),
        (1.6, 0.78),
        (3.0, 1.23),
        (5.0, 1.52),
        (7.0, 1.60),
        (9.0, 1.50),
        (11.0, 1.18),
        (12.5, 0.66),
        (13.5, 0.20),
        (14.0, 0.03),
    ]

    body = cq.Workplane("YZ").circle(sections[0][1])
    previous_x = sections[0][0]
    for x_pos, radius in sections[1:]:
        body = body.workplane(offset=x_pos - previous_x).circle(radius)
        previous_x = x_pos

    return body.loft(combine=True, clean=True).translate((-HULL_LENGTH / 2.0, 0.0, 0.0))


def _gondola_shape() -> cq.Workplane:
    cabin = (
        cq.Workplane("XY")
        .box(3.1, 1.05, 0.92)
        .edges("|Z")
        .fillet(0.08)
        .translate((0.0, 0.0, -0.46))
    )
    nose_cap = (
        cq.Workplane("YZ")
        .circle(0.34)
        .workplane(offset=0.34)
        .circle(0.20)
        .loft(combine=True, clean=True)
        .translate((1.38, 0.0, -0.43))
    )
    return cabin.union(nose_cap)


def _top_fin_shape() -> cq.Workplane:
    profile = [
        (0.05, 0.00),
        (-0.20, 1.18),
        (-0.62, 1.02),
        (-0.62, 0.22),
        (-0.12, 0.00),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.10, both=True)


def _bottom_fin_shape() -> cq.Workplane:
    profile = [
        (0.05, 0.00),
        (-0.12, 0.00),
        (-0.62, -0.22),
        (-0.62, -1.02),
        (-0.20, -1.18),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.10, both=True)


def _horizontal_fin_shape(side_sign: float) -> cq.Workplane:
    profile = [
        (0.05, 0.00),
        (-0.12, 0.00),
        (-0.60, 0.20 * side_sign),
        (-0.60, 0.92 * side_sign),
        (-0.16, 1.18 * side_sign),
    ]
    return cq.Workplane("XY").polyline(profile).close().extrude(0.10, both=True)


def _rudder_shape() -> cq.Workplane:
    profile = [
        (0.00, -0.40),
        (-0.34, -0.48),
        (-0.40, 0.40),
        (0.00, 0.40),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.08, both=True)


def _elevator_shape() -> cq.Workplane:
    profile = [
        (0.00, -0.38),
        (-0.42, -0.46),
        (-0.36, 0.46),
        (0.00, 0.38),
    ]
    return cq.Workplane("XY").polyline(profile).close().extrude(0.08, both=True)


def _pod_shape(side_sign: float) -> cq.Workplane:
    pylon = cq.Workplane("XY").box(0.32, 0.44, 0.18).translate((0.00, 0.22 * side_sign, 0.00))
    nacelle = (
        cq.Workplane("YZ").cylinder(0.46, 0.10).translate((0.00, 0.53 * side_sign, 0.00))
        .union(cq.Workplane("XY").sphere(0.10).translate((0.23, 0.53 * side_sign, 0.00)))
        .union(cq.Workplane("XY").sphere(0.07).translate((-0.23, 0.53 * side_sign, 0.00)))
    )
    return pylon.union(nacelle)


def _add_propeller(part, material) -> None:
    part.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="hub",
    )
    part.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=material,
        name="spinner",
    )
    part.visual(
        Box((0.028, 0.62, 0.10)),
        origin=Origin(rpy=(0.0, 0.24, 0.0)),
        material=material,
        name="blade_y",
    )
    part.visual(
        Box((0.028, 0.10, 0.56)),
        origin=Origin(rpy=(0.0, -0.18, 0.0)),
        material=material,
        name="blade_z",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communications_blimp")

    envelope_mat = model.material("envelope_fabric", rgba=(0.91, 0.93, 0.95, 1.0))
    frame_mat = model.material("frame_grey", rgba=(0.46, 0.50, 0.54, 1.0))
    cabin_mat = model.material("cabin_grey", rgba=(0.32, 0.36, 0.40, 1.0))
    tail_mat = model.material("tail_orange", rgba=(0.88, 0.49, 0.20, 1.0))
    control_mat = model.material("control_red", rgba=(0.74, 0.25, 0.18, 1.0))
    glass_mat = model.material("window_tint", rgba=(0.16, 0.23, 0.28, 1.0))
    tire_mat = model.material("tire_black", rgba=(0.10, 0.10, 0.11, 1.0))
    metal_mat = model.material("metal_dark", rgba=(0.18, 0.19, 0.20, 1.0))

    envelope = model.part("envelope")
    envelope.visual(
        mesh_from_cadquery(_envelope_shape(), "blimp_hull"),
        material=envelope_mat,
        name="hull",
    )
    envelope.visual(
        Box((2.4, 0.56, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -1.48)),
        material=frame_mat,
        name="keel",
    )
    for index, (x_pos, y_pos) in enumerate(((-0.80, -0.28), (-0.80, 0.28), (0.80, -0.28), (0.80, 0.28))):
        envelope.visual(
            Cylinder(radius=0.045, length=0.33),
            origin=Origin(xyz=(x_pos, y_pos, -1.735)),
            material=frame_mat,
            name=f"suspension_{index}",
        )
    envelope.visual(
        Box((0.55, 0.16, 0.36)),
        origin=Origin(xyz=(-5.65, 0.0, 0.83)),
        material=frame_mat,
        name="top_mount",
    )
    envelope.visual(
        Box((0.55, 0.16, 0.36)),
        origin=Origin(xyz=(-5.65, 0.0, -0.83)),
        material=frame_mat,
        name="bottom_mount",
    )
    envelope.visual(
        Box((0.55, 0.36, 0.16)),
        origin=Origin(xyz=(-5.65, -0.83, 0.0)),
        material=frame_mat,
        name="port_mount",
    )
    envelope.visual(
        Box((0.55, 0.36, 0.16)),
        origin=Origin(xyz=(-5.65, 0.83, 0.0)),
        material=frame_mat,
        name="starboard_mount",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_cadquery(_gondola_shape(), "equipment_gondola"),
        material=cabin_mat,
        name="cabin",
    )
    gondola.visual(
        Box((0.04, 0.62, 0.22)),
        origin=Origin(xyz=(1.55, 0.0, -0.22)),
        material=glass_mat,
        name="front_window",
    )
    gondola.visual(
        Box((0.12, 0.05, 0.24)),
        origin=Origin(xyz=(-1.12, -0.10, -1.04)),
        material=frame_mat,
        name="fork_port",
    )
    gondola.visual(
        Box((0.12, 0.05, 0.24)),
        origin=Origin(xyz=(-1.12, 0.10, -1.04)),
        material=frame_mat,
        name="fork_starboard",
    )
    gondola.visual(
        Box((0.16, 0.28, 0.06)),
        origin=Origin(xyz=(-1.12, 0.0, -0.90)),
        material=frame_mat,
        name="fork_bridge",
    )
    gondola.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(xyz=(-1.12, -0.09, -1.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="axle_port",
    )
    gondola.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(xyz=(-1.12, 0.09, -1.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="axle_starboard",
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -1.90)),
    )

    port_pod = model.part("port_pod")
    port_pod.visual(
        mesh_from_cadquery(_pod_shape(-1.0), "port_pod"),
        material=metal_mat,
        name="nacelle",
    )
    port_pod.visual(
        Cylinder(radius=0.024, length=0.10),
        origin=Origin(xyz=(0.36, -0.53, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="shaft",
    )

    starboard_pod = model.part("starboard_pod")
    starboard_pod.visual(
        mesh_from_cadquery(_pod_shape(1.0), "starboard_pod"),
        material=metal_mat,
        name="nacelle",
    )
    starboard_pod.visual(
        Cylinder(radius=0.024, length=0.10),
        origin=Origin(xyz=(0.36, 0.53, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="shaft",
    )

    model.articulation(
        "gondola_to_port_pod",
        ArticulationType.FIXED,
        parent=gondola,
        child=port_pod,
        origin=Origin(xyz=(-0.25, -0.525, -0.28)),
    )
    model.articulation(
        "gondola_to_starboard_pod",
        ArticulationType.FIXED,
        parent=gondola,
        child=starboard_pod,
        origin=Origin(xyz=(-0.25, 0.525, -0.28)),
    )

    port_propeller = model.part("port_propeller")
    _add_propeller(port_propeller, material=metal_mat)
    starboard_propeller = model.part("starboard_propeller")
    _add_propeller(starboard_propeller, material=metal_mat)

    model.articulation(
        "port_pod_to_port_propeller",
        ArticulationType.CONTINUOUS,
        parent=port_pod,
        child=port_propeller,
        origin=Origin(xyz=(0.46, -0.53, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=50.0),
    )
    model.articulation(
        "starboard_pod_to_starboard_propeller",
        ArticulationType.CONTINUOUS,
        parent=starboard_pod,
        child=starboard_propeller,
        origin=Origin(xyz=(0.46, 0.53, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=50.0),
    )

    top_fin = model.part("top_fin")
    top_fin.visual(
        mesh_from_cadquery(_top_fin_shape(), "top_fin"),
        material=tail_mat,
        name="fin",
    )
    top_fin.visual(
        Box((0.18, 0.12, 0.18)),
        origin=Origin(xyz=(-0.05, 0.0, 0.09)),
        material=tail_mat,
        name="root",
    )
    bottom_fin = model.part("bottom_fin")
    bottom_fin.visual(
        mesh_from_cadquery(_bottom_fin_shape(), "bottom_fin"),
        material=tail_mat,
        name="fin",
    )
    bottom_fin.visual(
        Box((0.18, 0.12, 0.18)),
        origin=Origin(xyz=(-0.05, 0.0, -0.09)),
        material=tail_mat,
        name="root",
    )
    port_fin = model.part("port_fin")
    port_fin.visual(
        mesh_from_cadquery(_horizontal_fin_shape(-1.0), "port_fin"),
        material=tail_mat,
        name="fin",
    )
    port_fin.visual(
        Box((0.18, 0.18, 0.12)),
        origin=Origin(xyz=(-0.05, -0.09, 0.0)),
        material=tail_mat,
        name="root",
    )
    starboard_fin = model.part("starboard_fin")
    starboard_fin.visual(
        mesh_from_cadquery(_horizontal_fin_shape(1.0), "starboard_fin"),
        material=tail_mat,
        name="fin",
    )
    starboard_fin.visual(
        Box((0.18, 0.18, 0.12)),
        origin=Origin(xyz=(-0.05, 0.09, 0.0)),
        material=tail_mat,
        name="root",
    )

    model.articulation(
        "envelope_to_top_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=top_fin,
        origin=Origin(xyz=(-5.65, 0.0, 1.01)),
    )
    model.articulation(
        "envelope_to_bottom_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=bottom_fin,
        origin=Origin(xyz=(-5.65, 0.0, -1.01)),
    )
    model.articulation(
        "envelope_to_port_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=port_fin,
        origin=Origin(xyz=(-5.65, -1.01, 0.0)),
    )
    model.articulation(
        "envelope_to_starboard_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=starboard_fin,
        origin=Origin(xyz=(-5.65, 1.01, 0.0)),
    )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_cadquery(_rudder_shape(), "rudder"),
        material=control_mat,
        name="surface",
    )
    port_elevator = model.part("port_elevator")
    port_elevator.visual(
        mesh_from_cadquery(_elevator_shape(), "port_elevator"),
        material=control_mat,
        name="surface",
    )
    starboard_elevator = model.part("starboard_elevator")
    starboard_elevator.visual(
        mesh_from_cadquery(_elevator_shape(), "starboard_elevator"),
        material=control_mat,
        name="surface",
    )

    model.articulation(
        "top_fin_to_rudder",
        ArticulationType.REVOLUTE,
        parent=top_fin,
        child=rudder,
        origin=Origin(xyz=(-0.62, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "port_fin_to_port_elevator",
        ArticulationType.REVOLUTE,
        parent=port_fin,
        child=port_elevator,
        origin=Origin(xyz=(-0.60, -0.58, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "starboard_fin_to_starboard_elevator",
        ArticulationType.REVOLUTE,
        parent=starboard_fin,
        child=starboard_elevator,
        origin=Origin(xyz=(-0.60, 0.58, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.45),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_mat,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.07, length=0.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="hub",
    )

    model.articulation(
        "gondola_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=wheel,
        origin=Origin(xyz=(-1.12, 0.0, -1.11)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    return model


def _aabb_center(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    port_pod = object_model.get_part("port_pod")
    starboard_pod = object_model.get_part("starboard_pod")
    port_propeller = object_model.get_part("port_propeller")
    starboard_propeller = object_model.get_part("starboard_propeller")
    wheel = object_model.get_part("wheel")
    rudder = object_model.get_part("rudder")
    port_elevator = object_model.get_part("port_elevator")
    starboard_elevator = object_model.get_part("starboard_elevator")

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="hull",
        negative_elem="cabin",
        min_gap=0.22,
        max_gap=0.55,
        name="main hull stays distinct above the gondola",
    )
    ctx.expect_overlap(
        envelope,
        gondola,
        axes="x",
        elem_a="hull",
        elem_b="cabin",
        min_overlap=2.2,
        name="gondola sits under the hull center section",
    )
    ctx.expect_overlap(
        envelope,
        gondola,
        axes="y",
        elem_a="hull",
        elem_b="cabin",
        min_overlap=0.90,
        name="gondola remains centered under the envelope",
    )
    ctx.expect_contact(
        port_propeller,
        port_pod,
        elem_a="hub",
        elem_b="shaft",
        name="port propeller sits on the port shaft",
    )
    ctx.expect_contact(
        starboard_propeller,
        starboard_pod,
        elem_a="hub",
        elem_b="shaft",
        name="starboard propeller sits on the starboard shaft",
    )
    ctx.expect_origin_gap(
        gondola,
        wheel,
        axis="z",
        min_gap=1.00,
        name="mooring wheel hangs below the gondola",
    )
    ctx.expect_origin_gap(
        gondola,
        wheel,
        axis="x",
        min_gap=0.90,
        name="mooring wheel sits aft under the cabin tail",
    )

    rudder_joint = object_model.get_articulation("top_fin_to_rudder")
    rudder_limits = rudder_joint.motion_limits
    if rudder_limits is not None and rudder_limits.upper is not None:
        neutral_rudder = ctx.part_element_world_aabb(rudder, elem="surface")
        with ctx.pose({rudder_joint: rudder_limits.upper}):
            deflected_rudder = ctx.part_element_world_aabb(rudder, elem="surface")
        neutral_y = _aabb_center(neutral_rudder, 1)
        deflected_y = _aabb_center(deflected_rudder, 1)
        ctx.check(
            "rudder deflects sideways at its upper limit",
            neutral_y is not None and deflected_y is not None and deflected_y < neutral_y - 0.05,
            details=f"neutral_y={neutral_y}, deflected_y={deflected_y}",
        )

    port_joint = object_model.get_articulation("port_fin_to_port_elevator")
    port_limits = port_joint.motion_limits
    if port_limits is not None and port_limits.upper is not None:
        neutral_port = ctx.part_element_world_aabb(port_elevator, elem="surface")
        with ctx.pose({port_joint: port_limits.upper}):
            raised_port = ctx.part_element_world_aabb(port_elevator, elem="surface")
        neutral_z = _aabb_center(neutral_port, 2)
        raised_z = _aabb_center(raised_port, 2)
        ctx.check(
            "port elevator raises at its upper limit",
            neutral_z is not None and raised_z is not None and raised_z > neutral_z + 0.03,
            details=f"neutral_z={neutral_z}, raised_z={raised_z}",
        )

    starboard_joint = object_model.get_articulation("starboard_fin_to_starboard_elevator")
    starboard_limits = starboard_joint.motion_limits
    if starboard_limits is not None and starboard_limits.upper is not None:
        neutral_starboard = ctx.part_element_world_aabb(starboard_elevator, elem="surface")
        with ctx.pose({starboard_joint: starboard_limits.upper}):
            raised_starboard = ctx.part_element_world_aabb(starboard_elevator, elem="surface")
        neutral_z = _aabb_center(neutral_starboard, 2)
        raised_z = _aabb_center(raised_starboard, 2)
        ctx.check(
            "starboard elevator raises at its upper limit",
            neutral_z is not None and raised_z is not None and raised_z > neutral_z + 0.03,
            details=f"neutral_z={neutral_z}, raised_z={raised_z}",
        )

    return ctx.report()


object_model = build_object_model()
