from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _frustum(z0: float, z1: float, r0: float, r1: float) -> cq.Workplane:
    """Tapered circular lighthouse-tower band in world-local coordinates."""
    return (
        cq.Workplane("XY")
        .circle(r0)
        .workplane(offset=z1 - z0)
        .circle(r1)
        .loft(combine=True)
        .translate((0.0, 0.0, z0))
    )


def _tube(z0: float, z1: float, r_outer: float, r_inner: float) -> cq.Workplane:
    """Short hollow cylinder with a real central clearance hole."""
    height = z1 - z0
    outer = cq.Workplane("XY").circle(r_outer).extrude(height)
    cutter = cq.Workplane("XY").circle(r_inner).extrude(height + 0.02).translate((0.0, 0.0, -0.01))
    return outer.cut(cutter).translate((0.0, 0.0, z0))


def _add_tangent_box(
    part,
    *,
    name: str,
    radius: float,
    theta: float,
    z: float,
    size: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * math.cos(theta), radius * math.sin(theta), z),
            rpy=(0.0, 0.0, theta),
        ),
        material=material,
        name=name,
    )


def _tower_radius(z: float) -> float:
    bottom_z, top_z = 0.25, 3.95
    bottom_r, top_r = 0.66, 0.36
    t = max(0.0, min(1.0, (z - bottom_z) / (top_z - bottom_z)))
    return bottom_r + (top_r - bottom_r) * t


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_lighthouse")

    model.material("whitewashed_plaster", rgba=(0.92, 0.88, 0.78, 1.0))
    model.material("deep_red_paint", rgba=(0.65, 0.05, 0.04, 1.0))
    model.material("dark_iron", rgba=(0.02, 0.025, 0.03, 1.0))
    model.material("weathered_stone", rgba=(0.45, 0.43, 0.39, 1.0))
    model.material("copper_roof", rgba=(0.18, 0.32, 0.30, 1.0))
    model.material("lantern_glass", rgba=(0.55, 0.78, 0.92, 0.34))
    model.material("warm_light", rgba=(1.0, 0.78, 0.20, 1.0))
    model.material("brass", rgba=(0.86, 0.58, 0.16, 1.0))
    model.material("lens_blue", rgba=(0.25, 0.55, 1.0, 0.62))

    lighthouse = model.part("lighthouse")

    # Stone plinth and tapered striped tower.
    lighthouse.visual(
        Cylinder(radius=0.82, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material="weathered_stone",
        name="stone_plinth",
    )
    lighthouse.visual(
        Cylinder(radius=0.11, length=4.10),
        origin=Origin(xyz=(0.0, 0.0, 2.05)),
        material="weathered_stone",
        name="tower_core",
    )
    bands = [
        (0.24, 0.92, "deep_red_paint"),
        (0.90, 1.58, "whitewashed_plaster"),
        (1.56, 2.24, "deep_red_paint"),
        (2.22, 2.90, "whitewashed_plaster"),
        (2.88, 3.96, "deep_red_paint"),
    ]
    for i, (z0, z1, mat) in enumerate(bands):
        lighthouse.visual(
            mesh_from_cadquery(
                _frustum(z0, z1, _tower_radius(z0), _tower_radius(z1)),
                f"tower_band_{i}",
                tolerance=0.001,
                angular_tolerance=0.08,
            ),
            material=mat,
            name=f"tower_band_{i}",
        )

    # Surface-mounted door and small windows make the tower read as inhabited.
    lighthouse.visual(
        Box((0.060, 0.26, 0.58)),
        origin=Origin(xyz=(_tower_radius(0.62) + 0.004, 0.0, 0.62)),
        material="dark_iron",
        name="front_door",
    )
    for i, (z, theta) in enumerate(((1.55, math.pi), (2.30, 0.0), (3.05, math.pi))):
        r = _tower_radius(z) + 0.012
        _add_tangent_box(
            lighthouse,
            name=f"tower_window_{i}",
            radius=r,
            theta=theta,
            z=z,
            size=(0.055, 0.20, 0.32),
            material="dark_iron",
        )

    # Broad outer gallery deck and rail around the lantern room.
    lighthouse.visual(
        Cylinder(radius=0.96, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 4.02)),
        material="dark_iron",
        name="gallery_deck",
    )
    lighthouse.visual(
        Cylinder(radius=0.58, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 4.16)),
        material="dark_iron",
        name="lantern_sill",
    )

    rail_radius = 0.88
    rail_segments = 20
    gate_half_angle = 0.40
    for i in range(rail_segments):
        theta0 = -math.pi + (2.0 * math.pi * i / rail_segments)
        theta1 = -math.pi + (2.0 * math.pi * (i + 1) / rail_segments)
        mid = (theta0 + theta1) / 2.0
        if abs(math.atan2(math.sin(mid), math.cos(mid))) < gate_half_angle:
            continue
        chord = 2.0 * rail_radius * math.sin((theta1 - theta0) / 2.0)
        seg_radius = rail_radius * math.cos((theta1 - theta0) / 2.0)
        for z, label in ((4.48, "mid"), (4.78, "top")):
            _add_tangent_box(
                lighthouse,
                name=f"gallery_{label}_rail_{i}",
                radius=seg_radius,
                theta=mid,
                z=z,
                size=(0.035, chord + 0.018, 0.035),
                material="dark_iron",
            )
    for i in range(rail_segments):
        theta = -math.pi + (2.0 * math.pi * i / rail_segments)
        if abs(math.atan2(math.sin(theta), math.cos(theta))) < gate_half_angle:
            continue
        lighthouse.visual(
            Cylinder(radius=0.022, length=0.78),
            origin=Origin(xyz=(rail_radius * math.cos(theta), rail_radius * math.sin(theta), 4.44)),
            material="dark_iron",
            name=f"gallery_post_{i}",
        )

    # Fixed hinge pin at the gallery gap; the gate has hollow sleeves around it.
    hinge_x, hinge_y = 0.90, -0.24
    lighthouse.visual(
        Cylinder(radius=0.014, length=0.82),
        origin=Origin(xyz=(hinge_x, hinge_y, 4.48)),
        material="dark_iron",
        name="gate_hinge_pin",
    )

    # Octagonal lantern room: glass panels, iron mullions, top and bottom frames.
    lantern_radius = 0.52
    for i in range(8):
        theta = i * math.tau / 8.0
        chord = 2.0 * lantern_radius * math.sin(math.pi / 8.0)
        _add_tangent_box(
            lighthouse,
            name=f"lantern_glass_{i}",
            radius=lantern_radius,
            theta=theta,
            z=4.72,
            size=(0.018, chord * 0.90, 0.86),
            material="lantern_glass",
        )
        lighthouse.visual(
            Cylinder(radius=0.022, length=0.96),
            origin=Origin(
                xyz=(
                    lantern_radius * math.cos(theta + math.pi / 8.0),
                    lantern_radius * math.sin(theta + math.pi / 8.0),
                    4.72,
                )
            ),
            material="dark_iron",
            name=f"lantern_mullion_{i}",
        )
        for z, label in ((4.26, "lower"), (5.14, "upper")):
            _add_tangent_box(
                lighthouse,
                name=f"lantern_{label}_frame_{i}",
                radius=lantern_radius * math.cos(math.pi / 8.0),
                theta=theta,
                z=z,
                size=(0.040, chord + 0.010, 0.070),
                material="dark_iron",
            )

    lighthouse.visual(
        Cylinder(radius=0.035, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 4.74)),
        material="dark_iron",
        name="beacon_shaft",
    )
    lighthouse.visual(
        mesh_from_cadquery(_frustum(5.14, 5.72, 0.62, 0.07), "lantern_roof", tolerance=0.001),
        material="copper_roof",
        name="lantern_roof",
    )
    lighthouse.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.0, 0.0, 5.76)),
        material="brass",
        name="roof_finial",
    )

    # Rotating beacon carriage.  Its frame is on the central shaft at the lantern sill.
    beacon = model.part("beacon_carriage")
    beacon.visual(
        mesh_from_cadquery(_tube(0.34, 0.56, 0.085, 0.035), "beacon_hub", tolerance=0.0008),
        material="brass",
        name="beacon_hub",
    )
    beacon.visual(
        Box((0.24, 0.055, 0.055)),
        origin=Origin(xyz=(0.205, 0.0, 0.45)),
        material="brass",
        name="lens_crossarm",
    )
    beacon.visual(
        Box((0.22, 0.050, 0.050)),
        origin=Origin(xyz=(-0.195, 0.0, 0.45)),
        material="brass",
        name="rear_crossarm",
    )
    beacon.visual(
        Sphere(radius=0.085),
        origin=Origin(xyz=(0.17, 0.0, 0.45)),
        material="warm_light",
        name="lamp_globe",
    )
    beacon.visual(
        Box((0.090, 0.20, 0.24)),
        origin=Origin(xyz=(0.34, 0.0, 0.45)),
        material="lens_blue",
        name="front_lens",
    )
    beacon.visual(
        Box((0.070, 0.16, 0.20)),
        origin=Origin(xyz=(-0.30, 0.0, 0.45)),
        material="lens_blue",
        name="rear_lens",
    )
    model.articulation(
        "shaft_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=lighthouse,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 4.29)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5),
    )

    # Hinged small gate in the outer gallery rail.  The hollow sleeves wrap the
    # fixed pin with clearance, while the bars fill the rail gap at q=0.
    gate = model.part("gallery_gate")
    gate.visual(
        mesh_from_cadquery(_tube(0.04, 0.33, 0.038, 0.014), "gate_lower_sleeve", tolerance=0.0008),
        material="dark_iron",
        name="lower_sleeve",
    )
    gate.visual(
        mesh_from_cadquery(_tube(0.45, 0.74, 0.038, 0.014), "gate_upper_sleeve", tolerance=0.0008),
        material="dark_iron",
        name="upper_sleeve",
    )
    gate.visual(
        Box((0.034, 0.035, 0.74)),
        origin=Origin(xyz=(0.054, 0.0, 0.39)),
        material="dark_iron",
        name="hinge_stile",
    )
    for z, label in ((0.20, "lower"), (0.43, "middle"), (0.66, "upper")):
        gate.visual(
            Box((0.036, 0.46, 0.034)),
            origin=Origin(xyz=(0.054, 0.23, z)),
            material="dark_iron",
            name=f"{label}_bar",
        )
    gate.visual(
        Box((0.038, 0.038, 0.70)),
        origin=Origin(xyz=(0.054, 0.46, 0.39)),
        material="dark_iron",
        name="latch_stile",
    )
    model.articulation(
        "rail_to_gate",
        ArticulationType.REVOLUTE,
        parent=lighthouse,
        child=gate,
        origin=Origin(xyz=(hinge_x, hinge_y, 4.10)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beacon = object_model.get_part("beacon_carriage")
    gate = object_model.get_part("gallery_gate")
    lighthouse = object_model.get_part("lighthouse")
    beacon_joint = object_model.get_articulation("shaft_to_beacon")
    gate_joint = object_model.get_articulation("rail_to_gate")

    ctx.allow_overlap(
        beacon,
        lighthouse,
        elem_a="beacon_hub",
        elem_b="beacon_shaft",
        reason="The rotating beacon hub is visually a hollow bearing collar captured on the central shaft.",
    )
    ctx.allow_overlap(
        gate,
        lighthouse,
        elem_a="lower_sleeve",
        elem_b="gate_hinge_pin",
        reason="The lower gate hinge sleeve is captured around the fixed gallery hinge pin.",
    )
    ctx.allow_overlap(
        gate,
        lighthouse,
        elem_a="upper_sleeve",
        elem_b="gate_hinge_pin",
        reason="The upper gate hinge sleeve is captured around the fixed gallery hinge pin.",
    )

    ctx.check(
        "beacon uses continuous rotation",
        beacon_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(beacon_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={beacon_joint.articulation_type}, axis={beacon_joint.axis}",
    )
    ctx.check(
        "gallery gate is vertically hinged",
        gate_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(gate_joint.axis) == (0.0, 0.0, -1.0),
        details=f"type={gate_joint.articulation_type}, axis={gate_joint.axis}",
    )
    ctx.expect_within(
        gate,
        lighthouse,
        axes="z",
        inner_elem="latch_stile",
        outer_elem="gallery_deck",
        margin=0.95,
        name="gate remains at gallery height",
    )
    ctx.expect_within(
        beacon,
        lighthouse,
        axes="xy",
        inner_elem="front_lens",
        outer_elem="lantern_sill",
        margin=0.01,
        name="beacon lens stays inside lantern footprint",
    )
    ctx.expect_within(
        lighthouse,
        beacon,
        axes="xy",
        inner_elem="beacon_shaft",
        outer_elem="beacon_hub",
        margin=0.001,
        name="beacon hub is centered on shaft",
    )
    ctx.expect_overlap(
        lighthouse,
        beacon,
        axes="z",
        elem_a="beacon_shaft",
        elem_b="beacon_hub",
        min_overlap=0.18,
        name="beacon hub remains captured on shaft",
    )
    for sleeve, label in (("lower_sleeve", "lower"), ("upper_sleeve", "upper")):
        ctx.expect_within(
            lighthouse,
            gate,
            axes="xy",
            inner_elem="gate_hinge_pin",
            outer_elem=sleeve,
            margin=0.001,
            name=f"{label} hinge sleeve is centered on pin",
        )
        ctx.expect_overlap(
            lighthouse,
            gate,
            axes="z",
            elem_a="gate_hinge_pin",
            elem_b=sleeve,
            min_overlap=0.24,
            name=f"{label} hinge sleeve remains captured on pin",
        )

    closed_latch = ctx.part_element_world_aabb(gate, elem="latch_stile")
    closed_lens = ctx.part_element_world_aabb(beacon, elem="front_lens")
    with ctx.pose({gate_joint: 1.10, beacon_joint: math.pi / 2.0}):
        open_latch = ctx.part_element_world_aabb(gate, elem="latch_stile")
        turned_lens = ctx.part_element_world_aabb(beacon, elem="front_lens")

    def _center_x(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) / 2.0

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) / 2.0

    ctx.check(
        "gallery gate swings outward from rail",
        closed_latch is not None
        and open_latch is not None
        and _center_x(open_latch) > _center_x(closed_latch) + 0.25,
        details=f"closed={closed_latch}, open={open_latch}",
    )
    ctx.check(
        "beacon carriage visibly rotates",
        closed_lens is not None
        and turned_lens is not None
        and _center_y(turned_lens) > _center_y(closed_lens) + 0.25
        and _center_x(turned_lens) < _center_x(closed_lens) - 0.20,
        details=f"closed={closed_lens}, turned={turned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
