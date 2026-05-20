from __future__ import annotations

import math

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _superellipse_section(
    x: float,
    half_width: float,
    half_height: float,
    *,
    z_center: float = 0.0,
    exponent: float = 2.4,
    samples: int = 36,
) -> list[tuple[float, float, float]]:
    section: list[tuple[float, float, float]] = []
    for i in range(samples):
        angle = 2.0 * math.pi * i / samples
        c = math.cos(angle)
        s = math.sin(angle)
        y = math.copysign(abs(c) ** (2.0 / exponent), c) * half_width
        z = math.copysign(abs(s) ** (2.0 / exponent), s) * half_height + z_center
        section.append((x, y, z))
    return section


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_blimp")

    hull_white = model.material("hull_white", rgba=(0.92, 0.94, 0.95, 1.0))
    hull_grey = model.material("hull_grey", rgba=(0.76, 0.79, 0.82, 1.0))
    structure_grey = model.material("structure_grey", rgba=(0.29, 0.31, 0.34, 1.0))
    nacelle_grey = model.material("nacelle_grey", rgba=(0.23, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.70, 0.73, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    canopy_glass = model.material("canopy_glass", rgba=(0.58, 0.72, 0.82, 0.38))
    sensor_black = model.material("sensor_black", rgba=(0.10, 0.11, 0.12, 1.0))

    envelope = model.part("envelope")
    envelope_shell = section_loft(
        [
            _superellipse_section(15.8, 0.20, 0.18, exponent=2.0),
            _superellipse_section(13.6, 1.45, 1.32, exponent=2.15),
            _superellipse_section(8.0, 2.95, 2.72, exponent=2.3),
            _superellipse_section(1.0, 3.35, 3.05, exponent=2.45),
            _superellipse_section(-6.0, 3.05, 2.86, exponent=2.35),
            _superellipse_section(-11.6, 2.10, 1.98, exponent=2.2),
            _superellipse_section(-14.7, 0.82, 0.76, exponent=2.0),
            _superellipse_section(-15.95, 0.10, 0.10, exponent=2.0),
        ]
    )
    envelope.visual(
        mesh_from_geometry(envelope_shell, "envelope_shell"),
        material=hull_white,
        name="envelope_shell",
    )
    envelope.visual(
        Box((2.4, 1.0, 0.45)),
        origin=Origin(xyz=(2.0, 0.0, -3.20)),
        material=hull_grey,
        name="keel_pad",
    )
    envelope.visual(
        Box((1.9, 0.9, 0.75)),
        origin=Origin(xyz=(-15.05, 0.0, 0.0)),
        material=hull_grey,
        name="tail_mount",
    )

    gondola = model.part("gondola")
    cabin_shell = section_loft(
        [
            _superellipse_section(2.65, 0.16, 0.22, z_center=-1.03, exponent=2.2),
            _superellipse_section(1.90, 0.60, 0.68, z_center=-1.03, exponent=2.35),
            _superellipse_section(0.55, 0.95, 0.82, z_center=-1.05, exponent=2.7),
            _superellipse_section(-1.00, 0.88, 0.76, z_center=-1.08, exponent=2.65),
            _superellipse_section(-2.15, 0.42, 0.42, z_center=-1.06, exponent=2.2),
        ]
    )
    gondola.visual(
        mesh_from_geometry(cabin_shell, "gondola_shell"),
        material=hull_grey,
        name="cabin_shell",
    )
    canopy_shell = section_loft(
        [
            _superellipse_section(1.85, 0.48, 0.48, z_center=-0.96, exponent=2.4),
            _superellipse_section(1.15, 0.68, 0.56, z_center=-0.96, exponent=2.7),
            _superellipse_section(0.30, 0.52, 0.45, z_center=-1.00, exponent=2.6),
        ]
    )
    gondola.visual(
        mesh_from_geometry(canopy_shell, "gondola_canopy"),
        material=canopy_glass,
        name="canopy",
    )
    gondola.visual(
        Box((1.3, 0.82, 0.22)),
        origin=Origin(xyz=(0.35, 0.0, -0.11)),
        material=hull_grey,
        name="roof_block",
    )
    for sy in (-1.0, 1.0):
        _add_member(
            gondola,
            (0.62, 0.22 * sy, -0.22),
            (1.00, 0.54 * sy, -0.72),
            radius=0.07,
            material=structure_grey,
        )
        _add_member(
            gondola,
            (-0.82, 0.22 * sy, -0.22),
            (-1.18, 0.54 * sy, -0.70),
            radius=0.07,
            material=structure_grey,
        )
    gondola.visual(
        Box((0.40, 0.36, 0.30)),
        origin=Origin(xyz=(-1.35, 0.92, -0.62)),
        material=structure_grey,
        name="left_mount_pad",
    )
    gondola.visual(
        Box((0.40, 0.36, 0.30)),
        origin=Origin(xyz=(-1.35, -0.92, -0.62)),
        material=structure_grey,
        name="right_mount_pad",
    )
    gondola.visual(
        Cylinder(radius=0.10, length=0.18),
        origin=Origin(xyz=(1.72, 0.0, -1.72)),
        material=structure_grey,
        name="sensor_collar",
    )
    gondola.visual(
        Cylinder(radius=0.06, length=0.28),
        origin=Origin(xyz=(1.72, 0.0, -1.88)),
        material=structure_grey,
    )
    gondola.visual(
        Sphere(radius=0.14),
        origin=Origin(xyz=(1.72, 0.0, -2.10)),
        material=sensor_black,
        name="sensor_bulb",
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(2.0, 0.0, -3.425)),
    )

    rotor_geom = FanRotorGeometry(
        0.78,
        0.11,
        3,
        thickness=0.10,
        blade_pitch_deg=25.0,
        blade_sweep_deg=18.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.16),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.05, rear_collar_radius=0.09),
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        pylon = model.part(f"{side_name}_pylon")
        pylon.visual(
            Box((0.42, 0.26, 0.28)),
            origin=Origin(xyz=(-0.02, 0.13 * side_sign, -0.06)),
            material=structure_grey,
            name="root_block",
        )
        _add_member(
            pylon,
            (-0.06, 0.17 * side_sign, 0.00),
            (-0.58, 0.96 * side_sign, 0.10),
            radius=0.045,
            material=structure_grey,
            name="upper_arm",
        )
        _add_member(
            pylon,
            (-0.14, 0.13 * side_sign, 0.08),
            (-0.60, 0.92 * side_sign, 0.12),
            radius=0.055,
            material=structure_grey,
            name="lower_brace",
        )
        pylon.visual(
            Box((0.20, 0.18, 0.14)),
            origin=Origin(xyz=(-0.64, 1.02 * side_sign, -0.06)),
            material=structure_grey,
            name="tip_block",
        )
        pylon.visual(
            Box((0.20, 0.18, 0.14)),
            origin=Origin(xyz=(-0.54, 0.84 * side_sign, 0.08)),
            material=structure_grey,
            name="tip_connector",
        )
        pylon.visual(
            Cylinder(radius=0.075, length=0.20),
            origin=Origin(xyz=(-0.64, 1.02 * side_sign, -0.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="pivot_fork",
        )
        model.articulation(
            f"gondola_to_{side_name}_pylon",
            ArticulationType.FIXED,
            parent=gondola,
            child=pylon,
            origin=Origin(xyz=(-1.35, 1.10 * side_sign, -0.62)),
        )

        engine = model.part(f"{side_name}_engine")
        pod_shell = section_loft(
            [
                _superellipse_section(1.02, 0.07, 0.09, z_center=-0.28, exponent=2.0),
                _superellipse_section(0.62, 0.21, 0.18, z_center=-0.28, exponent=2.25),
                _superellipse_section(-0.10, 0.28, 0.23, z_center=-0.28, exponent=2.5),
                _superellipse_section(-0.90, 0.21, 0.17, z_center=-0.29, exponent=2.3),
                _superellipse_section(-1.18, 0.08, 0.07, z_center=-0.29, exponent=2.0),
            ]
        )
        engine.visual(
            mesh_from_geometry(pod_shell, f"{side_name}_engine_shell"),
            material=nacelle_grey,
            name="pod_shell",
        )
        engine.visual(
            Cylinder(radius=0.07, length=0.26),
            origin=Origin(xyz=(-0.02, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="trunnion",
        )
        engine.visual(
            Cylinder(radius=0.05, length=0.12),
            origin=Origin(xyz=(1.08, 0.0, -0.28), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="nose_spindle",
        )
        engine.visual(
            Cylinder(radius=0.055, length=0.18),
            origin=Origin(xyz=(-1.12, 0.0, -0.29), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="exhaust",
        )
        model.articulation(
            f"{side_name}_vector",
            ArticulationType.REVOLUTE,
            parent=pylon,
            child=engine,
            origin=Origin(xyz=(-0.64, 1.02 * side_sign, -0.06)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=1.8,
                lower=-1.0,
                upper=0.7,
            ),
        )

        propeller = model.part(f"{side_name}_propeller")
        propeller.visual(
            Cylinder(radius=0.05, length=0.22),
            origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="shaft",
        )
        propeller.visual(
            mesh_from_geometry(rotor_geom, f"{side_name}_rotor"),
            origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_black,
            name="rotor",
        )
        model.articulation(
            f"{side_name}_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=engine,
            child=propeller,
            origin=Origin(xyz=(1.14, 0.0, -0.28)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=28.0),
        )

    tailplane = model.part("tailplane")
    tailplane.visual(
        Box((0.40, 0.70, 0.18)),
        origin=Origin(xyz=(-0.20, 0.0, 0.0)),
        material=structure_grey,
        name="root_box",
    )
    tailplane.visual(
        Box((1.10, 2.20, 0.12)),
        origin=Origin(xyz=(-0.55, 1.45, 0.0)),
        material=hull_white,
        name="left_stabilizer",
    )
    tailplane.visual(
        Box((1.10, 2.20, 0.12)),
        origin=Origin(xyz=(-0.55, -1.45, 0.0)),
        material=hull_white,
        name="right_stabilizer",
    )
    tailplane.visual(
        Cylinder(radius=0.06, length=2.20),
        origin=Origin(xyz=(-0.07, 1.45, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hull_white,
    )
    tailplane.visual(
        Cylinder(radius=0.06, length=2.20),
        origin=Origin(xyz=(-0.07, -1.45, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hull_white,
    )
    model.articulation(
        "envelope_to_tailplane",
        ArticulationType.FIXED,
        parent=envelope,
        child=tailplane,
        origin=Origin(xyz=(-16.00, 0.0, 0.0)),
    )

    fin = model.part("fin")
    fin.visual(
        Box((0.38, 0.55, 0.24)),
        origin=Origin(xyz=(-0.19, 0.0, 0.12)),
        material=structure_grey,
        name="fin_root",
    )
    fin.visual(
        Box((1.04, 0.14, 2.55)),
        origin=Origin(xyz=(-0.52, 0.0, 1.275)),
        material=hull_white,
        name="fin_panel",
    )
    fin.visual(
        Cylinder(radius=0.07, length=2.55),
        origin=Origin(xyz=(-0.03, 0.0, 1.275)),
        material=hull_white,
        name="fin_leading_edge",
    )
    model.articulation(
        "envelope_to_fin",
        ArticulationType.FIXED,
        parent=envelope,
        child=fin,
        origin=Origin(xyz=(-16.00, 0.0, 0.375)),
    )

    rudder = model.part("rudder")
    rudder.visual(
        Box((0.40, 0.10, 2.10)),
        origin=Origin(xyz=(-0.25, 0.0, 1.05)),
        material=hull_white,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.05, length=2.10),
        origin=Origin(xyz=(-0.05, 0.0, 1.05)),
        material=structure_grey,
        name="rudder_leading_edge",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=fin,
        child=rudder,
        origin=Origin(xyz=(-1.04, 0.0, 0.15)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.6,
            upper=0.6,
        ),
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Box((0.40, 2.20, 0.10)),
        origin=Origin(xyz=(-0.25, 1.10, 0.0)),
        material=hull_white,
        name="elevator_panel",
    )
    left_elevator.visual(
        Cylinder(radius=0.05, length=2.20),
        origin=Origin(xyz=(-0.05, 1.10, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structure_grey,
        name="elevator_leading_edge",
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=tailplane,
        child=left_elevator,
        origin=Origin(xyz=(-1.10, 0.35, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.45,
        ),
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Box((0.40, 2.20, 0.10)),
        origin=Origin(xyz=(-0.25, -1.10, 0.0)),
        material=hull_white,
        name="elevator_panel",
    )
    right_elevator.visual(
        Cylinder(radius=0.05, length=2.20),
        origin=Origin(xyz=(-0.05, -1.10, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structure_grey,
        name="elevator_leading_edge",
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=tailplane,
        child=right_elevator,
        origin=Origin(xyz=(-1.10, -0.35, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    tailplane = object_model.get_part("tailplane")
    fin = object_model.get_part("fin")
    rudder = object_model.get_part("rudder")
    left_pylon = object_model.get_part("left_pylon")
    right_pylon = object_model.get_part("right_pylon")
    left_engine = object_model.get_part("left_engine")
    right_engine = object_model.get_part("right_engine")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    left_vector = object_model.get_articulation("left_vector")
    right_vector = object_model.get_articulation("right_vector")
    rudder_hinge = object_model.get_articulation("rudder_hinge")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")
    right_elevator_hinge = object_model.get_articulation("right_elevator_hinge")

    ctx.allow_overlap(
        left_pylon,
        left_engine,
        reason="The left nacelle fairing and trunnion are intentionally nested into the pylon's vectoring mount.",
    )
    ctx.allow_overlap(
        left_pylon,
        left_engine,
        elem_a="pivot_fork",
        elem_b="trunnion",
        reason="The left vectoring nacelle rides on a trunnion nested inside the pylon fork.",
    )
    ctx.allow_overlap(
        left_pylon,
        left_engine,
        elem_a="tip_block",
        elem_b="trunnion",
        reason="The left trunnion is housed inside the pylon tip block that carries the vectoring pivot.",
    )
    ctx.allow_overlap(
        right_pylon,
        right_engine,
        reason="The right nacelle fairing and trunnion are intentionally nested into the pylon's vectoring mount.",
    )
    ctx.allow_overlap(
        right_pylon,
        right_engine,
        elem_a="pivot_fork",
        elem_b="trunnion",
        reason="The right vectoring nacelle rides on a trunnion nested inside the pylon fork.",
    )
    ctx.allow_overlap(
        right_pylon,
        right_engine,
        elem_a="tip_block",
        elem_b="trunnion",
        reason="The right trunnion is housed inside the pylon tip block that carries the vectoring pivot.",
    )

    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="keel_pad",
        negative_elem="roof_block",
        max_gap=0.001,
        max_penetration=1e-6,
        name="gondola roof seats against the hull keel",
    )
    ctx.expect_overlap(
        gondola,
        envelope,
        axes="xy",
        elem_a="roof_block",
        elem_b="keel_pad",
        min_overlap=0.70,
        name="gondola roof stays centered under the keel pad",
    )
    ctx.expect_contact(
        left_pylon,
        gondola,
        elem_a="root_block",
        elem_b="left_mount_pad",
        name="left pylon mounts to the gondola side pad",
    )
    ctx.expect_contact(
        right_pylon,
        gondola,
        elem_a="root_block",
        elem_b="right_mount_pad",
        name="right pylon mounts to the gondola side pad",
    )
    ctx.expect_overlap(
        left_engine,
        left_pylon,
        axes="yz",
        elem_a="trunnion",
        elem_b="pivot_fork",
        min_overlap=0.14,
        name="left engine trunnion stays carried by the pylon fork",
    )
    ctx.expect_overlap(
        right_engine,
        right_pylon,
        axes="yz",
        elem_a="trunnion",
        elem_b="pivot_fork",
        min_overlap=0.14,
        name="right engine trunnion stays carried by the pylon fork",
    )
    ctx.expect_origin_gap(
        gondola,
        left_engine,
        axis="x",
        min_gap=1.8,
        name="left engine sits aft of the gondola frame",
    )
    ctx.expect_origin_gap(
        gondola,
        right_engine,
        axis="x",
        min_gap=1.8,
        name="right engine sits aft of the gondola frame",
    )
    ctx.expect_origin_distance(
        left_engine,
        right_engine,
        axes="y",
        min_dist=3.8,
        name="engine nacelles sit on opposite sides of the gondola",
    )
    ctx.expect_gap(
        left_propeller,
        left_engine,
        axis="x",
        positive_elem="shaft",
        negative_elem="nose_spindle",
        max_gap=0.001,
        max_penetration=1e-6,
        name="left propeller shaft meets the nacelle spindle",
    )
    ctx.expect_gap(
        right_propeller,
        right_engine,
        axis="x",
        positive_elem="shaft",
        negative_elem="nose_spindle",
        max_gap=0.001,
        max_penetration=1e-6,
        name="right propeller shaft meets the nacelle spindle",
    )

    ctx.expect_gap(
        envelope,
        tailplane,
        axis="x",
        positive_elem="tail_mount",
        negative_elem="root_box",
        max_gap=0.001,
        max_penetration=0.0,
        name="tailplane seats on the tail mount rear face",
    )
    ctx.expect_gap(
        envelope,
        fin,
        axis="x",
        positive_elem="tail_mount",
        negative_elem="fin_root",
        max_gap=0.001,
        max_penetration=0.0,
        name="fin seats on the tail mount rear face",
    )
    ctx.expect_gap(
        fin,
        rudder,
        axis="x",
        positive_elem="fin_panel",
        negative_elem="rudder_leading_edge",
        max_gap=0.001,
        max_penetration=0.0,
        name="rudder aligns on the fin trailing hinge line",
    )
    ctx.expect_overlap(
        rudder,
        fin,
        axes="z",
        elem_a="rudder_panel",
        elem_b="fin_panel",
        min_overlap=1.9,
        name="rudder remains tall and distinct from the fin",
    )
    ctx.expect_gap(
        tailplane,
        left_elevator,
        axis="x",
        positive_elem="left_stabilizer",
        negative_elem="elevator_leading_edge",
        max_gap=0.001,
        max_penetration=0.0,
        name="left elevator aligns on the tailplane hinge line",
    )
    ctx.expect_gap(
        tailplane,
        right_elevator,
        axis="x",
        positive_elem="right_stabilizer",
        negative_elem="elevator_leading_edge",
        max_gap=0.001,
        max_penetration=0.0,
        name="right elevator aligns on the tailplane hinge line",
    )

    left_prop_rest = ctx.part_world_position(left_propeller)
    right_prop_rest = ctx.part_world_position(right_propeller)
    with ctx.pose(
        {
            left_vector: left_vector.motion_limits.upper,
            right_vector: right_vector.motion_limits.upper,
        }
    ):
        left_prop_up = ctx.part_world_position(left_propeller)
        right_prop_up = ctx.part_world_position(right_propeller)

    ctx.check(
        "left vectoring pod can pitch the propeller upward",
        left_prop_rest is not None
        and left_prop_up is not None
        and left_prop_up[2] > left_prop_rest[2] + 0.10,
        details=f"rest={left_prop_rest}, up={left_prop_up}",
    )
    ctx.check(
        "right vectoring pod can pitch the propeller upward",
        right_prop_rest is not None
        and right_prop_up is not None
        and right_prop_up[2] > right_prop_rest[2] + 0.10,
        details=f"rest={right_prop_rest}, up={right_prop_up}",
    )

    rudder_rest = _aabb_center(ctx.part_element_world_aabb(rudder, elem="rudder_panel"))
    with ctx.pose({rudder_hinge: rudder_hinge.motion_limits.upper}):
        rudder_deflected = _aabb_center(ctx.part_element_world_aabb(rudder, elem="rudder_panel"))
    ctx.check(
        "rudder swings laterally about the fin hinge",
        rudder_rest is not None
        and rudder_deflected is not None
        and rudder_deflected[1] > rudder_rest[1] + 0.05,
        details=f"rest={rudder_rest}, deflected={rudder_deflected}",
    )

    left_elevator_rest = _aabb_center(
        ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
    )
    right_elevator_rest = _aabb_center(
        ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
    )
    with ctx.pose(
        {
            left_elevator_hinge: left_elevator_hinge.motion_limits.upper,
            right_elevator_hinge: right_elevator_hinge.motion_limits.upper,
        }
    ):
        left_elevator_up = _aabb_center(
            ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
        )
        right_elevator_up = _aabb_center(
            ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
        )
    ctx.check(
        "left elevator deflects upward",
        left_elevator_rest is not None
        and left_elevator_up is not None
        and left_elevator_up[2] > left_elevator_rest[2] + 0.03,
        details=f"rest={left_elevator_rest}, up={left_elevator_up}",
    )
    ctx.check(
        "right elevator deflects upward",
        right_elevator_rest is not None
        and right_elevator_up is not None
        and right_elevator_up[2] > right_elevator_rest[2] + 0.03,
        details=f"rest={right_elevator_rest}, up={right_elevator_up}",
    )

    return ctx.report()


object_model = build_object_model()
