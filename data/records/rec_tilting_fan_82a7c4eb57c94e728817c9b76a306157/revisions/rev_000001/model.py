from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def _add_guard_side(
    head,
    *,
    x: float,
    prefix: str,
    material: Material,
    outer_radius: float = 0.180,
    inner_radius: float = 0.038,
) -> None:
    """Add one axial guard grille face in the YZ plane."""

    for ring_radius, tube, suffix in (
        (outer_radius, 0.0060, "outer_ring"),
        (0.132, 0.0038, "middle_ring"),
        (0.085, 0.0032, "inner_ring"),
    ):
        head.visual(
            mesh_from_geometry(
                TorusGeometry(ring_radius, tube, radial_segments=16, tubular_segments=96),
                f"{prefix}_{suffix}",
            ),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=f"{prefix}_{suffix}",
        )

    # The central badge is stationary and gives the guard spokes a real hub to
    # terminate into rather than floating at the middle of the cage.
    head.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_center_badge",
    )

    spoke_count = 16
    spoke_radius = 0.0028
    spoke_length = outer_radius - inner_radius + 0.014
    spoke_mid = (outer_radius + inner_radius) / 2.0
    for i in range(spoke_count):
        theta = 2.0 * pi * i / spoke_count
        head.visual(
            Cylinder(radius=spoke_radius, length=spoke_length),
            origin=Origin(
                xyz=(x, spoke_mid * cos(theta), spoke_mid * sin(theta)),
                rpy=(theta - pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_spoke_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    plastic = _mat("satin_ivory_plastic", (0.82, 0.80, 0.72, 1.0))
    dark = _mat("dark_rubber", (0.035, 0.036, 0.038, 1.0))
    guard_mat = _mat("black_powder_coated_wire", (0.025, 0.027, 0.030, 1.0))
    metal = _mat("brushed_bearing_metal", (0.52, 0.54, 0.56, 1.0))
    rotor_mat = _mat("translucent_smoke_blades", (0.34, 0.48, 0.62, 0.78))

    base = model.part("base")

    base_foot = ExtrudeGeometry(
        superellipse_profile(0.46, 0.34, exponent=2.65, segments=72),
        0.050,
        center=True,
    )
    base.visual(
        mesh_from_geometry(base_foot, "base_foot"),
        origin=Origin(xyz=(-0.030, 0.0, 0.025)),
        material=plastic,
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.215),
        origin=Origin(xyz=(-0.040, 0.0, 0.155)),
        material=plastic,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.072, length=0.030),
        origin=Origin(xyz=(-0.040, 0.0, 0.060)),
        material=plastic,
        name="pedestal_lower_collar",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.035),
        origin=Origin(xyz=(-0.040, 0.0, 0.270)),
        material=plastic,
        name="pedestal_upper_collar",
    )
    base.visual(
        Box((0.145, 0.510, 0.080)),
        origin=Origin(xyz=(-0.010, 0.0, 0.260)),
        material=plastic,
        name="yoke_bridge",
    )

    for sign, idx in ((1.0, 0), (-1.0, 1)):
        y = sign * 0.255
        base.visual(
            Box((0.085, 0.045, 0.430)),
            origin=Origin(xyz=(0.0, y, 0.315)),
            material=plastic,
            name=f"side_support_{idx}",
        )
        # Large bearing collar around the tilt axis.  The torus hole leaves
        # visible clearance for the moving trunnion shaft.
        base.visual(
            mesh_from_geometry(
                TorusGeometry(0.043, 0.010, radial_segments=18, tubular_segments=72),
                f"side_bearing_{idx}",
            ),
            origin=Origin(xyz=(0.0, sign * 0.224, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"side_bearing_{idx}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.046),
            origin=Origin(xyz=(0.0, sign * 0.255, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"bearing_bolt_{idx}",
        )

    # Low, soft feet make the scale read like a tabletop fan.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            base.visual(
                Cylinder(radius=0.030, length=0.010),
                origin=Origin(xyz=(-0.030 + sx * 0.150, sy * 0.105, 0.005)),
                material=dark,
                name=f"rubber_foot_{int(sx > 0)}_{int(sy > 0)}",
            )

    rotor_center_x = 0.110

    head = model.part("head")
    _add_guard_side(head, x=rotor_center_x + 0.066, prefix="front_guard", material=guard_mat)
    _add_guard_side(head, x=rotor_center_x - 0.066, prefix="rear_guard", material=guard_mat)

    for i in range(8):
        theta = 2.0 * pi * i / 8.0 + pi / 8.0
        head.visual(
            Cylinder(radius=0.0038, length=0.145),
            origin=Origin(
                xyz=(rotor_center_x, 0.180 * cos(theta), 0.180 * sin(theta)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=guard_mat,
            name=f"cage_rib_{i}",
        )

    # Rear motor canister, axle, and trunnion hardware are all part of the
    # tilting head rather than the freely spinning rotor.
    head.visual(
        Cylinder(radius=0.075, length=0.128),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plastic,
        name="motor_canister",
    )
    head.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plastic,
        name="rear_motor_boss",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="rotor_axle",
    )
    head.visual(
        Cylinder(radius=0.035, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="tilt_shaft",
    )
    for sign, idx in ((1.0, 0), (-1.0, 1)):
        head.visual(
            Cylinder(radius=0.037, length=0.028),
            origin=Origin(xyz=(0.0, sign * 0.195, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"trunnion_cap_{idx}",
        )

    rotor = model.part("rotor")
    fan_rotor = FanRotorGeometry(
        outer_radius=0.142,
        hub_radius=0.036,
        blade_count=5,
        thickness=0.028,
        blade_pitch_deg=33.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18, tip_clearance=0.006),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.030, bore_diameter=0.018),
    )
    rotor.visual(
        mesh_from_geometry(fan_rotor, "rotor_blades"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rotor_mat,
        name="rotor_blades",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        # Positive travel tilts the fan face upward about the side-bracket axis.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.35, upper=0.55),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(rotor_center_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.check(
        "head tilt axis is through side brackets",
        tilt.axis == (0.0, -1.0, 0.0),
        details=f"tilt axis={tilt.axis}",
    )
    ctx.check(
        "rotor has continuous axle spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.allow_overlap(
        base,
        head,
        elem_a="side_bearing_0",
        elem_b="tilt_shaft",
        reason="The thick trunnion shaft is intentionally captured inside the side bearing bushing.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="side_bearing_1",
        elem_b="tilt_shaft",
        reason="The thick trunnion shaft is intentionally captured inside the side bearing bushing.",
    )
    ctx.allow_overlap(
        head,
        rotor,
        elem_a="rotor_axle",
        elem_b="rotor_blades",
        reason="The drive shaft is intentionally seated through the rotor hub bore.",
    )

    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="rotor_blades",
        outer_elem="front_guard_outer_ring",
        margin=0.006,
        name="rotor disk fits inside the circular guard",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        positive_elem="front_guard_outer_ring",
        negative_elem="rotor_blades",
        min_gap=0.020,
        name="front guard stands ahead of rotor",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="rotor_blades",
        negative_elem="rear_guard_outer_ring",
        min_gap=0.020,
        name="rear guard stands behind rotor",
    )
    ctx.expect_within(
        head,
        base,
        axes="xz",
        inner_elem="tilt_shaft",
        outer_elem="side_bearing_0",
        margin=0.002,
        name="tilt shaft is centered in side bearing",
    )
    ctx.expect_within(
        head,
        base,
        axes="xz",
        inner_elem="tilt_shaft",
        outer_elem="side_bearing_1",
        margin=0.002,
        name="tilt shaft is centered in opposite bearing",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="y",
        elem_a="tilt_shaft",
        elem_b="side_bearing_0",
        min_overlap=0.006,
        name="tilt shaft remains inserted in side bearing",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="y",
        elem_a="tilt_shaft",
        elem_b="side_bearing_1",
        min_overlap=0.006,
        name="tilt shaft remains inserted in opposite bearing",
    )
    ctx.expect_within(
        head,
        rotor,
        axes="yz",
        inner_elem="rotor_axle",
        outer_elem="rotor_blades",
        margin=0.001,
        name="drive shaft is centered in rotor hub",
    )
    ctx.expect_overlap(
        head,
        rotor,
        axes="x",
        elem_a="rotor_axle",
        elem_b="rotor_blades",
        min_overlap=0.020,
        name="rotor hub remains on the drive shaft",
    )

    closed = ctx.part_element_world_aabb(head, elem="front_guard_outer_ring")
    with ctx.pose({tilt: 0.45}):
        raised = ctx.part_element_world_aabb(head, elem="front_guard_outer_ring")
    closed_z = (closed[0][2] + closed[1][2]) / 2.0 if closed else None
    raised_z = (raised[0][2] + raised[1][2]) / 2.0 if raised else None
    ctx.check(
        "positive tilt raises the fan face",
        closed_z is not None and raised_z is not None and raised_z > closed_z + 0.015,
        details=f"closed_z={closed_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
