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
)


def _annular_guard_plate(outer_radius: float, inner_radius: float, depth: float, x_center: float):
    """A flat circular guard ring lying in the YZ plane and centered on x_center."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((x_center - depth / 2.0, 0.0, 0.0))
    )


def _oval_base():
    """A broad, low desk-fan base footprint in the XY plane."""
    return cq.Workplane("XY").ellipse(0.22, 0.17).extrude(0.045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.038, 0.042, 1.0))
    black_wire = model.material("black_wire", rgba=(0.006, 0.006, 0.007, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    blade_blue = model.material("translucent_blue_blades", rgba=(0.36, 0.67, 0.95, 0.78))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_oval_base(), "oval_base", tolerance=0.0015),
        material=dark_plastic,
        name="oval_base",
    )
    stand.visual(
        Cylinder(radius=0.035, length=0.530),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=dark_plastic,
        name="tall_mast",
    )
    stand.visual(
        Cylinder(radius=0.025, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.545), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="yoke_crossbar",
    )
    for index, y, boss_name in ((0, -0.235, "pivot_boss_0"), (1, 0.235, "pivot_boss_1")):
        stand.visual(
            Box((0.055, 0.035, 0.330)),
            origin=Origin(xyz=(0.0, y, 0.710)),
            material=dark_plastic,
            name=f"side_bracket_{index}",
        )
        stand.visual(
            Cylinder(radius=0.045, length=0.024),
            origin=Origin(xyz=(0.0, math.copysign(0.214, y), 0.760), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=boss_name,
        )

    head = model.part("head")
    rear_x = -0.005
    fan_x = 0.055
    front_x = 0.115
    outer_radius = 0.182
    inner_guard_radius = 0.168

    head.visual(
        Cylinder(radius=0.070, length=0.130),
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_nose",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.342),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_axle",
    )
    for y, trunnion_name in ((-0.184, "trunnion_0"), (0.184, "trunnion_1")):
        head.visual(
            Cylinder(radius=0.023, length=0.030),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=trunnion_name,
        )

    ring_specs = (
        ("rear_outer_ring", rear_x, outer_radius, inner_guard_radius),
        ("rear_middle_ring", rear_x, 0.132, 0.126),
        ("rear_inner_ring", rear_x, 0.092, 0.086),
        ("front_outer_ring", front_x, outer_radius, inner_guard_radius),
        ("front_middle_ring", front_x, 0.132, 0.126),
        ("front_inner_ring", front_x, 0.092, 0.086),
    )
    for ring_name, x, outer, inner in ring_specs:
        head.visual(
            mesh_from_cadquery(
                _annular_guard_plate(outer, inner, 0.012, x),
                ring_name,
                tolerance=0.0012,
            ),
            material=black_wire,
            name=ring_name,
        )

    head.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(front_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_wire,
        name="front_badge",
    )

    radial_inner = 0.043
    radial_outer = 0.174
    radial_mid = (radial_inner + radial_outer) / 2.0
    radial_len = radial_outer - radial_inner
    for face_index, x in enumerate((rear_x, front_x)):
        for spoke_index in range(12):
            theta = spoke_index * math.tau / 12.0
            head.visual(
                Box((0.010, 0.006, radial_len)),
                origin=Origin(
                    xyz=(x, radial_mid * math.sin(theta), radial_mid * math.cos(theta)),
                    rpy=(-theta, 0.0, 0.0),
                ),
                material=black_wire,
                name=f"guard_spoke_{face_index}_{spoke_index}",
            )

    for index, (y, z) in enumerate(((0.0, outer_radius), (outer_radius, 0.0), (0.0, -outer_radius), (-outer_radius, 0.0))):
        head.visual(
            Cylinder(radius=0.0045, length=front_x - rear_x),
            origin=Origin(xyz=(fan_x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_wire,
            name=f"cage_tie_{index}",
        )

    rotor = model.part("rotor")
    rotor_geometry = FanRotorGeometry(
        outer_radius=0.145,
        hub_radius=0.035,
        blade_count=5,
        thickness=0.032,
        blade_pitch_deg=33.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.13, tip_clearance=0.004),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.028, bore_diameter=0.010),
    )
    rotor.visual(
        mesh_from_geometry(rotor_geometry, "fan_blades"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_blue,
        name="fan_blades",
    )
    rotor.visual(
        Cylinder(radius=0.010, length=0.055),
        origin=Origin(xyz=(-0.0125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="rotor_shaft",
    )

    model.articulation(
        "stand_to_head",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.50, upper=0.55),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(fan_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("stand_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.check("head tilt is horizontal revolute", tilt.articulation_type == ArticulationType.REVOLUTE and tilt.axis == (0.0, 1.0, 0.0))
    ctx.check("rotor spin is continuous axle", spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0))

    rest_rotor_position = ctx.part_world_position(rotor)
    ctx.allow_overlap(
        head,
        rotor,
        elem_a="motor_nose",
        elem_b="rotor_shaft",
        reason="The rotating fan shaft is intentionally captured inside the motor nose bearing.",
    )
    ctx.check(
        "rotor is high above base",
        rest_rotor_position is not None and rest_rotor_position[2] > 0.72,
        details=f"rotor_position={rest_rotor_position}",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="rotor_shaft",
        outer_elem="motor_nose",
        margin=0.0,
        name="shaft is centered in motor bearing",
    )
    ctx.expect_overlap(
        rotor,
        head,
        axes="x",
        elem_a="rotor_shaft",
        elem_b="motor_nose",
        min_overlap=0.015,
        name="shaft is inserted in motor bearing",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="fan_blades",
        outer_elem="front_outer_ring",
        margin=0.0,
        name="rotor fits inside circular guard",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        positive_elem="front_outer_ring",
        negative_elem="fan_blades",
        min_gap=0.020,
        name="front guard clears spinning rotor",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="fan_blades",
        negative_elem="rear_outer_ring",
        min_gap=0.020,
        name="rear guard clears spinning rotor",
    )
    ctx.expect_gap(
        stand,
        head,
        axis="y",
        positive_elem="pivot_boss_1",
        negative_elem="trunnion_1",
        min_gap=0.0,
        max_gap=0.006,
        name="side trunnion sits beside bracket boss",
    )

    with ctx.pose({tilt: 0.45}):
        down_position = ctx.part_world_position(rotor)
    with ctx.pose({tilt: -0.40}):
        up_position = ctx.part_world_position(rotor)
    ctx.check(
        "head tilts about side brackets",
        (
            rest_rotor_position is not None
            and down_position is not None
            and up_position is not None
            and down_position[2] < rest_rotor_position[2] - 0.015
            and up_position[2] > rest_rotor_position[2] + 0.015
        ),
        details=f"rest={rest_rotor_position}, down={down_position}, up={up_position}",
    )

    return ctx.report()


object_model = build_object_model()
