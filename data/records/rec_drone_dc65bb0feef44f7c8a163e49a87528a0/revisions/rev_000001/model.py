from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _regular_polygon(radius: float, sides: int, *, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + 2.0 * math.pi * i / sides),
            radius * math.sin(phase + 2.0 * math.pi * i / sides),
        )
        for i in range(sides)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="agricultural_sprayer_hexacopter")

    carbon = Material("matte_carbon", rgba=(0.015, 0.018, 0.018, 1.0))
    dark = Material("black_composite", rgba=(0.01, 0.01, 0.012, 1.0))
    metal = Material("brushed_metal", rgba=(0.55, 0.58, 0.56, 1.0))
    yellow = Material("sprayer_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    tank_plastic = Material("translucent_tank", rgba=(0.82, 0.92, 0.88, 0.68))
    nozzle_blue = Material("nozzle_blue", rgba=(0.05, 0.22, 0.9, 1.0))

    frame = model.part("frame")

    frame_plate = ExtrudeWithHolesGeometry(
        _regular_polygon(0.38, 6, phase=math.pi / 6.0),
        [
            [
                (0.17 * math.cos(a) + p[0], 0.17 * math.sin(a) + p[1])
                for p in _circle_profile(0.033, 18)
            ]
            for a in [2.0 * math.pi * i / 6.0 for i in range(6)]
        ],
        0.035,
        center=True,
    )
    frame.visual(
        mesh_from_geometry(frame_plate, "hex_lightened_frame"),
        origin=Origin(),
        material=carbon,
        name="hex_frame_plate",
    )

    arm_length = 0.92
    motor_radius = 0.90
    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        frame.visual(
            Box((arm_length, 0.070, 0.045)),
            origin=Origin(
                xyz=(0.5 * arm_length * c, 0.5 * arm_length * s, 0.010),
                rpy=(0.0, 0.0, angle),
            ),
            material=carbon,
            name=f"motor_arm_{i}",
        )
        frame.visual(
            Cylinder(radius=0.082, length=0.105),
            origin=Origin(xyz=(motor_radius * c, motor_radius * s, 0.055)),
            material=dark,
            name=f"motor_pod_{i}",
        )
        frame.visual(
            Cylinder(radius=0.060, length=0.012),
            origin=Origin(xyz=(motor_radius * c, motor_radius * s, 0.1135)),
            material=metal,
            name=f"motor_cap_{i}",
        )

    frame.visual(
        Box((0.32, 0.22, 0.09)),
        origin=Origin(xyz=(-0.03, 0.0, 0.075)),
        material=yellow,
        name="top_battery_case",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.07, 0.0, 0.1375)),
        material=metal,
        name="gps_cap",
    )

    for x in (-0.29, 0.29):
        frame.visual(
            Box((0.055, 0.95, 0.040)),
            origin=Origin(xyz=(x, 0.0, -0.035)),
            material=dark,
            name=f"skid_crossbar_{0 if x < 0 else 1}",
        )
        for y in (-0.42, 0.42):
            frame.visual(
                Box((0.040, 0.040, 0.63)),
                origin=Origin(xyz=(x, y, -0.370)),
                material=dark,
                name=f"landing_strut_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
    for y in (-0.45, 0.45):
        frame.visual(
            Box((1.05, 0.055, 0.040)),
            origin=Origin(xyz=(0.0, y, -0.705)),
            material=dark,
            name=f"skid_rail_{0 if y < 0 else 1}",
        )

    tank = model.part("tank")
    tank_shape = (
        cq.Workplane("XY")
        .box(0.62, 0.46, 0.32)
        .edges()
        .fillet(0.045)
    )
    tank.visual(
        mesh_from_cadquery(tank_shape, "rounded_chemical_tank", tolerance=0.002),
        origin=Origin(),
        material=tank_plastic,
        name="tank_body",
    )
    tank.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(-0.16, 0.0, 0.1775)),
        material=yellow,
        name="filler_cap",
    )
    for x in (-0.23, 0.23):
        for y in (-0.14, 0.14):
            tank.visual(
                Box((0.038, 0.030, 0.195)),
                origin=Origin(xyz=(x, y, 0.155)),
                material=metal,
                name=f"tank_hanger_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
    tank.visual(
        Box((0.18, 0.18, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        material=metal,
        name="center_bracket",
    )
    tank.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
        material=metal,
        name="hinge_pin",
    )
    tank.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.347)),
        material=metal,
        name="pin_head",
    )

    model.articulation(
        "frame_to_tank",
        ArticulationType.FIXED,
        parent=frame,
        child=tank,
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(),
        material=metal,
        name="boom_hub",
    )
    boom_half_len = 0.86
    for side, y in (("0", 0.47), ("1", -0.47)):
        boom.visual(
            Box((0.045, boom_half_len, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=yellow,
            name=f"boom_bar_{side}",
        )
        boom.visual(
            Box((0.020, boom_half_len, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.0235)),
            material=dark,
            name=f"hose_line_{side}",
        )
    nozzle_mesh = mesh_from_geometry(
        ConeGeometry(0.018, 0.035, radial_segments=18),
        "spray_nozzle",
    )
    for idx, y in enumerate((-0.78, -0.53, -0.28, 0.28, 0.53, 0.78)):
        boom.visual(
            Cylinder(radius=0.0065, length=0.055),
            origin=Origin(xyz=(0.0, y, -0.0415)),
            material=metal,
            name=f"nozzle_stem_{idx}",
        )
        boom.visual(
            nozzle_mesh,
            origin=Origin(xyz=(0.0, y, -0.082)),
            material=nozzle_blue,
            name=f"spray_nozzle_{idx}",
        )

    model.articulation(
        "tank_to_boom",
        ArticulationType.REVOLUTE,
        parent=tank,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, -0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=math.pi / 2.0),
    )

    prop_geometry = FanRotorGeometry(
        0.34,
        0.055,
        2,
        thickness=0.026,
        blade_pitch_deg=18.0,
        blade_sweep_deg=27.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=10.0, camber=0.10),
        hub=FanRotorHub(style="spinner", bore_diameter=0.010),
    )
    prop_mesh = mesh_from_geometry(prop_geometry, "broad_sprayer_propeller")

    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        prop = model.part(f"propeller_{i}")
        prop.visual(prop_mesh, origin=Origin(), material=dark, name="rotor")
        prop.visual(
            Cylinder(radius=0.042, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, -0.0130)),
            material=dark,
            name="shaft_collar",
        )
        model.articulation(
            f"propeller_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=prop,
            origin=Origin(
                xyz=(motor_radius * math.cos(angle), motor_radius * math.sin(angle), 0.145)
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=80.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tank = object_model.get_part("tank")
    boom = object_model.get_part("boom")
    boom_joint = object_model.get_articulation("tank_to_boom")

    ctx.allow_overlap(
        tank,
        boom,
        elem_a="hinge_pin",
        elem_b="boom_hub",
        reason="The folding boom hub is intentionally captured around the vertical hinge pin.",
    )
    ctx.expect_overlap(
        boom,
        tank,
        axes="xy",
        min_overlap=0.025,
        elem_a="boom_hub",
        elem_b="hinge_pin",
        name="boom hub surrounds hinge pin in plan",
    )
    ctx.expect_overlap(
        boom,
        tank,
        axes="z",
        min_overlap=0.025,
        elem_a="boom_hub",
        elem_b="hinge_pin",
        name="boom hub is axially retained on hinge pin",
    )

    continuous = [
        object_model.get_articulation(f"propeller_{i}_spin").articulation_type
        for i in range(6)
    ]
    ctx.check(
        "six continuous propeller joints",
        all(joint_type == ArticulationType.CONTINUOUS for joint_type in continuous),
        details=f"joint types={continuous}",
    )

    with ctx.pose({boom_joint: 0.0}):
        deployed_aabb = ctx.part_world_aabb(boom)
    with ctx.pose({boom_joint: math.pi / 2.0}):
        folded_aabb = ctx.part_world_aabb(boom)

    if deployed_aabb is not None and folded_aabb is not None:
        dep_min, dep_max = deployed_aabb
        fol_min, fol_max = folded_aabb
        dep_x = dep_max[0] - dep_min[0]
        dep_y = dep_max[1] - dep_min[1]
        fol_x = fol_max[0] - fol_min[0]
        fol_y = fol_max[1] - fol_min[1]
        ctx.check(
            "boom deploys as wide spray bar",
            dep_y > 1.60 and dep_x < 0.16,
            details=f"deployed extents x={dep_x:.3f}, y={dep_y:.3f}",
        )
        ctx.check(
            "boom folds on center hinge",
            fol_x > 1.60 and fol_y < 0.16,
            details=f"folded extents x={fol_x:.3f}, y={fol_y:.3f}",
        )
    else:
        ctx.fail("boom pose aabbs available", "boom AABBs could not be measured")

    return ctx.report()


object_model = build_object_model()
