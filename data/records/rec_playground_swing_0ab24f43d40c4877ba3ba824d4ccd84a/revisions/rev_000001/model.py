from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)


SWING_LIMIT = math.radians(35.0)


def _cylinder_along_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _hanger_angle(bottom_y: float, bottom_z: float) -> float:
    """Roll angle that aligns a local-Z cylinder from top pivot to bottom point."""
    return math.atan2(bottom_y, -bottom_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tire_gantry_swing")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.62, 1.0))
    dark_pin = model.material("dark_bushed_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    rubber = model.material("weathered_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    top_z = 2.35
    pivot_z = 2.21
    side_x = 1.42
    foot_y = 0.72
    ground_z = 0.06

    gantry = model.part("gantry")
    gantry.visual(
        Cylinder(radius=0.055, length=3.05),
        origin=Origin(xyz=(0.0, 0.0, top_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="top_beam",
    )

    leg_top_z = top_z - 0.02
    leg_len = math.sqrt((leg_top_z - ground_z) ** 2 + foot_y**2)
    leg_roll = math.atan2(foot_y, leg_top_z - ground_z)
    for x in (-side_x, side_x):
        gantry.visual(
            Cylinder(radius=0.043, length=leg_len),
            origin=Origin(
                xyz=(x, foot_y / 2.0, (leg_top_z + ground_z) / 2.0),
                rpy=(leg_roll, 0.0, 0.0),
            ),
            material=galvanized,
            name=f"front_leg_{'0' if x < 0 else '1'}",
        )
        gantry.visual(
            Cylinder(radius=0.043, length=leg_len),
            origin=Origin(
                xyz=(x, -foot_y / 2.0, (leg_top_z + ground_z) / 2.0),
                rpy=(-leg_roll, 0.0, 0.0),
            ),
            material=galvanized,
            name=f"rear_leg_{'0' if x < 0 else '1'}",
        )
        gantry.visual(
            Cylinder(radius=0.040, length=1.64),
            origin=Origin(xyz=(x, 0.0, ground_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"ground_foot_{'0' if x < 0 else '1'}",
        )

    tire_radius = 0.43
    attach_radius = 0.34
    tire_center_z = -1.45
    tire_top_z = tire_center_z + 0.08
    lower_ball_radius = 0.030
    stem_length = 0.035
    bottom_z = tire_top_z + stem_length

    attach_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    attach_points = [
        (attach_radius * math.cos(a), attach_radius * math.sin(a), bottom_z, a)
        for a in attach_angles
    ]
    pivot_xs = [p[0] for p in attach_points]

    # Three visible clevis brackets are welded to the underside of the top beam.
    for i, x in enumerate(pivot_xs):
        gantry.visual(
            Box((0.012, 0.12, 0.15)),
            origin=Origin(xyz=(x - 0.060, 0.0, pivot_z + 0.002)),
            material=galvanized,
            name=f"clevis_plate_{i}_0",
        )
        gantry.visual(
            Box((0.012, 0.12, 0.15)),
            origin=Origin(xyz=(x + 0.060, 0.0, pivot_z + 0.002)),
            material=galvanized,
            name=f"clevis_plate_{i}_1",
        )
        gantry.visual(
            Box((0.16, 0.085, 0.030)),
            origin=Origin(xyz=(x, 0.0, pivot_z + 0.080)),
            material=galvanized,
            name=f"clevis_saddle_{i}",
        )

    hangers = []
    for i, (_, bottom_y, _, _) in enumerate(attach_points):
        hanger = model.part(f"hanger_{i}")
        hangers.append(hanger)

        hanger.visual(
            _cylinder_along_x(radius=0.032, length=0.108),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_pin,
            name="top_barrel",
        )

        rod_len = math.sqrt(bottom_y**2 + bottom_z**2)
        rod_roll = _hanger_angle(bottom_y, bottom_z)
        hanger.visual(
            Cylinder(radius=0.016, length=rod_len),
            origin=Origin(
                xyz=(0.0, bottom_y / 2.0, bottom_z / 2.0),
                rpy=(rod_roll, 0.0, 0.0),
            ),
            material=galvanized,
            name="rigid_link",
        )
        hanger.visual(
            Sphere(radius=lower_ball_radius),
            origin=Origin(xyz=(0.0, bottom_y, bottom_z)),
            material=dark_pin,
            name="lower_knuckle",
        )
        hanger.visual(
            Cylinder(radius=0.012, length=stem_length),
            origin=Origin(xyz=(0.0, bottom_y, bottom_z - stem_length / 2.0)),
            material=dark_pin,
            name="lower_stem",
        )

    tire = model.part("tire")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            tire_radius,
            0.16,
            inner_radius=0.265,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.10),
            tread=TireTread(style="block", depth=0.018, count=28, land_ratio=0.56),
            sidewall=TireSidewall(style="rounded", bulge=0.08),
            shoulder=TireShoulder(width=0.020, radius=0.008),
        ),
        "playground_tire",
    )
    tire.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=rubber,
        name="rubber_tire",
    )
    # Slightly raised pads/eye bosses on the top sidewall make the three
    # hanger-to-tire attachment points explicit without closing the central hole.
    for i, (x, y, _, a) in enumerate(attach_points):
        tire.visual(
            Box((0.11, 0.075, 0.050)),
            origin=Origin(xyz=(x, y, 0.055), rpy=(0.0, 0.0, a)),
            material=dark_pin,
            name=f"lug_{i}",
        )

    main_joint = model.articulation(
        "gantry_to_hanger_0",
        ArticulationType.REVOLUTE,
        parent=gantry,
        child=hangers[0],
        origin=Origin(xyz=(pivot_xs[0], 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=-SWING_LIMIT, upper=SWING_LIMIT),
    )
    for i in (1, 2):
        model.articulation(
            f"gantry_to_hanger_{i}",
            ArticulationType.REVOLUTE,
            parent=gantry,
            child=hangers[i],
            origin=Origin(xyz=(pivot_xs[i], 0.0, pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=-SWING_LIMIT, upper=SWING_LIMIT),
            mimic=Mimic(main_joint.name),
        )

    model.articulation(
        "hanger_0_to_tire",
        ArticulationType.FIXED,
        parent=hangers[0],
        child=tire,
        origin=Origin(xyz=(0.0, 0.0, tire_center_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    gantry = object_model.get_part("gantry")
    tire = object_model.get_part("tire")
    main = object_model.get_articulation("gantry_to_hanger_0")
    top_joints = [object_model.get_articulation(f"gantry_to_hanger_{i}") for i in range(3)]

    for i, joint in enumerate(top_joints):
        limits = joint.motion_limits
        ctx.check(
            f"hanger_{i}_top_revolute_limit",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and limits is not None
            and math.isclose(limits.lower, -SWING_LIMIT, abs_tol=1e-6)
            and math.isclose(limits.upper, SWING_LIMIT, abs_tol=1e-6),
            details=f"joint={joint}",
        )
        ctx.expect_contact(
            f"hanger_{i}",
            gantry,
            elem_a="top_barrel",
            elem_b=f"clevis_plate_{i}_0",
            contact_tol=0.001,
            name=f"hanger_{i}_barrel_seated_in_clevis",
        )
        ctx.expect_contact(
            f"hanger_{i}",
            tire,
            elem_a="lower_stem",
            elem_b=f"lug_{i}",
            contact_tol=0.001,
            name=f"hanger_{i}_lower_stem_seats_on_tire_lug",
        )

    ctx.check(
        "side_hangers_mimic_main_pendulum",
        all(joint.mimic is not None and joint.mimic.joint == main.name for joint in top_joints[1:]),
        details="The two side hangers should follow the central driven swing angle.",
    )

    ctx.expect_gap(
        gantry,
        tire,
        axis="z",
        positive_elem="top_beam",
        negative_elem="rubber_tire",
        min_gap=1.20,
        name="tire_hangs_well_below_top_beam",
    )

    rest_pos = ctx.part_world_position(tire)
    with ctx.pose({main: SWING_LIMIT}):
        forward_pos = ctx.part_world_position(tire)
    with ctx.pose({main: -SWING_LIMIT}):
        rear_pos = ctx.part_world_position(tire)

    ctx.check(
        "tire_swings_forward_at_upper_limit",
        rest_pos is not None
        and forward_pos is not None
        and forward_pos[1] > rest_pos[1] + 0.70
        and forward_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, forward={forward_pos}",
    )
    ctx.check(
        "tire_swings_rearward_at_lower_limit",
        rest_pos is not None
        and rear_pos is not None
        and rear_pos[1] < rest_pos[1] - 0.70
        and rear_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, rear={rear_pos}",
    )

    return ctx.report()


object_model = build_object_model()
