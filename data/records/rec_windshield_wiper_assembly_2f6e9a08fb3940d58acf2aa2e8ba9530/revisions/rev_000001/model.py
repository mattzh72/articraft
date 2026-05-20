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
)


def _bar_between(part, name, start, end, *, width, thickness, material):
    """Add a flat rectangular linkage/arm member between two local XY points."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, 0.0, yaw),
        ),
        material=material,
        name=name,
    )


def _vertical_eye(part, name, x, y, z, *, radius, thickness, material):
    part.visual(
        Cylinder(radius=radius, length=thickness),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    cowl_black = model.material("satin_black_cowl", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("weathered_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    steel = model.material("zinc_plated_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("darkened_steel", rgba=(0.16, 0.17, 0.17, 1.0))
    motor_black = model.material("motor_black", rgba=(0.025, 0.027, 0.03, 1.0))

    # Root: a full-width automotive cowl pan with fixed motor and spindle housings.
    cowl = model.part("cowl")
    cowl.visual(
        Box((1.55, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cowl_black,
        name="cowl_panel",
    )
    cowl.visual(
        Cylinder(radius=0.025, length=1.55),
        origin=Origin(xyz=(0.0, 0.195, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="windshield_seal",
    )
    cowl.visual(
        Box((1.55, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.182, 0.039)),
        material=rubber,
        name="seal_foot",
    )
    cowl.visual(
        Box((1.55, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, -0.1975, 0.025)),
        material=cowl_black,
        name="front_flange",
    )
    cowl.visual(
        Box((0.24, 0.15, 0.045)),
        origin=Origin(xyz=(0.0, -0.12, 0.0575)),
        material=dark_steel,
        name="motor_pedestal",
    )
    cowl.visual(
        Cylinder(radius=0.085, length=0.055),
        origin=Origin(xyz=(0.0, -0.12, 0.1075)),
        material=motor_black,
        name="motor_can",
    )
    cowl.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, -0.12, 0.144)),
        material=dark_steel,
        name="motor_cap",
    )
    # Stamped support channels make the moving links read as mounted to the cowl.
    _bar_between(
        cowl,
        "link_support_0",
        (-0.52, 0.04, 0.064),
        (0.0, -0.12, 0.064),
        width=0.045,
        thickness=0.026,
        material=dark_steel,
    )
    _bar_between(
        cowl,
        "link_support_1",
        (0.52, 0.04, 0.064),
        (0.0, -0.12, 0.064),
        width=0.045,
        thickness=0.026,
        material=dark_steel,
    )

    spindle_x = (-0.52, 0.52)
    for i, x in enumerate(spindle_x):
        cowl.visual(
            Box((0.20, 0.12, 0.025)),
            origin=Origin(xyz=(x, 0.04, 0.0475)),
            material=dark_steel,
            name=f"spindle_pad_{i}",
        )
        cowl.visual(
            Cylinder(radius=0.045, length=0.085),
            origin=Origin(xyz=(x, 0.04, 0.1025)),
            material=dark_steel,
            name=f"spindle_housing_{i}",
        )
        cowl.visual(
            Cylinder(radius=0.055, length=0.012),
            origin=Origin(xyz=(x, 0.04, 0.151)),
            material=steel,
            name=f"top_washer_{i}",
        )

    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="drive_hub",
    )
    motor_crank.visual(
        Box((0.17, 0.028, 0.012)),
        origin=Origin(xyz=(0.085, 0.0, 0.020)),
        material=steel,
        name="crank_arm",
    )
    motor_crank.visual(
        Cylinder(radius=0.018, length=0.075),
        origin=Origin(xyz=(0.17, 0.0, 0.044)),
        material=steel,
        name="crank_pin",
    )

    crank_joint = model.articulation(
        "cowl_to_motor_crank",
        ArticulationType.CONTINUOUS,
        parent=cowl,
        child=motor_crank,
        origin=Origin(xyz=(0.0, -0.12, 0.153)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    crank_pin_world = (0.17, -0.12)
    drive_pin_world = (-0.41, -0.04)
    drive_dx = drive_pin_world[0] - crank_pin_world[0]
    drive_dy = drive_pin_world[1] - crank_pin_world[1]
    drive_length = math.hypot(drive_dx, drive_dy)
    drive_yaw = math.atan2(drive_dy, drive_dx)

    drive_link = model.part("drive_link")
    _vertical_eye(
        drive_link,
        "crank_eye",
        0.0,
        0.0,
        0.0,
        radius=0.034,
        thickness=0.014,
        material=steel,
    )
    _bar_between(
        drive_link,
        "drive_bar",
        (0.020, 0.0, 0.0),
        (drive_length - 0.020, 0.0, 0.0),
        width=0.024,
        thickness=0.010,
        material=steel,
    )
    _vertical_eye(
        drive_link,
        "spindle_eye",
        drive_length,
        0.0,
        0.0,
        radius=0.034,
        thickness=0.014,
        material=steel,
    )
    model.articulation(
        "motor_crank_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(0.17, 0.0, 0.065), rpy=(0.0, 0.0, drive_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0, lower=-1.4, upper=1.4),
    )

    # Spindles include their splined shafts, bellcranks, and fixed wiper arms.
    spindle_parts = []
    for i, x in enumerate(spindle_x):
        sign = 1.0 if i == 0 else -1.0
        spindle = model.part(f"spindle_{i}")
        spindle.visual(
            Cylinder(radius=0.022, length=0.100),
            origin=Origin(xyz=(0.0, 0.0, 0.050)),
            material=steel,
            name="shaft",
        )
        spindle.visual(
            Cylinder(radius=0.040, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material=steel,
            name="spline_cap",
        )
        drive_or_cross = (0.11 * sign, -0.08, 0.045) if i == 0 else (-0.12, 0.07, 0.045)
        if i == 0:
            _bar_between(
                spindle,
                "bellcrank_drive",
                (0.0, 0.0, 0.045),
                drive_or_cross,
                width=0.030,
                thickness=0.012,
                material=steel,
            )
            spindle.visual(
                Cylinder(radius=0.016, length=0.075),
                origin=Origin(xyz=(0.11, -0.08, 0.061)),
                material=steel,
                name="drive_pin",
            )
            cross_pin = (0.12, 0.07, 0.045)
            _bar_between(
                spindle,
                "bellcrank_cross",
                (0.0, 0.0, 0.045),
                cross_pin,
                width=0.030,
                thickness=0.012,
                material=steel,
            )
            spindle.visual(
                Cylinder(radius=0.016, length=0.075),
                origin=Origin(xyz=(0.12, 0.07, 0.061)),
                material=steel,
                name="cross_pin_0",
            )
        else:
            _bar_between(
                spindle,
                "bellcrank_cross",
                (0.0, 0.0, 0.045),
                drive_or_cross,
                width=0.030,
                thickness=0.012,
                material=steel,
            )
            spindle.visual(
                Cylinder(radius=0.016, length=0.075),
                origin=Origin(xyz=(-0.12, 0.07, 0.061)),
                material=steel,
                name="cross_pin_1",
            )
        spindle.visual(
            Cylinder(radius=0.038, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, 0.105)),
            material=dark_steel,
            name="arm_boss",
        )
        _bar_between(
            spindle,
            "wiper_arm",
            (0.0, 0.02, 0.125),
            (0.0, 0.52, 0.125),
            width=0.030,
            thickness=0.014,
            material=dark_steel,
        )
        spindle.visual(
            Box((0.060, 0.030, 0.018)),
            origin=Origin(xyz=(0.0, 0.535, 0.125)),
            material=dark_steel,
            name="arm_tip_pad",
        )
        spindle.visual(
            Cylinder(radius=0.012, length=0.030),
            origin=Origin(xyz=(0.0, 0.550, 0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="blade_pivot_stud",
        )
        model.articulation(
            f"cowl_to_spindle_{i}",
            ArticulationType.REVOLUTE,
            parent=cowl,
            child=spindle,
            origin=Origin(xyz=(x, 0.04, 0.157)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=15.0, velocity=6.0, lower=-1.15, upper=1.15),
        )
        spindle_parts.append(spindle)

    cross_link = model.part("cross_link")
    cross_length = 0.80
    _vertical_eye(
        cross_link,
        "cross_eye_0",
        0.0,
        0.0,
        0.0,
        radius=0.034,
        thickness=0.014,
        material=steel,
    )
    _bar_between(
        cross_link,
        "cross_bar",
        (0.020, 0.0, 0.0),
        (cross_length - 0.020, 0.0, 0.0),
        width=0.024,
        thickness=0.010,
        material=steel,
    )
    _vertical_eye(
        cross_link,
        "cross_eye_1",
        cross_length,
        0.0,
        0.0,
        radius=0.034,
        thickness=0.014,
        material=steel,
    )
    model.articulation(
        "spindle_0_to_cross_link",
        ArticulationType.REVOLUTE,
        parent=spindle_parts[0],
        child=cross_link,
        origin=Origin(xyz=(0.12, 0.07, 0.061)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0, lower=-1.2, upper=1.2),
    )

    for i, spindle in enumerate(spindle_parts):
        blade = model.part(f"blade_frame_{i}")
        blade.visual(
            Box((0.58, 0.024, 0.018)),
            origin=Origin(xyz=(0.0, 0.012, 0.000)),
            material=dark_steel,
            name="main_bridge",
        )
        blade.visual(
            Box((0.54, 0.012, 0.045)),
            origin=Origin(xyz=(0.0, 0.014, -0.030)),
            material=rubber,
            name="rubber_edge",
        )
        blade.visual(
            Box((0.095, 0.034, 0.030)),
            origin=Origin(xyz=(0.0, 0.025, 0.018)),
            material=steel,
            name="center_saddle",
        )
        for claw_index, sx in enumerate((-0.23, 0.23)):
            blade.visual(
                Box((0.065, 0.030, 0.020)),
                origin=Origin(xyz=(sx, 0.025, 0.014)),
                material=steel,
                name=f"end_claw_{claw_index}",
            )
        model.articulation(
            f"spindle_{i}_to_blade_frame_{i}",
            ArticulationType.REVOLUTE,
            parent=spindle,
            child=blade,
            origin=Origin(xyz=(0.0, 0.565, 0.125)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # The link eyelets are modeled as solid bearing eyes with captured solid
    # pins.  These are local, mechanically intentional pin overlaps.
    pin_overlaps = (
        ("motor_crank", "drive_link", "crank_pin", "crank_eye", "crank pin captured in the drive-link eye"),
        ("spindle_0", "drive_link", "drive_pin", "spindle_eye", "left spindle drive pin captured in the drive-link eye"),
        ("spindle_0", "cross_link", "cross_pin_0", "cross_eye_0", "left spindle cross pin captured in the cross-link eye"),
        ("spindle_1", "cross_link", "cross_pin_1", "cross_eye_1", "right spindle cross pin captured in the cross-link eye"),
    )
    for link_a, link_b, elem_a, elem_b, reason in pin_overlaps:
        ctx.allow_overlap(link_a, link_b, elem_a=elem_a, elem_b=elem_b, reason=reason)
        ctx.expect_overlap(
            link_a,
            link_b,
            elem_a=elem_a,
            elem_b=elem_b,
            axes="xy",
            min_overlap=0.018,
            name=f"{elem_a} is seated in {elem_b}",
        )

    cowl = object_model.get_part("cowl")
    motor_crank = object_model.get_part("motor_crank")
    spindle_0 = object_model.get_part("spindle_0")
    spindle_1 = object_model.get_part("spindle_1")
    drive_link = object_model.get_part("drive_link")
    cross_link = object_model.get_part("cross_link")
    blade_frame_0 = object_model.get_part("blade_frame_0")
    blade_frame_1 = object_model.get_part("blade_frame_1")

    crank_joint = object_model.get_articulation("cowl_to_motor_crank")
    drive_joint = object_model.get_articulation("motor_crank_to_drive_link")
    spindle_joint_0 = object_model.get_articulation("cowl_to_spindle_0")
    spindle_joint_1 = object_model.get_articulation("cowl_to_spindle_1")
    cross_joint = object_model.get_articulation("spindle_0_to_cross_link")
    blade_joint_0 = object_model.get_articulation("spindle_0_to_blade_frame_0")
    blade_joint_1 = object_model.get_articulation("spindle_1_to_blade_frame_1")

    cowl_aabb = ctx.part_world_aabb(cowl)
    ctx.check(
        "automotive cowl width",
        cowl_aabb is not None and (cowl_aabb[1][0] - cowl_aabb[0][0]) > 1.45,
        details=f"cowl_aabb={cowl_aabb}",
    )
    ctx.expect_gap(
        motor_crank,
        cowl,
        axis="z",
        positive_elem="drive_hub",
        negative_elem="motor_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="motor crank hub sits on motor cap",
    )
    for i, spindle in enumerate((spindle_0, spindle_1)):
        ctx.expect_gap(
            spindle,
            cowl,
            axis="z",
            positive_elem="spline_cap",
            negative_elem=f"top_washer_{i}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"spindle_{i} is seated on its housing",
        )

    rest_crank_aabb = ctx.part_world_aabb(motor_crank)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        rotated_crank_aabb = ctx.part_world_aabb(motor_crank)
    ctx.check(
        "motor crank rotates around drive axis",
        rest_crank_aabb is not None
        and rotated_crank_aabb is not None
        and rotated_crank_aabb[1][1] > rest_crank_aabb[1][1] + 0.10,
        details=f"rest={rest_crank_aabb}, rotated={rotated_crank_aabb}",
    )

    rest_drive = ctx.part_world_aabb(drive_link)
    with ctx.pose({drive_joint: 0.45}):
        moved_drive = ctx.part_world_aabb(drive_link)
    ctx.check(
        "drive linkage bar rotates on crank pin",
        rest_drive is not None
        and moved_drive is not None
        and max(abs(moved_drive[1][1] - rest_drive[1][1]), abs(moved_drive[0][1] - rest_drive[0][1])) > 0.06,
        details=f"rest={rest_drive}, moved={moved_drive}",
    )

    rest_cross = ctx.part_world_aabb(cross_link)
    with ctx.pose({cross_joint: 0.35}):
        moved_cross = ctx.part_world_aabb(cross_link)
    ctx.check(
        "cross-car linkage bar rotates on spindle pin",
        rest_cross is not None
        and moved_cross is not None
        and max(abs(moved_cross[1][1] - rest_cross[1][1]), abs(moved_cross[0][1] - rest_cross[0][1])) > 0.06,
        details=f"rest={rest_cross}, moved={moved_cross}",
    )

    for i, (spindle_joint, blade_frame) in enumerate(
        ((spindle_joint_0, blade_frame_0), (spindle_joint_1, blade_frame_1))
    ):
        rest_pos = ctx.part_world_position(blade_frame)
        with ctx.pose({spindle_joint: 0.45}):
            swept_pos = ctx.part_world_position(blade_frame)
        ctx.check(
            f"spindle_{i} sweeps its wiper arm",
            rest_pos is not None and swept_pos is not None and abs(swept_pos[0] - rest_pos[0]) > 0.10,
            details=f"rest={rest_pos}, swept={swept_pos}",
        )

    for i, (blade_joint, blade_frame) in enumerate(((blade_joint_0, blade_frame_0), (blade_joint_1, blade_frame_1))):
        rest_blade = ctx.part_world_aabb(blade_frame)
        with ctx.pose({blade_joint: 0.30}):
            tilted_blade = ctx.part_world_aabb(blade_frame)
        ctx.check(
            f"blade_frame_{i} pivots on arm support",
            rest_blade is not None
            and tilted_blade is not None
            and abs(tilted_blade[0][2] - rest_blade[0][2]) > 0.003,
            details=f"rest={rest_blade}, tilted={tilted_blade}",
        )

    return ctx.report()


object_model = build_object_model()
