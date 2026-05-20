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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_robotic_arm")

    yellow = model.material("powder_coated_yellow", rgba=(0.92, 0.62, 0.10, 1.0))
    dark = model.material("black_oxide_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    steel = model.material("brushed_service_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    rubber = model.material("replaceable_black_wear", rgba=(0.015, 0.014, 0.012, 1.0))
    orange = model.material("safety_orange_access", rgba=(0.95, 0.28, 0.05, 1.0))
    blue = model.material("sealed_cable_blue", rgba=(0.02, 0.11, 0.22, 1.0))

    def cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
        return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    def cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
        return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    # Root pedestal: a serviceable welded base with a column, inspection cover,
    # cable raceway, and a large top bearing face for the yaw cartridge.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.42, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="floor_plate",
    )
    for i in range(8):
        a = i * math.tau / 8.0
        pedestal.visual(
            Cylinder(radius=0.024, length=0.016),
            origin=Origin(xyz=(0.33 * math.cos(a), 0.33 * math.sin(a), 0.084)),
            material=steel,
            name=f"anchor_bolt_{i}",
        )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=yellow,
        name="main_column",
    )
    pedestal.visual(
        Cylinder(radius=0.22, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.69)),
        material=yellow,
        name="neck_sleeve",
    )
    pedestal.visual(
        Cylinder(radius=0.34, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=steel,
        name="top_bearing",
    )
    for x, y, sx, sy, nm in (
        (0.19, 0.00, 0.07, 0.18, "rib_0"),
        (-0.19, 0.00, 0.07, 0.18, "rib_1"),
        (0.00, 0.19, 0.18, 0.07, "rib_2"),
        (0.00, -0.19, 0.18, 0.07, "rib_3"),
    ):
        pedestal.visual(
            Box((sx, sy, 0.48)),
            origin=Origin(xyz=(x, y, 0.32)),
            material=yellow,
            name=nm,
        )
    pedestal.visual(
        Box((0.12, 0.026, 0.24)),
        origin=Origin(xyz=(0.0, -0.192, 0.39)),
        material=orange,
        name="access_door",
    )
    pedestal.visual(
        Box((0.07, 0.070, 0.52)),
        origin=Origin(xyz=(0.0, 0.205, 0.38)),
        material=blue,
        name="rear_cable_raceway",
    )

    # Yaw cartridge: a rotating turntable with a shoulder yoke.  The child frame
    # sits exactly at the yaw bearing plane; all pitch hardware is offset upward.
    shoulder = model.part("shoulder_cartridge")
    shoulder.visual(
        Cylinder(radius=0.32, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=steel,
        name="yaw_turntable",
    )
    for i in range(10):
        a = i * math.tau / 10.0
        shoulder.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.255 * math.cos(a), 0.255 * math.sin(a), 0.106)),
            material=dark,
            name=f"turntable_bolt_{i}",
        )
    shoulder.visual(
        Box((0.60, 0.46, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=yellow,
        name="yaw_gearbox",
    )
    shoulder.visual(
        Box((0.25, 0.10, 0.42)),
        origin=Origin(xyz=(0.0, 0.27, 0.47)),
        material=yellow,
        name="shoulder_cheek_0",
    )
    shoulder.visual(
        Box((0.25, 0.10, 0.42)),
        origin=Origin(xyz=(0.0, -0.27, 0.47)),
        material=yellow,
        name="shoulder_cheek_1",
    )
    for y, nm in ((0.327, "outer_bearing_0"), (-0.327, "outer_bearing_1")):
        geom, rot = cyl_y(0.175, 0.018)
        shoulder.visual(
            geom,
            origin=Origin(xyz=(0.0, y, 0.47), rpy=rot.rpy),
            material=dark,
            name=nm,
        )
    shoulder.visual(
        Box((0.22, 0.020, 0.16)),
        origin=Origin(xyz=(0.18, -0.241, 0.45)),
        material=orange,
        name="shoulder_service_cover",
    )
    shoulder.visual(
        Box((0.12, 0.40, 0.035)),
        origin=Origin(xyz=(-0.22, 0.0, 0.275)),
        material=blue,
        name="shoulder_cable_bridge",
    )

    model.articulation(
        "pedestal_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=650.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )

    # Upper arm: parallel box-section rails, removable covers, and an elbow fork.
    upper = model.part("upper_arm")
    geom, rot = cyl_y(0.135, 0.36)
    upper.visual(geom, origin=rot, material=dark, name="shoulder_hub")
    for y, nm in ((0.200, "shoulder_washer_0"), (-0.200, "shoulder_washer_1")):
        geom, rot = cyl_y(0.155, 0.040)
        upper.visual(geom, origin=Origin(xyz=(0.0, y, 0.0), rpy=rot.rpy), material=steel, name=nm)
    for y, nm in ((0.16, "upper_rail_0"), (-0.16, "upper_rail_1")):
        upper.visual(
            Box((0.76, 0.07, 0.12)),
            origin=Origin(xyz=(0.39, y, 0.02)),
            material=yellow,
            name=nm,
        )
    upper.visual(
        Box((0.50, 0.31, 0.04)),
        origin=Origin(xyz=(0.42, 0.0, 0.10)),
        material=orange,
        name="upper_access_cover",
    )
    upper.visual(
        Box((0.55, 0.31, 0.04)),
        origin=Origin(xyz=(0.44, 0.0, -0.06)),
        material=blue,
        name="upper_cable_tray",
    )
    for y, nm in ((0.165, "elbow_cheek_0"), (-0.165, "elbow_cheek_1")):
        upper.visual(
            Box((0.18, 0.07, 0.30)),
            origin=Origin(xyz=(0.86, y, 0.0)),
            material=yellow,
            name=nm,
        )
        geom, rot = cyl_y(0.135, 0.014)
        upper.visual(
            geom,
            origin=Origin(xyz=(0.86, math.copysign(0.207, y), 0.0), rpy=rot.rpy),
            material=dark,
            name=f"{nm}_cap",
        )
    for x in (0.24, 0.58):
        upper.visual(
            Box((0.06, 0.37, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.04)),
            material=rubber,
            name=f"upper_wear_band_{int(x * 100)}",
        )

    model.articulation(
        "shoulder_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.8, lower=-1.10, upper=1.35),
    )

    # Forearm: smaller parallel rails retained between the elbow fork plates and
    # carrying a second pitch fork for the wrist cartridge.
    forearm = model.part("forearm")
    geom, rot = cyl_y(0.105, 0.21)
    forearm.visual(geom, origin=rot, material=dark, name="elbow_hub")
    for y, nm in ((0.1175, "elbow_washer_0"), (-0.1175, "elbow_washer_1")):
        geom, rot = cyl_y(0.085, 0.025)
        forearm.visual(geom, origin=Origin(xyz=(0.0, y, 0.0), rpy=rot.rpy), material=steel, name=nm)
    for y, nm in ((0.13, "forearm_rail_0"), (-0.13, "forearm_rail_1")):
        forearm.visual(
            Box((0.54, 0.06, 0.09)),
            origin=Origin(xyz=(0.36, y, 0.0)),
            material=yellow,
            name=nm,
        )
    forearm.visual(
        Box((0.42, 0.25, 0.035)),
        origin=Origin(xyz=(0.38, 0.0, 0.061)),
        material=orange,
        name="forearm_access_cover",
    )
    forearm.visual(
        Box((0.44, 0.25, 0.035)),
        origin=Origin(xyz=(0.40, 0.0, -0.060)),
        material=blue,
        name="forearm_cable_tray",
    )
    for y, nm in ((0.13, "wrist_cheek_0"), (-0.13, "wrist_cheek_1")):
        forearm.visual(
            Box((0.14, 0.06, 0.22)),
            origin=Origin(xyz=(0.70, y, 0.0)),
            material=yellow,
            name=nm,
        )
        geom, rot = cyl_y(0.100, 0.012)
        forearm.visual(
            geom,
            origin=Origin(xyz=(0.70, math.copysign(0.166, y), 0.0), rpy=rot.rpy),
            material=dark,
            name=f"{nm}_cap",
        )

    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.86, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.9, lower=-1.35, upper=1.95),
    )

    # Wrist pitch cartridge: short, clear, and offset forward from the forearm
    # fork so the subsequent roll axis is obvious.
    wrist_pitch = model.part("wrist_pitch")
    geom, rot = cyl_y(0.078, 0.16)
    wrist_pitch.visual(geom, origin=rot, material=dark, name="pitch_hub")
    for y, nm in ((0.085, "pitch_washer_0"), (-0.085, "pitch_washer_1")):
        geom, rot = cyl_y(0.090, 0.030)
        wrist_pitch.visual(geom, origin=Origin(xyz=(0.0, y, 0.0), rpy=rot.rpy), material=steel, name=nm)
    geom, rot = cyl_x(0.075, 0.24)
    wrist_pitch.visual(
        geom,
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=rot.rpy),
        material=yellow,
        name="roll_motor_sleeve",
    )
    wrist_pitch.visual(
        Box((0.14, 0.13, 0.040)),
        origin=Origin(xyz=(0.12, 0.0, 0.090)),
        material=orange,
        name="wrist_service_lid",
    )
    wrist_pitch.visual(
        Box((0.11, 0.13, 0.030)),
        origin=Origin(xyz=(0.10, 0.0, -0.083)),
        material=blue,
        name="wrist_cable_exit",
    )

    model.articulation(
        "forearm_to_wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch,
        origin=Origin(xyz=(0.70, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.4, lower=-1.55, upper=1.55),
    )

    # Roll cartridge and quick-change plate.  This final part rotates about the
    # forward X axis and carries the bolted tool face.
    wrist_roll = model.part("wrist_roll")
    geom, rot = cyl_x(0.075, 0.08)
    wrist_roll.visual(
        geom,
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=rot.rpy),
        material=dark,
        name="roll_bearing",
    )
    geom, rot = cyl_x(0.090, 0.16)
    wrist_roll.visual(
        geom,
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=rot.rpy),
        material=yellow,
        name="roll_cartridge",
    )
    geom, rot = cyl_x(0.140, 0.035)
    wrist_roll.visual(
        geom,
        origin=Origin(xyz=(0.195, 0.0, 0.0), rpy=rot.rpy),
        material=steel,
        name="tool_flange",
    )
    wrist_roll.visual(
        Box((0.080, 0.26, 0.26)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=dark,
        name="quick_change_plate",
    )
    for i, (y, z) in enumerate(((0.090, 0.090), (-0.090, 0.090), (0.090, -0.090), (-0.090, -0.090))):
        geom, rot = cyl_x(0.013, 0.016)
        wrist_roll.visual(
            geom,
            origin=Origin(xyz=(0.293, y, z), rpy=rot.rpy),
            material=steel,
            name=f"tool_bolt_{i}",
        )
    wrist_roll.visual(
        Box((0.030, 0.20, 0.035)),
        origin=Origin(xyz=(0.296, 0.0, 0.070)),
        material=rubber,
        name="replaceable_tool_rail_0",
    )
    wrist_roll.visual(
        Box((0.030, 0.20, 0.035)),
        origin=Origin(xyz=(0.296, 0.0, -0.070)),
        material=rubber,
        name="replaceable_tool_rail_1",
    )

    model.articulation(
        "wrist_pitch_to_wrist_roll",
        ArticulationType.REVOLUTE,
        parent=wrist_pitch,
        child=wrist_roll,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=2.2, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    shoulder = object_model.get_part("shoulder_cartridge")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_pitch = object_model.get_part("wrist_pitch")
    wrist_roll = object_model.get_part("wrist_roll")

    yaw = object_model.get_articulation("pedestal_to_shoulder")
    shoulder_pitch = object_model.get_articulation("shoulder_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_pitch_joint = object_model.get_articulation("forearm_to_wrist_pitch")
    wrist_roll_joint = object_model.get_articulation("wrist_pitch_to_wrist_roll")

    ctx.expect_gap(
        shoulder,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="yaw_turntable",
        negative_elem="top_bearing",
        name="yaw cartridge seats on pedestal bearing",
    )
    ctx.expect_overlap(
        shoulder,
        pedestal,
        axes="xy",
        min_overlap=0.40,
        elem_a="yaw_turntable",
        elem_b="top_bearing",
        name="yaw bearing footprints overlap",
    )

    expected_axes = {
        "pedestal_to_shoulder": (0.0, 0.0, 1.0),
        "shoulder_to_upper_arm": (0.0, -1.0, 0.0),
        "upper_arm_to_forearm": (0.0, -1.0, 0.0),
        "forearm_to_wrist_pitch": (0.0, -1.0, 0.0),
        "wrist_pitch_to_wrist_roll": (1.0, 0.0, 0.0),
    }
    for joint_name, expected in expected_axes.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} axis is explicit",
            tuple(round(v, 6) for v in joint.axis) == expected,
            details=f"axis={joint.axis}, expected={expected}",
        )

    rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({yaw: 0.65}):
        yawed_elbow_pos = ctx.part_world_position(forearm)
    ctx.check(
        "yaw sweeps elbow around pedestal",
        rest_elbow_pos is not None
        and yawed_elbow_pos is not None
        and yawed_elbow_pos[1] > rest_elbow_pos[1] + 0.25,
        details=f"rest={rest_elbow_pos}, yawed={yawed_elbow_pos}",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_pitch)
    with ctx.pose({shoulder_pitch: 0.55, elbow: 0.35}):
        raised_wrist_pos = ctx.part_world_position(wrist_pitch)
    ctx.check(
        "pitch joints raise the wrist in positive motion",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.25,
        details=f"rest={rest_wrist_pos}, raised={raised_wrist_pos}",
    )

    rest_tool_pos = ctx.part_world_position(wrist_roll)
    with ctx.pose({wrist_pitch_joint: 0.7}):
        pitched_tool_pos = ctx.part_world_position(wrist_roll)
    ctx.check(
        "wrist pitch offsets the roll cartridge path",
        rest_tool_pos is not None
        and pitched_tool_pos is not None
        and pitched_tool_pos[2] > rest_tool_pos[2] + 0.10,
        details=f"rest={rest_tool_pos}, pitched={pitched_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
