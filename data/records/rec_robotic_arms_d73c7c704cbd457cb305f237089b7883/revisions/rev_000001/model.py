from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


X_AXIS_CYLINDER = (0.0, math.pi / 2.0, 0.0)
Y_AXIS_CYLINDER = (math.pi / 2.0, 0.0, 0.0)


def _cyl_x(part, radius: float, length: float, xyz, *, material: Material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=X_AXIS_CYLINDER),
        material=material,
        name=name,
    )


def _cyl_y(part, radius: float, length: float, xyz, *, material: Material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=Y_AXIS_CYLINDER),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_robot_arm")

    dark_paint = model.material("charcoal_powder_coat", rgba=(0.07, 0.08, 0.08, 1.0))
    safety_orange = model.material("safety_orange_paint", rgba=(0.95, 0.36, 0.06, 1.0))
    black_molded = model.material("black_molded_polymer", rgba=(0.015, 0.015, 0.014, 1.0))
    gunmetal = model.material("dark_gunmetal", rgba=(0.18, 0.19, 0.19, 1.0))
    worn_steel = model.material("zinc_fasteners", rgba=(0.62, 0.62, 0.56, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.02, 0.018, 0.015, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.52, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_paint,
        name="floor_plinth",
    )
    base.visual(
        Box((0.78, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, 0.25, 0.027)),
        material=rubber,
        name="skid_0",
    )
    base.visual(
        Box((0.78, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, -0.25, 0.027)),
        material=rubber,
        name="skid_1",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.224),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=safety_orange,
        name="pedestal_drum",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        material=gunmetal,
        name="pedestal_band",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.3175)),
        material=dark_paint,
        name="turntable_bearing",
    )
    for i in range(8):
        angle = 2.0 * math.pi * i / 8.0
        base.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.29 * math.cos(angle), 0.19 * math.sin(angle), 0.084)),
            material=worn_steel,
            name=f"base_bolt_{i}",
        )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.20, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=gunmetal,
        name="rotary_bearing",
    )
    shoulder.visual(
        Cylinder(radius=0.125, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=safety_orange,
        name="neck_column",
    )
    shoulder.visual(
        Box((0.30, 0.36, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=safety_orange,
        name="yoke_bridge",
    )
    shoulder.visual(
        Box((0.20, 0.07, 0.26)),
        origin=Origin(xyz=(0.02, 0.14, 0.300)),
        material=safety_orange,
        name="shoulder_cheek_0",
    )
    shoulder.visual(
        Box((0.20, 0.07, 0.26)),
        origin=Origin(xyz=(0.02, -0.14, 0.300)),
        material=safety_orange,
        name="shoulder_cheek_1",
    )
    _cyl_y(shoulder, 0.073, 0.040, (0.0, 0.190, 0.300), material=gunmetal, name="axis_cover_0")
    _cyl_y(shoulder, 0.073, 0.040, (0.0, -0.190, 0.300), material=gunmetal, name="axis_cover_1")
    for y in (-0.18, 0.18):
        for z in (0.205, 0.365):
            shoulder.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(0.12, y, z), rpy=X_AXIS_CYLINDER),
                material=worn_steel,
                name=f"shoulder_service_bolt_{'p' if y > 0 else 'n'}_{int(z * 1000)}",
            )

    upper_arm = model.part("upper_arm")
    _cyl_y(upper_arm, 0.075, 0.16, (0.0, 0.0, 0.0), material=gunmetal, name="shoulder_hub")
    upper_arm.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=dark_paint,
        name="shoulder_lug",
    )
    upper_arm.visual(
        Box((0.58, 0.16, 0.15)),
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        material=safety_orange,
        name="upper_link_core",
    )
    upper_arm.visual(
        Box((0.55, 0.18, 0.045)),
        origin=Origin(xyz=(0.39, 0.0, 0.092)),
        material=dark_paint,
        name="upper_link_top_rib",
    )
    upper_arm.visual(
        Box((0.55, 0.18, 0.045)),
        origin=Origin(xyz=(0.39, 0.0, -0.092)),
        material=dark_paint,
        name="upper_link_bottom_rib",
    )
    _cyl_x(upper_arm, 0.050, 0.40, (0.38, 0.0, -0.150), material=black_molded, name="upper_actuator_body")
    upper_arm.visual(
        Box((0.065, 0.12, 0.16)),
        origin=Origin(xyz=(0.18, 0.0, -0.075)),
        material=gunmetal,
        name="upper_actuator_mount_0",
    )
    upper_arm.visual(
        Box((0.065, 0.12, 0.16)),
        origin=Origin(xyz=(0.58, 0.0, -0.075)),
        material=gunmetal,
        name="upper_actuator_mount_1",
    )
    upper_arm.visual(
        Box((0.18, 0.06, 0.22)),
        origin=Origin(xyz=(0.74, 0.13, 0.0)),
        material=safety_orange,
        name="elbow_cheek_0",
    )
    upper_arm.visual(
        Box((0.18, 0.06, 0.22)),
        origin=Origin(xyz=(0.74, -0.13, 0.0)),
        material=safety_orange,
        name="elbow_cheek_1",
    )
    upper_arm.visual(
        Box((0.14, 0.30, 0.050)),
        origin=Origin(xyz=(0.68, 0.0, 0.095)),
        material=dark_paint,
        name="elbow_bridge_top",
    )
    upper_arm.visual(
        Box((0.14, 0.30, 0.050)),
        origin=Origin(xyz=(0.68, 0.0, -0.095)),
        material=dark_paint,
        name="elbow_bridge_bottom",
    )
    for x in (0.22, 0.50):
        for y in (-0.095, 0.095):
            upper_arm.visual(
                Cylinder(radius=0.010, length=0.012),
                origin=Origin(xyz=(x, y, 0.116)),
                material=worn_steel,
                name=f"upper_panel_bolt_{int(x * 100)}_{'p' if y > 0 else 'n'}",
            )

    forearm = model.part("forearm")
    _cyl_y(forearm, 0.065, 0.15, (0.0, 0.0, 0.0), material=gunmetal, name="elbow_hub")
    forearm.visual(
        Box((0.14, 0.14, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=dark_paint,
        name="elbow_lug",
    )
    forearm.visual(
        Box((0.48, 0.055, 0.10)),
        origin=Origin(xyz=(0.31, 0.075, 0.0)),
        material=safety_orange,
        name="forearm_rail_0",
    )
    forearm.visual(
        Box((0.48, 0.055, 0.10)),
        origin=Origin(xyz=(0.31, -0.075, 0.0)),
        material=safety_orange,
        name="forearm_rail_1",
    )
    forearm.visual(
        Box((0.050, 0.20, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        material=dark_paint,
        name="forearm_cross_web_0",
    )
    forearm.visual(
        Box((0.050, 0.20, 0.08)),
        origin=Origin(xyz=(0.48, 0.0, 0.0)),
        material=dark_paint,
        name="forearm_cross_web_1",
    )
    _cyl_x(forearm, 0.040, 0.32, (0.34, 0.0, 0.130), material=black_molded, name="forearm_actuator")
    forearm.visual(
        Box((0.060, 0.18, 0.13)),
        origin=Origin(xyz=(0.20, 0.0, 0.065)),
        material=gunmetal,
        name="forearm_actuator_mount_0",
    )
    forearm.visual(
        Box((0.060, 0.18, 0.13)),
        origin=Origin(xyz=(0.47, 0.0, 0.065)),
        material=gunmetal,
        name="forearm_actuator_mount_1",
    )
    forearm.visual(
        Box((0.14, 0.05, 0.18)),
        origin=Origin(xyz=(0.62, 0.105, 0.0)),
        material=safety_orange,
        name="wrist_cheek_0",
    )
    forearm.visual(
        Box((0.14, 0.05, 0.18)),
        origin=Origin(xyz=(0.62, -0.105, 0.0)),
        material=safety_orange,
        name="wrist_cheek_1",
    )
    for x in (0.20, 0.48, 0.62):
        forearm.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(x, 0.108, 0.0), rpy=Y_AXIS_CYLINDER),
            material=worn_steel,
            name=f"forearm_side_bolt_{int(x * 100)}",
        )

    wrist = model.part("wrist")
    _cyl_y(wrist, 0.055, 0.12, (0.0, 0.0, 0.0), material=gunmetal, name="wrist_hub")
    wrist.visual(
        Box((0.15, 0.13, 0.12)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=dark_paint,
        name="wrist_block",
    )
    _cyl_x(wrist, 0.060, 0.16, (0.10, 0.0, 0.0), material=gunmetal, name="roll_bearing")
    wrist.visual(
        Box((0.09, 0.16, 0.035)),
        origin=Origin(xyz=(0.105, 0.0, 0.075)),
        material=safety_orange,
        name="wrist_top_guard",
    )
    wrist.visual(
        Box((0.09, 0.16, 0.035)),
        origin=Origin(xyz=(0.105, 0.0, -0.075)),
        material=safety_orange,
        name="wrist_bottom_guard",
    )
    for y in (-0.075, 0.075):
        wrist.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(0.12, y, 0.096)),
            material=worn_steel,
            name=f"wrist_guard_bolt_{'p' if y > 0 else 'n'}",
        )

    tool_flange = model.part("tool_flange")
    _cyl_x(tool_flange, 0.045, 0.060, (0.030, 0.0, 0.0), material=gunmetal, name="roll_shaft")
    _cyl_x(tool_flange, 0.090, 0.025, (0.065, 0.0, 0.0), material=dark_paint, name="tool_plate")
    tool_flange.visual(
        Box((0.040, 0.12, 0.08)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=gunmetal,
        name="quick_mount_block",
    )
    for i, (y, z) in enumerate(((0.055, 0.055), (0.055, -0.055), (-0.055, 0.055), (-0.055, -0.055))):
        _cyl_x(tool_flange, 0.009, 0.008, (0.080, y, z), material=worn_steel, name=f"tool_bolt_{i}")

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.4, lower=-3.14, upper=3.14),
        motion_properties=MotionProperties(damping=2.0, friction=0.2),
    )
    model.articulation(
        "shoulder_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.0, lower=-1.05, upper=1.20),
        motion_properties=MotionProperties(damping=2.5, friction=0.25),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.740, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.2, lower=-2.15, upper=2.15),
        motion_properties=MotionProperties(damping=2.0, friction=0.2),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=2.0, lower=-1.65, upper=1.65),
        motion_properties=MotionProperties(damping=1.2, friction=0.1),
    )
    model.articulation(
        "wrist_to_tool_flange",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=tool_flange,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.8, lower=-3.14, upper=3.14),
        motion_properties=MotionProperties(damping=0.8, friction=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    tool_flange = object_model.get_part("tool_flange")

    base_yaw = object_model.get_articulation("base_to_shoulder")
    shoulder_pitch = object_model.get_articulation("shoulder_to_upper_arm")
    elbow_pitch = object_model.get_articulation("upper_arm_to_forearm")
    wrist_pitch = object_model.get_articulation("forearm_to_wrist")
    wrist_roll = object_model.get_articulation("wrist_to_tool_flange")

    serial_joints = (base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll)
    ctx.check(
        "five explicit rugged arm joints",
        len(serial_joints) == 5
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in serial_joints),
        details=f"joints={[j.name for j in serial_joints]}",
    )

    ctx.expect_gap(
        shoulder,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        elem_a="rotary_bearing",
        elem_b="turntable_bearing",
        name="rotary bearing seats on the base turntable",
    )
    ctx.expect_overlap(
        shoulder,
        base,
        axes="xy",
        min_overlap=0.25,
        elem_a="rotary_bearing",
        elem_b="turntable_bearing",
        name="base turntable supports the shoulder bearing footprint",
    )
    ctx.expect_within(
        upper_arm,
        shoulder,
        axes="y",
        margin=0.003,
        inner_elem="shoulder_lug",
        outer_elem="yoke_bridge",
        name="upper arm lug stays inside shoulder yoke width",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="y",
        margin=0.006,
        inner_elem="elbow_lug",
        outer_elem="elbow_bridge_top",
        name="forearm elbow lug stays inside upper-arm fork width",
    )

    upper_pos = ctx.part_world_position(upper_arm)
    forearm_pos = ctx.part_world_position(forearm)
    wrist_pos = ctx.part_world_position(wrist)
    flange_pos = ctx.part_world_position(tool_flange)
    ctx.check(
        "straight pose has spaced serial axes",
        upper_pos is not None
        and forearm_pos is not None
        and wrist_pos is not None
        and flange_pos is not None
        and forearm_pos[0] > upper_pos[0] + 0.68
        and wrist_pos[0] > forearm_pos[0] + 0.56
        and flange_pos[0] > wrist_pos[0] + 0.16,
        details=f"upper={upper_pos}, forearm={forearm_pos}, wrist={wrist_pos}, flange={flange_pos}",
    )

    rest_wrist = wrist_pos
    with ctx.pose({shoulder_pitch: 0.50}):
        raised_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "positive shoulder pitch raises the distal arm",
        rest_wrist is not None and raised_wrist is not None and raised_wrist[2] > rest_wrist[2] + 0.25,
        details=f"rest_wrist={rest_wrist}, raised_wrist={raised_wrist}",
    )

    rest_flange = flange_pos
    with ctx.pose({base_yaw: 0.65}):
        swept_flange = ctx.part_world_position(tool_flange)
    ctx.check(
        "base yaw sweeps the arm horizontally",
        rest_flange is not None and swept_flange is not None and abs(swept_flange[1]) > 0.55,
        details=f"rest_flange={rest_flange}, swept_flange={swept_flange}",
    )

    return ctx.report()


object_model = build_object_model()
