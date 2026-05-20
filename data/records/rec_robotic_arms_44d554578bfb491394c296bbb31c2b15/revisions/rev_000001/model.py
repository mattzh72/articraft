from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_robotic_arm")

    cast_dark = Material("dark_cast_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    molded_gray = Material("molded_gray_plastic", rgba=(0.55, 0.58, 0.57, 1.0))
    extrusion = Material("clear_anodized_extrusion", rgba=(0.72, 0.74, 0.72, 1.0))
    cartridge_blue = Material("blue_joint_cartridges", rgba=(0.08, 0.20, 0.55, 1.0))
    black = Material("black_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))
    safety_orange = Material("orange_axis_marks", rgba=(1.0, 0.43, 0.05, 1.0))

    y_cyl = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    x_cyl = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    base = model.part("base_pedestal")
    base.visual(Box((0.62, 0.48, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=cast_dark, name="stamped_foot")
    base.visual(Cylinder(0.285, 0.052), origin=Origin(xyz=(0.0, 0.0, 0.026)), material=cast_dark, name="low_base_pan")
    base.visual(Cylinder(0.155, 0.205), origin=Origin(xyz=(0.0, 0.0, 0.140)), material=molded_gray, name="pedestal_shell")
    base.visual(Cylinder(0.190, 0.045), origin=Origin(xyz=(0.0, 0.0, 0.265)), material=cartridge_blue, name="yaw_stator_ring")
    base.visual(Box((0.170, 0.110, 0.035)), origin=Origin(xyz=(-0.245, 0.0, 0.048)), material=cast_dark, name="rear_cable_trough")
    for i, (x, y) in enumerate(((-0.235, -0.165), (-0.235, 0.165), (0.235, -0.165), (0.235, 0.165))):
        base.visual(Cylinder(0.034, 0.026), origin=Origin(xyz=(x, y, 0.043)), material=cast_dark, name=f"floor_boss_{i}")
        base.visual(Cylinder(0.020, 0.007), origin=Origin(xyz=(x, y, 0.057)), material=black, name=f"floor_bolt_{i}")

    shoulder = model.part("shoulder_module")
    shoulder.visual(Cylinder(0.165, 0.055), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=cartridge_blue, name="yaw_rotor")
    shoulder.visual(Box((0.220, 0.220, 0.225)), origin=Origin(xyz=(0.020, 0.0, 0.160)), material=molded_gray, name="one_piece_column")
    shoulder.visual(Box((0.132, 0.038, 0.166)), origin=Origin(xyz=(0.184, 0.100, 0.270)), material=molded_gray, name="shoulder_yoke_0")
    shoulder.visual(Box((0.132, 0.038, 0.166)), origin=Origin(xyz=(0.184, -0.100, 0.270)), material=molded_gray, name="shoulder_yoke_1")
    shoulder.visual(Box((0.165, 0.255, 0.028)), origin=Origin(xyz=(0.184, 0.0, 0.347)), material=molded_gray, name="top_snap_bridge")
    shoulder.visual(Cylinder(0.065, 0.043), origin=Origin(xyz=(0.200, 0.074, 0.270), rpy=y_cyl.rpy), material=cartridge_blue, name="shoulder_cap_0")
    shoulder.visual(Cylinder(0.065, 0.043), origin=Origin(xyz=(0.200, -0.074, 0.270), rpy=y_cyl.rpy), material=cartridge_blue, name="shoulder_cap_1")
    for i, y in enumerate((0.116, -0.116)):
        shoulder.visual(Cylinder(0.012, 0.008), origin=Origin(xyz=(0.168, y, 0.315), rpy=y_cyl.rpy), material=black, name=f"shoulder_bolt_{i}_0")
        shoulder.visual(Cylinder(0.012, 0.008), origin=Origin(xyz=(0.232, y, 0.225), rpy=y_cyl.rpy), material=black, name=f"shoulder_bolt_{i}_1")
    shoulder.visual(Box((0.100, 0.012, 0.012)), origin=Origin(xyz=(0.200, 0.0, 0.358)), material=safety_orange, name="shoulder_axis_mark")

    upper = model.part("upper_arm")
    upper.visual(Cylinder(0.055, 0.105), origin=Origin(rpy=y_cyl.rpy), material=cartridge_blue, name="shoulder_inner_hub")
    upper.visual(Box((0.395, 0.100, 0.074)), origin=Origin(xyz=(0.232, 0.0, 0.0)), material=extrusion, name="upper_extrusion")
    upper.visual(Box((0.340, 0.052, 0.026)), origin=Origin(xyz=(0.250, 0.0, 0.050)), material=molded_gray, name="upper_snap_rib")
    upper.visual(Box((0.130, 0.035, 0.135)), origin=Origin(xyz=(0.482, 0.065, 0.0)), material=extrusion, name="elbow_yoke_0")
    upper.visual(Box((0.130, 0.035, 0.135)), origin=Origin(xyz=(0.482, -0.065, 0.0)), material=extrusion, name="elbow_yoke_1")
    upper.visual(Cylinder(0.052, 0.027), origin=Origin(xyz=(0.510, 0.061, 0.0), rpy=y_cyl.rpy), material=cartridge_blue, name="elbow_cap_0")
    upper.visual(Cylinder(0.052, 0.027), origin=Origin(xyz=(0.510, -0.061, 0.0), rpy=y_cyl.rpy), material=cartridge_blue, name="elbow_cap_1")
    for i, y in enumerate((0.083, -0.083)):
        upper.visual(Cylinder(0.010, 0.007), origin=Origin(xyz=(0.460, y, 0.045), rpy=y_cyl.rpy), material=black, name=f"elbow_bolt_{i}_0")
        upper.visual(Cylinder(0.010, 0.007), origin=Origin(xyz=(0.512, y, -0.045), rpy=y_cyl.rpy), material=black, name=f"elbow_bolt_{i}_1")
    upper.visual(Box((0.085, 0.008, 0.012)), origin=Origin(xyz=(0.510, 0.065, 0.070)), material=safety_orange, name="elbow_axis_mark")

    forearm = model.part("forearm")
    forearm.visual(Cylinder(0.047, 0.095), origin=Origin(rpy=y_cyl.rpy), material=cartridge_blue, name="elbow_inner_hub")
    forearm.visual(Box((0.320, 0.075, 0.060)), origin=Origin(xyz=(0.195, 0.0, 0.0)), material=extrusion, name="forearm_extrusion")
    forearm.visual(Box((0.290, 0.040, 0.022)), origin=Origin(xyz=(0.220, 0.0, 0.041)), material=molded_gray, name="forearm_snap_rib")
    forearm.visual(Box((0.102, 0.026, 0.105)), origin=Origin(xyz=(0.392, 0.050, 0.0)), material=extrusion, name="wrist_yoke_0")
    forearm.visual(Box((0.102, 0.026, 0.105)), origin=Origin(xyz=(0.392, -0.050, 0.0)), material=extrusion, name="wrist_yoke_1")
    forearm.visual(Cylinder(0.040, 0.022), origin=Origin(xyz=(0.400, 0.043, 0.0), rpy=y_cyl.rpy), material=cartridge_blue, name="wrist_cap_0")
    forearm.visual(Cylinder(0.040, 0.022), origin=Origin(xyz=(0.400, -0.043, 0.0), rpy=y_cyl.rpy), material=cartridge_blue, name="wrist_cap_1")
    forearm.visual(Box((0.065, 0.006, 0.010)), origin=Origin(xyz=(0.400, 0.050, 0.057)), material=safety_orange, name="wrist_axis_mark")

    wrist = model.part("wrist_pitch")
    wrist.visual(Cylinder(0.036, 0.064), origin=Origin(rpy=y_cyl.rpy), material=cartridge_blue, name="wrist_inner_hub")
    wrist.visual(Box((0.088, 0.055, 0.050)), origin=Origin(xyz=(0.078, 0.0, 0.0)), material=molded_gray, name="wrist_neck")
    wrist.visual(Cylinder(0.044, 0.085), origin=Origin(xyz=(0.1575, 0.0, 0.0), rpy=x_cyl.rpy), material=cartridge_blue, name="roll_cartridge")
    wrist.visual(Box((0.025, 0.090, 0.018)), origin=Origin(xyz=(0.156, 0.0, 0.049)), material=molded_gray, name="roll_clamp_tab")
    wrist.visual(Cylinder(0.008, 0.010), origin=Origin(xyz=(0.156, 0.034, 0.062)), material=black, name="roll_clamp_bolt_0")
    wrist.visual(Cylinder(0.008, 0.010), origin=Origin(xyz=(0.156, -0.034, 0.062)), material=black, name="roll_clamp_bolt_1")
    wrist.visual(Box((0.050, 0.006, 0.010)), origin=Origin(xyz=(0.204, 0.0, 0.046)), material=safety_orange, name="roll_axis_mark")

    flange = model.part("tool_flange")
    flange.visual(Cylinder(0.028, 0.055), origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=x_cyl.rpy), material=cartridge_blue, name="roll_output_shaft")
    flange.visual(Cylinder(0.064, 0.018), origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=x_cyl.rpy), material=cast_dark, name="standard_flange")
    flange.visual(Box((0.020, 0.105, 0.020)), origin=Origin(xyz=(0.083, 0.0, 0.0)), material=cast_dark, name="tool_clamp_bar")
    for i, (y, z) in enumerate(((0.036, 0.036), (-0.036, 0.036), (0.036, -0.036), (-0.036, -0.036))):
        flange.visual(Cylinder(0.006, 0.006), origin=Origin(xyz=(0.076, y, z), rpy=x_cyl.rpy), material=black, name=f"flange_bolt_{i}")

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.2875)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper,
        origin=Origin(xyz=(0.200, 0.0, 0.270)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.2, lower=-1.35, upper=1.65),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.510, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.5, lower=-2.15, upper=0.25),
    )
    model.articulation(
        "wrist_pitch_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.400, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.0, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=flange,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_pedestal")
    shoulder = object_model.get_part("shoulder_module")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_pitch")
    flange = object_model.get_part("tool_flange")

    ctx.expect_contact(base, shoulder, elem_a="yaw_stator_ring", elem_b="yaw_rotor", name="yaw cartridge seats on pedestal")
    ctx.expect_gap(shoulder, upper, axis="y", max_gap=0.004, max_penetration=0.004, positive_elem="shoulder_cap_0", negative_elem="shoulder_inner_hub", name="shoulder positive cap retains hub")
    ctx.expect_gap(upper, shoulder, axis="y", max_gap=0.004, max_penetration=0.004, positive_elem="shoulder_inner_hub", negative_elem="shoulder_cap_1", name="shoulder negative cap retains hub")
    ctx.expect_overlap(upper, shoulder, axes="xz", elem_a="shoulder_inner_hub", elem_b="shoulder_cap_0", min_overlap=0.060, name="shoulder axes are coaxial")
    ctx.expect_gap(upper, forearm, axis="y", max_gap=0.004, max_penetration=0.004, positive_elem="elbow_cap_0", negative_elem="elbow_inner_hub", name="elbow positive cap retains hub")
    ctx.expect_gap(forearm, upper, axis="y", max_gap=0.004, max_penetration=0.004, positive_elem="elbow_inner_hub", negative_elem="elbow_cap_1", name="elbow negative cap retains hub")
    ctx.expect_overlap(forearm, upper, axes="xz", elem_a="elbow_inner_hub", elem_b="elbow_cap_0", min_overlap=0.045, name="elbow axes are coaxial")
    ctx.expect_gap(forearm, wrist, axis="y", max_gap=0.004, max_penetration=0.004, positive_elem="wrist_cap_0", negative_elem="wrist_inner_hub", name="wrist positive cap retains hub")
    ctx.expect_gap(wrist, forearm, axis="y", max_gap=0.004, max_penetration=0.004, positive_elem="wrist_inner_hub", negative_elem="wrist_cap_1", name="wrist negative cap retains hub")
    ctx.expect_overlap(wrist, forearm, axes="xz", elem_a="wrist_inner_hub", elem_b="wrist_cap_0", min_overlap=0.035, name="wrist pitch axes are coaxial")
    ctx.expect_contact(wrist, flange, elem_a="roll_cartridge", elem_b="roll_output_shaft", name="roll shaft seats against cartridge")

    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch_joint")
    rest_flange = ctx.part_world_position(flange)
    with ctx.pose({shoulder_pitch: 0.55, elbow_pitch: -0.75, wrist_pitch: 0.50}):
        moved_flange = ctx.part_world_position(flange)
    ctx.check(
        "pitch chain changes flange height",
        rest_flange is not None and moved_flange is not None and abs(moved_flange[2] - rest_flange[2]) > 0.10,
        details=f"rest={rest_flange}, moved={moved_flange}",
    )

    return ctx.report()


object_model = build_object_model()
