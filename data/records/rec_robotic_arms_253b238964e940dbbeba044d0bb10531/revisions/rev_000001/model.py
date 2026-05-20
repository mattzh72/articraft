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
    model = ArticulatedObject(name="calibration_robotic_arm")

    anodized = model.material("hard_anodized_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    clear = model.material("clear_joint_covers", rgba=(0.45, 0.70, 0.95, 0.34))
    black = model.material("black_etched_marks", rgba=(0.01, 0.01, 0.01, 1.0))
    red = model.material("red_x_axis", rgba=(0.90, 0.12, 0.08, 1.0))
    green = model.material("green_y_axis", rgba=(0.10, 0.75, 0.18, 1.0))
    blue = model.material("blue_z_axis", rgba=(0.10, 0.28, 0.90, 1.0))
    brass = model.material("brass_adjusters", rgba=(0.92, 0.66, 0.26, 1.0))

    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    base = model.part("base")
    base.visual(Box((0.72, 0.56, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=anodized, name="floor_plate")
    base.visual(Cylinder(radius=0.024, length=0.035), origin=Origin(xyz=(0.27, 0.19, 0.012)), material=black, name="level_foot_0")
    base.visual(Cylinder(radius=0.024, length=0.035), origin=Origin(xyz=(0.27, -0.19, 0.012)), material=black, name="level_foot_1")
    base.visual(Cylinder(radius=0.024, length=0.035), origin=Origin(xyz=(-0.27, 0.19, 0.012)), material=black, name="level_foot_2")
    base.visual(Cylinder(radius=0.024, length=0.035), origin=Origin(xyz=(-0.27, -0.19, 0.012)), material=black, name="level_foot_3")
    base.visual(Cylinder(radius=0.18, length=0.36), origin=Origin(xyz=(0.0, 0.0, 0.26)), material=brushed, name="pedestal")
    base.visual(Cylinder(radius=0.225, length=0.140), origin=Origin(xyz=(0.0, 0.0, 0.480)), material=anodized, name="datum_turntable")
    base.visual(Cylinder(radius=0.102, length=0.080), origin=Origin(xyz=(0.0, 0.0, 0.508)), material=brushed, name="yaw_bearing")
    base.visual(Box((0.42, 0.052, 0.012)), origin=Origin(xyz=(0.0, 0.165, 0.526)), material=brushed, name="front_datum_rail")
    base.visual(Box((0.42, 0.052, 0.012)), origin=Origin(xyz=(0.0, -0.165, 0.526)), material=brushed, name="rear_datum_rail")
    base.visual(Box((0.004, 0.040, 0.004)), origin=Origin(xyz=(0.0, 0.165, 0.534)), material=black, name="yaw_zero_mark")
    for idx, x in enumerate((-0.15, -0.075, 0.075, 0.15)):
        base.visual(Box((0.004, 0.032, 0.004)), origin=Origin(xyz=(x, -0.165, 0.534)), material=black, name=f"yaw_tick_{idx}")
    base.visual(Cylinder(radius=0.010, length=0.090), origin=Origin(xyz=(-0.275, 0.0, 0.125)), material=blue, name="z_axis_post")

    shoulder = model.part("shoulder")
    shoulder.visual(Cylinder(radius=0.072, length=0.150), origin=Origin(xyz=(0.0, 0.0, -0.035)), material=brushed, name="yaw_spindle")
    shoulder.visual(Cylinder(radius=0.185, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=clear, name="yaw_cartridge")
    shoulder.visual(Box((0.160, 0.200, 0.235)), origin=Origin(xyz=(0.0, 0.0, 0.1825)), material=anodized, name="riser_block")
    shoulder.visual(Box((0.150, 0.070, 0.230)), origin=Origin(xyz=(0.0, 0.125, 0.330)), material=anodized, name="pitch_cheek_0")
    shoulder.visual(Box((0.150, 0.070, 0.230)), origin=Origin(xyz=(0.0, -0.125, 0.330)), material=anodized, name="pitch_cheek_1")
    shoulder.visual(Cylinder(radius=0.118, length=0.080), origin=Origin(xyz=(0.0, 0.170, 0.390), rpy=cyl_y.rpy), material=clear, name="shoulder_bearing_0")
    shoulder.visual(Cylinder(radius=0.118, length=0.080), origin=Origin(xyz=(0.0, -0.170, 0.390), rpy=cyl_y.rpy), material=clear, name="shoulder_bearing_1")
    shoulder.visual(Box((0.115, 0.030, 0.006)), origin=Origin(xyz=(0.0, 0.170, 0.510)), material=black, name="shoulder_index_0")
    shoulder.visual(Box((0.115, 0.030, 0.006)), origin=Origin(xyz=(0.0, -0.170, 0.510)), material=black, name="shoulder_index_1")
    shoulder.visual(Cylinder(radius=0.008, length=0.300), origin=Origin(xyz=(-0.105, 0.0, 0.390), rpy=cyl_y.rpy), material=green, name="pitch_axis_line")
    shoulder.visual(Cylinder(radius=0.012, length=0.045), origin=Origin(xyz=(0.094, 0.0, 0.080), rpy=cyl_x.rpy), material=brass, name="yaw_clamp_screw")

    upper_arm = model.part("upper_arm")
    upper_arm.visual(Cylinder(radius=0.042, length=0.440), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy), material=brushed, name="shoulder_axle")
    upper_arm.visual(Cylinder(radius=0.088, length=0.180), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy), material=clear, name="shoulder_hub")
    upper_arm.visual(Box((0.495, 0.105, 0.110)), origin=Origin(xyz=(0.3325, 0.0, 0.0)), material=brushed, name="metrology_beam")
    upper_arm.visual(Box((0.445, 0.028, 0.014)), origin=Origin(xyz=(0.3525, 0.0, 0.062)), material=anodized, name="top_datum_rail")
    upper_arm.visual(Box((0.120, 0.030, 0.004)), origin=Origin(xyz=(0.210, 0.0, 0.070)), material=black, name="adjust_slot_0")
    upper_arm.visual(Box((0.120, 0.030, 0.004)), origin=Origin(xyz=(0.400, 0.0, 0.070)), material=black, name="adjust_slot_1")
    upper_arm.visual(Box((0.030, 0.260, 0.080)), origin=Origin(xyz=(0.575, 0.0, 0.0)), material=anodized, name="elbow_bridge")
    upper_arm.visual(Box((0.150, 0.060, 0.145)), origin=Origin(xyz=(0.650, 0.122, 0.0)), material=anodized, name="elbow_fork_0")
    upper_arm.visual(Box((0.150, 0.060, 0.145)), origin=Origin(xyz=(0.650, -0.122, 0.0)), material=anodized, name="elbow_fork_1")
    upper_arm.visual(Cylinder(radius=0.092, length=0.070), origin=Origin(xyz=(0.675, 0.158, 0.0), rpy=cyl_y.rpy), material=clear, name="elbow_bearing_0")
    upper_arm.visual(Cylinder(radius=0.092, length=0.070), origin=Origin(xyz=(0.675, -0.158, 0.0), rpy=cyl_y.rpy), material=clear, name="elbow_bearing_1")
    upper_arm.visual(Box((0.080, 0.026, 0.004)), origin=Origin(xyz=(0.675, 0.158, 0.094)), material=black, name="elbow_index_0")
    upper_arm.visual(Box((0.080, 0.026, 0.004)), origin=Origin(xyz=(0.675, -0.158, 0.094)), material=black, name="elbow_index_1")

    forearm = model.part("forearm")
    forearm.visual(Cylinder(radius=0.034, length=0.380), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy), material=brushed, name="elbow_axle")
    forearm.visual(Cylinder(radius=0.073, length=0.140), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy), material=clear, name="elbow_hub")
    forearm.visual(Box((0.385, 0.110, 0.092)), origin=Origin(xyz=(0.2625, 0.0, 0.0)), material=brushed, name="forearm_beam")
    forearm.visual(Box((0.330, 0.022, 0.012)), origin=Origin(xyz=(0.260, 0.0, 0.052)), material=anodized, name="forearm_datum")
    forearm.visual(Box((0.090, 0.024, 0.004)), origin=Origin(xyz=(0.225, 0.0, 0.059)), material=black, name="fine_slot_0")
    forearm.visual(Box((0.090, 0.024, 0.004)), origin=Origin(xyz=(0.365, 0.0, 0.059)), material=black, name="fine_slot_1")
    forearm.visual(Box((0.130, 0.030, 0.090)), origin=Origin(xyz=(0.485, 0.060, 0.0)), material=anodized, name="wrist_fork_0")
    forearm.visual(Box((0.130, 0.030, 0.090)), origin=Origin(xyz=(0.485, -0.060, 0.0)), material=anodized, name="wrist_fork_1")
    forearm.visual(Cylinder(radius=0.078, length=0.095), origin=Origin(xyz=(0.555, 0.0, 0.0), rpy=cyl_x.rpy), material=clear, name="wrist_bearing")
    forearm.visual(Cylinder(radius=0.007, length=0.140), origin=Origin(xyz=(0.555, 0.0, 0.084), rpy=cyl_x.rpy), material=red, name="roll_axis_line")

    wrist = model.part("wrist")
    wrist.visual(Cylinder(radius=0.042, length=0.185), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_x.rpy), material=brushed, name="roll_spindle")
    wrist.visual(Cylinder(radius=0.062, length=0.060), origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=cyl_x.rpy), material=clear, name="roll_cartridge")
    wrist.visual(Box((0.050, 0.140, 0.100)), origin=Origin(xyz=(0.080, 0.0, 0.0)), material=anodized, name="wrist_block")
    wrist.visual(Box((0.110, 0.038, 0.120)), origin=Origin(xyz=(0.155, 0.085, 0.0)), material=anodized, name="tool_cheek_0")
    wrist.visual(Box((0.110, 0.038, 0.120)), origin=Origin(xyz=(0.155, -0.085, 0.0)), material=anodized, name="tool_cheek_1")
    wrist.visual(Cylinder(radius=0.060, length=0.050), origin=Origin(xyz=(0.165, 0.112, 0.0), rpy=cyl_y.rpy), material=clear, name="tool_bearing_0")
    wrist.visual(Cylinder(radius=0.060, length=0.050), origin=Origin(xyz=(0.165, -0.112, 0.0), rpy=cyl_y.rpy), material=clear, name="tool_bearing_1")
    wrist.visual(Box((0.050, 0.018, 0.004)), origin=Origin(xyz=(0.165, 0.112, 0.061)), material=black, name="wrist_index_0")
    wrist.visual(Box((0.050, 0.018, 0.004)), origin=Origin(xyz=(0.165, -0.112, 0.061)), material=black, name="wrist_index_1")

    tool_plate = model.part("tool_plate")
    tool_plate.visual(Cylinder(radius=0.030, length=0.265), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy), material=brushed, name="tool_axle")
    tool_plate.visual(Cylinder(radius=0.052, length=0.125), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy), material=clear, name="tool_hub")
    tool_plate.visual(Box((0.140, 0.070, 0.065)), origin=Origin(xyz=(0.100, 0.0, 0.0)), material=brushed, name="tool_neck")
    tool_plate.visual(Box((0.032, 0.185, 0.160)), origin=Origin(xyz=(0.185, 0.0, 0.0)), material=anodized, name="tool_flange")
    tool_plate.visual(Box((0.006, 0.145, 0.004)), origin=Origin(xyz=(0.204, 0.0, 0.000)), material=black, name="flange_cross_y")
    tool_plate.visual(Box((0.006, 0.004, 0.120)), origin=Origin(xyz=(0.204, 0.0, 0.000)), material=black, name="flange_cross_z")
    tool_plate.visual(Cylinder(radius=0.012, length=0.020), origin=Origin(xyz=(0.210, 0.058, 0.048), rpy=cyl_x.rpy), material=brass, name="datum_pin_0")
    tool_plate.visual(Cylinder(radius=0.012, length=0.020), origin=Origin(xyz=(0.210, -0.058, -0.048), rpy=cyl_x.rpy), material=brass, name="datum_pin_1")

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.0, lower=-1.35, upper=1.55),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.675, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.2, lower=-2.25, upper=2.25),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.555, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=tool_plate,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-1.70, upper=1.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    tool_plate = object_model.get_part("tool_plate")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    captured_pairs = (
        (base, shoulder, "yaw_bearing", "yaw_spindle", "captured yaw spindle runs inside the base bearing cartridge"),
        (base, shoulder, "datum_turntable", "yaw_spindle", "yaw spindle passes through the rotary table bore proxy"),
        (shoulder, upper_arm, "pitch_cheek_0", "shoulder_axle", "shoulder cheek has a bored axle passage"),
        (shoulder, upper_arm, "pitch_cheek_1", "shoulder_axle", "opposite shoulder cheek has a bored axle passage"),
        (shoulder, upper_arm, "shoulder_bearing_0", "shoulder_axle", "shoulder axle is captured by the datum-side bearing"),
        (shoulder, upper_arm, "shoulder_bearing_1", "shoulder_axle", "shoulder axle is captured by the opposite bearing"),
        (upper_arm, forearm, "elbow_fork_0", "elbow_axle", "elbow fork cheek has a bored axle passage"),
        (upper_arm, forearm, "elbow_fork_1", "elbow_axle", "opposite elbow fork cheek has a bored axle passage"),
        (upper_arm, forearm, "elbow_bearing_0", "elbow_axle", "elbow axle is captured by the datum-side bearing"),
        (upper_arm, forearm, "elbow_bearing_1", "elbow_axle", "elbow axle is captured by the opposite bearing"),
        (forearm, wrist, "wrist_bearing", "roll_spindle", "wrist roll spindle is seated inside the forearm bearing"),
        (wrist, tool_plate, "tool_cheek_0", "tool_axle", "tool cheek has a bored pitch-axle passage"),
        (wrist, tool_plate, "tool_cheek_1", "tool_axle", "opposite tool cheek has a bored pitch-axle passage"),
        (wrist, tool_plate, "tool_bearing_0", "tool_axle", "tool pitch axle is captured by the datum-side bearing"),
        (wrist, tool_plate, "tool_bearing_1", "tool_axle", "tool pitch axle is captured by the opposite bearing"),
    )
    for parent, child, parent_elem, child_elem, reason in captured_pairs:
        ctx.allow_overlap(parent, child, elem_a=parent_elem, elem_b=child_elem, reason=reason)

    ctx.expect_within(
        shoulder,
        base,
        axes="xy",
        inner_elem="yaw_spindle",
        outer_elem="yaw_bearing",
        margin=0.001,
        name="yaw spindle centered in bearing",
    )
    ctx.expect_overlap(
        shoulder,
        base,
        axes="z",
        elem_a="yaw_spindle",
        elem_b="yaw_bearing",
        min_overlap=0.065,
        name="yaw spindle has retained insertion",
    )
    ctx.expect_within(
        shoulder,
        base,
        axes="xy",
        inner_elem="yaw_spindle",
        outer_elem="datum_turntable",
        margin=0.001,
        name="yaw spindle passes through turntable center",
    )
    ctx.expect_overlap(
        shoulder,
        base,
        axes="z",
        elem_a="yaw_spindle",
        elem_b="datum_turntable",
        min_overlap=0.090,
        name="yaw spindle retained through turntable",
    )
    ctx.expect_gap(
        shoulder,
        base,
        axis="z",
        positive_elem="yaw_cartridge",
        negative_elem="yaw_bearing",
        min_gap=0.008,
        max_gap=0.016,
        name="yaw cartridge controlled axial gap",
    )

    for bearing_name in ("shoulder_bearing_0", "shoulder_bearing_1"):
        ctx.expect_within(
            upper_arm,
            shoulder,
            axes="xz",
            inner_elem="shoulder_axle",
            outer_elem=bearing_name,
            margin=0.001,
            name=f"{bearing_name} centers shoulder axle",
        )
        ctx.expect_overlap(
            upper_arm,
            shoulder,
            axes="y",
            elem_a="shoulder_axle",
            elem_b=bearing_name,
            min_overlap=0.055,
            name=f"{bearing_name} captures shoulder axle",
        )
    for cheek_name in ("pitch_cheek_0", "pitch_cheek_1"):
        ctx.expect_overlap(
            upper_arm,
            shoulder,
            axes="y",
            elem_a="shoulder_axle",
            elem_b=cheek_name,
            min_overlap=0.055,
            name=f"{cheek_name} contains shoulder axle bore",
        )
    ctx.expect_gap(
        shoulder,
        upper_arm,
        axis="y",
        positive_elem="shoulder_bearing_0",
        negative_elem="shoulder_hub",
        min_gap=0.030,
        max_gap=0.050,
        name="positive shoulder cheek clearance",
    )
    ctx.expect_gap(
        upper_arm,
        shoulder,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_bearing_1",
        min_gap=0.030,
        max_gap=0.050,
        name="negative shoulder cheek clearance",
    )

    for bearing_name in ("elbow_bearing_0", "elbow_bearing_1"):
        ctx.expect_within(
            forearm,
            upper_arm,
            axes="xz",
            inner_elem="elbow_axle",
            outer_elem=bearing_name,
            margin=0.001,
            name=f"{bearing_name} centers elbow axle",
        )
        ctx.expect_overlap(
            forearm,
            upper_arm,
            axes="y",
            elem_a="elbow_axle",
            elem_b=bearing_name,
            min_overlap=0.045,
            name=f"{bearing_name} captures elbow axle",
        )
    for fork_name in ("elbow_fork_0", "elbow_fork_1"):
        ctx.expect_overlap(
            forearm,
            upper_arm,
            axes="y",
            elem_a="elbow_axle",
            elem_b=fork_name,
            min_overlap=0.045,
            name=f"{fork_name} contains elbow axle bore",
        )

    ctx.expect_within(
        wrist,
        forearm,
        axes="yz",
        inner_elem="roll_spindle",
        outer_elem="wrist_bearing",
        margin=0.001,
        name="roll spindle centered in bearing",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="x",
        elem_a="roll_spindle",
        elem_b="wrist_bearing",
        min_overlap=0.090,
        name="wrist roll spindle retained",
    )

    for bearing_name in ("tool_bearing_0", "tool_bearing_1"):
        ctx.expect_within(
            tool_plate,
            wrist,
            axes="xz",
            inner_elem="tool_axle",
            outer_elem=bearing_name,
            margin=0.001,
            name=f"{bearing_name} centers tool axle",
        )
        ctx.expect_overlap(
            tool_plate,
            wrist,
            axes="y",
            elem_a="tool_axle",
            elem_b=bearing_name,
            min_overlap=0.035,
            name=f"{bearing_name} captures tool axle",
        )
    for cheek_name in ("tool_cheek_0", "tool_cheek_1"):
        ctx.expect_overlap(
            tool_plate,
            wrist,
            axes="y",
            elem_a="tool_axle",
            elem_b=cheek_name,
            min_overlap=0.030,
            name=f"{cheek_name} contains tool axle bore",
        )

    expected_axes = {
        base_yaw: (0.0, 0.0, 1.0),
        shoulder_pitch: (0.0, -1.0, 0.0),
        elbow_pitch: (0.0, -1.0, 0.0),
        wrist_roll: (1.0, 0.0, 0.0),
        wrist_pitch: (0.0, -1.0, 0.0),
    }
    for joint, expected in expected_axes.items():
        actual = tuple(float(v) for v in joint.axis)
        ctx.check(
            f"{joint.name} axis is explicit",
            all(abs(actual[i] - expected[i]) < 1.0e-9 for i in range(3)),
            details=f"expected {expected}, got {actual}",
        )

    def coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    def elem_center(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((coord(lo, i) + coord(hi, i)) * 0.5 for i in range(3))

    upper_rest = elem_center(upper_arm, "metrology_beam")
    with ctx.pose({base_yaw: 0.55}):
        upper_yawed = elem_center(upper_arm, "metrology_beam")
    ctx.check(
        "base yaw sweeps arm about vertical axis",
        upper_rest is not None and upper_yawed is not None and upper_yawed[1] > upper_rest[1] + 0.16,
        details=f"rest={upper_rest}, yawed={upper_yawed}",
    )

    upper_rest = elem_center(upper_arm, "metrology_beam")
    with ctx.pose({shoulder_pitch: 0.70}):
        upper_raised = elem_center(upper_arm, "metrology_beam")
    ctx.check(
        "shoulder pitch raises upper arm",
        upper_rest is not None and upper_raised is not None and upper_raised[2] > upper_rest[2] + 0.20,
        details=f"rest={upper_rest}, raised={upper_raised}",
    )

    forearm_rest = elem_center(forearm, "forearm_beam")
    with ctx.pose({elbow_pitch: 0.80}):
        forearm_raised = elem_center(forearm, "forearm_beam")
    ctx.check(
        "elbow pitch bends forearm upward",
        forearm_rest is not None and forearm_raised is not None and forearm_raised[2] > forearm_rest[2] + 0.17,
        details=f"rest={forearm_rest}, raised={forearm_raised}",
    )

    pin_rest = elem_center(tool_plate, "datum_pin_0")
    with ctx.pose({wrist_roll: 0.70}):
        pin_rolled = elem_center(tool_plate, "datum_pin_0")
    ctx.check(
        "wrist roll rotates asymmetric datum pin",
        pin_rest is not None and pin_rolled is not None and abs(pin_rolled[2] - pin_rest[2]) > 0.020,
        details=f"rest={pin_rest}, rolled={pin_rolled}",
    )

    flange_rest = elem_center(tool_plate, "tool_flange")
    with ctx.pose({wrist_pitch: 0.75}):
        flange_raised = elem_center(tool_plate, "tool_flange")
    ctx.check(
        "wrist pitch raises tool flange",
        flange_rest is not None and flange_raised is not None and flange_raised[2] > flange_rest[2] + 0.10,
        details=f"rest={flange_rest}, raised={flange_raised}",
    )

    return ctx.report()


object_model = build_object_model()
