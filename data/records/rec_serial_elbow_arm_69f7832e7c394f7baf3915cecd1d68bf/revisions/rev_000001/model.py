from __future__ import annotations

import math

import cadquery as cq
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


CYL_X = (0.0, math.pi / 2.0, 0.0)
CYL_Y = (math.pi / 2.0, 0.0, 0.0)


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_elbow_arm_study")

    machined = model.material("machined_gray", rgba=(0.48, 0.50, 0.50, 1.0))
    dark = model.material("dark_oxide", rgba=(0.06, 0.065, 0.07, 1.0))
    plate = model.material("shot_peened_plate", rgba=(0.32, 0.34, 0.35, 1.0))
    bearing = model.material("bearing_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    cover = model.material("cover_plate_black", rgba=(0.015, 0.018, 0.02, 1.0))

    shoulder_z = 0.49
    elbow_x = 0.48

    pedestal = model.part("pedestal")
    _box(pedestal, "mounting_plate", (0.46, 0.34, 0.035), (0.0, 0.0, 0.0175), plate)
    _cyl(pedestal, "base_register", 0.140, 0.045, (0.0, 0.0, 0.0575), machined)
    _cyl(pedestal, "pedestal_column", 0.070, 0.260, (0.0, 0.0, 0.210), machined)
    _box(pedestal, "shoulder_block", (0.28, 0.24, 0.055), (0.0, 0.0, 0.3675), plate)
    _box(pedestal, "guide_way_0", (0.22, 0.026, 0.022), (0.0, 0.075, 0.406), dark)
    _box(pedestal, "guide_way_1", (0.22, 0.026, 0.022), (0.0, -0.075, 0.406), dark)
    _box(pedestal, "yoke_cheek_0", (0.120, 0.032, 0.190), (0.0, 0.112, shoulder_z), plate)
    _box(pedestal, "yoke_cheek_1", (0.120, 0.032, 0.190), (0.0, -0.112, shoulder_z), plate)
    _cyl(pedestal, "shoulder_bearing_cap_0", 0.066, 0.018, (0.0, 0.134, shoulder_z), machined, CYL_Y)
    _cyl(pedestal, "shoulder_bearing_cap_1", 0.066, 0.018, (0.0, -0.134, shoulder_z), machined, CYL_Y)
    _cyl(pedestal, "shoulder_seal_0", 0.045, 0.007, (0.0, 0.1465, shoulder_z), dark, CYL_Y)
    _cyl(pedestal, "shoulder_seal_1", 0.045, 0.007, (0.0, -0.1465, shoulder_z), dark, CYL_Y)
    _box(pedestal, "shoulder_stop_bridge", (0.035, 0.220, 0.025), (-0.060, 0.0, 0.590), dark)
    _box(pedestal, "service_cover", (0.008, 0.120, 0.055), (0.144, 0.0, 0.368), cover)
    for i, (x, y) in enumerate(((-0.17, -0.12), (-0.17, 0.12), (0.17, -0.12), (0.17, 0.12))):
        _cyl(pedestal, f"base_bolt_{i}", 0.014, 0.008, (x, y, 0.039), bearing)
    for i, (y, z) in enumerate(((-0.040, 0.350), (0.040, 0.350), (-0.040, 0.386), (0.040, 0.386))):
        _cyl(pedestal, f"cover_screw_{i}", 0.005, 0.006, (0.151, y, z), bearing, CYL_X)

    upper_link = model.part("upper_link")
    _cyl(upper_link, "shoulder_hub", 0.052, 0.172, (0.0, 0.0, 0.0), dark, CYL_Y)
    _cyl(upper_link, "shoulder_collar_0", 0.061, 0.010, (0.0, 0.091, 0.0), bearing, CYL_Y)
    _cyl(upper_link, "shoulder_collar_1", 0.061, 0.010, (0.0, -0.091, 0.0), bearing, CYL_Y)
    _box(upper_link, "side_plate_0", (0.420, 0.014, 0.058), (0.255, 0.055, 0.0), plate)
    _box(upper_link, "side_plate_1", (0.420, 0.014, 0.058), (0.255, -0.055, 0.0), plate)
    _box(upper_link, "lower_web", (0.320, 0.105, 0.018), (0.250, 0.0, -0.036), machined)
    _box(upper_link, "access_cover", (0.210, 0.105, 0.010), (0.250, 0.0, 0.034), cover)
    for side, y in enumerate((0.065, -0.065)):
        for n, x in enumerate((0.140, 0.250, 0.360)):
            _box(upper_link, f"stiffener_{side}_{n}", (0.020, 0.008, 0.070), (x, y, 0.0), machined)
    _cyl(upper_link, "elbow_bearing_cap_0", 0.062, 0.018, (elbow_x, 0.071, 0.0), machined, CYL_Y)
    _cyl(upper_link, "elbow_bearing_cap_1", 0.062, 0.018, (elbow_x, -0.071, 0.0), machined, CYL_Y)
    _cyl(upper_link, "elbow_seal_0", 0.040, 0.007, (elbow_x, 0.0835, 0.0), dark, CYL_Y)
    _cyl(upper_link, "elbow_seal_1", 0.040, 0.007, (elbow_x, -0.0835, 0.0), dark, CYL_Y)
    _box(upper_link, "elbow_stop_0", (0.035, 0.012, 0.040), (0.430, 0.082, 0.045), dark)
    _box(upper_link, "elbow_stop_1", (0.035, 0.012, 0.040), (0.430, -0.082, 0.045), dark)

    forearm = model.part("forearm")
    forearm_angle = math.atan2(0.160, 0.380)
    _cyl(forearm, "elbow_hub", 0.045, 0.096, (0.0, 0.0, 0.0), dark, CYL_Y)
    _box(forearm, "dogleg_plate_0", (0.420, 0.012, 0.050), (0.195, 0.028, -0.082), plate, (0.0, forearm_angle, 0.0))
    _box(forearm, "dogleg_plate_1", (0.420, 0.012, 0.050), (0.195, -0.028, -0.082), plate, (0.0, forearm_angle, 0.0))
    _box(forearm, "center_web", (0.300, 0.060, 0.018), (0.210, 0.0, -0.095), machined, (0.0, forearm_angle, 0.0))
    for side, y in enumerate((0.0375, -0.0375)):
        for n, (x, z) in enumerate(((0.125, -0.052), (0.225, -0.095), (0.325, -0.137))):
            _box(forearm, f"raised_rib_{side}_{n}", (0.018, 0.008, 0.065), (x, y, z), machined, (0.0, forearm_angle, 0.0))
    _box(forearm, "access_cover", (0.180, 0.055, 0.008), (0.2285, 0.0, -0.0713), cover, (0.0, forearm_angle, 0.0))
    _cyl(forearm, "wrist_collar", 0.044, 0.080, (0.390, 0.0, -0.164), dark, CYL_Y)
    _box(forearm, "flange_pad", (0.050, 0.066, 0.050), (0.415, 0.0, -0.164), plate)
    _box(forearm, "mechanical_stop", (0.034, 0.085, 0.024), (-0.015, 0.0, 0.057), dark)

    flange = model.part("flange")
    _cyl(flange, "flange_disk", 0.046, 0.040, (0.020, 0.0, 0.0), machined, CYL_X)
    _cyl(flange, "pilot_boss", 0.018, 0.026, (0.052, 0.0, 0.0), dark, CYL_X)
    for i, (y, z) in enumerate(((0.022, 0.022), (0.022, -0.022), (-0.022, 0.022), (-0.022, -0.022))):
        _cyl(flange, f"flange_bolt_{i}", 0.005, 0.008, (0.043, y, z), bearing, CYL_X)

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(elbow_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-1.20, upper=1.35),
    )
    model.articulation(
        "flange_mount",
        ArticulationType.FIXED,
        parent=forearm,
        child=flange,
        origin=Origin(xyz=(0.440, 0.0, -0.164)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    flange = object_model.get_part("flange")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")

    ctx.check(
        "serial shoulder and elbow revolutes",
        shoulder.parent == "pedestal"
        and shoulder.child == "upper_link"
        and elbow.parent == "upper_link"
        and elbow.child == "forearm"
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE,
        details=f"shoulder={shoulder}, elbow={elbow}",
    )
    ctx.expect_within(
        upper_link,
        pedestal,
        axes="y",
        inner_elem="shoulder_hub",
        outer_elem="shoulder_block",
        margin=0.0,
        name="shoulder hub is captured between yoke cheeks",
    )
    ctx.expect_within(
        forearm,
        upper_link,
        axes="y",
        inner_elem="elbow_hub",
        outer_elem="lower_web",
        margin=0.0,
        name="elbow hub sits inside paired upper link plates",
    )
    ctx.expect_gap(
        flange,
        forearm,
        axis="x",
        positive_elem="flange_disk",
        negative_elem="flange_pad",
        max_gap=0.001,
        max_penetration=0.00001,
        name="bolted flange seats on forearm pad",
    )

    rest_forearm_pos = ctx.part_world_position(forearm)
    rest_flange_pos = ctx.part_world_position(flange)
    with ctx.pose({shoulder: -0.55}):
        raised_forearm_pos = ctx.part_world_position(forearm)
    with ctx.pose({elbow: 0.85}):
        folded_flange_pos = ctx.part_world_position(flange)
    ctx.check(
        "shoulder joint raises downstream elbow assembly",
        rest_forearm_pos is not None
        and raised_forearm_pos is not None
        and raised_forearm_pos[2] > rest_forearm_pos[2] + 0.05,
        details=f"rest={rest_forearm_pos}, raised={raised_forearm_pos}",
    )
    ctx.check(
        "elbow joint folds the end flange",
        rest_flange_pos is not None
        and folded_flange_pos is not None
        and folded_flange_pos[2] < rest_flange_pos[2] - 0.05,
        details=f"rest={rest_flange_pos}, folded={folded_flange_pos}",
    )

    return ctx.report()


object_model = build_object_model()
