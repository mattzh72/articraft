from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _cylinder_pose(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_cylinder(part, name, radius, length, xyz, axis, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_cylinder_pose(axis)),
        material=material,
        name=name,
    )


def _add_box(part, name, size, xyz, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_bolt_pair(part, prefix, x, y_abs, z, axis, material, radius=0.008, length=0.008):
    _add_cylinder(part, f"{prefix}_0", radius, length, (x, -y_abs, z), axis, material)
    _add_cylinder(part, f"{prefix}_1", radius, length, (x, y_abs, z), axis, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_robotic_leg")

    enamel = model.material("aged_olive_enamel", rgba=(0.34, 0.40, 0.30, 1.0))
    dark = model.material("oiled_gunmetal", rgba=(0.08, 0.09, 0.09, 1.0))
    steel = model.material("brushed_steel", rgba=(0.63, 0.65, 0.61, 1.0))
    zinc = model.material("zinc_bolts", rgba=(0.78, 0.76, 0.68, 1.0))
    orange = model.material("service_orange", rgba=(0.86, 0.38, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    hip_mount = model.part("hip_mount")
    _add_box(hip_mount, "pelvis_adapter", (0.30, 0.34, 0.13), (0.0, 0.0, 0.14), enamel)
    _add_box(hip_mount, "rear_service_hatch", (0.17, 0.010, 0.080), (-0.015, -0.175, 0.145), orange)
    _add_box(hip_mount, "hip_cheek_0", (0.12, 0.040, 0.20), (0.0, -0.105, -0.020), enamel)
    _add_box(hip_mount, "hip_cheek_1", (0.12, 0.040, 0.20), (0.0, 0.105, -0.020), enamel)
    _add_box(hip_mount, "hip_web_0", (0.045, 0.25, 0.026), (-0.052, 0.0, 0.072), dark)
    _add_box(hip_mount, "hip_web_1", (0.045, 0.25, 0.026), (0.052, 0.0, 0.072), dark)
    _add_cylinder(hip_mount, "hip_pin", 0.034, 0.285, (0.0, 0.0, 0.0), "y", steel)
    for i, (x, y) in enumerate(((-0.105, -0.120), (-0.105, 0.120), (0.105, -0.120), (0.105, 0.120))):
        _add_cylinder(hip_mount, f"adapter_bolt_{i}", 0.010, 0.012, (x, y, 0.211), "z", zinc)
    _add_bolt_pair(hip_mount, "hatch_bolt_a", -0.085, 0.155, 0.120, "y", zinc, radius=0.006, length=0.007)
    _add_bolt_pair(hip_mount, "hatch_bolt_b", 0.055, 0.155, 0.170, "y", zinc, radius=0.006, length=0.007)

    thigh = model.part("thigh_shell")
    _add_cylinder(thigh, "hip_hub", 0.065, 0.140, (0.0, 0.0, 0.0), "y", steel)
    _add_box(thigh, "main_shell", (0.155, 0.115, 0.340), (0.0, 0.0, -0.230), enamel)
    _add_box(thigh, "front_actuator_bay", (0.045, 0.088, 0.275), (0.099, 0.0, -0.235), dark)
    _add_box(thigh, "thigh_hatch", (0.008, 0.072, 0.150), (0.125, 0.0, -0.220), orange)
    _add_box(thigh, "side_rail_0", (0.045, 0.014, 0.300), (-0.040, -0.064, -0.230), steel)
    _add_box(thigh, "side_rail_1", (0.045, 0.014, 0.300), (-0.040, 0.064, -0.230), steel)
    _add_box(thigh, "upper_band", (0.174, 0.132, 0.026), (0.0, 0.0, -0.095), steel)
    _add_box(thigh, "lower_band", (0.174, 0.132, 0.026), (0.0, 0.0, -0.350), steel)
    _add_box(thigh, "knee_bridge", (0.150, 0.230, 0.050), (0.0, 0.0, -0.380), dark)
    _add_box(thigh, "knee_cheek_0", (0.135, 0.035, 0.150), (0.0, -0.095, -0.475), enamel)
    _add_box(thigh, "knee_cheek_1", (0.135, 0.035, 0.150), (0.0, 0.095, -0.475), enamel)
    _add_cylinder(thigh, "knee_pin", 0.027, 0.260, (0.0, 0.0, -0.480), "y", steel)
    for i, z in enumerate((-0.160, -0.280)):
        _add_bolt_pair(thigh, f"bay_bolt_{i}", 0.128, 0.030, z, "x", zinc, radius=0.006, length=0.007)
    for i, (y, z) in enumerate(((-0.098, -0.438), (0.098, -0.438), (-0.098, -0.515), (0.098, -0.515))):
        _add_cylinder(thigh, f"knee_bolt_{i}", 0.007, 0.008, (0.052, y, z), "x", zinc)

    shank = model.part("shank_shell")
    _add_cylinder(shank, "knee_hub", 0.055, 0.130, (0.0, 0.0, 0.0), "y", steel)
    _add_box(shank, "main_shell", (0.130, 0.100, 0.310), (0.015, 0.0, -0.205), enamel)
    _add_box(shank, "rear_actuator_bay", (0.035, 0.078, 0.240), (-0.068, 0.0, -0.210), dark)
    _add_box(shank, "shank_hatch", (0.007, 0.062, 0.130), (-0.088, 0.0, -0.205), orange)
    _add_cylinder(shank, "cable_conduit", 0.011, 0.285, (0.010, 0.061, -0.215), "z", dark)
    _add_box(shank, "conduit_clamp_0", (0.060, 0.024, 0.024), (0.010, 0.056, -0.120), zinc)
    _add_box(shank, "conduit_clamp_1", (0.060, 0.024, 0.024), (0.010, 0.056, -0.300), zinc)
    _add_box(shank, "ankle_bridge", (0.128, 0.205, 0.050), (0.0, 0.0, -0.365), dark)
    _add_box(shank, "ankle_cheek_0", (0.115, 0.032, 0.140), (0.0, -0.085, -0.455), enamel)
    _add_box(shank, "ankle_cheek_1", (0.115, 0.032, 0.140), (0.0, 0.085, -0.455), enamel)
    _add_cylinder(shank, "ankle_pin", 0.023, 0.230, (0.0, 0.0, -0.440), "y", steel)
    _add_box(shank, "upper_band", (0.150, 0.116, 0.024), (0.015, 0.0, -0.085), steel)
    _add_box(shank, "lower_band", (0.150, 0.116, 0.024), (0.015, 0.0, -0.325), steel)
    for i, z in enumerate((-0.155, -0.255)):
        _add_bolt_pair(shank, f"shank_bay_bolt_{i}", -0.091, 0.025, z, "x", zinc, radius=0.0055, length=0.006)
    for i, (y, z) in enumerate(((-0.088, -0.415), (0.088, -0.415), (-0.088, -0.495), (0.088, -0.495))):
        _add_cylinder(shank, f"ankle_bolt_{i}", 0.006, 0.008, (0.045, y, z), "x", zinc)

    foot = model.part("foot")
    _add_cylinder(foot, "ankle_hub", 0.045, 0.110, (0.0, 0.0, 0.0), "y", steel)
    _add_box(foot, "ankle_adapter", (0.090, 0.095, 0.085), (0.020, 0.0, -0.073), enamel)
    _add_box(foot, "adapter_plate", (0.140, 0.125, 0.026), (0.025, 0.0, -0.118), steel)
    _add_box(foot, "sole", (0.390, 0.170, 0.060), (0.105, 0.0, -0.145), rubber)
    _add_box(foot, "toe_cap", (0.125, 0.165, 0.035), (0.255, 0.0, -0.102), enamel)
    _add_box(foot, "heel_block", (0.105, 0.160, 0.035), (-0.095, 0.0, -0.102), enamel)
    _add_box(foot, "sole_tread_0", (0.050, 0.180, 0.012), (-0.025, 0.0, -0.181), dark)
    _add_box(foot, "sole_tread_1", (0.050, 0.180, 0.012), (0.095, 0.0, -0.181), dark)
    _add_box(foot, "sole_tread_2", (0.050, 0.180, 0.012), (0.215, 0.0, -0.181), dark)
    for i, (x, y) in enumerate(((-0.025, -0.045), (-0.025, 0.045), (0.075, -0.045), (0.075, 0.045))):
        _add_cylinder(foot, f"foot_bolt_{i}", 0.0065, 0.009, (x, y, -0.1005), "z", zinc)

    damping = MotionProperties(damping=1.5, friction=0.35)
    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=2.0, lower=-0.65, upper=1.05),
        motion_properties=damping,
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=230.0, velocity=2.2, lower=0.0, upper=2.05),
        motion_properties=damping,
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.440)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5, lower=-0.55, upper=0.70),
        motion_properties=damping,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_mount = object_model.get_part("hip_mount")
    thigh = object_model.get_part("thigh_shell")
    shank = object_model.get_part("shank_shell")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.allow_overlap(
        hip_mount,
        thigh,
        elem_a="hip_pin",
        elem_b="hip_hub",
        reason="The visible hip pin is intentionally captured through the thigh bearing hub.",
    )
    ctx.allow_overlap(
        thigh,
        shank,
        elem_a="knee_pin",
        elem_b="knee_hub",
        reason="The knee pin is intentionally nested through the shank bearing hub.",
    )
    ctx.allow_overlap(
        shank,
        foot,
        elem_a="ankle_pin",
        elem_b="ankle_hub",
        reason="The ankle pin is intentionally captured through the foot bearing hub.",
    )

    ctx.expect_within(
        thigh,
        hip_mount,
        axes="y",
        inner_elem="hip_hub",
        outer_elem="hip_pin",
        name="hip hub is retained on the pin span",
    )
    ctx.expect_overlap(
        hip_mount,
        thigh,
        axes="y",
        elem_a="hip_pin",
        elem_b="hip_hub",
        min_overlap=0.12,
        name="hip pin crosses the hub",
    )
    ctx.expect_within(
        shank,
        thigh,
        axes="y",
        inner_elem="knee_hub",
        outer_elem="knee_pin",
        name="knee hub is retained on the pin span",
    )
    ctx.expect_overlap(
        thigh,
        shank,
        axes="y",
        elem_a="knee_pin",
        elem_b="knee_hub",
        min_overlap=0.11,
        name="knee pin crosses the hub",
    )
    ctx.expect_within(
        foot,
        shank,
        axes="y",
        inner_elem="ankle_hub",
        outer_elem="ankle_pin",
        name="ankle hub is retained on the pin span",
    )
    ctx.expect_overlap(
        shank,
        foot,
        axes="y",
        elem_a="ankle_pin",
        elem_b="ankle_hub",
        min_overlap=0.09,
        name="ankle pin crosses the hub",
    )

    ctx.check(
        "serial pitch joints",
        hip.parent == "hip_mount"
        and hip.child == "thigh_shell"
        and knee.parent == "thigh_shell"
        and knee.child == "shank_shell"
        and ankle.parent == "shank_shell"
        and ankle.child == "foot",
        details="The leg must remain a hip-knee-ankle serial chain.",
    )
    ctx.check(
        "plausible joint limits",
        hip.motion_limits.lower < 0.0
        and hip.motion_limits.upper > 0.8
        and knee.motion_limits.lower == 0.0
        and knee.motion_limits.upper > 1.8
        and ankle.motion_limits.lower < -0.4
        and ankle.motion_limits.upper > 0.5,
        details="Hip, knee, and ankle pitch limits should be leg-like and mechanically bounded.",
    )

    rest_ankle = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.65}):
        flexed_ankle = ctx.part_world_position(foot)
    ctx.check(
        "hip flexes the distal chain forward",
        rest_ankle is not None and flexed_ankle is not None and flexed_ankle[0] > rest_ankle[0] + 0.20,
        details=f"rest ankle={rest_ankle}, hip-flexed ankle={flexed_ankle}",
    )

    with ctx.pose({knee: 0.75}):
        bent_ankle = ctx.part_world_position(foot)
    ctx.check(
        "knee bends the shank backward",
        rest_ankle is not None and bent_ankle is not None and bent_ankle[0] < rest_ankle[0] - 0.20,
        details=f"rest ankle={rest_ankle}, knee-bent ankle={bent_ankle}",
    )

    toe_rest = ctx.part_element_world_aabb(foot, elem="toe_cap")
    with ctx.pose({ankle: 0.50}):
        toe_lifted = ctx.part_element_world_aabb(foot, elem="toe_cap")
    ctx.check(
        "ankle dorsiflexion lifts the toe",
        toe_rest is not None and toe_lifted is not None and toe_lifted[1][2] > toe_rest[1][2] + 0.05,
        details=f"rest toe aabb={toe_rest}, lifted toe aabb={toe_lifted}",
    )

    return ctx.report()


object_model = build_object_model()
