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
    model = ArticulatedObject(name="compact_service_manipulator")

    paint = model.material("industrial_blue", rgba=(0.08, 0.18, 0.30, 1.0))
    link_paint = model.material("safety_yellow", rgba=(0.92, 0.64, 0.10, 1.0))
    dark = model.material("dark_bearing", rgba=(0.03, 0.035, 0.04, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    def cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
        return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    def cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
        return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    def add_cylinder_y(part, name: str, radius: float, length: float, xyz, material: Material) -> None:
        geom, rot = cyl_y(radius, length)
        part.visual(geom, origin=Origin(xyz=xyz, rpy=rot.rpy), material=material, name=name)

    def add_cylinder_x(part, name: str, radius: float, length: float, xyz, material: Material) -> None:
        geom, rot = cyl_x(radius, length)
        part.visual(geom, origin=Origin(xyz=xyz, rpy=rot.rpy), material=material, name=name)

    base = model.part("base_column")
    base.visual(Box((0.46, 0.46, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=dark, name="floor_plinth")
    base.visual(Box((0.245, 0.245, 0.480)), origin=Origin(xyz=(0.0, 0.0, 0.300)), material=paint, name="square_column")
    base.visual(Cylinder(radius=0.175, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.560)), material=steel, name="top_flange")
    base.visual(Cylinder(radius=0.155, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.580)), material=dark, name="top_bearing")
    base.visual(Cylinder(radius=0.120, length=0.020), origin=Origin(xyz=(0.0, 0.0, 0.590)), material=steel, name="thrust_washer")

    for idx, (x, y) in enumerate(((-0.165, -0.165), (-0.165, 0.165), (0.165, -0.165), (0.165, 0.165))):
        base.visual(Cylinder(radius=0.016, length=0.012), origin=Origin(xyz=(x, y, 0.066)), material=steel, name=f"plinth_bolt_{idx}")

    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = 0.132 * math.cos(angle)
        y = 0.132 * math.sin(angle)
        base.visual(Cylinder(radius=0.010, length=0.010), origin=Origin(xyz=(x, y, 0.605)), material=steel, name=f"flange_bolt_{idx}")

    upper = model.part("upper_link")
    upper.visual(Cylinder(radius=0.145, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=dark, name="shoulder_bearing")
    upper.visual(Box((0.220, 0.180, 0.120)), origin=Origin(xyz=(0.0, 0.0, 0.110)), material=paint, name="shoulder_housing")
    upper.visual(Box((0.160, 0.135, 0.050)), origin=Origin(xyz=(0.055, 0.0, 0.175)), material=paint, name="shoulder_neck")

    arm_start = (0.055, 0.0, 0.175)
    arm_end = (0.365, 0.0, 0.300)
    dx = arm_end[0] - arm_start[0]
    dz = arm_end[2] - arm_start[2]
    arm_len = math.sqrt(dx * dx + dz * dz)
    arm_angle = -math.atan2(dz, dx)
    arm_center = ((arm_start[0] + arm_end[0]) / 2.0, 0.0, (arm_start[2] + arm_end[2]) / 2.0)
    upper.visual(Box((arm_len, 0.096, 0.070)), origin=Origin(xyz=arm_center, rpy=(0.0, arm_angle, 0.0)), material=link_paint, name="upper_beam")
    upper.visual(Box((arm_len * 0.86, 0.020, 0.090)), origin=Origin(xyz=(arm_center[0], 0.056, arm_center[2]), rpy=(0.0, arm_angle, 0.0)), material=paint, name="upper_rib_pos")
    upper.visual(Box((arm_len * 0.86, 0.020, 0.090)), origin=Origin(xyz=(arm_center[0], -0.056, arm_center[2]), rpy=(0.0, arm_angle, 0.0)), material=paint, name="upper_rib_neg")

    upper.visual(Box((0.080, 0.140, 0.086)), origin=Origin(xyz=(0.385, 0.0, 0.332)), material=paint, name="elbow_bridge")
    upper.visual(Box((0.098, 0.026, 0.185)), origin=Origin(xyz=(0.465, 0.065, 0.340)), material=paint, name="elbow_cheek_pos")
    upper.visual(Box((0.098, 0.026, 0.185)), origin=Origin(xyz=(0.465, -0.065, 0.340)), material=paint, name="elbow_cheek_neg")
    add_cylinder_y(upper, "elbow_cap_pos", 0.058, 0.014, (0.465, 0.085, 0.340), steel)
    add_cylinder_y(upper, "elbow_cap_neg", 0.058, 0.014, (0.465, -0.085, 0.340), steel)
    for idx, (zoff, xoff) in enumerate(((0.038, 0.0), (-0.038, 0.0), (0.0, 0.038), (0.0, -0.038))):
        add_cylinder_y(upper, f"elbow_cap_bolt_pos_{idx}", 0.006, 0.006, (0.465 + xoff, 0.095, 0.340 + zoff), steel)
        add_cylinder_y(upper, f"elbow_cap_bolt_neg_{idx}", 0.006, 0.006, (0.465 + xoff, -0.095, 0.340 + zoff), steel)

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.043, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(Box((0.260, 0.080, 0.066)), origin=Origin(xyz=(0.155, 0.0, 0.0)), material=link_paint, name="forearm_beam")
    forearm.visual(Box((0.215, 0.020, 0.082)), origin=Origin(xyz=(0.165, 0.046, 0.0)), material=paint, name="forearm_rib_pos")
    forearm.visual(Box((0.215, 0.020, 0.082)), origin=Origin(xyz=(0.165, -0.046, 0.0)), material=paint, name="forearm_rib_neg")
    forearm.visual(Box((0.050, 0.142, 0.104)), origin=Origin(xyz=(0.278, 0.0, 0.0)), material=paint, name="wrist_bridge")
    forearm.visual(Box((0.080, 0.026, 0.150)), origin=Origin(xyz=(0.340, 0.066, 0.0)), material=paint, name="wrist_cheek_pos")
    forearm.visual(Box((0.080, 0.026, 0.150)), origin=Origin(xyz=(0.340, -0.066, 0.0)), material=paint, name="wrist_cheek_neg")
    forearm.visual(
        Cylinder(radius=0.018, length=0.118),
        origin=Origin(xyz=(0.340, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_axle",
    )
    add_cylinder_y(forearm, "wrist_cap_pos", 0.044, 0.012, (0.340, 0.085, 0.0), steel)
    add_cylinder_y(forearm, "wrist_cap_neg", 0.044, 0.012, (0.340, -0.085, 0.0), steel)

    wrist = model.part("wrist_block")
    wrist.visual(
        Cylinder(radius=0.034, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_hub",
    )
    wrist.visual(Box((0.124, 0.088, 0.074)), origin=Origin(xyz=(0.082, 0.0, 0.0)), material=paint, name="nose_block")
    add_cylinder_x(wrist, "tool_flange", 0.046, 0.022, (0.151, 0.0, 0.0), steel)
    add_cylinder_x(wrist, "rubber_face", 0.033, 0.010, (0.168, 0.0, 0.0), rubber)
    for idx, (y, z) in enumerate(((-0.027, -0.027), (-0.027, 0.027), (0.027, -0.027), (0.027, 0.027))):
        add_cylinder_x(wrist, f"flange_bolt_{idx}", 0.0055, 0.006, (0.165, y, z), steel)

    model.articulation(
        "base_to_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.465, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.1, lower=-0.55, upper=0.82),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_block")
    shoulder = object_model.get_articulation("base_to_upper")
    elbow = object_model.get_articulation("upper_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    ctx.check(
        "three purposeful revolute joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder, elbow, wrist_joint)
        ),
        details="The manipulator should expose shoulder yaw, elbow pitch, and wrist pitch only.",
    )
    ctx.check(
        "joint axes match manipulator layout",
        shoulder.axis == (0.0, 0.0, 1.0) and elbow.axis == (0.0, -1.0, 0.0) and wrist_joint.axis == (0.0, -1.0, 0.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist_joint.axis}",
    )

    ctx.expect_contact(
        upper,
        base,
        elem_a="shoulder_bearing",
        elem_b="top_bearing",
        contact_tol=0.012,
        name="rotating shoulder bearing is seated on base bearing",
    )
    ctx.expect_gap(
        upper,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_pos",
        negative_elem="elbow_hub",
        min_gap=0.003,
        max_gap=0.012,
        name="positive elbow cheek clears moving hub",
    )
    ctx.expect_gap(
        forearm,
        upper,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_neg",
        min_gap=0.003,
        max_gap=0.012,
        name="negative elbow cheek clears moving hub",
    )
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        positive_elem="wrist_cheek_pos",
        negative_elem="wrist_hub",
        min_gap=0.006,
        max_gap=0.018,
        name="positive wrist cheek clears moving hub",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        positive_elem="wrist_hub",
        negative_elem="wrist_cheek_neg",
        min_gap=0.006,
        max_gap=0.018,
        name="negative wrist cheek clears moving hub",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_axle",
        elem_b="wrist_hub",
        reason="The steel wrist axle is intentionally captured through the rotating wrist hub.",
    )
    ctx.expect_within(
        forearm,
        wrist,
        axes="xz",
        inner_elem="wrist_axle",
        outer_elem="wrist_hub",
        margin=0.001,
        name="wrist axle remains centered inside the wrist hub bore",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="y",
        elem_a="wrist_axle",
        elem_b="wrist_hub",
        min_overlap=0.075,
        name="wrist axle spans through the captured hub",
    )

    with ctx.pose({elbow: 0.70, wrist_joint: -0.70}):
        ctx.expect_gap(
            forearm,
            upper,
            axis="x",
            positive_elem="forearm_beam",
            negative_elem="elbow_bridge",
            min_gap=0.030,
            name="raised forearm stays forward of elbow bridge",
        )
        ctx.expect_gap(
            wrist,
            forearm,
            axis="x",
            positive_elem="nose_block",
            negative_elem="wrist_bridge",
            min_gap=0.005,
            name="wrist block stays ahead of nose bridge while pitched",
        )

    with ctx.pose({elbow: -0.45, wrist_joint: 0.70}):
        ctx.expect_gap(
            forearm,
            upper,
            axis="x",
            positive_elem="forearm_beam",
            negative_elem="elbow_bridge",
            min_gap=0.028,
            name="lowered forearm clears the elbow support bridge",
        )

    return ctx.report()


object_model = build_object_model()
