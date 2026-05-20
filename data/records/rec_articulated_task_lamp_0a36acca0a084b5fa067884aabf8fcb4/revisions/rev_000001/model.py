from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_led_strip_lamp")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.75, 0.77, 0.79, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    housing_white = model.material("housing_white", rgba=(0.95, 0.96, 0.97, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.97, 0.86, 0.78))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.010, 0.090, 0.160)),
        origin=Origin(xyz=(0.005, 0.0, 0.080)),
        material=brushed_aluminum,
        name="mount_plate",
    )
    wall_bracket.visual(
        Box((0.020, 0.040, 0.036)),
        origin=Origin(xyz=(0.015, 0.0, 0.028)),
        material=brushed_aluminum,
        name="lower_stem",
    )
    wall_bracket.visual(
        Box((0.020, 0.012, 0.034)),
        origin=Origin(xyz=(0.019, 0.015, 0.070)),
        material=brushed_aluminum,
        name="shoulder_cheek_left",
    )
    wall_bracket.visual(
        Box((0.020, 0.012, 0.034)),
        origin=Origin(xyz=(0.019, -0.015, 0.070)),
        material=brushed_aluminum,
        name="shoulder_cheek_right",
    )
    wall_bracket.visual(
        Box((0.018, 0.010, 0.036)),
        origin=Origin(xyz=(0.033, 0.014, 0.070)),
        material=graphite,
        name="shoulder_ear_left",
    )
    wall_bracket.visual(
        Box((0.018, 0.010, 0.036)),
        origin=Origin(xyz=(0.033, -0.014, 0.070)),
        material=graphite,
        name="shoulder_ear_right",
    )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.050, 0.100, 0.170)),
        mass=0.65,
        origin=Origin(xyz=(0.020, 0.0, 0.085)),
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="shoulder_hub",
    )
    first_arm.visual(
        Box((0.150, 0.018, 0.008)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=brushed_aluminum,
        name="first_arm_beam",
    )
    first_arm.visual(
        Box((0.012, 0.028, 0.008)),
        origin=Origin(xyz=(0.144, 0.0, 0.0)),
        material=brushed_aluminum,
        name="elbow_bridge",
    )
    first_arm.visual(
        Box((0.022, 0.010, 0.028)),
        origin=Origin(xyz=(0.160, 0.014, 0.0)),
        material=graphite,
        name="elbow_ear_left",
    )
    first_arm.visual(
        Box((0.022, 0.010, 0.028)),
        origin=Origin(xyz=(0.160, -0.014, 0.0)),
        material=graphite,
        name="elbow_ear_right",
    )
    first_arm.inertial = Inertial.from_geometry(
        Box((0.170, 0.030, 0.030)),
        mass=0.28,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_hub",
    )
    second_arm.visual(
        Box((0.135, 0.018, 0.008)),
        origin=Origin(xyz=(0.0675, 0.0, 0.0)),
        material=brushed_aluminum,
        name="second_arm_beam",
    )
    second_arm.visual(
        Box((0.014, 0.030, 0.012)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=brushed_aluminum,
        name="head_bridge",
    )
    second_arm.visual(
        Box((0.018, 0.008, 0.024)),
        origin=Origin(xyz=(0.145, 0.010, 0.0)),
        material=graphite,
        name="head_ear_left",
    )
    second_arm.visual(
        Box((0.018, 0.008, 0.024)),
        origin=Origin(xyz=(0.145, -0.010, 0.0)),
        material=graphite,
        name="head_ear_right",
    )
    second_arm.inertial = Inertial.from_geometry(
        Box((0.155, 0.030, 0.035)),
        mass=0.24,
        origin=Origin(xyz=(0.073, 0.0, 0.0)),
    )

    led_bar = model.part("led_bar")
    led_bar.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="terminal_hub",
    )
    led_bar.visual(
        Box((0.010, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=graphite,
        name="terminal_web",
    )
    led_bar.visual(
        Box((0.018, 0.290, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=housing_white,
        name="bar_housing",
    )
    led_bar.visual(
        Box((0.016, 0.270, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=warm_diffuser,
        name="diffuser",
    )
    led_bar.inertial = Inertial.from_geometry(
        Box((0.030, 0.300, 0.050)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    model.articulation(
        "bracket_to_first_arm",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=first_arm,
        origin=Origin(xyz=(0.033, 0.0, 0.070)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-1.30,
            upper=1.10,
        ),
    )
    model.articulation(
        "first_to_second_arm",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-2.10,
            upper=1.20,
        ),
    )
    model.articulation(
        "second_arm_to_led_bar",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=led_bar,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.20,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_bracket = object_model.get_part("wall_bracket")
    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    led_bar = object_model.get_part("led_bar")

    shoulder = object_model.get_articulation("bracket_to_first_arm")
    elbow = object_model.get_articulation("first_to_second_arm")
    lamp_pitch = object_model.get_articulation("second_arm_to_led_bar")

    ctx.expect_contact(first_arm, wall_bracket, name="first arm is carried by the wall bracket hinge")
    ctx.expect_contact(second_arm, first_arm, name="second arm is carried by the elbow hinge")
    ctx.expect_contact(led_bar, second_arm, name="LED bar is carried by the terminal hinge")

    elbow_rest = ctx.part_world_position(second_arm)
    with ctx.pose({shoulder: 0.60}):
        elbow_raised = ctx.part_world_position(second_arm)
    ctx.check(
        "shoulder revolute joint raises the elbow",
        elbow_rest is not None and elbow_raised is not None and elbow_raised[2] > elbow_rest[2] + 0.05,
        details=f"rest={elbow_rest}, raised={elbow_raised}",
    )

    lamp_rest = ctx.part_world_position(led_bar)
    with ctx.pose({elbow: 0.85}):
        lamp_raised = ctx.part_world_position(led_bar)
    ctx.check(
        "elbow revolute joint lifts the lamp head",
        lamp_rest is not None and lamp_raised is not None and lamp_raised[2] > lamp_rest[2] + 0.04,
        details=f"rest={lamp_rest}, raised={lamp_raised}",
    )

    rest_diffuser = ctx.part_element_world_aabb(led_bar, elem="diffuser")
    with ctx.pose({lamp_pitch: 0.75}):
        pitched_diffuser = ctx.part_element_world_aabb(led_bar, elem="diffuser")

    rest_z_span = None if rest_diffuser is None else rest_diffuser[1][2] - rest_diffuser[0][2]
    pitched_z_span = None if pitched_diffuser is None else pitched_diffuser[1][2] - pitched_diffuser[0][2]
    ctx.check(
        "LED bar rotates around the terminal axis",
        rest_z_span is not None and pitched_z_span is not None and pitched_z_span > rest_z_span + 0.08,
        details=f"rest_z_span={rest_z_span}, pitched_z_span={pitched_z_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
