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
    model = ArticulatedObject(name="robotic_leg_module")

    housing_paint = model.material("housing_paint", rgba=(0.26, 0.29, 0.33, 1.0))
    structure_dark = model.material("structure_dark", rgba=(0.13, 0.14, 0.16, 1.0))
    actuator_metal = model.material("actuator_metal", rgba=(0.60, 0.64, 0.70, 1.0))
    foot_pad = model.material("foot_pad", rgba=(0.09, 0.09, 0.10, 1.0))

    lateral_cylinder = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    upper_housing = model.part("upper_housing")
    upper_housing.visual(
        Box((0.19, 0.11, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=housing_paint,
        name="main_shell",
    )
    upper_housing.visual(
        Box((0.13, 0.085, 0.045)),
        origin=Origin(xyz=(0.03, 0.0, 0.138)),
        material=housing_paint,
        name="top_cap",
    )
    upper_housing.visual(
        Box((0.085, 0.018, 0.10)),
        origin=Origin(xyz=(0.01, 0.056, -0.055)),
        material=structure_dark,
        name="hip_clevis_right",
    )
    upper_housing.visual(
        Box((0.085, 0.018, 0.10)),
        origin=Origin(xyz=(0.01, -0.056, -0.055)),
        material=structure_dark,
        name="hip_clevis_left",
    )
    upper_housing.visual(
        Cylinder(radius=0.05, length=0.03),
        origin=Origin(xyz=(0.0, 0.066, -0.09), rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="hip_pod_right",
    )
    upper_housing.visual(
        Cylinder(radius=0.05, length=0.03),
        origin=Origin(xyz=(0.0, -0.066, -0.09), rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="hip_pod_left",
    )
    upper_housing.inertial = Inertial.from_geometry(
        Box((0.22, 0.13, 0.25)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.037, length=0.094),
        origin=Origin(rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="hip_drum",
    )
    thigh.visual(
        Box((0.085, 0.062, 0.12)),
        origin=Origin(xyz=(0.02, 0.0, -0.07)),
        material=structure_dark,
        name="upper_link_block",
    )
    thigh.visual(
        Box((0.07, 0.05, 0.23)),
        origin=Origin(xyz=(0.04, 0.0, -0.175)),
        material=structure_dark,
        name="main_link_beam",
    )
    thigh.visual(
        Box((0.05, 0.10, 0.036)),
        origin=Origin(xyz=(0.05, 0.0, -0.281)),
        material=structure_dark,
        name="knee_yoke",
    )
    thigh.visual(
        Box((0.06, 0.016, 0.10)),
        origin=Origin(xyz=(0.055, 0.045, -0.345)),
        material=structure_dark,
        name="knee_clevis_right",
    )
    thigh.visual(
        Box((0.06, 0.016, 0.10)),
        origin=Origin(xyz=(0.055, -0.045, -0.345)),
        material=structure_dark,
        name="knee_clevis_left",
    )
    thigh.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(0.055, 0.061, -0.345), rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="knee_pod_right",
    )
    thigh.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(0.055, -0.061, -0.345), rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="knee_pod_left",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.15, 0.09, 0.40)),
        mass=7.0,
        origin=Origin(xyz=(0.04, 0.0, -0.19)),
    )

    shank = model.part("shank")
    shank.visual(
        Cylinder(radius=0.034, length=0.074),
        origin=Origin(rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="knee_drum",
    )
    shank.visual(
        Box((0.07, 0.055, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, -0.055)),
        material=structure_dark,
        name="upper_shank_block",
    )
    shank.visual(
        Box((0.06, 0.045, 0.20)),
        origin=Origin(xyz=(0.028, 0.0, -0.175)),
        material=structure_dark,
        name="main_shank_beam",
    )
    shank.visual(
        Box((0.04, 0.088, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, -0.246)),
        material=structure_dark,
        name="ankle_yoke",
    )
    shank.visual(
        Box((0.045, 0.014, 0.08)),
        origin=Origin(xyz=(0.025, 0.04, -0.302)),
        material=structure_dark,
        name="ankle_clevis_right",
    )
    shank.visual(
        Box((0.045, 0.014, 0.08)),
        origin=Origin(xyz=(0.025, -0.04, -0.302)),
        material=structure_dark,
        name="ankle_clevis_left",
    )
    shank.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.025, 0.056, -0.302), rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="ankle_pod_right",
    )
    shank.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.025, -0.056, -0.302), rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="ankle_pod_left",
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.13, 0.08, 0.34)),
        mass=5.0,
        origin=Origin(xyz=(0.025, 0.0, -0.16)),
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.026, length=0.066),
        origin=Origin(rpy=lateral_cylinder.rpy),
        material=actuator_metal,
        name="ankle_drum",
    )
    foot.visual(
        Box((0.055, 0.05, 0.05)),
        origin=Origin(xyz=(0.015, 0.0, -0.03)),
        material=structure_dark,
        name="ankle_block",
    )
    foot.visual(
        Box((0.16, 0.08, 0.04)),
        origin=Origin(xyz=(0.04, 0.0, -0.062)),
        material=structure_dark,
        name="sole_body",
    )
    foot.visual(
        Box((0.055, 0.075, 0.026)),
        origin=Origin(xyz=(0.095, 0.0, -0.048)),
        material=housing_paint,
        name="toe_block",
    )
    foot.visual(
        Box((0.145, 0.075, 0.012)),
        origin=Origin(xyz=(0.04, 0.0, -0.085)),
        material=foot_pad,
        name="sole_pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.18, 0.09, 0.10)),
        mass=2.5,
        origin=Origin(xyz=(0.04, 0.0, -0.055)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.2,
            lower=math.radians(-40.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.055, 0.0, -0.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=2.8,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.025, 0.0, -0.302)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=3.2,
            lower=math.radians(-35.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper_housing = object_model.get_part("upper_housing")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.expect_contact(
        thigh,
        upper_housing,
        elem_a="hip_drum",
        elem_b="hip_clevis_right",
        name="hip drum seats on right upper clevis ear",
    )
    ctx.expect_contact(
        thigh,
        upper_housing,
        elem_a="hip_drum",
        elem_b="hip_clevis_left",
        name="hip drum seats on left upper clevis ear",
    )
    ctx.expect_contact(
        shank,
        thigh,
        elem_a="knee_drum",
        elem_b="knee_clevis_right",
        name="knee drum seats on right thigh clevis ear",
    )
    ctx.expect_contact(
        shank,
        thigh,
        elem_a="knee_drum",
        elem_b="knee_clevis_left",
        name="knee drum seats on left thigh clevis ear",
    )
    ctx.expect_contact(
        foot,
        shank,
        elem_a="ankle_drum",
        elem_b="ankle_clevis_right",
        name="ankle drum seats on right shank clevis ear",
    )
    ctx.expect_contact(
        foot,
        shank,
        elem_a="ankle_drum",
        elem_b="ankle_clevis_left",
        name="ankle drum seats on left shank clevis ear",
    )

    def element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
            0.5 * (aabb[0][2] + aabb[1][2]),
        )

    hip_center = element_center("thigh", "hip_drum")
    knee_center = element_center("shank", "knee_drum")
    ankle_center = element_center("foot", "ankle_drum")
    ctx.check(
        "joint axes are clearly separated in profile",
        hip_center is not None
        and knee_center is not None
        and ankle_center is not None
        and knee_center[2] < hip_center[2] - 0.20
        and ankle_center[2] < knee_center[2] - 0.22
        and knee_center[0] > hip_center[0] + 0.03
        and ankle_center[0] > hip_center[0] + 0.04,
        details=f"hip={hip_center}, knee={knee_center}, ankle={ankle_center}",
    )

    rest_knee_center = element_center("shank", "knee_drum")
    with ctx.pose({hip_pitch: hip_pitch.motion_limits.upper}):
        flexed_knee_center = element_center("shank", "knee_drum")

    ctx.check(
        "positive hip motion swings the thigh forward",
        rest_knee_center is not None
        and flexed_knee_center is not None
        and flexed_knee_center[0] > rest_knee_center[0] + 0.10,
        details=f"rest={rest_knee_center}, flexed={flexed_knee_center}",
    )

    rest_ankle_center = element_center("foot", "ankle_drum")
    with ctx.pose({knee_pitch: knee_pitch.motion_limits.upper}):
        folded_ankle_center = element_center("foot", "ankle_drum")

    ctx.check(
        "positive knee motion folds the shank backward",
        rest_ankle_center is not None
        and folded_ankle_center is not None
        and folded_ankle_center[0] < rest_ankle_center[0] - 0.12,
        details=f"rest={rest_ankle_center}, folded={folded_ankle_center}",
    )

    rest_toe_center = element_center("foot", "toe_block")
    with ctx.pose({ankle_pitch: ankle_pitch.motion_limits.upper}):
        raised_toe_center = element_center("foot", "toe_block")

    ctx.check(
        "positive ankle motion lifts the toe",
        rest_toe_center is not None
        and raised_toe_center is not None
        and raised_toe_center[2] > rest_toe_center[2] + 0.035,
        details=f"rest={rest_toe_center}, raised={raised_toe_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
