from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    housing_dark = model.material("housing_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    housing_mid = model.material("housing_mid", rgba=(0.33, 0.36, 0.40, 1.0))
    link_grey = model.material("link_grey", rgba=(0.64, 0.66, 0.70, 1.0))
    joint_dark = model.material("joint_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.12, 0.13, 0.14, 1.0))

    upper_housing = model.part("upper_leg_housing")
    upper_housing.visual(
        Box((0.088, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=housing_dark,
        name="main_pod",
    )
    upper_housing.visual(
        Box((0.050, 0.090, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=housing_mid,
        name="mounting_cap",
    )
    upper_housing.visual(
        Box((0.012, 0.056, 0.042)),
        origin=Origin(xyz=(-0.034, 0.0, -0.004)),
        material=housing_dark,
        name="left_yoke",
    )
    upper_housing.visual(
        Box((0.012, 0.056, 0.042)),
        origin=Origin(xyz=(0.034, 0.0, -0.004)),
        material=housing_dark,
        name="right_yoke",
    )
    upper_housing.visual(
        Box((0.044, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.022, 0.026)),
        material=housing_mid,
        name="front_nose",
    )
    upper_housing.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.045, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=joint_dark,
        name="left_hip_cap",
    )
    upper_housing.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=joint_dark,
        name="right_hip_cap",
    )
    upper_housing.inertial = Inertial.from_geometry(
        Box((0.088, 0.090, 0.078)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    thigh = model.part("thigh_link")
    thigh.visual(
        Cylinder(radius=0.013, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=joint_dark,
        name="hip_hub",
    )
    thigh.visual(
        Box((0.044, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=link_grey,
        name="upper_knuckle",
    )
    thigh.visual(
        Box((0.032, 0.046, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.0675)),
        material=link_grey,
        name="thigh_beam",
    )
    thigh.visual(
        Box((0.010, 0.042, 0.030)),
        origin=Origin(xyz=(-0.017, 0.0, -0.130)),
        material=link_grey,
        name="left_knee_ear",
    )
    thigh.visual(
        Box((0.010, 0.042, 0.030)),
        origin=Origin(xyz=(0.017, 0.0, -0.130)),
        material=link_grey,
        name="right_knee_ear",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.048, 0.046, 0.145)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.0725)),
    )

    shank = model.part("shank_link")
    shank.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=joint_dark,
        name="knee_hub",
    )
    shank.visual(
        Box((0.020, 0.032, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=link_grey,
        name="upper_shell",
    )
    shank.visual(
        Box((0.028, 0.040, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.0565)),
        material=link_grey,
        name="shank_beam",
    )
    shank.visual(
        Box((0.018, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, -0.011, -0.058)),
        material=housing_mid,
        name="rear_rib",
    )
    shank.visual(
        Box((0.010, 0.034, 0.030)),
        origin=Origin(xyz=(-0.015, 0.0, -0.110)),
        material=link_grey,
        name="left_ankle_ear",
    )
    shank.visual(
        Box((0.010, 0.034, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, -0.110)),
        material=link_grey,
        name="right_ankle_ear",
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.042, 0.040, 0.122)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
    )

    foot = model.part("ankle_foot")
    foot.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=joint_dark,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.016, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=housing_mid,
        name="ankle_shell",
    )
    foot.visual(
        Box((0.028, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, -0.029)),
        material=housing_mid,
        name="foot_spine",
    )
    foot.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, 0.010, -0.040)),
        material=foot_rubber,
        name="sole",
    )
    foot.visual(
        Box((0.036, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, -0.026, -0.032)),
        material=foot_rubber,
        name="heel_pad",
    )
    foot.visual(
        Box((0.032, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.045, -0.031)),
        material=foot_rubber,
        name="toe_pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.050, 0.090, 0.046)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.010, -0.023)),
    )

    model.articulation(
        "hip_joint",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-1.0,
            upper=1.2,
        ),
    )
    model.articulation(
        "knee_joint",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=3.5,
            lower=0.0,
            upper=2.2,
        ),
    )
    model.articulation(
        "ankle_joint",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.113)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
            lower=-0.7,
            upper=0.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("upper_leg_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")

    hip = object_model.get_articulation("hip_joint")
    knee = object_model.get_articulation("knee_joint")
    ankle = object_model.get_articulation("ankle_joint")

    ctx.check(
        "all joints use a lateral hinge axis",
        hip.axis == (1.0, 0.0, 0.0)
        and knee.axis == (1.0, 0.0, 0.0)
        and ankle.axis == (1.0, 0.0, 0.0),
        details=f"hip={hip.axis}, knee={knee.axis}, ankle={ankle.axis}",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    thigh_aabb = ctx.part_world_aabb(thigh)
    shank_aabb = ctx.part_world_aabb(shank)
    foot_aabb = ctx.part_world_aabb(foot)
    ctx.check(
        "rest pose stacks into a descending leg chain",
        housing_aabb is not None
        and thigh_aabb is not None
        and shank_aabb is not None
        and foot_aabb is not None
        and thigh_aabb[0][2] < housing_aabb[0][2] - 0.05
        and shank_aabb[0][2] < thigh_aabb[0][2] - 0.07
        and foot_aabb[0][2] < shank_aabb[0][2] - 0.02,
        details=(
            f"housing={housing_aabb}, thigh={thigh_aabb}, "
            f"shank={shank_aabb}, foot={foot_aabb}"
        ),
    )

    rest_thigh_beam = ctx.part_element_world_aabb(thigh, elem="thigh_beam")
    with ctx.pose({hip: 0.75}):
        flexed_thigh_beam = ctx.part_element_world_aabb(thigh, elem="thigh_beam")
    ctx.check(
        "hip flexion swings the thigh forward",
        rest_thigh_beam is not None
        and flexed_thigh_beam is not None
        and ((flexed_thigh_beam[0][1] + flexed_thigh_beam[1][1]) * 0.5)
        > ((rest_thigh_beam[0][1] + rest_thigh_beam[1][1]) * 0.5) + 0.04,
        details=f"rest={rest_thigh_beam}, flexed={flexed_thigh_beam}",
    )

    rest_foot_pos = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.3}):
        knee_flexed_foot_pos = ctx.part_world_position(foot)
    ctx.check(
        "knee flexion tucks the foot upward and forward",
        rest_foot_pos is not None
        and knee_flexed_foot_pos is not None
        and knee_flexed_foot_pos[1] > rest_foot_pos[1] + 0.08
        and knee_flexed_foot_pos[2] > rest_foot_pos[2] + 0.04,
        details=f"rest={rest_foot_pos}, flexed={knee_flexed_foot_pos}",
    )

    rest_toe = ctx.part_element_world_aabb(foot, elem="toe_pad")
    with ctx.pose({ankle: 0.45}):
        raised_toe = ctx.part_element_world_aabb(foot, elem="toe_pad")
    ctx.check(
        "ankle articulation changes toe height",
        rest_toe is not None
        and raised_toe is not None
        and raised_toe[1][2] > rest_toe[1][2] + 0.01,
        details=f"rest={rest_toe}, raised={raised_toe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
