from __future__ import annotations

from math import pi

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


PIN_AXIS_Y = Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=PIN_AXIS_Y.rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_shoulder_elbow_wrist")

    dark_plate = Material("dark_anodized_plate", rgba=(0.08, 0.09, 0.10, 1.0))
    blue_arm = Material("blue_powder_coat", rgba=(0.05, 0.27, 0.62, 1.0))
    light_arm = Material("light_blue_powder_coat", rgba=(0.08, 0.45, 0.78, 1.0))
    joint_orange = Material("orange_joint_caps", rgba=(0.92, 0.42, 0.08, 1.0))
    rubber_black = Material("black_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))
    machined = Material("machined_aluminum", rgba=(0.62, 0.66, 0.68, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.26, 0.024, 0.42)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=dark_plate,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.16, 0.030, 0.16)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=dark_plate,
        name="shoulder_backing",
    )
    side_plate.visual(
        Cylinder(radius=0.074, length=0.036),
        origin=y_cylinder_origin(0.0, 0.030, 0.0),
        material=machined,
        name="shoulder_housing",
    )
    side_plate.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=y_cylinder_origin(0.0, 0.048, 0.0),
        material=dark_plate,
        name="shoulder_bearing_face",
    )
    for i, (x, z) in enumerate(
        ((-0.125, -0.165), (-0.125, 0.165), (0.035, -0.165), (0.035, 0.165))
    ):
        side_plate.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=y_cylinder_origin(x, 0.014, z),
            material=rubber_black,
            name=f"mount_bolt_{i}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.060, length=0.060),
        origin=y_cylinder_origin(0.0, 0.083, 0.0),
        material=joint_orange,
        name="shoulder_rotor",
    )
    upper_arm.visual(
        Box((0.420, 0.046, 0.056)),
        origin=Origin(xyz=(0.240, 0.083, 0.0)),
        material=blue_arm,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.340, 0.052, 0.022)),
        origin=Origin(xyz=(0.250, 0.083, 0.039)),
        material=blue_arm,
        name="upper_top_rib",
    )
    upper_arm.visual(
        Box((0.340, 0.052, 0.022)),
        origin=Origin(xyz=(0.250, 0.083, -0.039)),
        material=blue_arm,
        name="upper_bottom_rib",
    )
    upper_arm.visual(
        Cylinder(radius=0.048, length=0.060),
        origin=y_cylinder_origin(0.480, 0.083, 0.0),
        material=machined,
        name="elbow_housing",
    )
    upper_arm.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=y_cylinder_origin(0.480, 0.047, 0.0),
        material=rubber_black,
        name="elbow_pin_cap",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.044, length=0.055),
        origin=y_cylinder_origin(0.0, 0.0575, 0.0),
        material=joint_orange,
        name="elbow_rotor",
    )
    forearm.visual(
        Box((0.320, 0.042, 0.048)),
        origin=Origin(xyz=(0.180, 0.0575, 0.0)),
        material=light_arm,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.270, 0.048, 0.018)),
        origin=Origin(xyz=(0.190, 0.0575, 0.0325)),
        material=light_arm,
        name="forearm_top_rib",
    )
    forearm.visual(
        Box((0.270, 0.048, 0.018)),
        origin=Origin(xyz=(0.190, 0.0575, -0.0325)),
        material=light_arm,
        name="forearm_bottom_rib",
    )
    forearm.visual(
        Cylinder(radius=0.036, length=0.047),
        origin=y_cylinder_origin(0.360, 0.0575, 0.0),
        material=machined,
        name="wrist_housing",
    )
    forearm.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=y_cylinder_origin(0.360, 0.0285, 0.0),
        material=rubber_black,
        name="wrist_pin_cap",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=y_cylinder_origin(0.0, 0.0485, 0.0),
        material=joint_orange,
        name="wrist_rotor",
    )
    wrist.visual(
        Box((0.140, 0.036, 0.038)),
        origin=Origin(xyz=(0.083, 0.0485, 0.0)),
        material=machined,
        name="wrist_link",
    )
    wrist.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=y_cylinder_origin(0.158, 0.0485, 0.0),
        material=machined,
        name="tool_flange",
    )
    wrist.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=y_cylinder_origin(0.166, 0.0485, 0.0),
        material=rubber_black,
        name="flange_bore",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.480, 0.083, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.2, lower=-1.45, upper=1.70),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.360, 0.0575, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.5, lower=-1.75, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist")

    ctx.expect_gap(
        upper_arm,
        side_plate,
        axis="y",
        positive_elem="shoulder_rotor",
        negative_elem="shoulder_bearing_face",
        max_gap=0.001,
        max_penetration=0.0,
        name="shoulder rotor seated against grounded bearing face",
    )
    ctx.expect_overlap(
        upper_arm,
        side_plate,
        axes="xz",
        elem_a="shoulder_rotor",
        elem_b="shoulder_housing",
        min_overlap=0.070,
        name="shoulder rotor is coaxial with grounded housing",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_rotor",
        negative_elem="elbow_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="forearm elbow rotor seated against upper arm housing",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        elem_a="elbow_rotor",
        elem_b="elbow_housing",
        min_overlap=0.055,
        name="elbow rotor is coaxial with upper arm housing",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        positive_elem="wrist_rotor",
        negative_elem="wrist_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="wrist rotor seated against forearm housing",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="xz",
        elem_a="wrist_rotor",
        elem_b="wrist_housing",
        min_overlap=0.045,
        name="wrist rotor is coaxial with forearm housing",
    )

    rest_forearm = ctx.part_world_position(forearm)
    rest_wrist = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.55}):
        raised_forearm = ctx.part_world_position(forearm)
        raised_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "positive shoulder motion lifts the serial arm",
        rest_forearm is not None
        and rest_wrist is not None
        and raised_forearm is not None
        and raised_wrist is not None
        and raised_forearm[2] > rest_forearm[2] + 0.18
        and raised_wrist[2] > rest_wrist[2] + 0.30,
        details=f"rest_forearm={rest_forearm}, raised_forearm={raised_forearm}, "
        f"rest_wrist={rest_wrist}, raised_wrist={raised_wrist}",
    )

    rest_wrist_aabb = ctx.part_element_world_aabb(wrist, elem="wrist_link")
    with ctx.pose({elbow: 0.60}):
        bent_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "positive elbow motion lifts the wrist joint",
        rest_wrist is not None
        and bent_wrist is not None
        and bent_wrist[2] > rest_wrist[2] + 0.16,
        details=f"rest_wrist={rest_wrist}, bent_wrist={bent_wrist}",
    )

    with ctx.pose({wrist_joint: 0.70}):
        rotated_wrist_aabb = ctx.part_element_world_aabb(wrist, elem="wrist_link")
    rest_center_z = (
        (rest_wrist_aabb[0][2] + rest_wrist_aabb[1][2]) / 2.0
        if rest_wrist_aabb is not None
        else None
    )
    rotated_center_z = (
        (rotated_wrist_aabb[0][2] + rotated_wrist_aabb[1][2]) / 2.0
        if rotated_wrist_aabb is not None
        else None
    )
    ctx.check(
        "positive wrist motion pitches the short wrist member",
        rest_center_z is not None
        and rotated_center_z is not None
        and rotated_center_z > rest_center_z + 0.040,
        details=f"rest_center_z={rest_center_z}, rotated_center_z={rotated_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
