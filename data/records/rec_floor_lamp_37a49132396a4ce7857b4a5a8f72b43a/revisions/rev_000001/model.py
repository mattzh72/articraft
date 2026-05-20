from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_HEIGHT = 0.055
SHOULDER_Z = 0.95
LOWER_ARM_LENGTH = 0.39
UPPER_ARM_LENGTH = 0.33
YOKE_AXIS_Y = 0.348


def _merge_meshes(*meshes: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _base_mesh() -> MeshGeometry:
    base_plate = LatheGeometry(
        [
            (0.0, 0.0),
            (0.138, 0.0),
            (0.162, 0.006),
            (0.176, 0.018),
            (0.176, 0.041),
            (0.158, 0.052),
            (0.110, BASE_HEIGHT),
            (0.0, BASE_HEIGHT),
        ],
        segments=72,
    )

    base_collar = CylinderGeometry(radius=0.034, height=0.012, radial_segments=48).translate(
        0.0,
        0.0,
        BASE_HEIGHT + 0.006,
    )
    mast = CylinderGeometry(radius=0.014, height=0.838, radial_segments=40).translate(
        0.0,
        0.0,
        0.486,
    )
    shoulder_backbone = BoxGeometry((0.034, 0.022, 0.048)).translate(0.0, -0.025, 0.914)
    shoulder_cheek_0 = BoxGeometry((0.010, 0.030, 0.050)).translate(0.021, -0.003, 0.925)
    shoulder_cheek_1 = BoxGeometry((0.010, 0.030, 0.050)).translate(-0.021, -0.003, 0.925)

    return _merge_meshes(
        base_plate,
        base_collar,
        mast,
        shoulder_backbone,
        shoulder_cheek_0,
        shoulder_cheek_1,
    )


def _shade_mesh() -> MeshGeometry:
    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.010),
            (0.040, 0.004),
            (0.050, 0.032),
            (0.060, 0.096),
            (0.068, 0.154),
            (0.072, 0.186),
        ],
        [
            (0.026, -0.008),
            (0.031, 0.006),
            (0.040, 0.034),
            (0.053, 0.096),
            (0.060, 0.154),
            (0.066, 0.184),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    ).rotate_x(-math.pi / 2.0)
    rear_collar = CylinderGeometry(radius=0.034, height=0.012, radial_segments=40).rotate_x(
        -math.pi / 2.0
    ).translate(0.0, -0.004, 0.0)
    trunnion_0 = CylinderGeometry(radius=0.010, height=0.016, radial_segments=32).rotate_y(
        math.pi / 2.0
    ).translate(0.048, 0.0, 0.0)
    trunnion_1 = CylinderGeometry(radius=0.010, height=0.016, radial_segments=32).rotate_y(
        math.pi / 2.0
    ).translate(-0.048, 0.0, 0.0)

    return _merge_meshes(shade_shell, rear_collar, trunnion_0, trunnion_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))

    base = model.part("base")
    base.visual(mesh_from_geometry(_base_mesh(), "base"), material=matte_black, name="base_shell")
    base.visual(
        Box((0.010, 0.030, 0.050)),
        origin=Origin(xyz=(0.021, -0.003, 0.925)),
        material=graphite,
        name="shoulder_cheek_0",
    )
    base.visual(
        Box((0.010, 0.030, 0.050)),
        origin=Origin(xyz=(-0.021, -0.003, 0.925)),
        material=graphite,
        name="shoulder_cheek_1",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Box((0.020, 0.37, 0.016)),
        origin=Origin(xyz=(0.0, 0.185, 0.0)),
        material=satin_steel,
        name="lower_beam",
    )
    lower_arm.visual(
        Box((0.014, 0.28, 0.008)),
        origin=Origin(xyz=(0.0, 0.175, 0.010)),
        material=satin_steel,
        name="lower_strap",
    )
    lower_arm.visual(
        Box((0.040, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.366, 0.0)),
        material=graphite,
        name="elbow_crosshead",
    )
    lower_arm.visual(
        Cylinder(radius=0.013, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="shoulder_hub",
    )
    lower_arm.visual(
        Box((0.010, 0.024, 0.040)),
        origin=Origin(xyz=(0.018, LOWER_ARM_LENGTH - 0.008, 0.0)),
        material=graphite,
        name="elbow_cheek_0",
    )
    lower_arm.visual(
        Box((0.010, 0.024, 0.040)),
        origin=Origin(xyz=(-0.018, LOWER_ARM_LENGTH - 0.008, 0.0)),
        material=graphite,
        name="elbow_cheek_1",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.018, 0.31, 0.014)),
        origin=Origin(xyz=(0.0, 0.175, 0.0)),
        material=satin_steel,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.028, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=satin_steel,
        name="elbow_neck",
    )
    upper_arm.visual(
        Box((0.013, 0.22, 0.007)),
        origin=Origin(xyz=(0.0, 0.165, 0.009)),
        material=satin_steel,
        name="upper_strap",
    )
    upper_arm.visual(
        Box((0.032, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.322, 0.0)),
        material=graphite,
        name="head_crosshead",
    )
    upper_arm.visual(
        Box((0.132, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.330, 0.0)),
        material=graphite,
        name="yoke_bridge",
    )
    upper_arm.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="elbow_hub",
    )
    upper_arm.visual(
        Box((0.010, 0.052, 0.080)),
        origin=Origin(xyz=(0.061, 0.356, 0.0)),
        material=graphite,
        name="yoke_arm_0",
    )
    upper_arm.visual(
        Box((0.010, 0.052, 0.080)),
        origin=Origin(xyz=(-0.061, 0.356, 0.0)),
        material=graphite,
        name="yoke_arm_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.069, YOKE_AXIS_Y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="yoke_boss_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(-0.069, YOKE_AXIS_Y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="yoke_boss_1",
    )

    shade = model.part("shade")
    shade.visual(mesh_from_geometry(_shade_mesh(), "shade"), material=matte_black, name="shade_shell")
    shade.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="trunnion_0",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(-0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="trunnion_1",
    )

    base_to_lower = model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=math.radians(-25.0),
            upper=math.radians(70.0),
        ),
    )
    lower_to_upper = model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.0, LOWER_ARM_LENGTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=math.radians(-85.0),
            upper=math.radians(95.0),
        ),
    )
    upper_to_shade = model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.0, YOKE_AXIS_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=math.radians(-65.0),
            upper=math.radians(55.0),
        ),
    )

    base_to_lower.meta["qc_samples"] = [0.0, math.radians(35.0), math.radians(60.0)]
    lower_to_upper.meta["qc_samples"] = [0.0, math.radians(-40.0), math.radians(55.0)]
    upper_to_shade.meta["qc_samples"] = [0.0, math.radians(-35.0), math.radians(35.0)]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    tilt = object_model.get_articulation("upper_arm_to_shade")

    ctx.expect_contact(
        lower_arm,
        base,
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_0",
        name="lower arm hub seats against shoulder cheek",
    )
    ctx.expect_contact(
        upper_arm,
        lower_arm,
        elem_a="elbow_hub",
        elem_b="elbow_cheek_0",
        name="upper arm hub seats inside elbow clevis",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        elem_a="trunnion_0",
        elem_b="yoke_arm_0",
        name="shade trunnion sits between yoke arms",
    )

    rest_shade = ctx.part_world_position(shade)
    with ctx.pose({shoulder: math.radians(40.0)}):
        raised_shade = ctx.part_world_position(shade)
    ctx.check(
        "shoulder hinge raises the lamp reach",
        rest_shade is not None and raised_shade is not None and raised_shade[2] > rest_shade[2] + 0.22,
        details=f"rest={rest_shade}, raised={raised_shade}",
    )

    with ctx.pose({shoulder: math.radians(30.0), elbow: 0.0}):
        elbow_rest = ctx.part_world_position(shade)
    with ctx.pose({shoulder: math.radians(30.0), elbow: math.radians(55.0)}):
        elbow_raised = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge lifts the upper span",
        elbow_rest is not None and elbow_raised is not None and elbow_raised[2] > elbow_rest[2] + 0.12,
        details=f"rest={elbow_rest}, raised={elbow_raised}",
    )

    with ctx.pose({shoulder: math.radians(25.0), elbow: math.radians(20.0), tilt: 0.0}):
        level_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shoulder: math.radians(25.0), elbow: math.radians(20.0), tilt: math.radians(35.0)}):
        tilted_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade tilt aims the spotlight upward",
        level_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][2] > level_aabb[0][2] + 0.03,
        details=f"level={level_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
