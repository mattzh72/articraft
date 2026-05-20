from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder descriptor and rotation for a cylinder whose long axis is +X."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder descriptor and rotation for a cylinder whose long axis is +Y."""
    return Cylinder(radius=radius, length=length), Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def _at(
    xyz: tuple[float, float, float],
    base: Origin | None = None,
) -> Origin:
    if base is None:
        return Origin(xyz=xyz)
    return Origin(xyz=xyz, rpy=base.rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fully_articulated_robotic_arm")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    blue = model.material("blue_servo_covers", rgba=(0.05, 0.23, 0.82, 1.0))
    steel = model.material("polished_steel", rgba=(0.86, 0.87, 0.84, 1.0))
    rubber = model.material("black_rubber_grip", rgba=(0.015, 0.015, 0.014, 1.0))
    amber = model.material("amber_status_lens", rgba=(1.0, 0.55, 0.08, 1.0))

    cyl_x_small, cyl_x_origin = _cyl_x(0.001, 0.001)
    cyl_y_small, cyl_y_origin = _cyl_y(0.001, 0.001)
    del cyl_x_small, cyl_y_small

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.24, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.16, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="bearing_plinth",
    )
    base.visual(
        Box((0.30, 0.08, 0.028)),
        origin=Origin(xyz=(0.0, -0.145, 0.020)),
        material=dark_metal,
        name="cable_tray",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.145, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="rotating_disk",
    )
    turret.visual(
        Cylinder(radius=0.075, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_metal,
        name="servo_column",
    )
    turret.visual(
        Box((0.050, 0.180, 0.140)),
        origin=Origin(xyz=(-0.080, 0.0, 0.220)),
        material=dark_metal,
        name="shoulder_web",
    )
    turret.visual(
        Box((0.110, 0.025, 0.140)),
        origin=Origin(xyz=(-0.005, 0.075, 0.220)),
        material=blue,
        name="shoulder_plate_0",
    )
    turret.visual(
        Box((0.110, 0.025, 0.140)),
        origin=Origin(xyz=(-0.005, -0.075, 0.220)),
        material=blue,
        name="shoulder_plate_1",
    )
    turret.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=_at((-0.083, 0.0, 0.305), cyl_y_origin),
        material=amber,
        name="status_beacon",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.050, length=0.129),
        origin=_at((0.0, 0.0, 0.0), cyl_y_origin),
        material=steel,
        name="proximal_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.036, length=0.390),
        origin=_at((0.206, 0.0, 0.0), cyl_x_origin),
        material=aluminum,
        name="main_tube",
    )
    upper_arm.visual(
        Box((0.330, 0.050, 0.040)),
        origin=Origin(xyz=(0.230, 0.0, 0.042)),
        material=blue,
        name="top_servo_cover",
    )
    upper_arm.visual(
        Box((0.050, 0.155, 0.105)),
        origin=Origin(xyz=(0.3835, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_web",
    )
    upper_arm.visual(
        Box((0.105, 0.020, 0.125)),
        origin=Origin(xyz=(0.455, 0.064, 0.0)),
        material=blue,
        name="elbow_plate_0",
    )
    upper_arm.visual(
        Box((0.105, 0.020, 0.125)),
        origin=Origin(xyz=(0.455, -0.064, 0.0)),
        material=blue,
        name="elbow_plate_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.043, length=0.112),
        origin=_at((0.0, 0.0, 0.0), cyl_y_origin),
        material=steel,
        name="proximal_hub",
    )
    forearm.visual(
        Cylinder(radius=0.030, length=0.315),
        origin=_at((0.173, 0.0, 0.0), cyl_x_origin),
        material=aluminum,
        name="main_tube",
    )
    forearm.visual(
        Box((0.260, 0.040, 0.034)),
        origin=Origin(xyz=(0.195, 0.0, -0.037)),
        material=blue,
        name="lower_servo_cover",
    )
    forearm.visual(
        Box((0.060, 0.116, 0.090)),
        origin=Origin(xyz=(0.311, 0.0, 0.0)),
        material=dark_metal,
        name="wrist_web",
    )
    forearm.visual(
        Box((0.088, 0.018, 0.100)),
        origin=Origin(xyz=(0.376, 0.052, 0.0)),
        material=blue,
        name="wrist_plate_0",
    )
    forearm.visual(
        Box((0.088, 0.018, 0.100)),
        origin=Origin(xyz=(0.376, -0.052, 0.0)),
        material=blue,
        name="wrist_plate_1",
    )

    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(
        Cylinder(radius=0.034, length=0.090),
        origin=_at((0.0, 0.0, 0.0), cyl_y_origin),
        material=steel,
        name="pitch_hub",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.020, length=0.095),
        origin=_at((0.050, 0.0, -0.026), cyl_x_origin),
        material=aluminum,
        name="short_link",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.043, length=0.040),
        origin=Origin(xyz=(0.120, 0.0, -0.020)),
        material=dark_metal,
        name="yaw_lower_flange",
    )

    wrist_yaw = model.part("wrist_yaw")
    wrist_yaw.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="yaw_upper_flange",
    )
    wrist_yaw.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=_at((0.045, 0.0, 0.028), cyl_x_origin),
        material=aluminum,
        name="roll_bearing_arm",
    )
    wrist_yaw.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=_at((0.070, 0.0, 0.020), cyl_x_origin),
        material=dark_metal,
        name="roll_mount",
    )

    wrist_roll = model.part("wrist_roll")
    wrist_roll.visual(
        Cylinder(radius=0.033, length=0.035),
        origin=_at((0.0175, 0.0, 0.0), cyl_x_origin),
        material=steel,
        name="roll_flange",
    )
    wrist_roll.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=_at((0.043, 0.0, 0.0), cyl_x_origin),
        material=steel,
        name="flange_neck",
    )
    wrist_roll.visual(
        Box((0.080, 0.110, 0.070)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=dark_metal,
        name="palm_body",
    )
    for y, suffix in ((0.040, "0"), (-0.040, "1")):
        wrist_roll.visual(
            Box((0.035, 0.035, 0.012)),
            origin=Origin(xyz=(0.140, y, 0.023)),
            material=steel,
            name=f"finger_lug_top_{suffix}",
        )
        wrist_roll.visual(
            Box((0.035, 0.035, 0.012)),
            origin=Origin(xyz=(0.140, y, -0.023)),
            material=steel,
            name=f"finger_lug_bottom_{suffix}",
        )

    finger_0 = model.part("finger_0")
    finger_0.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="knuckle",
    )
    finger_0.visual(
        Box((0.118, 0.022, 0.022)),
        origin=Origin(xyz=(0.073, 0.0, 0.0)),
        material=dark_metal,
        name="finger_beam",
    )
    finger_0.visual(
        Box((0.050, 0.026, 0.026)),
        origin=Origin(xyz=(0.140, -0.003, 0.0)),
        material=rubber,
        name="rubber_pad",
    )

    finger_1 = model.part("finger_1")
    finger_1.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="knuckle",
    )
    finger_1.visual(
        Box((0.118, 0.022, 0.022)),
        origin=Origin(xyz=(0.073, 0.0, 0.0)),
        material=dark_metal,
        name="finger_beam",
    )
    finger_1.visual(
        Box((0.050, 0.026, 0.026)),
        origin=Origin(xyz=(0.140, 0.003, 0.0)),
        material=rubber,
        name="rubber_pad",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.4),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-1.35, upper=1.45),
        motion_properties=MotionProperties(damping=0.10, friction=0.04),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-0.25, upper=2.35),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    model.articulation(
        "wrist_pitch_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch,
        origin=Origin(xyz=(0.376, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.2, lower=-1.7, upper=1.7),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )
    model.articulation(
        "wrist_yaw_joint",
        ArticulationType.REVOLUTE,
        parent=wrist_pitch,
        child=wrist_yaw,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.6, lower=-1.9, upper=1.9),
        motion_properties=MotionProperties(damping=0.04, friction=0.015),
    )
    model.articulation(
        "wrist_roll_joint",
        ArticulationType.CONTINUOUS,
        parent=wrist_yaw,
        child=wrist_roll,
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.2),
        motion_properties=MotionProperties(damping=0.03, friction=0.01),
    )
    model.articulation(
        "finger_0_joint",
        ArticulationType.REVOLUTE,
        parent=wrist_roll,
        child=finger_0,
        origin=Origin(xyz=(0.140, 0.040, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=0.55),
        motion_properties=MotionProperties(damping=0.03, friction=0.02),
    )
    model.articulation(
        "finger_1_joint",
        ArticulationType.REVOLUTE,
        parent=wrist_roll,
        child=finger_1,
        origin=Origin(xyz=(0.140, -0.040, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.55, upper=0.0),
        motion_properties=MotionProperties(damping=0.03, friction=0.02),
        mimic=Mimic(joint="finger_0_joint", multiplier=-1.0, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    expected_joints = (
        "base_yaw",
        "shoulder_pitch",
        "elbow_pitch",
        "wrist_pitch_joint",
        "wrist_yaw_joint",
        "wrist_roll_joint",
        "finger_0_joint",
        "finger_1_joint",
    )
    ctx.check(
        "robotic arm has full joint stack",
        all(object_model.get_articulation(name) is not None for name in expected_joints),
        details=f"expected joints={expected_joints}",
    )

    for name in (
        "shoulder_pitch",
        "elbow_pitch",
        "wrist_pitch_joint",
        "wrist_yaw_joint",
        "finger_0_joint",
        "finger_1_joint",
    ):
        joint = object_model.get_articulation(name)
        limits = joint.motion_limits if joint is not None else None
        ctx.check(
            f"{name} has finite limits",
            limits is not None and limits.lower is not None and limits.upper is not None,
            details=f"limits={limits}",
        )

    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_roll = object_model.get_part("wrist_roll")
    finger_0 = object_model.get_part("finger_0")
    finger_1 = object_model.get_part("finger_1")
    shoulder = object_model.get_articulation("shoulder_pitch")
    finger_joint = object_model.get_articulation("finger_0_joint")

    ctx.expect_contact(
        base,
        turret,
        elem_a="bearing_plinth",
        elem_b="rotating_disk",
        contact_tol=0.001,
        name="turret bearing sits on base",
    )
    ctx.expect_within(
        upper_arm,
        turret,
        axes="y",
        inner_elem="proximal_hub",
        outer_elem="shoulder_web",
        margin=0.001,
        name="shoulder hub captured between yoke plates",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="y",
        inner_elem="proximal_hub",
        outer_elem="elbow_web",
        margin=0.001,
        name="elbow hub captured between yoke plates",
    )

    rest_aabb = ctx.part_world_aabb(wrist_roll)
    with ctx.pose({shoulder: 0.70}):
        lifted_aabb = ctx.part_world_aabb(wrist_roll)
    if rest_aabb is not None and lifted_aabb is not None:
        rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        lifted_z = (lifted_aabb[0][2] + lifted_aabb[1][2]) * 0.5
    else:
        rest_z = lifted_z = None
    ctx.check(
        "shoulder pitch raises wrist",
        rest_z is not None and lifted_z is not None and lifted_z > rest_z + 0.18,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    closed_0 = ctx.part_world_aabb(finger_0)
    closed_1 = ctx.part_world_aabb(finger_1)
    with ctx.pose({finger_joint: 0.50}):
        open_0 = ctx.part_world_aabb(finger_0)
        open_1 = ctx.part_world_aabb(finger_1)

    def _outer_span(aabb_a, aabb_b) -> float | None:
        if aabb_a is None or aabb_b is None:
            return None
        return max(aabb_a[1][1], aabb_b[1][1]) - min(aabb_a[0][1], aabb_b[0][1])

    closed_span = _outer_span(closed_0, closed_1)
    open_span = _outer_span(open_0, open_1)
    ctx.check(
        "mimicked gripper opens symmetrically",
        closed_span is not None and open_span is not None and open_span > closed_span + 0.05,
        details=f"closed_span={closed_span}, open_span={open_span}",
    )

    return ctx.report()


object_model = build_object_model()
