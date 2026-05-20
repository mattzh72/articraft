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


def _add_arm_branch(
    model: ArticulatedObject,
    *,
    prefix: str,
    side_sign: float,
    ceiling_plate,
    arm_metal,
    joint_metal,
    head_white,
    diffuser,
    grip_dark,
) -> None:
    upper_arm = model.part(f"{prefix}_upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_metal,
        name="shoulder_barrel",
    )
    upper_arm.visual(
        Box((0.250, 0.040, 0.024)),
        origin=Origin(xyz=(side_sign * 0.145, 0.0, 0.0)),
        material=arm_metal,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.160, 0.024, 0.010)),
        origin=Origin(xyz=(side_sign * 0.145, 0.0, -0.015)),
        material=joint_metal,
        name="upper_cable_cover",
    )
    upper_arm.visual(
        Box((0.030, 0.056, 0.018)),
        origin=Origin(xyz=(side_sign * 0.250, 0.0, 0.021)),
        material=joint_metal,
        name="elbow_housing",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.320, 0.065, 0.055)),
        mass=3.6,
        origin=Origin(xyz=(side_sign * 0.145, 0.0, 0.0)),
    )

    forearm = model.part(f"{prefix}_forearm")
    forearm.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_metal,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.280, 0.036, 0.022)),
        origin=Origin(xyz=(side_sign * 0.150, 0.0, 0.0)),
        material=arm_metal,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.180, 0.022, 0.010)),
        origin=Origin(xyz=(side_sign * 0.145, 0.0, -0.014)),
        material=joint_metal,
        name="forearm_cable_cover",
    )
    forearm.visual(
        Box((0.040, 0.220, 0.040)),
        origin=Origin(xyz=(side_sign * 0.300, 0.0, 0.0)),
        material=arm_metal,
        name="yoke_block",
    )
    for suffix, y_pos in (("inboard", -0.111), ("outboard", 0.111)):
        forearm.visual(
            Box((0.070, 0.014, 0.070)),
            origin=Origin(xyz=(side_sign * 0.335, y_pos, 0.0)),
            material=arm_metal,
            name=f"yoke_arm_{suffix}",
        )
    forearm.inertial = Inertial.from_geometry(
        Box((0.380, 0.235, 0.080)),
        mass=3.0,
        origin=Origin(xyz=(side_sign * 0.170, 0.0, 0.0)),
    )

    head = model.part(f"{prefix}_lamp_head")
    head.visual(
        Box((0.050, 0.080, 0.040)),
        origin=Origin(xyz=(side_sign * 0.018, 0.0, -0.005)),
        material=arm_metal,
        name="head_neck",
    )
    for name, y_pos in (("left_trunnion", -0.096), ("right_trunnion", 0.096)):
        head.visual(
            Box((0.024, 0.016, 0.048)),
            origin=Origin(xyz=(side_sign * 0.032, y_pos, -0.004)),
            material=joint_metal,
            name=name,
        )
    head.visual(
        Cylinder(radius=0.096, length=0.048),
        origin=Origin(xyz=(side_sign * 0.058, 0.0, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_white,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.083, length=0.004),
        origin=Origin(xyz=(side_sign * 0.082, 0.0, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser,
        name="diffuser_lens",
    )
    head.visual(
        Cylinder(radius=0.068, length=0.028),
        origin=Origin(xyz=(side_sign * 0.030, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_metal,
        name="rear_housing",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(side_sign * 0.078, 0.0, -0.103), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_dark,
        name="handle_bar",
    )
    for name, y_pos in (("left_post", -0.040), ("right_post", 0.040)):
        head.visual(
            Cylinder(radius=0.005, length=0.060),
            origin=Origin(xyz=(side_sign * 0.078, y_pos, -0.073)),
            material=grip_dark,
            name=name,
        )
    head.inertial = Inertial.from_geometry(
        Box((0.220, 0.210, 0.160)),
        mass=2.4,
        origin=Origin(xyz=(side_sign * 0.060, 0.0, -0.025)),
    )

    shoulder_axis = (0.0, 1.0, 0.0) if side_sign < 0.0 else (0.0, -1.0, 0.0)
    elbow_axis = shoulder_axis
    tilt_axis = shoulder_axis

    model.articulation(
        f"ceiling_to_{prefix}_shoulder",
        ArticulationType.REVOLUTE,
        parent=ceiling_plate,
        child=upper_arm,
        origin=Origin(xyz=(side_sign * 0.150, 0.0, -0.132)),
        axis=shoulder_axis,
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=math.radians(-55.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        f"{prefix}_shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(side_sign * 0.290, 0.0, 0.0)),
        axis=elbow_axis,
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=math.radians(-10.0),
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        f"{prefix}_forearm_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=head,
        origin=Origin(xyz=(side_sign * 0.335, 0.0, 0.0)),
        axis=tilt_axis,
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=math.radians(-75.0),
            upper=math.radians(65.0),
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_arm_surgical_exam_lamp")

    ceiling_white = model.material("ceiling_white", rgba=(0.95, 0.95, 0.96, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.89, 0.90, 0.91, 1.0))
    joint_metal = model.material("joint_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    diffuser = model.material("diffuser", rgba=(0.96, 0.97, 0.98, 0.92))
    grip_dark = model.material("grip_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Box((0.560, 0.210, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=ceiling_white,
        name="mount_plate",
    )
    ceiling_plate.visual(
        Box((0.300, 0.130, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=ceiling_white,
        name="service_canopy",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.024, length=0.070),
        origin=Origin(xyz=(-0.150, 0.0, -0.064)),
        material=joint_metal,
        name="left_drop_post",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.024, length=0.070),
        origin=Origin(xyz=(0.150, 0.0, -0.064)),
        material=joint_metal,
        name="right_drop_post",
    )
    for prefix, x_pos in (("left", -0.150), ("right", 0.150)):
        ceiling_plate.visual(
            Box((0.050, 0.068, 0.032)),
            origin=Origin(xyz=(x_pos, 0.0, -0.092)),
            material=joint_metal,
            name=f"{prefix}_shoulder_block",
        )
        for side_name, y_pos in (("inboard", -0.026), ("outboard", 0.026)):
            ceiling_plate.visual(
                Cylinder(radius=0.027, length=0.012),
                origin=Origin(
                    xyz=(x_pos, y_pos, -0.132),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=joint_metal,
                name=f"{prefix}_clevis_{side_name}",
            )
    ceiling_plate.inertial = Inertial.from_geometry(
        Box((0.560, 0.210, 0.155)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    _add_arm_branch(
        model,
        prefix="left",
        side_sign=-1.0,
        ceiling_plate=ceiling_plate,
        arm_metal=arm_metal,
        joint_metal=joint_metal,
        head_white=ceiling_white,
        diffuser=diffuser,
        grip_dark=grip_dark,
    )
    _add_arm_branch(
        model,
        prefix="right",
        side_sign=1.0,
        ceiling_plate=ceiling_plate,
        arm_metal=arm_metal,
        joint_metal=joint_metal,
        head_white=ceiling_white,
        diffuser=diffuser,
        grip_dark=grip_dark,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling_plate = object_model.get_part("ceiling_plate")
    left_head = object_model.get_part("left_lamp_head")
    right_head = object_model.get_part("right_lamp_head")
    left_upper = object_model.get_part("left_upper_arm")
    right_upper = object_model.get_part("right_upper_arm")
    left_forearm = object_model.get_part("left_forearm")
    right_forearm = object_model.get_part("right_forearm")

    left_shoulder = object_model.get_articulation("ceiling_to_left_shoulder")
    right_shoulder = object_model.get_articulation("ceiling_to_right_shoulder")
    left_elbow = object_model.get_articulation("left_shoulder_to_elbow")
    right_elbow = object_model.get_articulation("right_shoulder_to_elbow")
    left_tilt = object_model.get_articulation("left_forearm_to_head_tilt")
    right_tilt = object_model.get_articulation("right_forearm_to_head_tilt")

    ctx.expect_origin_gap(
        ceiling_plate,
        left_head,
        axis="z",
        min_gap=0.10,
        name="left head hangs below ceiling plate",
    )
    ctx.expect_origin_gap(
        ceiling_plate,
        right_head,
        axis="z",
        min_gap=0.10,
        name="right head hangs below ceiling plate",
    )
    ctx.expect_origin_distance(
        left_head,
        right_head,
        axes="x",
        min_dist=1.10,
        name="lamp heads are widely separated across the plate",
    )
    ctx.expect_origin_distance(
        left_upper,
        right_upper,
        axes="x",
        min_dist=0.25,
        name="independent shoulder joints sit apart on the ceiling plate",
    )

    left_rest = ctx.part_world_position(left_head)
    right_rest = ctx.part_world_position(right_head)

    with ctx.pose({left_shoulder: math.radians(35.0)}):
        left_raised = ctx.part_world_position(left_head)
        right_static = ctx.part_world_position(right_head)
    ctx.check(
        "left shoulder raises only the left branch",
        left_rest is not None
        and right_rest is not None
        and left_raised is not None
        and right_static is not None
        and left_raised[2] > left_rest[2] + 0.12
        and abs(right_static[0] - right_rest[0]) < 1e-6
        and abs(right_static[2] - right_rest[2]) < 1e-6,
        details=f"left_rest={left_rest}, left_raised={left_raised}, right_rest={right_rest}, right_static={right_static}",
    )

    with ctx.pose({right_shoulder: math.radians(35.0)}):
        right_raised = ctx.part_world_position(right_head)
    ctx.check(
        "right shoulder raises the right branch",
        right_rest is not None and right_raised is not None and right_raised[2] > right_rest[2] + 0.12,
        details=f"right_rest={right_rest}, right_raised={right_raised}",
    )

    with ctx.pose({left_elbow: math.radians(60.0)}):
        left_elbow_raised = ctx.part_world_position(left_head)
    ctx.check(
        "left elbow bends the forearm upward",
        left_rest is not None
        and left_elbow_raised is not None
        and left_elbow_raised[2] > left_rest[2] + 0.10,
        details=f"left_rest={left_rest}, left_elbow_raised={left_elbow_raised}",
    )

    with ctx.pose({right_elbow: math.radians(60.0)}):
        right_elbow_raised = ctx.part_world_position(right_head)
    ctx.check(
        "right elbow bends the forearm upward",
        right_rest is not None
        and right_elbow_raised is not None
        and right_elbow_raised[2] > right_rest[2] + 0.10,
        details=f"right_rest={right_rest}, right_elbow_raised={right_elbow_raised}",
    )

    left_head_rest_aabb = ctx.part_world_aabb(left_head)
    right_head_rest_aabb = ctx.part_world_aabb(right_head)
    with ctx.pose({left_tilt: math.radians(40.0), right_tilt: math.radians(40.0)}):
        left_head_tilted_aabb = ctx.part_world_aabb(left_head)
        right_head_tilted_aabb = ctx.part_world_aabb(right_head)
    ctx.check(
        "head tilt changes the lamp head envelope on both branches",
        left_head_rest_aabb is not None
        and right_head_rest_aabb is not None
        and left_head_tilted_aabb is not None
        and right_head_tilted_aabb is not None
        and left_head_tilted_aabb[1][2] > left_head_rest_aabb[1][2] + 0.02
        and right_head_tilted_aabb[1][2] > right_head_rest_aabb[1][2] + 0.02,
        details=(
            f"left_rest_aabb={left_head_rest_aabb}, left_tilted_aabb={left_head_tilted_aabb}, "
            f"right_rest_aabb={right_head_rest_aabb}, right_tilted_aabb={right_head_tilted_aabb}"
        ),
    )

    ctx.expect_origin_distance(
        left_forearm,
        right_forearm,
        axes="x",
        min_dist=0.70,
        name="forearms remain on separate branches",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
