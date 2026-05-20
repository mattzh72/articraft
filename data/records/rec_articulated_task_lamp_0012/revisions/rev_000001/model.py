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

BASE_RADIUS = 0.09
BASE_HEIGHT = 0.012
PIVOT_RADIUS = 0.014
PIVOT_COLLAR_LENGTH = 0.006
HUB_LENGTH = 0.078
OUTER_PIVOT_Y = 0.042
ARM_RAIL_Y = 0.034
BASE_JOINT_HEIGHT = 0.104
LOWER_ARM_LENGTH = 0.24
UPPER_ARM_LENGTH = 0.22
SHADE_LENGTH = 0.136
SHADE_WIDTH = 0.092
SHADE_HEIGHT = 0.038


def _add_box(part, size, xyz, material, name, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_y_pivot(part, radius, length, xyz, material, name) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_table_lamp")

    steel = model.material("steel", rgba=(0.60, 0.62, 0.67, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    enamel = model.material("enamel", rgba=(0.80, 0.84, 0.88, 1.0))
    reflector_finish = model.material("reflector_finish", rgba=(0.92, 0.93, 0.95, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.092),
        origin=Origin(xyz=(-0.030, 0.0, 0.058)),
        material=steel,
        name="column",
    )
    _add_box(base, (0.014, 0.084, 0.012), (-0.026, 0.0, 0.092), steel, "back_bridge")
    _add_box(base, (0.026, 0.024, 0.022), (-0.026, 0.0, BASE_JOINT_HEIGHT), steel, "shoulder_block")
    _add_box(base, (0.020, 0.018, 0.028), (-0.024, OUTER_PIVOT_Y, BASE_JOINT_HEIGHT), steel, "right_support")
    _add_box(base, (0.020, 0.018, 0.028), (-0.024, -OUTER_PIVOT_Y, BASE_JOINT_HEIGHT), steel, "left_support")
    _add_y_pivot(base, PIVOT_RADIUS, PIVOT_COLLAR_LENGTH, (0.0, OUTER_PIVOT_Y, BASE_JOINT_HEIGHT), steel, "right_base_pivot")
    _add_y_pivot(base, PIVOT_RADIUS, PIVOT_COLLAR_LENGTH, (0.0, -OUTER_PIVOT_Y, BASE_JOINT_HEIGHT), steel, "left_base_pivot")
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    _add_y_pivot(lower_arm, 0.011, HUB_LENGTH, (0.0, 0.0, 0.0), steel, "lower_hub")
    _add_box(lower_arm, (0.032, 0.060, 0.010), (0.025, 0.0, 0.0), graphite, "lower_root_block")
    _add_box(lower_arm, (0.182, 0.010, 0.012), (0.121, ARM_RAIL_Y, 0.0), steel, "lower_right_rail")
    _add_box(lower_arm, (0.182, 0.010, 0.012), (0.121, -ARM_RAIL_Y, 0.0), steel, "lower_left_rail")
    _add_box(lower_arm, (0.010, 0.058, 0.008), (0.102, 0.0, 0.0), graphite, "lower_mid_brace")
    _add_box(lower_arm, (0.010, 0.058, 0.008), (0.164, 0.0, 0.0), graphite, "lower_front_brace")
    _add_box(lower_arm, (0.022, 0.070, 0.010), (0.218, 0.0, 0.0), graphite, "lower_tip_block")
    _add_box(lower_arm, (0.014, 0.016, 0.010), (0.220, 0.042, 0.0), graphite, "lower_right_pivot_ear")
    _add_box(lower_arm, (0.014, 0.016, 0.010), (0.220, -0.042, 0.0), graphite, "lower_left_pivot_ear")
    _add_y_pivot(lower_arm, PIVOT_RADIUS, PIVOT_COLLAR_LENGTH, (LOWER_ARM_LENGTH, OUTER_PIVOT_Y, 0.0), steel, "right_elbow_pivot")
    _add_y_pivot(lower_arm, PIVOT_RADIUS, PIVOT_COLLAR_LENGTH, (LOWER_ARM_LENGTH, -OUTER_PIVOT_Y, 0.0), steel, "left_elbow_pivot")
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.082, 0.020)),
        mass=0.48,
        origin=Origin(xyz=(LOWER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    _add_y_pivot(upper_arm, 0.011, HUB_LENGTH, (0.0, 0.0, 0.0), steel, "upper_hub")
    _add_box(upper_arm, (0.032, 0.060, 0.010), (0.025, 0.0, 0.0), graphite, "upper_root_block")
    _add_box(upper_arm, (0.170, 0.010, 0.012), (0.119, ARM_RAIL_Y, 0.0), steel, "upper_right_rail")
    _add_box(upper_arm, (0.170, 0.010, 0.012), (0.119, -ARM_RAIL_Y, 0.0), steel, "upper_left_rail")
    _add_box(upper_arm, (0.010, 0.058, 0.008), (0.096, 0.0, 0.0), graphite, "upper_mid_brace")
    _add_box(upper_arm, (0.010, 0.058, 0.008), (0.154, 0.0, 0.0), graphite, "upper_front_brace")
    _add_box(upper_arm, (0.022, 0.070, 0.010), (0.198, 0.0, 0.0), graphite, "upper_tip_block")
    _add_box(upper_arm, (0.014, 0.016, 0.010), (0.200, 0.042, 0.0), graphite, "upper_right_pivot_ear")
    _add_box(upper_arm, (0.014, 0.016, 0.010), (0.200, -0.042, 0.0), graphite, "upper_left_pivot_ear")
    _add_y_pivot(upper_arm, PIVOT_RADIUS, PIVOT_COLLAR_LENGTH, (UPPER_ARM_LENGTH, OUTER_PIVOT_Y, 0.0), steel, "right_head_pivot")
    _add_y_pivot(upper_arm, PIVOT_RADIUS, PIVOT_COLLAR_LENGTH, (UPPER_ARM_LENGTH, -OUTER_PIVOT_Y, 0.0), steel, "left_head_pivot")
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.082, 0.020)),
        mass=0.42,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    shade_yoke = model.part("shade_yoke")
    _add_y_pivot(shade_yoke, 0.011, HUB_LENGTH, (0.0, 0.0, 0.0), graphite, "yoke_hub")
    _add_box(shade_yoke, (0.040, 0.014, 0.006), (0.020, 0.0, -0.010), steel, "knuckle_stem")
    shade_yoke.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=graphite,
        name="yoke_swivel_plate",
    )
    shade_yoke.inertial = Inertial.from_geometry(
        Box((0.056, 0.082, 0.022)),
        mass=0.18,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )

    shade_head = model.part("shade_head")
    shade_head.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="shade_neck",
    )
    _add_box(shade_head, (0.020, 0.028, 0.016), (0.010, 0.0, 0.008), graphite, "shade_neck_block")
    _add_box(shade_head, (0.010, 0.084, 0.024), (0.020, 0.0, 0.012), enamel, "rear_wall")
    _add_box(shade_head, (0.104, 0.090, 0.004), (0.070, 0.0, 0.026), enamel, "shade_top")
    _add_box(shade_head, (0.104, 0.004, 0.034), (0.070, 0.044, 0.009), enamel, "right_side_wall")
    _add_box(shade_head, (0.104, 0.004, 0.034), (0.070, -0.044, 0.009), enamel, "left_side_wall")
    _add_box(shade_head, (0.008, 0.084, 0.016), (0.123, 0.0, 0.000), enamel, "front_lip")
    _add_box(shade_head, (0.038, 0.040, 0.004), (0.034, 0.0, 0.010), reflector_finish, "reflector_mount")
    _add_box(shade_head, (0.084, 0.068, 0.002), (0.068, 0.0, 0.007), reflector_finish, "reflector")
    shade_head.inertial = Inertial.from_geometry(
        Box((SHADE_LENGTH, SHADE_WIDTH, 0.036)),
        mass=0.34,
        origin=Origin(xyz=(0.068, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_HEIGHT), rpy=(0.0, -0.95, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.25, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.25, upper=0.30),
    )
    model.articulation(
        "upper_to_yoke",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade_yoke,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.80, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.40, upper=0.35),
    )
    model.articulation(
        "yoke_to_shade_head",
        ArticulationType.REVOLUTE,
        parent=shade_yoke,
        child=shade_head,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade_yoke = object_model.get_part("shade_yoke")
    shade_head = object_model.get_part("shade_head")
    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    head_tilt = object_model.get_articulation("upper_to_yoke")
    head_swivel = object_model.get_articulation("yoke_to_shade_head")

    base_plate = base.get_visual("base_plate")
    right_base_pivot = base.get_visual("right_base_pivot")
    left_base_pivot = base.get_visual("left_base_pivot")
    lower_hub = lower_arm.get_visual("lower_hub")
    right_elbow_pivot = lower_arm.get_visual("right_elbow_pivot")
    left_elbow_pivot = lower_arm.get_visual("left_elbow_pivot")
    upper_hub = upper_arm.get_visual("upper_hub")
    right_head_pivot = upper_arm.get_visual("right_head_pivot")
    left_head_pivot = upper_arm.get_visual("left_head_pivot")
    lower_right_rail = lower_arm.get_visual("lower_right_rail")
    lower_left_rail = lower_arm.get_visual("lower_left_rail")
    upper_right_rail = upper_arm.get_visual("upper_right_rail")
    upper_left_rail = upper_arm.get_visual("upper_left_rail")
    yoke_hub = shade_yoke.get_visual("yoke_hub")
    yoke_swivel_plate = shade_yoke.get_visual("yoke_swivel_plate")
    shade_neck = shade_head.get_visual("shade_neck")
    shade_neck_block = shade_head.get_visual("shade_neck_block")
    shade_top = shade_head.get_visual("shade_top")
    rear_wall = shade_head.get_visual("rear_wall")
    front_lip = shade_head.get_visual("front_lip")
    reflector = shade_head.get_visual("reflector")
    reflector_mount = shade_head.get_visual("reflector_mount")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        shade_yoke,
        shade_head,
        reason="Hidden swivel spindle passes through the head collar bore at the shade pivot.",
        elem_a=yoke_swivel_plate,
        elem_b=shade_neck,
    )
    ctx.allow_overlap(
        shade_yoke,
        shade_head,
        reason="Swivel spindle is captured inside the head collar boss around the shade pivot.",
        elem_a=yoke_swivel_plate,
        elem_b=shade_neck_block,
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=96,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_contact(base, lower_arm, elem_a=right_base_pivot, elem_b=lower_hub)
    ctx.expect_contact(base, lower_arm, elem_a=left_base_pivot, elem_b=lower_hub)
    ctx.expect_contact(lower_arm, upper_arm, elem_a=right_elbow_pivot, elem_b=upper_hub)
    ctx.expect_contact(lower_arm, upper_arm, elem_a=left_elbow_pivot, elem_b=upper_hub)
    ctx.expect_contact(upper_arm, shade_yoke, elem_a=right_head_pivot, elem_b=yoke_hub)
    ctx.expect_contact(upper_arm, shade_yoke, elem_a=left_head_pivot, elem_b=yoke_hub)
    ctx.expect_contact(shade_yoke, shade_head, elem_a=yoke_swivel_plate, elem_b=shade_neck)
    ctx.expect_contact(shade_head, shade_head, elem_a=reflector_mount, elem_b=rear_wall)
    ctx.expect_gap(
        lower_arm,
        lower_arm,
        axis="y",
        min_gap=0.050,
        positive_elem=lower_right_rail,
        negative_elem=lower_left_rail,
    )
    ctx.expect_gap(
        upper_arm,
        upper_arm,
        axis="y",
        min_gap=0.050,
        positive_elem=upper_right_rail,
        negative_elem=upper_left_rail,
    )
    ctx.expect_within(shade_head, shade_head, axes="xy", inner_elem=reflector, outer_elem=shade_top)
    ctx.expect_overlap(shade_head, shade_head, axes="xy", min_overlap=0.060, elem_a=reflector, elem_b=shade_top)
    ctx.expect_gap(
        shade_head,
        shade_head,
        axis="z",
        min_gap=0.005,
        max_gap=0.020,
        positive_elem=shade_top,
        negative_elem=reflector,
    )
    ctx.expect_gap(
        shade_head,
        shade_head,
        axis="x",
        min_gap=0.005,
        max_gap=0.018,
        positive_elem=front_lip,
        negative_elem=reflector,
    )
    ctx.expect_gap(
        shade_head,
        base,
        axis="z",
        min_gap=0.14,
    )

    front_rest_aabb = ctx.part_element_world_aabb(shade_head, elem=front_lip)
    assert front_rest_aabb is not None
    front_rest_center = (
        (front_rest_aabb[0][0] + front_rest_aabb[1][0]) * 0.5,
        (front_rest_aabb[0][1] + front_rest_aabb[1][1]) * 0.5,
        (front_rest_aabb[0][2] + front_rest_aabb[1][2]) * 0.5,
    )

    with ctx.pose({shoulder: 0.20, elbow: 0.20, head_tilt: 0.18, head_swivel: 0.30}):
        ctx.expect_gap(
            shade_head,
            base,
            axis="z",
            min_gap=0.095,
        )
        ctx.expect_contact(upper_arm, shade_yoke, elem_a=right_head_pivot, elem_b=yoke_hub)
        ctx.expect_contact(upper_arm, shade_yoke, elem_a=left_head_pivot, elem_b=yoke_hub)
        ctx.expect_contact(shade_yoke, shade_head, elem_a=yoke_swivel_plate, elem_b=shade_neck)

    with ctx.pose({shoulder: -0.20, elbow: -0.10, head_tilt: -0.35, head_swivel: -0.30}):
        ctx.expect_gap(
            shade_head,
            base,
            axis="z",
            min_gap=0.18,
        )
        ctx.expect_contact(lower_arm, upper_arm, elem_a=right_elbow_pivot, elem_b=upper_hub)
        ctx.expect_contact(lower_arm, upper_arm, elem_a=left_elbow_pivot, elem_b=upper_hub)

    folded_head_position = None
    extended_head_position = None
    shoulder_limits = shoulder.motion_limits
    elbow_limits = elbow.motion_limits
    if (
        shoulder_limits is not None
        and elbow_limits is not None
        and shoulder_limits.lower is not None
        and shoulder_limits.upper is not None
        and elbow_limits.lower is not None
        and elbow_limits.upper is not None
    ):
        with ctx.pose({shoulder: shoulder_limits.lower, elbow: elbow_limits.lower}):
            folded_head_position = ctx.part_world_position(shade_head)
        with ctx.pose({shoulder: shoulder_limits.upper, elbow: elbow_limits.upper}):
            extended_head_position = ctx.part_world_position(shade_head)
    assert folded_head_position is not None
    assert extended_head_position is not None
    ctx.check(
        "scissor_arm_changes_head_reach",
        abs(extended_head_position[0] - folded_head_position[0]) > 0.09
        and abs(extended_head_position[2] - folded_head_position[2]) > 0.07,
        details=(
            f"folded={folded_head_position}, extended={extended_head_position}"
        ),
    )

    head_tilt_limits = head_tilt.motion_limits
    if head_tilt_limits is not None and head_tilt_limits.lower is not None and head_tilt_limits.upper is not None:
        with ctx.pose({head_tilt: head_tilt_limits.lower}):
            tilt_low_aabb = ctx.part_element_world_aabb(shade_head, elem=front_lip)
        with ctx.pose({head_tilt: head_tilt_limits.upper}):
            tilt_high_aabb = ctx.part_element_world_aabb(shade_head, elem=front_lip)
        assert tilt_low_aabb is not None
        assert tilt_high_aabb is not None
        low_z = (tilt_low_aabb[0][2] + tilt_low_aabb[1][2]) * 0.5
        high_z = (tilt_high_aabb[0][2] + tilt_high_aabb[1][2]) * 0.5
        ctx.check(
            "shade_tilt_moves_beam_height",
            abs(high_z - low_z) > 0.05,
            details=f"low_z={low_z:.4f}, high_z={high_z:.4f}",
        )

    swivel_limits = head_swivel.motion_limits
    if swivel_limits is not None and swivel_limits.lower is not None and swivel_limits.upper is not None:
        with ctx.pose({head_swivel: swivel_limits.lower}):
            swivel_left_aabb = ctx.part_element_world_aabb(shade_head, elem=front_lip)
            ctx.expect_contact(shade_yoke, shade_head, elem_a=yoke_swivel_plate, elem_b=shade_neck)
        with ctx.pose({head_swivel: swivel_limits.upper}):
            swivel_right_aabb = ctx.part_element_world_aabb(shade_head, elem=front_lip)
            ctx.expect_contact(shade_yoke, shade_head, elem_a=yoke_swivel_plate, elem_b=shade_neck)
        assert swivel_left_aabb is not None
        assert swivel_right_aabb is not None
        left_y = (swivel_left_aabb[0][1] + swivel_left_aabb[1][1]) * 0.5
        right_y = (swivel_right_aabb[0][1] + swivel_right_aabb[1][1]) * 0.5
        ctx.check(
            "shade_swivel_changes_aim",
            abs(right_y - left_y) > 0.06,
            details=f"left_y={left_y:.4f}, right_y={right_y:.4f}",
        )

    moved_front_aabb = None
    with ctx.pose({head_tilt: 0.18, head_swivel: 0.30}):
        moved_front_aabb = ctx.part_element_world_aabb(shade_head, elem=front_lip)
    assert moved_front_aabb is not None
    moved_front_center = (
        (moved_front_aabb[0][0] + moved_front_aabb[1][0]) * 0.5,
        (moved_front_aabb[0][1] + moved_front_aabb[1][1]) * 0.5,
        (moved_front_aabb[0][2] + moved_front_aabb[1][2]) * 0.5,
    )
    ctx.check(
        "head_pose_changes_front_lip_pose",
        abs(moved_front_center[1] - front_rest_center[1]) > 0.02
        and abs(moved_front_center[2] - front_rest_center[2]) > 0.01,
        details=f"rest={front_rest_center}, moved={moved_front_center}",
    )

    for joint in (shoulder, elbow, head_tilt, head_swivel):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
