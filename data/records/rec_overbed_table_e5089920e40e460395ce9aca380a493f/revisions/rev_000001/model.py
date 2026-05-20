from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pneumatic_overbed_table")

    model.material("powder_coat", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("tray_laminate", rgba=(0.92, 0.92, 0.89, 1.0))
    model.material("steel", rgba=(0.76, 0.79, 0.82, 1.0))
    model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("blue_accent", rgba=(0.19, 0.36, 0.70, 1.0))

    base_frame = model.part("base_frame")
    rail_size = (0.74, 0.05, 0.036)
    rail_z = 0.112
    for index, y in enumerate((-0.175, 0.175)):
        base_frame.visual(
            Box(rail_size),
            origin=Origin(xyz=(0.06, y, rail_z)),
            material="powder_coat",
            name=f"rail_{index}",
        )
    base_frame.visual(
        Box((0.065, 0.40, 0.036)),
        origin=Origin(xyz=(-0.2875, 0.0, rail_z)),
        material="powder_coat",
        name="rear_crossbeam",
    )
    base_frame.visual(
        Box((0.14, 0.14, 0.028)),
        origin=Origin(xyz=(-0.205, -0.11, 0.116)),
        material="powder_coat",
        name="column_stiffener",
    )
    base_frame.visual(
        Box((0.13, 0.11, 0.030)),
        origin=Origin(xyz=(-0.205, -0.14, 0.115)),
        material="powder_coat",
        name="column_pedestal",
    )

    outer_sleeve = model.part("outer_sleeve")
    wall_height = 0.38
    outer_sleeve.visual(
        Box((0.009, 0.060, wall_height)),
        origin=Origin(xyz=(-0.043, 0.0, wall_height / 2)),
        material="powder_coat",
        name="left_wall",
    )
    outer_sleeve.visual(
        Box((0.009, 0.060, wall_height)),
        origin=Origin(xyz=(0.043, 0.0, wall_height / 2)),
        material="powder_coat",
        name="right_wall",
    )
    outer_sleeve.visual(
        Box((0.077, 0.009, wall_height)),
        origin=Origin(xyz=(0.0, 0.0255, wall_height / 2)),
        material="powder_coat",
        name="front_wall",
    )
    outer_sleeve.visual(
        Box((0.077, 0.009, wall_height)),
        origin=Origin(xyz=(0.0, -0.0255, wall_height / 2)),
        material="powder_coat",
        name="rear_wall",
    )
    outer_sleeve.visual(
        Box((0.112, 0.078, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="powder_coat",
        name="lower_boot",
    )

    model.articulation(
        "base_to_sleeve",
        ArticulationType.FIXED,
        parent=base_frame,
        child=outer_sleeve,
        origin=Origin(xyz=(-0.205, -0.14, 0.13)),
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.060, 0.034, 0.68)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material="steel",
        name="lift_post",
    )
    inner_column.visual(
        Box((0.077, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material="charcoal",
        name="entry_bushing",
    )
    inner_column.visual(
        Box((0.077, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        material="charcoal",
        name="retainer_bushing",
    )
    inner_column.visual(
        Box((0.08, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, 0.07, 0.28)),
        material="steel",
        name="head_brace",
    )
    inner_column.visual(
        Box((0.12, 0.34, 0.028)),
        origin=Origin(xyz=(0.015, 0.10, 0.374)),
        material="steel",
        name="head_arm",
    )
    inner_column.visual(
        Box((0.020, 0.045, 0.038)),
        origin=Origin(xyz=(0.102, 0.26, 0.341)),
        material="steel",
        name="paddle_hanger",
    )
    inner_column.visual(
        Box((0.030, 0.040, 0.016)),
        origin=Origin(xyz=(0.090, 0.280, 0.360)),
        material="steel",
        name="paddle_bridge",
    )
    inner_column.visual(
        Box((0.05, 0.095, 0.045)),
        origin=Origin(xyz=(0.0, 0.18, 0.3825)),
        material="steel",
        name="tray_head",
    )

    model.articulation(
        "sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.22),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.76, 0.40, 0.024)),
        origin=Origin(xyz=(0.38, 0.0, 0.012)),
        material="tray_laminate",
        name="tray_top",
    )
    tray.visual(
        Box((0.018, 0.40, 0.012)),
        origin=Origin(xyz=(0.751, 0.0, 0.03)),
        material="tray_laminate",
        name="front_rim",
    )
    tray.visual(
        Box((0.76, 0.018, 0.012)),
        origin=Origin(xyz=(0.38, -0.191, 0.03)),
        material="tray_laminate",
        name="left_rim",
    )
    tray.visual(
        Box((0.76, 0.018, 0.012)),
        origin=Origin(xyz=(0.38, 0.191, 0.03)),
        material="tray_laminate",
        name="right_rim",
    )
    tray.visual(
        Box((0.15, 0.12, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, -0.02)),
        material="powder_coat",
        name="tilt_bracket",
    )

    model.articulation(
        "column_to_tray",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=tray,
        origin=Origin(xyz=(0.0, 0.18, 0.405)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.75),
    )

    release_paddle = model.part("release_paddle")
    release_paddle.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material="charcoal",
        name="pivot_barrel",
    )
    release_paddle.visual(
        Box((0.010, 0.028, 0.018)),
        origin=Origin(xyz=(-0.003, 0.0, 0.009)),
        material="charcoal",
        name="pivot_hub",
    )
    release_paddle.visual(
        Box((0.065, 0.018, 0.010)),
        origin=Origin(xyz=(0.03, 0.0, -0.010)),
        material="blue_accent",
        name="paddle_body",
    )
    release_paddle.visual(
        Box((0.020, 0.028, 0.008)),
        origin=Origin(xyz=(0.055, 0.0, -0.018)),
        material="blue_accent",
        name="finger_pad",
    )

    model.articulation(
        "column_to_paddle",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=release_paddle,
        origin=Origin(xyz=(0.12, 0.26, 0.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=0.45),
    )

    caster_specs = (
        ("caster_fork_0", "caster_wheel_0", "caster_lock_0", (-0.30, -0.175, 0.038), -1.0),
        ("caster_fork_1", "caster_wheel_1", "caster_lock_1", (-0.30, 0.175, 0.038), 1.0),
        ("caster_fork_2", "caster_wheel_2", "caster_lock_2", (0.42, -0.175, 0.038), -1.0),
        ("caster_fork_3", "caster_wheel_3", "caster_lock_3", (0.42, 0.175, 0.038), 1.0),
    )
    for index, (fork_name, wheel_name, lock_name, fork_xyz, side_sign) in enumerate(caster_specs):
        caster_fork = model.part(fork_name)
        caster_fork.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.046)),
            material="powder_coat",
            name="swivel_stem",
        )
        caster_fork.visual(
            Box((0.032, 0.034, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.0465)),
            material="powder_coat",
            name="crown",
        )
        caster_fork.visual(
            Box((0.012, 0.010, 0.040)),
            origin=Origin(xyz=(0.0, -0.016, 0.020)),
            material="powder_coat",
            name="arm_inboard",
        )
        caster_fork.visual(
            Box((0.012, 0.010, 0.040)),
            origin=Origin(xyz=(0.0, 0.016, 0.020)),
            material="powder_coat",
            name="arm_outboard",
        )

        model.articulation(
            f"base_to_{fork_name}",
            ArticulationType.FIXED,
            parent=base_frame,
            child=caster_fork,
            origin=Origin(xyz=fork_xyz),
        )

        caster_wheel = model.part(wheel_name)
        caster_wheel.visual(
            Cylinder(radius=0.038, length=0.022),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material="charcoal",
            name="tire",
        )

        model.articulation(
            f"{fork_name}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster_fork,
            child=caster_wheel,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=15.0),
        )

        caster_lock = model.part(lock_name)
        caster_lock.visual(
            Box((0.010, 0.008, 0.016)),
            origin=Origin(xyz=(0.0, 0.004 * side_sign, 0.008)),
            material="blue_accent",
            name="tab_hub",
        )
        caster_lock.visual(
            Box((0.024, 0.020, 0.016)),
            origin=Origin(xyz=(-0.004, 0.010 * side_sign, -0.008)),
            material="blue_accent",
            name="tab_pedal",
        )

        model.articulation(
            f"{fork_name}_to_lock",
            ArticulationType.REVOLUTE,
            parent=caster_fork,
            child=caster_lock,
            origin=Origin(xyz=(0.0, 0.017 * side_sign, 0.036)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.35, upper=0.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_column = object_model.get_part("inner_column")
    tray = object_model.get_part("tray")
    release_paddle = object_model.get_part("release_paddle")
    lift_joint = object_model.get_articulation("sleeve_to_column")
    tray_joint = object_model.get_articulation("column_to_tray")
    paddle_joint = object_model.get_articulation("column_to_paddle")
    wheel_joints = [
        object_model.get_articulation("caster_fork_0_to_wheel"),
        object_model.get_articulation("caster_fork_1_to_wheel"),
        object_model.get_articulation("caster_fork_2_to_wheel"),
        object_model.get_articulation("caster_fork_3_to_wheel"),
    ]
    lock_joints = [
        object_model.get_articulation("caster_fork_0_to_lock"),
        object_model.get_articulation("caster_fork_1_to_lock"),
        object_model.get_articulation("caster_fork_2_to_lock"),
        object_model.get_articulation("caster_fork_3_to_lock"),
    ]

    ctx.expect_within(
        inner_column,
        outer_sleeve,
        axes="xy",
        inner_elem="lift_post",
        margin=0.018,
        name="lift post stays centered inside the sleeve at rest",
    )
    ctx.expect_overlap(
        inner_column,
        outer_sleeve,
        axes="z",
        elem_a="lift_post",
        min_overlap=0.10,
        name="lift post stays inserted in the sleeve at rest",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({lift_joint: lift_joint.motion_limits.upper}):
        ctx.expect_within(
            inner_column,
            outer_sleeve,
            axes="xy",
            inner_elem="lift_post",
            margin=0.018,
            name="lift post stays centered inside the sleeve when raised",
        )
        ctx.expect_overlap(
            inner_column,
            outer_sleeve,
            axes="z",
            elem_a="lift_post",
            min_overlap=0.08,
            name="lift post retains insertion at max height",
        )
        raised_tray_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray rises with the pneumatic column",
        rest_tray_pos is not None
        and raised_tray_pos is not None
        and raised_tray_pos[2] > rest_tray_pos[2] + 0.18,
        details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
    )

    with ctx.pose({tray_joint: 0.0}):
        ctx.expect_gap(
            tray,
            release_paddle,
            axis="z",
            min_gap=0.025,
            max_gap=0.08,
            name="release paddle hangs just beneath the tray",
        )
        ctx.expect_overlap(
            tray,
            release_paddle,
            axes="xy",
            min_overlap=0.015,
            name="release paddle sits under the tray's right side",
        )
        rest_tray_aabb = ctx.part_world_aabb(tray)

    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        tilted_tray_aabb = ctx.part_world_aabb(tray)

    ctx.check(
        "tray tilts upward from the column head",
        rest_tray_aabb is not None
        and tilted_tray_aabb is not None
        and tilted_tray_aabb[1][2] > rest_tray_aabb[1][2] + 0.18,
        details=f"rest={rest_tray_aabb}, tilted={tilted_tray_aabb}",
    )
    ctx.check(
        "paddle joint remains a compact revolute lever",
        paddle_joint.articulation_type == ArticulationType.REVOLUTE
        and paddle_joint.motion_limits is not None
        and paddle_joint.motion_limits.upper is not None
        and paddle_joint.motion_limits.upper <= 0.5,
        details=f"type={paddle_joint.articulation_type}, limits={paddle_joint.motion_limits}",
    )
    ctx.check(
        "caster wheels spin on continuous axle joints",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in wheel_joints
        ),
        details=str([(joint.name, joint.articulation_type, joint.motion_limits) for joint in wheel_joints]),
    )
    ctx.check(
        "caster lock tabs use compact revolute pivots",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper - joint.motion_limits.lower <= 0.7
            for joint in lock_joints
        ),
        details=str([(joint.name, joint.articulation_type, joint.motion_limits) for joint in lock_joints]),
    )

    return ctx.report()


object_model = build_object_model()
