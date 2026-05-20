from __future__ import annotations

import math

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
    model = ArticulatedObject(name="bare_side_wall_monitor_arm")

    plate_mat = model.material("powder_coated_wall_plate", rgba=(0.23, 0.24, 0.25, 1.0))
    arm_mat = model.material("satin_anodized_aluminum", rgba=(0.50, 0.52, 0.52, 1.0))
    dark_mat = model.material("black_plastic_end_caps", rgba=(0.015, 0.016, 0.018, 1.0))
    glass_mat = model.material("dark_glass_screen", rgba=(0.02, 0.025, 0.035, 1.0))
    bolt_mat = model.material("dark_socket_bolts", rgba=(0.04, 0.04, 0.04, 1.0))

    # Object frame: +X projects out from the wall, +Z is up, +Y is lateral.
    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.035, 0.180, 0.340)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0)),
        material=plate_mat,
        name="wall_plate",
    )
    side_plate.visual(
        Cylinder(radius=0.030, length=0.135),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=plate_mat,
        name="wall_bearing",
    )
    for index, z in enumerate((-0.058, 0.058)):
        side_plate.visual(
            Box((0.050, 0.100, 0.030)),
            origin=Origin(xyz=(0.015, 0.0, z)),
            material=plate_mat,
            name=f"bearing_land_{index}",
        )
    for index, (y, z) in enumerate(
        ((-0.055, -0.115), (0.055, -0.115), (-0.055, 0.115), (0.055, 0.115))
    ):
        side_plate.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(xyz=(0.004, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_mat,
            name=f"mount_bolt_{index}",
        )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=0.036, length=0.046),
        origin=Origin(),
        material=arm_mat,
        name="root_bearing",
    )
    primary_link.visual(
        Box((0.412, 0.048, 0.035)),
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        material=arm_mat,
        name="link_bar",
    )
    primary_link.visual(
        Cylinder(radius=0.034, length=0.046),
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        material=arm_mat,
        name="end_bearing",
    )
    primary_link.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.480, 0.0, 0.027)),
        material=dark_mat,
        name="elbow_cap",
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=0.036, length=0.044),
        origin=Origin(),
        material=arm_mat,
        name="root_bearing",
    )
    secondary_link.visual(
        Box((0.314, 0.044, 0.032)),
        origin=Origin(xyz=(0.191, 0.0, 0.0)),
        material=arm_mat,
        name="link_bar",
    )
    secondary_link.visual(
        Cylinder(radius=0.032, length=0.044),
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        material=arm_mat,
        name="end_bearing",
    )
    secondary_link.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.380, 0.0, 0.027)),
        material=dark_mat,
        name="wrist_cap",
    )

    tilt_yoke = model.part("tilt_yoke")
    tilt_yoke.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(),
        material=arm_mat,
        name="swivel_bearing",
    )
    tilt_yoke.visual(
        Box((0.047, 0.040, 0.030)),
        origin=Origin(xyz=(0.0545, 0.0, 0.0)),
        material=arm_mat,
        name="short_neck",
    )
    tilt_yoke.visual(
        Box((0.028, 0.390, 0.030)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=arm_mat,
        name="crossbar",
    )
    for index, y in enumerate((-0.186, 0.186)):
        tilt_yoke.visual(
            Box((0.034, 0.026, 0.120)),
            origin=Origin(xyz=(0.095, y, 0.0)),
            material=arm_mat,
            name=f"yoke_ear_{index}",
        )
    tilt_yoke.visual(
        Cylinder(radius=0.012, length=0.404),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_mat,
        name="tilt_pin",
    )

    monitor_head = model.part("monitor_head")
    monitor_head.visual(
        Cylinder(radius=0.018, length=0.360),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_mat,
        name="tilt_boss",
    )
    monitor_head.visual(
        Box((0.045, 0.330, 0.215)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=dark_mat,
        name="housing",
    )
    monitor_head.visual(
        Box((0.004, 0.300, 0.185)),
        origin=Origin(xyz=(0.0595, 0.0, 0.0)),
        material=glass_mat,
        name="screen_face",
    )
    monitor_head.visual(
        Box((0.014, 0.105, 0.060)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=arm_mat,
        name="rear_mount_pad",
    )

    model.articulation(
        "plate_to_primary",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=primary_link,
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "secondary_to_yoke",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=tilt_yoke,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=tilt_yoke,
        child=monitor_head,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    side_plate = object_model.get_part("side_plate")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    tilt_yoke = object_model.get_part("tilt_yoke")
    monitor_head = object_model.get_part("monitor_head")

    plate_joint = object_model.get_articulation("plate_to_primary")
    elbow_joint = object_model.get_articulation("primary_to_secondary")
    swivel_joint = object_model.get_articulation("secondary_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_head")

    ctx.allow_overlap(
        side_plate,
        primary_link,
        elem_a="wall_bearing",
        elem_b="root_bearing",
        reason="The wall pivot is a coaxial captured bearing stack.",
    )
    ctx.allow_overlap(
        primary_link,
        secondary_link,
        elem_a="end_bearing",
        elem_b="root_bearing",
        reason="The elbow joint is represented as nested bearing races around the same vertical pin.",
    )
    ctx.allow_overlap(
        secondary_link,
        tilt_yoke,
        elem_a="end_bearing",
        elem_b="swivel_bearing",
        reason="The head swivel is a compact nested bearing around the wrist pin.",
    )
    ctx.allow_overlap(
        tilt_yoke,
        monitor_head,
        elem_a="tilt_pin",
        elem_b="tilt_boss",
        reason="The tilt pin intentionally passes through the monitor trunnion boss.",
    )
    for ear in ("yoke_ear_0", "yoke_ear_1"):
        ctx.allow_overlap(
            tilt_yoke,
            monitor_head,
            elem_a=ear,
            elem_b="tilt_boss",
            reason="The side yoke ear captures the head trunnion boss at the tilt axis.",
        )

    ctx.expect_overlap(
        side_plate,
        primary_link,
        axes="xyz",
        elem_a="wall_bearing",
        elem_b="root_bearing",
        min_overlap=0.020,
        name="wall bearing captures primary pivot",
    )
    ctx.expect_overlap(
        primary_link,
        secondary_link,
        axes="xyz",
        elem_a="end_bearing",
        elem_b="root_bearing",
        min_overlap=0.020,
        name="elbow bearings share a vertical pin",
    )
    ctx.expect_overlap(
        secondary_link,
        tilt_yoke,
        axes="xyz",
        elem_a="end_bearing",
        elem_b="swivel_bearing",
        min_overlap=0.018,
        name="wrist swivel is retained in the secondary link",
    )
    ctx.expect_overlap(
        tilt_yoke,
        monitor_head,
        axes="xyz",
        elem_a="tilt_pin",
        elem_b="tilt_boss",
        min_overlap=0.010,
        name="tilt pin passes through monitor boss",
    )

    rest_pos = ctx.part_world_position(monitor_head)
    with ctx.pose({plate_joint: 0.45, elbow_joint: -0.70, swivel_joint: 0.35, tilt_joint: 0.40}):
        moved_pos = ctx.part_world_position(monitor_head)
    ctx.check(
        "articulated arm moves monitor head",
        rest_pos is not None
        and moved_pos is not None
        and abs(moved_pos[1] - rest_pos[1]) > 0.10,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
