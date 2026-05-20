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
    model = ArticulatedObject(name="service_bay_tv_wall_bracket")

    dark_steel = model.material("powder_coated_black_steel", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_steel = model.material("worn_pin_steel", rgba=(0.42, 0.43, 0.42, 1.0))
    wall_grey = model.material("service_bay_wall", rgba=(0.48, 0.49, 0.47, 1.0))
    bolt_black = model.material("blackened_bolt_heads", rgba=(0.03, 0.03, 0.028, 1.0))

    # Object frame: +X projects out from the wall, +Z is vertical, +Y is lateral.
    # The root frame is centered on the first vertical wall-pivot axis.
    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.040, 0.420, 0.750)),
        origin=Origin(xyz=(-0.090, 0.0, 0.0)),
        material=wall_grey,
        name="wall_backer",
    )
    wall_mount.visual(
        Box((0.120, 0.145, 0.075)),
        origin=Origin(xyz=(-0.038, 0.0, 0.170)),
        material=dark_steel,
        name="upper_wall_standoff",
    )
    wall_mount.visual(
        Box((0.120, 0.145, 0.075)),
        origin=Origin(xyz=(-0.038, 0.0, -0.170)),
        material=dark_steel,
        name="lower_wall_standoff",
    )
    wall_mount.visual(
        Cylinder(radius=0.060, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=dark_steel,
        name="upper_wall_hinge_lug",
    )
    wall_mount.visual(
        Cylinder(radius=0.060, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=dark_steel,
        name="lower_wall_hinge_lug",
    )
    wall_mount.visual(
        Cylinder(radius=0.020, length=0.440),
        origin=Origin(),
        material=satin_steel,
        name="wall_hinge_pin",
    )
    for idx, (y, z) in enumerate(((-0.145, 0.285), (0.145, 0.285), (-0.145, -0.285), (0.145, -0.285))):
        wall_mount.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(-0.064, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"wall_bolt_{idx}",
        )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        Cylinder(radius=0.048, length=0.170),
        origin=Origin(),
        material=satin_steel,
        name="wall_pivot_barrel",
    )
    primary_arm.visual(
        Box((0.450, 0.056, 0.035)),
        origin=Origin(xyz=(0.265, 0.0, 0.095)),
        material=dark_steel,
        name="upper_primary_plate",
    )
    primary_arm.visual(
        Box((0.450, 0.056, 0.035)),
        origin=Origin(xyz=(0.265, 0.0, -0.095)),
        material=dark_steel,
        name="lower_primary_plate",
    )
    primary_arm.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(xyz=(0.520, 0.0, 0.155)),
        material=dark_steel,
        name="upper_elbow_lug",
    )
    primary_arm.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(xyz=(0.520, 0.0, -0.155)),
        material=dark_steel,
        name="lower_elbow_lug",
    )
    primary_arm.visual(
        Box((0.360, 0.018, 0.195)),
        origin=Origin(xyz=(0.285, 0.035, 0.0)),
        material=dark_steel,
        name="primary_side_web",
    )
    primary_arm.visual(
        Cylinder(radius=0.016, length=0.360),
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        material=satin_steel,
        name="elbow_pin",
    )

    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        Cylinder(radius=0.038, length=0.140),
        origin=Origin(),
        material=satin_steel,
        name="elbow_pivot_barrel",
    )
    secondary_arm.visual(
        Box((0.320, 0.044, 0.030)),
        origin=Origin(xyz=(0.190, 0.0, 0.080)),
        material=dark_steel,
        name="upper_secondary_plate",
    )
    secondary_arm.visual(
        Box((0.320, 0.044, 0.030)),
        origin=Origin(xyz=(0.190, 0.0, -0.080)),
        material=dark_steel,
        name="lower_secondary_plate",
    )
    secondary_arm.visual(
        Cylinder(radius=0.036, length=0.080),
        origin=Origin(xyz=(0.380, 0.0, 0.130)),
        material=dark_steel,
        name="upper_head_lug",
    )
    secondary_arm.visual(
        Cylinder(radius=0.036, length=0.080),
        origin=Origin(xyz=(0.380, 0.0, -0.130)),
        material=dark_steel,
        name="lower_head_lug",
    )
    secondary_arm.visual(
        Box((0.245, 0.015, 0.165)),
        origin=Origin(xyz=(0.200, -0.029, 0.0)),
        material=dark_steel,
        name="secondary_side_web",
    )
    secondary_arm.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        material=satin_steel,
        name="head_pin",
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Cylinder(radius=0.034, length=0.120),
        origin=Origin(),
        material=satin_steel,
        name="head_swivel_barrel",
    )
    swivel_yoke.visual(
        Box((0.110, 0.080, 0.070)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=dark_steel,
        name="short_head_stem",
    )
    swivel_yoke.visual(
        Box((0.070, 0.310, 0.045)),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_yoke_bridge",
    )
    swivel_yoke.visual(
        Box((0.060, 0.032, 0.250)),
        origin=Origin(xyz=(0.175, 0.140, 0.0)),
        material=dark_steel,
        name="tilt_cheek_0",
    )
    swivel_yoke.visual(
        Box((0.060, 0.032, 0.250)),
        origin=Origin(xyz=(0.175, -0.140, 0.0)),
        material=dark_steel,
        name="tilt_cheek_1",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.016, length=0.320),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="tilt_pin",
    )

    mounting_frame = model.part("mounting_frame")
    mounting_frame.visual(
        Cylinder(radius=0.028, length=0.220),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="tilt_tube",
    )
    mounting_frame.visual(
        Box((0.050, 0.160, 0.180)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_back_plate",
    )
    mounting_frame.visual(
        Box((0.040, 0.430, 0.040)),
        origin=Origin(xyz=(0.075, 0.0, 0.240)),
        material=dark_steel,
        name="upper_frame_rail",
    )
    mounting_frame.visual(
        Box((0.040, 0.430, 0.040)),
        origin=Origin(xyz=(0.075, 0.0, -0.240)),
        material=dark_steel,
        name="lower_frame_rail",
    )
    mounting_frame.visual(
        Box((0.040, 0.035, 0.500)),
        origin=Origin(xyz=(0.075, 0.200, 0.0)),
        material=dark_steel,
        name="frame_side_0",
    )
    mounting_frame.visual(
        Box((0.040, 0.035, 0.500)),
        origin=Origin(xyz=(0.075, -0.200, 0.0)),
        material=dark_steel,
        name="frame_side_1",
    )
    mounting_frame.visual(
        Box((0.040, 0.390, 0.045)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=dark_steel,
        name="center_frame_tie",
    )
    for idx, (y, z) in enumerate(((-0.200, 0.150), (0.200, 0.150), (-0.200, -0.150), (0.200, -0.150))):
        mounting_frame.visual(
            Cylinder(radius=0.015, length=0.016),
            origin=Origin(xyz=(0.101, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"vesa_bolt_{idx}",
        )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=primary_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=-1.90, upper=1.90),
    )
    model.articulation(
        "secondary_to_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=swivel_yoke,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "swivel_to_frame",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=mounting_frame,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=-0.35, upper=0.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    secondary_arm = object_model.get_part("secondary_arm")
    swivel_yoke = object_model.get_part("swivel_yoke")
    mounting_frame = object_model.get_part("mounting_frame")

    yaw_joints = [
        object_model.get_articulation("wall_to_primary"),
        object_model.get_articulation("primary_to_secondary"),
        object_model.get_articulation("secondary_to_swivel"),
    ]
    tilt_joint = object_model.get_articulation("swivel_to_frame")
    all_joints = yaw_joints + [tilt_joint]

    ctx.allow_overlap(
        wall_mount,
        "primary_arm",
        elem_a="wall_hinge_pin",
        elem_b="wall_pivot_barrel",
        reason="The steel wall hinge pin is intentionally captured inside the primary arm barrel.",
    )
    ctx.allow_overlap(
        "primary_arm",
        "secondary_arm",
        elem_a="elbow_pin",
        elem_b="elbow_pivot_barrel",
        reason="The elbow pin is intentionally captured inside the secondary arm pivot barrel.",
    )
    ctx.allow_overlap(
        "secondary_arm",
        swivel_yoke,
        elem_a="head_pin",
        elem_b="head_swivel_barrel",
        reason="The pan-head pin is intentionally captured inside the swivel yoke barrel.",
    )
    ctx.allow_overlap(
        swivel_yoke,
        mounting_frame,
        elem_a="tilt_pin",
        elem_b="tilt_tube",
        reason="The horizontal tilt pin is intentionally nested through the frame tilt tube.",
    )

    ctx.expect_within(
        wall_mount,
        "primary_arm",
        axes="xy",
        inner_elem="wall_hinge_pin",
        outer_elem="wall_pivot_barrel",
        margin=0.0,
        name="wall hinge pin is centered in primary barrel",
    )
    ctx.expect_overlap(
        wall_mount,
        "primary_arm",
        axes="z",
        elem_a="wall_hinge_pin",
        elem_b="wall_pivot_barrel",
        min_overlap=0.15,
        name="wall hinge pin spans primary barrel",
    )
    ctx.expect_within(
        "primary_arm",
        "secondary_arm",
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="elbow_pivot_barrel",
        margin=0.0,
        name="elbow pin is centered in secondary barrel",
    )
    ctx.expect_overlap(
        "primary_arm",
        "secondary_arm",
        axes="z",
        elem_a="elbow_pin",
        elem_b="elbow_pivot_barrel",
        min_overlap=0.12,
        name="elbow pin spans secondary barrel",
    )
    ctx.expect_within(
        "secondary_arm",
        swivel_yoke,
        axes="xy",
        inner_elem="head_pin",
        outer_elem="head_swivel_barrel",
        margin=0.0,
        name="head pin is centered in swivel barrel",
    )
    ctx.expect_overlap(
        "secondary_arm",
        swivel_yoke,
        axes="z",
        elem_a="head_pin",
        elem_b="head_swivel_barrel",
        min_overlap=0.10,
        name="head pin spans swivel barrel",
    )
    ctx.expect_within(
        swivel_yoke,
        mounting_frame,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_tube",
        margin=0.0,
        name="tilt pin is centered in frame tube",
    )
    ctx.expect_overlap(
        swivel_yoke,
        mounting_frame,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_tube",
        min_overlap=0.20,
        name="tilt pin spans frame tube",
    )

    ctx.check(
        "four articulated bracket joints are revolute",
        len(all_joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in all_joints),
    )
    ctx.check(
        "three pan joints use vertical axes",
        all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in yaw_joints),
        details=f"axes={[j.axis for j in yaw_joints]}",
    )
    ctx.check(
        "frame tilt uses horizontal lateral axis",
        tuple(tilt_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "arm joints have useful service swing",
        all(
            j.motion_limits is not None
            and j.motion_limits.lower is not None
            and j.motion_limits.upper is not None
            and (j.motion_limits.upper - j.motion_limits.lower) > 2.3
            for j in yaw_joints
        ),
    )
    ctx.check(
        "tilt joint has restrained tv pitch range",
        tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower <= -0.30
        and tilt_joint.motion_limits.upper >= 0.20,
    )

    ctx.expect_origin_gap(
        mounting_frame,
        wall_mount,
        axis="x",
        min_gap=1.0,
        name="rest pose projects tv frame out from wall",
    )
    frame_aabb = ctx.part_world_aabb(mounting_frame)
    if frame_aabb is not None:
        frame_dims = tuple(frame_aabb[1][i] - frame_aabb[0][i] for i in range(3))
        ctx.check(
            "mounting frame is wide and tall",
            frame_dims[1] > 0.42 and frame_dims[2] > 0.49,
            details=f"dims={frame_dims}",
        )

    rest_secondary = ctx.part_world_position(secondary_arm)
    with ctx.pose({"wall_to_primary": 0.65}):
        swung_secondary = ctx.part_world_position(secondary_arm)
    ctx.check(
        "primary arm swings secondary pivot sideways",
        rest_secondary is not None
        and swung_secondary is not None
        and swung_secondary[1] > rest_secondary[1] + 0.28,
        details=f"rest={rest_secondary}, swung={swung_secondary}",
    )

    rest_frame = ctx.part_world_position(mounting_frame)
    with ctx.pose({"secondary_to_swivel": 0.85}):
        swiveled_frame = ctx.part_world_position(mounting_frame)
    ctx.check(
        "head swivel pans the mounting frame",
        rest_frame is not None
        and swiveled_frame is not None
        and swiveled_frame[1] > rest_frame[1] + 0.12,
        details=f"rest={rest_frame}, swiveled={swiveled_frame}",
    )

    rest_frame_aabb = ctx.part_world_aabb(mounting_frame)
    with ctx.pose({"swivel_to_frame": 0.25}):
        tilted_frame_aabb = ctx.part_world_aabb(mounting_frame)
    ctx.check(
        "tilt joint pitches rectangular frame",
        rest_frame_aabb is not None
        and tilted_frame_aabb is not None
        and tilted_frame_aabb[1][0] > rest_frame_aabb[1][0] + 0.035,
        details=f"rest={rest_frame_aabb}, tilted={tilted_frame_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
