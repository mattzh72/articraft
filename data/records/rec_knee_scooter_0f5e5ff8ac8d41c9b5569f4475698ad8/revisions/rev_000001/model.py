from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_knee_scooter")

    frame_mat = model.material("satin_black_frame", rgba=(0.02, 0.022, 0.024, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    pad_mat = model.material("soft_knee_pad", rgba=(0.03, 0.035, 0.04, 1.0))
    tire_mat = model.material("matte_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    rim_mat = model.material("silver_rim", rgba=(0.78, 0.78, 0.74, 1.0))
    blue_mat = model.material("blue_clamp_hardware", rgba=(0.06, 0.18, 0.55, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.074,
            0.044,
            rim=WheelRim(inner_radius=0.048, flange_height=0.005, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.025,
                width=0.044,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0035, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        "scooter_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.100,
            0.052,
            inner_radius=0.074,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.005, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "scooter_tire",
    )

    def add_wheel_visuals(part, *, side: float) -> None:
        # Mirror the dish so the fuller hub face points inward toward the axle
        # bracket on both sides of the scooter.
        wheel_origin = Origin(rpy=(0.0, 0.0, -pi / 2.0 if side > 0.0 else pi / 2.0))
        part.visual(tire_mesh, origin=wheel_origin, material=tire_mat, name="tire")
        part.visual(wheel_mesh, origin=wheel_origin, material=rim_mat, name="rim")

    center_frame = model.part("center_frame")
    center_frame.visual(
        Cylinder(radius=0.026, length=0.980),
        origin=Origin(xyz=(-0.090, 0.0, 0.320), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_mat,
        name="main_tube",
    )
    center_frame.visual(
        Cylinder(radius=0.018, length=0.544),
        origin=Origin(xyz=(-0.550, 0.0, 0.120), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="rear_axle",
    )
    center_frame.visual(
        Cylinder(radius=0.018, length=0.215),
        origin=Origin(xyz=(-0.550, 0.0, 0.220)),
        material=frame_mat,
        name="rear_drop",
    )
    center_frame.visual(
        Box((0.125, 0.090, 0.110)),
        origin=Origin(xyz=(-0.170, 0.0, 0.375)),
        material=frame_mat,
        name="pad_pedestal",
    )
    center_frame.visual(
        Box((0.520, 0.240, 0.035)),
        origin=Origin(xyz=(-0.170, 0.0, 0.425)),
        material=frame_mat,
        name="pad_plate",
    )
    center_frame.visual(
        Box((0.500, 0.255, 0.070)),
        origin=Origin(xyz=(-0.170, 0.0, 0.4775)),
        material=pad_mat,
        name="knee_pad",
    )
    center_frame.visual(
        Box((0.150, 0.035, 0.075)),
        origin=Origin(xyz=(0.430, 0.055, 0.325)),
        material=frame_mat,
        name="head_lug_0",
    )
    center_frame.visual(
        Box((0.150, 0.035, 0.075)),
        origin=Origin(xyz=(0.430, -0.055, 0.325)),
        material=frame_mat,
        name="head_lug_1",
    )
    center_frame.visual(
        Box((0.065, 0.112, 0.046)),
        origin=Origin(xyz=(0.372, 0.0, 0.325)),
        material=frame_mat,
        name="head_bridge",
    )
    center_frame.visual(
        Cylinder(radius=0.045, length=0.480),
        origin=Origin(xyz=(0.480, 0.0, 0.550)),
        material=frame_mat,
        name="head_tube",
    )
    center_frame.visual(
        Cylinder(radius=0.058, length=0.024),
        origin=Origin(xyz=(0.480, 0.0, 0.318)),
        material=steel_mat,
        name="lower_bearing",
    )
    center_frame.visual(
        Cylinder(radius=0.058, length=0.024),
        origin=Origin(xyz=(0.480, 0.0, 0.782)),
        material=steel_mat,
        name="upper_bearing",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.024, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel_mat,
        name="steerer",
    )
    front_fork.visual(
        Box((0.135, 0.560, 0.060)),
        origin=Origin(xyz=(0.025, 0.0, -0.300)),
        material=frame_mat,
        name="fork_crown",
    )
    front_fork.visual(
        Box((0.046, 0.045, 0.260)),
        origin=Origin(xyz=(0.055, 0.250, -0.365)),
        material=frame_mat,
        name="fork_leg_0",
    )
    front_fork.visual(
        Box((0.046, 0.045, 0.260)),
        origin=Origin(xyz=(0.055, -0.250, -0.365)),
        material=frame_mat,
        name="fork_leg_1",
    )
    front_fork.visual(
        Box((0.055, 0.544, 0.036)),
        origin=Origin(xyz=(0.055, 0.0, -0.430)),
        material=steel_mat,
        name="front_axle",
    )
    front_fork.visual(
        Box((0.110, 0.125, 0.080)),
        origin=Origin(xyz=(0.000, 0.0, 0.300)),
        material=blue_mat,
        name="clamp_block",
    )
    front_fork.visual(
        Box((0.082, 0.018, 0.080)),
        origin=Origin(xyz=(0.076, 0.059, 0.340)),
        material=blue_mat,
        name="hinge_ear_0",
    )
    front_fork.visual(
        Box((0.082, 0.018, 0.080)),
        origin=Origin(xyz=(0.076, -0.059, 0.340)),
        material=blue_mat,
        name="hinge_ear_1",
    )
    upper_stem = model.part("upper_stem")
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="hinge_barrel",
    )
    upper_stem.visual(
        Cylinder(radius=0.022, length=0.530),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=frame_mat,
        name="main_stem",
    )
    upper_stem.visual(
        Box((0.060, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=blue_mat,
        name="bar_clamp",
    )
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.555), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="handlebar",
    )
    upper_stem.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=(0.0, 0.290, 0.555), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pad_mat,
        name="grip_0",
    )
    upper_stem.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=(0.0, -0.290, 0.555), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pad_mat,
        name="grip_1",
    )

    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_1 = model.part("rear_wheel_1")
    front_wheel_0 = model.part("front_wheel_0")
    front_wheel_1 = model.part("front_wheel_1")
    add_wheel_visuals(rear_wheel_0, side=1.0)
    add_wheel_visuals(rear_wheel_1, side=-1.0)
    add_wheel_visuals(front_wheel_0, side=1.0)
    add_wheel_visuals(front_wheel_1, side=-1.0)

    model.articulation(
        "frame_to_fork",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=front_fork,
        origin=Origin(xyz=(0.480, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "fork_to_stem",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=upper_stem,
        origin=Origin(xyz=(0.076, 0.0, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "rear_wheel_joint_0",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.550, 0.306, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "rear_wheel_joint_1",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=rear_wheel_1,
        origin=Origin(xyz=(-0.550, -0.306, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_joint_0",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel_0,
        origin=Origin(xyz=(0.055, 0.306, -0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_joint_1",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel_1,
        origin=Origin(xyz=(0.055, -0.306, -0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_frame = object_model.get_part("center_frame")
    front_fork = object_model.get_part("front_fork")
    upper_stem = object_model.get_part("upper_stem")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    fork_joint = object_model.get_articulation("frame_to_fork")
    stem_joint = object_model.get_articulation("fork_to_stem")

    ctx.allow_overlap(
        center_frame,
        front_fork,
        elem_a="head_tube",
        elem_b="steerer",
        reason="The steering steerer is intentionally nested inside the head-tube bearing sleeve.",
    )
    for bearing_name in ("lower_bearing", "upper_bearing"):
        ctx.allow_overlap(
            center_frame,
            front_fork,
            elem_a=bearing_name,
            elem_b="steerer",
            reason="The steerer intentionally passes through the bearing race at the end of the head tube.",
        )
    ctx.expect_within(
        front_fork,
        center_frame,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube",
        margin=0.002,
        name="steerer is centered in head tube",
    )
    ctx.expect_overlap(
        front_fork,
        center_frame,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube",
        min_overlap=0.35,
        name="steerer remains retained through head tube",
    )
    for bearing_name in ("lower_bearing", "upper_bearing"):
        ctx.expect_within(
            front_fork,
            center_frame,
            axes="xy",
            inner_elem="steerer",
            outer_elem=bearing_name,
            margin=0.002,
            name=f"steerer is centered in {bearing_name}",
        )
        ctx.expect_overlap(
            front_fork,
            center_frame,
            axes="z",
            elem_a="steerer",
            elem_b=bearing_name,
            min_overlap=0.018,
            name=f"steerer passes through {bearing_name}",
        )

    wheel_joint_names = (
        "rear_wheel_joint_0",
        "rear_wheel_joint_1",
        "front_wheel_joint_0",
        "front_wheel_joint_1",
    )
    ctx.check(
        "all four wheels have spin joints",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in wheel_joint_names
        ),
        details="Each wheel should be a continuous axle joint.",
    )
    ctx.check(
        "front fork yaws about steering axis",
        fork_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(fork_joint.axis) == (0.0, 0.0, 1.0)
        and fork_joint.motion_limits is not None
        and fork_joint.motion_limits.lower < 0.0
        and fork_joint.motion_limits.upper > 0.0,
        details=f"type={fork_joint.articulation_type}, axis={fork_joint.axis}, limits={fork_joint.motion_limits}",
    )
    ctx.check(
        "upper stem folds on clamp hinge",
        stem_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(stem_joint.axis) == (0.0, 1.0, 0.0)
        and stem_joint.motion_limits is not None
        and stem_joint.motion_limits.lower == 0.0
        and stem_joint.motion_limits.upper > 1.2,
        details=f"type={stem_joint.articulation_type}, axis={stem_joint.axis}, limits={stem_joint.motion_limits}",
    )

    ctx.expect_gap(
        front_fork,
        upper_stem,
        axis="y",
        positive_elem="hinge_ear_0",
        negative_elem="hinge_barrel",
        max_gap=0.010,
        max_penetration=1e-6,
        name="positive hinge ear clips barrel side",
    )
    ctx.expect_gap(
        upper_stem,
        front_fork,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="hinge_ear_1",
        max_gap=0.010,
        max_penetration=1e-6,
        name="negative hinge ear clips barrel side",
    )
    ctx.expect_overlap(
        upper_stem,
        front_fork,
        axes="xz",
        elem_a="hinge_barrel",
        elem_b="hinge_ear_0",
        min_overlap=0.025,
        name="hinge barrel sits between clamp ears",
    )

    rest_front_pos = ctx.part_world_position(front_wheel_0)
    with ctx.pose({fork_joint: 0.55}):
        steered_front_pos = ctx.part_world_position(front_wheel_0)
    ctx.check(
        "steering yaw carries front wheels",
        rest_front_pos is not None
        and steered_front_pos is not None
        and abs(steered_front_pos[0] - rest_front_pos[0]) > 0.05,
        details=f"rest={rest_front_pos}, steered={steered_front_pos}",
    )

    def aabb_center_xz(part, elem: str) -> tuple[float, float] | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        return ((box[0][0] + box[1][0]) / 2.0, (box[0][2] + box[1][2]) / 2.0)

    rest_hinge_pos = ctx.part_world_position(upper_stem)
    rest_handle = aabb_center_xz(upper_stem, "handlebar")
    with ctx.pose({stem_joint: stem_joint.motion_limits.upper}):
        folded_hinge_pos = ctx.part_world_position(upper_stem)
        folded_handle = aabb_center_xz(upper_stem, "handlebar")
        ctx.expect_gap(
            front_fork,
            upper_stem,
            axis="y",
            positive_elem="hinge_ear_0",
            negative_elem="hinge_barrel",
            max_gap=0.010,
            max_penetration=1e-6,
            name="folded stem remains captured by hinge ear",
        )
    ctx.check(
        "upper stem folds forward while hinge stays attached",
        rest_hinge_pos is not None
        and folded_hinge_pos is not None
        and rest_handle is not None
        and folded_handle is not None
        and abs(folded_hinge_pos[0] - rest_hinge_pos[0]) < 0.001
        and abs(folded_hinge_pos[2] - rest_hinge_pos[2]) < 0.001
        and folded_handle[0] > rest_handle[0] + 0.30,
        details=(
            f"hinge rest={rest_hinge_pos}, folded={folded_hinge_pos}; "
            f"handle rest={rest_handle}, folded={folded_handle}"
        ),
    )

    rear_positions = (ctx.part_world_position(rear_wheel_0), ctx.part_world_position(rear_wheel_1))
    front_positions = (ctx.part_world_position(front_wheel_0), ctx.part_world_position(front_wheel_1))
    ctx.check(
        "rear wheels are fixed under rear frame",
        all(pos is not None and pos[0] < -0.45 and 0.08 < pos[2] < 0.16 for pos in rear_positions),
        details=f"rear wheel positions={rear_positions}",
    )
    ctx.check(
        "front wheels are carried by fork",
        all(pos is not None and pos[0] > 0.45 and 0.08 < pos[2] < 0.16 for pos in front_positions),
        details=f"front wheel positions={front_positions}",
    )

    return ctx.report()


object_model = build_object_model()
