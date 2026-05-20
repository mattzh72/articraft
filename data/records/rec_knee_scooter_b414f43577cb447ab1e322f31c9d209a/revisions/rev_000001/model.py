from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


MESH_REVISION = "knee_scooter_v5"
MESH_SESSION = 0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, f"{MESH_REVISION}_{MESH_SESSION}_{name}")


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    radius: float,
    width: float,
    axle_length: float,
    rubber,
    alloy,
    dark_steel,
) -> None:
    spin_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    part.visual(Cylinder(radius=radius, length=width), origin=spin_origin, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=radius * 0.72, length=width * 0.72),
        origin=spin_origin,
        material=alloy,
        name="rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.26, length=axle_length),
        origin=spin_origin,
        material=dark_steel,
        name="hub",
    )


def _brake_lever_visuals(part, mesh_prefix: str, *, lever_metal) -> None:
    part.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=lever_metal,
        name="pivot_barrel",
    )
    part.visual(Box((0.020, 0.012, 0.018)), origin=Origin(xyz=(0.016, 0.0, -0.010)), material=lever_metal, name="body")
    part.visual(Box((0.076, 0.012, 0.014)), origin=Origin(xyz=(0.044, 0.0, -0.016)), material=lever_metal, name="blade")


def _axis_close(axis: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(axis, expected))


def _overall_aabb(ctx: TestContext, parts: list) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for part in parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            continue
        part_min, part_max = aabb
        for i in range(3):
            mins[i] = min(mins[i], part_min[i])
            maxs[i] = max(maxs[i], part_max[i])
    return (mins[0], mins[1], mins[2]), (maxs[0], maxs[1], maxs[2])


def build_object_model() -> ArticulatedObject:
    global MESH_SESSION
    MESH_SESSION += 1
    model = ArticulatedObject(name="compact_knee_scooter")

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    alloy = model.material("alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    pad_cover = model.material("pad_cover", rgba=(0.19, 0.21, 0.24, 1.0))
    pad_foam = model.material("pad_foam", rgba=(0.27, 0.29, 0.32, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    lever_metal = model.material("lever_metal", rgba=(0.70, 0.72, 0.75, 1.0))

    wheel_radius = 0.10
    wheel_width = 0.05
    axle_length = 0.06

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(Box((0.70, 0.38, 0.80)), mass=7.5, origin=Origin(xyz=(0.0, 0.0, 0.40)))
    frame.visual(Box((0.34, 0.06, 0.05)), origin=Origin(xyz=(-0.05, 0.0, 0.22)), material=frame_paint, name="main_beam")
    frame.visual(Box((0.10, 0.22, 0.04)), origin=Origin(xyz=(-0.21, 0.0, 0.18)), material=frame_paint, name="rear_root")
    frame.visual(Box((0.08, 0.11, 0.02)), origin=Origin(xyz=(-0.18, 0.15, 0.21)), material=frame_paint, name="rear_left_carrier")
    frame.visual(Box((0.08, 0.11, 0.02)), origin=Origin(xyz=(-0.18, -0.15, 0.21)), material=frame_paint, name="rear_right_carrier")
    frame.visual(Box((0.02, 0.01, 0.14)), origin=Origin(xyz=(-0.22, 0.095, 0.14)), material=dark_steel, name="rear_left_inner_plate")
    frame.visual(Box((0.02, 0.01, 0.14)), origin=Origin(xyz=(-0.22, 0.205, 0.14)), material=dark_steel, name="rear_left_outer_plate")
    frame.visual(Box((0.02, 0.01, 0.14)), origin=Origin(xyz=(-0.22, -0.095, 0.14)), material=dark_steel, name="rear_right_inner_plate")
    frame.visual(Box((0.02, 0.01, 0.14)), origin=Origin(xyz=(-0.22, -0.205, 0.14)), material=dark_steel, name="rear_right_outer_plate")
    frame.visual(Cylinder(radius=0.018, length=0.26), origin=Origin(xyz=(-0.05, 0.0, 0.35)), material=frame_paint, name="pad_post")
    frame.visual(Box((0.14, 0.10, 0.012)), origin=Origin(xyz=(-0.05, 0.0, 0.481)), material=alloy, name="pad_bracket")
    frame.visual(Box((0.08, 0.05, 0.05)), origin=Origin(xyz=(0.07, 0.0, 0.265)), material=frame_paint, name="head_bridge")
    frame.visual(Box((0.034, 0.05, 0.12)), origin=Origin(xyz=(0.126, 0.0, 0.305)), material=frame_paint, name="head_column")

    knee_pad = model.part("knee_pad")
    knee_pad.inertial = Inertial.from_geometry(Box((0.30, 0.16, 0.07)), mass=1.4, origin=Origin(xyz=(0.0, 0.0, 0.035)))
    knee_pad.visual(Box((0.12, 0.08, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=alloy, name="underplate")
    knee_pad.visual(Box((0.29, 0.16, 0.046)), origin=Origin(xyz=(0.0, 0.0, 0.033)), material=pad_cover, name="cushion")

    steering_fork = model.part("steering_fork")
    steering_fork.inertial = Inertial.from_geometry(Box((0.50, 0.44, 0.90)), mass=3.4, origin=Origin(xyz=(0.05, 0.0, 0.10)))
    steering_fork.visual(Cylinder(radius=0.017, length=0.48), origin=Origin(xyz=(0.0, 0.0, 0.24)), material=dark_steel, name="steer_stem")
    steering_fork.visual(Box((0.08, 0.06, 0.24)), origin=Origin(xyz=(0.04, 0.0, -0.07)), material=dark_steel, name="fork_spine")
    steering_fork.visual(Box((0.12, 0.28, 0.02)), origin=Origin(xyz=(0.10, 0.0, -0.12)), material=dark_steel, name="fork_bridge")
    steering_fork.visual(Box((0.18, 0.12, 0.02)), origin=Origin(xyz=(0.16, 0.11, -0.12)), material=dark_steel, name="front_left_carrier")
    steering_fork.visual(Box((0.18, 0.12, 0.02)), origin=Origin(xyz=(0.16, -0.11, -0.12)), material=dark_steel, name="front_right_carrier")
    steering_fork.visual(Box((0.02, 0.01, 0.18)), origin=Origin(xyz=(0.14, 0.055, -0.18)), material=dark_steel, name="front_left_inner_plate")
    steering_fork.visual(Box((0.02, 0.01, 0.18)), origin=Origin(xyz=(0.14, 0.165, -0.18)), material=dark_steel, name="front_left_outer_plate")
    steering_fork.visual(Box((0.02, 0.01, 0.18)), origin=Origin(xyz=(0.14, -0.055, -0.18)), material=dark_steel, name="front_right_inner_plate")
    steering_fork.visual(Box((0.02, 0.01, 0.18)), origin=Origin(xyz=(0.14, -0.165, -0.18)), material=dark_steel, name="front_right_outer_plate")
    steering_fork.visual(Box((0.05, 0.06, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.47)), material=dark_steel, name="handlebar_clamp")
    steering_fork.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.50), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handlebar",
    )
    steering_fork.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.0, 0.17, 0.50), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    steering_fork.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(xyz=(0.0, -0.17, 0.50), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )
    steering_fork.visual(Box((0.02, 0.016, 0.02)), origin=Origin(xyz=(0.023, 0.12, 0.50)), material=dark_steel, name="left_lever_perch")
    steering_fork.visual(Box((0.02, 0.016, 0.02)), origin=Origin(xyz=(0.023, -0.12, 0.50)), material=dark_steel, name="right_lever_perch")

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.2,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel",
        radius=wheel_radius,
        width=wheel_width,
        axle_length=axle_length,
        rubber=rubber,
        alloy=alloy,
        dark_steel=dark_steel,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.2,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel",
        radius=wheel_radius,
        width=wheel_width,
        axle_length=axle_length,
        rubber=rubber,
        alloy=alloy,
        dark_steel=dark_steel,
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.1,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_left_wheel,
        "front_left_wheel",
        radius=wheel_radius,
        width=wheel_width,
        axle_length=axle_length,
        rubber=rubber,
        alloy=alloy,
        dark_steel=dark_steel,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.1,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_right_wheel,
        "front_right_wheel",
        radius=wheel_radius,
        width=wheel_width,
        axle_length=axle_length,
        rubber=rubber,
        alloy=alloy,
        dark_steel=dark_steel,
    )

    left_brake_lever = model.part("left_brake_lever")
    left_brake_lever.inertial = Inertial.from_geometry(Box((0.12, 0.02, 0.14)), mass=0.18, origin=Origin(xyz=(-0.04, 0.0, -0.04)))
    _brake_lever_visuals(left_brake_lever, "left_brake_lever", lever_metal=lever_metal)

    right_brake_lever = model.part("right_brake_lever")
    right_brake_lever.inertial = Inertial.from_geometry(Box((0.12, 0.02, 0.14)), mass=0.18, origin=Origin(xyz=(-0.04, 0.0, -0.04)))
    _brake_lever_visuals(right_brake_lever, "right_brake_lever", lever_metal=lever_metal)

    model.articulation(
        "knee_pad_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=knee_pad,
        origin=Origin(xyz=(-0.05, 0.0, 0.487)),
    )
    model.articulation(
        "steering_head",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_fork,
        origin=Origin(xyz=(0.16, 0.0, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.22, 0.15, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.22, -0.15, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.14, 0.11, -0.23)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.14, -0.11, -0.23)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "left_brake_pivot",
        ArticulationType.REVOLUTE,
        parent=steering_fork,
        child=left_brake_lever,
        origin=Origin(xyz=(0.039, 0.12, 0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=0.0, upper=0.70),
    )
    model.articulation(
        "right_brake_pivot",
        ArticulationType.REVOLUTE,
        parent=steering_fork,
        child=right_brake_lever,
        origin=Origin(xyz=(0.039, -0.12, 0.50)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=0.0, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    knee_pad = object_model.get_part("knee_pad")
    steering_fork = object_model.get_part("steering_fork")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    left_brake_lever = object_model.get_part("left_brake_lever")
    right_brake_lever = object_model.get_part("right_brake_lever")

    steering_head = object_model.get_articulation("steering_head")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    left_brake_pivot = object_model.get_articulation("left_brake_pivot")
    right_brake_pivot = object_model.get_articulation("right_brake_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "steering_head_is_revolute",
        steering_head.articulation_type == ArticulationType.REVOLUTE,
        f"type={steering_head.articulation_type}",
    )
    ctx.check(
        "steering_head_axis_vertical",
        _axis_close(steering_head.axis, (0.0, 0.0, 1.0)),
        f"axis={steering_head.axis}",
    )
    for joint_name, joint in [
        ("rear_left_spin", rear_left_spin),
        ("rear_right_spin", rear_right_spin),
        ("front_left_spin", front_left_spin),
        ("front_right_spin", front_right_spin),
    ]:
        ctx.check(
            f"{joint_name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name}_axis_y",
            _axis_close(joint.axis, (0.0, 1.0, 0.0)),
            f"axis={joint.axis}",
        )
    for joint_name, joint in [("left_brake_pivot", left_brake_pivot), ("right_brake_pivot", right_brake_pivot)]:
        ctx.check(
            f"{joint_name}_is_revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name}_axis_x",
            _axis_close(joint.axis, (1.0, 0.0, 0.0)),
            f"axis={joint.axis}",
        )

    ctx.expect_contact(knee_pad, frame, name="knee_pad_connected_to_frame")
    ctx.expect_contact(steering_fork, frame, name="steering_fork_supported_by_head_tube")
    ctx.expect_contact(rear_left_wheel, frame, name="rear_left_wheel_supported")
    ctx.expect_contact(rear_right_wheel, frame, name="rear_right_wheel_supported")
    ctx.expect_contact(front_left_wheel, steering_fork, name="front_left_wheel_supported")
    ctx.expect_contact(front_right_wheel, steering_fork, name="front_right_wheel_supported")
    ctx.expect_contact(left_brake_lever, steering_fork, name="left_brake_lever_supported")
    ctx.expect_contact(right_brake_lever, steering_fork, name="right_brake_lever_supported")

    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="y",
        min_dist=0.28,
        max_dist=0.34,
        name="rear_track_width",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.18,
        max_dist=0.24,
        name="front_track_width",
    )
    ctx.expect_origin_distance(
        front_left_wheel,
        rear_left_wheel,
        axes="x",
        min_dist=0.48,
        max_dist=0.56,
        name="wheelbase_left_side",
    )
    ctx.expect_origin_gap(
        knee_pad,
        rear_left_wheel,
        axis="z",
        min_gap=0.32,
        max_gap=0.40,
        name="knee_pad_height_above_wheel_axle",
    )

    overall_min, overall_max = _overall_aabb(
        ctx,
        [
            frame,
            knee_pad,
            steering_fork,
            rear_left_wheel,
            rear_right_wheel,
            front_left_wheel,
            front_right_wheel,
            left_brake_lever,
            right_brake_lever,
        ],
    )
    overall_size = (
        overall_max[0] - overall_min[0],
        overall_max[1] - overall_min[1],
        overall_max[2] - overall_min[2],
    )
    ctx.check(
        "overall_length_realistic",
        0.65 <= overall_size[0] <= 0.85,
        f"length={overall_size[0]:.3f}",
    )
    ctx.check(
        "overall_width_realistic",
        0.32 <= overall_size[1] <= 0.48,
        f"width={overall_size[1]:.3f}",
    )
    ctx.check(
        "overall_height_realistic",
        0.85 <= overall_size[2] <= 1.05,
        f"height={overall_size[2]:.3f}",
    )
    knee_pad_aabb = ctx.part_world_aabb(knee_pad)
    if knee_pad_aabb is not None:
        _, knee_pad_max = knee_pad_aabb
        ctx.check(
            "knee_pad_top_height_realistic",
            0.50 <= knee_pad_max[2] <= 0.55,
            f"pad_top={knee_pad_max[2]:.3f}",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    steer_limits = steering_head.motion_limits
    if steer_limits is not None and steer_limits.lower is not None and steer_limits.upper is not None:
        with ctx.pose({steering_head: steer_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_head_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_head_lower_no_floating")
            ctx.expect_contact(steering_fork, frame, name="steering_head_lower_contact")
            ctx.expect_contact(front_left_wheel, steering_fork, name="steering_head_lower_left_wheel_contact")
            ctx.expect_contact(front_right_wheel, steering_fork, name="steering_head_lower_right_wheel_contact")
        with ctx.pose({steering_head: steer_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_head_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_head_upper_no_floating")
            ctx.expect_contact(steering_fork, frame, name="steering_head_upper_contact")
            ctx.expect_contact(front_left_wheel, steering_fork, name="steering_head_upper_left_wheel_contact")
            ctx.expect_contact(front_right_wheel, steering_fork, name="steering_head_upper_right_wheel_contact")

    for lever, joint in [(left_brake_lever, left_brake_pivot), (right_brake_lever, right_brake_pivot)]:
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.expect_contact(lever, steering_fork, name=f"{joint.name}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                ctx.expect_contact(lever, steering_fork, name=f"{joint.name}_upper_contact")

    with ctx.pose(
        {
            steering_head: 0.35,
            front_left_spin: 1.10,
            front_right_spin: -0.85,
            rear_left_spin: 2.00,
            rear_right_spin: -1.60,
            left_brake_pivot: 0.45,
            right_brake_pivot: 0.55,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        ctx.expect_contact(front_left_wheel, steering_fork, name="combined_pose_front_left_contact")
        ctx.expect_contact(front_right_wheel, steering_fork, name="combined_pose_front_right_contact")
        ctx.expect_contact(rear_left_wheel, frame, name="combined_pose_rear_left_contact")
        ctx.expect_contact(rear_right_wheel, frame, name="combined_pose_rear_right_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
