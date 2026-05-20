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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.15, 0.16, 0.17, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.54, 0.56, 0.60, 1.0))
    flywheel_red = model.material("flywheel_red", rgba=(0.74, 0.13, 0.12, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))

    floor_frame = model.part("floor_frame")
    floor_frame.inertial = Inertial.from_geometry(
        Box((1.10, 0.58, 0.88)),
        mass=32.0,
        origin=Origin(xyz=(0.02, 0.0, 0.44)),
    )
    floor_frame.visual(
        Box((0.08, 0.56, 0.04)),
        origin=Origin(xyz=(-0.36, 0.0, 0.02)),
        material=steel_dark,
        name="rear_stabilizer",
    )
    floor_frame.visual(
        Box((0.08, 0.56, 0.04)),
        origin=Origin(xyz=(0.42, 0.0, 0.02)),
        material=steel_dark,
        name="front_stabilizer",
    )
    floor_frame.visual(
        Box((0.74, 0.04, 0.07)),
        origin=Origin(xyz=(0.03, 0.185, 0.055)),
        material=frame_black,
        name="left_floor_rail",
    )
    floor_frame.visual(
        Box((0.74, 0.04, 0.07)),
        origin=Origin(xyz=(0.03, -0.185, 0.055)),
        material=frame_black,
        name="right_floor_rail",
    )
    floor_frame.visual(
        Box((0.20, 0.09, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, 0.105)),
        material=frame_black,
        name="bb_platform",
    )
    floor_frame.visual(
        Box((0.12, 0.10, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.125)),
        material=frame_black,
        name="bb_pedestal",
    )
    floor_frame.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.02, 0.0, 0.19), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="bottom_bracket_shell",
    )
    floor_frame.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.18)),
        material=frame_black,
        name="upper_module_receiver",
    )
    floor_frame.visual(
        Box((0.08, 0.18, 0.085)),
        origin=Origin(xyz=(0.20, 0.0, 0.0425)),
        material=frame_black,
        name="fork_module_receiver",
    )

    floor_left_brace = tube_from_spline_points(
        [(-0.18, 0.173, 0.09), (-0.10, 0.168, 0.13), (-0.06, 0.145, 0.17), (-0.04, 0.09, 0.17)],
        radius=0.024,
        samples_per_segment=12,
        radial_segments=18,
    )
    floor_right_brace = tube_from_spline_points(
        [(-0.18, -0.173, 0.09), (-0.10, -0.168, 0.13), (-0.06, -0.145, 0.17), (-0.04, -0.09, 0.17)],
        radius=0.024,
        samples_per_segment=12,
        radial_segments=18,
    )
    floor_front_left = tube_from_spline_points(
        [(0.05, 0.175, 0.082), (0.11, 0.162, 0.074), (0.17, 0.125, 0.055)],
        radius=0.016,
        samples_per_segment=10,
        radial_segments=18,
    )
    floor_front_right = tube_from_spline_points(
        [(0.05, -0.175, 0.082), (0.11, -0.162, 0.074), (0.17, -0.125, 0.055)],
        radius=0.016,
        samples_per_segment=10,
        radial_segments=18,
    )
    rear_center_spine = tube_from_spline_points(
        [(-0.36, 0.0, 0.04), (-0.24, 0.0, 0.075), (-0.12, 0.0, 0.125), (-0.05, 0.0, 0.17)],
        radius=0.028,
        samples_per_segment=14,
        radial_segments=18,
    )
    front_center_spine = tube_from_spline_points(
        [(0.42, 0.0, 0.04), (0.32, 0.0, 0.045), (0.22, 0.0, 0.052), (0.10, 0.0, 0.075), (0.02, 0.0, 0.105)],
        radius=0.025,
        samples_per_segment=12,
        radial_segments=18,
    )
    floor_frame.visual(
        mesh_from_geometry(floor_left_brace, "floor_left_brace"),
        material=frame_black,
        name="floor_left_brace",
    )
    floor_frame.visual(
        mesh_from_geometry(floor_right_brace, "floor_right_brace"),
        material=frame_black,
        name="floor_right_brace",
    )
    floor_frame.visual(
        mesh_from_geometry(floor_front_left, "floor_front_left"),
        material=frame_black,
        name="floor_front_left",
    )
    floor_frame.visual(
        mesh_from_geometry(floor_front_right, "floor_front_right"),
        material=frame_black,
        name="floor_front_right",
    )
    floor_frame.visual(
        mesh_from_geometry(rear_center_spine, "rear_center_spine"),
        material=frame_black,
        name="rear_center_spine",
    )
    floor_frame.visual(
        mesh_from_geometry(front_center_spine, "front_center_spine"),
        material=frame_black,
        name="front_center_spine",
    )

    fork_module = model.part("fork_module")
    fork_module.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.54)),
        mass=10.0,
        origin=Origin(xyz=(0.18, 0.0, 0.28)),
    )
    left_fork_leg = tube_from_spline_points(
        [(0.00, 0.08, 0.00), (0.07, 0.08, 0.16), (0.13, 0.08, 0.31), (0.18, 0.07, 0.40)],
        radius=0.028,
        samples_per_segment=12,
        radial_segments=18,
    )
    right_fork_leg = tube_from_spline_points(
        [(0.00, -0.08, 0.00), (0.07, -0.08, 0.16), (0.13, -0.08, 0.31), (0.18, -0.07, 0.40)],
        radius=0.028,
        samples_per_segment=12,
        radial_segments=18,
    )
    fork_module.visual(mesh_from_geometry(left_fork_leg, "left_fork_leg"), material=frame_black, name="left_fork_leg")
    fork_module.visual(mesh_from_geometry(right_fork_leg, "right_fork_leg"), material=frame_black, name="right_fork_leg")
    fork_module.visual(
        Box((0.05, 0.14, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.00)),
        material=steel_dark,
        name="fork_base_block",
    )
    fork_module.visual(
        Cylinder(radius=0.03, length=0.18),
        origin=Origin(xyz=(0.18, 0.0, 0.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="fork_axle_bridge",
    )
    fork_module.visual(
        Box((0.08, 0.16, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, 0.25)),
        material=frame_black,
        name="fork_mid_plate",
    )

    axle_module = model.part("axle_module")
    axle_module.inertial = Inertial.from_geometry(
        Box((0.56, 0.16, 0.60)),
        mass=14.0,
        origin=Origin(xyz=(0.15, -0.03, 0.27)),
    )
    housing_profile = [
        (-0.06, -0.02),
        (-0.03, -0.16),
        (0.07, -0.23),
        (0.22, -0.25),
        (0.40, -0.18),
        (0.48, -0.05),
        (0.50, 0.03),
        (0.45, 0.14),
        (0.28, 0.22),
        (0.10, 0.24),
        (-0.01, 0.19),
        (-0.06, 0.08),
    ]
    housing_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(housing_profile, 0.016).rotate_y(-pi / 2.0).rotate_z(pi / 2.0),
        "drive_housing",
    )
    axle_module.visual(
        housing_mesh,
        origin=Origin(xyz=(0.15, -0.120, 0.26)),
        material=frame_black,
        name="drive_housing",
    )
    axle_module.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.34, -0.119028, 0.39), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="flywheel_bearing_boss",
    )
    axle_module.visual(
        Box((0.12, 0.06, 0.08)),
        origin=Origin(xyz=(0.28, -0.145, 0.35)),
        material=steel_dark,
        name="flywheel_mount_block",
    )
    axle_module.visual(
        Box((0.08, 0.04, 0.06)),
        origin=Origin(xyz=(0.10, -0.110, 0.27)),
        material=steel_dark,
        name="bottom_mount_block",
    )
    axle_module.visual(
        Box((0.09, 0.11, 0.06)),
        origin=Origin(xyz=(0.095, -0.073, 0.35)),
        material=steel_dark,
        name="fork_side_mount_bracket",
    )
    axle_module.visual(
        Cylinder(radius=0.018, length=0.061),
        origin=Origin(xyz=(0.34, -0.1495, 0.39), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="flywheel_axle_stub",
    )

    flywheel = model.part("flywheel")
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=0.07),
        mass=11.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    flywheel.visual(
        Cylinder(radius=0.235, length=0.07),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="flywheel_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.078, length=0.05),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="flywheel_hub",
    )
    flywheel.visual(
        Box((0.22, 0.022, 0.035)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=flywheel_red,
        name="flywheel_spoke",
    )
    flywheel.visual(
        Cylinder(radius=0.020, length=0.10),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="flywheel_handle",
    )

    upper_body_module = model.part("upper_body_module")
    upper_body_module.inertial = Inertial.from_geometry(
        Box((0.78, 0.48, 1.02)),
        mass=18.0,
        origin=Origin(xyz=(0.08, 0.0, 0.52)),
    )
    upper_main_beam = tube_from_spline_points(
        [(0.00, 0.0, 0.00), (-0.06, 0.0, 0.18), (-0.11, 0.0, 0.37), (-0.15, 0.0, 0.56)],
        radius=0.034,
        samples_per_segment=14,
        radial_segments=18,
    )
    upper_front_beam = tube_from_spline_points(
        [(0.03, 0.0, 0.02), (0.10, 0.0, 0.20), (0.19, 0.0, 0.42), (0.26, 0.0, 0.63)],
        radius=0.034,
        samples_per_segment=14,
        radial_segments=18,
    )
    upper_body_module.visual(mesh_from_geometry(upper_main_beam, "upper_main_beam"), material=frame_black, name="upper_main_beam")
    upper_body_module.visual(mesh_from_geometry(upper_front_beam, "upper_front_beam"), material=frame_black, name="upper_front_beam")
    upper_body_module.visual(
        Cylinder(radius=0.041, length=0.22),
        origin=Origin(xyz=(-0.15, 0.0, 0.57), rpy=(0.0, -0.21, 0.0)),
        material=steel_dark,
        name="seat_column_outer",
    )
    upper_body_module.visual(
        Cylinder(radius=0.040, length=0.32),
        origin=Origin(xyz=(0.26, 0.0, 0.77)),
        material=steel_dark,
        name="handlebar_mast",
    )
    upper_body_module.visual(
        Cylinder(radius=0.020, length=0.50),
        origin=Origin(xyz=(0.26, 0.0, 0.97), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="handlebar",
    )
    upper_body_module.visual(
        Box((0.10, 0.12, 0.08)),
        origin=Origin(xyz=(0.00, 0.0, 0.02)),
        material=steel_dark,
        name="upper_module_base",
    )
    upper_body_module.visual(
        Box((0.09, 0.10, 0.06)),
        origin=Origin(xyz=(0.26, 0.0, 0.94)),
        material=steel_dark,
        name="bar_clamp_block",
    )

    crank_set = model.part("crank_set")
    crank_set.inertial = Inertial.from_geometry(
        Box((0.42, 0.28, 0.18)),
        mass=4.0,
        origin=Origin(),
    )
    crank_set.visual(
        Cylinder(radius=0.017, length=0.23),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="crank_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.10, length=0.012),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="chainring",
    )
    crank_set.visual(
        Box((0.04, 0.018, 0.17)),
        origin=Origin(xyz=(0.0, -0.112, -0.095)),
        material=steel_dark,
        name="right_crank_arm",
    )
    crank_set.visual(
        Box((0.04, 0.018, 0.17)),
        origin=Origin(xyz=(0.0, 0.112, 0.095)),
        material=steel_dark,
        name="left_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, -0.112, -0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.112, 0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Box((0.10, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, -0.147, -0.18)),
        material=grip_black,
        name="right_pedal",
    )
    crank_set.visual(
        Box((0.10, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, 0.147, 0.18)),
        material=grip_black,
        name="left_pedal",
    )

    seat_post = model.part("seat_post")
    seat_post.inertial = Inertial.from_geometry(
        Box((0.34, 0.20, 0.48)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
    )
    seat_post.visual(
        Cylinder(radius=0.032, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=steel_mid,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.09, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=steel_dark,
        name="seat_clamp",
    )
    seat_post.visual(
        Box((0.26, 0.17, 0.05)),
        origin=Origin(xyz=(-0.020, 0.0, 0.395), rpy=(0.0, 0.21, 0.0)),
        material=saddle_black,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.13, 0.10, 0.04)),
        origin=Origin(xyz=(0.135, 0.0, 0.392), rpy=(0.0, 0.21, 0.0)),
        material=saddle_black,
        name="saddle_nose",
    )

    model.articulation(
        "frame_to_fork",
        ArticulationType.FIXED,
        parent=floor_frame,
        child=fork_module,
        origin=Origin(xyz=(0.18, 0.0, 0.105)),
    )
    model.articulation(
        "frame_to_axle_module",
        ArticulationType.FIXED,
        parent=floor_frame,
        child=axle_module,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_upper_body",
        ArticulationType.FIXED,
        parent=floor_frame,
        child=upper_body_module,
        origin=Origin(xyz=(-0.10, 0.0, 0.24)),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=floor_frame,
        child=crank_set,
        origin=Origin(xyz=(0.02, 0.0, 0.19)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=12.0),
    )
    model.articulation(
        "axle_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=axle_module,
        child=flywheel,
        origin=Origin(xyz=(0.34, -0.215, 0.39)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "upper_body_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=upper_body_module,
        child=seat_post,
        origin=Origin(xyz=(-0.127, 0.0, 0.462), rpy=(0.0, -0.21, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.20, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    floor_frame = object_model.get_part("floor_frame")
    crank_set = object_model.get_part("crank_set")
    upper_body = object_model.get_part("upper_body_module")
    seat_post = object_model.get_part("seat_post")
    flywheel = object_model.get_part("flywheel")

    crank_joint = object_model.get_articulation("frame_to_crank")
    flywheel_joint = object_model.get_articulation("axle_to_flywheel")
    seat_joint = object_model.get_articulation("upper_body_to_seat_post")

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.allow_overlap(
        floor_frame,
        crank_set,
        elem_a="bottom_bracket_shell",
        elem_b="crank_spindle",
        reason="The crank spindle intentionally passes through the bottom-bracket shell proxy.",
    )
    ctx.allow_overlap(
        upper_body,
        seat_post,
        elem_a="seat_column_outer",
        elem_b="inner_post",
        reason="The height-adjustable seat post is intentionally represented as sliding inside the outer seat tube proxy.",
    )
    ctx.allow_overlap(
        upper_body,
        seat_post,
        elem_a="upper_main_beam",
        elem_b="inner_post",
        reason="The welded receiver beam is simplified as a continuous solid tube around the sliding seat post entry.",
    )

    ctx.expect_overlap(
        seat_post,
        upper_body,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_column_outer",
        min_overlap=0.18,
        name="seat post remains inserted in the seat tube at rest",
    )

    right_pedal_rest = ctx.part_element_world_aabb(crank_set, elem="right_pedal")
    flywheel_spoke_rest = ctx.part_element_world_aabb(flywheel, elem="flywheel_spoke")
    seat_rest = ctx.part_world_position(seat_post)
    seat_upper = seat_joint.motion_limits.upper if seat_joint.motion_limits is not None else None

    with ctx.pose({seat_joint: seat_upper if seat_upper is not None else 0.16}):
        ctx.expect_overlap(
            seat_post,
            upper_body,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_column_outer",
            min_overlap=0.06,
            name="seat post retains insertion at maximum height",
        )
        seat_extended = ctx.part_world_position(seat_post)

    with ctx.pose({crank_joint: pi / 2.0}):
        right_pedal_rotated = ctx.part_element_world_aabb(crank_set, elem="right_pedal")

    with ctx.pose({flywheel_joint: pi / 2.0}):
        flywheel_spoke_rotated = ctx.part_element_world_aabb(flywheel, elem="flywheel_spoke")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    pedal_rest_center = aabb_center(right_pedal_rest)
    pedal_rot_center = aabb_center(right_pedal_rotated)
    flywheel_rest_center = aabb_center(flywheel_spoke_rest)
    flywheel_rot_center = aabb_center(flywheel_spoke_rotated)

    ctx.check(
        "seat post raises upward when extended",
        seat_rest is not None
        and seat_extended is not None
        and seat_extended[2] > seat_rest[2] + 0.12,
        details=f"rest={seat_rest}, extended={seat_extended}",
    )
    ctx.check(
        "crank positive rotation advances the right pedal",
        pedal_rest_center is not None
        and pedal_rot_center is not None
        and pedal_rot_center[0] > pedal_rest_center[0] + 0.12,
        details=f"rest={pedal_rest_center}, rotated={pedal_rot_center}",
    )
    ctx.check(
        "flywheel positive rotation moves the visible spoke",
        flywheel_rest_center is not None
        and flywheel_rot_center is not None
        and abs(flywheel_rot_center[2] - flywheel_rest_center[2]) > 0.08,
        details=f"rest={flywheel_rest_center}, rotated={flywheel_rot_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
