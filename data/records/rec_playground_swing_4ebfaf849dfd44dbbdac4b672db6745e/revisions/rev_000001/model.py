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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toddler_bucket_swing_set")

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    frame_green = model.material("frame_green", rgba=(0.19, 0.43, 0.28, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    hanger_yellow = model.material("hanger_yellow", rgba=(0.93, 0.79, 0.16, 1.0))
    bucket_black = model.material("bucket_black", rgba=(0.12, 0.13, 0.14, 1.0))
    bar_orange = model.material("bar_orange", rgba=(0.92, 0.43, 0.14, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.70, 1.70, 2.25)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 1.125)),
    )

    leg_radius = 0.042
    left_side_frame = wire_from_points(
        [
            (-0.74, -0.78, leg_radius),
            (-0.74, 0.0, 2.12),
            (-0.74, 0.78, leg_radius),
        ],
        radius=leg_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.11,
        corner_segments=10,
    )
    right_side_frame = wire_from_points(
        [
            (0.74, -0.78, leg_radius),
            (0.74, 0.0, 2.12),
            (0.74, 0.78, leg_radius),
        ],
        radius=leg_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.11,
        corner_segments=10,
    )
    frame.visual(save_mesh("left_a_frame", left_side_frame), material=frame_green, name="left_a_frame")
    frame.visual(save_mesh("right_a_frame", right_side_frame), material=frame_green, name="right_a_frame")
    frame.visual(
        Cylinder(radius=0.050, length=1.48),
        origin=Origin(xyz=(0.0, 0.0, 2.12), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_green,
        name="top_beam",
    )
    frame.visual(
        Cylinder(radius=0.026, length=1.44),
        origin=Origin(xyz=(0.0, -0.66, 0.36), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_green,
        name="rear_spreader",
    )
    frame.visual(
        Cylinder(radius=0.026, length=1.44),
        origin=Origin(xyz=(0.0, 0.66, 0.36), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_green,
        name="front_spreader",
    )
    frame.visual(
        Cylinder(radius=0.022, length=1.15),
        origin=Origin(xyz=(-0.74, 0.0, 0.60), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_green,
        name="left_side_brace",
    )
    frame.visual(
        Cylinder(radius=0.022, length=1.15),
        origin=Origin(xyz=(0.74, 0.0, 0.60), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_green,
        name="right_side_brace",
    )

    def add_hanger(part_name: str, side_sign: float) -> None:
        hanger = model.part(part_name)
        hanger.inertial = Inertial.from_geometry(
            Box((0.08, 0.08, 1.30)),
            mass=4.2,
            origin=Origin(xyz=(0.0, 0.0, -0.65)),
        )
        hanger.visual(
            Box((0.062, 0.060, 0.040)),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=bracket_gray,
            name="top_head",
        )
        hanger.visual(
            Box((0.022, 0.038, 1.18)),
            origin=Origin(xyz=(0.0, 0.0, -0.610)),
            material=hanger_yellow,
            name="link_bar",
        )
        hanger.visual(
            Box((0.040, 0.050, 0.10)),
            origin=Origin(xyz=(0.010 * side_sign, 0.0, -1.20)),
            material=bracket_gray,
            name="lower_mount_block",
        )
        hanger.visual(
            Cylinder(radius=0.012, length=0.040),
            origin=Origin(xyz=(0.010 * side_sign, 0.0, -1.20), rpy=(0.0, pi / 2.0, 0.0)),
            material=bracket_gray,
            name="lower_bushing",
        )

        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=hanger,
            origin=Origin(xyz=(0.18 * side_sign, 0.0, 2.07)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.5,
                lower=-0.75,
                upper=0.75,
            ),
        )

    add_hanger("left_hanger", side_sign=-1.0)
    add_hanger("right_hanger", side_sign=1.0)

    bucket = model.part("bucket_seat")
    bucket.inertial = Inertial.from_geometry(
        Box((0.40, 0.34, 0.48)),
        mass=6.5,
        origin=Origin(xyz=(0.18, 0.0, -0.16)),
    )
    bucket.visual(
        Box((0.025, 0.040, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, 0.010)),
        material=bracket_gray,
        name="left_mount_lug",
    )
    bucket.visual(
        Box((0.025, 0.040, 0.040)),
        origin=Origin(xyz=(0.300, 0.0, 0.010)),
        material=bracket_gray,
        name="right_mount_lug",
    )
    bucket.visual(
        Box((0.290, 0.240, 0.018)),
        origin=Origin(xyz=(0.18, 0.0, -0.270)),
        material=bucket_black,
        name="seat_floor",
    )
    bucket.visual(
        Box((0.340, 0.018, 0.400)),
        origin=Origin(xyz=(0.18, -0.118, -0.100)),
        material=bucket_black,
        name="back_wall",
    )
    bucket.visual(
        Box((0.020, 0.240, 0.340)),
        origin=Origin(xyz=(0.020, 0.0, -0.140)),
        material=bucket_black,
        name="left_side_wall",
    )
    bucket.visual(
        Box((0.020, 0.240, 0.340)),
        origin=Origin(xyz=(0.340, 0.0, -0.140)),
        material=bucket_black,
        name="right_side_wall",
    )
    bucket.visual(
        Box((0.340, 0.016, 0.150)),
        origin=Origin(xyz=(0.18, 0.122, -0.200)),
        material=bucket_black,
        name="front_lower_wall",
    )
    bucket.visual(
        Box((0.090, 0.050, 0.200)),
        origin=Origin(xyz=(0.065, 0.104, -0.080)),
        material=bucket_black,
        name="left_front_bolster",
    )
    bucket.visual(
        Box((0.090, 0.050, 0.200)),
        origin=Origin(xyz=(0.295, 0.104, -0.080)),
        material=bucket_black,
        name="right_front_bolster",
    )
    bucket.visual(
        Box((0.070, 0.120, 0.180)),
        origin=Origin(xyz=(0.18, 0.055, -0.190)),
        material=bucket_black,
        name="center_pommel",
    )
    bucket.visual(
        Cylinder(radius=0.015, length=0.360),
        origin=Origin(xyz=(0.18, -0.125, 0.090), rpy=(0.0, pi / 2.0, 0.0)),
        material=bucket_black,
        name="rear_rim",
    )
    bucket.visual(
        Cylinder(radius=0.014, length=0.240),
        origin=Origin(xyz=(0.045, 0.0, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bucket_black,
        name="left_rim",
    )
    bucket.visual(
        Cylinder(radius=0.014, length=0.240),
        origin=Origin(xyz=(0.315, 0.0, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bucket_black,
        name="right_rim",
    )
    bucket.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.045, 0.112, 0.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=bracket_gray,
        name="left_bar_pivot_boss",
    )
    bucket.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.315, 0.112, 0.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=bracket_gray,
        name="right_bar_pivot_boss",
    )

    model.articulation(
        "left_hanger_to_bucket",
        ArticulationType.FIXED,
        parent="left_hanger",
        child=bucket,
        origin=Origin(xyz=(0.0, 0.0, -1.28)),
    )

    safety_bar = model.part("safety_bar")
    safety_bar.inertial = Inertial.from_geometry(
        Box((0.30, 0.17, 0.16)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.085, -0.045)),
    )
    safety_bar.visual(
        Box((0.006, 0.028, 0.028)),
        origin=Origin(xyz=(-0.158, 0.022, 0.0)),
        material=bracket_gray,
        name="left_pivot_pad",
    )
    safety_bar.visual(
        Box((0.026, 0.040, 0.018)),
        origin=Origin(xyz=(-0.145, 0.047, 0.0)),
        material=bar_orange,
        name="left_lower_arm",
    )
    safety_bar.visual(
        Box((0.018, 0.110, 0.018)),
        origin=Origin(xyz=(-0.135, 0.0975, -0.035), rpy=(-0.69, 0.0, 0.0)),
        material=bar_orange,
        name="left_diagonal_arm",
    )
    safety_bar.visual(
        Box((0.006, 0.028, 0.028)),
        origin=Origin(xyz=(0.158, 0.022, 0.0)),
        material=bracket_gray,
        name="right_pivot_pad",
    )
    safety_bar.visual(
        Box((0.026, 0.040, 0.018)),
        origin=Origin(xyz=(0.145, 0.047, 0.0)),
        material=bar_orange,
        name="right_lower_arm",
    )
    safety_bar.visual(
        Box((0.018, 0.110, 0.018)),
        origin=Origin(xyz=(0.135, 0.0975, -0.035), rpy=(-0.69, 0.0, 0.0)),
        material=bar_orange,
        name="right_diagonal_arm",
    )
    safety_bar.visual(
        Cylinder(radius=0.010, length=0.270),
        origin=Origin(xyz=(0.0, 0.140, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=bar_orange,
        name="gate_hoop",
    )
    safety_bar.visual(
        Box((0.014, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.140, -0.105)),
        material=bar_orange,
        name="center_guard",
    )

    model.articulation(
        "bucket_to_safety_bar",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=safety_bar,
        origin=Origin(xyz=(0.180, 0.112, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_hanger = object_model.get_part("left_hanger")
    right_hanger = object_model.get_part("right_hanger")
    bucket = object_model.get_part("bucket_seat")
    safety_bar = object_model.get_part("safety_bar")

    left_joint = object_model.get_articulation("frame_to_left_hanger")
    right_joint = object_model.get_articulation("frame_to_right_hanger")
    bar_joint = object_model.get_articulation("bucket_to_safety_bar")

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
        "pendulum joints use left-right hinge axis",
        tuple(left_joint.axis) == (1.0, 0.0, 0.0) and tuple(right_joint.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_joint.axis}, right={right_joint.axis}",
    )
    ctx.check(
        "safety bar uses side-pivot hinge axis",
        tuple(bar_joint.axis) == (1.0, 0.0, 0.0),
        details=f"bar axis={bar_joint.axis}",
    )

    ctx.expect_contact(left_hanger, frame, name="left hanger mounted to top beam")
    ctx.expect_contact(right_hanger, frame, name="right hanger mounted to top beam")
    ctx.expect_contact(bucket, left_hanger, name="bucket mounted on left hanger")
    ctx.expect_contact(bucket, right_hanger, name="bucket supported by right hanger")
    ctx.expect_contact(safety_bar, bucket, name="safety bar pivots on bucket rim")
    ctx.expect_overlap(bucket, left_hanger, axes="z", min_overlap=0.05, name="left hanger reaches bucket elevation")
    ctx.expect_overlap(bucket, right_hanger, axes="z", min_overlap=0.05, name="right hanger reaches bucket elevation")

    bucket_rest = ctx.part_world_position(bucket)
    assert bucket_rest is not None
    with ctx.pose({left_joint: 0.35, right_joint: 0.35}):
        bucket_swung = ctx.part_world_position(bucket)
        assert bucket_swung is not None
        ctx.check(
            "bucket swings forward as pendulum",
            bucket_swung[1] > bucket_rest[1] + 0.40 and bucket_swung[2] > bucket_rest[2] + 0.06,
            details=f"rest={bucket_rest}, swung={bucket_swung}",
        )
        ctx.expect_contact(bucket, right_hanger, name="right hanger stays aligned when swing is posed")

    bar_rest_aabb = ctx.part_world_aabb(safety_bar)
    assert bar_rest_aabb is not None
    with ctx.pose({bar_joint: 1.10}):
        bar_open_aabb = ctx.part_world_aabb(safety_bar)
        assert bar_open_aabb is not None
        ctx.check(
            "safety bar lifts clear of bucket opening",
            bar_open_aabb[0][2] > bar_rest_aabb[0][2] + 0.10
            and bar_open_aabb[1][2] > bar_rest_aabb[1][2] + 0.10,
            details=f"closed={bar_rest_aabb}, open={bar_open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
