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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_housing_shell():
    outer_profile = [
        (0.055, -0.090),
        (0.120, -0.088),
        (0.185, -0.074),
        (0.225, -0.045),
        (0.232, 0.000),
        (0.225, 0.045),
        (0.185, 0.074),
        (0.120, 0.088),
        (0.055, 0.090),
    ]
    inner_profile = [
        (0.045, -0.074),
        (0.105, -0.072),
        (0.165, -0.060),
        (0.198, -0.036),
        (0.204, 0.000),
        (0.198, 0.036),
        (0.165, 0.060),
        (0.105, 0.072),
        (0.045, 0.074),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(math.pi / 2.0)


def _build_handlebar():
    return tube_from_spline_points(
        [
            (0.295, -0.205, 0.990),
            (0.245, -0.125, 1.025),
            (0.275, 0.000, 1.040),
            (0.245, 0.125, 1.025),
            (0.295, 0.205, 0.990),
        ],
        radius=0.022,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _build_seat_sleeve():
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.122, 0.122, 0.014),
        [rounded_rect_profile(0.078, 0.078, 0.008)],
        0.34,
        cap=True,
        center=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stationary_exercise_bike")

    frame_black = model.material("frame_black", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.11, 0.12, 0.13, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.45, 0.47, 0.49, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.10, 0.64, 0.08)),
        origin=Origin(xyz=(-0.40, 0.0, 0.04)),
        material=frame_black,
        name="rear_stabilizer",
    )
    frame.visual(
        Box((0.10, 0.64, 0.08)),
        origin=Origin(xyz=(0.42, 0.0, 0.04)),
        material=frame_black,
        name="front_stabilizer",
    )
    frame.visual(
        Box((0.84, 0.12, 0.10)),
        origin=Origin(xyz=(0.01, 0.0, 0.12)),
        material=frame_black,
        name="center_rail",
    )
    frame.visual(
        Box((0.44, 0.13, 0.11)),
        origin=Origin(xyz=(-0.28, 0.0, 0.38), rpy=(0.0, 1.02, 0.0)),
        material=frame_black,
        name="seat_strut",
    )
    frame.visual(
        Box((0.26, 0.14, 0.12)),
        origin=Origin(xyz=(-0.15, 0.0, 0.235), rpy=(0.0, 0.83, 0.0)),
        material=frame_black,
        name="bottom_bracket_gusset",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.15),
        origin=Origin(xyz=(-0.07, 0.0, 0.29), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="bottom_bracket_shell",
    )
    frame.visual(
        _save_mesh("seat_sleeve", _build_seat_sleeve()),
        origin=Origin(xyz=(-0.17, 0.0, 0.50)),
        material=metal_gray,
        name="seat_sleeve",
    )
    frame.visual(
        Box((0.12, 0.10, 0.52)),
        origin=Origin(xyz=(0.28, 0.0, 0.77), rpy=(0.0, 0.18, 0.0)),
        material=frame_black,
        name="handlebar_mast",
    )
    frame.visual(
        Box((0.10, 0.08, 0.10)),
        origin=Origin(xyz=(0.27, 0.0, 1.005)),
        material=metal_gray,
        name="handlebar_stem",
    )
    frame.visual(
        _save_mesh("housing_shell", _build_housing_shell()),
        origin=Origin(xyz=(0.22, 0.0, 0.33)),
        material=dark_plastic,
        name="housing_shell",
    )
    frame.visual(
        Box((0.08, 0.08, 0.16)),
        origin=Origin(xyz=(0.22, 0.11, 0.33)),
        material=metal_gray,
        name="right_axle_block",
    )
    frame.visual(
        Box((0.08, 0.08, 0.16)),
        origin=Origin(xyz=(0.22, -0.11, 0.33)),
        material=metal_gray,
        name="left_axle_block",
    )
    frame.visual(
        _save_mesh("handlebar", _build_handlebar()),
        material=frame_black,
        name="handlebar",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.98, 0.64, 1.08)),
        mass=34.0,
        origin=Origin(xyz=(0.02, 0.0, 0.54)),
    )

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=0.035, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=metal_gray,
        name="inner_post",
    )
    seat_post.visual(
        Box((0.08, 0.10, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=metal_gray,
        name="seat_clamp",
    )
    seat_post.visual(
        Box((0.18, 0.12, 0.03)),
        origin=Origin(xyz=(0.01, 0.0, 0.275)),
        material=saddle_black,
        name="saddle_base",
    )
    seat_post.visual(
        Box((0.24, 0.18, 0.05)),
        origin=Origin(xyz=(-0.01, 0.0, 0.31)),
        material=saddle_black,
        name="saddle_rear",
    )
    seat_post.visual(
        Box((0.16, 0.09, 0.04)),
        origin=Origin(xyz=(0.13, 0.0, 0.30)),
        material=saddle_black,
        name="saddle_nose",
    )
    seat_post.inertial = Inertial.from_geometry(
        Box((0.32, 0.18, 0.56)),
        mass=3.2,
        origin=Origin(xyz=(0.04, 0.0, 0.17)),
    )

    crank_set = model.part("crank_set")
    crank_set.visual(
        Cylinder(radius=0.018, length=0.26),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="spindle",
    )
    crank_set.visual(
        Cylinder(radius=0.085, length=0.010),
        origin=Origin(xyz=(0.0, 0.110, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="chainring_disc",
    )
    crank_set.visual(
        Box((0.17, 0.022, 0.035)),
        origin=Origin(xyz=(0.050, 0.110, -0.069), rpy=(0.0, 0.95, 0.0)),
        material=metal_gray,
        name="right_crank_arm",
    )
    crank_set.visual(
        Box((0.17, 0.022, 0.035)),
        origin=Origin(xyz=(-0.050, -0.110, 0.069), rpy=(0.0, 0.95 + math.pi, 0.0)),
        material=metal_gray,
        name="left_crank_arm",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.099, 0.135, -0.138), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="right_pedal_spindle",
    )
    crank_set.visual(
        Box((0.090, 0.036, 0.030)),
        origin=Origin(xyz=(0.099, 0.165, -0.138)),
        material=dark_plastic,
        name="right_pedal",
    )
    crank_set.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(-0.099, -0.135, 0.138), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="left_pedal_spindle",
    )
    crank_set.visual(
        Box((0.090, 0.036, 0.030)),
        origin=Origin(xyz=(-0.099, -0.165, 0.138)),
        material=dark_plastic,
        name="left_pedal",
    )
    crank_set.inertial = Inertial.from_geometry(
        Box((0.28, 0.38, 0.34)),
        mass=4.5,
        origin=Origin(),
    )

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.162, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="flywheel_rim",
    )
    flywheel.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="flywheel_hub",
    )
    flywheel.visual(
        Box((0.032, 0.020, 0.026)),
        origin=Origin(xyz=(0.122, 0.0, 0.086)),
        material=dark_plastic,
        name="balance_weight",
    )
    flywheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.162, length=0.050),
        mass=12.0,
        origin=Origin(),
    )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.17, 0.0, 0.67)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.14,
        ),
    )
    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank_set,
        origin=Origin(xyz=(-0.07, 0.0, 0.29)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=14.0,
        ),
    )
    model.articulation(
        "flywheel_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.22, 0.0, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=24.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat_post = object_model.get_part("seat_post")
    crank_set = object_model.get_part("crank_set")
    flywheel = object_model.get_part("flywheel")
    seat_joint = object_model.get_articulation("seat_height")
    crank_joint = object_model.get_articulation("crank_rotation")
    flywheel_joint = object_model.get_articulation("flywheel_rotation")

    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason="The seat sleeve visual is a simplified outer sleeve proxy for the prismatic telescoping fit.",
    )
    ctx.allow_overlap(
        frame,
        crank_set,
        elem_a="bottom_bracket_shell",
        elem_b="spindle",
        reason="The bottom bracket shell is a simplified solid housing around the crank spindle bearings.",
    )

    ctx.check(
        "seat post uses prismatic adjustment",
        seat_joint.joint_type == ArticulationType.PRISMATIC,
        details=f"joint_type={seat_joint.joint_type}",
    )
    ctx.check(
        "crank uses continuous rotation",
        crank_joint.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={crank_joint.joint_type}",
    )
    ctx.check(
        "flywheel uses continuous rotation",
        flywheel_joint.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={flywheel_joint.joint_type}",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.001,
        name="seat post stays centered in sleeve at rest",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.18,
        name="seat post remains inserted at rest",
    )

    rest_pos = ctx.part_world_position(seat_post)
    with ctx.pose({seat_joint: 0.14}):
        ctx.expect_within(
            seat_post,
            frame,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="seat_sleeve",
            margin=0.001,
            name="seat post stays centered when raised",
        )
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.04,
            name="seat post keeps retained insertion when raised",
        )
        raised_pos = ctx.part_world_position(seat_post)
    ctx.check(
        "seat post raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.expect_within(
        flywheel,
        frame,
        axes="xz",
        inner_elem="flywheel_rim",
        outer_elem="housing_shell",
        margin=0.005,
        name="flywheel stays within housing footprint",
    )

    def _aabb_center(bounds):
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    with ctx.pose({crank_joint: 0.0}):
        pedal_rest = _aabb_center(ctx.part_element_world_aabb(crank_set, elem="right_pedal"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        pedal_quarter = _aabb_center(ctx.part_element_world_aabb(crank_set, elem="right_pedal"))
    ctx.check(
        "crank pedal moves through an orbital path",
        pedal_rest is not None
        and pedal_quarter is not None
        and math.hypot(
            pedal_quarter[0] - pedal_rest[0],
            pedal_quarter[2] - pedal_rest[2],
        )
        > 0.10,
        details=f"rest={pedal_rest}, quarter={pedal_quarter}",
    )

    with ctx.pose({flywheel_joint: 0.0}):
        weight_rest = _aabb_center(ctx.part_element_world_aabb(flywheel, elem="balance_weight"))
    with ctx.pose({flywheel_joint: math.pi / 2.0}):
        weight_quarter = _aabb_center(ctx.part_element_world_aabb(flywheel, elem="balance_weight"))
    ctx.check(
        "flywheel marker rotates around the axle",
        weight_rest is not None
        and weight_quarter is not None
        and math.hypot(
            weight_quarter[0] - weight_rest[0],
            weight_quarter[2] - weight_rest[2],
        )
        > 0.10,
        details=f"rest={weight_rest}, quarter={weight_quarter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
