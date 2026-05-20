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


WHEEL_RADIUS = 0.10
WHEEL_WIDTH = 0.038
REAR_WHEEL_X = -0.205
FRONT_STEER_X = 0.290
STEERING_Z = 0.262
FOLD_Z = 0.405
FRONT_WHEEL_LOCAL_X = 0.020
FRONT_WHEEL_Y = 0.140
REAR_WHEEL_Y = 0.135


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 12,
    radial_segments: int = 18,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
        ),
        name,
    )


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _rounded_pad_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    thickness: float,
    corner_radius: float,
    center: tuple[float, float, float],
):
    pad = ExtrudeGeometry(
        rounded_rect_profile(width, depth, corner_radius, corner_segments=8),
        thickness,
        center=True,
    ).translate(*center)
    return mesh_from_geometry(pad, name)


def _add_wheel_visuals(part, *, tire_material, rim_material, hub_material) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.76, length=WHEEL_WIDTH * 0.72),
        origin=spin_origin,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.28, length=WHEEL_WIDTH * 0.96),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.12, length=WHEEL_WIDTH * 1.06),
        origin=spin_origin,
        material=hub_material,
        name="axle_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_knee_scooter")

    frame_dark = model.material("frame_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    frame_light = model.material("frame_light", rgba=(0.63, 0.66, 0.69, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    clamp_accent = model.material("clamp_accent", rgba=(0.82, 0.20, 0.14, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.86, 0.42, 0.54)),
        mass=6.8,
        origin=Origin(xyz=(0.00, 0.00, 0.27)),
    )
    frame.visual(
        Box((0.330, 0.150, 0.022)),
        origin=Origin(xyz=(-0.060, 0.000, 0.377)),
        material=frame_dark,
        name="pad_tray",
    )
    frame.visual(
        _rounded_pad_mesh(
            "knee_scooter_knee_pad",
            width=0.330,
            depth=0.180,
            thickness=0.055,
            corner_radius=0.030,
            center=(-0.060, 0.000, 0.4155),
        ),
        material=pad_vinyl,
        name="knee_pad",
    )
    frame.visual(
        Box((0.045, 0.100, 0.110)),
        origin=Origin(xyz=(-0.150, 0.000, 0.322)),
        material=frame_dark,
        name="rear_pad_post",
    )
    frame.visual(
        Box((0.045, 0.100, 0.110)),
        origin=Origin(xyz=(0.020, 0.000, 0.322)),
        material=frame_dark,
        name="front_pad_post",
    )
    frame.visual(
        Box((0.070, 0.180, 0.020)),
        origin=Origin(xyz=(-0.220, 0.000, 0.292)),
        material=frame_dark,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.080, 0.160, 0.022)),
        origin=Origin(xyz=(0.040, 0.000, 0.352)),
        material=frame_dark,
        name="front_crossmember",
    )
    frame.visual(
        Box((0.060, 0.092, 0.180)),
        origin=Origin(xyz=(REAR_WHEEL_X, 0.070, 0.210)),
        material=frame_dark,
        name="rear_left_support",
    )
    frame.visual(
        Box((0.060, 0.092, 0.180)),
        origin=Origin(xyz=(REAR_WHEEL_X, -0.070, 0.210)),
        material=frame_dark,
        name="rear_right_support",
    )
    frame.visual(
        _tube_mesh(
            "knee_scooter_left_upper_rail",
            [
                (-0.205, 0.070, 0.260),
                (-0.155, 0.075, 0.315),
                (-0.020, 0.075, 0.360),
                (0.125, 0.068, 0.333),
                (0.235, 0.050, 0.292),
            ],
            radius=0.016,
        ),
        material=frame_light,
        name="left_upper_rail",
    )
    frame.visual(
        _tube_mesh(
            "knee_scooter_right_upper_rail",
            _mirror_y(
                [
                    (-0.205, 0.070, 0.260),
                    (-0.155, 0.075, 0.315),
                    (-0.020, 0.075, 0.360),
                    (0.125, 0.068, 0.333),
                    (0.235, 0.050, 0.292),
                ]
            ),
            radius=0.016,
        ),
        material=frame_light,
        name="right_upper_rail",
    )
    frame.visual(
        _tube_mesh(
            "knee_scooter_left_lower_rail",
            [
                (-0.205, 0.070, 0.210),
                (-0.060, 0.068, 0.205),
                (0.110, 0.058, 0.228),
                (0.215, 0.040, 0.236),
            ],
            radius=0.013,
            samples_per_segment=10,
        ),
        material=frame_light,
        name="left_lower_rail",
    )
    frame.visual(
        _tube_mesh(
            "knee_scooter_right_lower_rail",
            _mirror_y(
                [
                    (-0.205, 0.070, 0.210),
                    (-0.060, 0.068, 0.205),
                    (0.110, 0.058, 0.228),
                    (0.215, 0.040, 0.236),
                ]
            ),
            radius=0.013,
            samples_per_segment=10,
        ),
        material=frame_light,
        name="right_lower_rail",
    )
    frame.visual(
        _tube_mesh(
            "knee_scooter_center_spine",
            [
                (-0.020, 0.000, 0.351),
                (0.060, 0.000, 0.330),
                (0.150, 0.000, 0.283),
                (0.240, 0.000, 0.240),
            ],
            radius=0.014,
            samples_per_segment=10,
        ),
        material=frame_light,
        name="center_spine",
    )
    frame.visual(
        Box((0.085, 0.036, 0.190)),
        origin=Origin(xyz=(0.248, 0.066, 0.295)),
        material=frame_dark,
        name="left_upright_cheek",
    )
    frame.visual(
        Box((0.085, 0.036, 0.190)),
        origin=Origin(xyz=(0.248, -0.066, 0.295)),
        material=frame_dark,
        name="right_upright_cheek",
    )
    frame.visual(
        Box((0.052, 0.132, 0.040)),
        origin=Origin(xyz=(0.220, 0.000, 0.250)),
        material=frame_dark,
        name="upright_web",
    )
    frame.visual(
        Box((0.040, 0.028, 0.040)),
        origin=Origin(xyz=(0.272, 0.034, 0.255)),
        material=frame_light,
        name="left_head_bearing",
    )
    frame.visual(
        Box((0.040, 0.028, 0.040)),
        origin=Origin(xyz=(0.272, -0.034, 0.255)),
        material=frame_light,
        name="right_head_bearing",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.16, 0.34, 0.46)),
        mass=2.0,
        origin=Origin(xyz=(0.00, 0.00, 0.02)),
    )
    fork.visual(
        Cylinder(radius=0.020, length=0.166),
        origin=Origin(xyz=(0.000, 0.000, 0.061)),
        material=frame_light,
        name="steerer_column",
    )
    fork.visual(
        Box((0.070, 0.084, 0.060)),
        origin=Origin(xyz=(0.000, 0.000, 0.108)),
        material=frame_dark,
        name="fold_clamp_body",
    )
    fork.visual(
        Box((0.050, 0.016, 0.080)),
        origin=Origin(xyz=(0.000, 0.032, 0.143)),
        material=frame_dark,
        name="left_hinge_ear",
    )
    fork.visual(
        Box((0.050, 0.016, 0.080)),
        origin=Origin(xyz=(0.000, -0.032, 0.143)),
        material=frame_dark,
        name="right_hinge_ear",
    )
    fork.visual(
        Box((0.044, 0.326, 0.024)),
        origin=Origin(xyz=(0.032, 0.000, -0.027)),
        material=frame_dark,
        name="fork_crown",
    )
    fork.visual(
        Box((0.040, 0.014, 0.030)),
        origin=Origin(xyz=(FRONT_WHEEL_LOCAL_X, 0.167, -0.177)),
        material=frame_dark,
        name="left_axle_mount",
    )
    fork.visual(
        Box((0.040, 0.014, 0.030)),
        origin=Origin(xyz=(FRONT_WHEEL_LOCAL_X, -0.167, -0.177)),
        material=frame_dark,
        name="right_axle_mount",
    )
    fork.visual(
        Box((0.038, 0.300, 0.020)),
        origin=Origin(xyz=(0.026, 0.000, -0.044)),
        material=frame_light,
        name="fork_yoke",
    )
    fork.visual(
        Box((0.020, 0.016, 0.132)),
        origin=Origin(xyz=(FRONT_WHEEL_LOCAL_X, 0.171, -0.101)),
        material=frame_light,
        name="left_fork_blade",
    )
    fork.visual(
        Box((0.020, 0.016, 0.132)),
        origin=Origin(xyz=(FRONT_WHEEL_LOCAL_X, -0.171, -0.101)),
        material=frame_light,
        name="right_fork_blade",
    )
    fork.visual(
        Box((0.028, 0.056, 0.026)),
        origin=Origin(xyz=(0.041, 0.000, 0.122)),
        material=clamp_accent,
        name="folding_latch",
    )

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.12, 0.44, 0.62)),
        mass=1.8,
        origin=Origin(xyz=(0.010, 0.000, 0.300)),
    )
    stem.visual(
        Box((0.060, 0.048, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
        material=frame_dark,
        name="hinge_block",
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.500),
        origin=Origin(xyz=(0.000, 0.000, 0.275)),
        material=frame_light,
        name="outer_stem",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.180),
        origin=Origin(xyz=(0.000, 0.000, 0.470)),
        material=frame_light,
        name="inner_stem",
    )
    stem.visual(
        Box((0.080, 0.060, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.525)),
        material=frame_dark,
        name="handlebar_clamp",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.420),
        origin=Origin(xyz=(0.000, 0.000, 0.545), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_light,
        name="handlebar",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.000, 0.155, 0.545), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.000, -0.155, 0.545), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.65,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        tire_material=rubber,
        rim_material=frame_light,
        hub_material=frame_dark,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.65,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        tire_material=rubber,
        rim_material=frame_light,
        hub_material=frame_dark,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.65,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_left_wheel,
        tire_material=rubber,
        rim_material=frame_light,
        hub_material=frame_dark,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.65,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_right_wheel,
        tire_material=rubber,
        rim_material=frame_light,
        hub_material=frame_dark,
    )

    steering = model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(FRONT_STEER_X, 0.000, STEERING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.60, upper=0.60),
    )
    stem_fold = model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.000, 0.000, FOLD_Z - STEERING_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_left_wheel,
        origin=Origin(xyz=(FRONT_WHEEL_LOCAL_X, FRONT_WHEEL_Y, -0.162)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=22.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_right_wheel,
        origin=Origin(xyz=(FRONT_WHEEL_LOCAL_X, -FRONT_WHEEL_Y, -0.162)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=22.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, REAR_WHEEL_Y, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=22.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, -REAR_WHEEL_Y, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    frame = object_model.get_part("frame")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    steering = object_model.get_articulation("steering")
    stem_fold = object_model.get_articulation("stem_fold")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")

    for part_name, part in [
        ("frame", frame),
        ("fork", fork),
        ("stem", stem),
        ("front_left_wheel", front_left_wheel),
        ("front_right_wheel", front_right_wheel),
        ("rear_left_wheel", rear_left_wheel),
        ("rear_right_wheel", rear_right_wheel),
    ]:
        ctx.check(f"{part_name}_present", part is not None, f"missing part {part_name}")

    ctx.check(
        "steering_axis_vertical",
        tuple(steering.axis) == (0.0, 0.0, 1.0),
        f"expected steering axis (0,0,1), got {steering.axis}",
    )
    ctx.check(
        "stem_fold_axis_transverse",
        tuple(stem_fold.axis) == (0.0, 1.0, 0.0),
        f"expected fold axis (0,1,0), got {stem_fold.axis}",
    )
    for joint_name, joint in [
        ("front_left_spin", front_left_spin),
        ("front_right_spin", front_right_spin),
        ("rear_left_spin", rear_left_spin),
        ("rear_right_spin", rear_right_spin),
    ]:
        ctx.check(
            f"{joint_name}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint_name} should spin around the wheel axle, got {joint.axis}",
        )

    ctx.expect_contact(
        fork,
        frame,
        elem_a="steerer_column",
        elem_b="left_head_bearing",
        contact_tol=0.001,
        name="fork_seats_on_left_head_bearing",
    )
    ctx.expect_contact(
        fork,
        frame,
        elem_a="steerer_column",
        elem_b="right_head_bearing",
        contact_tol=0.001,
        name="fork_seats_on_right_head_bearing",
    )
    ctx.expect_contact(
        stem,
        fork,
        elem_a="hinge_block",
        elem_b="left_hinge_ear",
        contact_tol=0.001,
        name="stem_contacts_left_hinge_ear",
    )
    ctx.expect_contact(
        stem,
        fork,
        elem_a="hinge_block",
        elem_b="right_hinge_ear",
        contact_tol=0.001,
        name="stem_contacts_right_hinge_ear",
    )
    ctx.expect_contact(
        front_left_wheel,
        fork,
        elem_a="axle_cap",
        elem_b="left_axle_mount",
        contact_tol=0.001,
        name="front_left_wheel_mounted",
    )
    ctx.expect_contact(
        front_right_wheel,
        fork,
        elem_a="axle_cap",
        elem_b="right_axle_mount",
        contact_tol=0.001,
        name="front_right_wheel_mounted",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        elem_a="tire",
        elem_b="rear_left_support",
        contact_tol=0.001,
        name="rear_left_wheel_mounted",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        elem_a="tire",
        elem_b="rear_right_support",
        contact_tol=0.001,
        name="rear_right_wheel_mounted",
    )

    pad_aabb = ctx.part_element_world_aabb(frame, elem="knee_pad")
    ctx.check(
        "knee_pad_height_realistic",
        pad_aabb is not None and pad_aabb[0][2] > 0.38 and pad_aabb[1][2] < 0.47,
        f"unexpected knee pad aabb: {pad_aabb}",
    )

    rest_left = ctx.part_world_position(front_left_wheel)
    rest_right = ctx.part_world_position(front_right_wheel)
    with ctx.pose({steering: 0.45}):
        turned_left = ctx.part_world_position(front_left_wheel)
        turned_right = ctx.part_world_position(front_right_wheel)
    ctx.check(
        "front_wheels_follow_steering",
        rest_left is not None
        and rest_right is not None
        and turned_left is not None
        and turned_right is not None
        and turned_left[0] < rest_left[0] - 0.04
        and turned_right[0] > rest_right[0] + 0.04,
        f"rest_left={rest_left}, turned_left={turned_left}, rest_right={rest_right}, turned_right={turned_right}",
    )

    rest_stem_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({stem_fold: 1.10}):
        folded_stem_aabb = ctx.part_world_aabb(stem)
    ctx.check(
        "stem_folds_forward",
        rest_stem_aabb is not None
        and folded_stem_aabb is not None
        and folded_stem_aabb[1][0] > rest_stem_aabb[1][0] + 0.18
        and folded_stem_aabb[1][2] < rest_stem_aabb[1][2] - 0.10,
        f"rest_stem_aabb={rest_stem_aabb}, folded_stem_aabb={folded_stem_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
